//! Bounded ZX0 v2 blocks used by the ZMODEM extension.
use std::io;

pub const BLOCK_SIZE: usize = 4096;
const MAX_ENCODED: usize = 8192;

/// Compresses independent ZX0 v2 blocks (at most 4096 decoded bytes each).
pub fn compress(input: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(input.len());
    for block in input.chunks(BLOCK_SIZE) {
        let encoded = ::zx0::Compressor::new().compress(block).output;
        if encoded.len() <= MAX_ENCODED && encoded.len() < block.len() {
            out.extend_from_slice(&(block.len() as u16).to_le_bytes());
            out.extend_from_slice(&(encoded.len() as u16).to_le_bytes());
            out.extend_from_slice(&encoded);
        } else {
            out.extend_from_slice(&(block.len() as u16).to_le_bytes());
            out.extend_from_slice(&((block.len() as u16) | 0x8000).to_le_bytes());
            out.extend_from_slice(block);
        }
    }
    out
}

/// Decompresses the block envelope, rejecting malformed or overlong input.
pub fn decompress(input: &[u8], expected_size: usize) -> io::Result<Vec<u8>> {
    let mut at = 0usize;
    let mut output = Vec::new();
    while at < input.len() {
        if input.len() - at < 4 {
            return Err(invalid("truncated ZX0 block header"));
        }
        let decoded = u16::from_le_bytes([input[at], input[at + 1]]) as usize;
        let raw_len = u16::from_le_bytes([input[at + 2], input[at + 3]]);
        at += 4;
        let raw = raw_len & 0x8000 != 0;
        let stored = (raw_len & 0x7fff) as usize;
        if decoded == 0
            || decoded > BLOCK_SIZE
            || stored == 0
            || stored > MAX_ENCODED
            || stored > input.len() - at
            || decoded > expected_size.saturating_sub(output.len())
        {
            return Err(invalid("invalid ZX0 block lengths"));
        }
        let block = &input[at..at + stored];
        at += stored;
        if raw {
            if stored != decoded {
                return Err(invalid("raw ZX0 block length mismatch"));
            }
            output.extend_from_slice(block);
        } else {
            let decoded_block = decode_block(block, decoded)?;
            output.extend_from_slice(&decoded_block);
        }
    }
    if output.len() != expected_size {
        return Err(invalid("ZX0 decoded size mismatch"));
    }
    Ok(output)
}

fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

struct Bits<'a> {
    data: &'a [u8],
    byte: usize,
    bit_value: u16,
}
impl<'a> Bits<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            byte: 0,
            bit_value: 0x8000,
        }
    }
    fn bit(&mut self) -> io::Result<bool> {
        if self.bit_value == 0x8000 {
            let next = self.byte()?;
            self.bit_value = ((next as u16) << 8) | 0x80;
        }
        let bit = self.bit_value & 0x8000 != 0;
        self.bit_value <<= 1;
        Ok(bit)
    }
    fn byte(&mut self) -> io::Result<u8> {
        let value = *self
            .data
            .get(self.byte)
            .ok_or_else(|| invalid("truncated ZX0 input"))?;
        self.byte += 1;
        Ok(value)
    }
    fn gamma(&mut self, inverted: bool) -> io::Result<usize> {
        let mut value = 1usize;
        while !self.bit()? {
            value = value
                .checked_mul(2)
                .and_then(|value| value.checked_add(usize::from(self.bit().ok()? ^ inverted)))
                .ok_or_else(|| invalid("ZX0 gamma overflow"))?;
            if value > 65536 {
                return Err(invalid("ZX0 gamma overflow"));
            }
        }
        Ok(value)
    }
}

fn decode_block(input: &[u8], expected: usize) -> io::Result<Vec<u8>> {
    #[derive(Clone, Copy)]
    enum State {
        Literals,
        LastOffset,
        NewOffset,
    }

    let mut bits = Bits::new(input);
    let mut output = Vec::with_capacity(expected);
    let mut offset = 1usize;
    let mut state = State::Literals;
    loop {
        match state {
            State::Literals => {
                let length = bits.gamma(false)?;
                if length > expected.saturating_sub(output.len()) {
                    return Err(invalid("ZX0 output exceeds block"));
                }
                for _ in 0..length {
                    output.push(bits.byte()?);
                }
                state = if bits.bit()? {
                    State::NewOffset
                } else {
                    State::LastOffset
                };
            }
            State::LastOffset => {
                let length = bits.gamma(false)?;
                copy_match(&mut output, offset, length, expected)?;
                state = if bits.bit()? {
                    State::NewOffset
                } else {
                    State::Literals
                };
            }
            State::NewOffset => {
                let high = bits.gamma(true)?;
                if high == 256 {
                    if output.len() != expected || bits.byte != input.len() {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidData,
                            format!(
                                "ZX0 size/trailing mismatch (output {}/{expected}, input {}/{})",
                                output.len(),
                                bits.byte,
                                input.len()
                            ),
                        ));
                    }
                    return Ok(output);
                }
                let low = bits.byte()?;
                offset = high
                    .checked_mul(128)
                    .and_then(|value| value.checked_sub((low >> 1) as usize))
                    .ok_or_else(|| invalid("invalid ZX0 offset"))?;
                bits.bit_value = (bits.bit_value >> 1) | ((low as u16) << 15);
                let length = bits
                    .gamma(false)?
                    .checked_add(1)
                    .ok_or_else(|| invalid("ZX0 length overflow"))?;
                copy_match(&mut output, offset, length, expected)?;
                state = if bits.bit()? {
                    State::NewOffset
                } else {
                    State::Literals
                };
            }
        }
    }
}

fn copy_match(out: &mut Vec<u8>, offset: usize, length: usize, max: usize) -> io::Result<()> {
    if offset == 0 || offset > out.len() || length > max.saturating_sub(out.len()) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "invalid ZX0 match (offset {offset}, length {length}, output {})",
                out.len()
            ),
        ));
    }
    for _ in 0..length {
        let b = out[out.len() - offset];
        out.push(b);
    }
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_compressed_and_raw_blocks() {
        let repetitive = vec![0x5a; BLOCK_SIZE * 2 + 17];
        let mut input = repetitive.clone();
        let mut state = 0x1234_5678u32;
        input.extend((0..BLOCK_SIZE).map(|_| {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state >> 24) as u8
        }));
        let encoded = compress(&input);
        let mut third_block = 0;
        for _ in 0..2 {
            let stored =
                u16::from_le_bytes([encoded[third_block + 2], encoded[third_block + 3]]) & 0x7fff;
            third_block += 4 + stored as usize;
        }
        assert_ne!(
            u16::from_le_bytes([encoded[third_block + 2], encoded[third_block + 3]]) & 0x8000,
            0
        );
        assert_eq!(decompress(&encoded, input.len()).unwrap(), input);
    }
    #[test]
    fn rejects_truncation_and_size_mismatch() {
        let input = vec![0x41; 90];
        let encoded = compress(&input);
        assert_eq!(
            decompress(&encoded, input.len() + 1).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        assert_eq!(
            decompress(&encoded[..encoded.len() - 1], input.len())
                .unwrap_err()
                .kind(),
            io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn empty_stream_is_an_empty_file() {
        assert!(compress(&[]).is_empty());
        assert!(decompress(&[], 0).unwrap().is_empty());
    }
    #[test]
    fn decodes_reference_zx0_v2_vector() {
        let compressed = [
            0x1f, 0x41, 0x42, 0x52, 0x41, 0x20, 0xf6, 0xab, 0x43, 0x44, 0xf5, 0xf2, 0x55, 0x58,
        ];
        assert_eq!(decode_block(&compressed, 16).unwrap(), b"ABRA ABRACADABRA");
    }
}
