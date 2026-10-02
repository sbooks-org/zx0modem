//! ZMODEM with negotiated receive windows and CRC16 subpackets.
use crate::file_metadata::DosMetadata;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const ZPAD: u8 = b'*';
const ZDLE: u8 = 0x18;
const ZRQINIT: u8 = 0;
const ZRINIT: u8 = 1;
const ZACK: u8 = 3;
const ZFILE: u8 = 4;
const ZSKIP: u8 = 5;
const ZABORT: u8 = 7;
const ZFIN: u8 = 8;
const ZRPOS: u8 = 9;
const ZDATA: u8 = 10;
const ZEOF: u8 = 11;
const ZFERR: u8 = 12;
const ZCRCE: u8 = 0x68;
const ZCRCG: u8 = 0x69;
const ZCRCQ: u8 = 0x6a;
const ZCRCW: u8 = 0x6b;
const ZBIN: u8 = b'A';
const ZHEX: u8 = b'B';
const ZBIN32: u8 = b'C';
const ZX0_CAP: u8 = 0x80;
const CHUNK: usize = 512;

/// Payload progress, excluding protocol framing. Compressed transfers count
/// encoded bytes; their receive-side total is unknown until the file ends.
#[derive(Clone, Copy, Debug)]
pub struct Progress<'a> {
    pub path: &'a Path,
    pub transferred: u64,
    pub total: Option<u64>,
    pub compressed: bool,
    /// True only after the final ZFIN/OO handshake succeeds.
    pub complete: bool,
}

#[derive(Clone, Copy)]
struct Header {
    kind: u8,
    pos: u32,
}

fn read_byte<R: Read>(io: &mut R) -> io::Result<u8> {
    let mut b = [0];
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        match io.read(&mut b) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "ZMODEM peer closed",
                ));
            }
            Ok(_) => return Ok(b[0]),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) && Instant::now() < deadline =>
            {
                continue;
            }
            Err(e) => return Err(e),
        }
    }
}
fn receive_oo<R: Read>(io: &mut R) -> io::Result<()> {
    let mut first = read_byte(io)?;
    if first == 0x11 {
        first = read_byte(io)?;
    }
    if first != b'O' || read_byte(io)? != b'O' {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid ZMODEM finish sequence",
        ));
    }
    Ok(())
}
fn write_all<W: Write>(io: &mut W, bytes: &[u8]) -> io::Result<()> {
    io.write_all(bytes)?;
    io.flush()
}
fn crc16_update(mut crc: u16, byte: u8) -> u16 {
    crc ^= (byte as u16) << 8;
    for _ in 0..8 {
        crc = if crc & 0x8000 != 0 {
            (crc << 1) ^ 0x1021
        } else {
            crc << 1
        };
    }
    crc
}
fn crc16(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0, |crc, &b| crc16_update(crc, b))
}
fn crc16_wire(crc: u16) -> u16 {
    crc16_update(crc16_update(crc, 0), 0)
}
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in bytes {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}
fn escaped<W: Write>(io: &mut W, b: u8) -> io::Result<()> {
    if b == ZDLE
        || b == 0x10
        || b == 0x11
        || b == 0x13
        || b == 0x90
        || b == 0x91
        || b == 0x93
        || b == 0x9d
    {
        io.write_all(&[ZDLE, b ^ 0x40])
    } else {
        io.write_all(&[b])
    }
}
pub(crate) fn send_header<W: Write>(io: &mut W, kind: u8, pos: u32, hex: bool) -> io::Result<()> {
    let mut bytes = [0u8; 24];
    let mut frame = io::Cursor::new(bytes.as_mut_slice());
    encode_header(&mut frame, kind, pos, hex)?;
    let len = frame.position() as usize;
    write_all(io, &bytes[..len])
}
fn encode_header<W: Write>(io: &mut W, kind: u8, pos: u32, hex: bool) -> io::Result<()> {
    let data = [
        kind,
        pos as u8,
        (pos >> 8) as u8,
        (pos >> 16) as u8,
        (pos >> 24) as u8,
    ];
    let hex = hex || matches!(kind, ZRINIT | ZACK | ZSKIP | ZABORT | ZRPOS | ZFERR);
    if hex {
        let crc = crc16_wire(crc16(&data));
        const HEX: &[u8; 16] = b"0123456789abcdef";
        io.write_all(&[ZPAD, ZPAD, ZDLE, ZHEX])?;
        for b in data.into_iter().chain(crc.to_be_bytes()) {
            io.write_all(&[HEX[(b >> 4) as usize], HEX[(b & 15) as usize]])?;
        }
        io.write_all(b"\r\n")
    } else {
        let crc = crc16_wire(crc16(&data));
        io.write_all(&[ZPAD, ZDLE, ZBIN])?;
        for b in data {
            escaped(io, b)?;
        }
        for b in crc.to_be_bytes() {
            escaped(io, b)?;
        }
        Ok(())
    }
}
fn unescape<R: Read>(io: &mut R) -> io::Result<u8> {
    let b = read_byte(io)?;
    if b == ZDLE {
        Ok(read_byte(io)? ^ 0x40)
    } else {
        Ok(b)
    }
}
fn hexval(b: u8) -> io::Result<u8> {
    match b {
        b'0'..=b'9' => Ok(b - b'0'),
        b'a'..=b'f' => Ok(b - b'a' + 10),
        b'A'..=b'F' => Ok(b - b'A' + 10),
        _ => Err(io::Error::new(io::ErrorKind::InvalidData, "bad ZMODEM hex")),
    }
}
fn receive_header<R: Read>(io: &mut R) -> io::Result<Header> {
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        if Instant::now() >= deadline {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "ZMODEM header timeout",
            ));
        }
        let b = read_byte(io)?;
        if b != ZPAD {
            continue;
        }
        let mut p = read_byte(io)?;
        while p == ZPAD {
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "ZMODEM header timeout",
                ));
            }
            p = read_byte(io)?
        }
        if p != ZDLE {
            continue;
        }
        let format = read_byte(io)?;
        let mut data = [0u8; 5];
        if format == ZHEX {
            for b in &mut data {
                *b = (hexval(read_byte(io)?)? << 4) | hexval(read_byte(io)?)?;
            }
            let hi = (hexval(read_byte(io)?)? << 4) | hexval(read_byte(io)?)?;
            let lo = (hexval(read_byte(io)?)? << 4) | hexval(read_byte(io)?)?;
            if crc16_wire(crc16(&data)) != u16::from_be_bytes([hi, lo]) {
                continue;
            }
            let _ = read_byte(io)?;
            let _ = read_byte(io)?;
        } else if format == ZBIN || format == ZBIN32 {
            for b in &mut data {
                *b = unescape(io)?;
            }
            if format == ZBIN {
                let hi = unescape(io)?;
                let lo = unescape(io)?;
                if crc16_wire(crc16(&data)) != u16::from_be_bytes([hi, lo]) {
                    continue;
                }
            } else {
                let mut c = [0; 4];
                for b in &mut c {
                    *b = unescape(io)?;
                }
                if crc32(&data) != u32::from_le_bytes(c) {
                    continue;
                }
            }
        } else {
            continue;
        }
        return Ok(Header {
            kind: data[0],
            pos: u32::from_le_bytes([data[1], data[2], data[3], data[4]]),
        });
    }
}
pub(crate) fn send_data<W: Write>(io: &mut W, data: &[u8], end: u8) -> io::Result<()> {
    let mut frame = Vec::with_capacity(data.len() * 2 + 6);
    encode_data(&mut frame, data, end)?;
    write_all(io, &frame)
}
fn encode_data<W: Write>(io: &mut W, data: &[u8], end: u8) -> io::Result<()> {
    let mut crc = 0;
    for &b in data {
        escaped(io, b)?;
        crc = crc16_update(crc, b)
    }
    io.write_all(&[ZDLE, end])?;
    crc = crc16_wire(crc16_update(crc, end));
    for b in crc.to_be_bytes() {
        escaped(io, b)?;
    }
    Ok(())
}
fn receive_data<R: Read>(io: &mut R) -> io::Result<(Vec<u8>, u8)> {
    let mut data = Vec::new();
    loop {
        let b = read_byte(io)?;
        if b == ZDLE {
            let n = read_byte(io)?;
            if [ZCRCE, ZCRCG, ZCRCQ, ZCRCW].contains(&n) {
                let hi = unescape(io)?;
                let lo = unescape(io)?;
                let crc = crc16_wire(crc16_update(
                    data.iter().fold(0, |crc, &b| crc16_update(crc, b)),
                    n,
                ));
                if crc != u16::from_be_bytes([hi, lo]) {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "ZMODEM data CRC mismatch",
                    ));
                }
                return Ok((data, n));
            }
            data.push(n ^ 0x40);
        } else {
            data.push(b);
        }
        if data.len() > 8192 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "oversized ZMODEM packet",
            ));
        }
    }
}
fn wait_kind<R: Read>(io: &mut R, accepted: &[u8]) -> io::Result<Header> {
    for _ in 0..32 {
        let h = receive_header(io)?;
        if accepted.contains(&h.kind) {
            return Ok(h);
        }
        if h.kind == ZABORT || h.kind == ZFERR {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "ZMODEM peer aborted transfer",
            ));
        }
    }
    Err(io::Error::new(
        io::ErrorKind::InvalidData,
        "too many unexpected ZMODEM headers",
    ))
}

/// Send a file. ZX0 is requested only when `compress` is true; this extension
/// requires the receiver's ZX0 capability flag before compressed bytes are sent.
/// A receiver's ZSKIP is an error, not a successfully installed file.
/// `prepare` runs after the initial ZRINIT, before ZFILE, with receiver flags.
pub fn send<T: Read + Write>(
    io: &mut T,
    path: &Path,
    compress: bool,
    mut prepare: impl FnMut(&mut T, u8) -> io::Result<()>,
    mut progress: impl FnMut(Progress<'_>) -> io::Result<()>,
) -> io::Result<()> {
    let mut file = File::open(path)?;
    let metadata = DosMetadata::from_file(&file.metadata()?)?;
    let name = path
        .file_name()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "missing filename"))?
        .to_string_lossy();
    let mut contents = Vec::new();
    file.read_to_end(&mut contents)?;
    let size = contents.len() as u64;
    if contents.len() > u32::MAX as usize {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "ZMODEM supports at most 32-bit offsets",
        ));
    }
    let init = wait_kind(io, &[ZRINIT])?;
    // Zero means unspecified/unlimited in ZMODEM; retain the conservative
    // single-packet path unless the peer explicitly advertises more room.
    let advertised = (init.pos & 0xffff) as usize;
    let window = if advertised == 0 { CHUNK } else { advertised };
    let supports_zx0 = ((init.pos >> 24) as u8) & ZX0_CAP != 0;
    let use_zx0 = compress && supports_zx0;
    let wire = if use_zx0 {
        crate::zx0::compress(&contents)
    } else {
        contents
    };
    if wire.len() > u32::MAX as usize {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "ZMODEM wire stream exceeds 32-bit offsets",
        ));
    }
    let mut status = Progress {
        path,
        transferred: 0,
        total: Some(wire.len() as u64),
        compressed: use_zx0,
        complete: false,
    };
    let flags = if use_zx0 { ZX0_CAP } else { 1 };
    let mut meta = Vec::new();
    meta.extend_from_slice(name.as_bytes());
    meta.push(0);
    meta.extend_from_slice(size.to_string().as_bytes());
    meta.push(0);
    meta.extend_from_slice(&metadata.encode(flags));
    prepare(io, (init.pos >> 24) as u8)?;
    progress(status)?;
    send_header(io, ZFILE, 0, false)?;
    send_data(io, &meta, ZCRCW)?;
    let mut h = wait_kind(io, &[ZRPOS, ZSKIP])?;
    if h.kind == ZSKIP {
        return Err(io::Error::other("ZMODEM receiver skipped the file"));
    }
    let mut offset = h.pos as usize;
    if offset > wire.len() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "invalid ZRPOS"));
    }
    status.transferred = offset as u64;
    progress(status)?;
    let mut frame = Vec::with_capacity(CHUNK * 2 + 30);
    let mut retries = 0;
    while offset < wire.len() {
        let start = offset;
        let boundary = (start + window).min(wire.len());
        frame.clear();
        encode_header(&mut frame, ZDATA, start as u32, false)?;
        while offset < boundary {
            let end = (offset + CHUNK).min(boundary);
            encode_data(
                &mut frame,
                &wire[offset..end],
                if end == boundary { ZCRCW } else { ZCRCG },
            )?;
            // No drain between packets: queue a complete frame in one write,
            // reuse its storage, and allow the UART to run throughout a burst.
            io.write_all(&frame)?;
            frame.clear();
            offset = end;
            status.transferred = offset as u64;
            progress(status)?;
        }
        io.flush()?;
        h = wait_kind(io, &[ZRPOS, ZACK])?;
        if h.kind == ZRPOS {
            let requested = h.pos as usize;
            if requested < start || requested > boundary {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "invalid ZRPOS"));
            }
            retries += 1;
            if retries > 5 {
                return Err(io::Error::other("ZMODEM retry limit exceeded"));
            }
            offset = requested;
            status.transferred = offset as u64;
            progress(status)?;
        } else {
            if h.pos as usize != boundary {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "invalid ZACK"));
            }
            retries = 0;
        }
    }
    send_header(io, ZEOF, wire.len() as u32, false)?;
    h = wait_kind(io, &[ZRINIT])?;
    let _ = h;
    send_header(io, ZFIN, 0, true)?;
    let _ = wait_kind(io, &[ZFIN])?;
    write_all(io, b"OO")?;
    status.complete = true;
    progress(status)?;
    Ok(())
}

/// Receive one file into `directory`, refusing collisions and unsafe names.
pub fn receive<T: Read + Write>(
    io: &mut T,
    directory: &Path,
    mut progress: impl FnMut(Progress<'_>) -> io::Result<()>,
) -> io::Result<PathBuf> {
    let mut init = [0u8; 4];
    init[..2].copy_from_slice(&(CHUNK as u16).to_le_bytes());
    init[3] = ZX0_CAP;
    send_header(io, ZRINIT, u32::from_le_bytes(init), true)?;
    let mut output: Option<(PathBuf, File)> = None;
    let mut metadata_size = None;
    let mut file_metadata = None;
    let mut compressed = false;
    let mut wire = Vec::new();
    let mut data_start = 0u32;
    loop {
        let h = receive_header(io)?;
        match h.kind {
            ZRQINIT => send_header(io, ZRINIT, u32::from_le_bytes(init), true)?,
            ZFILE => {
                let (meta, end) = receive_data(io)?;
                if end != ZCRCW {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "invalid ZFILE packet end",
                    ));
                }
                let nul = meta.iter().position(|&b| b == 0).ok_or_else(|| {
                    io::Error::new(io::ErrorKind::InvalidData, "missing ZFILE name")
                })?;
                let tail = &meta[nul + 1..];
                let nul2 = tail.iter().position(|&b| b == 0).ok_or_else(|| {
                    io::Error::new(io::ErrorKind::InvalidData, "missing ZFILE size")
                })?;
                let size = std::str::from_utf8(&tail[..nul2])
                    .ok()
                    .and_then(|s| s.parse::<usize>().ok())
                    .ok_or_else(|| {
                        io::Error::new(io::ErrorKind::InvalidData, "invalid ZFILE size")
                    })?;
                if size > u32::MAX as usize {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "ZMODEM supports at most 32-bit offsets",
                    ));
                }
                let metadata = DosMetadata::decode(&tail[nul2 + 1..])?;
                let name = std::str::from_utf8(&meta[..nul])
                    .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid filename"))?;
                if name.is_empty()
                    || name == "."
                    || name == ".."
                    || name.contains('/')
                    || name.contains('\\')
                    || name.contains(':')
                {
                    send_header(io, ZSKIP, 0, false)?;
                    continue;
                }
                let flags = tail[nul2 + 3];
                compressed = flags & ZX0_CAP != 0;
                let path = directory.join(name);
                let file = match OpenOptions::new().write(true).create_new(true).open(&path) {
                    Ok(f) => f,
                    Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                        send_header(io, ZSKIP, 0, false)?;
                        continue;
                    }
                    Err(e) => return Err(e),
                };
                metadata_size = Some(size);
                file_metadata = Some(metadata);
                output = Some((path, file));
                progress(Progress {
                    path: &output.as_ref().unwrap().0,
                    transferred: 0,
                    total: if compressed { None } else { Some(size as u64) },
                    compressed,
                    complete: false,
                })?;
                wire.clear();
                data_start = 0;
                send_header(io, ZRPOS, 0, false)?;
            }
            ZDATA => {
                if output.is_none() {
                    send_header(io, ZRPOS, 0, true)?;
                    continue;
                }
                if h.pos as usize != wire.len() {
                    send_header(io, ZRPOS, wire.len() as u32, true)?;
                    continue;
                }
                let (packet, end) = match receive_data(io) {
                    Ok(packet) => packet,
                    Err(e) if e.kind() == io::ErrorKind::InvalidData => {
                        send_header(io, ZRPOS, wire.len() as u32, true)?;
                        continue;
                    }
                    Err(e) => return Err(e),
                };
                wire.extend_from_slice(&packet);
                if wire.len() > u32::MAX as usize
                    || wire.len()
                        > metadata_size
                            .unwrap_or(0)
                            .saturating_mul(2)
                            .saturating_add(65536)
                {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "ZMODEM input exceeds limit",
                    ));
                }
                data_start = wire.len() as u32;
                progress(Progress {
                    path: &output.as_ref().unwrap().0,
                    transferred: wire.len() as u64,
                    total: if compressed {
                        None
                    } else {
                        metadata_size.map(|n| n as u64)
                    },
                    compressed,
                    complete: false,
                })?;
                if end == ZCRCW {
                    send_header(io, ZACK, data_start, true)?;
                }
            }
            ZEOF => {
                if output.is_none() || h.pos != data_start {
                    send_header(io, ZRPOS, data_start, true)?;
                    continue;
                }
                let (path, mut file) = output.take().unwrap();
                let size = metadata_size.take().unwrap();
                if compressed {
                    let bytes = crate::zx0::decompress(&wire, size)?;
                    file.write_all(&bytes)?;
                } else {
                    if wire.len() != size {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidData,
                            "received size mismatch",
                        ));
                    }
                    file.write_all(&wire)?;
                }
                file.flush()?;
                let metadata = file_metadata.take().unwrap();
                metadata.apply_time(&file)?;
                metadata.apply_readonly(&file)?;
                drop(file);
                send_header(io, ZRINIT, u32::from_le_bytes(init), true)?;
                loop {
                    let fin = receive_header(io)?;
                    if fin.kind == ZFIN {
                        send_header(io, ZFIN, 0, true)?;
                        receive_oo(io)?;
                        progress(Progress {
                            path: &path,
                            transferred: wire.len() as u64,
                            total: Some(wire.len() as u64),
                            compressed,
                            complete: true,
                        })?;
                        return Ok(path);
                    }
                    if fin.kind == ZRQINIT {
                        send_header(io, ZRINIT, u32::from_le_bytes(init), true)?;
                    }
                }
            }
            ZFIN => {
                send_header(io, ZFIN, 0, true)?;
                receive_oo(io)?;
                if let Some((path, file)) = output.take() {
                    drop(file);
                    let _ = std::fs::remove_file(path);
                }
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "transfer ended before ZEOF",
                ));
            }
            ZABORT | ZFERR => {
                return Err(io::Error::new(
                    io::ErrorKind::Other,
                    "ZMODEM peer aborted transfer",
                ));
            }
            _ => {
                let _ = data_start;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn crc16_xmodem_known_vector() {
        assert_eq!(crc16(b"123456789"), 0x31c3);
    }
    #[test]
    fn escaped_zdle_roundtrips() {
        let mut bytes = Vec::new();
        escaped(&mut bytes, ZDLE).unwrap();
        assert_eq!(bytes, [ZDLE, 0x58]);
    }
    #[test]
    fn finish_sequence_accepts_optional_xon_but_requires_both_o_bytes() {
        let mut plain: &[u8] = b"OO";
        assert!(receive_oo(&mut plain).is_ok());

        let mut with_xon: &[u8] = b"\x11OO";
        assert!(receive_oo(&mut with_xon).is_ok());

        let mut malformed: &[u8] = b"OX";
        assert_eq!(
            receive_oo(&mut malformed).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
    #[test]
    fn receiver_skip_is_not_reported_as_success() {
        use std::os::unix::net::UnixStream;
        use std::time::{SystemTime, UNIX_EPOCH};
        let path = std::env::temp_dir().join(format!(
            "zmodem-skip-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::write(&path, b"do not overwrite the destination").unwrap();
        let (mut tx, mut rx) = UnixStream::pair().unwrap();
        let peer = std::thread::spawn(move || {
            send_header(&mut rx, ZRINIT, 0, true).unwrap();
            assert_eq!(receive_header(&mut rx).unwrap().kind, ZFILE);
            let _ = receive_data(&mut rx).unwrap();
            send_header(&mut rx, ZSKIP, 0, true).unwrap();
        });
        let mut completed = false;
        let result = send(
            &mut tx,
            &path,
            false,
            |_, _| Ok(()),
            |p| {
                completed |= p.complete;
                Ok(())
            },
        );
        peer.join().unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::Other);
        assert!(!completed, "skipped files must never show completion");
    }
    #[test]
    fn missing_finish_bytes_never_report_complete_progress() {
        use std::os::unix::net::UnixStream;
        use std::time::{SystemTime, UNIX_EPOCH};
        let directory = std::env::temp_dir().join(format!(
            "zmodem-finish-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir(&directory).unwrap();
        let (mut tx, mut rx) = UnixStream::pair().unwrap();
        let peer = std::thread::spawn(move || {
            assert_eq!(receive_header(&mut tx).unwrap().kind, ZRINIT);
            send_header(&mut tx, ZFILE, 0, false).unwrap();
            let mut metadata = b"DATA.BIN\0".to_vec();
            metadata.extend_from_slice(b"4\0");
            metadata.extend_from_slice(&[1, 0, 1, 0, 0, 0, 33, 0]);
            send_data(&mut tx, &metadata, ZCRCW).unwrap();
            assert_eq!(receive_header(&mut tx).unwrap().kind, ZRPOS);
            send_header(&mut tx, ZDATA, 0, false).unwrap();
            send_data(&mut tx, b"data", ZCRCW).unwrap();
            assert_eq!(receive_header(&mut tx).unwrap().kind, ZACK);
            send_header(&mut tx, ZEOF, 4, false).unwrap();
            assert_eq!(receive_header(&mut tx).unwrap().kind, ZRINIT);
            send_header(&mut tx, ZFIN, 0, true).unwrap();
            assert_eq!(receive_header(&mut tx).unwrap().kind, ZFIN);
            // Disconnect after all payload bytes, but before the final OO.
        });
        let mut completed = false;
        let mut transferred = 0;
        let result = receive(&mut rx, &directory, |p| {
            completed |= p.complete;
            transferred = p.transferred;
            Ok(())
        });
        peer.join().unwrap();
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::UnexpectedEof);
        assert_eq!(transferred, 4);
        assert!(!completed);
        assert_eq!(std::fs::read(directory.join("DATA.BIN")).unwrap(), b"data");
        std::fs::remove_dir_all(directory).unwrap();
    }
    #[test]
    fn invalid_metadata_never_creates_file_or_reports_complete() {
        use std::os::unix::net::UnixStream;
        use std::time::{SystemTime, UNIX_EPOCH};
        let directory = std::env::temp_dir().join(format!(
            "zmodem-metadata-error-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
        ));
        std::fs::create_dir_all(&directory).unwrap();
        for metadata in [
            [2, 0, 1, 0, 0, 0, 33, 0], // Unsupported version.
            [1, 0, 1, 0, 0, 0, 0, 0],  // Missing calendar date.
        ] {
            let (mut tx, mut rx) = UnixStream::pair().unwrap();
            let sender = std::thread::spawn(move || {
                assert_eq!(receive_header(&mut tx).unwrap().kind, ZRINIT);
                send_header(&mut tx, ZFILE, 0, false).unwrap();
                let mut packet = b"DATA.BIN\0".to_vec();
                packet.extend_from_slice(b"4\0");
                packet.extend_from_slice(&metadata);
                send_data(&mut tx, &packet, ZCRCW).unwrap();
            });
            let mut complete = false;
            let result = receive(&mut rx, &directory, |p| {
                complete |= p.complete;
                Ok(())
            });
            sender.join().unwrap();
            assert_eq!(result.unwrap_err().kind(), io::ErrorKind::InvalidData);
            assert!(!complete);
            assert!(!directory.join("DATA.BIN").exists());
        }
        std::fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn transfers_binary_files_in_both_modes() {
        use std::os::unix::fs::PermissionsExt;
        use std::os::unix::net::UnixStream;
        use std::time::{SystemTime, UNIX_EPOCH};

        for (compressed, readonly) in [(false, false), (false, true), (true, false), (true, true)] {
            let root = std::env::temp_dir().join(format!(
                "serial-zmodem-{}-{}-{}",
                std::process::id(),
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos(),
                compressed
            ));
            let receive_dir = root.join("received");
            std::fs::create_dir_all(&receive_dir).unwrap();
            let source = root.join("payload.bin");
            let mut input = Vec::with_capacity(70_000);
            for index in 0..65_536u32 {
                input.push((index.wrapping_mul(73).wrapping_add(index >> 3) & 0xff) as u8);
            }
            input.extend(std::iter::repeat_n(0x5a, 4_464));
            std::fs::write(&source, &input).unwrap();
            let mut metadata = [1, u8::from(readonly), 1, 0, 0, 0, 0, 0];
            metadata[4..6].copy_from_slice(&((12u16 << 11) | (34 << 5) | 28).to_le_bytes());
            metadata[6..8].copy_from_slice(&((44u16 << 9) | (2 << 5) | 29).to_le_bytes());
            let timestamp = DosMetadata::decode(&metadata).unwrap().modified().unwrap();
            File::options()
                .write(true)
                .open(&source)
                .unwrap()
                .set_modified(timestamp + std::time::Duration::from_secs(1))
                .unwrap();
            std::fs::set_permissions(
                &source,
                std::fs::Permissions::from_mode(if readonly { 0o440 } else { 0o640 }),
            )
            .unwrap();
            let baseline = receive_dir.join("umask-reference");
            std::fs::write(&baseline, b"").unwrap();
            let creation_mode = std::fs::metadata(&baseline).unwrap().permissions().mode() & 0o777;
            let (mut tx, mut rx) = UnixStream::pair().unwrap();
            let receiver_dir = receive_dir.clone();
            let receiver = std::thread::spawn(move || {
                let mut events = Vec::new();
                let result = receive(&mut rx, &receiver_dir, |p| {
                    events.push((p.transferred, p.total, p.complete));
                    Ok(())
                });
                (result, events)
            });
            let mut sent = Vec::new();
            let send_result = send(
                &mut tx,
                &source,
                compressed,
                |_, _| Ok(()),
                |p| {
                    sent.push((p.transferred, p.total, p.complete));
                    Ok(())
                },
            );
            let (receive_result, received) = receiver.join().unwrap();
            if let Err(error) = send_result {
                panic!(
                    "sender failed: {error}; receiver: {:?}",
                    receive_result.as_ref().err()
                );
            }
            let received_path = receive_result.unwrap();
            assert_eq!(std::fs::read(&received_path).unwrap(), input);
            let received_metadata = std::fs::metadata(&received_path).unwrap();
            assert_eq!(received_metadata.modified().unwrap(), timestamp);
            assert_eq!(
                received_metadata.permissions().mode() & 0o777,
                if readonly {
                    creation_mode & !0o222
                } else {
                    creation_mode
                }
            );
            let wire_size = if compressed {
                crate::zx0::compress(&input).len()
            } else {
                input.len()
            } as u64;
            for events in [&sent, &received] {
                assert_eq!(events[0].0, 0);
                assert_eq!(events.last(), Some(&(wire_size, Some(wire_size), true)));
                assert!(events[..events.len() - 1].iter().all(|p| !p.2));
                assert!(events.iter().all(|p| p.0 <= wire_size));
            }
            assert!(sent.iter().all(|p| p.1 == Some(wire_size)));
            assert!(received[..received.len() - 1].iter().all(|p| {
                p.1 == if compressed {
                    None
                } else {
                    Some(input.len() as u64)
                }
            }));
            std::fs::remove_dir_all(root).unwrap();
        }
    }

    #[test]
    fn negotiated_windows_preserve_payload_and_replay_rejected_bursts() {
        use std::os::unix::net::UnixStream;
        use std::time::{SystemTime, UNIX_EPOCH};

        for advertised in [0u32, 512, 61440] {
            for compressed in [false, true] {
                let path = std::env::temp_dir().join(format!(
                    "zmodem-window-{}-{}",
                    std::process::id(),
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_nanos(),
                ));
                let mut random = 0x12345678u32;
                let input: Vec<u8> = (0..150_001)
                    .map(|_| {
                        random ^= random << 13;
                        random ^= random >> 17;
                        random ^= random << 5;
                        random as u8
                    })
                    .collect();
                std::fs::write(&path, &input).unwrap();
                let expected = if compressed {
                    crate::zx0::compress(&input)
                } else {
                    input
                };
                let wire_len = expected.len();
                let (mut tx, mut rx) = UnixStream::pair().unwrap();
                rx.set_read_timeout(Some(Duration::from_secs(30))).unwrap();
                tx.set_read_timeout(Some(Duration::from_secs(30))).unwrap();
                let peer = std::thread::spawn(move || {
                    send_header(&mut rx, ZRINIT, advertised | 0xc000_0000, true).unwrap();
                    assert_eq!(receive_header(&mut rx).unwrap().kind, ZFILE);
                    let (_, marker) = receive_data(&mut rx).unwrap();
                    assert_eq!(marker, ZCRCW);
                    send_header(&mut rx, ZRPOS, 0, true).unwrap();
                    let capacity = if advertised == 0 {
                        CHUNK
                    } else {
                        advertised as usize
                    };
                    let mut offset = 0;
                    let mut reject = advertised > 512;
                    while offset < expected.len() {
                        let header = receive_header(&mut rx).unwrap();
                        assert_eq!((header.kind, header.pos), (ZDATA, offset as u32));
                        let boundary = (offset + capacity).min(expected.len());
                        let mut cursor = offset;
                        while cursor < boundary {
                            let (packet, marker) = receive_data(&mut rx).unwrap();
                            let end = (cursor + CHUNK).min(boundary);
                            assert_eq!(packet, expected[cursor..end]);
                            assert_eq!(marker, if end == boundary { ZCRCW } else { ZCRCG });
                            cursor = end;
                        }
                        if reject {
                            reject = false;
                            send_header(&mut rx, ZRPOS, offset as u32, true).unwrap();
                        } else {
                            offset = boundary;
                            send_header(&mut rx, ZACK, offset as u32, true).unwrap();
                        }
                    }
                    let eof = receive_header(&mut rx).unwrap();
                    assert_eq!((eof.kind, eof.pos), (ZEOF, expected.len() as u32));
                    send_header(&mut rx, ZRINIT, advertised, true).unwrap();
                    assert_eq!(receive_header(&mut rx).unwrap().kind, ZFIN);
                    send_header(&mut rx, ZFIN, 0, true).unwrap();
                    receive_oo(&mut rx).unwrap();
                });
                let mut events = Vec::new();
                let result = send(
                    &mut tx,
                    &path,
                    compressed,
                    |_, flags| {
                        assert_eq!(flags, 0xc0);
                        Ok(())
                    },
                    |p| {
                        events.push((p.transferred, p.complete));
                        Ok(())
                    },
                );
                drop(tx);
                peer.join().unwrap();
                std::fs::remove_file(path).unwrap();
                result.unwrap();
                assert_eq!(events.last(), Some(&(wire_len as u64, true)));
                assert!(events[..events.len() - 1].iter().all(|p| !p.1));
                if advertised > 512 {
                    assert!(
                        events.contains(&(512, false)),
                        "progress inside a live burst"
                    );
                    assert!(events.windows(2).any(|pair| pair[1].0 < pair[0].0));
                }
            }
        }
    }

    #[test]
    fn invalid_acknowledgments_and_repeated_rejections_fail_without_completion() {
        use std::os::unix::net::UnixStream;
        use std::time::{SystemTime, UNIX_EPOCH};

        for reject in [false, true] {
            let path = std::env::temp_dir().join(format!(
                "zmodem-ack-{}-{}",
                std::process::id(),
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos(),
            ));
            std::fs::write(&path, b"payload").unwrap();
            let (mut tx, mut rx) = UnixStream::pair().unwrap();
            rx.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
            tx.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
            let peer = std::thread::spawn(move || {
                send_header(&mut rx, ZRINIT, 61440, true).unwrap();
                assert_eq!(receive_header(&mut rx).unwrap().kind, ZFILE);
                receive_data(&mut rx).unwrap();
                send_header(&mut rx, ZRPOS, 0, true).unwrap();
                for _ in 0..if reject { 6 } else { 1 } {
                    assert_eq!(receive_header(&mut rx).unwrap().kind, ZDATA);
                    assert_eq!(receive_data(&mut rx).unwrap(), (b"payload".to_vec(), ZCRCW));
                    send_header(&mut rx, if reject { ZRPOS } else { ZACK }, 0, true).unwrap();
                }
            });
            let mut complete = false;
            let result = send(
                &mut tx,
                &path,
                false,
                |_, _| Ok(()),
                |p| {
                    complete |= p.complete;
                    Ok(())
                },
            );
            drop(tx);
            peer.join().unwrap();
            std::fs::remove_file(path).unwrap();
            assert!(result.is_err());
            assert!(!complete);
        }
    }
}
