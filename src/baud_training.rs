//! Opt-in, pre-transfer baud training for ZRECVX. Control exchanges use 9600;
//! every probe returns there before selecting a speed. No file data is sent here.
use serialport::{ClearBuffer, SerialPort};
use std::io::{self, Write};
use std::time::{Duration, Instant};

const BASE: u32 = 9600;
const RATES: [u32; 5] = [BASE, 19200, 38400, 57600, 115200];
const REPLY_TIMEOUT: Duration = Duration::from_secs(2);

/// Train an advertised ZRECVX/ZRECVY receiver, returning the verified baud rate.
/// `maximum` is an inclusive ceiling chosen from the supported candidate rates.
/// The caller must start at 9600 and restore 9600 after the file transfer.
/// On failure this function restores 9600 itself. Ordinary ZRECV must never
/// receive this request; its ZRINIT does not advertise capability bit 0x40.
pub fn train(port: &mut dyn SerialPort, maximum: u32) -> io::Result<u32> {
    if !RATES.contains(&maximum) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "unsupported training ceiling",
        ));
    }
    if port.baud_rate().map_err(io::Error::other)? != BASE {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "experimental baud training must start at 9600",
        ));
    }
    let previous_timeout = port.timeout();
    port.set_timeout(Duration::from_millis(50))
        .map_err(io::Error::other)?;
    let result = train_inner(port, maximum);
    let restore_timeout = port.set_timeout(previous_timeout).map_err(io::Error::other);
    if result.is_err() || restore_timeout.is_err() {
        // The DOS peer also has an idle/first-header deadline, so a lost X
        // cannot strand it indefinitely at a trial or locked speed.
        let _ = port.clear(ClearBuffer::All);
        let restore_baud = port.set_baud_rate(BASE).map_err(io::Error::other);
        if restore_baud.is_ok() {
            let _ = port.clear(ClearBuffer::Input);
            let _ = port.write_all(b"X");
        }
        restore_baud?;
    }
    let rate = result?;
    restore_timeout?;
    Ok(rate)
}

fn train_inner(port: &mut dyn SerialPort, maximum: u32) -> io::Result<u32> {
    crate::zmodem::send_header(&mut ControlWriter(port), 18, 0x3154425a, true)?;
    expect(port, b"BT1\n", Instant::now() + REPLY_TIMEOUT)?;
    let mut best = None;
    for (index, rate) in RATES.iter().copied().enumerate() {
        if rate > maximum {
            break;
        }
        let index = index as u8;
        let passed = probe(port, index, rate).is_ok();
        // Reconcile at the known-good rate even when a high-speed reply was
        // corrupted or lost. A one-sided successful probe is never enough.
        port.clear(ClearBuffer::All).map_err(io::Error::other)?;
        port.set_baud_rate(BASE).map_err(io::Error::other)?;
        port.clear(ClearBuffer::Input).map_err(io::Error::other)?;
        let receiver_passed = status(port, index)?;
        if !passed || !receiver_passed {
            break;
        }
        best = Some(index);
    }
    let index = best.ok_or_else(|| io::Error::other("baud training failed even at 9600"))?;
    port.write_all(&[b'L', index])?;
    expect(port, &[b'L', index], Instant::now() + REPLY_TIMEOUT)?;
    let rate = RATES[usize::from(index)];
    port.set_baud_rate(rate).map_err(io::Error::other)?;
    // DOS drains the baseline acknowledgement, switches, and resumes ZMODEM.
    std::thread::sleep(Duration::from_millis(400));
    Ok(rate)
}

fn probe(port: &mut dyn SerialPort, index: u8, rate: u32) -> io::Result<()> {
    port.write_all(&[b'P', index])?;
    expect(port, &[b'A', index], Instant::now() + REPLY_TIMEOUT)?;
    port.set_baud_rate(rate).map_err(io::Error::other)?;
    expect(port, &[b'R', index], Instant::now() + REPLY_TIMEOUT)?;
    let mut block = [0u8; 512];
    for round in 0..4u8 {
        for (i, byte) in block.iter_mut().enumerate() {
            *byte = (i as u8)
                .wrapping_mul(73)
                .wrapping_add(round.wrapping_mul(29))
                .wrapping_add(index.wrapping_mul(19));
        }
        // Exercise the same escaping/CRC path as file data, not a cheaper
        // UART echo that could overestimate the DOS CPU's receive capacity.
        crate::zmodem::send_data(&mut ControlWriter(port), &block, 0x6b)?;
        expect(port, &[b'K', index, round], Instant::now() + REPLY_TIMEOUT)?;
    }
    Ok(())
}

fn status(port: &mut dyn SerialPort, index: u8) -> io::Result<bool> {
    let deadline = Instant::now() + Duration::from_secs(4);
    while Instant::now() < deadline {
        port.write_all(&[b'S', index])?;
        let attempt = (Instant::now() + Duration::from_millis(250)).min(deadline);
        match expect(port, &[b'B', index], attempt) {
            Ok(()) => match read_byte(port, attempt) {
                Ok(0) => return Ok(false),
                Ok(1) => return Ok(true),
                Ok(_) => continue,
                Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(e),
            },
            Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e),
        }
    }
    Err(io::Error::new(
        io::ErrorKind::TimedOut,
        "baud training did not recover at 9600",
    ))
}

fn expect(port: &mut dyn SerialPort, expected: &[u8], deadline: Instant) -> io::Result<()> {
    let mut matched = 0;
    while matched < expected.len() {
        let byte = read_byte(port, deadline)?;
        if byte == expected[matched] {
            matched += 1;
        } else {
            matched = usize::from(byte == expected[0]);
        }
    }
    Ok(())
}

fn read_byte(port: &mut dyn SerialPort, deadline: Instant) -> io::Result<u8> {
    let mut byte = [0];
    while Instant::now() < deadline {
        match port.read(&mut byte) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "training peer closed",
                ));
            }
            Ok(_) => return Ok(byte[0]),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut
                        | io::ErrorKind::WouldBlock
                        | io::ErrorKind::Interrupted
                ) => {}
            Err(e) => return Err(e),
        }
    }
    Err(io::Error::new(
        io::ErrorKind::TimedOut,
        "baud training reply timed out",
    ))
}

// SerialPort::flush can wait indefinitely for CTS on some drivers. The initial
// protocol header needs only a bounded driver-queue drain; subsequent commands
// are acknowledged by the receiver before any baud change.
struct ControlWriter<'a>(&'a mut dyn SerialPort);
impl Write for ControlWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.0.write(bytes)
    }
    fn flush(&mut self) -> io::Result<()> {
        let deadline = Instant::now() + REPLY_TIMEOUT;
        while self.0.bytes_to_write().map_err(io::Error::other)? != 0 {
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "training UART output stalled",
                ));
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        Ok(())
    }
}
