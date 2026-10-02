use crossterm::{
    cursor::MoveToColumn,
    event::{self, Event, KeyCode, KeyEvent, KeyModifiers},
    queue,
    terminal::{Clear, ClearType, disable_raw_mode, enable_raw_mode, size},
};
use serialport::SerialPort;
use std::{
    fmt::Write as _,
    io::{self, Read, Write},
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

const MARKER_PREFIX: &[u8] = b"\x1b]777;BAUD;";
const TRANSFER_PREFIX: &[u8] = b"**\x18B";

struct RawMode;
impl RawMode {
    fn enter() -> io::Result<Self> {
        enable_raw_mode()?;
        Ok(Self)
    }
}
impl Drop for RawMode {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
    }
}

pub fn run(
    mut port: Box<dyn SerialPort>,
    send_file: Option<PathBuf>,
    receive_dir: PathBuf,
    experimental_baud: Option<u32>,
) -> io::Result<()> {
    let _raw = RawMode::enter()?;
    let mut stdout = io::stdout().lock();
    let mut buf = [0u8; 1024];
    let mut hold = Vec::new();
    let mut command = false;
    loop {
        match port.read(&mut buf) {
            Ok(n) => {
                for (index, byte) in buf[..n].iter().enumerate() {
                    hold.push(*byte);
                    loop {
                        if hold.starts_with(MARKER_PREFIX) {
                            if let Some(end) = hold.iter().position(|b| *b == 7) {
                                let text = String::from_utf8_lossy(&hold[MARKER_PREFIX.len()..end]);
                                if let Ok(new_baud) = text.parse::<u32>() {
                                    if valid_baud(new_baud) {
                                        stdout.flush()?;
                                        std::thread::sleep(Duration::from_millis(300));
                                        port.set_baud_rate(new_baud).map_err(io::Error::other)?;
                                        let _ = writeln!(
                                            io::stderr(),
                                            "\r\n[serial speed: {new_baud}]\r"
                                        );
                                    }
                                }
                                hold.drain(..=end);
                                continue;
                            }
                            break;
                        }
                        if hold.starts_with(TRANSFER_PREFIX) {
                            // A complete hex header is prefix + 14 hex digits + CRLF.
                            if hold.len() < 20 {
                                break;
                            }
                            if &hold[4..6] == b"00" {
                                // ZRQINIT: the DOS sender is asking the host to receive.
                                stdout.flush()?;
                                hold.extend_from_slice(&buf[index + 1..n]);
                                let mut replay = ReplayPort {
                                    prefix: std::mem::take(&mut hold),
                                    port: &mut *port,
                                    transfer_deadline: None,
                                };
                                match download(&mut replay, &receive_dir) {
                                    Ok(path) => eprintln!("\r\n[received {}]\r", path.display()),
                                    Err(e) => eprintln!("\r\n[receive failed: {e}]\r"),
                                }
                                hold.clear();
                                break;
                            }
                            if &hold[4..6] == b"01" {
                                // ZRINIT: the DOS receiver is ready for the selected file.
                                if let Some(path) = send_file.as_deref() {
                                    stdout.flush()?;
                                    hold.extend_from_slice(&buf[index + 1..n]);
                                    let mut replay = ReplayPort {
                                        prefix: std::mem::take(&mut hold),
                                        port: &mut *port,
                                        transfer_deadline: None,
                                    };
                                    let result = upload(&mut replay, path, experimental_baud);
                                    if let Err(e) = result {
                                        eprintln!("\r\n[send failed: {e}]\r");
                                    }
                                    hold.clear();
                                    break;
                                }
                            }
                        }
                        if is_prefix(&hold, MARKER_PREFIX) || is_prefix(&hold, TRANSFER_PREFIX) {
                            break;
                        }
                        stdout.write_all(&[hold.remove(0)])?;
                    }
                }
                // DOS prompts often have no newline; display them immediately.
                stdout.flush()?;
            }
            Err(e)
                if e.kind() == io::ErrorKind::TimedOut || e.kind() == io::ErrorKind::WouldBlock => {
            }
            Err(e) => return Err(e),
        }
        if event::poll(Duration::from_millis(1))? {
            if let Event::Key(key) = event::read()? {
                if command {
                    command = false;
                    match key {
                        KeyEvent {
                            code: KeyCode::Char('?'),
                            ..
                        } => eprintln!(
                            "\r\nCtrl-] commands: ? help, q quit, s send, r receive, b baud, ] send Ctrl-] literally\r"
                        ),
                        KeyEvent {
                            code: KeyCode::Char('q'),
                            ..
                        } => break,
                        KeyEvent {
                            code: KeyCode::Char(']'),
                            ..
                        } => port.write_all(&[0x1d])?,
                        KeyEvent {
                            code: KeyCode::Char('s'),
                            ..
                        } => {
                            if let Some(path) = send_file.as_deref() {
                                let mut io = ReplayPort {
                                    prefix: Vec::new(),
                                    port: &mut *port,
                                    transfer_deadline: None,
                                };
                                if let Err(e) = upload(&mut io, path, experimental_baud) {
                                    eprintln!("\r\n[send failed: {e}]\r");
                                }
                            } else {
                                eprintln!("\r\n[restart with --send FILE to select an upload]\r");
                            }
                        }
                        KeyEvent {
                            code: KeyCode::Char('r'),
                            ..
                        } => {
                            let mut io = ReplayPort {
                                prefix: Vec::new(),
                                port: &mut *port,
                                transfer_deadline: None,
                            };
                            match download(&mut io, &receive_dir) {
                                Ok(path) => eprintln!("\r\n[received {}]\r", path.display()),
                                Err(e) => eprintln!("\r\n[receive failed: {e}]\r"),
                            }
                        }
                        KeyEvent {
                            code: KeyCode::Char('b'),
                            ..
                        } => {
                            eprintln!("\r\nBaud change: type digits then Enter, Esc cancels\r");
                            let mut digits = String::new();
                            loop {
                                if let Event::Key(k) = event::read()? {
                                    match k.code {
                                        KeyCode::Char(c) if c.is_ascii_digit() => {
                                            digits.push(c);
                                        }
                                        KeyCode::Enter => break,
                                        KeyCode::Esc => {
                                            digits.clear();
                                            break;
                                        }
                                        _ => {}
                                    }
                                }
                            }
                            if let Ok(rate) = digits.parse::<u32>() {
                                if valid_baud(rate) {
                                    port.set_baud_rate(rate).map_err(io::Error::other)?;
                                }
                            }
                        }
                        _ => {}
                    }
                // Unix crossterm reports the legacy 0x1D byte as Ctrl-5.
                // Enhanced keyboard protocols can report Ctrl-] directly.
                } else if key.modifiers.contains(KeyModifiers::CONTROL)
                    && matches!(key.code, KeyCode::Char(']' | '5'))
                {
                    command = true;
                } else {
                    let bytes: &[u8] = match key.code {
                        KeyCode::Char(c) if key.modifiers.contains(KeyModifiers::CONTROL) => {
                            let ch = (c as u8) & 0x1f;
                            port.write_all(&[ch])?;
                            continue;
                        }
                        KeyCode::Char(c) => {
                            let mut b = [0; 4];
                            port.write_all(c.encode_utf8(&mut b).as_bytes())?;
                            continue;
                        }
                        KeyCode::Enter => b"\r",
                        KeyCode::Backspace => b"\x08",
                        KeyCode::Tab => b"\t",
                        KeyCode::Esc => b"\x1b",
                        _ => continue,
                    };
                    port.write_all(bytes)?;
                }
            }
        }
    }
    stdout.flush()?;
    Ok(())
}

fn upload(io: &mut ReplayPort<'_>, path: &Path, experimental: Option<u32>) -> io::Result<()> {
    let mut bar = ProgressBar::new("Send", io::stderr().lock())?;
    // USB drivers can wait for their whole TX queue, not just this packet, to
    // drain before accepting more. Keep the terminal's short timeout out of
    // binary uploads; restore it on every exit.
    let previous_timeout = io.port.timeout();
    io.port
        .set_timeout(Duration::from_secs(30))
        .map_err(io::Error::other)?;
    let mut restore_baud = false;
    let result = serial_zmodem::zmodem::send(
        io,
        path,
        true,
        |io, capabilities| {
            if let Some(maximum) = experimental.filter(|_| capabilities & 0x40 != 0) {
                restore_baud = true;
                eprintln!("\r\n[training link from 9600 baud]\r");
                let rate = serial_zmodem::baud_training::train(io.port, maximum)?;
                io.transfer_deadline = Some(Instant::now() + Duration::from_secs(15));
                eprintln!("[transfer speed: {rate} baud; console returns to 9600]\r");
            }
            Ok(())
        },
        |progress| bar.update(progress),
    );
    let restored_baud = if restore_baud {
        // Allow OO to leave the USB/UART buffers before the receiver restores
        // its console after four BIOS ticks.
        std::thread::sleep(Duration::from_millis(50));
        io.port.set_baud_rate(9600).map_err(io::Error::other)
    } else {
        Ok(())
    };
    io.transfer_deadline = None;
    let restored_timeout = io
        .port
        .set_timeout(previous_timeout)
        .map_err(io::Error::other);
    restored_baud?;
    restored_timeout?;
    result
}

fn download(io: &mut ReplayPort<'_>, directory: &Path) -> io::Result<PathBuf> {
    let mut bar = ProgressBar::new("Recv", io::stderr().lock())?;
    serial_zmodem::zmodem::receive(io, directory, |progress| bar.update(progress))
}

struct ProgressBar<W: Write> {
    output: W,
    direction: &'static str,
    started: Instant,
    last_redraw: Option<Instant>,
    line: String,
}

impl<W: Write> ProgressBar<W> {
    fn new(direction: &'static str, mut output: W) -> io::Result<Self> {
        write!(output, "\r\n{direction}: waiting for peer...")?;
        output.flush()?;
        Ok(Self {
            output,
            direction,
            started: Instant::now(),
            last_redraw: None,
            line: String::with_capacity(160),
        })
    }

    fn update(&mut self, progress: serial_zmodem::zmodem::Progress<'_>) -> io::Result<()> {
        let now = Instant::now();
        if self.last_redraw.is_none() {
            self.started = now;
        }
        let finishing = progress
            .total
            .is_some_and(|total| progress.transferred >= total);
        if !progress.complete
            && !finishing
            && self
                .last_redraw
                .is_some_and(|last| now.duration_since(last) < Duration::from_millis(100))
        {
            return Ok(());
        }
        self.last_redraw = Some(now);
        let width = size()
            .ok()
            .map(|(width, _)| width)
            .filter(|&width| width > 0)
            .unwrap_or(80);
        let columns = usize::from(width).saturating_sub(1);
        let percent = if progress.complete {
            Some(100)
        } else {
            progress.total.map(|total| {
                if total == 0 {
                    0
                } else {
                    (progress.transferred.saturating_mul(100) / total).min(99)
                }
            })
        };
        let mut cells = [b'-'; 16];
        if let Some(percent) = percent {
            let filled = percent as usize * cells.len() / 100;
            cells[..filled].fill(b'=');
            if filled < cells.len() {
                cells[filled] = b'>';
            }
        } else {
            // Encoded receive length is not in ZFILE metadata; never invent a
            // percentage from the decoded size of a compressed file.
            cells[(progress.transferred / 512) as usize % 16] = b'>';
        }
        self.line.clear();
        write!(&mut self.line, "{} ", self.direction).unwrap();
        let name = progress
            .path
            .file_name()
            .unwrap_or(progress.path.as_os_str())
            .to_string_lossy();
        for ch in name
            .chars()
            .flat_map(char::escape_default)
            .take(columns.saturating_sub(63).min(24))
        {
            self.line.push(ch);
        }
        write!(
            &mut self.line,
            " [{}] ",
            std::str::from_utf8(&cells).unwrap()
        )
        .unwrap();
        if let Some(percent) = percent {
            write!(&mut self.line, "{percent:3}% ").unwrap();
        } else {
            self.line.push_str(" --% ");
        }
        write!(&mut self.line, "{}", progress.transferred).unwrap();
        if let Some(total) = progress.total {
            write!(&mut self.line, "/{total}").unwrap();
        } else {
            self.line.push_str("/?");
        }
        self.line.push_str(if progress.compressed {
            " wire B "
        } else {
            " B "
        });
        if finishing && !progress.complete {
            self.line.push_str("finishing");
        } else {
            let seconds = now.duration_since(self.started).as_secs_f64().max(0.001);
            write!(
                &mut self.line,
                "{:.1} KiB/s",
                progress.transferred as f64 / seconds / 1024.0
            )
            .unwrap();
        }
        // Everything in the rendered line is ASCII, including escaped names.
        // Leave the last terminal column unused to prevent line wrapping.
        self.line.truncate(columns);
        queue!(
            &mut self.output,
            MoveToColumn(0),
            Clear(ClearType::CurrentLine)
        )?;
        self.output.write_all(self.line.as_bytes())?;
        self.output.flush()
    }
}

impl<W: Write> Drop for ProgressBar<W> {
    fn drop(&mut self) {
        // Never leave the next DOS prompt or an error message on the bar line.
        let _ = write!(&mut self.output, "\r\n");
        let _ = self.output.flush();
    }
}

fn valid_baud(rate: u32) -> bool {
    matches!(rate, 1200 | 2400 | 9600 | 19200 | 38400 | 57600 | 115200)
}
fn is_prefix(data: &[u8], prefix: &[u8]) -> bool {
    data.len() <= prefix.len() && prefix.starts_with(data)
}

struct ReplayPort<'a> {
    prefix: Vec<u8>,
    port: &'a mut dyn SerialPort,
    // Experimental transfers must not hang on a failed fast link. Allow the
    // queued burst's wire time plus a fixed response bound, never RX noise.
    transfer_deadline: Option<Instant>,
}
impl ReplayPort<'_> {
    fn check_deadline(&self) -> io::Result<()> {
        if self
            .transfer_deadline
            .is_some_and(|deadline| Instant::now() >= deadline)
        {
            return Err(io::Error::new(
                io::ErrorKind::ConnectionAborted,
                "experimental transfer stalled; restoring 9600 baud",
            ));
        }
        Ok(())
    }
}
impl Read for ReplayPort<'_> {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        self.check_deadline()?;
        if !self.prefix.is_empty() {
            let n = out.len().min(self.prefix.len());
            out[..n].copy_from_slice(&self.prefix[..n]);
            self.prefix.drain(..n);
            return Ok(n);
        }
        self.port.read(out)
    }
}
impl Write for ReplayPort<'_> {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.check_deadline()?;
        let n = self.port.write(data)?;
        if n > 0 && self.transfer_deadline.is_some() {
            let pending = self.port.bytes_to_write().map_err(io::Error::other)?;
            let baud = self.port.baud_rate().map_err(io::Error::other)?.max(1);
            let wire_time = Duration::from_secs_f64(f64::from(pending) * 10.0 / f64::from(baud));
            self.transfer_deadline = Some(Instant::now() + wire_time + Duration::from_secs(15));
        }
        Ok(n)
    }
    fn flush(&mut self) -> io::Result<()> {
        if self.transfer_deadline.is_some() {
            while self.port.bytes_to_write().map_err(io::Error::other)? != 0 {
                self.check_deadline()?;
                std::thread::sleep(Duration::from_millis(5));
            }
            Ok(())
        } else {
            self.port.flush()
        }
    }
}
