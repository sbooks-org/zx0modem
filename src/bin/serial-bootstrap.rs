//! Explicit-port, printable DOS bootstrap followed by binary ZX0 and ZMODEM.
use serial_zmodem::{zmodem, zx0};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::collections::VecDeque;
use std::env;
use std::fs;
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender};
use std::thread;
use std::time::{Duration, Instant};

const HELP: &str = "serial-bootstrap --port DEVICE [OPTIONS]
serial-bootstrap --dry-run [OPTIONS]

Bootstrap a DOS machine already at a CTTY COM1 command prompt. Requires a
null-modem connection with RTS/CTS; both ends must initially use --baud, 8N1.
The explicit live command authorizes opening only DEVICE; no port discovery.

Options:
  --port DEVICE          Required for live operation; never auto-select a port
  --baud RATE            Initial DOS command-prompt speed (default: 9600)
  --dir DIRECTORY        Built bootstrap bundle (default: build/dos)
  --input-device DEVICE  DOS COPY source: COM1 (default) or CON for serial BIOS
  --char-delay-ms N       ASCII pacing (default: 10 for COM1, 60 for CON)
  --line-delay-ms N       Additional ASCII line pacing (default: 250)
  --resume               Skip COPY; launch the existing DOS BOOT.COM
  --dry-run              Validate/prepare the complete bundle and print sizes;
                         never enumerate or open serial ports
  --help, -h             Show this help without accessing serial ports

Required files: BOOTSTRAP.COM, BOOT2.BIN, ZRECV.COM, BAUDRATE.COM, ZSEND.COM.
BOOTSTRAP.COM must be printable ASCII with CRLF lines of at most 78 columns.
Live stages: COPY COM1 /A BOOT.COM /B, ASCII plus Ctrl-Z/CR, BOOT,
switch to 9600, checksum-protected BOOT2, acknowledged ZX0 blocks installing
ZRECV.COM, then ZMODEM transfers of BAUDRATE.COM and ZSEND.COM at 9600.
Except BOOT.COM with --resume, DOS destination files must be absent; skips fail.
No automatic retries of the installation or changes to the final DOS baud.
Console handoffs recognize standard DOS prompts (C> or C:\\PATH>), including
ANSI cursor positioning. --resume requires BOOT.COM from this same bundle.
Protocol stages have finite deadlines, including ZMODEM and output draining.
";

struct Options {
    port: Option<String>,
    baud: u32,
    directory: PathBuf,
    input_device: &'static str,
    char_delay: Duration,
    line_delay: Duration,
    dry_run: bool,
    resume: bool,
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message.into())
}

fn options() -> io::Result<Option<Options>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.iter().any(|arg| arg == "--help" || arg == "-h") {
        print!("{HELP}");
        return Ok(None);
    }
    let mut result = Options {
        port: None,
        baud: 9600,
        directory: PathBuf::from("build/dos"),
        input_device: "COM1",
        char_delay: Duration::from_millis(10),
        line_delay: Duration::from_millis(250),
        dry_run: false,
        resume: false,
    };
    let mut explicit_char_delay = false;
    let mut args = args.iter();
    while let Some(arg) = args.next() {
        if arg == "--dry-run" {
            result.dry_run = true;
            continue;
        }
        if arg == "--resume" {
            result.resume = true;
            continue;
        }
        if !matches!(
            arg.as_str(),
            "--port"
                | "--baud"
                | "--dir"
                | "--input-device"
                | "--char-delay-ms"
                | "--line-delay-ms"
        ) {
            return Err(invalid(format!("unknown option {arg:?}; use --help")));
        }
        let value = args
            .next()
            .ok_or_else(|| invalid(format!("{arg} requires a value")))?;
        match arg.as_str() {
            "--port" => {
                if value.is_empty() || value.starts_with("--") {
                    return Err(invalid("--port requires an explicit device name"));
                }
                result.port = Some(value.clone());
            }
            "--dir" => result.directory = PathBuf::from(value),
            "--input-device" => {
                result.input_device = match value.to_ascii_uppercase().as_str() {
                    "COM1" => "COM1",
                    "CON" => "CON",
                    _ => return Err(invalid("--input-device must be COM1 or CON")),
                };
            }
            "--baud" => {
                result.baud = value
                    .parse()
                    .map_err(|_| invalid("--baud requires a positive integer"))?;
                if result.baud == 0 {
                    return Err(invalid("--baud must not be zero"));
                }
            }
            "--char-delay-ms" | "--line-delay-ms" => {
                let millis: u64 = value
                    .parse()
                    .map_err(|_| invalid(format!("{arg} requires a nonnegative integer")))?;
                let delay = Duration::from_millis(millis);
                if arg == "--char-delay-ms" {
                    result.char_delay = delay;
                    explicit_char_delay = true;
                } else {
                    result.line_delay = delay;
                }
            }
            _ => unreachable!(),
        }
    }
    // A serial BIOS may only fill its small keyboard queue once per 55 ms
    // timer tick. COPY CON also competes with ANSI echo output on that UART.
    if result.input_device == "CON" && !explicit_char_delay {
        result.char_delay = Duration::from_millis(60);
    }
    if !result.dry_run && result.port.is_none() {
        return Err(invalid(
            "live operation requires --port DEVICE; use --dry-run for offline preparation",
        ));
    }
    Ok(Some(result))
}

struct Bundle {
    bootstrap: Vec<u8>,
    boot2: Vec<u8>,
    receiver_size: usize,
    receiver_wire: Vec<u8>,
    receiver_blocks: usize,
    baudrate: (PathBuf, usize),
    sender: (PathBuf, usize),
}

fn load(directory: &Path, name: &str, max_size: usize) -> io::Result<Vec<u8>> {
    // These fixed, uppercase DOS 8.3 names are also the names sent to the target.
    let path = directory.join(name);
    let metadata = fs::metadata(&path)
        .map_err(|e| io::Error::new(e.kind(), format!("{}: {e}", path.display())))?;
    if !metadata.is_file() || metadata.len() == 0 || metadata.len() > max_size as u64 {
        return Err(invalid(format!(
            "{} must be a regular file of 1..={max_size} bytes",
            path.display()
        )));
    }
    let bytes = fs::read(&path)?;
    if bytes.is_empty() || bytes.len() > max_size {
        return Err(invalid(format!(
            "{} changed size during preparation",
            path.display()
        )));
    }
    if name.ends_with(".COM") && (bytes.starts_with(b"MZ") || bytes.starts_with(b"ZM")) {
        return Err(invalid(format!(
            "{} contains an EXE header, not a flat COM program",
            path.display()
        )));
    }
    Ok(bytes)
}

fn validate_printable(bytes: &[u8]) -> io::Result<()> {
    let mut column = 0;
    let mut at = 0;
    while at < bytes.len() {
        match bytes[at] {
            b'\r' if bytes.get(at + 1) == Some(&b'\n') => {
                column = 0;
                at += 2;
            }
            0x20..=0x7e => {
                column += 1;
                if column > 78 {
                    return Err(invalid("BOOTSTRAP.COM has a line longer than 78 columns"));
                }
                at += 1;
            }
            _ => {
                return Err(invalid(format!(
                    "BOOTSTRAP.COM contains a non-printable byte or invalid CRLF at offset {at}"
                )));
            }
        }
    }
    if !bytes.ends_with(b"\r\n") {
        return Err(invalid(
            "BOOTSTRAP.COM must end in CRLF before the COPY Ctrl-Z terminator",
        ));
    }
    Ok(())
}

impl Bundle {
    fn prepare(directory: &Path) -> io::Result<Self> {
        let bootstrap = load(directory, "BOOTSTRAP.COM", 65280)?;
        validate_printable(&bootstrap)?;
        let boot2 = load(directory, "BOOT2.BIN", 8192)?;
        let receiver = load(directory, "ZRECV.COM", 60000)?;
        let baudrate = load(directory, "BAUDRATE.COM", 65280)?;
        let sender = load(directory, "ZSEND.COM", 65280)?;
        let receiver_wire = zx0::compress(&receiver);
        if zx0::decompress(&receiver_wire, receiver.len())? != receiver {
            return Err(invalid("ZRECV.COM failed ZX0 round-trip validation"));
        }
        let receiver_blocks = receiver.len().div_ceil(zx0::BLOCK_SIZE);
        Ok(Self {
            bootstrap,
            boot2,
            receiver_size: receiver.len(),
            receiver_wire,
            receiver_blocks,
            baudrate: (directory.join("BAUDRATE.COM"), baudrate.len()),
            sender: (directory.join("ZSEND.COM"), sender.len()),
        })
    }

    fn describe(&self, options: &Options) {
        println!("Validated bundle: {}", options.directory.display());
        if options.resume {
            println!("1. Resume: keep existing BOOT.COM; wait for the DOS prompt and launch BOOT");
        } else {
            println!(
                "1. COPY {} /A BOOT.COM /B: {} literal ASCII bytes, {} CRLF lines; {} baud, {} ms/byte + {} ms/line",
                options.input_device,
                self.bootstrap.len(),
                self.bootstrap.iter().filter(|&&b| b == b'\n').count(),
                options.baud,
                options.char_delay.as_millis(),
                options.line_delay.as_millis()
            );
        }
        println!(
            "2. B1> at 9600 baud: BOOT2.BIN {} bytes, {} wire bytes including length/checksum",
            self.boot2.len(),
            self.boot2.len() + 4
        );
        println!(
            "3. B2>: ZRECV.COM {} decoded bytes, {} ZX0-envelope bytes, {} blocks, {} wire bytes including checksums/total length",
            self.receiver_size,
            self.receiver_wire.len(),
            self.receiver_blocks,
            self.receiver_wire.len() + 2 * self.receiver_blocks + 2
        );
        println!(
            "4. B3>: ZRECV + ZMODEM BAUDRATE.COM ({} bytes; ZX0 negotiated)",
            self.baudrate.1
        );
        println!(
            "5. ZRECV + ZMODEM ZSEND.COM ({} bytes; ZX0 negotiated)",
            self.sender.1
        );
    }
}

fn checksum(bytes: &[u8]) -> u16 {
    bytes
        .iter()
        .fold(0u16, |sum, &byte| sum.wrapping_add(u16::from(byte)))
}

fn wire_budget(size: usize, baud: u32) -> Duration {
    Duration::from_secs(60 + (size as u64 * 40).div_ceil(u64::from(baud)))
}

fn ascii_budget(bundle: &Bundle, options: &Options) -> io::Result<Duration> {
    let chars = bundle.bootstrap.len() as u32 + 128;
    let lines = bundle.bootstrap.iter().filter(|&&b| b == b'\n').count() as u32 + 8;
    options
        .char_delay
        .checked_mul(chars)
        .and_then(|d| {
            options
                .line_delay
                .checked_mul(lines)
                .and_then(|l| d.checked_add(l))
        })
        .and_then(|d| d.checked_add(wire_budget(chars as usize, options.baud)))
        .filter(|d| Instant::now().checked_add(*d).is_some())
        .ok_or_else(|| invalid("ASCII pacing delays are too large for a finite deadline"))
}

struct OutputRequest {
    bytes: Vec<u8>,
    flush: bool,
}

struct Transport {
    port: Box<dyn SerialPort>,
    deadline: Instant,
    stage: &'static str,
    capture: bool,
    pending: VecDeque<u8>,
    recent: VecDeque<u8>,
    output_buffer: Vec<u8>,
    output_request: SyncSender<OutputRequest>,
    output_result: Receiver<(Vec<u8>, io::Result<usize>)>,
}

impl Transport {
    fn open(name: &str, baud: u32) -> io::Result<Self> {
        let port = serialport::new(name, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::Hardware)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(|e| io::Error::other(format!("opening explicit port {name}: {e}")))?;
        Self::from_port(port)
    }

    fn from_port(port: Box<dyn SerialPort>) -> io::Result<Self> {
        let mut output_port = port.try_clone().map_err(io::Error::other)?;
        let (output_request, requests) = mpsc::sync_channel::<OutputRequest>(1);
        let (results, output_result) = mpsc::sync_channel(1);
        // Native writes and POSIX tcdrain can block beyond the configured
        // timeout. A dedicated writer lets the foreground drain echoes and
        // enforce its deadline. The buffer is returned and reused each time;
        // its copy is needed to keep driver calls off the foreground thread.
        thread::Builder::new()
            .name("serial-bootstrap-output".into())
            .spawn(move || {
                while let Ok(request) = requests.recv() {
                    let result = if request.flush {
                        output_port.flush().map(|()| 0)
                    } else {
                        output_port.write(&request.bytes)
                    };
                    if results.send((request.bytes, result)).is_err() {
                        break;
                    }
                }
            })?;
        Ok(Self {
            port,
            deadline: Instant::now() + Duration::from_secs(30),
            stage: "opening port",
            pending: VecDeque::with_capacity(8192),
            capture: false,
            recent: VecDeque::with_capacity(256),
            output_buffer: Vec::with_capacity(8192),
            output_request,
            output_result,
        })
    }

    fn output(&mut self, bytes: &[u8], flush: bool) -> io::Result<usize> {
        self.check_deadline()?;
        let mut buffer = std::mem::take(&mut self.output_buffer);
        buffer.clear();
        buffer.extend_from_slice(&bytes[..bytes.len().min(8192)]);
        self.output_request
            .send(OutputRequest {
                bytes: buffer,
                flush,
            })
            .map_err(|_| io::Error::other("serial output worker stopped"))?;
        loop {
            self.check_deadline()?;
            if self.port.bytes_to_read().map_err(io::Error::other)? > 0 {
                self.pump()?;
            }
            match self.output_result.recv_timeout(Duration::from_millis(1)) {
                Ok((buffer, result)) => {
                    self.output_buffer = buffer;
                    self.check_deadline()?;
                    return result;
                }
                Err(RecvTimeoutError::Disconnected) => {
                    return Err(io::Error::other("serial output worker stopped"));
                }
                Err(RecvTimeoutError::Timeout) => {}
            }
        }
    }

    fn stage(&mut self, name: &'static str, budget: Duration) -> io::Result<()> {
        self.stage = name;
        self.deadline = Instant::now()
            .checked_add(budget)
            .ok_or_else(|| invalid("stage deadline overflow"))?;
        eprintln!("{name}");
        Ok(())
    }

    fn check_deadline(&self) -> io::Result<()> {
        if Instant::now() >= self.deadline {
            // Not TimedOut/Interrupted/WouldBlock: zmodem::send retries those.
            return Err(io::Error::new(
                io::ErrorKind::ConnectionAborted,
                "protocol deadline expired",
            ));
        }
        Ok(())
    }

    fn recent_text(&self) -> String {
        self.recent
            .iter()
            .flat_map(|byte| std::ascii::escape_default(*byte))
            .map(char::from)
            .collect()
    }

    fn remember(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if self.recent.len() == 256 {
                self.recent.pop_front();
            }
            self.recent.push_back(byte);
        }
    }

    fn pump(&mut self) -> io::Result<()> {
        self.check_deadline()?;
        let mut bytes = [0; 1024];
        match self.port.read(&mut bytes) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "serial port closed",
                ));
            }
            Ok(count) => {
                self.remember(&bytes[..count]);
                if self.capture {
                    if self.pending.len() + count > 8192 {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidData,
                            "serial protocol input exceeded the 8192-byte receive buffer",
                        ));
                    }
                    self.pending.extend(&bytes[..count]);
                }
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut
                        | io::ErrorKind::WouldBlock
                        | io::ErrorKind::Interrupted
                ) => {}
            Err(e) => return Err(e),
        }
        self.check_deadline()
    }

    fn delay(&mut self, delay: Duration) -> io::Result<()> {
        let end = Instant::now()
            .checked_add(delay)
            .ok_or_else(|| invalid("pacing delay overflow"))?;
        // Even zero-delay pacing services available echoes between writes.
        while Instant::now() < end || self.port.bytes_to_read().map_err(io::Error::other)? > 0 {
            let remaining = end.saturating_duration_since(Instant::now());
            let timeout = remaining
                .min(Duration::from_millis(20))
                .max(Duration::from_millis(1));
            self.port.set_timeout(timeout).map_err(io::Error::other)?;
            self.pump()?;
            // Echoes can remain continuous on a serial BIOS. Do not turn a
            // byte's pacing interval into an unbounded wait for console silence.
            if Instant::now() >= end {
                break;
            }
        }
        self.port
            .set_timeout(Duration::from_millis(20))
            .map_err(io::Error::other)?;
        self.check_deadline()
    }

    fn paced(&mut self, bytes: &[u8], options: &Options) -> io::Result<()> {
        for &byte in bytes {
            self.write_all(&[byte])?;
            self.delay(options.char_delay)?;
            if byte == b'\n' {
                self.delay(options.line_delay)?;
            }
        }
        Ok(())
    }

    fn command(&mut self, command: &[u8], options: &Options) -> io::Result<()> {
        self.paced(command, options)?;
        self.write_all(b"\r")?;
        self.flush()?;
        Ok(())
    }

    fn wait_dos_prompt(&mut self) -> io::Result<()> {
        let mut prompt = DosPrompt::default();
        loop {
            let mut byte = [0];
            self.read_exact(&mut byte)?;
            if prompt.push(byte[0]) {
                return Ok(());
            }
        }
    }

    fn wait_marker(&mut self, marker: &[u8]) -> io::Result<()> {
        let mut matched = 0;
        let mut previous = 0;
        loop {
            let mut byte = [0];
            self.read_exact(&mut byte)?;
            let byte = byte[0];
            if previous == b'B' && byte == b'!' {
                return Err(io::Error::other(format!(
                    "{}: DOS bootstrap reported B! (length, checksum, decode, or destination-file error)",
                    self.stage
                )));
            }
            previous = byte;
            matched = if byte == marker[matched] {
                matched + 1
            } else {
                usize::from(byte == marker[0])
            };
            if matched == marker.len() {
                return Ok(());
            }
        }
    }
}

impl Read for Transport {
    fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> {
        self.check_deadline()?;
        if bytes.is_empty() {
            return Ok(0);
        }
        while self.pending.is_empty() {
            self.pump()?;
        }
        let count = bytes.len().min(self.pending.len());
        for byte in &mut bytes[..count] {
            *byte = self.pending.pop_front().unwrap();
        }
        Ok(count)
    }
}

impl Write for Transport {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        loop {
            self.check_deadline()?;
            match self.output(bytes, false) {
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::TimedOut
                            | io::ErrorKind::WouldBlock
                            | io::ErrorKind::Interrupted
                    ) =>
                {
                    self.pump()?
                }
                result => {
                    self.check_deadline()?;
                    return result;
                }
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        self.output(&[], true).map(|_| ())
    }
}

// This is a prompt recognizer, not a terminal renderer. Serial video BIOSes
// can insert a CSI cursor-position sequence between every visible character.
#[derive(Default)]
enum ConsoleEscape {
    #[default]
    Text,
    Start,
    Csi,
    Osc,
    OscEnd,
}

struct DosPrompt {
    line: [u8; 128],
    len: usize,
    escape: ConsoleEscape,
    csi_parameter: u8,
    csi_column: u16,
    overflow: bool,
}

impl Default for DosPrompt {
    fn default() -> Self {
        Self {
            line: [0; 128],
            len: 0,
            escape: ConsoleEscape::Text,
            csi_parameter: 0,
            csi_column: 0,
            overflow: false,
        }
    }
}

impl DosPrompt {
    fn push(&mut self, byte: u8) -> bool {
        use ConsoleEscape::*;
        match self.escape {
            Start => {
                self.escape = match byte {
                    b'[' => {
                        self.csi_parameter = 0;
                        self.csi_column = 0;
                        Csi
                    }
                    b']' => Osc,
                    _ => Text,
                };
                return false;
            }
            Csi => {
                if byte == b';' {
                    self.csi_parameter = self.csi_parameter.saturating_add(1);
                } else if byte.is_ascii_digit() && self.csi_parameter == 1 {
                    self.csi_column = self
                        .csi_column
                        .saturating_mul(10)
                        .saturating_add(u16::from(byte - b'0'));
                } else if (0x40..=0x7e).contains(&byte) {
                    // Video BIOS output need not contain literal CR/LF:
                    // returning to column one or scrolling starts a new line.
                    if byte == b'S'
                        || (matches!(byte, b'H' | b'f')
                            && self.csi_parameter <= 1
                            && self.csi_column <= 1)
                    {
                        self.len = 0;
                        self.overflow = false;
                    }
                    self.escape = Text;
                }
                return false;
            }
            Osc => {
                if byte == 7 {
                    self.escape = Text;
                } else if byte == 0x1b {
                    self.escape = OscEnd;
                }
                return false;
            }
            OscEnd => {
                self.escape = if byte == b'\\' { Text } else { Osc };
                return false;
            }
            Text => {}
        }
        match byte {
            0x1b => self.escape = Start,
            b'\r' | b'\n' => {
                self.len = 0;
                self.overflow = false;
            }
            8 => {
                self.len = self.len.saturating_sub(1);
            }
            0x20..=0x7e if self.len < self.line.len() => {
                self.line[self.len] = byte;
                self.len += 1;
            }
            0x20..=0x7e => self.overflow = true,
            _ => {}
        }
        let line = &self.line[..self.len];
        byte == b'>'
            && !self.overflow
            && line.first().is_some_and(u8::is_ascii_alphabetic)
            && (line.len() == 2
                || line.len() == 3 && line[1] == b':'
                || line.get(1..3) == Some(b":\\"))
    }
}

fn start_bootstrap(
    io: &mut Transport,
    options: &Options,
    bundle: &Bundle,
    ascii_time: Duration,
) -> io::Result<()> {
    io.stage("Waiting for initial DOS prompt", Duration::from_secs(30))?;
    io.capture = true;
    io.command(b"", options)?;
    io.wait_dos_prompt()?;

    if !options.resume {
        io.stage("ASCII DOS COPY bootstrap", ascii_time)?;
        io.capture = false;
        io.pending.clear();
        // CTTY may redirect handles without aliasing the explicit CON device.
        // /A stops input at Ctrl-Z; /B prevents adding it to the output file.
        let capture = format!("COPY {} /A BOOT.COM /B", options.input_device);
        io.command(capture.as_bytes(), options)?;
        io.delay(options.line_delay.max(Duration::from_millis(500)))?;
        io.paced(&bundle.bootstrap, options)?;
        // Capture before EOF: completion and the prompt can arrive in flush().
        io.capture = true;
        io.stage("Waiting for DOS prompt after COPY", Duration::from_secs(30))?;
        io.write_all(b"\x1a\r")?;
        io.flush()?;
        io.wait_dos_prompt()?;
    }

    io.stage(
        "Launching BOOT and waiting for B1> at 9600 baud",
        Duration::from_secs(30),
    )?;
    // No delay after this CR: BOOT gives us nine BIOS ticks to switch speeds.
    io.command(b"BOOT", options)?;
    if options.baud != 9600 {
        io.port.set_baud_rate(9600).map_err(io::Error::other)?;
    }
    io.wait_marker(b"B1>")
}

fn live(options: &Options, bundle: &Bundle, ascii_time: Duration) -> io::Result<()> {
    let mut io = Transport::open(
        options.port.as_deref().expect("validated explicit port"),
        options.baud,
    )?;
    let result = (|| -> io::Result<()> {
        start_bootstrap(&mut io, options, bundle, ascii_time)?;
        io.stage(
            "Sending checksum-protected BOOT2.BIN",
            wire_budget(bundle.boot2.len(), 9600),
        )?;
        io.write_all(&(bundle.boot2.len() as u16).to_le_bytes())?;
        io.write_all(&bundle.boot2)?;
        io.write_all(&checksum(&bundle.boot2).to_le_bytes())?;
        io.flush()?;
        io.wait_marker(b"B2>")?;

        io.stage(
            "Installing ZRECV.COM with acknowledged ZX0 blocks",
            wire_budget(bundle.receiver_wire.len(), 9600),
        )?;
        io.write_all(&(bundle.receiver_size as u16).to_le_bytes())?;
        let mut at = 0;
        let mut block_number = 0;
        while at < bundle.receiver_wire.len() {
            let stored = usize::from(
                u16::from_le_bytes([bundle.receiver_wire[at + 2], bundle.receiver_wire[at + 3]])
                    & 0x7fff,
            );
            let end = at + 4 + stored;
            io.write_all(&bundle.receiver_wire[at..end])?;
            io.write_all(&checksum(&bundle.receiver_wire[at + 4..end]).to_le_bytes())?;
            io.flush()?;
            block_number += 1;
            io.wait_marker(b"+").map_err(|e| {
                io::Error::new(
                    e.kind(),
                    format!("ZRECV block {block_number}/{}: {e}", bundle.receiver_blocks),
                )
            })?;
            at = end;
        }
        io.wait_marker(b"B3>")?;
        for (stage, (path, size)) in [
            ("ZMODEM installing BAUDRATE.COM", &bundle.baudrate),
            ("ZMODEM installing ZSEND.COM", &bundle.sender),
        ] {
            io.stage(
                stage,
                wire_budget(*size, 9600)
                    .checked_add(ascii_time)
                    .ok_or_else(|| invalid("transfer deadline overflow"))?,
            )?;
            io.wait_dos_prompt()?;
            io.command(b"ZRECV", options)?;
            zmodem::send(&mut io, path, true, |_, _| Ok(()), |_| Ok(()))?;
            io.flush()?;
        }
        Ok(())
    })();
    result.map_err(|e| {
        io::Error::new(
            e.kind(),
            format!(
                "{}: {e}; recent serial input: {}",
                io.stage,
                io.recent_text()
            ),
        )
    })
}

fn run() -> io::Result<()> {
    let Some(options) = options()? else {
        return Ok(());
    };
    // All filesystem and codec validation precedes the only serial-open path.
    let bundle = Bundle::prepare(&options.directory)?;
    let ascii_time = ascii_budget(&bundle, &options)?;
    bundle.describe(&options);
    if options.dry_run {
        println!("Dry run complete; no serial ports enumerated or opened.");
        return Ok(());
    }
    live(&options, &bundle, ascii_time)?;
    println!("Installed ZRECV.COM, BAUDRATE.COM, and ZSEND.COM; DOS remains at 9600 baud.");
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("serial-bootstrap: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn video_bios(text: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        let mut column = 1;
        for byte in text {
            match byte {
                b'\r' => column = 1,
                b'\n' => bytes.extend_from_slice(b"\x1b[1S"),
                _ => {
                    bytes.extend_from_slice(format!("\x1b[25;{column}H").as_bytes());
                    bytes.push(*byte);
                    column += 1;
                }
            }
        }
        bytes
    }

    #[test]
    fn prompt_recognizer_ignores_video_bios_controls_and_echoed_commands() {
        let mut prompt = DosPrompt::default();
        for byte in b"\x1b[1S"
            .iter()
            .copied()
            .chain(video_bios(b"1 File(s) copied\r\nC:\\ZM"))
        {
            assert!(!prompt.push(byte));
        }
        assert!(prompt.push(b'>'));
        for text in [b"\r\nECHO C:\\ZM>".as_slice(), b"\r\nB1>", b"\r\nZRECV>"] {
            assert!(!text.iter().any(|byte| prompt.push(*byte)));
        }
        for text in [b"\r\nC>".as_slice(), b"\r\nD:\\TOOLS>"] {
            assert!(text.iter().any(|byte| prompt.push(*byte)));
        }
    }

    #[cfg(unix)]
    fn read_command(peer: &mut serialport::TTYPort) -> Vec<u8> {
        let mut command = Vec::new();
        loop {
            let mut byte = [0];
            peer.read_exact(&mut byte).unwrap();
            if byte[0] == b'\r' {
                return command;
            }
            command.push(byte[0]);
            assert!(command.len() < 128, "unexpected console command");
        }
    }

    #[cfg(unix)]
    fn exercise_launch(resume: bool) {
        let (mut peer, mut port) = serialport::TTYPort::pair().unwrap();
        peer.set_timeout(Duration::from_secs(3)).unwrap();
        port.set_timeout(Duration::from_millis(20)).unwrap();
        let (release, finished) = mpsc::channel();
        let console = thread::spawn(move || {
            assert_eq!(read_command(&mut peer), b"");
            peer.write_all(&video_bios(b"\r\nC:\\ZM>")).unwrap();
            if !resume {
                assert_eq!(read_command(&mut peer), b"COPY CON /A BOOT.COM /B");
                let mut captured = Vec::new();
                loop {
                    let mut byte = [0];
                    peer.read_exact(&mut byte).unwrap();
                    if byte[0] == 0x1a {
                        break;
                    }
                    captured.push(byte[0]);
                    assert!(captured.len() < 128);
                }
                assert_eq!(read_command(&mut peer), b"");
                assert_eq!(captured, b"PRINTABLE\r\n");
                // More than the previous blind 500 ms delay: no command may
                // arrive while COPY still owns the console.
                peer.set_timeout(Duration::from_millis(900)).unwrap();
                assert!(matches!(
                    peer.read(&mut [0]).unwrap_err().kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ));
                peer.write_all(&video_bios(b"1 File(s) copied\r\n"))
                    .unwrap();
                peer.set_timeout(Duration::from_millis(150)).unwrap();
                assert!(matches!(
                    peer.read(&mut [0]).unwrap_err().kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ));
                peer.write_all(b"\x1b[1S").unwrap();
                peer.write_all(&video_bios(b"C:\\ZM>")).unwrap();
                peer.set_timeout(Duration::from_secs(3)).unwrap();
            }
            // Resume must arrive here without COPY, EOF, or a second upload.
            assert_eq!(read_command(&mut peer), b"BOOT");
            peer.write_all(b"BOOT\r\nB1>").unwrap();
            // Keep the PTY live until the host has consumed the marker.
            finished.recv_timeout(Duration::from_secs(3)).unwrap();
        });
        let options = Options {
            port: None,
            baud: 9600,
            directory: PathBuf::from("."),
            input_device: "CON",
            char_delay: Duration::ZERO,
            line_delay: Duration::ZERO,
            dry_run: false,
            resume,
        };
        let bundle = Bundle {
            bootstrap: b"PRINTABLE\r\n".to_vec(),
            boot2: Vec::new(),
            receiver_size: 0,
            receiver_wire: Vec::new(),
            receiver_blocks: 0,
            baudrate: (PathBuf::new(), 0),
            sender: (PathBuf::new(), 0),
        };
        let mut transport = Transport::from_port(Box::new(port)).unwrap();
        let result = start_bootstrap(&mut transport, &options, &bundle, Duration::from_secs(5));
        let _ = release.send(());
        console.join().unwrap();
        result.unwrap();
    }

    #[test]
    #[cfg(unix)]
    fn boot_waits_for_delayed_copy_prompt() {
        exercise_launch(false);
    }

    #[test]
    #[cfg(unix)]
    fn resume_launches_existing_boot_without_copying() {
        exercise_launch(true);
    }
}
