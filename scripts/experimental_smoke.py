#!/usr/bin/env python3
"""Run ZRECVX and real Rust baud training over isolated rate-aware PTYs.

Requires Unicorn, UASM-built DOS images, rustc, and cargo build --release.
No physical serial device is opened. UART divider/termios mismatches lose bytes;
selected higher rates corrupt bytes, rather than passing everything over pipes.
BIOS ticks use wall time. This is not a physical UART electrical/FIFO simulation.
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import os
from pathlib import Path
import pty
import random
import select
import shlex
import struct
import subprocess
import tempfile
import termios
import time
import tty

from bootstrap_smoke import Machine, dos_timestamp
from terminal_smoke import controlling_terminal, read_until


PEER = r'''
use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, FromRawFd, IntoRawFd};
use std::path::Path;
use std::time::Duration;
use serialport::{SerialPort, TTYPort, DataBits, FlowControl, Parity, StopBits, ClearBuffer};
#[path = "TERMINAL_SOURCE"]
mod terminal;

// macOS IOSSIOSPEED is a hardware-only ioctl. PTYs use standard termios speeds,
// which Python observes to enforce loss on actual host/DOS rate disagreement.
struct PtyPort(TTYPort);
impl Read for PtyPort {
    fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> { self.0.read(bytes) }
}
impl Write for PtyPort {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> { self.0.write(bytes) }
    fn flush(&mut self) -> io::Result<()> { self.0.flush() }
}
macro_rules! getters {
    ($($name:ident -> $result:ty),* $(,)?) => {$(
        fn $name(&self) -> $result { self.0.$name() }
    )*};
}
macro_rules! setters {
    ($($name:ident ($arg:ident: $kind:ty) -> $result:ty),* $(,)?) => {$(
        fn $name(&mut self, $arg: $kind) -> $result { self.0.$name($arg) }
    )*};
}
impl SerialPort for PtyPort {
    fn baud_rate(&self) -> serialport::Result<u32> {
        let mut settings = unsafe { std::mem::zeroed() };
        if unsafe { libc::tcgetattr(self.0.as_raw_fd(), &mut settings) } != 0 {
            return Err(io::Error::last_os_error().into());
        }
        let speed = unsafe { libc::cfgetospeed(&settings) };
        [(9600, libc::B9600), (19200, libc::B19200), (38400, libc::B38400),
         (57600, libc::B57600), (115200, libc::B115200)].into_iter()
            .find_map(|(rate, value)| (value == speed).then_some(rate))
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "unsupported PTY rate").into())
    }
    fn set_baud_rate(&mut self, rate: u32) -> serialport::Result<()> {
        let speed = match rate {
            9600 => libc::B9600, 19200 => libc::B19200, 38400 => libc::B38400,
            57600 => libc::B57600, 115200 => libc::B115200,
            _ => return Err(io::Error::new(io::ErrorKind::InvalidInput, "unsupported PTY rate").into()),
        };
        let mut settings = unsafe { std::mem::zeroed() };
        if unsafe {
            libc::tcgetattr(self.0.as_raw_fd(), &mut settings) != 0
                || libc::cfsetispeed(&mut settings, speed) != 0
                || libc::cfsetospeed(&mut settings, speed) != 0
                || libc::tcsetattr(self.0.as_raw_fd(), libc::TCSANOW, &settings) != 0
        } { return Err(io::Error::last_os_error().into()); }
        Ok(())
    }
    getters!(name -> Option<String>, data_bits -> serialport::Result<DataBits>,
        flow_control -> serialport::Result<FlowControl>, parity -> serialport::Result<Parity>,
        stop_bits -> serialport::Result<StopBits>, timeout -> Duration,
        bytes_to_read -> serialport::Result<u32>, bytes_to_write -> serialport::Result<u32>,
        set_break -> serialport::Result<()>, clear_break -> serialport::Result<()>);
    setters!(set_data_bits(value: DataBits) -> serialport::Result<()>,
        set_flow_control(value: FlowControl) -> serialport::Result<()>,
        set_parity(value: Parity) -> serialport::Result<()>,
        set_stop_bits(value: StopBits) -> serialport::Result<()>,
        set_timeout(value: Duration) -> serialport::Result<()>,
        write_request_to_send(value: bool) -> serialport::Result<()>,
        write_data_terminal_ready(value: bool) -> serialport::Result<()>);
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> { self.0.read_clear_to_send() }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> { self.0.read_data_set_ready() }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> { self.0.read_ring_indicator() }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> { self.0.read_carrier_detect() }
    fn clear(&self, buffer: ClearBuffer) -> serialport::Result<()> { self.0.clear(buffer) }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(PtyPort(self.0.try_clone_native()?)))
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    let file = std::fs::OpenOptions::new().read(true).write(true).open(&args[1])?;
    let mut port = PtyPort(unsafe { TTYPort::from_raw_fd(file.into_raw_fd()) });
    port.set_timeout(Duration::from_millis(100))?;
    if args[3] == "terminal" {
        terminal::run(Box::new(port), Some((&args[2]).into()), std::env::temp_dir(), Some(115200))?;
        eprintln!("TERMINAL_RETURNED");
        let mut release = unsafe { std::fs::File::from_raw_fd(args[4].parse()?) };
        release.read_exact(&mut [0])?;
        return Ok(());
    }
    let enabled = args[3] != "disabled";
    let recovery = args[3] == "lost-lock";
    let mut selected = 9600;
    let result = serial_zmodem::zmodem::send(
        &mut port, Path::new(&args[2]), true,
        |io, flags| {
            eprintln!("FLAGS={flags:02x}");
            if enabled && flags & 0x40 != 0 {
                let maximum = if args[3] == "capped" { 57600 } else { 115200 };
                selected = serial_zmodem::baud_training::train(io, maximum)?;
                eprintln!("LOCKED={selected}");
            }
            Ok(())
        }, |_| Ok(()));
    port.set_baud_rate(9600)?;
    eprintln!("RESTORED={}", port.baud_rate()?);
    if recovery {
        assert!(result.is_err(), "lost lock acknowledgement unexpectedly succeeded");
        eprintln!("EXPECTED_LOCK_ERROR={}", result.unwrap_err());
        // The receiver's first post-lock header deadline must recover on its own.
        std::thread::sleep(Duration::from_secs(12));
        port.clear(serialport::ClearBuffer::All)?;
        serial_zmodem::zmodem::send(&mut port, Path::new(&args[2]), true,
            |_, _| Ok(()), |_| Ok(()))?;
        eprintln!("RECOVERED=9600");
    } else {
        result?;
    }
    eprintln!("TRANSFER_OK RATE={selected}");
    // Keep the PTY configured while DOS drains OO and restores UART ownership.
    std::thread::sleep(Duration::from_secs(1));
    Ok(())
}
'''


class RateMachine(Machine):
    def __init__(self, image, master, slave, maximum, lost_lock=False, corrupt_window=False):
        super().__init__(image, realtime=True)
        self.master = master
        self.slave = slave
        self.maximum = maximum
        self.lost_lock = lost_lock
        self.dropped_lock = 0
        self.lock_drop_remaining = 0
        self.training_seen = False
        self.mismatches = 0
        self.corruptions = 0
        self.rates = [9600]
        self.corrupt_window = corrupt_window
        self.window_corrupted = False
        self.rate_bytes = {}
        self.screen = None
        self.observed = bytearray()
        self.deadline = time.monotonic() + 90
        os.set_blocking(master, False)

    @property
    def baud(self):
        divisor = self.divisor[0] | (self.divisor[1] << 8)
        return 115200 // divisor if divisor else 0

    def host_baud(self):
        speed = termios.tcgetattr(self.slave)[5]
        for baud in (9600, 19200, 38400, 57600, 115200):
            if speed == getattr(termios, f"B{baud}"):
                return baud
        raise AssertionError(f"unsupported PTY termios speed {speed}")

    def receive_peer(self):
        if self.screen is not None and select.select([self.screen], [], [], 0)[0]:
            self.observed.extend(os.read(self.screen, 8192))
        ready, _, _ = select.select([self.master], [], [], 0)
        if not ready:
            return
        try:
            data = os.read(self.master, 8192)
        except OSError as error:
            if error.errno in (errno.EIO, errno.EAGAIN):
                return
            raise
        if self.host_baud() != self.baud:
            self.mismatches += len(data)
            return
        if self.baud > self.maximum and data:
            data = bytes([data[0] ^ 1]) + data[1:]
            self.corruptions += 1
        if self.corrupt_window and self.handles and not self.window_corrupted and len(data) > 32:
            data = data[:32] + bytes([data[32] ^ 1]) + data[33:]
            self.window_corrupted = True
        self.incoming.extend(data)

    def emit(self, data):
        self.output.extend(data)
        if self.output.endswith(b"BT1\n"):
            self.training_seen = True
        if self.lost_lock and self.training_seen and not self.dropped_lock and data == b"L":
            self.lock_drop_remaining = 2
        if self.lock_drop_remaining:
            self.lock_drop_remaining -= len(data)
            self.dropped_lock += len(data)
            return
        if self.host_baud() != self.baud:
            self.mismatches += len(data)
            return
        self.rate_bytes[self.baud] = self.rate_bytes.get(self.baud, 0) + len(data)
        view = memoryview(data)
        while view:
            try:
                count = os.write(self.master, view)
                view = view[count:]
            except BlockingIOError:
                select.select([], [self.master], [], 0.1)

    def port_out(self, uc, port, size, value, opaque):
        # Account for bytes queued at the old speed before a divisor change.
        changing = port in (0x3F8, 0x3F9) and self.lcr & 0x80
        if changing:
            self.receive_peer()
        super().port_out(uc, port, size, value, opaque)
        if changing and self.baud != self.rates[-1]:
            self.rates.append(self.baud)


def compile_peer(args, tmp):
    source = tmp / "experimental-peer.rs"
    source.write_text(PEER.replace("TERMINAL_SOURCE",
                                  str(Path(__file__).resolve().parents[1] / "src/terminal.rs")))
    deps = args.library.parent / "deps"
    serialports = sorted(deps.glob("libserialport-*.rlib"), key=lambda p: p.stat().st_mtime)
    assert serialports, f"no serialport rlib in {deps}; run cargo build --release"
    libcs = sorted(deps.glob("liblibc-*.rlib"), key=lambda p: p.stat().st_mtime)
    assert libcs, f"no libc rlib in {deps}; run cargo build --release"
    crossterms = sorted(deps.glob("libcrossterm-*.rlib"), key=lambda p: p.stat().st_mtime)
    assert crossterms, f"no crossterm rlib in {deps}; run cargo build --release"
    executable = tmp / "experimental-peer"
    subprocess.run(shlex.split(args.rustc) + [str(source), "--edition=2024",
        "--extern", f"serial_zmodem={args.library}",
        "--extern", f"serialport={serialports[-1]}",
        "--extern", f"libc={libcs[-1]}", "-L", f"dependency={deps}",
        "--extern", f"crossterm={crossterms[-1]}",
        "-o", str(executable)], check=True)
    return executable


def scenario(executable, image, source, maximum, selected, mode="enabled", flags=0xC0,
             corrupt_window=False):
    master, slave = pty.openpty()
    tty.setraw(slave)
    settings = termios.tcgetattr(slave)
    settings[4] = settings[5] = termios.B9600
    termios.tcsetattr(slave, termios.TCSANOW, settings)
    try:
        machine = RateMachine(image, master, slave, maximum, mode == "lost-lock", corrupt_window)
        with subprocess.Popen([str(executable), os.ttyname(slave), str(source), mode],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              env={**os.environ, "TZ": "UTC"}) as peer:
            try:
                machine.run()
                status = peer.wait(timeout=10)
                report = peer.stderr.read().decode()
                assert machine.exit == 0, (machine.exit, machine.console, report)
                assert status == 0 and "TRANSFER_OK" in report, report
                assert f"FLAGS={flags:02x}" in report, report
                assert "RESTORED=9600" in report, report
                assert machine.host_baud() == 9600
                assert machine.divisor == [12, 0]
                assert not machine.handles
                assert bytes(machine.files[source.name]) == source.read_bytes()
                stamp, date = dos_timestamp((2000, 2, 29, 12, 34, 57))
                assert machine.metadata[source.name] == (1, stamp, date)
                if corrupt_window:
                    assert machine.window_corrupted, "window corruption was not injected"
                trained = mode != "disabled" and flags & 0x40
                if mode == "lost-lock":
                    assert machine.dropped_lock == 2, machine.dropped_lock
                    assert "EXPECTED_LOCK_ERROR=" in report and "RECOVERED=9600" in report, report
                    assert 115200 in machine.rates and machine.mismatches > 0
                elif trained:
                    assert f"LOCKED={selected}" in report, report
                    assert machine.training_seen and selected in machine.rates
                    assert machine.rate_bytes.get(selected, 0) > 20
                    if mode == "capped":
                        assert max(machine.rates) <= selected, machine.rates
                    if maximum < 115200:
                        assert machine.corruptions > 0, "higher-rate failure was never injected"
                        assert any(rate > maximum for rate in machine.rates), machine.rates
                else:
                    assert "LOCKED=" not in report and not machine.training_seen, report
                    assert set(machine.rates) == {9600}, machine.rates
                print(f"PASS {mode} flags={flags:02x} ceiling={maximum}: lock={selected}, "
                      f"bytes/date/readonly preserved, UART+host restored9600 "
                      f"(corruptions={machine.corruptions}, mismatched-bytes={machine.mismatches})")
            except BaseException:
                peer.kill()
                peer.wait()
                diagnostic = peer.stderr.read().decode(errors="replace")
                print(f"FAIL rates={machine.rates} DOS={machine.console!r} peer={diagnostic}")
                raise
    finally:
        os.close(master)
        os.close(slave)


def terminal_scenario(executable, image, source):
    master, slave = pty.openpty()
    screen_master, screen_slave = pty.openpty()
    release_read, release_write = os.pipe()
    fcntl.ioctl(screen_slave, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 100, 0, 0))
    saved_screen = termios.tcgetattr(screen_slave)
    tty.setraw(slave)
    settings = termios.tcgetattr(slave)
    settings[4] = settings[5] = termios.B9600
    termios.tcsetattr(slave, termios.TCSANOW, settings)
    child = None
    machine = RateMachine(image, master, slave, 115200)
    machine.screen = screen_master
    try:
        child = subprocess.Popen([str(executable), os.ttyname(slave), str(source), "terminal",
                                  str(release_read)],
                                 stdin=screen_slave, stdout=screen_slave, stderr=screen_slave,
                                 pass_fds=(release_read,),
                                 preexec_fn=controlling_terminal,
                                 env={**os.environ, "TZ": "UTC"})
        deadline = time.monotonic() + 3
        while termios.tcgetattr(screen_slave)[3] & termios.ICANON:
            assert child.poll() is None and time.monotonic() < deadline, "terminal did not enter raw mode"
            time.sleep(0.005)
        machine.run()
        assert machine.exit == 0, machine.console
        assert bytes(machine.files[source.name]) == source.read_bytes()
        stamp, date = dos_timestamp((2000, 2, 29, 12, 34, 57))
        assert machine.metadata[source.name] == (1, stamp, date)
        assert machine.divisor == [12, 0] and machine.host_baud() == 9600
        assert machine.training_seen and 115200 in machine.rates
        assert not machine.handles
        # This prompt must be visible before local input or exit flushes stdout.
        machine.emit(b"C:\\DOS>")
        machine.observed.extend(read_until(screen_master, b"C:\\DOS>", timeout=5))
        assert b"[transfer speed: 115200 baud; console returns to 9600]" in machine.observed, machine.observed
        assert b"100%" in machine.observed, machine.observed
        assert child.poll() is None
        os.write(screen_master, b"\x1dq")
        machine.observed.extend(read_until(screen_master, b"TERMINAL_RETURNED", timeout=5))
        assert termios.tcgetattr(screen_slave) == saved_screen
        assert machine.host_baud() == 9600
        os.write(release_write, b"\0")
        assert child.wait(timeout=5) == 0
        print("PASS production terminal: trained115200 upload, bytes/date/readonly, 100%, "
              "post-DOS prompt at9600, Ctrl-]q and terminal settings restored")
    except BaseException:
        print(f"FAIL terminal rates={machine.rates} DOS={machine.console!r} screen={machine.observed!r}")
        raise
    finally:
        if child is not None and child.poll() is None:
            child.kill()
            child.wait()
        for fd in (master, slave, screen_master, screen_slave, release_read, release_write):
            os.close(fd)

def allocation_failures(image):
    from unicorn.x86_const import UC_X86_REG_AX, UC_X86_REG_BX

    class NoMemory(Machine):
        def interrupt(self, uc, number, opaque):
            if number == 0x21 and uc.reg_read(UC_X86_REG_AX) >> 8 == self.failed_service:
                uc.reg_write(UC_X86_REG_AX, 8)
                uc.reg_write(UC_X86_REG_BX, 0)
                self.carry(True)
                return
            super().interrupt(uc, number, opaque)

    for service in (0x4A, 0x48):
        machine = NoMemory(image)
        machine.failed_service = service
        machine.run()
        assert machine.exit == 1
        assert not machine.files and not machine.handles
        assert b"**\x18B" not in machine.output, "receiver advertised capacity after allocation failure"
    print("PASS ZRECVY memory exhaustion: no advertised window or created file")



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", type=Path, default=Path("build/dos"))
    parser.add_argument("--rustc", default="rustc")
    parser.add_argument("--library", type=Path, default=Path("target/release/libserial_zmodem.rlib"))
    args = parser.parse_args()
    experimental = (args.dir / "ZRECVX.COM").read_bytes()
    normal = (args.dir / "ZRECV.COM").read_bytes()
    buffered = (args.dir / "ZRECVY.COM").read_bytes()
    allocation_failures(buffered)
    with tempfile.TemporaryDirectory(prefix="experimental-cpu-") as temporary:
        tmp = Path(temporary)
        executable = compile_peer(args, tmp)
        source = tmp / "RATE.BIN"
        source.write_bytes(bytes(range(256)) * 8 + b"Fixed-speed transfer after baud lock\r\n")
        source.chmod(0o444)
        os.utime(source, (951827697, 951827697))  # 2000-02-29 12:34:57 UTC.
        try:
            for ceiling in (115200, 38400, 9600):
                scenario(executable, experimental, source, ceiling, ceiling)
            scenario(executable, experimental, source, 115200, 9600, mode="disabled")
            scenario(executable, normal, source, 115200, 9600, flags=0x80)
            scenario(executable, experimental, source, 115200, 9600, mode="lost-lock")
            terminal_scenario(executable, experimental, source)
            scenario(executable, buffered, source, 115200, 57600, mode="capped")
            for label, payload, corrupt in (
                ("multiwindow", random.Random(17).randbytes(150000), False),
                ("window-retry", random.Random(19).randbytes(150000), True),
                ("compressed", bytes(range(256)) * 1024, False),
                ("empty", b"", False),
            ):
                source.chmod(0o644)
                source.write_bytes(payload)
                source.chmod(0o444)
                os.utime(source, (951827697, 951827697))
                scenario(executable, buffered, source, 115200, 115200,
                         corrupt_window=corrupt)
                print(f"PASS ZRECVY {label}: {len(payload)} original bytes")
        finally:
            source.chmod(0o644)


if __name__ == "__main__":
    main()
