#!/usr/bin/env python3
"""Exercise the production terminal loop using two isolated Unix PTYs.

Build the release library first. The temporary Rust entry point passes an
already-open PTY to terminal::run, avoiding hardware-only baud ioctls on macOS.
Keyboard decoding, serial reads/writes, stdout buffering, and raw-mode cleanup
are the real implementations. No physical serial port is enumerated or opened.
"""
from __future__ import annotations

import argparse
import fcntl
import os
from pathlib import Path
import pty
import random
import re
import select
import shlex
import struct
import subprocess
import tempfile
import termios
import time
import tty

ROOT = Path(__file__).resolve().parents[1]


def library(name: str) -> Path:
    candidates = list((ROOT / "target/release/deps").glob(f"lib{name}-*.rlib"))
    if not candidates:
        raise RuntimeError("run cargo build --release before this smoke check")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def read_until(fd: int, expected: bytes, timeout: float = 3) -> bytes:
    received = bytearray()
    deadline = time.monotonic() + timeout
    while expected not in received:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            raise AssertionError(f"did not receive {expected!r}; observed {received!r}")
        received.extend(os.read(fd, 4096))
    return bytes(received)


def controlling_terminal() -> None:
    os.setsid()
    fcntl.ioctl(0, termios.TIOCSCTTY, 0)


def exercise(executable: Path) -> None:
    directory = executable.parent
    payload = random.Random(7).randbytes(16384)
    source = directory / "PUT.BIN"
    source.write_bytes(payload)
    downloads = directory / "downloads"
    downloads.mkdir()
    screen_master, screen_slave = pty.openpty()
    fcntl.ioctl(screen_slave, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 80, 0, 0))
    wire_master, wire_slave = pty.openpty()
    release_read, release_write = os.pipe()
    saved_terminal = termios.tcgetattr(screen_slave)
    tty.setraw(wire_slave)
    child = None
    try:
        child = subprocess.Popen(
            [str(executable), str(wire_slave), str(release_read), str(source), str(downloads)],
            stdin=screen_slave, stdout=screen_slave, stderr=screen_slave,
            pass_fds=(wire_slave, release_read), preexec_fn=controlling_terminal,
        )
        # Wait for the application's actual raw-mode transition, not a sleep.
        deadline = time.monotonic() + 3
        while termios.tcgetattr(screen_slave)[3] & termios.ICANON:
            if child.poll() is not None or time.monotonic() >= deadline:
                raise AssertionError("terminal did not enter raw mode")
            time.sleep(0.005)

        # No trailing newline, keyboard input, or exit to flush this prompt.
        os.write(wire_master, b"C:\\DOS>")
        observed = read_until(screen_master, b"C:\\DOS>")
        assert child.poll() is None
        print(f"PASS remote prompt visible while terminal is running: {observed!r}")

        # Literal bytes from a conventional terminal, through crossterm itself.
        os.write(screen_master, b"\x1d?")
        observed = read_until(screen_master, b"Ctrl-] commands:")
        assert not select.select([wire_master], [], [], 0.1)[0], "local help leaked onto serial wire"
        print("PASS actual Ctrl-] ? displays local help without serial output")

        os.write(screen_master, b"DIR\r")
        assert read_until(wire_master, b"DIR\r") == b"DIR\r"
        os.write(screen_master, b"\x1d]")
        assert read_until(wire_master, b"\x1d") == b"\x1d"
        print("PASS ordinary typing and escaped literal Ctrl-] reach the serial peer")

        # Real ZMODEM peers and the production terminal renderer, not canned
        # progress events. Slow acknowledgements expose intermediate frames.
        for direction, manual, compressed, empty in (
                ("recv", False, False, False), ("send", False, False, False),
                ("recv", True, True, False), ("send", True, False, False),
                ("send", True, False, True)):
            if empty:
                source.write_bytes(b"")
            if direction == "recv":
                path = directory / ("GETM.BIN" if manual else "GETA.BIN")
                path.write_bytes(payload)
                expected = downloads / path.name
                mode = "send-compressed" if compressed else "send"
            else:
                path = directory / ("sent-empty" if empty else "sent-manual" if manual else "sent-auto")
                path.mkdir()
                expected = path / source.name
                mode = "recv"
            if manual:
                os.write(screen_master, b"\x1dr" if direction == "recv" else b"\x1ds")
                # Let the local command enter transfer mode before its peer
                # can trigger auto-detection and leave a second command queued.
                read_until(screen_master, b"Recv: waiting for peer..." if direction == "recv"
                           else b"Send: waiting for peer...")
            with subprocess.Popen([str(executable), "peer", str(wire_master), mode, str(path)],
                                  pass_fds=(wire_master,)) as peer:
                try:
                    observed = read_until(screen_master, b"100%", timeout=15)
                    assert peer.wait(timeout=5) == 0
                    assert expected.read_bytes() == (b"" if empty else payload)
                    frames = re.findall(rb"(?:Recv|Send) [^\r\n\x1b]*", observed)
                    if direction == "recv" and compressed:
                        assert any(b"--%" in frame and b"/? wire B" in frame for frame in frames), frames
                    elif not empty:
                        assert any(0 < int(p) < 99 for p in re.findall(rb"\b(\d+)%", observed)), frames
                    assert any(b"KiB/s" in frame for frame in frames), frames
                    assert any(b"[================]" in frame and b"100%" in frame for frame in frames), frames
                    print(f"PASS {'empty' if empty else 'manual' if manual else 'auto'} {direction}: "
                          f"{frames[len(frames)//2].decode()} -> {frames[-1].decode()}")
                finally:
                    if peer.poll() is None:
                        peer.kill()
            # The serial peer can resume a no-newline prompt after the bar.
            os.write(wire_master, b"C:\\DOS>")
            read_until(screen_master, b"C:\\DOS>")

        os.write(screen_master, b"\x1dq")
        read_until(screen_master, b"terminal returned")
        assert not select.select([wire_master], [], [], 0.1)[0], "local quit leaked onto serial wire"
        assert termios.tcgetattr(screen_slave) == saved_terminal
        os.write(release_write, b"\0")
        assert child.wait(timeout=3) == 0
        print("PASS actual Ctrl-] q exits and restores terminal settings")
    finally:
        if child is not None and child.poll() is None:
            child.kill()
            child.wait()
        for fd in (screen_master, screen_slave, wire_master, wire_slave, release_read, release_write):
            os.close(fd)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rustc", default="rustc", help="for example: 'rustc +stable'")
    args = parser.parse_args()
    source = r'''
use std::{io::{Read, Write}, os::fd::FromRawFd, path::PathBuf, time::Duration};
use serialport::SerialPort;
#[path = "TERMINAL_SOURCE"]
mod terminal;
fn main() {
    let args: Vec<_> = std::env::args().collect();
    if args[1] == "peer" {
        let fd = args[2].parse().unwrap();
        let mut wire = unsafe { std::fs::File::from_raw_fd(fd) };
        let path = std::path::Path::new(&args[4]);
        let slow = |p: serial_zmodem::zmodem::Progress<'_>| {
            if p.transferred > 0 && !p.complete {
                std::thread::sleep(Duration::from_millis(50));
            }
            Ok(())
        };
        if args[3].starts_with("send") {
            wire.write_all(b"**\x18B00000000000000\r\n").unwrap();
            serial_zmodem::zmodem::send(&mut wire, path, args[3] == "send-compressed", |_, _| Ok(()), slow).unwrap();
        } else {
            serial_zmodem::zmodem::receive(&mut wire, path, slow).unwrap();
        }
        return;
    }
    let fd = std::env::args().nth(1).unwrap().parse().unwrap();
    // The parent transfers ownership of an inherited, raw serial PTY fd.
    let mut port = unsafe { serialport::TTYPort::from_raw_fd(fd) };
    port.set_timeout(Duration::from_millis(20)).unwrap();
    terminal::run(Box::new(port), Some(PathBuf::from(&args[3])), PathBuf::from(&args[4]), None).unwrap();
    // Keep the controlling session alive for the parent's termios assertion.
    // macOS revokes the slave PTY when the session leader exits.
    let release_fd = std::env::args().nth(2).unwrap().parse().unwrap();
    let mut release = unsafe { std::fs::File::from_raw_fd(release_fd) };
    eprintln!("terminal returned");
    release.read_exact(&mut [0]).unwrap();
}
'''.replace("TERMINAL_SOURCE", str(ROOT / "src/terminal.rs"))
    with tempfile.TemporaryDirectory(prefix="serial-terminal-smoke-") as temporary:
        tmp = Path(temporary)
        (tmp / "terminal_smoke.rs").write_text(source)
        command = shlex.split(args.rustc) + [
            str(tmp / "terminal_smoke.rs"), "--edition=2024", "-o", str(tmp / "terminal-smoke"),
            "-L", f"dependency={ROOT / 'target/release/deps'}",
        ]
        for name in ("serialport", "crossterm", "serial_zmodem"):
            command += ["--extern", f"{name}={library(name)}"]
        subprocess.run(command, check=True)
        exercise(tmp / "terminal-smoke")


if __name__ == "__main__":
    main()
