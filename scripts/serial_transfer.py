#!/usr/bin/env python3
"""Drive the release serial-zmodem console through a controlling-terminal PTY.

Only serial-zmodem opens the explicitly selected physical port. --self-test uses
an isolated fake console process to exercise this driver, not the transport.
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import json
import os
from pathlib import Path
import pty
import re
import select
import struct
import subprocess
import sys
import termios
import time
import tty

ROOT = Path(__file__).resolve().parents[1]
PROGRESS = re.compile(
    rb"(Send|Recv) [^\r\n\x1b]*? \[[=>-]{16}\]\s+(\d+|--)% (\d+)/(\d+|\?) (wire )?B "
    rb"(?:(\d+(?:\.\d+)?) KiB/s|finishing)"
)


class DosPrompt:
    """Port of src/bin/serial-bootstrap.rs's bounded DosPrompt recognizer.

    In particular, CSI positioning between every character is not a newline;
    returning to column one or scrolling is. No literal CR/LF is required.
    """

    def __init__(self):
        self.line = bytearray()
        self.escape = "text"
        self.parameter = self.column = 0
        self.overflow = False

    def push(self, byte):
        if self.escape == "start":
            self.escape = {ord("["): "csi", ord("]"): "osc"}.get(byte, "text")
            self.parameter = self.column = 0
            return False
        if self.escape == "csi":
            if byte == ord(";"):
                self.parameter = min(255, self.parameter + 1)
            elif 48 <= byte <= 57 and self.parameter == 1:
                self.column = min(65535, self.column * 10 + byte - 48)
            elif 0x40 <= byte <= 0x7e:
                if byte == ord("S") or (byte in b"Hf" and self.parameter <= 1 and self.column <= 1):
                    self.line.clear()
                    self.overflow = False
                self.escape = "text"
            return False
        if self.escape == "osc":
            if byte == 7:
                self.escape = "text"
            elif byte == 27:
                self.escape = "osc_end"
            return False
        if self.escape == "osc_end":
            self.escape = "text" if byte == ord("\\") else "osc"
            return False
        if byte == 27:
            self.escape = "start"
        elif byte in (10, 13):
            self.line.clear()
            self.overflow = False
        elif byte == 8:
            del self.line[-1:]
        elif 32 <= byte <= 126:
            if len(self.line) < 128:
                self.line.append(byte)
            else:
                self.overflow = True
        line = self.line
        return (byte == ord(">") and not self.overflow and bool(line)
                and (65 <= line[0] <= 90 or 97 <= line[0] <= 122)
                and (len(line) == 2 or len(line) == 3 and line[1] == ord(":")
                     or line[1:3] == b":\\"))


def controlling_terminal():
    os.setsid()
    fcntl.ioctl(0, termios.TIOCSCTTY, 0)


class Console:
    def __init__(self, argv, timeout, echo=True, log=None):
        self.deadline = time.monotonic() + timeout
        self.echo, self.log = echo, log
        self.master, slave = pty.openpty()
        try:
            fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 160, 0, 0))
            tty.setraw(slave)
            self.child = subprocess.Popen(argv, stdin=slave, stdout=slave, stderr=slave,
                                          preexec_fn=controlling_terminal)
        except BaseException:
            os.close(self.master)
            raise
        finally:
            os.close(slave)
        self.recent = bytearray()

    def read(self, wait=0.1):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("deadline expired; recent console: " + repr(bytes(self.recent[-1024:])))
        if not select.select([self.master], [], [], min(wait, remaining))[0]:
            return b""
        try:
            data = os.read(self.master, 8192)
        except OSError as error:
            if error.errno != errno.EIO:
                raise
            data = b""
        if not data:
            raise RuntimeError(f"serial-zmodem exited before completion (status {self.child.poll()})")
        self.recent.extend(data)
        del self.recent[:-65536]
        if self.echo:
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
        if self.log:
            self.log.write(data)
            self.log.flush()
        return data

    def send(self, data):
        while data:
            data = data[os.write(self.master, data):]

    def command(self, command, char_delay):
        for byte in command.encode("ascii"):
            if time.monotonic() >= self.deadline:
                raise TimeoutError("deadline expired while typing command")
            self.send(bytes([byte]))
            time.sleep(char_delay)
        self.send(b"\r")

    def close(self):
        # Ctrl-] q belongs to the host relay, not DOS. Never send Ctrl-C, CAN,
        # reset commands, retries, file deletions, or baud changes to the peer.
        try:
            if self.child.poll() is None:
                self.send(b"\x1dq")
                deadline = time.monotonic() + 2
                # Terminal restoration can wait for PTY output to drain.
                # Waiting for the child without reading creates a deadlock.
                while self.child.poll() is None and time.monotonic() < deadline:
                    readable, _, _ = select.select([self.master], [], [], 0.05)
                    if readable and not os.read(self.master, 65536):
                        break
        except OSError:
            pass
        finally:
            # Release the PTY before forcibly stopping a stuck relay.
            os.close(self.master)
            if self.child.poll() is None:
                self.child.terminate()
                try:
                    self.child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.child.kill()
                    self.child.wait(timeout=2)


def drive(console, command, transfer, prompt_wait=1.0, char_delay=0.06, direction="Send"):
    prompt = DosPrompt()
    enter_at = time.monotonic() + prompt_wait
    sent_enter = False
    ready = False
    while not ready:
        data = console.read()
        for byte in data:
            ready = prompt.push(byte) or ready
        if not ready and not sent_enter and time.monotonic() >= enter_at:
            console.send(b"\r")
            sent_enter = True
    # Discard the initial prompt's recognition state before typing exactly once.
    prompt = DosPrompt()
    console.command(command, char_delay)
    history = bytearray()
    consumed = 0
    start = finish = None
    failure = None
    complete = False
    result = {"command": command}
    while True:
        data = console.read()
        now = time.monotonic()
        history.extend(data)
        speed = re.search(rb"\[transfer speed: (\d+) baud;", history)
        if speed:
            result["transfer_baud"] = int(speed[1])
        for match in PROGRESS.finditer(history):
            if match.end() <= consumed:
                continue
            consumed = match.end()
            observed_direction, percent, count, total, wire, rate = match.groups()
            if observed_direction.decode() != direction:
                continue
            if start is None:
                start = now  # first payload progress: AFTER link training
                result["payload_started_unix_s"] = time.time()
            if percent == b"100":
                complete = True
                finish = now
                result.update(payload_completed_unix_s=time.time(), payload_bytes=int(count),
                              compressed=bool(wire), reported_kib_s=float(rate) if rate else None)
        if len(history) > 65536:
            removed = len(history) - 65536
            del history[:removed]
            consumed = max(0, consumed - removed)
        if b"[send failed:" in history or b"[receive failed:" in history or b"serial-zmodem:" in history:
            failure = "upload failed (see console output)" if direction == "Send" else "download failed (see console output)"
        returned = False
        for byte in data:
            returned = prompt.push(byte) or returned
        if returned:
            if failure:
                raise RuntimeError(failure)
            if transfer and not complete:
                raise RuntimeError("DOS prompt returned without successful 100% transfer completion (skip/error)")
            if transfer and direction == "Recv" and not re.search(rb"\[received [^\r\n]+\]", history):
                raise RuntimeError("DOS prompt returned without host receive confirmation")
            if complete:
                seconds = finish - start
                result["observed_payload_seconds"] = seconds
                # Same-read completion is below the observer's resolution; do
                # not manufacture throughput. The host's rate remains available.
                result["observed_payload_bytes_s"] = result["payload_bytes"] / seconds if seconds > 0 else None
            result["console_prompt_restored"] = True
            return result


def smoke_child(scenario):
    """Isolated console actor; never opens a serial port."""
    def emit(data):
        os.write(1, data)

    def take(expected):
        received = bytearray()
        deadline = time.monotonic() + 3
        while len(received) < len(expected):
            if not select.select([0], [], [], max(0, deadline - time.monotonic()))[0]:
                raise AssertionError(f"missing input {expected!r}: {received!r}")
            received.extend(os.read(0, len(expected) - len(received)))
        assert received == expected, (received, expected)

    def quiet(delay):
        assert not select.select([0], [], [], delay)[0], "receiver launched before prompt / duplicate command"

    if scenario == "shutdown":
        take(b"\x1dq")
        emit(b"x" * (256 * 1024))
        termios.tcdrain(1)
        return
    if scenario == "timeout":
        take(b"\r")
        take(b"\x1dq")
        return
    if scenario == "ansi":
        emit(b"COPY still writing...")
        take(b"\r")
        quiet(0.1)
        emit(b"\x1b]0;C:\\FALSE>\x07\x1b[1S")
        for column, byte in enumerate(b"C:\\ZM>", 1):
            emit(f"\x1b[25;{column}H".encode() + bytes([byte]))
            if byte != ord(">"):
                quiet(0.015)
    else:
        emit(b"C:\\ZM>")  # no newline, no Enter needed
    take(b"ZSENDY TEST.BIN\r" if scenario == "download" else b"DIR\r" if scenario == "command" else b"ZRECVY\r")
    if scenario == "command":
        emit(b"\r\nDirectory of C:\\ZM\r\nTEST BIN 1024\r\nC:\\ZM>")
        take(b"\x1dq")
        return
    quiet(0.15)  # simulated training: not part of the payload measurement
    if scenario == "skip":
        emit(b"\r\n[send failed: ZMODEM receiver skipped the file]\r\nC:\\ZM>")
    else:
        emit(b"\r\n[transfer speed: 57600 baud; console returns to 9600]\r\n")
        direction = b"Recv" if scenario == "download" else b"Send"
        emit(b"\x1b[1G\x1b[2K" + direction + b" TEST.BIN [>---------------]   0% 0/1024 wire B 0.0 KiB/s")
        quiet(0.1)
        emit(b"\x1b[1G\x1b[2K" + direction + b" TEST.BIN [================] 100% 1024/1024 wire B 10.0 KiB/s\r\n")
        if scenario == "download":
            emit(b"[received readback/TEST.BIN]\r\n")
        emit(b"C:\\ZM>")
    take(b"\x1dq")


def self_test():
    for scenario in ("plain", "ansi", "download", "command", "skip", "timeout"):
        console = Console([sys.executable, str(Path(__file__).resolve()), "--_smoke-child", scenario],
                          timeout=0.5 if scenario == "timeout" else 5, echo=False)
        try:
            try:
                command = "ZSENDY TEST.BIN" if scenario == "download" else "DIR" if scenario == "command" else "ZRECVY"
                result = drive(console, command, scenario != "command", prompt_wait=0.15, char_delay=0,
                               direction="Recv" if scenario == "download" else "Send")
            except TimeoutError:
                assert scenario == "timeout"
            except RuntimeError as error:
                assert scenario == "skip" and "upload failed" in str(error), str(error)
            else:
                assert scenario in ("plain", "ansi", "download", "command")
                assert result["console_prompt_restored"]
                if scenario != "command":
                    assert result["payload_bytes"] == 1024 and result["compressed"]
                    assert 0 < result["observed_payload_seconds"] < 0.2, result
                    assert result["reported_kib_s"] == 10.0
                    assert result["transfer_baud"] == 57600
        finally:
            console.close()
        assert console.child.returncode == 0, scenario
        print(f"PASS isolated PTY driver: {scenario}")
    console = Console([sys.executable, str(Path(__file__).resolve()), "--_smoke-child", "shutdown"],
                      timeout=5, echo=False)
    console.close()
    assert console.child.returncode == 0, "PTY shutdown must drain output without killing the relay"
    print("PASS isolated PTY driver: shutdown with pending output")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, nargs="?", help="file to upload; DOS basename is unchanged")
    parser.add_argument("--port", help="explicit physical serial device; no discovery")
    parser.add_argument("--executable", type=Path, default=ROOT / "target/release/serial-zmodem",
                        help="actual relay executable (override for baseline comparisons)")
    parser.add_argument("--receiver", choices=("ZRECV", "ZRECVX", "ZRECVY"), default="ZRECV")
    parser.add_argument("--experimental-baud", action="store_true", help="train ZRECVX/ZRECVY; console stays at 9600")
    parser.add_argument("--max-baud", type=int, choices=(9600, 19200, 38400, 57600, 115200),
                        help="training ceiling for controlled-rate comparisons")
    parser.add_argument("--baud", type=int, default=9600, help="console baud; transfer receivers require 9600")
    parser.add_argument("--timeout", type=float, default=180, help="overall deadline in seconds, including prompt/training")
    parser.add_argument("--prompt-wait", type=float, default=1, help="seconds before sending one Enter if prompt is absent")
    parser.add_argument("--char-delay", type=float, default=0.06, help="seconds between command characters")
    parser.add_argument("--command-only", metavar="COMMAND", help="type this explicit DOS command once and wait for prompt; no upload")
    parser.add_argument("--command", help="explicit transfer launch command, e.g. 'ZSENDY QBTY001.EXE'; with source or --receive-dir")
    parser.add_argument("--download", metavar="DOS_NAME", help="run ZSEND DOS_NAME and receive a file; no upload")
    parser.add_argument("--receive-dir", type=Path, help="existing host directory for --download; existing files are refused")
    parser.add_argument("--log", type=Path, help="capture raw console bytes to a NEW file (never truncate)")
    parser.add_argument("--self-test", action="store_true", help="isolated PTY driver smoke; no hardware or build needed")
    parser.add_argument("--_smoke-child", choices=("plain", "ansi", "download", "command", "skip", "timeout", "shutdown"), help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args._smoke_child:
        smoke_child(args._smoke_child)
        return 0
    if args.self_test:
        self_test()
        return 0
    if not args.port:
        parser.error("--port is required for live operation")
    if args.timeout <= 0 or args.prompt_wait < 0 or args.char_delay < 0 or args.baud <= 0:
        parser.error("timeout/baud must be positive; prompt-wait/char-delay must be nonnegative")
    if args.max_baud is not None and not args.experimental_baud:
        parser.error("--max-baud requires --experimental-baud")
    if args.command_only is not None:
        if args.source or args.download or args.receive_dir or args.experimental_baud or args.command:
            parser.error("--command-only cannot be combined with transfer options")
        command = args.command_only
    elif args.download or args.receive_dir:
        if args.source or args.experimental_baud or args.baud != 9600:
            parser.error("--download requires 9600 baud, no source, and no experimental training")
        if not args.receive_dir or not args.receive_dir.is_dir():
            parser.error("--download requires an existing --receive-dir")
        if args.download and not re.fullmatch(r"[A-Za-z0-9_~$!-]{1,8}(?:\.[A-Za-z0-9_~$!-]{1,3})?", args.download):
            parser.error("--download requires a plain DOS 8.3 basename, not a path or command")
        if not args.download and not args.command:
            parser.error("--receive-dir requires --download or --command")
        command = args.command or ("ZSEND " + args.download)
    else:
        if args.source is None or not args.source.is_file():
            parser.error("an existing source file is required")
        if args.baud != 9600:
            parser.error("ZRECV/ZRECVX/ZRECVY start at 9600; use --baud 9600")
        if args.experimental_baud and args.receiver == "ZRECV":
            parser.error("--experimental-baud requires --receiver ZRECVX or ZRECVY")
        command = args.command or args.receiver
    if not command or any(not 32 <= ord(char) <= 126 for char in command):
        parser.error("command must be nonempty printable ASCII, without line breaks/control keys")
    executable = args.executable.resolve()
    if not executable.is_file():
        parser.error("build the actual executable first: cargo +stable build --release")
    argv = [str(executable), "--port", args.port, "--baud", str(args.baud)]
    if args.source:
        argv += ["--send", str(args.source.resolve())]
    if args.receive_dir:
        argv += ["--receive-dir", str(args.receive_dir.resolve())]
    if args.experimental_baud:
        argv.append("--experimental-baud")
    if args.max_baud is not None:
        argv += ["--max-baud", str(args.max_baud)]
    console = None
    log = None
    try:
        if args.log:
            log = args.log.open("xb")
        console = Console(argv, args.timeout, log=log)
        result = drive(console, command, args.source is not None or bool(args.receive_dir),
                       args.prompt_wait, args.char_delay, "Recv" if args.receive_dir else "Send")
        if args.source:
            result.update(source=str(args.source), source_bytes=args.source.stat().st_size,
                          receiver=args.receiver)
        if args.receive_dir:
            result.update(download=args.download, receive_dir=str(args.receive_dir))
        print("\nserial-transfer: " + json.dumps(result, sort_keys=True), file=sys.stderr)
        return 0
    except (OSError, RuntimeError, TimeoutError, KeyboardInterrupt) as error:
        print(f"\nserial-transfer: FAILED: {error or 'interrupted'}; no remote reset or retry sent", file=sys.stderr)
        return 1
    finally:
        if console:
            console.close()
        if log:
            log.close()


if __name__ == "__main__":
    sys.exit(main())
