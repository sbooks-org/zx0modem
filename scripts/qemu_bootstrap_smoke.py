#!/usr/bin/env python3
"""Run the unmodified release installer against FreeDOS and SeaBIOS in QEMU.

Requires QEMU, mtools, UASM, and FreeDOS 1.4's 144m/x86BOOT.img. All writes
use a temporary copy of that image. No physical serial device is opened.
The virtual null-modem cable paces both directions at 9600 baud (8N1).
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import os
from pathlib import Path
import pty
import re
import select
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tty

ROOT = Path(__file__).resolve().parents[1]
# Darwin's serialport backend issues IOSSIOSPEED even for PTYs. QEMU, not the
# host PTY, owns the virtual UART clock. Adapt only that ioctl, only on PTYs;
# the release executable, byte stream, termios, reads and writes are unchanged.
PTY_SPEED = r'''
#include <sys/ioctl.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
static int pty_ioctl(int fd, unsigned long request, ...) {
    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    if (request == 0x80045402UL) {
        const char *name = ttyname(fd);
        if (name && !strncmp(name, "/dev/ttys", 9)) return 0;
    }
    return ioctl(fd, request, arg);
}
__attribute__((used)) static struct { const void *replacement, *original; }
interpose __attribute__((section("__DATA,__interpose"))) = {
    (const void *)pty_ioctl, (const void *)ioctl
};
'''
INIT = '''.286
.model tiny
.code
org 100h
start:
    mov ax,00e3h
    xor dx,dx
    int 14h
    int 20h
end start
'''


def connect(path: Path, timeout: float = 20) -> socket.socket:
    deadline = time.monotonic() + timeout
    while True:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(str(path))
            return sock
        except (FileNotFoundError, ConnectionRefusedError):
            sock.close()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"QEMU did not create {path}")
            time.sleep(.02)


class Wire:
    def __init__(self, serial: socket.socket, directory: Path):
        self.serial = serial
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        serial.setblocking(False)
        self.path = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.received = bytearray()
        self.error = None
        self.directory = directory
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        ends = [self.master, self.serial.fileno()]
        queues = [deque(), deque()]
        due = [time.monotonic(), time.monotonic()]
        try:
            with (self.directory / "host-wire.bin").open("wb") as host, \
                 (self.directory / "dos-wire.bin").open("wb") as dos:
                logs = [host, dos]
                while not self.stop.is_set():
                    ready, _, _ = select.select(
                        [ends[i] for i in range(2) if len(queues[i]) < 65536], [], [], .0005)
                    now = time.monotonic()
                    for i in range(2):
                        if ends[i] in ready:
                            data = os.read(ends[i], 4096)
                            if not data:
                                return
                            logs[i].write(data)
                            if i == 1:
                                self.received.extend(data)
                            if not queues[i]:
                                due[i] = now
                            queues[i].extend(data)
                        # Carry fractional time forward; an idle line gets no
                        # accumulated burst credit. 960 characters/s at 8N1.
                        count = min(len(queues[i]), max(0, int((now - due[i]) * 960) + 1))
                        if count:
                            data = bytes(queues[i][j] for j in range(count))
                            try:
                                sent = os.write(ends[1-i], data)
                            except BlockingIOError:
                                continue
                            for _ in range(sent):
                                queues[i].popleft()
                            due[i] += sent / 960
        except Exception as error:
            self.error = error

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)
        self.serial.close()
        os.close(self.master)
        os.close(self.slave)
        if self.error:
            raise self.error


def qmp_command(connection, command, arguments=None):
    request = {"execute": command}
    if arguments is not None:
        request["arguments"] = arguments
    connection.write(json.dumps(request).encode() + b"\n")
    while True:
        response = json.loads(connection.readline())
        if "error" in response:
            raise RuntimeError(response)
        if "return" in response:
            return response["return"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boot-image", type=Path, required=True)
    parser.add_argument("--uasm", default="uasm")
    parser.add_argument("--qemu", default="qemu-system-i386")
    parser.add_argument("--artifacts", type=Path, help="retain wire logs and a DOS screenshot here")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="zm-dos-", dir="/tmp") as temporary:
        tmp = Path(temporary)
        image = tmp / "dos.img"
        shutil.copyfile(args.boot_image, image)
        (tmp / "FDCONFIG.SYS").write_bytes(
            b"FILES=40\r\nBUFFERS=20\r\n"
            b"SHELL=\\FREEDOS\\BIN\\COMMAND.COM \\FREEDOS\\BIN /E:2048 /P=\\FDAUTO.BAT\r\n")
        (tmp / "FDAUTO.BAT").write_bytes(b"@echo off\r\nPROMPT $P$G\r\nINIT\r\n")
        (tmp / "init.asm").write_text(INIT)
        environment = os.environ.copy()
        environment.pop("UASM", None)
        subprocess.run([args.uasm, "-bin", f"-Fo={tmp / 'INIT.COM'}", str(tmp / "init.asm")],
                       check=True, env=environment)
        subprocess.run(["mcopy", "-o", "-i", str(image), str(tmp / "FDCONFIG.SYS"),
                        str(tmp / "FDAUTO.BAT"), str(tmp / "INIT.COM"), "::"], check=True)
        if sys.platform == "darwin":
            (tmp / "pty_speed.c").write_text(PTY_SPEED)
            subprocess.run(["cc", "-dynamiclib", "-o", str(tmp / "pty_speed.dylib"),
                            str(tmp / "pty_speed.c")], check=True)
            environment["DYLD_INSERT_LIBRARIES"] = str(tmp / "pty_speed.dylib")
        wire = None
        control = None
        with (tmp / "qemu.log").open("wb") as log, subprocess.Popen([
            args.qemu, "-machine", "pc,accel=tcg", "-m", "16", "-boot", "a",
            "-drive", f"file={image},format=raw,if=floppy", "-nographic",
            "-serial", f"unix:{tmp / 'serial.sock'},server=on,wait=off",
            "-qmp", f"unix:{tmp / 'qmp.sock'},server=on,wait=off", "-monitor", "none",
            "-no-reboot"], stdin=subprocess.DEVNULL, stdout=log, stderr=log) as qemu:
            try:
                wire = Wire(connect(tmp / "serial.sock"), tmp)
                control = connect(tmp / "qmp.sock")
                control.settimeout(10)
                channel = control.makefile("rwb", buffering=0)
                json.loads(channel.readline())
                qmp_command(channel, "qmp_capabilities")
                # The normal startup banner precedes the first prompt. Wait
                # for actual DOS, rather than relying on a VM boot delay.
                deadline = time.monotonic() + 30
                while b"A:\\>" not in re.sub(rb"\x1b\[[0-?]*[ -/]*[@-~]", b"", wire.received):
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"No DOS prompt: {wire.received[-512:]!r}")
                    time.sleep(.02)
                result = subprocess.run([
                    str(ROOT / "target/release/serial-bootstrap"), "--port", wire.path,
                    "--input-device", "CON",
                    "--dir", str(ROOT / "build/dos")],
                    env=environment, capture_output=True, timeout=600)
                (tmp / "installer.log").write_bytes(result.stdout + result.stderr)
                print((result.stdout + result.stderr).decode(errors="replace"), end="", flush=True)
                result.check_returncode()
                before = len(wire.received)
                for byte in b"ECHO GUARD-RELEASED\r":
                    os.write(wire.slave, bytes([byte]))
                    time.sleep(.01)
                deadline = time.monotonic() + 10
                while True:
                    reply = re.sub(rb"\x1b\[[0-?]*[ -/]*[@-~]", b"", wire.received[before:])
                    if b"\r\nGUARD-RELEASED\r\n" in reply and reply.rstrip().endswith(b"A:\\>"):
                        break
                    if time.monotonic() >= deadline:
                        raise AssertionError(f"DOS console did not recover: {reply!r}")
                    time.sleep(.02)
                print("PASS DOS executes a console command after UART ownership is released")
                qmp_command(channel, "screendump", {"filename": str(tmp / "screen.ppm")})
                qmp_command(channel, "quit")
                qemu.wait(timeout=10)
                for dos_name, host_name in [("BOOT.COM", "BOOTSTRAP.COM"),
                        ("ZRECV.COM", "ZRECV.COM"), ("BAUDRATE.COM", "BAUDRATE.COM"),
                        ("ZSEND.COM", "ZSEND.COM")]:
                    target = tmp / dos_name
                    subprocess.run(["mcopy", "-i", str(image), f"::{dos_name}", str(target)], check=True)
                    assert target.read_bytes() == (ROOT / "build/dos" / host_name).read_bytes(), dos_name
                    print(f"PASS real DOS file {dos_name}: byte-identical to {host_name}")
            finally:
                if qemu.poll() is None:
                    qemu.terminate()
                    qemu.wait(timeout=10)
                if wire:
                    wire.close()
                if control:
                    control.close()
                if args.artifacts:
                    args.artifacts.mkdir(parents=True, exist_ok=True)
                    for name in ("host-wire.bin", "dos-wire.bin", "installer.log", "qemu.log", "screen.ppm"):
                        if (tmp / name).exists():
                            shutil.copyfile(tmp / name, args.artifacts / name)
    print("PASS QEMU/FreeDOS COPY CON -> BOOT -> ZX0 -> both ZMODEM installs at 9600 baud")


if __name__ == "__main__":
    main()
