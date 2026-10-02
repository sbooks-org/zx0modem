#!/usr/bin/env python3
"""Execute the actual bootstrap/sender/receiver binaries in a 16-bit Unicorn CPU.

Emulates only the DOS calls and UART registers used here, not a complete DOS
kernel or physical serial timing. Requires `pip install unicorn`, built DOS
images, and a Rust library build. Never opens a physical serial device.
"""
from __future__ import annotations

import argparse
from collections import deque
import os
from pathlib import Path
import shlex
import struct
import subprocess
import tempfile
import time
import calendar

from unicorn import Uc, UC_ARCH_X86, UC_MODE_16, UC_HOOK_CODE, UC_HOOK_INTR, UC_HOOK_INSN
from unicorn.x86_const import *
from pack_bootstrap import build, validate_ascii

BASE = 0x10000
SEG = BASE >> 4


class Machine:
    def __init__(self, image: bytes, incoming: bytes = b"", files=None,
                 metadata=None, failures=(), command="", realtime=False):
        self.uc = Uc(UC_ARCH_X86, UC_MODE_16)
        self.uc.mem_map(0, 0x100000)
        self.uc.mem_write(BASE, b"\xa5" * 65536)
        self.uc.mem_write(BASE + 0x100, image)
        self.uc.mem_write(0x400, struct.pack("<H", 0x3F8) + bytes(6))
        self.uc.mem_write(0x46C, bytes(4))
        self.uc.mem_write(0x41A, struct.pack("<HH", 0x1E, 0x1E))
        self.uc.mem_write(0x480, struct.pack("<HH", 0x1E, 0x3E))
        for register in (UC_X86_REG_CS, UC_X86_REG_DS, UC_X86_REG_ES, UC_X86_REG_SS):
            self.uc.reg_write(register, SEG)
        for register in (UC_X86_REG_AX, UC_X86_REG_BX, UC_X86_REG_CX, UC_X86_REG_DX,
                         UC_X86_REG_SI, UC_X86_REG_DI, UC_X86_REG_BP):
            self.uc.reg_write(register, 0xD38B)
        self.uc.reg_write(UC_X86_REG_SP, 0xFFFE)
        self.uc.reg_write(UC_X86_REG_EFLAGS, 0x602)  # DF set; real stage must CLD.
        tail = command.encode("ascii")
        assert len(tail) <= 126
        self.uc.mem_write(BASE + 0x80, bytes([len(tail)]) + tail + b"\r")
        self.incoming = deque(incoming)
        self.output = bytearray()
        self.files = {} if files is None else files
        self.handles = {}
        self.positions = {}
        self.metadata = {} if metadata is None else metadata
        self.failures = set(failures)
        self.services = []
        self.console = bytearray()
        self.next_handle = 5
        self.allocations = {SEG: 0x8000}
        self.exit = None
        self.lcr = 3
        self.divisor = [12, 0]
        self.mcr = 3
        self.ier = 5
        self.instructions = 0
        self.tick = 0
        self.user_ticks = 0
        self.pic_eois = 0
        self.stop_at = None
        self.peer = None
        self.deadline = time.monotonic() + 20
        self.realtime = realtime
        self.started = time.monotonic()
        self.uc.hook_add(UC_HOOK_CODE, self.code)
        self.uc.hook_add(UC_HOOK_INTR, self.interrupt)
        self.uc.hook_add(UC_HOOK_INSN, self.port_in, None, 1, 0, UC_X86_INS_IN)
        self.uc.hook_add(UC_HOOK_INSN, self.port_out, None, 1, 0, UC_X86_INS_OUT)

    def carry(self, value):
        flags = self.uc.reg_read(UC_X86_REG_EFLAGS)
        self.uc.reg_write(UC_X86_REG_EFLAGS, (flags | 1) if value else (flags & ~1))

    def code(self, uc, address, size, _):
        self.instructions += 1
        if address == self.stop_at:
            uc.emu_stop()
        if self.instructions % 1000 == 0:
            # BIOS time advances while busy-polling, including the startup guard.
            self.tick = (int((time.monotonic() - self.started) * 18.2065)
                         if self.realtime else self.tick + 1)
            uc.mem_write(0x46C, struct.pack("<I", self.tick))
            if time.monotonic() > self.deadline:
                raise TimeoutError(f"CPU timeout at {address:05x}; UART tail {self.output[-80:]!r}")

    def pointer(self, offset):
        return (self.uc.reg_read(UC_X86_REG_DS) << 4) + offset

    def string(self, address, end=0):
        out = bytearray()
        while True:
            byte = self.uc.mem_read(address, 1)[0]
            if byte == end:
                return out.decode("ascii")
            out.append(byte)
            address += 1
            if len(out) > 4096:
                raise AssertionError("unterminated DOS string")

    def interrupt(self, uc, number, _):
        ax = uc.reg_read(UC_X86_REG_AX)
        ah = ax >> 8
        dx = uc.reg_read(UC_X86_REG_DX)
        if number == 0x08:
            # Execute the installed real-mode IRQ0 handler with an IRET frame.
            flags = uc.reg_read(UC_X86_REG_EFLAGS)
            sp = (uc.reg_read(UC_X86_REG_SP) - 6) & 0xFFFF
            stack = (uc.reg_read(UC_X86_REG_SS) << 4) + sp
            uc.mem_write(stack, struct.pack("<HHH", uc.reg_read(UC_X86_REG_IP),
                                          uc.reg_read(UC_X86_REG_CS), flags & 0xFFFF))
            offset, segment = struct.unpack("<HH", uc.mem_read(0x20, 4))
            uc.reg_write(UC_X86_REG_SP, sp)
            uc.reg_write(UC_X86_REG_EFLAGS, flags & ~0x300)
            uc.reg_write(UC_X86_REG_CS, segment)
            uc.reg_write(UC_X86_REG_IP, offset)
            return
        if number == 0x1C:
            self.user_ticks += 1
            return
        if number == 0x16:
            # No local key pressed.
            uc.reg_write(UC_X86_REG_EFLAGS, uc.reg_read(UC_X86_REG_EFLAGS) | 0x40)
            return
        if number == 0x14:
            assert ah == 0 and dx == 0, ("unexpected BIOS UART call", hex(ax), dx)
            self.divisor = [12, 0]
            return
        if number != 0x21:
            raise AssertionError(f"unexpected INT {number:02x} AX={ax:04x}")
        self.carry(False)
        self.services.append(ax)
        if ax in self.failures:
            uc.reg_write(UC_X86_REG_AX, 5)  # DOS access denied.
            self.carry(True)
            return
        if ah == 0x4C:
            self.exit = ax & 0xFF
            uc.emu_stop()
        elif ah == 0x4A:
            segment = uc.reg_read(UC_X86_REG_ES)
            paragraphs = uc.reg_read(UC_X86_REG_BX)
            assert segment in self.allocations and 0 < paragraphs <= self.allocations[segment]
            self.allocations[segment] = paragraphs
        elif ah == 0x48:
            paragraphs = uc.reg_read(UC_X86_REG_BX)
            start = max(segment + size for segment, size in self.allocations.items())
            if paragraphs == 0 or start + paragraphs > 0x9000:
                uc.reg_write(UC_X86_REG_AX, 8)
                uc.reg_write(UC_X86_REG_BX, max(0, 0x9000 - start))
                self.carry(True)
            else:
                self.allocations[start] = paragraphs
                uc.reg_write(UC_X86_REG_AX, start)
        elif ah == 0x49:
            segment = uc.reg_read(UC_X86_REG_ES)
            assert segment != SEG and segment in self.allocations
            del self.allocations[segment]
        elif ah == 0x35:
            uc.reg_write(UC_X86_REG_ES, 0)
            uc.reg_write(UC_X86_REG_BX, 0)
        elif ah == 0x25:
            pass  # Save/restore INT1B vector; no Ctrl-Break raised by this harness.
        elif ah == 0x09:
            data = self.string(self.pointer(dx), ord('$')).encode("ascii")
            self.console.extend(data)
            self.emit(data)
            if self.peer:
                # DOS/BIOS console output does not service the application's
                # UART RX loop. At 9600 baud, ANSI-decorated text can take far
                # longer than filling a real 16550's 16-byte receive FIFO.
                import select
                assert len(self.incoming) <= 16, (
                    "UART RX overrun while DOS console output blocks the receiver",
                    len(self.incoming), data)
                until = time.monotonic() + len(data) * 10 / 960
                while time.monotonic() < until:
                    ready, _, _ = select.select([self.peer.stdout], [], [],
                                               max(0, until - time.monotonic()))
                    if not ready:
                        break
                    pending = os.read(self.peer.stdout.fileno(), 8192)
                    if not pending:
                        break
                    self.incoming.extend(pending)
                    assert len(self.incoming) <= 16, (
                        "UART RX overrun while DOS console output blocks the receiver",
                        len(self.incoming), data)
        elif ah == 0x5B:
            name = self.string(self.pointer(dx)).upper()
            if name in self.files:
                uc.reg_write(UC_X86_REG_AX, 80)
                self.carry(True)
            else:
                handle = self.next_handle
                self.next_handle += 1
                self.files[name] = bytearray()
                self.handles[handle] = name
                self.positions[handle] = 0
                self.metadata[name] = (uc.reg_read(UC_X86_REG_CX), 0, 0x21)
                uc.reg_write(UC_X86_REG_AX, handle)
        elif ah == 0x3D:
            name = self.string(self.pointer(dx)).upper()
            if name not in self.files:
                uc.reg_write(UC_X86_REG_AX, 2)
                self.carry(True)
            else:
                handle = self.next_handle
                self.next_handle += 1
                self.handles[handle] = name
                self.positions[handle] = 0
                uc.reg_write(UC_X86_REG_AX, handle)
        elif ah in (0x3F, 0x40):
            handle = uc.reg_read(UC_X86_REG_BX)
            length = uc.reg_read(UC_X86_REG_CX)
            data = self.files[self.handles[handle]]
            position = self.positions[handle]
            if ah == 0x3F:
                block = bytes(data[position:position + length])
                if block:
                    uc.mem_write(self.pointer(dx), block)
                length = len(block)
            else:
                block = bytes(uc.mem_read(self.pointer(dx), length))
                if position > len(data):
                    data.extend(bytes(position - len(data)))
                data[position:position + length] = block
            self.positions[handle] += length
            uc.reg_write(UC_X86_REG_AX, length)
        elif ah == 0x42:
            handle = uc.reg_read(UC_X86_REG_BX)
            offset = (uc.reg_read(UC_X86_REG_CX) << 16) | dx
            if offset & 0x80000000:
                offset -= 0x100000000
            origin = (0, self.positions[handle], len(self.files[self.handles[handle]]))[ax & 0xFF]
            position = origin + offset
            assert position >= 0
            self.positions[handle] = position
            uc.reg_write(UC_X86_REG_AX, position & 0xFFFF)
            uc.reg_write(UC_X86_REG_DX, position >> 16)
        elif ah == 0x57:
            name = self.handles[uc.reg_read(UC_X86_REG_BX)]
            attributes, stamp, date = self.metadata.get(name, (0, 0, 0x21))
            if ax == 0x5700:
                uc.reg_write(UC_X86_REG_CX, stamp)
                uc.reg_write(UC_X86_REG_DX, date)
            else:
                assert ax == 0x5701
                self.metadata[name] = (attributes, uc.reg_read(UC_X86_REG_CX), dx)
        elif ah == 0x43:
            name = self.string(self.pointer(dx)).upper()
            if name not in self.files:
                uc.reg_write(UC_X86_REG_AX, 2)
                self.carry(True)
            else:
                attributes, stamp, date = self.metadata.get(name, (0, 0, 0x21))
                if ax == 0x4300:
                    uc.reg_write(UC_X86_REG_CX, attributes)
                else:
                    assert ax == 0x4301
                    assert name not in self.handles.values(), "attributes set before close"
                    self.metadata[name] = (uc.reg_read(UC_X86_REG_CX), stamp, date)
        elif ah == 0x3E:
            handle = uc.reg_read(UC_X86_REG_BX)
            del self.handles[handle]
            del self.positions[handle]
        elif ah == 0x41:
            name = self.string(self.pointer(dx)).upper()
            self.files.pop(name, None)
            self.metadata.pop(name, None)
        else:
            raise AssertionError(f"unsupported DOS call AX={ax:04x}")

    def receive_peer(self):
        if self.peer is not None and not self.incoming:
            import select
            ready, _, _ = select.select([self.peer.stdout], [], [], 0.01)
            if ready:
                data = os.read(self.peer.stdout.fileno(), 8192)
                if not data:
                    raise EOFError(f"Rust peer closed; UART tail {self.output[-80:]!r}")
                self.incoming.extend(data)

    def port_in(self, uc, port, size, _):
        assert size == 1
        register = port - 0x3F8
        if register == 5:
            self.receive_peer()
            return 0x60 | bool(self.incoming)
        if register == 6:
            return 0xB0  # CTS/DSR/DCD asserted.
        if register == 3:
            return self.lcr
        if register == 4:
            return self.mcr
        if register in (0, 1) and self.lcr & 0x80:
            return self.divisor[register]
        if register == 0:
            assert self.incoming, "UART read without RX ready"
            return self.incoming.popleft()
        if register == 1:
            return self.ier
        raise AssertionError(f"unexpected IN {port:x}")

    def emit(self, data):
        self.output.extend(data)
        if self.peer:
            self.peer.stdin.write(data)
            self.peer.stdin.flush()

    def port_out(self, uc, port, size, value, _):
        assert size == 1
        if port == 0x20:
            assert value == 0x20
            self.pic_eois += 1
            return
        register = port - 0x3F8
        if register in (0, 1) and self.lcr & 0x80:
            self.divisor[register] = value
        elif register == 0:
            self.emit(bytes([value]))
        elif register == 3:
            self.lcr = value
        elif register == 4:
            self.mcr = value
        elif register == 1:
            self.ier = value
        elif register == 2:
            pass  # FIFO configuration.
        else:
            raise AssertionError(f"unexpected OUT {port:x}")

    def run(self):
        self.uc.emu_start(BASE + 0x100, BASE + 65536,
                          count=200_000_000 if self.realtime else 20_000_000)
        if self.exit is None and self.stop_at is None:
            raise AssertionError(f"CPU instruction limit; UART tail {self.output[-80:]!r}")
        if self.exit is not None:
            assert self.ier == 5, "UART interrupt ownership was not restored"
            assert bytes(self.uc.mem_read(0x20, 4)) == bytes(4), "IRQ0 vector was not restored"


def checksum(data):
    return struct.pack("<H", sum(data) & 0xFFFF)


def stage2_stream(stage2, target, encoded):
    blocks = bytearray(struct.pack("<H", len(target)))
    at = 0
    while at < len(encoded):
        stored = int.from_bytes(encoded[at + 2:at + 4], "little") & 0x7FFF
        block = encoded[at + 4:at + 4 + stored]
        assert len(block) == stored
        blocks += encoded[at:at + 4 + stored] + checksum(block)
        at += 4 + stored
    return struct.pack("<H", len(stage2)) + stage2 + checksum(stage2) + blocks


PEER = r'''
use std::io::{self, Read, Write};
struct Stdio;
impl Read for Stdio {
    fn read(&mut self, b: &mut [u8]) -> io::Result<usize> { io::stdin().read(b) }
}
impl Write for Stdio {
    fn write(&mut self, b: &[u8]) -> io::Result<usize> { io::stdout().write(b) }
    fn flush(&mut self) -> io::Result<()> { io::stdout().flush() }
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    if args[1] == "encode" {
        let data = std::fs::read(&args[2]).unwrap();
        io::stdout().write_all(&serial_zmodem::zx0::compress(&data)).unwrap();
    } else {
        let result = if args[1] == "receive" {
            serial_zmodem::zmodem::receive(
                &mut Stdio, std::path::Path::new(&args[2]), |_| Ok(())).map(|_| ())
        } else {
            serial_zmodem::zmodem::send(
                &mut Stdio, std::path::Path::new(&args[2]), true,
                |_, _| Ok(()), |_| Ok(()))
        };
        match result {
            Ok(()) => eprintln!("TRANSFER_OK"),
            Err(ref error) => eprintln!("TRANSFER_ERROR: {error}"),
        }
        // A serial connection remains open after send returns. Drain trailing
        // DOS console output until the emulator closes this end of the link.
        let mut console = [0; 1024];
        while io::stdin().read(&mut console).unwrap() != 0 {}
        if result.is_err() { std::process::exit(2); }
    }
}
'''


def dos_timestamp(fields):
    year, month, day, hour, minute, second = fields
    return ((hour << 11) | (minute << 5) | (second // 2),
            ((year - 1980) << 9) | (month << 5) | day)


def run_peer(machine, executable, mode, path, success=True):
    with subprocess.Popen([str(executable), mode, str(path)], stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          env={**os.environ, "TZ": "UTC"}) as peer:
        machine.peer = peer
        try:
            machine.run()
            peer.stdin.close()
            status = peer.wait(timeout=10)
            report = peer.stderr.read()
            if success:
                assert machine.exit == 0, (machine.exit, machine.console)
                assert status == 0 and b"TRANSFER_OK" in report, report
            else:
                assert machine.exit != 0, machine.console
                assert status != 0 and b"TRANSFER_OK" not in report, report
            assert not machine.handles, "DOS handle leak"
            assert machine.divisor == [12, 0], "DOS speed not restored"
        finally:
            if peer.poll() is None:
                peer.kill()


def metadata_smoke(tmp, executable, receiver, sender):
    payload = bytes(range(256)) * 5 + b"DOS metadata roundtrip\r\n"
    dates = ((1980, 1, 1, 0, 0, 1), (2000, 2, 29, 12, 34, 57),
             (2100, 3, 1, 23, 59, 59), (2107, 12, 31, 23, 59, 59))
    for index, fields in enumerate(dates):
        name = f"DATE{index}.BIN"
        source = tmp / name
        source.write_bytes(payload)
        source.chmod(0o444 if index % 2 else 0o644)
        stamp = calendar.timegm((*fields, 0, 0, 0))
        os.utime(source, (stamp, stamp))
        expected_time, expected_date = dos_timestamp(fields)
        expected_attributes = index % 2
        machine = Machine(receiver)
        run_peer(machine, executable, "send", source)
        assert bytes(machine.files[name]) == payload
        assert machine.metadata[name] == (expected_attributes, expected_time, expected_date)
        assert 0x5701 in machine.services and 0x4301 in machine.services
        source.chmod(0o644)

        # A/H/S/archive bits are legal source attributes but are never preserved.
        destination = tmp / f"received-{index}"
        destination.mkdir()
        probe = destination / "creation-mode"
        probe.write_bytes(b"")
        normal_mode = probe.stat().st_mode & 0o777
        probe.unlink()
        machine = Machine(sender, files={name: bytearray(payload)},
                          metadata={name: (0x26 | expected_attributes, expected_time, expected_date)},
                          command=f" {name}")
        run_peer(machine, executable, "receive", destination)
        output = destination / name
        assert output.read_bytes() == payload
        assert int(output.stat().st_mtime) == stamp - fields[-1] % 2
        expected_mode = normal_mode & ~0o222 if expected_attributes else normal_mode
        assert output.stat().st_mode & 0o777 == expected_mode
        assert machine.metadata[name][0] == 0x26 | expected_attributes
        output.chmod(0o644)
    print("PASS real ZSEND/ZRECV bytes, DOS 2-second dates (1980..2107), readonly/writable, ignored A/H/S")

    source = tmp / "FAIL.BIN"
    source.write_bytes(payload)
    source.chmod(0o444)
    os.utime(source, (946684800, 946684800))
    for service in (0x5701, 0x4301):
        machine = Machine(receiver, failures=(service,))
        run_peer(machine, executable, "send", source, success=False)
        assert service in machine.services
        assert b"Received file." not in machine.console, machine.console
        assert not machine.files, "metadata failure retained a partial DOS file"
    source.chmod(0o644)
    for service in (0x5700, 0x4300):
        machine = Machine(sender, files={"FAIL.BIN": bytearray(payload)},
                          metadata={"FAIL.BIN": (0, 0, 0x2821)},
                          command=" FAIL.BIN", failures=(service,))
        # Automatic host receive starts only after ZSEND announces itself.
        # These failures happen before UART initialization or ZRQINIT.
        machine.run()
        assert machine.exit == 1, machine.console
        assert service in machine.services
        assert machine.output == machine.console, "metadata error announced a transfer"
        assert machine.files == {"FAIL.BIN": bytearray(payload)}
        assert machine.metadata == {"FAIL.BIN": (0, 0, 0x2821)}
        assert not machine.handles
        assert machine.divisor == [12, 0]
    print("PASS get/set DOS metadata failures never report transfer success")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", type=Path, default=Path("build/dos"))
    parser.add_argument("--uasm", default="uasm")
    parser.add_argument("--rustc", default="rustc")
    parser.add_argument("--library", type=Path, default=Path("target/release/libserial_zmodem.rlib"))
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="bootstrap-cpu-") as temporary:
        tmp = Path(temporary)
        # Exercise all byte values, dirty registers/RAM, and the maximum stub size.
        payload = bytes(range(256)) * 4 + bytes(range(176))
        (tmp / "all.bin").write_bytes(payload)
        build(tmp / "all.bin", tmp / "all.com", args.uasm)
        packed = (tmp / "all.com").read_bytes()
        machine = Machine(packed)
        machine.stop_at = BASE + 0x6000
        machine.run()
        assert bytes(machine.uc.mem_read(BASE + 0x6000, len(payload))) == payload
        assert machine.uc.reg_read(UC_X86_REG_SP) == 0xFFFE
        print("PASS printable loader: every byte value, dirty registers/RAM, stack restored")

        # Real handler code: the clock crosses midnight, IRQ callbacks still
        # run, caller flags/registers survive, and the original owner returns.
        (tmp / "guard.asm").write_text(""".286
.model tiny
.code
org 100h
start:
    jmp main
include uart_guard.inc
main:
    mov dx,03f8h
    call UART_GuardAcquire
    mov ax,0040h
    mov es,ax
    mov word ptr es:[006Ch],00AEh
    mov word ptr es:[006Eh],0018h
    stc
    int 08h
    jnc failed
    int 08h
    jnc failed
    call UART_GuardRelease
    mov ax,4c00h
    int 21h
failed:
    call UART_GuardRelease
    mov ax,4c01h
    int 21h
end start
""")
        dos = Path(__file__).resolve().parents[1] / "dos"
        environment = os.environ.copy()
        environment.pop("UASM", None)
        subprocess.run([args.uasm, "-bin", f"-I={dos}", f"-Fo={tmp / 'guard.com'}",
                        str(tmp / "guard.asm")], check=True, env=environment)
        guard = Machine((tmp / "guard.com").read_bytes())
        guard.run()
        assert guard.exit == 0
        assert bytes(guard.uc.mem_read(0x46C, 5)) == b"\0\0\0\0\1"
        assert guard.user_ticks == guard.pic_eois == 2
        assert guard.uc.reg_read(UC_X86_REG_SP) == 0xFFFE
        assert guard.uc.reg_read(UC_X86_REG_DS) == SEG
        assert guard.uc.reg_read(UC_X86_REG_ES) == 0x40
        for register in (UC_X86_REG_BX, UC_X86_REG_CX, UC_X86_REG_SI,
                         UC_X86_REG_DI, UC_X86_REG_BP):
            assert guard.uc.reg_read(register) == 0xD38B
        print("PASS UART guard: midnight, timer callbacks, register/flag and owner restoration")

        (tmp / "peer.rs").write_text(PEER)
        subprocess.run(shlex.split(args.rustc) + [str(tmp / "peer.rs"), "--edition=2024",
            "--extern", f"serial_zmodem={args.library}", "-L",
            f"dependency={args.library.parent / 'deps'}", "-o", str(tmp / "peer")], check=True)
        bootstrap = (args.dir / "BOOTSTRAP.COM").read_bytes()
        validate_ascii(bootstrap)
        stage2 = (args.dir / "BOOT2.BIN").read_bytes()
        target = (args.dir / "ZRECV.COM").read_bytes()
        encoded = subprocess.check_output([str(tmp / "peer"), "encode", str(args.dir / "ZRECV.COM")])
        stream = stage2_stream(stage2, target, encoded)
        machine = Machine(bootstrap, stream)
        machine.run()
        assert machine.exit == 0, (machine.exit, machine.output)
        assert bytes(machine.files["ZRECV.COM"]) == target
        assert b"B1>" in machine.output and b"B2>" in machine.output and b"B3>" in machine.output
        assert machine.divisor == [12, 0]
        print("PASS ASCII COM -> binary stage2 -> ZX0 -> byte-identical ZRECV.COM at 9600")

        for name in ("BAUDRATE.COM", "ZSEND.COM"):
            receiver = Machine(bytes(machine.files["ZRECV.COM"]), files=machine.files)
            with subprocess.Popen([str(tmp / "peer"), "send", str(args.dir / name)],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE) as peer:
                receiver.peer = peer
                try:
                    receiver.run()
                    assert receiver.exit == 0, (receiver.exit, receiver.output[-100:])
                    peer.stdin.close()
                    assert peer.wait(timeout=10) == 0
                finally:
                    if peer.poll() is None:
                        peer.kill()
            assert bytes(machine.files[name]) == (args.dir / name).read_bytes()
            print(f"PASS installed ZRECV receives {name} from real Rust ZMODEM sender")

        metadata_smoke(tmp, tmp / "peer", target, (args.dir / "ZSEND.COM").read_bytes())

        raw = bytearray()
        for offset in range(0, len(target), 4096):
            block = target[offset:offset + 4096]
            raw += struct.pack("<HH", len(block), 0x8000 | len(block)) + block
        raw_machine = Machine(bootstrap, stage2_stream(stage2, target, raw))
        raw_machine.run()
        assert raw_machine.exit == 0 and bytes(raw_machine.files["ZRECV.COM"]) == target
        print("PASS raw-block fallback including the final short block")

        for length in (0, 8193):
            bad = Machine(bootstrap, struct.pack("<H", length))
            bad.run()
            assert bad.exit != 0 and not bad.files and b"B!" in bad.output
        invalid_size = struct.pack("<H", len(stage2)) + stage2 + checksum(stage2) + b"\0\0"
        bad = Machine(bootstrap, invalid_size)
        bad.run()
        assert bad.exit != 0 and not bad.files and b"B!" in bad.output
        print("PASS stage/image size boundaries fail before execution or installation")

        # Corruption must not execute an unverified stage2 or retain a partial file.
        corrupt = bytearray(stream)
        corrupt[2 + len(stage2)] ^= 1
        bad = Machine(bootstrap, corrupt)
        bad.run()
        assert bad.exit != 0 and not bad.files and b"B!" in bad.output
        corrupt = bytearray(stream)
        corrupt[-1] ^= 1
        bad = Machine(bootstrap, corrupt)
        bad.run()
        assert bad.exit != 0 and "ZRECV.COM" not in bad.files and b"B!" in bad.output
        existing = {"ZRECV.COM": bytearray(b"keep this file")}
        bad = Machine(bootstrap, stream, files=existing)
        bad.run()
        assert bad.exit != 0 and existing["ZRECV.COM"] == b"keep this file"
        print("PASS corruption and existing-file refusal leave no damaged installation")


if __name__ == "__main__":
    main()
