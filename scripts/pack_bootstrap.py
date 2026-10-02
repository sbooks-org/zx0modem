#!/usr/bin/env python3
"""Assemble an ASCII-only 80286 COM that reconstructs BOOT1 at offset 6000h.

The output is executable machine code, not a DEBUG script. Each word is built
with printable AND/SUB instructions, then written with printable AND/XOR
instructions. CR LF space encodes OR AX,200Ah between records; AX is reset
before use. Finally a distant AA placeholder becomes RET/NOP, and RET enters
BOOT1. No assumptions about initial general registers or unused RAM contents.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import tempfile

LOAD = 0x6000
MAX_STAGE1 = 1200


def printable_sum(value: int) -> tuple[int, int, int]:
    """Represent a byte residue as a sum of three printable byte values."""
    total = value if value >= 0x60 else value + 0x100
    a = min(0x7E, total - 0x40)
    b = min(0x7E, total - a - 0x20)
    return a, b, total - a - b


def subtraction_terms(value: int) -> tuple[int, int, int]:
    target = (-value) & 0xFFFF
    low = printable_sum(target & 0xFF)
    carry = sum(low) >> 8
    high = printable_sum(((target >> 8) - carry) & 0xFF)
    return tuple(lo | (hi << 8) for lo, hi in zip(low, high))


def set_ax(value: int) -> list[str]:
    # The two masks have no common set bits, so initial AX is immaterial.
    return ["and ax,2020h", "and ax,4040h"] + [
        f"sub ax,0{part:04X}h" for part in subtraction_terms(value)
    ]


def assembly(payload: bytes) -> tuple[str, int]:
    if not payload or len(payload) > MAX_STAGE1:
        raise ValueError(f"BOOT1 must contain 1..{MAX_STAGE1} bytes")
    padded = payload + (b"\x90" if len(payload) & 1 else b"")
    words = len(padded) // 2
    # Fixed instruction lengths, checked against the resulting image below.
    tail_offset = 0x100 + 28 + 26 * words + 67
    if tail_offset + 4 >= LOAD:
        raise ValueError("printable decoder overlaps BOOT1")
    lines = [".286", ".model tiny", ".code", "org 100h", "start:"]
    lines += ["and ax,2020h", "and ax,4040h", "push ax", "pop cx"]
    lines += set_ax(LOAD - 0x20) + ["push ax", "pop bx", "or ax,200Ah"]
    for index in range(0, len(padded), 2):
        lines += set_ax(int.from_bytes(padded[index:index + 2], "little"))
        lines += ["and word ptr [bx+20h],cx", "xor word ptr [bx+20h],ax",
                  "inc bx", "inc bx", "or ax,200Ah"]
    lines += set_ax(tail_offset - 0x20) + ["push ax", "pop bx"]
    lines += set_ax(0x4141 ^ 0x90C3) + ["xor word ptr [bx+20h],ax"]
    lines += set_ax(LOAD) + ["push ax"]
    # More than the 286 prefetch queue separates the modification from use.
    lines += ["inc dx"] * 16
    lines += ["tail_code db 'AA'", "db 0Dh,0Ah", "end start"]
    return "\n".join(lines) + "\n", tail_offset


def validate_ascii(image: bytes) -> None:
    if any(byte not in (10, 13) and not 0x20 <= byte <= 0x7E for byte in image):
        raise ValueError("assembler emitted non-printable bootstrap bytes")
    if b"\r" in image.replace(b"\r\n", b"") or b"\n" in image.replace(b"\r\n", b""):
        raise ValueError("bootstrap has an unpaired line ending")
    if any(len(line) > 78 for line in image.split(b"\r\n")):
        raise ValueError("bootstrap exceeds DOS console line limit")


def build(source: Path, destination: Path, uasm: str) -> None:
    payload = source.read_bytes()
    text, tail_offset = assembly(payload)
    environment = os.environ.copy()
    # UASM consumes this environment variable as assembler arguments.
    environment.pop("UASM", None)
    with tempfile.TemporaryDirectory(prefix="printable-com-") as temporary:
        asm = Path(temporary) / "bootascii.asm"
        binary = Path(temporary) / "BOOTSTRAP.COM"
        asm.write_text(text, encoding="ascii")
        subprocess.run([uasm, "-bin", f"-Fo={binary}", str(asm)],
                       check=True, env=environment)
        image = binary.read_bytes()
    if len(image) != tail_offset - 0x100 + 4 or image[-4:] != b"AA\r\n":
        raise ValueError("UASM instruction sizing changed; refusing invalid tail address")
    validate_ascii(image)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(image)
    print(f"printable COM: {len(image)} bytes, {len(payload)} decoded bytes, "
          f"longest line {max(map(len, image.split(b'\r\n')))}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="BOOT1.BIN assembled at ORG 6000h")
    parser.add_argument("destination", type=Path, help="printable BOOTSTRAP.COM")
    parser.add_argument("--uasm", default="uasm")
    args = parser.parse_args()
    build(args.source, args.destination, args.uasm)


if __name__ == "__main__":
    main()
