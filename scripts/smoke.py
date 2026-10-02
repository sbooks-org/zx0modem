#!/usr/bin/env python3
"""Build DOS programs and printable bootstrap with UASM in a temporary directory.

This is a compile-only smoke check: it never opens a serial device, starts an
emulator, sends bytes, changes a UART rate, or writes output into the checkout.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from pack_bootstrap import build as pack_bootstrap

ROOT = Path(__file__).resolve().parents[1]
DOS_DIR = ROOT / "dos"
CPU_UPGRADE = re.compile(r"^\s*\.(?:386|486|586|686|PENTIUM|MMX)\b|\bUSE32\b", re.I)
REGISTER_32 = re.compile(r"\b(?:EAX|EBX|ECX|EDX|ESI|EDI|EBP|ESP|EIP|EFLAGS|CR[0-9]+|DR[0-7])\b", re.I)


def source_files(sources: list[Path]) -> list[Path]:
    if sources:
        return [path.resolve() for path in sources]
    # baud.asm is an included module; boot stages are raw BIN images.
    return sorted(path.resolve() for path in DOS_DIR.glob("*.asm") if path.name != "baud.asm")

def verify_cpu_sources(programs: list[Path]) -> None:
    checked: set[Path] = set()
    for path in sorted(DOS_DIR.glob("*.asm")) + sorted(DOS_DIR.glob("*.inc")):
        checked.add(path.resolve())
    for path in programs:
        checked.add(path)

    for path in sorted(checked):
        if not path.is_file():
            raise ValueError(f"source does not exist: {path}")
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), 1):
            code = line.split(";", 1)[0]
            if CPU_UPGRADE.search(code) or REGISTER_32.search(code):
                raise ValueError(f"non-80286 source construct at {path}:{number}: {line.strip()}")

    for path in programs:
        code = "\n".join(line.split(";", 1)[0] for line in path.read_text(encoding="utf-8").splitlines())
        if not re.search(r"^\s*\.286\b", code, re.I | re.M):
            raise ValueError(f"{path} must explicitly select the 80286 instruction set (.286)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uasm", help="UASM executable (default: $UASM or PATH lookup)")
    parser.add_argument("--source", action="append", type=Path, help="COM program source; repeat to select programs")
    args = parser.parse_args()

    programs = source_files(args.source or [])
    if not programs:
        parser.error("no DOS .COM source files found")
    try:
        verify_cpu_sources(programs)
    except (OSError, ValueError) as error:
        print(f"smoke: {error}", file=sys.stderr)
        return 2

    assembler = args.uasm or os.environ.get("UASM") or shutil.which("uasm") or shutil.which("UASM")
    if not assembler:
        print("smoke: UASM not found; set UASM or pass --uasm", file=sys.stderr)
        return 2
    environment = os.environ.copy()
    environment.pop("UASM", None)

    try:
        with tempfile.TemporaryDirectory(prefix="serial-zmodem-uasm-") as temporary:
            output_dir = Path(temporary)
            for source in programs:
                extension = "bin" if source.stem in ("boot1", "boot2") else "com"
                variants = [(source.stem, [])]
                if source.stem == "zrecv":
                    variants.append(("zrecvx", ["-DEXPERIMENTAL_BAUD=1"]))
                    variants.append(("zrecvy", ["-DEXPERIMENTAL_BAUD=1", "-DBUFFERED_RECEIVE=1"]))
                for name, defines in variants:
                    image = output_dir / f"{name}.{extension}"
                    command = [assembler, "-bin", *defines, f"-Fo={image}",
                               f"-I={DOS_DIR}", str(source)]
                    print(f"smoke: assembling {source.name} as {image.name}")
                    result = subprocess.run(command, cwd=ROOT, check=False, env=environment)
                    if result.returncode:
                        return result.returncode
                    if not image.is_file() or image.stat().st_size == 0:
                        print(f"smoke: UASM produced no COM image for {source}", file=sys.stderr)
                        return 1
                    print(f"smoke: produced temporary {image.name} ({image.stat().st_size} bytes)")
            if (output_dir / "boot1.bin").is_file():
                pack_bootstrap(output_dir / "boot1.bin", output_dir / "BOOTSTRAP.COM", assembler)
    except OSError as error:
        print(f"smoke: cannot execute UASM ({error}); no device was opened", file=sys.stderr)
        return 2

    print("smoke: compile-only check passed; no serial device was opened")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
