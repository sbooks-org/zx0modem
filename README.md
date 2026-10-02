# serial-zmodem

A Rust serial-console relay and ZMODEM endpoint for an attached DOS PC, plus DOS real-mode utilities assembled with UASM for an 80286. The host prefers `/dev/cu.*` USB serial ports, falls back to `/dev/tty.*`, and accepts `--port` for explicit selection. Serial is 8N1 with RTS/CTS hardware flow control.

## Host

```sh
cargo run --release -- --baud 9600 --send FILE
```

`--send FILE` selects a file for automatic upload when the DOS receiver announces ZRINIT. It also enables `Ctrl-] s` to resend that file. `--receive-dir DIR` chooses where incoming files are stored; existing filenames are not overwritten. Without `--send`, use `Ctrl-] r` to receive manually.

`--list-ports` enumerates matching device paths without opening them. At the terminal, press `Ctrl-] ?` for commands: `s` send, `r` receive, `b` set the host baud, `q` quit, `]` send a literal Ctrl-]. Startup sends no serial data.

Ctrl-] means hold Ctrl and press `]`, release both, then press the command key
(no Enter needed). Both the legacy `0x1D` encoding and an explicit Ctrl-] key
event are accepted. Received text is flushed after each serial read, so DOS
prompts display even without a newline.

Uploads and downloads show a live progress bar with the filename, payload byte
count, percentage, and average KiB/s. This applies to both automatic transfers
and `Ctrl-] s` / `Ctrl-] r`. The display refreshes at most ten times per second
during data transfer and reaches 100% only after the finish handshake succeeds.
Compressed transfers count encoded bytes, labelled `wire B`; compressed receives
show a moving indicator and `--%` until the encoded total becomes known.

After building, `python3 scripts/terminal_smoke.py --rustc 'rustc +stable'`
checks the production terminal loop with isolated keyboard and serial PTYs:
prompt display, Ctrl-] help/quit, keyboard forwarding, live upload/download bars,
compressed and empty-file progress, byte-identical transfers, and raw-mode restoration.
It does not open physical serial hardware or verify RTS/CTS wiring.

### Prompt-driven physical transfers

`scripts/serial_transfer.py` drives the actual release `serial-zmodem` through a
controlling-terminal PTY; it is not a second serial/ZMODEM implementation.
Close other terminals using the adapter first. Live operation requires an
explicit `--port`, and the DOS console must be at 9600 baud for transfers.
It waits for a standard `C>` / `C:\PATH>` prompt (including ANSI serial-BIOS
cursor positioning and prompts without newlines), sends one Enter if no prompt
appears after `--prompt-wait` seconds, then types the receiver command once.

```sh
cargo +stable build --release
# Inspect first; never assume a destination filename is unused.
python3 scripts/serial_transfer.py --port /dev/cu.usbserial-BG04SR6F \
  --command-only 'DIR'

# Install the separately named buffered receiver through the existing ZRECVX.
# This does not replace ZRECVX.COM; ZRECVY.COM must not already exist on DOS.
python3 scripts/serial_transfer.py --port /dev/cu.usbserial-BG04SR6F \
  --receiver ZRECVX --experimental-baud build/dos/ZRECVY.COM

# Run Y only after installation. QBASICQ.EXE must be absent in this DOS directory.
python3 scripts/serial_transfer.py --port /dev/cu.usbserial-BG04SR6F \
  --receiver ZRECVY --experimental-baud --timeout 300 QBASICQ.EXE
```

`--receiver ZRECV` uses the ordinary 9600-baud path; experimental training is
opt-in and requires `ZRECVX` or `ZRECVY`. `--executable PATH` selects a preserved
release binary for baseline comparisons. For an X/Y comparison, inspect `DIR`
and make local copies of the same source under **different unused DOS 8.3
basenames**; the transmitted basename is unchanged. Never delete or overwrite
existing DOS files to make a benchmark run. Existing destinations are refused,
and skip/error/timeout exits are nonzero rather than reported as success.

Progress is streamed to stdout; `--log NEW_FILE` additionally saves raw console
bytes without truncating an existing log. A final `serial-transfer:` JSON record
on stderr includes source size, encoded payload size, the selected training baud,
the relay's reported KiB/s, payload-start/completion timestamps, and observed bytes/s.
The payload interval starts at the first progress frame, after training, and
ends at the 100% finish-handshake frame: prompt wait and link training are
excluded, but metadata/finish protocol overhead is included. PTY observation
has scheduling/read granularity; a same-read start/finish has null observed
bytes/s, not invented precision. Compressed throughput counts **encoded payload
bytes**, not the original source size or framing/escaping overhead.

Read back into a fresh host directory to compare bytes and metadata:

```sh
mkdir readback
python3 scripts/serial_transfer.py --port /dev/cu.usbserial-BG04SR6F \
  --download QBASICQ.EXE --receive-dir readback --timeout 600
cmp QBASICQ.EXE readback/QBASICQ.EXE
# For a separately installed sender, an explicit launch command is also allowed:
python3 scripts/serial_transfer.py --port /dev/cu.usbserial-BG04SR6F \
  --command 'ZSENDY OTHER.EXE' --receive-dir readback --timeout 300
```

Success requires transfer completion (and host receive confirmation for
downloads), then the returning DOS prompt, before quitting only the host relay
with Ctrl-] q. The driver sends no automatic retry, remote cancellation, reset,
delete, or overwrite command. On timeout it closes the host process; it cannot
promise the remote receiver has already returned to DOS. `--command-only`
runs exactly the explicitly supplied command and waits for the next prompt;
this confirms console return, **not** the command's DOS exit status. Use it for
inspection, not unattended destructive commands.

`python3 scripts/serial_transfer.py --self-test` exercises the orchestration
through isolated PTYs: delayed ANSI/no-newline prompts, no premature/duplicate
receiver launch, skip failure, timeout, payload timing excluding training, and
clean relay shutdown with pending PTY output.
It does not open a physical port or test the Rust transport.

### File dates and read-only state

ZMODEM transfers preserve DOS's last-write date/time as the host file's `mtime`,
in both directions, including compressed transfers. DOS timestamps have
two-second resolution: fractional seconds and odd seconds are truncated.
DOS has no timezone, so the host interprets DOS dates in its local timezone;
set `TZ` when the DOS clock uses a different zone. Ambiguous daylight-saving
times follow the host's timezone rules. Dates outside 1980–2107 and nonexistent
local times are rejected rather than silently changed.

DOS `+R` maps to removing all host write bits (`chmod a-w`), **not** removing read
permission. A host source is read-only when none of its write bits are set.
Writable incoming files retain normal creation permissions and the host's
umask. Execute/read bits and DOS `+A`, `+H`, and `+S` are not transferred.
Metadata is applied only after the payload validates; failures are transfer
errors, not successful completion. Existing destination names remain protected.

Rebuild/update both DOS transfer programs together with the host: older
versions do not preserve this metadata. The custom ZFILE trailer is now
`version=1, attributes, compression, reserved=0, time-le16, date-le16`,
after the existing filename-NUL and decimal-size-NUL fields. Only attribute
bit 0 is retained.

## DOS

Build the `.286` COM programs with UASM:

```sh
make dos UASM=/path/to/uasm
```

Outputs: `build/dos/BAUDRATE.COM`, `ZSEND.COM`, `ZRECV.COM`, and experimental `ZRECVX.COM` / `ZRECVY.COM`. DOS programs select the first BIOS-configured COM port; all transfer programs start at 9600 baud, 8N1. `BAUDRATE` preserves the current divisor until its marker is sent. RTS/CTS wiring is required. Run `ZSEND FILE` to send a DOS 8.3 file; it requests host reception automatically. Run `ZRECV` before starting a host upload; it repeats ZRINIT until a sender connects.

`BAUDRATE RATE` switches the selected COM port to one of 1200, 2400, 9600, 19200, 38400, 57600, or 115200 baud. It sends `ESC ] 777 ; BAUD ; RATE BEL` at the old speed before changing the UART. The host applies that notice automatically after a 300 ms guard interval. INT 14h supports the standard rates through 9600; higher rates use direct divisor programming and require a conventional 1.8432 MHz UART clock.

The Rust sender uses the negotiated ZX0 extension when the receiver advertises support; the DOS receiver decodes it. DOS sender uploads remain uncompressed. This extension is not understood by ordinary ZMODEM peers, so compression is sent only when the peer advertises the capability.

During binary transfers, the DOS programs temporarily own the UART and IRQ0.
Some serial-console BIOSes poll COM1 from their timer interrupt even when UART
interrupts are disabled. The transfer guard disables UART interrupts and replaces
that polling timer handler, preserving BIOS clock ticks, midnight rollover, and
INT 1Ch callbacks. The original timer vector and UART interrupt enable register
are restored on exit. BIOS-specific background work, including USB keyboard
polling and floppy motor shutdown, is deferred until release; disk interrupts
remain enabled. Local cancellation reads the existing BIOS keyboard queue rather
than calling INT 16h, which may itself consume serial input. Do not run alongside
a TSR that owns the same UART or replaces IRQ0.

The regular receiver finishes file-close and console status output before sending ZRINIT
for the next header. Printing through a serial BIOS after ZRINIT can overrun the
UART while the sender transmits ZFIN, leaving both peers waiting at completion.

### Experimental baud training

Install `build/dos/ZRECVX.COM` on DOS using a normal 9600-baud transfer, then
start the host with both endpoints initially at 9600:

```sh
target/release/serial-zmodem --port /dev/cu.YOUR_DEVICE --baud 9600 \
  --experimental-baud --send FILE
```

Run `ZRECVX` on DOS. The host trains automatically before the upload; manual
`Ctrl-] s` uses the same negotiation. No training bytes are sent merely by
starting the terminal. `ZRECV` never trains, and `ZRECVX` without the host option
accepts an ordinary transfer at 9600. The bootstrap installer still installs
the regular receiver, not the experimental one.

Training checks 9600, 19200, 38400, 57600, and 115200 in ascending order.
Each probe sends four checked 512-byte blocks through the real ZMODEM
escaping/CRC decoder. Both peers return to 9600 after every probe to agree on
its result. At the first failure, they select the last verified rate; a failure
at 9600 aborts training. The selected speed stays fixed throughout the file
transfer. Progress remains visible, and both endpoints restore the console
to 9600 afterward. Lost probe/lock replies have bounded recovery; a failed
fast transfer aborts rather than silently restarting the file at another rate.

Use `--max-baud 57600` with `--experimental-baud` to cap the candidates for a
controlled comparison or a conservative link limit. Both the host CLI and
`scripts/serial_transfer.py` accept it. The default ceiling is 115200; other
supported ceilings are 9600, 19200, and 38400. A ceiling does not bypass training.

This is experimental, not a guarantee of a particular physical-link speed.
The UART needs a conventional 1.8432 MHz clock, RTS/CTS wiring, and sufficient
CPU/FIFO capacity. Emulator checks cover protocol state, corruption, rate
disagreement, fallback, and restoration—not electrical margins or real UART
timing. Keep `ZRECV.COM` available as the fixed-speed alternative.

### Buffered receiver: ZRECVY

`ZRECVY.COM` retains X's opt-in baud training but allocates a separate 64 KiB
conventional-memory segment and advertises a 60 KiB receive window. It relocates
its stack and shrinks its DOS process allocation first. Insufficient memory
causes an explicit failure before announcing a window or creating a file.
`ZRECVX.COM` and the regular receiver retain their 512-byte windows; Y is a
separate executable, not a replacement for either.

The host honors the advertised window: one ZDATA header, 512-byte CRC-protected
subpackets ending in ZCRCG, then ZCRCW at the window/file boundary. Y receives
directly into the extra segment. Only after the entire window validates does it
decompress/write to disk and acknowledge, allowing the next burst. Compression
blocks may cross window boundaries. A damaged window is discarded and retried
from its committed position, with at most five retries; disk/decompression state
is untouched by a rejected window. Y also uses a table-driven CRC16.

Host writes are batched into reusable packet storage instead of one syscall per
byte, and drains occur at acknowledgement boundaries. Binary uploads temporarily
use a 30-second serial I/O timeout: drivers may wait for an entire queued burst
to drain, so neither the terminal's 20 ms timeout nor a single-packet timeout
is sufficient for USB/UART backpressure. Terminal timeout and
9600-baud console state are restored afterward.

Upload progress advances as payload is queued and may rewind on retransmission;
100% still requires the final handshake. Short-term queued-byte rates can exceed
the physical line rate, so compare final completed-transfer rates.

### Measured hardware throughput

An actual `QBASICQ.EXE` transfer used 267,630 source bytes and 191,875 compressed
payload bytes. Distinct DOS filenames preserved the existing executable and
receiver. These are individual completed runs, not repeated-run averages:

| Host / DOS receiver | Actual baud | Payload time | Payload rate | Original-file rate |
| --- | ---: | ---: | ---: | ---: |
| Original host / existing ZRECVX | 57,600 | 54.53 s | 3.44 KiB/s | 4.79 KiB/s |
| Batched host / ZRECVY, capped at 57,600 | 57,600 | 49.02 s | 3.82 KiB/s | 5.33 KiB/s |
| Batched host / ZRECVY, unrestricted training | 115,200 | 32.45 s | 5.78 KiB/s | 8.06 KiB/s |

At the **same baud rate**, throughput improved **11.2%**, reducing elapsed time
by **10.1%**. Including the faster selected baud, throughput improved **68.1%**
and time fell **40.5%**. Training outcomes vary: the existing X also selected
115,200 during deployment, so Y alone cannot be credited for the higher baud.

Timing starts at the first payload progress frame, excludes compression and
baud training, and includes the final transfer handshake. Payload rates exclude
ZMODEM framing and byte-escaping overhead. At 57,600, Y reaches 68.0% of the
5,760 B/s raw 8N1 ceiling; it does not saturate the link. Progress slows at
window commits, which still perform synchronous decompression and disk writes.
CPU and disk costs were not individually profiled.

The 115,200-baud result was read back with a separately installed `ZSENDY.COM`
and compared byte-for-byte with the source. Both SHA-256 hashes were
`df66d863e22ef4bad13292de261b006bd384b9661489a86706a8f97b25479e8c`.
DOS `FC /B` also found no differences against the baseline and capped-rate
results. The deployed `ZRECVY.COM` was read back and matched its host build;
the existing `ZRECVX.COM` and `QBASICQ.EXE` were not replaced.

## Bootstrapping a bare serial DOS prompt

`serial-bootstrap` installs all three DOS utilities without DEBUG, BASIC, an
existing transfer program, or binary-safe console input. The first file is an
actual executable `.COM` containing only bytes `20h..7Eh` and CRLF. Ctrl-Z is
used only to finish DOS COPY, not as part of the generated executable.

Build with a **host-native UASM**, Rust, and Python 3.9+:

```sh
make all UASM=/path/to/uasm
target/release/serial-bootstrap --dry-run
```

If rustup has no default toolchain, use `CARGO='cargo +stable'` with Make.
On Apple Silicon, an x86-64-only UASM needs an appropriate runtime or a native
UASM build; the packer does not substitute another assembler.

For a fresh installation, choose a writable DOS directory with none of
`BOOT.COM`, `ZRECV.COM`, `BAUDRATE.COM`, or `ZSEND.COM` present. Set COM1 to a
known 8N1 speed using the machine's existing setup, and run `CTTY COM1`
(or use a serial BIOS prompt).
The installer needs DOS 3+, an 80286-compatible CPU, a full 64K COM memory
segment, and a conventional COM1 UART with a null-modem RTS/CTS connection.
Resident software must not consume COM1 input while the receiver is running.

Then, on the host, explicitly choose the port and **current** prompt speed:

```sh
target/release/serial-bootstrap --port /dev/cu.usbserial-FT07AEQI --baud 9600
```

This command transmits immediately. Close the normal terminal first; do not
type into the DOS session during installation. No device is auto-selected.
`--dry-run` validates and compresses the bundle without even enumerating ports.
Use `--dir PATH` for another bundle directory. ASCII pacing defaults to 10 ms
per byte for COM1 or 60 ms for CON, plus 250 ms per line. The slower CON default
allows a serial BIOS timer tick between characters so COPY does not overflow
its keyboard queue. `--char-delay-ms` overrides either default; increase it and
`--line-delay-ms` further for a slow BIOS, UART, or disk.
The installer sends Enter and waits for a standard DOS prompt (`C>` or
`C:\PATH>`). Use `PROMPT $P$G` first if the shell has a custom prompt.
It recognizes serial BIOS ANSI cursor positioning and scrolling, including
output without literal CR/LF.

The transfer sequence is:

1. Type `COPY COM1 /A BOOT.COM /B`, followed by the printable executable
   in short CRLF lines, then Ctrl-Z and Enter. Source `/A` recognizes EOF;
   destination `/B` prevents appending an EOF byte.
2. Wait for the DOS prompt after COPY, then type `BOOT`. A fixed delay is not
   sufficient when DOS is still writing the file. Its printable instructions
   reconstruct a 456-byte binary receiver in memory, then enter it. The receiver
   selects 9600, waits about half a second for the host to switch, and sends `B1>`.
3. Send `BOOT2.BIN` as a length-prefixed binary image plus a 16-bit additive
   checksum. The receiver verifies it before executing it.
4. After `B2>`, send independent, bounded ZX0 blocks for `ZRECV.COM`. Each has
   a stored-data checksum; the installer acknowledges each written block with
   `+`. Raw blocks are used when compression would expand the data. No disk
   I/O or decompression occurs while another block is arriving.
5. After `B3>` and the DOS prompt, run `ZRECV` and use ZMODEM to install
   `BAUDRATE.COM`. Wait for the prompt again before running `ZRECV` to install
   `ZSEND.COM`. The link remains at 9600.

For a BIOS that maps the physical `CON` device to serial, use
`--input-device CON`; the capture command becomes
`COPY CON /A BOOT.COM /B`. The default explicitly names COM1 because
CTTY implementations can redirect standard handles without rebinding CON.
Stock DOS `TYPE > BOOTSTRAP.COM` alone is not an stdin-copy command.
See [FreeCOM's CTTY implementation](https://github.com/FDOS/freecom/blob/master/cmd/ctty.c)
and [COPY ASCII/binary handling](https://github.com/FDOS/freecom/blob/master/cmd/copy.c).
The host artifact is named `BOOTSTRAP.COM`, but DOS receives `BOOT.COM`:
`BOOTSTRAP` has nine letters and is not an 8.3 basename.

`ZRECV.COM` is created exclusively, never overwritten; a failed binary-stage
installation removes its own partial file. ZMODEM skips are errors rather than
false installation successes. A protocol failure reports the stage and recent
input and stops; there is no automatic retry. Check the DOS prompt and files
before retrying. Checksums detect accidental corruption, not a malicious peer.

If COPY succeeded but installation stopped before creating `ZRECV.COM`, keep
`BOOT.COM` and use `--resume` from the DOS prompt:

```sh
target/release/serial-bootstrap --port /dev/cu.usbserial-BG04SR6F --input-device CON --resume
```

This skips COPY entirely, launches the existing `BOOT.COM`, and performs the
binary installation and both ZMODEM transfers. The existing file must be an
intact copy of this bundle's `BOOTSTRAP.COM`; resume does not repair a corrupt
copy or resume partial ZMODEM transfers. Other destination files must still be
absent. Close any interactive terminal before opening the port with the installer.

After updating the bootstrap bundle, replace the old DOS `BOOT.COM`; **do not
use `--resume` with an older loader**. In particular, the old 3,167-byte loader
does not protect the UART from serial BIOS polling. Rename that copy out of the
way and perform a fresh COPY installation with the new bundle.

### How the printable executable works

`scripts/pack_bootstrap.py` generates `.286` assembly and invokes UASM. Printable
AND/SUB instructions construct each receiver word; printable AND/XOR
instructions write it to `6000h`, independent of initial RAM/register contents.
CR LF space is deliberately the instruction `OR AX,200Ah` between records.
A final self-modified RET transfers to the reconstructed receiver. Assembly
output is rejected unless every byte is printable or CRLF, every line is at
most 78 characters, and the computed code offsets match.

The current `BOOTSTRAP.COM` is 6,027 bytes with a maximum line length of 70.
It is original bootstrap code, not an antivirus test file. Do not rewrap,
trim spaces, convert line endings, or pass it through text transformations.

### Verification

```sh
cargo test
make smoke UASM=/path/to/uasm
python3 -m venv /tmp/bootstrap-venv
/tmp/bootstrap-venv/bin/pip install unicorn
/tmp/bootstrap-venv/bin/python scripts/bootstrap_smoke.py --uasm /path/to/uasm
/tmp/bootstrap-venv/bin/python scripts/experimental_smoke.py --rustc 'rustc +stable'
```

On Unix, `cargo test` also exercises the real bootstrap transport over isolated
PTYs: delayed COPY completion must not receive `BOOT` until the ANSI-rendered
DOS prompt arrives, and resume must launch without sending COPY or file data.
These tests do not open an attached serial adapter.

The CPU smoke requires the release Rust library and built DOS bundle
(`make all`). It accepts `--rustc 'rustc +stable'` if needed. It executes the
actual printable loader and DOS binaries in a 16-bit Unicorn CPU with emulated
UART registers and DOS file services. It checks byte equality through the
binary/ZX0 install and both real Rust-to-DOS ZMODEM transfers, plus bidirectional
file metadata, calendar boundaries, metadata errors, corruption, and
existing-file refusal. It also checks IRQ0 ownership restoration, midnight
rollover, and a 16-byte RX FIFO during blocking DOS console output. That FIFO
case catches finish-header loss which QEMU's buffered serial backend can mask.
This is not a complete DOS kernel or proof of physical UART timing.

The experimental smoke runs the actual `ZRECVX.COM`, `ZRECVY.COM`, and Rust
training code over rate-aware PTYs. It injects corruption to exercise 115200
selection, 38400/9600 fallback, lost-lock recovery, and rejected-window replay.
Y cases cover multiwindow data, compressed-block carry, empty files, and memory
exhaustion. It verifies metadata, terminal progress, and the returned DOS prompt.
A test-only PTY adapter replaces the hardware-only baud ioctl on macOS.
No physical serial hardware is opened by these smoke scripts.

For a real DOS end-to-end regression, download the
[FreeDOS 1.4 Floppy Edition](https://www.freedos.org/download/) and extract
`144m/x86BOOT.img`. Install QEMU and mtools, then run:

```sh
python3 scripts/qemu_bootstrap_smoke.py \
  --boot-image /path/to/144m/x86BOOT.img --uasm /path/to/uasm \
  --artifacts /tmp/dos-bootstrap-proof
```

The harness boots an isolated copy of FreeDOS with SeaBIOS serial-console
redirection and invokes the actual release installer. Both directions of its
virtual null-modem cable are paced at 9600 baud. It performs COPY CON, both binary
bootstrap stages, and both ZMODEM installs; reads back all four DOS files for
byte equality; and executes a DOS command afterward to check console restoration.
On macOS, a test-only interposer adapts the hardware baud ioctl for the PTY;
the release executable and all protocol traffic are unchanged. The source boot
image and attached physical ports are never modified by this harness.

## Hardware caution

Port enumeration does not open or transmit to attached hardware. Opening a port changes its line configuration; connect only when the DOS side is ready at the same baud rate. Automated smoke scripts use isolated PTYs or emulators. Live hardware operation requires explicitly selecting the attached port.
