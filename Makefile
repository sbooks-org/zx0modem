UASM ?= uasm
# UASM treats an exported UASM variable as an input path; pass it only as argv.
unexport UASM
CARGO ?= cargo
DOS_BUILD ?= build/dos
DOS_IMAGES := $(DOS_BUILD)/BAUDRATE.COM $(DOS_BUILD)/ZSEND.COM $(DOS_BUILD)/ZRECV.COM $(DOS_BUILD)/ZRECVX.COM $(DOS_BUILD)/ZRECVY.COM
BOOT_IMAGES := $(DOS_BUILD)/BOOT1.BIN $(DOS_BUILD)/BOOT2.BIN $(DOS_BUILD)/BOOTSTRAP.COM

.PHONY: all host dos bootstrap smoke clean
all: host dos bootstrap

host:
	$(CARGO) build --release

dos: $(DOS_IMAGES)

$(DOS_BUILD):
	mkdir -p "$@"

$(DOS_BUILD)/BAUDRATE.COM: dos/baudrate.asm dos/serial.inc dos/uart_guard.inc dos/baud.asm | $(DOS_BUILD)
	$(UASM) -bin -Fo="$@" -I=dos/ "$<"

$(DOS_BUILD)/ZSEND.COM: dos/zsend.asm dos/serial.inc dos/uart_guard.inc dos/baud.asm dos/zmodem.inc dos/zx0.inc dos/filemeta.inc | $(DOS_BUILD)
	$(UASM) -bin -Fo="$@" -I=dos/ "$<"

$(DOS_BUILD)/ZRECV.COM: dos/zrecv.asm dos/serial.inc dos/uart_guard.inc dos/baud.asm dos/zmodem.inc dos/zx0.inc dos/filemeta.inc | $(DOS_BUILD)
	$(UASM) -bin -Fo="$@" -I=dos/ "$<"

$(DOS_BUILD)/ZRECVX.COM: dos/zrecv.asm dos/serial.inc dos/uart_guard.inc dos/baud.asm dos/zmodem.inc dos/zx0.inc dos/filemeta.inc dos/training.inc | $(DOS_BUILD)
	$(UASM) -bin -DEXPERIMENTAL_BAUD=1 -Fo="$@" -I=dos/ "$<"

$(DOS_BUILD)/ZRECVY.COM: dos/zrecv.asm dos/serial.inc dos/uart_guard.inc dos/baud.asm dos/zmodem.inc dos/zx0.inc dos/filemeta.inc dos/training.inc dos/buffered.inc | $(DOS_BUILD)
	$(UASM) -bin -DEXPERIMENTAL_BAUD=1 -DBUFFERED_RECEIVE=1 -Fo="$@" -I=dos/ "$<"

bootstrap: $(BOOT_IMAGES) $(DOS_IMAGES)

$(DOS_BUILD)/BOOT1.BIN: dos/boot1.asm dos/uart_guard.inc | $(DOS_BUILD)
	$(UASM) -bin -Fo="$@" -I=dos/ "$<"

$(DOS_BUILD)/BOOT2.BIN: dos/boot2.asm dos/serial.inc dos/uart_guard.inc dos/zx0.inc | $(DOS_BUILD)
	$(UASM) -bin -Fo="$@" -I=dos/ "$<"

$(DOS_BUILD)/BOOTSTRAP.COM: $(DOS_BUILD)/BOOT1.BIN scripts/pack_bootstrap.py
	python3 scripts/pack_bootstrap.py --uasm "$(UASM)" "$<" "$@"

smoke:
	python3 scripts/smoke.py --uasm "$(UASM)"

clean:
	rm -rf "$(DOS_BUILD)"
	$(CARGO) clean
