#!/bin/sh
# Build and flash the STM32F103 Rust torque node.
#
# Usage:
#   ./flash.sh                 # build + verify, print the flash command (no write)
#   ./flash.sh --flash         # build + verify + st-flash --reset write
#
# MOE is never enabled by this firmware: flashing and the MOE-off checklist are
# inherently torque-free. Enabling gate drive still requires HIL validation and
# an explicit Safety-Reviewer sign-off.

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RUST_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/../.." && pwd)
FW_DIR="$RUST_ROOT/f103_fw"
ELF="$RUST_ROOT/target/thumbv7m-none-eabi/release/f103_fw"
BIN="$SCRIPT_DIR/f103_fw.bin"

# The linker script reserves the top flash page for the NVM record. Application
# code must stay below 252 KiB; a larger image would overwrite the parameters.
MAX_BYTES=$((252 * 1024))

# The target is selected by f103_fw/.cargo/config.toml, so build from the crate
# directory; invoking with --manifest-path from the parent would use the host
# target and fail on the vector-table link_section.
cd "$FW_DIR"
MOE_ARG=""
if [ "${2:-}" = "--moe" ]; then
    MOE_ARG="--features moe"
    BIN="$SCRIPT_DIR/f103_fw_moe.bin"
    echo "WARNING: building WITH gate-drive enable (MOE)."
    echo "Only flash this after HIL validation and Safety-Reviewer sign-off."
fi
# shellcheck disable=SC2086
cargo build --release $MOE_ARG

arm-none-eabi-objcopy -O binary "$ELF" "$BIN"
arm-none-eabi-size "$ELF"

BIN_BYTES=$(wc -c < "$BIN" | tr -d ' ')
if [ "$BIN_BYTES" -gt "$MAX_BYTES" ]; then
    echo "refusing to flash: $BIN_BYTES bytes exceeds $MAX_BYTES byte application slot" >&2
    exit 1
fi
echo "image: $BIN_BYTES bytes (limit $MAX_BYTES), NVM pages preserved"

if [ "${1:-}" = "--flash" ]; then
    echo "flashing $BIN at 0x08000000"
    # OpenOCD is the primary writer: some ST-Link V2 clones report a bogus
    # 64 MiB flash size to st-flash, which then fails verification at offset 0
    # even though the image was written. OpenOCD detects the real 256 KiB.
    openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
        -c "adapter speed 100" -c init -c "reset halt" \
        -c "flash write_image erase $BIN 0x08000000" \
        -c "verify_image $BIN 0x08000000" -c "reset run" -c exit
    echo "flash complete; MOE remains low"
else
    echo "dry run. To write the image, run: $0 --flash"
    echo "manual: openocd -f interface/stlink.cfg -f target/stm32f1x.cfg ..."
fi
