# STM32F103 mainboard firmware

The active firmware for the STM32F103RCT6 torque node is the Rust workspace in
`rust/`. The C tree in `eferu-foc/` is a pinned upstream reference with the JBOT
patches. It stays in the repository because it documents the hardware and is a
known-good hoverboard build, **not** because it is the firmware for this
self-balancing robot.

## Layout

| Path | Role |
|---|---|
| `rust/f103_core/` | Hardware-independent logic: FOC port, protocol, alerts, safety, battery, thermal, console, trace. Host-tested. |
| `rust/f103_fw/` | PAC/HAL firmware for the F103: clocks, TIM1/TIM8, ADC/DMA, both UARTs, IWDG, ISRs. |
| `rust/tools/calibration/` | Console bridge, binary trace decoder, build/flash script. |
| `rust/tools/foc_golden/` | Host recorder and golden vectors from the unmodified generated C controller. |
| `eferu-foc/` | Pinned EFeru C firmware, JBOT patches and upstream documentation. Reference and rollback only. |

## Board pinout

![STM32F103 mainboard pinout](mainboard_pinout.png)

The same image and the pin tables are in `../docs/hardware.md`. The upstream
reference also contains schematics and photos under
`eferu-foc/eferu/docs/`.

## Active firmware (Rust)

- The 16 kHz FOC controller is a bit-exact fixed-point port of the generated C
  controller, replayed against golden vectors in `rust/tools/foc_golden/`.
- USART2 (PA2/PA3, 115200 8N1) carries the stock 8-byte command and 18-byte
  feedback frames used by the Pico link. USART3 (PB10/PB11, 115200 8N1) is a
  separate text/binary debug console (`$GET`, `$GETP`, `$WATCH`, `$TRACE`,
  `$DUMP`, `$ADC`, `$SAVE`, ...).
- The link-loss timeout is monitored from boot and drives the three low-pitch
  buzzer beeps, matching the old firmware's warning. Missing or stale commands
  always force the safe output.
- The default build keeps **MOE low**: the torque loop can be observed on the
  bench, but no gate can be driven until HIL validation and explicit sign-off.
  The `moe` Cargo feature is the only path that writes `BDTR.MOE`.

### Checks and build

From `firmware/stm32f103/rust`:

```sh
cargo fmt --all -- --check
cargo test -p f103_core
cargo clippy -p f103_core --all-targets -- -D warnings
cd f103_fw && cargo build --release
```

The release image is built and packaged by `rust/tools/calibration/flash.sh`.
Run it without arguments to build, check the size limit and print the flash
command; `--flash` performs the write and needs explicit approval.

### Flashing the mainboard

Hardware: ST-Link V2, **SWDIO, SWCLK and GND only**. There is no NRST wire.

1. Power the mainboard and hold its power button during the whole write. The
   firmware-owned latch on PA5 is released on reset; without the button held,
   `st-flash` reports `NRST is not connected` and `Failed to connect to target`.
2. Run `./flash.sh --flash` from `rust/tools/calibration/`.
3. Confirm the Pico bridge reports a fresh feedback frame after boot.

Do not use `st-flash --connect-under-reset` with this wiring.

## Reference C firmware (`eferu-foc/`)

The nested checkout `eferu-foc/eferu/` is pinned at commit `4f141cb` from
`https://github.com/EFeru/hoverboard-firmware-hack-FOC.git`. **Do not push
there.** It is kept because it contains measured hardware facts that the Rust
port depends on: timer topology, ADC/DMA trigger alignment, dead time, hall and
phase mappings, current scales, board variants, schematics, pinout photos and a
working `VARIANT_USART` configuration.

It is a working hoverboard controller, but it is not suitable for this robot:

- on serial-link loss it requests `OPEN_MODE` (freewheel) only after about
  800 ms, which lets a balancing robot fall;
- it latches power off on inactivity, dead battery or overheat, removing torque
  while the robot still needs it;
- the wire protocol has only an XOR checksum, no sequence, timestamp or version,
  and very little telemetry;
- balancing is marked `TODO` for `VARIANT_HOVERBOARD`, and the generated C FOC
  core cannot be tuned or extended without re-validating the Simulink output.

The Rust firmware replaces those behaviours while keeping the validated pin
map and 16 kHz current-loop structure.

### JBOT patches in the reference tree

- `eferu-foc/eferu-config.patch` — current diff of `Inc/config.h` against the
  pinned commit.
- `eferu-foc/eferu-jbot-config.h` — full snapshot of the flashed `Inc/config.h`;
  byte-identical to the patched file in the working tree.
- `eferu-foc/eferu-jbot.patch` — earlier revision of the config change, kept for
  history. Do not apply it on top of the current snapshot.
- `eferu-foc/eferu-bench.patch` — bench-only Makefile `unlock` target fix; it is
  not applied to the flashed source.

### Historical C build profile

This table documents the last known-good C image, **not** the active firmware:

| Parameter | Value | Note |
|---|---|---|
| Build variant | `VARIANT_USART` | `make -e VARIANT=VARIANT_USART` |
| Control | `FOC_CTRL`, `TRQ_MODE` | ±1000 → ±`I_MOT_MAX` |
| `I_MOT_MAX` / `I_DC_MAX` | 10 A / 12 A | Conservative bench limits |
| `N_MOT_MAX` | 400 rpm | Bench speed ceiling |
| `SERIAL_TIMEOUT` | 160 ticks ≈ 0.8 s | Too slow for balancing |
| USART2 | `FEEDBACK_SERIAL_USART2` | PA2/PA3, 115200 8N1, 8-byte command / 18-byte feedback |
| USART3 | `DEBUG_SERIAL_USART3` | PB10/PB11, 115200 8N1, text debug |
| Inputs | `PRI_INPUT1/2 = 3, -1000, 0, 1000, 0` | Serial, no deadband |
| `FLASH_WRITE_KEY` | `0x1002` | No dual inputs |
| `FILTER` | `19661` | α ≈ 0.3 |
| `SPEED_COEFFICIENT` / `STEER_COEFFICIENT` | 16384 / 8192 | 0.5 yaw halves differential |
| `TANK_STEERING`, `INVERT_L/R_DIRECTION` | Not defined | Stock mixer and directions |

Do not raise `I_MOT_MAX` towards 15 A without measured MOSFET and pack
temperatures.

### Reference build and restore

```sh
cd firmware/stm32f103/eferu-foc/eferu
make -e VARIANT=VARIANT_USART          # build/hover.bin
make flash                             # same button-held SWD procedure
```

To restore the patched configuration:

```sh
cp firmware/stm32f103/eferu-foc/eferu-jbot-config.h \
   firmware/stm32f103/eferu-foc/eferu/Inc/config.h
```

Alternatively, apply `eferu-foc/eferu-config.patch` to the pinned `Inc/config.h`.
Do not apply the patch on top of the full snapshot.
