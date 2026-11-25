# STM32F103 Rust torque node firmware

This document describes the Rust firmware that replaces the EFeru C build on the
STM32F103RCT6 hoverboard mainboard. It is an implementation note, not a release
approval. The C tree at `firmware/stm32f103/eferu-foc/eferu` stays the behavioural reference
and rollback path; the pinned commit `4f141cbc97b297e194fd58e56444bd0322f902ac`
plus `firmware/stm32f103/eferu-foc/eferu-jbot.patch` defines the flashed configuration.

Related documents: `docs/eferu-motor-interface-study.md` (measured interface
facts), `docs/prompt-f103-rust-firmware.md` (accepted scope and decisions),
`docs/skills/rust_robotics_guidelines.md` (safety workflow).

## 1. Scope

Port the reference core and peripherals to Rust, then add what the balancing
robot needs:

- full FOC controller, diagnostics, battery and temperature monitoring, buzzer,
  non-volatile parameters, 100 Hz feedback and a high-detail calibration trace;
- two independent UART roles: USART2 control link, USART3 debug console;
- both wire contracts with a runtime switch: legacy 8-byte frames for the pinned
  Pico link (compiled but disabled in this full-behaviour build);
- the stock safe state on control-link loss (`FREEWHEEL`), 800 ms timeout,
  auto-arming on a near-zero command, and the stock hard latch-off.

Not ported: variant-specific inputs (Nunchuk/PPM/PWM, Hovercar, LCD, cruise,
sideboard). The previous torque-node variant with `RAMP_TO_ZERO`, 100 ms timeout
and no hard latch-off is superseded by this full-behaviour copy. Enabling MOE is
still gated behind HIL validation and Safety-Reviewer sign-off.

## 2. Workspace and build

```
firmware/stm32f103/rust/
  Cargo.toml                     isolated workspace; the root workspace is untouched
  f103_core/                     no_std, forbid(unsafe), I/O-free pure logic
  f103_fw/                       no_std/no_main PAC firmware and HAL
    memory.x                     FLASH 252 K + 4 K reserved for NVM, RAM 48 K
    .cargo/config.toml           target thumbv7m-none-eabi, link.x
  tools/foc_golden/              host recorder for golden vectors from the C model
```

Verification gates, from `firmware/stm32f103/rust`:

```sh
cargo fmt --all -- --check
cargo test -p f103_core                      # 78 tests
cargo clippy -p f103_core --all-targets -- -D warnings
cd f103_fw && cargo build --release
cd f103_fw && cargo clippy --release -- -D warnings
arm-none-eabi-size ../target/thumbv7m-none-eabi/release/f103_fw
```

Latest sizes: `text 50 640 B`, `data 672 B`, `bss 948 B`. The Pico side is a
separate workspace: `firmware/pico2w/core` (142 tests) and `firmware/pico2w/fw`
(`cargo check` and `cargo clippy`; its `cargo test` target is an embedded binary
and does not build on the host).

## 3. Hardware facts taken from the reference

- 64 MHz from HSI/2 x PLL16; do not assume the 72 MHz configuration.
- PWM 16 kHz centre-aligned, ARR 2000, dead time 48. Left = TIM8 U/V/W on
  PC6/PC7/PC8 and complements PA7/PB0/PB1; right = TIM1 on PA8/PA9/PA10 and
  complements PB13/PB14/PB15.
- TIM1 is master (TRGO enable), TIM8 gated slave via ITR0; TIM8 TRGO starts a
  dual ADC1/ADC2 regular-simultaneous conversion. DMA1 channel 1 completes at
  16 kHz and its TC IRQ is the FOC tick.
- Ten packed 16-bit ADC fields: `(dcr, dcl, rlA, rlB, rrB, rrC, batt1, l_tx2,
  temp, l_rx2)`. Current is `offset - sample`; the third phase is `-a - b`.
- Hall inputs are active low; generated code order is `A<<2 | B<<1 | C`.
- USART2 PA2 TX / PA3 RX is the control link (not 5 V tolerant); USART3 PB10 TX /
  PB11 RX is the debug console (5 V tolerant). Both 115200 8N1. Both are on
  APB1, so their BRR is computed from PCLK1 = 32 MHz (`0x0116` at 115200), not
  from SYSCLK; using 64 MHz would halve both baud rates and break the Pico link.
- PA4 buzzer, PB2 status LED, PA5 power latch held high by this firmware.

## 4. Runtime structure

- **FOC ISR (16 kHz, highest priority):** snapshots ADC fields, runs calibration
  gate, steps both generated controllers, publishes duty/DC/speed/diagnostics,
  drives the buzzer, and optionally encodes one trace frame. It never formats
  text or waits for a consumer.
- **5 ms DWT-timed main loop:** decodes control frames, runs `SafeController::tick`,
  applies the differential mixer, publishes wheel targets, evaluates battery,
  temperature, inactivity, serial timeout and motor faults, selects the alert,
  emits 100 Hz feedback, and drains console input.
- **USART2 ISR:** circular DMA RX with IDLE detection publishes the write
  position; the main loop consumes only bytes written since the previous IDLE
  frame, wrapping at the end of the buffer, as in `usart2_rx_check()` in the C
  reference. A TXE-driven ring emits feedback frames byte by byte.
- **USART3 ISR:** RXNE feeds the console line buffer; TXE drains either the text
  reply ring or, when a trace is active, the binary trace ring.

`f103_core::foc_generated` is a literal fixed-point port of the generated C
controller: three distinct PI implementations, back-calculation anti-windup,
hall-based angle and speed estimation, 30-degree-offset Q14 tables, the C axis
conventions for Clarke/Park, motor limits, diagnostics and the three-track
scheduler. It is compiled into the firmware and held in OPEN mode until the
safety controller is armed.

## 5. Wire contracts

Both contracts are always compiled; the mode is runtime-only and resets to
`Legacy` at boot.

- **Legacy command:** `CD AB <steer i16> <speed i16> <xor16>`, 8 bytes.
- **Legacy feedback:** 18 bytes, nine little-endian u16 words with XOR checksum.
- **Hardened frame:** version 2, message type, flags, u16 sequence, u32 timestamp,
  explicit length, CRC-16/CCITT-FALSE and COBS with a 0x00 delimiter. Command
  payload is `(steer, speed)`; feedback payload is 25 bytes and also carries
  error code, hall codes, DC currents and duty.
- **Trace frame:** message type 3, 88-byte payload of internal FOC state.

Golden vectors are pinned on both sides: command `CD AB 34 12 78 56 81 EF`,
feedback `CD AB 64 00 9C FF C8 00 38 FF 36 10 5D 01 00 00 AE BA`, hardened
command `03 02 01 09 02 01 06 05 04 03 04 64 05 38 FF C4 40 00`, CRC check
`crc16(b"123456789") == 0x29B1`. The Pico mirror lives in
`firmware/pico2w/core/src/hardened.rs`.

## 6. Safety model

This is a full behavioural copy of the pinned reference, not the earlier
torque-node variant:

- Default safe mode is `FREEWHEEL` (`util.c:1032-1033`), with `RAMP_TO_ZERO` and
  `ACTIVE_BRAKE` still selectable in code. The command timeout is 800 ms
  (`SERIAL_TIMEOUT = 160` in `config.h:670`).
- Auto-arming is restored (`main.c:270-275`): no fault plus a near-zero command
  sets `enable = 1` and plays the two short confirmation beeps. A latched fault
  is never cleared by this path.
- Hard latch-off is restored: a dead pack, overheat or inactivity releases the
  PA5 latch after the reference power-off sweep (`util.c:1561-1575`).
- Mixer defaults match the reference: speed 1.0, yaw 0.5.
- The wire contract is legacy-only: `$SET PROTOCOL HARDENED` is refused. The
  hardened codec remains compiled but is not used by this build.
- **MOE is still never set by default.** Gate drive lives behind the optional
  `moe` Cargo feature of `f103_fw`; the default image cannot energize the bridge.
  Even the `moe` image requires armed + calibrated + fault-free at every FOC
  tick. The HIL checklist and sign-off form are in `docs/moe-hil-checklist.md`.
- IWDG, DC overcurrent counting, current limits and DWT timing counters are
  active. `$GET` reports `cyc=`, `max=` and `miss=` (deadline is 4000 cycles).
- Audible alerts keep the reference priority from `main.c:566-597`: thermal
  poweroff, dead battery, motor diagnostic, ADC timeout, serial timeout, general
  timeout, thermal warning, battery level 1/2, backward drive. The serial
  timeout uses `SerialTimeoutMonitor` and is raised from the first tick without
  a valid frame, so a missing Pico link beeps three low-pitch bursts even before
  any command has been accepted. `ADC timeout`, `general timeout` and the input
  index beeps are inactive because their input paths (`CONTROL_ADC`, PPM/PWM,
  Nunchuk, sideboard) are not part of this build; temperature alerts stay
  disabled as in `config.h`. `$GET` reports the same link timeout as
  `fault=LINK_TIMEOUT`.

## 7. Calibration and debugging interface

USART3 console commands:

| Command | Effect |
|---|---|
| `$HELP` | grammar summary |
| `$GET` | health summary: ticks, faults, mode, protocol, battery, temperature, diagnostics, DWT timing, NVM source |
| `$GETP` | all 13 tunables as `P <NAME> <ACTIVE_VALUE>` |
| `$SET <KEY> <VALUE>` | immediate clamped change; actuator keys refused while armed |
| `$SET WATCH <0-255>` | blackbox decimation; `0` disables |
| `$SET PROTOCOL LEGACY\|HARDENED` | runtime wire switch |
| `$TRACE <0-255>` | binary trace every N FOC ticks; `0` disables |
| `$ARM` | bench arm: run the generated torque loop with MOE still low; requires ADC calibration |
| `$DISARM` | return the generated controller to OPEN mode |
| `$INIT` | re-run ADC offset calibration and reset both controllers |
| `$SAVE` | write the active parameters to NVM; refused while armed |
| `$DUMP` | 32-sample blackbox, one ASCII line per 5 ms tick |
| `$ADC` | raw packed ADC fields |

Tunables: `IMAX`, `NMAX`, `STEER`, `SPEED`, `IDKP`, `IDKI`, `IQKP`, `IQKI`,
`NKP`, `NKI`, `CURRFILT`, `VDMAX`, `OPENRATE`. `Tunable::set` owns the clamping:
IMAX <= 8000, NMAX <= 6400, VDMAX <= 14400, mixer coefficients <= 16384.
Mixer coefficients change only the differential split; the other keys change
the generated controller's gains and limits. Runtime values do not become
persistent until `$SAVE`.

The trace payload uses native widths (u32 tick, i16 fields, i32 integrators),
so scripts see the exact fixed-point state. At 115200 8N1 one frame costs about
1 ms on the wire; `$TRACE` clamps any non-zero decimation below 160 to 160
(100 Hz) so console replies are not starved. `TRACE_DROPPED` reports ring
overflow. The layout, round-trip tests and a golden vector are in
`f103_core/src/trace.rs`.

`$ARM` is a bench-only switch that lets the generated current loop run so HIL
can observe `iq`/`id`, the PI integrators and the computed duty. It never writes
the timer `BDTR.MOE` register, so no gate can be driven and no torque can be
produced; `$DISARM` returns the controller to OPEN mode.

### 7.1 Host calibration tools

`firmware/stm32f103/rust/tools/calibration` contains dependency-free Python
helpers that talk to USART3 (115200 8N1, configured with `termios`):

- `f103_trace.py` mirrors the hardened COBS/CRC16 frame codec and the 88-byte
  trace payload. `--selftest` checks the CRC check value, both protocol golden
  vectors, a full trace round trip and a chunked-stream demultiplexer.
- `f103_bridge.py` exposes `get`, `getp`, `set <KEY> <VALUE>`, `save`, `arm`,
  `disarm`, `adc`, `dump`, `init`, `protocol` and `trace`. The `trace` subcommand
  writes one CSV row per frame with all internal FOC fields.
- `flash.sh` builds, runs `objcopy`, checks the image against the 252 KiB
  application slot and flashes with `st-flash --reset write ... 0x8000000`.
  It defaults to a dry run; `--flash` performs the write.

The MOE-off HIL checklist is in `tools/calibration/README.md`.

## 8. Golden vectors and NVM

`tools/foc_golden` compiles the **unmodified** generated C controller on the
host and records 120 ticks for the left phase-current selection
(`z_selPhaCurMeasABC = 0`) and the right selection (`= 1`). Two host tests replay
both CSVs and assert all eight outputs bit-exactly, with no tolerance. The
`host_limits.h` shim only satisfies the generated source's 32-bit `long` check;
it does not touch the reference. Regeneration commands are in
`tools/foc_golden/README.md`.

NVM uses the two top flash pages and a keyed, versioned, CRC-checked 16-word
record (`FLASH_WRITE_KEY = 0x1002`, version 2). A foreign key, version or CRC
falls back to compiled defaults; a corrupt limit is clamped, never amplified.
Protocol mode and safe mode are deliberately not persisted.

## 9. Status and remaining gates

Implemented: full FOC port with golden vectors, safety controller, dual wire
contracts, 100 Hz feedback, monitors and alerts, buzzer, NVM, DWT timing and the
calibration interface.

Not yet done, and required before the robot can balance:

1. HIL with wheels lifted: phase order, hall mapping, current sign, duty scaling,
   limits, each safe mode, buzzer, watchdog and FOC jitter measured with DWT.
2. A safety-reviewed arm path and explicit approval before MOE is enabled.
3. Resolution of the GPL-3.0 derivative-work obligation for this port.
4. Optional: 22-byte feedback with wheel counters, and `$SET` persistence of
   limits after HIL sign-off.

No flashing, deployment or publishing has been performed from this work.
