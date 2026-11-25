# Prompt: Rust Firmware for the STM32F103RCT6 Hoverboard Mainboard (JBOT Torque Node)

> Purpose of this document: it is a **prompt** to hand to an AI coding agent. It is
> written in English because every committed artifact in this repository must be
> English (`AGENTS.md`). It is not a specification and not a permission to flash.
> Copy the section between the markers into a fresh agent session together with
> `AGENTS.md`, `docs/skills/rust_robotics_guidelines.md`, and
> `docs/eferu-motor-interface-study.md`.
>
> Verification note: the pin map and timings in §2 were checked against the local
> pinned tree (`Inc/defines.h`, `Inc/config.h`, `Src/setup.c`, `Src/main.c`,
> `Src/util.c`, `Inc/util.h`) and `eferu-jbot.patch`, not against upstream docs.
> Three differences from the project prose are recorded there: the flashed clock
> is 64 MHz (not 72), the feedback `cmd1`/`cmd2` fields are i16 (not u8), and the
> PWM high/low pin order is PC/PA for the left bridge and PA/PB for the right.

---
<!-- PROMPT START -->

You are a Rust embedded engineer working in the JBOT repository. Your task is to
design and implement a **Rust firmware replacement** for the STM32F103RCT6
hoverboard mainboard, using the proven `EFeru/hoverboard-firmware-hack-FOC` C
firmware as the hardware reference. The result must be a robot-grade motor node:
complete motor control, maximum observability, maximum precision, and maximum
communication speed, with **two independent UART roles** (control link to the
Pico 2 W and a dedicated debug UART).

## 0. How to work

1. Read first and obey: `AGENTS.md`, `docs/skills/rust_robotics_guidelines.md`,
   `docs/eferu-motor-interface-study.md`, `docs/hardware.md`, `docs/pico-bridge.md`,
   `docs/architecture.md`, `firmware/README.md`, and the pinned reference tree
   `firmware/stm32f103/eferu-foc/eferu/` plus `firmware/stm32f103/eferu-foc/eferu-jbot.patch`.
2. Trace every change to a requirement or task ID (`SYS-*`, `INT-*`, `SAF-*`,
   `M3-*`). If you cannot find one, state the gap and ask before coding.
3. **Stop and ask before any code** if any of these gates is unresolved:
   - Scope is now fixed: **full Rust FOC**, replacing the generated C controller.
     This reverses the recorded ADR-011 decision (`firmware/README.md` §5.1) to keep
     the C core. Obtain explicit Architect and Safety-Reviewer sign-off on the
     rewrite and on the bench acceptance plan before implementation. Do not
     silently fall back to a C/FFI shell if the port gets difficult; report the
     problem and ask.
   - The safe state on control-link loss must be selected explicitly from the three
     options in §2.4. The reference falls back to `OPEN_MODE` freewheel, which lets
     a balancing robot fall. Default target is `RAMP_TO_ZERO`; confirm before code.
   - Upstream is **GPL-3.0**. A Rust port of the generated controller is a derived
     work and stays GPL-3.0 when distributed. Do not copy generated C comments or
     structures into the workspace without making that obligation explicit.
4. Never `git push`, publish, deploy, or flash hardware without explicit user
   approval. Never weaken a watchdog, current limit, timeout, or E-stop to pass a
   test.
5. All comments, doc comments, commit messages and documentation in English.
6. Measure, do not assume. Every claim about latency, jitter, precision, or memory
   must cite a command, log, or measurement artifact.

## 1. Reference firmware: what is proven, what is not

`firmware/stm32f103/eferu-foc/eferu/` is pinned at commit
`4f141cbc97b297e194fd58e56444bd0322f902ac` with `eferu-jbot.patch`. It is a
**validated motor drive** on the actual donor board: 16 kHz FOC, two 3-phase
bridges, hall commutation, phase/DC-link current sensing, battery and temperature
measurement, power latch, and a working UART command channel. Treat its pin map,
timer topology, ADC triggering, dead time, and current-scale constants as measured
hardware facts.

It is **not** a robot control interface: `VARIANT_HOVERBOARD` marks balancing as
`TODO`, the torque path is verified correct but mechanically limited by stick-slip,
and the wire protocol is weak. Do not try to fix drivetrain stick-slip in firmware.
The generated C FOC core is the **behavioural reference to port**, not a component
to retain: use it to derive fixed-point gains, limits, angle estimation and SVPWM,
and validate the Rust port against its currents and hall states on the bench.

## 2. Fixed hardware facts — take from the reference, do not invent

### 2.1 Core and timing

| Property | Value |
|---|---|
| MCU | STM32F103RCT6, Cortex-M3, 256 KB flash, 48 KB SRAM; silicon rated 72 MHz |
| Clock actually flashed | **64 MHz**, HSI/2 (4 MHz) × PLL16 (`RCC_PLLSOURCE_HSI_DIV2`, `RCC_PLL_MUL16` in `main.c`). Do not compute timers from 72 MHz; docs that say 72 MHz describe capability, not this build |
| FPU | None; all control math is fixed-point or measured soft-float cost |
| Rust target | `thumbv7m-none-eabi` (`no_std`, `panic = "abort"`) |
| PWM | TIM1 (right), TIM8 (left), center-aligned, 16 kHz, ARR = 64 MHz / 2 / 16000 = 2000 |
| Dead time | 48 timer counts; complementary outputs, OSSR/OSSI enabled, MOE gated until ready |
| Timer sync | TIM1 master `TRGO_ENABLE`; TIM8 gated slave on ITR0; TIM8 CNT preloaded with `ADC_TOTAL_CONV_TIME` so phase-current sampling lands in the low-FET on region |
| ADC trigger | TIM8 TRGO, ADC1 + ADC2 dual regular-simultaneous, DMA1_Channel1 circular, 5 ranks |
| ADC timing | `ADC_CONV_CLOCK_CYCLES = ADC_CONV_TIME_7C5 = 20`, `ADC_CLOCK_DIV = 4`, `ADC_TOTAL_CONV_TIME = 80` |
| Main loop | `DELAY_IN_MAIN_LOOP = 5 ms`; feedback at 100 Hz (every 2 ticks); text debug at 125 ms (every 25 ticks) |
| Current scale | `A2BIT_CONV = 50` (1 A = 50 ADC bit units) |

### 2.2 Pin map (BOARD_VARIANT 0; variant 1 differs where noted)

| Function | Pins |
|---|---|
| Left halls U/V/W | PB5 / PB6 / PB7 |
| Right halls U/V/W | PC10 / PC11 / PC12 |
| Left PWM (TIM8) UH/UL, VH/VL, WH/WL | PC6/PA7, PC7/PB0, PC8/PB1 |
| Right PWM (TIM1) UH/UL, VH/VL, WH/WL | PA8/PB13, PA9/PB14, PA10/PB15 |
| ADC1 ranks | PC1/CH11, PA0/CH0, PC4/CH14, PC2/CH12 (VBAT; variant 1: PA1/CH1), internal temperature |
| ADC2 ranks | PC0/CH10, PC3/CH13, PC5/CH15, PA2/CH2, PA3/CH3 |
| ADC buffer fields | `dcr, dcl, rlA, rlB, rrB, rrC, batt1, l_tx2, temp, l_rx2` |
| LED, buzzer, latch, button, charge | PB2; PA4 (v1: PC13); PA5 (v1: PC15); PA1 (v1: PB9); PA12 (v1: PA11) |
| SWD | PA13 / PA14 |
| Control UART (USART2, left cable) | PA2 TX, PA3 RX; DMA1 Ch7 TX, Ch6 RX; circular RX + IDLE interrupt; **not 5 V tolerant** |
| Debug UART (USART3, right cable) | PB10 TX, PB11 RX; DMA1 Ch2 TX, Ch3 RX; **5 V tolerant**; shares pins with I2C2 (unused) |

There is no other UART on this part. Never connect the 12/15 V cable wire to logic.
Keep sideboards disconnected: they transmit on the same wires.

### 2.3 Parameter baseline from the flashed reference

| Parameter | Flashed value |
|---|---|
| Control | `CTRL_TYP_SEL = FOC_CTRL`, `CTRL_MOD_REQ = TRQ_MODE` (±1000 → ±`I_MOT_MAX`) |
| Limits | `I_MOT_MAX = 10 A`, `I_DC_MAX = 12 A`, `N_MOT_MAX = 400 rpm` (keep `I_DC_MAX = I_MOT_MAX + 2 A`; hard ceiling 2000 rpm) |
| Input shaping | `FILTER = 19661` (α = 0.3, Q16.16), `RATE = 480` (fixdt(1,16,4), ≈6000 units/s) |
| Mixer | `SPEED_COEFFICIENT = 16384` (1.0), `STEER_COEFFICIENT = 8192` (0.5) — **0.5 silently halves yaw; use 1.0 for the robot** |
| Serial timeout | `SERIAL_TIMEOUT = 160` ticks × 5 ms = **800 ms**, not 160 ms; expiry requests `OPEN_MODE` |
| UART | USART2 and USART3 both 115200 8N1; `SERIAL_START_FRAME = 0xABCD`; `SERIAL_BUFFER_SIZE = 64` |
| Diagnostics | `DIAG_ENA = 1`; hall/phase error code gates motor enable |
| Persistence | `FLASH_WRITE_KEY = 0x1002`; `$SAVE` in flash overrides `config.h` on every boot |
| Optional | `ENABLE_ODOMETRY` off; enabling adds 4 bytes (`wheelR_cnt`, `wheelL_cnt`) to feedback |
| Clock source | HSI/2 × 16 = 64 MHz (see §2.1), verified in `Src/main.c:635-656` and the ARR formula in `Src/setup.c` |

### 2.4 Safe-state selection on control-link loss (choose before code)

Expose this as one validated enum in configuration and in the debug console. It is
a safety contract, not a tuning knob; the selected value is reported in telemetry.

| Mode | Behaviour on timeout / invalid command stream | Use |
|---|---|---|
| `FREEWHEEL` | Immediate `OPEN_MODE`, zero duty, high impedance — the stock reference behaviour | Compatibility only; not acceptable while balancing |
| `RAMP_TO_ZERO` (default target) | Bounded torque ramp to zero within the configured timeout; MOE stays on until torque is zero, then outputs are disabled | Recommended for the balancing robot; requires a measured ramp time |
| `ACTIVE_BRAKE` | Controlled braking torque within the current limit for a bounded duration, then zero and MOE off | Requires a measured brake-current budget and thermal check before enabling |

Rules for every mode: the timeout is 100 ms target (20 ticks × 5 ms), not the
flashed 800 ms; the mode applies on CRC/sequence/age failure as well as on no
frames; recovery requires a fresh valid command **and** a fresh arm, never an
automatic resume; the IWDG and current limits stay active throughout.

**Accepted decisions (recorded 2026-11-24):** `RAMP_TO_ZERO` is the selected
default. Full functional port is required — core plus peripherals — covering
FOC, diagnostics, battery, temperature, buzzer, parameters and 100 Hz feedback,
but not the reference's variant-specific input paths (Nunchuk/PPM/PWM, Hovercar,
LCD, cruise, sideboard). The FOC controller must be ported at full fidelity and
validated against golden vectors from the unmodified generated C. Both wire
contracts stay available with a runtime switch: legacy 8-byte for compatibility
and hardened COBS/CRC16 as the target, with mirrored golden vectors on Pico and
STM32. Auto latch-off on battery/temperature/inactivity is **not** ported: on a
balancing robot it removes torque and causes a fall; those conditions raise a
safe-state fault and an alert instead.

### 2.5 Wire frames exactly as flashed (compatibility mode)

From `Inc/util.h` and `Src/main.c:124-140`, little-endian, no padding:

- Command, 8 bytes: `start u16 = 0xABCD`, `steer i16`, `speed i16`,
  `checksum u16 = start ^ steer ^ speed`.
- Feedback, 18 bytes: `start u16`, `cmd1 i16`, `cmd2 i16`, `speedR_meas i16`,
  `speedL_meas i16`, `batVoltage i16`, `boardTemp i16`, `cmdLed u16`,
  `checksum u16`. With `ENABLE_ODOMETRY`, `wheelR_cnt i16` and `wheelL_cnt i16`
  are inserted after `speedL_meas` → 22 bytes. `cmd1`/`cmd2` are **i16**, not bytes.
- Mixer (`Src/util.c:1706-1722`): `speedR = (speed - steer) >> 4`,
  `speedL = (speed + steer) >> 4`, with both inputs first scaled by
  `COEFFICIENT >> 14` and clamped to `INPUT_MIN..INPUT_MAX`. Preserve the sign
  convention when porting; it is part of the wheel-direction contract.

## 3. Problems in the reference that the new firmware must eliminate

1. **Weak wire protocol.** XOR checksum, no sequence, timestamp, version or length;
   an 8-byte frame is accepted only when the byte count since the last IDLE gap is
   exactly 8. Stale/corrupt frames can be applied inside the timeout window.
2. **800 ms stale-torque window and freewheel on link loss.** Must become one of
   the three safe states in §2.4 with a 100 ms command budget.
3. **Poor observability.** Feedback carries only input echo, RPM, battery and
   temperature. No hall state, error code, DC/phase current, duty, applied torque,
   loop timing or fault history on the wire. Debug text is mutually exclusive with
   feedback on the same cable.
4. **Runtime tuning is slow.** Generated FOC PI gains are not settable; `$SET` only
   covers a short parameter list at 8 Hz, while `DEBUG_SERIAL_PROTOCOL` is disabled.
5. **Silent configuration source.** Flash overrides `config.h`; a rebuild can appear
   to have no effect.
6. **No robot logic on the mainboard.** Keep posture authority on the Pico; this
   firmware is a precise, observable torque actuator, not a balancer.
7. **Latency myth.** The 8-byte command at 200 Hz is 1600 B/s (≈1.4% of a 115200
   link); raising baud does not remove the dominant lag. Optimize for jitter,
   determinism, and feedback richness, not for a headline baud number.

## 4. Functional requirements

### 4.1 Full motor control
- Independent left and right channels; both FOC current loops at 16 kHz, now
  implemented in Rust (no C core, no FFI fallback).
- Modes: `TRQ_MODE` (primary; ±1000 → ±`I_MOT_MAX`), `VLT_MODE`, `SPD_MODE`,
  `OPEN_MODE` (safe state only). Commutation/sinusoidal are not required.
- Link-loss behaviour is the selected mode of §2.4 and is enforced by the same
  state machine that handles faults and arming, not by a separate timer path.
- Enforce phase-current limit, DC-link limit, speed limit, hall/phase diagnostics,
  battery undervoltage and temperature poweroff. Limits are validated at startup
  and never silently overridden by stale flash.
- Disable PWM outputs (MOE low) until self-test passes; re-arm only after a clean
  enable sequence. A fault must not leave a channel partially driven.

### 4.2 Maximum precision
- Fixed-point math with explicit Q formats and documented saturation/clamping.
  No `f64`. Use `f32` only where a measurement proves the cost fits the 62.5 µs
  FOC budget.
- Keep the reference's ADC/DMA trigger alignment. Sample phase currents in the
  low-FET on region; preserve `ADC_TOTAL_CONV_TIME` offset and 7.5-cycle sample
  time for current channels and 239.5 cycles for temperature.
- Port the fixed-point PI gains, angle estimation, saturation and SVPWM constants
  from `Src/BLDC_controller.c` / `BLDC_controller_data.c`; there is no Simulink
  model in this repository, so re-derive and document each constant. Do not tune
  them to mask mechanical stick-slip.
- Preserve hall-mapping and phase-order validation from the reference. Provide a
  bring-up ladder (sinusoidal → FOC voltage → FOC torque) as a test procedure.
- Deterministic timing from hardware timers, not delay chains. Expose loop period,
  execution time and deadline misses as counters.

### 4.3 Maximum communication speed
- USART2 control link: DMA circular RX with IDLE detection and a fixed-size parser;
  target 200–400 Hz commands and ≥100 Hz feedback. Raise baud only after
  measuring that it helps (suggest 921600 as an optional profile, not a default
  claim).
- Apply the newest valid command only; never replay stale setpoints. Validate
  version, length, CRC, sequence and age before touching the actuator.
- Keep the UART path off the FOC deadline. ISR copies bytes and returns; parsing
  happens in a lower-priority task. No allocation, formatting or logging in ISR.

### 4.4 UART #1 — control link to Pico 2 W (USART2, left cable)
- Keep the stock 8-byte command layout as a compatibility mode until the Pico
  side is upgraded: `0xABCD`, steer i16, speed i16, XOR checksum.
- Target frame adds: magic, version, message type, u16 sequence, u32 monotonic
  timestamp, CRC-16/CCITT-FALSE, and explicit length. Framing is COBS with `0x00`
  delimiter, ≤256 bytes. This is a **shared contract** with `jbot_pico_core`:
  freeze it in an ADR with golden byte vectors and property tests on both ends.
- Feedback must report applied torque/current, hall state, error code, DC current
  per motor, duty, battery, board temperature, sequence echo and timestamp — not
  just the input echo. Optional wheel counters (`ENABLE_ODOMETRY` equivalent) to
  remove RPM-integration drift on the Pico.

### 4.5 UART #2 — dedicated debug console (USART3, right cable)
- Completely independent from the control link. Text console with
  `$GET / $HELP / $WATCH / $SET / $INIT / $SAVE` compatibility, plus binary
  telemetry dump and blackbox upload.
- Runtime-settable: control mode, control type, current/speed limits, filter,
  rate, mixer coefficients, serial timeout, safe-state selection. Mark which
  values persist and provide an explicit "show active source (flash vs config)"
  command. A `FLASH_WRITE_KEY` change must invalidate old records.
- `$WATCH` equivalent for up to 15 named variables; stream at a configurable
  rate without blocking the control loop.

**Implemented subset (stage 4):** `$HELP`, `$GET`, `$WATCH` (toggle),
`$SET WATCH <0-255>` (decimation), `$SET PROTOCOL LEGACY|HARDENED`,
`$INIT` (re-run ADC offset calibration), `$SAVE` (writes the parameter record
to the reserved flash pages), `$DUMP` (32-sample blackbox, streamed one line per
5 ms tick), `$ADC` (raw packed fields). `$GET` reports `proto=`, `bat=`, `temp=`,
`diag=`, the DWP cycle counters (`cyc=`, `max=`, `miss=`) and `nvm=FLASH|DEFAULT`.
Both directions use bounded rings and a TXE-driven ISR, so no reply blocks the
control loop. Grammar and formatting live in `f103_core::console` and are
golden-tested on the host. Live changes to control mode, limits, mixer, timeout
or safe-state selection are intentionally **not** exposed until HIL validation
and a new safety decision; `$SET` rejects them.

NVM decision: the two top flash pages store a keyed, versioned and CRC-checked
record with `i_max`, `n_max` and mixer coefficients. The key is the flashed
`VARIANT_USART` value `0x1002`; a foreign key, version or CRC falls back to the
compiled defaults, and any stored limit is clamped to the hardware maximum so a
corrupt record cannot raise current or speed. `$SAVE` refuses to write while
armed because flash erase stalls instruction fetch. Protocol mode and safe mode
are **not** persisted: both are safety contracts and reset to their audited
defaults at boot. The restored record is reported but does not yet change the
active drive values; that requires HIL approval.

### 4.6 Maximum debugging / observability
- Counters: frames received, CRC errors, oversize, resyncs, sequence gaps,
  command age, timeouts, loop overruns, deadline misses, fault codes, watchdog
  resets, echo mismatches.
- Blackbox ring buffer (static, bounded) of setpoints, applied torque, currents,
  hall state, faults and timing, dumped on request or after a fault.
- No logging in the FOC or control path. Use `defmt`/deferred output or a
  buffered dump at low rate.
- A GPIO timing marker for logic-analyzer measurement of FOC and command egress.

**Calibration interface.** A script must be able to sweep gains and observe the
result without a rebuild or a flash erase per iteration:

- `$GETP` lists every tunable as `P <NAME> <ACTIVE_VALUE>`. `$SET <KEY> <VALUE>`
  changes one value immediately; all keys are clamped to hardware-proven maxima
  in `Tunable::set`, and actuator-key changes are refused while the node is
  armed. Mixer coefficients change the differential split only; the other keys
  change the generated controller's gains/limits.
- `$SAVE` writes the active set of tunables into the keyed NVM record. A foreign
  key, version mismatch or CRC failure falls back to compiled defaults, and a
  corrupt limit is clamped rather than amplified.
- `$TRACE <N>` streams a binary COBS/CRC16 frame every N FOC ticks; `0` disables
  it. Each frame carries both motors' mode, scheduler token, electrical angle,
  speed, target, filtered `iq/id`, Vd/Vq, both PI integrators (full 32-bit Q16),
  phase amplitudes, error codes, DC currents and the three applied duties. All
  fields use native i16/u16/i32 widths — no lossy bit packing — so a script sees
  the exact fixed-point state. The 88-byte little-endian layout and a golden
  vector are pinned in `f103_core::trace`.
- At 115200 8N1 a frame costs about 1 ms on the wire. `$TRACE` therefore clamps
  any non-zero decimation below 160 to 160, giving at most 100 Hz; a faster
  stream requires a baud-rate change and a new bandwidth decision, not just a
  smaller N. The `TRACE_DROPPED` counter exposes ring overflow.
- `$GET` remains the low-rate health summary with DWT timing (`cyc/max/miss`),
  battery, temperature and diagnostics. `$DUMP` remains the post-fault blackbox.

Runtime changes are diagnostic until HIL sign-off: MOE stays low, so tuning
cannot produce torque during bring-up.

## 5. Rust implementation constraints

- `#![no_std]`, `#![no_main]`, target `thumbv7m-none-eabi`, `panic = "abort"`.
- Embassy or RTIC with static futures/tasks; no heap in the control path. Use
  `static` buffers, `heapless`, fixed arrays, SPSC/watch-style handoff.
- No `unwrap`, `expect`, `panic!`, `format!`, `String` or `Box` in control or
  transport paths. Typed errors and a defined safe state instead.
- No `static mut`. Shared state through `critical-section` or a reviewed
  interior-mutability cell; keep critical sections to a few instructions.
- `unsafe` only in a small, reviewed PAC/HAL module with a `// SAFETY:`
  argument for ownership, alignment and lifetime. Pure computation modules use
  `#![forbid(unsafe_code)]`.
- No `transmute` of wire bytes into structs: parse explicitly with
  `from_le_bytes`/`zerocopy` after alignment review. Golden vectors required.
- Independent IWDG fed only after health checks pass; never from a timer ISR
  unconditionally.
- Keep `jbot_common` free of hardware and async-runtime dependencies. Do not
  change public types in `jbot_common` or crate boundaries without approval.
- The FOC loop owns the highest interrupt priority and runs from a hardware timer
  at 62.5 µs. UART/DMA and housekeeping are lower priority. Measure worst-case
  execution time and stack before claiming the budget is met.
- No C source is linked into the final image. The pinned C tree stays in the repo
  only as a behavioural reference and rollback artifact.

## 6. Suggested module layout

```
firmware/stm32f103/rust/
  Cargo.toml            # no_std, panic=abort, thumbv7m-none-eabi profile
  memory.x              # 256 KB flash / 48 KB RAM
  src/main.rs           # init, self-test, task spawn, watchdog policy
  src/pin.rs            # single authoritative pin map, BOARD_VARIANT 0/1
  src/hal/              # PAC setup: RCC, TIM1/TIM8, ADC1/ADC2+DMA, GPIO, IWDG
  src/foc/              # Clarke/Park, current PI, SVPWM, limits, diagnostics
  src/control/          # mode/state machine, input filter, rate limiter, mixer
  src/protocol/         # frame encode/decode, COBS, CRC16, seq, age, golden tests
  src/uart_control.rs   # USART2 RX/TX, DMA, IDLE, latest-value command
  src/uart_debug.rs     # USART3 console, parameter table, watch, blackbox dump
  src/telemetry.rs      # feedback frame, counters, blackbox ring
  src/safety.rs         # IWDG, fault latch, link-loss safe state, arming
  tests/                # golden vectors, property tests, Host-side unit tests
```

One owner per crate. Add no dependency without checking the workspace first and
getting an explicit decision; versions must be pinned and documented.

## 7. Delivery order (do not big-bang)

1. **Contract freeze**: ADR for the wire format, the selected safe state (§2.4),
   persistence, and approval of the full Rust rewrite; golden byte vectors; host
   tests; Pico-side mirror plan.
2. **Skeleton**: cross-build, pin map, 64 MHz clock, PWM/ADC setup matching the
   reference, MOE gated off, timing marker, watchdog, no motors spinning.
3. **Observability first**: debug UART console and counters working against the
   real board with wheels lifted.
4. **Full Rust FOC core**: port Clarke/Park, current PI, SVPWM, limits and
   diagnostics; validate currents, hall states and duty against the pinned C
   build on the bench before enabling full torque; bring-up ladder, never skip
   hall checks.

   **Status (2026-11-24):** `f103_core::foc_generated` is a literal fixed-point
   port of the unmodified generated controller. `tools/foc_golden/` contains a
   host harness that compiles the C reference and records `foc_golden_left.csv`
   (`z_selPhaCurMeasABC = 0`) and `foc_golden_right.csv` (`= 1`). Two host tests
   replay all 120 ticks of both traces and assert all eight outputs bit-exactly.
   The port is wired into the FOC ISR with `z_ctrlModReq = TRQ_MODE` and the
   pinned `i_max = 8000` / `n_max = 6400`; it stays in OPEN mode until the safety
   controller is armed, and MOE remains low. This proves numerical fidelity,
   **not** hardware correctness: phase order, hall mapping, current sign, duty
   scaling and timing still require bench validation with wheels lifted.
5. **Control link**: compatibility mode on USART2, then the hardened frame with
   both ends upgraded.
6. **Hardening**: fault injection for all three safe states, timeout tests,
   jitter measurement, soak test.

## 8. Acceptance gates

Run and report all of:

```
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo build --release --target thumbv7m-none-eabi -p <firmware-crate>
```

Additional evidence:
- Golden vectors for command and feedback frames; COBS/CRC round-trip;
  corrupted, truncated, oversized, reordered and stale frames never move the
  actuator.
- Property test: arbitrary byte streams never panic or overrun buffers.
- Frame layout check against the pinned source: 8-byte command and 18/22-byte
  feedback with i16 `cmd1`/`cmd2`, plus the mixer sign convention.
- 64 MHz clock verification: PWM ARR = 2000 and the ADC/DMA trigger alignment
  measured on a scope, not inferred from a 72 MHz assumption.
- HIL with wheels lifted: phase order, hall mapping, current sign, current limits,
  MOE gating, each selectable link-loss safe state, watchdog reset, timing markers.
- Measured FOC period/execution time and command-to-current latency with
  p50/p99/p99.9/max; RAM/stack report; one-hour soak with no overrun or reset.
- A rollback path to the pinned C firmware is documented and kept unflashed.

Definition of done: requirement IDs stated, contracts documented, no new
allocation/blocking/`unwrap`/`unsafe` in hot paths, bounded queues with explicit
overflow policy, tests for normal/malformed/fault paths, green quality gates,
measurements attached, documentation in English, no secrets, no `git push`.

## 9. Forbidden

- Claiming a balance result without the release fixture.
- Tuning FOC gains to compensate for drivetrain stick-slip.
- Disabling a current limit, timeout, watchdog or E-stop.
- Inventing a wire layout, pin, unit or timestamp convention locally.
- Flashing, pushing, publishing or deploying without explicit approval.

## 10. Report format for every step

State: requirement ID(s) and scope; files changed; contract/units changed;
verification commands with results; measurements or explicit "not measured";
remaining risks and open questions; what is deliberately not covered.

<!-- PROMPT END -->
