# EFeru motor interface study (2026-11-24)

| Field | Value |
|---|---|
| Status | Read-only study. No firmware, protocol, or control-path code was changed. |
| Source of truth | `firmware/stm32f103/eferu-foc/eferu/` at pinned commit `4f141cbc97b297e194fd58e56444bd0322f902ac` plus `firmware/stm32f103/eferu-foc/eferu-jbot.patch`, exactly the tree used for the flashed `VARIANT_USART` build. |
| Upstream | `EFeru/hoverboard-firmware-hack-FOC`, C, GPL-3.0, 16 kHz FOC core generated from a Simulink model. |
| Related evidence | `docs/latency-and-slew-2026-11-23.md`, `docs/drive-diagnosis-2026-11-23.md`, `docs/audit-2026-11-22.md`, `firmware/README.md`. |

## 1. What is actually running

| Layer | Current value | Where |
|---|---|---|
| Build variant | `VARIANT_USART`, `-D VARIANT_USART` in `platformio.ini` | `platformio.ini:48` |
| Control type | `FOC_CTRL` (2) | `config.h:157` |
| Control mode | `TRQ_MODE` (3); `±1000` command units map to `±I_MOT_MAX` | `config.h:158`, `eferu-jbot.patch` |
| Current limits | `I_MOT_MAX=10 A`, `I_DC_MAX=12 A`, `N_MOT_MAX=400 rpm` | `config.h:164-166` after patch |
| Input filter | `FILTER=19661` (alpha 0.3, tau about 14 ms at 200 Hz); default is `6553` (alpha 0.1, tau about 47 ms) | `config.h:356`, `config.h:200` |
| Mixer coefficients | `SPEED_COEFFICIENT=16384` (1.0), `STEER_COEFFICIENT=8192` (0.5) — the `VARIANT_USART` block does **not** override them | `config.h:201-202`, `config.h:688-700` |
| Input rate limiter | `RATE=480` fixdt, about 6000 command units/s — does not limit our steps | `config.h:199`, `util.c:1690` |
| Serial timeout | `SERIAL_TIMEOUT=160` ticks = 160 x 5 ms = **800 ms** | `config.h:670` |
| UART | USART2 (left cable): control RX + feedback TX; USART3 (right cable): debug text TX; both 115200 8N1 | `config.h:329-330`, `config.h:361`, `config.h:674`, `config.h:680` |
| Text parameter protocol | `DEBUG_SERIAL_PROTOCOL` **disabled** (commented out); plain ASCII print every 125 ms instead | `config.h:263`, `main.c:502-518` |
| Odometry in feedback | `ENABLE_ODOMETRY` **disabled**: 18-byte feedback frame, not 22 | `main.c:124-140` |
| PWM / dead time | `PWM_FREQ=16000` Hz, `DEAD_TIME=48` | `config.h:28-29` |
| Current measurement scale | `A2BIT_CONV=50` (1 A = 50 ADC bit units) | `config.h:36` |

## 2. Inventory of parameters relevant to motor communication

### 2.1 Wire protocol (Pico <-> STM32 on USART2)

| Parameter | Value | Effect |
|---|---|---|
| `SERIAL_START_FRAME` | `0xABCD` | Command and feedback frame magic. |
| Command frame | 8 bytes: `<HhhH` = start, steer, speed, XOR checksum | `util.c:1268-1313`. No sequence number, no timestamp, no version, no length field. |
| Frame acceptance | DMA idle-line interrupt; **exactly** 8 bytes since the previous idle gap | `usart2_rx_check`, `util.c:1091-1126`. Sending back-to-back frames without a gap can be dropped. |
| Checksum | XOR of start, steer, speed | Weak integrity check: a two-field error that cancels in XOR passes. No evidence of corruption on the bench. |
| Feedback frame | 18 bytes at 100 Hz (every second 5 ms main tick), DMA TX | `main.c:520-559`. Fields: start, cmd1 echo, cmd2 echo, speedR, speedL, batVoltage (x100), boardTemp (x10 deg C), cmdLed, XOR checksum. |
| `ENABLE_ODOMETRY` | off; if on, +4 bytes `wheelR_cnt`, `wheelL_cnt` -> 22 bytes | `main.c:131-134`. Gives wheel position counters instead of integrating RPM on the Pico. |
| `SERIAL_TIMEOUT` | 160 ticks = 800 ms; every valid frame resets it | `util.c:960-981`, `util.c:1295-1309`. On expiry `ctrlModReq = OPEN_MODE` (freewheel), `util.c:1032-1033`. |
| `USART2_BAUD` / `USART3_BAUD` | 115200, overridable with `-D` build flags | 8 B @ 200 Hz = 1600 B/s, about 1.4% of link capacity. Raising baud does not remove the dominant lag. |
| `SERIAL_BUFFER_SIZE` | 64 bytes | DMA RX buffer; must stay larger than the largest expected frame. |

### 2.2 Command chain and mixer (what the Pico request becomes)

| Parameter | Value | Effect |
|---|---|---|
| `CTRL_MOD_REQ` | `TRQ_MODE` | Torque request, `±1000` -> `±I_MOT_MAX`. `VLT_MODE` stalls at low command (cogging); `SPD_MODE` is the alternative with FOC. |
| `CTRL_TYP_SEL` | `FOC_CTRL` | `COM_CTRL` (0), `SIN_CTRL` (1), `FOC_CTRL` (2). |
| `FILTER` | 19661 (alpha 0.3) | First-order filter on the input command. The single largest protocol-side lag; reducing 6553 -> 19661 cut measured command-to-wheel dead time from 180-190 ms to 115-145 ms. |
| `RATE` | 480 | Rate limiter on the input command, about 6000 u/s; not the current bottleneck. |
| `SPEED_COEFFICIENT` | 16384 (1.0), fixdt(0,16,14) | Common-mode gain in `mixerFcn`. |
| `STEER_COEFFICIENT` | 8192 (0.5) | Differential gain. `jbot_pico_core::eferu::Command::from_wheel_torque` assumes 1.0, so yaw authority is silently halved once yaw is enabled. |
| `INVERT_L_DIRECTION` / `INVERT_R_DIRECTION` | undefined | Sign of `pwml`/`pwmr`; the Pico mixer inverse assumes this. |
| `TANK_STEERING` | undefined | If defined, bypasses the mixer and maps steer/speed straight to wheels. |
| `I_MOT_MAX`, `I_DC_MAX`, `N_MOT_MAX` | 10 A, 12 A, 400 rpm | Final torque, DC-link, and speed limits. Keep `I_DC_MAX = I_MOT_MAX + 2 A`. |
| `FIELD_WEAK_ENA`, `FIELD_WEAK_HI/LO/MAX` | 0, 1000, 750, 5 A | Only relevant above the base speed; not useful at 400 rpm. |
| `PHASE_ADV_MAX` | 25 deg | Only for `SIN_CTRL`. |

### 2.3 FOC inner loop (generated C, not exposed by config.h)

The file `Inc/BLDC_controller.h` and `Src/BLDC_controller.c` are MATLAB/Simulink generated fixed-point code. The `P` parameter structure contains the PI gains, current filters, angle estimation, motor-limit saturation, and SVPWM constants; `i_max`, `n_max`, `z_ctrlTypSel` are written from `config.h` at startup. These gains are **not** runtime-settable through the stock parameter console, and there is no Simulink model in this repository, so retuning them means editing generated C without a model to regenerate from. `DIAG_ENA=1` enables hall/phase diagnostics and the `z_errCode` used by the enable interlock in `main.c:270`.

### 2.4 Runtime parameter console (undocumented in local docs)

`Src/comms.c` implements a text command protocol, enabled only with `DEBUG_SERIAL_PROTOCOL` plus one `DEBUG_SERIAL_USARTx`:

- Commands: `$GET`, `$HELP`, `$WATCH`, `$SET`, `$INIT`, `$SAVE` (`comms.c:76-84`). Processed at 8 Hz by `process_debug()` (`main.c:502-506`).
- Tunable parameters: `CTRL_MOD`, `CTRL_TYP`, `I_MOT_MAX`, `N_MOT_MAX`, `FI_WEAK_ENA`, `FI_WEAK_HI/LO/MAX`, `PHA_ADV_MAX`.
- Read-only variables: `DC_CURR`, `RDC_CURR`, `LDC_CURR`, `CMDL`, `CMDR`, `SPD_AVG`, `SPDL`, `SPDR`, `RATE`, `SPD_COEF`, `STR_COEF`, `BATV`, `TEMP` (`comms.c:87-146`).
- Only parameters whose `addr` is nonzero persist: `I_MOT_MAX` (addr 1) and `N_MOT_MAX` (addr 2); `CTRL_MOD`, `CTRL_TYP` and field-weakening entries have addr 0 and are RAM-only. `$SAVE` writes to emulated EEPROM; the VARIANT_USART `FLASH_WRITE_KEY` is `0x1002` (`config.h:346`), so changing that key invalidates stored input calibration.
- Hard constraint: `DEBUG_SERIAL_USART2` and `FEEDBACK_SERIAL_USART2` cannot coexist (`config.h:726-727`); same for USART3. The console therefore cannot share the control cable. On this robot the right cable (USART3) is free, so the console can be enabled there without touching the control link.
- Enabling it replaces the periodic `in1:... cmdL:...` print with the console and watched-parameter output.

This is the cheapest way to change motor-facing parameters without reflashing, and it makes `I_MOT_MAX`, `N_MOT_MAX`, and the control mode observable from the bench.

### 2.5 Safety and validation parameters

| Parameter | Value | Relevance |
|---|---|---|
| `TIMEOUT` | 20 | Generic input timeout counter; not the serial timeout. |
| `STARTUP_DELAY_MS` | 500 | Power latch delay; prevents spurious power-on from UART back-powering. |
| `BAT_*`, `TEMP_*` | calibration set | Battery undervoltage and temperature poweroff are **disabled** (`BAT_DEAD_ENABLE=1`, `TEMP_POWEROFF_ENABLE=0`); do not enable temperature limits without calibration. |
| `ADC_PROTECT_TIMEOUT/THRESH` | 100 / 200 | ADC input protection; not used by the serial link. |
| `DIAG_ENA` | 1 | Motor diagnostic errors gate motor enable. |

## 3. Discrepancies found between docs and source

| Claim | Actual source | Consequence |
|---|---|---|
| `docs/pico-bridge.md` and `firmware/README.md` say `SERIAL_TIMEOUT` was reduced to 20 (100 ms) | `config.h:670` still has 160 (800 ms); `eferu-balance.patch` does not exist | A stale torque request can drive the motors for 800 ms after the link dies. This is a safety item, not a tuning item. |
| Pico torque mixer assumes `STEER_COEFFICIENT=1.0` | `VARIANT_USART` leaves the default 0.5 | Yaw differential is half of the requested value. |
| `firmware/README.md` calls the bench build "upstream stock config" | The local `VARIANT_USART` block already carries `FILTER=19661` and `DEBUG_SERIAL_USART3` from the jbot patch | Documentation understates what is running. |
| `docs/latency-and-slew-2026-11-23.md` attributes ~200 ms mostly to the mainboard filter | Later source-level pass re-scoped the mainboard contribution to about 20-30 ms; the rest is Pico slew ramp, dead zone, and wheel mechanics | Do not expect a reflash of the mainboard to fix balance. |

## 4. Ranked options to improve communication with the motors

Ordered by evidence and risk, not by effort. None of these were applied.

1. **Fix the two known config mismatches** — set `SERIAL_TIMEOUT=20` (100 ms) and `STEER_COEFFICIENT=16384` in the `VARIANT_USART` block, with a 200 Hz command stream verified on the bench. This closes the stale-command window and the yaw-authority loss. Low risk; no new protocol. It is a safety/protocol change and needs Architect + Safety review before flashing.
2. **Enable the existing `$GET/$SET/$WATCH/$SAVE` console on USART3** — live tuning of `I_MOT_MAX`, `N_MOT_MAX`, `CTRL_MOD` and read-only `SPDL/SPDR/CMDL/CMDR/DC_CURR/TEMP`, with EEPROM persistence for the current limits. No change to the control link; the right cable must stay dedicated to debug. Low risk, high diagnostic value.
3. **Add `ENABLE_ODOMETRY` (22-byte feedback)** — wheel counters would remove RPM integration drift on the Pico. This changes the wire format and the Pico parser (`jbot_pico_core::eferu::Feedback`), so it is a protocol contract change with golden-vector and loopback tests, not a config-only edit. Medium risk.
4. **Harden the command frame** (CRC16, sequence, age, version, length) — the current XOR checksum and idle-gap length check have no stale-replay protection inside the 800 ms window. Worth doing only if option 1 is rejected or if corruption is observed; it requires changing both ends and the shared contract. High process cost.
5. **Raise UART baud** to 921600 or 1 Mbaud — marginal: the 8-byte command at 200 Hz is 1600 B/s and the measured mainboard contribution to delay is 20-30 ms. Only useful to speed up text/telemetry dumps. Not recommended as a latency fix.
6. **Touch FOC PI gains or field weakening** — no model, no measured gain deficiency, and the generated C would need re-validation from scratch. The bench evidence points at mechanical stick-slip (dead zone about 34-69 command units) instead. Do not tune here to compensate for the drivetrain.

Evidence for the ranking: `docs/drive-diagnosis-2026-11-23.md` verified that `in2` -> `cmdL/cmdR` matches the request exactly at 40/80/120 units, so the command path is not the cause of the balance failures. The blocker is drivetrain stick-slip plus lag, so protocol work should be judged on observability and safety, not on whether it fixes balance.

## 5. Rust firmware options

The project already runs Rust on the Pico 2 W; the F103 still runs generated C. Three realistic scopes:

| Option | Scope | Assessment |
|---|---|---|
| **A. Keep the F103 C FOC core** (recommended baseline) | Do all protocol, telemetry, tuning, and control work in `firmware/pico2w/fw` and `firmware/pico2w/core`, which are already `no_std`, host-tested, and own the wire contract. Change the F103 only through the pinned patch for the two config fixes. | Lowest risk. Preserves the validated 16 kHz current loop. Matches `firmware/README.md` section 5.1. Does not require repatriating a Simulink model. |
| **B. Rust protocol shell around the C FOC core** | Reimplement framing, CRC, sequence, timeouts, and the debug console on the F103 in `no_std` Rust (Embassy or RTIC), calling the generated controller through a small reviewed FFI boundary. | Medium risk. Gives typed validation and latest-value command semantics; keeps the proven current loop. Still needs a hardware-validated FFI/HAL layer, bounded queues, and a full bench regression. `unsafe` policy and stop-and-escalate triggers apply. |
| **C. Full Rust FOC rewrite on the STM32F103** | Replace the generated C controller with Rust: ADC/DMA phase currents, hall capture, Clarke/Park, current PI, SVPWM, 16 kHz scheduling, limits, diagnostics. | High risk, not justified by current evidence. The F103 has no FPU, so fixed-point discipline is mandatory (`docs/skills/rust_robotics_guidelines.md` sections 2.3, 2.6, 6.2). The generated C has no corresponding model in this repo, so correctness would have to be re-derived and re-validated on hardware. It also reverses the explicit architecture decision in `firmware/README.md` section 5.1. Requires Architect and Safety approval before any code. |

Additional constraints for any port:

- Upstream is **GPL-3.0**; a derived Rust implementation that is distributed stays under GPL-3.0. Do not copy generated C comments or structures into the workspace without that license obligation being explicit.
- The wire contract is shared with `jbot_pico_core`; one change must be mirrored with golden vectors and host tests, per `docs/skills/rust_robotics_guidelines.md` sections 3 and 7.
- Quality gates for any change: `cargo fmt --all -- --check`, `cargo check --workspace --all-targets`, `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`, plus `cargo test` in `firmware/pico2w/core` and a cross-build if firmware is touched.

## 6. Decisions needed before implementation

1. Which outcome is meant by "improve communication with the motors": lower latency, richer feedback, reliable commands, or live parameter tuning? The evidence above says latency is already close to the achievable floor on this hardware.
2. Is the current safety envelope acceptable for a 100 ms serial timeout with freewheel on expiry, or is a controlled brake state required instead?
3. Should the right cable be permanently assigned to the parameter console, and is the operator willing to use `$SET/$SAVE` instead of reflashing?
4. Is option B or C in scope at all, given the architecture decision to keep the F103 C core? If yes, an Architect/Safety review and a bench acceptance plan are prerequisites.

## 7. Wiki cross-check and the actual `SERIAL_TIMEOUT` (follow-up, 2026-11-24)

All 29 wiki pages were read (temporary clone removed afterwards; no repository
changes beyond this document). The wiki confirms most of section 2 and adds
several details that matter for this robot.

### 7.1 What the wiki adds

- **Parameters** (`Parameters.md`): `RATE=480` advances the input by 30 units
  per 5 ms tick, about 6000 units/s. `RATE > 32767` becomes a negative rate.
  A `STEER_COEFFICIENT > 32767` becomes negative. `TANK_STEERING` ignores both
  coefficients but still applies `RATE` and `FILTER`.
- **Current and speed limits** (`Setup-current-and-speed-limits.md`):
  `I_MOT_MAX` is the phase-current limit and applies only to FOC; `I_DC_MAX` is
  the DC-link limit for commutation/sinusoidal. Keep `I_DC_MAX = I_MOT_MAX + 2 A`.
  Limits set "on the go" or through `$SET` are stored in flash and then
  **override `config.h` on every boot**; only a full chip erase or changing
  `FLASH_WRITE_KEY` (currently `0x1002` for `VARIANT_USART`) bypasses them.
  Hard firmware ceiling is 2000 rpm. FOC limits PWM duty to about 90 % to
  sample current, so it is slower than sinusoidal.
- **Debug protocol** (`Debug-Serial.md`): independently confirms the
  `$GET/$HELP/$WATCH/$SET/$INIT/$SAVE` table from section 2.4, `$` prefix and
  newline terminator, and the 15-value `WATCH` limit. The wiki labels `CMDL` and
  `CMDR` as "Motor Command RPM", while the source calls them "Left/Right Motor
  Command"; in this build they are mixer outputs in command units (-1000..1000),
  not RPM. Trust the source.
- **USART variant** (`Variant-USART.md`): 115200 8N1, little-endian, 8-byte
  command, 18-byte feedback every 10 ms, idle-line detection, and rejection of
  wrong length/start/checksum. Only USART3 (right cable) is 5 V tolerant.
  This page states "SERIAL_TIMEOUT ... (160ms by default)".
- **Diagnostics** (`Diagnostics.md`): **three low-pitch beeps = serial
  timeout**; one low beep = hall/motor error, two = ADC timeout. Motors are
  enabled only when both inputs are between -50 and 50, signalled by a double
  beep after the startup melody.
- **Battery** (`Battery.md`): confirms the regenerative-overvoltage risk that
  can trip the BMS with a full pack, about 8 minutes inactivity poweroff, and
  the `BAT_CALIB_REAL_VOLTAGE`/`BAT_CALIB_ADC` calibration procedure.
- **Input calibration** (`Input-Calibration.md`): input limits and
  current/speed limits survive a reflash in flash memory; auto-calibration can
  be triggered accidentally by holding the button, so do not substitute a
  jumper for the button. Our serial variant does not need input calibration.
- **Firmware compatibility** (`Firmware-Compatibility.md`): recommended bring-up
  ladder is SIN control -> FOC voltage mode -> FOC speed mode, to isolate phase
  wiring, hall mapping, current sensing, and speed sign. Cogging or
  full-speed-in-speed-mode points to phases/halls, not to the command path.
- **Variants** (`Variants.md`): stock `VARIANT_USART` is a VOLTAGE-mode variant;
  our patch to `TRQ_MODE` is deliberate, not upstream default.
- **Sideboards** (`Sideboards.md`): they do not control motors; upstream
  balancing is explicitly "not implemented yet", which supports keeping the
  balance loop on the Pico.

### 7.2 Determining `SERIAL_TIMEOUT` definitively

The wiki and the source disagree:

| Source | Claim |
|---|---|
| `Variant-USART.md` | "160ms by default" |
| `config.h:670` comment | `160 ~= 0.8 sec. Calculation: 0.8 sec / 0.005 sec` |

The code wins. `handleTimeout()` is called once per main loop from
`readCommand()` (`util.c:1065`), and `VARIANT_USART` uses
`DELAY_IN_MAIN_LOOP = 5` ms (`config.h:33`; the 2 ms variant applies only to
`VARIANT_TRANSPOTTER`). The 5 ms period is independently confirmed by the debug
print at `main_loop_counter % 25 == 0` (125 ms) and feedback at `% 2 == 0`
(10 ms). Therefore:

**`SERIAL_TIMEOUT = 160` ticks x 5 ms = 800 ms, not 160 ms.** The wiki figure
is a documentation error, likely from assuming a 1 ms tick.

Evidence from the local compiled artifacts:

- `build/util.lst:2630`: `movs r2, #160` inside `handleTimeout()` -> the last
  local build compiled `SERIAL_TIMEOUT=160`.
- `build/util.lst:4453` and `:4467`: `movw r1, #19661` in the
  `filtLowPass32()` calls -> the same build has the patched `FILTER=19661`.
- `build/hover.bin` is 32,316 bytes, MD5 `f82f379520c7702fd6c116b4f6992cd4`
  (built 2026-11-23). `firmware/README.md` records a different bench binary,
  32,312 bytes, MD5 `64ab7b58ceea8ba4a7c1f5ea3d6914d0`. Two builds exist, but
  both come from trees before the never-created `eferu-balance.patch`, so both
  carry `SERIAL_TIMEOUT=160`.

Practical conclusion: **assume 800 ms until proven otherwise on the board.**
Two ways to confirm the flashed value:

1. Read the MCU flash with SWD (`st-flash read /tmp/mainboard.bin 0x8000000
   0x8000` or OpenOCD) and compare the `handleTimeout` immediate against a
   locally rebuilt binary; this is definitive.
2. Behavioural check: with no valid frames on USART2 (e.g. Pico unplugged),
   time from the last command to the first three low-pitch beeps. The board
   emits them whenever `timeoutFlgSerial` is set (`main.c:581-582`); 800 ms vs
   100 ms is easy to distinguish. Keep the wheels lifted.

To change it, edit `SERIAL_TIMEOUT` in `config.h:670` from 160 to 20 (100 ms),
rebuild, and reflash. It is a plain `#define` inside a conditional block, so a
`-D` build flag alone is not sufficient. Because expiry falls back to
`OPEN_MODE` freewheel (`util.c:1032-1033`), the 100 ms choice needs the normal
200 Hz command stream and a safety review before it is accepted.

## 8. Executive conclusion: problems with this firmware on our mainboard and motors

### 8.1 Problems actually encountered

| Problem | Evidence | Impact |
|---|---|---|
| No balance controller on the mainboard | `VARIANT_HOVERBOARD` is marked "Balancing controller: TODO" both in `config.h:590` and in the wiki (`Variants.md`, `Sideboards.md`) | The F103 is only a torque actuator; all posture authority lives on the Pico. A mainboard-only solution does not exist. |
| No stable, repeatable self-balance has been demonstrated | `docs/audit-2026-11-22.md`, `docs/latency-and-slew-2026-11-23.md`, `docs/drive-diagnosis-2026-11-23.md` | Every 10 s hold was supported by rope/hand or failed after release; the single "PASS" was re-scored as no wheel authority. |
| Drivetrain stick-slip / dead zone, not a firmware setting | Breakaway threshold 34-69 command units, asymmetric per wheel (left about 30-35, right about 40-50, travels with the motor); dither and dead-zone inverse made it worse | Below breakaway the law cannot correct; above it the response pumps. This is the dominant blocker. |
| Command-to-wheel lag was initially blamed on the mainboard filter | `FILTER` 6553 -> 19661 cut dead time from 180-190 ms to 115-145 ms; later source pass puts the mainboard at only 20-30 ms | The remaining lag is Pico slew ramp, dead zone, FOC and wheel mechanics. Further mainboard tuning has little payoff. |
| Config did not match the documented/programmed intent | `SERIAL_TIMEOUT=160` (800 ms) vs 100 ms in `firmware/README.md` and `docs/pico-bridge.md`; wiki says 160 ms; `STEER_COEFFICIENT=0.5` vs Pico inverse assuming 1.0 | Up to 800 ms of stale torque after link loss; yaw differential silently halved. |
| EEPROM/flash overrides `config.h` silently | Wiki `Setup-current-and-speed-limits.md` and `Input-Calibration.md`; `FLASH_WRITE_KEY=0x1002` | A rebuild can appear to have no effect. Full erase or a key change is required to know which limits are active. |
| Weak wire protocol | XOR checksum, no sequence, timestamp, version or length field; 8-byte frame accepted only with an exact idle gap; feedback echoes inputs, not applied torque or current | Silent stale/corrupt frames possible inside the timeout window; no way to prove what the motor actually received. |
| Freewheel on link loss | `util.c:1032-1033` requests `OPEN_MODE` when `timeoutFlgSerial` is set | For a balancing machine, losing the link means falling; there is no controlled stop state on the mainboard. |
| Poor observability | Feedback has RPM, battery, temperature and input echo only; `DEBUG_SERIAL_PROTOCOL` off; debug UART is mutually exclusive with feedback on the same cable; no mainboard blackbox | Failures must be inferred from Pico telemetry; hall state, error codes, DC current, duty and actual current are not on the wire. |
| FOC core is generated C with no model in this repository | `Inc/BLDC_controller.h`, `Src/BLDC_controller.c`, `README.md:92-96` | PI gains are not runtime-tunable; any change needs a full re-validation, and a Rust port would have to re-derive the controller. GPL-3.0 applies to derived work. |
| Safety inputs are not wired into a real E-stop | Charge-detect line is initialised but never read (`Firmware-Compatibility.md:63`, `firmware/README.md` section 1); power latch is held by firmware | Arming while charging is not prevented by the mainboard; the battery-line kill switch is the only real safety path. |
| Build/flash friction | Nested git-ignored upstream clone plus patches, ST-Link clone unlock workaround, board must run on its own battery while flashing, 15 V wire on the cable, sideboards contend on the same wires | Easy to flash the wrong configuration or damage hardware; sideboards must stay disconnected. |

### 8.2 What is missing

1. **A repeatable release fixture** - the largest gap. Without it no gain, filter or calibration change can be judged; hand release injects 4-212 dps at the cue. See `docs/release-fixture.md`.
2. **A loaded torque/dead-zone measurement and its use in the control path** - stand measurements exist; the floor-loaded value and its temperature/voltage dependence do not. The `cal` inverse on the Pico is unvalidated in closed loop.
3. **A trustworthy command contract** - CRC instead of XOR, sequence, age/timestamp, version, length, and feedback of the applied torque/current plus error codes. This is a shared `jbot_pico_core` contract change with golden vectors, not a local patch.
4. **Runtime parameter access and a clear source of truth for limits** - enable the existing `$GET/$SET/$WATCH/$SAVE` on USART3, and document that `$SAVE` beats `config.h` until full erase or `FLASH_WRITE_KEY` change.
5. **A safe link-loss state on the mainboard** - a bounded ramp to zero or a brake state instead of immediate `OPEN_MODE`, matched to the robot's 100 ms command budget.
6. **Bench instrumentation** - DC-link/phase current and MOSFET temperature under load, plus a scope or logic analyser for command-to-current latency and 200 Hz jitter. No such measurement exists yet.
7. **Consistent documentation** - three conflicting timeout figures and one mixer mismatch should be corrected in the project docs, not just in code.
8. **A bounded Rust path** - keep the generated C FOC core; move protocol, validation and telemetry into `no_std` Rust (`pico_core` already does this). A full Rust FOC rewrite needs a recovered controller model, a reviewed HAL/FFI layer, fixed-point discipline and HIL validation before it is even a candidate.

### 8.3 Bottom line

The firmware is a good **motor drive** and a poor **robot control interface**. The
command path is verified correct end to end; the failures come from the
mechanical drivetrain, a weak and poorly documented protocol, and missing
measurement infrastructure. Fix the config mismatches and enable runtime
tuning first; treat protocol hardening and a Rust rewrite as separate,
safety-reviewed projects; and do not claim any balance result until the release
fixture exists.

Task completed at the study level. Run `/compact` or start a new session to clear context before implementation work.
