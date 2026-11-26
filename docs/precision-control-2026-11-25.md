# Precision drive control toolchain (2026-11-25)

## Scope

Requirement IDs: `SYS-02`, `SYS-07`, `INT-01`–`INT-10`, `M3-*`, `SAF-*`.
Affected crates/targets: `jbot_pico_core`, `pico-bridge` (Pico 2 W), the pinned
EFeru C build on the STM32F103. No hardware was flashed or run as part of this
change.

The goal is to make the drive observable and its torque scale correctable at
runtime, so that the balance loop can be tuned from measured data instead of
from a hall-speed proxy.

## What changed

### Mainboard feedback contract (wire layout unchanged, semantics changed)

The 18-byte feedback frame keeps its length, checksum and word positions.
Words 1 and 2 used to echo the parsed `steer`/`speed` inputs; they now carry
the per-wheel commands the F103 **actually applies**:

| Word | Field | Meaning |
|---|---|---|
| 0 | `start` | `0xABCD` |
| 1 | `appliedL` | left wheel command after rate limit, filter and mixer, `-1000..=1000` |
| 2 | `appliedR` | right wheel command after the same chain, `-1000..=1000` |
| 3–8 | speed, battery, temperature, LED, checksum | unchanged |

The fields are zero while the drive is disabled. The old input echo was taken
before the rate limiter and the filter, so the Pico could not see the drive's
real state. Both firmwares must be flashed together: an old Pico decoding a new
mainboard (or the reverse) will misread words 1 and 2. The checksum still covers
all words, so a mixed pair is likely to fail validation rather than silently
apply a wrong value.

### Runtime knobs on the STM32F103 (debug USART3, text protocol)

| Name | Type | Range | Effect |
|---|---|---|---|
| `ANG_MEAS` | parameter | `0..=1` | 0 = hall-speed-integrated angle estimator, 1 = measured hall angle |
| `FILT` | variable | `0..=65535` | input low-pass coefficient in `fixdt(0,16,16)`; `65535` disables the filter |

`GET ANG_MEAS`, `SET ANG_MEAS 1`, `GET FILT`, `SET FILT 65535`, `INIT <index>`
restores the compiled default. Both are runtime-only (no EEPROM address, so a
bench value does not survive power-up); `INIT` restores `ANG_MEAS` from
`ANGLE_MEAS_ENA` and `FILT` from `FILTER`.

### Runtime torque budget on the Pico

`tmax <Nm>` sets the common-mode torque budget used to convert the control law
into EFeru command units. It rebuilds `Tuning` from the plant with the new
budget when the value changes; the change takes effect on the next 200 Hz tick.
`tmax off` restores the compiled `PlantParams::NOMINAL` value (5.0 Nm).
Range: `1.0..=30.0` Nm. Runtime-only: it resets at boot.

This is the single knob that corrects a wrong torque-constant assumption:
`units_per_nm = 1000 / max_torque_nm`. All Nm gains stay in Nm; the torque
limit, yaw gain and lift/fall guards follow the budget. Re-check the guards
after changing it.

### Telemetry

`stat` adds `tmax`, `al`, `ar`. Each telemetry line adds `al` and `ar`, the
per-wheel command reported by the mainboard. Together with `cwl`/`cwr`
(requested) and `L`/`R` (hall rpm), this is enough to measure the command path
without an external instrument.

## Bench procedure

Do all of this with the robot supported and the balance law disarmed.

1. **Confirm the link.** Flash both firmwares. `stat` should show `fb_ok=1`
   and `al`/`ar` following `test` commands after the mainboard's filter delay.
2. **Measure the torque constant Kt.** Lock one wheel, apply a constant
   `test <units>` (1000 units = 10 A on this build), and measure the reaction
   force `F` on a lever arm `L`: `Kt = F·L / I`. Repeat both wheels, both signs.
   Set `tmax` to `2 · Kt · 10 A` (both wheels, full 10 A each).
3. **Measure the dead zone.** With `stream 100`, ramp `test` in 5-unit steps
   and note the first `al`/`ar` that produces wheel motion. The measured
   breakaway should replace `deadband_nm = breakaway_units / units_per_nm`.
4. **A/B the angle source.** `SET ANG_MEAS 1` on the debug port, repeat step 3
   and compare the breakaway and the low-speed torque quality. Keep the value
   that gives the lower, more repeatable breakaway.
5. **A/B the input filter.** `SET FILT 65535`, repeat a 1-degree reference
   step and compare settling. The Pico already slews at 3000 units/s, so the
   mainboard filter is likely redundant.
6. **Re-derive the gains.** With the corrected `tmax`, sweep `pscale`
   0.6 / 0.8 / 1.0 and `dscale` 1.0 / 1.4 / 1.8. The previous ladder stopped
   at `pscale 0.25`; if the torque budget was the cause, the full-gain point
   should now be usable.
7. **Delay compensation.** Keep `predict off` or at most 40 ms. The earlier
   115–190 ms figure came from hall-speed measurement at low rpm, not from the
   actuator. Re-measure the delay with the applied-command telemetry before
   raising it.

## Applied-command check (2026-11-25)

The Pico CDC console stopped replying after a host session closed the port;
physically re-enumerating USB restored it without a reflash. The symptom
matches the historical tooling note in `docs/wp0-torque-identification.md`.

With the balance law disarmed and the wheels stationary, the console sequence
was `estop`, `stream 100`, `clear`, `test 5 5`, `test 0 0`, `estop`. The `test`
command takes two numeric arguments; its reply is formatted as
`ok test l=<left> r=<right>`. `cwl`/`cwr` mirror the request immediately, while
`al`/`ar` come from the mainboard feedback words:

| Request | First matching `al`/`ar` | Observation |
|---|---|---|
| `test 10 10` | 114 ms after the command | `al=10 ar=10`, `L=R=0` |
| `test 0 0` | 57 ms after the command | `al=0 ar=0` |

Telemetry arrived at about 90 lines/s, so the resolution is roughly 10 ms.
The rise figure includes the 200 Hz Pico command interval, the mainboard input
filter and the feedback frame interval; it bounds the command-reporting path
and is not a measurement of actuator transport delay. Both wheels stayed at
zero rpm at 5 and 10 units, so no torque constant or breakaway value was
derived. The bench was returned to `estop=1`, `armed=0`, `L=R=0`, `al=ar=0`.

## Dead-zone probe (2026-11-25)

A 5-unit ramp from 5 to 60 units was applied with a 400 ms hold and an explicit
zero between steps. First motion appeared at `test 20 20`: the left wheel
reached 16 rpm while the right wheel stayed at zero. A confirmation pass over
16, 18, 20, 22 and -20 units was not repeatable: 20 units produced no motion
in that pass, while -20 turned the left wheel to -17 rpm. The wheel also kept
reporting -16 rpm briefly after the command returned to zero, which matches
the known hall-speed measurement lag.

This confirms the earlier WP0 note that a hand-held/bench rig is not a valid
fixture: neither the breakaway threshold nor its sign symmetry is repeatable.
No `db` value was changed; Kt and dead-zone compensation still require a
locked wheel, a known lever arm and a force measurement.

## Torque-scale identification without external tools

The primary method is spin-up: with the body held and the wheels free, apply a
short `test N 0` command and take the least-squares slope of the left-wheel rpm
from the 200 Hz telemetry. With the per-wheel inertia `0.0078 kg m^2` from
`config/robot.toml`, `M = I · alpha` and `units_per_nm = N / M`. The tool is
`archive/python-tools/pico/one_wheel_spin_id.py`; it drives one wheel only,
never arms the balance
law, zeroes the command and latches `estop` at the end.

Historical spin-up data in `docs/wp0-torque-identification.md` gives 170–186
units/Nm from 99/120/150-unit steps, i.e. close to the compiled 200 units/Nm
and a full-scale budget near 5.7 Nm. That does **not** support the 3.2x error
hypothesis behind `tmax = 16`.

The measurement was repeated on the current firmware with the body held and
the left wheel free:

| Command | alpha, rad/s² | M, Nm | units/Nm | Reading |
|---:|---:|---:|---:|---|
| 40 | 14.7 | 0.114 | 350 | Inflated by breakaway friction |
| 60 | 31.9 | 0.249 | 241 | Usable |
| 80 | 48.9 | 0.381 | 210 | Usable |
| 100 | 34.0 | 0.265 | 377 | Invalid: torque-speed envelope |

The 100-unit step is discarded because acceleration fell below the 80-unit
step. The asymptotic 60/80-unit points give 210–241 units/Nm, consistent with
the compiled 200 units/Nm and the earlier 170–186 measurement. No `tmax` change
is warranted; the 3.2x hypothesis remains unsupported.

A hanging mass is an optional cross-check when a marked container is available:

1. Measure the wheel radius `r` with a tape measure. The 6.5 in tire is about
   0.0825 m; measure the diameter instead if the tire is not at nominal size.
2. Weigh a mass `m` on a kitchen scale. One litre of water is 1 kg.
3. Support the robot so one wheel hangs free. Wind a string around the tire and
   hang the mass. Choose the command sign that opposes the fall.
4. With `clear`, increase `test N N` in small steps until the mass neither
   falls nor rises, then record `N`. Repeat for both wheels and both winding
   directions. Re-latch `estop` after every hold and never arm the balance law.
5. The load torque is `m · g · r`, so `units_per_nm = N / (m · g · r)` and the
   new budget is `tmax = 1000 / units_per_nm`.
   `archive/python-tools/pico/kt_calc.py` does the arithmetic.

A 1 kg mass on a 0.0825 m wheel gives 0.81 Nm. If the current 200 units/Nm is
correct, the holding command is about 162 units; if the 3.2x torque hypothesis
is correct, it is about 50 units. Friction and hall-speed lag make a single
balance point noisy, so repeat with 0.5, 1 and 2 kg and use a line fit if time
allows.

## F103 debug bridge (2026-11-25)

`dbg on` forwards the mainboard's periodic USART3 telemetry, but the console
parser forwards only the first whitespace token of a `dbg` line: `dbg $GET
ANG_MEAS` sends just `$GET`, confirmed by `ok dbg tx 4`. No parameter reply
appeared within 2 s. The `ANG_MEAS`/`FILT` A/B therefore could not be performed
without changing the console forwarding and confirming the GP13-to-PB11 wiring.
No F103 parameter was changed and `dbg` was returned to `off`.

## Verification performed

- `cargo fmt --all -- --check`, `cargo check --workspace --all-targets`,
  `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`
  in the root workspace: pass.
- Same four gates in `firmware/pico2w/core`: pass (137 tests).
- `cargo fmt --all -- --check`, `cargo check --workspace --target thumbv8m.main-none-eabihf`,
  `cargo clippy --workspace --target thumbv8m.main-none-eabihf -- -D warnings`,
  `cargo build --release --target thumbv8m.main-none-eabihf` in
  `firmware/pico2w/fw`: pass.
- `make -e VARIANT=VARIANT_USART BUILD_DIR=build/precision -j4` in `eferu`:
  pass with no warnings. The new image is `eferu/build/precision/hover.bin`;
  the pinned `build/hover.bin` was not overwritten.
- The C changes are kept as `firmware/stm32f103/eferu-foc/precision.patch`
  because the `eferu/` checkout is an upstream repository and is not pushed.
  Apply it from the `eferu/` root with `git apply ../precision.patch`.

## Risks and open questions

- The mainboard feedback change is a wire-contract change and needs a paired
  flash. Do not flash one side alone.
- `tmax` changes the lift/fall guard thresholds because they derive from the
  torque budget. Re-arm and re-check the guards after every change.
- `ANG_MEAS = 1` may trade standstill accuracy for hall-sector torque ripple.
  It must be judged on the bench, not assumed.
- No Kt measurement, dead-zone sweep or actuator-delay measurement was done in
  this change. The numeric values above remain hypotheses until step 2–5
  produce artifacts; only the command-reporting latency is measured above.
