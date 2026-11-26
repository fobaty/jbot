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
- No Kt measurement, dead-zone sweep or hardware latency measurement was done
  in this change. The numeric values above remain hypotheses until step 2–5
  produce artifacts.
