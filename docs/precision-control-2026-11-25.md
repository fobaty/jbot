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

`GET ANG_MEAS` and `SET ANG_MEAS 1` work at runtime. `FILT` is a **variable**,
not a parameter: `SET FILT 65535` is rejected with
`Err3:"This command cannot be used with a Variable"`. `INIT <name>` is also
rejected (`Err2:"Parameter not found"`); restore `ANG_MEAS` with
`SET ANG_MEAS 0`. Both values are runtime-only and disappear on power-up;
changing `FILT` requires recompiling `FILTER` and reflashing the F103.

`DEBUG_SERIAL_PROTOCOL` is now enabled inside `VARIANT_USART` in `Inc/config.h`;
the C changes are kept in `firmware/stm32f103/eferu-foc/precision.patch` because
the `eferu/` checkout is upstream. The image is
`eferu/build/precision-proto/hover.bin` (SHA-256 `267322b5…`, 40408 bytes),
flashed and verified with OpenOCD. The Pico bridge fix in commit `c297027`
forwards the whole command line: `dbg $GET ANG_MEAS` returns
`ok dbg tx 13` and the F103 replies `# name:"ANG_MEAS" value:0 init:0 min:0 max:1`.
`dbg $GET FILT` returns `value:19661`. Both values are runtime defaults.

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
   and compare the breakaway and the low-speed torque quality. Restore with
   `SET ANG_MEAS 0`.
5. **A/B the input filter.** Not possible at runtime: `FILT` is a read-only
   variable in this protocol. Two images with different `FILTER` values would
   be needed. The Pico already slews at 3000 units/s, so the mainboard filter
   may be redundant, but this is unproven.
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

`dbg on` forwards the mainboard's periodic USART3 telemetry. The console
originally sent only the first whitespace token: `dbg $GET ANG_MEAS` became
`$GET` (`ok dbg tx 4`). Commit `c297027` forwards the full remainder, verified
on the rebuilt and flashed Pico (`ok dbg tx 13`). The mainboard now accepts `GET`/`SET` because
`DEBUG_SERIAL_PROTOCOL` is enabled in the flashed build; the GP13-to-PB11 path
is working. No F103 parameter was changed during verification and `dbg` was
returned to `off`.

## F103 A/B result (2026-11-25)

`ANG_MEAS` was switched from 0 to 1 with the wheels idle, `estop` latched and
the balance law disarmed. Pico pitch stayed at -1.12 deg in both states (sd
0.000), and the gyro scatter changed from 0.080 to 0.070 rad/s sd, which is not
a meaningful difference. The value was restored to 0 with `SET ANG_MEAS 0` and
verified with `GET`. `ANG_MEAS` selects the F103 electrical angle source, so
body pitch is not a sensitive comparator; a real A/B still needs a repeatable
breakaway fixture.

`FILT` is a variable and `SET FILT 65535` is rejected with `Err3`, so no filter
A/B was possible. The command-reporting rise with the current build and
`FILT=19661` is 104 ms, consistent with the earlier 114 ms measurement.

## Balance trials (2026-11-25)

All runs used `hold_sweep` at `tmax=5.0` and ended with `estop`. The bench
scripts live in `bench-temp/` and are gitignored: `probe-pico.py` checks for a
live console plus fresh `T`-line feedback (and restores `stream 200`, which a
Pico reboot clears), `run-stand.sh` arms from a supported quiet pose and waits
for a release, `ready-notify.sh` raises a macOS cue at ARM.

Fixture qualification was attempted four times and never passed:

| Artifact | Result |
|---|---|
| `csv/jbot-hold-1795656650515.json` | 0/3: two 20 s release timeouts, then no quiet baseline. The operator never removed the support. |
| `csv/jbot-hold-1795657060916.json` | 0/3: cue 3.1 dps with 0.001 m travel and 3 wheel-active samples (load not transferred), then cue 11.7 dps with `pre_cue=10.2` and a fall. |
| `csv/jbot-hold-1795657869027.json` | 1/3: first cue 0.2 dps but 0 wheel-active samples (still hanging); second cue 13.8 dps with `pre_cue=11.6` from re-tying the rope between profiles. |
| `csv/jbot-hold-1795657299836.json` | No arm: the mainboard was asleep (`mb=none`, `bat=0.00`) after idling. |

A hand-held rope and a hand-removed stand are unsuitable release methods:
their cues were 11.7, 13.1, 13.8 and 19.2 dps, matching the 13-32 dps range in
`docs/release-fixture.md`. Cutting a single thread under a rigid anchor is the
only method that produced a clean cue: **0.7 dps** (pre 0.7 / post 0.5) in
`csv/jbot-hold-1795658605321.csv`.

Two further runs were invalid before the control law could be judged:

- `csv/jbot-hold-1795658209552.csv`: the robot was already falling at ARM
  (`gy=-7.75 dps` and accelerating), so the 10.7 dps cue came from a falling
  body, not from the release. The strict gates `--quiet-gy 3 --quiet-pitch 3`
  were added after this.
- `csv/jbot-hold-1795658556143.csv`: ARM was quiet (`gy=0.94 dps`) but removing
  the stand injected 13.1 dps.

The single valid control result is `csv/jbot-hold-1795658605321.json`:
`pscale=0.30`, `slew=100`, `db=0.00`, clean cue 0.7 dps, **FAIL** with the robot
down at 20.13 deg. Travel was only 0.011 m with 93 wheel-active samples and 72
consecutive still samples while leaning, so the gate marked it
`externally_supported=1`. The likely limiter is `slew=100` units/s: reaching
50-70 units of torque takes about half a second, too slow for an inverted
pendulum. A repeat at `slew=300` (`csv/jbot-hold-1795658697895.csv`) was spoiled
by a 19.2 dps cue and carries no information. No conclusion about `pscale` or
`dscale` is justified yet.

Operationally, the Pico CDC stopped answering after host sessions closed and
the USB device repeatedly disappeared; recovery required a physical USB
re-plug. The mainboard also fell asleep whenever the motors stayed idle for a
while and needed a 2-3 s power-button hold until the beep.

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
