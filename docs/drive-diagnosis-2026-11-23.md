# Drive and release diagnosis — 2026-11-23

Session aimed at a repeatable 10 s self-balance. The hold was **not achieved**.
The useful result is a set of measurements that localise the blocker.

## Release grading corrected

`hold_sweep` originally graded the release by the peak gyro rate over the first
150 ms *after* the cue. That conflated two different things:

- a hand swing at the release instant, which invalidates the trial, and
- the natural fall of a free inverted pendulum after a clean release, which is
  a result of the control law.

A clean release (`release_pre_cue=0.5 dps`, `release_post_cue=3.8 dps` at the
cue) was reported `VOID` because the gyro later reached 56 dps while the body
fell. The metric now uses a narrow window of a few samples on each side of the
cue (`release_cue_rate`); the longer-window peak remains in JSON as
`release_gy_peak`, diagnostic only. A fall after a clean release is a `FAIL`
for the law, not a `VOID` for the fixture.

Two further gates were added: a sustained lean with stationary wheels is
`NO - externally supported`, and a pitch above 20 deg ends the sweep with an
abort reason instead of silently skipping the remaining profiles. The baseline
now requires stillness rather than an exact pose; `zero` is captured at that
pose and defines vertical for the law.

## Manual release is not repeatable

Measured cue rates across attempts: 3.8, 4.8, 9.0, 33.9, 56.7 and 211.8 dps.
The fixture gate of 2 dps is unattainable with a hand release. A fixed anchor
with a cut thread is still required before the WP3 gate can be closed.

## Motor-drive diagnosis

A host-driven ramp of `test` torque in 5 u steps (slower than the firmware's
600 u/s spin ramp) with the wheels suspended gave:

| Command | Peak gyro | Body motion | Wheels |
|---|---|---|---|
| 5–40 u | <=0.38 dps | none | nearly still |
| 45–60 u | 2–7 dps | 0.3 deg | starting |
| 65–75 u | 25 dps | 1.5 deg | breakaway kick |
| 80–90 u | 5–15 dps | 0.1–0.3 deg | steady |
| 100 u | 56 dps | 2 deg | another kick |

This is stick-slip in the drivetrain, not a weak motor: once the gears break
away the reaction torque swings the body hard.

## STM32 input chain verified

The STM32 USART3 debug serial was read with a PIO UART bridge on GP12/GP13
forwarded to the Pico USB console. That bridge was later removed because its
PIO driver wedged the USB console; use a 3.3 V USB-UART adapter on the right
cable for this check. With the bridge live, the EFeru periodic print showed:

| Pico request | STM32 debug |
|---|---|
| `test 40 40` | `in2:40 cmdL:39 cmdR:39` |
| `test 80 80` | `in2:80 cmdL:80 cmdR:80` |
| `test 120 120` | `in2:120 cmdL:120 cmdR:120` |

The command path is correct: no input deadband, no scaling error, no lost
frames. `FILTER`, `RATE` and the input limits are not the cause. The
nonlinearity is downstream, in the FOC current loop and gear train.

A later capture over the same GP12/GP13 bridge
(`csv/jbot-stm32-debug-1795484081193.txt`) caught the mainboard restart line
`Using the configuration from config.h`. EEPROM is not overriding the build,
so the patched `FILTER=19661` (alpha 0.3, tau ~14 ms) and `TRQ_MODE` are the
values actually running. The periodic print also reported both inputs as
`TYP:2` (serial), confirming the `VARIANT_USART` build. Idle board temperature
was 34.9-37.3 C during that capture; no loaded temperature was recorded.

## Control-law experiments

All runtime knobs were exercised with a software `home` disturbance, so the
release impulse could not contaminate the result:

- `pscale 0.30–0.45`, `slew 100–3000`: torque for a 1 deg error is 28–32 u,
  below the ~54 u breakaway, so the wheels do not move.
- `pscale 0.60–0.80`: torque crosses the dead zone but the loop diverges in
  about 0.5 s (`pitch` 29–38 deg, disarm).
- Dead-zone compensation, both the original quadratic inverse and a
  saturating variant, either under-compensated or produced an immediate
  overtorque. The saturating variant was reverted.
- A runtime `dscale` (derivative-only gain scale) was added to separate the
  proportional torque needed to cross the dead zone from the gyro term.
  `pscale 0.80 / dscale 0.20` still diverged, so the P/D ratio is not the
  root cause either.

Offline identification of `a1`/`a2` from the closed-loop CSVs failed: the
command is correlated with the error, so a regression returns a negative
`A1` and an `A2` varying by an order of magnitude. Those recordings cannot
identify the plant.

## Per-wheel breakaway calibration

Swap test confirmed that the breakaway threshold travels with the physical
motor, not the mainboard channel: before the swap the left wheel started at
~35 u and the right at ~50 u; after swapping both harnesses the left channel
started at ~45 u and the right at ~30 u. Restoring the harness returned the
thresholds to ~30 u left / ~40 u right
(`csv/jbot-breakaway-restored-1795480697492.csv`). At 80 u both wheels reach
nearly equal rpm, so the channel gains match and only the near-zero threshold
differs. This rules out an FOC gain or input-scale error in the mainboard.

A runtime `cal` command now carries one smooth breakaway inverse per wheel, in
EFeru command units. It is applied once at the firmware command choke point, so
`test` and the balance law see identical calibration, and it is zero at boot.
The inverse blends linearly over the last 10 u before zero: a hard sign step
would be a relay and reproduce the overtorque that made the earlier symmetric
dead-zone compensation unusable.

The calibration is validated with `cargo run -q -p jbot_console --example
motor_cal` on the suspended stand: first with `cal off` to bracket the
thresholds, then with `--cal L R` to confirm the first motion moves toward zero
and the rpm curve becomes proportional. Only after that is it enabled for a
guarded balance attempt.

## Balance attempts with per-wheel calibration (2026-11-23)

Three held-stance attempts used the calibrated drive on the floor with a
software `home` reference snap and a safety rope:

| Profile | Result | Recording |
|---|---|---|
| `cal 32 28`, `pscale 0.30`, `slew 100` | Diverged in ~4 s: pitch 2 to -48 deg, gyro +/-220 dps, fall latch | `csv/jbot-recovery-1795481850571.csv` |
| `cal 32 28`, `pscale 0.10` | Effective torque stayed below breakaway; wheels never turned | `csv/jbot-recovery-1795482049833.csv` |
| `cal 32 28`, `pscale 0.20` | Held only while supported; after release pitch reached -19 deg in ~300 ms, then +/-40 deg and 320 dps | `csv/jbot-recovery-1795482253787.csv` |

The inverse removed the stand-measured dead zone (first motion moved from
35/25 u to 10 u command), but it did not remove the mechanical lag. While the
body is still held, the law asks for only a few units of torque; at release the
wheels are stationary, so the first useful correction arrives after the body
has already fallen several degrees. Full compensation then behaves like a
relay and pumps the oscillation instead of catching it. The remaining blocker
is therefore drive dither or a mechanical solution, not the calibration itself.

## Dither knob

To attack the breakaway delay directly, a runtime `dither <units|off>` command
adds a 12 Hz common-mode triangle wave after the per-wheel inverse. It is off
at boot, capped at 60 units, bypassed by the e-stop, and cannot steer because
the same value goes to both wheels. The 12 Hz rate is above the body's balance
mode but below the frequency where the flashed mainboard filter attenuates the
drive. Telemetry reports `dth` (configured amplitude) and `dthv` (value applied
on the tick), so the effect is visible in every recording.

The stand step test (`csv/jbot-dither-step-*`) measured the delay from a 20 u
step to sustained wheel motion: about 146 ms with dither off, 127 ms at 30 u and
95 ms at 60 u across two runs (the off value also varied down to 109 ms between
runs). The improvement is small. In the closed balance loop, 60 u dither was
actively harmful: it injected common-mode motion of +/-125 u at the wheels and
the pitch diverged in 3.4 s (`csv/jbot-recovery-1795483568191.csv`). Dither
therefore stays off by default and is not considered a fix.

## Current conclusion

The drive's stick-slip plus actuator lag, not the gains and not the command
path, is the blocker. Viable next steps are drive dither to keep the gear
train moving, a mechanical reduction of backlash/friction, or an open-loop
identification on a suspended body. No gain or guard change should be treated
as a fix until the release fixture exists.

## Artifacts

- `csv/jbot-hold-*`: hold trials with release grading fields.
- `csv/jbot-recovery-*`: software-disturbance trials.
- `/tmp/jbot_openloop_*`: suspended open-loop ramps (not committed).
- `firmware/stm32f103/eferu-jbot.patch`: EFeru patch including
  `DEBUG_SERIAL_USART3` for the bench bridge.
