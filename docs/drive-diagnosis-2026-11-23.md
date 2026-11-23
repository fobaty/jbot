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

A PIO UART bridge on GP12/GP13 forwards the STM32 USART3 debug serial to the
Pico USB console (`dbg on`, `dbg <text>`). With the bridge live, the EFeru
periodic print shows:

| Pico request | STM32 debug |
|---|---|
| `test 40 40` | `in2:40 cmdL:39 cmdR:39` |
| `test 80 80` | `in2:80 cmdL:80 cmdR:80` |
| `test 120 120` | `in2:120 cmdL:120 cmdR:120` |

The command path is correct: no input deadband, no scaling error, no lost
frames. `FILTER`, `RATE` and the input limits are not the cause. The
nonlinearity is downstream, in the FOC current loop and gear train.

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
