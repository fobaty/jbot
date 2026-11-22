# Offline telemetry analysis

`jbot-analyze` turns a recorded console session into the numbers the bench cannot
show while the robot is falling over: whether the attitude and rate channels
belong together, what gain the firmware actually applied, where the drive starts
moving, and whether the pitch error is ringing.

The crate is `crates/jbot_analysis` (binary `jbot-analyze`). It uses only the
standard library: no CSV crate, no plotting, no network. It reads the files the
console already writes and prints a report.

## Usage

```sh
cargo run -p jbot_analysis --release -- jbot-telemetry-1795045179.csv
cargo run -p jbot_analysis --release -- --actions other-actions.csv session.csv
cargo run -p jbot_analysis --release -- jbot-telemetry-*.csv
```

* The telemetry file is the only required argument; several may be given.
* `--actions` overrides the action log. By default the analyzer replaces
  `telemetry` with `actions` in the file name and uses the sibling file when it
  exists.
* Exit code is non-zero when any file fails to parse.

## Inputs

Telemetry CSV, as written by `crates/jbot_console/src/record.rs`. Only the first
block of columns is required:

```
t_ms, step, loop_hz, imu_fresh, pitch_deg, roll_deg, heading_deg,
gy_dps, gx_dps, gz_dps, left_rpm, right_rpm, bat_v, age_ms, estop, torque_units
```

Later firmware added `vset, wset, test_left_units, test_right_units, ref_deg,
demand_deg, trim_deg, position_m, mode`. A recording without them is still
readable: the missing fields default to zero and the report names them in a
`schema:` line. Arming, falls and the pitch law are reported as not measurable
instead of guessed, because a defaulted `mode` column would otherwise look like
"the law never ran".

Action log CSV: `t_ms,kind,detail`, where `kind` is `tx`, `rx` or `session`.
Only `tx` rows matter here: `mount` commands split the session into segments.

## Report sections

### Summary

Rows, wall-clock length, per-mode sample counts, armed and emergency-stop time,
recorded and inferred fall latches, and the ranges of pitch, roll, wheel speed
and torque.

`falls` counts captured `mode 4` rows. A mid-run latch usually lasts one control
tick while the telemetry stream samples at 50 Hz from a 200 Hz loop, so the row
is often overwritten before it is printed. The analyzer therefore also counts
**suspected latches**: an armed row followed by a disarmed row while the body was
past 15° (the firmware recover threshold). `mode 4` and the transition are
reported separately because they are different evidence.

### Motion

Standard deviation of pitch, roll and both rate channels, plus the share of the
angular variance that lives in pitch.

A forward-lean check on a correctly mapped machine puts nearly all of the motion
in pitch. When the share is below 30 % the report says so: the tilt landed in
roll, which is what a wrong mount does. This is a hint, not a verdict — a
sideways tilt produces the same signature.

### Axis check

For each axis, the angle derivative is compared with its own rate channel and
with the other axis' rate channel (Pearson correlation and least-squares slope
through the origin, gaps between 1 and 50 ms). Verdicts:

| Verdict | Meaning |
|---|---|
| `consistent` | Direct correlation ≥ 0.7: the angle integrates its own rate |
| `inverted` | Direct correlation ≤ −0.7: the two disagree in sign |
| `swapped` | The cross channel correlates ≥ 0.7 and beats the direct one by 0.15 |
| `unpaired` | The axis moved (angle spread ≥ 1°) but neither channel correlates, or the slope is far out of scale |
| `unknown` | Too still to judge: no verdict is invented |

The slope is printed and gates the verdict only through a wide band
(0.3..3.0). The BNO055 fusion filters the Euler angle while the gyro is raw, so
a perfectly paired axis reads a slope around 0.8 instead of 1.0; requiring a
slope near one would report healthy sessions as broken. A slope far outside the
band, such as the 0.03 seen when a 30° tilt pairs with 0.7 °/s of rate, means the
two signals are not the same motion even though they correlate.

The headline verdict names the axis behind it (`(axis: roll)`) when one pair is
definitely wrong, and names the axis left out (`(roll not judged)`) when the
session only moved one of them. A still axis never downgrades a clean one: a
forward-lean check proves the pitch pair.

`mount normal` mixes two angle sources: the angles are the BNO055 Euler output,
while the rates are the raw gyro. The BNO055 defines Euler pitch about its own X
axis and roll about Y, so since the 2026-11-19 fix the firmware pairs raw X with
pitch and raw Y with roll. The pitch pair then closes cleanly (slope 1.00 on the
2026-11-19 session); before the fix the firmware fed raw Y to the pitch channel,
and this check reported `pitch ... unpaired`, which is the finding that exposed
the fault. The roll sign on the Euler path has not been bench-verified, so a
`normal` session that moves roll may still report `roll ... inverted`; the
balance law consumes pitch only. The gravity-derived mounts pair both channels
from one matrix, so they do not have this caveat.

When the action log contains `mount` commands the report prints the same check
per segment, so a mount sweep can be read row by row.

### Pitch law

A two-predictor least-squares fit of

```
torque_units = kp * error_rad + kd * rate_rad_s
```

over armed samples that are not saturated and whose IMU sample is fresh. The
result is the gain the firmware really applied, in EFeru units, with a nominal
newton-metre conversion at 200 units/Nm. `R2`, the residual, the sample count and
the fraction of armed samples that were clamped are printed with it so a bad fit
is visible.

Saturation is judged against the EFeru full scale (±1000 units), not against the
largest command in the session: an observed maximum confounds "nothing was
clamped" with "the session stayed far from the limit".

A negative `kp` is called out explicitly: the law would be fighting itself.

### Drive breakaway

The smallest command seen while the wheels were still reported at zero speed,
and the largest command seen while a drive sample was still stuck. The gap is the
dead zone bracket. When the two overlap the drives were already rolling and the
bracket is reported as overlapping instead of pretending to a bound.

This is a bracket from one recording, not an identification: the step size is
whatever the law happened to produce. Use it to choose the force ladder, then
confirm with a dedicated ramp.

### Pitch error while armed

Peak-to-peak pitch error, sign changes per second and, when the sign changes
cluster, the implied period. "No period" means the error drifted or stepped
rather than oscillated, which is a different fault than ringing.

## Limits

* The analyzer cannot choose the mount. Telemetry does not carry the gravity
  vector, so the mount must be selected on the bench (lean the robot forward and
  confirm pitch, not roll, responds) and then verified here by the motion share
  and the axis check.
* The nominal Nm conversion assumes 200 EFeru units/Nm and a 5 Nm plant. The
  plant has not been identified; treat newton-metre figures as a scale aid. The
  2026-11-22 audit records the measurement needed to replace that assumption
  (`docs/audit-2026-11-22.md`, phase 1).
* A `law_fit` from a run with `pscale < 0.25` measures an unstable loop: the
  stability floor is `A1/A2 = 5.39 Nm/rad` against `kp_base = 21.57 Nm/rad`.
  Such a fit is valid evidence for the command path, not for the recovery margin.
* Before the working-tree fix the `vset` column recorded a setpoint that the
  balance law never consumed; `wset` likewise produced no yaw torque because
  `yaw_kp_units` was zero.
* The law fit measures what the firmware commanded, not the torque the motors
  delivered. A dead zone, a slipping wheel or a saturated drive hides in the
  residual, not in the gains.
* Segments come only from `mount` commands in the action log. A sweep where the
  mount was changed without a recorded command reads as one segment.

## Reading the recorded sessions

The 14 sessions captured before the first successful balance run share one
pattern: `mount` is `0` or `1`, pitch and rate agree internally, the tilt sits in
the wrong channel, and the machine leaves the recover window within a few
seconds. The analyzer's job is to make that visible per file; the fix — choosing
the right mount, then identifying the plant — is `docs/design_balance_home.md`
§8 and §9.
