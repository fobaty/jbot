# Inverted-lever calibration

Software-only identification of the torque a wheeled inverted pendulum needs
at each pitch angle. No new hardware, no manual pushes. The robot starts lying
on its back or face, and a closed-loop driver lifts it very smoothly, holding
for a moment at every few degrees. The torque that holds each angle is recorded
and becomes the feedforward correction used during maneuvers.

## Poses stored in NVM

Three poses are captured once and saved with `pose ...; save`. They survive
power cycles and do not need to be re-entered for every session:

| Pose | Stored pitch | Meaning |
|---|---|---|
| `back` | -80.4 deg | lying on the back, wheels up |
| `face` | +83.5 deg | lying face down |
| `home` | -2.5 deg | captured vertical zero |

The `home` value is also the pitch zero offset (`zof=-2.50 deg`). A construction
change invalidates all three; clear them with `forget` and recapture.

## The lift-from-lying algorithm

The disturbance-based trials in `docs/wp3-bench-log.md` were dominated by the
way the body was released, not by the control law. Starting from a lying pose
removes that problem: the body is supported by the floor, so the initial
condition is stable and repeatable. The driver then:

1. Reads the three stored poses with the `pose` command.
2. Classifies the current pose from pitch telemetry (within 25 degrees of a
   stored pose, otherwise it refuses to run).
3. If lying, raises the pitch target a few degrees toward home at a time.
4. Drives the common-mode `test` torque with a PI controller plus rate damping.
   The integral term grows until it overcomes the loaded dead zone (~50 units),
   then settles at the torque that holds the current angle. This is the moment
   the body is "carried" from one angle to the next, very smoothly.
5. At each target it waits for the body to be still (pitch within 0.6 steps and
   rate below 8 dps) for `hold-ms`, then records the mean holding torque.
6. Stops when the body is within 6 degrees of home. Every run ends with `estop`.

The robot stays disarmed throughout. Only the existing `test` command moves the
motors, so the 200 Hz balance loop is never touched and no reflash is needed.

## Why lifting from a lying pose fails

The first `lift_cal` trial (`csv/jbot-lift-<timestamp>.csv`, 2026-11-22) settled
the question. The robot lay at -98.7 degrees, the driver ramped the common-mode
`test` command from -50 to -200 units, and the body only rocked between -66 and
-101 degrees. Individual jerks reached -47 degrees at 139 dps, then fell back.
At that tilt the body rests on its shell and the wheels are off the floor, so
the motor torque has no ground reaction to pivot against. The body cannot be
held at an intermediate lying angle regardless of torque magnitude. This is the
same physical limit recorded in `docs/wp1-simulation-results.md`: self-righting
from supine is unreachable with this drive.

A wheeled inverted pendulum regulates pitch only while its wheels touch the
floor. The useful operating range for maneuver corrections is the small band
around upright, roughly +/-5 degrees, where the `home` reference and the balance
law work together. The feedforward curve must be measured there.

## Dynamic gyro-null experiment (2026-11-22)

`crates/jbot_console/src/bin/gyro_null_cal.rs` implements the requested
dynamic method: instead of placing the body at fixed angles, it applies a short
torque starting very small (4 units) and ramps up, measuring `alpha = d(gy)/dt`
to find the torque where the body stops accelerating. No static placement, no
reflash.

`csv/jbot-gyronull-1795398968223.csv` shows why this cannot identify the
feedforward near upright. At 4 units the wheels never turn (`L=R=0 rpm`) and
`gy` is only sensor noise within +/-5.6 dps, so `alpha` is indistinguishable
from zero. The search reports a false null of -4 units at every pitch from -4.1
to +0.6 degrees. The loaded dead zone (~50 units) is far above the requested
4-unit start, so the command never reaches the motors. This is a dead-zone
limit, not a tuning failure: any torque under ~50 units produces no wheel
motion and therefore no gyro signal to null.

Raising the start above the dead zone would move the body, but then the search
is measuring breakaway friction, not the gravity holding torque. Near upright
the gravity moment is itself below the dead zone, so no gyro-null measurement
there can separate the two. The existing bounded feedforward (+3.23 units/deg
from the lever ladder, A/B verified) remains the best available estimate.

### Dynamic dead-zone search (successful, 2026-11-22)

The same driver was extended with a dynamic dead-zone search: it starts at 3
units and grows the command, watching wheel RPM and gyro rate, until the wheels
break away. `csv/jbot-gyronull-1795399203498.csv` found **53.8 units** (the
command where RPM jumped to 22 and gyro to 80 dps), matching the earlier static
estimate of ~50 units but obtained entirely dynamically. This is the reusable
result: the loaded dead zone can be auto-identified without manual placement.

The subsequent gyro-null phase in that run is not valid. The dead-zone search
left the body already swinging (gyro -109..+43 dps), so fitting `alpha` over a
140 ms window returned ~0 and reported false nulls. The torque-null needs a
quiet body before each probe. The driver was therefore updated: after the
dead-zone search it waits for `|gy| <= 8 dps` (held for 400 ms) before phase 2,
and repeats that wait before every individual probe. Telemetry rows for probe
windows now store the computed `alpha` instead of a hardcoded zero, so the CSV
shows whether the fit is trustworthy. This re-settle change is implemented and
passes build/lint/tests, but has not yet been validated on hardware; the
dead-zone measurement from the earlier run remains the sound, reusable result.

## Driver (superseded by the reference-hold method)

`crates/jbot_console/src/bin/lift_cal.rs`:

```sh
cd /Users/fobaty/dev/jbot
cargo run -q -p jbot_console --bin lift_cal --release -- \
  --run --from auto --step 5 --hold-ms 900 \
  --kp 1.5 --ki 3.0 --kd 0.4 --max-units 200
```

- `--from back|face|auto` overrides pose detection when the body rests between
  poses. `auto` uses the stored poses and pitch.
- `--step` is the pitch increment per hold, degrees. `--hold-ms` is how long
  the body must stay still before a point is accepted.
- `--kp`, `--ki`, `--kd` tune the lift servo. `--ki` is what finds the holding
  torque; increasing it makes the lift more eager, decreasing it makes it
  gentler. `--max-units` is the torque ceiling (firmware limit is 300).
- Aborts on 0.4 m of travel, a body rate above 250 dps, an armed/guarded law or
  an estop latch.
- Output: `csv/jbot-lift-<timestamp>.csv` (per-sample telemetry, readable by
  `jbot-analyze`) and `csv/jbot-lift-<timestamp>.json` (the recorded
  angle-to-torque points and fit).

## Reference-hold driver (current method)

`crates/jbot_console/src/bin/ref_hold_cal.rs` measures the holding torque in the
range that actually matters:

```sh
cd /Users/fobaty/dev/jbot
cargo run -q -p jbot_console --bin ref_hold_cal --release -- \
  --run --pscale 0.25 --slew 100 \
  --angles -5,-4,-3,-2,-1,0,1,2,3,4,5 \
  --settle-ms 1500 --hold-ms 800 --tol 1.0
```

It arms the balance law on the safe profile with `ff off`, steps the `home`
reference through each angle, waits for the body to settle (pitch span within
`tol`, rate below 8 dps), then records the mean applied balance torque `trq`
over `hold-ms`. A least-squares fit gives `T(pitch)`, which is the feedforward
that offloads the integral term the law currently uses to hold these angles.
Output: `csv/jbot-refhold-<timestamp>.csv` and `.json`.

The robot stays on its wheels throughout, so no reflash is needed and the
200 Hz balance loop is untouched. Every run ends with `home 0` and `estop`.
Aborts on guard/latch, 120 dps body rate or 0.4 m travel.

## What the driver computes

For each hold point the reference-hold driver stores `(pitch, torque)` where
`torque` is the mean balance command that kept the body still at that pitch.
A least-squares fit gives the holding-torque line `T(pitch) = slope*pitch +
intercept`. The law currently supplies this torque through its pitch integral;
a feedforward that supplies the same torque lets that integral stay near zero:

    ff(pitch) = T(pitch) = slope*pitch + intercept

The sign comes from the measurement itself: a negative pitch needs a negative
holding torque. The driver prints the fitted `ff(pitch)` so the constants in
`jbot_pico_core::control::feedforward_units` can be updated from real data
instead of a guess.

## Firmware feedforward

`jbot_pico_core::control::feedforward_units` currently uses a bounded slope of
+3.23 units/degree relative to home -2.5 degrees, clamped to +/-60 units, added
to the common-mode balance torque before the SafeFall fade.

The feature is **off at boot** and enabled only with the console command
`ff on` (`ff off` disables, `ff` queries). Runtime-only, like `pscale` and
`slew`. Telemetry reports the applied value in the `ff` field. The firmware with
this feature was flashed on 2026-11-22 (`picotool load -u -v -x -t elf`,
verified OK); boot telemetry shows `ff=0` and `zof=-2.50 deg`. Once the lift
calibration produces a stable curve, these constants are updated from the
measured slope and reflashed.

### Earlier A/B hold result

An initial A/B at `pscale 0.25 / slew 100` used a negative slope and was worse
than no feedforward: `csv/jbot-recovery-1795396331682.csv` (`ff off`) had pitch
p2p 1.9 degrees and no oscillation, while `csv/jbot-recovery-1795396373463.csv`
(`ff on`) had p2p 3.4 degrees and a 3.93 s period. The ladder torque is what
*caused* a lean, so returning home needs the opposite sign. The corrected
+3.23 units/degree firmware was re-tested in
`csv/jbot-recovery-1795396542330.csv`: p2p 1.2 degrees, no oscillation, 0.000 m
travel. The lift calibration replaces this guessed slope with a measured one.

## Historical: upright torque ladder (superseded)

The earlier `lever_cal` driver applied a torque ladder from upright and watched
where the body settled. Four runs gave two useful facts even though the method
was not repeatable enough for a control law:

- A loaded dead zone around 50 units (0.29 Nm at 175 units/Nm): commands up to
  +/-50 produced no wheel motion; 60 units gave about -15 degrees and 80 about
  -32 degrees (`csv/jbot-lever-1795395400911.csv`,
  `csv/jbot-lever-1795395510038.csv`).
- The lying poses used for NVM: `csv/jbot-lever-1795395641356.csv` reached
  +83.5 degrees and `csv/jbot-lever-1795395651406.csv` reached -80.4 degrees.

The combined `lever-fit` tool wrote `csv/lever-calibration.json`. These results
are kept as a record only; the lift-from-lying method above is now the source
of the feedforward curve. The ladder cannot identify the static gravity lever
`m*g*L` because there is no static equilibrium at a nonzero pitch with wheels
on the floor.

## Safety

- Rope attached, hands on or immediately above the body, zone at most 0.5 m.
- Fresh step-by-step approval for every run.
- Start at 200 units or less; the driver never exceeds the requested maximum.
- No manual pushes; all excitation is command-driven.
- Pose values are read from NVM, never rewritten by the lift driver.
- The run always ends with `estop`; use `clear` before the next trial.

## Related

- `docs/wp0-torque-identification.md` for the units-per-Nm scale.
- `docs/wp1-simulation-results.md` for the negative supine-recovery result.
- `docs/wp3-bench-log.md` for the disturbance trials that motivated this method.
- `docs/pscale-ladder.md` for the currently blocked gain ladder.
