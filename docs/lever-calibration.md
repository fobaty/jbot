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

`--slew` accepts up to 3000 units/s, `--db 0.25` sets runtime deadband
compensation in newton-metres, `--cal LEFT RIGHT` sets per-wheel breakaway
compensation, and `--dither 30` adds a 12 Hz triangle wave to hold the drive at
the edge of stiction. All are recorded in the JSON next to `pscale`.

It arms the balance law on the safe profile with `ff off`, steps the `home`
reference through each angle, waits for the body to settle (pitch span within
`tol`, rate below 8 dps), then records the mean common-mode torque over
`hold-ms`. By default the fit uses the **applied** commands reported by the
mainboard, `(al + ar) / 2`, not the requested `trq`: the former is what the
drive actually delivered. `--torque-source requested` reproduces the older
method for an A/B comparison. A stale mainboard frame (`mb != ok` or `age >
300 ms`) aborts the trial instead of fitting stale applied words.
A least-squares fit gives `T(pitch)`, which is the feedforward that offloads
the integral term the law currently uses to hold these angles.
Output: `csv/jbot-refhold-<timestamp>.csv` and `.json`; both requested and
applied per-wheel commands are recorded, plus `torque_source` in the JSON.
The command-path delay and its spread can be scored from the same CSV:

```sh
cargo run -q -p jbot_analysis --bin command_path -- csv/jbot-refhold-<timestamp>.csv
```

First run (2026-11-26): `csv/jbot-refhold-1795718229688.csv` aborted at the
first angle. The baseline pitch was 7.38 deg with the body externally supported,
the wheels reached about 200 rpm, and `x` moved 0.428 m from the origin before
the 0.4 m travel guard fired. No hold point was recorded, so no torque curve
exists yet. `command_path` on that CSV reports 15 gradual command steps with a
6-7 ms applied-word reporting delay; that is feedback-frame quantization, not
actuator delay. The driver ended with `estop=1`, `armed=0`, `al=ar=0`. Retry
only with both wheels on the floor and the body free to balance, the rope slack
as a catch rather than a support.

Second run (2026-11-26): `csv/jbot-refhold-1795718437631.csv` got no hold point
either. The `level` command was silently refused, so the law saw a 5.8 deg error
and ramped torque to +59 units while the wheels stayed still and the body was
externally held; removing the support injected about 115 dps and the body fell
to -17.3 deg. The driver now checks that `level` was captured and rejects a
baseline with a lean and motionless wheels, mirroring `hold_sweep`'s
`externally_supported` gate. No firmware was changed.

Third run (2026-11-26): `csv/jbot-refhold-1795718675014.csv` passed the new
baseline check (pitch 1.69 deg, level 1.62 deg) but still got no hold. During the
`-3 deg` ramp the body stayed at about 1.9 deg while the applied command rose
only to 30-34 units with both wheels at 0 rpm. That band is at or below the
measured loaded breakaway of roughly 40-60 units, so the drive never broke away;
when the external support was removed the body reached 22.9 deg at 272 dps and
the guard fired. The limiter here is torque authority/dead zone and the 100
units/s slew, not the reference ramp. A runtime trial with a higher slew and
deadband compensation is needed before any curve can be measured.

Fourth run (2026-11-26): `csv/jbot-refhold-1795718893508.csv` used `slew=300`
and `db=0.25` and still got no hold. The wheels stayed at 0 rpm until the
applied command passed roughly 40 units, then the body ran away to -33 deg.
`jbot-analyze` reports a breakaway of 63 units (0.32 Nm nominal) and an
oscillation period near 0.93 s; the fitted `kp 0.7 Nm/rad` is unreliable because
most of the recording is an externally supported baseline. `db=0.25` sits below
the observed breakaway. More runtime trials without a repeatable fixture only
mix support artefacts with control behaviour, so the next step is the fixture in
`docs/release-fixture.md`, not another gain change.

Fifth run (2026-11-26): `csv/jbot-refhold-1795719007893.csv` used `db=0.35`
(70 units) and still had both wheels at 0 rpm while the command oscillated
within roughly +/-30 units; the body was released only when the operator let go
and then fell to -29.8 deg. `jbot-analyze` reports an unpaired pitch angle/rate
channel for this session and a breakaway of 50 units. Correction: the operator only supported the body lightly to keep it upright,
not by clamping it. Motionless wheels at 30 units are therefore explained by
the measured 50-63 unit breakaway, not by a rigid fixture. `db` and `slew` were
the wrong knobs: the quadratic deadband term is small near equilibrium and a
larger `db` subtracts torque above the threshold. The firmware already provides
per-wheel breakaway compensation (`cal l|r`) and a 12 Hz dither that keeps the
drive at the edge of stiction; both are now driver options
(`--cal LEFT RIGHT`, `--dither UNITS`).

Sixth run (2026-11-26): `csv/jbot-refhold-1795719670696.csv` used `cal 50 50`
and `dither 30`. The wheels did break away (the inferred breakaway fell to 15
units) but the applied command oscillated roughly between -76 and +106 units
while `trq` stayed within +/-15 units, so the dither/cal combination overwhelmed
the law: the body reached 23 deg and then -8.8 deg at -254 dps before the guard.
`jbot-analyze` finds a 0.64 s oscillation and `kp 0.4 Nm/rad`. Next trial holds
`cal` fixed and reduces only the dither amplitude.

Seventh run (2026-11-26): `csv/jbot-refhold-1795719988511.csv` used `dither 10`
with the same `cal 50 50`. Breakaway dropped to 13 units, but the applied
command still swung between roughly -60 and +100 units while `trq` stayed
within +/-15 units; the body reached 31 deg and then -3.2 deg at -300 dps.
`kp` stays near 0.4 Nm/rad, far below the 5.39 Nm/rad stability floor, because
the 100 units/s slew cannot ramp the requested torque before the body moves.
The next single-variable change is `slew 300` with `cal` and `dither` unchanged;
if that still fails, stop runtime trials and measure Kt/breakaway on a locked
wheel before touching any gain again.

Eighth run (2026-11-26): `csv/jbot-refhold-1795720087862.csv` used `slew 300`
with `cal 50 50`, `dither 10`. `kp` rose to 1.0 Nm/rad but remained far below
the 5.39 Nm/rad floor; the oscillation period shortened to 0.36 s and the body
reached 15.8 deg at 278 dps in 0.8 s. All runtime knobs have now been tried in
isolation; none produced a single stable hold. Runtime tuning is closed and WP0
is the next step.

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
