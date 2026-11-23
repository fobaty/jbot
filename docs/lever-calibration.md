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

## Driver

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

## What the driver computes

For each hold point it stores `(pitch, torque)` where `torque` is the mean
common-mode command that kept the body still at that pitch. A least-squares fit
gives the holding-torque line `T(pitch) = slope*pitch + intercept`. The
feedforward correction added by the balance law is the torque needed to cancel
gravity, which is the negative of the holding torque measured here:

    ff(pitch) = -(T(pitch) - T(home))

The sign matters: a positive `test` command drives the body negative, so the
correction that holds a negative pitch is negative. The driver prints both the
measured `T(pitch)` and the recommended `ff(pitch)` so the constants in
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
