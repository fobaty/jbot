# Inverted-lever calibration

Software-only identification of the torque a wheeled inverted pendulum needs
at each pitch angle. No new hardware, no manual pushes: the driver raises the
commanded torque in small steps and watches where the body settles.

## Physics

With the wheels on the floor, a common-mode torque `tau` on the two motors
produces a reaction torque on the body and a traction force at the contact
patch. At quasi-static pitch `theta` the balance of moments is approximately

    tau = A * sin(theta) + B * sign(tau) + C

where `A = m*g*L` is the gravity lever, `B` is the loaded dead zone and `C` is
a static bias. Using the WP0 scale of 175 command units per Nm, `A/175` should
be close to the 23.5 Nm estimated from the COM height of 0.25 m. The available
budget is about 1000 units (5.7 Nm), so the statically holdable range ends near
14 degrees; the driver reports where the body actually reaches a lying pose.

## Driver

`crates/jbot_console/src/bin/lever_cal.rs`:

```sh
cd /Users/fobaty/dev/jbot
cargo run -q -p jbot_console --bin lever_cal --release -- \
  --run --direction back --max-units 120 --step 10 \
  --hold-ms 250 --settle-ms 250
```

- Direction `back` uses positive torque, `face` negative.
- `--hold-ms` is short on purpose: a sustained command would drive the robot
  out of the 0.5 m zone. The driver aborts on 0.4 m of travel, on a body rate
  above 400 dps, or when 65 degrees of tilt is reached.
- Every run ends with `estop`. The robot must be upright, still, disarmed and
  linked before the ladder starts.
- `--commit-poses` writes the learned `back`/`face`/`home` pitches to NVM with
  the `pose` command and `save`; without it the run is a dry run.
- The CSV is compatible with `jbot-analyze` and adds `phase`, `level` and
  `commit` columns.

## What the driver computes

1. Quiet, upright baseline pitch, offered as the `home` pose.
2. For each torque level, the mean pitch of the quasi-static part of the hold
   window (samples with `|gy| < 30 dps`).
3. A least-squares fit of `tau = A*sin(theta) + B*sign(tau) + C`.
4. `A/175` as the gravity lever in Nm, `B/175` as the loaded dead zone in Nm.
5. A feedforward table for 2, 4, 6, 8, 10 and 12 degrees.
6. The largest observed pitch, offered as the `back` or `face` pose.

## How to use the result

The fit is an identification, not a control-law change. Feedforward torque can
later be added to the balance controller so the feedback loop only has to
correct the residual; the fitted `A` also replaces the current `m*g*L`
assumption when re-deriving gains. Do not enable dead-zone compensation or
change `PlantParams::NOMINAL` until the same curve has been reproduced on both
directions and the fit is stable across runs.

## First results (2026-11-22)

Two dry runs at `max-units 80`, `step 10`, `hold 200 ms`:

- `csv/jbot-lever-1795395400911.csv` (`back`): commands 10-50 units produced no
  wheel rotation and almost no pitch change; 60 units gave about -15 degrees,
  70 about -21 degrees and 80 about -32 degrees.
- `csv/jbot-lever-1795395510038.csv` (`face`): commands -10 to -50 again stayed
  inside the dead zone; -60 gave about +9 degrees, -70 about +23 degrees and
  -80 about +42 degrees.

The combined offline tool produced `csv/lever-calibration.json`:

```sh
cargo run -q -p jbot_analysis --bin lever-fit -- \
  csv/jbot-lever-1795395400911.csv csv/jbot-lever-1795395510038.csv
```

It reports a loaded dead zone of 50 units (0.29 Nm at 175 units/Nm), a home
pitch of -2.4 degrees and an empirical angle-to-torque table from -12 to +12
degrees. The table is a measurement of the dynamic response with wheels on the
floor; it is **not** the static gravity lever `m*g*L`, which this setup cannot
identify because there is no static equilibrium at a nonzero pitch. Do not
interpret the per-run fit coefficient as the gravity lever.

Two further runs at `max-units 120` reached the lying poses:
`csv/jbot-lever-1795395641356.csv` (`face`, +83.5 degrees) and
`csv/jbot-lever-1795395651406.csv` (`back`, -80.4 degrees). Combining all four
recordings gives:

- dead zone 50 units (0.29 Nm),
- home -2.5 degrees,
- back -80.4 degrees, face +83.5 degrees,
- an empirical angle-to-torque table from -12 to +12 degrees in
  `csv/lever-calibration.json`.

The learned pose values were committed to NVM on 2026-11-22 with
`pose back -80.4`, `pose face 83.5`, `pose home -2.5` and `save`. Telemetry
confirmed the new zero offset (`zof=-2.50 deg`). As with any pose change, the
values must be invalidated with `forget` and recaptured after a construction
change.

## Firmware feedforward (not flashed yet)

`jbot_pico_core::control::feedforward_units` computes a bounded holding torque
from the measured slope: -3.23 units per degree relative to the home pitch of
-2.5 degrees, clamped to +/-60 units. It is added to the common-mode balance
torque before the SafeFall fade.

The feature is **off at boot** and enabled only with the console command
`ff on` (`ff off` disables, `ff` queries). Runtime-only, like `pscale` and
`slew`, so it cannot leak into a fresh session. Telemetry reports the applied
value in the `ff` field. The code and unit tests are in place; flashing and a
The firmware was flashed on 2026-11-22 (`picotool load -u -v -x -t elf`,
verified OK); boot telemetry shows `ff=0` and the persisted `zof=-2.50 deg`.

### A/B hold test and sign correction

`csv/jbot-recovery-1795396331682.csv` (`ff off`) and
`csv/jbot-recovery-1795396373463.csv` (`ff on`) are two 10 s hold gates at
`pscale 0.25 / slew 100`. With the first, negative slope the result was worse:

- `ff off`: pitch p2p 1.9 degrees, no oscillation, kp 6.6 Nm/rad, R2 0.75.
- `ff on`: pitch p2p 3.4 degrees, a 3.93 s period, kp 0.9 Nm/rad, R2 negative.

The ladder curve is the torque that *caused* a lean (+60 units gave -15
degrees). Returning the body home needs the opposite sign, so the corrective
slope was changed to +3.23 units per degree. The corrected code passes all
quality gates but has not been reflashed or re-tested yet.

The feedforward is an empirical correction, not the gravity lever `m*g*L`.
Verify with the gain ladder that enabling it does not change the stability
floor before relying on it.

## Safety

- Rope attached, hands on or immediately above the body, zone at most 0.5 m.
- Fresh step-by-step approval for every run.
- Start at 120 units or less; the driver never exceeds the requested maximum.
- No manual pushes; all excitation is command-driven.
- The pose commit is explicit: `pose home` changes the pitch zero offset and
  disarms the machine, so it is only sent with `--commit-poses`.

## Related

- `docs/wp0-torque-identification.md` for the units-per-Nm scale.
- `docs/wp1-simulation-results.md` for the negative supine-recovery result.
- `docs/pscale-ladder.md` for the currently blocked gain ladder.
