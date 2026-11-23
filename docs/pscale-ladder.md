# pscale ladder procedure

Re-run of audit Phase 3 after the sign/derivative correction. The compiled
`pscale=1.0` and `slew=3000` are development defaults, not a soft start; all
stable armed runs in the audit used `pscale 0.10-0.30`. The stability floor for
the corrected plant is `pscale >= 0.25`. No gain auto-tuning exists in the
200 Hz path, so the ladder is manual and gated.

This procedure is **code and procedure only**. Every bench step needs fresh
step-by-step approval and a secured robot: rope, hands on or immediately above
the body, wheels on the floor, test zone at most 0.5 m.

## Tooling

`recovery_driver` is the single entry point. It clears the latch, applies a
safe runtime profile, snaps the pitch reference to the measured attitude,
waits for a stable upright baseline, then either holds for 10 s or applies N
identical command-driven `home` steps. It always ends with `estop`.

```sh
cd /Users/fobaty/dev/jbot
cargo run -q -p jbot_console --bin recovery_driver --release -- \
  --run --relevel --hold-only --pscale 0.25 --slew 100
```

Use `--relevel`, not `--snap-ref`, for the hold gate: the gate scores absolute
pitch against level, so the level must be captured with the body genuinely
upright (the driver refuses a tilted capture). Snapping the reference to a
tilted attitude would make `|pitch| < 2 deg` impossible by construction.

Its CSV has the same first 25 columns as `spin_driver`, so the existing
scorer consumes it directly:

```sh
cargo run -q -p jbot_analysis --bin jbot-analyze -- csv/jbot-recovery-<ts>.csv
```

The report includes `oscillation` (`crossings_per_s`, `period_s`,
`error_p2p_deg`), `law_fit` and `breakaway`. A regular crossing train with a
stable period is a control loop fighting itself, not an external disturbance.

## Preconditions

- Sign and axis check passed after the last IMU or wiring change
  (`docs/axis-sign-check.md`): forward lean raises `pitch`, the pitch rate
  matches `d(pitch)/dt`, counter-clockwise gives positive `gz`, and the wheel
  signs are the bench-verified ones. A failed check is a hard stop.
- Mainboard awake: telemetry `mb=ok`, `age < 300 ms`, fresh battery voltage.
- Zero and level captured (`zcapt=1`, `lcapt=1`). Recapture `level` only while
  the body is still, and `forget` after any construction change.
- No persistent gain changes: `pscale` and `slew` are runtime-only and reset to
  the compiled defaults at boot. Apply them in every run.
- Start at `pscale 0.25`, `slew 100`. Never start at the compiled defaults.
  The first hold run at `slew 200` oscillated and latched; `slew 100` passed
  the hold gate with a well-fitted pitch law (`kp 6.2 Nm/rad`, `kd 1.36 Nm/rad`,
  R2 0.94). Keep `slew` fixed while stepping `pscale`.
- **Current status:** the ladder is blocked at 0.25. A 3 degree disturbance
  latched, a 1 degree disturbance failed on the second repetition, and the
  0.30 rung oscillated and latched during the hold gate. A first loaded
  dead-zone measurement puts it at roughly 40-60 units (0.20-0.30 Nm), about
  1.5-2 times the free-wheel value, but it is not a calibration: the hand-held
  rig changed the load during the ramp. Do not repeat the ladder until the
  dead zone is measured on a fixed-load fixture and `PlantParams::NOMINAL` is
  re-derived. See `docs/wp3-bench-log.md` session 8 and
  `docs/wp0-torque-identification.md`.

## Ladder

1. **Hold gate at 0.25.** Run with `--hold-only --pscale 0.25 --slew 200`.
   Acceptance, measured from the CSV: 10 s hold, `|pitch| < 2 deg`,
   `|x| < 0.1 m`, no torque saturation, no guard latch, `estop=0` until the
   driver ends the run.
2. **If the hold gate fails, stop.** Do not raise the gain. Score the recording:
   a stable oscillation period points to loop tuning, growing position drift
   points to the outer loop, and a large dead zone points to the unmeasured
   loaded breakaway. Fix the identified cause first.
3. **Commanded disturbance.** If the hold gate passes, run the same profile
   with `--angle 3 --trials 3 --hold-ms 500 --settle-ms 2500`. Record the CSV
   and score it. Require all three trials to recover with no fall and no latch.
4. **Raise in one step at a time:** `0.25 -> 0.30 -> 0.40 -> 0.60`, keeping
   `slew` fixed for comparability. Repeat steps 1-3 at each rung and score with
   `jbot-analyze`. Stop and latch on any guard, any saturation, or a worsening
   oscillation score.
5. **Do not exceed 0.60** without a separate Architect decision. `pscale` is a
   bench multiplier on `pitch_kp`/`pitch_kd` and the yaw differential; the fall
   guards and the torque budget do not scale with it.

## What this ladder does not solve

A tuned gain profile still needs a repeatable release fixture. The
rope-on-floor hand-held setup cannot hold a fixed pivot or release the body
without an impulse, which is why session 7 could never satisfy the baseline
precondition. Closing the WP3 gate requires both this ladder and that fixture.

## Related

- `docs/audit-2026-11-22.md` sections 3 and 5 for the stability floor and the
  original Phase 3/4 wording.
- `docs/wp3-bench-log.md` session 7 for the observed failure modes.
- `docs/release-fixture.md` for the fixture needed to make the baseline repeatable.
- `docs/plan-balance-and-tricks.md` for the WP3 gate and the Tier plan.
