# WP0: measured torque scale and breakaway (2026-11-22)

## Setup

The robot was mechanically fixed with both wheels free in the air. The first
levels were taken through the Rust console's MOTOR TEST panel; the full series
was then run unattended by `spin-driver`, which sent `test <u>` commands, read
the 200 Hz telemetry stream and wrote `csv/jbot-spin-1795388868571.csv`.

Method: `M = I * alpha` per wheel with `I = 0.0078 kg m^2` from
`config/robot.toml`. Acceleration is a least-squares slope of wheel speed over
the **beginning** of each torque plateau, while the wheels are still below
20 rad/s. The middle of a high-command plateau is unusable because the
torque-speed envelope already reduces the available torque there.

## Results

| Command, units | alpha, rad/s^2 | Torque, Nm | units/Nm |
|---:|---:|---:|---:|
| 60 (wake) | 25.60 | 0.1996 | 300.5 |
| 39 | 12.70 | 0.0990 | 393.8 |
| 48 | 21.90 | 0.1708 | 281.0 |
| 60 | 31.40 | 0.2449 | 245.0 |
| 75 | 46.14 | 0.3599 | 208.4 |
| 99 | 68.33 | 0.5330 | 185.7 |
| 120 | 86.36 | 0.6736 | 178.2 |
| 150 | 113.01 | 0.8815 | **170.2** |

- Breakaway bracket: motion starts at or below 39 units; the affine fit gives
  an intercept of about **26 units** under these free-wheel conditions. This is
  not the loaded dead zone of 120-160 units estimated from the bench runs.
- The small-command ratios are inflated because a fixed breakaway term is a
  large fraction of the command. The asymptotic ratios at 99/120/150 units are
  **170-186 units/Nm**, consistent with the earlier spin-up estimate of
  175-185 units/Nm.
- Taking 175 units/Nm as the working value gives a full-scale budget of
  `1000 / 175 ≈ 5.7 Nm` common-mode, close to the 5.0 Nm used in
  `PlantParams::NOMINAL`. There is no evidence here for the 25-40 Nm that a
  static lying recovery would need.

## Consequences for the plan

1. The WP1 negative result stands: no strategy recovered from 45 deg at budgets
   up to 40 Nm in the nonlinear model, and the measured budget is about 5.7 Nm.
   Physical self-righting and backflips remain out of scope.
2. No gain or threshold change is justified by this measurement: the design
   assumption of about 5 Nm is confirmed. Tier 1 work (position hold, spin,
   controlled fall) can continue in WP2 code without changing the torque model.
3. The measured breakaway applies to free wheels only. A first loaded
   measurement was made later (see below); it is approximate, not yet a basis
   for dead-zone compensation.

## Loaded dead zone (first estimate, 2026-11-22)

A command-driven ramp with the wheels on the floor and the robot held upright
was recorded as `csv/jbot-spin-1795394390161.csv` (`spin-driver --levels
10,...,120 --hold-ms 500`). Plateau hall speeds by command level:

| Command (units) | Plateau wheel speed (rpm) |
| --- | --- |
| 10-20 | 0 |
| 30-40 | 8-9, intermittent |
| 50-60 | about 23, consistent |
| 70-100 | 0-18, inconsistent |
| 110-120 | 56-81, consistent |

Separately, `jbot-analyze` reported a loaded breakaway of 47-61 units
(0.23-0.30 Nm nominal) on three armed recovery recordings in the same session.
Combining the two, the loaded dead zone is **roughly 40-60 units (0.20-0.30 Nm)**,
about 1.5-2 times the free-wheel value of 26 units.

This is an estimate, not a calibration: the 70-100 unit readings are
non-monotonic because the hand-held rig changed the normal load and wheel
contact during the run. A fixture with a fixed, known load is needed before
this number enters `PlantParams::NOMINAL` or dead-zone compensation is enabled.

## Tooling note

During WP0, closing the host serial port left the Pico's USB CDC console task
unable to resume: the write into an undrained endpoint blocked and each later
host session saw no telemetry until the Pico USB cable was re-enumerated.
This was not part of WP0. The final change bounds every console write with a
timeout and drops telemetry lines on timeout, so a host that closes mid-line
cannot park the task. A pure DTR gate blocked the console after a Pico
re-enumeration, and the first bounded-write variant left the endpoint unusable;
both were reverted. See `docs/wp3-bench-log.md` for the root cause. The final
bounded-write fix was flashed on 2026-11-23 and three close/reopen cycles were
confirmed to resume telemetry without touching the USB cable.

Quality gates after the tooling changes: `cargo fmt --all -- --check`,
`cargo check --workspace --all-targets`, `cargo test --workspace` and
`cargo clippy --workspace --all-targets -- -D warnings` all pass.
