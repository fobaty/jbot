# WP1 results: nonlinear recovery simulation (2026-11-22)

## What was built

`crates/jbot_analysis/src/recovery_sim.rs` (host-only) and the
`recovery-sweep` binary. The plant uses the full nonlinear two-body pendulum
equations on level ground with the combined-wheel geometry from
`config/robot.toml`, plus a 0.5 Nm dead zone, a 2-tick transport delay and a
linear torque-speed envelope (40 rad/s no-load wheel speed). The controller in
the loop mirrors `Tuning::from_plant` and `BalanceController::update` from
`jbot_pico_core`, including the 12 Hz rate filter equivalent, slope trim,
integrator and the 200 Hz period. A unit test pins the derived coefficients to
the documented values (A1 25.49, A2 4.73, A1/A2 5.39, kp 21.57, kd 5.92).

Two strategies were swept:

* `StandUp`: linear pitch-reference ramp from the measured angle to zero with
  rate feedforward and yaw disabled (the mjbots hoverbot pattern).
* `Throw`: accelerate in the fall direction, reverse abruptly, hand back to
  the balance loop (the BalancingWii pattern), with four parameter sets.

Run with:

```sh
cargo run -p jbot_analysis --bin recovery-sweep --release
```

Sweep: budgets 5/10/20/40 Nm, start pitches 5/10/20/30/45/60/75 deg,
settling criterion 5 deg and 0.1 m/s for 0.5 s, guards 3 m travel and 3 m/s
peak speed.

## Feasibility map: largest start pitch that settled

| Strategy | 5 Nm | 10 Nm | 20 Nm | 40 Nm |
|---|---:|---:|---:|---:|
| StandUp, 60 deg/s | 20 deg | 20 deg | 20 deg | 20 deg |
| StandUp, 90 deg/s | 20 deg | 30 deg | 30 deg | 30 deg |
| StandUp, 120 deg/s | 20 deg | 30 deg | 30 deg | 30 deg |
| StandUp, 180 deg/s | 20 deg | 30 deg | 30 deg | 30 deg |
| StandUp, 360 deg/s | 30 deg | 30 deg | 30 deg | 30 deg |
| Throw, 4/2 rad/s, 0.5 s | 10 deg | 20 deg | 30 deg | 30 deg |
| Throw, 8/4 rad/s, 1.0 s | 10 deg | 20 deg | 30 deg | 30 deg |
| Throw, 12/6 rad/s, 1.5 s | - | 20 deg | 20 deg | 20 deg |
| Throw, 20/10 rad/s, 2.0 s | - | 30 deg | 30 deg | 30 deg |

Representative rows (final pitch, travel, peak speed, peak applied torque,
duration):

* 5 Nm, StandUp 360 deg/s, start 30 deg: 0.29 deg, 2.56 m, 1.75 m/s,
  4.50 Nm, 6.1 s - successful but close to the 3 m travel guard.
* 10 Nm, StandUp 120 deg/s, start 30 deg: 0.41 deg, 1.92 m, 1.47 m/s,
  5.66 Nm, 7.5 s - successful.
* 40 Nm, StandUp 360 deg/s, start 45 deg: -15.14 deg, 3.01 m, 2.24 m/s,
  10.87 Nm, 1.6 s - travel guard hit.
* 40 Nm, Throw 20/10 rad/s, start 45 deg: 51.42 deg, 0.98 m, 3.01 m/s,
  39.43 Nm, 0.6 s - speed guard hit.

## Interpretation and gate decision

1. **No strategy recovered from 45 deg or more at any budget up to 40 Nm.**
   At 45 deg the attempts end on the travel guard (stand-up) or the speed
   guard (throw); at 60/75 deg they end almost immediately at the 85 deg
   simulation stop. The WP1 self-righting gate is therefore **not met**, and
   the plan's next step (one supervised physical raise trial) is **not
   authorised by simulation**.
2. **More torque alone does not buy a lying recovery in this model.** Above
   10 Nm the envelope saturates the same way for every budget; the binding
   limits are travel and speed, not torque. This differs from the linear,
   guard-free sweep in `docs/design_balance_home.md` section 6.1 (40 Nm gave
   85 deg). The difference is the action of the real guards plus the torque
   envelope and delay, which that sweep did not include.
3. **The throw pattern is not better than a fast stand-up ramp here.** Its
   best result equals the 30 deg ceiling, and it spends much more torque to
   get there. The crude open-loop phase logic may be part of the reason, so
   this is not a proof that momentum recovery is impossible.
4. **30 deg is reachable at the present 5 Nm budget**, with 2.56 m of travel
   under the 3 m guard. That is consistent with the existing 30 deg fall
   guard being the right design boundary, not an overly cautious one.

## What this does and does not prove

The model has no body-to-ground contact, no wheel slip, no current-loop
dynamics and no thermal limit. Results above about 75 deg are optimistic by
construction. The dead zone (0.5 Nm), delay (2 ticks) and torque constant are
still estimates; WP0 must measure them on the bench. A negative simulation
result closes the simulated route, not the physical question, but it is a
sufficient reason not to attempt a physical raise yet.

## Next steps

1. WP0: measure units/Nm, loaded dead zone and the real torque budget.
2. If the measured budget is much above 5 Nm, re-run this sweep with the
   measured value before revisiting self-righting.
3. Extend the model with body-ground contact if a lying recovery is still
   required, and treat the throw phases as tuned parameters rather than fixed
   constants.
4. Do not flash, change gains or attempt a physical raise on the basis of
   this document.

Quality gates after this change: `cargo fmt --all -- --check`,
`cargo check --workspace --all-targets`, `cargo test --workspace` (36 + 7 + 30
tests), and `cargo clippy --workspace --all-targets -- -D warnings` all pass.
