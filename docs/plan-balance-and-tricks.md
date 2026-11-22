# Plan: reliable balance and the full trick set

Decision thresholds in this plan are proposals, not measurements. They must be
revisited after WP0 supplies a measured torque budget. Technical claims about
existing code and public projects are sourced; scope boundaries are judgement
calls and are marked as such.

## 1. Scope: "all tricks" is three different problems

**Tier 1 — reachable with the present two-wheel machine.** Stable upright hold,
position hold on slopes, driving, spin/pivot in place, rejection of a commanded
disturbance, a controlled soft fall, and (still unproven) a dynamic self-righting
manoeuvre.

**Tier 2 — needs a verified torque budget.** A backflip and fast recovery from a
supine pose. Static analysis in `docs/design_balance_home.md` section 6.1 needs
25-40 Nm; the measured budget is about 5.5 Nm. `I_MOT_MAX` is 10 A and the
config itself says to move towards 15 A only after measuring MOSFET and pack
temperature. This tier is blocked until WP0 produces a measured number.

**Tier 3 — needs new actuators.** Jumping, step climbing, recovery from a full
side or upside-down pose. The closest public blueprint
(`oyzlight/mujoco-lqr-vmc-wheel-legged-robot`) uses four hip actuators plus two
wheels, 7 Nm hip / 2 Nm wheel peaks, LQR + VMC at 200 Hz with 0.5 ms physics,
and still excludes full side/upside-down recovery. It is simulation-only.

## 2. Present blockers, from repository facts

* Torque scaling is only partially identified: spin-up gave 175-185 command
  units/Nm versus 200 units/Nm in the design (`TORQUE_LIMIT=1000`,
  `max_torque_nm=5.0`); the loaded dead zone is provisionally 120-160 units.
  The `home 3` run at `pscale=0.30, slew=100` reported about 80 units and
  diverged, which is below that provisional dead zone.
* The plant in `firmware/pico_core/src/plant.rs` is linearized around upright:
  `lift_limit=53.1 deg`, guard at 30 deg, `LYING_RAD=45 deg`. It has dead zone
  and transport delay but no large-angle geometry or motor speed/torque envelope.
* There is no open-loop `test <u>` command for torque identification and no
  command-driven disturbance source; manual pushes were abandoned as
  unreproducible.
* Arm/fall used to have only two states. WP2 adds a mode enum, a guard grace
  period and a separately filtered guard tilt in pure, tested code (not yet
  flashed).
* Flashed `VARIANT_USART` falls back to `DEFAULT_STEER_COEFFICIENT=8192` (0.5).
  `Command::from_wheel_torque` used to invert the mixer assuming 1.0, so yaw
  differential was half of the request; WP2 corrects the inverse in code (not
  yet flashed). See `docs/decision-yaw-authority.md`.
* Serial timeout is about 0.8 s in source and about 2 s to full stop; the
  100 ms design target in `config/robot.toml` is not met.
* `docs/design_balance_home.md` section 2 uses `L=0.18 m`, while
  `config/robot.toml` and section 10 use `L=0.25 m`. The geometry must be
  reconciled before any torque number is treated as exact.

## 3. Work packages

### WP0 — Measure the drive (bench, no control-law change)

The open-loop commands already exist: `test <u> [u]` in
`firmware/pico2w/src/console.rs` and the host driver
`archive/python-tools/pico/jbot_torque_id.py`, which spins the wheels free and derives
`M = I*alpha` from the hall feedback. Hold the robot by hand on the bench and
run that driver over several levels; it writes results into `csv/`. Derive the
loaded dead zone and units/Nm with an uncertainty band; if possible cross-check
with a known lever arm and force gauge. Reconcile `L`; update
`config/robot.toml` and `PlantParams::NOMINAL`, then re-derive `Tuning` and
record the result in the audit document.

An offline pass over the four existing telemetry sessions was also made. The
breakaway fit is inconclusive there: the drive was commanded while already
rolling, giving overlapping brackets of 52-96 units (0.26-0.48 Nm nominal),
and the fitted pitch gain depends on the `pscale` used in each run. Fresh
spin-up data is therefore required rather than re-fitting old logs.

**Gate:** a measured common-mode torque budget. If it differs from 5 Nm by
more than 20 %, every gain and every fall threshold is re-derived before the
next trial.

### WP1 — Nonlinear simulation (PC only, no flashing)

Extend the plant with `sin(pitch)` instead of the linear term, a motor
speed/torque envelope, battery sag, dead zone and transport delay. Keep a
parity test against the present linear model near zero so existing results
remain comparable. Implement, as pure functions, both public recovery patterns:

* `StandUp`: linear pitch-reference ramp from the measured angle to zero with
  rate feedforward and yaw disabled (mjbots hoverbot);
* `Throw`: accelerate in the fall direction, reverse abruptly, then let the
  balance loop catch the body, with reduced authority in the mid-angle band
  (BalancingWii).

Sweep start angles 0-90 deg and torque budgets 5, 10, 20, 40 Nm. Publish a
feasibility map with travel and peak speed, checked against the existing
3 m and 3 m/s guards.

**Gate:** one strategy succeeds at the measured WP0 budget within the guards,
or a documented negative result that closes Tier 1 self-righting.

**Result:** `docs/wpi-simulation-results.md`. Neither strategy recovered from
45 deg or more at any budget up to 40 Nm, so the gate is not met. The simulation
used the nominal 5 Nm class as well as larger budgets; WP0 must still measure
the real drive. No physical raise trial is authorised by this result.

### WP2 — Control code (development and unit tests, no flashing)

Add an explicit mode enum (`Balance`, `Drive`, `Spin`, `Raise`, `SafeFall`,
`Fault`), a guard grace period for `Raise`, a separate filtered tilt for the
guard while keeping the control path on its 12 Hz-filtered rate, the yaw
authority fix (either set the mixer inverse to 0.5 or change the mainboard
coefficient to 1.0 - an explicit design decision), and the telemetry fields
needed to observe a raise. All decisions stay in the tested pure
`SafetyState`/`BalanceController` functions. Run the four standard Rust
quality gates before any flash.

**Gate:** 100 % tests, zero warnings, and a written yaw-authority decision.

The written decision is in `docs/decision-yaw-authority.md`. Option 1 was
approved and implemented: the Pico mixer inverse now matches the flashed
`STEER_COEFFICIENT = 0.5`, with a unit test modelling the mainboard mixer. The
mode enum, guard grace hook and separately filtered guard tilt are also in
place with unit tests. The guard now reads a separate 4 Hz filtered tilt while
the control path keeps its 12 Hz rate filter. All of this was flashed to the
Pico on 2026-11-22 and verified to boot and stream telemetry; the mainboard was
left off. The guard filter's effect on fall latency must still be observed in
the first armed trial. The same reflash carries a USB console fix:
`console_task` now gates writes on DTR, with a two-second grace for a host that
does not assert it after a Pico re-enumeration; closing and reopening the port
was confirmed to resume telemetry without USB re-enumeration. A pure DTR gate
and a bounded-write variant were both tried and reverted; see the WP3 log.

### WP3 — Bench trials (each session needs fresh step-by-step approval)

Wake the mainboard, `clear`, reapply `pscale`/`slew`, capture `level` if needed.
Disturbances must be command-driven, never manual pushes. Record 15 s of
telemetry with automatic `estop`. Rope and hands on the body, test zone at most
0.5 m, minimum torque, smooth ramp and damping.

**Gate:** repeatable recovery from N identical commanded disturbances with no
fall and no latch.

### WP4 — Tricks rollout, in this order

1. Spin/pivot - needs the WP2 yaw fix.
2. Controlled soft fall - needs the `SafeFall` mode.
3. Dynamic self-righting - needs WP1 and WP3.
4. Backflip - only if WP0 measures enough torque and WP1 confirms it in the
   nonlinear model.

### WP5 — Tier 3 morphology

Only if jumping or full side/upside-down recovery is required: add actuators
(legs or a reaction wheel) and move to an LQR + VMC architecture following the
wheel-legged reference. This is a new design, not an extension of the
two-wheel law.

## 4. Decision gates at a glance

| Condition | Consequence |
|---|---|
| Measured common torque below 10 Nm | Backflip and supine recovery are off scope; concentrate on Tier 1 |
| Stand-up or throw succeeds in WP1 at the measured budget and under 3 m / 3 m/s | Allow one supervised raise trial in WP3 |
| Measured yaw differential is half the request | Apply the WP2 mixer/coefficient fix before any spin work |
| 30 deg recovery still fails after WP0-WP2 | Stop gain tuning; identify the motor torque constant with a load cell first |
| Any Tier 3 trick is requested | Open a new mechanical/control design; do not stretch the two-wheel law |

## 5. Approval boundary

WP1 is PC-only and can start without touching hardware. Flashing, any change of
gains, torque limits or `I_MOT_MAX`, and every physical raise trial need fresh
step-by-step approval. Nothing is pushed without explicit approval.

## 6. Sources

Local: `docs/design_balance_home.md`, `docs/audit-2026-11-22.md`,
`docs/research-balancing-tricks-2026-11-22.md`, `config/robot.toml`,
`firmware/pico_core/src/{control,plant,eferu}.rs`,
`firmware/pico2w/src/{console,main,state}.rs`,
`firmware/stm32f103/eferu-jbot-config.h`.

External: `github.com/mahowik/BalancingWii`, `github.com/mjbots/hoverbot`,
`github.com/oyzlight/mujoco-lqr-vmc-wheel-legged-robot`,
`github.com/Deng-ge-open-source/RL_SelfRisingRobot_StackForce`,
`arxiv.org/abs/2606.14270`, `ieeexplore.ieee.org/document/8979168`.
