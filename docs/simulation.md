# 2D WIP Simulation Baseline

| Field | Value |
|---|---|
| Document | Deterministic fixed-step WIP simulation |
| Code | SIM-001 |
| Version | 0.1 |
| Date | 2026-10-22 |
| Status | Draft, Phase 1 baseline |
| Scope | Nonlinear 2D plant, RK4, LQR, disturbances, sensors, command delay, trace |
| Code map | `crates/jbot_sim/src/rk4.rs`, `crates/jbot_sim/src/wip_sim.rs`, `crates/jbot_sim/src/sensors.rs`, `crates/jbot_sim/src/rng.rs`, `crates/jbot_sim/src/report.rs` |

## 1. Purpose

`jbot_sim` is the first closed-loop simulator for the reduced-order WIP model.
It runs the nonlinear plant from `jbot_control` at a fixed step, applies the LQR
law, records every control period, and checks recovery from the Phase 1
disturbances: a 10 degree tilt and a 10 N*s horizontal impulse.

The simulator is deliberately small and deterministic. It is a verification
tool for the model, controller, and sensor pipeline, not a replacement for the
3D simulator, estimator validation, or hardware tests.

## 2. Contract

```rust
let config = SimulationConfig::with_slope(
    0.001,                  // dt, seconds
    16.0,                   // total torque from both wheels, N*m
    30.0_f64.to_radians(),  // fall limit
    0.0,                    // slope, positive uphill
)?;
let simulator = WipSimulator::new(plant, config)?;
let outcome = simulator.run(&lqr_controller, initial_state, 8.0, &impulses)?;

let sensors = SensorModel::new(sensor_config, 0.09, seed)?;
let estimator = EncoderImuEstimator::new(0.09)?;
let outcome = simulator.run_with_sensors(
    &lqr_controller,
    &estimator,
    &mut sensors,
    initial_state,
    8.0,
    &impulses,
)?;

let outcome = simulator.run_with_parameter_changes(
    &lqr_controller,
    initial_state,
    8.0,
    &impulses,
    &parameter_changes,
)?;
```

The run requires:

- `dt_s > 0` and finite.
- `duration_s > 0`, finite, and an integer multiple of `dt_s`.
- `max_total_torque_nm > 0` and finite.
- `0 < fall_limit_rad < pi/2`.
- `|slope_rad| < pi/2` and finite.
- Every impulse time inside `[0, duration_s)` and aligned to a step.
- A finite initial state and finite controller output.
- A finite sensor sample when the sensor path is active.

Violations return a typed `SimError`; the simulator never panics on invalid
input.

## 3. Time base and determinism

The simulator advances in exact fixed steps:

```text
t_k = k * dt
```

There is no wall clock or hidden global state. Plant integration itself uses no
random numbers; sensor noise uses an explicitly seeded `Pcg32` stream, and the
command-delay queue is local to a run. Two runs with the same plant,
configuration, initial state, controller, impulses, and sensor seed produce
bit-identical traces. `identical_inputs_produce_identical_traces` checks the
plant-only case, and `sensor_path_is_deterministic_for_a_seed` checks the noisy
sensor path, both with `PartialEq` over the whole trace.

The controller sees the state at the start of the period and holds the commanded
torque for the whole RK4 step. This matches the zero-order-hold assumption used
for the discrete LQR design. In the sensor path the controller instead sees the
delayed, noisy estimate supplied by `StateEstimate`, while the trace still
records the true plant state.

## 4. Plant and integration

The plant is `WheeledInvertedPendulum` from `jbot_control`. Its nonlinear
derivative is integrated with classical RK4:

```text
k1 = f(x_k, u_k)
k2 = f(x_k + dt/2 * k1, u_k)
k3 = f(x_k + dt/2 * k2, u_k)
k4 = f(x_k + dt * k3, u_k)
x_{k+1} = x_k + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
```

The baseline uses `dt = 1 ms`, matching the 1 kHz balance loop. The integrator
accepts any positive step; the Phase 1 tests use 1 ms. `rk4_matches_exponential_decay`
verifies the implementation against the analytic solution of `x_dot = -x` over
1 s.

The 2D model and its assumptions are documented in `docs/control_model.md`.

## 5. Actuator model

The LQR output is the total torque of both wheels. The simulator applies a
symmetric saturation:

```text
u_applied = clamp(u_requested, -u_max, u_max)
```

Each trace sample records whether the request was saturated. `command_delay_steps`
delays the actuator command by an integer number of control periods: the
simulator queues each requested torque and applies the value from
`command_delay_steps` steps earlier, holding zero during warm-up. Saturation is
applied after the delay, matching an actuator that clamps the command it
receives.

The actuator path also supports optional current and slew-rate limits, a dead
zone, and wheel friction:

```text
u_delayed = queue(request, command_delay_steps)
u_rate = move_toward(previous, u_delayed, rate_limit * dt)
u_shaped = dead_zone(u_rate)
u_applied = clamp(u_shaped, -min(torque_limit, current_limit * kt), +...)
friction = coulomb * sign(omega_wheel) + viscous * omega_wheel
u_plant = u_applied - friction
```

The trace records `u_applied`, while the plant integrates `u_plant`. Coulomb
friction is not regularized at zero wheel speed; it is intended for parameter
studies, not for stick-slip validation. The model still does not cover the
per-wheel `u/2` split, slip, spring compliance, thermal derating, or
torque-speed limits. Those belong to `M1-15`, `M1-20`, and the hardware phase.

## 6. Disturbances

### 6.1. Initial pitch

A tilt disturbance is applied through the initial state:

```rust
let initial = StateVector::new(0.0, 0.0, 10.0_f64.to_radians(), 0.0);
```

### 6.2. Horizontal force impulse

For an impulse `J` applied to the wheel axle at zero pitch, the instantaneous
velocity change is:

```text
delta_x_dot = J / a11
a11 = M_b + M_w + I_w / r^2
```

The simulator adds this velocity change at the aligned step before the
controller runs. The baseline Phase 1 check uses `J = 10 N*s` at `t = 0.1 s`.

### 6.3. Slope

A constant slope adds a gravity force along the travel direction:

```text
F_slope = (M_b + M_w) * g * sin(slope)
```

and shifts the inertial coupling angle from `theta` to `theta + slope`. `theta`
remains the body pitch from world vertical, so the LQR pitch target is still
zero. On a slope the steady-state solution leans uphill and accepts a small
position offset; the controller is not expected to hold `x = 0`.

### 6.4. Mass and CoM-height change

`ParameterChange` replaces the plant parameters from an aligned step onward:

```rust
let change = ParameterChange {
    at_s: 1.0,
    parameters: new_parameters,
};
```

The simulator validates every changed parameter set before the run starts and
then rebuilds the plant at the specified step. This covers changes in body
mass, wheel mass, inertias, wheel radius, CoM height, and gravity. The change is
deterministic and is stored in the record format.

## 7. Fall detection and trace

At every step the simulator records:

- time,
- full state `[x, x_dot, theta, theta_dot]`,
- applied torque,
- saturation flag.

If `|theta|` exceeds `fall_limit_rad`, the sample is recorded, `fell_at_s` is
set, and the run stops. Otherwise the run executes the full duration.

`Trace` exposes the sample slice, duration step, final state, maximum absolute
pitch, and maximum absolute torque.

### 7.1. Versioned records

`SimulationRecord` stores the plant parameters, simulation config, initial
state, duration, impulses, fall time, and trace. It writes and reads a
line-oriented text format with an explicit version line:

```text
jbot-sim-record 5
dt_s 0.001
...
samples 8000
time_s x_m x_dot_mps pitch_rad pitch_rate_radps torque_nm saturated
...
```

Version 5 adds the actuator limits and friction fields. Version 4 adds
`command_delay_steps`. Version 3 adds `parameter_changes`. Version 2 adds
`slope_rad`; version 1 is still accepted as a horizontal record with zero slope
and no parameter changes. Older versions default the newer fields to zero (or
`None` for optional limits). The parser validates the version, required fields,
finite values, impulse and
parameter-change counts, sample counts, and exact sample timestamps. Unknown
fields and trailing data are rejected. `SimulationRecord::replay` reconstructs
the plant and simulator from the record and reruns the controller.

The round trip and replay are covered by
`record_round_trips_and_replays_identically`: parsing a written record yields
an equal value, and replay reproduces an identical outcome. Version-1
compatibility, unsupported versions, and misaligned sample times have dedicated
tests.

### 7.2. CSV and SVG reports

`Trace::write_csv` emits the full state and applied torque, one row per control
period. `Trace::write_svg` produces a self-contained three-panel plot of pitch,
velocity, and torque without external dependencies. The SVG path downsamples to
about 2000 points so a long run remains viewable; the CSV keeps every sample.
Both methods work on the trace from a live run or from `SimulationRecord::replay`.

## 8. Baseline verification

The Phase 1 tests in `crates/jbot_sim/src/wip_sim.rs` show:

- The LQR gain stabilizes the nonlinear plant after a 10 degree initial tilt.
  The run does not fall, the final pitch is below `1e-3 rad`, and torque stays
  within the 16 N*m total limit.
- The plant recovers after a 10 N*s impulse at 0.1 s. The pitch exceeds
  `0.02 rad` (the disturbance is visible), does not reach the 30 degree fall
  limit, and returns below `1e-3 rad`.
- Torque saturation is enforced and reported.
- On a 10 degree uphill slope the controller stabilizes at a positive pitch
  offset and a bounded position offset. The 20 s run does not fall and stays
  within the 16 N*m total torque limit.
- A 20 percent body-mass and CoM-height increase at `t = 1 s` is absorbed
  without a fall; the final pitch returns below `1e-3 rad`.
- With a 10 ms command delay, IMU/encoder noise, a 5 N*m total torque limit,
  and a 10 N*s impulse at `t = 1 s`, the closed loop does not fall, saturates
  the command, keeps the pitch peak below `0.35 rad` (20 degrees), and returns
  the final pitch below `0.02 rad`.
- On a 10 degree slope, with noisy sensors, friction, and a 5 ms command delay,
  the loop holds for 600 s. The final pitch stays between `0.06` and `0.1 rad`
  and the position stays within 1 m. This does not include wheel slip or motor
  thermal models.
- A 0.5 m/s velocity step with a 10 N*m torque limit and a 100 A current limit
  settles within 5 percent in 1.0 s, peaks at 0.588 m/s, and finishes at
  0.507 m/s. Peak torque is 4.19 N*m (41.9 A at the configured 0.1 N*m/A).
- Repeated runs are bit-identical.

These checks validate the controller on the 2D nonlinear plant and on a noisy,
delayed sensor/actuator path. They do not cover wheel slip, motor thermal
behavior, yaw, or estimator bias compensation.

## 9. Sensor models

`SensorModel` converts the true plant state and applied torque into one
timestamped `SensorSample`. It is deliberately separate from the plant so tests
can inject ideal or noisy measurements without changing the dynamics.

The sample contains:

- IMU pitch and pitch rate with optional constant bias and Gaussian noise.
- Left and right wheel position/velocity with optional noise and quantization.
- Total motor current derived from torque through `motor_torque_constant_nm_per_a`,
  plus battery voltage from nominal voltage and internal resistance.
- A simplified planar lidar range to an optional wall, clamped to the sensor
  range and optionally noisy.

All parameters live in `SensorConfig`. Noise is zero by default; the default
battery and motor values are simulation placeholders, not hardware limits. A
non-zero `measurement_delay_steps` delays every sample by an integer number of
ticks while preserving the original sample timestamp. During buffer warm-up the
oldest available sample is returned.

The PRNG is a small PCG32 implementation in `crates/jbot_sim/src/rng.rs`. It
takes a caller-provided seed and has no global state. Tests cover seed
reproducibility, seed sensitivity, uniform/normal ranges, bias, quantization,
delay, lidar clamping, and configuration validation. The noise stream is
deterministic for a given toolchain and platform; cross-platform bit identity is
not guaranteed because the Gaussian transform uses `ln` and `cos`.

The current model emits the same wheel angle and velocity for both sides because
the plant is planar; differential wheel data and IMU accelerometer axes belong
to the 3D model.

## 10. Limitations

- Only the 2D fore-aft plant; no roll, yaw, or four-bar height/offset coupling.
- No leg inertia, spring, friction, slip, or wheel-level torque allocation.
- Sensor models cover the planar state only; both wheels report the same values,
  and there are no accelerometer axes, CAN latency, or dropout models.
- Current is derived from torque with a fixed torque constant. The current
  limit is a static torque cap; there is no thermal or torque-speed derating.
- Command delay is a fixed integer number of steps with no jitter, loss, or
  estimation/compensation. Friction is a lumped wheel model without stick-slip
  regularization.
- The battery model is a nominal voltage source with internal resistance; it has
  no state of charge, sag dynamics, or protection behavior.
- The record format is text-only; no compression or streaming writer.
- No time scaling for interactive debugging.
- Reports are static CSV/SVG exports; there is no interactive viewer or live
  telemetry integration.

## 11. Next increments

1. Validate the estimator and delay compensation against the modeled bias and
   delay (`M1-22`, `M1-25`).
2. Add slip and spring-compliance studies (`M1-15`) and yaw tracking (`M1-18`).
3. Assemble the Gate G1 report from the trace exports and metrics (`M1-37`).
