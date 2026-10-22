# JBOT-WheeledBiped Development Roadmap

| Parameter | Value |
|---|---|
| Document | Development Roadmap |
| Code | RB-001 |
| Version | 1.0 |
| Date | 2026-10-21 |
| Status | Working document, synchronized with SRS-001 |
| Related documents | `docs/specification.md`, `docs/architecture.md`, `docs/system_requirements.md` |
| Scope | Full cycle: mathematics and simulation → test bench → onboard software → autonomous navigation → extensions |

---

## 0. How to read this document

### 0.1. Planning principles

1. **Model first, hardware second.** Each control loop is verified in simulation before power is applied to the motors.
2. **Early risk, early.** Long-lead components (actuators, lidar, RPi 5, battery adapter) are ordered in Phases 1–2.
3. **One critical path.** Balancing on the test bench is the main technical risk; everything else must not block it.
4. **Safety is not a phase.** E-stop, watchdog, limits, and logging are designed from Phase 1 and verified at every stage.
5. **Each phase ends with a measurable gate criterion.** Open P1 requirements are not silently carried over to the next phase.

### 0.2. Notation

- **P1 / P2 / P3** — requirement priorities from `docs/specification.md`.
- **Gate Gx** — phase acceptance gate; without passing it, the next phase is not considered started.
- **DoD** — Definition of Done, task completion criteria.
- **HIL** — Hardware-in-the-Loop, real MCU + real drivers + RPi.
- **WIP** — Wheeled Inverted Pendulum.

### 0.3. Indicative timeline

Estimates are given for a team of 2–3 engineers (mechanics/electronics, embedded, ROS/navigation) and are not a commitment. Key dependencies and risks matter more than the calendar plan.

| Phase | Duration | Key outcome | Gate |
|---|---|---|---|
| 1. Simulation, kinematics, balancing mathematics | 4–6 weeks | WIP model, LQR, simulator, stability report | G1 |
| 2. Power section, power supply, low-level test bench | 6–10 weeks | ESP32 + FOC + IMU + protocol, rotating test bench | G2 |
| 3. RPi 5, lidar, basic Rust software | 4–6 weeks | RPi–MCU link, telemetry, lidar, EKF, HIL | G3 |
| 4. SLAM, localization, navigation, autonomy | 6–10 weeks | Maps, costmap, planners, 100 autonomous runs | G4 |
| 5. Attachments, docking station, AI | 6–10 weeks | Camera/AI, manipulator, auto-charging, system tests | G5 |
| **Total** | **26–42 weeks** | Acceptance of the baseline version | — |

---

## 1. Initial state and scope

### 1.1. What has already been done

- [x] System requirements have been formulated (`docs/system_requirements.md`).
- [x] Architecture and safety concept have been designed (`docs/architecture.md`).
- [x] An SRS with requirement traceability has been created (`docs/specification.md`).
- [x] Cargo workspace initialized: `jbot_common`, `jbot_hardware`, `jbot_estimation`, `jbot_control`, `jbot_navigation`, `jbot_node`.
- [x] Type skeletons and test stubs added.

### 1.2. What is NOT included in the baseline version

- Stair flights and steps above 100 mm.
- Movement on water, sand, mud, and loose soil.
- Fully autonomous charging without a docking station.
- Manipulation operations as a mandatory function (interface and compatibility only).
- Certification for hazardous environments.

### 1.3. Critical path

```
WIP model → LQR in simulation → one hip QDD + one wheel on the test bench → all four actuators on the test bench
→ balancing on the safety frame → RPi–MCU link → EKF/odometry
→ SLAM and localization → planners → autonomous runs → extensions → acceptance
```

Running in parallel with the critical path: procurement, power design, safety analysis, CI, and documentation.

---

## 2. Phase 1. Simulation, four-bar kinematics, and balancing mathematics (LQR/WIP)

**Goal:** obtain a reproducible mathematical model of the robot and synthesize the balance controller before hardware assembly.

**Indicative duration:** 4–6 weeks.

### 2.1. Tasks

#### 1.1. Model parameters and assumptions

- [x] Fix mass-inertia parameters: body mass, wheels, legs, center-of-mass height, moments of inertia, wheel radius, track width.
- [ ] Determine actuator limits: maximum torque, speed, current, acceleration; assess sufficiency for recovery after a 10 N·s impulse.
- [ ] Describe the 1-DOF four-bar leg: link lengths A–B, A–C, B–D, C–D–E, hip gear ratio, height change range (target ≥ 100 mm), optional spring rate/travel, static load.
- [x] Fix assumptions: solid dry surface, no slip during synthesis, small angles during linearization.
- [x] Consolidate parameters in `config/robot.toml` with validation and versioning.

#### 1.2. Kinematics

- [x] Implement differential drive: `v = (v_l + v_r) / 2`, `ω = (v_r - v_l) / L`, and the inverse transform.
- [x] Implement four-bar leg kinematics (closed loop): forward/inverse solution, Jacobian, coupler curve, relationship of body height and fore-aft offset to hip angle; keep the 2-link model only as a comparison.
- [ ] Define coordinate frames `map`, `odom`, `base_link`, `imu_link`, `laser_link` in accordance with REP-105.
- [ ] Implement pose and velocity transforms between coordinate frames in `jbot_common`.
- [ ] Cover kinematics with unit tests: boundary angles, degenerate configurations, round-trip transforms.

#### 1.3. WIP dynamics

- [~] Derive the 2D WIP model as a reduced-order baseline using the Lagrangian method, then extend to 3D with roll, yaw, and the four-bar height/offset coupling.
- [x] Perform linearization around the vertical position and zero velocity.
- [x] Obtain state and control matrices:

  ```
  x = [x, ẋ, θ, θ̇]ᵀ
  ẋ = A·x + B·u
  u = [T_l, T_r]ᵀ  or  u = [F, τ_yaw]ᵀ
  ```

- [~] Verify controllability and observability; record the parameter range in which the system remains controllable.
- [x] Perform discretization with a 1 ms step (ZOH) for the 1 kHz loop.
- [ ] Account for limits: torque saturation, dead zone, friction, slip, 5–10 ms command delay.

#### 1.4. LQR synthesis and auxiliary loops

- [x] Implement the Riccati equation solution and `K` matrix (LQR) computation in `jbot_control` using `nalgebra`.
- [~] Set weights `Q` and `R`; formalize the tuning procedure and document the trade-off "recovery time / current / oscillation".
- [ ] Implement the velocity loop and yaw loop as outer loops relative to balancing.
- [ ] Implement a 25–35° tilt limit with transition to `DAMPING` and fall detection.
- [ ] Implement torque distribution between wheels accounting for saturation and balancing priority over yaw.
- [ ] Compare PD, LQR, and VMC; select the baseline algorithm and record an ADR.

#### 1.5. State estimation for simulation

- [ ] Implement a complementary filter and Madgwick for pitch/roll.
- [ ] Implement an EKF for velocity and bias estimation by fusing IMU and wheel odometry.
- [ ] Simulate IMU noise, zero bias, encoder quantization, and measurement delay.
- [ ] Provide for command delay estimation / compensation.

#### 1.6. Simulator

- [ ] Create the `jbot_sim` crate (or a `sim` module in the workspace) with a fixed step and deterministic PRNG.
- [ ] Implement an integrator (RK4) with a 0.1–1 ms step and time scaling for debugging.
- [ ] Implement a sensor model: IMU, encoders, current, battery voltage, lidar (simplified).
- [ ] Implement disturbance injection: 10 N·s impulse, 10° tilt, 10° slope, change in mass/CoM height.
- [ ] Implement recording and replay of traces for regression tests.
- [ ] Prepare visualization: `rerun` or equivalent; plots of current, tilt, velocity, trajectory.

#### 1.7. Model verification

- [ ] Verify the response to a step velocity command: overshoot, settling time, current.
- [ ] Verify recovery after a 10 N·s impulse (MOB-07) and a 10° deviation (MOB-08).
- [ ] Verify 3D recovery: roll deviation, yaw disturbance, and height step (MOB-31).
- [ ] Verify holding on a 10° slope (MOB-20/MOB-21) in simulation.
- [ ] Verify stability with a 10 ms delay, sensor noise, and torque saturation.
- [ ] Run a Monte Carlo over mass, friction, and CoM position spread.
- [ ] Record stability margins and admissible parameter ranges.

### 2.2. Phase artifacts

- `jbot_common`: state types, kinematics, coordinate frames, configuration.
- `jbot_control`: PD, LQR, limits, torque distribution.
- `jbot_estimation`: complementary filter, Madgwick, EKF skeleton.
- `jbot_sim`: plant, sensor, and disturbance model.
- `config/robot.toml` and simulation profiles.
- Report: "Model, linearization, and balance controller synthesis".
- ADR: choice of baseline algorithm (PD/LQR/VMC) and loop architecture.

### 2.3. Gate G1 — pass criteria

- [ ] The WIP model is reproducible; parameters are documented.
- [ ] The four-bar coupler curve and height range are validated; the 2D model is shown to be a valid reduced-order approximation of the 3D model.
- [ ] LQR stabilizes the model at zero command for at least 10 minutes of simulation without falling.
- [ ] Recovery after a 10 N·s impulse and a 10° deviation without falling.
- [ ] Current and torque do not exceed specified limits during typical maneuvers.
- [ ] The simulator is deterministic: the same seed and configuration produce the same trace.
- [ ] Reproducible tests and plots suitable for comparison with the real system exist.

### 2.4. Phase risks

| Risk | Impact | Mitigation |
|---|---|---|
| Incorrect mass-inertia parameters | Controller does not work on hardware | Refine parameters after assembly; stability margin reserve |
| Model ignores backlash and friction | Fall on the test bench | Friction identification; robust/adaptive additions |
| ESP32 does not provide 1 kHz with jitter < 50 µs | Phase 2 schedule slip | Early benchmark; fallback to STM32H7 |
| Too high CoM / small base | Physically uncontrollable | Iterate mechanics before purchasing final parts |

---

## 3. Phase 2. Assembly of the power section, power supply, and low-level test bench (ESP32 + 4 actuators)

**Goal:** assemble a safe power test bench with one hip QDD and one wheel hub, then all four actuators; configure the integrated QDD loops, start the IMU and communication protocol; obtain the first balancing platform on the safety frame.

**Indicative duration:** 6–10 weeks.

> **Important:** ESP32-S3 is used as a prototype MCU. It does not have native CAN FD (only TWAI/classic CAN), so classic CAN 1 Mbit/s or UART with COBS+CRC is allowed on the test bench, and for CAN FD an external controller (MCP2518FD and similar) over SPI is used. The RobStride-class hip QDDs use classic CAN 1 Mbit/s, not CAN FD. The production candidate is STM32H723/H743 (ADR-001); firmware and protocol must allow porting.

### 3.1. Tasks

#### 2.1. Power and protection design

- [ ] Develop the Milwaukee M18 connection scheme: adapter, reverse polarity protection, 40 A fuse (18 V baseline; 30 A at 24–36 V), TVS, precharge, main contactor.
- [ ] Calculate the power circuit for four actuators: 20–25 A continuous, 40 A fuse at 18 V, peak up to 55 A < 10 s; sag down to 15 V.
- [ ] Design DC-DC: 12 V / 5 A, 5 V / 5–6 A, 3.3 V / 1 A; efficiency ≥ 90%, soft-start, OVP, thermal shutdown.
- [ ] Provide a bidirectional shunt and power monitor (INA226 class) at 100–500 Hz.
- [ ] Design a two-channel hardware E-stop: disconnecting actuator power circuits while retaining 5 V / 12 V for logging and proper RPi shutdown.
- [ ] Perform routing: star ground, separation of power and logic ground, CAN shielding, ferrites, input protection.
- [ ] Prepare the schematic, BOM, wiring diagrams, and conduct a safety review.

#### 2.2. Test bench and power electronics

- [ ] Procure 2 hip QDDs (RobStride RS06/RS03 class) and 2 wheel hub motors/drivers; add an external FOC stage only if the wheel driver is not integrated.
- [ ] Assemble a test bench with one hip QDD and one wheel hub, using their native feedback plus a current sensor.
- [ ] Organize test bench power via a laboratory PSU with current limiting; then switch to the battery through protection.
- [ ] Provide an emergency button, fuse, and braking/bleeder element if needed.
- [ ] Check driver and motor heating under load; determine thermal limits and derating.
- [ ] Assemble the test bench frame with body fixation capability (safety stops/cable) for the first balance tests.

#### 2.3. Low-level firmware

- [ ] Create `firmware/` as a `no_std` crate; choose `embassy` (or `rtic`) and configure the build for ESP32-S3.
- [ ] Implement clock, timer, PWM, ADC, SPI/I2C, and watchdog initialization.
- [ ] Implement the ICM-42688-P IMU driver (SPI, DRDY, DMA) with timestamps at the sampling instant.
- [ ] Implement QDD feedback and wheel encoder reading and speed/position calculation.
- [ ] Configure the hip QDD integrated FOC and CAN protocol; implement FOC (electrical angle calibration, Park transforms, 20–40 kHz current loop) only for external wheel drivers.
- [ ] Implement current, torque, speed, and temperature limiting across all four actuators, including per-actuator and total limits.
- [ ] Implement the state machine `BOOT → SELF_TEST → IDLE → BALANCE → DAMPING → FAULT → E_STOP`.
- [ ] Implement the watchdog: IWDG, 100 ms command timeout, loss of RPi heartbeat → `DAMPING`.
- [ ] Implement a blackbox ring buffer of the last N seconds (P2).
- [ ] Ensure no dynamic memory allocation, `panic!`, or `unwrap()` in the control loop.

#### 2.4. Communication protocol

- [ ] Implement `jbot_protocol` (or a module in `jbot_common`) with a shared `no_std`-compatible frame format.
- [ ] Implement an 8-byte header: version, type, flags, sequence, timestamp; payload up to 56 bytes; CRC-16.
- [ ] Implement `postcard` serialization, little-endian, version and length validation.
- [ ] Implement the UART transport: COBS, CRC-16, resynchronization on `0x00`, max 256 bytes.
- [ ] Implement the actuator bus: classic CAN 1 Mbit/s for the RobStride QDDs (ESP32 TWAI or STM32 bxCAN/FDCAN in classic mode); provide an SPI CAN FD option for the RPi–MCU backbone.
- [ ] Implement messages: `ControlCommand`, `ModeCommand`, `EmergencyStop`, `Heartbeat`, `RobotState`, `ImuSample`, `JointFeedback`, `PowerFeedback`, `FaultReport`.
- [ ] Cover the protocol with round-trip and property tests (truncation, corrupted CRC, sequence overflow, unknown version).

#### 2.5. First balance tests

- [ ] Verify rotation direction, phasing, encoder operation, and torque sign on one hip QDD and one wheel hub.
- [ ] Verify the actuator current/torque loop (inside the QDD or external wheel driver): step command, overshoot, settling time, heating.
- [ ] Verify the velocity loop under inertial load.
- [ ] Verify hip height/roll response and the four-bar coupler curve on the bench.
- [ ] Assemble the four-actuator platform (2 hips + 2 wheels) on the test bench; verify odometry and direction of motion.
- [ ] Run PD balancing on the safety frame at zero command.
- [ ] Transfer the LQR parameters from Phase 1; compare behavior with simulation.
- [ ] Conduct tests for holding, impulse, 10° deviation, 10° slope (within test bench limits).
- [ ] Measure 1 kHz loop jitter (GPIO/DWT/oscilloscope) and end-to-end command delay.
- [ ] Conduct a 30-minute thermal run.

#### 2.6. Test bench safety

- [ ] Verify E-stop actuation with motors running; ensure logic remains powered.
- [ ] Verify behavior on loss of communication, power loss, brownout, overheating, overcurrent.
- [ ] Verify that after reset the drivers are disabled by default.
- [ ] Prepare safe test bench operating instructions and a first-power-on checklist.

### 3.2. Phase artifacts

- Schematics, BOM, wiring diagrams for the power section and test bench.
- `firmware/`: ESP32-S3 firmware (balance, CAN, IMU, protocol, safety; FOC only for external wheel drivers).
- `jbot_protocol`: shared frame and message format.
- Test bench with one and four actuators, safety frame.
- Test protocol: actuator loops, communication, jitter, delay, thermal mode, E-stop.
- ADR-001: MCU decision (ESP32-S3 prototype / STM32H7 production).
- ADR on actuators: hip QDD integration and wheel driver choice.

### 3.3. Gate G2 — pass criteria

- [ ] Both wheel hubs and both hip QDDs run stably; the hip QDD internal loop is configured and the external wheel FOC (if used) runs at 20–40 kHz.
- [ ] Four-bar hip motion covers the height range and the coupler curve matches the kinematic model.
- [ ] Balance loop 1 kHz; measured jitter ≤ 50 µs or a decision to migrate to STM32H7 is recorded.
- [ ] Communication between the RPi equivalent (PC) and MCU: command 100 Hz, telemetry 200–500 Hz, frame loss measured.
- [ ] Loss of a valid command > 100 ms leads to `DAMPING`.
- [ ] E-stop disconnects the power section, logging is preserved.
- [ ] The robot maintains balance on the safety frame at zero command for at least 10 minutes.
- [ ] Recovery after a 10 N·s impulse and 10° deviation is confirmed on the test bench.
- [ ] 30-minute thermal run without overheating of actuators and drivers.
- [ ] The communication protocol is versioned and covered by tests.

### 3.4. Phase risks

| Risk | Impact | Mitigation |
|---|---|---|
| ESP32-S3 cannot handle 1 kHz + CAN + jitter | Schedule slip | Early benchmark; migration to STM32H7; keep FOC inside the actuators |
| Interference from motors on IMU/CAN | Unstable estimation, bus errors | Shielding, ferrites, spacing, twisted pair, oscilloscope verification |
| Battery BMS shuts down under peak | Sudden power loss | Derating, precharge, current monitoring, tests with a real battery |
| Actuator phasing/calibration error | Jerk, overheating | Step-by-step QDD commissioning, current limiting, safety frame |
| Driver overheating | Shutdown, failure | Thermal calculation, heatsinks, derating, temperature monitoring |

---

## 4. Phase 3. Integration of Raspberry Pi 5, lidar, and basic Rust software

**Goal:** combine the real MCU and RPi 5 into a single system: transport, telemetry, lidar, state estimation, and supervisor. Balancing remains on the MCU.

**Indicative duration:** 4–6 weeks.

### 4.1. Tasks

#### 3.1. RPi 5 platform

- [ ] Install a 64-bit OS (Ubuntu 24.04 LTS arm64 or Raspberry Pi OS 64-bit) on NVMe.
- [ ] Configure cooling and temperature control; verify throttling under load.
- [ ] Configure 5 V / 5 A power with separate protection and short thick wiring.
- [ ] Configure the network, SSH/VPN, disable unnecessary services; isolate the robot network.
- [ ] Prepare systemd units for `jbot-node`, `zenohd`, the lidar driver, and the ROS2 bridge; `Restart=on-failure`, watchdog, resource limits.
- [ ] Make the root FS predominantly read-only, with a separate partition for maps and logs (P2).
- [ ] Configure cross-compilation for `aarch64-unknown-linux-gnu` (or musl) and CI builds.

#### 3.2. RPi–MCU transport

- [ ] Implement the `Transport` trait in `jbot_hardware`: `SocketCan`, `SerialCobs`, `Mock`.
- [ ] Connect CAN FD (HAT/adapter) and configure `socketcan`; verify 1 Mbit/s arbitration and data phase.
- [ ] Implement a backup UART at 921600/1 Mbit, 8N1, COBS + CRC-16.
- [ ] Implement frame reception/transmission, version, sequence, CRC, and timeout validation.
- [ ] Implement metrics: frame loss, CRC errors, bus-off, command age.
- [ ] Implement switching to the backup transport when the primary degrades.
- [ ] Implement RPi–MCU heartbeat and handshake; on loss — `DAMPING` on the MCU.

#### 3.3. Time synchronization

- [ ] Implement MCU–RPi time offset and drift estimation (EWMA) based on frame timestamps.
- [ ] Provide a GPIO sync pulse / 1 PPS (optional).
- [ ] Ensure timestamps at the sensor sampling instant, not at the processing instant.
- [ ] Verify IMU/odometry/lidar desynchronization ≤ 10 ms.

#### 3.4. Lidar

- [ ] Select a lidar: 2D 360°, ≥ 10 m, ≥ 10 Hz (RPLIDAR S2/A2M12, LDROBOT and similar) or a 3D lidar.
- [ ] Implement a Rust driver (or a wrapper over a high-performance driver) that publishes scans/point clouds.
- [ ] Bring measurements to the `laser_link` frame; verify angular and range calibration.
- [ ] Implement lidar health monitoring: rate, gaps, staleness, temperature.
- [ ] Implement raw scan recording for replay and regression tests.

#### 3.5. Basic `jbot_node` software

- [ ] Assemble the composition root: configuration, transport, estimation, telemetry publication, supervisor.
- [ ] Implement Tokio tasks per the architecture: ingress, estimation, control egress, telemetry, supervisor, shutdown.
- [ ] Implement latest-value semantics for commands (`watch`) and bounded queues for telemetry.
- [ ] Implement signal handling and graceful shutdown (`CancellationToken`).
- [ ] Implement TOML configuration with typing, validation, and hash logging.
- [ ] Implement structured logging (`tracing`, JSON) and metrics.
- [ ] Implement publication to Zenoh: state, telemetry, diagnostics, commands.
- [ ] Implement the `zenoh-bridge-ros2dds` bridge for RViz/rosbag (optional in this phase).

#### 3.6. State estimation

- [ ] Port/advance the EKF from Phase 1: IMU + wheel odometry, 100–200 Hz.
- [ ] Estimate pose, velocity, gyroscope zero bias; publish `odom → base_link`.
- [ ] Implement delay and outlier compensation; validate on HIL.
- [ ] Compare the estimate against a reference (test bench, external marker) and record errors.

#### 3.7. HIL and fault tolerance

- [ ] Assemble a HIL test bench: MCU + drivers + RPi + lidar (or lidar simulator).
- [ ] Verify fault injection: killing `jbot-node`, CAN/UART disconnection, lidar staleness, RPi reboot.
- [ ] Verify that an RPi failure does not cause a fall: the MCU transitions to `DAMPING`/`IDLE` on timeout.
- [ ] Verify boot to safe `IDLE` ≤ 10 s.
- [ ] Conduct a short soak test (2–4 hours) with metric logging.

### 4.2. Phase artifacts

- Configured RPi 5 image, systemd units, deployment instructions.
- `jbot_hardware`: CAN FD, UART, lidar, mock transport.
- `jbot_estimation`: EKF 100–200 Hz.
- `jbot_node` v0.1: telemetry, supervisor, configuration, logging.
- Lidar driver and scan recording pipeline.
- HIL protocol: delays, jitter, faults, recovery.
- Telemetry dashboard (Rerun/Zenoh) and a set of metrics.

### 4.3. Gate G3 — pass criteria

- [ ] End-to-end command delay RPi–MCU–actuator ≤ 10 ms (measured).
- [ ] Loss of heartbeat/command > 100 ms → `DAMPING`; an RPi failure does not cause a fall.
- [ ] Stable CAN FD (or UART) communication with telemetry at 200–500 Hz; losses and errors within budget.
- [ ] Lidar publishes data ≥ 10 Hz; desynchronization with IMU ≤ 10 ms.
- [ ] EKF runs at 100–200 Hz; pose and velocity are reproducible on replay.
- [ ] Boot to `IDLE` ≤ 10 s; logs and configuration are preserved.
- [ ] HIL fault tests passed; metrics are available to the operator.
- [ ] `cargo fmt`, `clippy -D warnings`, `cargo test` pass; aarch64 cross-build succeeds.

### 4.4. Phase risks

| Risk | Impact | Mitigation |
|---|---|---|
| CAN FD issues on RPi (driver/HAT) | Delays, losses | Early HAT selection, oscilloscope tests, backup UART |
| Time desynchronization | EKF/SLAM errors | Timestamps at sampling, offset estimation, GPIO sync |
| Lidar produces noise/little data | Poor SLAM | Choose a proven model, calibration, filtering |
| RPi throttling | Task gaps | Cooling, limits, frequency degradation |
| Zenoh/ROS2 bridge complexity | Integration delay | Start with Zenoh, add the bridge later; topic contracts |

---

## 5. Phase 4. Integration of SLAM, navigation, and autonomous movement

**Goal:** build a map, localize in it, and ensure autonomous point-to-point movement while respecting safety limits.

**Indicative duration:** 6–10 weeks.

### 5.1. Tasks

#### 4.1. SLAM selection and integration

- [ ] Conduct ADR-008: Rust-native SLAM vs `slam_toolbox`/Cartographer via the ROS2 bridge.
- [ ] Implement the pipeline: lidar → filtering → scan matching → map → pose.
- [ ] Implement loop closure and relocalization when returning to a known area.
- [ ] Implement map storage: resolution ≤ 5 cm, area ≥ 1000 m², PGM/YAML format or a documented equivalent.
- [ ] Implement map versioning and binding to the robot configuration.
- [ ] Implement map loading ≤ 10 s for 1000 m² (P2).
- [ ] Implement SLAM data recording/replay for regression.

#### 4.2. Localization

- [ ] Implement map localization: scan matching + EKF, 100–200 Hz for estimation, 10–30 Hz for lidar correction.
- [ ] Implement the `map → odom → base_link` tree per REP-105.
- [ ] Implement "kidnapping" detection and relocalization ≤ 10 s (P2).
- [ ] Implement degradation: on staleness/quality degradation — speed limiting, then stop.
- [ ] Verify accuracy: ≤ 50 mm and 2° (RMS) relative to reference markers.

#### 4.3. Obstacle maps and costmap

- [ ] Implement the static map, obstacle map, and inflation accounting for dimensions.
- [ ] Implement a dynamic obstacle layer based on lidar and data lifetime.
- [ ] Implement the staleness policy: data older than 300 ms → stop.
- [ ] Implement safe distance: < 0.25 m at low speed → stop.
- [ ] Implement speed limiting near obstacles (≤ 0.3 m/s).

#### 4.4. Planning and motion control

- [ ] Implement a global planner A*/Dijkstra accounting for traversability and dimensions.
- [ ] Implement a local planner/controller: DWA, MPC, or equivalent accounting for WIP dynamics.
- [ ] Account for limits: acceleration, jerk, tilt, torque, slip.
- [ ] Implement path following with speed limiting and smooth braking.
- [ ] Implement recovery behaviors: costmap clearing, replanning, turning, backing off.
- [ ] Implement teleoperation priority over autonomous movement.
- [ ] Implement the behavior layer: waypoints, patrol, pause, return, emergency stop.
- [ ] Isolate navigation in a separate process/task with resource limits.

#### 4.5. Autonomous tests

- [ ] First run scenarios in simulation with the WIP model and sensor noise.
- [ ] Build a room map ≥ 100 m² with loop closure; measure drift (≤ 1% of path).
- [ ] Perform 100 autonomous runs to the goal in a static environment; success rate ≥ 95%, collisions — 0.
- [ ] Verify dynamic obstacles: stop/replan ≤ 1 s.
- [ ] Verify lidar failure, data staleness, robot "kidnapping".
- [ ] Verify operation without ROS2 and with the ROS2/RViz bridge.
- [ ] Conduct regression on recorded logs; compare metrics with the reference.

### 5.2. Phase artifacts

- `jbot_navigation`: costmap, global/local planners, behavior layer.
- SLAM pipeline and map server with versioning.
- Map localization and REP-105 TF tree.
- A set of maps and configurations for the test site.
- Protocols: 100 runs, dynamic obstacles, degradation, localization accuracy.
- Navigation report and reproducible logs.

### 5.3. Gate G4 — pass criteria

- [ ] A map ≥ 100 m² is built with loop closure; drift ≤ 1% of path.
- [ ] Localization ≤ 50 mm and 2° (RMS); pose publication delay ≤ 100 ms.
- [ ] 100 autonomous runs: ≥ 95 successful, 0 collisions.
- [ ] Dynamic obstacles: stop/replan ≤ 1 s.
- [ ] Data staleness > 300 ms → stop; distance < 0.25 m → stop.
- [ ] Works without ROS2; the ROS2/RViz bridge does not block the safety loop.
- [ ] Navigation is isolated and does not affect balancing jitter.
- [ ] Log-based regression tests are reproducible.

### 5.4. Phase risks

| Risk | Impact | Mitigation |
|---|---|---|
| SLAM in Rust is not mature enough | Phase delay | Early ADR; fallback `slam_toolbox` via the bridge |
| WIP dynamics complicates the local planner | Falls during maneuvers | Acceleration/tilt limits, model-based MPC, simulation |
| Poor localization in long corridors | Mission failures | Loop closure, multiple sensors, landmarks |
| Dynamic obstacles | Collisions | Costmap layers, safe distance, stop priority |
| RPi resources under SLAM + navigation | Throttling, delays | Separate processes, limits, NVMe, cooling |

---

## 6. Phase 5. Enhancement of attachments, docking station, and AI functions

**Goal:** extend the platform with a camera, AI accelerator, manipulator, and auto-charging without compromising balancing, safety, and baseline navigation.

**Indicative duration:** 6–10 weeks.

### 6.1. Tasks

#### 5.1. Extension interfaces

- [ ] Implement mechanical and electrical interfaces for the camera (USB 3.0 / CSI-2, 5 V with protection).
- [ ] Implement the manipulator interface: 12 V / 5 A, protection, separate branch, CAN FD 29-bit ID.
- [ ] Provide a PCIe/USB reserve for the AI accelerator (Hailo-8L or equivalent), accounting for power and heat.
- [ ] Implement an external diagnostic isolated CAN port (P2).
- [ ] Ensure that extensions do not increase balancing loop jitter by > 50 µs.

#### 5.2. Camera and AI

- [ ] Connect an RGB-D camera (RealSense D435i / OAK-D), implement capture ≥ 30 fps at 640×480.
- [ ] Implement calibration, storage in configuration, and verification at startup.
- [ ] Implement camera synchronization with IMU/lidar ≤ 10 ms (P2).
- [ ] Choose the runtime: `ort`, `tract`, or `candle`; choose the accelerator or CPU.
- [ ] Implement object detection ≥ 10 Hz on the RPi 5 or AI accelerator (P2).
- [ ] Publish detections with timestamps; do not block navigation and control.
- [ ] Implement degradation: camera failure does not block lidar navigation.
- [ ] Verify the AI accelerator's power and thermal budget under sustained load.

#### 5.3. Manipulator (P3)

- [ ] Implement an emulator of the manipulator CAN node to verify IDs, power, and E-stop.
- [ ] Implement the control interface in `jbot_control`/the behavior layer, outside the 1 kHz loop.
- [ ] Implement supervisor limits when the load is extended (speed, torque, tilt).
- [ ] Implement emergency stop of the manipulator via the common E-stop.
- [ ] Verify that manipulator failure/disconnection does not compromise balancing and navigation.

#### 5.4. Docking station and auto-charging

- [ ] Design guides, power contacts, docking sensor, IR/CAN channel.
- [ ] Implement de-energizing of contacts until docking is confirmed; reverse polarity and short-circuit protection.
- [ ] Implement the state protocol: `IDLE`, `APPROACH`, `DOCKED`, `CHARGING`, `FAULT`.
- [ ] Implement the return-to-dock and charging mission under battery BMS control.
- [ ] Ensure docking accuracy ≤ ±20 mm and ±2° in heading.
- [ ] Limit docking station power ≤ 150 W.
- [ ] Implement charge monitoring, SoC/SoH, and event logging.
- [ ] Verify that the docking station is not mandatory for baseline operation.

#### 5.5. System validation

- [ ] Conduct a 24-hour run with battery replacement and scheduled pauses (P2).
- [ ] Verify E-stop, brownout, overheating, loss of communication, battery disconnection.
- [ ] Verify EMI after installing the camera, AI accelerator, and manipulator.
- [ ] Verify the thermal mode of all components in typical and aggressive cycles.
- [ ] Verify runtime: ≥ 30 min typical cycle, ≥ 20 min aggressive (P1).
- [ ] Update documentation: operator manual, maintenance, protocols, BOM.
- [ ] Conduct acceptance tests per `docs/specification.md`, section 12.

### 6.2. Phase artifacts

- Installed camera, AI accelerator, manipulator (or its emulator), and docking station.
- AI detection pipeline with timestamps and degradation.
- Docking station firmware/software and the auto-charging mission.
- Updated schematics, BOM, configurations, and calibrations.
- System and acceptance test protocols.
- Final report and delivery package.

### 6.3. Gate G5 — pass criteria

- [ ] Camera and AI operate without affecting balancing jitter and navigation.
- [ ] Failure of any extension does not compromise balancing and baseline navigation.
- [ ] The manipulator (or emulator) is controlled via CAN, the E-stop is shared, balancing is not compromised.
- [ ] Docking station: docking ≤ ±20 mm / ±2°, contacts de-energized until docking, charging under BMS.
- [ ] Return to dock and charging work as a mission.
- [ ] 24-hour run without critical failures.
- [ ] All P1 SRS requirements are confirmed by protocols; P2 ones are implemented or justified.
- [ ] The documentation and source code package has been delivered.

### 6.4. Phase risks

| Risk | Impact | Mitigation |
|---|---|---|
| AI accelerator power/heat | Brownout, throttling | Separate protection, power budget, tests under load |
| Manipulator compromises balancing | Fall | Supervisor, limits, isolation from the 1 kHz loop |
| Inaccurate docking | Charging failure | Guides, IR/CAN, multiple attempts, sensors |
| EMI from new nodes | Communication failures | Shielding, ferrites, spacing, spectrometer verification |
| AI integration complexity | Phase delay | Start with the CPU model, accelerator as optimization |

---

## 7. Cross-cutting work (performed in all phases)

### 7.1. Safety

- [ ] Hazard analysis and FMEA from the very beginning; update at every gate.
- [ ] E-stop specification: two-channel NC loop, disconnection of the power section, preservation of logic.
- [ ] Watchdog chain: hardware E-stop → MCU IWDG → 100 ms command timeout → RPi heartbeat → systemd → supervisor.
- [ ] Limits on current, temperature, tilt, voltage; latched faults and manual reset.
- [ ] Proof-test of E-stop and protections at least once every 12 months (P2).
- [ ] Safety audit before every field test.

### 7.2. Testing and CI

- [ ] `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test` on every PR.
- [ ] Protocol property tests: round-trip, corrupted frames, sequence wraparound.
- [ ] Integration tests: CAN/UART loopback, `vcan`, timeouts, failover.
- [ ] aarch64 cross-build and firmware build in CI.
- [ ] `cargo deny`, `cargo audit`.
- [ ] HIL tests and regression on recorded logs.
- [ ] Definition of Done for control loop changes: unit + HIL + jitter/delay + rollback.

### 7.3. Documentation and configuration

- [ ] Maintain ADRs for key decisions (MCU, drivers, SLAM, middleware, AI runtime).
- [ ] Version the protocol and maintain a firmware/host compatibility matrix.
- [ ] TOML configuration: typing, validation, hash logging.
- [ ] Calibrations in non-volatile memory; recovery after reboot.
- [ ] Structured logs (`tracing`) with node/task/robot ID; no high-frequency logging in the loop.
- [ ] Metrics: frame loss, CRC, bus-off, command age, jitter, voltage, temperatures, failures.
- [ ] MCU blackbox and log export (P2).

### 7.4. Procurement and production

- [ ] Form the BOM and order long-lead items in Phases 1–2: actuators, drivers, RPi 5, lidar, battery adapter, CAN FD HAT.
- [ ] Provide a spare kit of fuses, connectors, and consumables.
- [ ] Verify component availability and alternatives (second source).
- [ ] Manufacture/order PCBs and cable assemblies with margin.

### 7.5. Project management

- [ ] Maintain a risk register and update it at every gate.
- [ ] Maintain the requirements traceability matrix (SRS → task → test → protocol).
- [ ] Hold a weekly progress review and a monthly risk review.
- [ ] Record deviations from the SRS only through approval and an SRS addendum.

---

## 8. Milestones and dependencies

| Milestone | Content | Depends on |
|---|---|---|
| M0 | Baseline documentation and workspace | — |
| M1 | WIP model + LQR + simulator | M0 |
| M2 | One motor + FOC + IMU on the test bench | M1, actuator procurement |
| M3 | Two motors, balancing on the safety frame | M2 |
| M4 | RPi–MCU link, telemetry, lidar, EKF | M3, RPi/lidar |
| M5 | Map + localization | M4 |
| M6 | Autonomous runs ≥ 95/100 | M5 |
| M7 | Camera/AI, manipulator, docking station | M6 |
| M8 | Acceptance tests and delivery | M7 |

**Key dependencies:**

- Phase 2 cannot be completed without actuators, drivers, and the battery adapter.
- Phase 3 requires a stable protocol and firmware from Phase 2.
- Phase 4 requires working localization and lidar from Phase 3.
- Phase 5 requires free RPi resources and power/thermal margin.

---

## 9. Traceability matrix (summary)

| SRS section | Requirements | Phases |
|---|---|---|
| 3. Computing architecture | SYS-01…SYS-08 | 2, 3 |
| 4. Autonomy and power | AUT-01…AUT-30 | 2, 3, 5 |
| 5. Mobility and dynamics | MOB-01…MOB-30 | 1, 2, 4 |
| 6. Navigation | NAV-01…NAV-35 | 3, 4 |
| 7. Extensibility | EXT-01…EXT-33 | 5 |
| 8. Interfaces and software | INT-01…INT-22 | 2, 3, 4, 5 |
| 9. Safety | SAF-01…SAF-13 | all |
| 10. Reliability | REL-01…REL-08 | 2, 5 |

---

## 10. Project acceptance criteria

1. All P1 requirements are confirmed by test protocols.
2. P2 requirements are implemented or have an agreed deferral justification.
3. There are no critical failures or open safety defects.
4. Balancing loop jitter ≤ 50 µs; command loss → `DAMPING` within 100 ms.
5. Autonomous navigation: ≥ 95 successful runs out of 100 without collisions.
6. Runtime from M18 XC5.0 in a typical cycle ≥ 30 min.
7. The documentation and source code package has been delivered in full.
8. E-stop and protections have passed proof-test; failure behavior complies with the SRS.

---

## 11. Open questions affecting the plan

1. Final MCU: ESP32-S3 (prototype) → STM32H723/H743 (production)?
2. Actuators: hip QDD integrated (RobStride); wheel hub integrated or external FOC?
3. Leg topology: decided — 1-DOF four-bar. Remaining: link lengths, spring, height range, and E-position.
4. Lidar and camera: specific models, mounting, power, thermal budget.
5. SLAM: Rust-native stack or `slam_toolbox` via the ROS2 bridge?
6. AI: Hailo-8L or CPU-only; runtime `ort` / `tract` / `candle`.
7. Firmware update path: CAN bootloader / USB DFU / debug probe.
8. Docking station: contact standard and M18 charging protocol.
9. Mass, payload, and height range after prototype manufacturing.
10. Requirements for the enclosure, connectors, and maintainability.
11. Power pack: M18 18 V baseline vs 24–36 V preferred.

---

## 12. Immediate actions (first 2 weeks)

- [ ] Approve the model parameters and order long-lead components (actuators, RPi 5, lidar, CAN FD HAT).
- [x] Create `config/robot.toml` with WIP parameters and validation.
- [x] Implement differential drive kinematics and tests in `jbot_common`.
- [x] Implement the linearized WIP model and LQR in `jbot_control`.
- [ ] Create the `jbot_sim` skeleton with RK4 and trace recording.
- [ ] Conduct the first run: 10 min holding, 10 N·s impulse, 10° deviation.
- [ ] Prepare ADR-001 (MCU) and the ADR on drivers.
- [ ] Start hazard analysis and the E-stop specification.
