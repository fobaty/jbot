# Task Breakdown — JBOT-WheeledBiped

| Parameter | Value |
|---|---|
| Document | Detailed task breakdown (Task Breakdown) |
| Code | TB-001 |
| Version | 1.0 |
| Date | 2026-10-21 |
| Status | Working document |
| Basis | `docs/specification.md`, `docs/roadmap.md`, `docs/architecture.md`, `docs/system_requirements.md` |
| Scope | Full lifecycle: math and simulation → power stage → MCU firmware → RPi 5 drivers → navigation and SLAM → extensions → acceptance |

---

## 0. Rules for working with the plan

### 0.1. Identifier structure

- `M1` — math, kinematics, dynamics, control and simulation.
- `M2` — power stage, power supply, protection and energy.
- `M3` — low-level balance controller and MCU firmware in Rust.
- `M4` — sensor drivers and transport on Raspberry Pi 5.
- `M5` — Raspberry Pi 5 onboard software: composition, estimation, supervisor, middleware.
- `M6` — navigation, localization and SLAM.
- `M7` — safety and fault tolerance (cross-cutting module).
- `M8` — integration, HIL, system and acceptance tests.
- `M9` — extensions: camera, AI, manipulator, docking station.
- `M10` — cross-cutting work: CI, documentation, configuration, procurement, management.

Format: `M<module>-<number>`.

### 0.2. Statuses and priorities

- `[ ]` — not started; `[~]` — in progress; `[x]` — done; `[!]` — blocked.
- **P1** — mandatory for acceptance of the base version.
- **P2** — target requirement; implemented by the end of the corresponding phase or with a justified deferral.
- **P3** — interface reservation or prospective implementation.
- **Gate G1…G5** — phase acceptance gate; tasks included in a gate cannot be closed partially.

### 0.3. Definition of Done for tasks

1. Code builds without errors and warnings for the target platform.
2. Unit tests exist; for the protocol and parsers — property tests.
3. For control-loop tasks, jitter and latency are measured, and results are attached to the task.
4. Configuration, calibrations and documentation are updated.
5. `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test` pass.
6. For hardware tasks, there is a bench test protocol and recorded measurements.
7. For safety tasks, a review has been performed and the traceability matrix updated.
8. The change can be rolled back; the protocol and firmware version are compatible with the host.

### 0.4. Critical path

```text
M1 WIP model and LQR
  → M3 single hip QDD + wheel
  → M3 four actuators and balance on the safety frame
  → M4 RPi–MCU transport and lidar
  → M5 EKF and supervisor
  → M6 SLAM and localization
  → M6 planners and autonomous runs
  → M9 extensions
  → M8 acceptance
```

In parallel: `M2` (power), `M7` (safety), `M10` (CI, documentation, procurement).

### 0.5. Module summary

| Module | Phases | Tasks | Estimate, person-days | Key result | Gate |
|---|---|---:|---:|---|---|
| M1. Math, dynamics and simulation | 1 | 38 | 30–45 | WIP model, LQR, simulator | G1 |
| M2. Power stage and supply | 1–2, 5 | 24 | 35–50 | Safe power circuit, DC-DC, E-stop | G2 |
| M3. Balance controller and MCU firmware | 1–2 | 45 | 50–70 | FOC, 1 kHz balance, protocol, bench | G2 |
| M4. Sensor drivers and RPi 5 transport | 3 | 27 | 30–45 | CAN/UART, lidar, camera, synchronization | G3 |
| M5. RPi 5 onboard software | 3 | 26 | 35–50 | `jbot_node`, EKF, supervisor, Zenoh | G3 |
| M6. Navigation and SLAM | 4 | 39 | 55–75 | Maps, localization, costmap, planners | G4 |
| M7. Safety and fault tolerance | all | 16 | 12–18 | Safety case, E-stop, watchdog, fault codes | all |
| M8. Integration and testing | 3–5 | 22 | 30–45 | HIL, system and acceptance protocols | G3–G5 |
| M9. Extensions | 5 | 31 | 40–60 | Camera/AI, manipulator, docking station | G5 |
| M10. Cross-cutting work | all | 22 | 18–28 | CI, documentation, procurement, management | all |
| **Total** | | **290** | **335–486** | | |

> Estimates are ideal person-days for a team of 2–3 engineers. The calendar plan and buffers are defined separately in `docs/roadmap.md`.

---

## 1. M1. Math, kinematics, dynamics and simulation

**Goal:** obtain a reproducible robot model, synthesize the balance controller and verify it in simulation before powering the motors.

**Inputs:** `docs/specification.md` sections 3, 5, 6; `docs/architecture.md` sections 1, 3, 7.
**Outputs:** `jbot_common`, `jbot_control`, `jbot_estimation`, `jbot_sim`, `config/robot.toml`, model and stability report.

### 1.1. Parameters, assumptions and configuration

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M1-01 | Fix the mass-inertial parameters: mass of the body, wheels and legs, CoM height, moments of inertia, wheel radius, track width | P1 | — | Parameters are consolidated into a table, with the source and uncertainty of each value indicated |
| [ ] M1-02 | Determine actuator limits: maximum torque, speed, current, acceleration; verify sufficiency for a 10 N·s impulse | P1 | M1-01 | The calculation confirms recovery after 10 N·s with margin; limits are entered into the configuration |
| [ ] M1-03 | Describe the 1-DOF four-bar leg: base A–B, upper crank A–C, lower rocker B–D, wheel fork C–D–E, hip gear ratio, height range of at least 100 mm, optional spring rate/travel, static load | P1 | M1-01 | Design and parameters are described; the coupler curve and height/fore-aft coupling are documented; whether the hip needs a 1 kHz loop is determined |
| [ ] M1-04 | Fix the model assumptions: rigid dry surface, no slip during synthesis, small linearization angles | P1 | M1-01 | Assumptions and validity limits are recorded in the report |
| [ ] M1-05 | Implement `config/robot.toml`: typed schema, range validation, versioning, hash logging | P1 | M1-01…M1-04 | Invalid config is rejected with a clear error; hash is written to the log at startup |

### 1.2. Kinematics and coordinate frames

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M1-06 | Implement forward and inverse kinematics of the differential drive: `v`, `ω`, `v_l`, `v_r`, track width | P1 | M1-01 | Unit tests for forward and inverse transforms, including zero speed and in-place rotation |
| [ ] M1-07 | Implement four-bar leg kinematics: FK, IK, Jacobian, coupler curve, relationship of body height and fore-aft offset to hip angle (2-link model only for comparison) | P1 | M1-03 | Tests for boundary angles and singular configurations; IK returns an error outside the workspace; computed height range is at least 100 mm |
| [ ] M1-08 | Define coordinate frames `map`, `odom`, `base_link`, `imu_link`, `laser_link` per REP-105 | P1 | M1-06 | Axis diagram and transform rules are documented; directions follow ROS conventions |
| [ ] M1-09 | Implement pose and velocity transforms between coordinate frames in `jbot_common` | P1 | M1-08 | Round-trip tests for poses and velocities; no error accumulation on a typical trajectory |
| [ ] M1-10 | Cover kinematics with tests: boundary angles, degenerate configurations, saturation, velocity signs | P1 | M1-06, M1-07, M1-09 | `cargo test` is green; all public kinematics functions are covered |

### 1.3. WIP dynamics

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M1-11 | Derive the 2D WIP model as a reduced-order baseline using the Lagrangian method, then extend to 3D with roll, yaw, and four-bar height/offset coupling | P1 | M1-01, M1-06 | Equations and intermediate derivations are documented in the report; both models are compared; dimensions are consistent |
| [ ] M1-12 | Linearize the model in the vicinity of the vertical and zero speed; obtain matrices `A`, `B` | P1 | M1-11 | Matrices are reproduced from configuration parameters; verified by numerical linearization |
| [ ] M1-13 | Verify controllability and observability; determine the parameter range where the system is controllable | P1 | M1-12 | Report contains matrix ranks and parameter bounds; there is a test for loss of controllability |
| [ ] M1-14 | Perform ZOH discretization with a 1 ms step for the 1 kHz loop | P1 | M1-12 | Discrete model is compared with the continuous one; stability preservation test |
| [ ] M1-15 | Account for non-idealities: torque saturation, dead zone, friction, slip, optional spring compliance, 5–10 ms command delay | P1 | M1-12 | Non-ideality model is parameterized and verified in simulation |

### 1.4. Controller synthesis

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M1-16 | Implement the Riccati equation solution and LQR matrix `K` calculation in `jbot_control` on `nalgebra` | P1 | M1-14 | Test on a known system; `K` is stable, closed-loop eigenvalues inside the unit circle |
| [ ] M1-17 | Set weights `Q` and `R`; document the tuning procedure and the "recovery time / current / oscillation" trade-off | P1 | M1-16 | Table of runs and selected weights; justification in the report |
| [ ] M1-18 | Implement outer speed and yaw loops with anti-windup and output limiting | P1 | M1-16 | Step response without sustained overshoot; limits are respected |
| [ ] M1-19 | Implement a tilt limit of 25–35°, fall detection and transition to `DAMPING` | P1 | M1-16 | Threshold exceedance test; transition occurs before loss of stability |
| [ ] M1-20 | Implement torque distribution between wheels taking into account saturation and balance priority over yaw | P1 | M1-16, M1-18 | When one wheel saturates, balance is preserved; yaw degrades predictably |
| [ ] M1-21 | Compare PD, LQR and VMC; select the base algorithm and write an ADR | P1 | M1-17, M1-18 | ADR contains comparison metrics and justification of the choice |
| [ ] M1-26 | Implement 3D allocation: height via common hip angle, roll via differential hip height; verify coupled four-bar limits | P1 | M1-07, M1-16 | Height and roll commands respect coupler-curve limits; balance is preserved in simulation |

### 1.5. State estimation for simulation and HIL

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M1-22 | Implement a complementary filter and Madgwick for pitch and roll | P1 | M1-01 | Tests on synthetic signals; pitch error within the specified value |
| [ ] M1-23 | Implement the EKF framework for estimating velocity and displacement by fusing IMU and wheel odometry | P1 | M1-06, M1-22 | EKF converges on a synthetic trajectory; covariances retain positive definiteness |
| [ ] M1-24 | Simulate IMU noise, zero bias, encoder quantization and measurement delay | P1 | M1-22 | Models are parameterized; tests of robustness to noise and bias |
| [ ] M1-25 | Implement command delay estimation and compensation | P2 | M1-15, M1-23 | 10 ms delay is compensated; degradation under incorrect estimation is limited |

### 1.6. `jbot_sim` simulator

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M1-26 | Create the `jbot_sim` crate with a fixed step and deterministic PRNG | P1 | M1-12 | The same seed and configuration produce a bit-identical trace |
| [ ] M1-27 | Implement an RK4 integrator with a 0.1–1 ms step and time scaling for debugging | P1 | M1-26 | Test on an analytical solution; energy does not grow on a passive system |
| [ ] M1-28 | Implement sensor models: IMU, encoders, current, battery voltage, simplified lidar | P1 | M1-24, M1-26 | Models produce data with timestamps and configurable noise |
| [ ] M1-29 | Implement disturbance injection: 10 N·s impulse, 10° tilt, 10° slope, change in mass and CoM height | P1 | M1-26 | Disturbances are reproducible and parameterized |
| [ ] M1-30 | Implement trace recording and replay for regression tests | P1 | M1-26 | Replay produces an identical result; format is versioned |
| [ ] M1-31 | Prepare `rerun` visualization or equivalent: current, tilt, speed, trajectory | P2 | M1-30 | Plots are generated from the recorded trace without manual processing |

### 1.7. Model verification and Gate G1

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M1-32 | Verify the response to a step speed command: overshoot, settling time, current | P1 | M1-18, M1-26 | Metrics within acceptable limits; plots attached |
| [ ] M1-33 | Verify recovery after a 10 N·s impulse and a 10° deviation | P1 | M1-29 | Both scenarios complete without falling; current within limits |
| [ ] M1-34 | Verify holding on a 10° slope in simulation | P1 | M1-29 | Holding for at least 10 min without slipping and model overheating |
| [ ] M1-35 | Verify stability with 10 ms delay, sensor noise and torque saturation | P1 | M1-24, M1-25 | The system is stable in all combinations; bounds are documented |
| [ ] M1-36 | Run Monte Carlo over the spread of mass, friction and CoM position | P1 | M1-35 | The probability of falling and acceptable parameter ranges are determined |
| [ ] M1-37 | Record stability margins and acceptable ranges; prepare the report "Model, linearization and balance controller synthesis" | P1 | M1-32…M1-36 | Report contains plots, tables and conclusions; suitable for comparison with the real system |
| [ ] M1-38 | Conduct Gate G1 review and update the traceability matrix | P1 | M1-32…M1-37 | All G1 criteria are confirmed; open P1 requirements are not silently deferred |

---

## 2. M2. Power stage, supply and protection

**Goal:** design and manufacture a safe power subsystem: M18, protection, DC-DC, E-stop, battery monitoring.

**Inputs:** `docs/specification.md` sections 4, 9, 10; `docs/architecture.md` section 4.
**Outputs:** schematics, BOM, wiring diagrams, power bench, measurement protocols, power monitor calibrations.

### 2.1. Design and calculation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-01 | Develop the M18 input circuit: adapter, reverse polarity protection, 40 A fuse (18 V baseline; 30 A at 24–36 V), TVS, precharge, main contactor | P1 | M1-01 | Circuit has passed review; ratings are justified by calculation |
| [ ] M2-02 | Calculate the four-actuator power circuit: 20–25 A continuous, 40 A fuse at 18 V, peak up to 55 A < 10 s, sag down to 15 V; coordinate protections | P1 | M2-01 | Calculation and protection coordination are documented; margin of at least 20% |
| [ ] M2-03 | Design DC-DC 12 V / 5 A, 5 V / 5–6 A, 3.3 V / 1 A with efficiency ≥ 90%, soft-start, OVP, thermal shutdown | P1 | M2-02 | Circuits and thermal calculation are ready; selected components are available |
| [ ] M2-04 | Design a bidirectional shunt and an INA226-class power monitor at 100–500 Hz | P1 | M2-02 | Circuit includes Kelvin connection and filtering; range and accuracy correspond to AUT-19, AUT-20 |
| [ ] M2-05 | Design a two-channel hardware E-stop: disconnecting actuator power circuits while preserving 5 V and 12 V | P1 | M2-01 | Circuit complies with SAF-02, SAF-03, AUT-30; logic remains powered |
| [ ] M2-06 | Produce the grounding, shielding and EMI plan: star point, separation of power and logic ground, shields, ferrites, TVS | P1 | M2-03, M2-05 | Plan has been verified on a prototype; critical loops and spacing are described |
| [ ] M2-07 | Prepare the schematic, BOM, wiring diagrams and conduct a safety review | P1 | M2-01…M2-06 | Documentation set is ready; review comments are closed |

### 2.2. Manufacturing and bring-up

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-08 | Lay out and order PCBs for the power stage and power monitoring; perform assembly | P1 | M2-07 | Boards are manufactured; visual and electrical inspection passed |
| [ ] M2-09 | Manufacture cable assemblies, connectors and harnesses taking into account currents, cross-sections, protection and service access | P1 | M2-07 | Harnesses are verified for integrity and voltage drop under load |
| [ ] M2-10 | Perform bus bring-up without load: power-up sequence, levels, ripple, efficiency, thermal conditions | P1 | M2-08, M2-09 | All buses are within tolerance; ripple and efficiency measured and recorded |
| [ ] M2-11 | Verify buses under load: RPi 5, lidar, camera, fans, actuators | P1 | M2-10 | Sags within limits; no protection tripping under typical load |

### 2.3. Battery monitoring and energy

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-12 | Calibrate voltage and current measurement: error ≤ ±0.5% and ≤ ±2% of full scale | P1 | M2-04, M2-10 | Calibration with a reference instrument; coefficients saved |
| [ ] M2-13 | Implement coulomb-counting SoC with voltage correction, error ≤ ±5% | P1 | M2-12 | Comparison with a reference wattmeter on a discharge profile |
| [ ] M2-14 | Implement SoH estimation, accumulated energy and charge/discharge logging | P2 | M2-13 | Events and energy are saved; data available in telemetry |
| [ ] M2-15 | Implement thresholds 16.0 / 15.5 / 15.0 V with 500 ms confirmation and independence of decisions from RPi | P1 | M2-12 | Tests with a voltage simulator; MCU makes decisions autonomously |
| [ ] M2-16 | Provide derating before the battery BMS protection trips: limiting acceleration, torque and speed | P1 | M2-15 | Load profile does not cause spontaneous BMS shutdown |
| [ ] M2-17 | Implement power telemetry: voltage, current, SoC, SoH, temperature at a rate of at least 50 Hz | P1 | M2-12, M2-13 | Data is published over CAN and in Zenoh with timestamps |
| [ ] M2-18 | Log threshold events with timestamp, current and voltage | P1 | M2-15 | Events are saved on power loss and available for analysis |

### 2.4. Power stage testing and safety

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-19 | Verify E-stop with motors running; confirm logic power is preserved and RPi shuts down correctly | P1 | M2-05, M2-11 | Power stage is disconnected; logs are preserved; RPi shuts down normally |
| [ ] M2-20 | Verify brownout, overcurrent, reverse polarity, short circuit and BMS behavior | P1 | M2-11 | No scenario leads to a dangerous state; failures are recorded |
| [ ] M2-21 | Conduct thermal and EMI runs of the power stage under typical and peak load | P1 | M2-11 | Temperatures within limits; EMI does not disturb CAN, IMU and lidar |
| [ ] M2-22 | Verify runtime on M18 XC5.0: ≥ 30 min typical, ≥ 20 min aggressive, ≥ 45 min idle (P2) | P1 | M2-16 | Protocol with load profile, temperature, SoC and SoH |
| [ ] M2-23 | Provide service access to fuses, connectors and battery; battery replacement ≤ 2 min without tools | P1 | M2-08, M2-09 | Verified on the assembled robot; documented in the manual |
| [ ] M2-24 | Prepare power FMEA, protection proof-test procedure and first power-up checklist | P1 | M2-19…M2-22 | Documents approved; proof-test is reproducible |

---

## 3. M3. Low-level balance controller on Rust/ESP32

**Goal:** implement deterministic MCU firmware: hip QDD CAN control, wheel actuator loops (integrated or external), IMU and encoder acquisition, 1 kHz 3D balance loop, communication protocol, watchdog and safe states.

**Platform:** ESP32-S3 as a prototype; STM32H723/H743 as a production candidate. Code and protocol must allow porting without changing control logic.

**Inputs:** `docs/architecture.md` sections 3.2, 5, 7; `docs/specification.md` sections 3, 5, 8, 9.
**Outputs:** `firmware/`, `jbot_protocol`, bench with one and four actuators, actuator/jitter/latency/E-stop protocols.

### 3.1. Firmware architecture and HAL

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-01 | Benchmark ESP32-S3 and STM32H7; select the MCU and write ADR-001 | P1 | M1-02 | Actuator/CAN, 1 kHz, jitter measured; decision recorded |
| [ ] M3-02 | Create `firmware/` as a `no_std` crate; configure target, `embassy` or `rtic`, build and memory map | P1 | M3-01 | Firmware builds, flashes and starts on the selected board |
| [ ] M3-03 | Implement initialization of clocks, timers, PWM, ADC, SPI, I2C, GPIO and watchdog | P1 | M3-02 | All peripherals pass self-test at startup |
| [ ] M3-04 | Ensure safe default states: actuators disabled after reset, outputs in a known state | P1 | M3-03 | Verified with an oscilloscope and logic; complies with SAF-10 |

### 3.2. Sensors and actuators

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-05 | Implement the ICM-42688-P IMU driver: SPI, DRDY, DMA, timestamp at sample time | P1 | M3-03 | Data is read at 1–4 kHz; timestamp corresponds to the sample time |
| [ ] M3-06 | Implement self-test, gyroscope and accelerometer bias calibration, calibration persistence | P1 | M3-05 | Calibration survives reboot; error within acceptable limits |
| [ ] M3-07 | Implement QDD feedback and wheel encoder reading, position and speed calculation taking into account direction and resolution | P1 | M3-03 | Direction and sign of speed match physical rotation |
| [ ] M3-08 | Implement acquisition of current, voltage, temperatures and bus current via ADC synchronized to PWM | P1 | M3-03, M2-12 | Measurements are synchronous; noise and offset within acceptable limits |

### 3.3. FOC and actuator loops

> The hip QDDs integrate FOC and the current loop. M3-09…M3-16 apply fully to external wheel drivers; for the hip QDDs they reduce to configuring the vendor loop, verifying CAN scaling, and commissioning.

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-09 | Implement FOC math: Clarke, Park, inverse Park, SVPWM | P1 | M3-02 | Unit tests for transforms; amplitude and phase are preserved |
| [ ] M3-10 | Implement electrical angle calibration and phase order verification | P1 | M3-07, M3-09 | Calibration is reproducible; incorrect phase order is detected |
| [ ] M3-11 | Implement a 20–40 kHz current loop with current limiting and integrator saturation | P1 | M3-09, M3-10 | Step response without sustained oscillations; overshoot within tolerance |
| [ ] M3-12 | Implement a 1–2 kHz speed loop with acceleration and speed limiting | P1 | M3-11 | Response under inertial load; stability under load changes |
| [ ] M3-13 | Implement limits on current, torque, speed and temperature with derating | P1 | M3-11, M3-12 | Exceeding limits leads to limiting, then to a fault |
| [ ] M3-14 | Perform single-actuator bring-up on one hip QDD and one wheel hub: rotation direction, phase order, encoder, torque sign | P1 | M3-10…M3-13 | All signs and directions match; no jerks or overheating |
| [ ] M3-15 | Verify the current loop: step, overshoot, settling time, heating | P1 | M3-11 | Metrics attached; heating within limits |
| [ ] M3-16 | Verify the speed loop under inertial load | P1 | M3-12 | Metrics attached; stability confirmed |

### 3.4. Balance loop and motion

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-17 | Implement the 1 kHz 3D balance task with a fixed period, without dynamic memory allocation | P1 | M3-05, M3-07, M3-12 | Period is stable; jitter measured; no heap or blocking in the loop |
| [ ] M3-18 | Port PD/LQR from `jbot_control`; take parameters from configuration | P1 | M1-16, M3-17 | Behavior on the bench matches simulation within acceptable limits |
| [ ] M3-19 | Implement the state machine `BOOT → SELF_TEST → IDLE → BALANCE → DAMPING → FAULT → E_STOP → SHUTDOWN` | P1 | M3-03 | Transitions are validated; invalid commands do not change state |
| [ ] M3-20 | Implement tilt/roll limiting, fall detection and transition to `DAMPING` | P1 | M3-17, M3-19 | Configurable tilt threshold 25–35° and roll limit; transition before the dangerous angle |
| [ ] M3-21 | Implement torque distribution between wheels and hip height/roll allocation, with saturation and balance priority | P1 | M3-18 | When saturated, balance is preserved; yaw and roll degrade predictably |
| [ ] M3-22 | Implement processing of speed and yaw commands: latest-value, validity check, acceleration limiting | P1 | M3-19 | Stale and invalid commands are ignored; acceleration is limited |
| [ ] M3-23 | Implement hip height and roll control outside the 1 kHz balance loop (four-bar, P2) | P2 | M3-07, M3-17 | Height is adjustable by at least 100 mm; roll is reachable within coupler-curve limits; balance loop does not degrade |
| [ ] M3-24 | Implement four-actuator platform odometry on the MCU and publish it in telemetry | P1 | M3-07 | Odometry matches the reference on straights and turns |
| [ ] M3-25 | Start PD balancing on the safety frame with a zero command | P1 | M3-18, M3-19 | Holding for at least 10 min without falling |
| [ ] M3-26 | Move LQR to the bench and compare with simulation | P1 | M3-25 | Differences are explained; parameters refined |
| [ ] M3-27 | Conduct bench tests: 10 N·s impulse, 10° deviation, 10° slope | P1 | M3-26 | All scenarios without falling; current and temperature within limits |
| [ ] M3-28 | Measure 1 kHz jitter and end-to-end latency; if jitter > 50 µs, record the transition to STM32H7 | P1 | M3-25 | Measurement protocol with oscilloscope/GPIO; decision documented |
| [ ] M3-29 | Conduct a 30 min thermal run on the bench | P1 | M3-27 | Actuators and drivers without overheating; derating does not trip falsely |

### 3.5. Communication protocol

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-30 | Implement `jbot_protocol` with a `no_std`-compatible 8-byte header, payload up to 56 bytes and CRC-16 | P1 | M3-02 | Header is fixed; version, type, flags, sequence and timestamp are validated |
| [ ] M3-31 | Implement messages via `postcard` and `serde`: commands, state, IMU, joints, power, faults | P1 | M3-30 | Schema is versioned; incompatible major version is rejected |
| [ ] M3-32 | Implement UART transport: COBS, CRC-16, resynchronization on `0x00`, maximum 256 bytes | P1 | M3-30 | Corrupted and truncated frames are recovered; error counters increase |
| [ ] M3-33 | Implement CAN: classic 1 Mbit/s on ESP32 TWAI for the bench; provide for CAN FD via SPI or STM32 FDCAN | P1 | M3-30 | Exchange works on the bench; CAN FD path is verified or documented |
| [ ] M3-34 | Implement the full message set: `ControlCommand`, `ModeCommand`, `EmergencyStop`, `Heartbeat`, `RobotState`, `ImuSample`, `JointFeedback`, `PowerFeedback`, `FaultReport` | P1 | M3-31 | All messages pass round-trip; IDs and rates correspond to the architecture |
| [ ] M3-35 | Cover the protocol with round-trip, property and fuzz tests: truncation, corrupted CRC, sequence overflow, unknown version | P1 | M3-31, M3-32 | `proptest` is green; invalid frames do not panic |
| [ ] M3-36 | Implement processing of sequence, duplicates and stale commands | P1 | M3-34 | Duplicates are idempotent; old commands are not applied |
| [ ] M3-37 | Ensure exchange with the PC: command 100 Hz, telemetry 200–500 Hz; measure losses and latency | P1 | M3-34 | Losses within budget; metrics available |

### 3.6. Safety, diagnostics and firmware operation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-38 | Implement IWDG, 100 ms command timeout and loss of RPi heartbeat with transition to `DAMPING` | P1 | M3-19, M3-34 | Loss of communication > 100 ms leads to `DAMPING`; IWDG resets a hung firmware |
| [ ] M3-39 | Implement fault codes, latched faults, manual reset and self-test before reset | P1 | M3-19 | Reset is possible only when the robot is stationary and self-test succeeds |
| [ ] M3-40 | Implement a blackbox ring of the last N seconds with upload over CAN/UART (P2) | P2 | M3-38 | Data is preserved on failure; upload is reproducible |
| [ ] M3-41 | Conduct an audit: absence of heap, `panic!` and `unwrap()` in the loop; add `defmt` logging | P1 | M3-17 | Static analysis and review confirm the absence of prohibited constructs |
| [ ] M3-42 | Implement saving calibrations to non-volatile memory and restoration after reboot | P1 | M3-06, M3-10 | Calibrations are not lost; corrupted data is detected |
| [ ] M3-43 | Conduct fault injection on the bench: CAN/UART disconnection, brownout, overheating, overload, E-stop | P1 | M3-38, M3-39 | All faults lead to a safe state; events are logged |
| [ ] M3-44 | Implement the firmware update and rollback path: USB DFU or debug probe on the prototype, CAN bootloader in the future | P2 | M3-02 | Update without disassembly; failed update is rolled back |
| [ ] M3-45 | Prepare firmware documentation: memory map, pinout, protocol, version, compatibility matrix | P1 | M3-34, M3-44 | The document allows building and flashing the board from scratch |

---

## 4. M4. Sensor drivers and transport in Rust for Raspberry Pi 5

**Goal:** ensure reliable data reception from the MCU, lidar and camera, a unified time base and sensor health.

**Inputs:** `docs/architecture.md` sections 5, 6.3, 6.5, 6.9; `docs/specification.md` sections 6.1, 7.2, 8.1.
**Outputs:** `jbot_hardware`: transport, lidar and camera drivers, health metrics, replay, systemd deployment.

### 4.1. Raspberry Pi 5 platform

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-01 | Install a 64-bit OS on NVMe; configure cooling, 5 V / 5 A power, network, SSH/VPN and isolation | P1 | M2-11 | Boot from NVMe; no throttling under typical load; network is isolated |
| [ ] M4-02 | Prepare systemd units for `jbot-node`, `zenohd`, drivers and bridge; `Restart=on-failure`, watchdog, resource limits | P1 | M4-01 | Services start after boot; crashes are restarted; logs are available |
| [ ] M4-03 | Configure `aarch64-unknown-linux-gnu` or musl cross-compilation and CI build | P1 | M4-01 | Artifact is built in CI and runs on RPi |
| [ ] M4-04 | Make the root FS mostly read-only, with a separate partition for maps and logs (P2) | P2 | M4-01 | Reboot does not lose configuration; logs and maps are written to a separate partition |

### 4.2. RPi–MCU transport

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-05 | Implement the `Transport` trait with `SocketCan`, `SerialCobs`, `Mock` implementations | P1 | M3-34 | All implementations pass a common test suite; `Mock` is deterministic |
| [ ] M4-06 | Connect the CAN FD HAT/adapter, configure `socketcan`, verify 1 Mbit/s arbitration and 5 Mbit/s data | P1 | M4-05, M2-11 | Stable exchange; termination and stub length verified with an oscilloscope |
| [ ] M4-07 | Implement backup UART 921600/1 Mbit, 8N1, COBS + CRC-16 | P1 | M4-05, M3-32 | Exchange is stable; corrupted frames are recovered |
| [ ] M4-08 | Implement frame reception/transmission with validation of version, sequence, CRC, length and timeouts | P1 | M4-05 | Invalid frames are discarded; error counters are available |
| [ ] M4-09 | Implement metrics: frame loss, CRC errors, bus-off, command age, latency | P1 | M4-08 | Metrics are published in telemetry and available to the operator |
| [ ] M4-10 | Implement switching to the backup transport when the primary degrades | P1 | M4-06, M4-07 | Failover occurs without loss of safety; event is logged |
| [ ] M4-11 | Implement RPi–MCU heartbeat and handshake; loss of heartbeat puts the MCU into `DAMPING` | P1 | M4-08, M3-38 | Loss of communication is detected within 100 ms; recovery is correct |
| [ ] M4-12 | Implement MCU–RPi clock offset and drift estimation via EWMA; optionally GPIO 1 PPS | P1 | M4-08 | IMU/odometry/lidar desynchronization ≤ 10 ms; timestamps are set at sample time |

### 4.3. Lidar driver

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-13 | Select a lidar and write an ADR: 2D 360°, ≥ 10 m, ≥ 10 Hz or 3D with comparable characteristics | P1 | M4-01 | Model selected taking into account power, mounting, driver and availability |
| [ ] M4-14 | Implement the driver in Rust: serial/USB interface, scans or point cloud, asynchronous publication | P1 | M4-13, M4-05 | Data is published at ≥ 10 Hz; errors do not block the runtime |
| [ ] M4-15 | Transform measurements to `laser_link`; perform angular and range calibration | P1 | M4-14, M1-08 | Calibration verified against reference objects; errors within tolerance |
| [ ] M4-16 | Implement lidar health monitoring: rate, gaps, staleness, temperature, protocol errors | P1 | M4-14 | On degradation, a fault is published; navigation receives a stale flag |
| [ ] M4-17 | Implement recording of raw scans for replay and regression tests | P1 | M4-14 | Recording is reproducible; format is versioned |
| [ ] M4-18 | Implement scan preprocessing: outlier filtering, range limiting, conversion to point cloud/grid | P1 | M4-15 | Preprocessing is deterministic; parameters are configurable |

### 4.4. Camera driver

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-19 | Connect an RGB-D camera of the RealSense D435i or OAK-D class via USB 3.0/CSI-2; implement capture at ≥ 30 fps at 640×480 | P1 | M4-01 | Capture is stable; frame drops are accounted for |
| [ ] M4-20 | Implement camera calibration, storage in configuration and verification at startup | P2 | M4-19 | Incorrect calibration is detected; parameters are versioned |
| [ ] M4-21 | Ensure camera synchronization with IMU and lidar ≤ 10 ms | P2 | M4-12, M4-19 | Timestamps are consistent; desynchronization is measured |
| [ ] M4-22 | Ensure camera failure isolation: loss of camera does not block lidar navigation | P1 | M4-19 | When the camera is disconnected, navigation continues; a fault is published |

### 4.5. Sensor integration and replay

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-23 | Implement a sensor health monitor and publication of statuses with timestamps | P1 | M4-16, M4-22 | All sensors have status `OK/DEGRADED/STALE/FAULT` |
| [ ] M4-24 | Implement publication of sensor data in Zenoh with explicit QoS and timestamps | P1 | M4-14, M4-23 | Subscriber receives data; loss policy is explicitly defined |
| [ ] M4-25 | Implement a harness for recording and replaying sensor streams | P1 | M4-17 | Replay reproduces the scenario; metrics are comparable with the reference |
| [ ] M4-26 | Conduct HIL injection: lidar staleness, camera disconnection, time jump, USB disconnection | P1 | M4-24, M4-25 | All faults are detected; safe behavior confirmed |
| [ ] M4-27 | Prepare driver documentation: models, interfaces, calibrations, limitations, troubleshooting | P1 | M4-14, M4-19 | The document is sufficient for sensor replacement and recalibration |

---

## 5. M5. Raspberry Pi 5 onboard software

**Goal:** assemble the `jbot_node` composition root: configuration, transport, state estimation, supervisor, telemetry, middleware and correct shutdown.

**Inputs:** `docs/architecture.md` sections 6, 7, 8; `docs/specification.md` sections 3, 8, 9, 10.
**Outputs:** `jbot_node`, `jbot_estimation`, `jbot_common`, configuration profiles, systemd deployment, HIL protocol.

### 5.1. Composition and infrastructure

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-01 | Extend `jbot_common`: typed state, command, modes, fault codes, configuration | P1 | M1-05 | Types are serializable; schema version is fixed |
| [ ] M5-02 | Implement TOML loading with validation, defaults, profiles and hash logging | P1 | M5-01 | Invalid config is rejected; `sim/bench/robot` profiles are supported |
| [ ] M5-03 | Implement Tokio tasks: ingress, estimation, control egress, telemetry, supervisor, shutdown | P1 | M4-05, M5-02 | Tasks start and stop in a coordinated manner; errors are not lost |
| [ ] M5-04 | Implement channel policy: `watch` for commands and state, bounded `mpsc` for frames and logs, `broadcast` for faults | P1 | M5-03 | Queues are bounded; telemetry overflow does not block E-stop |
| [ ] M5-05 | Implement signal handling and correct shutdown via `CancellationToken` | P1 | M5-03 | SIGTERM/SIGINT lead to safe shutdown and log flush |
| [ ] M5-06 | Implement structured `tracing` JSON logging and metrics with node/task/robot ID | P1 | M5-03 | Logs are parsed; there is no high-frequency logging in the loop |
| [ ] M5-07 | Ensure task isolation: navigation, perception and logging do not block estimation and supervisor | P1 | M5-03 | Under load, estimation and supervisor meet deadlines |

### 5.2. Middleware and protocol

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-08 | Implement Zenoh publish/subscribe: state, telemetry, diagnostics, commands | P1 | M5-03 | Topic contracts and QoS are documented; E-stop is not lost |
| [ ] M5-09 | Connect `zenoh-bridge-ros2dds` for RViz and rosbag; verify that the bridge does not block safety | P1 | M5-08 | RViz sees TF and scans; if the bridge crashes, control is unaffected |
| [ ] M5-10 | Implement handshake and a compatibility matrix for firmware and host protocol | P1 | M4-11, M5-02 | Incompatible version is rejected with a clear error |
| [ ] M5-11 | Implement configuration profiles for simulation, bench and robot with explicit differences | P1 | M5-02 | Profiles pass validation; differences are documented |

### 5.3. State estimation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-12 | Implement a 100–200 Hz EKF: IMU, wheel odometry, estimation of pose, velocity and gyroscope bias | P1 | M1-23, M4-12 | EKF converges; estimates are reproducible on replay |
| [ ] M5-13 | Implement publication of `odom → base_link` and prepare `map → odom` for SLAM | P1 | M5-12, M1-08 | TF tree conforms to REP-105 |
| [ ] M5-14 | Implement compensation for delay, outliers and measurement gaps | P1 | M5-12 | Jumps and gaps do not lead to divergence |
| [ ] M5-15 | Validate EKF on HIL and compare with the reference; record errors | P1 | M5-12, M8-06 | Pose and speed within acceptable limits; report with metrics |
| [ ] M5-16 | Ensure deterministic EKF replay on recorded logs | P1 | M5-12, M4-25 | The same log produces the same estimate |

### 5.4. Supervisor and safety

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-17 | Implement supervisor: voltage, temperature, latency, command age, fault flags, RPi resources | P1 | M5-03, M2-17 | When limits are exceeded, a safe stop command is generated |
| [ ] M5-18 | Implement command validation and safety envelope: stale, invalid and untrusted commands are not applied | P1 | M5-17, M3-36 | Negative tests confirm rejection of application |
| [ ] M5-19 | Implement priority of teleoperation over autonomous motion | P1 | M5-18 | Teleoperator takes control immediately |
| [ ] M5-20 | Ensure startup to `IDLE` ≤ 10 s and correct shutdown by command and at low charge | P1 | M5-05, M5-17 | Time is measured; logs and configuration are preserved |
| [ ] M5-21 | Integrate systemd watchdog and restart of `jbot_node` without loss of safety | P1 | M4-02, M5-05 | A hang leads to restart; MCU transitions to `DAMPING` |

### 5.5. HIL, telemetry and operation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-22 | Conduct fault injection: kill `jbot-node`, CAN/UART disconnection, lidar staleness, RPi reboot | P1 | M5-17, M4-26 | RPi failure does not cause a fall; recovery is correct |
| [ ] M5-23 | Conduct a 2–4 hour soak test with metric logging | P1 | M5-22 | No leaks, degradation or task misses |
| [ ] M5-24 | Implement a Rerun/Zenoh telemetry dashboard with key metrics | P2 | M5-08 | Operator sees state, battery, latencies and faults |
| [ ] M5-25 | Implement orchestration of blackbox and log upload from the MCU | P2 | M3-40, M5-08 | Logs are available without disassembling the robot |
| [ ] M5-26 | Conduct Gate G3 review and prepare the HIL protocol | P1 | M5-20…M5-23 | All G3 criteria are confirmed |

---

## 6. M6. Navigation and SLAM

**Goal:** build a map, localize in it and ensure autonomous waypoint motion subject to safety constraints.

**Inputs:** `docs/specification.md` section 6; `docs/architecture.md` sections 3.3, 6.7; lidar and EKF data from M4–M5.
**Outputs:** `jbot_navigation`: SLAM, localization, costmap, planners, behavior layer, set of maps and test protocols.

### 6.1. SLAM and maps

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-01 | Conduct ADR-008: Rust-native SLAM versus `slam_toolbox`/Cartographer via the ROS2 bridge | P1 | M4-14, M5-10 | Decision justified by maturity, latency, resources and risks |
| [ ] M6-02 | Implement map types: occupancy grid, resolution ≤ 5 cm, area ≥ 1000 m², metadata | P1 | M6-01 | Map is serializable; metadata includes robot configuration |
| [ ] M6-03 | Implement scan preprocessing for SLAM: filtering, downsampling, dynamic object removal | P1 | M4-18 | Preprocessing improves stability; parameters are configurable |
| [ ] M6-04 | Implement scan matching frontend: ICP/CSM or equivalent with convergence control | P1 | M6-03 | Matching error within tolerance; outliers are discarded |
| [ ] M6-05 | Implement occupancy grid construction with probability updates and memory limiting | P1 | M6-04 | A map ≥ 100 m² is built without destroying structure |
| [ ] M6-06 | Implement loop closure and pose graph optimization | P1 | M6-05 | When returning to a known area, drift is corrected |
| [ ] M6-07 | Implement relocalization when returning to a known area | P1 | M6-06 | Relocalization ≤ 10 s (P2) |
| [ ] M6-08 | Implement map saving and loading in PGM/YAML or a documented equivalent | P1 | M6-02 | Map loads into ROS2 tools or equivalent |
| [ ] M6-09 | Implement map versioning and binding to robot configuration | P1 | M6-08 | Incompatible map is rejected; version is visible to the operator |
| [ ] M6-10 | Ensure loading of a 1000 m² map ≤ 10 s (P2) | P2 | M6-08 | Time measured on NVMe |
| [ ] M6-11 | Implement recording and replay of SLAM data for regression | P1 | M4-25, M6-05 | The same log produces the same map |
| [ ] M6-12 | Implement automatic map updates only on an authorized command | P2 | M6-09 | Without a command, the map does not change |

### 6.2. Localization

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-13 | Implement localization in the map: scan matching + EKF, estimation 100–200 Hz, lidar correction 10–30 Hz | P1 | M6-04, M5-12 | Pose is published with latency ≤ 100 ms |
| [ ] M6-14 | Implement the TF tree `map → odom → base_link` per REP-105 | P1 | M6-13, M5-13 | TF is consistent; RViz shows no breaks |
| [ ] M6-15 | Implement kidnapping detection and relocalization ≤ 10 s (P2) | P2 | M6-13 | Kidnapping scenario is detected; robot stops until recovery |
| [ ] M6-16 | Implement localization degradation: speed limiting, then stop | P1 | M6-13 | On staleness or low data quality, motion is prohibited |
| [ ] M6-17 | Validate accuracy ≤ 50 mm and 2° (RMS) relative to reference markers | P1 | M6-13, M8-07 | Measurement protocol and plots attached |

### 6.3. Costmap and obstacles

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-18 | Implement the static map, obstacle map and inflation taking into account dimensions | P1 | M6-08, M1-01 | Inflation corresponds to footprint; narrow passages are not falsely blocked |
| [ ] M6-19 | Implement a dynamic obstacle layer from lidar with data lifetime | P1 | M6-18, M4-14 | Dynamics are updated; stale obstacles are removed |
| [ ] M6-20 | Implement staleness policy: data older than 300 ms leads to a stop | P1 | M6-19, M4-16 | Data delay test confirms the stop |
| [ ] M6-21 | Implement safe distance: < 0.25 m at low speed leads to a stop | P1 | M6-19 | Obstacle test confirms the stop |
| [ ] M6-22 | Implement speed limiting near obstacles ≤ 0.3 m/s | P1 | M6-21 | Speed is reduced to the threshold; limiting is visible in telemetry |

### 6.4. Planning and motion control

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-23 | Implement a global planner A*/Dijkstra taking into account traversability and dimensions | P1 | M6-18 | Path is built; unreachable goal returns an error |
| [ ] M6-24 | Implement a local planner/controller DWA, MPC or equivalent taking into account WIP dynamics | P1 | M6-23, M1-12 | Robot follows the path without losing balance; constraints are respected |
| [ ] M6-25 | Account for constraints on acceleration, jerk, tilt, torque and slip | P1 | M6-24, M3-21 | Constraints are not violated during typical maneuvers |
| [ ] M6-26 | Implement path following with speed limiting and smooth braking | P1 | M6-24 | Stop from 1.0 m/s while maintaining balance; path ≤ 0.5 m |
| [ ] M6-27 | Implement recovery behaviors: costmap clearing, replanning, rotation, backing off | P2 | M6-23, M6-24 | Behaviors are triggered by condition and do not create dangerous maneuvers |
| [ ] M6-28 | Implement priority of teleoperation over autonomous motion | P1 | M5-19, M6-24 | Teleoperator takes control without delay |
| [ ] M6-29 | Implement the behavior layer: waypoints, patrol, pause, return, emergency stop | P1 | M6-26 | Missions are executed; emergency stop has the highest priority |
| [ ] M6-30 | Isolate navigation in a separate process or task with resource limits | P1 | M5-03, M6-24 | Navigation does not affect balance jitter and does not block the supervisor |
| [ ] M6-31 | Implement a mission interface for returning to the dock and patrolling | P2 | M6-29 | Mission is started and canceled by command; state is published |

### 6.5. Navigation tests

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-32 | Run scenarios in simulation with the WIP model and sensor noise | P1 | M1-26, M6-24 | Scenarios are reproducible; metrics collected |
| [ ] M6-33 | Build a map of a room ≥ 100 m² with loop closure; measure drift ≤ 1% of path | P1 | M6-06, M6-11 | Protocol and map attached |
| [ ] M6-34 | Perform 100 autonomous runs to the goal; success rate ≥ 95%, collisions 0 | P1 | M6-29, M6-33 | Protocol for each run; failures analyzed |
| [ ] M6-35 | Verify dynamic obstacles: stop or replanning ≤ 1 s | P1 | M6-19, M6-24 | Scenario with a person/cart passed |
| [ ] M6-36 | Verify lidar failure, data staleness and robot kidnapping | P1 | M6-16, M6-20 | All scenarios lead to safe behavior |
| [ ] M6-37 | Verify operation without ROS2 and with the ROS2/RViz bridge | P1 | M5-09, M6-30 | Base navigation does not depend on ROS2; bridge does not block safety |
| [ ] M6-38 | Run regression on recorded logs and compare metrics with the reference | P1 | M6-11, M8-15 | Regression is reproducible; deviations are explained |
| [ ] M6-39 | Prepare the navigation report and Gate G4 protocol | P1 | M6-33…M6-38 | All G4 criteria are confirmed |

---

## 7. M7. Safety and fault tolerance

**Goal:** ensure Linux-independent protection, predictable behavior under faults and traceability of safety requirements.

**Inputs:** `docs/specification.md` section 9; `docs/architecture.md` section 7.
**Outputs:** hazard analysis, FMEA, safety case, fault catalog, proof-test procedures, checklists.

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M7-01 | Conduct hazard analysis and FMEA from the very beginning; update at each gate | P1 | M1-01 | Hazard and mitigation register covers mechanics, power, software and operation |
| [ ] M7-02 | Specify a two-channel NC E-stop: disconnecting the power stage, preserving logic, latched state | P1 | M2-05 | Specification verified on the bench; complies with SAF-02, SAF-03 |
| [ ] M7-03 | Specify and verify the watchdog chain: E-stop → IWDG → 100 ms timeout → RPi heartbeat → systemd → supervisor | P1 | M3-38, M5-21 | Each link is verified separately and in combination |
| [ ] M7-04 | Record the limits matrix: current, temperature, voltage, tilt, speed, command age | P1 | M1-02, M2-15 | Limits are consistent in firmware, supervisor and documentation |
| [ ] M7-05 | Implement a fault code catalog with severity, latched flag, context and timestamp | P1 | M3-39, M5-17 | Codes are stable; published over CAN and in Zenoh/ROS2 |
| [ ] M7-06 | Conduct a safety review before each field test and gate | P1 | M7-01 | Comments are closed; admission decision recorded |
| [ ] M7-07 | Prepare the E-stop and protection proof-test procedure at least once every 12 months | P2 | M7-02, M7-03 | Procedure is reproducible; results are archived |
| [ ] M7-08 | Ensure network security: SSH/VPN, no direct access to control ports, network isolation | P1 | M4-01 | Verified by scanning; access policy documented |
| [ ] M7-09 | Implement authentication of Zenoh/ROS2 endpoints where supported | P2 | M5-08 | Unauthorized access is rejected |
| [ ] M7-10 | Ensure rejection of invalid, stale and untrusted commands | P1 | M5-18, M3-36 | Negative tests confirm no change of the setpoint |
| [ ] M7-11 | Ensure brownout transition to a safe state with drivers disabled | P1 | M3-38, M2-20 | Brownout test confirms the safe state |
| [ ] M7-12 | Publish faults over CAN and in Zenoh/ROS2 with code, severity and time | P1 | M7-05 | Messages reach the operator; losses are not hidden |
| [ ] M7-13 | Implement software stop as a separate path that does not replace the hardware E-stop | P1 | M7-02 | Command stops motion; E-stop remains independent |
| [ ] M7-14 | Maintain the safety case and traceability matrix of SAF requirements | P1 | M7-01 | Each SAF requirement has a task, test and protocol |
| [ ] M7-15 | Prepare safe operation instructions, first power-up and field test checklists | P1 | M2-24, M3-43 | Instructions are available to the operator; checklists are mandatory |
| [ ] M7-16 | Maintain an incident, near-miss and emergency stop log | P2 | M7-12 | Log is preserved; events are analyzed at review |

---

## 8. M8. Integration, HIL and testing

**Goal:** assemble the complete system, verify it on HIL and confirm requirements with test protocols.

**Inputs:** M2–M6.
**Outputs:** HIL bench, test suites, protocols, acceptance report, delivery kit.

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M8-01 | Record the testing strategy by levels: unit, property, integration, simulation, HIL, system, regression | P1 | M1-01 | Level and tool matrix approved |
| [ ] M8-02 | Ensure unit and property tests in all crates; include in CI | P1 | M10-01 | Coverage of critical modules; tests are stable |
| [ ] M8-03 | Implement CAN/UART loopback and `vcan` integration tests | P1 | M4-05, M3-32 | Tests are reproducible in CI |
| [ ] M8-04 | Implement tests for timeouts, failover and heartbeat loss | P1 | M4-10, M4-11 | All scenarios pass; safe state confirmed |
| [ ] M8-05 | Implement a set of simulator regression traces and golden metrics | P1 | M1-30 | Deviations from the reference are detected |
| [ ] M8-06 | Assemble the HIL bench: MCU, drivers, RPi, lidar or simulator | P1 | M3-29, M4-26 | Bench is reproducible; measurements are available |
| [ ] M8-07 | Implement latency and jitter measurement tools, measurement protocol | P1 | M3-28, M5-23 | RPi–MCU–actuator latency ≤ 10 ms; jitter ≤ 50 µs |
| [ ] M8-08 | Implement a fault injection suite: power, communication, sensors, RPi, firmware | P1 | M7-03, M5-22 | Each fault has an expected safe response |
| [ ] M8-09 | Verify startup to safe `IDLE` ≤ 10 s | P1 | M5-20 | Time measured on the assembled system |
| [ ] M8-10 | Conduct a 2–4 hour soak test and a 24-hour run with battery replacement | P1 | M8-06, M2-22 | No critical failures; logs and metrics preserved |
| [ ] M8-11 | Conduct system thermal, EMI and vibration checks | P1 | M2-21, M8-06 | All parameters within acceptable limits |
| [ ] M8-12 | Prepare acceptance test programs and methods per section 12 of the SRS | P1 | M8-01 | Documents approved before acceptance begins |
| [ ] M8-13 | Conduct the 100 autonomous runs protocol | P1 | M6-34, M8-06 | ≥ 95 successful, 0 collisions; protocol signed |
| [ ] M8-14 | Conduct runtime tests on M18 XC5.0 by profiles | P1 | M2-22, M8-06 | ≥ 30 min typical, ≥ 20 min aggressive; profile attached |
| [ ] M8-15 | Conduct regression on recorded logs and compare metrics | P1 | M4-25, M6-38 | Results are reproducible; deviations are explained |
| [ ] M8-16 | Prepare the final acceptance report and close Gate G5 | P1 | M8-09…M8-15 | All P1 confirmed; P2 implemented or justified |
| [ ] M8-17 | Prepare a spare kit of fuses, connectors and consumables | P1 | M10-14 | Kit is included in the delivery |
| [ ] M8-18 | Prepare the operator manual, maintenance manual and protocol description | P1 | M10-21 | Documents verified for completeness and currency |
| [ ] M8-19 | Prepare the release package: tags, changelog, artifacts, build instructions | P1 | M10-22 | Package builds reproducibly from a clean environment |
| [ ] M8-20 | Conduct gate reviews G1–G5 with recording of decisions and open issues | P1 | M1-38, M5-26, M6-39, M8-16 | Review protocols are preserved |
| [ ] M8-21 | Maintain the traceability matrix "requirement → task → test → protocol" | P1 | M10-18 | Matrix is complete; gaps identified before acceptance |
| [ ] M8-22 | Update the risk register before each gate and on architecture changes | P1 | M10-17 | Risks have an owner, probability, impact and mitigations |

---

## 9. M9. Extensions: camera, AI, manipulator, docking station

**Goal:** extend the platform without compromising balance, safety and base navigation.

**Inputs:** M5–M8, free RPi resources and power/thermal margin.
**Outputs:** extension interfaces, AI pipeline, manipulator emulator/integration, docking station and auto-charging mission.

### 9.1. Extension interfaces

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-01 | Implement the mechanical and electrical camera interface: USB 3.0/CSI-2, 5 V with protection and current margin | P1 | M2-11, M4-01 | Camera is powered and transmits data; mount is adjustable |
| [ ] M9-02 | Implement the manipulator interface: 12 V / 5 A, protection, separate branch, CAN FD 29-bit ID | P3 | M2-11, M4-06 | Interface is reserved and verified on the emulator |
| [ ] M9-03 | Implement a PCIe/USB reserve for the Hailo-8L AI accelerator or analog, taking into account power and heat | P2 | M4-01, M2-11 | Accelerator is detected; power and cooling within budget |
| [ ] M9-04 | Implement an external isolated diagnostic CAN port | P2 | M2-06, M4-06 | Port is isolated; does not create ground loops |
| [ ] M9-05 | Verify that extensions do not increase balance loop jitter by more than 50 µs | P1 | M9-01…M9-04, M3-28 | Before/after measurements; impact within tolerance |

### 9.2. Camera and AI

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-06 | Ensure capture at ≥ 30 fps at 640×480 and stable frame delivery | P2 | M9-01, M4-19 | Drops are accounted for; latency within budget |
| [ ] M9-07 | Implement storage and verification of camera calibration at startup | P2 | M4-20 | Incorrect calibration blocks AI, but not navigation |
| [ ] M9-08 | Ensure camera synchronization with IMU and lidar ≤ 10 ms | P2 | M4-21, M9-06 | Timestamps are consistent; verified on a moving object |
| [ ] M9-09 | Select the AI runtime: `ort`, `tract` or `candle`; select the accelerator or CPU | P2 | M9-03 | ADR justified by latency, memory and compatibility |
| [ ] M9-10 | Implement object detection at ≥ 10 Hz on RPi 5 or the AI accelerator | P2 | M9-09 | Latency and accuracy metrics attached |
| [ ] M9-11 | Publish detections with timestamps without blocking navigation and control | P2 | M9-10, M5-08 | Publication is asynchronous; drops do not block other tasks |
| [ ] M9-12 | Ensure degradation: camera failure does not block lidar navigation | P1 | M4-22, M9-10 | Camera disconnect test; navigation continues |
| [ ] M9-13 | Verify the power and thermal budget of the AI accelerator under sustained load | P2 | M9-03, M9-10 | No brownout or throttling; temperatures within limits |

### 9.3. Manipulator

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-14 | Implement a CAN node emulator for the manipulator to verify ID, power and E-stop | P3 | M9-02, M4-06 | Emulator responds to commands; E-stop verified |
| [ ] M9-15 | Implement the manipulator control interface in `jbot_control` or the behavior layer, outside the 1 kHz loop | P1 | M9-14, M5-03 | Commands do not affect 1 kHz; latency within acceptable limits |
| [ ] M9-16 | Implement supervisor constraints with an extended load: speed, torque, tilt | P3 | M9-15, M5-17 | Constraints trigger before loss of balance |
| [ ] M9-17 | Implement manipulator emergency stop via the common E-stop and a software command | P3 | M9-14, M7-02 | Stop is independent of Linux; state is latched |
| [ ] M9-18 | Verify that manipulator failure or disconnection does not disrupt balance and navigation | P1 | M9-15 | Hot/cold failure test; base functionality preserved |

### 9.4. Docking station and auto-charging

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-19 | Design guides, power contacts, docking sensor and IR/CAN channel | P3 | M2-07, M9-02 | Design ensures docking accuracy and contact protection |
| [ ] M9-20 | Ensure contacts are de-energized until docking is confirmed, with reverse polarity and short circuit protection | P3 | M9-19 | Bench test confirms safety |
| [ ] M9-21 | Implement the state protocol `IDLE`, `APPROACH`, `DOCKED`, `CHARGING`, `FAULT` | P3 | M9-19 | Transitions are validated; errors do not leave contacts energized |
| [ ] M9-22 | Implement the mission for returning to the dock and charging under battery BMS control | P3 | M6-31, M9-21 | Mission is executed; interruption is safe |
| [ ] M9-23 | Ensure docking accuracy ≤ ±20 mm and ±2° in heading | P3 | M9-19, M6-13 | Series of attempts; metrics attached |
| [ ] M9-24 | Limit docking station power ≤ 150 W and ensure charging under BMS control | P3 | M9-20 | Current and power within limits; BMS controls charging |
| [ ] M9-25 | Implement charge monitoring, SoC/SoH and event logging | P3 | M9-22, M2-13 | Charging events available in telemetry |
| [ ] M9-26 | Verify that the docking station is not required for base operation | P1 | M9-22 | Disconnecting the dock does not affect base functions |

### 9.5. System validation of extensions

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-27 | Conduct a 24-hour run with battery replacement and planned pauses | P2 | M8-10, M9-13 | No critical failures; configuration is preserved |
| [ ] M9-28 | Verify EMI after installing the camera, AI accelerator and manipulator | P1 | M9-05 | CAN, IMU and lidar operate without degradation |
| [ ] M9-29 | Verify the thermal conditions of all components in typical and aggressive cycles | P1 | M9-13 | Temperatures within limits; no throttling |
| [ ] M9-30 | Confirm runtime ≥ 30 min typical and ≥ 20 min aggressive with extensions | P1 | M9-29, M2-22 | Protocol with profile and temperatures |
| [ ] M9-31 | Update schematics, BOM, configurations, calibrations and documentation for extensions | P1 | M9-01…M9-30 | Documentation set is current |

---

## 10. M10. Cross-cutting work

**Goal:** ensure quality, reproducibility, manageability and completeness of the project at all phases.

### 10.1. CI, quality and reproducibility

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-01 | Configure CI: `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test` on every PR | P1 | — | CI blocks merge on violation |
| [ ] M10-02 | Include protocol property tests: round-trip, corrupted frames, sequence wraparound | P1 | M3-35 | Tests run in CI |
| [ ] M10-03 | Include CAN/UART loopback and `vcan` integration tests | P1 | M8-03 | Tests are stable in CI |
| [ ] M10-04 | Configure aarch64 cross-build and firmware build in CI | P1 | M4-03, M3-02 | Artifacts are published |
| [ ] M10-05 | Include `cargo deny` and `cargo audit` | P1 | M10-01 | Vulnerabilities and license conflicts block merge |
| [ ] M10-06 | Configure a nightly HIL/regression run | P1 | M8-06 | Results are available; regressions are detected |
| [ ] M10-07 | Maintain ADRs for key decisions: MCU, drivers, SLAM, middleware, AI runtime | P1 | — | Each decision has an ADR with a status |

### 10.2. Configuration, logs and compatibility

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-08 | Version the protocol and maintain a firmware/host compatibility matrix | P1 | M3-45, M5-10 | Incompatible versions are detected before startup |
| [ ] M10-09 | Ensure calibrations are saved to non-volatile memory and restored after reboot | P1 | M3-42, M4-20 | Calibrations are not lost; corruption is detected |
| [ ] M10-10 | Standardize structured logs and metrics: fields, levels, rates, storage | P1 | M5-06 | Logs are suitable for automated analysis |
| [ ] M10-11 | Implement a tool for MCU blackbox upload and analysis | P2 | M3-40 | Data is read and correlated with RPi logs |
| [ ] M10-12 | Implement storage of maps and logs on a separate partition with rotation and overflow protection | P2 | M4-04 | Disk does not overflow; old data is deleted per policy |

### 10.3. Procurement and manufacturing

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-13 | Form the BOM and order long-lead items: actuators, drivers, RPi 5, lidar, battery adapter, CAN FD HAT | P1 | M1-02, M2-07 | Orders placed; lead times tracked |
| [ ] M10-14 | Provide a spare kit of fuses, connectors and consumables | P1 | M10-13 | Kit is included in the delivery and stored in inventory |
| [ ] M10-15 | Verify component availability and alternatives (second source) | P1 | M10-13 | Critical items have a replacement or justification |
| [ ] M10-16 | Manufacture/order PCBs and cable assemblies with margin | P1 | M2-07 | Spare units available for bench and robot |

### 10.4. Project management and documentation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-17 | Maintain the risk register and update it at each gate | P1 | — | Each risk has an owner, probability, impact and mitigations |
| [ ] M10-18 | Maintain the traceability matrix "SRS → task → test → protocol" | P1 | M8-21 | Matrix is complete and current |
| [ ] M10-19 | Hold a weekly progress review and a monthly risk review | P1 | — | Meeting protocols and decisions are preserved |
| [ ] M10-20 | Record deviations from the SRS only through approval and an addendum to the SRS | P1 | M10-18 | Deviations are not lost; changes are traceable |
| [ ] M10-21 | Maintain the documentation set: operator, maintenance, protocol, BOM, schematics, calibrations | P1 | M8-18 | Documentation matches the released version |
| [ ] M10-22 | Maintain releases, tags and changelog; record firmware, host and protocol versions | P1 | M10-08 | Any delivery is reproducible by tag |

---

## 11. Requirements traceability matrix

| Requirement group | Key tasks |
|---|---|
| SYS-01…SYS-04 (MCU, 1 kHz, actuators, IMU) | M3-01, M3-05, M3-11, M3-12, M3-17, M3-28, M8-07 |
| SYS-05…SYS-08 (upper level, communication, latency, RPi failure) | M4-05…M4-12, M5-03, M5-12, M5-21, M8-07 |
| AUT-01…AUT-07 (M18, connection, battery replacement) | M2-01, M2-05, M2-23, M7-04 |
| AUT-08…AUT-13 (energy budget, DC-DC, margin) | M2-02, M2-03, M2-10, M2-11, M2-22 |
| AUT-14…AUT-18 (runtime, profiles) | M2-22, M8-10, M8-14, M9-30 |
| AUT-19…AUT-30 (battery monitoring, thresholds, E-stop) | M2-04, M2-12…M2-19, M3-08, M7-11 |
| MOB-01…MOB-05 (kinematics, actuators, modes) | M1-03, M1-06, M1-07, M3-13, M3-19 |
| MOB-06…MOB-11 (balance, impulse, tilt, fall) | M1-19, M1-33, M3-20, M3-25…M3-27 |
| MOB-12…MOB-16 (speed, reverse, braking) | M1-18, M1-20, M3-21, M3-22, M6-26 |
| MOB-17…MOB-25, MOB-31 (terrain, height, steps, 3D balance) | M1-07, M1-26, M3-21, M3-23, M6-25, M8-11 |
| MOB-26…MOB-30 (load, mass, thermal conditions) | M2-21, M3-13, M3-29, M8-11, M9-29 |
| NAV-01…NAV-05 (lidar, IMU, odometry, synchronization) | M4-12…M4-25, M5-12 |
| NAV-06…NAV-13 (SLAM, maps, versioning) | M6-01…M6-12 |
| NAV-14…NAV-19 (localization, TF, degradation) | M5-12, M6-13…M6-17 |
| NAV-20…NAV-26 (costmap, planners, isolation) | M6-18…M6-31 |
| NAV-27…NAV-35 (100 runs, dynamics, safety, logs) | M6-32…M6-39, M8-13, M8-15 |
| EXT-01…EXT-06 (modularity, protocol, configuration) | M5-01, M5-02, M10-07, M10-08 |
| EXT-07…EXT-13 (camera, synchronization, degradation) | M4-19…M4-22, M9-01, M9-06…M9-12 |
| EXT-14…EXT-21 (manipulator) | M9-02, M9-14…M9-18 |
| EXT-22…EXT-28 (docking station) | M9-19…M9-26 |
| EXT-29…EXT-33 (AI accelerator, CAN, NVMe, jitter) | M9-03…M9-05, M9-13, M4-04 |
| INT-01…INT-10 (CAN FD, UART, frame format, power) | M2-03, M3-30…M3-37, M4-05…M4-12 |
| INT-11…INT-22 (Rust, no_std, queues, middleware, CI) | M3-02, M3-41, M5-03…M5-11, M10-01…M10-12 |
| SAF-01…SAF-13 (E-stop, watchdog, latched, network) | M2-05, M3-38…M3-43, M5-17…M5-21, M7-01…M7-16 |
| REL-01…REL-08 (24 h, conditions, service, calibrations) | M2-23, M3-42, M4-04, M8-10, M9-27, M10-09, M10-12 |

---

## 12. Milestones and execution order

| Milestone | Content | Tasks | Gate |
|---|---|---|---|
| M0 | Base documentation and workspace | M1-01…M1-05, M10-01, M10-07 | — |
| M1 | WIP model + LQR + simulator | M1-06…M1-38 | G1 |
| M2 | One hip QDD + one wheel + IMU on the bench | M3-01…M3-16, M2-01…M2-11 | — |
| M3 | Two motors, balance on the safety frame | M3-17…M3-29, M2-12…M2-24 | G2 |
| M4 | RPi–MCU communication, telemetry, lidar, EKF | M4-01…M4-27, M5-01…M5-23 | G3 |
| M5 | Map + localization | M6-01…M6-17 | — |
| M6 | Autonomous runs ≥ 95/100 | M6-18…M6-39 | G4 |
| M7 | Camera/AI, manipulator, docking station | M9-01…M9-31 | — |
| M8 | Acceptance tests and delivery | M8-01…M8-22, M10-13…M10-22 | G5 |

**Parallelism rule:** `M2`, `M7` and `M10` are executed in parallel with the main critical path; `M9` tasks do not start until G4 is closed, except for interface reservation and procurement.

---

## 13. Next two weeks

| ID | Task | Result by the end of week 2 |
|---|---|---|
| M1-01…M1-05 | Model parameters and `config/robot.toml` | Approved parameters and validated config |
| M1-06, M1-09, M1-10 | Differential drive kinematics and transforms | Kinematics tests in `jbot_common` |
| M1-11…M1-16 | 2D/3D WIP linearization and LQR | Matrices `A`, `B`, `K` and unit tests |
| M1-26…M1-30 | `jbot_sim` framework with RK4 and trace recording | First deterministic run |
| M1-32…M1-35 | First verification scenarios | 10 min hold, 10 N·s impulse, 10° deviation |
| M3-01 | ADR-001 on MCU | Decision ESP32-S3 prototype / STM32H7 production |
| M2-01…M2-07 | Power architecture and E-stop | Schematic, BOM, safety review |
| M7-01, M7-02 | Hazard analysis and E-stop specification | Draft safety document |
| M10-13 | Order long-lead components | Placed orders |

---

## 14. Risks affecting the task breakdown

| Risk | Affected tasks | Planned response |
|---|---|---|
| Incorrect mass-inertial parameters | M1-01…M1-38 | Refinement after assembly, stability margin, re-identification |
| ESP32-S3 does not provide 1 kHz and jitter ≤ 50 µs | M3-01, M3-17, M3-28 | Early benchmark, transition to STM32H7, keeping FOC inside the actuators |
| CAN FD on RPi is unstable | M4-06, M4-10 | Backup UART, oscilloscope verification, selection of a proven HAT |
| Lidar produces noise or too little data | M4-13…M4-18, M6-04 | Proven model, calibration, filtering, backup driver |
| Rust SLAM is not mature enough | M6-01…M6-12 | Backup `slam_toolbox` via the bridge, early ADR |
| WIP dynamics complicates the local planner | M6-24…M6-26 | MPC/DWA with a model, acceleration and tilt constraints, simulation |
| RPi throttling during SLAM and AI | M5-07, M6-30, M9-13 | Cooling, limits, separate processes, NVMe |
| AI accelerator power and heat | M9-03, M9-13 | Separate protection, power budget, tests under load |
| Manipulator disrupts balance | M9-15…M9-18 | Supervisor, constraints, moving out of the 1 kHz loop |
| Firmware and host version drift | M10-08, M5-10 | Compatibility matrix, CI check, version in every frame |

---

## 15. Open questions and decision tasks

| Question | Decision task | Deadline |
|---|---|---|
| Final MCU: ESP32-S3, STM32H723/H743 or Teensy 4.1 | M3-01, ADR-001 | Before purchasing the control board |
| Actuators: hip QDD integrated (RobStride); wheel hub integrated or external FOC | M3-01, M2-07, ADR | Before manufacturing the power board |
| Leg topology: 1-DOF four-bar decided; link lengths, spring, height range, E-position | M1-03, M3-23 | Before detailed mechanical design |
| Power pack: M18 18 V baseline vs 24–36 V preferred | M2-01, M2-02 | Before manufacturing the power board |
| Lidar and camera: models, mounting, power, heat | M4-13, M4-19, ADR | Before purchasing sensors |
| SLAM: Rust-native or `slam_toolbox` via bridge | M6-01, ADR-008 | Before the start of phase 4 |
| AI: Hailo-8L or CPU-only; runtime `ort`/`tract`/`candle` | M9-09, ADR | Before phase 5 |
| Firmware update path: CAN bootloader, USB DFU or probe | M3-44, M10-08 | Before production |
| Docking station: M18 contacts and charging protocol | M9-19, M9-21 | Before phase 5 |
| Mass, load and height range after the prototype | M1-01, M1-03 | After prototype assembly |
| Enclosure, connectors and maintainability | M2-23, M10-16 | Before manufacturing the enclosure |

---

## 16. Summary criteria for project completion

1. All P1 requirements are confirmed by test protocols, and the traceability matrix is filled in.
2. P2 requirements are implemented or have an approved deferral justification.
3. Balance loop jitter ≤ 50 µs; loss of a valid command > 100 ms leads to `DAMPING`.
4. Autonomous navigation: ≥ 95 successful runs out of 100 without collisions.
5. Runtime on M18 XC5.0 in a typical cycle ≥ 30 min.
6. E-stop and protections have passed proof-test; behavior under faults complies with the SRS.
7. The documentation set, source code, configurations, calibrations and maps are delivered in full.
8. All critical failures are closed; no open safety defects.

---

## 17. References

- `docs/specification.md` — technical specification and requirements.
- `docs/roadmap.md` — phases, gates and risks.
- `docs/architecture.md` — architecture, protocols, safety concept.
- `docs/system_requirements.md` — system requirements.
- `docs/agent_workflow.md` — agent roles and task execution order.
