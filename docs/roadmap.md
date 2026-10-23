# JBOT-WheeledBiped Development Roadmap

| Parameter | Value |
|---|---|
| Document | Development Roadmap |
| Code | RB-001 |
| Version | 1.0 |
| Date | 2026-10-21 |
| Status | Working document, synchronized with SRS-001 |
| Related documents | `docs/specification.md`, `docs/architecture.md`, `docs/system_requirements.md`, `docs/hardware.md`, `docs/bom.md` |
| Scope | Full cycle: BOM → test bench → onboard software → autonomous navigation → extensions |

---

## 0. How to read this document

### 0.1. Planning principles

1. **Hardware interfaces first.** Freeze the BOM interfaces and envelopes before firmware, procurement, and mechanical detail design.
2. **Early risk, early.** Long-lead components (actuators, lidar, RPi 5, battery adapter) are ordered in Phase 2.
3. **One critical path.** Balancing on the test bench is the main technical risk; everything else must not block it.
4. **Safety is not a phase.** E-stop, watchdog, limits, and logging are designed from the start and verified at every stage.
5. **Each phase ends with a measurable gate criterion.** Open P1 requirements are not silently carried over to the next phase.

### 0.2. Notation

- **P1 / P2 / P3** — requirement priorities from `docs/specification.md`.
- **Gate Gx** — phase acceptance gate; without passing it, the next phase is not considered started.
- **DoD** — Definition of Done, task completion criteria.
- **HIL** — Hardware-in-the-Loop, real MCU + real drivers + RPi.
- Phase labels 2–5 are retained for continuity; the former Phase 1 simulation track is retired.

### 0.3. Indicative timeline

Estimates are given for a team of 2–3 engineers (mechanics/electronics, embedded, ROS/navigation) and are not a commitment. Key dependencies and risks matter more than the calendar plan.

| Phase | Duration | Key outcome | Gate |
|---|---|---|---|
| 2. Hardware assembly and low-level test bench | 6–10 weeks | BOM, MCU + IMU + protocol, rotating test bench | G2 |
| 3. RPi 5, lidar, basic Rust software | 4–6 weeks | RPi–MCU link, telemetry, lidar, EKF, HIL | G3 |
| 4. SLAM, localization, navigation, autonomy | 6–10 weeks | Maps, costmap, planners, 100 autonomous runs | G4 |
| 5. Attachments, docking station, AI | 6–10 weeks | Camera/AI, manipulator, auto-charging, system tests | G5 |
| **Total** | **22–36 weeks** | Acceptance of the baseline version | — |

---

## 1. Initial state and scope

### 1.1. What has already been done

- [x] System requirements have been formulated (`docs/system_requirements.md`).
- [x] Architecture and safety concept have been designed (`docs/architecture.md`).
- [x] An SRS with requirement traceability has been created (`docs/specification.md`).
- [x] Cargo workspace initialized: `jbot_common`, `jbot_hardware`, `jbot_navigation`, `jbot_node`.
- [x] Type skeletons and test stubs added.
- [x] Provisional hardware selection and BOM baseline recorded (`docs/hardware.md`, `docs/bom.md`).

### 1.2. What is NOT included in the baseline version

- Stair flights and steps above 100 mm.
- Movement on water, sand, mud, and loose soil.
- Fully autonomous charging without a docking station.
- Manipulation operations as a mandatory function (interface and compatibility only).
- Certification for hazardous environments.

### 1.3. Critical path

```
BOM and interface freeze → one RS02 hip + one RS05 wheel on the test bench → all four actuators on the test bench
→ balancing on the safety frame → RPi–MCU link → EKF/odometry
→ SLAM and localization → planners → autonomous runs → extensions → acceptance
```

Running in parallel with the critical path: BOM completion, procurement, safety analysis, CI, and documentation.

---

## 2. Hardware bill of materials and interface freeze

**Goal:** turn the provisional hardware direction into a complete, orderable list of components and assemblies before detailed firmware and mechanical work.

**Indicative duration:** 1–2 weeks.

### 2.1. Tasks

- [ ] Complete the compute, actuation, power, sensing, chassis, and bench sections of [`docs/bom.md`](bom.md).
- [ ] Resolve every `TBD` that affects an interface, envelope, connector, or mounting point.
- [ ] Verify RS02 continuous torque and thermal margin for the 15 kg platform before ordering hips.
- [ ] Confirm the final MCU, CAN controller, and CAN transceiver after ADR-001.
- [ ] Record the battery voltage decision (18 V baseline versus 24–36 V preferred).
- [ ] Record substitutions with their ADR links and update `config/robot.toml` and `docs/hardware.md`.
- [ ] Freeze the interfaces needed by firmware, harnesses, and the chassis.

### 2.2. Gate B1 — BOM baseline

- [ ] Every robot and bench assembly has an owner, quantity, and provisional part.
- [ ] Interface-critical `TBD` items are closed or explicitly deferred.
- [ ] Long-lead items and second sources are identified.
- [ ] The BOM is consistent with `config/robot.toml` and `docs/hardware.md`.

### 2.3. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| BOM is incomplete or an interface is wrong | Rework, schedule slip | Interface freeze, vendor datasheets, second sources |
| RS02 hip torque/thermal margin is insufficient for 15 kg | Redesign | Bench thermal test before ordering; larger hip or lower mass |
| Battery decision is delayed | Power and chassis rework | Decide voltage and connector early |
| Part is unavailable | Procurement delay | Track lead times and alternatives |

---

## 3. Phase 2. Hardware assembly and low-level test bench (MCU + 4 actuators)

**Goal:** assemble a safe test bench from the BOM with one RS02 hip and one RS05 wheel, then all four actuators; configure the integrated RobStride loops, start the BNO055 and communication protocol; obtain the first balancing platform on the safety frame.

**Indicative duration:** 6–10 weeks.

> **Important:** ESP32-S3, Raspberry Pi Pico 2 W (RP2350), and STM32H723/H743 are the MCU candidates for ADR-001. ESP32-S3 has only TWAI/classic CAN; RP2350 has no native CAN and needs an external MCP2515/MCP2518FD over SPI, plus an external ADC because Pico 2 W exposes only three ADC inputs. STM32H7 has native FDCAN. The actuator bus uses classic CAN 1 Mbit/s through a TJA1051T/3 transceiver; CAN FD requires an FD-capable controller and transceiver. The RobStride RS02/RS05 selection is provisional and may be replaced by a cheaper equivalent. Firmware and protocol must allow porting without changing control logic.

### 3.1. Tasks

#### 2.1. Procurement and bench assembly

- [ ] Procure 2 RobStride RS02 hip actuators (approximately 17 N*m peak) and 2 RS05 wheel actuators as the provisional baseline; verify continuous torque and thermal margin first, because preliminary sizing flags RS02 as marginal for 15 kg. Substitute cheaper units only with an equivalent CAN/torque/thermal envelope. Add an external FOC stage only if the wheel driver is not integrated.
- [ ] Assemble a test bench with one RS02 hip and one RS05 wheel, using their native feedback plus a current sensor.
- [ ] Organize test bench power via a laboratory PSU with current limiting; then switch to the battery through the BOM protection.
- [ ] Provide an emergency button, fuse, and braking/bleeder element if needed.
- [ ] Check driver and motor heating under load; determine thermal limits and derating.
- [ ] Assemble the test bench frame with body fixation capability (safety stops/cable) for the first balance tests.

#### 2.2. Low-level firmware

- [ ] Create `firmware/` as a `no_std` crate; choose `embassy` (or `rtic`) and configure the build for the selected MCU (ESP32-S3, Pico 2 W/RP2350, or STM32H7 after ADR-001).
- [ ] Implement clock, timer, PWM, ADC, SPI/I2C, and watchdog initialization.
- [ ] Implement the BNO055 IMU driver over I2C/UART with calibration persistence and timestamps. Use gyro propagation or a high-rate IMU (ICM-42688-P fallback) for the 1 kHz loop because BNO055 fusion output is limited to 100 Hz.
- [ ] Implement RS02/RS05 feedback and speed/position calculation.
- [ ] Configure the RobStride RS02/RS05 integrated FOC and CAN protocol; implement FOC (electrical angle calibration, Park transforms, 20–40 kHz current loop) only for external wheel drivers.
- [ ] Implement current, torque, speed, and temperature limiting across all four actuators, including per-actuator and total limits.
- [ ] Implement the state machine `BOOT → SELF_TEST → IDLE → BALANCE → DAMPING → FAULT → E_STOP`.
- [ ] Implement the watchdog: IWDG, 100 ms command timeout, loss of RPi heartbeat → `DAMPING`.
- [ ] Implement a blackbox ring buffer of the last N seconds (P2).
- [ ] Ensure no dynamic memory allocation, `panic!`, or `unwrap()` in the control loop.

#### 2.3. Communication protocol

- [ ] Implement `jbot_protocol` (or a module in `jbot_common`) with a shared `no_std`-compatible frame format.
- [ ] Implement an 8-byte header: version, type, flags, sequence, timestamp; payload up to 56 bytes; CRC-16.
- [ ] Implement `postcard` serialization, little-endian, version and length validation.
- [ ] Implement the UART transport: COBS, CRC-16, resynchronization on `0x00`, max 256 bytes.
- [ ] Implement the actuator bus: classic CAN 1 Mbit/s through a TJA1051T/3 transceiver for the RobStride actuators (ESP32 TWAI or STM32 FDCAN in classic mode); provide an SPI CAN FD path for the RPi–MCU backbone if required.
- [ ] Implement messages: `ControlCommand`, `ModeCommand`, `EmergencyStop`, `Heartbeat`, `RobotState`, `ImuSample`, `JointFeedback`, `PowerFeedback`, `FaultReport`.
- [ ] Integrate the ExpressLRS 2.4G receiver over CRSF UART, bind it to the LiteRadio 3 Pro, and validate channel mapping and failsafe without bypassing the hardware E-stop.
- [ ] Cover the protocol with round-trip and property tests (truncation, corrupted CRC, sequence overflow, unknown version).

#### 2.4. First balance tests

- [ ] Verify rotation direction, phasing, encoder operation, and torque sign on one RS02 hip and one RS05 wheel.
- [ ] Verify the actuator current/torque loop (inside the QDD or external wheel driver): step command, overshoot, settling time, heating.
- [ ] Verify the velocity loop under inertial load.
- [ ] Verify hip height/roll response and the four-bar coupler curve on the bench.
- [ ] Assemble the four-actuator platform (2 hips + 2 wheels) on the test bench; verify odometry and direction of motion.
- [ ] Run PD balancing on the safety frame at zero command.
- [ ] Tune the selected balance controller on the bench and compare against the documented design targets.
- [ ] Conduct tests for holding, impulse, 10° deviation, 10° slope (within test bench limits).
- [ ] Measure 1 kHz loop jitter (GPIO/DWT/oscilloscope) and end-to-end command delay.
- [ ] Conduct a 30-minute thermal run.

#### 2.5. Test bench safety

- [ ] Verify E-stop actuation with motors running; ensure logic remains powered.
- [ ] Verify behavior on loss of communication, power loss, brownout, overheating, overcurrent.
- [ ] Verify that after reset the drivers are disabled by default.
- [ ] Prepare safe test bench operating instructions and a first-power-on checklist.

### 3.2. Phase artifacts

- Completed BOM, wiring diagrams, and harness drawings for the test bench.
- `firmware/`: MCU firmware (balance, CAN, IMU, protocol, safety; FOC only for external wheel drivers).
- `jbot_protocol`: shared frame and message format.
- Test bench with one and four actuators, safety frame.
- Test protocol: actuator loops, communication, jitter, delay, thermal mode, E-stop.
- ADR-001: MCU decision (Pico 2 W / ESP32-S3 / STM32H7 after benchmark).
- ADR on actuators: provisional RS02 hip and RS05 wheel selection, substitution criteria, and wheel driver choice.

### 3.3. Gate G2 — pass criteria

- [ ] Both RS05 wheels and both RS02 hips run stably; the RobStride internal loop is configured and the external wheel FOC (if used) runs at 20–40 kHz.
- [ ] Four-bar hip motion covers the required height range.
- [ ] Balance loop 1 kHz; measured jitter ≤ 50 µs or a decision to migrate to STM32H7 is recorded.
- [ ] Communication between the RPi equivalent (PC) and MCU: command 100 Hz, telemetry 200–500 Hz, frame loss measured.
- [ ] Loss of a valid command > 100 ms leads to `DAMPING`.
- [ ] E-stop disconnects actuator power, logging is preserved.
- [ ] The robot maintains balance on the safety frame at zero command for at least 10 minutes.
- [ ] Recovery after a 10 N·s impulse and 10° deviation is confirmed on the test bench.
- [ ] 30-minute thermal run without overheating of actuators and drivers.
- [ ] The communication protocol is versioned and covered by tests.

### 3.4. Phase risks

| Risk | Impact | Mitigation |
|---|---|---|
| Selected MCU cannot handle 1 kHz + CAN + jitter | Schedule slip | Early benchmark; migration to STM32H7; keep FOC inside the actuators |
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

- [ ] Implement the EKF from IMU and wheel odometry at 100–200 Hz.
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
- `jbot_node`: EKF at 100–200 Hz.
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
- [ ] Implement a local planner/controller: DWA, MPC, or equivalent accounting for balance constraints.
- [ ] Account for limits: acceleration, jerk, tilt, torque, slip.
- [ ] Implement path following with speed limiting and smooth braking.
- [ ] Implement recovery behaviors: costmap clearing, replanning, turning, backing off.
- [ ] Implement teleoperation priority over autonomous movement.
- [ ] Implement the behavior layer: waypoints, patrol, pause, return, emergency stop.
- [ ] Isolate navigation in a separate process/task with resource limits.

#### 4.5. Autonomous tests

- [ ] First run navigation scenarios on the HIL bench with sensor noise.
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
| Balance dynamics complicate the local planner | Falls during maneuvers | Acceleration/tilt limits, conservative speed, HIL validation |
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
- [ ] Implement the control interface in the behavior layer, outside the 1 kHz loop.
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

| Stage | Content | Depends on |
|---|---|---|
| S0 | Documentation, workspace, and BOM baseline | — |
| S1 | One motor + FOC + IMU on the test bench | S0, actuator procurement |
| S2 | Two motors, balancing on the safety frame | S1 |
| S3 | RPi–MCU link, telemetry, lidar, EKF | S2, RPi/lidar |
| S4 | Map + localization | S3 |
| S5 | Autonomous runs ≥ 95/100 | S4 |
| S6 | Camera/AI, manipulator, docking station | S5 |
| S7 | Acceptance tests and delivery | S6 |

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

1. Final MCU: Pico 2 W (RP2350), ESP32-S3, or STM32H723/H743?
2. Actuators: provisional RobStride RS02/RS05 with integrated FOC; replace only with an equivalent CAN/torque/thermal envelope?
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

- [ ] Complete `docs/bom.md` and order long-lead components after the interface freeze (actuators, RPi 5, lidar, CAN hardware).
- [ ] Prepare ADR-001 (MCU) and the ADR on actuators.
- [ ] Start hazard analysis and the E-stop specification.
- [ ] Set up the safety bench frame and a current-limited first-power-up procedure.
