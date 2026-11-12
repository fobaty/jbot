# JBOT-WheeledBiped Development Roadmap

| Parameter | Value |
|---|---|
| Document | Development Roadmap |
| Code | RB-001 |
| Version | 1.1 |
| Date | 2026-11-12 |
| Status | Working document, synchronized with SRS-001 and HW-001 |
| Related documents | `docs/specification.md`, `docs/architecture.md`, `docs/system_requirements.md`, `docs/hardware.md`, `docs/bom.md`, `firmware/README.md` |
| Scope | Donor hoverboard two-wheel stage 1 → hip-actuated stage 2 → mission computer, navigation, extensions → acceptance |

---

## 0. How to read this document

### 0.1. Planning principles

1. **The donor hardware is fixed.** Stage 1 is built on a stock hoverboard mainboard, its two hub
   motors, and its 10s2p pack. No new motor controller, wheel actuator, or battery is bought for
   stage 1.
2. **Stage 1 is the critical path now.** Balancing the donor platform on its own power stage is the
   main technical risk; no later work is allowed to block it.
3. **Fail on the bench, not in the field.** Direction, limits, thermal margin, and pack sag are
   measured in stage 1 before anything is designed around them.
4. **Safety is not a stage.** Kill switch, watchdog, limits, and logging are designed from the start
   and verified at every stage.
5. **Each stage ends with a measurable gate.** Open P1 requirements are not silently carried into the
   next stage.

### 0.2. Notation

- **P1 / P2 / P3** — requirement priorities from `docs/specification.md`.
- **Stage 1–5** — development stages. The stages are sequential; stage 1 is the donor hoverboard
  bring-up.
- **Gate G1…G5** — stage acceptance gate; without passing it, the next stage is not considered
  started. **B1** is the BOM baseline gate before stage 1.
- **Milestone S0…S6** — delivery milestones in section 9.
- **DoD** — Definition of Done, task completion criteria.
- **HIL** — Hardware-in-the-Loop, real mainboard + real drivers + mission computer.
- **Balance rate** — the rate of the balance task on the mainboard. It is 200 Hz in stage 1, with the
  16 kHz FOC current loop underneath. The rate is re-measured if the controller changes.

### 0.3. Indicative timeline

Estimates are given for a team of 2–3 engineers (mechanics/electronics, embedded, ROS/navigation) and
are not a commitment. Key dependencies and risks matter more than the calendar plan.

| Stage | Duration | Key outcome | Gate |
|---|---|---|---|
| 1. Donor hoverboard bring-up and balance | 2–3 weeks | Radio, IMU, 200 Hz balance, drive/steer/spin on the donor wheels | G1 |
| 2. Hips, legs, and 3D balance | 6–10 weeks | Two hip actuators, four-bar legs, height and roll, four actuators | G2 |
| 3. Mission computer, lidar, basic software | 4–6 weeks | Mission computer link, telemetry, lidar, EKF, HIL | G3 |
| 4. SLAM, localization, navigation, autonomy | 6–10 weeks | Maps, costmap, planners, 100 autonomous runs | G4 |
| 5. Attachments, docking station, AI | 6–10 weeks | Camera/AI, manipulator, auto-charging, system tests | G5 |
| **Total** | **24–39 weeks** | Acceptance of the baseline version | — |

---

## 1. Initial state and scope

### 1.1. What has already been done

- [x] Donor hardware decided: stage 1 reuses the hoverboard STM32F103RCT6 mainboard, both direct-drive
      hub motors, the 10s2p pack, the power button, and the charge port. Both sideboards, the deck,
      and the covers are discarded.
- [x] EFeru firmware baseline pinned and patched (`firmware/README.md`): builds for `VARIANT_PWM`, not
      yet flashed. The profile limits each motor to 10 A and 400 rpm for the original pack.
- [x] Donor hardware, stage I/O map, radio link, IMU, and charge interlock documented
      (`docs/hardware.md`, `docs/bom.md`, `firmware/README.md`).
- [x] System requirements formulated (`docs/system_requirements.md`).
- [x] Architecture and safety concept designed (`docs/architecture.md`).
- [x] SRS with requirement traceability created (`docs/specification.md`).
- [x] Cargo workspace initialized: `jbot_common`, `jbot_hardware`, `jbot_navigation`, `jbot_node`.
- [x] Type skeletons and test stubs added.

### 1.2. What is NOT included in the baseline version

- Stair flights and steps above 100 mm.
- Movement on water, sand, mud, and loose soil.
- Fully autonomous charging without a docking station.
- Manipulation operations as a mandatory function (interface and compatibility only).
- Certification for hazardous environments.

### 1.3. Critical path

```
donor bring-up: sideboard removal, inspection, EFeru baseline, radio, IMU
→ balance on the safety rig → drive, reverse, turn, spin in place
→ stage 2 hips and 3D balance
→ mission computer–mainboard link → EKF/odometry
→ SLAM and localization → planners → autonomous runs → extensions → acceptance
```

Running in parallel with the critical path: BOM completion, the few added parts, safety analysis, CI,
and documentation.

---

## 2. Hardware baseline and bill of materials

**Goal:** turn the provisional hardware direction into a complete, orderable list of components and
assemblies before detailed firmware and mechanical work. The donor parts are reused, not ordered.

**Indicative duration:** 1–2 weeks.

### 2.1. Tasks

- [ ] Complete the donor reuse list and the added-parts list in [`docs/bom.md`](bom.md): main fuse,
      kill switch, DC-DC 5 V, XR1 receiver, BNO055, debug probe, USB-UART adapter, safety rig.
- [ ] Record explicitly that the mainboard, both hub motors, the 10s2p pack, the power button, and the
      charge port come from the donor and are not purchased.
- [ ] Record that the **only long-lead item is the stage 2 hip actuator**. Leave it unordered until
      stage 1 balance is proven and the required hip torque is derived from the measured centre-of-mass
      height.
- [ ] Write the hip actuator selection criteria: torque and thermal margin for the measured mass,
      integrated driver preferred, UART interface, position and temperature feedback, envelope.
- [ ] Resolve every `TBD` that affects an interface, envelope, connector, or mounting point.
- [ ] Size the main fuse (30–40 A) and the kill switch (latching, ≥ 40 A) after the first current
      measurement on the real pack.
- [ ] Record substitutions with their ADR links and update `config/robot.toml`, `docs/hardware.md`, and
      `docs/bom.md`.
- [ ] Freeze the interfaces needed by firmware, harnesses, and the chassis: the two 4-pin cables, SWD,
      and the fused DC-DC branch.

### 2.2. Gate B1 — hardware baseline

- [ ] Every donor part is marked as reused, with quantity and condition, and is not on the order list.
- [ ] Every added part has an owner, quantity, and provisional part.
- [ ] Interface-critical `TBD` items are closed or explicitly deferred.
- [ ] The hip actuator is recorded as the only open long-lead item, with criteria and a decision point.
- [ ] The BOM is consistent with `config/robot.toml`, `docs/hardware.md`, and `firmware/README.md`.

### 2.3. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Donor mainboard is damaged or the pack is at end of life | Stage 1 blocked | Inspect and measure before buying anything else; a second donor is the fallback |
| Hip actuator is selected too late | Stage 2 slip | Criteria fixed during stage 1; order only after the first balance run |
| The 2p pack cannot supply two motors at 10 A plus regen | BMS trip, fall | Keep the 10 A / 400 rpm profile, derate early, measure sag under load |
| BOM is incomplete or an interface is wrong | Rework, schedule slip | Interface freeze, datasheets, continuity checks on the real board |
| Part is unavailable | Procurement delay | Track availability and alternatives for the added parts |

---

## 3. Stage 1. Donor hoverboard bring-up and balance (G1)

**Goal:** turn the stock hoverboard into a radio-controlled balancing platform: remove the sideboards,
prove the donor board and the pack, flash the EFeru baseline, wire the CRSF receiver and the BNO055,
then replace the stock input path with the Rust controller and balance, drive, and turn.

**Indicative duration:** 2–3 weeks.

> **Note:** the donor mainboard is the only board that drives the wheel motors in stages 1 and 2. It has
> exactly two 3-phase bridges, so a third and fourth motor need external drivers in stage 2. CAN is not
> routed on this board: stages 1 and 2 use UART/CRSF only.

### 3.1. Donor preparation

- [ ] Disconnect both sideboards and set them aside for reference. They must never be reconnected: they
      transmit their own protocol on the two signal wires the receiver and the IMU need.
- [ ] Remove the deck and the covers; keep the mainboard, both hub motors, the pack, the power button,
      and the charge port.
- [ ] Visual inspection: burnt MOSFETs, cracked shunts, corroded connectors, chafed motor and hall
      harnesses, damaged capacitors.
- [ ] Continuity inspection: equal phase-to-phase resistance on both motors, hall sensors toggling when
      the wheel is turned, no phase-to-ground and no phase-to-15 V short, no short across the DC link.
- [ ] Insulate the 15 V pins of both 4-pin cables; identify the two signal wires with a multimeter
      before connecting anything.
- [ ] Mount the mainboard, pack, fuse, kill switch, DC-DC, and receiver on the bench frame; keep power
      wiring short and twisted.

### 3.2. EFeru baseline

- [ ] Recreate the pinned upstream tree and apply `eferu-jbot.patch`; build `VARIANT_PWM`
      (`firmware/README.md`, section 2).
- [ ] Attach SWD on PA13/PA14, unlock RDP if the chip is read-protected (mass erase is accepted; the
      stock firmware is not recoverable), and flash the baseline.
- [ ] Verify the debug UART on PA2 TX at 115200 8N1: `in1/in2`, `cmdL/cmdR`, `BatADC/BatV`,
      `TempADC/Temp`.
- [ ] Calibrate the battery divider `BAT_CALIB_REAL_VOLTAGE` / `BAT_CALIB_ADC` against a multimeter at
      two pack voltages.
- [ ] Verify hall and phase direction on both wheels with the wheels lifted; correct by swapping two
      phase wires or the hall order, never in software.
- [ ] Confirm the profile limits: 10 A per motor, 12 A DC, 400 rpm.

### 3.3. Radio link

- [ ] Bind the BetaFPV LiteRadio 3 Pro to the RadioMaster XR1 on the same ExpressLRS major version.
- [ ] Wire the XR1 CRSF output to the left cable: USART2, PA3 RX / PA2 TX, 420 kbaud, 3.3 V logic.
- [ ] Use the CRSF-to-PWM adapter on the right cable (PB10/PB11) only for the first bench test of the
      EFeru baseline, then remove it.
- [ ] Record the channel map: CH1 steering, CH2 throttle, CH3 arm, CH4 mode, CH5 rate, CH6–CH8 spare.
- [ ] Set the failsafe on the transmitter and verify it with the motors lifted: loss of frames zeroes the
      setpoints and keeps the balance torque.
- [ ] Implement the arming path: the radio is the only device that can arm; a disarmed state never
      balances.

### 3.4. IMU

- [ ] Mount the BNO055 rigidly on the chassis, low and centred between the wheels, axes aligned with
      the robot frame.
- [ ] Wire I2C2 to the right cable: PB10 SCL, PB11 SDA, 100 kHz, address 0x28 or 0x29.
- [ ] Run accelerometer, gyroscope, and magnetometer calibration; persist the offsets and restore them
      at boot.
- [ ] Verify 100 Hz fusion output, gyro propagation to the 200 Hz balance rate, and the sign convention
      against the wheel direction.

### 3.5. Rust controller

- [ ] Create `firmware/hoverboard-rs` as its own workspace: `thumbv7m-none-eabi`, `stm32f1xx-hal` 0.11,
      RTIC 2, static priorities for the 16 kHz FOC interrupt and the 200 Hz balance task.
- [ ] Reuse the generated fixed-point FOC core and SVPWM through one FFI boundary
      (`BLDC_controller_step` plus plain structs); everything above the boundary is Rust.
- [ ] Implement clocks, timers, 16 kHz PWM, ADC/DMA, hall capture, and the watchdog.
- [ ] Implement the CRSF parser, channel mapping, arming, and failsafe.
- [ ] Implement the BNO055 driver, the pitch estimate, and gyro propagation between fusion samples.
- [ ] Implement the 200 Hz balance task with a fixed period, no heap, no blocking, and no `panic!`.
- [ ] Implement the 16 kHz current loop with 10 A per motor and 400 rpm limits, thermal derating, and
      integrator saturation.
- [ ] Implement the state machine `BOOT → SELF_TEST → IDLE → BALANCE → DAMPING → FAULT → E_STOP` and
      the watchdog (100 ms command timeout, link loss → `DAMPING`).
- [ ] Implement telemetry to the handset over CRSF: pack voltage, current, board temperature, state.

### 3.6. First balance and driving

- [ ] Balance on the safety rig at zero command; tune PD, then the full controller.
- [ ] Drive forward, reverse, turn, and spin in place under radio control.
- [ ] Verify tilt cutoff: it fires before the dangerous angle and overrides every command source.
- [ ] Verify command loss: setpoints go to zero while the balance torque is preserved.
- [ ] Two-minute thermal run at low load; record MOSFET, motor, and pack temperature.
- [ ] Verify the voltage thresholds (warning 36.0 V, limiting 33.0 V, stop 30.0 V) and regen limiting;
      start runs at about 90 % charge.
- [ ] Verify the charge interlock: with the charger plugged in, arming is refused and the motor output
      stays at zero.

### 3.7. Stage artifacts

- Donor board with the EFeru baseline and the rebuild instructions.
- `firmware/hoverboard-rs`: Rust controller with the FOC core behind FFI.
- Wiring diagram for the bench: left cable CRSF, right cable IMU, DC-DC branch, fuse, kill switch.
- Calibration records: battery divider, BNO055 offsets, channel map.
- Bench protocol: balance, drive, tilt cutoff, link loss, thermal, charge interlock.

### 3.8. Gate G1 — pass criteria

- [ ] Firmware boots from the pack, with no bench supply connected.
- [ ] Radio link up: CRSF frames at 420 kbaud on USART2, channel map and failsafe verified.
- [ ] Battery voltage within 0.2 V of a multimeter at two pack voltages.
- [ ] Both wheels spin in the commanded direction, first lifted and then on the ground.
- [ ] The robot balances on the safety rig at zero command for 10 minutes.
- [ ] Tilt cutoff works and overrides every command source.
- [ ] Loss of the radio link zeroes the setpoints without cutting the balance torque.
- [ ] A 2-minute thermal run leaves MOSFETs and motors within their limits.
- [ ] No sideboard is connected; the two cables carry only the receiver and the IMU.

### 3.9. Stage risks

| Risk | Impact | Mitigation |
|---|---|---|
| Chip is read-protected and the unlock erases the stock firmware | No way back to stock | Accepted; the EFeru baseline is the reference from then on |
| Board is the alternate layout | Wrong pin and sensor configuration | Try `BOARD_VARIANT 1`; record the variant |
| Receiver and IMU cannot share the right cable | Contention, no link | Stage 1 uses the left cable for CRSF and the right cable for I2C2 |
| Regen on a full pack trips the BMS | DC-link spike, fall | Start at about 90 % charge, limit regen current, measure the behaviour |
| The 2p pack sags under 10 A per motor | Early limiting, BMS cut | Keep the bench profile, derate early, log pack voltage |
| Hall or phase order is wrong | Jerk, overheating | Verify direction lifted first; fix in wiring |

---

## 4. Stage 2. Hip actuation, legs, and 3D balance (G2)

**Goal:** add two hip actuators and one planar 1-DOF four-bar leg per side, move the IMU off the shared
right cable, use USART3 for the hip driver or the mission computer, and control height and roll through
the hips while pitch and yaw stay on the wheels.

**Indicative duration:** 6–10 weeks.

> **Note:** the donor mainboard cannot drive a third and fourth motor. The hip actuators need their own
> drivers. The interface preference is UART; CAN is a deferred interface because the F103 CAN pins are
> not routed on this board.

### 4.1. Hip actuator selection and test

- [ ] Select the hip actuator after stage 1 balance is proven; derive the required torque from the
      measured centre-of-mass height and the measured wheel torque margin.
- [ ] Confirm the selection criteria: torque and thermal margin, integrated driver, UART interface,
      position/speed/current/temperature feedback, envelope, and mounting.
- [ ] Procure two actuators; this is the only long-lead item in the plan.
- [ ] Bench-test one actuator: current-loop step, overshoot, settling time, backlash, thermal rise.
- [ ] Record continuous and peak torque and the derating curve for the chosen actuator.

### 4.2. Leg mechanism

- [ ] Synthesize the four-bar link lengths for at least 100 mm of body height range and the required
      fore-aft offset; record the coupler curve.
- [ ] Design and manufacture the chassis plate, wheel forks, links, bearings, and pivot shafts.
- [ ] Add hard stops so a hip fault cannot drive the leg past the coupler curve.
- [ ] Verify the coupler curve, backlash, and stiffness on the bench before powering the hips.

### 4.3. Stage-2 I/O move

- [ ] Move the BNO055 from I2C2 to I2C1 remapped on PB8/PB9 with two soldered 30 AWG wires; check for
      test pads first.
- [ ] Re-run the IMU calibration and re-verify orientation after the move.
- [ ] Free the right cable: USART3 (PB10/PB11) now carries the hip driver or the mission computer,
      115200 8N1, framed with CRC.
- [ ] Keep the balance code independent of which I2C peripheral carries the IMU.
- [ ] Record that a CAN interface is deferred: it requires a future mainboard revision or an external
      adapter on the mission computer, not a firmware setting.

### 4.4. Control and allocation

- [ ] Implement the hip link on USART3: height or position setpoints, feedback, CRC, sequence, 300 ms
      validity window, and a timeout that holds the leg.
- [ ] Implement the hip height loop at 50–100 Hz outside the 200 Hz balance loop.
- [ ] Implement height and roll allocation through the hips, and pitch and yaw through the wheels.
- [ ] Implement saturation priority: balance first, then roll, then height. A saturated hip never
      reduces the wheel torque needed to stay upright.
- [ ] Keep the balance loop at 200 Hz; re-measure the achievable rate if the controller changes.
- [ ] Implement odometry for four actuators and publish it in telemetry.

### 4.5. Stage-2 tests

- [ ] Balance on the rig with the hips locked, then with the hips moving.
- [ ] Height sweep of at least 100 mm while balancing.
- [ ] Roll command through the hips with the wheels holding pitch.
- [ ] Drive, reverse, turn, and spin in place at two leg heights.
- [ ] Thirty-minute thermal run with four actuators; record hub motor, hip actuator, and MOSFET
      temperatures.
- [ ] Field runs with the tether: no fall in any stage-2 scenario.

### 4.6. Stage artifacts

- Two hip actuators with test records and derating curves.
- Four-bar legs, chassis, forks, and hard stops with a measured coupler curve.
- USART3 hip link and the updated I/O map.
- Control allocation documentation and tuned parameters.
- Stage-2 test protocol and logs.

### 4.7. Gate G2 — pass criteria

- [ ] Four actuators run stably: two hub motors and two hip actuators, each with its own current limit.
- [ ] Hip motion covers at least 100 mm of body height inside the coupler curve.
- [ ] Balance runs at 200 Hz; jitter and end-to-end latency measured, and a decision recorded if the
      donor board cannot hold them.
- [ ] Mission computer equivalent (PC) to mainboard: command 100 Hz, telemetry 200–500 Hz, frame loss
      measured.
- [ ] Loss of a valid command > 100 ms leads to `DAMPING`.
- [ ] The kill switch and E-stop disconnect actuator power; logging is preserved.
- [ ] No fall in the stage-2 field tests: rig, height sweep, roll command, drive and turn.
- [ ] Thirty-minute thermal run without overheating of hub motors, hip actuators, and MOSFETs.
- [ ] The communication protocol is versioned and covered by tests.

### 4.8. Stage risks

| Risk | Impact | Mitigation |
|---|---|---|
| Hip actuator torque or thermal margin is insufficient | Redesign, slip | Measure the required torque in stage 1; bench thermal test before ordering |
| The actuator has no suitable UART interface | Extra driver board, delay | Prefer integrated UART drivers; CAN requires an external adapter |
| Coupler curve limits the height range | Missed requirement | Synthesize link lengths early; verify on the bench; hard stops |
| Soldering the IMU to I2C1 fails | Stage 2 blocked | Check PB8/PB9 test pads; fallback keeps the IMU on I2C2 and leaves USART3 unused |
| Added mass raises the centre of mass | More hip torque needed | Keep the pack and electronics low and centred; measure mass and CoM |
| Hip motion disturbs balance | Fall | Allocation priority, rate separation, hard stops, tether |

---

## 5. Stage 3. Mission computer, lidar, and basic Rust software (G3)

**Goal:** combine the mainboard and the mission computer (Raspberry Pi 5 class) into one system:
transport, telemetry, lidar, state estimation, and supervisor. Balancing remains on the mainboard.

**Indicative duration:** 4–6 weeks.

### 5.1. Mission computer platform

- [ ] Install a 64-bit OS (Ubuntu 24.04 LTS arm64 or Raspberry Pi OS 64-bit) on NVMe.
- [ ] Configure cooling and temperature control; verify throttling under load.
- [ ] Add a dedicated 5 V / 5 A converter for the mission computer, rated for at least 60 V input, on
      its own fused branch. The stage 1 converter is 5 V / 1 A for the receiver and IMU only.
- [ ] Configure the network, SSH/VPN, disable unnecessary services; isolate the robot network.
- [ ] Prepare systemd units for `jbot-node`, `zenohd`, the lidar driver, and the ROS2 bridge;
      `Restart=on-failure`, watchdog, resource limits.
- [ ] Make the root FS predominantly read-only, with a separate partition for maps and logs (P2).
- [ ] Configure cross-compilation for `aarch64-unknown-linux-gnu` (or musl) and CI builds.

### 5.2. Mission computer–mainboard transport

- [ ] Implement the `Transport` trait in `jbot_hardware`: `SerialCobs`, `Mock`; reserve a `SocketCan`
      implementation for a future CAN interface.
- [ ] Bring up the primary link on USART3 (PB10/PB11) at 115200 8N1, COBS + CRC-16, and verify the
      waveform and termination with an oscilloscope.
- [ ] Add a second UART profile at 921600 for the same link and record the achievable rate and error
      rate before raising the default.
- [ ] Implement frame reception/transmission, version, sequence, CRC, and timeout validation.
- [ ] Implement metrics: frame loss, CRC errors, command age, latency. Bus-off is not applicable until
      a CAN interface exists.
- [ ] Implement transport degradation: on sustained CRC errors or loss, hold the last valid setpoint
      inside the validity window, then `DAMPING`.
- [ ] Implement mission computer–mainboard heartbeat and handshake; on loss — `DAMPING` on the mainboard.
- [ ] Reserve an optional USB-CAN adapter for diagnostics only; it is never part of the control path.

### 5.3. Time synchronization

- [ ] Implement mainboard–mission computer time offset and drift estimation (EWMA) from frame timestamps.
- [ ] Provide a GPIO sync pulse / 1 PPS (optional).
- [ ] Ensure timestamps at the sensor sampling instant, not at the processing instant.
- [ ] Verify IMU/odometry/lidar desynchronization ≤ 10 ms.

### 5.4. Lidar

- [ ] Select a lidar: 2D 360°, ≥ 10 m, ≥ 10 Hz (RPLIDAR S2/A2M12, LDROBOT and similar) or a 3D lidar.
- [ ] Implement a Rust driver (or a wrapper over a high-performance driver) that publishes scans/point
      clouds.
- [ ] Bring measurements to the `laser_link` frame; verify angular and range calibration.
- [ ] Implement lidar health monitoring: rate, gaps, staleness, temperature.
- [ ] Implement raw scan recording for replay and regression tests.

### 5.5. Basic `jbot_node` software

- [ ] Assemble the composition root: configuration, transport, estimation, telemetry publication,
      supervisor.
- [ ] Implement Tokio tasks per the architecture: ingress, estimation, control egress, telemetry,
      supervisor, shutdown.
- [ ] Implement latest-value semantics for commands (`watch`) and bounded queues for telemetry.
- [ ] Implement signal handling and graceful shutdown (`CancellationToken`).
- [ ] Implement TOML configuration with typing, validation, and hash logging.
- [ ] Implement structured logging (`tracing`, JSON) and metrics.
- [ ] Implement publication to Zenoh: state, telemetry, diagnostics, commands.
- [ ] Implement the `zenoh-bridge-ros2dds` bridge for RViz/rosbag (optional in this stage).

### 5.6. State estimation

- [ ] Implement the EKF from IMU and wheel odometry at 100–200 Hz.
- [ ] Estimate pose, velocity, gyroscope zero bias; publish `odom → base_link`.
- [ ] Implement delay and outlier compensation; validate on HIL.
- [ ] Compare the estimate against a reference (test bench, external marker) and record errors.

### 5.7. HIL and fault tolerance

- [ ] Assemble a HIL test bench: mainboard, hub motors, hip actuators, mission computer, lidar (or a
      lidar simulator).
- [ ] Verify fault injection: killing `jbot-node`, UART disconnection, lidar staleness, mission
      computer reboot.
- [ ] Verify that a mission computer failure does not cause a fall: the mainboard transitions to
      `DAMPING`/`IDLE` on timeout.
- [ ] Verify boot to safe `IDLE` ≤ 10 s.
- [ ] Conduct a short soak test (2–4 hours) with metric logging.

### 5.8. Stage artifacts

- Configured mission computer image, systemd units, deployment instructions.
- `jbot_hardware`: UART, lidar, mock transport; a reserved CAN path.
- `jbot_node`: EKF at 100–200 Hz.
- `jbot_node` v0.1: telemetry, supervisor, configuration, logging.
- Lidar driver and scan recording pipeline.
- HIL protocol: delays, jitter, faults, recovery.
- Telemetry dashboard (Rerun/Zenoh) and a set of metrics.

### 5.9. Gate G3 — pass criteria

- [ ] End-to-end command delay mission computer–mainboard–actuator ≤ 10 ms (measured).
- [ ] Loss of heartbeat/command > 100 ms → `DAMPING`; a mission computer failure does not cause a fall.
- [ ] Stable UART link with telemetry at 200–500 Hz; losses and errors within budget. An optional CAN
      interface is verified or explicitly deferred.
- [ ] Lidar publishes data ≥ 10 Hz; desynchronization with IMU ≤ 10 ms.
- [ ] EKF runs at 100–200 Hz; pose and velocity are reproducible on replay.
- [ ] Boot to `IDLE` ≤ 10 s; logs and configuration are preserved.
- [ ] HIL fault tests passed; metrics are available to the operator.
- [ ] `cargo fmt`, `clippy -D warnings`, `cargo test` pass; aarch64 cross-build succeeds.

### 5.10. Stage risks

| Risk | Impact | Mitigation |
|---|---|---|
| The 115200 UART cannot carry the telemetry rate | Delays, losses | Measure early; raise to 921600; reduce telemetry before adding hardware |
| Optional CAN adapter on USB is unstable | Diagnostics gaps | Keep CAN out of the control path; UART remains primary |
| Time desynchronization | EKF/SLAM errors | Timestamps at sampling, offset estimation, GPIO sync |
| Lidar produces noise/little data | Poor SLAM | Choose a proven model, calibration, filtering |
| Mission computer throttling | Task gaps | Cooling, limits, frequency degradation |
| Zenoh/ROS2 bridge complexity | Integration delay | Start with Zenoh, add the bridge later; topic contracts |

---

## 6. Stage 4. SLAM, navigation, and autonomous movement (G4)

**Goal:** build a map, localize in it, and ensure autonomous point-to-point movement while respecting
safety limits.

**Indicative duration:** 6–10 weeks.

### 6.1. SLAM selection and integration

- [ ] Conduct ADR-008: Rust-native SLAM vs `slam_toolbox`/Cartographer via the ROS2 bridge.
- [ ] Implement the pipeline: lidar → filtering → scan matching → map → pose.
- [ ] Implement loop closure and relocalization when returning to a known area.
- [ ] Implement map storage: resolution ≤ 5 cm, area ≥ 1000 m², PGM/YAML format or a documented
      equivalent.
- [ ] Implement map versioning and binding to the robot configuration.
- [ ] Implement map loading ≤ 10 s for 1000 m² (P2).
- [ ] Implement SLAM data recording/replay for regression.

### 6.2. Localization

- [ ] Implement map localization: scan matching + EKF, 100–200 Hz for estimation, 10–30 Hz for lidar
      correction.
- [ ] Implement the `map → odom → base_link` tree per REP-105.
- [ ] Implement "kidnapping" detection and relocalization ≤ 10 s (P2).
- [ ] Implement degradation: on staleness/quality degradation — speed limiting, then stop.
- [ ] Verify accuracy: ≤ 50 mm and 2° (RMS) relative to reference markers.

### 6.3. Obstacle maps and costmap

- [ ] Implement the static map, obstacle map, and inflation accounting for dimensions.
- [ ] Implement a dynamic obstacle layer based on lidar and data lifetime.
- [ ] Implement the staleness policy: data older than 300 ms → stop.
- [ ] Implement safe distance: < 0.25 m at low speed → stop.
- [ ] Implement speed limiting near obstacles (≤ 0.3 m/s).

### 6.4. Planning and motion control

- [ ] Implement a global planner A*/Dijkstra accounting for traversability and dimensions.
- [ ] Implement a local planner/controller: DWA, MPC, or equivalent accounting for balance constraints
      and the stage-2 height/roll allocation.
- [ ] Account for limits: acceleration, jerk, tilt, torque, slip.
- [ ] Implement path following with speed limiting and smooth braking.
- [ ] Implement recovery behaviors: costmap clearing, replanning, turning, backing off.
- [ ] Implement teleoperation priority over autonomous movement.
- [ ] Implement the behavior layer: waypoints, patrol, pause, return, emergency stop.
- [ ] Isolate navigation in a separate process/task with resource limits.

### 6.5. Autonomous tests

- [ ] First run navigation scenarios on the HIL bench with sensor noise.
- [ ] Build a room map ≥ 100 m² with loop closure; measure drift (≤ 1% of path).
- [ ] Perform 100 autonomous runs to the goal in a static environment; success rate ≥ 95%,
      collisions — 0.
- [ ] Verify dynamic obstacles: stop/replan ≤ 1 s.
- [ ] Verify lidar failure, data staleness, robot "kidnapping".
- [ ] Verify operation without ROS2 and with the ROS2/RViz bridge.
- [ ] Conduct regression on recorded logs; compare metrics with the reference.

### 6.6. Stage artifacts

- `jbot_navigation`: costmap, global/local planners, behavior layer.
- SLAM pipeline and map server with versioning.
- Map localization and REP-105 TF tree.
- A set of maps and configurations for the test site.
- Protocols: 100 runs, dynamic obstacles, degradation, localization accuracy.
- Navigation report and reproducible logs.

### 6.7. Gate G4 — pass criteria

- [ ] A map ≥ 100 m² is built with loop closure; drift ≤ 1% of path.
- [ ] Localization ≤ 50 mm and 2° (RMS); pose publication delay ≤ 100 ms.
- [ ] 100 autonomous runs: ≥ 95 successful, 0 collisions.
- [ ] Dynamic obstacles: stop/replan ≤ 1 s.
- [ ] Data staleness > 300 ms → stop; distance < 0.25 m → stop.
- [ ] Works without ROS2; the ROS2/RViz bridge does not block the safety loop.
- [ ] Navigation is isolated and does not affect balancing jitter.
- [ ] Log-based regression tests are reproducible.

### 6.8. Stage risks

| Risk | Impact | Mitigation |
|---|---|---|
| SLAM in Rust is not mature enough | Stage slip | Early ADR; fallback `slam_toolbox` via the bridge |
| Balance dynamics complicate the local planner | Falls during maneuvers | Acceleration/tilt limits, conservative speed, HIL validation |
| Poor localization in long corridors | Mission failures | Loop closure, multiple sensors, landmarks |
| Dynamic obstacles | Collisions | Costmap layers, safe distance, stop priority |
| Mission computer resources under SLAM + navigation | Throttling, delays | Separate processes, limits, NVMe, cooling |

---

## 7. Stage 5. Attachments, docking station, and AI functions (G5)

**Goal:** extend the platform with a camera, AI accelerator, manipulator, and auto-charging without
compromising balancing, safety, and baseline navigation.

**Indicative duration:** 6–10 weeks.

### 7.1. Extension interfaces

- [ ] Implement mechanical and electrical interfaces for the camera (USB 3.0 / CSI-2, 5 V with
      protection).
- [ ] Implement the manipulator interface: 12 V / 5 A, protection, separate branch, a reserved UART
      link (an optional CAN link is possible through a USB-CAN adapter on the mission computer).
- [ ] Provide a PCIe/USB reserve for the AI accelerator (Hailo-8L or equivalent), accounting for power
      and heat.
- [ ] Implement an external diagnostic isolated UART port, with optional CAN (P2).
- [ ] Ensure that extensions do not increase balancing loop jitter by > 50 µs.

### 7.2. Camera and AI

- [ ] Connect an RGB-D camera (RealSense D435i / OAK-D), implement capture ≥ 30 fps at 640×480.
- [ ] Implement calibration, storage in configuration, and verification at startup.
- [ ] Implement camera synchronization with IMU/lidar ≤ 10 ms (P2).
- [ ] Choose the runtime: `ort`, `tract`, or `candle`; choose the accelerator or CPU.
- [ ] Implement object detection ≥ 10 Hz on the mission computer or AI accelerator (P2).
- [ ] Publish detections with timestamps; do not block navigation and control.
- [ ] Implement degradation: camera failure does not block lidar navigation.
- [ ] Verify the AI accelerator's power and thermal budget under sustained load.

### 7.3. Manipulator (P3)

- [ ] Implement an emulator of the manipulator node over the reserved UART link to verify power, timing,
      and E-stop.
- [ ] Implement the control interface in the behavior layer, outside the balance loop.
- [ ] Implement supervisor limits when the load is extended (speed, torque, tilt).
- [ ] Implement emergency stop of the manipulator via the common E-stop.
- [ ] Verify that manipulator failure/disconnection does not compromise balancing and navigation.

### 7.4. Docking station and auto-charging

- [ ] Design guides, power contacts, docking sensor, and an IR/UART channel.
- [ ] Implement de-energizing of contacts until docking is confirmed; reverse polarity and
      short-circuit protection.
- [ ] Implement the state protocol: `IDLE`, `APPROACH`, `DOCKED`, `CHARGING`, `FAULT`.
- [ ] Implement the return-to-dock and charging mission under the donor pack BMS control.
- [ ] Ensure docking accuracy ≤ ±20 mm and ±2° in heading.
- [ ] Limit docking station power ≤ 150 W and charge through the donor pack's original charge port.
- [ ] Implement charge monitoring, SoC/SoH, and event logging.
- [ ] Verify that the docking station is not mandatory for baseline operation.

### 7.5. System validation

- [ ] Conduct a 24-hour run with battery replacement and scheduled pauses (P2).
- [ ] Verify E-stop, brownout, overheating, loss of communication, battery disconnection.
- [ ] Verify EMI after installing the camera, AI accelerator, and manipulator.
- [ ] Verify the thermal mode of all components in typical and aggressive cycles.
- [ ] Verify runtime: ≥ 30 min typical cycle, ≥ 20 min aggressive (P1).
- [ ] Update documentation: operator manual, maintenance, protocols, BOM.
- [ ] Conduct acceptance tests per `docs/specification.md`, section 12.

### 7.6. Stage artifacts

- Installed camera, AI accelerator, manipulator (or its emulator), and docking station.
- AI detection pipeline with timestamps and degradation.
- Docking station firmware/software and the auto-charging mission.
- Updated schematics, BOM, configurations, and calibrations.
- System and acceptance test protocols.
- Final report and delivery package.

### 7.7. Gate G5 — pass criteria

- [ ] Camera and AI operate without affecting balancing jitter and navigation.
- [ ] Failure of any extension does not compromise balancing and baseline navigation.
- [ ] The manipulator (or emulator) is controlled over the reserved UART link, the E-stop is shared,
      and balancing is not compromised.
- [ ] Docking station: docking ≤ ±20 mm / ±2°, contacts de-energized until docking, charging under BMS.
- [ ] Return to dock and charging work as a mission.
- [ ] 24-hour run without critical failures.
- [ ] All P1 SRS requirements are confirmed by protocols; P2 ones are implemented or justified.
- [ ] The documentation and source code package has been delivered.

### 7.8. Stage risks

| Risk | Impact | Mitigation |
|---|---|---|
| AI accelerator power/heat | Brownout, throttling | Separate protection, power budget, tests under load |
| Manipulator compromises balancing | Fall | Supervisor, limits, isolation from the balance loop |
| Inaccurate docking | Charging failure | Guides, IR/UART, multiple attempts, sensors |
| EMI from new nodes | Communication failures | Shielding, ferrites, spacing, spectrum verification |
| AI integration complexity | Stage slip | Start with the CPU model, accelerator as optimization |

---

## 8. Cross-cutting work (performed in all stages)

### 8.1. Safety

- [ ] Hazard analysis and FMEA from the very beginning; update at every gate.
- [ ] E-stop specification: two-channel NC loop, disconnection of the power section, preservation of
      logic; the kill switch in the battery line remains the only stop that works with a hung firmware.
- [ ] Watchdog chain: kill switch → mainboard IWDG → 100 ms command timeout → mission computer
      heartbeat → systemd → supervisor.
- [ ] Limits on current, temperature, tilt, voltage; latched faults and manual reset.
- [ ] Proof-test of E-stop and protections at least once every 12 months (P2).
- [ ] Safety audit before every field test.

### 8.2. Testing and CI

- [ ] `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test` on every PR.
- [ ] Protocol property tests: round-trip, corrupted frames, sequence wraparound.
- [ ] Integration tests: UART loopback, `vcan` for the reserved CAN path, timeouts, degradation.
- [ ] aarch64 cross-build and firmware build in CI.
- [ ] `cargo deny`, `cargo audit`.
- [ ] HIL tests and regression on recorded logs.
- [ ] Definition of Done for control loop changes: unit + HIL + jitter/delay + rollback.

### 8.3. Documentation and configuration

- [ ] Maintain ADRs for key decisions (controller, drivers, SLAM, middleware, AI runtime).
- [ ] Version the protocol and maintain a firmware/host compatibility matrix.
- [ ] TOML configuration: typing, validation, hash logging.
- [ ] Calibrations in non-volatile memory; recovery after reboot.
- [ ] Structured logs (`tracing`) with node/task/robot ID; no high-frequency logging in the loop.
- [ ] Metrics: frame loss, CRC, command age, jitter, voltage, temperatures, failures.
- [ ] Mainboard blackbox and log export (P2).

### 8.4. Procurement and production

- [ ] Order the added stage 1 parts after the BOM freeze: fuse, kill switch, DC-DC, receiver, IMU,
      probe.
- [ ] Treat the stage 2 hip actuator as the only long-lead item; place the order after stage 1 balance
      is proven.
- [ ] Provide a spare kit of fuses, connectors, and consumables.
- [ ] Verify component availability and alternatives (second source).
- [ ] Manufacture/order PCBs and cable assemblies with margin.

### 8.5. Project management

- [ ] Maintain a risk register and update it at every gate.
- [ ] Maintain the requirements traceability matrix (SRS → task → test → protocol).
- [ ] Hold a weekly progress review and a monthly risk review.
- [ ] Record deviations from the SRS only through approval and an SRS addendum.

---

## 9. Milestones and dependencies

| Milestone | Content | Gate | Depends on |
|---|---|---|---|
| S0 | Documentation, workspace, and BOM baseline | B1 | — |
| S1 | Donor wheels and balance on the safety rig | G1 | S0, donor inspection |
| S2 | Hips, legs, and 3D balance | G2 | S1, hip actuator |
| S3 | Mission computer, transport, telemetry, lidar, EKF | G3 | S2, mission computer, lidar |
| S4 | SLAM, localization, navigation, 100 autonomous runs | G4 | S3 |
| S5 | Camera/AI, manipulator, docking station | G5 | S4 |
| S6 | Acceptance tests and delivery | — | S5 |

**Key dependencies:**

- Stage 1 cannot start until the donor board passes inspection; it does not wait for procurement.
- Stage 2 requires the hip actuator and the stage-1 balance result, which fixes the required torque.
- Stage 3 requires a stable protocol and firmware from stage 1 and the stage-2 I/O move.
- Stage 4 requires working localization and lidar from stage 3.
- Stage 5 requires free mission computer resources and power/thermal margin.

---

## 10. Traceability matrix (summary)

| SRS section | Requirements | Stages |
|---|---|---|
| 3. Computing architecture | SYS-01…SYS-08 | 1, 2, 3 |
| 4. Autonomy and power | AUT-01…AUT-30 | 1, 3, 5 |
| 5. Mobility and dynamics | MOB-01…MOB-30 | 1, 2, 4 |
| 6. Navigation | NAV-01…NAV-35 | 3, 4 |
| 7. Extensibility | EXT-01…EXT-33 | 5 |
| 8. Interfaces and software | INT-01…INT-22 | 1, 2, 3, 4, 5 |
| 9. Safety | SAF-01…SAF-13 | all |
| 10. Reliability | REL-01…REL-08 | 1, 2, 5 |

---

## 11. Project acceptance criteria

1. All P1 requirements are confirmed by test protocols.
2. P2 requirements are implemented or have an agreed deferral justification.
3. There are no critical failures or open safety defects.
4. Balancing loop jitter ≤ 50 µs at the 200 Hz balance rate; command loss → `DAMPING` within 100 ms.
5. Autonomous navigation: ≥ 95 successful runs out of 100 without collisions.
6. Runtime from the donor 10s2p pack (158 W·h) in a typical cycle ≥ 30 min, confirmed by measurement.
7. The documentation and source code package has been delivered in full.
8. E-stop and protections have passed proof-test; failure behavior complies with the SRS.

---

## 12. Open questions affecting the plan

1. Hip actuator: model, torque, and thermal margin; integrated UART driver preferred, CAN deferred.
2. Does the mission computer appear in stage 2 or stage 3? In stage 2 USART3 is already taken by the hip
   driver, so both devices cannot share the right cable.
3. Leg topology: decided — 1-DOF four-bar. Remaining: link lengths, spring, height range, and
   E-position.
4. Lidar and camera: specific models, mounting, power, thermal budget.
5. SLAM: Rust-native stack or `slam_toolbox` via the ROS2 bridge?
6. AI: Hailo-8L or CPU-only; runtime `ort` / `tract` / `candle`.
7. Firmware update path: SWD over the debug probe during stages 1–2; USB DFU or a CAN bootloader later.
8. Docking station: contact standard and the charging protocol through the donor pack's charge port.
9. Mass, payload, and height range after prototype manufacturing.
10. Requirements for the enclosure, connectors, and maintainability.
11. Regen and BMS behaviour on the donor 2p pack: how much regen current it accepts at high SoC.
12. Whether a future mainboard revision exposes CAN, or CAN stays an external adapter on the mission
    computer.

---

## 13. Immediate actions (first 2 weeks)

- [ ] Disconnect the sideboards, inspect the donor board and harnesses, and record the continuity
      measurements.
- [ ] Build and flash the EFeru baseline; verify the debug UART and calibrate the battery divider.
- [ ] Wire the XR1 to the left cable, bind it, and verify the channel map and failsafe.
- [ ] Mount the BNO055 on I2C2 and run the full calibration.
- [ ] Complete `docs/bom.md` for the added parts (fuse, kill switch, DC-DC, receiver, IMU) and place
      that order.
- [ ] Start `firmware/hoverboard-rs`: workspace, clocks, RTIC tasks, FOC core via FFI.
- [ ] Set up the safety rig and the first-power-on checklist.
- [ ] Start hazard analysis and the E-stop specification.
