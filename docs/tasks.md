# Task Breakdown — JBOT-WheeledBiped

| Parameter | Value |
|---|---|
| Document | Detailed task breakdown (Task Breakdown) |
| Code | TB-001 |
| Version | 1.1 |
| Date | 2026-11-12 |
| Status | Working document |
| Basis | `docs/specification.md`, `docs/roadmap.md`, `docs/architecture.md`, `docs/system_requirements.md`, `docs/hardware.md`, `docs/bom.md`, `firmware/README.md` |
| Scope | Donor hoverboard stage 1 → hips stage 2 → mission computer and navigation → extensions → acceptance |

---

## 0. Rules for working with the plan

### 0.1. Identifier structure

- `M2` — hoverboard testbed and bring-up: donor board, EFeru baseline, radio, IMU, first balance.
- `M3` — Rust balance controller and low-level firmware for the donor mainboard, stages 1–2.
- `M4` — sensor drivers and transport on the mission computer.
- `M5` — mission computer onboard software: composition, estimation, supervisor, middleware.
- `M6` — navigation, localization and SLAM.
- `M7` — safety and fault tolerance (cross-cutting module).
- `M8` — integration, HIL, system and acceptance tests.
- `M9` — extensions: camera, AI, manipulator, docking station.
- `M10` — cross-cutting work: CI, documentation, configuration, procurement, management.

Format: `M<module>-<number>`. Module identifiers `M2`–`M10` are stable; the former `M1` simulation
track is retired, and `M2` is the donor hoverboard bring-up. Hardware scope lives in the BOM.

### 0.2. Statuses and priorities

- `[ ]` — not started; `[~]` — in progress; `[x]` — done; `[!]` — blocked.
- **P1** — mandatory for acceptance of the base version.
- **P2** — target requirement; implemented by the end of the corresponding stage or with a justified
  deferral.
- **P3** — interface reservation or prospective implementation.
- **Gate G1…G5** — stage acceptance gate; tasks included in a gate cannot be closed partially. `B1`
  is the BOM baseline gate before stage 1.

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
M2 donor bring-up: sideboards removed, EFeru baseline, radio, IMU, first balance (G1)
  → M3 stage 1: 200 Hz balance, drive, turn, spin in place
  → M3 stage 2: hips, legs, 3D balance
  → M4 mission computer–mainboard transport and lidar
  → M5 EKF and supervisor
  → M6 SLAM and localization
  → M6 planners and autonomous runs
  → M9 extensions
  → M8 acceptance
```

In parallel: `M7` (safety), `M10` (CI, documentation, procurement), and BOM completion.

### 0.5. Module summary

| Module | Stages | Tasks | Estimate, person-days | Key result | Gate |
|---|---|---:|---:|---|---|
| M2. Hoverboard testbed and bring-up | 1 | 27 | 20–30 | Sideboard removal, EFeru baseline, CRSF, BNO055 | G1 |
| M3. Rust balance controller and low-level firmware | 1–2 | 48 | 55–80 | 16 kHz FOC, 200 Hz balance, hips, protocol | G1–G2 |
| M4. Sensor drivers and mission computer transport | 3 | 27 | 30–45 | UART, lidar, camera, synchronization | G3 |
| M5. Mission computer onboard software | 3 | 26 | 35–50 | `jbot_node`, EKF, supervisor, Zenoh | G3 |
| M6. Navigation and SLAM | 4 | 39 | 55–75 | Maps, localization, costmap, planners | G4 |
| M7. Safety and fault tolerance | all | 16 | 12–18 | Safety case, E-stop, watchdog, fault codes | all |
| M8. Integration and testing | 3–5 | 22 | 30–45 | HIL, system and acceptance protocols | G3–G5 |
| M9. Extensions | 5 | 31 | 40–60 | Camera/AI, manipulator, docking station | G5 |
| M10. Cross-cutting work | all | 22 | 18–28 | CI, documentation, procurement, management | all |
| **Total** | | **258** | **295–431** | | |

> Estimates are ideal person-days for a team of 2–3 engineers. The calendar plan and buffers are
> defined separately in `docs/roadmap.md`.

---

## 2. Hardware bill of materials

Power-stage design and protection work is no longer tracked as a separate task module. Hardware scope
is captured as a component and assembly list in [`docs/bom.md`](bom.md). That list is the input to
procurement, bench assembly, and the interface freeze.

Gate `B1` is the BOM baseline. Before ordering:

- [ ] Complete the donor reuse list and the added-parts list in the BOM.
- [ ] Record that the mainboard, both hub motors, the 10s2p pack, the power button, and the charge port
      are reused and not ordered.
- [ ] Confirm that no new MCU or motor controller is bought for stages 1–2; the donor board is fixed.
- [ ] Verify hip actuator torque and thermal margin for the measured platform mass before ordering.
- [ ] Resolve every `TBD` item that affects an interface, envelope, or connector.
- [ ] Size the main fuse and kill switch after the first current measurement on the real pack.
- [ ] Record substitutions and their ADR links.

---

## 3. M2. Hoverboard testbed and bring-up

**Goal:** turn the stock hoverboard into an instrumented, radio-controlled balancing testbed: remove
the sideboards, prove the board and the pack, flash the EFeru baseline, wire the CRSF receiver and the
BNO055, implement the charge interlock and safety rig, and obtain the first balance runs.

**Platform:** donor STM32F103RCT6 mainboard at 72 MHz, two direct-drive hub motors, 10s2p 36 V pack
with BMS, firmware-held power latch on PA5, charge detect on PA12.

**Inputs:** `docs/hardware.md` sections 2–6; `firmware/README.md` sections 1–4; `docs/bom.md`
sections 1–3; `docs/specification.md` sections 4, 5, 9.
**Outputs:** inspected donor board, EFeru baseline, Rust controller skeleton, calibration records,
safety rig, G1 protocol.

### 3.1. Donor board preparation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-01 | Disconnect both sideboards, remove the deck and covers, and keep the mainboard, motors, pack, button and charge port | P1 | — | No sideboard is connected; the two 4-pin cables carry only the receiver and the IMU |
| [ ] M2-02 | Visual inspection of the mainboard and harnesses: MOSFETs, shunts, connectors, capacitors, chafing | P1 | M2-01 | Inspection record with photographs; every fault cleared or the board rejected |
| [ ] M2-03 | Continuity inspection: phase-to-phase resistance, hall toggling, no phase-to-ground or phase-to-15 V short, no DC-link short | P1 | M2-02 | Measurements recorded and compared between the two motors; no short found |
| [ ] M2-04 | Insulate the 15 V pins on both cables; mount the board, pack, fuse, kill switch, DC-DC and receiver on the bench frame | P1 | M2-03 | No exposed live 15 V pin; the kill switch breaks the pack positive; power wiring short and twisted |

### 3.2. EFeru baseline

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-05 | Recreate the pinned EFeru tree, apply `eferu-jbot.patch`, and build `VARIANT_PWM` | P1 | M2-01 | Build artifacts exist; size matches the documented profile |
| [ ] M2-06 | Flash over SWD on PA13/PA14, unlocking RDP if needed; record that the stock firmware is destroyed | P1 | M2-05 | Firmware boots from the pack with the startup melody and the expected beeps |
| [ ] M2-07 | Verify the debug UART on PA2 TX at 115200 8N1: `in1/in2`, `cmdL/cmdR`, `BatADC/BatV`, `TempADC/Temp` | P1 | M2-06 | Line prints every 125 ms; values respond to stick input |
| [ ] M2-08 | Calibrate `BAT_CALIB_REAL_VOLTAGE` and `BAT_CALIB_ADC` against a multimeter at two pack voltages | P1 | M2-07 | `BatV` within 0.2 V at both points |
| [ ] M2-09 | Verify hall and phase direction on both wheels, lifted first; correct by swapping phases or hall order, never in software | P1 | M2-06 | Both wheels turn in the commanded direction at low current |

### 3.3. Radio link

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-10 | Bind the LiteRadio 3 Pro to the XR1 on the same ExpressLRS major version | P1 | — | Bind confirmed; link quality visible on the handset |
| [ ] M2-11 | Wire the XR1 CRSF output to the left cable: USART2, PA3 RX / PA2 TX, 420 kbaud, 3.3 V logic | P1 | M2-10 | CRSF frames decoded; no other transmitter on the line |
| [ ] M2-12 | Use the CRSF-to-PWM adapter on the right cable (PB10/PB11) only for the first EFeru bench test, then remove it | P1 | M2-11, M2-07 | EFeru responds to sticks through the adapter; adapter removed before the Rust controller runs |
| [ ] M2-13 | Record the channel map and set the transmitter failsafe; verify link loss with the motors lifted | P1 | M2-11 | CH1 steering, CH2 throttle, CH3 arm, CH4 mode, CH5 rate; loss of frames zeroes setpoints |
| [ ] M2-14 | Implement the arming path: only the radio can arm, and the arm switch is checked at every transition | P1 | M2-11, M3-19 | Motors arm only from the switch; disarm is immediate and latched |

### 3.4. IMU

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-15 | Mount the BNO055 rigidly, low and centred, and wire I2C2 to the right cable: PB10 SCL, PB11 SDA, 100 kHz | P1 | M2-04 | No play in the mount; the bus answers at 0x28 or 0x29 |
| [ ] M2-16 | Run accelerometer, gyroscope and magnetometer calibration; persist the offsets | P1 | M2-15 | Calibration status 3; offsets survive reboot |
| [ ] M2-17 | Verify 100 Hz fusion, gyro propagation, and the pitch/roll sign convention against wheel direction | P1 | M2-16, M2-09 | Positive pitch leans forward; a slow tilt tracks without drift |

### 3.5. Power, charge and safety

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-18 | Implement the charge interlock on PA12: with the charger plugged in, arming is refused and torque stays zero | P1 | M2-08, M2-14 | Plugging in the charger inhibits arming; unplugging does not arm by itself |
| [ ] M2-19 | Implement the voltage thresholds 36.0 / 33.0 / 30.0 V and regen limiting; start runs at about 90 % charge | P1 | M2-08 | Warning, limiting and stop trigger; no BMS trip on a full pack |
| [ ] M2-20 | Verify the kill switch and the E-stop with the motors running; logic stays powered and logging is preserved | P1 | M2-04, M2-09 | Motor power is removed; the mainboard state is logged |
| [ ] M2-21 | Build the safety rig and the tether; write the first-power-on and per-run checklists | P1 | M2-04 | The rig holds the body at any tilt; checklists are mandatory before a run |

### 3.6. First balance runs

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M2-22 | Bring up the Rust controller far enough to spin one wheel: RTIC tasks, clock tree, and the FOC core through FFI (M3-02 continues the full crate) | P1 | M2-06 | The Rust binary boots and spins one wheel through the C core |
| [ ] M2-23 | Balance on the rig at zero command; tune PD first | P1 | M2-17, M2-22 | Ten minutes on the rig without touching the stops |
| [ ] M2-24 | Drive, reverse, turn, and spin in place under radio control | P1 | M2-23 | All four motions executed; current within 10 A per motor |
| [ ] M2-25 | Verify tilt cutoff and command-loss policy: tilt overrides every source; command loss zeroes setpoints without cutting balance torque | P1 | M2-23 | Both behaviours demonstrated with the rig |
| [ ] M2-26 | Two-minute thermal run; record MOSFET, motor and pack temperatures | P1 | M2-24 | No overheating; derating does not trip falsely |
| [ ] M2-27 | Close gate G1: record the measurements and the open issues | P1 | M2-07…M2-26 | All G1 criteria confirmed by protocol |

---

## 4. M3. Rust balance controller and low-level firmware

**Goal:** implement deterministic firmware on the donor mainboard: hall and current acquisition, the
16 kHz FOC current loop reused from the generated C core, the 200 Hz balance loop, the stage-2 hip
link, the communication protocol, watchdog, and safe states.

**Platform:** STM32F103RCT6 at 72 MHz, 256 KB flash, 48 KB RAM. The board has exactly two 3-phase
bridges, so the wheel motors are driven on-board and the stage-2 hip actuators use their own drivers.
CAN is not routed and stays a deferred interface.

**Inputs:** `docs/architecture.md` sections 3.2, 5, 7; `docs/specification.md` sections 3, 5, 8, 9;
`docs/hardware.md`; `firmware/README.md` section 5.
**Outputs:** `firmware/hoverboard-rs`, `jbot_protocol`, bench records, actuator/jitter/latency/E-stop
protocols.

### 4.1. Firmware architecture and HAL

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-01 | Confirm the donor mainboard as the fixed controller for stages 1–2 and close ADR-001: 72 MHz, 16 kHz FOC, 200 Hz balance headroom, flash/RAM map, pin budget | P1 | — | 200 Hz balance with 16 kHz FOC and CRSF measured on the board; ADR records that no new MCU is bought |
| [ ] M3-02 | Create `firmware/hoverboard-rs` as its own workspace: `thumbv7m-none-eabi`, `stm32f1xx-hal` 0.11, RTIC 2, build and memory map | P1 | M3-01, M2-06 | Firmware builds, flashes and starts on the donor board |
| [ ] M3-03 | Implement initialization of clocks, timers, 16 kHz PWM, ADC/DMA, hall capture, I2C, GPIO and watchdog | P1 | M3-02 | All peripherals pass the startup self-test |
| [ ] M3-04 | Ensure safe default states: motor output off after reset, outputs in a known state | P1 | M3-03 | Verified with an oscilloscope; complies with SAF-10 |

### 4.2. Sensors and actuators

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-05 | Implement the BNO055 driver over I2C: fusion configuration, calibration persistence, timestamping, gyro propagation to 200 Hz | P1 | M3-03 | Fusion valid at 100 Hz; the propagated pitch at 200 Hz is stable |
| [ ] M3-06 | Implement self-test, gyro and accelerometer bias calibration, and calibration persistence | P1 | M3-05 | Calibration survives reboot; error within acceptable limits |
| [ ] M3-07 | Implement hub motor hall feedback: speed and position from 15 hall states per electrical revolution, direction, resolution and sign | P1 | M3-03 | Direction and sign match physical rotation on both wheels |
| [ ] M3-08 | Implement acquisition of phase current, pack voltage, DC-link current and board temperature via ADC synchronized to PWM | P1 | M3-03 | Measurements are synchronous; noise and offset within acceptable limits |

### 4.3. FOC and actuator loops

> The donor mainboard carries the power stage. The generated fixed-point FOC core and SVPWM are reused
> through one FFI boundary (`BLDC_controller_step` plus plain structs); everything above it is Rust.

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-09 | Integrate the generated FOC core and SVPWM through the FFI boundary; keep the Rust side of the boundary unit-tested | P1 | M3-02 | One function, plain structs, no heap across it; a wheel spins from Rust |
| [ ] M3-10 | Implement electrical angle calibration and hall phase-order verification for both hub motors | P1 | M3-07, M3-09 | Calibration is reproducible; wrong hall order is detected |
| [ ] M3-11 | Run the 16 kHz current loop with current limiting and integrator saturation | P1 | M3-09, M3-10 | Step response without sustained oscillation; overshoot within tolerance |
| [ ] M3-12 | Implement a 1–2 kHz speed loop with acceleration and speed limiting | P1 | M3-11 | Response under inertial load; stable across load changes |
| [ ] M3-13 | Implement limits on current (10 A per motor bench profile), torque, speed (400 rpm) and temperature with derating | P1 | M3-11, M3-12 | Exceeding a limit leads to limiting, then to a fault |
| [ ] M3-14 | Single-wheel bring-up on one hub motor: rotation direction, hall sequence, encoder, torque sign | P1 | M3-10…M3-13 | All signs and directions match; no jerks or overheating |
| [ ] M3-15 | Verify the current loop: step, overshoot, settling time, heating | P1 | M3-11 | Metrics attached; heating within limits |
| [ ] M3-16 | Verify the speed loop under inertial load | P1 | M3-12 | Metrics attached; stability confirmed |

### 4.4. Balance loop and motion

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-17 | Implement the 200 Hz balance task with a fixed period, without dynamic memory allocation | P1 | M3-05, M3-07, M3-12 | Period is stable; jitter measured; no heap or blocking in the loop |
| [ ] M3-18 | Implement the selected balance controller in firmware; take parameters from configuration | P1 | M3-17 | Behaviour on the bench matches the documented design targets |
| [ ] M3-19 | Implement the state machine `BOOT → SELF_TEST → IDLE → BALANCE → DAMPING → FAULT → E_STOP → SHUTDOWN` | P1 | M3-03 | Transitions are validated; invalid commands do not change state |
| [ ] M3-20 | Implement tilt/roll limiting, fall detection and transition to `DAMPING` | P1 | M3-17, M3-19 | Configurable tilt threshold 25–35° and roll limit; transition before the dangerous angle |
| [ ] M3-21 | Implement torque distribution between the two wheels, and in stage 2 the hip height/roll allocation, with saturation and balance priority | P1 | M3-18 | When saturated, balance is preserved; yaw and roll degrade predictably |
| [ ] M3-22 | Implement processing of speed and yaw commands from CRSF and the USART3 setpoint link: latest-value, validity check, acceleration limiting | P1 | M3-19 | Stale and invalid commands are ignored; acceleration is limited |
| [ ] M3-23 | Implement hip height and roll control outside the 200 Hz balance loop (four-bar, stage 2, P2) | P2 | M3-07, M3-17 | Height adjustable by at least 100 mm; roll reachable; balance loop does not degrade |
| [ ] M3-24 | Implement odometry for the two wheels in stage 1 and for four actuators in stage 2, and publish it in telemetry | P1 | M3-07 | Odometry matches the reference on straights and turns |
| [ ] M3-25 | Start PD balancing on the safety frame with a zero command | P1 | M3-18, M3-19 | Holding for at least 10 min without falling |
| [ ] M3-26 | Tune the balance controller on the bench and compare against the design targets | P1 | M3-25 | Differences are explained; parameters refined |
| [ ] M3-27 | Conduct bench tests: 10 N·s impulse, 10° deviation, 10° slope | P1 | M3-26 | All scenarios without falling; current and temperature within limits |
| [ ] M3-28 | Measure the 200 Hz balance jitter and end-to-end latency; if jitter > 50 µs, record the controller decision | P1 | M3-25 | Measurement protocol with oscilloscope/GPIO; decision documented |
| [ ] M3-29 | Conduct a 30 min thermal run on the bench | P1 | M3-27 | Hub motors and MOSFETs without overheating; derating does not trip falsely |

### 4.5. Communication protocol

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-30 | Implement `jbot_protocol` with a `no_std`-compatible 8-byte header, payload up to 56 bytes and CRC-16 | P1 | M3-02 | Header is fixed; version, type, flags, sequence and timestamp are validated |
| [ ] M3-31 | Implement messages via `postcard` and `serde`: commands, state, IMU, joints, power, faults | P1 | M3-30 | Schema is versioned; incompatible major version is rejected |
| [ ] M3-32 | Implement UART transport: COBS, CRC-16, resynchronization on `0x00`, maximum 256 bytes | P1 | M3-30 | Corrupted and truncated frames are recovered; error counters increase |
| [ ] M3-33 | Implement the two board links: USART2 CRSF at 420 kbaud on the left cable, USART3 framed setpoints/telemetry at 115200 on the right cable. Reserve a CAN path behind a feature flag; CAN is not routed on the donor board and is deferred for stages 1–2 | P1 | M3-30 | Both links exchange on the bench; the CAN path is documented as deferred and validated only on `vcan` |
| [ ] M3-34 | Implement the full message set: `ControlCommand`, `ModeCommand`, `EmergencyStop`, `Heartbeat`, `RobotState`, `ImuSample`, `JointFeedback`, `PowerFeedback`, `FaultReport` | P1 | M3-31 | All messages pass round-trip; IDs and rates correspond to the architecture |
| [ ] M3-35 | Cover the protocol with round-trip, property and fuzz tests: truncation, corrupted CRC, sequence overflow, unknown version | P1 | M3-31, M3-32 | `proptest` is green; invalid frames do not panic |
| [ ] M3-36 | Implement processing of sequence, duplicates and stale commands | P1 | M3-34 | Duplicates are idempotent; old commands are not applied |
| [ ] M3-37 | Ensure exchange with the mission computer equivalent (PC): command 100 Hz, telemetry 200–500 Hz; measure losses and latency | P1 | M3-34 | Losses within budget; metrics available |

### 4.6. Safety, diagnostics and firmware operation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-38 | Implement IWDG, 100 ms command timeout and loss of mission computer heartbeat with transition to `DAMPING` | P1 | M3-19, M3-34 | Loss of communication > 100 ms leads to `DAMPING`; IWDG resets a hung firmware |
| [ ] M3-39 | Implement fault codes, latched faults, manual reset and self-test before reset | P1 | M3-19 | Reset is possible only when the robot is stationary and self-test succeeds |
| [ ] M3-40 | Implement a blackbox ring of the last N seconds with upload over UART (P2) | P2 | M3-38 | Data is preserved on failure; upload is reproducible |
| [ ] M3-41 | Conduct an audit: absence of heap, `panic!` and `unwrap()` in the loop; add `defmt` logging | P1 | M3-17 | Static analysis and review confirm the absence of prohibited constructs |
| [ ] M3-42 | Implement saving calibrations to non-volatile memory and restoration after reboot | P1 | M3-06, M3-10 | Calibrations are not lost; corrupted data is detected |
| [ ] M3-43 | Conduct fault injection on the bench: UART disconnection, brownout, overheating, overload, E-stop | P1 | M3-38, M3-39 | All faults lead to a safe state; events are logged |
| [ ] M3-44 | Implement the firmware update and rollback path: SWD over the debug probe in stages 1–2; USB DFU or a CAN bootloader is deferred | P2 | M3-02 | Update without disassembly; failed update is rolled back |
| [ ] M3-45 | Prepare firmware documentation: memory map, pinout, protocol, version, compatibility matrix, board variant | P1 | M3-34, M3-44 | The document allows building and flashing the board from scratch |
| [ ] M3-46 | Integrate the ExpressLRS 2.4G receiver over CRSF UART and bind it to the LiteRadio 3 Pro; define channel mapping and failsafe | P1 | M3-19, M3-22 | Channels and arming are validated; link loss and invalid frames are handled without bypassing the kill switch |

### 4.7. Stage 2: hips and 3D balance

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M3-47 | Implement the hip actuator link on USART3: height/position setpoints, feedback, CRC, sequence, 300 ms validity window and hold-on-timeout. A CAN hip driver would need an external adapter and is deferred | P1 | M3-33, M2-27 | Two hip actuators follow setpoints; a stale or corrupt frame holds the leg, never releases it |
| [ ] M3-48 | Integrate the two hip actuators with the two hub motors: 3D allocation, height sweep, roll command, tether runs | P1 | M3-21, M3-23, M3-47 | Four actuators stable; height range ≥ 100 mm; no fall in the stage-2 tests |

---

## 5. M4. Sensor drivers and mission computer transport

**Goal:** ensure reliable data reception from the mainboard, lidar and camera, a unified time base and
sensor health. The mission computer is the Raspberry Pi 5 class single-board computer of stage 3.

**Inputs:** `docs/architecture.md` sections 5, 6.3, 6.5, 6.9; `docs/specification.md` sections 6.1,
7.2, 8.1.
**Outputs:** `jbot_hardware`: transport, lidar and camera drivers, health metrics, replay, systemd
deployment.

### 5.1. Mission computer platform

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-01 | Install a 64-bit OS on NVMe; configure cooling, 5 V / 5 A power on its own fused converter, network, SSH/VPN and isolation | P1 | — | Boot from NVMe; no throttling under typical load; network is isolated |
| [ ] M4-02 | Prepare systemd units for `jbot-node`, `zenohd`, drivers and bridge; `Restart=on-failure`, watchdog, resource limits | P1 | M4-01 | Services start after boot; crashes are restarted; logs are available |
| [ ] M4-03 | Configure `aarch64-unknown-linux-gnu` or musl cross-compilation and CI build | P1 | M4-01 | Artifact is built in CI and runs on the mission computer |
| [ ] M4-04 | Make the root FS mostly read-only, with a separate partition for maps and logs (P2) | P2 | M4-01 | Reboot does not lose configuration; logs and maps are written to a separate partition |

### 5.2. Mission computer–mainboard transport

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-05 | Implement the `Transport` trait with `SerialCobs` and `Mock` implementations; reserve `SocketCan` for a future CAN interface | P1 | M3-34 | All implementations pass a common test suite; `Mock` is deterministic |
| [ ] M4-06 | Bring up the primary UART link on USART3 (PB10/PB11) at 115200 8N1 with COBS + CRC-16; verify termination and stub length with an oscilloscope. An optional USB-CAN adapter is reserved and not required | P1 | M4-05 | Stable exchange; corrupted frames are recovered; the CAN adapter is not in the control path |
| [ ] M4-07 | Add a second UART profile at 921600 and measure the achievable rate and error rate; keep 115200 until measured | P1 | M4-05, M3-32 | Rate and error rate recorded; the default is raised only after the measurement |
| [ ] M4-08 | Implement frame reception/transmission with validation of version, sequence, CRC, length and timeouts | P1 | M4-05 | Invalid frames are discarded; error counters are available |
| [ ] M4-09 | Implement metrics: frame loss, CRC errors, command age, latency. Bus-off is not applicable until a CAN interface exists | P1 | M4-08 | Metrics are published in telemetry and available to the operator |
| [ ] M4-10 | Implement transport degradation: on sustained CRC errors or loss, hold the last valid setpoint inside the validity window and then allow `DAMPING`; log the event | P1 | M4-06, M4-07 | Degradation is observable and never leaves an old setpoint applied indefinitely |
| [ ] M4-11 | Implement mission computer–mainboard heartbeat and handshake; loss of heartbeat puts the mainboard into `DAMPING` | P1 | M4-08, M3-38 | Loss of communication is detected within 100 ms; recovery is correct |
| [ ] M4-12 | Implement mainboard–mission computer clock offset and drift estimation via EWMA; optionally GPIO 1 PPS | P1 | M4-08 | IMU/odometry/lidar desynchronization ≤ 10 ms; timestamps are set at sample time |

### 5.3. Lidar driver

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-13 | Select a lidar and write an ADR: 2D 360°, ≥ 10 m, ≥ 10 Hz or 3D with comparable characteristics | P1 | M4-01 | Model selected taking into account power, mounting, driver and availability |
| [ ] M4-14 | Implement the driver in Rust: serial/USB interface, scans or point cloud, asynchronous publication | P1 | M4-13, M4-05 | Data is published at ≥ 10 Hz; errors do not block the runtime |
| [ ] M4-15 | Transform measurements to `laser_link`; perform angular and range calibration | P1 | M4-14 | Calibration verified against reference objects; errors within tolerance |
| [ ] M4-16 | Implement lidar health monitoring: rate, gaps, staleness, temperature, protocol errors | P1 | M4-14 | On degradation, a fault is published; navigation receives a stale flag |
| [ ] M4-17 | Implement recording of raw scans for replay and regression tests | P1 | M4-14 | Recording is reproducible; format is versioned |
| [ ] M4-18 | Implement scan preprocessing: outlier filtering, range limiting, conversion to point cloud/grid | P1 | M4-15 | Preprocessing is deterministic; parameters are configurable |

### 5.4. Camera driver

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-19 | Connect an RGB-D camera of the RealSense D435i or OAK-D class via USB 3.0/CSI-2; implement capture at ≥ 30 fps at 640×480 | P1 | M4-01 | Capture is stable; frame drops are accounted for |
| [ ] M4-20 | Implement camera calibration, storage in configuration and verification at startup | P2 | M4-19 | Incorrect calibration is detected; parameters are versioned |
| [ ] M4-21 | Ensure camera synchronization with IMU and lidar ≤ 10 ms | P2 | M4-12, M4-19 | Timestamps are consistent; desynchronization is measured |
| [ ] M4-22 | Ensure camera failure isolation: loss of camera does not block lidar navigation | P1 | M4-19 | When the camera is disconnected, navigation continues; a fault is published |

### 5.5. Sensor integration and replay

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M4-23 | Implement a sensor health monitor and publication of statuses with timestamps | P1 | M4-16, M4-22 | All sensors have status `OK/DEGRADED/STALE/FAULT` |
| [ ] M4-24 | Implement publication of sensor data in Zenoh with explicit QoS and timestamps | P1 | M4-14, M4-23 | Subscriber receives data; loss policy is explicitly defined |
| [ ] M4-25 | Implement a harness for recording and replaying sensor streams | P1 | M4-17 | Replay reproduces the scenario; metrics are comparable with the reference |
| [ ] M4-26 | Conduct HIL injection: lidar staleness, camera disconnection, time jump, USB disconnection | P1 | M4-24, M4-25 | All faults are detected; safe behavior confirmed |
| [ ] M4-27 | Prepare driver documentation: models, interfaces, calibrations, limitations, troubleshooting | P1 | M4-14, M4-19 | The document is sufficient for sensor replacement and recalibration |

---

## 6. M5. Mission computer onboard software

**Goal:** assemble the `jbot_node` composition root: configuration, transport, state estimation,
supervisor, telemetry, middleware and correct shutdown.

**Inputs:** `docs/architecture.md` sections 6, 7, 8; `docs/specification.md` sections 3, 8, 9, 10.
**Outputs:** `jbot_node`, `jbot_common`, configuration profiles, systemd deployment, HIL protocol.

### 6.1. Composition and infrastructure

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-01 | Extend `jbot_common`: typed state, command, modes, fault codes, configuration | P1 | — | Types are serializable; schema version is fixed |
| [ ] M5-02 | Implement TOML loading with validation, defaults, profiles and hash logging | P1 | M5-01 | Invalid config is rejected; `sim/bench/robot` profiles are supported |
| [ ] M5-03 | Implement Tokio tasks: ingress, estimation, control egress, telemetry, supervisor, shutdown | P1 | M4-05, M5-02 | Tasks start and stop in a coordinated manner; errors are not lost |
| [ ] M5-04 | Implement channel policy: `watch` for commands and state, bounded `mpsc` for frames and logs, `broadcast` for faults | P1 | M5-03 | Queues are bounded; telemetry overflow does not block E-stop |
| [ ] M5-05 | Implement signal handling and correct shutdown via `CancellationToken` | P1 | M5-03 | SIGTERM/SIGINT lead to safe shutdown and log flush |
| [ ] M5-06 | Implement structured `tracing` JSON logging and metrics with node/task/robot ID | P1 | M5-03 | Logs are parsed; there is no high-frequency logging in the loop |
| [ ] M5-07 | Ensure task isolation: navigation, perception and logging do not block estimation and supervisor | P1 | M5-03 | Under load, estimation and supervisor meet deadlines |

### 6.2. Middleware and protocol

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-08 | Implement Zenoh publish/subscribe: state, telemetry, diagnostics, commands | P1 | M5-03 | Topic contracts and QoS are documented; E-stop is not lost |
| [ ] M5-09 | Connect `zenoh-bridge-ros2dds` for RViz and rosbag; verify that the bridge does not block safety | P1 | M5-08 | RViz sees TF and scans; if the bridge crashes, control is unaffected |
| [ ] M5-10 | Implement handshake and a compatibility matrix for firmware and host protocol | P1 | M4-11, M5-02 | Incompatible version is rejected with a clear error |
| [ ] M5-11 | Implement configuration profiles for bench and robot with explicit differences | P1 | M5-02 | Profiles pass validation; differences are documented |

### 6.3. State estimation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-12 | Implement a 100–200 Hz EKF: IMU, wheel odometry, estimation of pose, velocity and gyroscope bias | P1 | M4-12 | EKF converges; estimates are reproducible on replay |
| [ ] M5-13 | Implement publication of `odom → base_link` and prepare `map → odom` for SLAM | P1 | M5-12 | TF tree conforms to REP-105 |
| [ ] M5-14 | Implement compensation for delay, outliers and measurement gaps | P1 | M5-12 | Jumps and gaps do not lead to divergence |
| [ ] M5-15 | Validate EKF on HIL and compare with the reference; record errors | P1 | M5-12, M8-06 | Pose and speed within acceptable limits; report with metrics |
| [ ] M5-16 | Ensure deterministic EKF replay on recorded logs | P1 | M5-12, M4-25 | The same log produces the same estimate |

### 6.4. Supervisor and safety

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-17 | Implement supervisor: voltage, temperature, latency, command age, fault flags, mission computer resources | P1 | M5-03 | When limits are exceeded, a safe stop command is generated |
| [ ] M5-18 | Implement command validation and safety envelope: stale, invalid and untrusted commands are not applied | P1 | M5-17, M3-36 | Negative tests confirm rejection of application |
| [ ] M5-19 | Implement priority of teleoperation over autonomous motion | P1 | M5-18 | Teleoperator takes control immediately |
| [ ] M5-20 | Ensure startup to `IDLE` ≤ 10 s and correct shutdown by command and at low charge | P1 | M5-05, M5-17 | Time is measured; logs and configuration are preserved |
| [ ] M5-21 | Integrate systemd watchdog and restart of `jbot_node` without loss of safety | P1 | M4-02, M5-05 | A hang leads to restart; the mainboard transitions to `DAMPING` |

### 6.5. HIL, telemetry and operation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M5-22 | Conduct fault injection: kill `jbot-node`, UART disconnection, lidar staleness, mission computer reboot | P1 | M5-17, M4-26 | A mission computer failure does not cause a fall; recovery is correct |
| [ ] M5-23 | Conduct a 2–4 hour soak test with metric logging | P1 | M5-22 | No leaks, degradation or task misses |
| [ ] M5-24 | Implement a Rerun/Zenoh telemetry dashboard with key metrics | P2 | M5-08 | Operator sees state, battery, latencies and faults |
| [ ] M5-25 | Implement orchestration of blackbox and log upload from the mainboard | P2 | M3-40, M5-08 | Logs are available without disassembling the robot |
| [ ] M5-26 | Conduct Gate G3 review and prepare the HIL protocol | P1 | M5-20…M5-23 | All G3 criteria are confirmed |

---

## 7. M6. Navigation and SLAM

**Goal:** build a map, localize in it and ensure autonomous waypoint motion subject to safety
constraints.

**Inputs:** `docs/specification.md` section 6; `docs/architecture.md` sections 3.3, 6.7; lidar and EKF
data from M4–M5.
**Outputs:** `jbot_navigation`: SLAM, localization, costmap, planners, behavior layer, set of maps and
test protocols.

### 7.1. SLAM and maps

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

### 7.2. Localization

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-13 | Implement localization in the map: scan matching + EKF, estimation 100–200 Hz, lidar correction 10–30 Hz | P1 | M6-04, M5-12 | Pose is published with latency ≤ 100 ms |
| [ ] M6-14 | Implement the TF tree `map → odom → base_link` per REP-105 | P1 | M6-13, M5-13 | TF is consistent; RViz shows no breaks |
| [ ] M6-15 | Implement kidnapping detection and relocalization ≤ 10 s (P2) | P2 | M6-13 | Kidnapping scenario is detected; robot stops until recovery |
| [ ] M6-16 | Implement localization degradation: speed limiting, then stop | P1 | M6-13 | On staleness or low data quality, motion is prohibited |
| [ ] M6-17 | Validate accuracy ≤ 50 mm and 2° (RMS) relative to reference markers | P1 | M6-13, M8-07 | Measurement protocol and plots attached |

### 7.3. Costmap and obstacles

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-18 | Implement the static map, obstacle map and inflation taking into account dimensions | P1 | M6-08 | Inflation corresponds to footprint; narrow passages are not falsely blocked |
| [ ] M6-19 | Implement a dynamic obstacle layer from lidar with data lifetime | P1 | M6-18, M4-14 | Dynamics are updated; stale obstacles are removed |
| [ ] M6-20 | Implement staleness policy: data older than 300 ms leads to a stop | P1 | M6-19, M4-16 | Data delay test confirms the stop |
| [ ] M6-21 | Implement safe distance: < 0.25 m at low speed leads to a stop | P1 | M6-19 | Obstacle test confirms the stop |
| [ ] M6-22 | Implement speed limiting near obstacles ≤ 0.3 m/s | P1 | M6-21 | Speed is reduced to the threshold; limiting is visible in telemetry |

### 7.4. Planning and motion control

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-23 | Implement a global planner A*/Dijkstra taking into account traversability and dimensions | P1 | M6-18 | Path is built; unreachable goal returns an error |
| [ ] M6-24 | Implement a local planner/controller DWA, MPC or equivalent taking balance constraints into account | P1 | M6-23 | Robot follows the path without losing balance; constraints are respected |
| [ ] M6-25 | Account for constraints on acceleration, jerk, tilt, torque and slip | P1 | M6-24, M3-21 | Constraints are not violated during typical maneuvers |
| [ ] M6-26 | Implement path following with speed limiting and smooth braking | P1 | M6-24 | Stop from 1.0 m/s while maintaining balance; path ≤ 0.5 m |
| [ ] M6-27 | Implement recovery behaviors: costmap clearing, replanning, rotation, backing off | P2 | M6-23, M6-24 | Behaviors are triggered by condition and do not create dangerous maneuvers |
| [ ] M6-28 | Implement priority of teleoperation over autonomous motion | P1 | M5-19, M6-24 | Teleoperator takes control without delay |
| [ ] M6-29 | Implement the behavior layer: waypoints, patrol, pause, return, emergency stop | P1 | M6-26 | Missions are executed; emergency stop has the highest priority |
| [ ] M6-30 | Isolate navigation in a separate process or task with resource limits | P1 | M5-03, M6-24 | Navigation does not affect balance jitter and does not block the supervisor |
| [ ] M6-31 | Implement a mission interface for returning to the dock and patrolling | P2 | M6-29 | Mission is started and canceled by command; state is published |

### 7.5. Navigation tests

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M6-32 | Run navigation scenarios on the HIL bench with sensor noise | P1 | M6-24 | Scenarios are reproducible; metrics collected |
| [ ] M6-33 | Build a map of a room ≥ 100 m² with loop closure; measure drift ≤ 1% of path | P1 | M6-06, M6-11 | Protocol and map attached |
| [ ] M6-34 | Perform 100 autonomous runs to the goal; success rate ≥ 95%, collisions 0 | P1 | M6-29, M6-33 | Protocol for each run; failures analyzed |
| [ ] M6-35 | Verify dynamic obstacles: stop or replanning ≤ 1 s | P1 | M6-19, M6-24 | Scenario with a person/cart passed |
| [ ] M6-36 | Verify lidar failure, data staleness and robot kidnapping | P1 | M6-16, M6-20 | All scenarios lead to safe behavior |
| [ ] M6-37 | Verify operation without ROS2 and with the ROS2/RViz bridge | P1 | M5-09, M6-30 | Base navigation does not depend on ROS2; bridge does not block safety |
| [ ] M6-38 | Run regression on recorded logs and compare metrics with the reference | P1 | M6-11, M8-15 | Regression is reproducible; deviations are explained |
| [ ] M6-39 | Prepare the navigation report and Gate G4 protocol | P1 | M6-33…M6-38 | All G4 criteria are confirmed |

---

## 8. M7. Safety and fault tolerance

**Goal:** ensure Linux-independent protection, predictable behavior under faults and traceability of
safety requirements.

**Inputs:** `docs/specification.md` section 9; `docs/architecture.md` section 7.
**Outputs:** hazard analysis, FMEA, safety case, fault catalog, proof-test procedures, checklists.

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M7-01 | Conduct hazard analysis and FMEA from the very beginning; update at each gate | P1 | — | Hazard and mitigation register covers mechanics, power, software and operation |
| [ ] M7-02 | Specify a two-channel NC E-stop: disconnecting the power stage, preserving logic, latched state; the battery kill switch is the path that survives a hung firmware | P1 | — | Specification verified on the bench; complies with SAF-02, SAF-03 |
| [ ] M7-03 | Specify and verify the watchdog chain: kill switch → IWDG → 100 ms timeout → mission computer heartbeat → systemd → supervisor | P1 | M3-38, M5-21 | Each link is verified separately and in combination |
| [ ] M7-04 | Record the limits matrix: current, temperature, voltage, tilt, speed, command age | P1 | — | Limits are consistent in firmware, supervisor and documentation |
| [ ] M7-05 | Implement a fault code catalog with severity, latched flag, context and timestamp | P1 | M3-39, M5-17 | Codes are stable; published over UART and in Zenoh/ROS2 |
| [ ] M7-06 | Conduct a safety review before each field test and gate | P1 | M7-01 | Comments are closed; admission decision recorded |
| [ ] M7-07 | Prepare the E-stop and protection proof-test procedure at least once every 12 months | P2 | M7-02, M7-03 | Procedure is reproducible; results are archived |
| [ ] M7-08 | Ensure network security: SSH/VPN, no direct access to control ports, network isolation | P1 | M4-01 | Verified by scanning; access policy documented |
| [ ] M7-09 | Implement authentication of Zenoh/ROS2 endpoints where supported | P2 | M5-08 | Unauthorized access is rejected |
| [ ] M7-10 | Ensure rejection of invalid, stale and untrusted commands | P1 | M5-18, M3-36 | Negative tests confirm no change of the setpoint |
| [ ] M7-11 | Ensure brownout transition to a safe state with motor output disabled | P1 | M3-38 | Brownout test confirms the safe state |
| [ ] M7-12 | Publish faults over UART and in Zenoh/ROS2 with code, severity and time | P1 | M7-05 | Messages reach the operator; losses are not hidden |
| [ ] M7-13 | Implement software stop as a separate path that does not replace the hardware E-stop | P1 | M7-02 | Command stops motion; E-stop remains independent |
| [ ] M7-14 | Maintain the safety case and traceability matrix of SAF requirements | P1 | M7-01 | Each SAF requirement has a task, test and protocol |
| [ ] M7-15 | Prepare safe operation instructions, first power-up and field test checklists | P1 | M2-21, M3-43 | Instructions are available to the operator; checklists are mandatory |
| [ ] M7-16 | Maintain an incident, near-miss and emergency stop log | P2 | M7-12 | Log is preserved; events are analyzed at review |

---

## 9. M8. Integration, HIL and testing

**Goal:** assemble the complete system, verify it on HIL and confirm requirements with test protocols.

**Inputs:** M2–M6 and the BOM.
**Outputs:** HIL bench, test suites, protocols, acceptance report, delivery kit.

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M8-01 | Record the testing strategy by levels: unit, property, integration, HIL, system, regression | P1 | — | Level and tool matrix approved |
| [ ] M8-02 | Ensure unit and property tests in all crates; include in CI | P1 | M10-01 | Coverage of critical modules; tests are stable |
| [ ] M8-03 | Implement UART loopback and `vcan` integration tests for the reserved CAN path | P1 | M4-05, M3-32 | Tests are reproducible in CI |
| [ ] M8-04 | Implement tests for timeouts, degradation and heartbeat loss | P1 | M4-10, M4-11 | All scenarios pass; safe state confirmed |
| [ ] M8-05 | Implement regression traces from bench and HIL logs with golden metrics | P1 | M8-03 | Deviations from the reference are detected |
| [ ] M8-06 | Assemble the HIL bench: mainboard, hub motors, hip actuators, mission computer, and lidar | P1 | M3-29, M4-26 | Bench is reproducible; measurements are available |
| [ ] M8-07 | Implement latency and jitter measurement tools, measurement protocol | P1 | M3-28, M5-23 | Mission computer–mainboard–actuator latency ≤ 10 ms; jitter ≤ 50 µs |
| [ ] M8-08 | Implement a fault injection suite: power, communication, sensors, mission computer, firmware | P1 | M7-03, M5-22 | Each fault has an expected safe response |
| [ ] M8-09 | Verify startup to safe `IDLE` ≤ 10 s | P1 | M5-20 | Time measured on the assembled system |
| [ ] M8-10 | Conduct a 2–4 hour soak test and a 24-hour run with battery replacement | P1 | M8-06 | No critical failures; logs and metrics preserved |
| [ ] M8-11 | Conduct system thermal, EMI and vibration checks | P1 | M8-06 | All parameters within acceptable limits |
| [ ] M8-12 | Prepare acceptance test programs and methods per section 12 of the SRS | P1 | M8-01 | Documents approved before acceptance begins |
| [ ] M8-13 | Conduct the 100 autonomous runs protocol | P1 | M6-34, M8-06 | ≥ 95 successful, 0 collisions; protocol signed |
| [ ] M8-14 | Conduct runtime tests on the donor 10s2p pack by profiles | P1 | M8-06 | ≥ 30 min typical, ≥ 20 min aggressive; profile attached |
| [ ] M8-15 | Conduct regression on recorded logs and compare metrics | P1 | M4-25, M6-38 | Results are reproducible; deviations are explained |
| [ ] M8-16 | Prepare the final acceptance report and close Gate G5 | P1 | M8-09…M8-15 | All P1 confirmed; P2 implemented or justified |
| [ ] M8-17 | Prepare a spare kit of fuses, connectors and consumables | P1 | M10-14 | Kit is included in the delivery |
| [ ] M8-18 | Prepare the operator manual, maintenance manual and protocol description | P1 | M10-21 | Documents verified for completeness and currency |
| [ ] M8-19 | Prepare the release package: tags, changelog, artifacts, build instructions | P1 | M10-22 | Package builds reproducibly from a clean environment |
| [ ] M8-20 | Conduct gate reviews G1–G5 with recording of decisions and open issues | P1 | M2-27, M5-26, M6-39, M8-16 | Review protocols are preserved |
| [ ] M8-21 | Maintain the traceability matrix "requirement → task → test → protocol" | P1 | M10-18 | Matrix is complete; gaps identified before acceptance |
| [ ] M8-22 | Update the risk register before each gate and on architecture changes | P1 | M10-17 | Risks have an owner, probability, impact and mitigations |

---

## 10. M9. Extensions: camera, AI, manipulator, docking station

**Goal:** extend the platform without compromising balance, safety and base navigation.

**Inputs:** M5–M8, free mission computer resources and power/thermal margin.
**Outputs:** extension interfaces, AI pipeline, manipulator emulator/integration, docking station and
auto-charging mission.

### 10.1. Extension interfaces

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-01 | Implement the mechanical and electrical camera interface: USB 3.0/CSI-2, 5 V with protection and current margin | P1 | M4-01 | Camera is powered and transmits data; mount is adjustable |
| [ ] M9-02 | Implement the manipulator interface: 12 V / 5 A, protection, separate branch, a reserved UART link (optional CAN through a USB-CAN adapter) | P3 | M4-06 | Interface is reserved and verified on the emulator |
| [ ] M9-03 | Implement a PCIe/USB reserve for the Hailo-8L AI accelerator or analog, taking into account power and heat | P2 | M4-01 | Accelerator is detected; power and cooling within budget |
| [ ] M9-04 | Implement an external isolated UART diagnostic port, with optional CAN | P2 | M4-06 | Port is isolated; does not create ground loops |
| [ ] M9-05 | Verify that extensions do not increase balance loop jitter by more than 50 µs | P1 | M9-01…M9-04, M3-28 | Before/after measurements; impact within tolerance |

### 10.2. Camera and AI

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-06 | Ensure capture at ≥ 30 fps at 640×480 and stable frame delivery | P2 | M9-01, M4-19 | Drops are accounted for; latency within budget |
| [ ] M9-07 | Implement storage and verification of camera calibration at startup | P2 | M4-20 | Incorrect calibration blocks AI, but not navigation |
| [ ] M9-08 | Ensure camera synchronization with IMU and lidar ≤ 10 ms | P2 | M4-21, M9-06 | Timestamps are consistent; verified on a moving object |
| [ ] M9-09 | Select the AI runtime: `ort`, `tract` or `candle`; select the accelerator or CPU | P2 | M9-03 | ADR justified by latency, memory and compatibility |
| [ ] M9-10 | Implement object detection at ≥ 10 Hz on the mission computer or the AI accelerator | P2 | M9-09 | Latency and accuracy metrics attached |
| [ ] M9-11 | Publish detections with timestamps without blocking navigation and control | P2 | M9-10, M5-08 | Publication is asynchronous; drops do not block other tasks |
| [ ] M9-12 | Ensure degradation: camera failure does not block lidar navigation | P1 | M4-22, M9-10 | Camera disconnect test; navigation continues |
| [ ] M9-13 | Verify the power and thermal budget of the AI accelerator under sustained load | P2 | M9-03, M9-10 | No brownout or throttling; temperatures within limits |

### 10.3. Manipulator

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-14 | Implement an emulator of the manipulator node over the reserved UART link to verify power, timing and E-stop | P3 | M9-02, M4-06 | Emulator responds to commands; E-stop verified |
| [ ] M9-15 | Implement the manipulator control interface in the behavior layer, outside the balance loop | P1 | M9-14, M5-03 | Commands do not affect the balance rate; latency within acceptable limits |
| [ ] M9-16 | Implement supervisor constraints with an extended load: speed, torque, tilt | P3 | M9-15, M5-17 | Constraints trigger before loss of balance |
| [ ] M9-17 | Implement manipulator emergency stop via the common E-stop and a software command | P3 | M9-14, M7-02 | Stop is independent of Linux; state is latched |
| [ ] M9-18 | Verify that manipulator failure or disconnection does not disrupt balance and navigation | P1 | M9-15 | Hot/cold failure test; base functionality preserved |

### 10.4. Docking station and auto-charging

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-19 | Design guides, power contacts, docking sensor and an IR/UART channel | P3 | M9-02 | Design ensures docking accuracy and contact protection |
| [ ] M9-20 | Ensure contacts are de-energized until docking is confirmed, with reverse polarity and short circuit protection | P3 | M9-19 | Bench test confirms safety |
| [ ] M9-21 | Implement the state protocol `IDLE`, `APPROACH`, `DOCKED`, `CHARGING`, `FAULT` | P3 | M9-19 | Transitions are validated; errors do not leave contacts energized |
| [ ] M9-22 | Implement the mission for returning to the dock and charging under donor pack BMS control | P3 | M6-31, M9-21 | Mission is executed; interruption is safe |
| [ ] M9-23 | Ensure docking accuracy ≤ ±20 mm and ±2° in heading | P3 | M9-19, M6-13 | Series of attempts; metrics attached |
| [ ] M9-24 | Limit docking station power ≤ 150 W and charge through the donor pack's original charge port under BMS control | P3 | M9-20 | Current and power within limits; BMS controls charging |
| [ ] M9-25 | Implement charge monitoring, SoC/SoH and event logging | P3 | M9-22 | Charging events available in telemetry |
| [ ] M9-26 | Verify that the docking station is not required for base operation | P1 | M9-22 | Disconnecting the dock does not affect base functions |

### 10.5. System validation of extensions

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M9-27 | Conduct a 24-hour run with battery replacement and planned pauses | P2 | M8-10, M9-13 | No critical failures; configuration is preserved |
| [ ] M9-28 | Verify EMI after installing the camera, AI accelerator and manipulator | P1 | M9-05 | UART, IMU and lidar operate without degradation |
| [ ] M9-29 | Verify the thermal conditions of all components in typical and aggressive cycles | P1 | M9-13 | Temperatures within limits; no throttling |
| [ ] M9-30 | Confirm runtime ≥ 30 min typical and ≥ 20 min aggressive with extensions | P1 | M9-29 | Protocol with profile and temperatures |
| [ ] M9-31 | Update schematics, BOM, configurations, calibrations and documentation for extensions | P1 | M9-01…M9-30 | Documentation set is current |

---

## 11. M10. Cross-cutting work

**Goal:** ensure quality, reproducibility, manageability and completeness of the project at all
stages.

### 11.1. CI, quality and reproducibility

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-01 | Configure CI: `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test` on every PR | P1 | — | CI blocks merge on violation |
| [ ] M10-02 | Include protocol property tests: round-trip, corrupted frames, sequence wraparound | P1 | M3-35 | Tests run in CI |
| [ ] M10-03 | Include UART loopback and `vcan` integration tests | P1 | M8-03 | Tests are stable in CI |
| [ ] M10-04 | Configure aarch64 cross-build and firmware build in CI | P1 | M4-03, M3-02 | Artifacts are published |
| [ ] M10-05 | Include `cargo deny` and `cargo audit` | P1 | M10-01 | Vulnerabilities and license conflicts block merge |
| [ ] M10-06 | Configure a nightly HIL/regression run | P1 | M8-06 | Results are available; regressions are detected |
| [ ] M10-07 | Maintain ADRs for key decisions: controller, drivers, SLAM, middleware, AI runtime, deferred CAN | P1 | — | Each decision has an ADR with a status |

### 11.2. Configuration, logs and compatibility

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-08 | Version the protocol and maintain a firmware/host compatibility matrix | P1 | M3-45, M5-10 | Incompatible versions are detected before startup |
| [ ] M10-09 | Ensure calibrations are saved to non-volatile memory and restored after reboot | P1 | M3-42, M4-20 | Calibrations are not lost; corruption is detected |
| [ ] M10-10 | Standardize structured logs and metrics: fields, levels, rates, storage | P1 | M5-06 | Logs are suitable for automated analysis |
| [ ] M10-11 | Implement a tool for mainboard blackbox upload and analysis | P2 | M3-40 | Data is read and correlated with mission computer logs |
| [ ] M10-12 | Implement storage of maps and logs on a separate partition with rotation and overflow protection | P2 | M4-04 | Disk does not overflow; old data is deleted per policy |

### 11.3. Procurement and manufacturing

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-13 | Maintain [`docs/bom.md`](bom.md): order the added parts, keep the donor parts off the order list, and treat the hip actuator as the only long-lead item | P1 | M2-27 | Hip actuator torque/thermal margin is verified before ordering; donor parts are reused; lead times tracked |
| [ ] M10-14 | Provide a spare kit of fuses, connectors and consumables | P1 | M10-13 | Kit is included in the delivery and stored in inventory |
| [ ] M10-15 | Verify component availability and alternatives (second source) | P1 | M10-13 | Critical items have a replacement or justification |
| [ ] M10-16 | Manufacture/order PCBs and cable assemblies with margin | P1 | M10-13 | Spare units available for bench and robot |

### 11.4. Project management and documentation

| ID | Task | Priority | Depends on | Acceptance criterion |
|---|---|---|---|---|
| [ ] M10-17 | Maintain the risk register and update it at each gate | P1 | — | Each risk has an owner, probability, impact and mitigations |
| [ ] M10-18 | Maintain the traceability matrix "SRS → task → test → protocol" | P1 | M8-21 | Matrix is complete and current |
| [ ] M10-19 | Hold a weekly progress review and a monthly risk review | P1 | — | Meeting protocols and decisions are preserved |
| [ ] M10-20 | Record deviations from the SRS only through approval and an addendum to the SRS | P1 | M10-18 | Deviations are not lost; changes are traceable |
| [ ] M10-21 | Maintain the documentation set: operator, maintenance, protocol, BOM, schematics, calibrations | P1 | M8-18 | Documentation matches the released version |
| [ ] M10-22 | Maintain releases, tags and changelog; record firmware, host and protocol versions | P1 | M10-08 | Any delivery is reproducible by tag |

---

## 12. Requirements traceability matrix

| Requirement group | Key tasks |
|---|---|
| SYS-01…SYS-04 (mainboard, balance rate, actuators, IMU) | M2-01, M2-05, M2-22, M3-01, M3-05, M3-11, M3-12, M3-17, M3-28, M8-07 |
| SYS-05…SYS-08 (upper level, communication, latency, mission computer failure) | M4-05…M4-12, M5-03, M5-12, M5-21, M8-07 |
| AUT-01…AUT-07 (donor pack, connection, battery replacement) | M2-04, M2-08, M7-04, BOM |
| AUT-08…AUT-13 (energy budget, DC-DC, margin) | BOM |
| AUT-14…AUT-18 (runtime, profiles) | M8-10, M8-14, M9-30 |
| AUT-19…AUT-30 (battery monitoring, thresholds, E-stop) | M2-18, M2-19, M3-08, M7-11, BOM |
| MOB-01…MOB-05 (geometry, actuators, modes) | M3-13, M3-19, BOM |
| MOB-06…MOB-11 (balance, impulse, tilt, fall) | M2-23…M2-25, M3-20, M3-25…M3-27 |
| MOB-12…MOB-16 (speed, reverse, braking) | M2-24, M3-21, M3-22, M6-26 |
| MOB-17…MOB-25, MOB-31 (terrain, height, steps, 3D balance) | M3-21, M3-23, M3-48, M6-25, M8-11 |
| MOB-26…MOB-30 (load, mass, thermal conditions) | M3-13, M3-29, M8-11, M9-29 |
| NAV-01…NAV-05 (lidar, IMU, odometry, synchronization) | M4-12…M4-25, M5-12 |
| NAV-06…NAV-13 (SLAM, maps, versioning) | M6-01…M6-12 |
| NAV-14…NAV-19 (localization, TF, degradation) | M5-12, M6-13…M6-17 |
| NAV-20…NAV-26 (costmap, planners, isolation) | M6-18…M6-31 |
| NAV-27…NAV-35 (100 runs, dynamics, safety, logs) | M6-32…M6-39, M8-13, M8-15 |
| EXT-01…EXT-06 (modularity, protocol, configuration) | M5-01, M5-02, M10-07, M10-08 |
| EXT-07…EXT-13 (camera, synchronization, degradation) | M4-19…M4-22, M9-01, M9-06…M9-12 |
| EXT-14…EXT-21 (manipulator) | M9-02, M9-14…M9-18 |
| EXT-22…EXT-28 (docking station) | M9-19…M9-26 |
| EXT-29…EXT-33 (AI accelerator, reserved CAN, NVMe, jitter) | M9-03…M9-05, M9-13, M4-04 |
| INT-01…INT-10 (UART, frame format, power) | M2-11, M3-30…M3-37, M4-05…M4-12, BOM |
| INT-11…INT-22 (Rust, no_std, queues, middleware, CI) | M3-02, M3-41, M5-03…M5-11, M10-01…M10-12 |
| SAF-01…SAF-13 (E-stop, watchdog, latched, network) | M2-20, M3-38…M3-43, M5-17…M5-21, M7-01…M7-16, BOM |
| REL-01…REL-08 (24 h, conditions, service, calibrations) | M3-42, M4-04, M8-10, M9-27, M10-09, M10-12 |

---

## 13. Milestones and execution order

| Milestone | Content | Tasks | Gate |
|---|---|---|---|
| S0 | Documentation, workspace, and BOM baseline | M10-01, M10-07, BOM | B1 |
| S1 | Donor wheels and balance on the safety rig | M2-01…M2-27, M3-01…M3-29 | G1 |
| S2 | Hips, legs, and 3D balance | M3-47, M3-48, M3-21, M3-23, M3-29 | G2 |
| S3 | Mission computer, transport, telemetry, lidar, EKF | M4-01…M4-27, M5-01…M5-26 | G3 |
| S4 | SLAM, localization, navigation, 100 runs | M6-01…M6-39 | G4 |
| S5 | Camera/AI, manipulator, docking station | M9-01…M9-31 | G5 |
| S6 | Acceptance tests and delivery | M8-01…M8-22, M10-13…M10-22 | — |

**Parallelism rule:** `M2` is the critical path and starts immediately; `M7` and `M10` run in parallel
with the main path; BOM completion runs ahead of procurement; `M9` tasks do not start until G4 is
closed, except for interface reservation and procurement.

---

## 14. Next two weeks

| ID | Task | Result by the end of week 2 |
|---|---|---|
| M2-01, M2-02 | Remove the sideboards, inspect the donor board and harnesses | Inspection record with continuity measurements |
| M2-05, M2-06 | Build and flash the EFeru baseline | Board boots from the pack; debug UART prints |
| M2-10, M2-11 | Bind the XR1 and wire CRSF to the left cable | Channel map and failsafe recorded |
| M2-15 | Mount and wire the BNO055 | IMU answers on I2C2 at 0x28 or 0x29 |
| M3-02 | Create the Rust workspace | Clock tree and blink on the donor board |
| M7-01, M7-02 | Hazard analysis and E-stop specification | Draft safety document |
| M10-13 | Order the added parts after the BOM freeze | Placed orders; the hip actuator stays deferred |

---

## 15. Risks affecting the task breakdown

| Risk | Affected tasks | Planned response |
|---|---|---|
| BOM is incomplete or an interface is wrong | BOM, M3-01, M10-13 | Freeze interfaces, obtain datasheets, keep spare and substitution options |
| The donor board cannot hold 200 Hz balance with 16 kHz FOC, CRSF and IMU | M3-01, M3-17, M3-28 | Measure early; drop to 100 Hz balance or record a controller change in an ADR |
| The donor board is damaged or the pack is at end of life | M2-02, M2-03, M8-14 | Inspect first; a second donor is the fallback, not a new design |
| The 2p pack cannot supply two motors at 10 A plus regen | M2-19, M3-13, M8-14 | Keep the bench profile, derate early, log sag and regen current |
| Hip actuator selection and thermal margin are insufficient | M3-29, M3-47, M10-13 | Derive the torque in stage 1; bench thermal test before ordering; substitute a larger hip or reduce mass |
| The UART link at 115200 cannot carry the telemetry rate | M4-06, M4-10 | Measure early; use the 921600 profile; reduce telemetry before adding hardware |
| Optional CAN adapter on USB is unstable | M4-06, M4-10 | Keep CAN out of the control path; UART remains primary |
| Lidar produces noise or too little data | M4-13…M4-18, M6-04 | Proven model, calibration, filtering, backup driver |
| Rust SLAM is not mature enough | M6-01…M6-12 | Backup `slam_toolbox` via the bridge, early ADR |
| Balance dynamics complicate the local planner | M6-24…M6-26 | Acceleration and tilt constraints, conservative speed, HIL validation |
| Mission computer throttling during SLAM and AI | M5-07, M6-30, M9-13 | Cooling, limits, separate processes, NVMe |
| AI accelerator power and heat | M9-03, M9-13 | Separate protection, power budget, tests under load |
| Manipulator disrupts balance | M9-15…M9-18 | Supervisor, constraints, moving out of the balance loop |
| Firmware and host version drift | M10-08, M5-10 | Compatibility matrix, CI check, version in every frame |

---

## 16. Open questions and decision tasks

| Question | Decision task | Deadline |
|---|---|---|
| Hip actuator: model, torque, integrated UART driver; CAN deferred | M3-47, M10-13, BOM | Before ordering the stage-2 actuator |
| Does the mission computer appear in stage 2 or stage 3 | M4-06, BOM | Before the right cable is allocated |
| Leg topology: 1-DOF four-bar decided; link lengths, spring, height range, E-position | M3-23, BOM | Before detailed mechanical design |
| Donor pack condition, replacement plan, and the confirmed 2p current limit | M2-04, BOM | Before stage 1 field runs |
| Lidar and camera: models, mounting, power, heat | M4-13, M4-19, ADR | Before purchasing sensors |
| SLAM: Rust-native or `slam_toolbox` via bridge | M6-01, ADR-008 | Before stage 4 |
| AI: Hailo-8L or CPU-only; runtime `ort`/`tract`/`candle` | M9-09, ADR | Before stage 5 |
| Firmware update path: SWD probe now; USB DFU or CAN bootloader later | M3-44, M10-08 | Before production |
| Docking station: contacts and charging protocol through the donor charge port | M9-19, M9-21 | Before stage 5 |
| Mass, load and height range after the prototype | BOM, M3-23 | After prototype assembly |
| Enclosure, connectors and maintainability | M10-16, BOM | Before manufacturing the enclosure |
| Future CAN: mainboard revision or external adapter on the mission computer | M3-33, M10-07 | Before any CAN device is specified |

---

## 17. Summary criteria for project completion

1. All P1 requirements are confirmed by test protocols, and the traceability matrix is filled in.
2. P2 requirements are implemented or have an approved deferral justification.
3. Balance loop jitter ≤ 50 µs at the 200 Hz balance rate; loss of a valid command > 100 ms leads to
   `DAMPING`.
4. Autonomous navigation: ≥ 95 successful runs out of 100 without collisions.
5. Runtime on the donor 10s2p pack in a typical cycle ≥ 30 min, confirmed by measurement.
6. E-stop and protections have passed proof-test; behavior under faults complies with the SRS.
7. The documentation set, source code, configurations, calibrations and maps are delivered in full.
8. All critical failures are closed; no open safety defects.

---

## 18. References

- `docs/specification.md` — technical specification and requirements.
- `docs/roadmap.md` — stages, gates and risks.
- `docs/architecture.md` — architecture, protocols, safety concept.
- `docs/system_requirements.md` — system requirements.
- `docs/hardware.md` — donor hardware, I/O map, radio, IMU.
- `firmware/README.md` — EFeru baseline and the Rust firmware plan.
- `docs/agent_workflow.md` — agent roles and task execution order.
