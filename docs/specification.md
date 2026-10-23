# System Requirements Specification for the Creation of a Robotic Complex
## JBOT-WheeledBiped (wheeled-biped balancing autonomous robot)

| Parameter | Value |
|---|---|
| Product code | JBOT-WheeledBiped |
| Document code | SRS-001 |
| Revision | 1.0 |
| Status | Draft SRS, mandatory for implementation after approval |
| Date | 2026-10-21 |
| Basis for development | `docs/architecture.md` v0.2, `docs/system_requirements.md`, `docs/roadmap.md` |
| Customer | Determined by contract |
| Contractor | Determined by contract |
| Scope of application | Autonomous mobile robotics, dynamics and navigation research |

---

## 1. General information

### 1.1. Document name and purpose

This System Requirements Specification (SRS) defines the requirements for the development, manufacture, software, testing, and acceptance of the **JBOT-WheeledBiped** robotic complex — an autonomous wheeled-biped balancing robot.

The SRS is a mandatory document for all project stages. Deviations from the requirements of this SRS are permitted only upon agreement with the Customer and are formalized as an addendum to the SRS.

### 1.2. Terms, definitions, and abbreviations

| Term | Definition |
|---|---|
| Robot | The JBOT-WheeledBiped product in full configuration |
| Battery | Milwaukee M18 rechargeable battery |
| BMS | Battery Management System, the built-in battery management system |
| FOC | Field-Oriented Control, vector control of a brushless motor |
| QDD | Quasi-Direct Drive |
| IMU | Inertial Measurement Unit |
| EKF | Extended Kalman Filter |
| SLAM | Simultaneous Localization and Mapping |
| Nav2 equivalent | Autonomous navigation stack in Rust, functionally equivalent to ROS2 Nav2 |
| SoC / SoH | State of Charge / State of Health of the battery |
| E-stop | Hardware emergency stop |
| MCU | Real-time microcontroller |
| RPi | Upper-level compute unit Raspberry Pi 5 |
| P1 | Mandatory requirement |
| P2 | Target requirement; phased implementation is permitted |
| P3 | Forward-looking requirement; reserved by the architecture |

### 1.3. Requirement priorities

- **P1** — requirement mandatory for acceptance of the baseline version.
- **P2** — target requirement; must be implemented by the completion of the corresponding roadmap stage or justified in documentation.
- **P3** — requirement to reserve interfaces and architectural capabilities; actual implementation is in subsequent versions.

---

## 2. Purpose and development goals

### 2.1. Purpose of the Robot

The Robot is intended for:

- autonomous movement indoors and on prepared outdoor sites;
- research and demonstration of dynamically stable locomotion using a wheeled-biped configuration;
- map building and navigation based on lidar data without external positioning infrastructure;
- waypoint mission execution, patrolling, and teleoperation;
- expansion with sensor and manipulator equipment without redesigning the control loop.

### 2.2. Development goals

1. Ensure safe autonomous movement powered by a production Milwaukee M18 battery.
2. Implement balancing and actuator control on the MCU with hard real-time.
3. Implement the SLAM, localization, and navigation stack in Rust on the RPi with a ROS2 bridge.
4. Ensure modular extensibility: camera, manipulator, automatic charging dock.
5. Ensure traceability of requirements, tests, and architectural decisions.

### 2.3. Project boundaries

**Included in the baseline version scope:**

- two RobStride RS05 wheel drives and two RS02 hips driving 1-DOF four-bar legs for maintaining height and posture;
- balancing on a plane, forward/backward movement, and turning;
- lidar SLAM, map building and loading, waypoint navigation;
- monitoring of the battery, temperatures, communication, and faults;
- hardware emergency stop;
- expansion interfaces for a camera, manipulator, and dock.

**Not included in the baseline version:**

- climbing stair flights;
- movement on water, mud, sand, and loose soil;
- fully autonomous charging without a dock;
- manipulation operations (interface and compatibility requirement);
- certification for operation in hazardous environments.

---

## 3. General system requirements

### 3.1. Robot composition

1. Mechanical platform: body/chassis (the base/ground link), two legs, each a planar 1-DOF four-bar with one wheel; four actuators total (2 RobStride RS02 hips + 2 RS05 wheels).
2. Power subsystem: M18 18 V baseline (24–36 V preferred), protection, contactor, DC-DC 12 V / 5 V / 3.3 V, E-stop.
3. Computing subsystem:
   - real-time MCU candidates: Pico 2 W (RP2350), ESP32-S3, STM32H723/H743; ADR-001 selects after benchmark;
   - Raspberry Pi 5 16 GB;
   - provisional RobStride RS02 hips and RS05 wheels with integrated FOC; substitution allowed only with an equivalent CAN/torque/thermal envelope;
   - CAN FD backbone where required, classic CAN 1 Mbit/s actuator bus via TJA1051T/3, backup UART; Pico 2 W needs an external CAN controller.
4. Sensor subsystem:
   - IMU BNO055 (100 Hz fusion); ICM-42688-P retained as a high-rate fallback;
   - 2D or 3D lidar;
   - optional RGB-D camera.
5. Software:
   - MCU firmware (`no_std`, Rust);
   - onboard `jbot-node` program in Rust;
   - crates `jbot_common`, `jbot_hardware`, `jbot_estimation`, `jbot_control`, `jbot_navigation`;
   - Zenoh / ROS2 bridge.
6. Set of operational documentation and test software.

### 3.2. Main performance indicators

| Indicator | Value | Priority |
|---|---|---|
| Dry mass without battery | Not more than 15 kg | P2 |
| Payload without manipulator | Not less than 2 kg | P1 |
| Maximum speed | Not less than 1.5 m/s | P2 |
| Operating speed in autonomous mode | Up to 1.0 m/s | P1 |
| Operating time from M18 XC5.0, typical cycle | Not less than 30 min | P1 |
| Operating time, aggressive cycle | Not less than 20 min | P1 |
| Operating time, predominantly idle | Not less than 45 min | P2 |
| Boot time to safe `IDLE` | Not more than 10 s | P1 |
| Operating temperature range | 0…+40 °C | P1 |
| Enclosure ingress protection rating | IP20, dry indoor spaces | P1 |

### 3.3. Computing architecture requirements

| ID | Requirement | Priority |
|---|---|---|
| SYS-01 | Balance and FOC control must be performed on the MCU, not on Linux | P1 |
| SYS-02 | Balance loop: 1 kHz, jitter not more than 50 µs | P1 |
| SYS-03 | FOC current loop: 20–40 kHz | P1 |
| SYS-04 | IMU sampling: 1–4 kHz | P1 |
| SYS-05 | High-level control and state estimation: 100–200 Hz | P1 |
| SYS-06 | Loss of RPi–MCU communication for more than 100 ms must lead to `DAMPING` | P1 |
| SYS-07 | End-to-end latency of the RPi–MCU–actuator command: not more than 10 ms | P1 |
| SYS-08 | The RPi is not a safety authority; an RPi failure must not cause a fall | P1 |

---

## 4. Autonomy requirements

### 4.1. Power source and compatibility

| ID | Requirement | Priority |
|---|---|---|
| AUT-01 | Baseline battery — Milwaukee M18 XC5.0, 5S Li-ion, 5.0 A·h, ~90 W·h, 15.0–21.0 V | P1 |
| AUT-02 | The Robot must operate with standard M18 platform batteries without modifying their housing | P1 |
| AUT-03 | Battery connection must be protected against reverse polarity and have a key/adapter preventing incorrect installation | P1 |
| AUT-04 | Use of M18 XC8.0 / XC12.0 is permitted with a corresponding recalculation of operating time | P2 |
| AUT-05 | Battery replacement is performed only with the Robot powered down; hot swapping is not required | P1 |
| AUT-06 | In the baseline version, the battery is charged with an external standard Milwaukee charger; an integrated charger is not required | P1 |
| AUT-07 | Power and signal contacts of the dock must be reserved architecturally and in software | P3 |

### 4.2. Energy budget

| ID | Requirement | Priority |
|---|---|---|
| AUT-08 | Typical consumption: not more than 200 W | P1 |
| AUT-09 | Peak consumption: not more than 600 W for less than 10 s | P1 |
| AUT-10 | Continuous input current: not less than 25 A; main branch fuse — 40 A at 18 V (30 A at 24–36 V); peak up to 55 A short-term | P1 |
| AUT-11 | DC-DC 12 V / 5 A, 5 V / 5–6 A, 3.3 V / 1 A; efficiency not less than 90 % at rated load | P1 |
| AUT-12 | The power circuit must remain operational during sags down to 15 V and actuator transients | P1 |
| AUT-13 | A power margin of not less than 20 % must be provided for the camera, AI accelerator, and manipulator | P2 |

### 4.3. Operating time

| ID | Requirement | Priority |
|---|---|---|
| AUT-14 | Typical mixed cycle: not less than 30 min at 20 ± 5 °C, battery SoH ≥ 90 %, 2 kg load, 50 % of the time at 0.5 m/s, 30 % idle, 20 % maneuvering | P1 |
| AUT-15 | Aggressive cycle with accelerations up to 1.0 m/s and frequent turns: not less than 20 min | P1 |
| AUT-16 | Predominantly idle mode with sensors enabled: not less than 45 min | P2 |
| AUT-17 | Operating time must be measured until the 15.0 V protection trips, not until the battery is fully discharged | P1 |
| AUT-18 | Results must be accompanied by the load profile, temperature, and initial SoC | P1 |

### 4.4. Battery monitoring and low-charge behavior

| ID | Requirement | Priority |
|---|---|---|
| AUT-19 | Measurement of battery voltage and current via a bidirectional shunt at 100–500 Hz | P1 |
| AUT-20 | Voltage measurement error — not more than ±0.5 %; current — not more than ±2 % of full scale | P1 |
| AUT-21 | SoC calculation by coulomb counting with voltage correction; SoC error — not more than ±5 % | P1 |
| AUT-22 | SoH and accumulated energy estimation; logging of charge/discharge events | P2 |
| AUT-23 | Warning threshold: 16.0 V — acceleration limiting and operator notification | P1 |
| AUT-24 | Limiting threshold: 15.5 V — torque and speed limiting | P1 |
| AUT-25 | Emergency stop threshold: 15.0 V confirmed for 500 ms — actuator shutdown, transition to `FAULT`, controlled shutdown | P1 |
| AUT-26 | Threshold decisions must be made by the MCU independently of the RPi | P1 |
| AUT-27 | The Robot must reduce load before the battery BMS protection trips; spontaneous BMS shutdown must not be a normal scenario | P1 |
| AUT-28 | Power telemetry: voltage, current, SoC, SoH, temperature — over CAN at not less than 50 Hz | P1 |
| AUT-29 | Threshold events must be logged with a timestamp, current, and voltage | P1 |
| AUT-30 | During E-stop, the actuator power circuits are disconnected, but 5 V / 12 V logic power is retained for logging and correct RPi shutdown | P1 |

### 4.5. Autonomy tests

1. Discharge on the test bench with a profile simulating the typical cycle; recording of voltage, current, temperature, and SoC.
2. Verification of the 16.0 / 15.5 / 15.0 V thresholds using a voltage simulator and load.
3. Verification of SoC accuracy by comparison with a reference wattmeter.
4. Verification of behavior when the BMS is disconnected and when voltage sags under peak load.
5. Verification that logging is preserved and the RPi shuts down correctly during E-stop.

---

## 5. Mobility and dynamics requirements

### 5.1. Kinematics and actuators

| ID | Requirement | Priority |
|---|---|---|
| MOB-01 | Wheeled-biped configuration: two legs, one wheel per leg; each leg is a planar 1-DOF four-bar (base A–B, upper crank A–C driven by a RobStride RS02 hip, lower rocker B–D passive, wheel fork C–D–E). Height and fore-aft offset are coupled along the coupler curve. | P1 |
| MOB-02 | Actuators — four total: two provisional RobStride RS02 hips with integrated FOC/encoder/driver and two RS05 wheels (integrated or external FOC); feedback on position, speed, current, and temperature | P1 |
| MOB-03 | Balance control — PD / LQR / VMC, configurable; supports 3D balance: pitch via wheel torque, yaw via differential wheel torque, height via common hip angle, roll via differential hip height | P1 |
| MOB-04 | Modes: `IDLE`, `BALANCE`, `DAMPING`, `FAULT`, `E_STOP`, `SHUTDOWN` | P1 |
| MOB-05 | Transitions between modes — only via a validated command and precondition checks | P1 |

### 5.2. Balancing and stability

| ID | Requirement | Priority |
|---|---|---|
| MOB-06 | Maintaining balance on a horizontal hard surface with zero command for not less than 10 min without falling | P1 |
| MOB-07 | Recovery after an external horizontal impulse of up to 10 N·s applied to the body, without falling | P1 |
| MOB-08 | Recovery after a short-term deviation of up to 10° on a plane | P1 |
| MOB-09 | Configurable tilt limit 25–35°; if exceeded — torque limiting and transition to `DAMPING` | P1 |
| MOB-10 | Fall detection and transition to a safe state with actuator shutdown | P2 |
| MOB-11 | Self-righting after a fall is not required | — |
| MOB-31 | 3D balance allocation: pitch and yaw via wheel torque, height via common hip angle, roll via differential hip height | P1 |
| MOB-32 | If a passive leg spring/damper is fitted, its stiffness and travel are identified and included in the balance model | P2 |

### 5.3. Speed, acceleration, and maneuverability

| ID | Requirement | Priority |
|---|---|---|
| MOB-12 | Maximum speed — not less than 1.5 m/s; autonomous operating — up to 1.0 m/s | P1 (operating) / P2 (maximum) |
| MOB-13 | Reverse — up to 0.5 m/s | P1 |
| MOB-14 | Turning radius in place — practically zero due to independent wheel control | P1 |
| MOB-15 | Acceleration and jerk limiting must prevent slipping and tipping | P1 |
| MOB-16 | Emergency braking from 1.0 m/s — while maintaining balance; braking distance not more than 0.5 m on a dry surface | P1 |

**Voltage note.** MOB-12 is marginal on the 18 V M18 baseline: a 48 V-rated hub motor loses roughly 60% of its speed at 18 V, giving about 1.5 m/s at full charge and less under load. A 24–36 V pack is preferred; otherwise derate the maximum-speed requirement.

### 5.4. Obstacle and terrain negotiation

| ID | Requirement | Priority |
|---|---|---|
| MOB-17 | Irregularities and thresholds up to 20 mm high — at a speed of up to 0.5 m/s | P1 |
| MOB-18 | Step/threshold up to 50 mm — at a speed of not more than 0.2 m/s using the hip four-bar to lift the wheel; validate in simulation | P1 |
| MOB-19 | Target capability to negotiate steps up to 100 mm in walking mode | P2 |
| MOB-20 | Longitudinal slope up to 10° and lateral slope up to 5° on a dry hard surface while maintaining balance | P1 |
| MOB-21 | Holding on a 10° slope for 10 min without slipping or overheating | P1 |
| MOB-22 | Seams, tile joints, and grooves up to 20 mm wide | P1 |
| MOB-23 | Surface transitions (carpet–tile, elevator threshold) up to 15 mm high | P1 |
| MOB-24 | Stair flights and steps above 100 mm are not required | — |
| MOB-25 | Body height adjustment — not less than 100 mm; the range is refined during detailed design | P2 |

**Design note (four-bar leg).** With one active hip DOF per leg, body height and wheel fore-aft offset are coupled along the coupler curve. Step negotiation (MOB-18/MOB-19) therefore requires a deliberate weight shift and roll; validate the 50 mm P1 target in simulation before committing the mechanical design. The first prototype uses one wheel per leg; dual wheels are deferred.

### 5.5. Load, mass, and thermal conditions

| ID | Requirement | Priority |
|---|---|---|
| MOB-26 | Balancing and movement with a 2 kg payload | P1 |
| MOB-27 | Operation with a load of up to 4 kg with speed and acceleration limiting | P2 |
| MOB-28 | Dry mass without battery — not more than 15 kg | P2 |
| MOB-29 | Thermal protection of actuators: warning, limiting, shutdown | P1 |
| MOB-30 | The temperature of the body and motors must not exceed safe limits during prolonged movement | P1 |

### 5.6. Mobility tests

1. Balancing test bench: 10 min hold, impulse impact, 10° deviation.
2. Test track: 20 mm, 50 mm, 10° slope, 5° lateral slope, 20 mm seam.
3. Acceleration/braking/reverse with slip and current monitoring.
4. Movement with a 2 kg and 4 kg load.
5. 30 min thermal run with recording of actuator and body temperatures.
6. Verification of transition to `DAMPING` when the tilt limit is exceeded, communication is lost, and the charge is low.

---

## 6. Navigation requirements

### 6.1. Sensor equipment

| ID | Requirement | Priority |
|---|---|---|
| NAV-01 | Lidar: 2D with a 360° angle, range not less than 10 m and frequency not less than 10 Hz, or a 3D lidar with comparable characteristics | P1 |
| NAV-02 | Minimum detectable distance — not more than 0.1 m | P1 |
| NAV-03 | IMU and wheel odometry must be used in the state estimation loop | P1 |
| NAV-04 | RGB-D camera — optional for baseline navigation, mandatory for visual localization/detection scenarios | P2 |
| NAV-05 | All sensors must share a common time base; desynchronization — not more than 10 ms | P1 |

### 6.2. SLAM and map building

| ID | Requirement | Priority |
|---|---|---|
| NAV-06 | Lidar-based SLAM at 10–30 Hz | P1 |
| NAV-07 | Map resolution — not more than 5 cm; mapping area — not less than 1000 m² | P1 |
| NAV-08 | Accumulated drift — not more than 1 % of the traveled path without loop closure | P1 |
| NAV-09 | Loop closure and relocalization when returning to a known area | P1 |
| NAV-10 | Saving and loading the map in a ROS2-compatible format (PGM/YAML) or a documented equivalent | P1 |
| NAV-11 | Loading time for a 1000 m² map — not more than 10 s | P2 |
| NAV-12 | Map versioning and binding to the robot configuration | P1 |
| NAV-13 | Automatic map updates only upon an authorized operator/mission command | P2 |

### 6.3. Localization and state estimation

| ID | Requirement | Priority |
|---|---|---|
| NAV-14 | EKF at 100–200 Hz, fusing IMU, wheel odometry, and lidar/visual pose | P1 |
| NAV-15 | Localization accuracy in the built map: not more than 50 mm and 2° (RMS) | P1 |
| NAV-16 | Pose publication latency — not more than 100 ms | P1 |
| NAV-17 | Detection of robot "kidnapping" and relocalization within not more than 10 s | P2 |
| NAV-18 | Upon localization degradation — speed reduction, then stop; movement based on stale data is prohibited | P1 |
| NAV-19 | Coordinate frames `map`, `odom`, `base_link` — in accordance with REP-105 | P1 |

### 6.4. Navigation stack (Nav2 equivalent in Rust)

| ID | Requirement | Priority |
|---|---|---|
| NAV-20 | `jbot_navigation` must implement: static map, obstacle map, inflation, global and local planners | P1 |
| NAV-21 | Global planner: A* / Dijkstra taking into account traversability and dimensions | P1 |
| NAV-22 | Local planner/controller: DWA, MPC, or equivalent taking into account the dynamics of the wheeled-biped configuration | P1 |
| NAV-23 | Behavioral scenarios must be supported: go to point, patrolling, pause, return, emergency stop | P1 |
| NAV-24 | Recovery behaviors must be implemented: costmap clearing, replanning, rotation in place, backing off | P2 |
| NAV-25 | The navigation core must operate without ROS2 installed; the `zenoh-bridge-ros2dds` bridge — only for interoperability with Nav2/RViz/rosbag | P1 |
| NAV-26 | Navigation must run in a separate process or task with resource limits, without blocking the safety loop | P1 |
| NAV-27 | Autonomous goal reaching: not less than 95 successful attempts out of 100 in a mapped environment without collisions | P1 |
| NAV-28 | Dynamic obstacle detection: stop or replan within not more than 1 s | P1 |
| NAV-29 | Safe distance to an obstacle — configurable; at a distance of less than 0.25 m at low speed — stop | P1 |
| NAV-30 | Maximum autonomous speed — 1.0 m/s; near obstacles — not more than 0.3 m/s | P1 |
| NAV-31 | Staleness of lidar/localization data for more than 300 ms — stop | P1 |
| NAV-32 | Support for teleoperation with priority over autonomous movement | P1 |
| NAV-33 | Support for waypoint missions, patrolling, and return to dock | P2 |
| NAV-34 | Recording and playback of sensor/command logs for regression tests | P1 |
| NAV-35 | Operation without GPS; the baseline environment — indoor spaces and flat prepared outdoor sites | P1 |

### 6.5. Navigation tests

1. Building a map of a room of not less than 100 m² with loop closure; drift verification.
2. 100 autonomous runs to the goal in a static environment; success rate not less than 95 %, collisions — 0.
3. Runs with dynamic obstacles (people/carts) with stopping and replanning.
4. Verification of localization accuracy relative to reference markers.
5. Degradation verification: lidar disconnection, stale data, robot "kidnapping".
6. Verification of operation without ROS2 and with the ROS2/RViz bridge.

---

## 7. Extensibility requirements

### 7.1. Architectural principles

| ID | Requirement | Priority |
|---|---|---|
| EXT-01 | Modular Cargo structure: `jbot_common`, `jbot_hardware`, `jbot_estimation`, `jbot_control`, `jbot_navigation`, `jbot_node` | P1 |
| EXT-02 | Single dependency direction; cyclic dependencies are prohibited | P1 |
| EXT-03 | New sensors and actuators are connected via `jbot_common` types/traits and ports, without modifying the balance loop | P1 |
| EXT-04 | The communication protocol is versioned; incompatible major versions are rejected | P1 |
| EXT-05 | Configuration — TOML with typing, validation, and hash logging | P1 |
| EXT-06 | Updating MCU firmware and onboard software without disassembling the Robot; availability of a rollback path | P2 |

### 7.2. Camera connection

| ID | Requirement | Priority |
|---|---|---|
| EXT-07 | Interfaces: USB 3.0 and/or CSI-2; 5 V power with protection and current margin | P1 |
| EXT-08 | Mechanical mounting with alignment and cable protection | P1 |
| EXT-09 | Synchronization of camera timestamps with IMU/lidar — not more than 10 ms | P2 |
| EXT-10 | Support for an RGB-D camera of the RealSense D435i / OAK-D class | P1 |
| EXT-11 | Capture at not less than 30 frames/s at 640×480; object detection — not less than 10 Hz on RPi 5 or an AI accelerator | P2 |
| EXT-12 | Camera calibration is stored in the configuration and verified at startup | P2 |
| EXT-13 | A camera failure must not block baseline lidar navigation | P1 |

### 7.3. Manipulator connection

| ID | Requirement | Priority |
|---|---|---|
| EXT-14 | Mechanical interface: mounting points, fasteners, and space for a manipulator with 4–6 degrees of freedom | P3 |
| EXT-15 | Electrical interface: 12 V / 5 A (60 W), protection, separate branch if necessary | P3 |
| EXT-16 | Communication: CAN FD, 29-bit identifiers, a separate ID range for manipulator nodes | P3 |
| EXT-17 | Manipulator control is integrated into `jbot_control` / the behavioral layer, but not into the 1 kHz balance loop | P1 |
| EXT-18 | The manipulator must not disturb balance; the supervisor limits speed/torque when the load is extended | P3 |
| EXT-19 | Target manipulator payload — not less than 1 kg at the gripper | P3 |
| EXT-20 | Manipulator emergency stop — via the common E-stop and a software command | P3 |
| EXT-21 | Manipulator installation is performed with the power off; hot plugging is not required | P3 |

### 7.4. Automatic charging dock

| ID | Requirement | Priority |
|---|---|---|
| EXT-22 | Architecturally reserved: power contacts, guides, docking sensor, CAN/IR channel | P3 |
| EXT-23 | Automatic docking accuracy: not more than ±20 mm and ±2° in heading | P3 |
| EXT-24 | Dock contacts must be de-energized until docking is confirmed; reverse polarity and short-circuit protection | P3 |
| EXT-25 | Charging — under battery BMS control; dock power — not more than 150 W | P3 |
| EXT-26 | Docking and state protocol: `IDLE`, `APPROACH`, `DOCKED`, `CHARGING`, `FAULT` | P3 |
| EXT-27 | Return to dock and charging must be implemented as a mission in the behavioral layer | P3 |
| EXT-28 | The dock must not be mandatory for baseline operation of the Robot | P1 |

### 7.5. Computing and sensor expansion

| ID | Requirement | Priority |
|---|---|---|
| EXT-29 | PCIe 2.0 x1 / USB 3.0 reservation for an AI accelerator (Hailo-8L or equivalent) | P2 |
| EXT-30 | Consideration of the power and thermal budget of the AI accelerator; separate overcurrent protection | P2 |
| EXT-31 | External diagnostic CAN port — isolated | P2 |
| EXT-32 | Ability to add an NVMe drive for maps, logs, and models | P2 |
| EXT-33 | All expansions must not increase the balance loop jitter by more than 50 µs | P1 |

### 7.6. Extensibility tests

1. Connection of a reference RGB-D camera: capture, calibration, synchronization, detection.
2. Connection of a manipulator emulator as a CAN node; verification of IDs, power, emergency stop.
3. Verification of the dock software interface on a simulator.
4. Verification that failure/disconnection of any expansion does not disrupt balance and baseline navigation.
5. Verification that jitter does not increase with expansions enabled.

---

## 8. Interface and software requirements

### 8.1. Buses and protocols

| ID | Requirement | Priority |
|---|---|---|
| INT-01 | Main bus — CAN FD ISO 11898-2: 1 Mbit/s arbitration, 5 Mbit/s data (2 Mbit/s fallback) | P1 |
| INT-02 | Frame up to 64 bytes: 8-byte header, up to 56 bytes of payload | P1 |
| INT-03 | 120 Ω termination at both ends, stub length less than 0.3 m | P1 |
| INT-04 | Backup UART: 921600/1 Mbit, 8N1, COBS, CRC-16 | P1 |
| INT-05 | All frames: protocol version, type, flags, 16-bit sequence, timestamp, CRC-16 | P1 |
| INT-06 | Control commands — latest-value; telemetry — loss-tolerant | P1 |
| INT-07 | E-stop and heartbeat have the highest priority and are not displaced by telemetry | P1 |
| INT-08 | Message schema — `postcard`/`serde`, little-endian, fixed header | P1 |
| INT-09 | External operator CAN port — isolated | P2 |
| INT-10 | Power: VBAT 15–21 V (M18 baseline; 24–36 V preferred), 12 V / 5 A, 5 V / 5–6 A, 3.3 V / 1 A | P1 |

> **Actuator bus note:** the RobStride RS02/RS05 actuators use classic CAN 1 Mbit/s through a TJA1051T/3 transceiver, not CAN FD. The MCU bridges the actuator bus to the CAN FD backbone; INT-01 applies to the RPi–MCU backbone.

### 8.2. Software requirements

| ID | Requirement | Priority |
|---|---|---|
| INT-11 | Primary language — Rust edition 2021+; MCU firmware — `no_std` | P1 |
| INT-12 | Dynamic memory allocation, `panic!`, and `unwrap()` are prohibited in the control loop | P1 |
| INT-13 | Queues are bounded; telemetry overflow must not block E-stop | P1 |
| INT-14 | `jbot_node` — composition root; binds transport, estimation, control, navigation, and supervisor | P1 |
| INT-15 | Middleware: Zenoh for internal exchange; ROS2 — via the bridge | P1 |
| INT-16 | DDS/Zenoh are not used in the 1 kHz balance loop | P1 |
| INT-17 | QoS: commands — reliable/keep-last; telemetry — best-effort; maps/TF — transient-local; E-stop — reliable, highest priority | P1 |
| INT-18 | Logging — structured (`tracing`), with node/task/robot ID; no high-frequency logging in the control loop | P1 |
| INT-19 | Metrics: frame loss, CRC errors, bus-off, command age, jitter, voltage, temperatures, faults | P1 |
| INT-20 | MCU blackbox: ring buffer of the last N seconds with upload capability | P2 |
| INT-21 | CI: `cargo fmt --check`, `cargo clippy -D warnings`, `cargo test`, aarch64 cross-build, firmware build, `cargo deny`, `cargo audit` | P1 |
| INT-22 | Deployment on RPi via systemd with `Restart=on-failure`, watchdog, and resource limits | P1 |

---

## 9. Safety requirements

| ID | Requirement | Priority |
|---|---|---|
| SAF-01 | Safety has the highest priority over functionality and performance | P1 |
| SAF-02 | E-stop — a hardware function independent of Linux and the MCU: dual-channel NC loop | P1 |
| SAF-03 | E-stop disconnects the actuator power circuits but preserves logic power for logging and shutdown | P1 |
| SAF-04 | Watchdog chain: hardware E-stop → MCU IWDG → 100 ms command watchdog → RPi heartbeat → systemd watchdog → supervisor | P1 |
| SAF-05 | Upon loss of a valid command for more than 100 ms, the MCU transitions to `DAMPING` | P1 |
| SAF-06 | On a critical fault, the actuators are shut down and the fault is latched until a manual reset | P1 |
| SAF-07 | Reset of a critical fault is possible only when the Robot is stationary and the self-test passes | P1 |
| SAF-08 | All faults are published over CAN and to Zenoh/ROS2 with a code, severity, and timestamp | P1 |
| SAF-09 | Invalid, stale, and invalidated commands do not change the actuator setpoint | P1 |
| SAF-10 | Brownout puts the MCU into a safe state; drivers are disabled by default | P1 |
| SAF-11 | External access: SSH/VPN, without directly exposing control ports to the internet | P1 |
| SAF-12 | Authentication of Zenoh/ROS2 endpoints where supported; isolation of the robot network | P2 |
| SAF-13 | Periodic proof-test of E-stop and protection circuits — at least once every 12 months | P2 |

---

## 10. Reliability, operation, and maintenance requirements

| ID | Requirement | Priority |
|---|---|---|
| REL-01 | Continuous operation for 24 h with battery replacement and scheduled pauses without loss of configuration | P2 |
| REL-02 | Operating conditions: 0…+40 °C, humidity 5–85 % non-condensing | P1 |
| REL-03 | Storage: −10…+50 °C, in a dry room | P1 |
| REL-04 | Battery replacement — not more than 2 min without tools | P1 |
| REL-05 | Access to the MCU, RPi, connectors, and fuses without fully disassembling the body | P1 |
| REL-06 | Calibrations are stored in non-volatile memory and restored after reboot | P1 |
| REL-07 | Maintenance of fault and event logs with preservation during power loss | P1 |
| REL-08 | Maps and logs are placed on a separate writable partition; the root FS is predominantly read-only | P2 |

---

## 11. Development stages and delivery scope

### 11.1. Stages

| Stage | Content | Result |
|---|---|---|
| 1 | Architecture, requirements, Cargo workspace structure | This SRS, `architecture.md`, crate skeletons |
| 2 | MCU firmware, actuator loops, IMU, CAN, protocol | Test bench with actuators, protocol, communication and jitter report |
| 3 | State estimation and balancing | Balancing test bench, stability report |
| 4 | SLAM, localization, navigation, ROS2 bridge | Autonomous runs, maps, navigation report |
| 5 | Extensions: camera, manipulator, dock | Interfaces, extension tests, final report |

### 11.2. Delivery scope

1. Robot in assembled form with battery and charger.
2. Firmware and onboard software source code, build instructions.
3. Schematics, BOM, assembly drawings.
4. Configurations, calibrations, and map data.
5. Operator manual, maintenance manual, protocol description.
6. Test programs and methods, test reports.
7. Spare set of fuses and consumables.

---

## 12. Test and acceptance procedure

### 12.1. Test levels

| Level | Content |
|---|---|
| Unit | Mathematics, protocol, CRC, finite state machines, constraints |
| Property | Frame round-trip, sequence overflow, malformed frames |
| Integration | CAN/UART loopback, transport failure, timeouts |
| Simulation | Plant model, sensor noise, balancing, navigation |
| HIL | Real MCUs, drivers, RPi, fault injection |
| System | E-stop, battery disconnection, overheating, communication loss, 24-hour run |
| Regression | Log playback, metric comparison |

### 12.2. Acceptance criteria

1. All P1 requirements are confirmed by test reports.
2. P2 requirements are implemented or have an agreed justification for deferral.
3. There are no critical faults and no open safety defects.
4. Balance loop jitter does not exceed 50 µs, loss of command leads to `DAMPING` within 100 ms.
5. Autonomous navigation: not less than 95 successful runs out of 100 without collisions.
6. Operating time from M18 XC5.0 in a typical cycle — not less than 30 min.
7. The documentation and source code package is delivered in full.

### 12.3. Test conditions

- Temperature 20 ± 5 °C unless otherwise specified.
- Dry hard surface, level area.
- Battery with SoH not less than 90 %.
- Load of 2 kg unless otherwise specified.
- Measuring instruments: reference wattmeter, oscilloscope, logic analyzer, stopwatch, tape measure, reference markers.

---

## 13. Requirements traceability

| SRS section | Source in the architecture |
|---|---|
| 3. Computing architecture | `architecture.md`, sec. 3, 5.7 |
| 4. Autonomy | `architecture.md`, sec. 4.1–4.7 |
| 5. Mobility and dynamics | `architecture.md`, sec. 1.1, 3.2, 7.3 |
| 6. Navigation | `architecture.md`, sec. 3.3, 6.4, 6.7 |
| 7. Extensibility | `architecture.md`, sec. 1, 5.5, 6.1, 6.4 |
| 8. Interfaces and software | `architecture.md`, sec. 5, 6 |
| 9. Safety | `architecture.md`, sec. 7 |
| 10. Reliability | `architecture.md`, sec. 4.5, 6.8, 8 |

---

## 14. Open questions

1. Final MCU selection: STM32H723/H743, Pico 2 W (RP2350), or ESP32-S3 — based on bench results for FOC, CAN, ADC, jitter, and latency.
2. Actuator interface: provisional RobStride RS02 hips and RS05 wheels; substitution requires an equivalent CAN/torque/thermal envelope.
3. Leg topology: decided — 1-DOF four-bar (base A–B, upper crank A–C, lower rocker B–D, wheel fork C–D–E, optional spring). Remaining: link lengths, spring rate/travel, height range, and E-position.
4. Selection of lidar and camera, their mounting, and impact on the mass/power budget.
5. Nav2 as is via the bridge or a fully Rust `jbot_navigation` stack.
6. AI accelerator: Hailo-8L or CPU-only; runtime `ort` / `tract` / `candle`.
7. Firmware update path: CAN bootloader / USB DFU / debug probe.
8. Requirements for the enclosure, connectors, and maintainability.
9. Refinement of mass, payload, and height adjustment range after prototype manufacturing.
10. Dock standard and M18 charging protocol parameters.
11. Power pack: M18 18 V baseline vs 24–36 V preferred; 18 V limits top speed and fuse margin.

---

## 15. Approval

| Role | Full name | Signature | Date |
|---|---|---|---|
| Customer | | | |
| Contractor | | | |
| Project manager | | | |
| Safety officer | | | |
