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
| Battery | Donor hoverboard pack: 10s2p Li-ion, 36 V nominal, 42.0 V full, ~30 V cutoff, 4.4 A·h, ~158 W·h, integrated BMS |
| BMS | Battery Management System, the built-in battery management system |
| FOC | Field-Oriented Control, vector control of a brushless motor |
| QDD | Quasi-Direct Drive |
| IMU | Inertial Measurement Unit |
| EKF | Extended Kalman Filter |
| SLAM | Simultaneous Localization and Mapping |
| Nav2 equivalent | Autonomous navigation stack in Rust, functionally equivalent to ROS2 Nav2 |
| SoC / SoH | State of Charge / State of Health of the battery |
| E-stop | Hardware emergency stop |
| MCU | Real-time microcontroller — donor hoverboard mainboard, STM32F103RCT6 (Cortex-M3, 72 MHz, 256 KB flash, 48 KB RAM) |
| RPi | Upper-level compute unit, Raspberry Pi 5 class; also referred to as the mission computer |
| Stage 1 | Platform with two hub wheels only, no hip actuators |
| Stage 2 | Platform with two hub wheels plus two hip actuators, four actuators total |
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

1. Ensure safe autonomous movement powered by the donor 10s2p Li-ion pack (36 V nominal, ~158 W·h).
2. Implement balancing and actuator control on the mainboard MCU with hard real-time.
3. Implement the SLAM, localization, and navigation stack in Rust on the mission computer with a ROS2 bridge.
4. Ensure modular extensibility: camera, manipulator, automatic charging dock.
5. Ensure traceability of requirements, tests, and architectural decisions.

### 2.3. Project boundaries

**Included in the baseline version scope:**

- stage 1: two donor hub wheels only, balancing and driving on a plane;
- stage 2: two hip actuators driving 1-DOF four-bar legs for maintaining height and posture;
- balancing on a plane, forward/backward movement, and turning;
- lidar SLAM, map building and loading, waypoint navigation;
- monitoring of the battery, temperatures, communication, and faults;
- main fuse, latching kill switch, and software protection chain;
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

1. Mechanical platform: a donor hoverboard base converted into a balancing robot. Stage 1 uses two direct-drive hub wheels only; stage 2 adds two legs, each a planar 1-DOF four-bar with one wheel, giving four actuators total (2 hub wheels + 2 hips).
2. Power subsystem: donor 10s2p pack (36 V nominal, 42.0 V full, ~30 V cutoff, 4.4 A·h, ~158 W·h) with integrated BMS and original charge port, 30–40 A main fuse, latching kill switch in the battery line, firmware-held power latch on PA5, charge-detect input PA12, DC-DC 12 V / 5 V / 3.3 V.
3. Computing subsystem:
   - real-time MCU: donor STM32F103RCT6 mainboard, fixed for stages 1–2; a later controller is an open option only if the platform outgrows it;
   - mission computer (stage 3): Raspberry Pi 5 class, linked over UART, setpoints only;
   - wheel drives: two donor hub motors (~350 W each, 6.5 in wheel) driven by the 12 MOSFETs in two 3-phase bridges, with FOC/SVPWM implemented in the mainboard firmware;
   - hip actuators (stage 2): model TBD, with external drivers, because the mainboard cannot drive them;
   - radio link: BetaFPV LiteRadio 3 Pro transmitter (ExpressLRS 2.4 GHz, 8 channels) to a RadioMaster XR1 receiver (CRSF, 420 kbaud) on USART2;
   - links: USART3 or I2C2 on the shared PB10/PB11 pins to the mission computer or hip driver; the F103 CAN controller exists but its pins are not routed on the donor board, so CAN is not used in stages 1–2.
4. Sensor subsystem:
   - IMU BNO055 on I2C2, 100 Hz fusion, gyro propagated to the 200 Hz balance rate; moves to I2C1 remapped (PB8/PB9, two soldered wires) at stage 2, when the hip driver takes USART3 and I2C2 is no longer free;
   - 2D or 3D lidar;
   - optional RGB-D camera.
5. Software:
   - MCU firmware (`no_std`, Rust);
   - onboard `jbot-node` program in Rust;
   - crates `jbot_common`, `jbot_hardware`, `jbot_navigation`, `jbot_node`;
   - Zenoh / ROS2 bridge.
6. Set of operational documentation and test software.

### 3.2. Main performance indicators

| Indicator | Value | Priority |
|---|---|---|
| Dry mass without battery | Not more than 15 kg | P2 |
| Payload without manipulator | Not less than 2 kg | P1 |
| Maximum speed | Not less than 1.5 m/s | P2 |
| Operating speed in autonomous mode | Up to 1.0 m/s | P1 |
| Operating time from the donor pack (158 W·h), typical cycle | Not less than 30 min | P1 |
| Operating time, aggressive cycle | Not less than 20 min | P1 |
| Operating time, predominantly idle | Not less than 45 min | P2 |
| Boot time to safe `IDLE` | Not more than 10 s | P1 |
| Operating temperature range | 0…+40 °C | P1 |
| Enclosure ingress protection rating | IP20, dry indoor spaces | P1 |

### 3.3. Computing architecture requirements

| ID | Requirement | Priority |
|---|---|---|
| SYS-01 | Balance and FOC control must be performed on the mainboard MCU (STM32F103RCT6), not on Linux | P1 |
| SYS-02 | Balance loop: 200 Hz, jitter not more than 50 µs | P1 |
| SYS-03 | FOC current loop: 16 kHz (SVPWM, two 3-phase bridges) | P1 |
| SYS-04 | IMU: BNO055 fusion at 100 Hz, gyro propagated to the 200 Hz balance rate | P1 |
| SYS-05 | High-level control and state estimation: 100–200 Hz; radio setpoints up to 250 Hz; mission computer setpoints 50–100 Hz | P1 |
| SYS-06 | Loss of mission computer–mainboard communication, or of the radio link, for more than 100 ms must lead to zero setpoints and `DAMPING`; torque is never removed while balancing | P1 |
| SYS-07 | End-to-end latency of the mission computer–mainboard–actuator command: not more than 10 ms | P1 |
| SYS-08 | The mission computer is not a safety authority and cannot arm the Robot; a mission computer failure must not cause a fall | P1 |

---

## 4. Autonomy requirements

### 4.1. Power source and compatibility

| ID | Requirement | Priority |
|---|---|---|
| AUT-01 | Baseline battery — donor hoverboard 10s2p Li-ion, 4.4 A·h, ~158 W·h, 36 V nominal, 42.0 V full, ~30 V cutoff | P1 |
| AUT-02 | The Robot must use the donor pack and its original charge port without modifying the pack housing | P1 |
| AUT-03 | Battery connection must be protected against reverse polarity and have a keyed connector preventing incorrect installation | P1 |
| AUT-04 | A pack of the same voltage class (10s) with a larger capacity is permitted with a corresponding recalculation of operating time | P2 |
| AUT-05 | Battery replacement is performed only with the Robot powered down; hot swapping is not required | P1 |
| AUT-06 | In the baseline version, the battery is charged with the original hoverboard charger through the original charge port; an integrated charger is not required | P1 |
| AUT-07 | Power and signal contacts of the dock must be reserved architecturally and in software | P3 |

### 4.2. Energy budget

| ID | Requirement | Priority |
|---|---|---|
| AUT-08 | Typical consumption: not more than 200 W | P1 |
| AUT-09 | Peak consumption: not more than 600 W for less than 10 s | P1 |
| AUT-10 | Pack continuous current — about 20 A; peak up to 40 A short-term; main branch fuse — 30–40 A at a DC rating of not less than 60 V | P1 |
| AUT-11 | DC-DC 12 V / 5 A, 5 V / 5–6 A, 3.3 V / 1 A; efficiency not less than 90 % at rated load | P1 |
| AUT-12 | The power circuit must remain operational during sags down to the 30 V pack cutoff and actuator transients | P1 |
| AUT-13 | A power margin of not less than 20 % must be provided for the camera, AI accelerator, and manipulator | P2 |

### 4.3. Operating time

| ID | Requirement | Priority |
|---|---|---|
| AUT-14 | Typical mixed cycle: not less than 30 min at 20 ± 5 °C, battery SoH ≥ 90 %, 2 kg load, 50 % of the time at 0.5 m/s, 30 % idle, 20 % maneuvering | P1 |
| AUT-15 | Aggressive cycle with accelerations up to 1.0 m/s and frequent turns: not less than 20 min | P1 |
| AUT-16 | Predominantly idle mode with sensors enabled: not less than 45 min | P2 |
| AUT-17 | Operating time must be measured until the 30.0 V protection trips, not until the battery is fully discharged | P1 |
| AUT-18 | Results must be accompanied by the load profile, temperature, and initial SoC | P1 |

### 4.4. Battery monitoring and low-charge behavior

| ID | Requirement | Priority |
|---|---|---|
| AUT-19 | Measurement of battery voltage and current via a bidirectional shunt at 100–500 Hz | P1 |
| AUT-20 | Voltage measurement error — not more than ±0.5 %; current — not more than ±2 % of full scale | P1 |
| AUT-21 | SoC calculation by coulomb counting with voltage correction; SoC error — not more than ±5 % | P1 |
| AUT-22 | SoH and accumulated energy estimation; logging of charge/discharge events | P2 |
| AUT-23 | Warning threshold: 36.0 V — acceleration limiting and operator notification | P1 |
| AUT-24 | Limiting threshold: 33.0 V — torque and speed limiting | P1 |
| AUT-25 | Emergency stop threshold: 30.0 V confirmed for 500 ms — actuator shutdown, transition to `FAULT`, controlled shutdown | P1 |
| AUT-26 | Threshold decisions must be made by the MCU independently of the mission computer | P1 |
| AUT-27 | The Robot must reduce load before the battery BMS protection trips; spontaneous BMS shutdown must not be a normal scenario | P1 |
| AUT-28 | Power telemetry: voltage, current, SoC, SoH, temperature — over the mainboard link at not less than 50 Hz (CAN only in a future controller generation) | P1 |
| AUT-29 | Threshold events must be logged with a timestamp, current, and voltage | P1 |
| AUT-30 | During E-stop, the actuator power circuits are disconnected; at stage 1 the latching kill switch in the battery line removes all battery power, so the event is latched in non-volatile memory as soon as it is detected | P1 |

### 4.5. Autonomy tests

1. Discharge on the test bench with a profile simulating the typical cycle; recording of voltage, current, temperature, and SoC.
2. Verification of the 36.0 / 33.0 / 30.0 V thresholds using a voltage simulator and load.
3. Verification of SoC accuracy by comparison with a reference wattmeter.
4. Verification of behavior when the BMS is disconnected and when voltage sags under peak load.
5. Verification that logging is preserved and the mission computer shuts down correctly during E-stop.

---

## 5. Mobility and dynamics requirements

### 5.1. Kinematics and actuators

| ID | Requirement | Priority |
|---|---|---|
| MOB-01 | Stage 1: two wheels only, no legs. Stage 2: wheeled-biped configuration — two legs, one wheel per leg; each leg is a planar 1-DOF four-bar (base A–B, upper crank A–C driven by a hip actuator, lower rocker B–D passive, wheel fork C–D–E). Height and fore-aft offset are coupled along the coupler curve. | P1 |
| MOB-02 | Stage 1 actuators — two donor direct-drive hub motors (~350 W each, 6.5 in wheel, radius 0.0825 m, 3 hall sensors each, ~2.5 N·m continuous and ~5 N·m peak per wheel), driven by the mainboard 3-phase bridges with FOC; bench limits 10 A and 400 rpm per motor. Stage 2 adds two hip actuators (model TBD) with external drivers — four actuators total. Feedback: hall position and speed plus phase and DC-link current on the wheels; hip actuators add direct temperature feedback. | P1 |
| MOB-03 | Configurable balance control: pitch via wheel torque, yaw via differential wheel torque; height via common hip angle and roll via differential hip height (stage 2 only) | P1 |
| MOB-04 | Modes: `IDLE`, `BALANCE`, `DAMPING`, `FAULT`, `E_STOP`, `SHUTDOWN` | P1 |
| MOB-05 | Transitions between modes — only via a validated command and precondition checks | P1 |

### 5.2. Balancing and stability

| ID | Requirement | Priority |
|---|---|---|
| MOB-06 | Maintaining balance on a horizontal hard surface with zero command for not less than 10 min without falling | P1 |
| MOB-07 | Recovery after an external horizontal impulse of up to 10 N·s applied to the body, without falling. P2 at stage 1 — the two hub motors deliver only about 5 N·m total at the 10 A bench limit, which is marginal. The target is validated after the hip stage. | P2 (stage 1) / P1 (stage 2) |
| MOB-08 | Recovery after a short-term deviation of up to 10° on a plane | P1 |
| MOB-09 | Configurable tilt limit 25–35°; if exceeded — torque limiting and transition to `DAMPING` | P1 |
| MOB-10 | Fall detection and transition to a safe state with actuator shutdown | P2 |
| MOB-11 | Self-righting after a fall is not required | — |
| MOB-31 | 3D balance allocation (stage 2): pitch and yaw via wheel torque, height via common hip angle, roll via differential hip height | P1 |
| MOB-32 | If a passive leg spring/damper is fitted, its stiffness and travel are identified and included in the balance model | P2 |

### 5.3. Speed, acceleration, and maneuverability

| ID | Requirement | Priority |
|---|---|---|
| MOB-12 | Maximum speed — not less than 1.5 m/s; autonomous operating — up to 1.0 m/s | P1 (operating) / P2 (maximum) |
| MOB-13 | Reverse — up to 0.5 m/s | P1 |
| MOB-14 | Turning radius in place — practically zero due to independent wheel control | P1 |
| MOB-15 | Acceleration and jerk limiting must prevent slipping and tipping | P1 |
| MOB-16 | Emergency braking from 1.0 m/s — while maintaining balance; braking distance not more than 0.5 m on a dry surface | P1 |

**Voltage note.** The donor pack sets the speed ceiling: at the 400 rpm bench limit the 6.5 in wheel
covers about 3.4 m/s no-load, so the 1.5 m/s maximum-speed target is met with margin at 42.0 V full
charge and is not voltage-limited until the pack approaches the 33.0 V limiting threshold. Torque, not
voltage, is the binding constraint near that threshold. No derating of MOB-12 is required, but maximum
speed must be verified at 33.0 V, not only at full charge.

### 5.4. Obstacle and terrain negotiation

| ID | Requirement | Priority |
|---|---|---|
| MOB-17 | Irregularities and thresholds up to 20 mm high — at a speed of up to 0.5 m/s | P1 |
| MOB-18 | Step/threshold up to 50 mm (stage 2) — at a speed of not more than 0.2 m/s using the hip four-bar to lift the wheel; validate on the bench | P1 |
| MOB-19 | Target capability to negotiate steps up to 100 mm in walking mode (stage 2) | P2 |
| MOB-20 | Longitudinal slope up to 10° and lateral slope up to 5° on a dry hard surface while maintaining balance | P1 |
| MOB-21 | Holding on a 10° slope for 10 min without slipping or overheating | P1 |
| MOB-22 | Seams, tile joints, and grooves up to 20 mm wide | P1 |
| MOB-23 | Surface transitions (carpet–tile, elevator threshold) up to 15 mm high | P1 |
| MOB-24 | Stair flights and steps above 100 mm are not required | — |
| MOB-25 | Body height adjustment (stage 2) — not less than 100 mm; the range is refined during detailed design | P2 |

**Design note (four-bar leg).** At stage 1 there is no hip DOF, so wheel height and fore-aft offset are
fixed and MOB-18/MOB-19 are stage 2 capabilities: a 50 mm step is out of reach with wheels alone and the
Robot stops before such an obstacle. At stage 2, with one active hip DOF per leg, body height and wheel
fore-aft offset are coupled along the coupler curve, so step negotiation requires a deliberate weight
shift and roll; validate the 50 mm P1 target on the bench before committing the mechanical design. The
prototype uses one wheel per leg; dual wheels per leg are deferred.

### 5.5. Load, mass, and thermal conditions

| ID | Requirement | Priority |
|---|---|---|
| MOB-26 | Balancing and movement with a 2 kg payload | P1 |
| MOB-27 | Operation with a load of up to 4 kg with speed and acceleration limiting | P2 |
| MOB-28 | Dry mass without battery — not more than 15 kg | P2 |
| MOB-29 | Thermal protection of the drives: current-based thermal model with warning, limiting, and shutdown; hip actuators add direct temperature feedback (stage 2) | P1 |
| MOB-30 | The temperature of the body and motors must not exceed safe limits during prolonged movement, monitored directly where sensors exist and by the thermal model otherwise | P1 |

### 5.6. Mobility tests

1. Balancing test bench (stage 1): 10 min hold, impulse impact (P2 at stage 1), 10° deviation.
2. Test track: 20 mm threshold, 10° slope, 5° lateral slope, 20 mm seam; the 50 mm step and slope-holding at height are repeated on the stage 2 four-actuator bench.
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
| NAV-03 | IMU and wheel odometry must be used in the state estimation loop; wheel odometry is derived from the hub motor hall sensors | P1 |
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
| EXT-01 | Modular Cargo structure: `jbot_common`, `jbot_hardware`, `jbot_navigation`, `jbot_node` | P1 |
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
| EXT-11 | Capture at not less than 30 frames/s at 640×480; object detection — not less than 10 Hz on the mission computer or an AI accelerator | P2 |
| EXT-12 | Camera calibration is stored in the configuration and verified at startup | P2 |
| EXT-13 | A camera failure must not block baseline lidar navigation | P1 |

### 7.3. Manipulator connection

| ID | Requirement | Priority |
|---|---|---|
| EXT-14 | Mechanical interface: mounting points, fasteners, and space for a manipulator with 4–6 degrees of freedom | P3 |
| EXT-15 | Electrical interface: 12 V / 5 A (60 W), protection, separate branch if necessary | P3 |
| EXT-16 | Communication: reserved CAN FD with 29-bit identifiers and a separate ID range for manipulator nodes; the stage 1–2 mainboard has no CAN transceiver, so the baseline link is UART and CAN FD is a deferred option | P3 |
| EXT-17 | Manipulator control is integrated into the behavioral layer, but not into the 200 Hz balance loop | P1 |
| EXT-18 | The manipulator must not disturb balance; the supervisor limits speed/torque when the load is extended | P3 |
| EXT-19 | Target manipulator payload — not less than 1 kg at the gripper | P3 |
| EXT-20 | Manipulator emergency stop — via the common E-stop and a software command | P3 |
| EXT-21 | Manipulator installation is performed with the power off; hot plugging is not required | P3 |

### 7.4. Automatic charging dock

| ID | Requirement | Priority |
|---|---|---|
| EXT-22 | Architecturally reserved: power contacts, guides, docking sensor, data channel (IR or UART) | P3 |
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
| EXT-31 | External diagnostic port — isolated; UART on the donor mainboard, CAN deferred to a future controller generation | P2 |
| EXT-32 | Ability to add an NVMe drive for maps, logs, and models | P2 |
| EXT-33 | All expansions must not increase the balance loop jitter by more than 50 µs | P1 |

### 7.6. Extensibility tests

1. Connection of a reference RGB-D camera: capture, calibration, synchronization, detection.
2. Connection of a manipulator emulator as a UART node (CAN only if a future controller provides it); verification of IDs, power, emergency stop.
3. Verification of the dock software interface on a simulator.
4. Verification that failure/disconnection of any expansion does not disrupt balance and baseline navigation.
5. Verification that jitter does not increase with expansions enabled.

---

## 8. Interface and software requirements

### 8.1. Buses and protocols

| ID | Requirement | Priority |
|---|---|---|
| INT-01 | Primary link for stages 1–2 — UART: USART2 (PA2 TX, PA3 RX, not 5 V tolerant) to the ExpressLRS receiver, USART3 (PB10/PB11) to the mission computer or hip driver. CAN FD is reserved for a future controller generation and is not present on the donor mainboard. | P1 |
| INT-02 | Frame up to 64 bytes: 8-byte header, up to 56 bytes of payload; over UART the same header is carried inside COBS framing | P1 |
| INT-03 | CAN FD (reserved, not fitted on the donor board): 120 Ω termination at both ends, stub length less than 0.3 m | P3 |
| INT-04 | UART: 8N1 with COBS and CRC-16; the ExpressLRS receiver link runs at 420 kbaud CRSF, the mission computer link at 921600 bit/s or 1 Mbit/s | P1 |
| INT-05 | All frames: protocol version, type, flags, 16-bit sequence, timestamp, CRC-16 | P1 |
| INT-06 | Control commands — latest-value; telemetry — loss-tolerant | P1 |
| INT-07 | E-stop and heartbeat have the highest priority and are not displaced by telemetry | P1 |
| INT-08 | Message schema — `postcard`/`serde`, little-endian, fixed header | P1 |
| INT-09 | External operator port — isolated; UART on the donor mainboard, CAN reserved | P2 |
| INT-10 | Power: VBAT 30–42 V (donor 10s pack), 12 V / 5 A, 5 V / 5–6 A, 3.3 V / 1 A | P1 |

> **Link note.** Stages 1–2 use UART only. USART2 (left 4-pin cable, PA2/PA3, not 5 V tolerant) carries the ExpressLRS CRSF receiver; USART3 and I2C2 share PB10/PB11 on the right cable, so exactly one function is active at a time — the BNO055 sits on I2C2 in stage 1. The STM32F103RCT6 has a CAN controller, but its pins are not routed on the donor board and no transceiver is fitted, so CAN FD appears only as a reserved future path (INT-03, EXT-16, EXT-31). INT-01 applies to the mission computer–mainboard link.

### 8.2. Software requirements

| ID | Requirement | Priority |
|---|---|---|
| INT-11 | Primary language — Rust edition 2021+; MCU firmware — `no_std` | P1 |
| INT-12 | Dynamic memory allocation, `panic!`, and `unwrap()` are prohibited in the control loop | P1 |
| INT-13 | Queues are bounded; telemetry overflow must not block E-stop | P1 |
| INT-14 | `jbot_node` — composition root; binds transport, estimation, control, navigation, and supervisor | P1 |
| INT-15 | Middleware: Zenoh for internal exchange; ROS2 — via the bridge | P1 |
| INT-16 | DDS/Zenoh are not used in the 200 Hz balance loop | P1 |
| INT-17 | QoS: commands — reliable/keep-last; telemetry — best-effort; maps/TF — transient-local; E-stop — reliable, highest priority | P1 |
| INT-18 | Logging — structured (`tracing`), with node/task/robot ID; no high-frequency logging in the control loop | P1 |
| INT-19 | Metrics: frame loss, CRC errors, bus-off, command age, jitter, voltage, temperatures, faults | P1 |
| INT-20 | MCU blackbox: ring buffer of the last N seconds with upload capability | P2 |
| INT-21 | CI: `cargo fmt --check`, `cargo clippy -D warnings`, `cargo test`, aarch64 cross-build, firmware build, `cargo deny`, `cargo audit` | P1 |
| INT-22 | Deployment on the mission computer via systemd with `Restart=on-failure`, watchdog, and resource limits | P1 |

---

## 9. Safety requirements

| ID | Requirement | Priority |
|---|---|---|
| SAF-01 | Safety has the highest priority over functionality and performance | P1 |
| SAF-02 | E-stop — a hardware function independent of Linux and the MCU: latching kill switch in the battery line plus a dual-channel NC loop. The power button is a soft latch and is not an E-stop. | P1 |
| SAF-03 | E-stop removes actuator power; logic power is preserved for logging and shutdown where a separate logic branch is fitted. At stage 1 the kill switch removes all battery power, so the event is latched in non-volatile memory as soon as it is detected. | P1 |
| SAF-04 | Watchdog chain: hardware E-stop → MCU IWDG → 100 ms command watchdog → radio and mission computer heartbeat → systemd watchdog → supervisor | P1 |
| SAF-05 | Upon loss of a valid command for more than 100 ms, the MCU transitions to `DAMPING` with zero setpoints; torque is never removed while balancing | P1 |
| SAF-06 | On a critical fault, the actuators are shut down and the fault is latched until a manual reset | P1 |
| SAF-07 | Reset of a critical fault is possible only when the Robot is stationary and the self-test passes | P1 |
| SAF-08 | All faults are published over the mainboard link and to Zenoh/ROS2 with a code, severity, and timestamp | P1 |
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
| REL-05 | Access to the mainboard, mission computer, connectors, and fuses without fully disassembling the body | P1 |
| REL-06 | Calibrations are stored in non-volatile memory and restored after reboot | P1 |
| REL-07 | Maintenance of fault and event logs with preservation during power loss | P1 |
| REL-08 | Maps and logs are placed on a separate writable partition; the root FS is predominantly read-only | P2 |

---

## 11. Development stages and delivery scope

### 11.1. Stages

| Stage | Content | Result |
|---|---|---|
| 1 | Architecture, requirements, Cargo workspace structure | This SRS, `architecture.md`, crate skeletons |
| 2 | Mainboard firmware for the stage 1 platform (two hub wheels only): FOC/SVPWM 16 kHz, balance loop 200 Hz, BNO055, ExpressLRS link, UART protocol | Test bench with two hub wheels, protocol, timing and jitter report |
| 3 | State estimation and balancing on the stage 1 platform | Balancing test bench, stability report |
| 4 | Stage 2 platform: two hip actuators, four-bar legs, height and roll control, four actuators total | Four-actuator bench, step and height report |
| 5 | SLAM, localization, navigation, ROS2 bridge | Autonomous runs, maps, navigation report |
| 6 | Extensions: camera, manipulator, dock | Interfaces, extension tests, final report |

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
| Integration | UART loopback, transport failure, timeouts (CAN only when a future controller provides it) |
| Simulation | Plant model, sensor noise, balancing, navigation |
| HIL | Mainboard, wheel bridges, hip drivers, mission computer, fault injection |
| System | E-stop, battery disconnection, overheating, communication loss, 24-hour run |
| Regression | Log playback, metric comparison |

### 12.2. Acceptance criteria

1. All P1 requirements are confirmed by test reports.
2. P2 requirements are implemented or have an agreed justification for deferral.
3. There are no critical faults and no open safety defects.
4. Balance loop jitter at 200 Hz does not exceed 50 µs, and loss of command leads to `DAMPING` within 100 ms.
5. Autonomous navigation: not less than 95 successful runs out of 100 without collisions.
6. Operating time from the donor pack (36 V nominal, ~158 W·h) in a typical cycle — not less than 30 min.
7. Stage 2: four actuators run together on the bench, the 50 mm step and body height range are demonstrated, and MOB-07 is validated.
8. The documentation and source code package is delivered in full.

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

1. Final MCU: fixed for stages 1–2 as the donor STM32F103RCT6 mainboard (Cortex-M3, 72 MHz, 256 KB flash, 48 KB RAM). A later controller (STM32H7 class or equivalent) remains an open option only if the platform outgrows the F103; no selection is required for stages 1–2.
2. Actuator interface: stage 1 uses two donor hub motors driven by the mainboard's 12 MOSFETs. Stage 2 adds two hip actuators (model TBD) with their own drivers on USART3; torque, thermal, and mechanical envelopes are still to be fixed. USART3 is the only free port, so the hip driver and the later mission computer must share it — the multiplexing or arbitration scheme is still open.
3. Leg topology (stage 2): decided — 1-DOF four-bar (base A–B, upper crank A–C, lower rocker B–D, wheel fork C–D–E, optional spring). Remaining: link lengths, spring rate/travel, height range, and E-position.
4. Selection of lidar and camera, their mounting, and impact on the mass/power budget.
5. Nav2 as is via the bridge or a fully Rust `jbot_navigation` stack.
6. AI accelerator: Hailo-8L or CPU-only; runtime `ort` / `tract` / `candle`.
7. Firmware update path: SWD debug probe or the STM32 system bootloader over UART; the donor board exposes no dedicated USB port, so a CAN bootloader is deferred with the future controller.
8. Requirements for the enclosure, connectors, and maintainability.
9. Refinement of mass, payload, and height adjustment range after prototype manufacturing.
10. Dock standard and the parameters of the original hoverboard charger.
11. Power pack: stages 1–2 use the donor 10s2p pack (36 V nominal, 42.0 V full, 4.4 A·h, ~158 W·h). A larger pack of the same voltage class is a future option; the runtime targets in section 3.2 assume the donor pack.

---

## 15. Approval

| Role | Full name | Signature | Date |
|---|---|---|---|
| Customer | | | |
| Contractor | | | |
| Project manager | | | |
| Safety officer | | | |
