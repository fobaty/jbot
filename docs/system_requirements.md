# System Requirements Specification (SRS)
## Wheeled-Bipedal Autonomous Robot (Project Code: JBOT-WheeledBiped)

---

## 1. Overview & Objectives
The **JBOT-WheeledBiped** is an advanced autonomous robotic platform combining wheeled mobility with bipedal balancing dynamics (similar to Roadrunner / Upkie concepts). It is designed for research, autonomous navigation, dynamic balancing, and extensible manipulation tasks in unstructured environments.

### Key Objectives:
- High-bandwidth real-time balance control loop (≥ 1 kHz) on low-level MCUs.
- High-level autonomous navigation, mapping (SLAM), and perception running on Raspberry Pi 5.
- Modular Rust-based software architecture (zenoh/ROS2 middleware, safe asynchronous communication).
- Modular hardware expansion slots for future manipulator arms and sensory upgrades.

---

## 2. Hardware Specifications

### 2.1. Power Subsystem
- **Battery:** Milwaukee M18 XC5.0 (18 V Li-ion pack) is the baseline. A 24–36 V pack is the preferred option for actuator speed and torque margin; the final voltage is an open decision.
- **Protection & Regulation:** BMS with overcurrent, thermal, and under-voltage protection; high-efficiency step-down DC-DC converters (12 V, 5 V, 3.3 V rails).
- **Estop:** Hardware emergency stop switch interrupting power to all four actuators immediately.

### 2.2. Compute Architecture
- **High-Level Computer:** Raspberry Pi 5 (16GB RAM) running Linux (Ubuntu Server / Raspberry Pi OS 64-bit). Responsible for navigation, perception, high-level planning, and middleware bridging.
- **Low-Level Microcontroller:** ESP32-S3 or Teensy 4.1. Responsible for the 1 kHz balance loop, IMU sampling, CAN coordination, safety watchdog, and supervisory control of the actuators. Hip QDD actuators integrate FOC, encoder, and driver; wheel hub motors may integrate their driver or use an external FOC stage (open decision).
- **Interconnect:** CAN FD backbone between the MCU and Raspberry Pi 5, plus a classic CAN 1 Mbit/s actuator bus for the RobStride-class hip QDDs; UART as a deterministic fallback.

### 2.3. Actuation & Mechanics
- **Actuator count:** four motors total — two hip QDDs in the body and two wheel hub motors, one wheel per leg.
- **Hip actuators:** QDD class 36–60 N·m peak (for example RobStride RS06 or RS03) with integrated FOC, encoder, and driver. For a 15 kg platform, RS02 is a marginal choice and RS05 is insufficient.
- **Wheel actuators:** 6.5–8 in hub motors with at least 5–8 N·m per wheel, or an equivalent QDD. Each wheel has its own motor; dual wheels per leg are not part of the baseline.
- **Wheels:** one smooth-tread wheel per leg for indoor use; off-road tires are an optional swap.
- **Wheel drive:** one powered wheel per leg avoids the scrub of a locked dual-wheel axle. Traction and slip must be validated on the target surface.

### 2.4. Leg Mechanism (1-DOF Four-Bar)
- **Topology:** each leg is a planar four-bar linkage with one active DOF. The links are the ground link **A–B** (the chassis/base), the upper crank **A–C** driven by the hip QDD, the lower rocker **B–D** (passive), and the wheel fork **C–D–E** (one rigid coupler) with the wheel axle at **E**.
- **Base:** the base is the chassis link **A–B**, which connects the hip motor axis A and the passive rocker pivot B. The moving wheel fork is not the base.
- **Knee:** the bend of the wheel fork is rigid; there is no articulated knee joint and no independent shank control.
- **Coupled motion:** the hip angle sets the wheel position along a fixed coupler curve. Body height and wheel fore-aft offset are coupled and cannot be commanded independently.
- **Compliance:** an optional spring/damper between the base and the upper crank provides passive suspension and impact relief. It does not add a controllable DOF; its stiffness and travel must be included in the model if fitted.

### 2.5. Sensors
- **IMU:** ICM-42688-P or BNO085 (6/9-DoF high-precision inertial measurement unit, mounted near base/body center).
- **LiDAR:** 2D LiDAR (e.g., RPLIDAR / LDROBOT) or 3D LiDAR (e.g., Livox / Ouster mini) for SLAM and obstacle detection.
- **Vision (Optional/Upgrade):** RGB-D camera (e.g., Intel RealSense D435i / OAK-D) for visual SLAM and object detection.

---

## 3. Software Architecture & Requirements

### 3.1. Language & Ecosystem
- **Primary Language:** Rust (edition 2021+), strictly adhering to safety, zero-cost abstractions, and `no_std` compatibility for low-level firmware where applicable.
- **Middleware:** Zenoh and/or ROS2 (rclrs / zenoh-ros2-bridge) for distributed node communication.

### 3.2. Real-Time & Low-Level Firmware (MCU)
- Deterministic loop frequency: $\ge 1000\text{ Hz}$ for the balance loop. Hip QDD current loops run inside the integrated actuators; wheel FOC runs in the hub driver or an external stage.
- Balance scope is 3D: pitch via wheel torque, yaw via differential wheel torque, height via the common hip angle, and roll via differential hip height. The 2D wheeled inverted pendulum is a reduced-order model.
- Hardware watchdog timer triggering safe shutdown/damping if communication with RPi 5 is lost ($> 100\text{ ms}$).

### 3.3. High-Level Software (RPi 5)
- **State Estimation:** Extended Kalman Filter (EKF) combining wheel odometry and IMU data.
- **Navigation:** Nav2 / custom Rust navigation stack supporting local costmaps, global path planning, and obstacle avoidance.
- **Safety Monitor:** System supervisor tracking CPU/GPU temperature, battery voltage, communication latency, and motor fault flags.
