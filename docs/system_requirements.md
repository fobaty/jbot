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
- **Battery:** Milwaukee M18 XC5.0 (18V Li-ion pack).
- **Protection & Regulation:** BMS with overcurrent, thermal, and under-voltage protection; high-efficiency step-down DC-DC converters (12V, 5V, 24V rails).
- **Estop:** Hardware emergency stop switch interrupting power to motor drivers immediately.

### 2.2. Compute Architecture
- **High-Level Computer:** Raspberry Pi 5 (16GB RAM) running Linux (Ubuntu Server / Raspberry Pi OS 64-bit). Responsible for navigation, perception, high-level planning, and middleware bridging.
- **Low-Level Microcontroller:** ESP32-S3 or Teensy 4.1. Responsible for FOC motor commutation, IMU data sampling, safety watchdog, and real-time balancing control loops.
- **Interconnect:** High-speed CAN bus (FD) and/or robust Serial (UART over USB/GPIO) supporting deterministic framing.

### 2.3. Actuation & Mechanics
- **Drive & Leg Joints:** Quasi-Direct Drive (QDD) / brushless DC (BLDC) motors driven by high-performance FOC controllers (e.g., SimpleFOC / custom ODrive-compatible boards).
- **Configuration:** Wheeled-bipedal hybrid kinematics (active wheel drive per leg, pitch/hip/knee joints for posture and height adjustment).

### 2.4. Sensors
- **IMU:** ICM-42688-P or BNO085 (6/9-DoF high-precision inertial measurement unit, mounted near base/body center).
- **LiDAR:** 2D LiDAR (e.g., RPLIDAR / LDROBOT) or 3D LiDAR (e.g., Livox / Ouster mini) for SLAM and obstacle detection.
- **Vision (Optional/Upgrade):** RGB-D camera (e.g., Intel RealSense D435i / OAK-D) for visual SLAM and object detection.

---

## 3. Software Architecture & Requirements

### 3.1. Language & Ecosystem
- **Primary Language:** Rust (edition 2021+), strictly adhering to safety, zero-cost abstractions, and `no_std` compatibility for low-level firmware where applicable.
- **Middleware:** Zenoh and/or ROS2 (rclrs / zenoh-ros2-bridge) for distributed node communication.

### 3.2. Real-Time & Low-Level Firmware (MCU)
- Deterministic loop frequency: $\ge 1000\text{ Hz}$ for balancing control and motor FOC.
- Hardware watchdog timer triggering safe shutdown/damping if communication with RPi 5 is lost ($> 100\text{ ms}$).

### 3.3. High-Level Software (RPi 5)
- **State Estimation:** Extended Kalman Filter (EKF) combining wheel odometry and IMU data.
- **Navigation:** Nav2 / custom Rust navigation stack supporting local costmaps, global path planning, and obstacle avoidance.
- **Safety Monitor:** System supervisor tracking CPU/GPU temperature, battery voltage, communication latency, and motor fault flags.
