# System Requirements Specification (SRS)
## Wheeled Balancing Robot on a Donor Hoverboard (Project Code: JBOT-Wheeled Biped)

---

## 1. Overview & Objectives

**JBOT-Wheeled Biped** is a self-balancing wheeled robot built on a donor
hoverboard and extended in two stages. Stage 1 is a two-wheel balancing platform
that drives and steers on command from a handheld radio. Stage 2 adds two hip
actuators so the legs can move the body independently. The wheel controller, the
radio link, and the balancing core stay the same across both stages.

### Key Objectives:

- Balance and motor control local to the hoverboard mainboard, independent of
  the radio link and of any mission computer.
- Radio command from a BetaFPV LiteRadio 3 Pro through a RadioMaster XR1
  receiver over CRSF.
- Tilt and tilt rate from a BNO055 mounted on the chassis.
- Conservative reuse of the donor power stage: original MOSFET bridges, current
  shunts, battery, and BMS.
- A Rust workspace that composes the same control logic on the host and on the
  target, with a separate embedded firmware project.

---

## 2. Hardware Specifications

### 2.1. Reused Donor Hardware

- **Mainboard:** STM32F103RCT6, Cortex-M3 at 72 MHz, 256 KB flash, 48 KB RAM,
  12 MOSFETs in two 3-phase bridges, hall inputs, current shunts, buzzer, status
  LED, and a firmware-held power latch. This is the only board that drives the
  motors.
- **Motors:** two direct-drive hub motors, about 350 W each, 6.5 in wheels
  (radius 0.0825 m), three hall sensors per motor. Bench limit: 10 A and 400 rpm
  per motor until thermal behaviour is measured.
- **Battery:** 10s2p Li-ion pack, 36 V nominal, 42.0 V full, about 30 V cutoff,
  4.4 A·h, ~158 W·h, with an integrated BMS and the original charge port.
- **User controls:** the original power button and charge port. The button is a
  soft latch, not an emergency stop.
- **Discarded:** both sideboards, the deck, and the covers. The sideboards are
  UART slaves with their own MCU and IMU and would contend with the radio and
  the IMU on the same wires.

### 2.2. Added Hardware

- **Radio transmitter:** BetaFPV LiteRadio 3 Pro, ExpressLRS 2.4 GHz, 8
  channels. It is an operator input, never a safety path.
- **Radio receiver:** RadioMaster XR1, ExpressLRS 2.4 GHz, CRSF output,
  420 kbaud, connected to USART2 on the left cable.
- **IMU:** Bosch BNO055, 9-DOF with onboard fusion, I2C at 100 Hz, mounted
  rigidly and low on the chassis.
- **Power:** 36–42 V to 5 V DC-DC rated for at least 60 V input, a main fuse, and
  a latching kill switch in the battery line.
- **Stage 2:** two hip actuators with drivers, model still open, and a leg
  mechanism. The mainboard cannot drive them; it has only two motor bridges.

### 2.3. Compute Architecture

- **Low-level controller:** the donor STM32F103RCT6 mainboard. It runs the FOC
  current loop at 16 kHz and the balance loop at 200 Hz, reads the BNO055 and the
  hall sensors, receives CRSF setpoints, and enforces tilt, current, and voltage
  limits.
- **Mission computer (stage 3):** Raspberry Pi 5 class or equivalent. It sends
  setpoints over USART3 and runs state estimation, telemetry, and navigation. It
  is not a safety authority and cannot arm the robot.
- **Interconnect:** CRSF on USART2; IMU on I2C2; mission computer or hip driver
  on USART3. The right cable's pins are shared between USART3 and I2C2, so the
  IMU moves to I2C1 (two soldered wires) when the link is needed.
- **CAN:** present on the MCU but not routed on this board. Reserved for a later
  controller or an external hip bus; not used in stage 1.

### 2.4. Actuation & Mechanics

- **Stage 1:** two hub motors, one wheel per side, no legs. Balance, forward and
  reverse drive, turning, and spin in place.
- **Stage 2:** two hip actuators, one per leg, driving a planar 1-DOF four-bar
  per side. The hip angle moves the wheel along a fixed coupler curve; body
  height and fore-aft offset are coupled.
- **Torque budget:** about 5 N·m total from the two wheels at the bench current
  limit. The centre of mass must stay low: at 10 kg, a 0.25 m CoM height needs
  about 25 N·m per radian of tilt, which leaves roughly 12° of recoverable lean.
- **Wheels:** one powered wheel per side, no shared axle, so turning in place
  needs no scrub.

### 2.5. Sensors

- **IMU:** BNO055, 100 Hz fusion, gyro propagated to the 200 Hz balance rate.
  ICM-42688-P remains the high-rate fallback.
- **Motor feedback:** hall sensors and phase current shunts on the mainboard.
- **Pack monitoring:** the mainboard voltage divider and DC-link shunt, with
  thresholds of 36.0 / 33.0 / 30.0 V.
- **LiDAR and camera:** not part of stage 1 or stage 2; reserved for the
  navigation stage.

---

## 3. Software Architecture & Requirements

### 3.1. Language & Ecosystem

- **Primary language:** Rust (edition 2021+), with `no_std` firmware for the
  mainboard and a host workspace for the higher-level software.
- **Embedded stack:** `thumbv7m-none-eabi`, `stm32f1xx-hal`, RTIC 2, with the
  proven fixed-point FOC core reused as generated C behind a thin shim.
- **Middleware:** Zenoh and/or ROS 2 on the mission computer, once it exists.

### 3.2. Real-Time & Low-Level Firmware (MCU)

- Deterministic balance loop at 200 Hz on the F103 mainboard, with the FOC
  current loop at 16 kHz.
- Balance scope for stage 1 is pitch and yaw only, through differential wheel
  torque. Stage 2 adds height and roll through the hips.
- Radio and mission-computer inputs are setpoints. Only the radio can arm.
- Loss of both links means zero setpoints while balancing continues.
- Hardware watchdog and tilt cutoff; a command timeout must never remove motor
  torque while the robot is balancing.

### 3.3. High-Level Software (Mission Computer)

- **State Estimation:** Extended Kalman Filter combining wheel odometry and IMU
  data.
- **Navigation:** Nav2-equivalent stack supporting local costmaps, global path
  planning, and obstacle avoidance.
- **Safety Monitor:** supervisor tracking CPU temperature, battery voltage,
  communication latency, and motor fault flags.
