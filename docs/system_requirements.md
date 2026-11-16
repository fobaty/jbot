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

- Balance and motor control local to the real-time MCU tier — a Raspberry Pi
  Pico 2 W for the balance loop and the STM32F103 mainboard for the FOC current
  loop — independent of the radio link and of any mission computer.
- Radio command from a BetaFPV LiteRadio 3 Pro through a RadioMaster XR1
  receiver over CRSF, on the Pico.
- Tilt and tilt rate from a BNO055 mounted on the chassis and read by the Pico.
- Conservative reuse of the donor power stage: original MOSFET bridges, current
  shunts, battery, and BMS.
- A Rust workspace that composes the same control logic on the host and on the
  target, with separate embedded firmware projects for the Pico and the mainboard.

---

## 2. Hardware Specifications

### 2.1. Reused Donor Hardware

- **Mainboard:** STM32F103RCT6, Cortex-M3 at 72 MHz, 256 KB flash, 48 KB RAM,
  12 MOSFETs in two 3-phase bridges, hall inputs, current shunts, buzzer, status
  LED, and a firmware-held power latch. This is the only board that drives the
  motors; it is kept as the torque actuator node.
- **Motors:** two direct-drive hub motors, about 350 W each, 6.5 in wheels
  (radius 0.0825 m), three hall sensors per motor. Bench limit: 10 A and 400 rpm
  per motor until thermal behaviour is measured.
- **Battery:** 10s2p Li-ion pack, 36 V nominal, 42.0 V full, about 30 V cutoff,
  4.4 A·h, ~158 W·h, with an integrated BMS and the original charge port.
- **User controls:** the original power button and charge port. The button is a
  soft latch, not an emergency stop.
- **Discarded:** both sideboards, the deck, and the covers. The sideboards are
  UART slaves with their own MCU and IMU and would contend with the Pico link on
  the same wires.

### 2.2. Added Hardware

- **Real-time controller:** Raspberry Pi Pico 2 W (RP2350, dual Cortex-M33 at
  150 MHz with FPU, 520 KB SRAM, 4 MB flash). It runs the balance loop, radio
  ingest, attitude estimation, limits, the state machine, telemetry, and the
  blackbox. Its wireless subsystem is not initialised in stage 1.
- **Radio transmitter:** BetaFPV LiteRadio 3 Pro, ExpressLRS 2.4 GHz, 8
  channels. It is an operator input, never a safety path.
- **Radio receiver:** RadioMaster XR1, ExpressLRS 2.4 GHz, CRSF output,
  420 kbaud, connected to the Pico's UART1.
- **IMU:** Bosch BNO055, 9-DOF with onboard fusion, on the Pico's I2C1 at
  400 kHz and 100 Hz, mounted rigidly and low on the chassis.
- **Power:** 36–42 V to 5 V DC-DC rated for at least 60 V input and 1.5 A, a main
  fuse, and a latching kill switch in the battery line.
- **Stage 2:** two hip actuators with drivers, model still open, and a leg
  mechanism. The mainboard cannot drive them; it has only two motor bridges.

### 2.3. Compute Architecture

- **Real-time controller (Tier 1a):** the Pico 2 W. It runs the 200 Hz balance
  loop, reads the BNO055, receives CRSF setpoints, enforces tilt, voltage, and
  torque limits, runs the safety state machine, and sends torque commands to the
  mainboard over UART.
- **Actuator node (Tier 1b):** the donor STM32F103RCT6 mainboard. It runs the
  16 kHz FOC current loop, applies the received torque request, enforces phase
  and DC-link current limits, holds the power latch, and zeroes torque after
  100 ms without a valid command.
- **Mission computer (stage 3, Tier 2):** Raspberry Pi 5 class or equivalent. It
  sends setpoints to the Pico over UART and runs state estimation, telemetry,
  and navigation. It is not a safety authority and cannot arm the robot.
- **Interconnect:** Pico UART0 to the mainboard USART2 (left cable, EFeru
  `TRQ_MODE` frames); Pico UART1 to the XR1; Pico I2C1 to the BNO055; Pico PIO
  UART to the mission computer. The mainboard's USART3 (right cable, PB10/PB11)
  is free and reserved for the stage 2 hip driver.
- **CAN:** present on the F103 but not routed on this board. Reserved for a later
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

- **IMU:** BNO055, 100 Hz fusion on the Pico, gyro propagated to the 200 Hz
  balance rate. ICM-42688-P remains the high-rate fallback.
- **Motor feedback:** hall sensors and phase current shunts on the mainboard,
  read locally by the FOC loop and reported to the Pico at 100 Hz.
- **Pack monitoring:** the mainboard voltage divider and DC-link shunt, with
  thresholds of 36.0 / 33.0 / 30.0 V evaluated by the Pico from the feedback.
- **LiDAR and camera:** not part of stage 1 or stage 2; reserved for the
  navigation stage.

---

## 3. Software Architecture & Requirements

### 3.1. Language & Ecosystem

- **Primary language:** Rust (edition 2021+), with `no_std` firmware on the Pico
  and a host workspace for the higher-level software.
- **Pico stack:** `thumbv8m.main-none-eabihf`, `embassy-rp` (RP2350), `embassy-executor`,
  `embassy-sync`, `heapless`, `defmt`, `probe-rs`.
- **Mainboard stack:** pinned upstream EFeru C, built with PlatformIO, patched to
  `TRQ_MODE` with a 100 ms serial timeout. No Rust on the F103 in stages 1–2.
- **Middleware:** Zenoh and/or ROS 2 on the mission computer, once it exists.

### 3.2. Real-Time & Low-Level Firmware (MCU)

- Deterministic balance loop at 200 Hz on the Pico 2 W, with the FOC current
  loop at 16 kHz on the F103 mainboard.
- The Pico sends torque commands at 200–400 Hz; the mainboard applies the latest
  one in its 5 ms loop, so the actuator path is quantized to 5 ms.
- Balance scope for stage 1 is pitch and yaw only, through differential wheel
  torque. Stage 2 adds height and roll through the hips.
- Radio and mission-computer inputs are setpoints. Only the radio can arm.
- Loss of both links means zero setpoints while balancing continues; the
  mainboard zeroes torque only after 100 ms without a valid frame.
- Pico hardware watchdog feed is health-gated; the mainboard's timeout and its
  own IWDG are independent of the Pico. Tilt cutoff zeroes torque and latches a
  fault.
- The Pico–mainboard link uses the stock EFeru frame format; golden byte vectors
  are part of the test suite.

### 3.3. High-Level Software (Mission Computer)

- **State Estimation:** Extended Kalman Filter combining wheel odometry and IMU
  data.
- **Navigation:** Nav2-equivalent stack supporting local costmaps, global path
  planning, and obstacle avoidance.
- **Safety Monitor:** supervisor tracking CPU temperature, battery voltage,
  communication latency, and motor fault flags.
