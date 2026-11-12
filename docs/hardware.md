# Hardware Baseline

| Field | Value |
|---|---|
| Document | Donor hardware, radio link, IMU, and interfaces |
| Code | HW-001 |
| Version | 0.2 |
| Date | 2026-11-12 |
| Status | Stage 1 baseline fixed; stage 2 actuators open |
| Scope | STM32F103RCT6 hoverboard mainboard, hub motors, 10s pack, XR1/LiteRadio, BNO055, hip actuators |

This document is the single description of the physical robot. Every value here
must match `config/robot.toml`, [`docs/bom.md`](bom.md), and the firmware pin map
in [`firmware/README.md`](../firmware/README.md). Items marked **open** may
change; items marked **donor** are fixed by the hoverboard.

## 1. Where the hardware comes from

The robot is built around a stock hoverboard. The mainboard is not replaced and
not bypassed: it remains the only board that drives the wheel motors.

| Donor part | Quantity | Specification | Role |
|---|---:|---|---|
| Mainboard | 1 | STM32F103RCT6, Cortex-M3 at 72 MHz, 256 KB flash, 48 KB RAM, 12 MOSFETs in two 3-phase bridges | FOC, balancing, power latch, telemetry |
| Hub motor | 2 | Direct drive, about 350 W, 15 hall states, 6.5 in wheel (radius 0.0825 m) | Drive and speed feedback |
| Battery | 1 | 10s2p Li-ion, 36 V nominal, 42.0 V full, about 30 V cutoff, 4.4 A·h, ~158 W·h, integrated BMS | Energy |
| Power button | 1 | Momentary button, firmware-held latch | Power on/off |
| Charge port | 1 | BMS charge input plus a charge-detect line | Charging, charge interlock |

**Discarded:** both sideboards, the deck, and the plastic covers. The sideboards
contain their own MCU (STM32F103C8T6 or GD32F130C6), a MPU6050-class IMU, the
battery LEDs, and the optical footpad sensors. They are UART slaves that
transmit on the same two signal wires the radio and the IMU need, and they do
not control the motors. Keeping them connected would put two transmitters on one
line.

The power button and the power latch are on the mainboard, so removing the
sideboards does not affect power-on. The only loss is the sideboard battery LEDs,
which CRSF telemetry replaces.

## 2. Mainboard

### 2.1. Capabilities

| Function | Detail |
|---|---|
| MCU | STM32F103RCT6, 72 MHz, 256 KB flash, 48 KB SRAM, single-precision FPU absent (soft float) |
| Power stage | 2× 3-phase MOSFET bridge, 12 FETs, 16 kHz PWM with 48-count dead time |
| Current sensing | Two phase shunts per motor plus a DC-link shunt, sampled by ADC/DMA |
| Rotor feedback | Three hall inputs per motor, 15 states per electrical revolution |
| Battery measurement | Resistive divider into ADC, calibrated in firmware |
| User I/O | Status LED and buzzer on the mainboard |
| Power control | Firmware-held latch on PA5; the button is only an input |
| Charge detect | PA12, input with pull-up, not read by the stock firmware |
| Debug | SWD on PA13/PA14, 3.3 V logic |

### 2.2. External connectors

The mainboard exposes exactly two 4-pin cables. Each carries GND, 12/15 V, and
two signal wires.

| Cable | Signal pins | MCU function | 5 V tolerant | Notes |
|---|---|---|---|---|
| Left | PA2, PA3 | USART2 (PA2 TX, PA3 RX) or ADC | No | 3.3 V logic only; used for CRSF |
| Right | PB10, PB11 | USART3 (PB10 TX, PB11 RX) **or** I2C2 (PB10 SCL, PB11 SDA) | Yes | Shared function; only one at a time |

There is no other UART: UART4/UART5 exist only on F105/F107 parts. The 12/15 V
wire stays live when the board is on and must never touch logic.

### 2.3. Pin constraints that shape the design

- **Two signal pairs, three devices.** Radio, mission computer, and IMU cannot
  all fit on the stock connectors. Stage 1 uses the left cable for CRSF and the
  right cable for the IMU.
- **I2C2 and USART3 share PB10/PB11.** The IMU and the mission-computer link
  cannot coexist on the right cable.
- **I2C1 is not routed.** PB8/PB9 are free on the package but reach no
  connector, so moving the IMU there requires two soldered 30 AWG wires.
- **CAN is not routed.** The F103 has a CAN controller, but its pins are not
  brought out, so stage 1 and stage 2 use UART links.

### 2.4. Stage I/O map

**Stage 1, no soldering:**

| Device | Interface | Pins | Cable |
|---|---|---|---|
| XR1 receiver, CRSF | USART2 | PA3 RX, PA2 TX | left |
| BNO055 | I2C2 | PB10 SCL, PB11 SDA | right |

**Stage 2, mission computer or hip driver added:**

| Device | Interface | Pins | Cable |
|---|---|---|---|
| XR1 receiver, CRSF | USART2 | PA3 RX, PA2 TX | left |
| Hip driver or mission computer | USART3 | PB11 RX, PB10 TX | right |
| BNO055 | I2C1 remapped | PB8 SCL, PB9 SDA | two soldered wires |

Moving the IMU between stages is a pin and remap change only. Keep the bus
behind an interface so the balance code does not depend on which I2C peripheral
is used.

## 3. Motors

| Parameter | Value | Source |
|---|---|---|
| Type | 3-phase brushless hub motor, direct drive | Donor |
| Rated power | about 350 W each | Donor label |
| Hall sensors | 3 per motor, 120° electrical | Donor |
| Wheel diameter | 6.5 in, radius 0.0825 m | Measured |
| Rated speed | about 400 rpm at 36 V, limited in firmware | Bench profile |
| Continuous torque | about 2.5 N·m per wheel at the 10 A bench limit | Estimated, to be measured |
| Peak torque | about 4-5 N·m per wheel only if the current limit is raised to 15 A; not enabled by default | Estimated |
| Speed at 400 rpm | 3.4 m/s no-load | Calculated |

The stock firmware profile limits each motor to 10 A and 400 rpm. Do not raise
these values until MOSFET and pack temperature are measured under load. The
original pack is a 4.4 A·h 2p cell arrangement; it cannot supply the 15 A per
motor that the upstream profile assumes.

## 4. Battery and power

### 4.1. Pack

| Parameter | Value |
|---|---|
| Configuration | 10s2p Li-ion 18650 |
| Nominal voltage | 36 V |
| Full charge | 42.0 V |
| Cutoff | about 30 V (BMS) |
| Capacity | 4.4 A·h, about 158 W·h |
| Protection | Integrated BMS: overvoltage, overcurrent, short circuit, cell balancing |
| Charging | Original charger through the original charge port |

The BMS is not a control layer. The firmware must derate before the BMS trips,
because a BMS cut under load removes torque instantly and the robot falls.

### 4.2. Voltage thresholds

| Level | Pack voltage | Cell voltage | Action |
|---|---:|---:|---|
| Warning | 36.0 V | 3.60 V | Limit acceleration, notify the operator |
| Limiting | 33.0 V | 3.30 V | Limit torque and speed |
| Stop | 30.0 V | 3.00 V | Motor shutdown, controlled stop |

These thresholds are the 10s equivalent of the stock firmware beeps and must be
re-verified on the real pack, because sag under load is large on a 2p pack.

### 4.3. Regenerative braking

A balancing robot regenerates whenever it leans back. On a nearly full pack the
BMS can open the charge path on overvoltage, and the DC link then spikes with
nowhere to put the energy. Start test runs at about 90 % charge, keep the regen
current limited, and never test on a freshly charged pack.

### 4.4. Charging interlock

The charge-detect line on PA12 is initialised by the stock firmware but never
read. The custom firmware treats it as an interlock: while the charger is
plugged in, arming is refused and the motor output stays at zero. The robot
cannot drive away with the cable attached.

### 4.5. Added power hardware

| Item | Requirement | Reason |
|---|---|---|
| Main fuse | Sized to the pack and harness, about 30–40 A | Protect the pack and wiring |
| Kill switch | Latching, in the battery line, reachable in a fall | The only stop that works when the firmware hangs |
| DC-DC 5 V | Input rated at least 60 V, output 5 V / 1 A | Receiver and IMU; the cable rail is 12–15 V and the pack is 42 V |

The power button is not an emergency stop. The firmware holds the latch on PA5;
a hung firmware ignores the button. The kill switch is the real safety path.

## 5. Radio link

| Item | Selection |
|---|---|
| Transmitter | **BetaFPV LiteRadio 3 Pro**, ExpressLRS 2.4 GHz, 8 channels |
| Receiver | **RadioMaster XR1**, ExpressLRS 2.4 GHz, CRSF output, 420 kbaud |
| Link | CRSF over USART2, 3.3 V logic, left cable |
| Arming | Radio only; the mission computer cannot arm |
| Failsafe | Set on the transmitter; loss of frames zeroes the setpoints |

### 5.1. Binding and firmware version

Both ends must run the same ExpressLRS major version. Bind before mounting the
receiver inside the chassis, then verify the channel map and the failsafe with
the motors lifted off the ground.

### 5.2. Channel map

The LiteRadio 3 Pro has two self-centring axes on the right stick and a ratchet
on the left vertical. Tank steering is therefore not usable; use one stick for
throttle and steering.

| Channel | Function | Notes |
|---|---|---|
| CH1 | Steering, `ω_ref` | Right stick horizontal, self-centring |
| CH2 | Throttle, `v_ref` | Right stick vertical, self-centring |
| CH3 | Arm switch | Two-position; up = armed |
| CH4 | Mode switch | Balance / damping / hold |
| CH5 | Rate selector | Optional: low / medium / high speed limit |
| CH6–CH8 | Spare | Telemetry request, future hips |

The exact channel order is a transmitter setting and must be recorded after
binding.

## 6. IMU

| Parameter | Value |
|---|---|
| Part | Bosch BNO055, 9-DOF with onboard sensor fusion |
| Interface | I2C, 100 kHz, address 0x28 or 0x29 |
| Fusion rate | 100 Hz |
| Used outputs | Euler pitch/roll, calibrated gyroscope, linear acceleration |
| Mounting | Rigidly fixed to the chassis, low and centred between the wheels, axes aligned with the robot frame |
| Calibration | Saved and restored at boot; re-run after any mechanical change |

The BNO055 output is 100 Hz, below the 200 Hz balance rate. The balance loop
propagates the gyro between fusion samples rather than waiting for a fresh
angle every period. A raw high-rate IMU (for example ICM-42688-P) remains the
fallback if the propagated estimate proves too noisy.

## 7. Stage 2 hip actuators

| Parameter | Value |
|---|---|
| Quantity | 2 |
| Model | Open; selection after the stage 1 balance is proven |
| Interface preference | Integrated driver with UART or CAN, position and torque feedback |
| Command source | Mainboard or hip driver over USART3, setpoints only |
| Required torque | Sized from the measured centre-of-mass height; the wheels alone give about 5 N·m total |
| Mounting | One actuator per leg at joint A, in the body, not on the unsprung leg |

Constraints:

- The mainboard cannot drive a third and fourth motor: it has exactly two
  three-phase bridges.
- A hip actuator with an integrated driver is strongly preferred; an external
  driver needs its own current loop and thermal design.
- The hip link is a 1-DOF four-bar. Height and fore-aft offset are coupled; the
  controller must respect the coupler curve and the mechanical limits.

## 8. Mission computer (stage 3)

| Parameter | Value |
|---|---|
| Role | Setpoints, telemetry, state estimation, navigation |
| Selection | Open; Raspberry Pi 5 class or equivalent |
| Link to the mainboard | USART3, 115200, framed with CRC |
| Authority | Setpoints only, inside a validity window; never arming, never motor torque |

## 9. Open items

- Measure continuous and peak torque per wheel at 10 A, and MOSFET temperature
  after a two-minute run.
- Verify the pack's real cutoff and BMS behaviour under regen.
- Confirm the BNO055 mounting orientation and calibration stability on the
  running robot.
- Select the stage 2 hip actuator and its driver interface.
- Confirm the ExpressLRS major version on the LiteRadio 3 Pro and the XR1.
- Decide whether the mission computer appears in stage 2 or stage 3.
