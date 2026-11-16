# Hardware Baseline

| Field | Value |
|---|---|
| Document | Donor hardware, real-time controller, radio link, IMU, and interfaces |
| Code | HW-001 |
| Version | 0.3 |
| Date | 2026-11-15 |
| Status | Stage 1 baseline fixed; stage 2 actuators open |
| Scope | STM32F103RCT6 hoverboard mainboard, Pico 2 W, hub motors, 10s pack, XR1/LiteRadio, BNO055, hip actuators |

This document is the single description of the physical robot. Every value here
must match `config/robot.toml`, [`docs/bom.md`](bom.md), and the firmware pin maps
in [`docs/pico-bridge.md`](pico-bridge.md) and
[`firmware/README.md`](../firmware/README.md). Items marked **open** may change;
items marked **donor** are fixed by the hoverboard.

> **Change in 0.3 (ADR-011).** The real-time controller is a Raspberry Pi Pico 2 W
> (RP2350). The hoverboard mainboard keeps the power stage and the 16 kHz FOC
> current loop and is driven as a torque actuator node over USART2. The BNO055 and
> the XR1 receiver are wired to the Pico, not to the mainboard. See
> [`docs/architecture.md`](architecture.md) §3.2.

## 1. Where the hardware comes from

The robot is built around a stock hoverboard plus one added controller board. The
mainboard is not replaced and not bypassed: it remains the only board that drives
the wheel motors. The Pico 2 W owns the robot's behaviour and the safety logic.

| Part | Quantity | Specification | Role |
|---|---:|---|---|
| Mainboard (**donor**) | 1 | STM32F103RCT6, Cortex-M3 at 72 MHz, 256 KB flash, 48 KB RAM, 12 MOSFETs in two 3-phase bridges | FOC, current limits, power latch, telemetry |
| Hub motor (**donor**) | 2 | Direct drive, about 350 W, 15 hall states, 6.5 in wheel (radius 0.0825 m) | Drive and speed feedback |
| Battery (**donor**) | 1 | 10s2p Li-ion, 36 V nominal, 42.0 V full, about 30 V cutoff, 4.4 A·h, ~158 W·h, integrated BMS | Energy |
| Power button (**donor**) | 1 | Momentary button, firmware-held latch | Power on/off |
| Charge port (**donor**) | 1 | BMS charge input plus a charge-detect line | Charging, charge interlock |
| Pico 2 W (**added**) | 1 | RP2350, dual Cortex-M33 at 150 MHz with FPU, 520 KB SRAM, 4 MB flash, 2 hardware UARTs, 3 PIO blocks | Balance control, arming, limits, telemetry, blackbox |
| DC-DC 5 V (**added**) | 1 | Input rated at least 60 V, output 5 V / 1.5 A | Pico, XR1 receiver, BNO055 |

**Discarded:** both sideboards, the deck, and the plastic covers. The sideboards
contain their own MCU (STM32F103C8T6 or GD32F130C6), a MPU6050-class IMU, the
battery LEDs, and the optical footpad sensors. They are UART slaves that
transmit on the same two signal wires the Pico now uses, and they do not control
the motors. Keeping them connected would put two transmitters on one line.

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
| Role after ADR-011 | Torque actuator node: receives a torque request, applies it, enforces local limits, times out in 100 ms |

### 2.2. External connectors

The mainboard exposes exactly two 4-pin cables. Each carries GND, 12/15 V, and
two signal wires.

| Cable | Signal pins | MCU function | 5 V tolerant | Notes |
|---|---|---|---|---|
| Left | PA2, PA3 | USART2 (PA2 TX, PA3 RX) | No | 3.3 V logic only; the Pico link |
| Right | PB10, PB11 | USART3 (PB10 TX, PB11 RX) **or** I2C2 (PB10 SCL, PB11 SDA) | Yes | Shared function; only one at a time |

There is no other UART: UART4/UART5 exist only on F105/F107 parts. The 12/15 V
wire stays live when the board is on and must never touch logic.

### 2.3. Pin constraints that shape the design

- **The left cable is the Pico link.** USART2 carries the 8-byte torque command in
  and the 18-byte feedback out. Nothing else shares it.
- **The right cable is free.** Because the radio and the IMU moved to the Pico,
  PB10/PB11 are available for USART3 — the stage 2 hip link — with no soldering
  and no remap.
- **I2C2 stays unused.** The F103 I2C2 shares PB10/PB11 with USART3; since the IMU
  is no longer on the mainboard, that conflict never arises.
- **I2C1 is not routed.** PB8/PB9 are free on the package but reach no connector.
  Nothing needs them now.
- **CAN is not routed.** The F103 has a CAN controller, but its pins are not
  brought out, so stage 1 and stage 2 use UART links.

### 2.4. Stage I/O map

**Stage 1 (and unchanged in stage 2 for the mainboard):**

| Device | Interface | Pins | Cable | Notes |
|---|---|---|---|---|
| Pico 2 W, torque link | Pico UART0 ↔ USART2 | GP0 TX → PA3 RX, GP1 RX ← PA2 TX | left | 115200 8N1, EFeru `TRQ_MODE` frames |
| Hip driver, stage 2 | USART3 | PB11 RX, PB10 TX | right | Free at stage 1; enabled when the hips exist |
| Debug | SWD | PA13, PA14 | — | ST-Link, PlatformIO upload |
| Charger sense | GPIO/ADC | PA12 | — | Read by the patched firmware and forwarded to the Pico, or wired to a Pico GPIO (open item) |

The stock firmware must be rebuilt with `CTRL_MOD_REQ = TRQ_MODE` and the serial
timeout shortened from 160 to 20 ticks (100 ms). No pin remap is needed: the
`VARIANT_USART` build already puts the protocol on USART2.

## 3. Pico 2 W

### 3.1. Capabilities

| Function | Detail |
|---|---|
| MCU | RP2350, dual Cortex-M33 at 150 MHz, single-precision FPU, 520 KB SRAM, 4 MB QSPI flash |
| Timers | Two 64-bit timers, PWM slices, alarm pool — the balance tick runs off hardware |
| UARTs | 2 hardware PL011 UARTs plus PIO-based UARTs |
| I2C | 2 hardware controllers, up to 1 MHz |
| PIO | 3 blocks, 4 state machines each; used here for one extra UART |
| USB | Full-speed device, used as the debug console |
| Watchdog | Hardware watchdog on the RP2350 |
| Wireless | CYW43439 on the 2 W variant; **not initialised** in stage 1 |
| Logic level | 3.3 V; GPIO are not 5 V tolerant |

The wireless subsystem is deliberately left off: it adds power draw, EMI, and a
second firmware stack, and the mission computer already provides connectivity.

### 3.2. Pin map

| GPIO | Function | Direction | Peer | Notes |
|---|---|---|---|---|
| GP0 | UART0 TX | out | STM32 PA3 (USART2 RX) | Torque command frames |
| GP1 | UART0 RX | in | STM32 PA2 (USART2 TX) | Feedback frames |
| GP2 | I2C1 SDA | bidir | BNO055 SDA | 400 kHz, external pull-ups on the module |
| GP3 | I2C1 SCL | out | BNO055 SCL | 400 kHz |
| GP4 | UART1 TX | out | XR1 RX | CRSF 420 kbaud |
| GP5 | UART1 RX | in | XR1 TX | CRSF 420 kbaud |
| GP6 | PIO UART TX | out | RPi GPIO15 (RX) | Mission link, 460800 |
| GP7 | PIO UART RX | in | RPi GPIO14 (TX) | Mission link, 460800 |
| GP10 | Scope/debug marker | out | Logic analyzer | Toggled at the balance step for jitter measurement |
| GP11 | Charge detect (candidate) | in | Mainboard PA12 | Only if the flag is not forwarded over the EFeru feedback frame |
| USB | CDC ACM | bidir | Host | Debug console and flashing |
| SWD | SWDIO/SWCLK | bidir | Probe | `probe-rs`, `defmt` RTT |

Reserved and unassigned: GP8, GP9, GP12-GP22, GP26-GP28 (ADC). Keep GP23-GP25
and the on-board regulator pins free. Do not use GP26-GP29 for the radio or the
IMU harness if a future analog sense is added.

### 3.3. Mounting and wiring rules

- Mount the Pico on a rigid plate, away from the motor phase cables, with the USB
  port reachable for a console without opening the robot.
- Keep the BNO055 wires short and twisted; the I2C bus runs at 400 kHz next to
  switching bridges.
- The Pico, the mainboard, and the mission computer share one ground at the
  battery star point. Never add a second return through USB while the robot is
  powered.
- Route the CRSF and mission-link wires away from the phase cables and add
  ferrites if the error counters move.

## 4. Motors

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

## 5. Battery and power

### 5.1. Pack

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

### 5.2. Voltage thresholds

| Level | Pack voltage | Cell voltage | Action |
|---|---:|---:|---|
| Warning | 36.0 V | 3.60 V | Limit acceleration, notify the operator |
| Limiting | 33.0 V | 3.30 V | Limit torque and speed |
| Stop | 30.0 V | 3.00 V | Motor shutdown, controlled stop |

These thresholds are the 10s equivalent of the stock firmware beeps and must be
re-verified on the real pack, because sag under load is large on a 2p pack. The
Pico evaluates them from the pack voltage reported in the mainboard feedback
frame; the mainboard keeps its own conservative check.

### 5.3. Regenerative braking

A balancing robot regenerates whenever it leans back. On a nearly full pack the
BMS can open the charge path on overvoltage, and the DC link then spikes with
nowhere to put the energy. Start test runs at about 90 % charge, keep the regen
current limited, and never test on a freshly charged pack.

### 5.4. Charging interlock

The charge-detect line on PA12 is initialised by the stock firmware but never
read. The patched firmware forwards it; the Pico treats it as an interlock and
refuses to arm while the charger is plugged in. Two options are open: extend the
EFeru feedback frame with a status byte, or wire PA12 to a spare Pico GPIO
(GP11 is reserved for it). Either way the robot cannot drive away with the cable
attached.

### 5.5. Added power hardware

| Item | Requirement | Reason |
|---|---|---|
| Main fuse | Sized to the pack and harness, about 30–40 A | Protect the pack and wiring |
| Kill switch | Latching, in the battery line, reachable in a fall | The only stop that works when either firmware hangs |
| DC-DC 5 V | Input rated at least 60 V, output 5 V / 1.5 A, with a 2 A fuse and a TVS | Pico (0.6 W peak), XR1 receiver, BNO055; the cable rail is 12–15 V and the pack is 42 V |

Do not power the Pico from the mainboard's internal 5 V rail without measuring
its headroom and inrush. A dedicated buck off the pack is the baseline; tapping
the donor rail is an open alternative.

The power button is not an emergency stop. The firmware holds the latch on PA5;
a hung firmware ignores the button. The kill switch is the real safety path.

## 6. Radio link

| Item | Selection |
|---|---|
| Transmitter | **BetaFPV LiteRadio 3 Pro**, ExpressLRS 2.4 GHz, 8 channels |
| Receiver | **RadioMaster XR1**, ExpressLRS 2.4 GHz, CRSF output, 420 kbaud |
| Link | CRSF over the Pico's UART1 (GP4 TX, GP5 RX), 3.3 V logic |
| Arming | Radio only; neither the mission computer nor any host process can arm |
| Failsafe | Set on the transmitter; loss of frames zeroes the setpoints and keeps balancing |

### 6.1. Binding and firmware version

Both ends must run the same ExpressLRS major version. Bind before mounting the
receiver inside the chassis, then verify the channel map and the failsafe with
the motors lifted off the ground.

### 6.2. Channel map

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

## 7. IMU

| Parameter | Value |
|---|---|
| Part | Bosch BNO055, 9-DOF with onboard sensor fusion |
| Interface | I2C1 on the Pico, 400 kHz, address 0x28 or 0x29 |
| Fusion rate | 100 Hz |
| Used outputs | Euler pitch/roll, calibrated gyroscope, linear acceleration |
| Mounting | Rigidly fixed to the chassis, low and centred between the wheels, axes aligned with the robot frame |
| Calibration | Saved and restored at boot by the Pico; re-run after any mechanical change |

The BNO055 output is 100 Hz, below the 200 Hz balance rate. The Pico propagates
the gyro between fusion samples rather than waiting for a fresh angle every
period. A raw high-rate IMU (for example ICM-42688-P) remains the fallback if the
propagated estimate proves too noisy; the Pico has spare SPI/I2C pins for it.

## 8. Stage 2 hip actuators

| Parameter | Value |
|---|---|
| Quantity | 2 |
| Model | Open; selection after the stage 1 balance is proven |
| Interface preference | Integrated driver with UART or CAN, position and torque feedback |
| Command source | Pico (stage 2 bridge) or the mission computer, via the mainboard's USART3 |
| Required torque | Sized from the measured centre-of-mass height; the wheels alone give about 5 N·m total |
| Mounting | One actuator per leg at joint A, in the body, not on the unsprung leg |

Constraints:

- The mainboard cannot drive a third and fourth motor: it has exactly two
  three-phase bridges.
- A hip actuator with an integrated driver is strongly preferred; an external
  driver needs its own current loop and thermal design.
- The hip link is a 1-DOF four-bar. Height and fore-aft offset are coupled; the
  controller must respect the coupler curve and the mechanical limits.
- USART3 on the right cable is free because the radio and the IMU left the
  mainboard, so the hip link needs no soldering.

## 9. Mission computer (stage 3)

| Parameter | Value |
|---|---|
| Role | Setpoints, telemetry, state estimation, navigation |
| Selection | Open; Raspberry Pi 5 class or equivalent |
| Link | UART to the Pico's PIO UART (GP6/GP7 ↔ GPIO14/GPIO15), 460800, COBS + CRC-16 |
| Authority | Setpoints only, inside a validity window; never arming, never motor torque |

## 10. Open items

- Measure continuous and peak torque per wheel at 10 A, and MOSFET temperature
  after a two-minute run.
- Verify the pack's real cutoff and BMS behaviour under regen.
- Confirm the BNO055 mounting orientation and calibration stability on the
  running robot.
- Select the stage 2 hip actuator and its driver interface.
- Confirm the ExpressLRS major version on the LiteRadio 3 Pro and the XR1.
- Decide whether the mission computer appears in stage 2 or stage 3.
- Confirm the XR1 pad assignment: which pad is TX and which is RX.
- Choose the Pico's 5 V source: dedicated buck or a tap on the mainboard rail,
  and measure the inrush.
- Choose the charge-detect path: an extra status byte in the EFeru feedback frame,
  or a wire from PA12 to GP11.
- Measure the Pico → mainboard actuator delay and jitter before tuning the
  balance gains; the number decides whether the 5 ms quantization is acceptable.
