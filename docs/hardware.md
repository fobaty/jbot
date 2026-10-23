# Provisional Hardware Selection

| Field | Value |
|---|---|
| Document | Provisional electronics and actuator selection |
| Code | HW-001 |
| Version | 0.1 |
| Date | 2026-10-22 |
| Status | Provisional, Phase 1/2 |
| Scope | MCU candidates, CAN, RC link, IMU, hip and wheel actuators |

This document records the current hardware direction. Items marked **provisional**
may be replaced when a cheaper or better-supported part meets the same electrical,
mechanical, thermal, and protocol envelope. Changes must be reflected in the BOM,
`config/robot.toml`, and the relevant ADR before they reach firmware.

## 1. Real-time controller candidates

| Candidate | Status | Notes |
|---|---|---|
| ESP32-S3 | Candidate | Dual Xtensa LX7 at 240 MHz, mature `esp-hal`, native TWAI classic CAN only, no CAN FD. |
| Raspberry Pi Pico 2 W (RP2350) | Third candidate | Dual Cortex-M33 at 150 MHz, 520 KB SRAM, PIO, `rp235x-hal`/`embassy-rp`. The W variant adds CYW43 Wi-Fi/BLE; the non-W Pico 2 is preferred for production. |
| STM32H723/H743 | Production candidate | Native FDCAN, many ADC/timers, mature `embassy-stm32`. |

ADR-001 must choose the controller from measured 1 kHz jitter, end-to-end latency,
CAN behavior, ADC availability, and toolchain maturity. Clock frequency alone is
not a selection criterion.

### 1.1. Pico 2 W constraints

- RP2350 has no native CAN controller. A Pico 2 W needs an external controller:
  MCP2518FD over SPI for CAN FD, or MCP2515 for classic CAN.
- Pico 2 W exposes only three ADC inputs (GPIO26-28). Four motor currents, bus
  voltage, and temperatures require an external ADC or multiplexer.
- The CYW43 wireless module is not required for the low-level control loop. Its
  RF activity, firmware blob, and shared SPI bus are accepted only if the W
  variant is kept; otherwise use Pico 2 non-W.

## 2. CAN physical layer

- **Selected transceiver:** TJA1051T/3, 3.3 V high-speed CAN transceiver.
- Intended bus: classic CAN 1 Mbit/s for the RobStride actuators.
- The transceiver does not provide a CAN controller. ESP32-S3 uses TWAI, STM32
  uses FDCAN in classic mode, and Pico 2 W needs an external MCP2515/MCP2518FD.
- CAN FD data-phase rates require an FD-capable transceiver and controller; the
  TJA1051T/3 selection is not a CAN FD qualification.

## 3. RC link

- **Receiver:** ExpressLRS 2.4 GHz receiver with CRSF serial output.
- **Transmitter:** LiteRadio 3 Pro (ELRS 2.4 GHz variant); the receiver must be
  bound and verified against this transmitter.
- The receiver connects to the MCU over UART/CRSF. Channel mapping, arming,
  neutral failsafe, and link-loss behavior must be validated.
- The RC link is a teleoperation input, not a safety path. The hardware E-stop
  remains independent and authoritative. Link-loss policy is subject to the
  Safety-Reviewer decision for the `DAMPING` transition.

## 4. IMU

- **Selected:** Bosch BNO055 with onboard 9-DOF sensor fusion, I2C or UART.
- The fused orientation output is limited to 100 Hz. A 1 kHz balance loop must
  not depend on a fresh fused sample every tick; propagate the gyro between
  samples or retain a high-rate IMU.
- The ICM-42688-P remains the fallback if raw high-rate measurements or lower
  latency are required.
- Calibration data must be persisted and restored; axis remapping and mounting
  orientation must be verified on the robot.

## 5. Actuators (provisional)

| Joint | Quantity | Selection | Notes |
|---|---:|---|---|
| Hip | 2 | RobStride RS02 | Approximately 17 N*m peak. Preliminary sizing flags RS02 as marginal for a 15 kg platform. Vendor CAN protocol, gear ratio, continuous torque, thermal limits, and encoder scaling must be confirmed. |
| Wheel drive | 2 | RobStride RS05 | Used as the wheel actuator. Torque/speed/thermal envelope and mounting interface are to be confirmed from the vendor datasheet. |

The actuator choice is provisional. A cheaper equivalent is acceptable only if
it preserves the CAN interface, torque/speed margin for the 10 N*s impulse case,
thermal derating, mounting envelope, and feedback quality. Record the substitution
in the BOM and ADR; do not silently change the control limits.

## 6. Open items

- ADR-001: final MCU after the three-way benchmark.
- RS02 continuous torque and thermal margin, especially for a 15 kg platform.
- RS02/RS05 vendor datasheets: continuous torque, peak duration, current, speed,
  encoder resolution, CAN IDs, and fault reporting.
- CAN FD vs classic CAN decision for the RPi-MCU backbone.
- BNO055 mounting, calibration, and whether a high-rate IMU is required.
- ExpressLRS binding, channel map, and failsafe policy approval.
