# Hardware Bill of Materials

| Field | Value |
|---|---|
| Document | Component and assembly list |
| Code | BOM-001 |
| Version | 0.1 |
| Date | 2026-10-22 |
| Status | Provisional |
| Scope | Complete robot, bench, and integration hardware |

This is the single place to track project hardware. The list starts from the
current provisional architecture and must be completed before procurement.
Quantities are per robot unless a line explicitly describes bench equipment.
`TBD` means the exact part, rating, or connector is still open.

## 1. Compute and control

| ID | Item | Qty | Provisional selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| C-01 | Companion computer | 1 | Raspberry Pi 5, 16 GB | Ethernet/Wi-Fi, USB, PCIe; cooling required | Selected |
| C-02 | Companion storage | 1 | NVMe SSD via PCIe HAT, or high-endurance microSD | Boot and logs; NVMe preferred | TBD |
| C-03 | Companion cooling | 1 | Active cooler | Sustained SLAM/AI load | TBD |
| C-04 | Real-time MCU board | 1 | Pico 2 W (RP2350), ESP32-S3, or STM32H723/H743 | ADR-001 decides after benchmark | Candidate |
| C-05 | MCU debug probe | 1 | CMSIS-DAP or vendor probe | Flashing, trace, recovery | TBD |
| C-06 | CAN controller | 1 | Native FDCAN/TWAI, or MCP2518FD/MCP2515 over SPI | Required for Pico 2 W; classic CAN for actuators | TBD |
| C-07 | CAN transceiver | 1+ | TJA1051T/3, 3.3 V classic CAN | 1 Mbit/s actuator bus; not CAN FD qualified | Selected |
| C-08 | CAN termination and protection | 1 set | 2 x 60 ohm split termination, common-mode choke, TVS | Per bus segment | TBD |
| C-09 | RPi-MCU link | 1 | CAN FD or UART | Final transport depends on ADR-002 | TBD |
| C-10 | Connectors and harnesses | 1 set | Locking connectors, strain relief | CAN, UART, power, debug | TBD |

## 2. Actuation

| ID | Item | Qty | Provisional selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| A-01 | Hip actuator | 2 | RobStride RS02 | Classic CAN; approximately 17 N*m peak; thermal margin must be verified | Provisional |
| A-02 | Wheel actuator | 2 | RobStride RS05 | Classic CAN; torque, speed, thermal, and mounting data pending | Provisional |
| A-03 | Actuator power harness | 4 | Vendor cable or equivalent | Current rating, connector keying, bend radius | TBD |
| A-04 | Actuator CAN harness | 1 set | Twisted pair with shield | Linear bus, short stubs, shared termination plan | TBD |
| A-05 | Actuator mounting hardware | 1 set | Vendor flanges or custom adapters | Backlash, stiffness, service access | TBD |
| A-06 | Wheel hubs and wheels | 2 | TBD | Tire diameter, tread, hub interface, inertia | TBD |

## 3. Power and safety

| ID | Item | Qty | Provisional selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| P-01 | Main battery | 1 | Milwaukee M18 XC5.0, 18 V nominal | Runtime target 30-50 min | Selected |
| P-02 | Battery adapter | 1 | M18 adapter with fused leads | Reverse polarity protection, contact retention | TBD |
| P-03 | Main fuse | 1 | 40 A at 18 V, 30 A at 24-36 V | Protect battery and harness | TBD |
| P-04 | Main contactor or relay | 1 | TBD | Controlled by E-stop and supervisor | TBD |
| P-05 | Precharge circuit | 1 | Resistor and bypass | Limit DC-link inrush | TBD |
| P-06 | Reverse polarity protection | 1 | Ideal-diode MOSFET or equivalent | Low loss at full load | TBD |
| P-07 | Input TVS and filtering | 1 set | Transient suppressor, bulk capacitance, ferrite | EMI and load-step margin | TBD |
| P-08 | Hardware E-stop | 1 | Two-channel normally closed loop | Removes actuator power, keeps logic alive | TBD |
| P-09 | DC-DC 12 V | 1 | 12 V / 5 A, efficiency >= 90% | RPi peripherals, fans, lidar | TBD |
| P-10 | DC-DC 5 V | 1 | 5 V / 5-6 A, soft-start, OVP | RPi 5 and USB peripherals | TBD |
| P-11 | DC-DC 3.3 V | 1 | 3.3 V / 1 A | MCU, IMU, RC receiver, logic | TBD |
| P-12 | Power monitor | 1 | INA226-class shunt monitor | Bus voltage, current, SoC input | TBD |
| P-13 | Power distribution board | 1 | Custom PCB or fused busbar | Branch fuses, connectors, test points | TBD |
| P-14 | Branch fuses | 1 set | Per actuator and DC-DC | Coordinated with main fuse | TBD |
| P-15 | Emergency stop button and cable | 1 | Latching, clearly marked | Reachable during bench and field tests | TBD |
| P-16 | Braking or bleeder element | 1 | TBD | Only if regeneration or stored energy requires it | TBD |

## 4. Sensing and teleoperation

| ID | Item | Qty | Provisional selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| S-01 | IMU | 1 | Bosch BNO055 | I2C or UART; 100 Hz fusion plus gyro propagation | Selected |
| S-02 | High-rate IMU fallback | 1 | ICM-42688-P or equivalent | SPI; only if BNO055 latency is insufficient | Fallback |
| S-03 | Lidar | 1 | 2D or 3D lidar, model TBD | Mounting height, field of view, power, interface | TBD |
| S-04 | Camera | 1 | RGB-D or USB camera, optional | USB 3.0/CSI-2; power and heat budget | Optional |
| S-05 | RC transmitter | 1 | LiteRadio 3 Pro, ExpressLRS 2.4 GHz | Operator input only, not a safety path | Selected |
| S-06 | RC receiver | 1 | ExpressLRS 2.4 GHz receiver with CRSF output | UART to MCU; binding and failsafe validation required | Selected |
| S-07 | RC antennas | 2 | Matched 2.4 GHz antennas | Diversity and mounting orientation | TBD |
| S-08 | Wheel position feedback | 2 | Integrated in RS05 | Resolution and sign convention to be verified | Provisional |
| S-09 | Hip position feedback | 2 | Integrated in RS02 | Resolution and sign convention to be verified | Provisional |

## 5. Chassis and mechanism

| ID | Item | Qty | Provisional selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| M-01 | Main chassis | 1 | TBD | Carries battery, electronics, hips, and legs | TBD |
| M-02 | Four-bar leg links | 4 sets | TBD | One 1-DOF four-bar per side; link lengths pending | TBD |
| M-03 | Bearings and bushings | 1 set | TBD | Hip, rocker, and wheel-fork joints | TBD |
| M-04 | Pivot shafts and fasteners | 1 set | TBD | Stiffness, preload, retaining method | TBD |
| M-05 | Leg springs | 2 | Optional | Rate and travel only if the spring option is kept | Optional |
| M-06 | Wheel forks | 2 | TBD | Rigid C-D-E coupler | TBD |
| M-07 | Electronics enclosure | 1 | TBD | IP rating, cooling, service access | TBD |
| M-08 | Cable management | 1 set | Sleeves, clips, strain relief | Protect CAN and power runs | TBD |
| M-09 | Labels and safety markings | 1 set | TBD | E-stop, battery, pinch points | TBD |

## 6. Bench and test equipment

| ID | Item | Qty | Provisional selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| T-01 | Current-limited lab PSU | 1 | 24 V / 20 A class | First power-up without battery | TBD |
| T-02 | Safety test frame | 1 | Welded or extruded frame with body fixation | Stops and tether for first balance tests | TBD |
| T-03 | CAN analyzer | 1 | USB-CAN or vendor tool | Bus capture, injection, timing checks | TBD |
| T-04 | Oscilloscope | 1 | 2-4 channel, >= 100 MHz | Ripple, E-stop, PWM, CAN integrity | TBD |
| T-05 | Current probe | 1 | DC to >= 100 kHz | Actuator and DC-link transients | TBD |
| T-06 | Logic analyzer | 1 | >= 8 channel | SPI, UART, CRSF, timing | TBD |
| T-07 | Multimeter and thermal probe | 1 set | True-RMS, thermocouples or IR | Bring-up and thermal runs | TBD |
| T-08 | Battery charger and storage | 1 set | M18-compatible | Safe charge and storage | TBD |

## 7. Open BOM decisions

- Final MCU board and debug probe after ADR-001.
- CAN FD versus classic CAN for the RPi-MCU backbone.
- Exact RS02 and RS05 variants, gear ratios, and cable lengths.
- Battery voltage: 18 V baseline versus 24-36 V preferred.
- Lidar and camera models, mounting, and power budget.
- Frame material, link geometry, bearings, and wheel size.
- Whether an external ADC is required for the selected MCU.
- Final connector family and service-loop requirements.
- Spare quantities for actuators, fuses, and harnesses.

## 8. Rules

- Every line must have an owner before ordering.
- Do not order a long-lead item until its interface and envelope are confirmed.
- A substitution must preserve CAN, torque/speed, thermal, mounting, and feedback
  requirements; record it here and in the relevant ADR.
- Keep robot hardware, bench equipment, and optional extensions visually distinct.
- Update this list whenever `config/robot.toml` or `docs/hardware.md` changes.
