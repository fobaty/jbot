# Hardware Bill of Materials

| Field | Value |
|---|---|
| Document | Component and assembly list |
| Code | BOM-001 |
| Version | 0.2 |
| Date | 2026-11-12 |
| Status | Stage 1 donor hardware fixed; stage 2 open |
| Scope | Robot, donor reuse, bench, and integration hardware |

This is the single place to track project hardware. Quantities are per robot
unless a line describes bench equipment. `TBD` means the exact part, rating, or
connector is still open. "Donor" means the part is reused from the hoverboard and
must not be ordered.

## 1. Reused from the donor hoverboard

| ID | Item | Qty | Specification | Interface / notes | Status |
|---|---|---:|---|---|---|
| D-01 | Mainboard | 1 | STM32F103RCT6, 12 MOSFETs, 2 hall inputs ×3, current shunts, buzzer, LED | The only board that drives the motors; power latch on PA5 | In hand |
| D-02 | Hub motor | 2 | Direct drive, about 350 W, 6.5 in wheel, 3 hall sensors | One per wheel; phases and halls wired to D-01 | In hand |
| D-03 | Battery pack | 1 | 10s2p Li-ion, 36 V nominal, 42.0 V full, 4.4 A·h, ~158 W·h, integrated BMS | Original charge port and charger | In hand |
| D-04 | Power button | 1 | Momentary, firmware-held latch | On the mainboard (PA5) | In hand |
| D-05 | Charge port | 1 | BMS charge input, charge-detect line to PA12 | Also used as the charging interlock | In hand |
| D-06 | Wheel and tyre | 2 | 6.5 in, solid or pneumatic as fitted | Tyre condition affects traction and rolling resistance | In hand |
| D-07 | Motor cables and hall harness | 2 sets | Original 3-phase plus 5-wire hall | Inspect for chafing before reuse | In hand |

Not reused and not ordered: both sideboards, deck, covers, and the original
charger cable beyond the pack's own charger.

## 2. Control and radio

| ID | Item | Qty | Selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| C-01 | Radio transmitter | 1 | **BetaFPV LiteRadio 3 Pro, ExpressLRS 2.4 GHz**, 8 channels | Operator input only; not a safety path | In hand |
| C-02 | Radio receiver | 1 | **RadioMaster XR1, ExpressLRS 2.4 GHz**, CRSF output | USART2, left cable, 420 kbaud, 3.3 V | In hand |
| C-03 | Receiver antenna | 1 | Supplied with the XR1 | Keep clear of motor phases and the DC-DC | In hand |
| C-04 | IMU | 1 | **Bosch BNO055**, 9-DOF fusion module | I2C2, right cable, 100 kHz, 100 Hz fusion | Selected |
| C-05 | Mainboard debug probe | 1 | ST-Link V2 or V3 | SWD on PA13/PA14, 3.3 V | TBD |
| C-06 | USB-UART adapter | 1 | 3.3 V logic, CP2102/FT232 class | Debug output and CRSF sniffing | TBD |
| C-07 | DC-DC 5 V | 1 | 36–42 V in, 5 V / 1 A out, input rated ≥ 60 V | Receiver and IMU only; never feed from the 15 V rail directly | TBD |
| C-08 | Mission computer | 1 | Raspberry Pi 5 class or equivalent | Stage 3; USART3 setpoints | Deferred |

## 3. Power and safety

| ID | Item | Qty | Selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| P-01 | Main fuse | 1 | 30–40 A, voltage rated ≥ 60 V DC | In the battery positive line | TBD |
| P-02 | Kill switch | 1 | Latching, ≥ 40 A, reachable in a fall | Battery line; the only stop that works with a hung firmware | TBD |
| P-03 | Battery connector | 1 set | Keyed, ≥ 40 A, reverse-polarity safe | Between the pack and the fuse | TBD |
| P-04 | Precharge | 1 | Resistor and bypass, if inrush is a problem | Protects the mainboard capacitors | Optional |
| P-05 | Branch fuse, logic | 1 | 2 A | DC-DC input | TBD |
| P-06 | Wiring and insulation | 1 set | 14–12 AWG power, 26–30 AWG signal | The 15 V cable pins must be insulated when unused | TBD |
| P-07 | Capacitor bleeder | 1 | 1 kΩ / 2 W across the DC link | Stored-energy safety during service | TBD |

## 4. Sensing and telemetry

| ID | Item | Qty | Selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| S-01 | IMU | 1 | Bosch BNO055 | Same as C-04 | Selected |
| S-02 | High-rate IMU fallback | 1 | ICM-42688-P class | Only if gyro propagation from BNO055 proves insufficient | Fallback |
| S-03 | Pack voltage sense | 1 | Existing mainboard divider | Calibrate `BAT_CALIB_ADC` against a multimeter | In hand |
| S-04 | Pack current sense | 1 | Existing DC-link shunt | Limited accuracy; sufficient for derating | In hand |
| S-05 | Temperature sense | 1 | Existing NTC on the mainboard | Board temperature, not cell temperature | In hand |
| S-06 | Telemetry to handset | 1 | CRSF link quality and pack voltage | Replaces the sideboard battery LEDs | With C-02 |

## 5. Stage 2: hip actuation and mechanism

| ID | Item | Qty | Selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| A-01 | Hip actuator | 2 | TBD; integrated driver with UART or CAN preferred | One per leg at joint A, in the body | Open |
| A-02 | Hip driver, if not integrated | 2 | TBD | Needs its own current loop, thermal design, and position feedback | Open |
| A-03 | Hip power harness | 2 | TBD | Sized to the actuator stall current | Open |
| A-04 | Hip feedback | 2 | Integrated encoder or absolute sensor | Position, speed, current, temperature | Open |
| M-01 | Main chassis | 1 | Custom plate or tube frame carrying the donor mainboard, pack, and hips | Battery and electronics low and centred | TBD |
| M-02 | Four-bar leg links | 2 sets | 1-DOF planar four-bar per side | Link lengths after CAD synthesis | TBD |
| M-03 | Bearings and bushings | 1 set | Hip, rocker, and wheel-fork joints | Backlash and stiffness | TBD |
| M-04 | Pivot shafts and fasteners | 1 set | TBD | Preload and retention | TBD |
| M-05 | Leg springs | 2 | Optional | Only if the passive compliance option is kept | Optional |
| M-06 | Wheel forks | 2 | Rigid C–D–E coupler | Carries the hub motor axle | TBD |
| M-07 | Electronics enclosure | 1 | TBD | Keeps the BNO055 rigid and the radio antenna clear | TBD |
| M-08 | Cable management | 1 set | Sleeves, clips, strain relief | Protect signal wires from motor phases | TBD |
| M-09 | Labels and safety markings | 1 set | TBD | Kill switch, battery, pinch points | TBD |

## 6. Bench and test equipment

| ID | Item | Qty | Selection | Interface / notes | Status |
|---|---|---:|---|---|---|
| T-01 | Current-limited lab PSU | 1 | 0–60 V / 5 A minimum | First power-up without the pack | TBD |
| T-02 | Safety rig | 1 | Frame or tether that holds the body | Required for every balance test | TBD |
| T-03 | Oscilloscope | 1 | 2–4 channel, ≥ 100 MHz | PWM, current shunt, CRSF, ripple | TBD |
| T-04 | Current probe | 1 | DC to ≥ 100 kHz | Phase and DC-link transients | TBD |
| T-05 | Logic analyzer | 1 | ≥ 8 channel | CRSF, I2C, UART timing | TBD |
| T-06 | Multimeter and thermal probe | 1 set | True-RMS, IR or thermocouple | Voltage calibration and thermal runs | TBD |
| T-07 | Battery charger | 1 | Original hoverboard charger | Only charger approved for this pack | In hand |
| T-08 | Fire-safe charging surface | 1 | Non-flammable, ventilated | Li-ion pack handling | TBD |
| T-09 | Servo tester | 1 | 1–2 ms PWM output | Verifies the stage 1 CRSF-to-PWM adapter | TBD |

## 7. Open BOM decisions

- Stage 2 hip actuator model, torque, and interface.
- Mission computer model and whether it appears in stage 2 or stage 3.
- Fuse rating after the first current measurement on the real pack.
- Whether a precharge circuit is needed at 42 V with this mainboard.
- Connector family for the battery, kill switch, and external devices.
- Spare quantities for fuses, hub motors, and hall harnesses.
- Whether the BNO055 module or a bare BNO055 with an external crystal is used.

## 8. Rules

- Donor parts are reused, not duplicated. Do not order a second mainboard or a
  second pack unless the first is destroyed.
- Do not order a long-lead item until its interface and envelope are confirmed.
- A substitution must preserve the interface, torque/speed, thermal, and
  feedback requirements; record it here and in the relevant ADR.
- Keep robot hardware, bench equipment, and optional extensions distinct.
- Update this list whenever `config/robot.toml` or [`docs/hardware.md`](hardware.md)
  changes.
