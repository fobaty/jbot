# Hoverboard firmware

| Field | Value |
|---|---|
| Document | Hoverboard testbed firmware |
| Code | FW-001 |
| Version | 0.1 |
| Date | 2026-11-12 |
| Status | Stage 1 built, not yet flashed |
| Scope | STM32F103RCT6 hoverboard mainboard, bench bring-up |

This directory holds the firmware used to turn a stock hoverboard mainboard into
the balancing testbed. It is deliberately separate from the `jbot` Cargo
workspace: the hoverboard firmware is a self-contained embedded project with its
own toolchain, and it must not affect `cargo test --workspace`.

## 1. Layout

| Path | Content |
|---|---|
| `eferu/` | Upstream clone of `EFeru/hoverboard-firmware-hack-FOC`, pinned commit `4f141cbc97b297e194fd58e56444bd0322f902ac` (2026-07-04). Git-ignored; recreate with the command in section 2. |
| `eferu-jbot.patch` | Local configuration delta applied on top of the pinned commit. |
| `hoverboard-rs/` | Planned Rust firmware (section 5). Not created yet. |

## 1.1. Donor hardware

Reused from the hoverboard:

| Part | Role | Notes |
|---|---|---|
| Mainboard, STM32F103RCT6, 12 MOSFETs | motor controller, power stage, sensors | the only programmable board |
| Two hub motors with hall sensors | drive and speed feedback | about 350 W each, direct drive |
| 10s2p 4.4 Ah pack with BMS | energy | 42 V full, about 30 V cutoff |
| Power button and latch | on/off | PA5 latch, short press powers off, long press calibrates |
| Charge port | charging | BMS charge input plus a charge-detect line to PA12 |

Not reused: both sideboards, the deck, the plastic covers.

Added to the robot: XR1 receiver, BNO055, a 36 V to 5 V converter rated for at
least 60 V input, a main fuse, and a hardware kill switch in the battery line.

Two consequences of this list:

- **Charging interlock.** The charge-detect input is initialised by EFeru but
  never read. The Rust firmware uses it to inhibit the motors whenever the
  charger is plugged in, so the robot cannot be armed while charging.
- **Regenerative braking.** A balancing robot regenerates every time it leans
  back. With a full pack, the BMS can cut the charge path on overvoltage and the
  DC link then spikes. Start runs at about 90 % charge and keep the regen
  current limited until the behaviour is measured.
- **The power button is not an E-stop.** The button is only read by the
  firmware, and the firmware holds the power latch on PA5. If the firmware
  hangs, the button no longer cuts power. It is also small and not reachable
  during a fall. The kill switch in the battery line is the real safety path.

## 2. Stage 1: EFeru baseline

The purpose of stage 1 is to prove the whole chain before any custom code:
toolchain, SWD access, option bytes, motor phases, hall sensors, current
sensors, battery measurement, and the radio link.

### 2.1. Recreate the upstream tree

```sh
cd firmware
mkdir -p eferu && cd eferu
git init -q
git remote add origin https://github.com/EFeru/hoverboard-firmware-hack-FOC.git
git fetch -q --depth 1 origin 4f141cbc97b297e194fd58e56444bd0322f902ac
git checkout -q FETCH_HEAD
git apply ../eferu-jbot.patch
```

### 2.2. Toolchain

PlatformIO is installed with pipx, so it stays isolated from the system Python:

```sh
pipx install platformio
```

The `ststm32` platform and the `arm-none-eabi-gcc` toolchain are downloaded on
the first build.

### 2.3. Build

```sh
cd firmware/eferu
pio run -e VARIANT_PWM
```

Artifacts:

- `.pio/build/VARIANT_PWM/firmware.elf`
- `.pio/build/VARIANT_PWM/firmware.bin`

Expected size for the current profile: about 30 KB flash, 5% RAM.

### 2.4. Flash

The board must be powered from its own battery while flashing. The MCU has no
power when the hoverboard is switched off, so connect the battery, switch the
board on, and only then attach the probe.

```sh
pio run -e VARIANT_PWM -t upload
```

Wiring: SWDIO = PA13, SWCLK = PA14, GND, and optionally 3.3 V sense. Do not
feed 3.3 V from the probe into a board that is already powered.

### 2.5. Recovering a read-protected chip

Hoverboard mainboards are normally shipped with RDP level 1. The first attach
then fails with a protection error instead of flashing. Unlocking performs a
mass erase and **destroys the stock firmware permanently**; this is accepted,
there is no stock binary to go back to.

OpenOCD ships as a PlatformIO tool; it is already installed on this machine
(`pio pkg install --global --tool platformio/tool-openocd` reinstalls it):

```sh
~/.platformio/packages/tool-openocd/bin/openocd \
  -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "init; reset halt; stm32f1x unlock 0; reset halt; shutdown"
```

Equivalent with stlink-tools (`brew install stlink`):

```sh
st-flash --connect-under-reset erase
```

After the erase, repeat the upload. If OpenOCD cannot halt the core at all,
retry with `-c "reset_config srst_only srst_nogate"` or hold the reset pad while
connecting.

### 2.6. Bench wiring for the link test

Wheels off the ground for every test until the direction and the limits are
verified. Disconnect both sideboards first (section 5.5); the right cable is
where the PWM input goes and the sideboard would otherwise drive the same wire.

| Signal | Mainboard pin | Cable | Notes |
|---|---|---|---|
| PWM channel 1 (steering) | PB10 | right | from the CRSF/ELRS to PWM adapter |
| PWM channel 2 (throttle) | PB11 | right | from the adapter |
| GND | GND | right | common ground with the adapter |
| Debug UART TX | PA2 | left | mainboard TX to a 3.3 V USB-UART RX |
| GND | GND | left | |

The two signal wires are the ones that are neither 15 V nor GND; identify them
with a multimeter before connecting anything. The red wire carries about 15 V
when the board is on and must never touch logic.

Do not power the radio adapter or the receiver from the 15 V rail. Use a
separate DC-DC rated for at least 60 V input.

Debug UART: 115200 8N1.

### 2.7. Expected behaviour

1. Power on: startup melody, then the board waits for the inputs to be near
   neutral.
2. With the radio on and both sticks centred, the board beeps twice and enables
   the motors.
3. Every 125 ms the debug UART prints:

   ```
   in1:%i in2:%i cmdL:%i cmdR:%i BatADC:%i BatV:%i TempADC:%i Temp:%i
   ```

4. `in1`/`in2` must move between about -1000 and +1000 with the sticks and
   return to about 0 when centred.
5. `cmdL`/`cmdR` must follow: right stick vertical moves both wheels the same
   way, right stick horizontal moves them in opposite directions.
6. `BatV` is the calibrated pack voltage multiplied by 100. A full 10s pack
   reads about 4200. If it does not match a multimeter reading, recalibrate
   `BAT_CALIB_REAL_VOLTAGE` / `BAT_CALIB_ADC` in `Inc/config.h`.

If every input reads zero and the motors never enable, the PWM signal is not
arriving: check the adapter output with a scope or a servo tester, and confirm
that the wires go to PB10/PB11 and not to the 15 V pin.

If the battery voltage reads zero or the buzzer and LED behave incorrectly, the
board may be the alternate layout: set `BOARD_VARIANT 1` and rebuild.

### 2.8. Profile applied by `eferu-jbot.patch`

| Setting | Upstream | This profile | Reason |
|---|---|---|---|
| `I_MOT_MAX` | 15 A | 10 A | Original 10s2p 4.4 Ah pack; raise only after thermal measurement |
| `I_DC_MAX` | 17 A | 12 A | Keep `I_DC_MAX = I_MOT_MAX + 2 A` |
| `N_MOT_MAX` | 1000 rpm | 400 rpm | About 3.5 m/s with 6.5 inch wheels |
| `DEBUG_SERIAL_USART2` | off | on | Left cable free in `VARIANT_PWM`; right cable is the PWM input |

`VARIANT_PWM` is selected through the PlatformIO environment, not through
`config.h`.

## 3. Known constraints of the stock EFeru behaviour

- No balancing controller. `VARIANT_HOVERBOARD` in `config.h` is marked
  `Balancing controller: [TODO]`, so balancing has to be written.
- No CRSF input. Only PWM, PPM, ADC, iBUS and a custom UART protocol exist.
- The right sensor cable is shared between USART3 and I2C2 (PB10/PB11), so the
  PWM input, the CRSF link and an I2C IMU cannot share it.

## 4. Stage 1 exit criteria

Stage 1 is done when all of the following hold on the bench:

- [ ] Firmware uploads and boots from the battery.
- [ ] Debug UART prints the expected line.
- [ ] Both sticks map to `in1`/`in2` and to `cmdL`/`cmdR` with the correct sign.
- [ ] Both wheels spin in the correct direction with the wheels lifted.
- [ ] `BatV` matches a multimeter within 0.2 V.
- [ ] No MOSFET or motor overheating after two minutes at low load.

## 5. Stage 2: Rust firmware

The Rust firmware reuses the proven parts of EFeru and reimplements the rest.
The split is intentional: the field-oriented control core is generated
fixed-point C from Simulink and is not worth porting, while everything that
makes this robot different is new code.

### 5.1. Planned split

| Layer | Language | Rationale |
|---|---|---|
| FOC core, SVPWM, current PI, fault codes | generated C via FFI | Proven at 16 kHz; a rewrite would have to be re-validated from scratch |
| Clocks, timers, ADC/DMA, hall capture | Rust | Board bring-up, no algorithmic risk |
| BNO055 driver, complementary filter | Rust | New functionality |
| CRSF parser, channel mapping, failsafe | Rust | New functionality |
| Balance, speed and yaw controllers | Rust | New functionality; unit-tested on the host |
| Safety state machine, limits, telemetry | Rust | New functionality |

The FFI boundary is one function, `BLDC_controller_step`, plus four plain
structs. Everything above it is Rust, and the C core can later be replaced
behind the same interface if a pure-Rust FOC is ever wanted.

### 5.2. Stack

- `thumbv7m-none-eabi`, `stm32f1xx-hal` 0.11, RTIC 2.
- RTIC is chosen over an async executor because the 16 kHz FOC interrupt and
  the 200 Hz balance task need static priorities and bounded latency, and
  because it keeps the interrupt structure identical to EFeru.
- The C core is compiled through the `cc` crate with the `arm-none-eabi-gcc`
  that PlatformIO already installed.

### 5.3. Milestones

1. **Blink and clock tree.** Rust binary that boots, sets 72 MHz, and blinks the
   LED. Proves the linker script, vector table, and flashing path.
2. **UART + CRSF.** Replaces the PWM adapter; prints channels and link quality.
3. **I2C + BNO055.** Reads calibrated accel and gyro at 100 Hz.
4. **FOC core via FFI.** Wheel spins under Rust control, current limits active.
5. **Balance loop.** PD on pitch with the robot held by a safety rig.
6. **Speed and yaw loops.** Drive, reverse, steer, spin in place.
7. **Safety and telemetry.** Tilt cutoff, link-loss policy, watchdog, battery
   telemetry to the handset.

### 5.4. Target I/O map

The mainboard exposes exactly two 4-pin cables. Each carries GND, 12/15 V and
two signal wires, and the right cable's signal pins are shared between USART3
and I2C2. Two signals per cable means at most two devices fit on the stock
connectors, which is enough for stage 2 and not enough for stage 3.

**Stage 2, no soldering:**

| Device | Interface | Pins | Cable |
|---|---|---|---|
| XR1 receiver, CRSF | USART2 | PA3 RX, PA2 TX | left |
| BNO055 | I2C2 | PB10 SCL, PB11 SDA | right |

**Stage 3, mission computer added:**

| Device | Interface | Pins | Cable |
|---|---|---|---|
| XR1 receiver, CRSF | USART2 | PA3 RX, PA2 TX | left |
| Robot MCU | USART3 | PB11 RX, PB10 TX | right |
| BNO055 | I2C1 remapped | PB8 SCL, PB9 SDA | two wires to the MCU |

I2C1 is not routed to any connector, so stage 3 needs two 30 AWG wires soldered
to the TQFP64 package. Check for a PB8/PB9 test pad first. Moving the IMU
between stages is a pin and remap change only; keep the bus behind an interface
so the balance code does not care.

### 5.5. Sideboards

The two sideboards are UART slaves with their own MCU, an IMU (MPU6050,
MPU6052C or ICM-20689 depending on the revision), the battery LEDs and the
optical footpad sensors. They do not control the motors.

They must be disconnected: they transmit their own protocol on the same wires
that now carry the receiver and the IMU, and two transmitters on one line is a
contention, not a bus.

Disconnecting them is safe on this build:

- The power button and the power latch are on the mainboard (PA5 on board
  variant 0).
- Motor phases and hall sensors are wired directly to the mainboard.
- The status LED is on the mainboard.
- The 15 V pins of both cables stay live; insulate them.

The only loss is the sideboard battery LEDs, which the CRSF telemetry replaces.

### 5.6. Command arbitration

Both links are inputs to one setpoint. Neither link ever drives the motors
directly.

| Condition | Authority |
|---|---|
| RC disarmed | motors off, no balancing |
| RC stick deflection above 15 % | RC, held for 2 s after the last deflection |
| Valid MCU frame received within 300 ms | MCU |
| Otherwise, RC link up | RC |
| Neither link | zero setpoints, keep balancing |

Rules:

- Only the RC can arm. The MCU cannot enable the motors.
- The balance loop depends on neither link. Losing both means standing still,
  not falling.
- Tilt cutoff overrides every source.
- MCU frames carry a CRC and a sequence counter; a stale frame is rejected the
  same way as a corrupt one.

### 5.7. Repository rules

The crate is its own Cargo workspace so that the root workspace keeps building
for the host. The root `.gitignore` ignores every `.cargo/config.toml`, so the
embedded target configuration needs an explicit exception.
