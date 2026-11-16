# Robot firmware

| Field | Value |
|---|---|
| Document | Two-board firmware: Pico 2 W real-time controller, F103 torque node |
| Code | FW-001 |
| Version | 0.2 |

| Status | Mainboard bring-up step 1 flashed; USART bench link verified; Pico firmware planned |
| Scope | STM32F103RCT6 hoverboard mainboard (EFeru C) and Raspberry Pi Pico 2 W (Rust) |

This directory holds all robot firmware: the upstream EFeru C on the hoverboard
mainboard and the Rust balance controller on the Pico 2 W. Both are deliberately
separate from the `jbot` Cargo workspace, each with its own toolchain, so they
never affect `cargo test --workspace`.

> **Change in 0.2 (ADR-011).** The balance loop moved off the mainboard to the
> Pico 2 W. The mainboard is now a torque actuator node: it keeps the 16 kHz FOC
> current loop, the current limits, the power latch, and an independent 100 ms
> serial timeout. The BNO055 and the XR1 receiver are wired to the Pico. See
> [`docs/pico-bridge.md`](../docs/pico-bridge.md).

## 1. Layout

| Path | Content |
|---|---|
| `stm32f103/eferu/` | Upstream clone of `EFeru/hoverboard-firmware-hack-FOC`, pinned commit `4f141cbc97b297e194fd58e56444bd0322f902ac` (2026-07-04). Git-ignored; recreate with the command in section 2. |
| `stm32f103/eferu-jbot.patch` | Local configuration delta applied on top of the pinned commit (limits, variant, debug output). |
| `stm32f103/eferu-bench.patch` | Bench-only delta: fixes the `unlock` target for the ST-Link clone. |
| `stm32f103/eferu-balance.patch` | Production torque-node delta: `CTRL_MOD_REQ = TRQ_MODE`, `SERIAL_TIMEOUT` 160 → 20. Not created yet. |
| `pico2w/` | Rust firmware for the Pico 2 W (section 5). Separate Cargo workspace. |

## 1.1. Hardware in scope

Reused from the hoverboard:

| Part | Role | Notes |
|---|---|---|
| Mainboard, STM32F103RCT6, 12 MOSFETs | power stage, FOC, sensors, power latch | torque actuator node |
| Two hub motors with hall sensors | drive and speed feedback | about 350 W each, direct drive |
| 10s2p 4.4 Ah pack with BMS | energy | 42 V full, about 30 V cutoff |
| Power button and latch | on/off | PA5 latch, short press powers off, long press calibrates |
| Charge port | charging | BMS charge input plus a charge-detect line to PA12 |

Not reused: both sideboards, the deck, the plastic covers.

Added to the robot: **Pico 2 W (RP2350)**, XR1 receiver, BNO055, a 36 V to 5 V
converter rated for at least 60 V input and 1.5 A, a main fuse, and a hardware
kill switch in the battery line.

Two consequences of this list:

- **Charging interlock.** The charge-detect input is initialised by EFeru but
  never read. The patch forwards it and the Pico treats it as an interlock: the
  motors stay off whenever the charger is plugged in, so the robot cannot be
  armed while charging.
- **Regenerative braking.** A balancing robot regenerates every time it leans
  back. With a full pack, the BMS can cut the charge path on overvoltage and the
  DC link then spikes. Start runs at about 90 % charge and keep the regen
  current limited until the behaviour is measured.
- **The power button is not an E-stop.** The button is only read by the
  firmware, and the firmware holds the power latch on PA5. If the firmware
  hangs, the button no longer cuts power. It is also small and not reachable
  during a fall. The kill switch in the battery line is the real safety path.
- **Two firmwares, two timeouts.** The Pico can hang; the mainboard cannot
  depend on it. The EFeru 100 ms serial timeout zeroes torque on its own, and the
  Pico's hardware watchdog resets the balance loop if it stalls.

## 2. Bring-up step 1: EFeru baseline

The purpose of this step is to prove the whole chain before any custom code:
toolchain, SWD access, option bytes, motor phases, hall sensors, current
sensors, battery measurement, and the radio link.

### 2.1. Recreate the upstream tree

```sh
cd firmware
mkdir -p stm32f103 && cd stm32f103
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

This is a pre-Pico bench test of the `VARIANT_PWM` build. The production wiring
is different: the left cable goes to the Pico and the right cable is free. Do not
copy this table into the robot harness.

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
arriving: check the adapter output with a scope, and confirm that the wires go to
PB10/PB11 and not to the 15 V pin.

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

### 2.9. Bench link-test build (`VARIANT_USART`)

Before the radio link exists, the board is driven over its own cable from a host
console through a 3.3 V USB-UART adapter, using the same wire protocol the Pico
bridge later speaks on USART2. This is the build currently flashed on the bench
board.

| Item | Value |
|---|---|
| Variant | `VARIANT_USART` (`-D VARIANT_USART`) |
| Source delta | `eferu-bench.patch`: unlock target only, `config.h` is upstream stock |
| Input | USART2, PA3 RX / PA2 TX, 115200 8N1, left cable |
| Limits | stock: `I_MOT_MAX` 15 A, `N_MOT_MAX` 1000 rpm |
| Control | `VLT_MODE`, `FOC_CTRL` |
| Silent-input timeout | `SERIAL_TIMEOUT` 160 x 5 ms, about 0.8 s |
| Binary | 32 312 bytes, md5 `64ab7b58ceea8ba4a7c1f5ea3d6914d0` |

```sh
cd firmware/stm32f103
git apply ../eferu-bench.patch       # once, on top of the pinned upstream commit
pio run -e VARIANT_USART             # .pio/build/VARIANT_USART/firmware.bin
pio run -e VARIANT_USART -t upload
```

The stock `unlock` target fails on the ST-Link clone with `init mode failed`; the
patch switches it to `interface/stlink.cfg`, adds `shutdown` and reminds you to
power-cycle. On a protected chip also run OpenOCD with `-c "adapter speed 100"`.

Command frames are 8 bytes (`<HhhH`: `0xABCD`, steer, speed, checksum), sent at
200-400 Hz; feedback frames are 18 bytes (`<HhhhhhhHH`) at 100 Hz. There is no
sequence number, so leave a small inter-frame gap: `usart2_rx_check` accepts a
frame only when exactly 8 bytes arrived since the last idle line. The host side -
the Pico bridge, the Mac console and the wire protocol - lives in
[`tools/hover/`](../tools/hover/README.md).

`VLT_MODE` with the stock 1000 rpm scale means commands below about 30 units can
stall on cogging. The balancing controller uses `TRQ_MODE` instead (section 2.10),
where the command is a torque request and cogging does not gate the low end.

### 2.10. Torque-node build (`eferu-balance.patch`)

The production stage 1 build. Same `VARIANT_USART`, two configuration changes:

| Setting | Upstream | This profile | Reason |
|---|---|---|---|
| `CTRL_MOD_REQ` | `VLT_MODE` | `TRQ_MODE` | The Pico sends a torque request; the F103 runs the current loop |
| `SERIAL_TIMEOUT` | 160 (800 ms) | 20 (100 ms) | Bound the age of a stale torque command |
| `I_MOT_MAX` | 15 A | 10 A | Original 10s2p pack; raise only after thermal measurement |
| `I_DC_MAX` | 17 A | 12 A | Keep `I_DC_MAX = I_MOT_MAX + 2 A` |
| `N_MOT_MAX` | 1000 rpm | 400 rpm | About 3.5 m/s with 6.5 in wheels |
| `ENABLE_ODOMETRY` | off | off (open) | Extends feedback to 22 bytes when enabled |

`TRQ_MODE` exists upstream and is valid only with `CTRL_FOC`. The command scale is
±1000 = ±`I_MOT_MAX`; there is no ramp, no interlock, and no arming in the F103 —
that is all on the Pico.

When the serial timeout expires, EFeru falls back to `OPEN_MODE`, i.e. freewheel.
For a balancing robot that means a fall, and it is accepted: a stale torque
command is more dangerous than losing balance under operator control. Because of
that, do **not** enable `ELECTRIC_BRAKE_ENABLE` for balance.

```sh
cd firmware/stm32f103
git apply ../eferu-balance.patch
pio run -e VARIANT_USART
pio run -e VARIANT_USART -t upload
```

## 3. Known constraints of the stock EFeru behaviour

- No balancing controller. `VARIANT_HOVERBOARD` in `config.h` is marked
  `Balancing controller: [TODO]`; the balance loop lives on the Pico instead.
- No CRSF input. Only PWM, PPM, ADC, iBUS and a custom UART protocol exist; the
  XR1 is therefore wired to the Pico, not to the mainboard.
- The right sensor cable is shared between USART3 and I2C2 (PB10/PB11). Because
  the IMU moved to the Pico, USART3 is free for the stage 2 hip link with no
  soldering.
- The command path is quantized to the 5 ms main loop, whatever rate the Pico
  sends at. Budget that delay into the controller.
- The F103 has no FPU. Do not move attitude estimation or the balance loop back
  onto it.

## 4. Bring-up step 1 exit criteria

This step is done when all of the following hold on the bench:

- [ ] Firmware uploads and boots from the battery.
- [ ] Debug UART prints the expected line.
- [ ] Both sticks map to `in1`/`in2` and to `cmdL`/`cmdR` with the correct sign.
- [ ] Both wheels spin in the correct direction with the wheels lifted.
- [ ] `BatV` matches a multimeter within 0.2 V.
- [ ] No MOSFET or motor overheating after two minutes at low load.

## 5. Bring-up step 2: Pico 2 W firmware

The balance controller is Rust on the Pico 2 W, in `firmware/pico2w/`. The
mainboard is not rewritten: it keeps the pinned EFeru C and its validated 16 kHz
FOC core.

### 5.1. Split of responsibilities

| Layer | Where | Language | Rationale |
|---|---|---|---|
| FOC core, SVPWM, current PI, hall capture, power latch | STM32F103 | generated C (EFeru) | Proven at 16 kHz; a rewrite would have to be re-validated from scratch |
| Torque-command receive, limits, 100 ms timeout | STM32F103 | C (EFeru) | Small, local, and independent of the Pico |
| CRSF parser, channel map, failsafe, arming | Pico 2 W | Rust | New functionality; the radio is the only arm source |
| BNO055 driver, attitude fusion, gyro propagation | Pico 2 W | Rust | New functionality |
| Balance, speed and yaw controllers | Pico 2 W | Rust | New functionality; unit-tested on the host |
| Safety state machine, limits, telemetry, blackbox | Pico 2 W | Rust | New functionality; the Pico is Tier 1 |
| Mission-computer link, parameter get/set | Pico 2 W | Rust | New functionality, shared schema with the host workspace |

The two boards meet at exactly one protocol: the stock EFeru 8-byte command and
18-byte feedback frames on USART2. Neither board shares memory or code with the
other.

### 5.2. Stack

- `thumbv8m.main-none-eabihf` (Cortex-M33 with FPU), `no_std`.
- `embassy-rp` (rp235x) for the HAL and the async executor; `embassy-time` for
  tickers; `embassy-sync` for `watch`/`signal`/bounded channels.
- `probe-rs` for flashing and RTT; `defmt` for logs; `panic-probe` for panics.
- `heapless` for fixed buffers, `micromath` for scalar math, `cobs` and `crc` for
  the mission link.
- No heap, no dynamic dispatch, no blocking waits in the 200 Hz path.

The full task table, state machine, protocol details, and test plan are in
[`docs/pico-bridge.md`](../docs/pico-bridge.md).

### 5.3. Milestones

1. **Blink and clock tree.** Rust binary that boots, sets 150 MHz, and blinks the
   LED. Proves the linker script, vector table, and flashing path.
2. **Mainboard link.** Sends stock torque frames and parses feedback; wheels
   lifted, torque verified against the echoed command.
3. **I2C + BNO055.** Reads calibrated accel and gyro at 100 Hz at 400 kHz.
4. **UART1 + CRSF.** Prints channels, link quality, and the arm switch.
5. **Balance loop.** PD on pitch with the robot held by a safety rig; GP10 marker
   on the scope for jitter.
6. **Speed and yaw loops.** Drive, reverse, steer, spin in place.
7. **Safety and telemetry.** Tilt cutoff, link-loss policy, watchdog, charge
   interlock, battery telemetry to the handset.
8. **Mission link.** Framed setpoints/telemetry over the PIO UART.

### 5.4. Wiring

The pin map is in [`docs/hardware.md`](../docs/hardware.md) §3.2, and the link
table is in `docs/pico-bridge.md` §3. In short: UART0 to the mainboard's left
cable, I2C1 to the BNO055, UART1 to the XR1, one PIO UART to the mission
computer, USB for the console.

Run the BNO055 in accelerometer-gyro fusion mode (`IMUPLUS`): the magnetometer
sees motor current and DC-DC fields as heading changes, which corrupts the yaw
reference.

### 5.5. Sideboards

The two sideboards are UART slaves with their own MCU, an IMU (MPU6050,
MPU6052C or ICM-20689 depending on the revision), the battery LEDs and the
optical footpad sensors. They do not control the motors.

They must be disconnected: they transmit their own protocol on the same two
signal wires that now carry the Pico link, and two transmitters on one line is a
contention, not a bus.

Disconnecting them is safe on this build:

- The power button and the power latch are on the mainboard (PA5 on board
  variant 0).
- Motor phases and hall sensors are wired directly to the mainboard.
- The status LED is on the mainboard.
- The 15 V pins of both cables stay live; insulate them.

The only loss is the sideboard battery LEDs, which the CRSF telemetry replaces.

### 5.6. Command arbitration

Both links are inputs to one setpoint inside the Pico. Neither link ever drives
the motors directly, and the mainboard only ever sees a torque request.

| Condition | Authority |
|---|---|
| Radio disarmed | zero torque, no balancing |
| Radio stick deflection above 15 % | radio, held for 2 s after the last deflection |
| Valid mission-computer frame within 300 ms | mission computer, clamped by the Pico |
| Otherwise, radio link up | radio |
| Neither link | zero setpoints, keep balancing |

Rules:

- Only the radio can arm. Neither the mission computer nor any host process can
  enable the motors.
- The balance loop depends on neither link. Losing both means standing still,
  not falling.
- Tilt cutoff overrides every source.
- Mission frames carry a CRC and a sequence counter; a stale frame is rejected
  the same way as a corrupt one.

### 5.7. Repository rules

`firmware/pico2w` is its own Cargo workspace so that the root workspace
keeps building for the host. The root `.gitignore` ignores every
`.cargo/config.toml`, so the embedded target configuration needs an explicit
exception. `firmware/eferu` stays out of version control: it is recreated from
the pinned upstream commit plus the patches.
