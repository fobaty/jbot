# Robot firmware

| Field | Value |
|---|---|
| Document | Two-board firmware: Pico 2 W real-time controller, F103 torque node |
| Code | FW-001 |
| Version | 0.3 |

| Status | Rust firmware is the active target for both boards; the pinned EFeru C tree is kept as hardware reference and rollback |
| Scope | STM32F103RCT6 hoverboard mainboard (Rust torque node) and Raspberry Pi Pico 2 W (Rust balance controller) |

This directory holds all robot firmware. The STM32F103 mainboard runs the Rust
torque node in `stm32f103/rust/`; the Pico 2 W runs the Rust balance controller
in `pico2w/`. The pinned EFeru C sources in `stm32f103/eferu-foc/` are a
hardware reference and a known-good hoverboard build, **not** the firmware for
this self-balancing robot. All firmware workspaces are separate from the `jbot`
Cargo workspace, each with its own toolchain, so they never affect
`cargo test --workspace`.

> **Start here.** For the active mainboard firmware, its checks and the flashing
> procedure, see [`stm32f103/README.md`](stm32f103/README.md). Section 5 below
> describes the reference C baseline and is kept for its hardware facts only.

## 1. Layout

| Path | Content |
|---|---|
| `stm32f103/rust/f103_core/` | Hardware-independent F103 logic: FOC port, protocol, alerts, safety, battery, thermal, console, trace. Host-tested. |
| `stm32f103/rust/f103_fw/` | F103 PAC/HAL firmware: clocks, TIM1/TIM8, ADC/DMA, both UARTs, IWDG, ISRs. |
| `stm32f103/rust/tools/` | FOC golden-vector recorder, console bridge, binary trace decoder, build/flash script. |
| `stm32f103/eferu-foc/` | Pinned upstream C firmware, JBOT patches and upstream documentation. Reference and rollback only. |
| `pico2w/fw/` | Pico 2 W firmware package: Embassy tasks, peripheral drivers, USB console, debug bridge. Owns `.cargo/config.toml` and `memory.x`. |
| `pico2w/core/` | `no_std`, I/O-free logic used by `fw/` and unit-tested on the host: EFeru frames, hardened protocol, BNO055 decoding, cascade balance law, plant and predictor. |

## 2. Hardware in scope

Reused from the hoverboard:

| Part | Role | Notes |
|---|---|---|
| Mainboard, STM32F103RCT6, 12 MOSFETs | power stage, FOC, sensors, power latch | Rust torque actuator node |
| Two hub motors with hall sensors | drive and speed feedback | about 350 W each, direct drive |
| 10s2p 4.4 Ah pack with BMS | energy | 42 V full, about 30 V cutoff |
| Power button and latch | on/off | PA5 latch, short press powers off, long press calibrates |
| Charge port | charging | BMS charge input plus a charge-detect line to PA12 |

Not reused: both sideboards, the deck, the plastic covers.

Added to the robot: **Pico 2 W (RP2350)**, XR1 receiver, BNO055, a 36 V to 5 V
converter rated for at least 60 V input and 1.5 A, a main fuse, and a hardware
kill switch in the battery line.

Important hardware consequences:

- **Charging interlock.** The charge-detect line is PA12 on the mainboard and
  has a candidate wire to the Pico. The robot must stay disabled while the
  charger is plugged in.
- **Regenerative braking.** A balancing robot regenerates every time it leans
  back. With a full pack the BMS can cut the charge path on overvoltage and the
  DC link then spikes. Start runs at about 90 % charge and keep regen current
  limited until it is measured.
- **The power button is not an E-stop.** The firmware holds the latch on PA5;
  if it hangs, the button no longer cuts power. The hardware kill switch in the
  battery line is the real safety path.
- **Two firmwares, two timeouts.** The Pico can hang; the mainboard cannot
  depend on it. The mainboard enforces its own serial timeout, and the Pico's
  hardware watchdog resets the balance loop if it stalls. Timeout values are
  firmware contracts, not tuning knobs; see `stm32f103/README.md`.

## 3. Active firmware

### 3.1. Split of responsibilities

| Layer | Where | Implementation |
|---|---|---|
| FOC core, SVPWM, current PI, hall capture, power latch | STM32F103 | Rust port of the generated C controller, bit-exact against golden vectors |
| Torque-command receive, limits, link-loss alarm, telemetry | STM32F103 | Rust (`f103_core` + `f103_fw`) |
| CRSF parser, channel map, failsafe, arming | Pico 2 W | Rust (new functionality) |
| BNO055 driver, attitude fusion, gyro propagation | Pico 2 W | Rust |
| Balance, speed and yaw controllers | Pico 2 W | Rust, unit-tested on the host |
| Safety state machine, limits, telemetry, blackbox | Pico 2 W | Rust |
| Mission-computer link, parameter get/set | Pico 2 W | Rust, schema shared with the host workspace |

The two boards meet at one protocol: the stock 8-byte command and 18-byte
feedback frames on USART2. They share no memory and no code.

### 3.2. STM32F103 Rust torque node

- 16 kHz FOC from a fixed-point port of the generated C controller, replayed
  against the golden vectors in `stm32f103/rust/tools/foc_golden/`.
- USART2 (PA2/PA3, 115200 8N1) carries commands and 100 Hz feedback. USART3
  (PB10/PB11, 115200 8N1) is an independent text/binary console with `$GET`,
  `$GETP`, `$WATCH`, `$TRACE`, `$DUMP`, `$ADC` and `$SAVE`.
- A missing serial link raises the three low-pitch buzzer beeps from the first
  main-loop tick and forces the safe output. Battery, temperature, motor and
  backward-drive alerts keep the reference priority.
- The default build keeps **MOE low**: the torque loop can be observed on the
  bench, but no gate can be driven until HIL validation and explicit sign-off.

Checks and build (from `firmware/stm32f103/rust`):

```sh
cargo fmt --all -- --check
cargo test -p f103_core
cargo clippy -p f103_core --all-targets -- -D warnings
cd f103_fw && cargo build --release
```

`firmware/stm32f103/rust/tools/calibration/flash.sh` builds the image, checks
the application size and prints the flash command; `--flash` performs the write
and needs explicit approval. SWDIO, SWCLK and GND only, no NRST; hold the board's
power button during the whole write. Full details are in
[`stm32f103/README.md`](stm32f103/README.md).

### 3.3. Pico 2 W Rust balance controller

Stack: `thumbv8m.main-none-eabihf`, `no_std`, `embassy-rp`/`embassy-time`/
`embassy-sync`, `defmt`, `heapless`, `micromath`, COBS/CRC16 for hardened frames.
No heap, dynamic dispatch or blocking waits in the 200 Hz path. Flashing uses
`picotool` via `cargo run --release`; `defmt` over RTT is available for a future
debug probe.

`pico2w/fw/` owns the hardware. Embassy tasks:

| Task | Rate | Work |
|---|---|---|
| `imu_task` | 100 Hz | Resets the BNO055, selects `IMUPLUS`, publishes the latest Euler/gyro/calibration sample |
| `link_rx_task` | event | Reads bytes from UART0 and resynchronises the 18-byte feedback parser |
| `control_task` | 200 Hz | Computes the cascade command, sends the 8-byte frame, toggles GP10, feeds the watchdog |
| `usb_task` | event | RP2350 USB CDC-ACM console |
| `console_task` | event | Line protocol: commands in, telemetry out |

`pico2w/core/` owns the control law, filters, BNO055 decoding, EFeru and
hardened wire formats, NVM schema, plant model and predictor. It has no I/O and
is host-tested:

```sh
cd firmware/pico2w/core
cargo test
cargo clippy --all-targets -- -D warnings
cd ../fw
cargo check
cargo build --release
```

Put the board into BOOTSEL mode before flashing. The image enumerates as a USB
CDC-ACM port; telemetry is visible without a probe:

```sh
python3 -m serial.tools.miniterm /dev/cu.usbmodemJBOT_PICO_011 115200
```

Console commands include `help`, `ping`, `stat`, `stream <hz>`, `speed <mps>`,
`steer <deg/s>`, `trim <deg>`, `reinit`, `estop`, `clear`, `stats reset` and
`reboot`. A telemetry line looks like:

```
T step=19043 loop=200 imu=ok cal=3 pitch=1.50 roll=-2.25 hdg=87.40 gy=0.06 gx=-0.12 gz=0.03 err=0x00 mb=ok L=0 R=0 bat=41.50 tmp=39.3 age=3 estop=0 vset=0.00 wset=0.00
```

`estop` latches a firmware-side emergency stop: the control task keeps sending
zero torque at 200 Hz until `clear`. The full protocol, `stat` fields and IMU
error codes are in [`docs/pico-bridge.md`](../docs/pico-bridge.md) §8.1.

Bench check after flashing: 200 control steps per second with no watchdog
reset, `imu=ok`, `err=0x00`, and `mb=ok` with `age` of 0–10 ms. Common faults:

| Symptom | Meaning |
|---|---|
| `mb=none`, `fb=0`, `fb_drop` rising | Mainboard link down; check power and GP16/GP17 ↔ PA3/PA2 |
| `imu=stale`, `imu_code=1` | I2C failure to the BNO055; check wiring and address strap |
| `loop` below 195 | 5 ms tick missing deadlines; suspect USB or `defmt` cost |

The pinned wire contract, pin tables and task model are in
[`docs/hardware.md`](../docs/hardware.md) and
[`docs/pico-bridge.md`](../docs/pico-bridge.md). The native console in
`crates/jbot_console` drives the same commands from the mission computer.

## 4. Command arbitration

Both links feed one setpoint inside the Pico. Neither drives the motors
directly; the mainboard only sees a torque request.

| Condition | Authority |
|---|---|
| Radio disarmed | zero torque, no balancing |
| Radio stick deflection above 15 % | radio, held for 2 s after the last deflection |
| Valid mission-computer frame within 300 ms | mission computer, clamped by the Pico |
| Otherwise, radio link up | radio |
| Neither link | zero setpoints, keep balancing |

Only the radio can arm. Tilt cutoff overrides every source. Mission frames carry
a CRC and a sequence counter; stale frames are rejected like corrupt ones.

## 5. Reference C firmware (`eferu-foc/`)

The pinned upstream tree is kept because it records measured hardware facts the
Rust port depends on: timer topology, ADC/DMA trigger alignment, dead time,
hall/phase mapping, current scales, board variants, schematics and photos under
`eferu-foc/eferu/docs/`. The annotated board pinout is in
[`stm32f103/README.md`](stm32f103/README.md).

It is a working `VARIANT_USART` / `FOC_CTRL` / `TRQ_MODE` hoverboard build
(10 A / 12 A / 400 rpm, 115200 8/18-byte frames), but it is not suitable for a
self-balancing robot: link loss ends in `OPEN_MODE` freewheel only after about
800 ms, inactivity or low battery latches power off, the protocol has only an
XOR checksum, and balancing is marked `TODO` upstream. Build, restore and
flashing details are in `eferu-foc/README.md`. Do not push to its upstream
remote.

## 6. Wiring and sideboards

In short: UART0 on the Pico goes to the mainboard's left cable (GP16 TX → PA3,
GP17 RX ← PA2, 115200 8N1), I2C1 to the BNO055, UART1 to the XR1, one PIO UART
to the mission computer, USB for the console. The full cross-board table with
every pin, direction and cable is in [`docs/wiring.md`](../docs/wiring.md).

Run the BNO055 in accelerometer-gyro fusion mode (`IMUPLUS`): the magnetometer
sees motor current and DC-DC fields as heading changes, which corrupts the yaw
reference.

Both sideboards must stay disconnected. They are UART slaves with their own
MCU, IMU, battery LEDs and footpad sensors; they transmit their own protocol on
the same two signal wires that now carry the Pico link. The power button, latch,
motor phases, hall sensors and status LED are all on the mainboard, so nothing
is lost except the sideboard battery LEDs. The 15 V pins on both cables stay
live — insulate them.

## 7. Repository rules and open items

Embedded firmware targets are selected by `.cargo/config.toml` in each firmware
workspace; the root `.gitignore` needs explicit exceptions for them and for the
`pico2w/core/` crate (the generic `core` rule is for crash dumps). The nested
`eferu-foc/eferu/.git` still points at upstream and must not be pushed.

Not done yet: MOE enable and HIL validation for the F103, CRSF/XR1 ingest,
arming, the mission link, non-zero balance gains, and bench verification of the
charge interlock. The F103 default image cannot energize the bridge.
