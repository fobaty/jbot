# Full research: GitHub user `hoverboardhavoc` and flashing feasibility for our board

Research date: 2026-11-24. Source: GitHub API (repository search, code search,
file contents). No code was copied into this workspace.

## 1. User inventory (10 public repositories)

| Repository | Language | Created / last push | Stars | License | Relevance |
|---|---|---|---|---|---|
| `hoverboard-firmware` | Rust | 2026-06-16 / **2026-11-23** | 5 | GPL-3.0-or-later | **Critical** — full vehicle firmware for the same hoverboard mainboard |
| `HoverboardImu` | C++ | 2024-09-08 / 2024-09-08 | 2 | — | IMU reverse-engineering on split boards |
| `Hoverboard-Firmware-Hack-Joystick-Controller` | C | 2025-05-05 / 2026-04-08 | 1 | GPL-3.0 | Joystick control over UART; adaptable to EFeru |
| `runtime-hal` | Rust | 2026-06-17 / 2026-11-01 | 1 | Apache-2.0 | Abandoned HAL reference |
| `havoc-radio` | C++ | 2026-11 | 0 | — | Unrelated (ESP8266 radio) |
| `imiev-drop-dolly` | OpenSCAD | 2026-09 | 0 | CC-BY-4.0 | Unrelated |
| `blobmill` | Python | 2026-06 | 0 | Apache-2.0 | Unrelated (decompilation tool) |
| `hoverboardhavoc.com` | JavaScript | 2026-06 / 2026-11-19 | 0 | — | Personal blog |
| `eibi-binary-table` | Python | 2026-11 | 0 | — | Unrelated |

Only the first four are relevant to a balancing robot.

### 1.1 `hoverboard-firmware` (the important one)

- HEAD `15c1591`, last push 2026-11-23, 26 open issues — actively developed.
- One Rust workspace with separate `motorcontroller/` and `shared/` workspaces,
  plus `bridge/`, `apps/`, `tools/`.
- HAL crate supports exactly four chips, selected by cargo feature
  (`motorcontroller/hal/Cargo.toml`): `gd32f103c8`, `gd32f130c8`,
  `gd32f103rc`, `mm32spin05`. **There is no STM32 feature.**
- Firmware images: `cargo image-f103`, `image-f130`, `image-f103rc`
  (the 12-FET, dual-motor board), and a CI-only non-flashable `image-mm32`.
- Settings are stored on the board in a log-structured flash store with per-record
  CRC (`shared/store/`); pin map, IMU wiring, drive mode, gains and limits are
  settings, not compile-time constants. Writes are refused while armed.
- Configuration can be done through `hoverboard-cli` via an ESP32-C3 bridge on
  the SWD header, or — with the ST-Link we already have — through
  `tools/swd-mailbox-config` over SWD (`docs/bench-board-config.md`).
- `tools/flash.sh` has serious safety guards: image-size floor, required
  symbols, no-`wfi` scan (GD32 SWD lockout), and an armed-bridge guard that
  reads timer MOE before halting. It does **not** verify that the board's chip
  matches the image (`specs/hal.md` section 7 says so explicitly).
- Balance: four-term state-feedback `torque = pitch + pitch rate + position +
  speed`, 250 Hz control loop, live tuning from the Android app,
  `tools/lqr-gains.py` to generate starting gains, documented tuning procedure
  in `docs/state-feedback-tuning.md`. Project claims a rigid-frame rover
  balances and has been ridden outdoors.
- Their `shared/config-checks/machines/EFeru.json` is a settings file for the
  12-FET board that matches our pinout: PA5 latch, PA4 buzzer, PB2 LED, PC2
  battery, TIM1/TIM8 gate pins, halls, current-sense ADC channels. Differences:
  `dead_time 32` (ours 48), `current.limit 10000`, `pole_pairs 15`,
  `align_offset 1`.

### 1.2 `HoverboardImu`

Reverse-engineering of an MPU-6050-class IMU at I2C `0x68` on split boards
(GD32F130C8). The register dump matches the MPU6050 map; WHO_AM_I did not
identify cleanly. Useful confirmation that hoverboard boards carry an
MPU-6050-class sensor, but it targets split boards, not our single board.

### 1.3 Joystick controller

C/SDL2 tool that drives Gen2.x firmware over UART from a Raspberry Pi or Mac:
configurable serial device/baud, joystick axis and arm button, live telemetry.
Easily adaptable to our 8-byte legacy command frame if we want a human-driven
maneuvering input for the bench.

### 1.4 `runtime-hal`

Abandoned 2026-11-02, replaced by the compile-time HAL. Valuable findings:
GD32F103RCT6 measured 2 advanced timers / 3 ADCs / 256 KiB; GD32 and ST both
report `DEV_ID 0x410` (clone ID is unreliable); genuine STM32 support was
listed as "plausible but untested", and the motor hot path was never run on
silicon.

## 2. Can we flash their firmware on our board?

### 2.1 Our chip, as measured

Earlier probes on this bench (ST-Link V2J37S7) reported:

- `st-info --probe`: `chipid 0x414`, `dev-type F1xx_HD`, `flash 262144`,
  `sram 65536`, voltage ≈ 3.23 V.
- OpenOCD with plain `interface/stlink.cfg` + `target/stm32f1x.cfg` detected a
  Cortex-M3 r1p1 and 256 KiB flash **without** `set CPUTAPID 0`.

Their bench GD32 boards need `set CPUTAPID 0` and read IDCODE
`0x13030410` (`docs/bench-board-config.md`). Our board does not. This points to
a **genuine STM32F103RC**, not a GD32 clone.

### 2.2 Verdict

**No — do not flash their prebuilt image blindly.** Reasons:

1. Their images are built for GD32 parts; the HAL has no STM32 chip descriptor
   or vector table. Their own documentation classifies genuine STM32 as
   "plausible but untested".
2. Even if the F10x register model is mostly compatible, differences in flash
   wait states, option bytes, clock tree, ADC and timer break input are exactly
   the class of thing that produces either a dead board or a board that drives
   the FETs incorrectly.
3. A freshly flashed board has an empty settings store and **drives nothing**
   until a board file and a machine file are applied. So "flash and it balances"
   is not how this firmware works.
4. Their flash script does not check chip identity, so it would happily write
   the GD32 image to our STM32.

What *is* safely flashable today: our own `f103_fw` build (already flashed once,
bit-exact golden vectors, MOE low), and the original C firmware (currently on
the board). No external image should be flashed until the chip is positively
identified and, if it is an STM32, a proper port is built and bench-tested.

## 3. IMU situation

Our robot already has an IMU: **BNO055 on Pico I2C1 at 400 kHz, 100 Hz fused
attitude** (`docs/architecture.md:53,547`). The Pico is the balance authority
(200 Hz); the STM32 is only the 16 kHz torque node.

Their firmware instead uses an MPU-6050-class raw IMU at 0x68 with a fixed-point
attitude filter, configurable model (`MPU6050` WHO_AM_I 0x68, clone `0x2E`),
axis sign map, gyro bias and axis roles (`motorcontroller/imu/src/lib.rs`).

**Moving the IMU is not necessary.** Keeping the BNO055 on the Pico preserves
our architecture and avoids both the GD32/STM32 issue and a new sensor wiring
job. The thing worth transferring is the **control law**, not the sensor.

## 4. How to get balance and maneuvering

### Option A — recommended: port the law to the Pico, keep our firmware

- Keep STM32 Rust firmware as the torque node (legacy link, 16 kHz FOC,
  calibration console and trace).
- Implement the four-term law on the Pico at 200 Hz using BNO055 pitch/pitch-rate
  plus hall-derived axle position/speed:
  `torque = gp*pitch + gr*pitch_rate + gx*position + gv*speed_error`.
- Maneuvering: a speed reference from the radio stick, `speed_max` and
  `speed_slew` exactly as in their design; that is what makes it drive and
  stop, not just stand.
- Starting gains from their guide (`gp=3000, gr=110, gx=0, gv=0,
  speed_slew=80`) are a reference, but must be re-tuned on our machine with our
  own mass/inertia/wheel-radius values; `tools/lqr-gains.py` is directly usable
  with our parameters (numpy only).
- Safety: their pattern of refusing setting writes while armed matches our
  `$SET` gate. MOE remains low until HIL sign-off.

### Option B — full port of their firmware to STM32F103RC

- Add an `stm32f103rc` chip feature/descriptor + vector table to their HAL,
  verify register paths against ST RM0008, bench-test the motor hot path.
- Use their store, CLI/app, balance law and three commutation methods.
- Then apply `EFeru.json` as a starting board file, adjust dead time to 48 and
  current limits, calibrate IMU sign/roles, and tune gains.
- High effort; GPL-3.0-or-later inherited. Only worth it if we want their whole
  vehicle stack rather than just balance.

### Option C — flash the GD32F103RC image as-is

Not recommended. Possible outcomes range from "no boot" to "boots but wrong
peripheral behaviour". Do not attempt without a positive chip identification
and a rollback plan; we have the C firmware backup in git.

## 5. Concrete next steps

1. Reconnect the ST-Link and positively identify the chip: read DBGMCU IDCODE
   and flash density, and compare with the GD32 signature
   (`0x13030410` with `set CPUTAPID 0`). This is the gate for any external image.
2. Decide A or B. For A, the deliverable is a Pico-side four-term controller
   with live gains over the existing console, plus `speed_max`/`speed_slew`
   maneuvering — no change to the STM32 firmware.
3. For either path, keep MOE disabled until HIL validation and Safety-Reviewer
   sign-off. Nothing here authorizes torque on the bench.
