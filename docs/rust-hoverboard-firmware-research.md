# Research: existing Rust firmware for hoverboard mainboards

Research date: 2026-11-24. Method: GitHub repository and code search via the
GitHub API (`search_repositories`, `search_code`, `get_file_contents`). Web
search was unavailable (HTTP 402), so conclusions are limited to indexed GitHub
repositories. All links and facts below came from the API responses.

## 1. Search results

Queries that returned relevant results:

| Query | Result |
|---|---|
| `hoverboard language:rust` | 4 repos (2 relevant, 2 unrelated) |
| `hoverboard in:name,description,readme language:rust` | 17 repos (3 relevant + this project) |
| `hoverboard-firmware-hack-FOC` | 37 repos, all C or C++ forks |
| `hoverboard foc language:rust` | 0 |
| `hoverboard bldc language:rust` | 0 |
| `topic:hoverboard language:rust` | 0 |
| `GD32F103 Rust motor firmware` | 0 |

The two genuinely relevant Rust hoverboard firmware projects are
**hoverboardhavoc/hoverboard-firmware** and **hoverkite/hoverkite**. One related
HAL project (**hoverboardhavoc/runtime-hal**) is abandoned. No other Rust FOC
firmware for these mainboards was found in the GitHub index.

| Project | URL | Created / last push | Stars | License | Boards |
|---|---|---|---|---|---|
| `hoverboardhavoc/hoverboard-firmware` | https://github.com/hoverboardhavoc/hoverboard-firmware | 2026-06-16 / 2026-11-23 | 5 | GPL-3.0-or-later | Single board GD32F103RC (the EFeru target), and split GD32F103C8 / GD32F130C8 |
| `hoverkite/hoverkite` | https://github.com/hoverkite/hoverkite | 2021-03-20 / 2026-11-09 | 17 | MIT OR Apache-2.0 | Split boards TT-SD2.2, GD32F130x8 (`gd32f1x0-hal 0.11`) |
| `hoverboardhavoc/runtime-hal` | https://github.com/hoverboardhavoc/runtime-hal | 2026-06-17 / 2026-11-02 | 1 | MIT OR Apache-2.0 | Abandoned; one binary for GD32F10x/F1x0, now replaced by the compile-time HAL in the firmware repo |
| `fobaty/jbot` (this project) | https://github.com/fobaty/jbot | 2026-10-22 / 2026-11-17 | 1 | see repository | STM32F103RC torque node + Pico balance loop |

## 2. hoverboardhavoc/hoverboard-firmware — closest match

This is the most relevant project found: it targets **the same single-board
hoverboard mainboard** that EFeru's firmware targets, and it is actively
developed (last push 2026-11-23, HEAD `15c1591`, 26 open issues).

Design highlights from its README and `docs/getting-started.md`:

- **Board is configured, not compiled.** Pin map, IMU, drive mode and tuning are
  settings stored on the board. Changing them needs no rebuild or reflash.
- Settings live in a **log-structured flash store** with per-record CRC; the
  newest record for a key wins, a missing key uses its default, and a torn write
  loses only one record (`shared/store/`).
- Settings are changed over the board's link (Bluetooth, UART or SWD). Writes
  are refused while armed and take effect at the next arm.
- **FOC, sine and six-step commutation** are all in the image; `machine.method`
  selects one. FOC without current sensing falls back to six-step and raises a
  fault. Commutation is checked on the host against behaviour recorded from the
  stock firmware (`motorcontroller/commutation/src/{foc,sine,sixstep,tests}.rs`).
- **250 Hz control loop + 16 kHz motor interrupt**; balance uses a state-feedback
  controller tuned live from a phone. A rigid-frame rover balances on it and has
  been ridden outdoors (project's own claim).
- **Board model** reads pin map from settings at boot and refuses a layout that
  would drive the wrong pin (`motorcontroller/board/`).
- Safety: mode machine, fault latches and an **arming gate that is the only path
  to energizing the motor bridge** (`motorcontroller/state/{mode,fault,moe}.rs`).
- Tooling: `hoverboard-cli` flashes through an ESP32-C3 bridge on the SWD header
  (no debug probe needed for normal use), edits settings and drives wheels;
  an Android app arms/drives/tunes over Bluetooth. Bundle contains every image.
- Status table marks HAL, store, firmware image, board model, link stack,
  safety, all three commutation methods, single-board dual-motor, optional
  protections, IMU/attitude, balance control, tones, Android app and CLI as
  working on hardware; foot pads are in progress.

### Its EFeru board configuration

`shared/config-checks/machines/EFeru.json` describes the same board as ours:

- power self-hold `PA5`, buzzer `PA4`, LED `PB2`, battery sense `PC2/ADC12`,
  10s pack, `pole_pairs 15`, `align_offset 1`;
- right motor hall `PC10/11/12`, gate TIM1 high `PA8/9/10`, low `PB13/14/15`,
  phase current on `PC4/PC5`, DC link `PC1`;
- left motor hall `PB5/6/7`, gate TIM8 high `PC6/7/8`, low `PA7/PB0/PB1`,
  phase current `PA0/PC3`, DC link `PC0`;
- `dead_time 32`, `current.limit 10000`, `sense: low_side_on`, `rail_skip 3848`,
  timer break input present but disabled.

Differences from our pinned build: dead time 32 vs 48, configurable limits/pin
map vs our compile-time constants, and a full settings store vs our 16-word NVM
record.

## 3. hoverkite/hoverkite — older, split boards only

A kite-flying project from 2021, still lightly maintained (last push
2026-11-09). Firmware crate `cross/hoverkite-firmware` uses `gd32f1x0-hal 0.11`,
`cortex-m 0.7`, `embedded-hal 1.0`, a BMI160 IMU, and a custom serial protocol
(`src/protocol.rs`). It targets split-board hoverboards (`TT-SD2.2`,
GD32F130x8), not the single board we have. Its README documents the same
flashing caveat we hit: the firmware does not latch power itself, so the power
button must be held during `cargo run`, and the ST-Link 3.3 V line should be
disconnected to avoid fighting the board's supply. License MIT OR Apache-2.0.

## 4. hoverboardhavoc/runtime-hal — abandoned reference

A runtime HAL that detects STM32F1-compatible GD32 parts at boot by a reserved
region read that deliberately faults, then measures timer/ADC instance counts by
register write-back. Validated only on three GD32 parts; the motor hot path was
never run on silicon. Marked abandoned 2026-11-02 and replaced by the
compile-time HAL in `hoverboard-firmware`. Useful as a reference on GD32 clone
quirks (e.g. the F103 RCT6 measured 2 advanced timers / 3 ADCs, and ST and GD32
both report `DEV_ID 0x410`), but not a firmware to adopt.

## 5. What this means for our firmware

1. **A mature Rust firmware for this exact board exists and is active.**
   `hoverboardhavoc/hoverboard-firmware` is a better base than our hand-written
   HAL if the goal is a full vehicle (pin configurability, balance control,
   phone/CLI tooling, flashing without a probe).
2. **Our niche is still distinct.** We have a bit-exact port of the generated C
   FOC controller (120/120 golden vectors), a runtime FOC trace with full 32-bit
   integrators, a 13-key calibration console, and a dual wire contract. The
   hoverboardhavoc project verifies commutation against recorded stock behaviour
   but is not a bit-exact port of the generated controller.
3. **Patterns worth adopting:** log-structured per-record CRC settings store,
   refusing setting writes while armed, board-layout validation against the
   chip at boot, and a separate bridge for flashing/configuration.
4. **Licensing.** The closest project is GPL-3.0-or-later, as is EFeru's C
   firmware. Our earlier open question about a GPL-3.0 derivative work remains:
   a bit-exact port of generated C controller code and tables is very likely a
   derivative work, so GPL-3.0-or-later is the safe interpretation. This should
   be resolved before any distribution.
5. **No copy-paste was done.** This research only read public repositories; no
   code from them was brought into this workspace.

## 6. Self-balancing specifically

### 6.1 On the hoverboard mainboard — yes

`hoverboardhavoc/hoverboard-firmware` is the only project found that balances on
the same single-board hoverboard controller that our F103 replaces. Its status
table marks balance control working on hardware, and the project states a
rigid-frame rover balances on it and has been ridden outdoors.

- **Law:** four-term state feedback — `torque = pitch + pitch rate + position +
  speed` (`docs/state-feedback-tuning.md`, `motorcontroller/control/src/sf.rs`).
- **Loop:** 250 Hz control plus 16 kHz motor interrupt. IMU driver and pitch/roll
  filter in `motorcontroller/imu` and `motorcontroller/attitude`.
- **Tuning:** live from the Android app. The documented starting set is
  `gp=3000, gr=110, gx=0, gv=0, speed_max=0, speed_slew=80`, with a step-by-step
  procedure (raise `gp` to 75 % of shake onset, `gr` to 65 % of buzz onset, then
  `gv`, `gx`, `speed_max`, `speed_slew`). A `tools/lqr-gains.py` produces model
  gains; the hand-tuned rover values on main `7d18788` were
  `gp=1955, gr=170, gx=5, gv=270, speed_max=57, speed_slew=246, pos_clamp=45`.
  Writes are refused while armed, matching our `$SET` policy.
- **Architecture difference:** their IMU and balance law run on the motor
  controller itself; our split is STM32 = 16 kHz FOC torque node, Pico 2 W =
  200 Hz balance authority. Their gains cannot be copied directly into `jbot`.

### 6.2 Other Rust self-balancing projects (different hardware, reference only)

| Project | Hardware | Control | Notes |
|---|---|---|---|
| `japaric/zen` | Blue Pill (STM32F103), MPU9250, HC-06 | PID tuned live over serial | 2018, 17 stars, Apache/MIT. Serial frame is **COBS with 0x00 delimiter and CRC16 in 18-byte frames** — close to our hardened protocol. Logs accel, angular rate, estimated tilt and raw PWM duty. |
| `HomayoonSl/self-balancing-robot` | STM32F103 Blue Pill, BMI160, AS5600, DRV8833 | `no_std` inverted pendulum, mechanical inertia rod | 2026, MIT, demo video on real hardware. Same MCU family, different motor drivers. |
| `tana/balance-robot2` | ESP32, stepper motors | LQR | 2022, MIT, 13 stars. Control-law reference. |
| `pgranieri/awaitbalance` | Nucleo STM32F207ZG, BNO085 | async/await | 2025, MIT. Architecture reference only. |
| `eighthkey/rtic-balance-controller` | STM32F407 custom PCB, RTIC | — | Empty skeleton (size 1 KB, one commit). |

### 6.3 Relevance to `jbot`

- The closest usable reference is `hoverboardhavoc`'s four-term law and tuning
  procedure; it validates the idea of tuning the balance terms from a host tool
  while the actuators stay disarmed.
- `japaric/zen` is the closest reference for our **calibration trace** concept:
  a serial bridge that streams tilt and controller state for script tuning.
- None of these are a drop-in replacement for our Pico-side 200 Hz balance loop;
  the porting work would be the control law and the IMU/encoder drivers, not the
  hoverboard FOC node.
