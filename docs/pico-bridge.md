# Pico 2 W Bridge Firmware

| Field | Value |
|---|---|
| Document | Real-time controller firmware (Pico 2 W / RP2350) |
| Code | FW-002 |
| Version | 0.1 |
| Status | Stage 1 Pico 2 W firmware specification |
| Status | Planned; not implemented |
| Scope | Balance control, radio ingest, IMU, mainboard torque link, mission link, safety |

This document specifies the firmware in `firmware/pico-bridge/`. It is the
authority for the Pico's pin map, task schedule, wire protocols, and safety
behaviour. The architecture rationale is in
[`docs/architecture.md`](architecture.md) §3.2; the physical pin table is in
[`docs/hardware.md`](hardware.md) §3.2.

## 1. Role

The Pico 2 W is the robot's real-time controller (Tier 1). It owns everything
that decides what the robot does:

- CRSF ingest from the XR1 receiver, channel mapping, failsafe, and the arm
  switch. The radio is the **only** source of arming authority.
- BNO055 sampling and attitude propagation.
- The 200 Hz balance loop: pitch PD, speed loop, yaw loop.
- Limits: tilt cutoff, torque and rate clamps, battery thresholds, charge
  interlock.
- The safety state machine, the watchdog, and the blackbox ring buffer.
- Torque egress to the mainboard and consumption of its feedback.
- The mission-computer link: setpoints in, telemetry and parameters out.

It does **not** own the power stage. The STM32F103 mainboard keeps the 16 kHz FOC
loop, the current limits, the hall capture, the pack measurement, and an
independent 100 ms command timeout.

## 2. Platform facts that shape the code

| Property | Value | Consequence |
|---|---|---|
| Core | Dual Cortex-M33 at 150 MHz, FPU | Float math in the control loop is affordable |
| SRAM | 520 KB | No need for fixed-point gymnastics; still `no_std`, no heap |
| Flash | 4 MB | Room for two images, calibration, and a blackbox partition |
| UARTs | 2 hardware PL011 + PIO | CRSF and the mainboard link on hardware; the mission link on PIO |
| Timers | 64-bit hardware timers, alarm pool | The 200 Hz tick does not drift and is not a scheduled software task |
| Watchdog | RP2350 hardware watchdog | Independent reset if the loop stops feeding it |
| Wireless (2 W) | CYW43439 | **Not initialised** in stage 1 |

Wireless stays off: it costs power and EMI, and the mission computer already
provides connectivity. Enabling it is a separate decision with its own power and
EMI budget.

## 3. Pins and links

| Function | Pins | Peripheral | Peer | Rate |
|---|---|---|---|---|
| Torque command | GP0 TX → STM32 PA3 | UART0 TX | Mainboard USART2 | 115200 8N1 |
| Mainboard feedback | GP1 RX ← STM32 PA2 | UART0 RX | Mainboard USART2 | 115200 8N1 |
| IMU | GP2 SDA, GP3 SCL | I2C1 | BNO055 | 400 kHz |
| Radio | GP4 TX → XR1 RX, GP5 RX ← XR1 TX | UART1 | XR1 receiver | 420 kbaud CRSF |
| Mission computer | GP6 TX → RPi GPIO15, GP7 RX ← RPi GPIO14 | PIO UART | Raspberry Pi 5 | 460800 8N1 |
| Balance marker | GP10 | GPIO | Logic analyzer | One toggle per balance step |
| Charge detect (candidate) | GP11 | GPIO in | Mainboard PA12 | Event |
| Debug console | USB CDC, SWD | USB, SWD | Host / probe | — |

All GPIO are 3.3 V and not 5 V tolerant. The XR1 output is 3.3 V. Do not connect
the 12/15 V cable rail to any Pico pin.

## 4. Protocols

### 4.1. Mainboard link (EFeru, unchanged)

The mainboard runs upstream EFeru C with `CTRL_MOD_REQ = TRQ_MODE` and
`SERIAL_TIMEOUT` shortened to 20 ticks (100 ms). The wire format is the stock one.

| Direction | Frame | Layout | Rate |
|---|---|---|---|
| Pico → mainboard | 8 bytes | `<HhhH` = `0xABCD`, steer (left torque), speed (right torque), checksum (16-bit XOR) | 200-400 Hz |
| Mainboard → Pico | 18 bytes | `<HhhhhhhHH` = start, `cmd1`, `cmd2`, `speedR_meas`, `speedL_meas`, `batVoltage`, `boardTemp`, `cmdLed`, checksum | 100 Hz |

Notes:

- In `TRQ_MODE` the two int16 fields are torque requests, `|value| ≤ 1000`
  mapping to `|I_q| ≤ I_MOT_MAX` (10 A in the jbot profile).
- The feedback frame echoes `cmd1`/`cmd2`, which gives a direct acceptance-rate
  measurement: compare the echoed value with what was sent.
- `usart2_rx_check` accepts a frame only when the byte count since the last IDLE
  equals `commandL_len` (8). Send with a small inter-frame gap; do not stream
  back-to-back bytes.
- There is no sequence number. The mainboard applies the latest valid frame and
  relies on its 100 ms timeout to bound staleness.
- Feedback units: `batVoltage` is volts times 100, so `4150` means `41.50 V`.
  `boardTemp` is a tenth of a degree Celsius, so `349` means `34.9 °C` — EFeru
  computes it from `board_temp_deg_c` with `TEMP_CAL_*_DEG_C` expressed ×10.
  The Rust decoder keeps the raw field as `board_temp_decideg_c` and converts in
  `board_temp_celsius()`, with a golden test built from a captured bench frame.
- Optional `ENABLE_ODOMETRY` extends the feedback to 22 bytes with per-wheel
  odometry. Not enabled until odometry is actually consumed (open item).

Golden byte vectors for both directions belong in the test suite.

### 4.2. Radio link (CRSF)

- CRSF frames at 420 kbaud on UART1: address, length, type, payload, CRC-8/DVB-S2.
- Channel data comes in `RC_CHANNELS_PACKED` (16 × 11-bit channels).
- Validate the CRC, then the failsafe bit. A failed frame does not update the
  setpoint; a timeout zeroes the setpoints but **keeps balancing**.
- Telemetry replies (`0x08` battery, `0x02` GPS placeholder, `0x1E` attitude) are
  sent at 10-50 Hz and are best effort.
- Arming is a two-position switch. Only its rising edge while the robot is
  stationary and self-test passed can enter `BALANCE`.

### 4.3. Mission-computer link (framed)

Reuses the schema from [`docs/architecture.md`](architecture.md) §5.3, identical
to the host-side implementation:

| Element | Value |
|---|---|
| Header | 9 bytes: version, msg_type, flags, seq u16, timestamp_us u32 |
| Payload | ≤ 53 bytes, `postcard` |
| CRC | CRC-16/CCITT-FALSE over header + payload |
| Framing | COBS, 0x00 delimiter, 72-byte wire buffer |
| Rate | 50-100 Hz setpoints in, 50-100 Hz telemetry out |
| Timeout | 300 ms without a valid frame loses authority |

The mission computer never arms and never sets motor torque. Its setpoints are
clamped by the Pico exactly like the radio's.

### 4.4. IMU

- BNO055 in accelerometer-gyro fusion (`IMUPLUS`) by default: the magnetometer
  picks up motor and DC-DC fields and corrupts heading.
- 100 Hz output; the Pico integrates the calibrated gyro between fused samples at
  200 Hz.
- Calibration is saved to flash and restored at boot. A failed restore forces a
  `SELF_TEST` retry and blocks arming.

## 5. Task schedule

| Task | Rate | Trigger | Priority |
|---|---|---|---|
| Balance step | 200 Hz | Hardware timer / alarm | Highest |
| Attitude propagation | 200 Hz | Inside the balance step | Highest |
| Mainboard command egress | 200-400 Hz | Timer, from the latest control output | High |
| UART RX (CRSF, mainboard, mission) | per byte | DMA / interrupt | High |
| IMU poll and fusion | 100 Hz | Timer / I2C completion | Medium |
| Housekeeping (battery, thermal, counters) | 20-50 Hz | Timer | Low |
| Telemetry egress | 10-50 Hz | Timer | Low |
| Blackbox flush | on event / idle | Main loop | Lowest |

Rules:

- The balance step has a hard deadline; it never waits on I/O. UART and I2C
  results are consumed from fixed-size single-producer/single-consumer queues.
- No heap allocation. No blocking waits in the control path.
- The GP10 marker toggles once per balance step so jitter can be measured on a
  scope without instrumentation in the loop.
- The watchdog is fed only after the balance step completes, the IMU sample is
  fresh, and the mainboard link is valid.

## 6. State machine

| State | Torque | Entry | Exit |
|---|---|---|---|
| `BOOT` | zero | Reset | Peripherals up, calibration load attempted |
| `SELF_TEST` | zero | `BOOT` done | IMU, pack, charger, mainboard link, radio all pass |
| `IDLE` | zero | Self-test pass | Arm switch rising edge with the robot still |
| `BALANCE` | commanded | Armed and upright | Disarm, link loss with no recovery, tilt, fault |
| `DAMPING` | rate damping only | Mode switch or controlled stop request | Stop, then `IDLE` or `BALANCE` |
| `FAULT` | zero | Any critical fault | Power cycle plus a clean self-test |
| `E_STOP` | hardware off | Kill switch opened | Kill switch restored and the robot re-initialised |
| `SHUTDOWN` | zero | Operator request, critical battery | Latch released |

Fault codes are enumerated; every fault carries a severity and a Pico timestamp
and is reported over CRSF telemetry and the mission link.

## 7. Limits and arbitration

| Quantity | Limit | Action |
|---|---|---|
| Torque request | ±1000 units | Clamp, then count |
| Body tilt | 25-35 deg, configurable | Zero torque, latch `FAULT_TILT` |
| Pack voltage | 36.0 / 33.0 / 30.0 V | Warn / derate / stop |
| Command age | 100 ms radio, 100 ms mainboard, 300 ms mission | Zero setpoints, keep balancing |
| Charge present | Any time | Block arming and motor output |
| IMU age | 20 ms | Zero torque, latch fault |

Arbitration:

- Only the radio can arm.
- A deflected stick takes authority for 2 s after the last deflection.
- A valid mission frame within 300 ms may hold authority, but never outside the
  radio's clamps and never while disarmed.
- Losing every link means zero setpoints and continued balancing, not a fall.

## 8. Telemetry, debug, blackbox

- **CRSF telemetry:** pack voltage, link quality, state, fault code at 10-50 Hz.
- **Mission telemetry:** full state at 50-100 Hz, including attitude, wheel
  speeds, torque commands, counters, and fault history.
- **USB CDC:** human-readable console and a text command set for bench work.
- **`defmt` over RTT:** structured logs, disabled in release builds if it costs
  loop time.
- **Blackbox:** ring buffer of the last N seconds of setpoints, attitude, torque,
  feedback, and faults. Dumped over the mission link or USB on request; a
  truncated snapshot is better than none.
- **Counters exposed everywhere:** CRSF CRC errors, CRSF timeouts, mainboard CRC
  errors, mainboard timeouts, echoed-command mismatches, loop overruns, and the
  maximum observed loop latency.

### 8.1. USB console (implemented)

The Pico enumerates as a USB CDC-ACM device (`0x1209:0x0001`, product
`JBOT pico bridge`, serial `JBOT-PICO-01`). macOS exposes it as
`/dev/cu.usbmodem*`; the CDC transport ignores the baud rate:

```sh
python3 -m serial.tools.miniterm /dev/cu.usbmodemJBOT_PICO_011 115200
```

Protocol v1 is line-based ASCII: the host sends one command per line, the Pico
answers each command with one `ok`/`err`/`stat` line and, while streaming, sends
`T` telemetry lines. `LF` and `CR` both end a line; blank lines are ignored.

#### Commands

| Command | Reply | Notes |
|---|---|---|
| `help` / `?` | `ok help: ...` | Command list |
| `ping` | `ok ping` | Liveness check |
| `stat` | `stat up=... loop=... stream=... trim=... estop=... steps=... imu=... imu_err=... imu_code=... fb=... fb_drop=...` | Counters and settings |
| `stream` | `ok stream <hz>` | Query the current rate |
| `stream <hz>` | `ok stream <hz>` | `0` disables the stream, clamped to `0..=200` |
| `trim` | `ok trim <deg>` | Query the pitch trim (the balance/level reference) |
| `trim <deg>` | `ok trim <deg>` | Accepted range `-90..=90` |
| `zero` | `ok zero pitch=<deg> roll=<deg>` | Capture the current attitude as the zero offsets; the captured pose reads as 0 afterwards |
| `level` | `ok level <deg>` | Capture the current (zeroed) pitch as the balance reference; equivalent to `trim` with an automatic value |
| `mount` | `ok mount <n> <name>` | Query the BNO055 mount selector |
| `mount <n|name>` | `ok mount <n> <name>` | Select the mounting orientation of the BNO055 (`normal`, `flipx`, `flipy`, `rotxp`, `rotxm`, `rotyp`, `rotym`); re-capture `zero` and `level` afterwards. Also disarms the balance law and invalidates the captures |
| `arm` | `ok armed` | Arm the balance law; refused unless the e-stop is clear, the IMU is fresh, and `zero` and `level` have been captured this session |
| `disarm` | `ok disarmed` | Disarm the balance law |
| `speed` | `ok speed <mps>` | Query the speed setpoint |
| `speed <mps>` | `ok speed <mps>` | Accepted range `-5..=5` |
| `steer` | `ok steer <deg/s>` | Query the steering setpoint |
| `steer <deg/s>` | `ok steer <deg/s>` | Accepted range `-180..=180` |
| `reinit` | `ok reinit` | Re-runs the BNO055 start-up sequence |
| `estop` | `ok estop` | Latches the emergency stop |
| `clear` | `ok clear` | Clears the latch |
| `stats reset` | `ok stats reset` | Zeroes the diagnostic counters, keeps the settings |
| `reboot` | `ok reboot` | Flushes the reply, then resets the chip |

Rejected input gets `err <reason>`: `err unknown command: <text>`,
`err stream needs 0..200`, `err trim needs degrees in -90..90`,
`err speed needs mps in -5..5`, `err steer needs degrees/s in -180..180`,
`err mount needs normal, flipx, flipy, rotxp, rotxm, rotyp or rotym`,
`err arm refused: e-stop latched`, `err arm refused: IMU sample is stale`,
`err arm refused: capture zero first`, `err arm refused: capture level first`,
`err zero needs a fresh IMU sample`, `err level needs a fresh IMU sample`,
`err use: stats reset`, `err line too long` (over 64 bytes), or
`err line is not utf8`.

`stat` fields: `up` is uptime in seconds, `loop` the measured control rate,
`stream` the telemetry rate, `trim` the pitch trim in degrees, `estop` the
latch state, `steps` the control ticks since boot, `imu`/`imu_err` the IMU
sample and error counters, `imu_code` the last IMU error code,
`fb`/`fb_drop` the valid feedback frames and the bytes discarded while
resynchronising the parser, `zero` the applied pitch zero offset in degrees,
`zroll` the applied roll zero offset in degrees, `level` the balance reference
in degrees (same value as `trim`), and `mount` the BNO055 mount selector
(0 normal, 1 flip-x, 2 flip-y, 3 rot +90° X, 4 rot −90° X,
5 rot +90° Y, 6 rot −90° Y).

IMU error codes for `imu_code`: `0` none, `1` I2C transfer failure, `2` wrong
chip id, `3` the chip did not report ready.

#### Telemetry

The control task publishes one snapshot per 5 ms tick to a `Signal`; the console
task decides how often to print it, so the rate is independent of the loop.

```
T step=19043 loop=200 imu=ok cal=3 pitch=1.50 roll=-2.25 hdg=87.40 gy=0.06 gx=-0.12 gz=0.03 err=0x00 mb=ok L=0 R=0 bat=41.50 tmp=39.3 age=3 estop=0 vset=0.00 wset=0.00 mnt=1 zof=1.75 zro=-0.50 lvl=0.30
```

| Field | Meaning |
|---|---|
| `step` | Control ticks since boot, 200 per second |
| `loop` | Measured control rate, hertz |
| `imu` | `ok` when the sample is younger than 50 ms, otherwise `stale` |
| `cal` | `CALIB_STAT` system byte, `0..=3` |
| `pitch`, `gy` | Pitch and pitch rate used by the control law, degrees and °/s (zero offsets applied; mount-compensated) |
| `roll`, `hdg` | Roll from the fused Euler frame or the gravity vector (zero offset applied), and raw heading from the BNO055, degrees |
| `gx`, `gz` | Roll and yaw rate in the robot frame (mount-compensated), °/s |
| `err` | `SYS_ERR` register, hexadecimal; `0x00` is healthy |
| `mb` | `ok` once a feedback frame has arrived, otherwise `none` |
| `L`, `R` | Measured wheel speed, rpm |
| `bat`, `tmp` | Pack voltage and board temperature; `tmp` is scaled from EFeru decidegrees |
| `age` | Age of the last feedback frame in milliseconds, `4294967295` when none arrived |
| `estop` | Firmware emergency stop latch, `0` or `1` |
| `vset`, `wset` | Speed setpoint in m/s and steering setpoint in °/s |
| `mnt` | BNO055 mount selector: `0` normal, `1` flip-x (180° about X), `2` flip-y (180° about Y), `3` rot +90° X, `4` rot −90° X, `5` rot +90° Y, `6` rot −90° Y |
| `zof`, `zro` | Applied pitch and roll zero offsets, degrees |
| `lvl` | Applied balance (level) reference, degrees; the balance law sees `pitch - zof - lvl` |
| `arm` | Operator arm latch: `1` once `arm` succeeded this session, `0` at boot and after every e-stop, disarm, mount change, or zero/level capture |

Unknown keys are ignored by the host, so fields can be added without breaking a
client.

#### Mount compensation and attitude calibration

The BNO055 reports fused Euler angles in its own frame. When the module is
mounted sideways with the board plane vertical (the current chassis mount),
the fusion output cannot be used directly, so the firmware
re-expresses the gravity vector and the angular rates in the robot frame
(X forward, Y left, Z up) with sign flips selected by `mount`. On the current
frame the module measures ≈85–90° of Euler roll at the upright stance, which
identifies it as rotated +90° about the robot Y axis: `mount 5` (`rotyp`)
reads pitch ≈ 0 standing vertical and pitch > 0 leaning forward. Always
re-verify on the bench after re-mounting before arming:

| `mnt` | Meaning | Sensor → robot vector map |
|---|---|---|
| `0` | normal, aligned with the robot frame | `(x, y, z)` (Euler pitch/roll are used as-is) |
| `1` | **flip-x**: rotated 180° about the forward axis | `(x, -y, -z)` |
| `2` | **flip-y**: rotated 180° about the left axis | `(-x, y, -z)` |
| `3` | **rot +90° X**: rotated +90° about the forward axis | `(x, -z, y)` |
| `4` | **rot −90° X**: rotated −90° about the forward axis | `(x, z, -y)` |
| `5` | **rot +90° Y**: rotated +90° about the left axis | `(z, y, -x)` |
| `6` | **rot −90° Y**: rotated −90° about the left axis | `(-z, y, x)` |

For the flipped mounts the attitude is derived from the re-mapped gravity
vector with `atan2` (pitch = `atan2(gx, -gz)`, roll = `atan2(gy, -gz)`), which
is exact for any angle and does not depend on the device's Euler convention;
worst-case error of the approximation is about 0.1°. Gyro rates are remapped
with the same sign flips. Heading is not meaningful under a flip and is left
as reported.

Two captured references sit on top of the raw (mapped) attitude:

1. **Zero (`zero`)** — the pose the operator declares vertical. Capture it
   with the robot held exactly upright (against a level or plumb). After
   capture, telemetry `pitch`/`roll` read zero at that pose. Both offsets
   apply to the control path as well.
2. **Level (`level`)** — the stance in which the robot must balance. A
   wheeled biped usually balances with a slight lean, because the centre of
   mass is offset from the wheel axis. Capture it with the robot held at its
   balance stance; the balance law then sees `pitch - zof - lvl`.
   `level` is exactly `trim` with an automatically captured value; the manual
   `trim` slider remains for fine adjustment.

Calibration order: select `mount`, power-cycle or `reinit`, capture `zero`,
capture `level`, verify that telemetry reads ≈0 for both, then `arm`.

The balance law is additionally gated by an operator arm latch: at boot, and
after every e-stop, disarm, mount change, or zero/level capture, the torque
command is zero regardless of the gains. `arm` is refused unless the e-stop
is clear, the IMU sample is fresh, and both captures are done this session.
Zero and level are stored in RAM only and are lost on reboot, so the arming
sequence must be repeated after every power cycle; persistence to flash is
tracked as backlog M3-05/M3-42 and is deliberately not faked in this
revision.

#### Safety and timing

- Nothing in the console is on the control path. A missing host only means
  the lines are dropped, and a slow host cannot stall the 200 Hz loop.
- `estop` is a firmware latch: while it is set the control task sends
  `Command::ZERO` regardless of the IMU and the gains. It survives a USB
  reconnect and is cleared only by `clear` or a reset.
- The mainboard's own ~800 ms serial timeout remains the second line of
  defence; frames keep flowing at 200 Hz while the latch is set.
- `stream` accepts up to 200 Hz. Above a few tens of hertz the USB link, not the
  firmware, becomes the limit.

The bench application in `tools/pico` drives this protocol: telemetry panels,
event log, CSV recording, and the same commands from a keyboard. The native
`jbot-console` (`crates/jbot_console`) speaks the same protocol and adds the
attitude horizon and compass view.

## 9. Build, flash, debug

```sh
cd firmware/pico-bridge
cargo build --release                     # thumbv8m.main-none-eabihf
cargo run --release                       # flash via probe-rs, RTT attached
```

- Target: `thumbv8m.main-none-eabihf` (Cortex-M33 + FPU).
- The crate is its own Cargo workspace so the root workspace keeps building for
  the host. The root `.gitignore` ignores every `.cargo/config.toml`; the embedded
  target configuration needs an explicit exception.
- Flashing: `probe-rs` over SWD in development, UF2 over BOOTSEL as the fallback.
- Do not power the Pico from the probe and the robot rail at the same time.

## 10. Testing

| Level | What |
|---|---|
| Unit (host) | Attitude propagation, balance math, limit clamps, state transitions, CRC, COBS, CRSF parsing, EFeru frame encode/decode |
| Golden vectors | 8-byte command and 18/22-byte feedback frames captured on the bench, plus CRSF and mission-link frames |
| Loopback | Pico UARTs wired to a USB-UART adapter, checking framing and timeouts |
| Bench | Pico plus the real mainboard, wheels lifted, torque commands verified against feedback echoes |
| HIL | Full robot on a safety rig; jitter measured on GP10; fault injection for each timeout |
| Soak | One hour of balance with counters logged; no overrun, no watchdog reset |

Definition of done for a control change: unit tests pass, the HIL balance test
passes, no new clippy warnings, jitter and latency inside budget, and a rollback
path exists.

## 11. Open items

- Confirm the XR1 pad assignment (TX vs RX) before building the harness.
- Decide the charge-detect path: a status byte in the EFeru feedback frame or
  PA12 → GP11.
- Decide whether `ENABLE_ODOMETRY` is enabled at the start.
- Measure the mainboard command-to-torque delay and the loop jitter before tuning
  the gains.
- Decide whether a second PIO state machine or the second hardware UART is the
  better home for the mission link once the pin map is frozen.
