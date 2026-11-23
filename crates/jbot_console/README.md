# `jbot_console`

Native Rust bench console for the JBOT Pico 2 W USB CDC console. It renders the
BNO055 attitude live: an artificial horizon with a pitch ladder, a bank scale and
a compass, alongside the control-loop, power, and drive instruments. The same
window sends the `speed`, `steer`, `trim`, `stream`, `estop`, and `clear`
commands defined in [`docs/pico-bridge.md`](../../docs/pico-bridge.md) section
8.1: the attitude horizon and compass, the drive setpoints, and the IMU
mount/zero/level calibration commands.

The goal is a single tool for bench bring-up that replaces the scattered
the archived `archive/python-tools/pico` Python scripts: one window for attitude, telemetry, drive setpoints,
and the firmware event log.

## Run

```bash
cargo run -p jbot_console                 # connect to the first Pico port
cargo run -p jbot_console -- --demo       # synthetic telemetry, no hardware
cargo run -p jbot_console -- --list       # list serial ports and exit
cargo run -p jbot_console -- --port /dev/cu.usbmodemJBOT_PICO_011 --stream 50
```

| Flag | Meaning |
|---|---|
| `--demo` | Synthetic attitude and telemetry so the UI runs without a board |
| `--port <path>` | Serial port to open; otherwise the first available Pico is used |
| `--stream <hz>` | Telemetry rate requested on connect, clamped to `1..=200` |
| `--list` | Print the detected serial ports and exit |
| `-h`, `--help` | Usage summary |

On connect the console requests the configured stream rate and sends `stat` to
populate the link panel. If the board rejects a command the reply is shown in the
event log as an error.

## Headless drivers

Two binaries share the same serial link for bench gates that the GUI cannot
score: `hold_sweep` (undisturbed 10 s hold over a `pscale`/`slew` ladder, see
`docs/pscale-ladder.md`) and `axis_check` (live sign and axis verification after
an IMU or wiring change, see `docs/axis-sign-check.md`). Both refuse to run
without `--run` and end with `estop`.

```bash
cargo run -q -p jbot_console --bin axis_check --release -- --run
cargo run -q -p jbot_console --bin hold_sweep --release -- --run --pscale 0.25 --slews 100
```

`hold_sweep --fixture-qual` runs three identical short releases and grades the
release fixture itself (see `docs/release-fixture.md`). `recovery_driver`
applies a software `home` disturbance instead of a manual release, so gain
comparisons are not contaminated by the operator's hand:

```bash
cargo run -q -p jbot_console --bin recovery_driver --release -- \
  --run --snap-ref --trials 1 --angle 1 --hold-ms 1000 \
  --settle-ms 2000 --pscale 0.30 --slew 100
```

## Runtime firmware knobs

| Command | Meaning |
|---|---|
| `pscale <0..2>` | Scale on all angular-output gains; runtime-only |
| `dscale <0..2>` | Scale on the pitch derivative gain only; runtime-only |
| `db <Nm\|off>` | Drive dead-zone compensation; runtime-only |
| `slew <100..10000>` | Torque ramp rate in command units per second |
| `dbg on\|off` | Bridge the STM32 debug serial (USART3) to this console |
| `dbg <text>` | Send a line to the STM32 debug serial |

`stat` reports `pscale`, `dscale`, `db`, `slew`. The debug bridge uses a PIO
UART on GP12 (RX) and GP13 (TX); wire them to STM32 PB10/PB11 on the right
cable with a common ground. It is a bench tool and never touches the control
link on UART0. See `docs/drive-diagnosis-2026-11-23.md`.

## Window

The layout is a fixed instrument board:

- **Attitude** — artificial horizon with 5° pitch ladder, ±60° bank scale,
  roll pointer, and a heading compass with cardinal marks. Both dials use a
  compact square footprint, with the HDG/TILT and gyro readouts beside them.
- **Instruments** — `ATTITUDE` (pitch, roll, heading, rates, calibration),
  `BATTERY` (pack voltage, board temperature, feedback age), `WHEELS` (left and
  right rpm, average speed), and `CONTROL` (loop rate, steps, IMU state, e-stop
  latch).
- **Drive** — speed and steering setpoints with `Apply` and `Zero`, quick presets,
  a latched `E-STOP`, and `Clear`.
- **Motor test (bench)** — a limited direct wheel-torque check: torque slider
  and presets in EFeru command units (±300 clamp, ramped start, 5 s auto-stop),
  refused while the e-stop is latched or the balance law is armed, with a live
  read-out of the applied per-wheel torque and a latching `STOP (0)` button.
  Built for held-robot bring-up; not a drive mode.
- **Link** — stream rate, `Ping` / `Stat` / `Help`, and `Reset stats`.
- **Calibration** — BNO055 mount selector (`normal` / `flip X` / `flip Y` /
  `rot ±90° X/Y`) or the guided **axis calibration**: stand the robot
  upright, lean it forward, backward, left and right, pressing the matching
  button after each pose; the console sweeps all mount selectors and applies
  the one whose attitude response matches the motion (a score table shows
  the verdict). **SET HOME** captures the balance reference for you (one button):
  hold the robot UPRIGHT until the console reports it still (all three gyro
  axes below 1.5 °/s for a full window), then hold the BALANCE STANCE the
  same way; the console issues `zero` and `level` itself at the quiet moment,
  so a capture never lands mid-swing. Manual `Calibrate ZERO` / `Calibrate
  LEVEL` remain as fallback. Also an `ARM BALANCE` / `DISARM` gate, a level
  trim slider, and `Reinit BNO055`. Arming is refused until zero and level
  are captured.
- **System** — `Reboot` behind an explicit confirmation, and **Record session → CSV**:
  writes two files with a shared millisecond clock into `csv/` at the repository
  root, ready to overlay operator actions on robot state for post-incident analysis:
  - `csv/jbot-telemetry-<unix>.csv` — one row per telemetry sample: attitude, rates,
    wheel rpm, pack voltage, setpoints, applied balance torque (`torque_units`),
    the balance reference, outer-loop demand, learned slope trim, travel since
    arming, balance mode, spin-test torque, estop.
  - `csv/jbot-actions-<unix>.csv` — one row per operator-visible event: every command
    sent (`tx`/`tx-drop`/`tx-down`), every non-telemetry reply (`rx`), connection
    events (`sys`), and demo/recording toggles (`ui`).
  Recording is independent of demo mode, so the recorder can be exercised without
  hardware. Firmware telemetry `trq` (applied balance torque) appears in the
  recording after the next firmware flash.
- **Always-on event log** — in addition to the in-window event panel, every line
  the console logs is appended to `logs/jbot-events.log` at the repository root
  (`unix_ms TAG text`, one line per event, flushed per line). No button required:
  post-incident analysis does not depend on remembering to press REC.
- **Event log** — every reply, disconnect, and safety action with a timestamp.

Status chips show the connection state, whether telemetry is fresh, the IMU
calibration byte, the mount selector, the zero and level offsets, and the
emergency-stop latch.

## Keyboard

Shortcuts are ignored while a text field has focus.

| Key | Action |
|---|---|
| `↑` / `↓` | Speed +0.1 / −0.1 m/s |
| `←` / `→` | Steering −5° / +5° per second |
| `Space` | Zero speed and steering |
| `E` | Latch the firmware e-stop |
| `C` | Clear the e-stop latch |
| `Z` | Calibrate zero (capture current attitude as offsets) |
| `L` | Calibrate level (capture balance stance as the reference) |

## Architecture

```
UI thread (egui)          serial thread
─────────────────         ─────────────
ConsoleApp  ──bounded──▶  command queue ──▶ USB CDC port
     ▲                    │
     └──bounded event◀────┘  parser ──▶ LinkEvent (line, telemetry, stat, error)
```

- `app.rs` — application state, panels, commands, demo generator.
- `attitude.rs` — horizon, compass, and instrument rendering.
- `link.rs` — the dedicated serial thread, bounded queues, reconnect loop, and
  the safety commands written on every disconnect.
- `protocol.rs` — tolerant line parser for `T`, `stat`, `ok`, `err`, and `JBOT`
  lines, with unit tests.

The UI never performs blocking I/O. Telemetry is loss-tolerant; a stalled queue
drops the newest sample rather than replaying stale state after a reconnect.
Closing the window, unplugging the board, or a write error latches the firmware
`estop` and sends `speed 0` and `steer 0` before the port is released.

## Tests

```bash
cargo test -p jbot_console
```

The protocol parser is covered for telemetry, diagnostics, replies, malformed
values, and unknown keys.

## Note on motion

The firmware currently runs with `Gains::SAFE`, so the torque command stays at
zero: **the wheels cannot spin, regardless of the speed and steering
setpoints sent from this console.** The drive setpoints and the emergency stop
behave correctly, but the robot will not move until the control gains are
tuned on the safety rig and enabled in the firmware. Balance gains are never
enabled silently; the bench procedure lives in `docs/pico-bridge.md` section
10.
