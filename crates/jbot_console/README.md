# `jbot_console`

Native Rust bench console for the JBOT Pico 2 W USB CDC console. It renders the
BNO055 attitude live: an artificial horizon with a pitch ladder, a bank scale and
a compass, alongside the control-loop, power, and drive instruments. The same
window sends the `speed`, `steer`, `trim`, `stream`, `estop`, and `clear`
commands defined in [`docs/pico-bridge.md`](../../docs/pico-bridge.md) section
8.1.

The goal is a single tool for bench bring-up that replaces the scattered
`tools/pico` Python scripts: one window for attitude, telemetry, drive setpoints,
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

## Window

The layout is a fixed instrument board:

- **Attitude** — artificial horizon with 5° pitch ladder, ±60° bank scale,
  roll pointer, and a heading compass with cardinal marks.
- **Instruments** — `ATTITUDE` (pitch, roll, heading, rates, calibration),
  `BATTERY` (pack voltage, board temperature, feedback age), `WHEELS` (left and
  right rpm, average speed), and `CONTROL` (loop rate, steps, IMU state, e-stop
  latch).
- **Drive** — speed and steering setpoints with `Apply` and `Zero`, quick presets,
  a latched `E-STOP`, and `Clear`.
- **Link** — stream rate, `Ping` / `Stat` / `Help`, and `Reset stats`.
- **IMU / trim** — pitch trim and `Reinit BNO055`.
- **System** — `Reboot` behind an explicit confirmation.
- **Event log** — every reply, disconnect, and safety action with a timestamp.

Status chips show the connection state, whether telemetry is fresh, the IMU
calibration byte, and the emergency-stop latch.

## Keyboard

Shortcuts are ignored while a text field has focus.

| Key | Action |
|---|---|
| `↑` / `↓` | Speed +0.1 / −0.1 m/s |
| `←` / `→` | Steering −5° / +5° per second |
| `Space` | Zero speed and steering |
| `E` | Latch the firmware e-stop |
| `C` | Clear the e-stop latch |

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
zero. The drive setpoints and the emergency stop behave correctly, but the robot
will not move until the control gains are tuned and enabled in the firmware.
