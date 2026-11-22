# Pico 2 W bench console

Laptop-side application for the JBOT Pico 2 W bridge firmware. It shows live
telemetry, keeps a timestamped event log, records CSV, and sends the console
commands from `docs/pico-bridge.md` section 8.1.

```
Mac ── USB CDC ── Pico 2 W ── UART0 ── STM32F103 mainboard
                              └─ I2C1 ── BNO055
```

| File | Role |
|---|---|
| `jbot_link.py` | Protocol layer: serial link, `T`/`stat`/`ok`/`err` parsing, event log, CSV writer. No UI. |
| `jbot_console.py` | curses application plus a `--no-ui` plain-text mode |
| `test_jbot_link.py` | Unit tests for the protocol layer |

The firmware side is described in `firmware/README.md` section 5.8.

## Setup

```sh
python3 -m pip install pyserial
```

Flash the Rust firmware first (section 5.8 of `firmware/README.md`). The
MicroPython `archive/python-tools/hover` build and this application cannot both own the port.

## Running

```sh
python3 archive/python-tools/pico/jbot_console.py                        # autodetect, TUI
python3 archive/python-tools/pico/jbot_console.py --port /dev/cu.usbmodemJBOT_PICO_011
python3 archive/python-tools/pico/jbot_console.py --list                 # show the port
python3 archive/python-tools/pico/jbot_console.py --stream 20 --csv run.csv
python3 archive/python-tools/pico/jbot_console.py --no-ui                # plain text
```

`--stream` is the telemetry rate requested on connect (`0..200` Hz, default
10). The Pico starts at 1 Hz after a reset, so the application sets the rate
itself on every connect. Bare filenames are resolved by `archive/python-tools/bench_paths.py`:
`--csv run.csv` writes `csv/run.csv`, and `--events file.log` writes
`logs/file.log`. An absolute path is used as given.

The port name comes from the USB serial number `JBOT-PICO-01`, so it survives
re-enumeration as `/dev/cu.usbmodemJBOT_PICO_011`. If several USB serial
devices are attached, pass `--port` explicitly.

## Keys

| Key | Action |
|---|---|
| `:` | type any command, `Enter` sends it |
| `s` | `stream <hz>` prompt |
| `t` | `trim <deg>` prompt |
| `p` | `ping` |
| `i` | `stat`, shown as one readable line in the log |
| `r` | `reinit`, re-runs the BNO055 start-up sequence |
| `e` | `estop`, latches the firmware emergency stop |
| `c` | `clear`, releases the latch |
| `L` | start/stop CSV logging |
| `R` | `reboot` the Pico, asks for confirmation |
| `x` | clear the on-screen event log |
| `q` | quit; the application sends `estop` before closing the port |

## Panels

- **RUNTIME** — uptime from `step / loop`, measured control rate, requested
  stream rate, e-stop state (red when latched).
- **IMU** — freshness, `CALIB_STAT` system byte, pitch, pitch rate, `SYS_ERR`.
- **MAINBOARD** — feedback presence, wheel speeds, pack voltage, board
  temperature, feedback age.
- **EVENTS** — timestamped log; `>` echoes what was sent, `ok`/`err` are
  firmware replies, warnings cover stale telemetry, disconnects and firmware
  restarts.

The application reconnects by itself every 2 s after a disconnect, and requests
`ping`, `stat` and `stream` again on every successful connect. A step counter
that goes backwards is reported as a firmware restart, so a reset during a run
is visible in the log.

## Logging

`--csv path` (or `L` at runtime) writes one row per telemetry sample with the
host timestamp first:

```
t_s,step,loop_hz,imu,cal,pitch_deg,gyro_dps,sys_err,mainboard,rpm_left,rpm_right,battery_v,board_temp_c,feedback_age_ms,estop
```

Rows are flushed as they are written, so a killed process loses nothing already
on screen. `--events path` writes `unix_time level text` lines into `logs/`.

## Headless mode

`--no-ui` prints every event and each new telemetry sample, and reads commands
from stdin, one per line:

```sh
printf 'stat\nstream 5\n' | python3 archive/python-tools/pico/jbot_console.py --no-ui
```

This is the mode to use from a script, a `tmux` session on the robot, or a
soak test: it has no terminal requirements and keeps the same safety behaviour
(`estop` on exit).

## Tests

```sh
python3 -m unittest discover -s archive/python-tools/pico -p 'test_*.py'
```

The suite covers telemetry and `stat` parsing, line classification, the event
log ring buffer and the CSV row shape. It needs no hardware: `Link._parse()` is
fed captured protocol lines.

## Safety

1. **Firmware e-stop**: `estop` latches inside the firmware, so it holds even if
   the cable is pulled or the console dies. Only `clear` or a reset releases it.
2. **Mainboard timeout**: the F103 stops on its own ~800 ms after the frames
   stop, so a hung Pico also ends with the motors off.
3. **Quit sends `estop`**: both the TUI and the headless loop latch the stop on
   exit, before closing the port. A new session therefore starts with `estop=1`
   after a clean quit; press `c` to clear it once the bench is ready.
4. **Zero gains**: the currently flashed image computes the balance law with
   `Gains::SAFE`, so every command frame carries zero torque regardless of what
   the console sends. The application has no arming or torque commands on
   purpose; motion control arrives with the radio link.
