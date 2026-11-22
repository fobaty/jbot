# Hoverboard bench console

Two pieces that turn the donor hoverboard into something you can drive from the
Mac while it sits on the bench:

| File | Runs on | Role |
|---|---|---|
| `hover_bridge.py` | Pico 2 W (`:main.py`) | owns the 20 Hz motor frame stream, forwards telemetry |
| `hover_drive.py` | Mac (terminal UI) | keyboard control, live telemetry, CSV log, e-stop |

```
Mac ── USB ── Pico 2 W ── UART0 ── hoverboard mainboard (USART2, left cable)
```

The mainboard must run the `VARIANT_USART` bench build; build and flash
instructions are in `firmware/README.md` section 2.9.

## Wiring

| Pico | Hoverboard left cable |
|---|---|
| GP16 (pin 21, TX) | PA3 (RX) |
| GP17 (pin 22, RX) | PA2 (TX) |
| GND (pin 23, or any GND) | GND |

Never connect the 15 V pin: the mainboard's USART2 is not 5 V tolerant.

## One-time setup

```bash
# Pico side: MicroPython must already be flashed, then
export PATH="$HOME/.local/bin:$PATH"
PORT=$(ls /dev/cu.usbmodem* | head -1)
mpremote connect $PORT fs cp archive/python-tools/hover/hover_bridge.py :hover_bridge.py
mpremote connect $PORT fs cp archive/python-tools/hover/hover_bridge.py :main.py
mpremote connect $PORT reset

# Mac side
python3 -m pip install --user pyserial   # only once
```

`main.py` starts the bridge on every power-up, so plugging the Pico in is enough.

## Running

```bash
python3 archive/python-tools/hover/hover_drive.py                    # autodetect the port
python3 archive/python-tools/hover/hover_drive.py --port /dev/cu.usbmodem2101 --max 200
python3 archive/python-tools/hover/hover_drive.py --log run1.csv     # -> csv/run1.csv
```

Log names are resolved into `csv/` at the repository root by
`archive/python-tools/bench_paths.py`; an absolute path is used as given. `hover_menu.py` runs
on the Pico itself, so its `hover_menu_log.csv` stays on the Pico filesystem.

```
  HOVERBOARD DRIVE   bridge:ok  link  98 fps  ok 4932   bad 0    log off
 =======================================================================
   THROTTLE  target   +20   applied   +16   #-----------------------
   STEER     target    +0   applied    +0   ------------------------

   MOTOR L      18 rpm  #-------------------
   MOTOR R      17 rpm  #-------------------

   BATTERY  41.82 V    TEMP  34.7 C    LED 0x0000
 =======================================================================
   dead-man auto stop in 19.6 s    slew 2/tick    step 10
   Up/W forward    Down/S back    Left/A + Right/D steer
   1..9 presets    +/- step       Space soft stop    X/Esc E-STOP
   F slew  L log  R reset link  Q quit
```

`target` is what the keys ask for, `applied` is what actually leaves the board:
it is slew limited so the wheels never jump.

| Key | Action |
|---|---|
| `Up` / `W` | forward, target speed `+step` |
| `Down` / `S` | backward, target speed `-step` |
| `Right` / `D` | steer right (`+step*5`) |
| `Left` / `A` | steer left (`-step*5`) |
| `1` .. `9` | presets 10, 20, 30, 50, 80, 100, 150, 200, 300 |
| `+` / `-` | speed step 1, 5, 10, 20, 50, 100 |
| `Space` | soft stop: targets to zero, still armed |
| `X` / `Esc` | **emergency stop**: zero frames immediately, latched |
| `C` | clear the emergency stop |
| `F` | slew rate 40 units/s (slow) / 160 units/s (fast) |
| `L` | CSV logging on/off |
| `R` | reset the link counters |
| `Q` | quit, stop frames are always sent |

Units: `1000` command units is `N_MOT_MAX` (1000 rpm), so `+20` is roughly
20 rpm. In the current `VLT_MODE` build very small commands (under ~30) can stall
on cogging; switch to `SPD_MODE` if you need slow but reliable motion (see
`Inc/config.h`, `CTRL_MOD_REQ`).

## Safety layers

1. **Emergency stop** (`X`): the console sends `S` at once, the bridge emits six
   zero frames and zeroes its stored command; the latch blocks motion keys until
   `C`.
2. **Bridge watchdog**: no `C` line for 500 ms and the bridge keeps sending zeros.
3. **Firmware timeout**: the mainboard stops by itself after ~800 ms without
   frames (`SERIAL_TIMEOUT`), so a pulled cable, a crashed console or a killed
   process all end with the motors stopped.
4. **Dead-man**: 20 s without a key press ramps the targets back to zero.
5. **Slew limiting**: 2 units per 50 ms tick in slow mode.

## Logging

`L` (or `--log file.csv`) writes one row per UI tick:

```
t_s,bat_v,temp_c,spd_l,spd_r,cmd_speed,cmd_steer,target_speed,target_steer
1759.42,41.82,34.7,18,17,16,0,20,0
```

## Protocol (for debugging)

Host → Pico: `C <steer> <speed>`, `S` (stop), `Q` (banner), `R` (reset counters).
Pico → host: `H HOVERBRIDGE 1`, `T <spdL> <spdR> <bat*100> <temp*10> <led>`,
`K <uptime> <steer> <speed> <fps> <ok> <bad>`.

```bash
# watch the raw stream without the UI
python3 - <<'PY'
import serial
s = serial.Serial("/dev/cu.usbmodem2101", 115200, timeout=1)
s.write(b"Q\n")
for _ in range(10):
    print(s.readline())
PY
```

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `cannot open ...: it may be in use` | another program holds the port; close `screen` (`Ctrl-A`, `K`, `y`) |
| `bridge is not answering` | `main.py` on the Pico is not the bridge; re-upload it |
| `NO TELEMETRY`, `link 0 fps` | hoverboard is off, or the UART wiring/power is wrong |
| `cmd` moves, `spdL/spdR` stay 0 | too small a command for `VLT_MODE`, cogging; raise the speed or switch to `SPD_MODE` |
| `bad` counter grows | UART noise; check GND between the Pico and the mainboard |

## Going back to the on-device menu

The older `hover_menu.py` (TUI drawn on the Pico itself, opened with `screen`)
still works:

```bash
mpremote connect $PORT fs cp :hover_menu.py :main.py   # menu on boot again
mpremote connect $PORT fs cp :hover_bridge.py :main.py # bridge again
mpremote connect $PORT fs rm :main.py                  # no autostart at all
```
