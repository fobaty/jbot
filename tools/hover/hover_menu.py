# MicroPython interactive menu for the hoverboard FOC firmware (VARIANT_USART).
#
# Terminal UI over the Pico's USB serial port: live telemetry, speed/steer keys,
# CSV logging, dead-man auto-stop. Keys are handled inside the script, so open
# the port with a plain terminal (screen / picocom / minicom), NOT `mpremote repl`
# (it sends Ctrl-C on connect and would stop the menu).
#
#   screen /dev/cu.usbmodemXXXX 115200      # quit screen with Ctrl-A then K
#
# Wiring: GP16 (TX) -> PA3, GP17 (RX) <- PA2, GND -> GND on the LEFT cable.
# Never connect the 15 V pin, USART2 is not 5 V tolerant.

import select
import struct
import sys
import time

from machine import Pin, UART

BAUD = 115200
START_FRAME = 0xABCD
CMD_FMT = "<HhhH"
FB_FMT = "<HhhhhhhHH"
FB_LEN = struct.calcsize(FB_FMT)

TICK_MS = 50            # 20 Hz command frames, well below the ~800 ms timeout
SLEW_SLOW = 2           # command units per tick -> 40 units/s
SLEW_FAST = 8           # -> 160 units/s
DEADMAN_S = 20          # auto ramp-down if nobody touches the keys
MAX_SPEED = 300
PRESETS = [10, 20, 30, 50, 80, 100, 150, 200, 300]
LOG_FILE = "hover_menu_log.csv"
WIDTH = 78

uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1), timeout=0, rxbuf=1024)
buf = bytearray()
STATS = {"bad": 0, "rx": 0}


def xor_checksum(fields):
    chk = 0
    for field in fields:
        chk ^= field & 0xFFFF
    return chk


def cmd_frame(steer, speed):
    steer = max(-1000, min(1000, int(steer)))
    speed = max(-1000, min(1000, int(speed)))
    return struct.pack(CMD_FMT, START_FRAME, steer, speed,
                       xor_checksum([START_FRAME, steer, speed]))


def poll_feedback():
    """Read the UART and return the validated feedback tuples of this tick."""
    global buf
    data = uart.read()
    if data:
        STATS["rx"] += len(data)
        buf.extend(data)
    frames = []
    i = 0
    while len(buf) - i >= 2:
        if buf[i] != 0xCD or buf[i + 1] != 0xAB:   # 0xABCD as little endian
            i += 1
            continue
        if len(buf) - i < FB_LEN:
            break
        fields = struct.unpack(FB_FMT, bytes(buf[i:i + FB_LEN]))
        i += FB_LEN
        if xor_checksum(fields[:-1]) != fields[-1]:
            STATS["bad"] += 1
            continue
        frames.append(fields)
    if i:
        buf = buf[i:]
    return frames


def approach(current, target, step):
    if target > current:
        return min(target, current + step)
    if target < current:
        return max(target, current - step)
    return current


def draw(lines):
    out = ["\x1b[H"]
    for line in lines:
        out.append(line[:WIDTH] + "\x1b[K\r\n")
    out.append("\x1b[J")
    sys.stdout.write("".join(out))


class State:
    def __init__(self):
        self.speed = 0           # target command
        self.steer = 0
        self.applied_speed = 0   # slew-limited command actually sent
        self.applied_steer = 0
        self.slew = SLEW_SLOW
        self.max_speed = MAX_SPEED
        self.last_input = time.ticks_ms()
        self.fps = 0
        self.frames = 0
        self.bad = 0
        self.rx = 0
        self.log_on = False
        self.log_file = None
        self.log_rows = 0
        self.quit = False
        self.feedback = None
        self.note = "w=+10  s=-10  a/d steer  space=stop"


def handle_keys(poller, state):
    keys = ""
    while poller.poll(0):
        ch = sys.stdin.read(1)
        if not ch:
            break
        keys += ch
    for c in keys:
        if c == "w":
            state.speed = min(state.max_speed, state.speed + 10)
        elif c == "s":
            state.speed = max(-state.max_speed, state.speed - 10)
        elif c == "d":
            state.steer = min(1000, state.steer + 50)
        elif c == "a":
            state.steer = max(-1000, state.steer - 50)
        elif c == " ":
            state.speed = 0
            state.steer = 0
        elif c == "f":
            state.slew = SLEW_FAST if state.slew == SLEW_SLOW else SLEW_SLOW
        elif c == "r":
            state.frames = 0
            STATS["rx"] = 0
            STATS["bad"] = 0
        elif c == "l":
            toggle_log(state)
        elif c == "q":
            state.quit = True
        elif c in "123456789":
            state.speed = PRESETS[int(c) - 1]
        else:
            continue
        state.last_input = time.ticks_ms()
    return keys


def toggle_log(state):
    if state.log_on:
        state.log_file.close()
        state.log_file = None
        state.log_on = False
        state.note = "log closed: %d rows on the Pico" % state.log_rows
        return
    state.log_file = open(LOG_FILE, "a")
    if state.log_rows == 0:
        state.log_file.write("t_ms,bat_v,temp_c,spd_l,spd_r,cmd1,cmd2,spd_cmd,steer_cmd\r\n")
    state.log_on = True
    state.note = "logging to %s" % LOG_FILE


def render(state, elapsed_s):
    f = state.feedback
    lines = []
    lines.append("HOVERBOARD FOC TEST    link %3d fps   bad %-3d rx %-7d %s"
                 % (state.fps, STATS["bad"], STATS["rx"], "LOG ON" if state.log_on else ""))
    lines.append("-" * WIDTH)
    if f is None:
        lines.append("  NO TELEMETRY - check wiring / board power")
        for _ in range(5):
            lines.append("")
    else:
        lines.append("  target    speed %+5d      steer %+5d"
                     % (state.speed, state.steer))
        lines.append("  applied   speed %+5d      steer %+5d      slew %d/tick"
                     % (state.applied_speed, state.applied_steer, state.slew))
        lines.append("")
        lines.append("  motor L  %6d rpm      motor R  %6d rpm" % (f[4], f[3]))
        lines.append("  battery  %6.2f V        temp   %5.1f C" % (f[5] / 100.0, f[6] / 10.0))
        lines.append("  led      0x%04X          cmd    speed %+5d steer %+5d"
                     % (f[7], state.applied_speed, state.applied_steer))
    if state.speed or state.steer:
        lines.append("  dead-man auto stop in %2d s" % (DEADMAN_S - int(elapsed_s)))
    else:
        lines.append("")
    lines.append("-" * WIDTH)
    lines.append("  w/s speed +-10     a/d steer -50/+50    SPACE stop")
    lines.append("  1..9 presets       f slew slow/fast    l log on/off")
    lines.append("  r reset stats      q quit              Ctrl-C emergency stop")
    lines.append("  %s" % state.note)
    draw(lines)


def main():
    state = State()
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    last_tick = time.ticks_ms()
    last_fps = time.ticks_ms()
    fps_count = 0
    draw(["\x1b[2J", "starting..."])
    total_rows = 0
    try:
        while not state.quit:
            now = time.ticks_ms()
            handle_keys(poller, state)

            # dead-man: ramp down when the keys stay untouched
            idle_s = time.ticks_diff(now, state.last_input) / 1000.0
            if idle_s > DEADMAN_S and (state.speed or state.steer):
                state.speed = 0
                state.steer = 0
                state.note = "dead-man stopped the motors"

            if time.ticks_diff(now, last_tick) >= TICK_MS:
                last_tick = now
                state.applied_speed = approach(state.applied_speed, state.speed, state.slew)
                state.applied_steer = approach(state.applied_steer, state.steer, state.slew)
                uart.write(cmd_frame(state.applied_steer, state.applied_speed))
                frames = poll_feedback()
                if frames:
                    state.feedback = frames[-1]
                    state.frames += len(frames)
                    fps_count += len(frames)
                if state.log_on and frames:
                    fr = frames[-1]
                    state.log_file.write("%d,%.2f,%.1f,%d,%d,%d,%d,%d,%d\r\n" % (
                        time.ticks_ms() & 0x7FFFFFFF, fr[5] / 100.0, fr[6] / 10.0,
                        fr[4], fr[3], fr[1], fr[2],
                        state.applied_speed, state.applied_steer))
                    state.log_rows += 1
                    state.log_file.flush()

            if time.ticks_diff(now, last_fps) >= 1000:
                state.fps = fps_count
                fps_count = 0
                last_fps = now
                render(state, idle_s)

            time.sleep_ms(5)
    except KeyboardInterrupt:
        state.note = "interrupted"
    finally:
        for _ in range(20):
            uart.write(cmd_frame(0, 0))
            time.sleep_ms(50)
        if state.log_on:
            state.log_file.close()
        sys.stdout.write("\x1b[H\x1b[J")
        print("stopped. frames %d, bad %d%s"
              % (state.frames, STATS["bad"],
                 ", log rows %d in %s" % (state.log_rows, LOG_FILE) if state.log_rows else ""))


main()
