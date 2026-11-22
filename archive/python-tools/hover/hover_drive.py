#!/usr/bin/env python3
"""Interactive drive console for the EFeru hoverboard firmware (VARIANT_USART).

The Raspberry Pi Pico 2 W in between runs `hover_bridge.py`: it owns the 20 Hz
motor frame stream and pushes telemetry back over USB serial. This console only
sends readable text commands (`C <steer> <speed>`, `S`, `Q`, `R`), so the whole
UI lives on the Mac side.

Usage:
    python3 hover_drive.py                          # autodetect the Pico
    python3 hover_drive.py --port /dev/cu.usbmodem2101 --max 200
    python3 hover_drive.py --log run1.csv           # start logging right away

Keys:
    Up / W        forward      (target speed + step)
    Down / S      backward     (target speed - step)
    Left / A      steer left   (target steer - step)
    Right / D     steer right  (target steer + step)
    1 .. 9        speed presets 10, 20, 30, 50, 80, 100, 150, 200, 300
    + / -         bigger / smaller speed step
    Space         soft stop (targets to zero, still armed)
    X / Escape    EMERGENCY STOP, zero frames now, latched
    C             clear the emergency stop
    F             slew rate slow (40 u/s) / fast (160 u/s)
    L             CSV logging on/off
    R             reset the link counters
    Q             quit (stop frames are always sent)
"""

import argparse
import curses
import glob
import os
import sys
import time

# Host tools live one level below the repository root; CSV captures belong in
# `csv/` there, not in whatever directory the tool was started from.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bench_paths import csv_path

try:
    import serial
except ImportError:  # pragma: no cover - depends on the host setup
    sys.exit("pyserial is missing: python3 -m pip install pyserial")

PRESETS = (10, 20, 30, 50, 80, 100, 150, 200, 300)
STEPS = (1, 5, 10, 20, 50, 100)
SLEW_SLOW = 2          # command units per 50 ms tick -> 40 units/s
SLEW_FAST = 8          # -> 160 units/s
DEADMAN_S = 20.0       # auto stop when the keyboard stays idle
CMD_PERIOD = 0.05      # 20 Hz command stream
UI_PERIOD = 0.05
STALE_TELEM_S = 1.0
STALE_BRIDGE_S = 2.0

PAIR_HEAD, PAIR_OK, PAIR_WARN, PAIR_STOP, PAIR_DIM = 1, 2, 3, 4, 5


def approach(current, target, step):
    if target > current:
        return min(target, current + step)
    if target < current:
        return max(target, current - step)
    return current


def bar(value, span, width):
    """Left anchored ASCII bar, saturating at `span`."""
    if span <= 0:
        return "-" * width
    filled = int(round(min(1.0, abs(value) / float(span)) * width))
    filled = max(0, min(width, filled))
    return "#" * filled + "-" * (width - filled)


def autodetect_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no /dev/cu.usbmodem* found - is the Pico plugged in?")
    if len(ports) > 1:
        print("several ports, using %s (%s)" % (ports[0], ", ".join(ports)))
    return ports[0]


class Link:
    """USB serial link to the Pico bridge."""

    def __init__(self, port, baud):
        self.port = port
        self.serial = serial.Serial(port, baud, timeout=0)
        self.buffer = b""
        self.bridge = False
        self.telemetry = None            # (spd_l, spd_r, battery_v, temp_c, led)
        self.telemetry_at = 0.0
        self.last_line_at = 0.0
        self.fps = 0
        self.ok = 0
        self.bad = 0
        self.applied = (0, 0)            # steer, speed the bridge reports sending

    def close(self):
        try:
            self.serial.close()
        except Exception:
            pass

    def send(self, text):
        try:
            self.serial.write(("%s\n" % text).encode())
        except Exception:
            pass

    def command(self, steer, speed):
        self.send("C %d %d" % (steer, speed))

    def stop(self):
        self.send("S")

    def query(self):
        self.send("Q")

    def reset(self):
        self.send("R")

    def poll(self):
        waiting = self.serial.in_waiting
        if not waiting:
            return 0
        self.buffer += self.serial.read(waiting)
        count = 0
        while b"\n" in self.buffer:
            raw, self.buffer = self.buffer.split(b"\n", 1)
            self._handle(raw.decode("ascii", "replace").strip())
            count += 1
        return count

    def _handle(self, line):
        if not line:
            return
        parts = line.split()
        now = time.monotonic()
        if parts[0] == "T" and len(parts) >= 6:
            self.telemetry = (int(parts[1]), int(parts[2]),
                              int(parts[3]) / 100.0, int(parts[4]) / 10.0,
                              int(parts[5]))
            self.telemetry_at = now
            self.last_line_at = now
        elif parts[0] == "K" and len(parts) >= 7:
            self.applied = (int(parts[2]), int(parts[3]))
            self.fps = int(parts[4])
            self.ok = int(parts[5])
            self.bad = int(parts[6])
            self.bridge = True
            self.last_line_at = now
        elif parts[0] == "H":
            self.bridge = True
            self.last_line_at = now

    def telemetry_age(self):
        if self.telemetry is None:
            return 1e9
        return time.monotonic() - self.telemetry_at

    def bridge_alive(self):
        return time.monotonic() - self.last_line_at < STALE_BRIDGE_S


class Drive:
    """Command state: what the operator asks for versus what leaves the board."""

    def __init__(self, max_speed):
        self.target_speed = 0
        self.target_steer = 0
        self.applied_speed = 0
        self.applied_steer = 0
        self.max_speed = max_speed
        self.step = 10
        self.slew = SLEW_SLOW
        self.estop = False
        self.last_key = time.monotonic()
        self.note = "ready"

    def say(self, text):
        self.note = text

    def touch(self):
        self.last_key = time.monotonic()

    def clamp(self):
        self.target_speed = max(-self.max_speed,
                                min(self.max_speed, self.target_speed))
        self.target_steer = max(-1000, min(1000, self.target_steer))

    def advance(self):
        self.applied_speed = approach(self.applied_speed, self.target_speed,
                                      self.slew)
        self.applied_steer = approach(self.applied_steer, self.target_steer,
                                      self.slew)

    def soft_stop(self):
        self.target_speed = 0
        self.target_steer = 0
        self.say("soft stop")

    def emergency(self):
        self.estop = True
        self.target_speed = 0
        self.target_steer = 0
        self.applied_speed = 0
        self.applied_steer = 0
        self.say("EMERGENCY STOP latched - press C to clear")

    def clear_estop(self):
        self.estop = False
        self.say("e-stop cleared")

    def busy(self):
        return bool(self.target_speed or self.target_steer)


class Session:
    """Everything the renderer needs."""

    def __init__(self, link, drive):
        self.link = link
        self.drive = drive
        self.log_file = None
        self.log_rows = 0
        self.log_path = None
        self.quit = False


def toggle_log(session, path=None):
    drive = session.drive
    if session.log_file is not None:
        session.log_file.close()
        session.log_file = None
        drive.say("log closed: %d rows in %s" % (session.log_rows, session.log_path))
        return
    session.log_path = csv_path(path or "hover_drive_log.csv")
    session.log_file = open(session.log_path, "a")
    if session.log_rows == 0:
        session.log_file.write(
            "t_s,bat_v,temp_c,spd_l,spd_r,cmd_speed,cmd_steer,"
            "target_speed,target_steer\n")
    drive.say("logging to %s" % session.log_path)


def log_row(session, tel):
    if session.log_file is None or tel is None:
        return
    spd_l, spd_r, bat, temp, _led = tel
    session.log_file.write("%.2f,%.2f,%.1f,%d,%d,%d,%d,%d,%d\n" % (
        time.time(), bat, temp, spd_l, spd_r,
        session.drive.applied_speed, session.drive.applied_steer,
        session.drive.target_speed, session.drive.target_steer))
    session.log_rows += 1
    session.log_file.flush()


def handle_key(ch, session):
    """Apply one key press. Returns False when the app should quit."""
    drive = session.drive
    link = session.link

    if ch in (ord("q"), ord("Q")):
        return False
    if ch in (ord("x"), ord("X"), 27):            # 27 == Escape
        drive.emergency()
        link.stop()
        link.command(0, 0)
        curses.beep()
        drive.touch()
        return True
    if ch in (ord("c"), ord("C")):
        drive.clear_estop()
        drive.touch()
        return True
    if drive.estop and ch not in (ord("l"), ord("L"), ord("f"), ord("F"),
                                  ord("r"), ord("R")):
        drive.say("E-STOP latched - press C to clear")
        drive.target_speed = 0
        drive.target_steer = 0
        return True

    if ch in (curses.KEY_UP, ord("w"), ord("W")):
        drive.target_speed += drive.step
    elif ch in (curses.KEY_DOWN, ord("s"), ord("S")):
        drive.target_speed -= drive.step
    elif ch in (curses.KEY_RIGHT, ord("d"), ord("D")):
        drive.target_steer += drive.step * 5
    elif ch in (curses.KEY_LEFT, ord("a"), ord("A")):
        drive.target_steer -= drive.step * 5
    elif ch == ord(" "):
        drive.soft_stop()
    elif ch in (ord("+"), ord("=")):
        drive.step = STEPS[min(len(STEPS) - 1, STEPS.index(drive.step) + 1)]
        drive.say("speed step %d" % drive.step)
    elif ch in (ord("-"), ord("_")):
        drive.step = STEPS[max(0, STEPS.index(drive.step) - 1)]
        drive.say("speed step %d" % drive.step)
    elif ord("1") <= ch <= ord("9"):
        sign = -1 if drive.target_speed < 0 else 1
        drive.target_speed = PRESETS[ch - ord("1")] * sign
        drive.say("preset %d" % drive.target_speed)
    elif ch in (ord("f"), ord("F")):
        drive.slew = SLEW_FAST if drive.slew == SLEW_SLOW else SLEW_SLOW
        drive.say("slew %d units/tick" % drive.slew)
    elif ch in (ord("l"), ord("L")):
        toggle_log(session)
    elif ch in (ord("r"), ord("R")):
        link.reset()
        drive.say("link counters reset")
    else:
        return True

    drive.clamp()
    drive.touch()
    return True


def render(scr, session):
    drive, link = session.drive, session.link
    height, width = scr.getmaxyx()
    scr.erase()
    if height < 16 or width < 70:
        scr.addnstr(0, 0, "window too small: need at least 70x16",
                    max(0, width - 1))
        scr.refresh()
        return

    w = min(width - 2, 92)
    x0 = max(0, (width - w) // 2)
    tel = link.telemetry
    fresh = link.telemetry_age() < STALE_TELEM_S
    alive = link.bridge_alive()
    hot = drive.estop
    body = PAIR_STOP if hot else (PAIR_WARN if drive.busy() else PAIR_OK)

    def put(row, text, attr=0, col=0):
        # addnstr(y, x, text, n, attr) - n must be the length, not the attribute
        if row >= height - 1:
            return
        clip = max(0, w - col - 1)
        if clip <= 0:
            return
        text = text[:clip]
        scr.addnstr(row, x0 + col, text, len(text), attr)

    head = " HOVERBOARD DRIVE   bridge:%s  link %3d fps  ok %-6d bad %-3d  %s" % (
        "ok" if alive else "??", link.fps, link.ok, link.bad,
        ("LOG %d rows" % session.log_rows) if session.log_file else "log off")
    put(0, head.ljust(w), PAIR_HEAD)
    put(1, "=" * w, PAIR_DIM)

    if not fresh:
        put(2, "  NO TELEMETRY  - hoverboard off, or a wiring / power problem",
            PAIR_WARN)
        for row in range(3, 10):
            put(row, "", PAIR_DIM)
    else:
        spd_l, spd_r, bat, temp, led = tel
        put(2, "  THROTTLE  target %+5d   applied %+5d   %s" % (
            drive.target_speed, drive.applied_speed,
            bar(drive.applied_speed, drive.max_speed, 24)), body)
        put(3, "  STEER     target %+5d   applied %+5d   %s" % (
            drive.target_steer, drive.applied_steer,
            bar(drive.applied_steer, 1000, 24)), body)
        put(4, "")
        put(5, "  MOTOR L  %6d rpm  %s" % (spd_l, bar(spd_l, drive.max_speed, 20)),
            PAIR_OK)
        put(6, "  MOTOR R  %6d rpm  %s" % (spd_r, bar(spd_r, drive.max_speed, 20)),
            PAIR_OK)
        put(7, "")
        bat_attr = PAIR_OK if bat > 36.0 else PAIR_WARN
        put(8, "  BATTERY %6.2f V    TEMP %5.1f C    LED 0x%04X" % (
            bat, temp, led), PAIR_WARN if temp > 55.0 else bat_attr)
        if not alive:
            put(9, "  bridge is not answering - is hover_bridge.py running on the Pico?",
                PAIR_STOP)

    put(10, "=" * w, PAIR_DIM)

    if hot:
        status = "  *** E-STOP LATCHED - motors forced to zero, press C to clear ***"
        status_attr = PAIR_STOP
    elif drive.busy():
        left = DEADMAN_S - (time.monotonic() - drive.last_key)
        status = "  dead-man auto stop in %4.1f s    slew %d/tick    step %d" % (
            max(0.0, left), drive.slew, drive.step)
        status_attr = PAIR_WARN
    else:
        status = "  idle    slew %d/tick    step %d" % (drive.slew, drive.step)
        status_attr = PAIR_DIM
    put(11, status, status_attr)

    put(12, "  Up/W forward    Down/S back    Left/A + Right/D steer", PAIR_DIM)
    put(13, "  1..9 presets    +/- step       Space soft stop    X/Esc E-STOP",
        PAIR_DIM)
    put(14, "  F slew  L log  R reset link  Q quit", PAIR_DIM)
    put(15, "  %s" % drive.note, PAIR_OK)
    scr.refresh()


def init_colors():
    try:
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(PAIR_HEAD, curses.COLOR_CYAN, -1)
        curses.init_pair(PAIR_OK, curses.COLOR_GREEN, -1)
        curses.init_pair(PAIR_WARN, curses.COLOR_YELLOW, -1)
        curses.init_pair(PAIR_STOP, curses.COLOR_RED, -1)
        curses.init_pair(PAIR_DIM, curses.COLOR_WHITE, -1)
    except curses.error:
        pass


def run(scr, session, log_path=None):
    curses.curs_set(0)
    scr.nodelay(True)
    scr.keypad(True)
    init_colors()
    session.link.query()
    if log_path:
        toggle_log(session, log_path)

    next_cmd = 0.0
    next_ui = 0.0
    try:
        while not session.quit:
            now = time.monotonic()

            ch = scr.getch()
            while ch != -1:
                if not handle_key(ch, session):
                    session.quit = True
                    break
                ch = scr.getch()

            if now >= next_cmd:
                next_cmd = now + CMD_PERIOD
                drive = session.drive
                if time.monotonic() - drive.last_key > DEADMAN_S and drive.busy():
                    drive.soft_stop()
                    drive.say("dead-man stopped the motors")
                drive.advance()
                session.link.command(drive.applied_steer, drive.applied_speed)

            session.link.poll()
            if now >= next_ui:
                next_ui = now + UI_PERIOD
                render(scr, session)
                log_row(session, session.link.telemetry)

            time.sleep(0.005)
    finally:
        for _ in range(10):
            session.link.command(0, 0)
        session.link.stop()
        if session.log_file is not None:
            session.log_file.close()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Interactive drive console for the hoverboard FOC firmware")
    parser.add_argument("--port", help="serial port (default: autodetect)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="USB serial baud rate (ignored by USB CDC)")
    parser.add_argument("--max", type=int, default=300,
                        help="speed clamp in command units (default 300)")
    parser.add_argument("--log", metavar="CSV", help="start logging to this file")
    args = parser.parse_args(argv)

    port = args.port or autodetect_port()
    try:
        link = Link(port, args.baud)
    except serial.SerialException as exc:
        sys.exit("cannot open %s: %s\n"
                 "If a terminal still holds the port, close it "
                 "(screen: Ctrl-A, K, y)." % (port, exc))

    session = Session(link, Drive(args.max))
    try:
        curses.wrapper(run, session, args.log)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    print("stopped. port %s, frames ok %d, bad %d" % (port, link.ok, link.bad))


if __name__ == "__main__":
    main()
