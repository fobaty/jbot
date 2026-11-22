#!/usr/bin/env python3
"""Interactive drive console for JBOT Pico 2 W bridge.

Provides a curses-based TUI inspired by hover_drive.py: arrow keys and W/A/S/D
control speed and steering setpoints, Space does a soft stop, E/Esc triggers
an e-stop, C clears it, and telemetry is displayed live.
"""

import argparse
import curses
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing: python3 -m pip install pyserial")

from jbot_link import Link, autodetect_port, parse_telemetry


def approach(current, target, step):
    if target > current:
        return min(target, current + step)
    if target < current:
        return max(target, current - step)
    return current


def bar(value, span, width):
    if span <= 0:
        return "-" * width
    filled = int(round(min(1.0, abs(value) / float(span)) * width))
    filled = max(0, min(width, filled))
    return "#" * filled + "-" * (width - filled)


class DriveState:
    def __init__(self, max_speed_mps):
        self.target_speed = 0.0      # m/s
        self.target_steer = 0.0      # deg/s
        self.applied_speed = 0.0
        self.applied_steer = 0.0
        self.max_speed = max_speed_mps
        self.step = 0.2              # m/s per key press
        self.slew = 0.1              # rate limit per 50ms tick
        self.estop = False
        self.note = "ready"

    def say(self, text):
        self.note = text

    def clamp(self):
        self.target_speed = max(-self.max_speed, min(self.max_speed, self.target_speed))
        self.target_steer = max(-180.0, min(180.0, self.target_steer))

    def advance(self):
        self.applied_speed = approach(self.applied_speed, self.target_speed, self.slew)
        self.applied_steer = approach(self.applied_steer, self.target_steer, self.slew)

    def soft_stop(self):
        self.target_speed = 0.0
        self.target_steer = 0.0
        self.say("soft stop")

    def emergency(self):
        self.estop = True
        self.target_speed = 0.0
        self.target_steer = 0.0
        self.applied_speed = 0.0
        self.applied_steer = 0.0
        self.say("EMERGENCY STOP latched - press c to clear")

    def clear_estop(self):
        self.estop = False
        self.say("e-stop cleared")

    def busy(self):
        return bool(abs(self.target_speed) > 0.01 or abs(self.target_steer) > 0.1)


def handle_key(ch, session):
    drive = session["drive"]
    link = session["link"]

    if ch in (ord("q"), ord("Q")):
        return False
    if ch in (ord("x"), ord("X"), 27):
        drive.emergency()
        link.write("estop")
        link.write("speed 0")
        link.write("steer 0")
        curses.beep()
        return True
    if ch in (ord("c"), ord("C")):
        drive.clear_estop()
        link.write("clear")
        return True
    if drive.estop:
        drive.target_speed = 0.0
        drive.target_steer = 0.0
        return True

    if ch in (curses.KEY_UP, ord("w"), ord("W")):
        drive.target_speed += drive.step
    elif ch in (curses.KEY_DOWN, ord("s"), ord("S")):
        drive.target_speed -= drive.step
    elif ch in (curses.KEY_RIGHT, ord("d"), ord("D")):
        drive.target_steer += 20.0
    elif ch in (curses.KEY_LEFT, ord("a"), ord("A")):
        drive.target_steer -= 20.0
    elif ch == ord(" "):
        drive.soft_stop()
        link.write("speed 0")
        link.write("steer 0")
    elif ch in (ord("+"), ord("=")):
        drive.step = round(min(2.0, drive.step + 0.1), 2)
        drive.say("speed step %.1f m/s" % drive.step)
    elif ch in (ord("-"), ord("_")):
        drive.step = round(max(0.05, drive.step - 0.1), 2)
        drive.say("speed step %.1f m/s" % drive.step)
    elif ch in (ord("l"), ord("L")):
        session["csv_active"] = not session["csv_active"]
        drive.say("CSV logging %s" % ("ON" if session["csv_active"] else "OFF"))
    else:
        return True

    drive.clamp()
    link.write("speed %.2f" % drive.applied_speed)
    link.write("steer %.1f" % drive.applied_steer)
    return True


def render(scr, session):
    drive, link = session["drive"], session["link"]
    height, width = scr.getmaxyx()
    scr.erase()
    if height < 16 or width < 70:
        scr.addnstr(0, 0, "window too small: need at least 70x16", max(0, width - 1))
        scr.refresh()
        return

    w = min(width - 2, 92)
    x0 = max(0, (width - w) // 2)
    tel = link.telemetry
    fresh = link.telemetry is not None and link.telemetry_age() < 1.0
    hot = drive.estop

    def put(row, text, attr=0, col=0):
        if row >= height - 1:
            return
        clip = max(0, w - col - 1)
        if clip <= 0:
            return
        text = text[:clip]
        scr.addnstr(row, x0 + col, text, len(text), attr)

    head = " JBOT DRIVE CONSOLE   port:%s  %s" % (
        link.port, "LOG ON" if session["csv_active"] else "log off")
    put(0, head.ljust(w), curses.color_pair(1))
    put(1, "=" * w, curses.color_pair(5))

    if tel is None:
        put(3, " Waiting for telemetry from Pico...", curses.color_pair(3))
        scr.refresh()
        return

    put(3, " SPEED & STEER SETPOINTS", curses.color_pair(1))
    put(4, " Target Speed:  %+6.2f m/s   (step: %.2f)  Applied: %+6.2f m/s" % (
        drive.target_speed, drive.step, drive.applied_speed))
    put(5, " Target Steer:  %+6.1f deg/s                 Applied: %+6.1f deg/s" % (
        drive.target_steer, drive.applied_steer))
    put(6, "-" * w, curses.color_pair(5))

    put(7, " TELEMETRY", curses.color_pair(1))
    put(8, " Step: %-8d Loop: %3d Hz  IMU: %-5s Cal: %d  Pitch: %+6.2f deg" % (
        tel.step, tel.loop, tel.imu, tel.cal, tel.pitch))
    put(9, " Mainboard: %-5s L: %+5d rpm  R: %+5d rpm  Bat: %5.2f V  Temp: %4.1f C" % (
        tel.mb, tel.L, tel.R, tel.bat, tel.tmp))
    put(10, " Age: %5s ms   Sys Err: 0x%02x       E-Stop: %s" % (
        tel.age_text, tel.err, "YES" if tel.stopped else "no"))
    put(11, "-" * w, curses.color_pair(5))

    put(12, " [W/S / Up/Down] Speed  [A/D / L/R] Steer  [Space] Stop  [E/Esc] E-Stop  [C] Clear  [Q] Quit")
    put(13, " Status: %s" % drive.note, curses.color_pair(3) if hot else 0)

    scr.refresh()


def main():
    parser = argparse.ArgumentParser(description="JBOT Pico drive console")
    parser.add_argument("--port", default=None, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--max-speed", type=float, default=2.0, help="Max speed m/s")
    args = parser.parse_args()

    port = args.port or autodetect_port()
    link = Link(port, args.baud)
    try:
        link.open()
    except Exception as e:
        sys.exit("failed to open %s: %s" % (port, e))

    drive = DriveState(args.max_speed)
    link.write("stream 20")
    link.write("clear")

    session = {
        "link": link,
        "drive": drive,
        "csv_active": False,
    }

    def wrapper(stdscr):
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_CYAN, -1)
        curses.init_pair(2, curses.COLOR_GREEN, -1)
        curses.init_pair(3, curses.COLOR_YELLOW, -1)
        curses.init_pair(4, curses.COLOR_RED, -1)
        curses.init_pair(5, curses.COLOR_WHITE, -1)
        stdscr.nodelay(True)
        stdscr.timeout(50)

        last_tick = time.monotonic()
        while True:
            ch = stdscr.getch()
            if ch != -1:
                if not handle_key(ch, session):
                    break

            now = time.monotonic()
            if now - last_tick >= 0.05:
                drive.advance()
                last_tick = now

            for _kind, _payload in link.poll():
                pass

            render(stdscr, session)
            time.sleep(0.01)

    try:
        curses.wrapper(wrapper)
    finally:
        try:
            link.write("estop")
            link.write("speed 0")
            link.write("steer 0")
        except Exception:
            pass
        link.close()
        print("Drive console closed.")


if __name__ == "__main__":
    main()
