#!/usr/bin/env python3
"""Terminal console for the JBOT Pico 2 W bridge firmware.

Shows live telemetry from the Pico's USB CDC port, keeps a timestamped event
log, writes CSV, and sends the line commands defined in `docs/pico-bridge.md`
section 8.1.

Usage:
    python3 tools/pico/jbot_console.py                     # autodetect, TUI
    python3 tools/pico/jbot_console.py --port /dev/cu.usbmodemJBOT_PICO_011
    python3 tools/pico/jbot_console.py --no-ui --csv run.csv
    python3 tools/pico/jbot_console.py --list

Keys:
    :            type any command, Enter sends it
    s            set the telemetry rate (stream <hz>)
    t            set the pitch trim (trim <deg>)
    p / i        ping / stat
    r            reinit the BNO055
    e / c        emergency stop / clear the latch
    L            toggle CSV logging
    R            reboot the Pico (asks for y)
    x            clear the on-screen event log
    q            quit
"""

import argparse
import curses
import queue
import sys
import threading
import time

from jbot_link import (
    DEFAULT_STREAM_HZ,
    STALE_TELEMETRY_S,
    CsvLog,
    EventLog,
    Link,
    autodetect_port,
)

PAIR_HEAD, PAIR_OK, PAIR_WARN, PAIR_STOP, PAIR_DIM = 1, 2, 3, 4, 5
RECONNECT_S = 2.0
UI_PERIOD = 1.0 / 30.0

KEY_HELP = (
    "  : cmd  s stream  v speed  w steer  t trim  ↑/↓ speed  ←/→ steer  "
    "space zero  e estop  c clear  p ping  i stat  e estop  L log  q quit"
)

LEVEL_STYLE = {
    "debug": PAIR_DIM,
    "info": 0,
    "warn": PAIR_WARN,
    "error": PAIR_STOP,
}


def clock(at=None):
    return time.strftime("%H:%M:%S", time.localtime(time.time() if at is None else at))


def format_stat(statistics):
    """One readable line from a parsed `stat` reply."""
    parts = []
    if "up" in statistics:
        parts.append("up %.1f s" % statistics["up"])
    if "loop" in statistics:
        parts.append("loop %d Hz" % statistics["loop"])
    if "stream" in statistics:
        parts.append("stream %d Hz" % statistics["stream"])
    if "trim" in statistics:
        parts.append("trim %+.2f deg" % statistics["trim"])
    if "estop" in statistics:
        parts.append("estop %s" % ("YES" if statistics["estop"] else "no"))
    if "steps" in statistics:
        parts.append("steps %d" % statistics["steps"])
    if "imu" in statistics:
        parts.append("imu samples %d, errors %d, code %d" % (
            statistics["imu"],
            statistics.get("imu_err", 0),
            statistics.get("imu_code", 0),
        ))
    if "fb" in statistics:
        parts.append("feedback frames %d, discarded %d B" % (
            statistics["fb"],
            statistics.get("fb_drop", 0),
        ))
    return "stat: " + ", ".join(parts)


class Session:
    """Everything the renderer and the command handler share."""

    def __init__(self, port, args):
        self.port = port
        self.args = args
        self.link = Link(port, args.baud)
        self.events = EventLog(args.events)
        self.csv = CsvLog(args.csv) if args.csv else None
        self.stream_hz = args.stream
        self.opened_at = time.monotonic()
        self.first_telemetry = True
        self.last_step = None
        self.stale_warned = False
        self.prompt = None
        self.confirm = None
        self.status = "starting"
        self.quit = False
        self.next_retry = 0.0
        self.notice = ""
        self._headless_seen = 0
        self._speed_setpoint = 0.0
        self._steer_setpoint = 0.0

    def log(self, level, text):
        self.events.add(level, text)

    def note(self, text):
        self.status = text

    def connect(self):
        try:
            self.link.open()
        except Exception as error:
            self.note("cannot open %s: %s" % (self.link.port, error))
            return False
        self.opened_at = time.monotonic()
        self.first_telemetry = True
        self.last_step = None
        self.stale_warned = False
        self.log("info", "connected %s" % self.link.port)
        self.link.write("ping")
        self.link.write("stat")
        self.link.write("stream %d" % self.stream_hz)
        self.note("connected")
        return True

    def toggle_csv(self):
        if self.csv is not None:
            rows = self.csv.rows
            path = self.csv.path
            self.csv.close()
            self.csv = None
            self.log("info", "csv closed: %d rows in %s" % (rows, path))
            return
        path = self.args.csv or time.strftime("jbot_telemetry_%Y%m%d_%H%M%S.csv")
        try:
            self.csv = CsvLog(path)
        except OSError as error:
            self.log("error", "cannot open csv %s: %s" % (path, error))
            return
        self.log("info", "csv logging to %s" % path)

    def send(self, command):
        command = command.strip()
        if not command:
            return
        if not self.link.is_open:
            self.log("warn", "not connected, dropped: %s" % command)
            return
        self.link.write(command)
        self.log("debug", "> %s" % command)
        self.note("> %s" % command)

    def handle_messages(self, messages):
        for kind, payload in messages:
            if kind == "telemetry":
                self.on_telemetry(payload)
            elif kind == "ok":
                self.log("info", payload)
                self.note(payload)
            elif kind == "err":
                self.log("warn", payload)
                self.note(payload)
            elif kind == "stat":
                self.log("info", format_stat(payload))
            elif kind == "banner":
                self.log("info", "firmware: %s" % payload)
            elif kind == "garbage":
                self.log("debug", "unparsed: %s" % payload)

    def on_telemetry(self, telemetry):
        if self.first_telemetry:
            self.first_telemetry = False
            self.log("info", "telemetry streaming at %d Hz" % self.stream_hz)
        if self.last_step is not None and telemetry.step < self.last_step:
            self.log(
                "warn",
                "firmware restarted: step %d -> %d" % (self.last_step, telemetry.step),
            )
        self.last_step = telemetry.step
        if self.stale_warned:
            self.stale_warned = False
            self.log("info", "telemetry resumed")
        if self.csv is not None:
            self.csv.write(telemetry)

    def check_stale(self):
        if not self.link.is_open or self.stream_hz == 0:
            return
        if self.link.telemetry is None:
            return
        if self.link.telemetry_age() > STALE_TELEMETRY_S and not self.stale_warned:
            self.stale_warned = True
            self.log(
                "warn",
                "no telemetry for %.1f s" % self.link.telemetry_age(),
            )

    def tick(self):
        now = time.monotonic()
        if not self.link.is_open:
            if now >= self.next_retry:
                self.next_retry = now + RECONNECT_S
                if self.connect():
                    self.next_retry = 0.0
            return
        messages = self.link.poll()
        if messages:
            self.handle_messages(messages)
        if not self.link.is_open:
            self.log("warn", "device disconnected")
            self.note("disconnected")
            self.next_retry = time.monotonic() + RECONNECT_S
        self.check_stale()


def draw(stdscr, session):
    height, width = stdscr.getmaxyx()
    stdscr.erase()
    link = session.link
    telemetry = link.telemetry

    connected = link.is_open
    state = "link:ok" if connected else "link:down"
    if connected:
        state += "  live" if not session.first_telemetry else "  waiting"
    header = " JBOT PICO CONSOLE  %s  %s" % (link.port or "(no port)", state)
    stdscr.addnstr(0, 0, header.ljust(max(0, width - 1)), width - 1,
                   curses.color_pair(PAIR_HEAD) | curses.A_REVERSE)
    stdscr.addnstr(1, 0, "=" * (width - 1), width - 1, curses.color_pair(PAIR_DIM))

    row = 2
    if telemetry is None:
        stdscr.addnstr(row, 1, "waiting for the first sample ...", width - 2,
                       curses.color_pair(PAIR_WARN))
        row += 1
        stdscr.addnstr(row, 1, session.status, width - 2)
    else:
        uptime = telemetry.step / max(telemetry.loop, 1)
        runtime = "  RUNTIME    uptime %6.1f s   loop %3d Hz   stream %3d Hz   estop %s" % (
            uptime,
            telemetry.loop,
            session.stream_hz,
            "YES" if telemetry.stopped else "no",
        )
        stdscr.addnstr(row, 0, runtime, width - 1,
                       curses.color_pair(PAIR_STOP) if telemetry.stopped else 0)
        row += 1

        imu_attr = 0 if telemetry.imu_ok else curses.color_pair(PAIR_WARN)
        imu = "  IMU        %-5s cal %d/3   pitch %+7.2f deg   gyro %+7.2f deg/s   " \
              "sys_err 0x%02x" % (
                  telemetry.imu,
                  telemetry.cal,
                  telemetry.pitch,
                  telemetry.gy,
                  telemetry.err,
              )
        stdscr.addnstr(row, 0, imu, width - 1, imu_attr)
        row += 1

        mb_attr = 0 if telemetry.mainboard_ok else curses.color_pair(PAIR_WARN)
        mainboard = "  MAINBOARD  %-5s L %+5d rpm  R %+5d rpm  battery %6.2f V  " \
                    "board %5.1f C  age %5s ms" % (
                        telemetry.mb,
                        telemetry.L,
                        telemetry.R,
                        telemetry.bat,
                        telemetry.tmp,
                        telemetry.age_text,
                    )
        stdscr.addnstr(row, 0, mainboard, width - 1, mb_attr)
        row += 1

    stdscr.addnstr(row, 0, "-" * (width - 1), width - 1, curses.color_pair(PAIR_DIM))
    row += 1
    stdscr.addnstr(row, 1, "EVENTS", width - 2, curses.color_pair(PAIR_HEAD))
    row += 1

    bottom = height - 3
    available = max(0, bottom - row)
    entries = list(session.events.entries)[-available:] if available else []
    for at, level, text in entries:
        line = "  %s  %-5s %s" % (clock(at), level, text)
        stdscr.addnstr(row, 0, line, width - 1, curses.color_pair(LEVEL_STYLE.get(level, 0)))
        row += 1

    if session.prompt is not None:
        prompt = "  :%s" % session.prompt
        stdscr.addnstr(height - 2, 0, prompt, width - 1, curses.A_BOLD)
        stdscr.move(height - 2, min(len(prompt), width - 2))
    elif session.confirm is not None:
        stdscr.addnstr(height - 2, 0, "  %s  (y/n)" % session.confirm, width - 1,
                       curses.color_pair(PAIR_WARN))
    else:
        stdscr.addnstr(height - 2, 0, "  " + session.status, width - 1,
                       curses.color_pair(PAIR_DIM))
    stdscr.addnstr(height - 1, 0, KEY_HELP, width - 1, curses.color_pair(PAIR_DIM))
    stdscr.refresh()


def handle_key(stdscr, session, key):
    if session.prompt is not None:
        if key in (10, 13, curses.KEY_ENTER):
            command = session.prompt
            session.prompt = None
            session.send(command)
        elif key == 27:
            session.prompt = None
            session.note("cancelled")
        elif key in (8, 127, curses.KEY_BACKSPACE):
            session.prompt = session.prompt[:-1]
        elif 32 <= key < 127:
            session.prompt += chr(key)
        return

    if session.confirm is not None:
        if key in (ord("y"), ord("Y")):
            session.send("reboot")
            session.log("warn", "reboot requested")
        session.confirm = None
        return

    if key == ord(":"):
        session.prompt = ""
    elif key == ord("s"):
        session.prompt = "stream "
    elif key == ord("v"):
        session.prompt = "speed "
    elif key == ord("w"):
        session.prompt = "steer "
    elif key == ord("t"):
        session.prompt = "trim "
    elif key == ord("p"):
        session.send("ping")
    elif key == ord("i"):
        session.send("stat")
    elif key == ord("r"):
        session.send("reinit")
        session.log("info", "BNO055 re-init requested")
    elif key == ord("e"):
        session.send("estop")
        session.log("warn", "emergency stop requested")
    elif key == ord("c"):
        session.send("clear")
        session.log("info", "e-stop clear requested")
    elif key == ord("W") or key == curses.KEY_UP:
        session._speed_setpoint = round(min(5.0, session._speed_setpoint + 0.1), 2)
        cmd = f"speed {session._speed_setpoint}"
        session.send(cmd)
    elif key == ord("S") or key == curses.KEY_DOWN:
        session._speed_setpoint = round(max(-5.0, session._speed_setpoint - 0.1), 2)
        cmd = f"speed {session._speed_setpoint}"
        session.send(cmd)
    elif key == ord("A") or key == curses.KEY_LEFT:
        session._steer_setpoint = round(max(-180.0, session._steer_setpoint - 10.0), 2)
        cmd = f"steer {session._steer_setpoint}"
        session.send(cmd)
    elif key == ord("D") or key == curses.KEY_RIGHT:
        session._steer_setpoint = round(min(180.0, session._steer_setpoint + 10.0), 2)
        cmd = f"steer {session._steer_setpoint}"
        session.send(cmd)
    elif key == ord(" "):
        session._speed_setpoint = 0.0
        session._steer_setpoint = 0.0
        session.send("speed 0")
        session.send("steer 0")
        session.log("warn", "motors zeroed")
    elif key == ord("L"):
        session.toggle_csv()
    elif key == ord("R"):
        session.confirm = "reboot the Pico"
    elif key == ord("x"):
        session.events.clear()
        session.note("event log cleared")
    elif key == ord("q"):
        session.quit = True


def run(stdscr, session):
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(PAIR_HEAD, curses.COLOR_CYAN, -1)
    curses.init_pair(PAIR_OK, curses.COLOR_GREEN, -1)
    curses.init_pair(PAIR_WARN, curses.COLOR_YELLOW, -1)
    curses.init_pair(PAIR_STOP, curses.COLOR_RED, -1)
    curses.init_pair(PAIR_DIM, curses.COLOR_WHITE, -1)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    try:
        curses.curs_set(1)
    except curses.error:
        pass

    while not session.quit:
        session.tick()
        draw(stdscr, session)
        key = stdscr.getch()
        if key != -1:
            handle_key(stdscr, session, key)
        time.sleep(UI_PERIOD)

    if session.link.is_open:
        session.send("estop")
        session.link.close()
    session.events.close()
    if session.csv is not None:
        session.csv.close()


def stdin_reader(commands):
    for line in sys.stdin:
        commands.put(line.strip())


def run_headless(session):
    commands = queue.Queue()
    threading.Thread(target=stdin_reader, args=(commands,), daemon=True).start()
    session.log("info", "headless mode, commands on stdin")
    last_render = 0.0
    last_step = None
    try:
        while not session.quit:
            session.tick()
            while not commands.empty():
                session.send(commands.get())
            pending = session.events.total - session._headless_seen
            if pending > 0:
                for at, level, text in list(session.events.entries)[-pending:]:
                    print("[%s] %-5s %s" % (clock(at), level, text), flush=True)
                session._headless_seen = session.events.total
            telemetry = session.link.telemetry
            now = time.monotonic()
            fresh = telemetry is not None and telemetry.step != last_step
            if fresh and now - last_render > 0.2:
                last_render = now
                last_step = telemetry.step
                print(
                    "%s step=%d loop=%d imu=%s cal=%d pitch=%+.2f gy=%+.2f err=0x%02x "
                    "mb=%s L=%d R=%d bat=%.2f tmp=%.1f age=%s estop=%d" % (
                        clock(),
                        telemetry.step,
                        telemetry.loop,
                        telemetry.imu,
                        telemetry.cal,
                        telemetry.pitch,
                        telemetry.gy,
                        telemetry.err,
                        telemetry.mb,
                        telemetry.L,
                        telemetry.R,
                        telemetry.bat,
                        telemetry.tmp,
                        telemetry.age_text,
                        telemetry.estop,
                    ),
                    flush=True,
                )
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if session.link.is_open:
            session.send("estop")
            session.link.close()
        session.events.close()
        if session.csv is not None:
            session.csv.close()


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port, autodetected by default")
    parser.add_argument("--baud", type=int, default=115200,
                        help="ignored by CDC, kept for familiarity")
    parser.add_argument("--stream", type=int, default=DEFAULT_STREAM_HZ,
                        help="telemetry rate requested on connect, 0..200 Hz")
    parser.add_argument("--csv", help="telemetry CSV path")
    parser.add_argument("--events", help="event log text path")
    parser.add_argument("--no-ui", action="store_true", help="plain text mode")
    parser.add_argument("--list", action="store_true", help="list serial ports")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.list:
        port = autodetect_port()
        print(port or "no /dev/cu.usbmodem* found")
        return 0
    port = args.port or autodetect_port()
    if port is None:
        sys.exit("no Pico serial port found, pass --port")
    if not 0 <= args.stream <= 200:
        sys.exit("--stream must be 0..200")
    session = Session(port, args)
    if args.no_ui:
        run_headless(session)
        return 0
    curses.wrapper(run, session)
    return 0


if __name__ == "__main__":
    sys.exit(main())
