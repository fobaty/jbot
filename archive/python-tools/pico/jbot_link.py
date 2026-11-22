#!/usr/bin/env python3
"""Serial link, protocol parsing, and logging for the JBOT Pico 2 W console.

This module is deliberately UI-free so it can be imported by the curses
application, by tests, and by headless scripts.

Wire protocol (v1), one ASCII line per message:

    Pico -> host   JBOT pico bridge 0.1 ready      banner after enumeration
    Pico -> host   T key=value ...                 telemetry sample
    Pico -> host   ok ...                          accepted command
    Pico -> host   err ...                         rejected command
    host -> Pico   help | ping | stat | stream <hz> | trim <deg> |
                   reinit | estop | clear | stats reset | reboot
"""

import glob
import time
from collections import deque

try:
    import serial
except ImportError:  # pragma: no cover - depends on the host setup
    serial = None

BAUD = 115200
DEFAULT_STREAM_HZ = 10
STALE_TELEMETRY_S = 2.5
# `age` when no mainboard feedback frame has arrived since boot.
AGE_NONE = 0xFFFFFFFF

TELEMETRY_FIELDS = (
    "step",
    "loop",
    "imu",
    "cal",
    "pitch",
    "gy",
    "err",
    "mb",
    "L",
    "R",
    "bat",
    "tmp",
    "age",
    "estop",
)

CSV_HEADER = (
    "t_s,step,loop_hz,imu,cal,pitch_deg,gyro_dps,sys_err,"
    "mainboard,rpm_left,rpm_right,battery_v,board_temp_c,feedback_age_ms,estop\n"
)


class Telemetry:
    """One parsed `T ...` sample."""

    __slots__ = TELEMETRY_FIELDS

    def __init__(self, **fields):
        for name in TELEMETRY_FIELDS:
            setattr(self, name, fields[name])

    def __repr__(self):
        return "Telemetry(step=%d, pitch=%.2f, bat=%.2f)" % (
            self.step,
            self.pitch,
            self.bat,
        )

    @property
    def imu_ok(self):
        return self.imu == "ok"

    @property
    def mainboard_ok(self):
        return self.mb == "ok"

    @property
    def stopped(self):
        return bool(self.estop)

    @property
    def age_text(self):
        """Feedback age for display; `--` when no frame has arrived yet."""
        return "--" if self.age == AGE_NONE else "%d" % self.age

    def csv_row(self, host_time=None):
        host_time = time.time() if host_time is None else host_time
        return "%.3f,%d,%d,%s,%d,%.2f,%.2f,%d,%s,%d,%d,%.2f,%.1f,%d,%d\n" % (
            host_time,
            self.step,
            self.loop,
            self.imu,
            self.cal,
            self.pitch,
            self.gy,
            self.err,
            self.mb,
            self.L,
            self.R,
            self.bat,
            self.tmp,
            self.age,
            self.estop,
        )


def parse_telemetry(line):
    """Parse a `T key=value ...` line. Returns None when it is not telemetry."""
    if not line.startswith("T "):
        return None
    fields = {}
    for token in line[2:].split():
        key, separator, value = token.partition("=")
        if separator:
            fields[key] = value
    try:
        return Telemetry(
            step=int(fields["step"]),
            loop=int(fields["loop"]),
            imu=fields["imu"],
            cal=int(fields["cal"]),
            pitch=float(fields["pitch"]),
            gy=float(fields["gy"]),
            err=int(fields["err"], 0),
            mb=fields["mb"],
            L=int(fields["L"]),
            R=int(fields["R"]),
            bat=float(fields["bat"]),
            tmp=float(fields["tmp"]),
            age=int(fields["age"]),
            estop=int(fields["estop"]),
        )
    except (KeyError, ValueError):
        return None


def parse_stat(line):
    """Parse a `stat key=value ...` reply into a dict, or None.

    Values are converted to int or float when possible and left as text
    otherwise, so a future firmware can add fields without breaking the host.
    """
    if not line.startswith("stat "):
        return None
    fields = {}
    for token in line[5:].split():
        key, separator, value = token.partition("=")
        if not separator:
            continue
        try:
            fields[key] = int(value)
        except ValueError:
            try:
                fields[key] = float(value)
            except ValueError:
                fields[key] = value
    return fields or None


def autodetect_port():
    """Return the first Pico USB serial port, or None."""
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    return ports[0] if ports else None


class Link:
    """USB serial link to the Pico console.

    `poll()` never blocks and returns a list of `(kind, payload)` messages:
    `("telemetry", Telemetry)`, `("stat", dict)`, `("ok", text)`, `("err", text)`,
    `("banner", text)`, `("garbage", text)`.
    """

    def __init__(self, port, baud=BAUD):
        self.port = port
        self.baud = baud
        self.serial = None
        self.buffer = b""
        self.telemetry = None
        self.telemetry_at = 0.0
        self.banner = None
        self.stat = None
        self.last_rx_at = 0.0
        self.rx_lines = 0
        self.tx_lines = 0
        self.bytes_in = 0
        self.bytes_out = 0

    @property
    def is_open(self):
        return self.serial is not None and self.serial.is_open

    def open(self):
        if serial is None:
            raise RuntimeError("pyserial is missing: python3 -m pip install pyserial")
        self.serial = serial.Serial(self.port, self.baud, timeout=0)
        self.buffer = b""
        self.telemetry = None
        self.telemetry_at = 0.0
        self.banner = None
        self.stat = None
        self.last_rx_at = time.monotonic()

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None

    def write(self, text):
        if not self.is_open:
            return False
        try:
            payload = ("%s\n" % text).encode("ascii", "replace")
            self.bytes_out += len(payload)
            self.serial.write(payload)
            self.tx_lines += 1
            return True
        except Exception:
            return False

    def telemetry_age(self):
        if self.telemetry is None:
            return float("inf")
        return time.monotonic() - self.telemetry_at

    def poll(self):
        """Drain the port and return parsed messages."""
        if not self.is_open:
            return []
        try:
            waiting = self.serial.in_waiting
            if waiting:
                chunk = self.serial.read(waiting)
                if chunk:
                    self.bytes_in += len(chunk)
                    self.buffer += chunk
        except Exception:
            self.close()
            return [("err", "serial read failed")]

        messages = []
        while b"\n" in self.buffer:
            raw, self.buffer = self.buffer.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            self.rx_lines += 1
            self.last_rx_at = time.monotonic()
            messages.append(self._parse(line))
        return messages

    def _parse(self, line):
        telemetry = parse_telemetry(line)
        if telemetry is not None:
            self.telemetry = telemetry
            self.telemetry_at = time.monotonic()
            return ("telemetry", telemetry)
        statistics = parse_stat(line)
        if statistics is not None:
            self.stat = statistics
            return ("stat", statistics)
        if line.startswith("ok"):
            return ("ok", line)
        if line.startswith("err"):
            return ("err", line)
        if line.startswith("JBOT"):
            self.banner = line
            return ("banner", line)
        return ("garbage", line)


class EventLog:
    """Timestamped ring buffer, optionally mirrored to a text file."""

    def __init__(self, path=None, capacity=400):
        self.entries = deque(maxlen=capacity)
        self.path = path
        self.file = open(path, "a", encoding="utf-8") if path else None
        self.total = 0

    def add(self, level, text, at=None):
        entry = (time.time() if at is None else at, level, text)
        self.entries.append(entry)
        self.total += 1
        if self.file is not None:
            self.file.write("%.3f %-5s %s\n" % entry)
            self.file.flush()
        return entry

    def clear(self):
        self.entries.clear()

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None


class CsvLog:
    """Telemetry CSV, one row per sample."""

    def __init__(self, path):
        self.path = path
        self.file = open(path, "a", encoding="utf-8")
        self.rows = 0
        self.file.write(CSV_HEADER)

    def write(self, telemetry, host_time=None):
        if self.file is None:
            return
        self.file.write(telemetry.csv_row(host_time))
        self.rows += 1
        self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None
