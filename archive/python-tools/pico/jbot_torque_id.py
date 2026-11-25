#!/usr/bin/env python3
"""Identify the EFeru torque scale from wheel spin-up telemetry.

No external force gauge is required. The robot is held with both wheels free,
a limited spin-test torque is applied, and the wheel angular acceleration is
read from the hall feedback at 200 Hz. With the per-wheel inertia from
``config/robot.toml`` the applied torque follows from ``M = I * alpha``.

The script never arms the balance law, stays inside the spin-test limits
(+-300 units, 600 units/s, 5 s expiry), clears the latch on request and latches
the emergency stop when it finishes. The operator must hold the body and keep
the robot inside the 0.5 m safe zone.
"""

import argparse
import math
import os
import re
import select
import sys
import termios
import time
import tty

BAUD = 115200
DEFAULT_PORT = "/dev/cu.usbmodemJBOT_PICO_011"
# Per-wheel inertia from config/robot.toml [mechanical]. Uncertainty +/-20 %.
WHEEL_INERTIA_KG_M2 = 0.0078
# Spin-test limits from firmware/pico2w/core/src/spin.rs.
SPIN_LIMIT_UNITS = 300
RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument(
        "--steps",
        default="20,40,60,80,100",
        help="comma-separated wheel-torque requests in EFeru units",
    )
    parser.add_argument("--settle", type=float, default=2.0, help="max wait for zero rpm, s")
    parser.add_argument("--duration", type=float, default=1.4, help="spin-up window, s")
    parser.add_argument("--fit-from", type=float, default=0.35, help="fit window start, s")
    parser.add_argument("--fit-to", type=float, default=1.15, help="fit window end, s")
    return parser.parse_args()


class Link:
    """Minimal POSIX serial link; no third-party dependency."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[2] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.buf = b""

    def write(self, command):
        os.write(self.fd, (command + "\n").encode())

    def lines(self, timeout=0.05):
        if select.select([self.fd], [], [], timeout)[0]:
            try:
                data = os.read(self.fd, 8192)
            except BlockingIOError:
                data = b""
            self.buf += data
        out = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            text = raw.decode(errors="replace").strip()
            if text:
                out.append(text)
        return out

    def drain(self, seconds=0.3):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.lines(0.05)

    def close(self):
        os.close(self.fd)


def telemetry(line):
    if not line.startswith("T "):
        return None
    fields = dict((p.split("=", 1) + [""])[:2] for p in line[2:].split() if "=" in p)
    try:
        return (
            float(fields["L"]),
            float(fields["R"]),
            float(fields.get("tsl", 0)),
            int(float(fields.get("estop", 0))),
            int(float(fields.get("arm", 0))),
            float(fields.get("pitch", 0)),
        )
    except (KeyError, ValueError):
        return None


def slope_rpm_per_s(samples):
    """Least-squares slope of left-wheel rpm against time."""
    n = len(samples)
    if n < 4:
        return 0.0
    sx = sum(t for t, _ in samples)
    sy = sum(v for _, v in samples)
    sxx = sum(t * t for t, _ in samples)
    sxy = sum(t * v for t, v in samples)
    divisor = n * sxx - sx * sx
    if abs(divisor) < 1e-9:
        return 0.0
    return (n * sxy - sx * sy) / divisor


def wait_rest(link, settle):
    deadline = time.monotonic() + settle
    quiet_since = None
    while time.monotonic() < deadline:
        for line in link.lines(0.05):
            sample = telemetry(line)
            if sample is None:
                continue
            left, right, _, _, _, _ = sample
            if max(abs(left), abs(right)) < 8:
                if quiet_since is None:
                    quiet_since = time.monotonic()
                elif time.monotonic() - quiet_since >= 0.35:
                    return True
            else:
                quiet_since = None
    return False


def measure(link, units, duration, fit_from, fit_to):
    if not wait_rest(link, 3.0):
        return None
    link.write("test %d" % units)
    start = time.monotonic()
    samples = []
    pitches = []
    while time.monotonic() - start < duration:
        for line in link.lines(0.05):
            sample = telemetry(line)
            if sample is None:
                continue
            elapsed = time.monotonic() - start
            left, right, applied, estop, armed, pitch = sample
            if estop or armed:
                continue
            samples.append((elapsed, left))
            pitches.append(pitch)
    link.write("test 0")
    time.sleep(0.4)
    window = [(t, v) for t, v in samples if fit_from <= t <= fit_to]
    slope = slope_rpm_per_s(window)
    alpha = slope * RPM_TO_RAD_S
    torque_wheel = WHEEL_INERTIA_KG_M2 * alpha
    units_per_nm = units / (2.0 * torque_wheel) if torque_wheel > 0.01 else float("nan")
    return {
        "units": units,
        "slope": slope,
        "alpha": alpha,
        "torque_wheel": torque_wheel,
        "units_per_nm": units_per_nm,
        "samples": len(window),
        "pitch_range": (min(pitches), max(pitches)) if pitches else (0.0, 0.0),
    }


def main():
    args = parse_args()
    steps = []
    for token in args.steps.split(","):
        try:
            value = int(float(token))
        except ValueError:
            sys.exit("invalid step: %s" % token)
        if not -SPIN_LIMIT_UNITS <= value <= SPIN_LIMIT_UNITS:
            sys.exit("step %d exceeds the spin-test limit" % value)
        steps.append(value)

    link = Link(args.port)
    try:
        link.drain(0.3)
        link.write("clear")
        link.write("stream 50")
        link.drain(0.5)
        print("Holding the body: wheels free, rope attached, e-stop reachable.")
        print("No arm command is sent by this script.")
        results = []
        for units in steps:
            result = measure(link, units, args.duration, args.fit_from, args.fit_to)
            results.append(result)
            if result is None:
                print("%4d  no clean rest within 3 s" % units)
                continue
            low, high = result["pitch_range"]
            print(
                "%4d units | %7.1f rpm/s | alpha %7.1f rad/s2 | "
                "M_wheel %6.3f Nm | %7.1f units/Nm total | pitch %+.1f..%+.1f | n=%d"
                % (
                    units,
                    result["slope"],
                    result["alpha"],
                    result["torque_wheel"],
                    result["units_per_nm"],
                    low,
                    high,
                    result["samples"],
                )
            )
        good = [
            r
            for r in results
            if r
            and 0.01 < r["torque_wheel"] < 2.5
            and r["samples"] >= 4
            and r["pitch_range"][1] - r["pitch_range"][0] < 1.0
        ]
        if good:
            mean = sum(r["units_per_nm"] for r in good) / len(good)
            print("mean torque scale: %.1f units/Nm total (design assumption: 200)" % mean)
        else:
            print("no usable spin-up windows; check that the wheels are free")
    finally:
        link.write("test 0")
        time.sleep(0.2)
        link.write("estop")
        link.close()
        print("e-stop latched; send 'clear' before further tests")


if __name__ == "__main__":
    main()
