#!/usr/bin/env python3
"""Identify per-wheel torque scale from one-wheel spin-up; no external gauge."""

import argparse
import math
import os
import select
import termios
import time
import tty

PORT = "/dev/cu.usbmodemJBOT_PICO_011"
WHEEL_INERTIA_KG_M2 = 0.0078
RPM_TO_RAD_S = 2.0 * math.pi / 60.0
SPIN_LIMIT_UNITS = 300


class Link:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.buf = b""

    def write(self, text):
        os.write(self.fd, (text + "\n").encode())

    def lines(self, timeout=0.05):
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if ready:
            self.buf += os.read(self.fd, 8192)
        out = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            text = raw.decode(errors="replace").strip()
            if text:
                out.append(text)
        return out

    def drain(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.lines(0.05)

    def close(self):
        os.close(self.fd)


def telemetry(line):
    if not line.startswith("T "):
        return None
    fields = dict(part.split("=", 1) for part in line[2:].split() if "=" in part)
    try:
        return (
            float(fields["L"]),
            int(float(fields.get("estop", 0))),
            int(float(fields.get("arm", 0))),
            float(fields.get("pitch", 0)),
        )
    except (KeyError, ValueError):
        return None


def slope(samples):
    n = len(samples)
    if n < 4:
        return 0.0
    sx = sum(t for t, _ in samples)
    sy = sum(v for _, v in samples)
    sxx = sum(t * t for t, _ in samples)
    sxy = sum(t * v for t, v in samples)
    divisor = n * sxx - sx * sx
    return 0.0 if abs(divisor) < 1e-9 else (n * sxy - sx * sy) / divisor


def wait_rest(link, settle):
    end = time.monotonic() + settle
    quiet_since = None
    while time.monotonic() < end:
        for line in link.lines(0.05):
            sample = telemetry(line)
            if sample is None:
                continue
            left, estop, armed, _ = sample
            if abs(left) < 8 and not estop and not armed:
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
    link.write("test %d 0" % units)
    start = time.monotonic()
    samples = []
    pitches = []
    valid = True
    while time.monotonic() - start < duration:
        for line in link.lines(0.05):
            sample = telemetry(line)
            if sample is None:
                continue
            left, estop, armed, pitch = sample
            if estop or armed:
                valid = False
            elapsed = time.monotonic() - start
            samples.append((elapsed, left))
            pitches.append(pitch)
    link.write("test 0 0")
    time.sleep(0.4)
    window = [(t, v) for t, v in samples if fit_from <= t <= fit_to]
    alpha = slope(window) * RPM_TO_RAD_S
    torque = WHEEL_INERTIA_KG_M2 * alpha
    return {
        "units": units,
        "alpha": alpha,
        "torque": torque,
        "units_per_nm": units / torque if torque > 0.01 else float("nan"),
        "n": len(window),
        "pitch": (min(pitches), max(pitches)) if pitches else (0.0, 0.0),
        "valid": valid,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--steps", default="40,60,80,100")
    parser.add_argument("--duration", type=float, default=1.4)
    parser.add_argument("--fit-from", type=float, default=0.35)
    parser.add_argument("--fit-to", type=float, default=1.15)
    args = parser.parse_args()

    steps = [int(float(token)) for token in args.steps.split(",")]
    for units in steps:
        if not -SPIN_LIMIT_UNITS <= units <= SPIN_LIMIT_UNITS:
            raise SystemExit("step %d exceeds the spin-test limit" % units)

    link = Link(args.port)
    try:
        link.drain(0.3)
        link.write("clear")
        link.write("stream 100")
        link.drain(0.5)
        print("Hold the body still; left wheel free, right wheel idle, e-stop reachable.")
        results = []
        for units in steps:
            result = measure(link, units, args.duration, args.fit_from, args.fit_to)
            if result is None:
                print("%4d units | no clean rest within 3 s" % units)
                continue
            results.append(result)
            low, high = result["pitch"]
            print(
                "%4d units | alpha %7.1f rad/s2 | M %6.3f Nm | %7.1f units/Nm | "
                "pitch %+.1f..%+.1f | n=%d | valid=%d"
                % (
                    units, result["alpha"], result["torque"],
                    result["units_per_nm"], low, high, result["n"], result["valid"],
                )
            )
        good = []
        previous_alpha = 0.0
        for index, result in enumerate(results):
            if not result["valid"] or result["n"] < 4 or not 0.01 < result["torque"] < 1.0:
                continue
            if index == 0 or result["alpha"] <= previous_alpha:
                continue
            good.append(result)
            previous_alpha = result["alpha"]
        if good:
            mean = sum(r["units_per_nm"] for r in good) / len(good)
            print("mean per-wheel scale: %.1f units/Nm (design 200) -> tmax %.2f Nm" % (mean, 1000.0 / mean))
        else:
            print("no usable spin-up windows; check that the wheels are free")
    finally:
        link.write("test 0 0")
        time.sleep(0.2)
        link.write("stream 0")
        link.write("estop")
        link.close()
        print("e-stop latched")


if __name__ == "__main__":
    main()
