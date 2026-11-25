#!/usr/bin/env python3
"""Calibration bridge for the STM32F103 Rust torque node (USART3 console).

Dependency-free: the serial port is configured with the standard-library
`termios` module, so this runs on macOS/Linux without `pip install pyserial`.

Examples:
    python3 f103_bridge.py --port /dev/cu.usbserial-XXXX get
    python3 f103_bridge.py --port /dev/cu.usbserial-XXXX getp
    python3 f103_bridge.py --port /dev/cu.usbserial-XXXX set IQKP 1500
    python3 f103_bridge.py --port /dev/cu.usbserial-XXXX arm
    python3 f103_bridge.py --port /dev/cu.usbserial-XXXX trace --seconds 10 --decim 160 --out trace.csv
"""

from __future__ import annotations

import argparse
import csv
import fcntl
import os
import sys
import termios
import time

from f103_trace import (
    FLAG_ARMED,
    FLAG_CALIBRATION_READY,
    FLAG_FAULT_LATCHED,
    MSG_TRACE,
    StreamDemux,
    TraceData,
)


class SerialPort:
    """Minimal non-blocking 115200 8N1 serial port using termios."""

    def __init__(self, path: str, baud: int = 115200) -> None:
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = baud
            attrs[5] = baud
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, os.O_NONBLOCK)
        except Exception:
            os.close(self.fd)
            raise

    def read(self, size: int = 4096) -> bytes:
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def write_line(self, line: str) -> None:
        os.write(self.fd, (line + "\n").encode("ascii"))

    def flush_input(self) -> None:
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self) -> None:
        os.close(self.fd)


def request(port: SerialPort, demux: StreamDemux, line: str, timeout: float = 1.0) -> list[str]:
    """Send one console command and collect its ASCII reply lines."""
    command = line.upper().split()[0]
    expected = {"$GETP": 13, "$DUMP": 32}.get(command, 1)
    port.flush_input()
    port.write_line(line)
    deadline = time.monotonic() + timeout
    reply: list[str] = []
    while time.monotonic() < deadline:
        lines, _ = demux.feed(port.read())
        reply.extend(lines)
        if len(reply) >= expected:
            break
        time.sleep(0.005)
    return reply


def flag_names(flags: int) -> str:
    names = []
    if flags & FLAG_ARMED:
        names.append("ARMED")
    if flags & FLAG_CALIBRATION_READY:
        names.append("CAL")
    if flags & FLAG_FAULT_LATCHED:
        names.append("FAULT")
    return "|".join(names) or "none"


TRACE_COLUMNS = [
    "tick",
    "seq",
    "timestamp_us",
    "flags",
    "flag_names",
    "dc_left",
    "dc_right",
    "duty_left_u",
    "duty_left_v",
    "duty_left_w",
    "duty_right_u",
    "duty_right_v",
    "duty_right_w",
]
for _motor in ("left", "right"):
    TRACE_COLUMNS += [
        f"{_motor}_mode",
        f"{_motor}_scheduler",
        f"{_motor}_err_code",
        f"{_motor}_angle_q6",
        f"{_motor}_speed_rpm",
        f"{_motor}_target",
        f"{_motor}_iq",
        f"{_motor}_id",
        f"{_motor}_vq",
        f"{_motor}_vd",
        f"{_motor}_phase_a",
        f"{_motor}_phase_b",
        f"{_motor}_phase_c",
        f"{_motor}_integral_q",
        f"{_motor}_integral_d",
    ]


def trace_row(t: TraceData, seq: int, timestamp_us: int) -> dict[str, object]:
    row: dict[str, object] = {
        "tick": t.tick,
        "seq": seq,
        "timestamp_us": timestamp_us,
        "flags": t.flags,
        "flag_names": flag_names(t.flags),
        "dc_left": t.dc_left,
        "dc_right": t.dc_right,
        "duty_left_u": t.duty_left[0],
        "duty_left_v": t.duty_left[1],
        "duty_left_w": t.duty_left[2],
        "duty_right_u": t.duty_right[0],
        "duty_right_v": t.duty_right[1],
        "duty_right_w": t.duty_right[2],
    }
    for motor_name, motor in (("left", t.left), ("right", t.right)):
        row.update(
            {
                f"{motor_name}_mode": motor.mode,
                f"{motor_name}_scheduler": motor.scheduler,
                f"{motor_name}_err_code": motor.err_code,
                f"{motor_name}_angle_q6": motor.angle_q6,
                f"{motor_name}_speed_rpm": motor.speed_rpm,
                f"{motor_name}_target": motor.target,
                f"{motor_name}_iq": motor.iq,
                f"{motor_name}_id": motor.id,
                f"{motor_name}_vq": motor.vq,
                f"{motor_name}_vd": motor.vd,
                f"{motor_name}_phase_a": motor.phase_a,
                f"{motor_name}_phase_b": motor.phase_b,
                f"{motor_name}_phase_c": motor.phase_c,
                f"{motor_name}_integral_q": motor.integral_q,
                f"{motor_name}_integral_d": motor.integral_d,
            }
        )
    return row


def do_trace(port: SerialPort, demux: StreamDemux, args: argparse.Namespace) -> int:
    decim = 0 if args.decim == 0 else max(args.decim, 160)
    port.flush_input()
    port.write_line(f"$TRACE {decim}")
    demux.start_frames()
    rows: list[dict[str, object]] = []
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        _, frames = demux.feed(port.read())
        for frame in frames:
            if frame.msg_type != MSG_TRACE:
                continue
            data = TraceData.from_frame(frame)
            if data is not None:
                rows.append(trace_row(data, frame.seq, frame.timestamp_us))
        if args.count and len(rows) >= args.count:
            break
        time.sleep(0.002)

    port.write_line("$TRACE 0")
    demux.stop_frames()
    end = time.monotonic() + 0.3
    while time.monotonic() < end:
        lines, _ = demux.feed(port.read())
        for line in lines:
            print(line)

    with open(args.out, "w", newline="", encoding="ascii") as handle:
        writer = csv.DictWriter(handle, fieldnames=TRACE_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"trace: {len(rows)} frames -> {args.out} (decimation {decim})")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial device, e.g. /dev/cu.usbserial-XXXX")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="action", required=True)

    sub.add_parser("help", help="show firmware command list")
    sub.add_parser("get", help="health summary ($GET)")
    sub.add_parser("getp", help="list tunables ($GETP)")
    sub.add_parser("arm", help="bench arm: torque loop live, MOE low")
    sub.add_parser("disarm", help="bench disarm")
    sub.add_parser("adc", help="raw ADC fields")
    sub.add_parser("dump", help="blackbox dump")
    sub.add_parser("save", help="persist tunables to NVM ($SAVE)")
    sub.add_parser("init", help="re-run ADC calibration ($INIT)")

    set_cmd = sub.add_parser("set", help="$SET <KEY> <VALUE>")
    set_cmd.add_argument("key")
    set_cmd.add_argument("value", type=int)
    protocol = sub.add_parser("protocol", help="switch wire contract")
    protocol.add_argument("mode", choices=("legacy", "hardened"))

    trace_cmd = sub.add_parser("trace", help="stream binary FOC trace to CSV")
    trace_cmd.add_argument("--seconds", type=float, default=10.0)
    trace_cmd.add_argument("--decim", type=int, default=160)
    trace_cmd.add_argument("--count", type=int, default=0)
    trace_cmd.add_argument("--out", default="trace.csv")

    args = parser.parse_args()
    port = SerialPort(args.port, args.baud)
    demux = StreamDemux()
    try:
        if args.action == "trace":
            return do_trace(port, demux, args)
        commands = {
            "help": "$HELP",
            "get": "$GET",
            "getp": "$GETP",
            "arm": "$ARM",
            "disarm": "$DISARM",
            "adc": "$ADC",
            "dump": "$DUMP",
            "save": "$SAVE",
            "init": "$INIT",
        }
        if args.action == "set":
            line = f"$SET {args.key.upper()} {args.value}"
        elif args.action == "protocol":
            line = f"$SET PROTOCOL {args.mode.upper()}"
        else:
            line = commands[args.action]
        for reply in request(port, demux, line):
            print(reply)
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    sys.exit(main())
