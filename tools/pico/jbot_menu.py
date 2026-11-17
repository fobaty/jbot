#!/usr/bin/env python3
"""Interactive terminal menu for the JBOT Pico 2 W bridge.

Inspired by hover_menu.py: a compact, zero-curses text UI running directly in
the terminal over USB serial. It prints a clean status block, streams telemetry,
and lets you drive the robot with W/A/S/D, number presets, space to stop, and
E for emergency stop.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: python3 -m pip install pyserial")

from jbot_link import Link, autodetect_port

PRESETS = (0.2, 0.5, 1.0, 1.5, 2.0)
WIDTH = 78


def clear_screen():
    sys.stdout.write("\x1b[2J\x1b[H")
    sys.stdout.flush()


def print_menu(link, speed, steer, estop, log_active):
    clear_screen()
    tel = link.telemetry
    print("=" * WIDTH)
    print(" JBOT PICO 2 W - INTERACTIVE DRIVE MENU".center(WIDTH))
    print("=" * WIDTH)
    print(" Port: %-25s  Status: %s" % (link.port, "ESTOP ACTIVE" if estop else "READY"))
    print(" Logging: %-15s" % ("ON" if log_active else "OFF"))
    print("-" * WIDTH)

    if tel is not None:
        print(" [ TELEMETRY ]")
        print(" Step: %-8d  Loop: %3d Hz  IMU: %-5s  Cal: %d" % (tel.step, tel.loop, tel.imu, tel.cal))
        print(" Pitch: %+6.2f deg   Gyro Y: %+6.2f dps   Sys Err: 0x%02x" % (tel.pitch, tel.gy, tel.err))
        print(" Mainboard: %-5s  Left: %+5d rpm  Right: %+5d rpm" % (tel.mb, tel.L, tel.R))
        print(" Battery: %5.2f V    Temp: %4.1f C     Age: %s ms" % (tel.bat, tel.tmp, tel.age_text))
    else:
        print(" Waiting for telemetry...")

    print("-" * WIDTH)
    print(" [ COMMANDS ]")
    print(" Target Speed:  %+6.2f m/s" % speed)
    print(" Target Steer:  %+6.1f deg/s" % steer)
    print("-" * WIDTH)
    print(" Controls:")
    print("   W / S or ↑ / ↓ : Speed +/-      A / D or ← / → : Steer left/right")
    print("   1..5           : Presets        Space          : Soft stop")
    print("   E              : Emergency stop C              : Clear e-stop")
    print("   L              : Toggle CSV log Q              : Quit")
    print("=" * WIDTH)
    print(" Enter command or key (non-blocking stdin requires termios; use miniterm or run with python):")


def main():
    parser = argparse.ArgumentParser(description="JBOT interactive menu")
    parser.add_argument("--port", default=None, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    port = args.port or autodetect_port()
    link = Link(port, args.baud)
    try:
        link.open()
    except Exception as e:
        sys.exit("failed to open %s: %s" % (port, e))

    link.write("stream 10")
    link.write("clear")

    speed = 0.0
    steer = 0.0
    estop = False
    log_active = False

    print("Connected to %s. Press Ctrl-C to quit." % port)
    try:
        while True:
            for _kind, _payload in link.poll():
                pass
            print_menu(link, speed, steer, estop, log_active)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            link.write("estop")
            link.write("speed 0")
            link.write("steer 0")
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    main()
