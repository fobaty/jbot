# Text bridge between a host console and the hoverboard FOC firmware.
#
# The Pico 2 W sits between the Mac (USB) and the hoverboard (USART2). It owns
# the 20 Hz motor frame stream, so the host only has to send readable text
# commands. Deploy this file as `:main.py` to start it on boot:
#
#   mpremote connect $PORT fs cp tools/pico/hover_bridge.py :main.py
#   mpremote connect $PORT reset
#
# Host -> Pico (one line each, newline terminated):
#   C <steer> <speed>   set the command, keeps being sent at 20 Hz
#   S                   immediate stop (zero frames right now)
#   Q                   handshake, the bridge answers with the banner
#   R                   reset the frame counters
#
# Pico -> host:
#   H HOVERBRIDGE 1                       banner, at boot and after `Q`
#   T <spdL> <spdR> <bat*100> <temp*10> <led>   10 Hz, latest telemetry frame
#   K <uptime> <steer> <speed> <fps> <ok> <bad>  1 Hz heartbeat
#
# Safety: if the host stays silent for WATCHDOG_MS the command is zeroed, and
# the firmware itself stops after ~0.8 s without frames. A blocking or dead host
# therefore never leaves the motors spinning.
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

CMD_TICK_MS = 50         # 20 Hz motor frames
TELEM_MS = 100           # 10 Hz telemetry lines to the host
KEEPALIVE_MS = 1000      # 1 Hz heartbeat line
WATCHDOG_MS = 500        # zero the command when the host goes silent
MAX_CMD = 1000

BANNER = "H HOVERBRIDGE 1"

uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1), timeout=0, rxbuf=1024)
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

rx = bytearray()          # raw bytes from the hoverboard
pending = ""              # partially received host line
frame = None              # latest validated telemetry tuple
stats = {"ok": 0, "bad": 0, "fps": 0}
cmd = [0, 0]              # steer, speed actually sent
last_host_ms = time.ticks_ms()


def clamp(value):
    return max(-MAX_CMD, min(MAX_CMD, int(value)))


def checksum(fields):
    chk = 0
    for field in fields:
        chk ^= field & 0xFFFF
    return chk


def say(text):
    try:
        print(text)
    except (OSError, ValueError):
        pass


def send_motor(steer, speed):
    uart.write(struct.pack(CMD_FMT, START_FRAME, steer, speed,
                           checksum([START_FRAME, steer, speed])))


def read_hoverboard():
    """Drain the UART, update the stats and the latest telemetry frame."""
    global frame, rx
    data = uart.read()
    if data:
        rx.extend(data)
    i = 0
    fresh = 0
    while len(rx) - i >= 2:
        if rx[i] != 0xCD or rx[i + 1] != 0xAB:      # 0xABCD, little endian
            i += 1
            continue
        if len(rx) - i < FB_LEN:
            break
        fields = struct.unpack(FB_FMT, bytes(rx[i:i + FB_LEN]))
        i += FB_LEN
        if checksum(fields[:-1]) != fields[-1]:
            stats["bad"] += 1
            continue
        stats["ok"] += 1
        frame = fields
        fresh += 1
    if i:
        rx = rx[i:]
    return fresh


def handle_command(text):
    global last_host_ms
    parts = text.split()
    if not parts:
        return
    op = parts[0].upper()
    if op == "C" and len(parts) >= 3:
        try:
            cmd[0] = clamp(int(parts[1]))
            cmd[1] = clamp(int(parts[2]))
        except ValueError:
            return
        last_host_ms = time.ticks_ms()
    elif op == "S":
        cmd[0] = 0
        cmd[1] = 0
        for _ in range(6):
            send_motor(0, 0)
        last_host_ms = time.ticks_ms()
    elif op == "R":
        stats["ok"] = 0
        stats["bad"] = 0
    elif op == "Q":
        say(BANNER)


def pump_stdin():
    global pending
    while poller.poll(0):
        chunk = sys.stdin.read(1)
        if not chunk:
            break
        if chunk == "\n" or chunk == "\r":
            if pending:
                handle_command(pending)
                pending = ""
        elif len(pending) < 64:
            pending += chunk


def main():
    say(BANNER)
    now = time.ticks_ms()
    last_cmd_ms = now
    last_telem_ms = now
    last_keep_ms = now
    fps_window = now
    fps_frames = 0
    while True:
        pump_stdin()
        now = time.ticks_ms()

        if time.ticks_diff(now, last_cmd_ms) >= CMD_TICK_MS:
            last_cmd_ms = now
            if time.ticks_diff(now, last_host_ms) > WATCHDOG_MS:
                cmd[0] = 0
                cmd[1] = 0
            send_motor(cmd[0], cmd[1])

        fresh = read_hoverboard()
        fps_frames += fresh
        now = time.ticks_ms()

        if time.ticks_diff(now, fps_window) >= 1000:
            stats["fps"] = fps_frames
            fps_frames = 0
            fps_window = now

        if fresh and time.ticks_diff(now, last_telem_ms) >= TELEM_MS:
            last_telem_ms = now
            f = frame
            say("T %d %d %d %d %d" % (f[4], f[3], f[5], f[6], f[7]))

        if time.ticks_diff(now, last_keep_ms) >= KEEPALIVE_MS:
            last_keep_ms = now
            say("K %d %d %d %d %d %d" % (now & 0x7FFFFFFF, cmd[0], cmd[1],
                                         stats["fps"], stats["ok"], stats["bad"]))

        time.sleep_ms(2)


if __name__ == "__main__":
    main()
