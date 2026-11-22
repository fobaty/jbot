# WP3 bench log (2026-11-22)

## Session 1 — firmware update and link check (no torque)

Safety state: robot mechanically fixed, both wheels free in the air, test zone
at most 0.5 m, hands on the body. No torque command was sent in this session.

Actions:

1. Pico reflashed with the WP2 firmware (`picotool load -u -v -x -t elf`,
   verified OK) after holding BOOTSEL while reconnecting USB. Mainboard was off
   at that time; telemetry showed `mb=none`, `bat=0.00` as expected.
2. Mainboard woken with a 2-3 s power-button hold until the beep. The Pico
   re-enumerated on USB. Telemetry after that: `mb=ok`, `bat=39.35 V`,
   `age=0..5 ms`, `estop=0`, `mode=0`, `opm=0`, `gtlt` follows pitch, `grace=0`.
   Imu is fresh, 200 Hz loop stable. No arm and no `test` command were issued.
3. USB console reconnect checked: closing the host port and opening it again
   resumed the banner and the telemetry stream without touching the USB cable.

### Console fix correction

The first reconnect fix gated the console on DTR. That worked for a plain port
close/reopen but blocked the console after the Pico re-enumerated when the
mainboard was powered on, because the host did not reassert DTR. The Pico had to
be replugged to recover. The gate was removed; instead every console write is
bounded by a 250 ms timeout and a dead session is dropped after a short pause.
The revised firmware was reflashed and the reconnect check passed. The ending
state of the first fix is recorded as a reverted approach, not as a result.

## Gate

The link check passes. The next step — a command-driven low-torque rotation to
confirm the corrected yaw differential — needs fresh step-by-step approval and
is not part of this log.
