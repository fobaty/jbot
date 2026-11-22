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
mainboard was powered on, because the host did not reassert DTR. The second
attempt bounded every write with a 250 ms timeout; cancelling a write left the
USB endpoint unusable and the console stopped answering after a driver run.
The final version keeps DTR gating for writes but gives a host two seconds to
assert DTR at session start before proceeding, so a re-enumerated device with
a DTR-less host still streams. It was reflashed and used for the yaw check
below; closing the port between host sessions no longer needs a USB replug.

## Session 2 — yaw differential check (no body rotation expected)

Approved command-driven disturbance, robot fixed, wheels in air. The final
console firmware was flashed first; the first two console approaches were
reverted because they could park the console task (see the note above).

`spin-driver --steer --levels 60 --hold-ms 1000` sent `test 60 -60`. The driver
had to wait for the wake pulse's wheels to coast down before its safety gate;
the flashed firmware has no electric brake. Evidence:
`csv/jbot-spin-1795391173099.csv`.

- Both wheels counter-rotate (hall readings `+71 / +82 rpm` over the first
  300 ms), confirming a pure `steer` command with zero common mode.
- Early-plateau acceleration is about 25 rad/s² (left) and 29 rad/s² (right),
  matching the 25-31 rad/s² measured for common-mode `test 60` in WP0. With the
  old unit-coefficient inverse the differential command would have produced
  roughly half that. The yaw-authority fix is therefore confirmed at the wheel.
- `gz` stays within +/-2.25 dps and heading is unchanged: the body is
  mechanically fixed, so it cannot yaw. A hand-held attempt is recorded in
  session 3 below.

## Session 3 — hand-held body-yaw attempt (inconclusive)

The operator held the body with both hands, wheels in the air and the rope in
place. The same `test 60 -60` command was repeated:
`csv/jbot-spin-1795391348038.csv`.

- Wheels counter-rotated as expected (about `+170 / +185 rpm` 0.6 s into the
  plateau), so the command path was active.
- `gz` stayed within +/-0.8 dps and heading did not change. The body did not
  rotate measurably: a two-handed grip still constrains the machine enough to
  absorb the roughly 0.7 Nm reaction torque. This is an operator/setup limit,
  not a failure of the yaw fix, which is already confirmed at the wheel.
- A free-suspension setup (single sling or pendant) would be needed to observe
  body yaw; it was not attempted in this session.

## Gate

The link check and the wheel-level yaw check pass. Body-yaw verification is
unconfirmed because the test setup constrained the body; it needs a free-hanging
setup and fresh step-by-step approval before any further trial.
