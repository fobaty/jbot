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
- A setup that leaves the body free to rotate was needed; the ground-contact
  pivot in session 4 provided it.

## Session 4 — ground-contact pivot, body yaw confirmed

The operator placed the wheels on the floor and used the rope only to arrest a
fall or runaway, leaving the body free to rotate; zone at most 0.5 m. The same
`test 60 -60` command was repeated: `csv/jbot-spin-1795391494838.csv`.

- During the 0.9 s plateau the heading changed by **+103.6 degrees** (unwrapped
  from `0.75` to `104.38`), with `gz` peaking near `-250 dps`. The body kept
  rotating after the command was released while the wheels coasted.
- Wheel hall speeds stayed modest (`+42 / +58 rpm` late in the plateau), so
  this was a controlled low-torque pivot, not a runaway.
- This confirms the yaw-authority fix at body level: the corrected inverse
  produces a real in-place turn from the same 60-unit differential command.

## Session 5 — first SafeFall attempt (not a soft landing)

SafeFall firmware was flashed, then `fall_driver --run --record-ms 8000`
executed `clear`, `level`, `arm` (confirmed `arm=1`) and `fall` with the robot
upright on the floor, rope and hands on the body, zone at most 0.5 m. Evidence:
`csv/jbot-fall-1795392138647.csv`.

- The mode entered correctly: `opm=4`, grace counting down from 6.00 s, and the
  pitch reference ramped from 1.26 to 5.69 degrees at the configured 15 deg/s.
- It lasted only about 0.3 s. Peak commanded torque was 438 units (about 2.5 Nm),
  but the body passed 71 degrees and the guard latched `Lying`; the mode ended
  on the guard, not on reaching the 60-degree target. The driver then sent
  `estop` as designed.
- This is consistent with WP1 and the WP0 torque budget: beyond roughly 25
  degrees, gravity torque (`m g L sin(theta)`) exceeds the 5.7 Nm available, so
  the machine falls faster than a 15 deg/s reference can follow. A true soft
  landing at the pillow angle is **not achievable** with this actuator; a
  different mechanism or a revised definition of "controlled fall" is needed.

## Gate

Link check, wheel-level yaw and body-level pivot pass. SafeFall is implemented
and flashed but does not produce a soft landing; further work needs an
Architect decision on the fall definition, plus fresh step-by-step approval.
