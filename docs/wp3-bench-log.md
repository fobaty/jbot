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

## Session 6 — guided tip-over repeat (still guard-ended)

The revised firmware (12-degree reference, 1.5 s linear torque fade) was
flashed. The mainboard had slept in the meantime and was woken with a 2-3 s
button hold; `fall_driver --run --record-ms 8000` then armed and requested the
fall with the robot upright on the floor, rope and hands on the body. Evidence:
`csv/jbot-fall-1795392571341.csv`.

- The mode entered correctly (`opm=4`, reference 0.45 to 8.26 degrees, torque
  peak 426 units) but lasted only about 0.52 s. The torque fade had reached
  roughly 65 percent; it did not complete.
- Pitch then swung to about -26 degrees and the guard ended the attempt. Peak
  pitch rate was about 533 dps, so the body was caught and righted by the
  operator. The rope-on-floor rig does not produce a repeatable, bounded fall:
  the direction and timing depend on how the operator holds the machine.
- Conclusion: the firmware behaviour is as designed, but the test method cannot
  demonstrate a controlled landing. Further bench work on the fall needs a
  rig that fixes the pivot and release conditions, not another hand-held trial.

## Session 7 — WP3 gate attempt (commanded-disturbance recovery)

A new `recovery_driver` was written to close the formal WP3 gate: arm the
balance law, apply N identical pitch-reference steps with `home`, return the
reference to the measured pitch and require recovery without a fall or a
guard latch. It ends every run with `estop`. Several setup faults were found
and fixed in the driver itself: an unguarded `level` recapture while the body
was tilted, a stability check that could not see telemetry lines, and a pitch
reference that did not match the measured attitude at `arm`.

Evidence from the series (all `csv/jbot-recovery-*.csv`):

- With the firmware defaults (`pscale=1.0`, `slew=3000`) the machine fell
  within roughly 300 ms of `arm`. The audit records that every stable armed
  run used `pscale 0.10-0.30` and that the compiled defaults are not a bench
  profile. This was a test-setup error, not a firmware defect.
- With a reference snapped to the measured pitch the machine held perfectly
  while it was still supported (`pitch=-3.99` degrees, torque within +/-4
  units for 1.7 s), confirming the law behaves correctly at zero mismatch.
- With the safe bench profile (`pscale=0.30`, `slew=100`) the machine stayed
  armed for about 4 s and repeatedly recovered from excursions past 20 degrees
  (guard state moved from recover back to hold several times). It never
  settled inside the +/-5 degree baseline window: it oscillated with a roughly
  one-second period while position drifted to -0.42 m, then latched off. The
  gate's stability precondition could not be met, so no disturbance trial ran.

### Outcome

The formal WP3 gate remains **not met**. The results point to two independent
gaps rather than a single bug:

1. The gain ladder has not been re-run on the current wiring (the audit notes
   the `pscale` ladder has to be repeated after the sign/derivative correction).
   `pscale 0.30` is a safe identification profile, not a tuned balance profile.
2. The rope-on-floor, hand-held setup cannot release the body reproducibly or
   hold a fixed pivot, the same limitation that blocked the fall trials.

Further recovery trials are not recommended until both a tuned gain profile
and a repeatable release fixture exist. The driver and its fixes are kept as
the tool for that future work.

## Session 8 — pscale ladder, hold gate

The ladder procedure in `docs/pscale-ladder.md` was started with a 10 s
undisturbed hold at `pscale 0.25`. Two runs were recorded:

- `csv/jbot-recovery-1795394005553.csv` (`slew 200`): failed. Pitch reached
  33.6 degrees, travel 0.46 m and the guard latched. `jbot-analyze` found a
  regular 1.22 s oscillation (p2p 45.6 degrees, 1.64 crossings/s) and a poor
  law fit (kp 0.5 Nm/rad, R2 0.37), with only 92 units of peak torque and no
  saturation. The fitted dead zone was about 61 units.
- `csv/jbot-recovery-1795394078817.csv` (`slew 100`): **hold gate passed**.
  11.8 s armed, pitch 1.3-4.1 degrees (p2p 1.3, std 0.6), zero travel, no fall
  and no latch. The fit is now credible: kp 6.2 Nm/rad, kd +1.36 Nm/rad,
  R2 0.94, residual 2 units. The axis check is inconclusive only because the
  body barely moved.

A driver bug was found during this session: an out-of-range `--slew` value was
silently ignored, so a request for 500 actually ran at 100. The driver now
clamps and reports the applied value. The passing result is therefore labelled
`slew 100`, and the procedure starts at that value.

### Commanded-disturbance attempts at 0.25

- `csv/jbot-recovery-1795394161845.csv` (`home 3 deg`, three trials): the first
  step drove pitch to 33.5 degrees, travel 0.37 m and disarmed the law. `jbot-analyze`
  found a 1.57 s oscillation with 42.6 degrees peak-to-peak and a poor law fit
  (R2 -1.8). Trials two and three ran with zero torque because the law was
  already off; the driver now aborts the series on disarm.
- `csv/jbot-recovery-1795394215846.csv` (`home 1 deg`): trial one peaked at
  17.6 degrees and travelled 0.27 m but stayed armed, though it did not settle
  inside the 3 degree residual window. Trial two peaked at 35 degrees,
  travelled 0.52 m and latched. The 1 degree disturbance is therefore not
  repeatable either.

The machine can hold still at `pscale 0.25 / slew 100` but cannot reject even a
minimal reference step twice in a row. The fitted `kp 6.2 Nm/rad` leaves little
authority against gravity at a few degrees of error, so the next step in the
ladder is a single rung to `pscale 0.30` with `slew` held at 100.

### Next rung: 0.30

`csv/jbot-recovery-1795394255629.csv` (`pscale 0.30 / slew 100`, hold only):
the machine never settled. Pitch oscillated between about +18 and -25 degrees
with a roughly 1.2 s period while position drifted to -0.44 m, and the guard
latched at the end. This reproduces the 0.30 behaviour from session 7, so the
rung is rejected; the ladder cannot advance past 0.25 on this setup.

### Ladder outcome

The ladder stops at `pscale 0.25 / slew 100`. That profile can hold still but
cannot reject even a 1 degree reference step twice, and the next rung is
unstable. The recordings repeatedly report a loaded breakaway around 47-61
units (0.23-0.30 Nm nominal), which is not in `PlantParams::NOMINAL`; until it
is measured and the gains are re-derived, raising `pscale` is guesswork. A
repeatable release fixture is still needed as well.

### Feedforward A/B and disturbance retry

With the corrected +3.23 units/degree feedforward, `csv/jbot-recovery-1795396542330.csv`
shows a better hold than the `ff off` baseline: pitch p2p 1.2 degrees versus
1.9, no oscillation and 0.000 m travel.

`csv/jbot-recovery-1795396635001.csv` repeated the 1 degree disturbance series
with `ff on`. Two of three trials passed: pitch stayed within 2.6-3.3 degrees
with no travel, no latch and residual pitch error under 1 degree. The third
trial peaked at 36.8 degrees and the guard latched. The offline score reports
a 42.7 degree peak-to-peak excursion over the whole run and one latch.

This is an improvement from 0/3 before feedforward, but the WP3 gate requires
every trial to recover: it is still not met.

## Gate

Link check, wheel-level yaw, body-level pivot and the 10 s hold gate at
`pscale 0.25 / slew 100` pass. With feedforward enabled, two of three 1 degree
disturbances recover, but the commanded-disturbance gate is not met. The gain
ladder is blocked at 0.25 and controlled fall remains blocked on test-rig
quality.
