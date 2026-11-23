# Command latency and slew isolation (2026-11-23)

Status: host-side measurements only. No change to the 200 Hz balance law.
Every run used the existing `test` command with the law disarmed and ended with
`estop`. The robot was held with the rope attached, hands on the body and the
e-stop reachable.

## Console blocker first

Streaming telemetry at 200 Hz and then closing the host port parked the USB
console task in `write_packet`: the IN endpoint stays enabled after a close, so
`read_packet` never reports the disconnect and a write already in flight waits
forever for a drain that no longer happens. This is why several driver runs had
to be preceded by a USB replug. The console now bounds every write (100 ms for
telemetry, 1 s for replies), drops a line on timeout and ends a session when
DTR drops from an asserted state. Three close/reopen cycles were verified
without re-enumeration. Root cause and history: `docs/wp3-bench-log.md`.

## Torque scale

No new measurement. The provisional value from the 40-unit spin-up step remains
**175–185 command units/Nm total** against the 200 units/Nm design assumption
(`docs/audit-2026-11-22.md` §6). Treat newton-metre figures as a scale aid until
a loaded measurement exists.

## Breakaway / dead zone

`csv/jbot-latency-1795450047466.csv`:

* 20 units produced no hall feedback at all, with the wheels free and the body
  held on blocks.
* 40 and 60 units turned the wheels, with breakaway in the range 20–40 units
  for this setup.

This is the **free-wheel breakaway**, not the loaded dead zone of a machine
standing on its wheels. Historical loaded brackets are above 120 units
(`docs/audit-2026-11-22.md` §6) and the dynamic gyro-null search found 53.8 units
(`csv/jbot-gyronull-1795399203498.csv`) under a different load condition. The
loaded dead zone must be measured with the wheels on the floor and the body
held upright; it is still not a single confirmed number.

## Command-to-torque delay

Same recording, telemetry at 200 Hz (5 ms per control tick):

* Host command to applied spin torque (`tsl`): the first nonzero sample appears
  within one telemetry sample. Both the reply-based lower bound and the
  send-based upper bound resolve to 0.0 ms, i.e. **one 5 ms tick at telemetry
  resolution**. A logic analyzer would be needed to separate the host USB leg
  from the control tick; none is available.
* Applied torque to first wheel motion (hall feedback): **190 ms at 40 units**
  and **180 ms at 60 units**. This includes the EFeru input filter (about 47 ms
  time constant), its input ramp, the wheel/body mechanics and the 100 Hz
  feedback frame. It is a feedback-observable proxy, not a direct
  command-to-motor-current measurement.
* GP10 cadence proxy: the firmware `loop` field stayed 199–200 Hz and the step
  counter advanced at 199.5–199.9 Hz in the probe windows. No logic-analyzer
  capture of GP10 exists, so tick jitter is still unmeasured directly.

The slow torque-to-wheel response is consistent with the `home 3` divergence at
`pscale=0.30 / slew=100` in the audit: a 20-degree error requested several
hundred units, but the applied torque never exceeded 80 units. Isolating `slew`
is therefore the next controlled experiment, not raising `pscale`.

## Host tools added

* `crates/jbot_console/src/bin/latency_cal.rs` — command-to-torque and
  torque-to-wheel timing, waits for the wheels to stop between levels.
* `crates/jbot_console/src/bin/hold_sweep.rs` — per-profile `pscale`/`slew`,
  `level` capture, arm, 10 s undisturbed hold, audit gate, `estop`, no firmware
  change. Not yet run; it needs mainboard feedback, wheels on the floor and a
  fresh safety confirmation.

Both tools stop the telemetry stream before closing the port. Quality gates
(`fmt`, `check`, `test`, `clippy -D warnings`) pass for the workspace and the
Pico firmware.
