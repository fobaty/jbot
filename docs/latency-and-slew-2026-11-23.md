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

## Hold sweep results

All runs used `hold_sweep` with runtime settings only; `level` was captured at
the quiet baseline and every profile ended with `estop`.

| Profile | Result | Peak torque | Wheel activity |
|---|---|---|---|
| `pscale 0.25 / slew 100` | PASS (10 s, pitch error ≤1.31°, travel 0) | 33 u | none |
| `pscale 0.25 / slew 300` | fall | 80 u | short burst |
| `pscale 0.25 / slew 1000` | fall | 219 u | short burst |
| `pscale 0.30 / slew 300` | fall | 111 u | short burst |
| `pscale 0.40 / slew 300` | fall | 89 u | short burst |

The passing run is **not a balance demonstration**: hall feedback stayed at
zero for the whole 10 s and travel was 0, so the body was supported externally
(rope/hands). Peak torque of 33 units is below the loaded breakaway, so the
wheels could not have corrected anything. The audit's hold gate alone cannot
distinguish this from real balance; `hold_sweep` now reports a separate
`wheel_active_samples` authority metric.

When the body was genuinely free, even briefly, every profile fell within a
few seconds. The rope-held body swings by ±10° before `arm`, and tightening the
quiet gate to 3°/5 dps made the driver find no baseline at all. Raising `slew`
at fixed `pscale` increases the torque but makes the divergence faster. The
gain ladder cannot be climbed with this release setup.

**Conclusion:** `pscale 0.25 / slew 100` satisfies the numeric 10 s gate with
external support, but no profile has demonstrated self-balancing. A repeatable
release fixture and a loaded breakaway measurement are the blockers. Do not
raise gains or disable guards to force a pass.

## Root-cause pass (offline telemetry fits)

* Hall step fit on `csv/jbot-latency-1795450047466.csv`: after the Pico applies
  torque there is a **190 ms (40 u) / 230 ms (60 u) dead time** before hall
  feedback moves, followed by a first-order settle with **τ ≈ 250 ms**. The
  EFeru input filter (`FILTER=6553`, α=0.1 at 200 Hz, τ≈47 ms, ~140 ms to 95 %)
  plus the 100 Hz feedback frame and wheel mechanics account for that dead
  time. This is the dominant dynamic lag, not the 5 ms Pico tick.
* Cross-correlation of `gy` against `d(pitch)/dt` on `csv/jbot-hold-1795451188996.csv`:
  best lag 0 ms, correlation 0.93. BNO055 fusion lag is not the bottleneck here.
* Loaded breakaway bracket from the hold runs: 33 u gave `rpm=0` for the whole
  10 s, 69 u turned the wheels (peak 71 rpm). Together with the dynamic search
  (53.8 u), the loaded dead zone is about **34–69 u**, near 54 u. At `pscale
  0.25` a 2° error commands only ~34 u, i.e. below breakaway — below this gain
  the law cannot correct small errors at all.
* Combined failure mode: dead zone forces a gain high enough to cross it, while
  the ~200 ms mainboard lag makes that gain oscillatory. Raising `slew` only
  makes the divergence faster (see the table above).

**Prepared change (not flashed):** `FILTER=19661` (α=0.3, τ≈14 ms) in the
`VARIANT_USART` block of `firmware/stm32f103/eferu/Inc/config.h` and in the
reference snapshot. This is a mainboard reflash, needs its own approval, and
must not be pushed to upstream EFeru. After measuring the new dead time the
`pscale` ladder can be repeated on a sound basis.

Both tools stop the telemetry stream before closing the port. Quality gates
(`fmt`, `check`, `test`, `clippy -D warnings`) pass for the workspace and the
Pico firmware.
