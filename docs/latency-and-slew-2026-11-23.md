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

## Mainboard filter change (flashed 2026-11-23)

`FILTER=19661` (α=0.3, τ≈14 ms) was applied in the `VARIANT_USART` block of
`firmware/stm32f103/eferu/Inc/config.h`, the reference snapshot was updated and
the mainboard was reflashed with `make -e VARIANT=VARIANT_USART && make flash`.
The nested EFeru repository was not pushed.

With the robot back in the same upright, wheels-supported pose, the
command-to-wheel dead time fell from 180–190 ms to **115–145 ms**
(`csv/jbot-latency-1795452335958.csv`). The first-order fit remains noisy
(L 145–245 ms, τ about 295 ms) because the wheel spin-up is not a pure first
order response, but the first-motion metric is consistent.

The gain ladder was repeated with the faster filter:

| Profile | Result | Peak torque | Wheel activity |
|---|---|---|---|
| `pscale 0.30 / slew 300` | fall | 98 u | yes |
| `pscale 0.40 / slew 300` | fall | 88 u | yes |
| `pscale 0.25 / slew 300` | fall | 112 u | yes |

All three genuinely drove the wheels (unlike the earlier supported PASS), but
none held for 10 s. The robot left the hand at a large angle and reached the
fall guard within a few seconds. Repeating the release several times produced
the same result.

**Final diagnosis:** the Pico 200 Hz loop, IMU freshness and the mainboard
filter are now measured and none is a 200 Hz-class bottleneck. The remaining
blockers are the loaded dead zone (~54 u) combined with an operator-held rope
release that starts each trial with a several-degree swing, plus a ~5.5 Nm
torque budget. A repeatable release fixture is required before any further
gain claim. Do not change guards or the 200 Hz law to force a pass.

Both tools stop the telemetry stream before closing the port. Quality gates
(`fmt`, `check`, `test`, `clippy -D warnings`) pass for the workspace and the
Pico firmware.

## Dead-zone compensation experiment (2026-11-23, afternoon)

### Changes

* Pico firmware flashed with a runtime `db <Nm|off>` knob stored in
  `state.rs`. It defaults to zero, is runtime-only, and is applied through
  `Tuning::with_deadband()` on every control tick. Guards, e-stop, the torque
  budget and the structure of the 200 Hz law are unchanged. `stat` reports
  `db=`; `db off` reverts instantly without reflashing.
* `PlantParams::NOMINAL.deadband_nm` corrected from 0.5 to 0.27 Nm to match the
  loaded bench measurement. A unit test confirms the override survives power
  scaling and produces no torque at zero effort.
* `hold_sweep` gained `--db`, `--release-file`, `--ready-file`,
  `--quiet-pitch`, `--quiet-gy` and `--release-timeout-ms`. Release is detected
  from telemetry when no marker file is supplied, and a missing cue ends the
  profile with `estop` instead of leaving the robot armed. Opening pitch rate
  above 25 dps marks the trial `VOID - release disturbance`.
* `gyro_null_cal::angular_acceleration()` was missing the milliseconds-to-
  seconds conversion. Its `alpha` column is now in dps² as declared, so the
  `tol_alpha` quiet gate is effective. The earlier 53.8 u dynamic search was
  taken with a gate that was almost always satisfied.

### Re-measured dead zone

With the corrected `alpha`: first wheel motion at **+41.4 u**, a stable signal
at **53.8 u**, and a torque null of **56.5 u** with zero pitch slope
(`T(pitch) = -0.00 * pitch + 56.5`). The holding torque is stiction, not
gravity. See `csv/jbot-gyronull-1795454459188.*`.

### Hold results

| Profile | Result | Notes | Recording |
|---|---|---|---|
| `db 0.00`, body supported | PASS, authority NO | peak torque 4-11 u, wheels never turned | `jbot-hold-1795454483436`, `...4501882` |
| `db 0.00`, wheels loaded, hand release | FAIL | pitch 39.93 deg, wheel-active 194 | `jbot-hold-1795454604294` |
| `db 0.27`, wheels loaded, hand release | FAIL at 1.2 s | torque injected at 1.4 deg error, pumping | `jbot-hold-1795454698802` |
| `db 0.00`, 80 s release wait | void | robot fell while armed, 1 recorded sample | `jbot-hold-1795454968091` |
| `db 0.00`, cued release, 3 deg / 3 dps | **PASS, authority YES** | 1599 samples, pitch max 1.38 deg, travel 0.011 m, torque 39 u, wheel-active 83 | `jbot-hold-1795455254278` |
| `db 0.10`, cued release | FAIL at 1.4 s | `gy` jumped to 8.5 dps 0.2 s after arm | `jbot-hold-1795455284671` |
| `db 0.00`, 2 deg / 2 dps gate | skipped | no quiet baseline in 10 s | `jbot-hold-1795455447727` |
| `db 0.00`, cued release repeat | FAIL | calm to 9 s, then `gy` 5.9 -> 134 dps, pitch 25 deg | `jbot-hold-1795455497614` |

### Conclusion

One full 10 s hold with real wheel authority was recorded at `db 0.00`,
`pscale 0.30`, `slew 300`. It is evidence that the existing law can stand the
machine, but it is **not repeatable yet**: the successful and the failed
baseline runs differ only in an operator disturbance at the release instant.
`db 0.10` and `db 0.27` did not help and showed the relay-like pumping the
original `Tuning::from_plant` comment warned about, so dead-zone compensation
stays disabled. No gain, guard or fail-safe change was made to force a pass.

A repeatable release fixture is still required before claiming the WP3 gate.
Two designs are described in `docs/release-fixture.md`: a screw-driven cradle
and a cut-the-thread release using the existing rope and a fixed anchor. The
operator declined to build either, so no further trials were run. Do not treat
the single PASS as a closed bench gate.

All quality gates pass after the experiment. No push was made.

## Second bench session (2026-11-23, after IMU remount)

### Axis and sign verification

`csv/jbot-axes-1795458719581.csv` (6786 samples) confirms the remounted IMU is
correctly paired: forward lean raises `pitch` to +23.9 deg, backward lean lowers
it to -28.9 deg, and `d(pitch)/dt` correlates with the reported pitch rate at
+0.896 with slope +0.838. The cross-axis fit against the roll rate is +0.376
with slope +0.023. `mount=0` (Normal) is therefore the right selector and no
`mount.rs` change is needed.

`csv/jbot-spintest-1795459715266.csv` resolves the yaw sign: `test 100 -100`
gives `gz` of -460 to -700 dps. With `+steer` mapped to left-wheel-forward and
right-wheel-backward, the physical turn is clockwise, so a negative `gz` is the
correct reading. The yaw damper sign is not inverted.

### Calibration and driver fixes

- Re-captured `zero` and `level` and saved NVM: `zero=0.12 deg`,
  `zroll=0.88 deg`, `level=-0.06 deg`, `zcapt=1`, `lcapt=1`.
- Found and worked around an NVM restore gap: boot restores the pitch zero only
  from `pose_home`, so `forget` followed by `zero`/`level` leaves `zcapt=0`
  after the next reboot and the arm gate refuses. `hold_sweep` now captures
  `zero` and `level` at its own quiet baseline before arming. A firmware fix
  restoring `zero_pitch_rad` directly is still pending.
- `hold_sweep` now requires wheel motion: a body held still by the rope was
  scoring `pass: true` with `wheel_active_samples=0`. The authority threshold
  is 20 samples and is covered by two unit tests.
- `hold_sweep` can detect the release from telemetry (pitch jump or wheel
  motion) when no sentinel file is supplied, removing dependence on operator
  terminal access and chat latency.

### Re-examination of the earlier "PASS"

`csv/jbot-hold-1795455254278.csv` was re-read sample by sample. The common
torque stayed between -21 and +12 units during the whole window and the wheel
speeds were at or near zero. The recorded "authority YES" came from a few
isolated 5 rpm encoder samples, not from sustained wheel corrections. That run
does **not** demonstrate self-balancing. No run in the repository does.

### New gain trials and the limiting mechanism

| Profile | Result | Recording |
|---|---|---|
| `0.25 / 100` | rope-supported, torque 33 u, wheels never moved | `jbot-hold-1795459088188` |
| `0.25 / 100`, support removed | fell, torque 18 u, below the ~54 u breakaway | `jbot-hold-1795459204773` |
| `0.40 / 300` | fell during release wait, torque 25 u | `jbot-hold-1795459471937` |
| `1.00 / 1000` | torque reached 177 u, body tumbled | `jbot-hold-1795459672170` |
| `0.60 / 1000`, auto release | torque 205 u, wheel authority yes, fell in 0.6 s | `jbot-hold-1795460015877` |
| `0.60 / 3000`, auto release | fell within 0.25 s of arm | `jbot-hold-1795460077000` |

All three auto-release trials start their scored windows at 130-330 deg/s of
opening pitch rate, injected by the operator's hand before the loop can react.
`hold_sweep` now grades the opening rate and labels such trials
`VOID - release disturbance` instead of `FAIL`, so they are not mistaken for a
control-law result. A usable trial needs a fixed-anchor release; see
`docs/release-fixture.md`.

The `0.60 / 1000` recording shows the control law sign is correct: torque rises
with `pitch` and reverses when `gy` changes sign. The failure is timing. At
`slew=1000` a torque reversal from -105 to +10 units takes about 0.1 s; with
the measured 115-190 ms mainboard delay the total loop lag is near 0.25 s,
against an inverted-pendulum time constant near 0.3 s. Raising the gain alone
cannot close that gap.

### What remains

1. ~~Reduce the mainboard command-to-torque delay below roughly 80 ms.~~
   Re-assessed below: the mainboard firmware contributes only about 20-30 ms;
   the rest of the measured delay is the Pico torque ramp, the loaded dead zone
   and wheel mechanics. Runtime `slew`/`db` are the right knobs; a mainboard
   reflash for the remaining ~7 ms is not justified. Moving the inner torque
   loop onto the mainboard stays a larger, separate option.
2. ~~Restore `zero_pitch_rad` from NVM at boot even without `pose_home`.~~ Done
   and flashed: a `zero captured` flag in the NVM flags byte lets boot restore
   the stored pitch zero; legacy records fall back to `pose_home`. Verified
   end to end after `reboot` (`zcapt=1`, `zero=-5.06 deg` restored).
3. Build the vertical release fixture from `docs/release-fixture.md`; hand
   releases still dominate every scored window.
4. Only then repeat the gain ladder. Do not treat any current PASS as WP3
   acceptance.

## Third bench session: latency re-measured (2026-11-23)

### Link health artefact

A `latency_cal` run was followed by a stale feedback link: `fb` froze, `age`
grew past 170 s and `fb_ok=0`, while `rx` kept rising at exactly 1600 B/s,
which is 200 command frames/s times the 8-byte frame length. A Pico `reboot`
did not recover it. The cause was external: the mainboard had been
disconnected. After reconnection `fb_ok=1`, `fb_drop=0`, `age=0` and the
feedback cadence was normal. No firmware or parser change was needed.

### Free-wheel re-measurement

`csv/jbot-latency-1795463791282.*` repeats the earlier probe with the wheels
on a stand, which removes floor friction from the breakaway threshold:

| Applied spin torque | Now | Earlier session | Pico 600 u/s ramp |
|---|---|---|---|
| 40 u | 185 ms | 190 ms | 67 ms |
| 60 u | 155 ms | 180 ms | 100 ms |
| 80 u | 155 ms | - | 133 ms |

Host command to applied `tsl` still resolves to 0 ms at 5 ms telemetry
resolution, `loop` stayed 199-200 Hz and feedback `age` stayed at 0-10 ms
throughout both sessions. The filter change to `FILTER=19661` therefore helped
by about 25 ms at 60 u and almost nothing at 40 u. A second run with the wheels
on the floor (`csv/jbot-latency-1795461821436.*`) measured 160-170 ms at 80 and
120 u and is not comparable to the free-wheel numbers because of the loaded
breakaway.

These are spin-path numbers: they include the fixed 600 u/s ramp. The balance
law applies torque through the `slew` knob (3000 u/s by default, five times
faster), so its command-to-wheel delay is correspondingly shorter.

### Source-level lag budget

The mainboard control loop runs every 5 ms (200 Hz), `readCommand()` consumes
DMA frames on each idle gap at the same rate, and `rateLimiter16` with
`RATE=480` fixdt allows about 6000 u/s, which does not limit our steps. The
`FILTER=19661` low pass has a time constant near 14 ms; raising it to the
maximum useful 0.5 would only reach about 7 ms. Feedback frames are sent every
10 ms (100 Hz). The remaining delay in the historical 115-190 ms figure is the
Pico torque ramp, the roughly 54 u loaded dead zone, the FOC current loop and
wheel mechanics. Runtime `slew` and `db` address those without a reflash, and
the release fixture remains the gate before any control conclusion.
