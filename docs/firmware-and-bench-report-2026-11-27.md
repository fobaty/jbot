# Firmware changes and bench report — 2026-11-27

## 1. What changed in the Pico firmware

Final flashed image: `firmware/pico2w/fw/pico2w-lag-fix.uf2`,
SHA-256 `a0893f424ff8be965b2dcf9fc51772f22cf83e01d27d4118bc6dc5e682dc7abe`.
It boots, answers `ping`, and all new console knobs reply correctly.

Changes relative to the previous working image:

* `D_FILTER_CUTOFF_HZ = 0.0`. `LowPass::new` maps a non-positive cutoff to
  `alpha = 1.0`, so the derivative term sees raw mapped gyro. The 12 Hz
  pitch-rate filter was removed from the balance path. This matches the
  mjbots hoverbot PID, which uses the raw gyro rate directly.
* Gyro propagation between 100 Hz IMU samples: `propagate_pitch_deg()` in
  `core/src/health.rs` carries the fused pitch across the intervening 200 Hz
  control ticks using the mapped gyro rate. A fresh fused sample always wins,
  so propagation cannot accumulate fusion drift.
* Health-gated hardware watchdog: `watchdog_feed_allowed()`
  (`core/src/health.rs`) feeds only with a fresh IMU sample; wheel feedback is
  additionally required while armed. A feedback gap while disarmed still
  feeds, so a benign link gap does not reset the board.
* Runtime-only bench knobs added to the console: `iscale`, `posg`, `velg`,
  `demlim`. They let `--minimal` disable the integral and outer loops without
  changing compiled defaults. They are dispatched from a single compact match
  arm; four separate large handlers caused the shared task stack to overflow
  at startup, which is why earlier builds of this firmware never booted.
* The final image is built on the known-good `8f95b46` flat-layout base.
  Builds from the later `firmware/pico2w/fw` workspace failed to boot on this
  board; bisection isolated the failure to the console handlers, not to the
  control code or the watchdog gate.
* Feedback decoding now reads `applied_left_units` / `applied_right_units`
  from the mainboard instead of the old command echo.
* `rate400` build option: 2.5 ms control tick, `MAX_STREAM_HZ` follows
  `CONTROL_HZ`. Artifact
  `firmware/pico2w/fw/pico2w-lag-fix-400hz.uf2`,
  SHA-256 `38409664147c8a7d7a72edb5cf008d8b75cd30a500d9ed0db10a76ce2b17a576`.
  Built and verified, not flashed.

Unchanged on purpose: the 4 Hz guard-tilt filter (outside the torque path),
ADC/battery/temperature filters, the FOC current filter, the torque rate
limiter, all safety guards and the e-stop.

## 2. What changed in the STM32F103 firmware

Flashed: `firmware/stm32f103/eferu-foc/eferu/build/lag-fix/hover.bin`,
SHA-256 `9867b4aa8965dd9cd300d5c35be08833d46f03e494b61d1019bbb262f5a205db`,
40,328 bytes, written and verified with OpenOCD over ST-Link.

* `FILTER = 65535`: the fixed-point input low-pass becomes a one-tick
  pass-through. The previous active value was `19661` (alpha 0.3 per 5 ms,
  tau about 14 ms); the upstream default is `6553` (tau about 47 ms).
* `SERIAL_TIMEOUT = 20` — 100 ms command timeout.
* `INACTIVITY_TIMEOUT = 0`, compile-time guarded so it cannot power the board
  off immediately.
* Preserved: TRQ_MODE, `I_MOT_MAX = 10 A`, mixer, 16 kHz PWM/FOC, the 18-byte
  feedback frame at 100 Hz, the rate limiter and anti-windup.

A second patch, `firmware/stm32f103/eferu-foc/fast-command.patch`, moves
setpoint conditioning from the 5 ms main loop to a 1 kHz tick. USART2 parsing
was already asynchronous in the IRQ. Binary
`build/fast-command/hover.bin`, SHA-256
`95630d10a74b54ee13a590909d6cbf2e86e7dcc37d2242741ac2753333cd7b92`.
It improves command-to-setpoint refresh from at most 5 ms to at most 1 ms.
It was built and verified but not flashed, because the conservative pairing
was tested first.

## 3. Host tooling changes

* `crates/jbot_console/src/bin/disturbance_probe.rs`: `--minimal` sends
  `ff off`, `db off`, `cal off`, `dither off`, `iscale 0`, `posg off`,
  `velg off`, `demlim off`.
* `bench-temp/runtime-lag-probe.sh`: profiles A–F with read-only preflight,
  safety gate, `predict off|50`, analyzer call and final `estop stat`.
  The analyzer invocation was fixed to `--bin jbot-analyze`.
* `docs/runtime-lag-probe.md`: profiles, artifacts, hashes and results.

## 4. What was tested on the bench

Paired firmware: Pico `a0893f42…` (200 Hz, raw gyro) + F103 `lag-fix`
`9867b4aa…` (input filter off). Captured `zero` and `level` before each run;
preflight confirmed `zcapt=1`, `lcapt=1`, `estop=0`, `fb_ok=1`.

Spin check on the stand: `test 60 60` spun both wheels to about 395 rpm;
a separate step measurement reached 30 rpm in about 172 ms including the
600 units/s ramp and the dead zone.

Balance profiles, all recorded in `csv/jbot-disturbance-*.csv`:

| Profile | Armed | Peak pitch | Outcome |
|---|---|---|---|
| A | 0.49 s | -62.9 deg | Fall and guard latch, 0.49 s oscillation period |
| B | 0.45 s | 43.2 deg | Fall, period shortened to 0.23 s; predictor over-compensates |
| C | 0.81 s | -32.8 deg | Longest survival; minimal law alone still insufficient |
| E | 0.51 s | -64.3 deg | Fall; 502 rpm peak, 1.2 % torque saturation |

Profiles D and F were not run: prediction already made B worse, so repeating
it on the minimal or full-gain law is not informative.

Key finding from E: the fitted pitch law gave `kp = 19.6 Nm/rad` with
`R^2 = 0.95`, matching the compiled model value of 21.9 Nm/rad. The sign is
correct and the torque was not significantly saturated. The law is therefore
not the blocker. The measured loaded breakaway was 90–118 units
(0.45–0.59 Nm) and the total torque budget is about 5 Nm.

Acceptance criterion (10 s, pitch below 2 deg, travel below 0.1 m, no guard,
no saturation) was not met by any profile.

## 5. Difference from mjbots hoverbot

| Property | mjbots hoverbot | This robot |
|---|---|---|
| Control rate | 400 Hz (`period_s = 0.0025`, a configurable default) | 200 Hz primary, 400 Hz build prepared |
| IMU | pi3hat at 400 Hz, attitude and motor responses in one cycle | BNO055 fused Euler at 100 Hz, gyro propagated to 200 Hz |
| Derivative input | raw `rate_dps.y()` | raw mapped gyro after removing the 12 Hz filter |
| Motor interface | moteus torque mode, `kp_scale = kd_scale = 0`, no input filter | EFeru `VARIANT_USART` over 115200 baud, torque mode, 5 ms main loop, 16 kHz FOC |
| Feedback | CAN responses in the same 2.5 ms cycle | 18-byte UART frame at 100 Hz |
| Drivetrain | direct-drive hub with clean torque, no notable dead zone | direct-drive hoverboard hub, no gearbox, but loaded stiction/cogging and 0.45–0.59 Nm breakaway |
| Determinism | dedicated CPU affinity for control and pi3hat | shared Embassy executor on one RP2350 core |

The 400 Hz value in mjbots is a chosen design point, not the hardware ceiling;
the configuration field can be changed. For this robot the control bandwidth
is about 1–2 Hz, so 200 Hz is already far above the loop bandwidth. Raising
the Pico rate alone cannot help while the F103 applies setpoints every 5 ms
and the drive has a large loaded dead zone. That is why the 400 Hz image was
kept as an experiment rather than adopted as the fix.

## 6. Tilt-recovery tuner

`crates/jbot_console/src/bin/tilt_recover_tune.rs` narrows the goal from a
10 s balance to a precise short hold. For each runtime-only profile the
operator places the body at the requested tilt (default 10 deg) with the rope
attached, confirms with Enter, and releases after arming. The driver checks:

* body motion from the gyro and axle speed from the mainboard feedback
  (`L`/`R` rpm); the gyro alone cannot prove that the wheels moved;
* peak pitch error and gyro rate, sign changes of the pitch error (limit cycle
  detection), and wheel-active samples (drive authority);
* a continuous hold window (default 2 s) inside the angle tolerance with the
  wheels turning, no guard, no e-stop, no disarm and bounded travel.

Each trial ends with `home 0` and `estop`. The grid contains six compact
profiles: full-gain minimal, host-selected minimal, both with dead-zone
compensation, a full-cascade variant and a dither variant. The grid stops at
the first profile that holds. CSV and JSON are written under `csv/`.

Usage:

```sh
cargo run -q -p jbot_console --release --bin tilt_recover_tune -- \
  --run --port /dev/cu.usbmodemJBOT_PICO_011 --tilt 10 --hold-ms 2000
```

### Bench run, 2026-11-27

`level` was re-captured with the body upright and `stat` reported
`zcapt=1 lcapt=1 estop=0 fb_ok=1` before the run. Target was 10 deg with a
2 s hold in a 0.5 m zone. Recording:
`csv/jbot-tilt-tune-1795802340837.csv` and `.json`.

| Profile | Verdict | Start error | Peak gyro | Wheel samples | Sign changes |
|---|---|---|---|---|---|
| `full-min` | bad start | 2.7 deg | 0 dps | 0 | 0 |
| `host-min` | guard | 15.7 deg | 251 dps | 7 | 1 |
| `full-cal` | bad start | 18.1 deg | 0 dps | 0 | 0 |
| `host-cal` | guard | 16.6 deg | 260 dps | 25 | 6 |
| `full-ff` | bad start | 11.9 deg | 0 dps | 0 | 0 |
| `full-dither` | arm refused | 15.6 deg | 0 dps | 0 | 0 |

No profile held the tilt. Three runs were rejected at the start because the
body was not actually at 10 deg (2.7, 18.1 and 11.9 deg — the gyro was 0 dps,
so the body was held still). The two runs that started near the target hit
the rate guard at 251 and 260 dps; wheel RPM was present in 7 and 25 samples,
which confirms drive authority, but the recovery was an abrupt jerk rather
than a controlled lift. `full-dither` was refused at arm time.

The run was stopped by operator request: the motion was reported as far too
abrupt. The e-stop latched at the end of the grid and the body fell; the robot
must be reset upright before any further trial.

### Testing paused — resumption protocol

Hardware testing is paused. Do not arm any profile without explicit, per-run
approval. Do not send Enter automatically. When testing resumes, use only the
gentle protocol, one profile at a time, each confirmed by the operator:

* start tilt 3–5 deg, not 10 deg;
* `pscale 0.4–0.6`, `slew 300–1000`, no dither;
* watchdog the gyro: e-stop immediately at the first rate excursion;
* one profile per run, reviewed before the next.

## 7. Conclusion and next direction

The control law is correct at full gains. The remaining blockers are physical:
loaded dead zone and stiction/cogging of the direct-drive hubs, a roughly 5 Nm
torque budget for a body with a high centre of mass, and 100 Hz attitude
estimation. Further gain tuning or predictor work is unlikely to help. The
short tilt-recovery goal fared no better: the two runs that started near 10 deg
converted into a gyro-rate guard at 251–260 dps rather than a controlled lift.

Hardware trials are paused pending operator approval (see the resumption
protocol in section 6). The next technical direction, in order: reduce
stiction/cogging and check hub/bearing play and wheel seating; improve angle
estimation (better encoder or IMU); lower the centre of mass; increase torque
authority only after thermal and supply-sag measurements justify it. Do not
weaken the rope, hands-above, e-stop or 0.5 m zone gates.
