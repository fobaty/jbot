# Runtime lag probe (2026-11-26)

## Purpose

Requirement IDs: `M3-18`, `M3-26`, `SYS-02`, `SAF-*`. No firmware change is
part of this probe: every knob below already exists in the flashed image.

The open question is why the body develops a 0.19 s limit cycle at
`pscale 1.00 / slew 1500` even though torque and wheel speed are present. The
leading hypothesis is phase loss from the drive path: a ~14 ms input filter on
the F103 (upstream default ~47 ms), a 5 ms main loop, a 100 Hz feedback frame and a 12 Hz filter on the
pitch-rate derivative. A host sweep with a 50 ms measurement delay selects
`pscale 0.75 / dscale 0.60` as the only settling point
(`docs/hold-id-audit-2026-11-26.md:114-121`).

## Profiles

| Profile | Knobs | Question |
|---|---|---|
| A | `pscale 0.75`, `dscale 0.60`, `slew 1500`, `db 0.20`, `cal 35 45`, `ff on`, `dither off`, `predict off` | Does the host-selected gain pair hold on hardware? |
| B | Same, plus `predict 50` | Does Smith compensation remove the residual lag? |
| C | `--minimal`: `ff off`, `db off`, `cal off`, `dither off`, `iscale 0`, outer loops off, `predict off` | Does a mjbots-like minimal law hold without compensation terms? |
| D | Same as C, plus `predict 50` | Does prediction help the minimal law? |
| E | `--minimal`, `pscale 1.00`, `dscale 1.00`, `slew 10000`, `predict off` | Maximum-accuracy candidate: full model gains and no slew limit |
| F | Same as E, plus `predict 50` | Does prediction help at full gains? |

Profile A reproduces the documented candidate exactly. Run A before B: the
gain pair already assumes 50 ms of delay, so adding prediction may
over-compensate. Profiles C and D test the control-law structure separately
from the compensation knobs. Never combine mechanisms without a recorded A/B
comparison.

## Preconditions

- Pico image `4c3263de...` and F103 image `267322b5...` are the paired build
  (`docs/pico-firmware-known-good-2026-11-25.md:90-96`).
- USB console survives open/stream/close; cable is strain-relieved.
- `stat` reports `fb_ok=1`, `zcapt=1`, `lcapt=1`, `estop=0`, `loop~200`,
  `imu_fresh=1`. A sleeping mainboard or a rebooted Pico invalidates the run.
- Rope attached, hands above the body, e-stop reachable, zone at most 0.5 m.
- No manual push. The excitation is `disturbance_probe`'s command-driven
  `home 3` step from an already-armed, quiet pose.

## Commands

```bash
./bench-temp/runtime-lag-probe.sh                 # read-only preflight
PROFILE=A ./bench-temp/runtime-lag-probe.sh run
PROFILE=B ./bench-temp/runtime-lag-probe.sh run
PROFILE=C ./bench-temp/runtime-lag-probe.sh run
PROFILE=D ./bench-temp/runtime-lag-probe.sh run
PROFILE=E ./bench-temp/runtime-lag-probe.sh run
PROFILE=F ./bench-temp/runtime-lag-probe.sh run
```

Recommended order: A, C, E, then B, D, F. The prepared firmware removes most
of the drive lag, so a 50 ms predictor may over-compensate; test the
no-predictor profiles first. Run one at a time and review each CSV before the
next. Stop at the first profile that passes.
The script sends `predict off|50` once, then delegates knob setup, arming, the
`home` step, the 8 s recording, `home 0` and `estop` to `disturbance_probe`,
and finally runs `jbot-analyze` on the newest `csv/jbot-disturbance-*.csv`.

## Acceptance and decision

A profile passes only if the body stays upright for 10 s with `|pitch| < 2
deg`, `|x| < 0.1 m`, no guard latch and no torque saturation. A shorter window
is evidence only, not acceptance.

- A passes: promote `pscale/dscale/slew/db/cal` from runtime into
  `config/robot.toml` under a separate approval, then verify push recovery.
- A fails, B passes: the predictor is the fix. Keep it runtime until the
  F103 filter is removed, then re-derive the gains against the shorter delay.
- Both fail with a short-period limit cycle: physical lag is confirmed.
  Proceed to the paired firmware change: `FILTER=65535`, `SERIAL_TIMEOUT=20`,
  gyro propagation between 100 Hz IMU samples, and disabling the pitch-rate
  low-pass so the D term sees raw mapped gyro. `I_MOT_MAX` stays at 10 A.
- C or D passes while A/B fail: the compensation terms, not the core law, are
  the problem. Keep the minimal profile and add terms back one at a time.
- A/B and C/D all fail after the paired firmware flash: increase the Pico
  balance rate to 400 Hz with the prepared `rate400` build, then, if still
  unstable, change the actuator path (F103 command application in the FOC ISR
  or `hoverboard-rs`) before considering any mechanical change.

## Do not

- Do not raise `I_MOT_MAX`; the controller used under 5 % of full scale.
- Do not weaken tilt, rate, range or fall guards to make a run pass.
- Do not repeat the `pscale 0.25 / slew 100` profiles already recorded in
  `docs/bench-2026-11-26.md`.
- Do not flash one board alone; the 18-byte feedback contract is paired.

## Firmware preparation (2026-11-26)

The runtime probe is deliberately first because no firmware change has been
validated on hardware yet. In parallel, two source-side changes are prepared
so the physical-lag hypothesis can be tested immediately if both runtime
profiles fail:

- Pico 2 W (`firmware/pico2w`): propagate the fused 100 Hz Euler pitch to the
  200 Hz control tick with the mapped gyro rate, reduce the derivative
  low-pass entirely (raw mapped gyro in the D term), and gate the hardware watchdog feed on
  completed control steps with usable sensors.
- STM32F103 (`firmware/stm32f103/eferu-foc/lag-fix.patch`): disable the
  ~14 ms input low-pass (`FILTER=65535`) and shorten the serial timeout from
  160 to 20 ticks (100 ms). `I_MOT_MAX`, `TRQ_MODE`, the mixer coefficients,
  the 16 kHz PWM and the 18-byte feedback layout are unchanged.

The prepared artifacts are:

| Board | Artifact | SHA-256 | Size |
|---|---|---|---|
| Pico 2 W, 200 Hz | `firmware/pico2w/fw/pico2w-lag-fix.uf2` | `460bfd763688631e3aec94a87a9fd38043a3207aa30345d9e0c42d2ba49aded1` | 291,328 B |
| Pico 2 W, 400 Hz | `firmware/pico2w/fw/pico2w-lag-fix-400hz.uf2` | `ddfdb668578456338a6a35e3ad5ec4ea9b8769ff18dfb3a0458adf62542fd8a6` | 291,328 B |
| STM32F103, baseline | `firmware/stm32f103/eferu-foc/eferu/build/lag-fix/hover.bin` | `9867b4aa8965dd9cd300d5c35be08833d46f03e494b61d1019bbb262f5a205db` | 40,328 B |
| STM32F103, 1 kHz command | `firmware/stm32f103/eferu-foc/eferu/build/fast-command/hover.bin` | `95630d10a74b54ee13a590909d6cbf2e86e7dcc37d2242741ac2753333cd7b92` | 40,476 B |

The F103 source of truth is `firmware/stm32f103/eferu-foc/lag-fix.patch`,
which applies cleanly to upstream HEAD and preserves TRQ_MODE, `I_MOT_MAX=10`,
the mixer coefficients, 16 kHz PWM and the 18-byte feedback layout. The active
`VARIANT_USART` build uses `FILTER=19661` (alpha 0.3 per 5 ms tick, tau about
14 ms), not the upstream default `6553` (alpha 0.1, tau about 47 ms). Both are
replaced by `65535`, which the fixed-point implementation in `Src/util.c`
makes a one-tick pass-through. The ~104 ms figure in
`docs/precision-control-2026-11-25.md:219` is a measured command-reporting
rise that also includes the 5 ms main-loop quantization and the 100 Hz
feedback interval, not the filter time constant.
`INACTIVITY_TIMEOUT=0` is compile-time guarded in `Src/main.c` because setting
it to zero alone would power off immediately.

The 400 Hz build is an alternative, not a replacement: the F103 still applies
torque in its 5 ms main loop, so the higher rate reduces sensing and control
quantization on the Pico side only. At 400 Hz the predictor ring
(`MAX_DELAY_TICKS = 40`) caps the horizon at 97.5 ms even though the console
accepts 150 ms; the prepared profiles use 50 ms, which is unaffected.

The 1 kHz F103 variant is built on top of the lag-fix baseline via
`firmware/stm32f103/eferu-foc/fast-command.patch`. USART2 parsing was already
asynchronous in the IRQ; only the setpoint conditioning ran in the 5 ms main
loop. The patch moves that conditioning to a 1 kHz tick while keeping timeout
qualification, the rate limiter, the filter, the mixer, the 100 Hz feedback
frame and the 16 kHz FOC handoff intact. Command-to-setpoint refresh improves
from at most 5 ms (mean 2.5 ms) to at most 1 ms (mean 0.5 ms). Use either
`lag-fix` or `fast-command` on the F103, never both; the latter is the
maximum variant, the former the conservative fallback. No current, voltage,
torque or feedback-rate limit was raised.

Neither artifact is flashed until a bench session is available. Use the
flashing procedures recorded in `docs/pico-firmware-known-good-2026-11-25.md`
and always flash Pico and F103 as a pair.

Gain defaults are intentionally left unchanged. `pscale`, `dscale`, `slew`,
`db`, `cal`, `predict` and `tmax` remain runtime-only until a profile passes
the acceptance criteria above.

Simplified hoverbot-style control (pitch PD on raw gyro plus speed-to-pitch
only) is the documented fallback, not part of this change set. It requires its
own approval and validation.
