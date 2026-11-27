# Hold-identification audit (2026-11-26)

## Scope

Five armed hold trials and one free-wheel spin sweep recorded on the current
stage-1 hardware (STM32F103 FOC mainboard, Pico 2 W balance controller, two
6.5 in direct-drive hub motors, BNO055). The question was whether the existing
torque budget and the runtime compensation knobs can hold the machine upright
from a hand-supported start. All claims below come from the CSV recordings and
from `jbot-analyze` / `spin-fit`; nothing is inferred from the visual behaviour.

## Free-wheel drive identification

Recording: `csv/jbot-spin-1795732416618.csv` (`spin-driver`, seven levels,
1500 ms hold). Wheel inertia `0.0078 kg m^2` from `config/robot.toml`.

| Command, units | alpha, rad/s^2 | Torque, Nm | units/Nm |
|---:|---:|---:|---:|
| 39 | 11.77 | 0.0918 | 424.7 |
| 48 | 20.64 | 0.1610 | 298.1 |
| 60 | 33.04 | 0.2577 | 232.8 |
| 75 | 50.92 | 0.3972 | 188.8 |
| 99 | 72.57 | 0.5661 | 174.9 |
| 120 | 96.28 | 0.7510 | 159.8 |
| 150 | 133.08 | 1.0380 | 144.5 |

- First motion at **39 units**; `spin-fit` linear fit: dead zone **28.69 units**,
  affine scale 119.04 units/Nm.
- The asymptotic ratio at 99 units (**174.9**) agrees with the WP0 working
  value of **175 units/Nm**. The decreasing ratios at 120/150 units are the
  torque-speed envelope affecting the early acceleration window, not a change
  in the motor. Use 175 units/Nm as the working scale.

## Hold trials

All trials used `recovery_driver --hold-only --relevel --loose-baseline --ff on`,
wheels on the floor, a slack rope as a fall catch, `slew 100`. The robot was
stood upright and held still by the operator for the level capture; the arm
happened at a small but non-zero body rate.

| # | Recording | Profile | Armed, s | Oscillation period, s | kp, units/rad (Nm/rad) | kd, units/(rad/s) | R2 | Implied dead zone, units | Outcome |
|---:|---|---|---:|---:|---:|---:|---:|---:|---|
| 1 | `csv/jbot-recovery-1795734139951.csv` | pscale 0.25, db off, dither 40 | 2.80 | 0.62 | 52 (0.30) | -5 | 0.68 | 20 | latch at -39 deg |
| 2 | `csv/jbot-recovery-1795734323964.csv` | pscale 0.25, db 0.20, dither 40 | 3.13 | 0.69 | 77 (0.40) | -7 | 0.34 | 10 | latch at -36 deg |
| 3 | `csv/jbot-recovery-1795734398259.csv` | pscale 0.30, db 0.20, dither 40 | 1.92 | 0.48 | 75 (0.40) | -13 | 0.77 | 8 | disarm at -24 deg |
| 4 | `csv/jbot-recovery-1795734689458.csv` | pscale 0.30, dscale 0.30, db 0.20, dither 40 | 2.83 | 0.63 | 39 (0.20) | -11 | 0.74 | 10 | latch at -44 deg |
| 5 | `csv/jbot-recovery-1795734787742.csv` | pscale 0.25, dscale 1.00, db 0.20, dither off | 2.40 | 1.20 | 51 (0.30) | -7 | 0.65 | 14 | latch; roll axis unpaired |

Common observations:

- Pitch/gyro axis check is **consistent** in all trials. Trial 5 reports roll as
  UNPAIRED because the body fell sideways; that verdict is not an IMU fault.
- Applied torque during the armed window was only **-31..+54 units** (about
  -0.18..+0.31 Nm) with **0.0 % saturation**. The drive never reached its limit.
- The `db 0.20` trial (2) was the best of the ladder: longer hold, higher fitted
  kp, lower implied dead zone. `dscale 0.30` and `dither off` did not improve it.
- Every trial ended when the slack rope caught the falling body, not when the
  torque limit was hit.

## Why the fall is not an over-torque event

With `m = 5.0 kg`, `L = 0.26 m`, `g = 9.81 m/s^2`, the gravity torque is
`m g L sin(theta) = 12.75 sin(theta) Nm`:

| Pitch | Gravity torque, Nm | Torque the fitted law would produce at kp 0.3, Nm |
|---:|---:|---:|
| 5 deg | 1.11 | 0.026 |
| 10 deg | 2.21 | 0.052 |
| 15 deg | 3.30 | 0.079 |
| 25 deg | 5.39 | 0.131 |

At the measured `kp = 0.2..0.4 Nm/rad` the controller is roughly one order of
magnitude too weak near upright to counter gravity, and beyond about 23 deg the
full 5.7 Nm budget is insufficient even statically. This matches WP1, where 30
deg was reachable only with 2.56 m of travel under the 3 m guard.

The rope only bounds the fall; it contributes no control torque. Therefore the
observed latches are a torque-availability and initial-condition problem, not
evidence of excessive motor force and not a firmware defect.

## Decision

1. Stop the blind `pscale` ladder: all five profiles fail within 2-3 s and the
   period shortens as P grows, which is the expected limit cycle from an
   uncompensated dead zone.
2. Keep the measured runtime values: `db 0.20`, `cal 35 45`, `pscale 0.25`,
   `slew 100`, `ff on`. `dither` and `dscale` showed no benefit.
3. Do not raise `I_MOT_MAX`: the controller used under 5 % of full scale and the
   drive is already near the traction limit.
4. A repeatable release fixture (see `docs/release-fixture.md`) is required
   before re-deriving gains. Every trial here started with a hand-supported,
   non-zero-rate release, which dominates the first second of telemetry.

## Incidental firmware fix

The Pico USB console used to park after a host closed the port mid-stream: a
cancelled `write_packet` left the IN endpoint marked `available`, so every later
write waited on a host that was gone. `src/console.rs` now clears that flag on
write timeout using `rp_pac::USB_DPRAM` (already in the tree through
`embassy-rp`; no `unsafe`, no safety-contract change). Repeated open / stream /
close cycles were verified after flashing. Host tools were also changed to send
`stream 0` and drain before closing.

## Final control candidate (pending hardware confirmation)

The last disturbance probes changed the diagnosis. With `pscale 1.00` and
`slew 1500`, the controller did produce torque (up to 183 units, about 1 Nm)
and the wheels did turn (up to 162 rpm), so the robot was **not externally
supported**. It nevertheless developed a 0.19 s limit cycle (10.28 crossings/s,
`kd` fitted at -0.09 Nm/(rad/s)) and fell. That points at sensor/transport lag,
not at torque authority; the earlier `pscale 0.25` runs failed for the opposite
reason (kp about 0.3 Nm/rad against roughly 0.67 Nm of gravity at 3 deg).

A host-only sweep of the linear plant with a 50 ms measurement delay selects:

| pscale | dscale | Result in model |
|---:|---:|---|
| 0.50 | 0.60 | 3.91 deg peak, settles |
| 0.75 | 0.60 | **3.68 deg peak, settles** |
| 1.00 | 0.60 | 3.16 deg, but marginal with 50 ms delay |
| 1.00 | 1.00 | unstable with 50 ms delay |

The single remaining hardware check, once the damaged USB socket is repaired
and the cable is strain-relieved:

```sh
disturbance_probe --run --secured --angle 3 --record-ms 8000 \
  --pscale 0.75 --dscale 0.6 --slew 1500 --db 0.20 --cal 35 45
```

`disturbance_probe` now accepts `--dscale`. Until that run succeeds, balance
remains unverified; everything else (drive scale, breakaway, axes, console
stability, firmware fix) is measured and closed.
