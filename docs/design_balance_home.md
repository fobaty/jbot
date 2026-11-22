# Design Record: Smooth Home-Pose Balance (pitch hold with soft transitions)

**Status:** v2 implemented, simulated, **not bench-validated**. Any change to the
200 Hz balance loop, the arm/disarm path, the fail-safe state machine, or the
actuator limits is a stop-and-escalate change under
`docs/skills/rust_robotics_guidelines.md` §0.2 and requires the balance HIL bench
test and review before merge.

**Requirement anchors:** `SYS-02` (balance loop), `SYS-07` (end-to-end latency),
`M3-13` (torque limits, balance priority), `M3-17` (200 Hz task), `M3-18` (balance
controller), `M3-19` (state machine), `M3-20` (tilt/fall detection), `M3-22` (command
processing, acceleration limiting), `SAF-01…SAF-13`, `M2-14` (arming). Tilt limit
source: `config/robot.toml` `[control] tilt_limit_rad` (30 degrees).

**Author context:** stage 1 donor hardware (two hub wheels, pitch-only control, no hip
actuators). "Home" for this stage is the captured upright equilibrium (`zero` +
`level`), with an operator bias of up to ±5 degrees.

---

## 1. Problem statement

The robot must hold a commanded pitch pose ("home") and transition to it smoothly,
from any attitude where recovery is physically possible, on uneven or non-horizontal
surfaces, without torque jerks and without runaway oscillation.

The v1 design (ramped reference over the old `Gains` pitch PD plus a `FALL` latch)
was implemented, flashed and failed on the bench: it could not lift the body from a
lean, drifted on a slope, and produced a large first correction. v2 replaces the
gain table and the inner law; this document is the record of what v2 is and why.

## 2. Physical limits (honest bounds)

* Stage 1 is an inverted pendulum. No attitude is a stable equilibrium; the controller
  stabilises one unstable operating point.
* The static lift limit follows from the identified plant: a full-torque command
  cancels gravity at `pitch = tau_max / (A1 / A2)`. With the nominal 5 Nm common-mode
  budget and `A1 / A2 = 5.39 Nm/rad` (bench-weighed plant, §10), that is **53 degrees**
  at zero pitch rate. Recovering from rest needs less, because the body must first be
  accelerated up to the neutral point; simulation recovers from about 0.75 of the limit
  at 3 Nm and about 0.63 at 8–12 Nm. The fall guard caps the accepted pose error at
  30 degrees regardless.
* **Lying flat is beyond the currently assumed torque budget.** Lifting a body whose
  centre of mass is `L = 0.25 m` from the wheel axle (`com_height_m` in
  `config/robot.toml`, the same value used in §10) needs roughly `m g L ≈ 24–25 Nm`,
  while `PlantParams::NOMINAL` assumes 5 Nm of common-mode torque. That number is
  **an assumption, not a measurement**: the donor drive is configured with a 10 A per
  motor phase current limit (`firmware/stm32f103/eferu` `I_MOT_MAX`), and the torque
  per amp of these hub motors has not been measured. If the real budget is
  substantially higher, the lift limit grows and flat recovery may become reachable,
  because the whole law scales with `max_torque_nm`. Until then the law refuses to
  arm beyond the assumed lift budget and latches a safe state instead of attempting
  a violent raise.
* Measured on the bench (2026-11-17, session `csv/jbot-telemetry-1794961022.csv`, robot
  disarmed): face-down rest at pitch ≈ **+87.5°**, on-back rest at ≈ **−98°**.
* IMU behaviour past the stop: pitch is derived from the gravity vector
  (`atan2(x, -z)` in `pico_core::mount`) and stays valid to ±180°. Raw BNO055 Euler
  heading/roll degenerate near ±90° of pitch. The balance law and every pitch-based
  guard use the gravity-derived pitch only.
* **On a slope, a pitch-only loop has no static equilibrium.** Pitch balance wants
  `tau = floor · theta`; holding position on an incline wants `tau = B2 · theta / B1`.
  The two conditions meet only at `theta = 0`, which is not an equilibrium on a slope.
  A position integrator is therefore mandatory, not optional: it learns the slope
  offset so the machine leans about 1.6 times the slope angle and stays put. This was
  confirmed in simulation — with `trim_gain = 0` the machine slides down every slope,
  including 4 degrees.

## 3. Architecture (v2)

Three layers, each with fixed per-tick work, no allocation, no I/O in the hot path.
Gains are derived once per boot from `PlantParams::NOMINAL` through
`Tuning::from_plant`; there is no gain table in the firmware.

### 3.1 Reference layer (`jbot_pico_core::reference`)

A trapezoidal trajectory generator owning the pitch reference.

* State: `ref_rad`, internal velocity; acceleration-limited with `a_max` (20 deg/s^2)
  and rate-limited with `v_max` (default 10 deg/s, console-tunable, cap 60 deg/s in `reference::V_MAX_CAP_RAD_S`).
* `reset(at_rad)` anchors the reference to a measured pose and drops velocity, but
  **does not clear the target**: the ramp then walks to the target.
* Target clamped to ±30 degrees; non-finite values reset to zero.

### 3.2 Cascade (`jbot_pico_core::control`)

```text
x ─► position loop ─► v_ref ─► speed loop ─► a_ref ─► a_ref / accel_per_rad
                                                            │
                                            dynamic pitch demand
                                                            │
    pitch_ref + demand + trim ─► pitch PD ─► common-mode torque
```

* **Position loop** (`kpos = 0.4 1/s`): base offset since arming becomes a speed
  reference, capped at `v_lim = 1.0 m/s`.
* **Speed loop** (`kv = 1.8 1/s`): speed error becomes an acceleration reference,
  capped at `a_lim = 1.0 m/s²`.
* **Demand stage**: `a_ref / accel_per_rad`, capped at ±8 degrees. `accel_per_rad`
  is the closed-loop base acceleration per radian of pitch demand,
  `(b1·floor − b2)·margin/(margin−1) ≈ 7.33 m/s²/rad` at the design point — not the
  open-loop sensitivity.
* **Trim integrator** (`ki = 0.01 rad/(m·s)`): learns the slope offset from the
  position error, saturating with anti-windup at ±20 degrees. This is the
  auto-correction the requirement asks for, and it is what makes a slope holdable.
* **Inner pitch PD**: `kp = margin · floor = 21.6 Nm/rad` (margin 4), `kd = 5.9`
  (damping ratio 1.6), both derived from the plant, not hand-set. A slow pitch-error
  integrator (`kp/20`, 1 Nm clamp, 3 degree window) removes the lean the drive dead
  zone would otherwise leave behind. The derivative term runs on the measured rate
  after a 12 Hz low-pass — no derivative kick.
* **Operator speed** is a step setpoint, turned into an acceleration-limited
  reference (`0.5 m/s²`); the position origin advances with it so the position loop
  measures tracking error while driving and holds a fixed point when the command is
  zero.
* **Yaw** uses only the headroom left after the common-mode torque is reserved
  (`M3-13`), so steering can never starve balance.
* Common-mode torque is clamped to the unit range and slew-limited before mixing.

### 3.3 Arming (`pico2w` control task)

Arming is refused — and the law stays latched off — when any of these holds:

* the mainboard feedback is stale (`FEEDBACK_MAX_AGE = 300 ms`); without wheel speed
  the outer loops cannot run and the position integrator would drift;
* the pose error to the target exceeds `fall_error_rad()` (0.70 of the lift limit,
  see below);
* the body is already on the ground (`|pitch| > LYING_RAD = 45°`).

Otherwise the reference is anchored to the measured pose only when the error is below
`ARM_ANCHOR_RAD = 5°`. A larger error starts at the target, so the law actually lifts
the body instead of slowly chasing it; the torque slew limiter is the soft start.

Arming on a slope with a **zero captured on flat ground** is possible only on the
shallow slopes that fit the 3 m range guard: the trim has to learn the offset while
the machine rolls back, and the required travel grows quickly with the angle. On
steeper terrain, capture `zero`/`level` while standing on the surface: the commanded
pose is then the local equilibrium and 10 and 12 degrees hold with under a metre of
travel. This is the documented workflow; an on-flat-ground zero at 10 degrees is
outside the envelope and is aborted by the range guard rather than chased.

### 3.4 Fall detection and guards

* `fall_error_rad = min(0.70 · lift_limit, 30°)`: with the corrected plant the
  uncapped value is ≈37°, so the 30° control-design cap binds. The threshold must
  stay below the static limit because lifting from rest needs torque to accelerate
  the body first.
* `LYING_RAD = 45°`: on the ground.
* `FALL_SPEED_MPS = 3.0`: a recovery from 30° peaks near 1.4 m/s, so this only
  catches a real runaway.
* `RANGE_LIMIT_M = 3.0 m` of travel since arming.
* Any guard zeroes the torque immediately (bypassing smoothing), clears the arm
  latch, increments `FALL_EVENTS`, and requires an explicit `arm` after the robot is
  placed back in a recoverable pose. **It never auto-rearms. E-stop remains absolute.**
* The arm and fall decisions are one pure function, `SafetyState::arm_refusal` and
  `SafetyState::fall_cause` in `jbot_pico_core::control`, returning a `StopReason`
  that names the violated limit. The control task only forwards torque; it cannot
  skip or reorder a check, and the thresholds are unit-tested without hardware.
  The reason is published over USB as `stop=<label>` and cleared by a new `arm`
  or by `clear`.
  The two entry points differ on purpose: arming compares the pose against the
  *target* and additionally requires fresh feedback, while a running attempt
  compares against the *applied* ramped reference, so a transition is never
  mistaken for a fall, and checks travel and speed.
* A stale IMU sample means zero torque but is not latched: the mainboard stops on its
  own serial timeout, and a single-sensor gap must not end the run.

## 4. Command and telemetry surface

* `home [deg]`: sets the home reference. With no argument it follows the captured
  `level`; an argument is clamped to **±5 degrees**, because a standing two-wheel
  machine cannot hold a larger commanded pitch while stationary — the inner loop has
  to trade angle against holding torque and the base walks.
* `vmax [deg/s]`: reference ramp rate; zero freezes the ramp in place.
* `arm` / `disarm`, `zero`, `level`, `estop`, `clear` behave as before. Any change of
  mount or calibration offsets clears the arm latch.
* Telemetry additions: `ref` (applied reference), `dem` (total outer-loop pitch
  demand), `trm` (learned slope trim), `x` (travel since arming), `mode`
  (`0 disarmed, 1 transit, 2 hold, 3 recover, 4 fall`).
* `stat` reports the SI gains (`kpnm`, `kdnm`, `kpos`, `kv`, `ki`) and the demand and
  trim limits (`pdem`, `ptrm`) in degrees.

## 5. Parameters (nominal donor plant)

| Name | Value | Source |
|---|---|---|
| `pitch_margin` | 4.0 | 4× the 5.39 Nm/rad stability floor |
| `pitch_kp` | 21.6 Nm/rad | derived |
| `pitch_kd` | 5.9 Nm/(rad/s) | derived, damping ratio 1.6 |
| `pitch_ki` | 1.08 Nm/(rad·s) | `kp/20`, removes the dead-zone lean |
| `pitch integral clamp` | 1.0 Nm | above the estimated 0.5 Nm dead zone |
| `kpos` | 0.4 1/s | decade below the pitch loop |
| `kv` | 1.8 1/s | decade below the pitch loop |
| `ki` (trim) | 0.01 rad/(m·s) | slow enough not to pump the stance |
| `v_lim` / `a_lim` | 1.0 m/s / 1.0 m/s² | operator-smooth |
| speed slew | 0.5 m/s² | turns the operator step into a lean |
| pitch demand limit | 8° | outer-loop authority |
| pitch trim limit | 20° | slope offset budget |
| `FALL_FRACTION` | 0.70 | sim: 0.75 works at 3 Nm, 0.63 at 8–12 Nm |
| `FALL_ERROR_CAP` | 30° | `tilt_limit_rad` |
| `RANGE_LIMIT_M` | 3.0 m | runaway guard |
| `FALL_SPEED_MPS` | 3.0 m/s | runaway guard |
| `ARM_ANCHOR_RAD` | 5° | anchor only small pose errors |
| `FEEDBACK_MAX_AGE` | 300 ms | three mainboard frames |
| `ref_v_max` / `ref_a_max` | 10 deg/s / 20 deg/s² | start slow, raise later |
| `pscale` | 1.0 (compiled law) at boot | bench-only runtime force scale on `pitch_kp`/`pitch_kd` (and yaw); fall guards unchanged |
| `TORQUE_SLEW_PER_S` | 3000 units/s | unchanged |
| `D_FILTER_CUTOFF_HZ` | 12 Hz | unchanged |
| deadband compensation | 0 Nm (off) | exact inverse behaves like a relay |

## 6. Simulation evidence

All numbers below come from the clean planar model
(`firmware/pico_core/src/plant.rs`, mirrored in the scratch harness
`jbot_sim/plant13.py` with keyword-only arguments). The Rust suite reproduces the
headline cases.

| Scenario | Result |
|---|---|
| Hold level from 1° | settles, peak pitch 2.4°, travel 15 cm |
| Recover from 20° | settles, peak travel 0.42 m, peak speed 0.62 m/s |
| Recover from 30° | settles, peak travel 1.73 m, peak speed 1.39 m/s |
| Drive at ±0.3–0.4 m/s | tracks the command, stays upright, settles |
| Hold 4°/6° slope (armed at equilibrium, flat-ground zero) | settles at ≈1.6α, speed → 0, travel < 2 m |
| Hold 10°/12° slope (zero/level captured on the slope) | settles at ≈1.6α, speed → 0, travel < 1 m |
| Hold 10°/12° slope (zero captured on flat ground) | outside the envelope; the 3 m range guard aborts |
| Drive onto a 4°/6° ramp while balancing | settles, travel 0.45/0.89 m |
| Drive onto a 10° ramp while balancing | runaway — outside the envelope |
| Slope 8° | **fails**: holding torque ≈ 0.6 Nm sits on the edge of the 0.5 Nm drive dead zone |
| ±30 % mass, `L ±20 %`, `Ib +50 %`, delay 0/3 ticks, 0.1° noise, 4 Nm push | all survive |
| Torque budget 3 Nm | recovers from 18°; 8 Nm from 40°; 12 Nm from 60° |

### 6.1 What a lying-down recovery would take

The same cascade, run without the fall guard, was swept from extreme starting pitches
to find the torque a raise would need (dead zone 0.5 Nm, 45 s):

| Common-mode budget | Highest start pitch that settles |
|---|---|
| 12 Nm | 60° |
| 20 Nm | 75° |
| 30 Nm | 75° |
| 40 Nm | 85° (a lying pose on this chassis) |

So the mechanism can raise the machine from a lying pose in simulation, but only with
roughly 25–40 Nm of common-mode torque — four to eight times the assumed budget — and
the `0.70` fall fraction is only calibrated up to 30°. Allowing arming from a lying
pose needs both a measured torque budget and a dedicated raise manoeuvre validated in
simulation; it is not a threshold change. Until then the guard refuses instead of
trying, which is the safe failure mode.

The 8° case is deliberately excluded from the slope test with a comment; every other
listed slope holds. Below the dead-zone threshold the exact deadband feed-forward
was tried and rejected: it behaves like a relay and pumps the machine exactly where
the holding torque equals the dead zone. It stays off until a measured dead zone and
a bench check exist.

## 7. What this design does NOT do

* No recovery from lying flat at the assumed 5 Nm budget; refused and latched
  instead. Whether the real drive has enough torque is the first bench measurement
  (§8 step 1).
* No online auto-tuning of the gains; the plant parameters are constants until a
  bench identification replaces `PlantParams::NOMINAL`. The `pscale` console
  command is a manual bench multiplier on the angular-output gains, set by the
  operator by hand; it is not an automatic gain search, and it never weakens
  the fall guards.
* No slope feedforward estimator; the trim integrator is the mechanism.
* No change to the watchdog, e-stop, command-age, or mainboard timeout handling.
* No change to the yaw mixer beyond the headroom rule.

## 8. Bench plan (before the result is trusted)

1. **Confirm the torque scale.** With the robot on blocks, `test` a known command and
   measure the wheel torque; fill in `units_per_nm`/`max_torque_nm` for the real drive.
2. **Confirm the wheel-speed sign.** Bench-verified (2026-11-19): a straight
   forward roll reads opposite-sign hall speeds (`speedL ≈ -speedR`, e.g.
   `+7`/`-10`), because the hub motors are mounted mirror-image, so the
   firmware measures forward speed as `0.5·(left - right)`. A counter-rolling
   pair (pivot) reads the same sign on both wheels and zero linear speed.
3. **Dead zone and stiction.** Slow torque ramp on blocks; note the first value at
   which the wheel moves. This decides whether deadband compensation is safe.
4. **Arm from ~5°** and watch `ref`, `dem`, `trm`, `x`. Expect a torque ramp, no step,
   and `dem` held well inside 8°.
5. **Hold on a slope.** Start at 4°, then 6°. Expect a steady lean of about 1.6α,
   `trm` growing slowly, `x` returning to zero.
6. **Push test** at equilibrium; confirm recovery without oscillation.
7. **Fall test.** Lean past `theta_fail`; expect immediate zero torque and latched
   `FALL`, then explicit `arm` only after placement.
8. **Regression.** E-stop from any mode drops the command within one tick;
   `cargo fmt`, `check`, `test`, `clippy -D warnings` stay green.

### 8.1 Link diagnosis (`fb=0`)

The link is proven only when `stat` shows `fb` growing. `rx` separates the two
failure classes: `rx=0` means no bytes arrive at all (wrong mainboard variant,
USART2 not enabled, crossed or missing ground, wrong baud), while `rx` growing
with `fb=0` means the bytes arrive but do not form valid frames (baud mismatch,
wrong frame length, wrong start marker). `fb_ok` is the live freshness flag the
control loop uses, and `stop` carries the supervisor latch reason over USB, so
a refused `arm` is diagnosable without RTT: `stop=stale_feedback` is the link,
`stop=pose_error` or `stop=lying` is the placement. These counters are cleared
by `stats reset`. The Pico side is UART0 GP16 (TX) → PA3 and GP17 (RX) ← PA2 on
the left cable, 115200 8N1, common ground; the mainboard must be the
`VARIANT_USART` build, which enables `CONTROL_SERIAL_USART2` and
`FEEDBACK_SERIAL_USART2`.

Two bench checks isolate the direction of the failure:

1. **Pico self-test (no mainboard).** Disconnect the cable and jumper GP16 to
   GP17 on the Pico. The bridge still sends 8-byte command frames every 5 ms, so
   `rx` must grow by about 1600 bytes per second while `fb` stays 0 and
   `fb_drop` grows (8-byte commands cannot form the 18-byte feedback frame).
   If `rx` stays 0 here, the fault is on the Pico side or in the counter.
2. **Mainboard self-test (no Pico).** Connect a 3.3 V USB-UART adapter's RX to
   PA2 with a common ground, 115200. Valid 18-byte frames must appear every
   10 ms. If nothing arrives, the mainboard is not running the `VARIANT_USART`
   build, is not powered, or the cable is not the left one.

Only when both pass does the fault live in the crossover between them.

### 8.2 Motor wiring convention (bench-verified, corrected 2026-11-19)

The flashed `VARIANT_USART` mainboard runs the stock mixer with no
`INVERT_L_DIRECTION` / `INVERT_R_DIRECTION`: the wire `speed` channel drives
both wheels the same way and `steer` counter-rotates them. The console `test`
command does not write those channels directly; its arguments are *wheel-torque*
requests that pass through `to_command`, so the two conventions have to be kept
apart.

Measured on the floor with the balance law disarmed (session
`csv/jbot-telemetry-1795141040.csv`, 2026-11-19):

- `test 50 50` reaches the wire as `speed = 50, steer = 0`, rolls the machine
  forward, and the hall feedback reads `L = +8`, `R = -8` rpm.
- `test 100 -100` reaches the wire as `speed = 0, steer = 100`, pivots the
  machine in place, and the gyro reads `gz ≈ -30 dps`.

Firmware consequences:

- `to_command` uses the plain mixer inverse, `Command::from_wheel_torque(left,
  right)`: a same-sign physical `WheelTorque` (both wheels rolling the same
  way) becomes wire `speed`, and a counter-rolling pair becomes wire `steer`.
  Negating the right wheel turns the balance law's common-mode torque into a
  pivot command and makes the machine spin on the spot.
- `Feedback::forward_speed_mps` is the half-difference
  `0.5·(left - right)`, because the hub motors are mounted mirror-image and a
  forward roll reads opposite-sign hall speeds.

Historical note: the 2026-11-18 version of this section concluded the opposite
(negate the right wheel; average the hall speeds). That conclusion compared the
`test` arguments as if they were wire channels, and the same-sign rpm it cites
came from armed runs that were pivoting in place rather than rolling. The
2026-11-19 motor test separates the two conventions and supersedes it.

The mainboard was NOT reflashed: the stock build is byte-verified against the
Link protocol and only the Pico mapping changed. If the mainboard is ever
reflashed with `INVERT_R_DIRECTION` to regain upstream semantics, both
conventions above must be flipped together with it; mixing them breaks the
armed behaviour silently.

### 8.3 Bench evidence (2026-11-18)

All of the following is from recordings made with the console's recorder
(`csv/jbot-telemetry-1795042712.csv`, `csv/jbot-actions-1795042712.csv`,
`logs/jbot-events.log`). Nothing here substitutes for §8 items 1 and 3-7, which
remain open.

* **Command path identification.** The armed segment (pscale 0.20, wheels
  held by hand) was fitted with `torque = kp_eff * (pitch - ref - demand)
  + kd_eff * pitch_rate`: `kp_eff = 5.76 Nm/rad` against the compiled
  5.70 (pscale 0.2 x 28.5) and `kd_eff = 1.41 Nm/(rad/s)` against the
  compiled 1.40; residual RMS 4.3 units (~0.02 Nm). The path from attitude
  error to applied common-mode torque is verified in sign and magnitude at
  the low gain scale; nothing is verified at full gain.
* **Soft start works.** The reference anchored to the measured pose, torque
  ramped from zero, no step was observed. `dem` reached the -8 deg demand
  cap only while the operator held the robot at ~12-13 deg of forward lean.
* **Travel estimate is consistent.** `x` (integrating feedback speed) grew
  to 0.63 m during the lean hold; RPM readings were self-consistent.
* **The arm gate refuses out-of-capture arming** as designed
  (`err arm refused: capture level first`), and a latched `stop=pose_error`
  from an earlier session required an explicit `clear` before the run.
* **No `mode=4` (fall)** rows exist in any v2 recording made so far. Every
  arm test ran at pscale <= 0.35 with the body near equilibrium; the law
  has never carried a recovery, held a slope, or been allowed to fall.
* **Zero-capture repeatability is operator-limited.** Identical "hold
  vertical" captures within a day vary by several degrees
  (pitch=-6.81, -3.26, -2.23, -0.55). This is what the auto-calibration
  feature controls (stillness-gated capture); use it for every zero/level.
* **`mount` was 1 (flipx)** at the end of the session, after the 0...5
  sweep. The IMU was still mounted sideways-vertical relative to the
  chassis at that time (superseded by §8.6).
* **Backward-tip fall at pscale 0.2 (2026-11-18, session
  `csv/jbot-telemetry-1795045179.csv`).** From a stable hold the operator
  nudged the body backward (pitch < 0). Breakaway followed: -2 deg at
  t=12.12 s, -5 deg at 12.87 s, -10 deg at 13.14 s, -16 deg at 13.24 s,
  -28 deg at 13.37 s (1.2 s tip-over). Wheels drove backward (sign
  correct), rpm grew to ~200/200, torque saturated at -1000 units; the
  outer demand slammed into its +8 deg cap by t=13.14 s and could not
  out-lean the fall (pitch - ref - demand stayed deeply negative). The
  fall guard tripped at the 28 deg pose-error threshold
  (`stop=pose_error`, `fall=2`), torque went to zero, e-stop was not
  needed, the link and USB stayed up through the fall, and the body
  came to rest near -54 deg. Conclusion filed at the time: pscale 0.2
  cannot recover a dynamic lean at all; every earlier "hold" at 0.1-0.35
  worked only because the robot was kept near equilibrium by hand.

**Confound (found 2026-11-19).** Every armed run in this section went
through the negated mixer described in the 2026-11-18 version of §8.2, so the
common-mode torque the law produced reached the wheels as a *steer* command:
the wheels counter-rotated instead of driving the body. The command-path fit
(the first bullet) still holds, because it only checks torque magnitude and
sign against the attitude error. The conclusions about recovery margin and
about "rpm ~200/200 driving backward" do not: same-sign rpm means a pivot on
this wiring. The pscale ladder has to be re-run after the §8.2 correction
before any statement about gain headroom is made.

### 8.4 Settings persistence (2026-11-18)

The "lost its axes" report is explained by the boot state, not by the law:
`MOUNT` initialised to `Mount::Normal` on every boot, `mount`/`zero`/`level`
lived in RAM only, and the two documents disagreed about the correct selector
(§8.3 records `mount 1`, `docs/pico-bridge.md` recorded `mount 5`). A reboot
therefore silently restored a wrong attitude mapping, `zero` and `level`
could be captured against it, and the arm gate accepted the result.

Change (M3-42, partial):

* The mount, the zero offsets and the level reference are stored in the last
  4 KiB erase block of the QSPI flash. `memory.x` ends `FLASH` at `4092K`, so
  the record cannot overlap the image.
* The record is 32 bytes with magic, version, CRC-32 and range checks
  (`firmware/pico_core/src/nvm.rs`, host-tested). Blank, foreign, truncated or
  out-of-range content decodes to "no record" and the compiled defaults apply.
* The **mount is restored at boot**. `zero` and `level` are stored but not
  applied: both must still be captured in the current session, and the arm
  gate is unchanged. `pscale` is deliberately not stored.
* Writes happen on `mount`, `zero`, `level` and the explicit `save`; an erase
  stalls the executor, so a write is refused while the law is armed. Nothing
  on the control path, no limit, and no fail-safe transition was changed.

Still open: a flash wear budget (one erase per capture; the ~100k cycle
rating is far beyond bench use, but it is not written down), and a deliberate
downgrade path — an older firmware refuses an unknown record version instead
of misreading it, which is the safe default but leaves the operator with the
compiled defaults and no explanation beyond the boot line.

### 8.5 Offline analysis of the recorded sessions (2026-11-18)

`jbot-analyze` (`crates/jbot_analysis`, `docs/telemetry-analysis.md`) was run over
all 14 recordings. It confirms §8.3 from the files alone and adds three numbers
the bench could not see:

* **The falling run reproduces the compiled law.** On
  `csv/jbot-telemetry-1795045179.csv` the two-predictor fit over 235 unsaturated
  armed samples gives `kp = 1201 units/rad = 6.0 Nm/rad` and
  `kd = 223 units/(rad/s) = 1.11 Nm/(rad/s)`, against the `stat` line's
  `kpnm=5.7`, `kdnm=1.4`. `R2 = 0.9996`, residual 3 units. The gain that was
  applied is the gain that was requested; the surviving 21 % `kd` gap is the
  BNO055 fusion lag in the angle derivative, not a firmware fault.
* **The dead zone is where the model says it is.** The breakaway bracket is
  84..284 units (0.42..1.42 Nm nominal): the drive stayed at zero speed up to
  84 units and was rolling from 284. This is consistent with the assumed 0.5 Nm
  dead zone and explains why the first 3 s of the fall produced no wheel motion
  at all — the law was still inside the dead zone.
* **The recovery failure was called a gain failure, not a sign failure
  (historical).** `mount 1` reads `pitch` against `gy` at correlation +0.90,
  slope 0.83, and the fit's `kp` is positive: the channels are paired and the
  law pushes the right way. 5.7 Nm/rad is about a fifth of the 28.5 Nm/rad the
  nominal plant needs. The command path is still confirmed, but the conclusion
  that the *machine* could not recover is confounded: §8.3's mixer mapping
  turned that common-mode torque into a pivot, so the wheels never produced the
  linear force the fit assumes.
* **`mount 0` was visibly wrong before the re-solder (historical).** In the
  sweep session (`csv/jbot-telemetry-1795041000.csv`, `mount=0`) 99 % of the
  angular variance sits in roll while pitch is flat, and the roll pair is
  unpaired: a tilt of the chassis did not reach the pitch channel at all. That
  was the pre-2026-11-19 module orientation; §8.6 supersedes it.
* **The roll pair did not close on the older sessions (historical).** Four of
  the eight pre-`mode` recordings put the roll angle against `gx` at
  correlation −0.6 to −0.8, and three cross the −0.7 inversion threshold. Those
  sessions predate the `mount` command and ran on the compiled default,
  `mount normal`, where the law was fed BNO055 Euler angles but the damper was
  fed the wrong raw axis. §8.6 records the corrected pairing.

Ten of the 14 files predate the `mode` column, so arming and falls are reported
as not measurable for them rather than as zero.

### 8.6 IMU re-soldered, `mount normal` is correct (2026-11-19)

The BNO055 was re-soldered to the chassis and the console's attitude window now
matches the physical pose with `mount normal`: upright reads pitch ≈ 0,
on-back ≈ −90°, face-down ≈ +90°. The module's own axes are *not* robot-aligned
(the Euler pitch is a rotation about the chip X axis, which runs along the
robot's axle direction), but the fused Euler angles are the lean angles the law
wants, so the Euler path is the correct one for this selector. The previous
`flipx` correction is gone:

* The compiled firmware default is `Mount::Normal` (`mount 0`).
* The console's mount selector also defaults to `normal` before the first
  `stat`/telemetry frame arrives; previously it started at `flipx`.
* The raw pitch rate is the **X** channel, not Y. Least squares on the
  2026-11-19 session (`csv/jbot-telemetry-1795141040.csv`, 1852 samples) gives
  `d(pitch)/dt = 1.003·gx - 0.077·gy + 0.022·gz`, `R^2 = 0.94`. The
  gravity-derived mounts keep `gyro.y` because their pitch is a robot-frame
  rotation about Y. The old `Mount::Normal` fed `gyro.y` to the damper, so the
  pitch damping term was measuring the sideways axis; that is the second fault
  fixed in §8.7.
* The flash record version is now 2. A version-1 record is refused, so the
  stored `mount 1` from the pre-resolder session cannot come back after a
  reboot; the firmware logs `nvm: no stored record, using compiled defaults`
  and starts on `mount normal`. Nothing else in the record was applied at boot,
  so no calibration is lost — `zero` and `level` are still captured per
  session.
* Run `save` once (`mount normal`, `zero` and `level` captured) to write a
  version-2 record for the new orientation.

The historical `mount 1` fits in §8.3 and §8.5 describe the old module
orientation and remain evidence of the wrong-mount failure mode, not the
current setup.

### 8.7 Spin on arm: mixer sign and pitch-rate axis (2026-11-19)

Arming made the robot spin in a circle instead of driving forward or backward.
Two independent faults explain it; both were reproduced from the recorded
session before any code changed.

1. **The mixer sign was inverted.** `to_command` negated the right wheel before
   the EFeru mixer inverse. That maps common-mode `WheelTorque` to the wire
   `steer` channel, i.e. counter-rotation, so every balance output pivoted the
   machine. The motor test separates the conventions cleanly: `test 50 50`
   (wire `speed=50`) rolls forward, `test 100 -100` (wire `steer=100`) pivots.
   The fix is the plain inverse `Command::from_wheel_torque(left, right)`.
2. **The pitch damper read the wrong raw axis.** Under `mount normal` the law
   consumed BNO055 Euler pitch but the rate filter consumed raw gyro Y.
   `jbot-analyze` on the same session reports `pitch vs gy: corr +0.04,
   UNPAIRED`, while raw X fits the pitch derivative with slope 1.00. The fix is
   `pitch_rate = gyro.x`, `roll_rate = gyro.y` on the Euler path.

The wheel-speed sign was corrected at the same time: a forward roll reads
opposite-sign hall speeds, so `forward_speed_mps` is the half-difference. The
old mean reported ~0 while the machine rolled and grew only during the pivot,
which also corrupted `x` and the travel guard.

Yaw is now in the loop: `Tuning::from_plant` sets `yaw_kp_units = 0.5 · units_per_nm`,
so a 1 rad/s yaw-rate error asks for 0.5 Nm per wheel, and the differential may only
use the torque headroom balance leaves unused (`M3-13`). The sign is the bench-verified
one: a positive yaw-rate error drives the right wheel forward and the left backward
(counter-clockwise), which reaches the wire as a negative `steer` through the plain
mixer inverse. If the steering direction is ever inverted on the bench, invert the
yaw sign and the wire convention together.

Open after this change: re-run the pscale ladder (start at 0.10-0.20) with the
fixed mixer and rate axis before any claim about recovery margin, and flash the
mainboard `TRQ_MODE` profile (§2.10 of `docs/pico-bridge.md`) so the Pico's
torque request is not applied as a voltage command.

## 9. Open items

1. Bench identification of mass, centre-of-mass height, total torque per 1000 units,
   and the drive dead zone. These decide the real lift limit and therefore whether
   the "lying down" requirement can be met: at the assumed 5 Nm it cannot, and the
   donor phase current limit (10 A per motor) leaves headroom if the torque per amp
   turns out to be higher. Until the torque is measured, raising
   `PlantParams::NOMINAL.max_torque_nm` on a guess is not allowed.
2. Decide whether a stronger drive (higher current limit or a geared hub) is in scope
   for stage 1; the cascade itself does not change.
3. Slope tolerance beyond 6–8° needs either a larger torque budget or a
   surface-normal estimate for feedforward.
4. Decide whether lying-down recovery is in scope. It needs a measured budget of
   ~25–40 Nm and a raise manoeuvre (momentum build-up plus catch), validated in
   simulation first; the current threshold logic cannot simply be widened.

## 10. Plant correction (2026-11-20)

`PlantParams::NOMINAL` now uses the bench-weighed `config/robot.toml` values, with
the per-wheel entries doubled because the plant equations take the combined
wheels: body 5.0 kg, combined wheels 4.6 kg, combined wheel inertia
0.0156 kg·m², COM 0.25 m, radius 0.0825 m, and the total 5.0 Nm common-mode
budget (2 × 2.5 Nm per wheel). The dead zone stays at the estimated 0.5 Nm. The
resulting coefficients are:

| Coefficient | Value |
|---|---|
| `A1` | 25.488 1/s² |
| `A2` | 4.727 1/(kg·m) |
| `B1` | 1.516 1/(kg·m) |
| `B2` | 2.679 1/s² |
| `S1` | 20.576 1/s² |
| `S2` | 10.082 1/s² |
| stability floor `A1/A2` | 5.392 Nm/rad |
| lift limit at 5 Nm | 53.1° |

The whole law re-derives from these numbers: `kp = 21.57 Nm/rad`,
`kd = 5.92 Nm/(rad/s)`, `accel_per_rad = 7.33 m/s²/rad`, `pitch_ki = 1.08`.
The level-ground hold, recovery, driving and turning simulations all pass with the
corrected plant; the slope envelope is the one behaviour that changed, as recorded
in §3.3 and §6.

The corrected plant makes the machine *less* stiff per newton-metre than the old
estimate (floor 5.39 vs 7.13 Nm/rad), so a given lean needs less holding torque,
while the dead zone is a larger fraction of the useful range. That is why the
pitch-error integrator, not a dead-zone feedforward, is the mechanism that removes
the residual lean: the exact inverse acts like a relay and pumps the stance.
