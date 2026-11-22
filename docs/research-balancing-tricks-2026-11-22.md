# Research: self-balancing robots, self-righting and tricks (2026-11-22)

## Scope and method

Question: what do public projects actually do to balance a two-wheel machine,
recover after a fall, and perform tricks, and which of those techniques transfer
to this robot (hoverboard hub motors, STM32 F103 FOC mainboard, Pico 2 W bridge,
BNO055, ~5.5 Nm measured torque budget, 5.5 kg body above the axle).

Sources: GitHub repository/code/issue search through the GitHub API and web
search through DuckDuckGo. The configured web-search integration returned
HTTP 402, so DuckDuckGo HTML results were used for papers and articles.
Everything below is attributed to a public source; nothing is asserted about
this robot's hardware beyond the repository's own audit records.

## Closest hardware analogues

| Project | Hardware | Control | What to borrow |
|---|---|---|---|
| [LuSeKa/HoverBot](https://github.com/LuSeKa/HoverBot) (130 stars, deprecated; successor [CharlestonRobotics/ChIMP](https://github.com/CharlestonRobotics/ChIMP)) | Arduino Mega, BNO055, ODrive 48 V in current mode, hoverboard hub motors | Five tuned gains: pitch/rate -> current, throttle -> speed, steering/yaw -> differential | The README names the exact failure mode we fixed: no velocity feedback to the onboard controller, so a fast roll cannot be caught. Our wheel-RPM feedback is an advantage. |
| [mjbots/hoverbot](https://github.com/mjbots/hoverbot) (63 stars, Apache 2.0) | Pi target, C++/Bazel, full CAD + BOM | Complete development and data-analysis tooling | Engineering layout of a balanced hoverbot: host-side simulation, configs per robot, deployment target. |
| [simplefoc/Arduino-FOC-balancer](https://github.com/simplefoc/Arduino-FOC-balancer) (278 stars) | Gimbal BLDC + encoders + SimpleFOC | Modular motor/sensor/MCU stack, FOC torque control | Structure for separating torque control from the balance law; relevant to the stage-2 actuator choice. |
| [EFeru issue #300](https://github.com/EFeru/hoverboard-firmware-hack-FOC/issues/300) | USART variant, commutation mode | Upstream self-balancing requested in 2022, still open, no implementation | Confirms the upstream FOC firmware has no balance mode; our external Pico loop is the right layer. |

## Self-righting and tricks

| Project | Result | Key detail |
|---|---|---|
| [mahowik/BalancingWii](https://github.com/mahowik/BalancingWii) (305 stars) | Auto-rise after a fall, position hold, manual rise, speed/steering | Implements a **two-phase momentum raise**, not a static hold. Source `BalancingWii.cpp` lines ~1282-1333: below 25 deg full gain; in the 25-80 deg band `dynK = (1000 - |angle_0.1deg|)/1000 + 0.08` de-rates the command; past 80 deg, phase 0 accelerates in the fall direction up to `MAX_RISE_SPEED=140`, phase 1 reverses up to `MAX_REVERSED_RISE_SPEED=100` to throw the body upright, phase 2 is neutral. `RISE_SPEED_K` scales the phase ramp. Steering is forced to zero while rising. |
| [oyzlight/mujoco-lqr-vmc-wheel-legged-robot](https://github.com/oyzlight/mujoco-lqr-vmc-wheel-legged-robot) | Forward/backward self-righting from 70 deg (validated to 110 deg), jump, 15 cm step climb, drive and pivot | LQR + virtual model control on a closed-linkage wheel-legged robot; 0.5 ms physics, 5 ms control; per-wheel motor peak 2 Nm, per-hip 7 Nm. **Full side-lying and upside-down are explicitly unsupported.** Simulation only, no hardware validation. Closest published blueprint for our stage-2 legs. |
| [Deng-ge-open-source/RL_SelfRisingRobot_StackForce](https://github.com/Deng-ge-open-source/RL_SelfRisingRobot_StackForce) | Two-axis toy that gets up after being pushed over | ESP32 + BNO055 + two MG90S servos, pretrained ONNX policy, local MuJoCo runtime and a browser sim2real flow. Morphology differs, but the sim -> train -> deploy pipeline is reusable. |
| [AhmetEminArslanbas/esp32-self-balancing-robot](https://github.com/AhmetEminArslanbas/esp32-self-balancing-robot) | Push recovery, position and heading hold, 180 deg in-place turn, self-righting | 500 Hz FreeRTOS cascade PID, encoderless position/heading, live tuning UI. Its `simulation/sim_balance.py` ports the firmware law line by line and adds DC-motor dynamics, Coulomb friction, IMU noise/bias, 2 ms delay and accel-angle error. Good template for improving our plant fidelity. Firmware is not fully published. |
| [ngzhili/Two-Wheel-Robot-DeepRL](https://github.com/ngzhili/Two-Wheel-Robot-DeepRL) | Balancing, driving, slope traversal | DQN/SAC/A2C in PyBullet, DOI 10.5281/zenodo.15045301. Evidence that RL handles slopes without an explicit trim integrator, but needs a simulator first. |
| [udesa-linar/balancio-kit](https://github.com/udesa-linar/balancio-kit) | Tiny low-cost robot running a neural policy on ESP32 | Educational RL/control stack with MPU6050 calibration, Python simulation, wireless control. |
| [giulioturrisi/Self-Balancing-Robot](https://github.com/giulioturrisi/Self-Balancing-Robot), [IsaacGymEnvs-DIY-Robots](https://github.com/giulioturrisi/IsaacGymEnvs-DIY-Robots) | LQR/ROS2 and Isaac Gym environments for TWIP and flywheel pendulums | Simulation-first workflow for control comparison. |
| [gonzafernan/self-balancing-lqg](https://github.com/gonzafernan/self-balancing-lqg), [pattacini/wheeled-inverted-pendulum](https://github.com/pattacini/wheeled-inverted-pendulum) | LQR/LQG with Kalman filtering | State-space alternatives to our cascade; useful once the plant is measured. |
| [psiorx/ADRC](https://github.com/psiorx/ADRC) | Active disturbance rejection on a wheeled inverted pendulum | Explicit disturbance estimation instead of an integrator; candidate for the slope/dead-zone problem. |

## Literature

- [IEEE 8979168](https://ieeexplore.ieee.org/document/8979168), "An Integrated Dynamic Fall Protection and Recovery System for Two-Wheeled Humanoids": combines passive protection with active recovery; argues a pure balance controller cannot cover every fall.
- [arXiv 2606.14270 / RA-L 2026](https://arxiv.org/abs/2606.14270), "Robust Fall Recovery for Armless Bipedal-Wheeled Robots Via Force-Guided Learning": FTSR, force-guided teacher-student RL with stage-wise rewards; deployed on hardware. Confirms that armless/bipedal-wheeled self-righting is a current research problem, not a threshold tweak.
- [IEEE 11002466](https://ieeexplore.ieee.org/document/11002466), composite self-balancing control, including inclined-surface braking.

## mjbots/hoverbot: detailed source read

The repository user pointed at deserves a closer read than a star count. All
line numbers below refer to the default branch at the time of this research.

**Architecture.** Raspberry Pi 4 + pi3hat + two moteus-c1 controllers, Apache
2.0, C++/Bazel, full CAD and BOM. `configs/hoverbot.cfg` uses a 2.5 ms control
period (400 Hz), wheel diameter 0.163 m, `min_voltage=16.0`, `max_tip_deg=65`,
`tip_filter_s=0.1`, pitch PID `kp=0.05, kd=0.005, ilimit=0`, yaw PID
`kp=0.02, kd=0.002, ilimit=0`, drive PID `kp=20, ki=20, kd=0, ilimit=10`,
drive pitch limit 45 deg, stand-up pitch rate 360 deg/s.

**Control loop** (`mech/hoverbot_control.cc`):

* `HandleTimer` (line 258) requests IMU and both servo statuses in one pi3hat
  cycle; a skipped cycle (`delta_s > 1.5 * period_s`) is warned about and
  rate-limited.
* `HandleStatus` (line 290) low-passes the *guard* tilt with
  `alpha = 0.5^(period/tip_filter_s)` for both pitch and roll, but
  `ControlPitch` (line 813) feeds the PID with the raw `imu_data_.euler_deg.pitch`
  and `rate_dps.y()`; the filter is therefore not in the control path.
* Velocity comes from the average servo speed (line 513):
  `v = -wheel_diameter * pi * average(velocity_dps / 360)`. Acceleration is
  hard-coded to zero with a TODO (line 521); there is a voltage filter (1 s)
  and a low-battery fault at 16 V.
* The tip guard (line 357) faults only when `|tip_pitch|` or `|tip_roll|` exceeds
  65 deg **and** `in_control_time_s > 5.0`; the grace period is what allows
  the stand-up manoeuvre to start from a fallen pose.
* `RunControl` (line 602) dispatches an explicit mode enum: `Configuring`,
  `Stopped`, `Fault`, `ZeroVelocity`, `Joint`, `StandUp`, `Pitch`, `Drive`.
  Requesting `Pitch`/`Drive` from `Stopped` or `ZeroVelocity` first enters
  `StandUp` (line 684) and is only accepted once the stand-up target reaches
  zero; mode changes clear both PID states and re-anchor the yaw target.
* `DoControl_StandUp` (line 854) ramps a pitch target linearly from the
  measured angle to zero at `pitch_rate_dps` and passes the ramp slope as
  pitch-rate feedforward, with yaw disabled. The source comment explicitly
  says this is not control-optimal but makes the motion smooth.
* `ControlPitch` (line 808) is a PID on measured pitch against
  `-command.pitch_deg + pitch_offset_deg` and measured rate against
  `-command.pitch_rate_dps`, output directly as `torque_Nm` on both wheels;
  yaw torque is added with opposite signs. Unlike our law, yaw is **not**
  limited to the headroom balance leaves.
* `ControlDrive` (line 878) is a velocity PID whose output is clamped to
  `+/-pitch_limit_deg` and used as the pitch reference. There is **no position
  loop and no slope trim**: the machine holds velocity, not place.

**What transfers to jbot.** The mode state machine, the stand-up reference ramp
with rate feedforward, the guard grace period, the command priority/timeout,
and the timing instrumentation. What does not transfer: the 400 Hz rate (our
F103 loop is 200 Hz), the unfiltered control path, the direct yaw addition
without headroom reservation, and the absence of position/slope control (we
have all three).

## Findings that change our assumptions

1. **Static lift limit is not the only route to self-righting.** `docs/design_balance_home.md` section 6.1 sweeps the same linear cascade without guards and finds 25-40 Nm for a lying start. BalancingWii gets up with a *dynamic* throw-and-catch: accelerate, abruptly reverse, then let the balance loop catch the body. This is exactly the "feel the body at each angle" the operator asked about. The 25-40 Nm number still applies to holding a large lean statically, but the dynamic manoeuvre needs its own design and validation. This does **not** mean we can simply relax `fall_error_rad`.
2. **Mid-angle gain de-rating is a known requirement.** BalancingWii lowers output authority in the 25-80 deg band (`dynK`); our `pscale` is a constant multiplier and does not do this.
3. **Momentum raise needs a dedicated state machine** with a neutral preparation phase, a forced zero-steering rule and an explicit catch handover. Our current arm/fall path has only two states (armed/latched).
4. **Hardware ceiling is real.** Even with legs and 7 Nm hip peaks, the wheel-legged reference supports 70-110 deg forward/back recovery but not full side or upside-down. With two wheels and ~5.5 Nm, spins, pivots and controlled falls are plausible; backflips and supine self-righting require either a stronger drive or extra actuators (stage-2 legs / reaction wheel).
5. **Simulation fidelity matters.** The most useful published sims include motor dynamics, Coulomb friction, sensor delay and noise. Our plant already has dead zone and transport delay, but not motor speed/torque limits or a nonlinear large-angle geometry.

## Proposed roadmap for this repository (gated)

1. **Simulation only, no hardware:** add a nonlinear large-angle plant (sin instead of the linear term, wheel speed/torque limits, dead zone, delay) and implement a pure two-phase raise function modelled on BalancingWii, with unit tests and a sweep over start angles and torque budgets. No flashing.
2. **Bench identification:** add the proposed open-loop `test <u>` slow torque-ramp command; measure the real dead zone and units/Nm under load; record into `csv/` through `archive/python-tools/bench_paths.py`.
3. **Behaviour layer:** introduce explicit modes (`balance`, `drive`, `spin`, `raise`, `safe-fall`) with entry/exit guards, estop priority and telemetry, keeping the existing safety function as the single source of truth.
4. **Only after 1-3 pass:** request fresh step-by-step safety approval for any raise trial (rope, hands on body, <=0.5 m zone, minimal torque, smooth ramp/damping).
5. **Stage 2:** if jumps, step climbing or recovery from full side-lying are required, adopt the LQR + VMC architecture from the wheel-legged reference rather than trying to stretch the two-wheel law.

Steps 1-3 are code and simulation; step 4 changes hardware interaction and must not start without approval.
