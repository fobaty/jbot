# Kinematics — JBOT-WheeledBiped

| Parameter | Value |
|---|---|
| Document | Kinematic model and baseline parameter set |
| Code | KIN-001 |
| Version | 0.1 |
| Date | 2026-10-22 |
| Status | Phase 1 baseline, preliminary parameters |
| Basis | `docs/architecture.md` section 1.2, `docs/specification.md` MOB-01, `config/robot.toml` |

This document records the planar four-bar leg model, the differential-drive model,
the frame and sign conventions, and the parameter provenance required by tasks
M1-01, M1-03, M1-06, and M1-07.

> **Preliminary values.** The link lengths and mass-inertial parameters are design
> estimates, not CAD or bench measurements. Each value in `config/robot.toml`
> carries its source and uncertainty. Replace them before Gate G1.

---

## 1. Coordinate conventions

### 1.1. Body frame (REP-103)

The body frame follows REP-103: `+x` forward, `+y` left, `+z` up, right-handed.
A positive yaw rate is a counter-clockwise rotation, so the right wheel must be
faster than the left wheel for a positive yaw rate.

### 1.2. Leg plane

Each four-bar leg is planar. The leg plane uses `+x` forward and `+y` up. Point
`A` is the origin and point `B` lies on the positive `x` axis:

- `A` — driven hip joint (QDD axis).
- `B` — passive rocker pivot on the chassis.
- `C` — crank joint on the upper crank `A–C`.
- `D` — rocker joint on the lower rocker `B–D`.
- `E` — wheel axle, rigidly attached to the wheel fork `C–D–E`.

The hip angle `θ` is measured from the `A→B` direction, positive counter-clockwise.
The `A–B` link is the chassis and is fixed to the body; the wheel fork is the moving
coupler, not the base.

---

## 2. Four-bar leg

### 2.1. Link map and baseline values

| Link | Description | Symbol | Baseline | Uncertainty | Source |
|---|---|---|---:|---:|---|
| A–B | Chassis ground link | `L0` | 140 mm | ±5 mm | Preliminary packaging sketch |
| A–C | Driven upper crank | `L1` | 110 mm | ±5 mm | Preliminary linkage synthesis |
| B–D | Passive lower rocker | `L2` | 130 mm | ±5 mm | Preliminary linkage synthesis |
| C–D | Coupler segment of the fork | `L3` | 60 mm | ±5 mm | Preliminary linkage synthesis |
| D–E | Fork link to the axle | `L4` | 180 mm | ±5 mm | Preliminary linkage synthesis |
| C–E | Fork link to the axle | `L5` | 170 mm | ±5 mm | Preliminary linkage synthesis |

Mass-inertial baseline:

| Parameter | Symbol | Baseline | Uncertainty | Source |
|---|---|---:|---:|---|
| Body mass | `m_b` | 6.0 kg | ±1.0 kg | 15 kg dry-mass budget (MOB-28), battery excluded |
| Body pitch inertia | `J_b` | 0.35 kg·m² | ±0.10 kg·m² | Uniform-body estimate about the axle |
| Wheel mass (each) | `m_w` | 0.9 kg | ±0.2 kg | Hub motor, rim, and tyre estimate |
| Wheel radius | `r_w` | 90 mm | ±5 mm | 180 mm nominal diameter |
| Wheel inertia (each) | `J_w` | 0.0040 kg·m² | ±20 % | Solid-disc estimate plus hub |
| Track width | `L` | 420 mm | ±20 mm | Packaging sketch |
| Body CoM height above axle | `h_c` | 250 mm | ±50 mm | Nominal stance estimate |

### 2.2. Closure equations

With `A = (0, 0)` and `B = (L0, 0)`:

```text
C(θ) = (L1 cos θ, L1 sin θ)
|C − D| = L3,  |B − D| = L2
|C − E| = L5,  |D − E| = L4
```

`D` is one of the two circle-circle intersections of the circles `(C, L3)` and
`(B, L2)`. `E` is one of the two intersections of `(C, L5)` and `(D, L4)`. Both
choices are part of the assembly definition and are stored as `rocker_branch` and
`fork_branch` in the configuration. The baseline uses the negative branch for both.

The implementation is in `jbot_common::kinematics::four_bar`:

- `LegGeometry::forward` solves the closure without joint limits.
- `LegKinematics::forward` applies the configured hip window.
- `LegKinematics::inverse` searches the one-dimensional coupler curve and returns
  `OutsideWorkspace` when the target is farther than 1 mm from it.
- `LegKinematics::jacobian` returns `dE/dθ` by a bounded central difference.
- `LegKinematics::coupler_curve` and `vertical_range` expose the workspace.

### 2.3. Coupler curve and coupling

The mechanism has one active DOF. Over the configured hip window
`θ ∈ [-1.55, -0.70] rad`:

| Quantity | Minimum | Maximum | Travel |
|---|---:|---:|---:|
| Wheel-axle height `E_y` | −279.6 mm | −138.9 mm | **140.7 mm** |
| Wheel-axle fore-aft position `E_x` | −71.7 mm | +20.1 mm | **91.7 mm** |

The 140.7 mm vertical travel exceeds the 100 mm requirement. Height and fore-aft
offset are coupled along the same curve and cannot be commanded independently:
the hip angle selects a single point on this curve. The window deliberately stops
short of the linkage toggle positions, where the Jacobian approaches zero and the
leg loses stiffness.

Nominal points:

| Hip angle | Crank joint `C` | Rocker joint `D` | Wheel axle `E` |
|---:|---|---|---|
| −1.55 rad | (2.3, −110.0) mm | (62.0, −104.0) mm | (20.1, −279.0) mm |
| −1.125 rad | (47.4, −99.2) mm | (101.9, −124.3) mm | (−22.8, −254.1) mm |
| −0.70 rad | (84.1, −70.9) mm | (107.9, −126.0) mm | (−71.7, −138.9) mm |

The chassis height above the ground is `r_w − E_y` when the wheel contacts a flat
surface. With the 90 mm wheel radius, the nominal hip-axis height ranges from
roughly 229 mm to 370 mm over the window.

### 2.4. Validity limits

- Rigid links, ideal revolute joints, no backlash and no compliance in the baseline.
- The optional spring between the base and the upper crank is not fitted in the
  baseline (`spring_stiffness_nm_per_rad = 0`). If fitted, its stiffness and travel
  must be added to the dynamics model.
- The leg model is planar. Out-of-plane loads, bearing compliance, and link bending
  are outside the Phase 1 model.
- The inverse solution is numerical, not closed-form; it is intended for workspace
  queries, calibration, and simulation, not for the 1 kHz hot path.

---

## 3. Differential drive

`jbot_common::kinematics::differential_drive` implements:

```text
v       = (v_l + v_r) / 2
ω       = (v_r − v_l) / L
v_l     = v − ω L / 2
v_r     = v + ω L / 2
```

`v_l` and `v_r` are linear contact speeds in m/s. Wheel angular speed is converted
with `v = r_w ω_w`. The forward and inverse transforms are exact inverses for a
finite, non-zero track width.

Validation rejects non-positive or non-finite track width and non-finite inputs.

---

## 4. Configuration contract

`config/robot.toml` is parsed by `jbot_common::config::RobotConfig`:

- `schema_version` must equal `CONFIG_SCHEMA_VERSION` (currently 1).
- Unknown fields are rejected, so a typo cannot silently fall back to a default.
- Every mass, inertia, link length, gear ratio, actuator limit, and loop rate must
  be finite and positive where physically required.
- The hip window must be non-empty and the linkage must assemble at the window
  endpoints and midpoint.
- The wheel-axle vertical range must be at least 100 mm.
- A fitted spring requires positive travel.
- `config_hash` computes a stable FNV-1a 64-bit hash of the raw file bytes; the node
  logs it at startup so a trace can be tied to the exact configuration.

Run the model tests with:

```bash
cargo test -p jbot_common
```

---

## 5. Open items

1. Replace preliminary masses, inertias, and link lengths with CAD and bench values.
2. Confirm the hip travel window against the mechanical hard stops and the QDD
   encoder range.
3. Measure the actual wheel radius under load and the tyre contact patch.
4. Select the optional spring rate and travel, then add its compliance to the
   dynamics model.
5. Validate the coupler curve and the 140.7 mm travel on the assembled leg before
   using it for balance-controller synthesis.
