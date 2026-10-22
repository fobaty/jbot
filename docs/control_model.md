# 2D WIP Model and LQR Baseline

| Field | Value |
|---|---|
| Document | Reduced-order WIP model and LQR synthesis |
| Code | CM-001 |
| Version | 0.1 |
| Date | 2026-10-22 |
| Status | Draft, Phase 1 baseline |
| Scope | 2D wheeled inverted pendulum, linearization, ZOH discretization, LQR |
| Code map | `crates/jbot_control/src/wip.rs`, `crates/jbot_control/src/lqr.rs` |

## 1. Purpose and scope

This document records the first control-oriented model of JBOT-WheeledBiped: a
planar wheeled inverted pendulum (WIP) driven by the total wheel torque. It is
the reduced-order baseline required by task `M1-11`; the 3D model with roll, yaw,
and four-bar height/offset coupling is a later increment.

The model is deliberately small. It captures the unstable pole, the
non-minimum-phase cart-pole behaviour, and the wheel reaction torque that
dominates the 1 kHz balance loop. It does **not** model the leg linkage mass,
compliance, friction, slip, or actuator limits.

## 2. Assumptions and validity limits

- Motion is constrained to the fore-aft plane: no roll, yaw, or lateral slip.
- The ground is rigid and dry; the wheels roll without slipping.
- The body is a rigid pendulum with a fixed centre of mass (CoM).
- The leg linkage is massless and rigid for this baseline; its kinematics are
  handled separately in `docs/kinematics.md`.
- The two wheels are identical and driven by the same torque command.
- Linearization is valid for small pitch (`|theta| << 1 rad`) and small
  velocities. Large-angle recovery is outside this model.
- Parameters are preliminary design estimates from `config/robot.toml`; they
  must be replaced by CAD or bench measurements before Gate G1.

## 3. Coordinates and parameters

The generalized coordinates are:

```text
q = [x, theta]^T
```

- `x` — wheel-centre displacement in the fore-aft direction, metres.
- `theta` — body pitch, positive forward, radians.
- `u` — total wheel torque applied by both wheels, N*m.
- Each wheel receives `u / 2`; the model assumes no yaw moment.

The reduced parameters are:

```text
a11 = M_b + M_w + I_w / r^2
B   = M_b * l
C   = I_b + M_b * l^2
D   = M_b * g * l
det = a11 * C - B^2
K_r = 1 + B / (a11 * r)
```

where:

| Symbol | Meaning | Baseline value |
|---|---|---:|
| `M_b` | body mass, kg | 6.0 |
| `M_w` | total wheel mass, kg | 1.8 |
| `I_b` | body pitch inertia about the axle, kg*m^2 | 0.35 |
| `I_w` | total wheel inertia, kg*m^2 | 0.008 |
| `r` | wheel radius, m | 0.09 |
| `l` | body CoM height above the axle, m | 0.25 |
| `g` | standard gravity, m/s^2 | 9.80665 |

`WipParameters::from_config` doubles the per-wheel mass and inertia from the
configuration, because the WIP input is the total wheel torque.

## 4. Nonlinear 2D model

The mass matrix and force balance are:

```text
M(q) = [ a11              B*cos(theta) ]
       [ B*cos(theta)     C            ]

a11*x_ddot + B*cos(theta)*theta_ddot + B*sin(theta)*theta_dot^2 = u / r
B*cos(theta)*x_ddot + C*theta_ddot - D*sin(theta) = -u
```

The first equation is the horizontal force balance; the second is the pitch
moment balance. The `B*sin(theta)*theta_dot^2` term is the centrifugal coupling
of the pendulum.

`WheeledInvertedPendulum::derivative` evaluates this model for arbitrary state
and torque. It rejects non-finite inputs and degenerate mass matrices.

## 5. Equilibrium linearization

At the upright equilibrium `x = 0`, `theta = 0`, `x_dot = 0`, `theta_dot = 0`,
`u = 0`, the linearized state equation is:

```text
x = [x, x_dot, theta, theta_dot]^T
x_dot = A*x + B_mat*u
```

```text
A = [ 0   1   0                    0 ]
    [ 0   0  -B*D/det             0 ]
    [ 0   0   0                    1 ]
    [ 0   0   a11*D/det           0 ]

B_mat = [ 0                              ]
        [ 1/(a11*r) + B*K_r/det         ]
        [ 0                              ]
        [ -a11*K_r/det                  ]
```

With the baseline parameters:

```text
A = [ 0   1    0           0 ]
    [ 0   0   -5.35420968  0 ]
    [ 0   0    0           1 ]
    [ 0   0   31.36729589  0 ]

B_mat = [ 0             ]
        [ 2.31871901    ]
        [ 0             ]
        [ -6.17666003   ]
```

The analytic linearization is cross-checked against central finite differences
in `analytic_linearization_matches_numeric_linearization`.

## 6. Zero-order-hold discretization

The 1 kHz loop uses `dt = 0.001 s`. The exact ZOH model is obtained from the
augmented matrix exponential:

```text
M_aug = [ A       B_mat ]
        [ 0       0     ]

exp(M_aug * dt) = [ A_d   B_d ]
                  [ 0     I   ]
```

With the baseline parameters:

```text
A_d = [ 1   0.001          -2.67711184e-6   -8.92369680e-10 ]
      [ 0   1              -5.35423767e-3   -2.67711184e-6  ]
      [ 0   0               1.00001568       1.00000523e-3  ]
      [ 0   0               3.13674599e-2    1.00001568     ]

B_d = [ 1.15936088e-6  ]
      [ 2.31872452e-3  ]
      [ -3.08833809e-6 ]
      [ -6.17669232e-3 ]
```

`zoh_matches_first_order_expansion_at_one_millisecond` verifies that `A_d` and
`B_d` agree with `I + A*dt` and `B*dt` within the expected second-order error.

## 7. Controllability and observability

The continuous and discrete controllability matrices have rank 4 for the
baseline parameters. The model is therefore controllable with a single total
wheel torque.

The observability matrix also has rank 4 when `x` and `theta` are measured. In
practice `x` comes from wheel odometry and `theta` from the IMU; velocity and
pitch rate are reconstructed by the estimator. The rank test is a necessary
condition, not a noise or delay analysis.

`model_is_controllable_and_observable` and `discrete_model_stays_controllable`
cover these checks. The admissible parameter range that preserves
controllability is not yet mapped; that is part of `M1-13`.

## 8. LQR synthesis

The baseline uses the discrete algebraic Riccati equation (DARE):

```text
P = A_d^T P A_d
    - A_d^T P B_d (R + B_d^T P B_d)^-1 B_d^T P A_d
    + Q

K = (R + B_d^T P B_d)^-1 B_d^T P A_d
u = -K x
```

The implementation uses the Joseph form:

```text
K[k]   = (R + B_d^T P[k] B_d)^-1 B_d^T P[k] A_d
P[k+1] = (A_d - B_d K[k])^T P[k] (A_d - B_d K[k])
         + K[k]^T R K[k] + Q
```

The Joseph form avoids the loss of symmetry and the stagnation of the direct
recursion at large `P` values. The solver iterates until the maximum absolute
change in `P` is below `1e-12`, with a hard limit of 200,000 iterations.

The default weights are:

```text
Q = diag(10, 1, 100, 10)
R = 1
```

They prioritize pitch and pitch-rate regulation, then position, then velocity.
`R = 1` is a placeholder based on the normalized torque command; it is not yet
scaled to the 8 N*m wheel limit.

For the baseline model the converged gain is:

```text
K = [-3.12102759, -4.55717702, -25.43982597, -5.91910071]
```

The discrete closed-loop eigenvalues of `A_d - B_d*K` have magnitudes:

```text
[0.97928595, 0.99667765, 0.99900000, 0.99900000]
```

All are inside the unit circle. The closed-loop poles are close to the unit
circle because the position loop is intentionally slow; this is expected for
the chosen `Q` and `R`.

## 9. Numerical notes

- The ZOH uses `nalgebra`'s matrix exponential on a 6x6 augmented matrix.
- `nalgebra`'s `Debug` output prints the column-major backing store, not
  display rows. Use explicit element access or `Display` when inspecting
  matrices.
- The Riccati solver validates that `R + B_d^T P B_d` is positive and finite at
  every iteration.
- The reference gain was cross-checked with `scipy.linalg.solve_discrete_are`
  for the same `A_d`, `B_d`, `Q`, and `R`.

## 10. Current limitations

- No 3D roll/yaw dynamics or four-bar height/offset coupling (`M1-11` extension).
- No leg inertia, spring, friction, slip, dead zone, or command delay (`M1-15`).
- No torque saturation or per-wheel allocation (`M1-20`).
- No outer velocity or yaw loop (`M1-18`).
- The `Q`/`R` selection is a starting point, not a tuned trade-off study
  (`M1-17`).
- The gain has not yet been validated on the nonlinear plant in a simulator.
- Parameter uncertainty is not propagated; the stability margins under parameter
  spread remain to be measured (`M1-36`).

## 11. Next increments

1. Validate the LQR gain on the nonlinear 2D plant with RK4 and disturbance
   injection.
2. Add torque saturation and the per-wheel torque split.
3. Extend the model to 3D with roll, yaw, and four-bar height/offset coupling.
4. Map controllability and stability margins over the parameter uncertainty.
5. Tune `Q` and `R` against the "recovery time / current / oscillation"
   trade-off and record the selected weights.
