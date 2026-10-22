//! Reduced-order 2D wheeled inverted pendulum (WIP) model.
//!
//! The model captures longitudinal balance: wheel-axle displacement `x`, body
//! pitch `theta`, and their rates. It deliberately omits yaw, roll, leg coupling,
//! friction, backlash, and actuator dynamics; those belong to the 3D extension
//! and the simulator.
//!
//! Sign conventions:
//! - `x` is the wheel-axle displacement in the forward direction.
//! - `theta` is the body pitch, positive when the body leans forward.
//! - `u` is the total wheel torque applied by both wheels, positive forward.
//!   The per-wheel torque is `u / 2`.
//!
//! The equations follow the Lagrangian derivation in `docs/control_model.md`.

use std::fmt;

use jbot_common::config::RobotConfig;
use nalgebra::{DMatrix, SMatrix, SVector};

/// State vector: `[x, x_dot, theta, theta_dot]`.
pub type StateVector = SVector<f64, 4>;
/// 4x4 state matrix.
pub type StateMatrix = SMatrix<f64, 4, 4>;
/// 4x1 input matrix.
pub type InputMatrix = SMatrix<f64, 4, 1>;

/// Standard gravity used unless a caller supplies a different value.
pub const STANDARD_GRAVITY_MPS2: f64 = 9.80665;

/// Physical parameters of the reduced-order WIP model.
///
/// `wheel_mass_kg` and `wheel_inertia_kg_m2` are totals for both wheels, not
/// per-wheel values. `from_config` performs the doubling from the per-wheel
/// configuration fields.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WipParameters {
    pub body_mass_kg: f64,
    pub wheel_mass_kg: f64,
    pub body_pitch_inertia_kg_m2: f64,
    pub wheel_inertia_kg_m2: f64,
    pub wheel_radius_m: f64,
    pub com_height_m: f64,
    pub gravity_mps2: f64,
}

impl WipParameters {
    /// Build WIP parameters from the validated robot configuration.
    pub fn from_config(config: &RobotConfig) -> Self {
        Self {
            body_mass_kg: config.mechanical.body_mass_kg,
            wheel_mass_kg: 2.0 * config.mechanical.wheel_mass_kg,
            body_pitch_inertia_kg_m2: config.mechanical.body_pitch_inertia_kg_m2,
            wheel_inertia_kg_m2: 2.0 * config.mechanical.wheel_inertia_kg_m2,
            wheel_radius_m: config.mechanical.wheel_radius_m,
            com_height_m: config.mechanical.com_height_m,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        }
    }

    /// Validate the parameter set and the positive-definiteness of the mass matrix.
    pub fn validate(&self) -> Result<(), WipError> {
        positive("body_mass_kg", self.body_mass_kg)?;
        positive("wheel_mass_kg", self.wheel_mass_kg)?;
        positive("body_pitch_inertia_kg_m2", self.body_pitch_inertia_kg_m2)?;
        non_negative("wheel_inertia_kg_m2", self.wheel_inertia_kg_m2)?;
        positive("wheel_radius_m", self.wheel_radius_m)?;
        positive("com_height_m", self.com_height_m)?;
        positive("gravity_mps2", self.gravity_mps2)?;

        let (a11, coupling, c22) = self.mass_terms();
        let determinant = a11 * c22 - coupling * coupling;
        if !determinant.is_finite() || determinant <= 1e-9 {
            return Err(WipError::DegenerateParameters { determinant });
        }
        Ok(())
    }

    fn mass_terms(&self) -> (f64, f64, f64) {
        let a11 = self.body_mass_kg
            + self.wheel_mass_kg
            + self.wheel_inertia_kg_m2 / (self.wheel_radius_m * self.wheel_radius_m);
        let coupling = self.body_mass_kg * self.com_height_m;
        let c22 = self.body_pitch_inertia_kg_m2 + self.body_mass_kg * self.com_height_m.powi(2);
        (a11, coupling, c22)
    }
}

/// Linear state-space model around the upright equilibrium.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContinuousModel {
    pub a: StateMatrix,
    pub b: InputMatrix,
}

impl ContinuousModel {
    /// Numerically rank the controllability matrix `[B, AB, A^2 B, A^3 B]`.
    pub fn controllability_rank(&self) -> usize {
        let mut matrix = StateMatrix::zeros();
        let mut column = self.b;
        for index in 0..4 {
            matrix.set_column(index, &column);
            column = self.a * column;
        }
        rank_of(DMatrix::from_column_slice(4, 4, matrix.as_slice()))
    }

    pub fn is_controllable(&self) -> bool {
        self.controllability_rank() == 4
    }

    /// Rank the observability matrix for a `P`-row output matrix `C`.
    pub fn observability_rank<const P: usize>(&self, output: &SMatrix<f64, P, 4>) -> usize {
        let mut matrix = DMatrix::<f64>::zeros(4 * P, 4);
        let mut power = *output;
        for block in 0..4 {
            for row in 0..P {
                for column in 0..4 {
                    matrix[(block * P + row, column)] = power[(row, column)];
                }
            }
            power *= self.a;
        }
        rank_of(matrix)
    }

    pub fn is_observable<const P: usize>(&self, output: &SMatrix<f64, P, 4>) -> bool {
        self.observability_rank(output) == 4
    }
}

/// Zero-order-hold discretization of [`ContinuousModel`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiscreteModel {
    pub a: StateMatrix,
    pub b: InputMatrix,
    pub dt_s: f64,
}

impl DiscreteModel {
    /// Closed-loop matrix for the state-feedback law `u = -K x`.
    pub fn closed_loop_matrix(&self, gain: &SMatrix<f64, 1, 4>) -> StateMatrix {
        self.a - self.b * gain
    }

    pub fn controllability_rank(&self) -> usize {
        let mut matrix = StateMatrix::zeros();
        let mut column = self.b;
        for index in 0..4 {
            matrix.set_column(index, &column);
            column = self.a * column;
        }
        rank_of(DMatrix::from_column_slice(4, 4, matrix.as_slice()))
    }

    pub fn is_controllable(&self) -> bool {
        self.controllability_rank() == 4
    }

    pub fn observability_rank<const P: usize>(&self, output: &SMatrix<f64, P, 4>) -> usize {
        let mut matrix = DMatrix::<f64>::zeros(4 * P, 4);
        let mut power = *output;
        for block in 0..4 {
            for row in 0..P {
                for column in 0..4 {
                    matrix[(block * P + row, column)] = power[(row, column)];
                }
            }
            power *= self.a;
        }
        rank_of(matrix)
    }

    pub fn is_observable<const P: usize>(&self, output: &SMatrix<f64, P, 4>) -> bool {
        self.observability_rank(output) == 4
    }
}

/// Nonlinear 2D WIP plant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheeledInvertedPendulum {
    parameters: WipParameters,
}

impl WheeledInvertedPendulum {
    /// Create a plant from validated parameters.
    pub fn new(parameters: WipParameters) -> Result<Self, WipError> {
        parameters.validate()?;
        Ok(Self { parameters })
    }

    pub fn parameters(&self) -> &WipParameters {
        &self.parameters
    }

    /// Nonlinear state derivative for the given state and total wheel torque.
    pub fn derivative(
        &self,
        state: StateVector,
        total_wheel_torque_nm: f64,
    ) -> Result<StateVector, WipError> {
        for (index, value) in state.iter().enumerate() {
            if !value.is_finite() {
                return Err(WipError::NonFiniteInput {
                    name: STATE_NAMES[index],
                    value: *value,
                });
            }
        }
        if !total_wheel_torque_nm.is_finite() {
            return Err(WipError::NonFiniteInput {
                name: "total_wheel_torque_nm",
                value: total_wheel_torque_nm,
            });
        }

        let x_dot = state[1];
        let theta = state[2];
        let theta_dot = state[3];

        let (a11, coupling, c22) = self.parameters.mass_terms();
        let gravity = self.parameters.body_mass_kg
            * self.parameters.gravity_mps2
            * self.parameters.com_height_m;
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        let determinant = a11 * c22 - coupling * coupling * cos_theta * cos_theta;

        if !determinant.is_finite() || determinant <= 1e-9 {
            return Err(WipError::DegenerateConfiguration {
                pitch_rad: theta,
                determinant,
            });
        }

        let rhs_x = total_wheel_torque_nm / self.parameters.wheel_radius_m
            + coupling * sin_theta * theta_dot * theta_dot;
        let rhs_theta = -total_wheel_torque_nm + gravity * sin_theta;

        let x_ddot = (rhs_x * c22 - coupling * cos_theta * rhs_theta) / determinant;
        let theta_ddot = (a11 * rhs_theta - coupling * cos_theta * rhs_x) / determinant;

        Ok(StateVector::new(x_dot, x_ddot, theta_dot, theta_ddot))
    }

    /// Analytic linearization around the upright, zero-velocity equilibrium.
    pub fn continuous(&self) -> ContinuousModel {
        let (a11, coupling, c22) = self.parameters.mass_terms();
        let gravity = self.parameters.body_mass_kg
            * self.parameters.gravity_mps2
            * self.parameters.com_height_m;
        let determinant = a11 * c22 - coupling * coupling;
        let reaction = 1.0 + coupling / (a11 * self.parameters.wheel_radius_m);

        let mut a = StateMatrix::zeros();
        a[(0, 1)] = 1.0;
        a[(1, 2)] = -coupling * gravity / determinant;
        a[(2, 3)] = 1.0;
        a[(3, 2)] = a11 * gravity / determinant;

        let mut b = InputMatrix::zeros();
        b[(1, 0)] =
            1.0 / (a11 * self.parameters.wheel_radius_m) + coupling * reaction / determinant;
        b[(3, 0)] = -a11 * reaction / determinant;

        ContinuousModel { a, b }
    }

    /// Numerically linearize the nonlinear plant to cross-check [`Self::continuous`].
    pub fn numeric_linearization(&self, epsilon: f64) -> Result<ContinuousModel, WipError> {
        if !epsilon.is_finite() || epsilon <= 0.0 {
            return Err(WipError::InvalidParameter {
                name: "epsilon",
                value: epsilon,
            });
        }

        let zero = StateVector::zeros();
        let mut a = StateMatrix::zeros();
        for column in 0..4 {
            let mut plus = zero;
            plus[column] += epsilon;
            let mut minus = zero;
            minus[column] -= epsilon;
            let forward = self.derivative(plus, 0.0)?;
            let backward = self.derivative(minus, 0.0)?;
            for row in 0..4 {
                a[(row, column)] = (forward[row] - backward[row]) / (2.0 * epsilon);
            }
        }

        let plus = self.derivative(zero, epsilon)?;
        let minus = self.derivative(zero, -epsilon)?;
        let mut b = InputMatrix::zeros();
        for row in 0..4 {
            b[(row, 0)] = (plus[row] - minus[row]) / (2.0 * epsilon);
        }

        Ok(ContinuousModel { a, b })
    }

    /// Exact zero-order-hold discretization at `dt_s`.
    pub fn discrete(&self, dt_s: f64) -> Result<DiscreteModel, WipError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(WipError::InvalidParameter {
                name: "dt_s",
                value: dt_s,
            });
        }

        let continuous = self.continuous();
        let mut augmented = SMatrix::<f64, 6, 6>::zeros();
        for row in 0..4 {
            for column in 0..4 {
                augmented[(row, column)] = continuous.a[(row, column)];
            }
            augmented[(row, 4)] = continuous.b[(row, 0)];
        }

        let exponential = (augmented * dt_s).exp();
        let mut a = StateMatrix::zeros();
        let mut b = InputMatrix::zeros();
        for row in 0..4 {
            for column in 0..4 {
                a[(row, column)] = exponential[(row, column)];
            }
            b[(row, 0)] = exponential[(row, 4)];
        }

        for value in a.iter().chain(b.iter()) {
            if !value.is_finite() {
                return Err(WipError::NumericalFailure {
                    reason: "non-finite ZOH matrix element",
                });
            }
        }

        Ok(DiscreteModel { a, b, dt_s })
    }
}

const STATE_NAMES: [&str; 4] = ["x_m", "x_dot_mps", "pitch_rad", "pitch_rate_radps"];

/// Errors from the reduced-order WIP model.
#[derive(Debug, Clone, PartialEq)]
pub enum WipError {
    InvalidParameter { name: &'static str, value: f64 },
    NonFiniteInput { name: &'static str, value: f64 },
    DegenerateParameters { determinant: f64 },
    DegenerateConfiguration { pitch_rad: f64, determinant: f64 },
    NumericalFailure { reason: &'static str },
}

impl fmt::Display for WipError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter { name, value } => {
                write!(f, "invalid WIP parameter {name}={value}")
            }
            Self::NonFiniteInput { name, value } => {
                write!(f, "non-finite WIP input {name}={value}")
            }
            Self::DegenerateParameters { determinant } => {
                write!(f, "WIP mass matrix is singular (determinant {determinant})")
            }
            Self::DegenerateConfiguration {
                pitch_rad,
                determinant,
            } => write!(
                f,
                "WIP configuration is degenerate at pitch {pitch_rad} rad (determinant {determinant})"
            ),
            Self::NumericalFailure { reason } => {
                write!(f, "WIP numerical failure: {reason}")
            }
        }
    }
}

impl std::error::Error for WipError {}

fn positive(name: &'static str, value: f64) -> Result<(), WipError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(WipError::InvalidParameter { name, value })
    }
}

fn non_negative(name: &'static str, value: f64) -> Result<(), WipError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(WipError::InvalidParameter { name, value })
    }
}

fn rank_of(matrix: DMatrix<f64>) -> usize {
    let maximum = matrix
        .iter()
        .fold(0.0_f64, |current, value| current.max(value.abs()));
    if maximum == 0.0 {
        return 0;
    }

    let threshold = maximum * 1e-9;
    matrix
        .svd(false, false)
        .singular_values
        .iter()
        .filter(|value| **value > threshold)
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baseline_parameters() -> WipParameters {
        WipParameters {
            body_mass_kg: 6.0,
            wheel_mass_kg: 1.8,
            body_pitch_inertia_kg_m2: 0.35,
            wheel_inertia_kg_m2: 0.008,
            wheel_radius_m: 0.09,
            com_height_m: 0.25,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        }
    }

    fn baseline() -> WheeledInvertedPendulum {
        WheeledInvertedPendulum::new(baseline_parameters()).expect("baseline WIP must be valid")
    }

    #[test]
    fn upright_equilibrium_has_zero_derivative() {
        let plant = baseline();
        let derivative = plant
            .derivative(StateVector::zeros(), 0.0)
            .expect("upright state is valid");

        for value in derivative.iter() {
            assert!(value.abs() <= 1e-15);
        }
    }

    #[test]
    fn analytic_linearization_matches_numeric_linearization() {
        let plant = baseline();
        let analytic = plant.continuous();
        let numeric = plant
            .numeric_linearization(1e-6)
            .expect("numeric linearization succeeds");

        for row in 0..4 {
            for column in 0..4 {
                assert!(
                    (analytic.a[(row, column)] - numeric.a[(row, column)]).abs() <= 1e-5,
                    "A[{row},{column}] {} vs {}",
                    analytic.a[(row, column)],
                    numeric.a[(row, column)]
                );
            }
            assert!(
                (analytic.b[(row, 0)] - numeric.b[(row, 0)]).abs() <= 1e-5,
                "B[{row}] {} vs {}",
                analytic.b[(row, 0)],
                numeric.b[(row, 0)]
            );
        }
    }

    #[test]
    fn model_is_controllable_and_observable() {
        let model = baseline().continuous();
        assert_eq!(model.controllability_rank(), 4);
        assert!(model.is_controllable());

        let full_state = SMatrix::<f64, 4, 4>::identity();
        assert_eq!(model.observability_rank(&full_state), 4);
        assert!(model.is_observable(&full_state));

        let measured = SMatrix::<f64, 2, 4>::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert_eq!(model.observability_rank(&measured), 4);
    }

    #[test]
    fn zoh_matches_first_order_expansion_at_one_millisecond() {
        let plant = baseline();
        let continuous = plant.continuous();
        let discrete = plant.discrete(0.001).expect("valid step");

        let mut linear_a = StateMatrix::identity();
        linear_a += continuous.a * discrete.dt_s;
        let linear_b = continuous.b * discrete.dt_s;

        for row in 0..4 {
            for column in 0..4 {
                assert!((discrete.a[(row, column)] - linear_a[(row, column)]).abs() <= 1e-3);
            }
            assert!((discrete.b[(row, 0)] - linear_b[(row, 0)]).abs() <= 5e-6);
        }
    }

    #[test]
    fn discrete_model_stays_controllable() {
        let model = baseline().discrete(0.001).expect("valid step");
        assert_eq!(model.controllability_rank(), 4);
        assert!(model.is_controllable());
    }

    #[test]
    fn rejects_degenerate_parameters_and_inputs() {
        let mut parameters = baseline_parameters();
        parameters.wheel_radius_m = 0.0;
        assert!(matches!(
            WheeledInvertedPendulum::new(parameters),
            Err(WipError::InvalidParameter {
                name: "wheel_radius_m",
                ..
            })
        ));

        let plant = baseline();
        let result = plant.derivative(StateVector::new(f64::NAN, 0.0, 0.0, 0.0), 0.0);
        assert!(matches!(
            result,
            Err(WipError::NonFiniteInput { name: "x_m", .. })
        ));

        assert!(matches!(
            plant.discrete(0.0),
            Err(WipError::InvalidParameter { name: "dt_s", .. })
        ));
    }
}
