//! Discrete-time linear-quadratic regulator for the 1 kHz balance loop.
//!
//! The Riccati recursion is solved once at startup, not inside the control
//! deadline. The resulting static gain is then applied as `u = -K x`.
//!
//! The weights below are Phase 1 placeholders. They are intentionally not
//! presented as tuned: M1-17 records the trade-off between recovery time,
//! current, and oscillation before Gate G1.

use std::fmt;

use nalgebra::{SMatrix, SVector};

use crate::wip::{DiscreteModel, StateMatrix, StateVector};

/// Diagonal state weights `[x, x_dot, theta, theta_dot]` and scalar input weight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LqrWeights {
    pub position: f64,
    pub velocity: f64,
    pub pitch: f64,
    pub pitch_rate: f64,
    pub input: f64,
}

impl Default for LqrWeights {
    fn default() -> Self {
        Self {
            position: 10.0,
            velocity: 1.0,
            pitch: 100.0,
            pitch_rate: 10.0,
            input: 1.0,
        }
    }
}

impl LqrWeights {
    fn validate(&self) -> Result<(), LqrError> {
        positive("position", self.position)?;
        positive("velocity", self.velocity)?;
        positive("pitch", self.pitch)?;
        positive("pitch_rate", self.pitch_rate)?;
        positive("input", self.input)
    }

    fn state_weight_matrix(&self) -> StateMatrix {
        StateMatrix::from_diagonal(&SVector::<f64, 4>::new(
            self.position,
            self.velocity,
            self.pitch,
            self.pitch_rate,
        ))
    }
}

/// Result of the discrete Riccati solve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LqrGain {
    /// State-feedback gain for `u = -K x`.
    pub gain: SMatrix<f64, 1, 4>,
    /// Converged discrete algebraic Riccati solution.
    pub riccati: StateMatrix,
    /// Number of Riccati iterations performed.
    pub iterations: usize,
}

impl LqrGain {
    /// Compute the total wheel torque for a state.
    pub fn control(&self, state: StateVector) -> f64 {
        -(self.gain * state)[0]
    }

    /// Compute the torque for `u = -K (x - reference)`.
    pub fn control_with_reference(&self, state: StateVector, reference: StateVector) -> f64 {
        -(self.gain * (state - reference))[0]
    }
}

/// Solve the discrete algebraic Riccati equation and return the LQR gain.
///
/// The implementation uses the Joseph form of the Riccati recursion, which is
/// numerically better conditioned than the direct form:
///
/// ```text
/// K[k]   = (R + B^T P[k] B)^-1 B^T P[k] A
/// P[k+1] = (A - B K[k])^T P[k] (A - B K[k]) + K[k]^T R K[k] + Q
/// K      = (R + B^T P B)^-1 B^T P A
/// ```
///
/// The input is scalar, so the inverse is a division by a positive scalar.
pub fn solve_discrete_lqr(
    model: &DiscreteModel,
    weights: &LqrWeights,
) -> Result<LqrGain, LqrError> {
    weights.validate()?;

    const MAX_ITERATIONS: usize = 200_000;
    const TOLERANCE: f64 = 1e-12;

    let q = weights.state_weight_matrix();
    let r = weights.input;
    let mut p = q;
    let mut iterations = 0;
    let mut converged = false;

    for iteration in 0..MAX_ITERATIONS {
        let denominator = r + (model.b.transpose() * p * model.b)[(0, 0)];

        if !denominator.is_finite() || denominator <= 0.0 {
            return Err(LqrError::NumericalFailure {
                reason: "non-positive Riccati denominator",
            });
        }

        let gain = (model.b.transpose() * p * model.a) / denominator;
        let closed_loop = model.a - model.b * gain;
        let mut p_next =
            closed_loop.transpose() * p * closed_loop + gain.transpose() * gain * r + q;

        // The Joseph form is symmetric in exact arithmetic; average with the
        // transpose to suppress round-off asymmetry before the next iteration.
        p_next = 0.5 * (p_next + p_next.transpose());

        let difference = max_abs_difference(&p_next, &p);
        p = p_next;
        iterations = iteration + 1;

        if difference < TOLERANCE {
            converged = true;
            break;
        }
    }

    if !converged {
        return Err(LqrError::NotConverged {
            iterations: MAX_ITERATIONS,
        });
    }

    let denominator = r + (model.b.transpose() * p * model.b)[(0, 0)];
    if !denominator.is_finite() || denominator <= 0.0 {
        return Err(LqrError::NumericalFailure {
            reason: "non-positive final Riccati denominator",
        });
    }

    let gain = (model.b.transpose() * p * model.a) / denominator;
    if gain.iter().any(|value| !value.is_finite()) || p.iter().any(|value| !value.is_finite()) {
        return Err(LqrError::NumericalFailure {
            reason: "non-finite LQR result",
        });
    }

    Ok(LqrGain {
        gain,
        riccati: p,
        iterations,
    })
}

/// State-feedback controller holding a precomputed gain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LqrController {
    gain: LqrGain,
}

impl LqrController {
    pub fn new(gain: LqrGain) -> Self {
        Self { gain }
    }

    pub fn gain(&self) -> &LqrGain {
        &self.gain
    }

    /// Total wheel torque for `u = -K x`.
    pub fn control(&self, state: StateVector) -> f64 {
        self.gain.control(state)
    }

    /// Total wheel torque for `u = -K (x - reference)`.
    pub fn control_with_reference(&self, state: StateVector, reference: StateVector) -> f64 {
        self.gain.control_with_reference(state, reference)
    }
}

/// Errors from weight validation and the Riccati solve.
#[derive(Debug, Clone, PartialEq)]
pub enum LqrError {
    InvalidWeight { name: &'static str, value: f64 },
    NotConverged { iterations: usize },
    NumericalFailure { reason: &'static str },
}

impl fmt::Display for LqrError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidWeight { name, value } => {
                write!(f, "invalid LQR weight {name}={value}")
            }
            Self::NotConverged { iterations } => {
                write!(
                    f,
                    "LQR Riccati recursion did not converge in {iterations} iterations"
                )
            }
            Self::NumericalFailure { reason } => {
                write!(f, "LQR numerical failure: {reason}")
            }
        }
    }
}

impl std::error::Error for LqrError {}

fn positive(name: &'static str, value: f64) -> Result<(), LqrError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(LqrError::InvalidWeight { name, value })
    }
}

fn max_abs_difference(left: &StateMatrix, right: &StateMatrix) -> f64 {
    left.iter()
        .zip(right.iter())
        .fold(0.0_f64, |current, (a, b)| current.max((a - b).abs()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wip::{WheeledInvertedPendulum, WipParameters, STANDARD_GRAVITY_MPS2};
    use nalgebra::DMatrix;

    fn baseline_model() -> DiscreteModel {
        let plant = WheeledInvertedPendulum::new(WipParameters {
            body_mass_kg: 6.0,
            wheel_mass_kg: 1.8,
            body_pitch_inertia_kg_m2: 0.35,
            wheel_inertia_kg_m2: 0.008,
            wheel_radius_m: 0.09,
            com_height_m: 0.25,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        })
        .expect("baseline WIP must be valid");
        plant.discrete(0.001).expect("valid step")
    }

    #[test]
    fn riccati_gain_matches_a_reference_solution() {
        let model = baseline_model();
        let solution =
            solve_discrete_lqr(&model, &LqrWeights::default()).expect("LQR must converge");

        // Reference from an independent Schur-based DARE solve with the same
        // weights: Q = diag(10, 1, 100, 10), R = 1.
        let expected = [-3.121_027_59, -4.557_177_02, -25.439_825_97, -5.919_100_71];
        for (index, value) in expected.iter().enumerate() {
            assert!(
                (solution.gain[(0, index)] - value).abs() <= 1e-3,
                "K[{index}]={} expected {value}",
                solution.gain[(0, index)]
            );
        }
        assert!(solution.iterations > 0);
    }

    #[test]
    fn closed_loop_is_asymptotically_stable() {
        let model = baseline_model();
        let solution =
            solve_discrete_lqr(&model, &LqrWeights::default()).expect("LQR must converge");
        let closed_loop =
            DMatrix::from_row_slice(4, 4, model.closed_loop_matrix(&solution.gain).as_slice());
        let eigenvalues = closed_loop.complex_eigenvalues();

        assert_eq!(eigenvalues.len(), 4);
        for eigenvalue in eigenvalues.iter() {
            assert!(
                eigenvalue.norm() < 1.0,
                "unstable closed-loop eigenvalue {eigenvalue}"
            );
        }
    }

    #[test]
    fn controller_produces_finite_torque() {
        let model = baseline_model();
        let solution =
            solve_discrete_lqr(&model, &LqrWeights::default()).expect("LQR must converge");
        let controller = LqrController::new(solution);
        let torque = controller.control(StateVector::new(0.1, 0.0, 0.02, 0.0));

        assert!(torque.is_finite());
        assert!(torque > 0.0);
    }

    #[test]
    fn rejects_invalid_weights() {
        let model = baseline_model();
        let weights = LqrWeights {
            input: 0.0,
            ..LqrWeights::default()
        };
        assert!(matches!(
            solve_discrete_lqr(&model, &weights),
            Err(LqrError::InvalidWeight { name: "input", .. })
        ));
    }
}
