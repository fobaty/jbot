//! Classical fourth-order Runge-Kutta step.

use jbot_control::StateVector;

use crate::SimError;

/// Advance a state by one RK4 step with a constant input.
///
/// The derivative closure receives the intermediate state and the same input
/// as the step. All intermediate values must be finite and the step size must
/// be positive.
pub fn rk4_step<F>(
    state: StateVector,
    input: f64,
    dt_s: f64,
    derivative: F,
) -> Result<StateVector, SimError>
where
    F: Fn(StateVector, f64) -> Result<StateVector, SimError>,
{
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(SimError::InvalidConfig {
            field: "dt_s",
            value: dt_s,
        });
    }
    if !input.is_finite() {
        return Err(SimError::InvalidConfig {
            field: "input",
            value: input,
        });
    }

    let k1 = derivative(state, input)?;
    let k2 = derivative(state + 0.5 * dt_s * k1, input)?;
    let k3 = derivative(state + 0.5 * dt_s * k2, input)?;
    let k4 = derivative(state + dt_s * k3, input)?;

    let next = state + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    if next.iter().any(|value| !value.is_finite()) {
        return Err(SimError::NonFiniteIntegration);
    }
    Ok(next)
}
