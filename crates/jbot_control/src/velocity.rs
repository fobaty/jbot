//! Cascaded velocity loop around the balancing LQR.

use std::cell::Cell;
use std::fmt;

use crate::lqr::LqrController;
use crate::wip::StateVector;

/// Proportional and integral gains for the outer velocity loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityGains {
    /// Pitch reference per velocity error, rad/(m/s).
    pub kp: f64,
    /// Pitch reference per integral of velocity error, rad/m.
    pub ki: f64,
    /// Absolute pitch reference limit, radians.
    pub max_pitch_reference_rad: f64,
    /// Absolute integrator limit, meters.
    pub max_integral_error_m: f64,
}

impl Default for VelocityGains {
    fn default() -> Self {
        Self {
            kp: 0.15,
            ki: 0.03,
            max_pitch_reference_rad: 10.0_f64.to_radians(),
            max_integral_error_m: 1.0,
        }
    }
}

impl VelocityGains {
    fn validate(&self) -> Result<(), VelocityError> {
        non_negative("kp", self.kp)?;
        non_negative("ki", self.ki)?;
        positive("max_pitch_reference_rad", self.max_pitch_reference_rad)?;
        if self.max_pitch_reference_rad >= std::f64::consts::FRAC_PI_2 {
            return Err(VelocityError::InvalidParameter {
                name: "max_pitch_reference_rad",
                value: self.max_pitch_reference_rad,
            });
        }
        non_negative("max_integral_error_m", self.max_integral_error_m)?;
        Ok(())
    }
}

/// Velocity loop that generates a pitch reference for the inner LQR.
#[derive(Debug, Clone)]
pub struct VelocityController {
    inner: LqrController,
    target_velocity_mps: f64,
    gains: VelocityGains,
    max_torque_nm: f64,
    dt_s: f64,
    integral_error_m: Cell<f64>,
    fallback_time_s: Cell<f64>,
    last_time_s: Cell<f64>,
}

impl VelocityController {
    pub fn new(
        inner: LqrController,
        target_velocity_mps: f64,
        gains: VelocityGains,
        max_torque_nm: f64,
        dt_s: f64,
    ) -> Result<Self, VelocityError> {
        if !target_velocity_mps.is_finite() {
            return Err(VelocityError::InvalidParameter {
                name: "target_velocity_mps",
                value: target_velocity_mps,
            });
        }
        gains.validate()?;
        positive("max_torque_nm", max_torque_nm)?;
        positive("dt_s", dt_s)?;
        Ok(Self {
            inner,
            target_velocity_mps,
            gains,
            max_torque_nm,
            dt_s,
            integral_error_m: Cell::new(0.0),
            fallback_time_s: Cell::new(0.0),
            last_time_s: Cell::new(f64::NAN),
        })
    }

    pub fn target_velocity_mps(&self) -> f64 {
        self.target_velocity_mps
    }

    pub fn integral_error_m(&self) -> f64 {
        self.integral_error_m.get()
    }

    pub fn reset(&self) {
        self.integral_error_m.set(0.0);
        self.fallback_time_s.set(0.0);
        self.last_time_s.set(f64::NAN);
    }

    /// Compute torque with a fixed step, for callers that do not provide time.
    pub fn control(&self, state: StateVector) -> f64 {
        let time_s = self.fallback_time_s.get();
        let torque = self.control_at(time_s, state);
        self.fallback_time_s.set(time_s + self.dt_s);
        torque
    }

    /// Compute torque at an absolute simulation time.
    pub fn control_at(&self, time_s: f64, state: StateVector) -> f64 {
        let last_time = self.last_time_s.get();
        let dt_s = if last_time.is_nan() {
            self.dt_s
        } else {
            (time_s - last_time).max(0.0)
        };
        self.last_time_s.set(time_s);

        let velocity_error = self.target_velocity_mps - state[1];
        let integral = self.integral_error_m.get();
        let candidate_integral = (integral + velocity_error * dt_s).clamp(
            -self.gains.max_integral_error_m,
            self.gains.max_integral_error_m,
        );
        let candidate_pitch = (self.gains.kp * velocity_error + self.gains.ki * candidate_integral)
            .clamp(
                -self.gains.max_pitch_reference_rad,
                self.gains.max_pitch_reference_rad,
            );
        let reference = StateVector::new(state[0], self.target_velocity_mps, candidate_pitch, 0.0);
        let torque = self
            .inner
            .control_with_reference(state, reference)
            .clamp(-self.max_torque_nm, self.max_torque_nm);

        let output_saturated = torque.abs() >= self.max_torque_nm;
        let reference_saturated = candidate_pitch.abs() >= self.gains.max_pitch_reference_rad;
        let pushing_further = velocity_error * candidate_pitch > 0.0;
        if !((output_saturated || reference_saturated) && pushing_further) {
            self.integral_error_m.set(candidate_integral);
        }

        torque
    }
}

/// Errors from velocity-loop configuration.
#[derive(Debug, Clone, PartialEq)]
pub enum VelocityError {
    InvalidParameter { name: &'static str, value: f64 },
}

impl fmt::Display for VelocityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter { name, value } => {
                write!(f, "invalid velocity-loop parameter {name}={value}")
            }
        }
    }
}

impl std::error::Error for VelocityError {}

fn positive(name: &'static str, value: f64) -> Result<(), VelocityError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(VelocityError::InvalidParameter { name, value })
    }
}

fn non_negative(name: &'static str, value: f64) -> Result<(), VelocityError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(VelocityError::InvalidParameter { name, value })
    }
}

#[cfg(test)]
mod tests {
    use crate::wip::{WheeledInvertedPendulum, WipParameters, STANDARD_GRAVITY_MPS2};

    use crate::lqr::{solve_discrete_lqr, LqrWeights};

    use super::*;

    fn controller(target_velocity_mps: f64, max_torque_nm: f64) -> VelocityController {
        let plant = WheeledInvertedPendulum::new(WipParameters {
            body_mass_kg: 6.0,
            wheel_mass_kg: 1.8,
            body_pitch_inertia_kg_m2: 0.35,
            wheel_inertia_kg_m2: 0.008,
            wheel_radius_m: 0.09,
            com_height_m: 0.25,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        })
        .expect("valid plant");
        let discrete = plant.discrete(0.001).expect("valid ZOH step");
        let gain = solve_discrete_lqr(&discrete, &LqrWeights::default()).expect("LQR converges");
        VelocityController::new(
            LqrController::new(gain),
            target_velocity_mps,
            VelocityGains::default(),
            max_torque_nm,
            0.001,
        )
        .expect("valid velocity controller")
    }

    #[test]
    fn zero_reference_at_rest_produces_zero_torque() {
        let controller = controller(0.0, 16.0);
        assert_eq!(controller.control_at(0.0, StateVector::zeros()), 0.0);
    }

    #[test]
    fn positive_velocity_error_requests_forward_lean() {
        let controller = controller(0.5, 16.0);
        assert!(controller.control_at(0.0, StateVector::zeros()) < 0.0);
    }

    #[test]
    fn integral_is_frozen_by_reference_saturation() {
        let inner = controller(0.0, 16.0).inner;
        let controller = VelocityController::new(
            inner,
            1.0,
            VelocityGains {
                kp: 1.0,
                ki: 0.1,
                max_pitch_reference_rad: 0.1,
                max_integral_error_m: 1.0,
            },
            16.0,
            0.001,
        )
        .expect("valid velocity controller");
        for _ in 0..100 {
            let _ = controller.control(StateVector::zeros());
        }
        assert_eq!(controller.integral_error_m(), 0.0);
    }

    #[test]
    fn integral_stays_bounded_under_output_saturation() {
        let controller = controller(1.0, 0.5);
        for _ in 0..10_000 {
            let _ = controller.control(StateVector::zeros());
        }
        assert!(controller.integral_error_m().abs() <= 1.0 + 1e-12);
        assert!(controller.integral_error_m().is_finite());
    }

    #[test]
    fn reset_clears_the_integrator() {
        let controller = controller(0.5, 16.0);
        for _ in 0..100 {
            let _ = controller.control(StateVector::zeros());
        }
        controller.reset();
        assert_eq!(controller.integral_error_m(), 0.0);
    }

    #[test]
    fn rejects_invalid_parameters() {
        let inner = controller(0.0, 16.0).inner;
        assert!(matches!(
            VelocityController::new(inner, 0.0, VelocityGains::default(), 0.0, 0.001),
            Err(VelocityError::InvalidParameter {
                name: "max_torque_nm",
                ..
            })
        ));
        assert!(matches!(
            VelocityController::new(inner, 0.0, VelocityGains::default(), 16.0, 0.0),
            Err(VelocityError::InvalidParameter { name: "dt_s", .. })
        ));
    }
}
