//! Balance control algorithms and trajectory generators.

pub mod lqr;
pub mod wip;

pub use lqr::{solve_discrete_lqr, LqrController, LqrError, LqrGain, LqrWeights};
pub use wip::{
    ContinuousModel, DiscreteModel, StateMatrix, StateVector, WheeledInvertedPendulum, WipError,
    WipParameters,
};

pub struct BalanceController {
    kp: f32,
    kd: f32,
}

impl BalanceController {
    /// Create a new balance controller with PID/PD gains.
    pub fn new(kp: f32, kd: f32) -> Self {
        Self { kp, kd }
    }

    /// Compute control torque/output based on pitch error and velocity.
    pub fn compute(&self, pitch_error: f32, pitch_rate: f32) -> f32 {
        self.kp * pitch_error + self.kd * pitch_rate
    }
}
