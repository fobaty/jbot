//! Kinematic models shared by estimation, control, simulation, and firmware.

pub mod differential_drive;
pub mod four_bar;

pub use differential_drive::{BodyTwist, DifferentialDrive, WheelSpeeds};
pub use four_bar::{AssemblyBranch, LegGeometry, LegKinematics, LegPose};

use std::fmt;

/// Errors produced by kinematic models.
///
/// The variants carry the offending value so callers can log a precise reason
/// without re-deriving it from the input.
#[derive(Debug, Clone, PartialEq)]
pub enum KinematicsError {
    /// A model parameter is not finite or violates its domain.
    InvalidParameter { name: &'static str, value: f64 },
    /// A runtime input is not finite.
    NonFiniteInput { name: &'static str, value: f64 },
    /// The requested joint angle is outside the configured travel window.
    JointLimit {
        angle_rad: f64,
        min_rad: f64,
        max_rad: f64,
    },
    /// The configured joint window is empty or inverted.
    InvalidJointRange { min_rad: f64, max_rad: f64 },
    /// The four-bar linkage cannot be assembled at this hip angle.
    NoSolution { angle_rad: f64 },
    /// The target point is not on the wheel-axle coupler curve.
    OutsideWorkspace { distance_m: f64 },
    /// The requested sample count is too small to represent the curve.
    InvalidSampleCount { samples: usize },
    /// The Jacobian cannot be evaluated at this configuration.
    Degenerate { angle_rad: f64 },
}

impl fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter { name, value } => {
                write!(f, "invalid kinematic parameter {name}={value}")
            }
            Self::NonFiniteInput { name, value } => {
                write!(f, "non-finite kinematic input {name}={value}")
            }
            Self::JointLimit {
                angle_rad,
                min_rad,
                max_rad,
            } => write!(
                f,
                "hip angle {angle_rad} rad is outside [{min_rad}, {max_rad}] rad"
            ),
            Self::InvalidJointRange { min_rad, max_rad } => {
                write!(f, "invalid hip joint range [{min_rad}, {max_rad}] rad")
            }
            Self::NoSolution { angle_rad } => {
                write!(f, "four-bar linkage has no assembly at {angle_rad} rad")
            }
            Self::OutsideWorkspace { distance_m } => {
                write!(f, "target is {distance_m} m away from the coupler curve")
            }
            Self::InvalidSampleCount { samples } => {
                write!(f, "at least two samples are required, got {samples}")
            }
            Self::Degenerate { angle_rad } => {
                write!(f, "kinematic Jacobian is degenerate at {angle_rad} rad")
            }
        }
    }
}

impl std::error::Error for KinematicsError {}

pub(crate) fn require_finite(name: &'static str, value: f64) -> Result<(), KinematicsError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(KinematicsError::NonFiniteInput { name, value })
    }
}

pub(crate) fn require_positive(name: &'static str, value: f64) -> Result<(), KinematicsError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(KinematicsError::InvalidParameter { name, value })
    }
}

pub(crate) fn vector_norm(v: [f64; 2]) -> f64 {
    v[0].hypot(v[1])
}
