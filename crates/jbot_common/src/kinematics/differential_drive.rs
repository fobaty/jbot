//! Differential-drive kinematics following REP-103 sign conventions.
//!
//! The body frame has +x forward, +y left, and +z up. Positive yaw is a
//! counter-clockwise rotation, so the right wheel must be faster than the left
//! wheel for a positive angular velocity.

use super::{require_finite, require_positive, KinematicsError};

/// Linear speeds of the left and right wheels at the ground contact.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelSpeeds {
    pub left_mps: f64,
    pub right_mps: f64,
}

/// Planar body twist expressed in the body frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BodyTwist {
    pub linear_mps: f64,
    pub angular_radps: f64,
}

/// Differential drive with a fixed track width.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDrive {
    track_width_m: f64,
}

impl DifferentialDrive {
    /// Create a model with a strictly positive track width.
    pub fn new(track_width_m: f64) -> Result<Self, KinematicsError> {
        require_positive("track_width_m", track_width_m)?;
        Ok(Self { track_width_m })
    }

    pub fn track_width_m(&self) -> f64 {
        self.track_width_m
    }

    /// Map wheel speeds to body linear and angular velocity.
    pub fn forward(&self, wheels: WheelSpeeds) -> Result<BodyTwist, KinematicsError> {
        require_finite("left_mps", wheels.left_mps)?;
        require_finite("right_mps", wheels.right_mps)?;

        Ok(BodyTwist {
            linear_mps: 0.5 * (wheels.left_mps + wheels.right_mps),
            angular_radps: (wheels.right_mps - wheels.left_mps) / self.track_width_m,
        })
    }

    /// Map a body twist to wheel speeds.
    pub fn inverse(&self, twist: BodyTwist) -> Result<WheelSpeeds, KinematicsError> {
        require_finite("linear_mps", twist.linear_mps)?;
        require_finite("angular_radps", twist.angular_radps)?;

        let half_track = 0.5 * self.track_width_m;
        Ok(WheelSpeeds {
            left_mps: twist.linear_mps - twist.angular_radps * half_track,
            right_mps: twist.linear_mps + twist.angular_radps * half_track,
        })
    }
}

/// Convert a wheel angular speed to a linear contact speed.
pub fn wheel_angular_to_linear(
    wheel_radius_m: f64,
    angular_radps: f64,
) -> Result<f64, KinematicsError> {
    require_positive("wheel_radius_m", wheel_radius_m)?;
    require_finite("angular_radps", angular_radps)?;
    Ok(wheel_radius_m * angular_radps)
}

/// Convert a linear contact speed to a wheel angular speed.
pub fn wheel_linear_to_angular(
    wheel_radius_m: f64,
    linear_mps: f64,
) -> Result<f64, KinematicsError> {
    require_positive("wheel_radius_m", wheel_radius_m)?;
    require_finite("linear_mps", linear_mps)?;
    Ok(linear_mps / wheel_radius_m)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-12;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= 1e-9,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn zero_speed_produces_zero_twist() {
        let drive = DifferentialDrive::new(0.4).expect("valid track width");
        let twist = drive
            .forward(WheelSpeeds {
                left_mps: 0.0,
                right_mps: 0.0,
            })
            .expect("finite speeds");

        assert_close(twist.linear_mps, 0.0);
        assert_close(twist.angular_radps, 0.0);
    }

    #[test]
    fn in_place_rotation_has_zero_linear_speed() {
        let drive = DifferentialDrive::new(0.4).expect("valid track width");
        let twist = drive
            .forward(WheelSpeeds {
                left_mps: -1.0,
                right_mps: 1.0,
            })
            .expect("finite speeds");

        assert_close(twist.linear_mps, 0.0);
        assert_close(twist.angular_radps, 5.0);
    }

    #[test]
    fn forward_and_inverse_round_trip() {
        let drive = DifferentialDrive::new(0.42).expect("valid track width");
        let cases = [
            BodyTwist {
                linear_mps: 0.0,
                angular_radps: 0.0,
            },
            BodyTwist {
                linear_mps: 0.5,
                angular_radps: 0.0,
            },
            BodyTwist {
                linear_mps: 0.0,
                angular_radps: -1.2,
            },
            BodyTwist {
                linear_mps: -0.35,
                angular_radps: 0.8,
            },
        ];

        for twist in cases {
            let wheels = drive.inverse(twist).expect("finite twist");
            let recovered = drive.forward(wheels).expect("finite speeds");
            assert!((recovered.linear_mps - twist.linear_mps).abs() <= EPS);
            assert!((recovered.angular_radps - twist.angular_radps).abs() <= EPS);
        }
    }

    #[test]
    fn yaw_sign_follows_rep_103() {
        let drive = DifferentialDrive::new(0.4).expect("valid track width");
        let twist = drive
            .forward(WheelSpeeds {
                left_mps: 1.0,
                right_mps: 2.0,
            })
            .expect("finite speeds");

        assert!(twist.angular_radps > 0.0);
    }

    #[test]
    fn rejects_invalid_track_width() {
        assert!(matches!(
            DifferentialDrive::new(0.0),
            Err(KinematicsError::InvalidParameter {
                name: "track_width_m",
                ..
            })
        ));
        assert!(matches!(
            DifferentialDrive::new(f64::INFINITY),
            Err(KinematicsError::InvalidParameter {
                name: "track_width_m",
                ..
            })
        ));
    }

    #[test]
    fn rejects_non_finite_inputs() {
        let drive = DifferentialDrive::new(0.4).expect("valid track width");
        let result = drive.forward(WheelSpeeds {
            left_mps: f64::NAN,
            right_mps: 0.0,
        });

        assert!(matches!(
            result,
            Err(KinematicsError::NonFiniteInput {
                name: "left_mps",
                ..
            })
        ));
    }

    #[test]
    fn wheel_speed_conversions_round_trip() {
        let radius = 0.09;
        let angular = wheel_linear_to_angular(radius, 1.0).expect("valid input");
        let linear = wheel_angular_to_linear(radius, angular).expect("valid input");
        assert_close(linear, 1.0);
    }
}
