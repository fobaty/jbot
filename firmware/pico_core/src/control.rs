//! Cascade balance law for the stage-1 wheeled biped.
//!
//! Structure (one 200 Hz step):
//!
//! 1. the outer speed loop turns a speed error into a pitch reference;
//! 2. the inner pitch loop is a PD on the pitch error and the pitch rate and
//!    produces the common-mode wheel torque;
//! 3. the yaw loop produces the differential wheel torque from the yaw-rate
//!    error.
//!
//! Sign convention: positive pitch means the body leans forward in the
//! direction of travel, and positive wheel torque drives the wheels forward.
//! Leaning forward therefore demands positive common torque: the wheels move
//! under the centre of mass. Positive yaw rate means a counter-clockwise turn
//! seen from above, which is produced by a positive right / negative left
//! differential.
//!
//! The gains are intentionally zero in [`Gains::SAFE`]. Any non-zero gain is a
//! tuning result and must be measured on the safety rig before it is enabled;
//! the firmware keeps the robot in `IDLE` (zero torque) until then.

use crate::eferu::{Command, TORQUE_LIMIT};

/// Torque limit expressed as a float, in EFeru command units.
pub const TORQUE_LIMIT_F32: f32 = TORQUE_LIMIT as f32;

/// Balance gains. Units are command units per SI input unit.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Gains {
    /// Inner pitch loop, proportional gain, command units per radian.
    pub pitch_kp: f32,
    /// Inner pitch loop, derivative gain, command units per radian per second.
    pub pitch_kd: f32,
    /// Outer speed loop, pitch reference per metre per second of speed error.
    pub speed_kp: f32,
    /// Outer speed loop, maximum pitch reference in radians.
    pub max_pitch_ref_rad: f32,
    /// Yaw loop, command units per radian per second of yaw-rate error.
    pub yaw_kp: f32,
}

impl Gains {
    /// All-zero gains: the controller is inert and the robot must stay in
    /// `IDLE`. Non-zero values come only from a bench tuning session.
    pub const SAFE: Self = Self {
        pitch_kp: 0.0,
        pitch_kd: 0.0,
        speed_kp: 0.0,
        max_pitch_ref_rad: 0.0,
        yaw_kp: 0.0,
    };

    /// True when at least one loop is active.
    pub fn is_active(self) -> bool {
        self.pitch_kp != 0.0 || self.pitch_kd != 0.0 || self.speed_kp != 0.0 || self.yaw_kp != 0.0
    }
}

/// Inputs consumed by one balance step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BalanceInput {
    /// Pitch angle, radians, positive when leaning forward.
    pub pitch_rad: f32,
    /// Pitch rate, radians per second, positive when falling forward.
    pub pitch_rate_rad_s: f32,
    /// Requested speed minus measured speed, metres per second.
    pub speed_error_mps: f32,
    /// Requested yaw rate minus measured yaw rate, radians per second.
    pub yaw_rate_error_rad_s: f32,
}

/// Wheel torque pair in EFeru command units, before rounding.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelTorque {
    /// Left wheel torque request.
    pub left: f32,
    /// Right wheel torque request.
    pub right: f32,
}

impl WheelTorque {
    /// Neutral request.
    pub const ZERO: Self = Self {
        left: 0.0,
        right: 0.0,
    };
}

/// Run one step of the cascade.
///
/// The common-mode (balance) torque is computed and clamped first. The yaw
/// differential can only use the headroom that remains, so a yaw request can
/// never take torque away from balance. The result is safe even if every input
/// is saturated or non-finite.
pub fn balance(input: &BalanceInput, gains: &Gains) -> WheelTorque {
    if !input.pitch_rad.is_finite()
        || !input.pitch_rate_rad_s.is_finite()
        || !input.speed_error_mps.is_finite()
        || !input.yaw_rate_error_rad_s.is_finite()
    {
        return WheelTorque::ZERO;
    }

    if !gains.is_active() {
        return WheelTorque::ZERO;
    }

    let pitch_ref = clamp(
        gains.speed_kp * input.speed_error_mps,
        gains.max_pitch_ref_rad.abs(),
    );
    let pitch_error = input.pitch_rad - pitch_ref;

    let common = gains.pitch_kp * pitch_error + gains.pitch_kd * input.pitch_rate_rad_s;
    let common = clamp(common, TORQUE_LIMIT_F32);

    // Balance has priority: reserve the common-mode torque, then let yaw use
    // only the remaining headroom on both wheels.
    let headroom = (TORQUE_LIMIT_F32 - common.abs()).max(0.0);
    let differential = clamp(gains.yaw_kp * input.yaw_rate_error_rad_s, headroom);

    WheelTorque {
        left: clamp(common - differential, TORQUE_LIMIT_F32),
        right: clamp(common + differential, TORQUE_LIMIT_F32),
    }
}

/// Convert a wheel torque pair into the EFeru steer/speed command.
pub fn to_command(torque: WheelTorque) -> Command {
    let left = torque_to_i16(torque.left);
    let right = torque_to_i16(torque.right);
    Command::from_wheel_torque(left, right)
}

fn torque_to_i16(value: f32) -> i16 {
    if !value.is_finite() {
        return 0;
    }
    let clamped = value.clamp(-TORQUE_LIMIT_F32, TORQUE_LIMIT_F32);
    // `f32::round` lives in `std`, so round half away from zero manually.
    let rounded = if clamped >= 0.0 {
        clamped + 0.5
    } else {
        clamped - 0.5
    };
    rounded as i16
}

fn clamp(value: f32, limit: f32) -> f32 {
    value.clamp(-limit, limit)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gains() -> Gains {
        Gains {
            pitch_kp: 2000.0,
            pitch_kd: 80.0,
            speed_kp: 0.5,
            max_pitch_ref_rad: 0.15,
            yaw_kp: 300.0,
        }
    }

    #[test]
    fn safe_gains_produce_no_torque() {
        let input = BalanceInput {
            pitch_rad: 0.5,
            pitch_rate_rad_s: 1.0,
            speed_error_mps: 1.0,
            yaw_rate_error_rad_s: 1.0,
        };
        assert_eq!(balance(&input, &Gains::SAFE), WheelTorque::ZERO);
    }

    #[test]
    fn forward_lean_produces_forward_torque() {
        let input = BalanceInput {
            pitch_rad: 0.1,
            ..BalanceInput::default()
        };
        let output = balance(&input, &gains());
        assert!(output.left > 0.0);
        assert!((output.left - output.right).abs() < 1e-3);
    }

    #[test]
    fn speed_error_tips_the_body_towards_the_reference() {
        let mut g = gains();
        g.pitch_kp = 1000.0;
        // A forward speed request needs a forward lean reference. From an
        // upright stance the controller first drives the wheels backwards,
        // which is what tips the body forward.
        let input = BalanceInput {
            speed_error_mps: 0.5,
            ..BalanceInput::default()
        };
        let output = balance(&input, &g);
        assert!(output.left < 0.0);
        assert!((output.left - output.right).abs() < 1e-3);

        // Once the body reaches the reference, the inner loop is neutral.
        let tracked = BalanceInput {
            pitch_rad: g.max_pitch_ref_rad,
            speed_error_mps: 0.5,
            ..BalanceInput::default()
        };
        let output = balance(&tracked, &g);
        assert!(output.left.abs() < 1e-3);
    }

    #[test]
    fn yaw_error_is_differential() {
        let input = BalanceInput {
            yaw_rate_error_rad_s: 1.0,
            ..BalanceInput::default()
        };
        let output = balance(&input, &gains());
        assert!(output.left < 0.0);
        assert!(output.right > 0.0);
        assert!((output.left + output.right).abs() < 1e-3);
    }

    #[test]
    fn balance_keeps_priority_over_yaw() {
        let input = BalanceInput {
            pitch_rad: 10.0,
            yaw_rate_error_rad_s: 10.0,
            ..BalanceInput::default()
        };
        let output = balance(&input, &gains());
        assert!(output.right <= TORQUE_LIMIT_F32);
        assert!(output.left >= -TORQUE_LIMIT_F32);
        assert!(output.left >= TORQUE_LIMIT_F32 - 1.0);
        assert!(output.right >= TORQUE_LIMIT_F32 - 1.0);
    }

    #[test]
    fn non_finite_input_produces_zero_torque() {
        let input = BalanceInput {
            pitch_rad: f32::NAN,
            ..BalanceInput::default()
        };
        assert_eq!(balance(&input, &gains()), WheelTorque::ZERO);
    }

    #[test]
    fn command_round_trip_keeps_wheel_direction() {
        let command = to_command(WheelTorque {
            left: 120.4,
            right: -45.6,
        });
        assert_eq!(command.speed, 37);
        assert_eq!(command.steer, 83);
    }

    #[test]
    fn to_command_clamps_non_finite_and_out_of_range() {
        assert_eq!(
            to_command(WheelTorque {
                left: f32::INFINITY,
                right: -5000.0
            }),
            Command::from_wheel_torque(0, -1000)
        );
    }
}
