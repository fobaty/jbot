//! Differential torque mixer, ported from `Src/util.c:mixerFcn`.
//!
//! The C function takes fixdt(1,16,4) inputs; this wrapper converts command
//! units to Q4 first so callers pass the same units as the wire protocol. The
//! sign convention is part of the wheel-direction contract: right = speed minus
//! steer, left = speed plus steer. Do not "simplify" it.

use crate::config::{SPEED_COEFF_Q14, STEER_COEFF_Q14};
use crate::fixed::clamp_command;

pub fn mix(speed: i16, steer: i16, speed_coeff: i16, steer_coeff: i16) -> (i16, i16) {
    let speed_q4 = (speed as i32) << 4;
    let steer_q4 = (steer as i32) << 4;
    let prod_speed = (speed_q4 * speed_coeff as i32) >> 14;
    let prod_steer = (steer_q4 * steer_coeff as i32) >> 14;
    let right = clamp_command((prod_speed - prod_steer) >> 4);
    let left = clamp_command((prod_speed + prod_steer) >> 4);
    (left, right)
}

pub fn mix_default(speed: i16, steer: i16) -> (i16, i16) {
    mix(speed, steer, SPEED_COEFF_Q14, STEER_COEFF_Q14)
}

/// Inverse used by the host when turning wheel torques into wire channels.
pub fn inverse(left: i16, right: i16, speed_coeff: i16, steer_coeff: i16) -> (i16, i16) {
    let prod_speed = ((left as i32 + right as i32) << 3) as i64;
    let prod_steer = ((left as i32 - right as i32) << 3) as i64;
    let speed = prod_speed * 1024 / speed_coeff as i64;
    let steer = prod_steer * 1024 / steer_coeff as i64;
    (speed as i16, steer as i16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_request_drives_both_wheels_equal() {
        assert_eq!(mix_default(500, 0), (500, 500));
    }

    #[test]
    fn yaw_request_is_differential_with_unit_coefficient() {
        assert_eq!(mix(0, 500, SPEED_COEFF_Q14, 16_384), (500, -500));
    }

    #[test]
    fn reference_default_coefficient_halves_yaw() {
        assert_eq!(mix_default(0, 500), (250, -250));
    }

    #[test]
    fn output_is_clamped() {
        assert_eq!(mix(1000, 1000, SPEED_COEFF_Q14, 16_384), (1000, 0));
    }

    #[test]
    fn inverse_round_trips_with_unit_coefficients() {
        let (left, right) = mix_default(400, -120);
        assert_eq!(
            inverse(left, right, SPEED_COEFF_Q14, STEER_COEFF_Q14),
            (400, -120)
        );
    }
}
