//! Saturating fixed-point primitives used by the control path.
//!
//! The F103 has no FPU, so all hot-path arithmetic uses integers with an
//! explicit Q format. Intermediate products are widened to i64 before shifting
//! to avoid overflow.

use crate::config::COMMAND_LIMIT;

pub const fn clamp_i32(value: i32, low: i32, high: i32) -> i32 {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}

pub const fn clamp_i16(value: i32, low: i16, high: i16) -> i16 {
    clamp_i32(value, low as i32, high as i32) as i16
}

pub fn clamp_command(value: i32) -> i16 {
    clamp_i16(value, -COMMAND_LIMIT, COMMAND_LIMIT)
}

/// First-order low pass in fixdt(0,16,16). `alpha` is in 0..=65535.
pub fn low_pass_q16(state: &mut i32, input: i32, alpha: u16) -> i32 {
    let delta = (input as i64 - *state as i64) * alpha as i64;
    *state += (delta >> 16) as i32;
    *state
}

/// Bit-exact port of the C `filtLowPass32`: `u` is a raw integer, `y` is
/// fixdt(1,32,16) and `coef` is fixdt(0,16,16). The 12-bit feedback shift and
/// 4-bit input shift are part of the reference behaviour and must not be
/// replaced by the generic `low_pass_q16`.
pub fn filt_low_pass32(u: i32, coef: u16, y: &mut i32) -> i32 {
    let tmp = ((i64::from(u) << 4) - i64::from(*y >> 12)) * i64::from(coef);
    let tmp = (tmp >> 4).clamp(i32::MIN as i64, i32::MAX as i64) as i32;
    *y = tmp.wrapping_add(*y);
    *y
}

/// Rate limiter in fixdt(1,16,4). One tick advances at most `rate >> 4`.
pub fn rate_limit(current: i16, target: i16, rate_q4: i16) -> i16 {
    let step = (rate_q4 as i32) >> 4;
    let delta = target as i32 - current as i32;
    if delta > step {
        clamp_command(current as i32 + step)
    } else if delta < -step {
        clamp_command(current as i32 - step)
    } else {
        target
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn low_pass_converges_and_clamps_alpha() {
        let mut state = 0;
        for _ in 0..200 {
            low_pass_q16(&mut state, 1000, 19661);
        }
        // Truncating shifts stop converging a few counts short of the target;
        // the C reference has the same behaviour.
        assert!((state - 1000).abs() < 10, "state was {state}");
        let mut frozen = 500;
        assert_eq!(low_pass_q16(&mut frozen, 1000, 0), 500);
    }

    #[test]
    fn filt_low_pass32_matches_c_reference_steps() {
        let mut y = 0;
        assert_eq!(filt_low_pass32(1_000, 655, &mut y), 655_000);
        assert_eq!(filt_low_pass32(1_000, 655, &mut y), 1_303_490);
        for _ in 0..2_000 {
            filt_low_pass32(1_000, 655, &mut y);
        }
        assert_eq!(y >> 16, 1_000);
    }

    #[test]
    fn rate_limit_matches_reference_step() {
        assert_eq!(rate_limit(0, 1000, 480), 30);
        assert_eq!(rate_limit(0, 10, 480), 10);
        assert_eq!(rate_limit(1000, -1000, 480), 970);
    }

    #[test]
    fn clamp_command_is_symmetric() {
        assert_eq!(clamp_command(5000), 1000);
        assert_eq!(clamp_command(-5000), -1000);
    }
}
