//! Pure FOC math. All hot-path arithmetic is integer with explicit Q formats;
//! the F103 has no FPU and soft float would break the 62.5 us budget.
//!
//! PI gains and hall-sector mapping are placeholders: the generated C has no
//! Simulink model in this repository, so each constant needs re-derivation and
//! bench validation against currents, duty and hall states before full torque.

use crate::fixed::clamp_i32;

pub const SIN_Q14: [i16; 64] = [
    0, 1606, 3196, 4756, 6270, 7723, 9102, 10394, 11585, 12665, 13623, 14449, 15137, 15679, 16069,
    16305, 16384, 16305, 16069, 15679, 15137, 14449, 13623, 12665, 11585, 10394, 9102, 7723, 6270,
    4756, 3196, 1606, 0, -1606, -3196, -4756, -6270, -7723, -9102, -10394, -11585, -12665, -13623,
    -14449, -15137, -15679, -16069, -16305, -16384, -16305, -16069, -15679, -15137, -14449, -13623,
    -12665, -11585, -10394, -9102, -7723, -6270, -4756, -3196, -1606,
];

/// Sine and cosine in Q14 from a 64-entry table with linear interpolation.
/// `angle` is a u16 covering one electrical revolution.
pub fn sin_cos_q14(angle: u16) -> (i32, i32) {
    let scaled = angle as u32 * 64;
    let index = (scaled >> 16) as usize;
    let fraction = (scaled & 0xFFFF) as i32;
    let a = SIN_Q14[index] as i32;
    let b = SIN_Q14[(index + 1) & 63] as i32;
    let interpolated = a + (((b - a) * fraction) >> 16);
    let cosine_index = (index + 16) & 63;
    let cosine_a = SIN_Q14[cosine_index] as i32;
    let cosine_b = SIN_Q14[(cosine_index + 1) & 63] as i32;
    let cosine = cosine_a + (((cosine_b - cosine_a) * fraction) >> 16);
    (interpolated, cosine)
}

/// Clarke transform. Inputs are ADC bit units scaled by `A2BIT_CONV`.
pub fn clarke(ia: i32, ib: i32, ic: i32) -> (i32, i32) {
    let alpha = (2 * ia - ib - ic) / 3;
    let beta = (ib - ic) * 927 / 1600; // 1/sqrt(3) in Q10
    (alpha, beta)
}

/// Park transform with sin/cos in Q14.
pub fn park(alpha: i32, beta: i32, sin: i32, cos: i32) -> (i32, i32) {
    let d = (alpha as i64 * cos as i64 + beta as i64 * sin as i64) >> 14;
    let q = (-alpha as i64 * sin as i64 + beta as i64 * cos as i64) >> 14;
    (d as i32, q as i32)
}

/// PI controller with integral clamping. `out_limit` is in the same unit as the
/// return value. No division and no float in the hot path.
pub fn pi_step(integral: &mut i32, error: i32, kp_q14: i32, ki_q14: i32, out_limit: i32) -> i32 {
    let added = (error as i64 * ki_q14 as i64) >> 14;
    *integral = clamp_i32(integral.saturating_add(added as i32), -out_limit, out_limit);
    let proportional = (error as i64 * kp_q14 as i64) >> 14;
    clamp_i32(proportional as i32 + *integral, -out_limit, out_limit)
}

/// Space-vector PWM duties for a phase vector in Q14. The reference caps duty
/// near 90 % to leave a current-sampling window; that cap is applied by the
/// caller so this function stays a pure transform.
pub fn svpwm(alpha: i32, beta: i32, arr: u16) -> (u16, u16, u16) {
    let half = arr as i32 / 2;
    let sqrt3_half = 14_189i64; // 0.8660254 in Q14
    let a = alpha as i64;
    let b = (-a / 2 + beta as i64 * sqrt3_half / 16_384) as i32;
    let c = (-a / 2 - beta as i64 * sqrt3_half / 16_384) as i32;
    let minimum = a.min(b as i64).min(c as i64) as i32;
    let maximum = a.max(b as i64).max(c as i64) as i32;
    let offset = -(minimum + maximum) / 2;
    let scale = half * 16_384 / 16_384;
    let duty = |phase: i32| -> u16 {
        let raw = half + scale * (phase + offset) / 16_384;
        clamp_i32(raw, 0, arr as i32) as u16
    };
    (duty(a as i32), duty(b), duty(c))
}

/// Provisional six-step hall sector. Must be verified on the bench against the
/// pinned C mapping and wheel direction before enabling torque.
pub fn hall_sector(u: bool, v: bool, w: bool) -> Option<u16> {
    let code = (u as u8) | ((v as u8) << 1) | ((w as u8) << 2);
    match code {
        1 => Some(0),
        3 => Some(1),
        2 => Some(2),
        6 => Some(3),
        4 => Some(4),
        5 => Some(5),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PWM_ARR;

    #[test]
    fn sin_cos_cardinal_points() {
        assert_eq!(sin_cos_q14(0).0, 0);
        let (s, c) = sin_cos_q14(16_384);
        assert!((s - 16_384).abs() <= 1);
        assert!(c.abs() <= 1);
    }

    #[test]
    fn clarke_balanced_currents_have_no_zero_sequence() {
        let (alpha, beta) = clarke(1000, -500, -500);
        assert_eq!(alpha, 1000);
        assert_eq!(beta, 0);
    }

    #[test]
    fn park_rotates_alpha_to_d() {
        let sin = SIN_Q14[16] as i32;
        let cos = SIN_Q14[32] as i32;
        let (d, q) = park(1000, 0, sin, cos);
        assert!(d.abs() <= 2);
        assert_eq!(q, -1000);
    }

    #[test]
    fn pi_clamps_output_and_integral() {
        let mut integral = 0;
        let out = pi_step(&mut integral, 1000, 16_384, 16_384, 100);
        assert_eq!(out, 100);
        assert_eq!(integral, 100);
        assert_eq!(pi_step(&mut integral, -1000, 16_384, 16_384, 100), -100);
    }

    #[test]
    fn svpwm_zero_vector_is_centered() {
        assert_eq!(svpwm(0, 0, PWM_ARR), (1000, 1000, 1000));
    }

    #[test]
    fn hall_sector_rejects_invalid_states() {
        assert_eq!(hall_sector(false, false, false), None);
        assert_eq!(hall_sector(true, true, true), None);
        assert_eq!(hall_sector(true, false, false), Some(0));
    }
}
