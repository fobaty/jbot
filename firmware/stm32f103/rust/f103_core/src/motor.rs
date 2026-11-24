//! Per-motor FOC state machine, independent of hardware.
//!
//! The hall sector provides a coarse electrical angle; interpolating inside a
//! sector from the timer capture is still open, so the sector centre is used
//! provisionally. PI outputs are voltage commands in Q14 and are validated on
//! the bench before any gate drive is enabled.

use crate::config::{PWM_ARR, PWM_MARGIN};
use crate::foc::{clarke, hall_sector, park, pi_step, sin_cos_q14, svpwm};

const VOLTAGE_LIMIT_Q14: i32 = 12_000;
/// 65536 / 6, rounded down to keep sector 5 in range after the half offset.
const SECTOR_ANGLE: u16 = 10_922;

#[derive(Clone, Copy)]
pub struct Currents {
    pub ia: i32,
    pub ib: i32,
    pub ic: i32,
}

#[derive(Clone, Copy)]
pub struct PiGains {
    pub kp_q14: i32,
    pub ki_q14: i32,
}

pub struct MotorController {
    integral_d: i32,
    integral_q: i32,
    angle: u16,
    sector: Option<u16>,
    pub duties: (u16, u16, u16),
}

impl Default for MotorController {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorController {
    pub const fn new() -> Self {
        Self {
            integral_d: 0,
            integral_q: 0,
            angle: 0,
            sector: None,
            duties: (PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2),
        }
    }

    /// Electrical angle from the three hall inputs. Invalid states keep the
    /// previous angle rather than jumping the voltage vector.
    pub fn update_angle(&mut self, u: bool, v: bool, w: bool) {
        if let Some(sector) = hall_sector(u, v, w) {
            self.sector = Some(sector);
            self.angle = sector * SECTOR_ANGLE + SECTOR_ANGLE / 2;
        }
    }

    pub fn angle(&self) -> u16 {
        self.angle
    }

    pub fn reset(&mut self) {
        self.integral_d = 0;
        self.integral_q = 0;
        self.duties = (PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);
    }

    /// One current-loop step. Returns the three duty values for TIM1/TIM8.
    /// `target_d` is normally zero except during field weakening.
    pub fn step(
        &mut self,
        currents: Currents,
        targets: (i32, i32),
        gains: PiGains,
    ) -> (u16, u16, u16) {
        let (target_d, target_q) = targets;
        let (alpha, beta) = clarke(currents.ia, currents.ib, currents.ic);
        let (sin, cos) = sin_cos_q14(self.angle);
        let (measured_d, measured_q) = park(alpha, beta, sin, cos);
        let voltage_d = pi_step(
            &mut self.integral_d,
            target_d - measured_d,
            gains.kp_q14,
            gains.ki_q14,
            VOLTAGE_LIMIT_Q14,
        );
        let voltage_q = pi_step(
            &mut self.integral_q,
            target_q - measured_q,
            gains.kp_q14,
            gains.ki_q14,
            VOLTAGE_LIMIT_Q14,
        );
        let out_alpha = voltage_d as i64 * cos as i64 - voltage_q as i64 * sin as i64;
        let out_beta = voltage_d as i64 * sin as i64 + voltage_q as i64 * cos as i64;
        let raw = svpwm((out_alpha >> 14) as i32, (out_beta >> 14) as i32, PWM_ARR);
        // The reference reserves a sampling window at both ends of the duty
        // range; applying it here keeps every caller on the same contract.
        self.duties = (
            clamp_margin(raw.0),
            clamp_margin(raw.1),
            clamp_margin(raw.2),
        );
        self.duties
    }
}

fn clamp_margin(duty: u16) -> u16 {
    duty.clamp(PWM_MARGIN, PWM_ARR - PWM_MARGIN)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_halls_keep_previous_angle() {
        let mut motor = MotorController::new();
        motor.update_angle(true, false, false);
        let angle = motor.angle();
        motor.update_angle(false, false, false);
        assert_eq!(motor.angle(), angle);
    }

    #[test]
    fn zero_target_centers_duties() {
        let mut motor = MotorController::new();
        let duties = motor.step(
            Currents {
                ia: 0,
                ib: 0,
                ic: 0,
            },
            (0, 0),
            PiGains {
                kp_q14: 16_384,
                ki_q14: 16_384,
            },
        );
        assert_eq!(duties, (PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2));
    }

    #[test]
    fn nonzero_target_produces_asymmetric_duties() {
        let mut motor = MotorController::new();
        let duties = motor.step(
            Currents {
                ia: 0,
                ib: 0,
                ic: 0,
            },
            (0, 1_000),
            PiGains {
                kp_q14: 16_384,
                ki_q14: 16_384,
            },
        );
        assert_ne!(duties.0, duties.1);
        assert!(duties.0 <= PWM_ARR && duties.1 <= PWM_ARR);
    }

    #[test]
    fn duties_keep_the_sampling_margin() {
        let mut motor = MotorController::new();
        let duties = motor.step(
            Currents {
                ia: 0,
                ib: 0,
                ic: 0,
            },
            (0, 40_000),
            PiGains {
                kp_q14: 16_384,
                ki_q14: 16_384,
            },
        );
        for duty in [duties.0, duties.1, duties.2] {
            assert!((PWM_MARGIN..=PWM_ARR - PWM_MARGIN).contains(&duty));
        }
    }
}
