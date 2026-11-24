//! Motor fault detection and debounce, ported from the generated controller.
//!
//! The reference sets three fault bits: illegal hall patterns, and a stalled
//! rotor under a large torque request. Bits are qualified for 1280 FOC ticks and
//! cleared only after 9600 clean ticks, then latched across mode changes.

use crate::config::{
    DIAG_DEQUALIFY_TICKS, DIAG_ENABLE, DIAG_INPUT_TARGET_THRESHOLD, DIAG_QUALIFY_TICKS,
    DIAG_STILL_SPEED,
};

pub const ERR_HALL_ZERO: u8 = 1;
pub const ERR_HALL_SEVEN: u8 = 2;
pub const ERR_STALLED: u8 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticInput {
    /// Hall code in the generated bit order: A << 2 | B << 1 | C.
    pub hall_sum: u8,
    pub motor_enabled: bool,
    /// Electrical speed in Q0 rpm (`n_mot`).
    pub speed: i32,
    /// Applied torque target in Q4.
    pub target: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorDiagnostics {
    qualify: u32,
    dequalify: u32,
    pub error_code: u8,
}

impl Default for MotorDiagnostics {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDiagnostics {
    pub const fn new() -> Self {
        Self {
            qualify: 0,
            dequalify: 0,
            error_code: 0,
        }
    }

    fn raw_error(input: DiagnosticInput) -> u8 {
        if !DIAG_ENABLE {
            return 0;
        }
        let mut code = 0;
        if input.hall_sum == 0 {
            code |= ERR_HALL_ZERO;
        }
        if input.hall_sum == 7 {
            code |= ERR_HALL_SEVEN;
        }
        if input.motor_enabled
            && input.speed.unsigned_abs() < DIAG_STILL_SPEED as u32
            && input.target.unsigned_abs() > DIAG_INPUT_TARGET_THRESHOLD as u32
        {
            code |= ERR_STALLED;
        }
        code
    }

    /// Advance one FOC tick and return the latched error code.
    pub fn update(&mut self, input: DiagnosticInput) -> u8 {
        let raw = Self::raw_error(input);
        if raw != 0 {
            self.dequalify = 0;
            if self.qualify < DIAG_QUALIFY_TICKS {
                self.qualify += 1;
            }
            if self.qualify >= DIAG_QUALIFY_TICKS {
                self.error_code |= raw;
            }
        } else {
            self.qualify = 0;
            if self.error_code != 0 {
                if self.dequalify < DIAG_DEQUALIFY_TICKS {
                    self.dequalify += 1;
                }
                if self.dequalify >= DIAG_DEQUALIFY_TICKS {
                    self.error_code = 0;
                    self.dequalify = 0;
                }
            }
        }
        self.error_code
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(hall_sum: u8) -> DiagnosticInput {
        DiagnosticInput {
            hall_sum,
            motor_enabled: false,
            speed: 0,
            target: 0,
        }
    }

    #[test]
    fn illegal_hall_is_qualified_and_latched() {
        let mut diag = MotorDiagnostics::new();
        for _ in 0..DIAG_QUALIFY_TICKS - 1 {
            assert_eq!(diag.update(input(0)), 0);
        }
        assert_eq!(diag.update(input(0)), ERR_HALL_ZERO);
        // A single clean tick must not clear a latched fault.
        assert_eq!(diag.update(input(1)), ERR_HALL_ZERO);
    }

    #[test]
    fn latch_clears_only_after_the_full_debounce() {
        let mut diag = MotorDiagnostics::new();
        for _ in 0..DIAG_QUALIFY_TICKS {
            diag.update(input(7));
        }
        for _ in 0..DIAG_DEQUALIFY_TICKS - 1 {
            assert_eq!(diag.update(input(1)), ERR_HALL_SEVEN);
        }
        assert_eq!(diag.update(input(1)), 0);
    }

    #[test]
    fn stall_needs_enable_low_speed_and_large_target() {
        let mut diag = MotorDiagnostics::new();
        let stalled = DiagnosticInput {
            hall_sum: 1,
            motor_enabled: true,
            speed: 10,
            target: DIAG_INPUT_TARGET_THRESHOLD + 1,
        };
        for _ in 0..DIAG_QUALIFY_TICKS {
            diag.update(stalled);
        }
        assert_eq!(diag.error_code, ERR_STALLED);
    }
}
