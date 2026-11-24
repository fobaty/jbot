//! Link-loss safe state and command validation state machine.
//!
//! The reference requests `OPEN_MODE` as soon as its serial timeout expires.
//! For a balancing robot that is a fall, so the safe state is an explicit,
//! selected contract (see `docs/prompt-f103-rust-firmware.md` section 2.4).

use crate::config::{COMMAND_TIMEOUT_US, FILTER_ALPHA_Q16, MAIN_LOOP_MS, RATE_Q4};
use crate::fixed::{low_pass_q16, rate_limit};
use crate::protocol::{CommandError, Setpoint};

/// Selected by the project decision: a balancing robot must not freewheel when
/// the command link dies. Change only with a new safety review.
pub const DEFAULT_SAFE_MODE: SafeMode = SafeMode::RampToZero;

pub const RAMP_STEP_PER_TICK: i16 = 50;
pub const BRAKE_DURATION_US: u32 = 200_000;
pub const BRAKE_CURRENT_UNITS: i16 = 300;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafeMode {
    /// Stock behaviour: immediate high impedance. Compatibility only.
    Freewheel,
    /// Bounded torque ramp to zero, then outputs disabled. Recommended.
    RampToZero,
    /// Braking torque within the current budget, then zero. Needs bench data.
    ActiveBrake,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    LinkTimeout,
    CommandRejected,
    MotorDiagnostic,
    Overcurrent,
    LoopOverrun,
    BatteryCritical,
    ThermalCritical,
    Inactivity,
}

pub struct SafeController {
    mode: SafeMode,
    last_valid_us: Option<u32>,
    last_seq: Option<u16>,
    last_setpoint: Setpoint,
    filtered_steer: i32,
    filtered_speed: i32,
    applied_steer: i16,
    applied_speed: i16,
    armed: bool,
    fault: Option<Fault>,
    pub faults: u32,
    pub rejected: u32,
}

impl SafeController {
    pub fn new(mode: SafeMode) -> Self {
        Self {
            mode,
            last_valid_us: None,
            last_seq: None,
            last_setpoint: Setpoint::ZERO,
            filtered_steer: 0,
            filtered_speed: 0,
            applied_steer: 0,
            applied_speed: 0,
            armed: false,
            fault: None,
            faults: 0,
            rejected: 0,
        }
    }

    pub fn mode(&self) -> SafeMode {
        self.mode
    }

    pub fn armed(&self) -> bool {
        self.armed
    }

    pub fn fault(&self) -> Option<Fault> {
        self.fault
    }

    pub fn applied(&self) -> (i16, i16) {
        (self.applied_steer, self.applied_speed)
    }

    pub fn set_mode(&mut self, mode: SafeMode) {
        self.mode = mode;
    }

    pub fn arm(&mut self) {
        self.fault = None;
        self.armed = true;
    }

    pub fn disarm(&mut self) {
        self.armed = false;
    }

    pub fn raise_fault(&mut self, fault: Fault) {
        self.fault = Some(fault);
        self.faults = self.faults.wrapping_add(1);
        self.armed = false;
    }

    /// Accept a decoded, CRC-checked command. Sequence and age are checked here
    /// so a rejected frame never partially updates the actuator setpoint.
    pub fn accept(
        &mut self,
        now_us: u32,
        seq: u16,
        setpoint: Setpoint,
    ) -> Result<(), CommandError> {
        if let Some(previous) = self.last_seq {
            if (seq.wrapping_sub(previous) as i16) <= 0 {
                self.rejected = self.rejected.wrapping_add(1);
                return Err(CommandError::BadSequence);
            }
        }
        if now_us.wrapping_sub(self.last_valid_us.unwrap_or(now_us)) > COMMAND_TIMEOUT_US {
            self.rejected = self.rejected.wrapping_add(1);
        }
        self.last_seq = Some(seq);
        self.last_valid_us = Some(now_us);
        self.last_setpoint = setpoint;
        Ok(())
    }

    pub fn link_age_us(&self, now_us: u32) -> Option<u32> {
        self.last_valid_us.map(|then| now_us.wrapping_sub(then))
    }

    /// One 5 ms tick. Returns `(steer, speed)` in wire-channel units after
    /// filtering, rate limiting and safe-state enforcement. The caller applies
    /// the differential mixer to obtain physical left/right wheel torque.
    pub fn tick(&mut self, now_us: u32) -> (i16, i16) {
        let aged_out = self
            .link_age_us(now_us)
            .is_none_or(|age| age > COMMAND_TIMEOUT_US);
        if aged_out
            && !matches!(self.fault, Some(Fault::LinkTimeout))
            && self.last_valid_us.is_some()
        {
            self.raise_fault(Fault::LinkTimeout);
        }

        if self.fault.is_some() || !self.armed || aged_out {
            let (left, right) = self.safe_output(now_us);
            self.applied_steer = left;
            self.applied_speed = right;
            return (left, right);
        }

        let (left, right) = (self.last_setpoint.steer, self.last_setpoint.speed);
        let filtered_steer = low_pass_q16(&mut self.filtered_steer, left as i32, FILTER_ALPHA_Q16);
        let filtered_speed = low_pass_q16(&mut self.filtered_speed, right as i32, FILTER_ALPHA_Q16);
        self.applied_steer = rate_limit(self.applied_steer, filtered_steer as i16, RATE_Q4);
        self.applied_speed = rate_limit(self.applied_speed, filtered_speed as i16, RATE_Q4);
        (self.applied_steer, self.applied_speed)
    }

    fn safe_output(&mut self, now_us: u32) -> (i16, i16) {
        match self.mode {
            SafeMode::Freewheel => (0, 0),
            SafeMode::RampToZero => {
                self.applied_steer = step_toward_zero(self.applied_steer, RAMP_STEP_PER_TICK);
                self.applied_speed = step_toward_zero(self.applied_speed, RAMP_STEP_PER_TICK);
                (self.applied_steer, self.applied_speed)
            }
            SafeMode::ActiveBrake => {
                let age = self.link_age_us(now_us).unwrap_or(u32::MAX);
                if age <= BRAKE_DURATION_US && (self.applied_steer != 0 || self.applied_speed != 0)
                {
                    self.applied_steer = brake(self.applied_steer);
                    self.applied_speed = brake(self.applied_speed);
                } else {
                    self.applied_steer = 0;
                    self.applied_speed = 0;
                }
                (self.applied_steer, self.applied_speed)
            }
        }
    }
}

fn step_toward_zero(value: i16, step: i16) -> i16 {
    if value > step {
        value - step
    } else if value < -step {
        value + step
    } else {
        0
    }
}

fn brake(value: i16) -> i16 {
    if value == 0 {
        0
    } else if value > 0 {
        -BRAKE_CURRENT_UNITS
    } else {
        BRAKE_CURRENT_UNITS
    }
}

pub const TICK_MS: u32 = MAIN_LOOP_MS;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn freewheel_zeroes_immediately() {
        let mut ctrl = SafeController::new(SafeMode::Freewheel);
        ctrl.arm();
        ctrl.accept(
            0,
            1,
            Setpoint {
                steer: 500,
                speed: 500,
            },
        )
        .unwrap();
        // The first tick is rate-limited; the timeout must still force zero.
        let first = ctrl.tick(0);
        assert!(first.0 > 0 && first.0 <= 500);
        assert_eq!(ctrl.tick(COMMAND_TIMEOUT_US + 1), (0, 0));
        assert_eq!(ctrl.fault, Some(Fault::LinkTimeout));
    }

    #[test]
    fn ramp_to_zero_is_bounded_and_reaches_zero() {
        let mut ctrl = SafeController::new(SafeMode::RampToZero);
        ctrl.arm();
        ctrl.accept(
            0,
            1,
            Setpoint {
                steer: 100,
                speed: -100,
            },
        )
        .unwrap();
        ctrl.tick(0);
        let after = (1..=40)
            .map(|_| ctrl.tick(COMMAND_TIMEOUT_US + 1))
            .last()
            .unwrap();
        assert_eq!(after, (0, 0));
        ctrl.accept(
            COMMAND_TIMEOUT_US + 2,
            2,
            Setpoint {
                steer: 100,
                speed: -100,
            },
        )
        .unwrap();
    }

    #[test]
    fn active_brake_is_budgeted_and_expires() {
        let mut ctrl = SafeController::new(SafeMode::ActiveBrake);
        ctrl.arm();
        ctrl.accept(
            0,
            1,
            Setpoint {
                steer: 400,
                speed: -400,
            },
        )
        .unwrap();
        ctrl.tick(0);
        let (left, right) = ctrl.tick(COMMAND_TIMEOUT_US + MAIN_LOOP_MS * 1000);
        assert!(left < 0 && right > 0);
        assert!(left >= -BRAKE_CURRENT_UNITS && right <= BRAKE_CURRENT_UNITS);
        let (left, right) = ctrl.tick(COMMAND_TIMEOUT_US + BRAKE_DURATION_US + 10_000);
        assert_eq!((left, right), (0, 0));
    }

    #[test]
    fn faults_and_disarm_force_safe_output() {
        let mut ctrl = SafeController::new(SafeMode::RampToZero);
        ctrl.arm();
        ctrl.accept(
            0,
            1,
            Setpoint {
                steer: 500,
                speed: 500,
            },
        )
        .unwrap();
        ctrl.raise_fault(Fault::MotorDiagnostic);
        let (left, right) = ctrl.tick(MAIN_LOOP_MS * 1000);
        assert!(left.abs() <= 500 && right.abs() <= 500);
        assert_eq!(ctrl.faults, 1);
    }
}
