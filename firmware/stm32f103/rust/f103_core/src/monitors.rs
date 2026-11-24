//! Input qualification and activity monitors from the pinned C firmware.
//!
//! These are pure counters so they can be tested against the exact 5 ms and
//! 16 kHz qualification windows recorded in `config.h` and `util.c`.

use crate::config::{ADC_PROTECT_TIMEOUT, INACTIVITY_CMD_THRESHOLD, INACTIVITY_TICKS};

/// Qualifies a raw control input against the configured range. The counter
/// starts saturated, so the very first out-of-range sample raises the flag,
/// matching `timeoutCntADC = ADC_PROTECT_TIMEOUT` in `util.c:140`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputQualifier {
    counter: u32,
    pub fault: bool,
}

impl Default for InputQualifier {
    fn default() -> Self {
        Self::new()
    }
}

impl InputQualifier {
    pub const fn new() -> Self {
        Self {
            counter: ADC_PROTECT_TIMEOUT,
            fault: false,
        }
    }

    pub fn update(&mut self, in_range: bool) -> bool {
        if in_range {
            self.fault = false;
            self.counter = 0;
        } else {
            if self.counter < ADC_PROTECT_TIMEOUT {
                self.counter += 1;
            }
            if self.counter >= ADC_PROTECT_TIMEOUT {
                self.fault = true;
            }
        }
        self.fault
    }
}

/// Reports prolonged lack of drive. A balancing robot maps this to a safe
/// state and a warning rather than the C firmware's hard latch-off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InactivityMonitor {
    counter: u32,
}

impl Default for InactivityMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl InactivityMonitor {
    pub const fn new() -> Self {
        Self { counter: 0 }
    }

    pub fn tick(&mut self, cmd_left: i16, cmd_right: i16, cruise_active: bool) -> bool {
        if cmd_left.unsigned_abs() > INACTIVITY_CMD_THRESHOLD as u16
            || cmd_right.unsigned_abs() > INACTIVITY_CMD_THRESHOLD as u16
            || cruise_active
        {
            self.counter = 0;
        } else if self.counter < INACTIVITY_TICKS {
            self.counter += 1;
        }
        self.counter >= INACTIVITY_TICKS
    }

    pub fn elapsed_ticks(&self) -> u32 {
        self.counter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_bad_sample_qualifies_immediately() {
        let mut q = InputQualifier::new();
        assert!(q.update(false));
        assert!(!q.update(true));
        for _ in 0..ADC_PROTECT_TIMEOUT - 1 {
            assert!(!q.update(false));
        }
        assert!(q.update(false));
    }

    #[test]
    fn inactivity_resets_on_any_drive_command() {
        let mut m = InactivityMonitor::new();
        for _ in 0..INACTIVITY_TICKS - 1 {
            assert!(!m.tick(0, 0, false));
        }
        assert!(!m.tick(60, 0, false));
        assert_eq!(m.elapsed_ticks(), 0);
        for _ in 0..INACTIVITY_TICKS {
            m.tick(0, 0, false);
        }
        assert!(m.tick(0, 0, false));
    }
}
