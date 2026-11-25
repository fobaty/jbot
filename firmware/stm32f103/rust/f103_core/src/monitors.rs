//! Input qualification and activity monitors from the pinned C firmware.
//!
//! These are pure counters so they can be tested against the exact 5 ms and
//! 16 kHz qualification windows recorded in `config.h` and `util.c`.

use crate::config::{
    ADC_PROTECT_TIMEOUT, INACTIVITY_CMD_THRESHOLD, INACTIVITY_TICKS, SERIAL_TIMEOUT_TICKS,
};

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

/// Mirrors `timeoutCntSerial_L` / `timeoutFlgSerial_L` from `util.c:148-149`
/// and `:961-963`. The counter starts saturated, so the first tick without a
/// valid command raises the flag, exactly as in the reference. A valid frame
/// clears both the flag and the counter (`util.c:1282-1283`).
///
/// This is tracked independently of the `SafeController` fault latch: the
/// buzzer must report a missing link from boot, before any frame has ever been
/// received, while the safe state is already enforced by the command age check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SerialTimeoutMonitor {
    counter: u16,
    pub flag: bool,
}

impl Default for SerialTimeoutMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl SerialTimeoutMonitor {
    pub const fn new() -> Self {
        Self {
            counter: SERIAL_TIMEOUT_TICKS,
            flag: false,
        }
    }

    /// One 5 ms main-loop tick. Returns true while the link is expired.
    pub fn tick(&mut self, valid_command: bool) -> bool {
        if valid_command {
            self.flag = false;
            self.counter = 0;
            return self.flag;
        }
        if self.counter >= SERIAL_TIMEOUT_TICKS {
            self.flag = true;
            self.counter = SERIAL_TIMEOUT_TICKS;
        } else {
            self.counter += 1;
            if self.counter >= SERIAL_TIMEOUT_TICKS {
                self.flag = true;
            }
        }
        self.flag
    }

    pub fn elapsed_ticks(&self) -> u16 {
        self.counter
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
    fn serial_timeout_is_raised_on_the_first_tick_without_a_command() {
        let mut m = SerialTimeoutMonitor::new();
        assert!(!m.flag);
        assert!(m.tick(false));
        assert_eq!(m.elapsed_ticks(), SERIAL_TIMEOUT_TICKS);
    }

    #[test]
    fn serial_timeout_is_cleared_only_by_a_valid_command() {
        let mut m = SerialTimeoutMonitor::new();
        assert!(m.tick(false));
        assert!(!m.tick(true));
        assert_eq!(m.elapsed_ticks(), 0);
        for _ in 0..SERIAL_TIMEOUT_TICKS - 1 {
            assert!(!m.tick(false));
        }
        assert!(m.tick(false));
        assert!(m.flag);
        assert!(!m.tick(true));
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
