//! Alert priority and buzzer patterns from `main.c:566-597` and `bldc.c:150-167`.
//!
//! The C firmware selects exactly one audible alert per main-loop tick. The
//! ordering is part of the safety contract: a motor fault outranks a serial
//! timeout, which outranks a low-battery warning.

use crate::config::{BUZZER_BURST_TICKS, BUZZER_HIGH_FREQ, BUZZER_LOW_FREQ, BUZZER_MED_FREQ};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Alert {
    None,
    ThermalPoweroff,
    BatteryDead,
    MotorDiagnostic,
    AdcTimeout,
    SerialTimeout,
    GeneralTimeout,
    ThermalWarning,
    BatteryLevel1,
    BatteryLevel2,
    Backward,
}

impl Alert {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::None => 0,
            Self::ThermalPoweroff => 1,
            Self::BatteryDead => 2,
            Self::MotorDiagnostic => 3,
            Self::AdcTimeout => 4,
            Self::SerialTimeout => 5,
            Self::GeneralTimeout => 6,
            Self::ThermalWarning => 7,
            Self::BatteryLevel1 => 8,
            Self::BatteryLevel2 => 9,
            Self::Backward => 10,
        }
    }

    pub const fn from_u8(value: u8) -> Self {
        match value {
            1 => Self::ThermalPoweroff,
            2 => Self::BatteryDead,
            3 => Self::MotorDiagnostic,
            4 => Self::AdcTimeout,
            5 => Self::SerialTimeout,
            6 => Self::GeneralTimeout,
            7 => Self::ThermalWarning,
            8 => Self::BatteryLevel1,
            9 => Self::BatteryLevel2,
            10 => Self::Backward,
            _ => Self::None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AlertInputs {
    pub thermal_poweroff: bool,
    pub battery_dead: bool,
    pub motor_error: bool,
    pub adc_timeout: bool,
    pub serial_timeout: bool,
    pub general_timeout: bool,
    pub thermal_warning: bool,
    pub battery_lvl1: bool,
    pub battery_lvl2: bool,
    pub backward: bool,
}

/// Returns the single highest-priority alert for this tick.
pub fn select_alert(input: &AlertInputs) -> Alert {
    if input.thermal_poweroff {
        Alert::ThermalPoweroff
    } else if input.battery_dead {
        Alert::BatteryDead
    } else if input.motor_error {
        Alert::MotorDiagnostic
    } else if input.adc_timeout {
        Alert::AdcTimeout
    } else if input.serial_timeout {
        Alert::SerialTimeout
    } else if input.general_timeout {
        Alert::GeneralTimeout
    } else if input.thermal_warning {
        Alert::ThermalWarning
    } else if input.battery_lvl1 {
        Alert::BatteryLevel1
    } else if input.battery_lvl2 {
        Alert::BatteryLevel2
    } else if input.backward {
        Alert::Backward
    } else {
        Alert::None
    }
}

/// Non-blocking buzzer generator. The caller feeds it once per FOC tick and
/// drives the returned level to the PA4 gate; no delay is spent in the loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BuzzerController {
    timer: u32,
    previous_window: bool,
    count: u8,
    freq: u16,
    pattern: u8,
    index: u8,
    level: bool,
}

impl Default for BuzzerController {
    fn default() -> Self {
        Self::new()
    }
}

impl BuzzerController {
    pub const fn new() -> Self {
        Self {
            timer: 0,
            previous_window: false,
            count: 0,
            freq: 0,
            pattern: 0,
            index: 0,
            level: false,
        }
    }

    /// `count` bursts, `freq` divider (about 8000/freq Hz), `pattern` silent
    /// windows between bursts. Zero `freq` silences the buzzer.
    pub fn set(&mut self, alert: Alert) {
        let (count, freq, pattern) = match alert {
            Alert::None => (0, 0, 0),
            Alert::MotorDiagnostic => (1, BUZZER_LOW_FREQ, 1),
            Alert::AdcTimeout => (2, BUZZER_LOW_FREQ, 1),
            Alert::SerialTimeout => (3, BUZZER_LOW_FREQ, 1),
            Alert::GeneralTimeout => (4, BUZZER_LOW_FREQ, 1),
            Alert::ThermalWarning => (5, BUZZER_LOW_FREQ, 1),
            Alert::BatteryLevel1 => (0, BUZZER_MED_FREQ, 6),
            Alert::BatteryLevel2 => (0, BUZZER_MED_FREQ, 30),
            Alert::Backward => (0, BUZZER_HIGH_FREQ, 1),
            // The robot does not latch power off; use a continuous urgent tone
            // instead of the C firmware's descending poweroff sweep.
            Alert::ThermalPoweroff | Alert::BatteryDead => (0, BUZZER_HIGH_FREQ, 0),
        };
        self.count = count;
        self.freq = freq;
        self.pattern = pattern;
    }

    /// Advance one FOC tick. `Some(level)` means the output must be set.
    pub fn tick(&mut self) -> Option<bool> {
        self.timer = self.timer.wrapping_add(1);
        let in_window = self.freq != 0
            && (self.timer / BUZZER_BURST_TICKS).is_multiple_of(u32::from(self.pattern + 1));
        if in_window {
            if !self.previous_window {
                self.previous_window = true;
                self.index += 1;
                if self.index > self.count + 2 {
                    self.index = 1;
                }
            }
            if self.timer.is_multiple_of(u32::from(self.freq))
                && (self.index <= self.count || self.count == 0)
            {
                self.level = !self.level;
                return Some(self.level);
            }
            None
        } else if self.previous_window {
            self.previous_window = false;
            None
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn priority_follows_the_reference_order() {
        assert_eq!(select_alert(&AlertInputs::default()), Alert::None);
        let mut input = AlertInputs {
            backward: true,
            battery_lvl1: true,
            ..AlertInputs::default()
        };
        assert_eq!(select_alert(&input), Alert::BatteryLevel1);
        input.serial_timeout = true;
        assert_eq!(select_alert(&input), Alert::SerialTimeout);
        input.motor_error = true;
        assert_eq!(select_alert(&input), Alert::MotorDiagnostic);
        input.battery_dead = true;
        assert_eq!(select_alert(&input), Alert::BatteryDead);
    }

    #[test]
    fn silence_produces_no_toggles() {
        let mut buzzer = BuzzerController::new();
        buzzer.set(Alert::None);
        for _ in 0..BUZZER_BURST_TICKS * 4 {
            assert_eq!(buzzer.tick(), None);
        }
    }

    #[test]
    fn continuous_tone_toggles_at_the_configured_divider() {
        let mut buzzer = BuzzerController::new();
        buzzer.set(Alert::ThermalPoweroff);
        let mut toggles = 0;
        for _ in 0..BUZZER_BURST_TICKS {
            if buzzer.tick().is_some() {
                toggles += 1;
            }
        }
        assert_eq!(toggles, BUZZER_BURST_TICKS / u32::from(BUZZER_HIGH_FREQ));
    }
}
