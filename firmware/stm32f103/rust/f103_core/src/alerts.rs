//! Alert priority and buzzer patterns from `main.c:566-597` and `bldc.c:150-167`.
//!
//! The C firmware selects exactly one audible alert per main-loop tick. The
//! ordering is part of the safety contract: a motor fault outranks a serial
//! timeout, which outranks a low-battery warning.

use crate::config::{
    BUZZER_BURST_TICKS, BUZZER_HIGH_FREQ, BUZZER_LOW_FREQ, BUZZER_MED_FREQ, BUZZER_VOLUME_DIVISOR,
};

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
    /// Remaining ISR ticks to hold the gate high for the current tone period.
    pulse: u16,
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
            pulse: 0,
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

    /// Continuous tone at a fixed divider, used by the power-on and power-off
    /// sweeps. Zero silences the output; it bypasses alert burst counting.
    pub fn set_frequency(&mut self, freq: u8) {
        self.count = 0;
        self.freq = u16::from(freq);
        self.pattern = 0;
    }

    /// Advance one FOC tick. `Some(level)` means the output must be set.
    /// The gate is held high for only a short slice of each tone period, so
    /// the tone frequency stays identical to the reference while the acoustic
    /// output drops several times compared with a 50% duty square wave.
    pub fn tick(&mut self) -> Option<bool> {
        self.timer = self.timer.wrapping_add(1);
        let in_window = self.freq != 0
            && (self.timer / BUZZER_BURST_TICKS).is_multiple_of(u32::from(self.pattern + 1));
        if !in_window {
            self.previous_window = false;
            if self.pulse > 0 {
                self.pulse = 0;
                return Some(false);
            }
            return None;
        }
        if !self.previous_window {
            self.previous_window = true;
            self.index += 1;
            if self.index > self.count + 2 {
                self.index = 1;
            }
        }
        if self.pulse > 0 {
            self.pulse -= 1;
            if self.pulse == 0 {
                return Some(false);
            }
            return None;
        }
        // The reference toggles the gate every `freq` FOC ticks, so one full
        // period is `2 * freq` ticks (8000/freq Hz at a 16 kHz FOC tick).
        // Triggering every `freq` ticks would raise every tone by an octave.
        if self.timer.is_multiple_of(u32::from(self.freq) * 2)
            && (self.index <= self.count || self.count == 0)
        {
            let high_ticks = (self.freq / BUZZER_VOLUME_DIVISOR).max(1);
            // Hold the gate high for `high_ticks` ticks, then force a falling
            // edge. A zero-length pulse would leave PA4 permanently high and
            // silence the tone instead of making it quieter.
            self.pulse = high_ticks;
            self.level = true;
            return Some(self.level);
        }
        None
    }
}

/// Power-on and power-off buzzer sweeps from `util.c:419-426` and `:1568-1571`.
/// Advanced once per 5 ms main-loop tick; each step lasts 100 ms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Melody {
    step: u8,
    ticks: u16,
    ascending: bool,
}

impl Melody {
    /// Descending divider sweep 8..=0: a rising tone ending in silence.
    pub const fn power_on() -> Self {
        Self {
            step: 0,
            ticks: 0,
            ascending: false,
        }
    }

    /// Ascending divider sweep 0..=7: the C firmware's power-off melody.
    pub const fn power_off() -> Self {
        Self {
            step: 0,
            ticks: 0,
            ascending: true,
        }
    }

    /// Returns the forced buzzer divider, or `None` once the sweep is over.
    pub fn tick(&mut self) -> Option<u8> {
        let last = if self.ascending { 7 } else { 8 };
        if self.step > last {
            return None;
        }
        let freq = if self.ascending {
            self.step
        } else {
            8 - self.step
        };
        self.ticks += 1;
        if self.ticks >= 20 {
            self.ticks = 0;
            self.step += 1;
        }
        Some(freq)
    }
}

/// Two short confirmation beeps on motor enable, from `main.c:272-273`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmBeep {
    step: u8,
    ticks: u16,
}

impl Default for ArmBeep {
    fn default() -> Self {
        Self::new()
    }
}

impl ArmBeep {
    pub const fn new() -> Self {
        Self { step: 0, ticks: 0 }
    }

    /// Silent state for boot: the two confirmation beeps belong to motor
    /// enable, not to power-on.
    pub const fn done() -> Self {
        Self { step: 4, ticks: 0 }
    }

    /// Returns the forced divider, or `None` when both beeps are finished.
    pub fn tick(&mut self) -> Option<u8> {
        const TONES: [u8; 4] = [6, 0, 4, 0];
        if self.step >= TONES.len() as u8 {
            return None;
        }
        let freq = TONES[self.step as usize];
        self.ticks += 1;
        if self.ticks >= 20 {
            self.ticks = 0;
            self.step += 1;
        }
        Some(freq)
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
        let mut rising_edges = 0;
        for _ in 0..BUZZER_BURST_TICKS {
            if buzzer.tick() == Some(true) {
                rising_edges += 1;
            }
        }
        assert_eq!(
            rising_edges,
            BUZZER_BURST_TICKS / (u32::from(BUZZER_HIGH_FREQ) * 2)
        );
    }

    #[test]
    fn every_sweep_tone_produces_a_falling_edge() {
        // A tone that never drives the gate low is silent, not quiet.
        for freq in 1..=8u8 {
            let mut buzzer = BuzzerController::new();
            buzzer.set_frequency(freq);
            let mut saw_high = false;
            let mut saw_low = false;
            for _ in 0..(u32::from(freq) * 4) {
                match buzzer.tick() {
                    Some(true) => saw_high = true,
                    Some(false) => saw_low = true,
                    None => {}
                }
            }
            assert!(saw_high && saw_low, "tone {freq} has no complete cycle");
        }
    }

    #[test]
    fn power_on_sweep_matches_the_reference() {
        let mut melody = Melody::power_on();
        let mut sequence = [u8::MAX; 9];
        for (index, slot) in sequence.iter_mut().enumerate() {
            let first = melody.tick();
            for _ in 1..20 {
                assert_eq!(melody.tick(), first, "each tone lasts 20 main ticks");
            }
            *slot = first.expect("melody not finished");
            assert!(index <= 8);
        }
        assert_eq!(sequence, [8, 7, 6, 5, 4, 3, 2, 1, 0]);
        assert_eq!(melody.tick(), None);
    }

    #[test]
    fn power_off_sweep_matches_the_reference() {
        let mut melody = Melody::power_off();
        let mut sequence = [u8::MAX; 8];
        for (index, slot) in sequence.iter_mut().enumerate() {
            let first = melody.tick();
            for _ in 1..20 {
                assert_eq!(melody.tick(), first, "each tone lasts 20 main ticks");
            }
            *slot = first.expect("melody not finished");
            assert!(index <= 7);
        }
        assert_eq!(sequence, [0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(melody.tick(), None);
    }

    #[test]
    fn arm_beep_starts_silent_when_done() {
        assert_eq!(ArmBeep::done().tick(), None);
    }

    #[test]
    fn arm_beep_matches_the_reference() {
        let mut beep = ArmBeep::new();
        let mut sequence = [u8::MAX; 4];
        for (index, slot) in sequence.iter_mut().enumerate() {
            let first = beep.tick();
            for _ in 1..20 {
                assert_eq!(beep.tick(), first);
            }
            *slot = first.expect("beep not finished");
            assert!(index <= 3);
        }
        assert_eq!(sequence, [6, 0, 4, 0]);
        assert_eq!(beep.tick(), None);
    }

    #[test]
    fn forced_frequency_drives_a_continuous_tone() {
        let mut buzzer = BuzzerController::new();
        buzzer.set_frequency(8);
        let mut rising_edges = 0;
        for _ in 0..BUZZER_BURST_TICKS {
            if buzzer.tick() == Some(true) {
                rising_edges += 1;
            }
        }
        assert_eq!(rising_edges, BUZZER_BURST_TICKS / 16);
        buzzer.set_frequency(0);
        // The short high pulse has already ended, so the gate is low and no
        // further transition is needed when the tone is cleared.
        assert_eq!(buzzer.tick(), None);
    }
}
