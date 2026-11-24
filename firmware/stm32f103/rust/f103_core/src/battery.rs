//! Battery voltage estimation, mirrored from `config.h` and `bldc.c`.
//!
//! The C build filters the raw `batt1` ADC field at 16 kHz divided by 1000 and
//! converts the result to centivolts for feedback. The same filter state and
//! thresholds are kept here so the robot sees identical levels.

use crate::config::{
    BAT_CALIB_ADC, BAT_CALIB_REAL_VOLTAGE, BAT_DEAD_ADC, BAT_FILT_COEF, BAT_LVL1_ADC, BAT_LVL2_ADC,
    BAT_LVL3_ADC, BAT_LVL4_ADC, BAT_LVL5_ADC,
};
use crate::fixed::filt_low_pass32;

const INIT_ADC: i32 = (400 * crate::config::BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;

/// Battery state in the same order as the reference sideboard LEDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryLevel {
    Dead,
    Level1,
    Level2,
    Level3,
    Level4,
    Level5,
    Normal,
}

#[derive(Clone)]
pub struct BatteryMonitor {
    filtered: i32,
}

impl Default for BatteryMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl BatteryMonitor {
    pub const fn new() -> Self {
        Self {
            filtered: INIT_ADC << 16,
        }
    }

    /// Feed one raw ADC sample at the divided (1 kHz) rate.
    pub fn update(&mut self, raw_adc: u16) -> i16 {
        filt_low_pass32(raw_adc as i32, BAT_FILT_COEF, &mut self.filtered);
        (self.filtered >> 16) as i16
    }

    pub fn raw(&self) -> i16 {
        (self.filtered >> 16) as i16
    }

    /// Raw ADC counts to centivolts, as in `main.c:494`.
    pub const fn centivolts(raw_adc: i16) -> i16 {
        (raw_adc as i32 * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC) as i16
    }

    pub const fn level(raw_adc: i16) -> BatteryLevel {
        let raw = raw_adc as i32;
        if raw < BAT_DEAD_ADC {
            BatteryLevel::Dead
        } else if raw < BAT_LVL1_ADC {
            BatteryLevel::Level1
        } else if raw < BAT_LVL2_ADC {
            BatteryLevel::Level2
        } else if raw < BAT_LVL3_ADC {
            BatteryLevel::Level3
        } else if raw < BAT_LVL4_ADC {
            BatteryLevel::Level4
        } else if raw < BAT_LVL5_ADC {
            BatteryLevel::Level5
        } else {
            BatteryLevel::Normal
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initial_state_matches_flashed_four_volts_per_cell() {
        let monitor = BatteryMonitor::new();
        assert_eq!(monitor.raw(), INIT_ADC as i16);
        assert_eq!(monitor.raw(), 1_503);
    }

    #[test]
    fn filter_converges_to_the_supplied_sample() {
        let mut monitor = BatteryMonitor::new();
        for _ in 0..2_000 {
            monitor.update(1_400);
        }
        assert_eq!(monitor.raw(), 1_400);
    }

    #[test]
    fn levels_and_centivolts_match_the_pinned_thresholds() {
        assert_eq!(
            BatteryMonitor::level((BAT_DEAD_ADC - 1) as i16),
            BatteryLevel::Dead
        );
        assert_eq!(
            BatteryMonitor::level((BAT_LVL1_ADC - 1) as i16),
            BatteryLevel::Level1
        );
        assert_eq!(
            BatteryMonitor::level((BAT_LVL2_ADC - 1) as i16),
            BatteryLevel::Level2
        );
        assert_eq!(
            BatteryMonitor::level((BAT_LVL3_ADC - 1) as i16),
            BatteryLevel::Level3
        );
        assert_eq!(
            BatteryMonitor::level((BAT_LVL4_ADC - 1) as i16),
            BatteryLevel::Level4
        );
        assert_eq!(
            BatteryMonitor::level((BAT_LVL5_ADC - 1) as i16),
            BatteryLevel::Level5
        );
        assert_eq!(
            BatteryMonitor::level((BAT_LVL5_ADC + 1) as i16),
            BatteryLevel::Normal
        );
        assert_eq!(BatteryMonitor::centivolts(1_492), 3_970);
    }
}
