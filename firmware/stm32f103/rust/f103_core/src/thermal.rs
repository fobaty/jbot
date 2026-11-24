//! Board temperature estimation from the internal ADC sensor.
//!
//! The NTC mapping is affine between the two calibration points from the
//! pinned `config.h`. Both alert thresholds stay disabled until the board's
//! sensor is calibrated on the bench, exactly as in the reference build.

use crate::config::{
    TEMP_CAL_HIGH_ADC, TEMP_CAL_HIGH_DECIC, TEMP_CAL_LOW_ADC, TEMP_CAL_LOW_DECIC, TEMP_FILT_COEF,
};
use crate::fixed::filt_low_pass32;

#[derive(Clone)]
pub struct TemperatureEstimator {
    filtered: i32,
}

impl Default for TemperatureEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl TemperatureEstimator {
    pub const fn new() -> Self {
        Self {
            filtered: TEMP_CAL_LOW_ADC << 16,
        }
    }

    /// Feed one raw temperature ADC sample at the divided (1 kHz) rate.
    pub fn update(&mut self, raw_adc: u16) -> i16 {
        filt_low_pass32(raw_adc as i32, TEMP_FILT_COEF, &mut self.filtered);
        self.decideg()
    }

    pub fn raw(&self) -> i16 {
        (self.filtered >> 16) as i16
    }

    /// Degrees Celsius multiplied by ten, matching the feedback field.
    pub fn decideg(&self) -> i16 {
        let raw = self.raw() as i32;
        let mapped = (TEMP_CAL_HIGH_DECIC - TEMP_CAL_LOW_DECIC) * (raw - TEMP_CAL_LOW_ADC)
            / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC)
            + TEMP_CAL_LOW_DECIC;
        mapped as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_points_map_exactly() {
        let mut estimator = TemperatureEstimator::new();
        assert_eq!(i32::from(estimator.decideg()), TEMP_CAL_LOW_DECIC);
        estimator.filtered = TEMP_CAL_HIGH_ADC << 16;
        assert_eq!(i32::from(estimator.decideg()), TEMP_CAL_HIGH_DECIC);
    }

    #[test]
    fn filter_tracks_a_changing_sensor() {
        let mut estimator = TemperatureEstimator::new();
        for _ in 0..2_000 {
            estimator.update(TEMP_CAL_HIGH_ADC as u16);
        }
        assert_eq!(i32::from(estimator.decideg()), TEMP_CAL_HIGH_DECIC);
    }
}
