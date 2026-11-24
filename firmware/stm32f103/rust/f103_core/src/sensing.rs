//! ADC field layout and current reconstruction, mirrored from `adc_buf_t`.
//!
//! The dual ADC reads five ranks per trigger and packs both converters into one
//! 32-bit word. DMA therefore produces ten 16-bit fields in a fixed order. The
//! current shunts are centred on a learned offset, so a phase current is
//! `offset - sample`, exactly as in the pinned C build.

use crate::config::{ADC_OFFSET_SAMPLES, COMMAND_LIMIT, I_MAX_Q4};
use crate::fixed::clamp_i32;
use crate::motor::Currents;

pub const ADC_WORDS: usize = 5;
pub const ADC_FIELDS: usize = 10;

// Indices into the packed 16-bit field array. Word 0 is (dcr, dcl), word 1 is
// (rlA, rlB), word 2 is (rrB, rrC), then battery, helper inputs and temperature.
pub const DCR: usize = 0;
pub const DCL: usize = 1;
pub const RLA: usize = 2;
pub const RLB: usize = 3;
pub const RRB: usize = 4;
pub const RRC: usize = 5;
pub const BATT1: usize = 6;
pub const L_TX2: usize = 7;
pub const TEMP: usize = 8;
pub const L_RX2: usize = 9;

const CURRENT_FIELDS: [usize; 6] = [DCR, DCL, RLA, RLB, RRB, RRC];

/// Running midpoint estimator for the six current channels.
#[derive(Clone, Copy)]
pub struct CurrentCalibration {
    offsets: [u16; 6],
    samples: u32,
}

impl Default for CurrentCalibration {
    fn default() -> Self {
        Self::new()
    }
}

impl CurrentCalibration {
    pub const fn new() -> Self {
        Self {
            offsets: [0; 6],
            samples: 0,
        }
    }

    pub fn ready(&self) -> bool {
        self.samples >= ADC_OFFSET_SAMPLES
    }

    /// Accumulate the running mean. Returns true once calibration is complete;
    /// the caller must not trust currents before that point.
    pub fn update(&mut self, fields: &[u16; ADC_FIELDS]) -> bool {
        if self.ready() {
            return true;
        }
        self.samples += 1;
        for (offset, field) in self.offsets.iter_mut().zip(CURRENT_FIELDS) {
            *offset = ((*offset as u32 + fields[field] as u32) / 2) as u16;
        }
        self.ready()
    }

    /// Reconstruct both motors' three-phase currents and the two DC-link
    /// currents in raw ADC counts. The third phase follows from KCL.
    pub fn sample(&self, fields: &[u16; ADC_FIELDS]) -> Sense {
        let ia_l = self.offsets[2] as i32 - fields[RLA] as i32;
        let ib_l = self.offsets[3] as i32 - fields[RLB] as i32;
        let ib_r = self.offsets[4] as i32 - fields[RRB] as i32;
        let ic_r = self.offsets[5] as i32 - fields[RRC] as i32;
        Sense {
            left: Currents {
                ia: ia_l,
                ib: ib_l,
                ic: -ia_l - ib_l,
            },
            right: Currents {
                ia: -ib_r - ic_r,
                ib: ib_r,
                ic: ic_r,
            },
            dc_left: self.offsets[1] as i32 - fields[DCL] as i32,
            dc_right: self.offsets[0] as i32 - fields[DCR] as i32,
        }
    }
}

/// One current sample for both motors, in raw ADC counts.
#[derive(Clone, Copy)]
pub struct Sense {
    pub left: Currents,
    pub right: Currents,
    pub dc_left: i32,
    pub dc_right: i32,
}

/// Map the command range onto the calibrated full-scale current in Q4. A zero
/// command always produces a zero target, with no floating-point path.
pub fn torque_to_q(command: i16) -> i32 {
    clamp_i32(
        command as i32 * I_MAX_Q4 / COMMAND_LIMIT as i32,
        -I_MAX_Q4,
        I_MAX_Q4,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fields(value: u16) -> [u16; ADC_FIELDS] {
        [value; ADC_FIELDS]
    }

    fn calibrated() -> CurrentCalibration {
        let mut cal = CurrentCalibration::new();
        // A zero midpoint keeps the integer running mean exact for the tests.
        for _ in 0..ADC_OFFSET_SAMPLES {
            cal.update(&fields(0));
        }
        assert!(cal.ready());
        cal
    }

    #[test]
    fn calibration_needs_the_full_window() {
        let mut cal = CurrentCalibration::new();
        for _ in 0..ADC_OFFSET_SAMPLES - 1 {
            assert!(!cal.update(&fields(2_048)));
        }
        assert!(cal.update(&fields(2_048)));
    }

    #[test]
    fn currents_are_inverted_about_the_learned_offset() {
        let cal = calibrated();
        let mut sample = fields(0);
        sample[RLA] = 100;
        sample[RLB] = 50;
        sample[RRB] = 80;
        sample[RRC] = 20;
        let sense = cal.sample(&sample);
        assert_eq!(sense.left.ia, -100);
        assert_eq!(sense.left.ib, -50);
        assert_eq!(sense.left.ic, 150);
        assert_eq!(sense.right.ib, -80);
        assert_eq!(sense.right.ic, -20);
        assert_eq!(sense.right.ia, 100);
    }

    #[test]
    fn dc_links_read_the_matching_packed_half() {
        let cal = calibrated();
        let mut sample = fields(0);
        sample[DCR] = 900;
        sample[DCL] = 1_100;
        let sense = cal.sample(&sample);
        assert_eq!(sense.dc_right, -900);
        assert_eq!(sense.dc_left, -1_100);
    }

    #[test]
    fn torque_targets_are_bounded_and_linear() {
        assert_eq!(torque_to_q(0), 0);
        assert_eq!(torque_to_q(COMMAND_LIMIT), I_MAX_Q4);
        assert_eq!(torque_to_q(-COMMAND_LIMIT), -I_MAX_Q4);
        assert_eq!(torque_to_q(500), I_MAX_Q4 / 2);
    }
}
