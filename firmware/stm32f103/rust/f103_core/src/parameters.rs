//! Non-volatile parameters and runtime tuning knobs.
//!
//! The flashed build stores a variant key plus `i_max`, `n_max` and the input
//! calibration words (`util.c:1527-1558`). A serial-controlled robot has no
//! analogue inputs to calibrate, so that space is used for the gains and limits
//! a calibration script needs to sweep. Safe-mode selection and the wire
//! protocol stay out of the record: they are safety contracts and must reset
//! to their audited defaults at boot.

use crate::config::{I_MAX_Q4, N_MOT_MAX_RPM, SPEED_COEFF_Q14, STEER_COEFF_Q14};

/// Matches the flashed `VARIANT_USART` key (`config.h:346`). Bumped version
/// invalidates records written by an older layout.
pub const FLASH_WRITE_KEY: u16 = 0x1002;
pub const PARAM_VERSION: u16 = 2;
pub const PARAM_WORDS: usize = 16;
const CRC_WORDS: usize = PARAM_WORDS - 2;
const CRC_BYTES: usize = CRC_WORDS * 2;

/// Limits copied from the generated controller. A corrupt or hostile record is
/// clamped to these, never allowed to raise them.
const MAX_GAIN: u16 = u16::MAX;
const MAX_VD: u16 = 14_400;
const MAX_OPEN_RATE: u16 = 32_767;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Parameters {
    pub i_max_q4: u16,
    pub n_max_q4: u16,
    pub steer_coeff_q14: u16,
    pub speed_coeff_q14: u16,
    pub id_kp: u16,
    pub id_ki: u16,
    pub iq_kp: u16,
    pub iq_ki: u16,
    pub n_kp: u16,
    pub n_ki: u16,
    pub curr_filt: u16,
    pub vd_max: u16,
    pub open_rate: u16,
}

impl Default for Parameters {
    fn default() -> Self {
        Self::defaults()
    }
}

impl Parameters {
    /// Values from `GeneratedParams::pinned()` and the pinned mixer config.
    pub const fn defaults() -> Self {
        Self {
            i_max_q4: I_MAX_Q4 as u16,
            n_max_q4: (N_MOT_MAX_RPM * 16) as u16,
            steer_coeff_q14: STEER_COEFF_Q14 as u16,
            speed_coeff_q14: SPEED_COEFF_Q14 as u16,
            id_kp: 819,
            id_ki: 737,
            iq_kp: 1_229,
            iq_ki: 1_229,
            n_kp: 4_833,
            n_ki: 251,
            curr_filt: 7_864,
            vd_max: 14_400,
            open_rate: 12_288,
        }
    }

    fn values(self) -> [u16; CRC_WORDS] {
        [
            PARAM_VERSION,
            self.i_max_q4,
            self.n_max_q4,
            self.steer_coeff_q14,
            self.speed_coeff_q14,
            self.id_kp,
            self.id_ki,
            self.iq_kp,
            self.iq_ki,
            self.n_kp,
            self.n_ki,
            self.curr_filt,
            self.vd_max,
            self.open_rate,
        ]
    }

    fn crc_of(values: &[u16; CRC_WORDS]) -> u16 {
        let mut bytes = [0u8; CRC_BYTES];
        for (index, word) in values.iter().enumerate() {
            bytes[index * 2..index * 2 + 2].copy_from_slice(&word.to_le_bytes());
        }
        crate::protocol::crc16(&bytes)
    }

    /// Persisted words: key, version, the tuning values, then CRC-16 of
    /// words 1..15. The key is written first so a foreign image is rejected
    /// before any value is applied.
    pub fn to_words(self) -> [u16; PARAM_WORDS] {
        let values = self.values();
        let mut words = [0u16; PARAM_WORDS];
        words[0] = FLASH_WRITE_KEY;
        words[1..CRC_WORDS + 1].copy_from_slice(&values);
        words[PARAM_WORDS - 1] = Self::crc_of(&values);
        words
    }

    /// Rejects foreign, outdated or damaged records and clamps every value to
    /// the hardware-proven maximum so a corrupt record cannot raise current,
    /// speed or mixer gain.
    pub fn from_words(words: &[u16; PARAM_WORDS]) -> Option<Self> {
        if words[0] != FLASH_WRITE_KEY || words[1] != PARAM_VERSION {
            return None;
        }
        let mut values = [0u16; CRC_WORDS];
        values.copy_from_slice(&words[1..CRC_WORDS + 1]);
        if Self::crc_of(&values) != words[PARAM_WORDS - 1] {
            return None;
        }
        Some(Self {
            i_max_q4: values[1].min(I_MAX_Q4 as u16),
            n_max_q4: values[2].min((N_MOT_MAX_RPM * 16) as u16),
            steer_coeff_q14: values[3].min(i16::MAX as u16),
            speed_coeff_q14: values[4].min(i16::MAX as u16),
            id_kp: values[5],
            id_ki: values[6],
            iq_kp: values[7],
            iq_ki: values[8],
            n_kp: values[9],
            n_ki: values[10],
            curr_filt: values[11],
            vd_max: values[12].min(MAX_VD),
            open_rate: values[13].min(MAX_OPEN_RATE),
        })
    }

    pub fn is_valid_record(words: &[u16; PARAM_WORDS]) -> bool {
        Self::from_words(words).is_some()
    }

    /// Copy only the fields that are exposed as tuning keys. Everything else
    /// in the generated controller stays at its pinned, validated value.
    pub fn apply_to(&self, target: &mut crate::foc_generated::GeneratedParams) {
        target.i_max = self.i_max_q4 as i16;
        target.n_max = self.n_max_q4 as i16;
        target.cf_idKp = self.id_kp;
        target.cf_idKi = self.id_ki;
        target.cf_iqKp = self.iq_kp;
        target.cf_iqKi = self.iq_ki;
        target.cf_nKp = self.n_kp;
        target.cf_nKi = self.n_ki;
        target.cf_curr_filt = self.curr_filt;
        target.Vd_max = self.vd_max as i16;
        target.dV_open_rate = self.open_rate as i32;
    }
}

/// Console-visible tuning keys. Scripts use stable uppercase names; the enum
/// owns clamping so no caller can bypass the hardware limits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tunable {
    IMax,
    NMax,
    SteerCoeff,
    SpeedCoeff,
    IdKp,
    IdKi,
    IqKp,
    IqKi,
    NKp,
    NKi,
    CurrFilt,
    VdMax,
    OpenRate,
}

impl Tunable {
    pub const ALL: &'static [(&'static str, Self)] = &[
        ("IMAX", Self::IMax),
        ("NMAX", Self::NMax),
        ("STEER", Self::SteerCoeff),
        ("SPEED", Self::SpeedCoeff),
        ("IDKP", Self::IdKp),
        ("IDKI", Self::IdKi),
        ("IQKP", Self::IqKp),
        ("IQKI", Self::IqKi),
        ("NKP", Self::NKp),
        ("NKI", Self::NKi),
        ("CURRFILT", Self::CurrFilt),
        ("VDMAX", Self::VdMax),
        ("OPENRATE", Self::OpenRate),
    ];

    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL
            .iter()
            .find(|(label, _)| label.eq_ignore_ascii_case(name))
            .map(|(_, key)| *key)
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::IMax => "IMAX",
            Self::NMax => "NMAX",
            Self::SteerCoeff => "STEER",
            Self::SpeedCoeff => "SPEED",
            Self::IdKp => "IDKP",
            Self::IdKi => "IDKI",
            Self::IqKp => "IQKP",
            Self::IqKi => "IQKI",
            Self::NKp => "NKP",
            Self::NKi => "NKI",
            Self::CurrFilt => "CURRFILT",
            Self::VdMax => "VDMAX",
            Self::OpenRate => "OPENRATE",
        }
    }

    pub const fn get(self, params: &Parameters) -> u16 {
        match self {
            Self::IMax => params.i_max_q4,
            Self::NMax => params.n_max_q4,
            Self::SteerCoeff => params.steer_coeff_q14,
            Self::SpeedCoeff => params.speed_coeff_q14,
            Self::IdKp => params.id_kp,
            Self::IdKi => params.id_ki,
            Self::IqKp => params.iq_kp,
            Self::IqKi => params.iq_ki,
            Self::NKp => params.n_kp,
            Self::NKi => params.n_ki,
            Self::CurrFilt => params.curr_filt,
            Self::VdMax => params.vd_max,
            Self::OpenRate => params.open_rate,
        }
    }

    /// Writes a clamped value and returns the value actually stored, so the
    /// console reply always reports the active setting.
    pub fn set(self, params: &mut Parameters, value: u32) -> u16 {
        let max = match self {
            Self::IMax => I_MAX_Q4 as u32,
            Self::NMax => (N_MOT_MAX_RPM * 16) as u32,
            Self::SteerCoeff | Self::SpeedCoeff => i16::MAX as u32,
            Self::VdMax => MAX_VD as u32,
            Self::OpenRate => MAX_OPEN_RATE as u32,
            _ => MAX_GAIN as u32,
        };
        let applied = value.min(max) as u16;
        match self {
            Self::IMax => params.i_max_q4 = applied,
            Self::NMax => params.n_max_q4 = applied,
            Self::SteerCoeff => params.steer_coeff_q14 = applied,
            Self::SpeedCoeff => params.speed_coeff_q14 = applied,
            Self::IdKp => params.id_kp = applied,
            Self::IdKi => params.id_ki = applied,
            Self::IqKp => params.iq_kp = applied,
            Self::IqKi => params.iq_ki = applied,
            Self::NKp => params.n_kp = applied,
            Self::NKi => params.n_ki = applied,
            Self::CurrFilt => params.curr_filt = applied,
            Self::VdMax => params.vd_max = applied,
            Self::OpenRate => params.open_rate = applied,
        }
        applied
    }

    /// True for values that change the actuator law rather than only the
    /// diagnostics. Used to refuse changes while armed.
    pub const fn is_actuator(self) -> bool {
        !matches!(self, Self::SteerCoeff | Self::SpeedCoeff)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_survive_a_round_trip() {
        let parameters = Parameters::defaults();
        assert_eq!(
            Parameters::from_words(&parameters.to_words()),
            Some(parameters)
        );
    }

    #[test]
    fn foreign_key_version_and_bad_crc_are_refused() {
        let mut words = Parameters::defaults().to_words();
        words[0] ^= 1;
        assert_eq!(Parameters::from_words(&words), None);
        words = Parameters::defaults().to_words();
        words[1] ^= 1;
        assert_eq!(Parameters::from_words(&words), None);
        words = Parameters::defaults().to_words();
        words[7] ^= 1;
        assert_eq!(Parameters::from_words(&words), None);
    }

    #[test]
    fn a_corrupt_limit_is_clamped_not_amplified() {
        let mut words = Parameters::defaults().to_words();
        words[2] = u16::MAX;
        words[4] = u16::MAX;
        words[13] = u16::MAX;
        let values = {
            let mut v = [0u16; CRC_WORDS];
            v.copy_from_slice(&words[1..CRC_WORDS + 1]);
            v
        };
        words[PARAM_WORDS - 1] = Parameters::crc_of(&values);
        let restored = Parameters::from_words(&words).unwrap();
        assert_eq!(restored.i_max_q4, I_MAX_Q4 as u16);
        assert_eq!(restored.steer_coeff_q14, i16::MAX as u16);
        assert_eq!(restored.vd_max, MAX_VD);
    }

    #[test]
    fn tuning_keys_respect_their_bounds() {
        let mut params = Parameters::defaults();
        assert_eq!(Tunable::from_name("iqkp"), Some(Tunable::IqKp));
        assert_eq!(Tunable::from_name("missing"), None);
        assert_eq!(Tunable::IMax.set(&mut params, u32::MAX), I_MAX_Q4 as u16);
        assert_eq!(Tunable::IMax.get(&params), I_MAX_Q4 as u16);
        assert_eq!(Tunable::IqKp.set(&mut params, 4_000), 4_000);
        assert_eq!(Tunable::IqKp.get(&params), 4_000);
        assert!(Tunable::IqKp.is_actuator());
        assert!(!Tunable::SteerCoeff.is_actuator());
    }
}
