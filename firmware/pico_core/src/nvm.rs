//! Non-volatile settings record for the bridge firmware.
//!
//! The record is fixed-size and carries its own magic, version and CRC, so a
//! blank, erased, or partially rewritten sector decodes to [`None`] instead of
//! a plausible-looking wrong setting. Storing a wrong mount silently is the
//! failure this module exists to prevent: the attitude mapping is geometry,
//! not a tuning choice, and the operator must not have to re-derive it after
//! every power cycle.
//!
//! This module is pure: it owns the byte layout and the validation rules, the
//! firmware crate owns the flash driver and the erase/write timing.

use crate::mount::Mount;

/// Size of one flash erase block on the RP2350 QSPI flash, bytes.
pub const ERASE_BLOCK: usize = 4096;
/// Encoded record size, bytes. Far below [`ERASE_BLOCK`] by design.
pub const RECORD_LEN: usize = 32;
/// Record magic: the ASCII bytes `JBNV` stored little-endian, so a hex dump
/// of a written sector starts with `J B N V`.
pub const MAGIC: u32 = 0x564E_424A;
/// Record format version. A different version is refused, not migrated.
///
/// Version 2 invalidates the pre-resolder record, which held `mount 1`
/// (flip-x) for the old IMU orientation. After the module was re-soldered the
/// fused Euler angles match the physical attitude, so a refused record falls
/// back to the compiled `Mount::Normal` default instead of silently restoring
/// a wrong 180-degree flip.
///
/// Version 3 adds three measured pose pitch references and invalidates v2
/// records; a refused record falls back to defaults and the operator captures
/// the pose references again.
pub const VERSION: u16 = 3;
/// Largest accepted magnitude of a stored zero/level angle, degrees.
///
/// The zero offsets are small by construction and the level reference is the
/// captured standing pose, so anything beyond a right angle is corruption.
const MAX_ANGLE_DEG: f32 = 90.0;
/// Largest accepted magnitude of a stored pose reference, degrees.
///
/// Pose references include the inverted poses, so they may exceed a right
/// angle; the Euler reading stays inside a half turn.
const MAX_POSE_ANGLE_DEG: f32 = 180.0;
/// Sentinel stored in a pose slot that has not been measured.
const POSE_UNSET: i16 = i16::MIN;

/// Offset of the magic word inside the encoded record.
const OFF_MAGIC: usize = 0;
/// Offset of the format version inside the encoded record.
const OFF_VERSION: usize = 4;
/// Offset of the mount selector inside the encoded record.
const OFF_MOUNT: usize = 6;
/// Offset of the reserved flag byte inside the encoded record.
const OFF_FLAGS: usize = 7;
/// Offset of the pitch zero offset inside the encoded record.
const OFF_ZERO_PITCH: usize = 8;
/// Offset of the roll zero offset inside the encoded record.
const OFF_ZERO_ROLL: usize = 12;
/// Offset of the level reference inside the encoded record.
const OFF_LEVEL: usize = 16;
/// Offset of the on-back pitch reference, deci-degrees.
const OFF_POSE_BACK: usize = 20;
/// Offset of the face-down pitch reference, deci-degrees.
const OFF_POSE_FACE: usize = 22;
/// Offset of the vertical pitch reference, deci-degrees.
const OFF_POSE_HOME: usize = 24;
/// Offset of the CRC over the bytes before it.
const OFF_CRC: usize = 28;
/// Number of bytes covered by the CRC.
const CRC_SPAN: usize = OFF_CRC;

/// Settings restored at boot.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Record {
    /// BNO055 mount selector, `0..=6`.
    pub mount: u8,
    /// Pitch zero offset, radians.
    pub zero_pitch_rad: f32,
    /// Roll zero offset, radians.
    pub zero_roll_rad: f32,
    /// Captured level reference, radians.
    pub level_rad: f32,
    /// Measured pitch of the on-back pose, degrees.
    pub pose_back_deg: Option<f32>,
    /// Measured pitch of the face-down pose, degrees.
    pub pose_face_deg: Option<f32>,
    /// Measured pitch of the vertical pose, degrees.
    pub pose_home_deg: Option<f32>,
}

impl Record {
    /// Build a record, refusing values that could not have been captured by a
    /// healthy sensor. Returns [`None`] instead of clamping: a corrupted
    /// capture must not be stored as a slightly different setting.
    pub fn new(
        mount: u8,
        zero_pitch_rad: f32,
        zero_roll_rad: f32,
        level_rad: f32,
        pose_back_deg: Option<f32>,
        pose_face_deg: Option<f32>,
        pose_home_deg: Option<f32>,
    ) -> Option<Self> {
        let record = Self {
            mount,
            zero_pitch_rad,
            zero_roll_rad,
            level_rad,
            pose_back_deg,
            pose_face_deg,
            pose_home_deg,
        };
        record.is_valid().then_some(record)
    }

    /// Whether every field is inside the range a real capture can produce.
    pub fn is_valid(&self) -> bool {
        self.mount < Mount::COUNT as u8
            && angle_ok(self.zero_pitch_rad)
            && angle_ok(self.zero_roll_rad)
            && angle_ok(self.level_rad)
            && pose_ok(self.pose_back_deg)
            && pose_ok(self.pose_face_deg)
            && pose_ok(self.pose_home_deg)
    }

    /// Encode the record into its fixed-size flash image.
    pub fn encode(&self) -> [u8; RECORD_LEN] {
        let mut bytes = [0xFFu8; RECORD_LEN];
        bytes[OFF_MAGIC..OFF_MAGIC + 4].copy_from_slice(&MAGIC.to_le_bytes());
        bytes[OFF_VERSION..OFF_VERSION + 2].copy_from_slice(&VERSION.to_le_bytes());
        bytes[OFF_MOUNT] = self.mount;
        bytes[OFF_FLAGS] = 0;
        bytes[OFF_ZERO_PITCH..OFF_ZERO_PITCH + 4]
            .copy_from_slice(&self.zero_pitch_rad.to_le_bytes());
        bytes[OFF_ZERO_ROLL..OFF_ZERO_ROLL + 4].copy_from_slice(&self.zero_roll_rad.to_le_bytes());
        bytes[OFF_LEVEL..OFF_LEVEL + 4].copy_from_slice(&self.level_rad.to_le_bytes());
        bytes[OFF_POSE_BACK..OFF_POSE_BACK + 2]
            .copy_from_slice(&encode_pose(self.pose_back_deg).to_le_bytes());
        bytes[OFF_POSE_FACE..OFF_POSE_FACE + 2]
            .copy_from_slice(&encode_pose(self.pose_face_deg).to_le_bytes());
        bytes[OFF_POSE_HOME..OFF_POSE_HOME + 2]
            .copy_from_slice(&encode_pose(self.pose_home_deg).to_le_bytes());
        let crc = crc32(&bytes[..CRC_SPAN]);
        bytes[OFF_CRC..OFF_CRC + 4].copy_from_slice(&crc.to_le_bytes());
        bytes
    }

    /// Decode a flash image. Returns [`None`] for a blank or erased sector, a
    /// foreign image, a version this firmware does not understand, a damaged
    /// payload, or values outside the capture range.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < RECORD_LEN {
            return None;
        }
        let magic = u32::from_le_bytes(bytes[OFF_MAGIC..OFF_MAGIC + 4].try_into().ok()?);
        if magic != MAGIC {
            return None;
        }
        let version = u16::from_le_bytes(bytes[OFF_VERSION..OFF_VERSION + 2].try_into().ok()?);
        if version != VERSION {
            return None;
        }
        let stored_crc = u32::from_le_bytes(bytes[OFF_CRC..OFF_CRC + 4].try_into().ok()?);
        if stored_crc != crc32(&bytes[..CRC_SPAN]) {
            return None;
        }
        let record = Self {
            mount: bytes[OFF_MOUNT],
            zero_pitch_rad: f32::from_le_bytes(
                bytes[OFF_ZERO_PITCH..OFF_ZERO_PITCH + 4].try_into().ok()?,
            ),
            zero_roll_rad: f32::from_le_bytes(
                bytes[OFF_ZERO_ROLL..OFF_ZERO_ROLL + 4].try_into().ok()?,
            ),
            level_rad: f32::from_le_bytes(bytes[OFF_LEVEL..OFF_LEVEL + 4].try_into().ok()?),
            pose_back_deg: decode_pose(read_pose(bytes, OFF_POSE_BACK)),
            pose_face_deg: decode_pose(read_pose(bytes, OFF_POSE_FACE)),
            pose_home_deg: decode_pose(read_pose(bytes, OFF_POSE_HOME)),
        };
        record.is_valid().then_some(record)
    }
}

/// Whether an angle in radians is finite and inside [`MAX_ANGLE_DEG`].
fn angle_ok(rad: f32) -> bool {
    rad.is_finite() && rad.abs() <= MAX_ANGLE_DEG.to_radians()
}

/// Whether an optional pose reference is finite and inside a half turn.
fn pose_ok(deg: Option<f32>) -> bool {
    deg.is_none_or(|value| value.is_finite() && value.abs() <= MAX_POSE_ANGLE_DEG)
}

/// Store a pose angle as deci-degrees, reserving the low end for "unset".
fn encode_pose(deg: Option<f32>) -> i16 {
    match deg {
        // `f32::round` lives in std, so round half away from zero manually.
        Some(value) => {
            let scaled = value * 10.0;
            let rounded = if scaled >= 0.0 {
                scaled + 0.5
            } else {
                scaled - 0.5
            };
            rounded as i16
        }
        None => POSE_UNSET,
    }
}

/// Read a pose slot from the encoded image.
fn read_pose(bytes: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes(
        bytes[offset..offset + 2]
            .try_into()
            .expect("pose slot is in range"),
    )
}

/// Turn an encoded pose slot back into degrees, or `None` when unset.
fn decode_pose(raw: i16) -> Option<f32> {
    if raw == POSE_UNSET {
        None
    } else {
        Some(f32::from(raw) / 10.0)
    }
}

/// CRC-32/ISO-HDLC, bitwise: the record is 28 bytes, so a table would cost
/// more flash than it saves.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Record {
        Record {
            mount: Mount::RotYp.as_u32() as u8,
            zero_pitch_rad: 0.021,
            zero_roll_rad: -0.004,
            level_rad: -0.012,
            pose_back_deg: Some(-99.8),
            pose_face_deg: Some(86.0),
            pose_home_deg: Some(-0.2),
        }
    }

    #[test]
    fn round_trip() {
        let record = sample();
        assert_eq!(Record::decode(&record.encode()), Some(record));
    }

    #[test]
    fn encoded_record_is_self_describing() {
        let bytes = sample().encode();
        assert_eq!(&bytes[OFF_MAGIC..OFF_MAGIC + 4], b"JBNV");
        assert_eq!(u16::from_le_bytes(bytes[4..6].try_into().unwrap()), VERSION);
        assert_eq!(bytes[OFF_MOUNT], 5);
    }

    #[test]
    fn erased_sector_decodes_to_none() {
        assert_eq!(Record::decode(&[0xFFu8; RECORD_LEN]), None);
        assert_eq!(Record::decode(&[0u8; RECORD_LEN]), None);
    }

    #[test]
    fn damaged_payload_is_refused() {
        let mut bytes = sample().encode();
        bytes[OFF_LEVEL] ^= 0x01;
        assert_eq!(Record::decode(&bytes), None);
    }

    #[test]
    fn wrong_version_is_refused() {
        let mut bytes = sample().encode();
        bytes[OFF_VERSION..OFF_VERSION + 2].copy_from_slice(&(VERSION + 1).to_le_bytes());
        let crc = crc32(&bytes[..CRC_SPAN]);
        bytes[OFF_CRC..OFF_CRC + 4].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(Record::decode(&bytes), None);
    }

    #[test]
    fn out_of_range_mount_is_refused() {
        let mut bytes = sample().encode();
        bytes[OFF_MOUNT] = Mount::COUNT as u8;
        let crc = crc32(&bytes[..CRC_SPAN]);
        bytes[OFF_CRC..OFF_CRC + 4].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(Record::decode(&bytes), None);
    }

    #[test]
    fn non_finite_angle_is_refused() {
        assert_eq!(Record::new(0, f32::NAN, 0.0, 0.0, None, None, None), None);
        assert_eq!(
            Record::new(0, 0.0, f32::INFINITY, 0.0, None, None, None),
            None
        );
    }

    #[test]
    fn unset_pose_round_trips() {
        let record = Record::new(0, 0.0, 0.0, 0.0, None, Some(86.0), None).unwrap();
        let decoded = Record::decode(&record.encode()).unwrap();
        assert_eq!(decoded.pose_back_deg, None);
        assert_eq!(decoded.pose_face_deg, Some(86.0));
        assert_eq!(decoded.pose_home_deg, None);
    }

    #[test]
    fn out_of_range_pose_is_refused() {
        assert_eq!(Record::new(0, 0.0, 0.0, 0.0, Some(181.0), None, None), None);
        assert_eq!(
            Record::new(0, 0.0, 0.0, 0.0, None, None, Some(f32::NAN)),
            None
        );
    }

    #[test]
    fn short_buffer_is_refused() {
        assert_eq!(Record::decode(&[0u8; RECORD_LEN - 1]), None);
    }
}
