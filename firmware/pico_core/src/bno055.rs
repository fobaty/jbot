//! BNO055 register map and decoding helpers.
//!
//! The driver itself lives in the firmware crate because it owns the I2C
//! peripheral. This module owns the parts worth testing on the host: device
//! identification, the register addresses used by the driver, and the scaling
//! of the raw sensor words.
//!
//! The device is configured in `IMUPLUS` mode: the magnetometer picks up motor
//! and DC-DC fields on this platform, so it is deliberately excluded from the
//! fusion. The accelerometer and the gyroscope are fused on-chip at 100 Hz; the
//! Pico integrates the calibrated gyro between samples at 200 Hz.

/// I2C address with `ADR` tied low.
pub const I2C_ADDR_LOW: u8 = 0x28;
/// I2C address with `ADR` tied high.
pub const I2C_ADDR_HIGH: u8 = 0x29;
/// Expected value of the `CHIP_ID` register.
pub const CHIP_ID: u8 = 0xA0;

/// Register map, only the registers the firmware touches.
pub mod reg {
    /// `CHIP_ID`, read-only, expected `0xA0`.
    pub const CHIP_ID: u8 = 0x00;
    /// `ACC_ID`, expected `0xFB`.
    pub const ACC_ID: u8 = 0x01;
    /// `MAG_ID`, expected `0x32`.
    pub const MAG_ID: u8 = 0x02;
    /// `GYR_ID`, expected `0x0F`.
    pub const GYR_ID: u8 = 0x03;
    /// `PAGE_ID`, 0 after reset.
    pub const PAGE_ID: u8 = 0x07;
    /// Gyroscope X/Y/Z, signed 16-bit little-endian, 1/16 degree per second.
    pub const GYRO_DATA: u8 = 0x14;
    /// Euler heading/roll/pitch, signed 16-bit little-endian, 1/16 degree.
    pub const EULER_DATA: u8 = 0x1A;
    /// Linear acceleration X/Y/Z, signed 16-bit little-endian, 1/100 m/s^2.
    pub const LINEAR_ACCEL_DATA: u8 = 0x28;
    /// Gravity vector X/Y/Z, signed 16-bit little-endian, 1/100 m/s^2.
    pub const GRAVITY_DATA: u8 = 0x2E;
    /// Calibration status, four 2-bit fields.
    pub const CALIB_STAT: u8 = 0x35;
    /// System status, bit field.
    pub const SYS_STATUS: u8 = 0x39;
    /// System error, 0 when healthy.
    pub const SYS_ERR: u8 = 0x3A;
    /// Unit selection.
    pub const UNIT_SEL: u8 = 0x3B;
    /// Operation mode.
    pub const OPR_MODE: u8 = 0x3D;
    /// Power mode.
    pub const PWR_MODE: u8 = 0x3E;
    /// System trigger: reset, self-test, external crystal.
    pub const SYS_TRIGGER: u8 = 0x3F;
}

/// `OPR_MODE` value for accelerometer-gyro fusion without the magnetometer.
pub const OPR_MODE_IMUPLUS: u8 = 0x08;
/// `OPR_MODE` value for the configuration mode.
pub const OPR_MODE_CONFIG: u8 = 0x00;
/// `PWR_MODE` value for normal operation.
pub const PWR_MODE_NORMAL: u8 = 0x00;
/// `UNIT_SEL` value: m/s^2, degrees per second, degrees Celsius, degrees.
pub const UNIT_SEL_DEFAULT: u8 = 0x00;
/// `SYS_TRIGGER` bit that performs a reset.
pub const SYS_TRIGGER_RESET: u8 = 0x20;
/// `SYS_TRIGGER` bit that runs the built-in self-test.
pub const SYS_TRIGGER_SELF_TEST: u8 = 0x01;

/// Time the device needs after a reset before it accepts configuration.
pub const RESET_DELAY_MS: u64 = 650;

/// Euler angles decoded from the device.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Euler {
    /// Heading in degrees, `0..360`.
    pub heading_deg: f32,
    /// Roll in degrees.
    pub roll_deg: f32,
    /// Pitch in degrees, positive when the nose points up.
    pub pitch_deg: f32,
}

/// Angular rate decoded from the device.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AngularRate {
    /// Rotation around X, degrees per second.
    pub x_dps: f32,
    /// Rotation around Y, degrees per second.
    pub y_dps: f32,
    /// Rotation around Z, degrees per second.
    pub z_dps: f32,
}

/// Per-sensor calibration state extracted from `CALIB_STAT`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Calibration {
    /// System calibration, 0 (uncalibrated) to 3.
    pub system: u8,
    /// Gyroscope calibration, 0 to 3.
    pub gyro: u8,
    /// Accelerometer calibration, 0 to 3.
    pub accel: u8,
    /// Magnetometer calibration, 0 to 3.
    pub mag: u8,
}

impl Calibration {
    /// Decode the packed `CALIB_STAT` byte.
    pub fn from_byte(byte: u8) -> Self {
        Self {
            system: (byte >> 6) & 0x03,
            gyro: (byte >> 4) & 0x03,
            accel: (byte >> 2) & 0x03,
            mag: byte & 0x03,
        }
    }

    /// True when the sensors used by `IMUPLUS` are fully calibrated.
    pub fn imu_ready(self) -> bool {
        self.system == 3 && self.gyro == 3 && self.accel == 3
    }
}

/// Scale applied to every Euler word: the device reports 1/16 degree.
const EULER_LSB_DEG: f32 = 1.0 / 16.0;
/// Scale applied to every gyroscope word: the device reports 1/16 degree/s.
const GYRO_LSB_DPS: f32 = 1.0 / 16.0;

/// Decode the six `EULER_DATA` bytes (`heading`, `roll`, `pitch`, in order).
pub fn decode_euler(bytes: &[u8; 6]) -> Euler {
    Euler {
        heading_deg: read_i16(&bytes[0..2]) as f32 * EULER_LSB_DEG,
        roll_deg: read_i16(&bytes[2..4]) as f32 * EULER_LSB_DEG,
        pitch_deg: read_i16(&bytes[4..6]) as f32 * EULER_LSB_DEG,
    }
}

/// Decode the six `GYRO_DATA` bytes (`x`, `y`, `z`, in order).
pub fn decode_gyro(bytes: &[u8; 6]) -> AngularRate {
    AngularRate {
        x_dps: read_i16(&bytes[0..2]) as f32 * GYRO_LSB_DPS,
        y_dps: read_i16(&bytes[2..4]) as f32 * GYRO_LSB_DPS,
        z_dps: read_i16(&bytes[4..6]) as f32 * GYRO_LSB_DPS,
    }
}

fn read_i16(bytes: &[u8]) -> i16 {
    i16::from_le_bytes([bytes[0], bytes[1]])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn words(values: [i16; 3]) -> [u8; 6] {
        let mut bytes = [0u8; 6];
        for (slot, value) in bytes.as_chunks_mut::<2>().0.iter_mut().zip(values) {
            slot.copy_from_slice(&value.to_le_bytes());
        }
        bytes
    }

    #[test]
    fn euler_is_scaled_by_one_sixteenth_degree() {
        // 90.0 degrees is 1440 LSB.
        let euler = decode_euler(&words([1440, -160, 320]));
        assert!((euler.heading_deg - 90.0).abs() < 1e-6);
        assert!((euler.roll_deg + 10.0).abs() < 1e-6);
        assert!((euler.pitch_deg - 20.0).abs() < 1e-6);
    }

    #[test]
    fn gyro_is_scaled_by_one_sixteenth_dps() {
        let rate = decode_gyro(&words([16, -32, 0]));
        assert!((rate.x_dps - 1.0).abs() < 1e-6);
        assert!((rate.y_dps + 2.0).abs() < 1e-6);
        assert!(rate.z_dps.abs() < 1e-6);
    }

    #[test]
    fn calibration_fields_are_unpacked() {
        let calibration = Calibration::from_byte(0b11_10_11_00);
        assert_eq!(calibration.system, 3);
        assert_eq!(calibration.gyro, 2);
        assert_eq!(calibration.accel, 3);
        assert_eq!(calibration.mag, 0);
    }

    #[test]
    fn imu_ready_ignores_the_magnetometer() {
        assert!(Calibration::from_byte(0b11_11_11_00).imu_ready());
        assert!(!Calibration::from_byte(0b10_11_11_00).imu_ready());
        assert!(!Calibration::from_byte(0b11_10_11_00).imu_ready());
    }
}
