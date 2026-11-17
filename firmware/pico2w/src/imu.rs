//! Async I2C driver for the BNO055 inertial measurement unit.
//!
//! Register addresses, operation modes and value decoding live in
//! [`jbot_pico_core::bno055`]; this module performs the I2C transfers and the
//! datasheet-mandated start-up sequence. It deliberately keeps the
//! magnetometer disabled (`IMUPLUS`) because the motors and the DC-DC converter
//! disturb the magnetic field on this platform.

use embassy_rp::i2c::{Async, Error as I2cError, I2c, Instance};
use embassy_time::Timer;
use jbot_pico_core::bno055::{self, reg, AngularRate, Calibration, Euler};

/// Address selected by the module's address strap. The bench module answers on
/// the high address.
pub const ADDRESS: u8 = bno055::I2C_ADDR_HIGH;

/// Delay after a soft reset before the device answers again.
const RESET_SETTLE_MS: u64 = bno055::RESET_DELAY_MS;
/// Delay after a mode change before the device produces fused data.
const MODE_SETTLE_MS: u64 = 20;
/// Delay after entering configuration mode.
const CONFIG_SETTLE_MS: u64 = 25;
/// Number of chip-id polls after a reset.
const READY_RETRIES: u32 = 20;
/// Delay between chip-id polls after a reset.
const READY_POLL_MS: u64 = 50;

/// Errors reported by the driver.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum ImuError {
    /// The I2C transfer failed.
    I2c(I2cError),
    /// The device answered but reported an unexpected chip id.
    WrongChipId(u8),
    /// The device did not answer after a reset.
    NotReady,
}

impl From<I2cError> for ImuError {
    fn from(error: I2cError) -> Self {
        Self::I2c(error)
    }
}

impl ImuError {
    /// Stable code for telemetry: 0 none, 1 I2C, 2 wrong chip id, 3 not ready.
    pub fn code(self) -> u32 {
        match self {
            Self::I2c(_) => 1,
            Self::WrongChipId(_) => 2,
            Self::NotReady => 3,
        }
    }
}

/// One fused measurement.
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuSample {
    /// Fused Euler angles.
    pub euler: Euler,
    /// Fused angular rate.
    pub gyro: AngularRate,
    /// Current calibration state.
    pub calibration: Calibration,
    /// `SYS_ERR` register, zero when the device reports no error.
    pub sys_err: u8,
}

impl ImuSample {
    /// True when the system, gyroscope and accelerometer are fully calibrated.
    pub fn imu_calibrated(self) -> bool {
        self.calibration.imu_ready()
    }
}

/// BNO055 driver owning one I2C bus.
pub struct Bno055<'d, T: Instance> {
    i2c: I2c<'d, T, Async>,
    address: u8,
}

impl<'d, T: Instance> Bno055<'d, T> {
    /// Wrap an already configured async I2C bus.
    pub fn new(i2c: I2c<'d, T, Async>, address: u8) -> Self {
        Self { i2c, address }
    }

    async fn read_reg(&mut self, register: u8) -> Result<u8, ImuError> {
        let mut value = [0u8; 1];
        self.i2c
            .write_read_async(self.address, [register], &mut value)
            .await?;
        Ok(value[0])
    }

    async fn read_burst<const N: usize>(&mut self, register: u8) -> Result<[u8; N], ImuError> {
        let mut value = [0u8; N];
        self.i2c
            .write_read_async(self.address, [register], &mut value)
            .await?;
        Ok(value)
    }

    async fn write_reg(&mut self, register: u8, value: u8) -> Result<(), ImuError> {
        self.i2c
            .write_async(self.address, [register, value])
            .await?;
        Ok(())
    }

    /// Configure the device for `IMUPLUS` fusion.
    ///
    /// The sequence follows the datasheet: configuration mode, soft reset,
    /// normal power, default units, then the fusion mode. A device that does
    /// not answer after the reset produces [`ImuError::NotReady`].
    pub async fn init(&mut self) -> Result<(), ImuError> {
        let chip_id = self.read_reg(reg::CHIP_ID).await?;
        if chip_id != bno055::CHIP_ID {
            return Err(ImuError::WrongChipId(chip_id));
        }

        self.write_reg(reg::OPR_MODE, bno055::OPR_MODE_CONFIG)
            .await?;
        Timer::after_millis(CONFIG_SETTLE_MS).await;

        self.write_reg(reg::SYS_TRIGGER, bno055::SYS_TRIGGER_RESET)
            .await?;
        Timer::after_millis(RESET_SETTLE_MS).await;
        self.wait_ready().await?;

        self.write_reg(reg::PWR_MODE, bno055::PWR_MODE_NORMAL)
            .await?;
        self.write_reg(reg::PAGE_ID, 0x00).await?;
        self.write_reg(reg::UNIT_SEL, bno055::UNIT_SEL_DEFAULT)
            .await?;
        self.write_reg(reg::OPR_MODE, bno055::OPR_MODE_IMUPLUS)
            .await?;
        Timer::after_millis(MODE_SETTLE_MS).await;

        Ok(())
    }

    async fn wait_ready(&mut self) -> Result<(), ImuError> {
        for _ in 0..READY_RETRIES {
            match self.read_reg(reg::CHIP_ID).await {
                Ok(bno055::CHIP_ID) => return Ok(()),
                Ok(other) => return Err(ImuError::WrongChipId(other)),
                Err(_) => Timer::after_millis(READY_POLL_MS).await,
            }
        }
        Err(ImuError::NotReady)
    }

    /// Read one fused sample. Four short transfers; the caller owns the rate.
    pub async fn sample(&mut self) -> Result<ImuSample, ImuError> {
        let euler = self.read_burst::<6>(reg::EULER_DATA).await?;
        let gyro = self.read_burst::<6>(reg::GYRO_DATA).await?;
        let calibration = self.read_reg(reg::CALIB_STAT).await?;
        let sys_err = self.read_reg(reg::SYS_ERR).await?;

        Ok(ImuSample {
            euler: bno055::decode_euler(&euler),
            gyro: bno055::decode_gyro(&gyro),
            calibration: Calibration::from_byte(calibration),
            sys_err,
        })
    }
}
