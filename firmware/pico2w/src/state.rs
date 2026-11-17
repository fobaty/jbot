//! Shared runtime state and counters.
//!
//! The tasks in [`crate::main`] own the hardware; this module is the small,
//! lock-free surface they share. Atomics keep the control loop free of locks
//! and let the USB console read the same values without disturbing it.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Nominal control loop rate, in hertz.
pub const CONTROL_HZ: u32 = 200;
/// Highest accepted telemetry rate, in hertz.
pub const MAX_STREAM_HZ: u32 = CONTROL_HZ;
/// Default telemetry rate: one line per second.
const DEFAULT_STREAM_HZ: u32 = 1;

/// One telemetry snapshot, published by the control task.
#[derive(Clone, Copy, Default)]
pub struct Telemetry {
    /// Control ticks since boot.
    pub control_steps: u32,
    /// Measured control loop rate, hertz.
    pub loop_hz: u32,
    /// The IMU sample used for this tick was fresh.
    pub imu_fresh: bool,
    /// System, gyro and accelerometer calibration is complete.
    pub imu_calibrated: bool,
    /// `CALIB_STAT` system byte, `0..=3`.
    pub imu_system_cal: u8,
    /// Pitch used by the control law, in degrees.
    pub pitch_deg: f32,
    /// Pitch rate used by the control law, in degrees per second.
    pub gyro_y_dps: f32,
    /// `SYS_ERR` register, zero when the BNO055 reports no fault.
    pub sys_err: u8,
    /// At least one mainboard feedback frame has been received.
    pub feedback_present: bool,
    /// Left wheel speed, rpm.
    pub speed_left_rpm: i16,
    /// Right wheel speed, rpm.
    pub speed_right_rpm: i16,
    /// Pack voltage, volts.
    pub battery_v: f32,
    /// Board temperature, degrees Celsius.
    pub board_temp_c: f32,
    /// Age of the last feedback frame, milliseconds.
    pub feedback_age_ms: u32,
    /// Latched emergency stop is active.
    pub estop: bool,
}

/// Latest snapshot, refreshed on every control step.
pub static TELEMETRY: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
/// Ask the IMU task to re-run the BNO055 start-up sequence.
pub static REINIT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Telemetry rate in hertz; zero disables the stream.
static STREAM_HZ: AtomicU32 = AtomicU32::new(DEFAULT_STREAM_HZ);
/// Pitch trim in radians, stored as `f32` bits.
static PITCH_TRIM_BITS: AtomicU32 = AtomicU32::new(0);
/// Latched emergency stop.
static ESTOP: AtomicBool = AtomicBool::new(false);
/// Measured control loop rate, hertz.
static LOOP_HZ: AtomicU32 = AtomicU32::new(0);

/// IMU samples delivered to the control task.
pub static IMU_SAMPLES: AtomicU32 = AtomicU32::new(0);
/// IMU read failures.
pub static IMU_ERRORS: AtomicU32 = AtomicU32::new(0);
/// Code of the last IMU error; zero means none since the last reset.
pub static IMU_LAST_ERROR: AtomicU32 = AtomicU32::new(0);
/// Valid mainboard feedback frames parsed.
pub static FB_FRAMES: AtomicU32 = AtomicU32::new(0);
/// Control ticks since boot.
pub static CONTROL_STEPS: AtomicU32 = AtomicU32::new(0);
/// Bytes discarded while resynchronising the feedback parser.
pub static FB_DISCARDED: AtomicU32 = AtomicU32::new(0);

/// Increment a counter.
pub fn count(counter: &AtomicU32) {
    counter.fetch_add(1, Ordering::Relaxed);
}

/// Read a counter.
pub fn read(counter: &AtomicU32) -> u32 {
    counter.load(Ordering::Relaxed)
}

/// Telemetry rate in hertz.
pub fn stream_hz() -> u32 {
    STREAM_HZ.load(Ordering::Relaxed)
}

/// Set the telemetry rate, clamped to [`MAX_STREAM_HZ`], and return the value
/// that was stored.
pub fn set_stream_hz(hz: u32) -> u32 {
    let clamped = hz.min(MAX_STREAM_HZ);
    STREAM_HZ.store(clamped, Ordering::Relaxed);
    clamped
}

/// Pitch trim in radians.
pub fn pitch_trim_rad() -> f32 {
    f32::from_bits(PITCH_TRIM_BITS.load(Ordering::Relaxed))
}

/// Set the pitch trim in radians.
pub fn set_pitch_trim_rad(rad: f32) {
    PITCH_TRIM_BITS.store(rad.to_bits(), Ordering::Relaxed);
}

/// Emergency stop latch.
pub fn estop() -> bool {
    ESTOP.load(Ordering::Relaxed)
}

/// Set or clear the emergency stop latch.
pub fn set_estop(active: bool) {
    ESTOP.store(active, Ordering::Relaxed);
}

/// Measured control loop rate, hertz.
pub fn loop_hz() -> u32 {
    LOOP_HZ.load(Ordering::Relaxed)
}

/// Publish the measured control loop rate.
pub fn set_loop_hz(hz: u32) {
    LOOP_HZ.store(hz, Ordering::Relaxed);
}

/// Zero every diagnostic counter. Settings are left alone.
pub fn reset_counters() {
    IMU_SAMPLES.store(0, Ordering::Relaxed);
    IMU_ERRORS.store(0, Ordering::Relaxed);
    IMU_LAST_ERROR.store(0, Ordering::Relaxed);
    FB_FRAMES.store(0, Ordering::Relaxed);
    FB_DISCARDED.store(0, Ordering::Relaxed);
}
