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
    /// Roll from the fused BNO055 Euler frame, in degrees.
    pub roll_deg: f32,
    /// Heading from the fused BNO055 Euler frame, in degrees.
    pub heading_deg: f32,
    /// Pitch rate used by the control law, in degrees per second.
    pub gyro_y_dps: f32,
    /// Roll rate from the BNO055, in degrees per second.
    pub gyro_x_dps: f32,
    /// Yaw rate from the BNO055, in degrees per second.
    pub gyro_z_dps: f32,
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
    /// Forward/backward speed setpoint, meters per second.
    pub speed_setpoint_mps: f32,
    /// Steering setpoint, radians per second.
    pub steer_setpoint_rps: f32,
    /// Mount selector of the BNO055: 0 normal, 1 flip-x, 2 flip-y.
    pub mount: u8,
    /// Applied pitch zero offset, degrees.
    pub zero_offset_deg: f32,
    /// Applied roll zero offset, degrees.
    pub roll_zero_offset_deg: f32,
    /// Applied balance (level) reference, degrees.
    pub level_deg: f32,
    /// Operator-controlled arm latch; torque is produced only when true.
    pub armed: bool,
}

/// Latest snapshot, refreshed on every control step.
pub static TELEMETRY: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
/// Ask the IMU task to re-run the BNO055 start-up sequence.
pub static REINIT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Telemetry rate in hertz; zero disables the stream.
static STREAM_HZ: AtomicU32 = AtomicU32::new(DEFAULT_STREAM_HZ);
/// Pitch trim in radians, stored as `f32` bits.
static PITCH_TRIM_BITS: AtomicU32 = AtomicU32::new(0);
/// Desired forward/backward speed setpoint in meters per second, stored as `f32` bits.
static SPEED_SETPOINT_BITS: AtomicU32 = AtomicU32::new(0);
/// Desired steering/yaw rate setpoint in radians per second, stored as `f32` bits.
static STEER_SETPOINT_BITS: AtomicU32 = AtomicU32::new(0);
/// Latched emergency stop.
static ESTOP: AtomicBool = AtomicBool::new(false);
/// Measured control loop rate, hertz.
static LOOP_HZ: AtomicU32 = AtomicU32::new(0);
/// BNO055 mount selector: 0 normal, 1 flip-x, 2 flip-y, 3..6 90-degree
/// rotations.
static MOUNT: AtomicU32 = AtomicU32::new(0);
/// Whether the latest IMU sample used by the control loop is fresh.
static IMU_FRESH: AtomicBool = AtomicBool::new(false);
/// Operator-controlled arm latch for the balance law. Cleared at boot, by
/// the e-stop, by `arm`'s refusal conditions, and by any change of the
/// mount or the calibration offsets. The torque path is inert until armed.
static ARMED: AtomicBool = AtomicBool::new(false);
/// Set once the zero offset has been captured this session.
static ZERO_CAPTURED: AtomicBool = AtomicBool::new(false);
/// Set once the level reference has been captured this session.
static LEVEL_CAPTURED: AtomicBool = AtomicBool::new(false);
/// Pitch zero offset in radians, stored as `f32` bits.
static ZERO_PITCH_BITS: AtomicU32 = AtomicU32::new(0);
/// Roll zero offset in radians, stored as `f32` bits.
static ZERO_ROLL_BITS: AtomicU32 = AtomicU32::new(0);
/// Latest mapped (pre-zero) pitch in degrees, stored as `f32` bits. Written
/// by the control task, read by the console to capture the zero offset.
static LATEST_MAPPED_PITCH_BITS: AtomicU32 = AtomicU32::new(0);
/// Latest mapped (pre-zero) roll in degrees, stored as `f32` bits.
static LATEST_MAPPED_ROLL_BITS: AtomicU32 = AtomicU32::new(0);

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

/// Forward/backward speed setpoint in meters per second.
pub fn speed_setpoint_mps() -> f32 {
    f32::from_bits(SPEED_SETPOINT_BITS.load(Ordering::Relaxed))
}

/// Set the forward/backward speed setpoint in meters per second.
pub fn set_speed_setpoint_mps(mps: f32) {
    SPEED_SETPOINT_BITS.store(mps.to_bits(), Ordering::Relaxed);
}

/// Steering setpoint in radians per second.
pub fn steer_setpoint_rps() -> f32 {
    f32::from_bits(STEER_SETPOINT_BITS.load(Ordering::Relaxed))
}

/// Set the steering setpoint in radians per second.
pub fn set_steer_setpoint_rps(rps: f32) {
    STEER_SETPOINT_BITS.store(rps.to_bits(), Ordering::Relaxed);
}

/// Emergency stop latch.
pub fn estop() -> bool {
    ESTOP.load(Ordering::Relaxed)
}

/// Set or clear the emergency stop latch. Latching the stop also disarms
/// the balance law; clearing the stop never re-arms it.
pub fn set_estop(active: bool) {
    ESTOP.store(active, Ordering::Relaxed);
    if active {
        ARMED.store(false, Ordering::Relaxed);
    }
}

/// Measured control loop rate, hertz.
pub fn loop_hz() -> u32 {
    LOOP_HZ.load(Ordering::Relaxed)
}

/// Publish the measured control loop rate.
pub fn set_loop_hz(hz: u32) {
    LOOP_HZ.store(hz, Ordering::Relaxed);
}

/// BNO055 mount selector.
pub fn mount() -> u32 {
    MOUNT.load(Ordering::Relaxed)
}

/// Set the BNO055 mount selector.
pub fn set_mount(mount: u32) {
    MOUNT.store(mount, Ordering::Relaxed);
}

/// Freshness of the latest IMU sample, as used by the control loop.
pub fn imu_fresh() -> bool {
    IMU_FRESH.load(Ordering::Relaxed)
}

/// Publish the IMU freshness flag.
pub fn set_imu_fresh(fresh: bool) {
    IMU_FRESH.store(fresh, Ordering::Relaxed);
}

/// True once the operator has armed the balance law this session.
pub fn armed() -> bool {
    ARMED.load(Ordering::Relaxed)
}

/// Arm or disarm the balance law.
pub fn set_armed(armed: bool) {
    ARMED.store(armed, Ordering::Relaxed);
}

/// True once the zero offset has been captured this session.
pub fn zero_captured() -> bool {
    ZERO_CAPTURED.load(Ordering::Relaxed)
}

/// Mark the zero offset as captured (or not, when calibration is invalidated).
pub fn set_zero_captured(captured: bool) {
    ZERO_CAPTURED.store(captured, Ordering::Relaxed);
}

/// True once the level reference has been captured this session.
pub fn level_captured() -> bool {
    LEVEL_CAPTURED.load(Ordering::Relaxed)
}

/// Mark the level reference as captured (or not, when calibration is invalidated).
pub fn set_level_captured(captured: bool) {
    LEVEL_CAPTURED.store(captured, Ordering::Relaxed);
}

/// Pitch zero offset in radians.
pub fn zero_pitch_rad() -> f32 {
    f32::from_bits(ZERO_PITCH_BITS.load(Ordering::Relaxed))
}

/// Roll zero offset in radians.
pub fn zero_roll_rad() -> f32 {
    f32::from_bits(ZERO_ROLL_BITS.load(Ordering::Relaxed))
}

/// Set the pitch zero offset in radians.
pub fn set_zero_pitch_rad(rad: f32) {
    ZERO_PITCH_BITS.store(rad.to_bits(), Ordering::Relaxed);
}

/// Set the roll zero offset in radians.
pub fn set_zero_roll_rad(rad: f32) {
    ZERO_ROLL_BITS.store(rad.to_bits(), Ordering::Relaxed);
}

/// Publish the latest mapped attitude (pre-zero), degrees.
pub fn set_latest_mapped_attitude(pitch_deg: f32, roll_deg: f32) {
    LATEST_MAPPED_PITCH_BITS.store(pitch_deg.to_bits(), Ordering::Relaxed);
    LATEST_MAPPED_ROLL_BITS.store(roll_deg.to_bits(), Ordering::Relaxed);
}

/// Latest mapped attitude (pre-zero) in degrees.
pub fn latest_mapped_attitude() -> (f32, f32) {
    (
        f32::from_bits(LATEST_MAPPED_PITCH_BITS.load(Ordering::Relaxed)),
        f32::from_bits(LATEST_MAPPED_ROLL_BITS.load(Ordering::Relaxed)),
    )
}

/// Zero every diagnostic counter. Settings are left alone.
pub fn reset_counters() {
    IMU_SAMPLES.store(0, Ordering::Relaxed);
    IMU_ERRORS.store(0, Ordering::Relaxed);
    IMU_LAST_ERROR.store(0, Ordering::Relaxed);
    FB_FRAMES.store(0, Ordering::Relaxed);
    FB_DISCARDED.store(0, Ordering::Relaxed);
}
