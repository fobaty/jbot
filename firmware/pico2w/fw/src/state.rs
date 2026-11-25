//! Shared runtime state and counters.
//!
//! The tasks in [`crate::main`] own the hardware; this module is the small,
//! lock-free surface they share. Atomics keep the control loop free of locks
//! and let the USB console read the same values without disturbing it.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use jbot_pico_core::calibration::MAX_BREAKAWAY_UNITS;
use jbot_pico_core::control::{StopReason, Tuning, MAX_DSCALE, MAX_PSCALE};
use jbot_pico_core::hardened::ProtocolMode;
use jbot_pico_core::mount::Mount;
use jbot_pico_core::plant::PlantParams;
use jbot_pico_core::spin::SpinPair;

/// Proportional pitch gain as a multiple of the stability floor `A1 / A2`.
/// Four is the design point: enough loop gain for a stiff stance without
/// demanding torque the donor drive cannot deliver.
pub const PITCH_MARGIN: f32 = 4.0;
/// Closed-loop pitch damping ratio. Above 1.0 feels stiff but stays calm with
/// the mainboard transport delay in the loop.
pub const PITCH_DAMPING: f32 = 1.6;

/// Balance tuning derived from the identified plant.
///
/// This is the single source of truth for the control gains: nothing else in
/// the firmware carries a gain table. Replacing [`PlantParams::NOMINAL`] with
/// a bench-identified plant changes the whole law coherently.
pub fn tuning() -> Tuning {
    Tuning::from_plant(&PlantParams::NOMINAL, PITCH_MARGIN, PITCH_DAMPING)
}

/// Nominal control loop rate, in hertz.
pub const CONTROL_HZ: u32 = 200;
/// Highest accepted telemetry rate, in hertz.
pub const MAX_STREAM_HZ: u32 = CONTROL_HZ;
/// Default telemetry rate: one line per second.
const DEFAULT_STREAM_HZ: u32 = 1;
/// Whether the STM32 debug bridge is forwarded to the USB console. Bench only.
static DEBUG_BRIDGE: AtomicBool = AtomicBool::new(false);

/// Control-link wire contract. Runtime-only: a bench switch must not leak into
/// the next power-up, and the STM32 side defaults to the same legacy frame.
static PROTOCOL_MODE: AtomicU8 = AtomicU8::new(ProtocolMode::DEFAULT.as_u8());

pub fn set_protocol_mode(mode: ProtocolMode) {
    PROTOCOL_MODE.store(mode.as_u8(), Ordering::Relaxed);
}

pub fn protocol_mode() -> ProtocolMode {
    ProtocolMode::from_u8(PROTOCOL_MODE.load(Ordering::Relaxed)).unwrap_or(ProtocolMode::DEFAULT)
}

/// Enable or disable forwarding of the debug UART to the USB console.
pub fn set_debug_bridge(enabled: bool) {
    DEBUG_BRIDGE.store(enabled, Ordering::Relaxed);
}

/// `true` while the debug UART is forwarded to the USB console.
pub fn debug_bridge_enabled() -> bool {
    DEBUG_BRIDGE.load(Ordering::Relaxed)
}

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
    /// Applied spin-test wheel torques, in EFeru command units.
    pub spin_left_units: f32,
    /// Applied spin-test wheel torques, in EFeru command units.
    pub spin_right_units: f32,
    /// Applied balance-law common-mode torque (pre-mixer), command units.
    pub balance_torque_common_units: f32,
    /// Applied (ramped) pitch reference in degrees, zeroed-offset frame.
    pub pitch_ref_deg: f32,
    /// Total pitch offset demanded by the outer loops, degrees.
    pub pitch_demand_deg: f32,
    /// Learned slope trim, degrees.
    pub slope_trim_deg: f32,
    /// Travel since arming, metres, positive forward.
    pub position_m: f32,
    /// Balance mode: 0 disarmed, 1 transit, 2 hold, 3 recover, 4 fall.
    pub balance_mode: u8,
    /// Separately filtered tilt seen by the fall guard, degrees.
    pub guard_tilt_deg: f32,
    /// Operational mode code from `jbot_pico_core::control::BalanceMode`.
    pub op_mode: u8,
    /// Remaining guard grace for a deliberate raise, seconds.
    pub raise_grace_s: f32,
    /// Applied calibrated feedforward, command units.
    pub feedforward_units: f32,
    /// Configured per-wheel breakaway compensation, command units.
    pub cal_left_units: f32,
    /// Configured per-wheel breakaway compensation, command units.
    pub cal_right_units: f32,
    /// Calibrated wheel torque actually sent on this tick, command units.
    pub cmd_left_units: f32,
    /// Calibrated wheel torque actually sent on this tick, command units.
    pub cmd_right_units: f32,
    /// Configured dither amplitude, command units.
    pub dither_units: f32,
    /// Dither actually added on this tick, command units.
    pub dither_applied_units: f32,
}

/// Balance mode values carried by [`Telemetry::balance_mode`].
pub const MODE_DISARMED: u8 = 0;
/// Armed and ramping towards the home reference.
pub const MODE_TRANSIT: u8 = 1;
/// Armed and parked on the home reference.
pub const MODE_HOLD: u8 = 2;
/// Armed and |pitch| above the recover threshold.
pub const MODE_RECOVER: u8 = 3;
/// Armed and |pitch| passed the fail threshold; the law was latched off this tick.
pub const MODE_FALL: u8 = 4;

/// Latest snapshot, refreshed on every control step.
pub static TELEMETRY: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
/// Ask the IMU task to re-run the BNO055 start-up sequence.
pub static REINIT: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Latest spin-test request, overwritten like every other command setpoint.
pub static SPIN_REQUEST: Signal<CriticalSectionRawMutex, SpinPair> = Signal::new();
/// One-shot request to begin a controlled fall while armed.
pub static SAFE_FALL_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Runtime enable for the calibrated feedforward term. Off at boot so a
/// bench-defined curve cannot leak into a fresh session.
static FEEDFORWARD_ENABLED: AtomicBool = AtomicBool::new(false);

/// Whether the calibrated feedforward term is applied.
pub fn feedforward_enabled() -> bool {
    FEEDFORWARD_ENABLED.load(Ordering::Relaxed)
}

/// Enable or disable the calibrated feedforward term.
pub fn set_feedforward_enabled(enabled: bool) {
    FEEDFORWARD_ENABLED.store(enabled, Ordering::Relaxed);
}

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
/// BNO055 mount selector: `normal` trusts the fused Euler angles, which
/// match the physical attitude on the current chassis (the module X axis
/// runs along the axle, so the raw X rate is the pitch rate). It remains
/// selectable over USB; any change still requires re-capturing
/// `zero`/`level`.
///
/// At boot the stored record overrides this default when one decodes from
/// flash (`crate::nvm`); the default only applies to a blank flash or a
/// damaged record.
static MOUNT: AtomicU32 = AtomicU32::new(Mount::Normal.as_u32());
/// Whether a settings record was decoded from flash at boot.
static NVM_PRESENT: AtomicBool = AtomicBool::new(false);
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
/// Measured on-back pitch reference in degrees; `NaN` means not measured.
static POSE_BACK_BITS: AtomicU32 = AtomicU32::new(f32::NAN.to_bits());
/// Measured face-down pitch reference in degrees; `NaN` means not measured.
static POSE_FACE_BITS: AtomicU32 = AtomicU32::new(f32::NAN.to_bits());
/// Measured vertical pitch reference in degrees; `NaN` means not measured.
static POSE_HOME_BITS: AtomicU32 = AtomicU32::new(f32::NAN.to_bits());
/// Applied spin-test left torque, in command units, stored as `f32` bits.
static SPIN_APPLIED_LEFT_BITS: AtomicU32 = AtomicU32::new(0);
/// Applied spin-test right torque, in command units, stored as `f32` bits.
static SPIN_APPLIED_RIGHT_BITS: AtomicU32 = AtomicU32::new(0);
/// Applied balance-law common-mode torque, stored as `f32` bits.
static BAL_TORQUE_COMMON_BITS: AtomicU32 = AtomicU32::new(0);
/// Explicitly commanded home reference, radians, stored as `f32` bits.
static HOME_TARGET_BITS: AtomicU32 = AtomicU32::new(0);
/// Whether [`HOME_TARGET_BITS`] holds a commanded value.
static HOME_TARGET_SET: AtomicBool = AtomicBool::new(false);
/// Commanded reference rate, radians per second, stored as `f32` bits.
/// Defaults to `reference::DEFAULT_V_MAX_RAD_S` (10 deg/s).
static REF_VMAX_BITS: AtomicU32 =
    AtomicU32::new(jbot_pico_core::reference::DEFAULT_V_MAX_RAD_S.to_bits());
/// Runtime scale on the angular-output gains ([`Tuning::scaled_by`]),
/// stored as `f32` bits. Runtime-only: resets to 1.0 at boot so a bench
/// session cannot leak a weakened law into the next power-up.
static PSCALE_BITS: AtomicU32 = AtomicU32::new(1.0f32.to_bits());
/// Runtime scale on only the pitch derivative gain, stored as `f32` bits.
/// Runtime-only like [`PSCALE_BITS`], so it cannot leak into a power-up.
static DSCALE_BITS: AtomicU32 = AtomicU32::new(1.0f32.to_bits());
/// Runtime inverse of the measured drive dead zone, newton-metres, stored as
/// `f32` bits. Off at boot like the feedforward gate: an empirical bench value
/// must never change the compiled law on a fresh power-up.
static DEADBAND_NM_BITS: AtomicU32 = AtomicU32::new(0);
/// Largest dead-zone compensation the console accepts, newton-metres.
pub const MAX_DEADBAND_NM: f32 = 0.6;
/// Per-wheel breakaway calibration, command units, stored as `f32` bits.
/// Runtime-only: zero at boot, like [`PSCALE_BITS`] and [`DEADBAND_NM_BITS`].
static CAL_LEFT_BITS: AtomicU32 = AtomicU32::new(0);
static CAL_RIGHT_BITS: AtomicU32 = AtomicU32::new(0);
/// Calibrated wheel torque sent on the latest control tick, `f32` bits.
static CMD_LEFT_BITS: AtomicU32 = AtomicU32::new(0);
static CMD_RIGHT_BITS: AtomicU32 = AtomicU32::new(0);
/// Dither amplitude, command units, `f32` bits. Runtime-only and zero at boot.
static DITHER_BITS: AtomicU32 = AtomicU32::new(0);
/// Largest dither amplitude the console accepts, command units. Roughly the
/// measured breakaway range, so the wave can reach the stiction threshold.
pub const MAX_DITHER_UNITS: f32 = 60.0;
/// Applied dither on the latest control tick, `f32` bits.
static DITHER_APPLIED_BITS: AtomicU32 = AtomicU32::new(0);
/// Smith-predictor horizon, milliseconds, stored as `f32` bits. Runtime-only
/// and zero at boot: delay compensation must be measured, never assumed.
static PREDICT_MS_BITS: AtomicU32 = AtomicU32::new(0);
/// Longest predictor horizon the console accepts, milliseconds.
pub const MAX_PREDICT_MS: f32 = 150.0;
/// Default rate of change of the applied wheel torque, command units per
/// second. The compiled value keeps the response snappy enough for balance.
pub const DEFAULT_TORQUE_SLEW_PER_S: f32 = 3_000.0;
/// Slowest torque ramp the console accepts, command units per second.
pub const MIN_TORQUE_SLEW_PER_S: f32 = 100.0;
/// Fastest torque ramp the console accepts, command units per second.
pub const MAX_TORQUE_SLEW_PER_S: f32 = 10_000.0;
/// Runtime torque slew rate stored as `f32` bits. Runtime-only, like
/// [`PSCALE_BITS`]: a bench session cannot leak a softened ramp into the
/// next power-up.
static TORQUE_SLEW_BITS: AtomicU32 = AtomicU32::new(DEFAULT_TORQUE_SLEW_PER_S.to_bits());

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
/// Times the balance law was latched off by the fall detector.
pub static FALL_EVENTS: AtomicU32 = AtomicU32::new(0);
/// Bytes discarded while resynchronising the feedback parser.
pub static FB_DISCARDED: AtomicU32 = AtomicU32::new(0);
/// CRC or framing failures of the hardened feedback decoder.
pub static FB_CRC_ERRORS: AtomicU32 = AtomicU32::new(0);
/// Raw bytes received from the mainboard link, including bytes that never
/// become a frame. Distinguishes a dead wire from a framing mismatch.
pub static LINK_BYTES: AtomicU32 = AtomicU32::new(0);

/// Last reason the balance supervisor latched off, stored as a
/// [`StopReason::code`]; zero means none since the last arm or clear.
static STOP_REASON: AtomicU32 = AtomicU32::new(0);
/// Whether the control loop currently has fresh wheel-speed feedback.
static LINK_FRESH: AtomicBool = AtomicBool::new(false);

/// Record the supervisor decision that latched the law off.
pub fn set_stop(reason: Option<StopReason>) {
    STOP_REASON.store(
        reason.map_or(0, |reason| u32::from(reason.code())),
        Ordering::Relaxed,
    );
}

/// Last recorded stop reason, if any.
pub fn stop_reason() -> Option<StopReason> {
    StopReason::from_code(STOP_REASON.load(Ordering::Relaxed) as u8)
}

/// Label of the last recorded stop reason, or `none`.
pub fn stop_reason_label() -> &'static str {
    stop_reason().map_or("none", StopReason::as_str)
}

/// Publish whether the control loop has fresh wheel-speed feedback.
pub fn set_link_fresh(fresh: bool) {
    LINK_FRESH.store(fresh, Ordering::Relaxed);
}

/// Freshness of the mainboard feedback as seen by the control loop.
pub fn link_fresh() -> bool {
    LINK_FRESH.load(Ordering::Relaxed)
}

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

/// Command the home reference to `rad`. Applies from the next control tick;
/// the reference generator ramps there smoothly.
pub fn set_home_target_rad(rad: f32) {
    HOME_TARGET_BITS.store(rad.to_bits(), Ordering::Relaxed);
    HOME_TARGET_SET.store(true, Ordering::Relaxed);
}

/// Drop the commanded home target; the reference falls back to the captured
/// `level` trim.
pub fn clear_home_target() {
    HOME_TARGET_SET.store(false, Ordering::Relaxed);
}

/// Commanded home reference, radians, or `None` when unset.
pub fn home_target_rad() -> Option<f32> {
    if HOME_TARGET_SET.load(Ordering::Relaxed) {
        Some(f32::from_bits(HOME_TARGET_BITS.load(Ordering::Relaxed)))
    } else {
        None
    }
}

/// Commanded reference rate, radians per second.
pub fn ref_v_max_rad_s() -> f32 {
    f32::from_bits(REF_VMAX_BITS.load(Ordering::Relaxed))
}

/// Runtime scale on the angular-output gains; 1.0 is the compiled tuning.
pub fn pscale() -> f32 {
    f32::from_bits(PSCALE_BITS.load(Ordering::Relaxed))
}

/// Set the runtime scale on the angular-output gains, clamped to
/// 0..=[`MAX_PSCALE`]. Non-finite input resets to the neutral 1.0.
/// Returns the value that was stored.
pub fn set_pscale(value: f32) -> f32 {
    let clamped = if value.is_finite() {
        value.clamp(0.0, MAX_PSCALE)
    } else {
        1.0
    };
    PSCALE_BITS.store(clamped.to_bits(), Ordering::Relaxed);
    clamped
}

/// Runtime scale on the pitch derivative gain only.
pub fn dscale() -> f32 {
    f32::from_bits(DSCALE_BITS.load(Ordering::Relaxed))
}

/// Set the derivative-only scale, clamped to `0..=[`MAX_DSCALE`]`.
/// Non-finite input resets to the neutral 1.0. Returns the stored value.
pub fn set_dscale(value: f32) -> f32 {
    let clamped = if value.is_finite() {
        value.clamp(0.0, MAX_DSCALE)
    } else {
        1.0
    };
    DSCALE_BITS.store(clamped.to_bits(), Ordering::Relaxed);
    clamped
}

/// Runtime dead-zone compensation in newton-metres; zero disables it.
pub fn deadband_nm() -> f32 {
    f32::from_bits(DEADBAND_NM_BITS.load(Ordering::Relaxed))
}

/// Set the runtime dead-zone compensation, clamped to
/// `0..=[`MAX_DEADBAND_NM`]`. Non-finite input disables it. Returns the
/// stored value.
pub fn set_deadband_nm(value: f32) -> f32 {
    let clamped = if value.is_finite() {
        value.clamp(0.0, MAX_DEADBAND_NM)
    } else {
        0.0
    };
    DEADBAND_NM_BITS.store(clamped.to_bits(), Ordering::Relaxed);
    clamped
}

/// Configured left-wheel breakaway compensation, command units.
pub fn cal_left_units() -> f32 {
    f32::from_bits(CAL_LEFT_BITS.load(Ordering::Relaxed))
}

/// Configured right-wheel breakaway compensation, command units.
pub fn cal_right_units() -> f32 {
    f32::from_bits(CAL_RIGHT_BITS.load(Ordering::Relaxed))
}

/// Set one wheel's breakaway compensation, clamped to
/// `0..=[`MAX_BREAKAWAY_UNITS`]`. Non-finite input disables it. Returns the
/// stored value.
fn set_cal_units(bits: &AtomicU32, value: f32) -> f32 {
    let clamped = if value.is_finite() {
        value.clamp(0.0, MAX_BREAKAWAY_UNITS)
    } else {
        0.0
    };
    bits.store(clamped.to_bits(), Ordering::Relaxed);
    clamped
}

/// Set the left-wheel breakaway compensation.
pub fn set_cal_left_units(value: f32) -> f32 {
    set_cal_units(&CAL_LEFT_BITS, value)
}

/// Set the right-wheel breakaway compensation.
pub fn set_cal_right_units(value: f32) -> f32 {
    set_cal_units(&CAL_RIGHT_BITS, value)
}

/// Disable both per-wheel breakaway compensations.
pub fn clear_cal() {
    set_cal_units(&CAL_LEFT_BITS, 0.0);
    set_cal_units(&CAL_RIGHT_BITS, 0.0);
}

/// Publish the calibrated wheel torque sent on this control tick.
pub fn set_command_applied(left: f32, right: f32) {
    CMD_LEFT_BITS.store(left.to_bits(), Ordering::Relaxed);
    CMD_RIGHT_BITS.store(right.to_bits(), Ordering::Relaxed);
}

/// Configured dither amplitude, command units.
pub fn dither_units() -> f32 {
    f32::from_bits(DITHER_BITS.load(Ordering::Relaxed))
}

/// Set the dither amplitude, clamped to `0..=[`MAX_DITHER_UNITS`]`.
/// Non-finite input disables it. Returns the stored value.
pub fn set_dither_units(value: f32) -> f32 {
    let clamped = if value.is_finite() {
        value.clamp(0.0, MAX_DITHER_UNITS)
    } else {
        0.0
    };
    DITHER_BITS.store(clamped.to_bits(), Ordering::Relaxed);
    clamped
}

/// Configured Smith-predictor horizon, milliseconds.
pub fn predict_ms() -> f32 {
    f32::from_bits(PREDICT_MS_BITS.load(Ordering::Relaxed))
}

/// Set the predictor horizon, clamped to `0..=[`MAX_PREDICT_MS`]`.
/// Non-finite input disables it. Returns the stored value.
pub fn set_predict_ms(value: f32) -> f32 {
    let clamped = if value.is_finite() {
        value.clamp(0.0, MAX_PREDICT_MS)
    } else {
        0.0
    };
    PREDICT_MS_BITS.store(clamped.to_bits(), Ordering::Relaxed);
    clamped
}

/// Publish the dither added on this control tick.
pub fn set_dither_applied(value: f32) {
    DITHER_APPLIED_BITS.store(value.to_bits(), Ordering::Relaxed);
}

/// Dither added on the latest control tick.
pub fn dither_applied() -> f32 {
    f32::from_bits(DITHER_APPLIED_BITS.load(Ordering::Relaxed))
}

/// Calibrated wheel torque sent on the latest control tick.
pub fn command_applied() -> (f32, f32) {
    (
        f32::from_bits(CMD_LEFT_BITS.load(Ordering::Relaxed)),
        f32::from_bits(CMD_RIGHT_BITS.load(Ordering::Relaxed)),
    )
}

/// Set the commanded reference rate, radians per second. Clamping is the
/// caller's responsibility; [`crate::console`] enforces the `0..=60 deg/s`
/// range and `PitchReference::set_v_max` clamps the rest.
pub fn set_ref_v_max_rad_s(rad_s: f32) {
    REF_VMAX_BITS.store(rad_s.to_bits(), Ordering::Relaxed);
}

/// Runtime rate of change of the applied wheel torque, command units per
/// second.
pub fn torque_slew_per_s() -> f32 {
    f32::from_bits(TORQUE_SLEW_BITS.load(Ordering::Relaxed))
}

/// Set the runtime torque slew rate, clamped to
/// [`MIN_TORQUE_SLEW_PER_S`]..=[`MAX_TORQUE_SLEW_PER_S`]. Non-finite input
/// resets to [`DEFAULT_TORQUE_SLEW_PER_S`]. Returns the stored value.
pub fn set_torque_slew_per_s(value: f32) -> f32 {
    let clamped = if value.is_finite() {
        value.clamp(MIN_TORQUE_SLEW_PER_S, MAX_TORQUE_SLEW_PER_S)
    } else {
        DEFAULT_TORQUE_SLEW_PER_S
    };
    TORQUE_SLEW_BITS.store(clamped.to_bits(), Ordering::Relaxed);
    clamped
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
/// the balance law and drops any pending spin-test request; clearing the
/// stop never re-arms anything.
pub fn set_estop(active: bool) {
    ESTOP.store(active, Ordering::Relaxed);
    if active {
        ARMED.store(false, Ordering::Relaxed);
        clear_spin_request();
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

/// Whether a settings record was decoded from flash at boot.
pub fn nvm_present() -> bool {
    NVM_PRESENT.load(Ordering::Relaxed)
}

/// Publish whether a settings record was decoded from flash at boot.
pub fn set_nvm_present(present: bool) {
    NVM_PRESENT.store(present, Ordering::Relaxed);
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
    if armed {
        // A fresh attempt supersedes the last latch reason; the supervisor
        // records a new one in the same control tick if it refuses.
        set_stop(None);
    }
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

/// Measured on-back pitch reference, or `None` when not captured.
pub fn pose_back_deg() -> Option<f32> {
    pose_from_bits(POSE_BACK_BITS.load(Ordering::Relaxed))
}

/// Measured face-down pitch reference, or `None` when not captured.
pub fn pose_face_deg() -> Option<f32> {
    pose_from_bits(POSE_FACE_BITS.load(Ordering::Relaxed))
}

/// Measured vertical pitch reference, or `None` when not captured.
pub fn pose_home_deg() -> Option<f32> {
    pose_from_bits(POSE_HOME_BITS.load(Ordering::Relaxed))
}

/// Store or clear the on-back pitch reference.
pub fn set_pose_back_deg(value: Option<f32>) {
    POSE_BACK_BITS.store(pose_to_bits(value), Ordering::Relaxed);
}

/// Store or clear the face-down pitch reference.
pub fn set_pose_face_deg(value: Option<f32>) {
    POSE_FACE_BITS.store(pose_to_bits(value), Ordering::Relaxed);
}

/// Store or clear the vertical pitch reference.
pub fn set_pose_home_deg(value: Option<f32>) {
    POSE_HOME_BITS.store(pose_to_bits(value), Ordering::Relaxed);
}

fn pose_from_bits(bits: u32) -> Option<f32> {
    let value = f32::from_bits(bits);
    value.is_finite().then_some(value)
}

fn pose_to_bits(value: Option<f32>) -> u32 {
    value.unwrap_or(f32::NAN).to_bits()
}

/// Publish the latest mapped attitude (pre-zero), degrees.
pub fn set_latest_mapped_attitude(pitch_deg: f32, roll_deg: f32) {
    LATEST_MAPPED_PITCH_BITS.store(pitch_deg.to_bits(), Ordering::Relaxed);
    LATEST_MAPPED_ROLL_BITS.store(roll_deg.to_bits(), Ordering::Relaxed);
}

/// Request a spin test; the latest request wins.
pub fn request_spin(pair: SpinPair) {
    SPIN_REQUEST.signal(pair);
}

/// Drop any pending spin-test request (e-stop or arm invalidates it).
pub fn clear_spin_request() {
    SPIN_REQUEST.reset();
}

/// Publish the spin-test torques applied on this tick.
pub fn set_spin_applied(left: f32, right: f32) {
    SPIN_APPLIED_LEFT_BITS.store(left.to_bits(), Ordering::Relaxed);
    SPIN_APPLIED_RIGHT_BITS.store(right.to_bits(), Ordering::Relaxed);
}

/// Spin-test torques applied on the latest control tick.
pub fn spin_applied() -> SpinPair {
    SpinPair {
        left: f32::from_bits(SPIN_APPLIED_LEFT_BITS.load(Ordering::Relaxed)),
        right: f32::from_bits(SPIN_APPLIED_RIGHT_BITS.load(Ordering::Relaxed)),
    }
}

/// Publish the balance-law common-mode torque applied on this tick.
pub fn set_balance_applied(common_units: f32) {
    BAL_TORQUE_COMMON_BITS.store(common_units.to_bits(), Ordering::Relaxed);
}

/// Balance-law common-mode torque applied on the latest control tick.
pub fn balance_applied() -> f32 {
    f32::from_bits(BAL_TORQUE_COMMON_BITS.load(Ordering::Relaxed))
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
    FB_CRC_ERRORS.store(0, Ordering::Relaxed);
    LINK_BYTES.store(0, Ordering::Relaxed);
    STOP_REASON.store(0, Ordering::Relaxed);
    FALL_EVENTS.store(0, Ordering::Relaxed);
}
