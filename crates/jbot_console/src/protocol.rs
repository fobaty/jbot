//! Line protocol used by the Pico 2 W USB console.
//!
//! The firmware sends one ASCII line per message:
//!
//! * `T key=value ...` telemetry sample
//! * `stat key=value ...` diagnostic counters
//! * `ok ...` accepted command
//! * `err ...` rejected command
//! * `JBOT ...` banner after enumeration
//!
//! Unknown keys are ignored, so this parser keeps working when the firmware
//! adds fields. A malformed value in a known key rejects the whole sample
//! instead of displaying a half-parsed state.

/// Telemetry rate requested on connect, in hertz.
pub const DEFAULT_STREAM_HZ: u32 = 20;
/// Highest telemetry rate accepted by the firmware.
pub const MAX_STREAM_HZ: u32 = 200;
/// `age` value when no mainboard feedback frame has arrived since boot.
pub const AGE_NONE: u32 = u32::MAX;

/// One parsed `T ...` sample.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Telemetry {
    /// Control ticks since boot.
    pub step: u32,
    /// Measured control loop rate, hertz.
    pub loop_hz: u32,
    /// The IMU sample used by the control tick was fresh.
    pub imu_fresh: bool,
    /// `CALIB_STAT` system byte, `0..=3`.
    pub cal: u8,
    /// Pitch used by the control law, degrees.
    pub pitch_deg: f32,
    /// Roll from the fused Euler frame, degrees.
    pub roll_deg: f32,
    /// Heading from the fused Euler frame, degrees.
    pub heading_deg: f32,
    /// Pitch rate, degrees per second.
    pub gyro_y_dps: f32,
    /// Roll rate, degrees per second.
    pub gyro_x_dps: f32,
    /// Yaw rate, degrees per second.
    pub gyro_z_dps: f32,
    /// `SYS_ERR` register; zero is healthy.
    pub sys_err: u8,
    /// At least one mainboard feedback frame has arrived.
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
    /// Firmware emergency stop latch.
    pub estop: bool,
    /// Speed setpoint, meters per second.
    pub speed_setpoint_mps: f32,
    /// Steering setpoint, radians per second.
    pub steer_setpoint_rps: f32,
    /// Both `roll` and `hdg` were present in the sample.
    pub has_orientation: bool,
    /// Both `vset` and `wset` were present in the sample.
    pub has_setpoints: bool,
    /// Firmware BNO055 mount selector: 0 normal, 1 flip-x, 2 flip-y,
    /// 3 rot +90° X, 4 rot −90° X, 5 rot +90° Y, 6 rot −90° Y.
    pub mount: Option<u8>,
    /// Applied pitch zero offset, degrees.
    pub zero_offset_deg: Option<f32>,
    /// Applied roll zero offset, degrees.
    pub roll_zero_offset_deg: Option<f32>,
    /// Applied balance (level) reference, degrees.
    pub level_deg: Option<f32>,
    /// Operator arm latch for the balance law.
    pub armed: Option<bool>,
    /// Applied spin-test torque on the left wheel, EFeru command units.
    pub test_left_units: Option<f32>,
    /// Applied spin-test torque on the right wheel, EFeru command units.
    pub test_right_units: Option<f32>,
    /// Applied balance-law common-mode torque (pre-mixer), command units.
    pub torque_common_units: Option<f32>,
    /// Applied (ramped) pitch reference in degrees, same frame as `pitch`.
    pub pitch_ref_deg: Option<f32>,
    /// Total pitch offset demanded by the outer loops, degrees.
    pub pitch_demand_deg: Option<f32>,
    /// Learned slope trim, degrees.
    pub slope_trim_deg: Option<f32>,
    /// Travel since arming, metres, positive forward.
    pub position_m: Option<f32>,
    /// Balance mode: 0 disarmed, 1 transit, 2 hold, 3 recover, 4 fall.
    pub balance_mode: Option<u8>,
    /// Separately filtered tilt seen by the fall guard, degrees.
    pub guard_tilt_deg: Option<f32>,
    /// Operational mode code from `BalanceMode`.
    pub op_mode: Option<u8>,
    /// Remaining guard grace for a deliberate raise, seconds.
    pub raise_grace_s: Option<f32>,
}

impl Telemetry {
    /// Steering setpoint in degrees per second, for display.
    pub fn steer_setpoint_deg_s(&self) -> f32 {
        self.steer_setpoint_rps.to_degrees()
    }

    /// Largest absolute tilt from the pitch and roll angles.
    pub fn max_tilt_deg(&self) -> f32 {
        self.pitch_deg.abs().max(self.roll_deg.abs())
    }

    /// Age text for the mainboard feedback frame.
    pub fn feedback_age_text(&self) -> String {
        if self.feedback_age_ms == AGE_NONE {
            "--".to_owned()
        } else {
            format!("{} ms", self.feedback_age_ms)
        }
    }

    /// Pack voltage mapped to `0.0..=1.0` over the 30-42 V range.
    pub fn battery_fraction(&self) -> f32 {
        ((self.battery_v - 30.0) / 12.0).clamp(0.0, 1.0)
    }

    /// Average absolute wheel speed, rpm.
    pub fn wheel_speed_rpm(&self) -> f32 {
        0.5 * (f32::from(self.speed_left_rpm).abs() + f32::from(self.speed_right_rpm).abs())
    }
}

/// One parsed `stat ...` reply.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Stat {
    /// Uptime in seconds.
    pub uptime_s: f32,
    /// Measured control loop rate, hertz.
    pub loop_hz: u32,
    /// Requested telemetry rate, hertz.
    pub stream_hz: u32,
    /// Pitch trim, degrees.
    pub trim_deg: f32,
    /// Firmware emergency stop latch.
    pub estop: bool,
    /// Control ticks since boot.
    pub steps: u32,
    /// IMU samples delivered to the control task.
    pub imu_samples: u32,
    /// IMU read failures.
    pub imu_errors: u32,
    /// Last IMU error code.
    pub imu_code: u32,
    /// Valid mainboard feedback frames.
    pub fb_frames: u32,
    /// Bytes discarded while resynchronising the feedback parser.
    pub fb_discarded: u32,
    /// CRC or framing failures of the hardened feedback decoder.
    pub fb_crc_errors: Option<u32>,
    /// Raw bytes received on the mainboard link.
    pub link_bytes: Option<u32>,
    /// Active mainboard link contract: `LEGACY8` or `HARDENED`.
    pub link_protocol: Option<String>,
    /// Whether the control loop currently has fresh wheel-speed feedback.
    pub link_fresh: Option<bool>,
    /// Last reason the balance supervisor latched off, if any.
    pub stop_reason: Option<String>,
    /// Applied pitch zero offset, degrees.
    pub zero_deg: Option<f32>,
    /// Applied roll zero offset, degrees.
    pub zroll_deg: Option<f32>,
    /// Applied balance (level) reference, degrees.
    pub level_deg: Option<f32>,
    /// BNO055 mount selector: 0 normal .. 6 rot −90° Y.
    pub mount: Option<u8>,
    /// Operator arm latch for the balance law.
    pub armed: Option<bool>,
    /// Commanded home reference, degrees (captured level when unset).
    pub home_deg: Option<f32>,
    /// Latched fall events since boot.
    pub fall_events: Option<u32>,
    /// Commanded reference rate for the home ramp, degrees per second.
    pub ref_vmax_deg_s: Option<f32>,
    /// Effective proportional pitch gain after the power scale, Nm per radian.
    pub kp_nm_per_rad: Option<f32>,
    /// Effective derivative pitch gain after the power scale, Nm per (rad/s).
    pub kd_nm_per_rad_s: Option<f32>,
    /// Position loop gain, metres offset to metres per second.
    pub position_gain_per_s: Option<f32>,
    /// Speed loop gain, acceleration per m/s of speed error.
    pub velocity_gain_per_s: Option<f32>,
    /// Slope trim integrator gain.
    pub trim_gain_rad_per_m_s: Option<f32>,
    /// Cap on the dynamic pitch demand, degrees.
    pub demand_limit_deg: Option<f32>,
    /// Cap on the total pitch demand including the slope trim, degrees.
    pub trim_limit_deg: Option<f32>,
    /// Whether the `zero` offset was captured this session.
    pub zero_captured: Option<bool>,
    /// Whether the `level` reference was captured this session.
    pub level_captured: Option<bool>,
    /// Runtime scale on the angular-output gains (1.0 = compiled tuning).
    pub pscale: Option<f32>,
    /// Original line, kept for the event log.
    pub raw: String,
}

/// A classified protocol line.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    /// Telemetry sample.
    Telemetry(Telemetry),
    /// Diagnostic reply.
    Stat(Stat),
    /// Accepted command.
    Ok(String),
    /// Rejected command.
    Err(String),
    /// Firmware banner.
    Banner(String),
    /// Line that does not match the protocol.
    Garbage(String),
}

/// Parse a `T key=value ...` telemetry line.
pub fn parse_telemetry(line: &str) -> Option<Telemetry> {
    let body = line.strip_prefix("T ")?;
    let mut telemetry = Telemetry::default();
    let mut saw_step = false;
    let mut saw_pitch = false;
    let mut saw_roll = false;
    let mut saw_heading = false;
    let mut saw_speed_setpoint = false;
    let mut saw_steer_setpoint = false;

    for token in body.split_whitespace() {
        let (key, value) = token.split_once('=')?;
        match key {
            "step" => {
                telemetry.step = parse_u32(value)?;
                saw_step = true;
            }
            "loop" => telemetry.loop_hz = parse_u32(value)?,
            "imu" => telemetry.imu_fresh = parse_imu_word(value)?,
            "cal" => telemetry.cal = parse_u8(value)?,
            "pitch" => {
                telemetry.pitch_deg = parse_f32(value)?;
                saw_pitch = true;
            }
            "roll" => {
                telemetry.roll_deg = parse_f32(value)?;
                saw_roll = true;
            }
            "hdg" => {
                telemetry.heading_deg = parse_f32(value)?;
                saw_heading = true;
            }
            "gy" => telemetry.gyro_y_dps = parse_f32(value)?,
            "gx" => telemetry.gyro_x_dps = parse_f32(value)?,
            "gz" => telemetry.gyro_z_dps = parse_f32(value)?,
            "err" => telemetry.sys_err = parse_u8_auto(value)?,
            "mb" => telemetry.feedback_present = parse_feedback_word(value)?,
            "L" => telemetry.speed_left_rpm = parse_i16(value)?,
            "R" => telemetry.speed_right_rpm = parse_i16(value)?,
            "bat" => telemetry.battery_v = parse_f32(value)?,
            "tmp" => telemetry.board_temp_c = parse_f32(value)?,
            "age" => telemetry.feedback_age_ms = parse_u32(value)?,
            "estop" => telemetry.estop = parse_flag(value)?,
            "vset" => {
                telemetry.speed_setpoint_mps = parse_f32(value)?;
                saw_speed_setpoint = true;
            }
            "wset" => {
                telemetry.steer_setpoint_rps = parse_f32(value)?.to_radians();
                saw_steer_setpoint = true;
            }
            "mnt" => telemetry.mount = Some(parse_mount(value)?),
            "zof" => telemetry.zero_offset_deg = Some(parse_f32(value)?),
            "zro" => telemetry.roll_zero_offset_deg = Some(parse_f32(value)?),
            "lvl" => telemetry.level_deg = Some(parse_f32(value)?),
            "arm" => telemetry.armed = Some(parse_flag(value)?),
            "tsl" => telemetry.test_left_units = Some(parse_f32(value)?),
            "tsr" => telemetry.test_right_units = Some(parse_f32(value)?),
            "trq" => telemetry.torque_common_units = Some(parse_f32(value)?),
            "ref" => telemetry.pitch_ref_deg = Some(parse_f32(value)?),
            "dem" => telemetry.pitch_demand_deg = Some(parse_f32(value)?),
            "trm" => telemetry.slope_trim_deg = Some(parse_f32(value)?),
            "x" => telemetry.position_m = Some(parse_f32(value)?),
            "mode" => telemetry.balance_mode = Some(parse_u8(value)?),
            "gtlt" => telemetry.guard_tilt_deg = Some(parse_f32(value)?),
            "opm" => telemetry.op_mode = Some(parse_u8(value)?),
            "grace" => telemetry.raise_grace_s = Some(parse_f32(value)?),
            _ => {}
        }
    }

    let has_orientation = saw_roll && saw_heading;
    let has_setpoints = saw_speed_setpoint && saw_steer_setpoint;
    if !saw_step || !saw_pitch {
        return None;
    }
    telemetry.has_orientation = has_orientation;
    telemetry.has_setpoints = has_setpoints;
    Some(telemetry)
}

/// Parse a `stat key=value ...` reply.
pub fn parse_stat(line: &str) -> Option<Stat> {
    let body = line.strip_prefix("stat ")?;
    let mut stat = Stat {
        raw: line.to_owned(),
        ..Stat::default()
    };
    let mut recognised = false;

    for token in body.split_whitespace() {
        let Some((key, value)) = token.split_once('=') else {
            continue;
        };
        match key {
            "up" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.uptime_s = parsed;
                    recognised = true;
                }
            }
            "loop" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.loop_hz = parsed;
                    recognised = true;
                }
            }
            "stream" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.stream_hz = parsed;
                    recognised = true;
                }
            }
            "trim" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.trim_deg = parsed;
                    recognised = true;
                }
            }
            "estop" => {
                if let Some(parsed) = parse_flag(value) {
                    stat.estop = parsed;
                    recognised = true;
                }
            }
            "steps" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.steps = parsed;
                    recognised = true;
                }
            }
            "imu" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.imu_samples = parsed;
                    recognised = true;
                }
            }
            "imu_err" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.imu_errors = parsed;
                    recognised = true;
                }
            }
            "imu_code" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.imu_code = parsed;
                    recognised = true;
                }
            }
            "fb" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.fb_frames = parsed;
                    recognised = true;
                }
            }
            "fb_drop" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.fb_discarded = parsed;
                    recognised = true;
                }
            }
            "fb_crc" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.fb_crc_errors = Some(parsed);
                    recognised = true;
                }
            }
            "rx" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.link_bytes = Some(parsed);
                    recognised = true;
                }
            }
            "fb_ok" => {
                if let Some(parsed) = parse_flag(value) {
                    stat.link_fresh = Some(parsed);
                    recognised = true;
                }
            }
            "proto" => {
                if matches!(value, "LEGACY8" | "HARDENED") {
                    stat.link_protocol = Some(value.to_owned());
                    recognised = true;
                }
            }
            "stop" => {
                stat.stop_reason = Some(value.to_owned());
                recognised = true;
            }
            "zero" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.zero_deg = Some(parsed);
                    recognised = true;
                }
            }
            "zroll" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.zroll_deg = Some(parsed);
                    recognised = true;
                }
            }
            "level" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.level_deg = Some(parsed);
                    recognised = true;
                }
            }
            "mount" => {
                if let Some(parsed) = parse_mount(value) {
                    stat.mount = Some(parsed);
                    recognised = true;
                }
            }
            "armed" => {
                if let Some(parsed) = parse_flag(value) {
                    stat.armed = Some(parsed);
                    recognised = true;
                }
            }
            "home" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.home_deg = Some(parsed);
                    recognised = true;
                }
            }
            "fall" => {
                if let Some(parsed) = parse_u32(value) {
                    stat.fall_events = Some(parsed);
                    recognised = true;
                }
            }
            "vmax" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.ref_vmax_deg_s = Some(parsed);
                    recognised = true;
                }
            }
            "kpnm" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.kp_nm_per_rad = Some(parsed);
                    recognised = true;
                }
            }
            "kdnm" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.kd_nm_per_rad_s = Some(parsed);
                    recognised = true;
                }
            }
            "kpos" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.position_gain_per_s = Some(parsed);
                    recognised = true;
                }
            }
            "kv" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.velocity_gain_per_s = Some(parsed);
                    recognised = true;
                }
            }
            "ki" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.trim_gain_rad_per_m_s = Some(parsed);
                    recognised = true;
                }
            }
            "pdem" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.demand_limit_deg = Some(parsed);
                    recognised = true;
                }
            }
            "ptrm" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.trim_limit_deg = Some(parsed);
                    recognised = true;
                }
            }
            "zcapt" => {
                if let Some(parsed) = parse_flag(value) {
                    stat.zero_captured = Some(parsed);
                    recognised = true;
                }
            }
            "lcapt" => {
                if let Some(parsed) = parse_flag(value) {
                    stat.level_captured = Some(parsed);
                    recognised = true;
                }
            }
            "pscale" => {
                if let Some(parsed) = parse_f32(value) {
                    stat.pscale = Some(parsed);
                    recognised = true;
                }
            }
            _ => {}
        }
    }

    recognised.then_some(stat)
}

/// Classify one complete line. Blank lines return `None`.
pub fn classify(line: &str) -> Option<Message> {
    let line = line.trim();
    if line.is_empty() {
        return None;
    }
    if let Some(telemetry) = parse_telemetry(line) {
        return Some(Message::Telemetry(telemetry));
    }
    if let Some(stat) = parse_stat(line) {
        return Some(Message::Stat(stat));
    }
    if line == "ok" || line.starts_with("ok ") {
        return Some(Message::Ok(line.to_owned()));
    }
    if line == "err" || line.starts_with("err ") {
        return Some(Message::Err(line.to_owned()));
    }
    if line.starts_with("JBOT") {
        return Some(Message::Banner(line.to_owned()));
    }
    Some(Message::Garbage(line.to_owned()))
}

fn parse_mount(value: &str) -> Option<u8> {
    match value {
        "0" | "1" | "2" | "3" | "4" | "5" | "6" => value.parse().ok(),
        _ => None,
    }
}

fn parse_u32(value: &str) -> Option<u32> {
    value.parse().ok()
}

fn parse_i16(value: &str) -> Option<i16> {
    value.parse().ok()
}

fn parse_u8(value: &str) -> Option<u8> {
    value.parse().ok()
}

fn parse_u8_auto(value: &str) -> Option<u8> {
    if let Some(hex) = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
    {
        u8::from_str_radix(hex, 16).ok()
    } else {
        parse_u8(value)
    }
}

fn parse_f32(value: &str) -> Option<f32> {
    let parsed: f32 = value.parse().ok()?;
    parsed.is_finite().then_some(parsed)
}

fn parse_flag(value: &str) -> Option<bool> {
    match value {
        "0" | "false" | "off" => Some(false),
        "1" | "true" | "on" => Some(true),
        _ => None,
    }
}

fn parse_imu_word(value: &str) -> Option<bool> {
    match value {
        "ok" => Some(true),
        "stale" => Some(false),
        _ => None,
    }
}

fn parse_feedback_word(value: &str) -> Option<bool> {
    match value {
        "ok" => Some(true),
        "none" => Some(false),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FULL: &str = "T step=19043 loop=200 imu=ok cal=3 pitch=1.50 roll=-2.25 hdg=87.40 gy=0.06 gx=-0.12 gz=0.03 err=0x00 mb=ok L=12 R=-13 bat=41.50 tmp=39.3 age=3 estop=0 vset=0.25 wset=-15.00 mnt=1 zof=1.75 zro=-0.50 lvl=0.30 arm=1 trq=123 ref=4.50 dem=-1.25 trm=2.75 x=0.412 mode=2 gtlt=1.23 opm=3 grace=0.45";

    #[test]
    fn parses_a_full_sample() {
        let telemetry = parse_telemetry(FULL).expect("sample should parse");
        assert_eq!(telemetry.step, 19043);
        assert_eq!(telemetry.loop_hz, 200);
        assert!(telemetry.imu_fresh);
        assert_eq!(telemetry.cal, 3);
        assert_eq!(telemetry.pitch_deg, 1.50);
        assert_eq!(telemetry.roll_deg, -2.25);
        assert_eq!(telemetry.heading_deg, 87.40);
        assert_eq!(telemetry.speed_left_rpm, 12);
        assert_eq!(telemetry.speed_right_rpm, -13);
        assert_eq!(telemetry.battery_v, 41.50);
        assert!(!telemetry.estop);
        assert!((telemetry.speed_setpoint_mps - 0.25).abs() < 1e-6);
        assert!((telemetry.steer_setpoint_deg_s() + 15.0).abs() < 1e-5);
        assert!(telemetry.has_orientation);
        assert!(telemetry.has_setpoints);
        assert_eq!(telemetry.mount, Some(1));
        assert_eq!(telemetry.zero_offset_deg, Some(1.75));
        assert_eq!(telemetry.roll_zero_offset_deg, Some(-0.50));
        assert_eq!(telemetry.level_deg, Some(0.30));
        assert_eq!(telemetry.armed, Some(true));
        assert_eq!(telemetry.torque_common_units, Some(123.0));
        assert_eq!(telemetry.pitch_ref_deg, Some(4.50));
        assert_eq!(telemetry.pitch_demand_deg, Some(-1.25));
        assert_eq!(telemetry.slope_trim_deg, Some(2.75));
        assert_eq!(telemetry.position_m, Some(0.412));
        assert_eq!(telemetry.balance_mode, Some(2));
        assert_eq!(telemetry.guard_tilt_deg, Some(1.23));
        assert_eq!(telemetry.op_mode, Some(3));
        assert_eq!(telemetry.raise_grace_s, Some(0.45));
    }

    #[test]
    fn legacy_sample_has_no_mount_fields() {
        let line = "T step=7 loop=199 imu=stale cal=1 pitch=-4.00 gy=0.10 err=0x05 mb=none L=0 R=0 bat=39.20 tmp=31.0 age=4294967295 estop=1";
        let telemetry = parse_telemetry(line).expect("legacy sample should parse");
        assert_eq!(telemetry.mount, None);
        assert_eq!(telemetry.zero_offset_deg, None);
        assert_eq!(telemetry.roll_zero_offset_deg, None);
        assert_eq!(telemetry.level_deg, None);
        assert_eq!(telemetry.armed, None);
    }

    #[test]
    fn rejects_out_of_range_mount() {
        assert!(parse_telemetry("T step=1 pitch=1.0 mnt=7").is_none());
        assert!(parse_telemetry("T step=1 pitch=1.0 mnt=normal").is_none());
        let telemetry = parse_telemetry("T step=1 pitch=1.0 mnt=2").expect("mnt=2 is valid");
        assert_eq!(telemetry.mount, Some(2));
    }

    #[test]
    fn parses_a_legacy_sample_without_orientation_or_setpoints() {
        let line = "T step=7 loop=199 imu=stale cal=1 pitch=-4.00 gy=0.10 err=0x05 mb=none L=0 R=0 bat=39.20 tmp=31.0 age=4294967295 estop=1";
        let telemetry = parse_telemetry(line).expect("legacy sample should parse");
        assert!(!telemetry.imu_fresh);
        assert!(!telemetry.feedback_present);
        assert_eq!(telemetry.sys_err, 5);
        assert!(!telemetry.has_orientation);
        assert!(!telemetry.has_setpoints);
        assert_eq!(telemetry.feedback_age_text(), "--");
    }

    #[test]
    fn ignores_unknown_keys() {
        let line = "T step=1 pitch=2.0 future=42 extra=word";
        let telemetry = parse_telemetry(line).expect("unknown keys are ignored");
        assert_eq!(telemetry.step, 1);
        assert_eq!(telemetry.pitch_deg, 2.0);
    }

    #[test]
    fn rejects_malformed_known_values() {
        assert!(parse_telemetry("T step=abc pitch=1.0").is_none());
        assert!(parse_telemetry("T step=1 pitch=NaN").is_none());
        assert!(parse_telemetry("T step=1 pitch=inf").is_none());
        assert!(parse_telemetry("T loop=1 pitch=1.0").is_none());
        assert!(parse_telemetry("T step=1").is_none());
        assert!(parse_telemetry("step=1 pitch=1.0").is_none());
    }

    #[test]
    fn parses_spin_test_fields() {
        let line = "T step=1 pitch=0.5 arm=0 tsl=150 tsr=-150";
        let telemetry = parse_telemetry(line).expect("sample should parse");
        assert_eq!(telemetry.test_left_units, Some(150.0));
        assert_eq!(telemetry.test_right_units, Some(-150.0));
        // Older firmware without the fields parses fine.
        let legacy = parse_telemetry("T step=1 pitch=0.5").expect("legacy sample");
        assert_eq!(legacy.test_left_units, None);
        assert_eq!(legacy.test_right_units, None);
    }

    #[test]
    fn parses_stat_reply() {
        let line = "stat up=12.5 loop=200 stream=20 trim=0.50 estop=0 steps=2500 imu=2500 imu_err=2 imu_code=1 fb=100 fb_drop=7 fb_crc=3 rx=1800 fb_ok=1 proto=HARDENED stop=pose_error zero=1.75 zroll=-0.50 level=0.30 mount=1 armed=1 kpnm=28.6 kdnm=7.2 kpos=0.40 kv=1.80 ki=0.010 pdem=8.0 ptrm=20.0 zcapt=1 lcapt=1 home=0.00 fall=2 vmax=10.00 pscale=0.50";
        let stat = parse_stat(line).expect("stat should parse");
        assert!((stat.uptime_s - 12.5).abs() < 1e-6);
        assert_eq!(stat.loop_hz, 200);
        assert_eq!(stat.stream_hz, 20);
        assert!((stat.trim_deg - 0.5).abs() < 1e-6);
        assert_eq!(stat.imu_errors, 2);
        assert_eq!(stat.fb_discarded, 7);
        assert_eq!(stat.fb_crc_errors, Some(3));
        assert_eq!(stat.link_bytes, Some(1800));
        assert_eq!(stat.link_fresh, Some(true));
        assert_eq!(stat.link_protocol.as_deref(), Some("HARDENED"));
        assert_eq!(stat.stop_reason.as_deref(), Some("pose_error"));
        assert_eq!(stat.zero_deg, Some(1.75));
        assert_eq!(stat.zroll_deg, Some(-0.50));
        assert_eq!(stat.level_deg, Some(0.30));
        assert_eq!(stat.mount, Some(1));
        assert_eq!(stat.armed, Some(true));
        assert_eq!(stat.kp_nm_per_rad, Some(28.6));
        assert_eq!(stat.kd_nm_per_rad_s, Some(7.2));
        assert_eq!(stat.position_gain_per_s, Some(0.40));
        assert_eq!(stat.velocity_gain_per_s, Some(1.80));
        assert_eq!(stat.trim_gain_rad_per_m_s, Some(0.010));
        assert_eq!(stat.demand_limit_deg, Some(8.0));
        assert_eq!(stat.trim_limit_deg, Some(20.0));
        assert_eq!(stat.zero_captured, Some(true));
        assert_eq!(stat.level_captured, Some(true));
        assert_eq!(stat.fall_events, Some(2));
        assert_eq!(stat.ref_vmax_deg_s, Some(10.00));
        assert_eq!(stat.home_deg, Some(0.0));
        assert_eq!(stat.pscale, Some(0.50));
        assert_eq!(stat.raw, line);
        // Firmware from before the link fields still parses.
        let legacy = parse_stat("stat up=1.0 fb=2").expect("legacy stat");
        assert_eq!(legacy.link_bytes, None);
        assert_eq!(legacy.link_fresh, None);
        assert_eq!(legacy.link_protocol, None);
        assert_eq!(legacy.fb_crc_errors, None);
        assert_eq!(legacy.stop_reason, None);
    }

    #[test]
    fn rejects_unknown_link_protocol_label() {
        let stat = parse_stat("stat up=1.0 proto=OTHER").expect("stat with known up field");
        assert_eq!(stat.link_protocol, None);
    }

    #[test]
    fn rejects_stat_without_known_fields() {
        assert!(parse_stat("stat future=1").is_none());
        assert!(parse_stat("stat ").is_none());
    }

    #[test]
    fn classifies_every_message_kind() {
        assert!(matches!(classify(FULL), Some(Message::Telemetry(_))));
        assert!(matches!(
            classify("stat up=1.0 loop=200"),
            Some(Message::Stat(_))
        ));
        assert!(matches!(classify("ok stream 20"), Some(Message::Ok(_))));
        assert!(matches!(
            classify("err line too long"),
            Some(Message::Err(_))
        ));
        assert!(matches!(
            classify("JBOT pico bridge 0.1 ready"),
            Some(Message::Banner(_))
        ));
        assert!(matches!(
            classify("not a protocol line"),
            Some(Message::Garbage(_))
        ));
        assert!(classify("  ").is_none());
    }

    #[test]
    fn maps_battery_and_wheel_helpers() {
        let telemetry = parse_telemetry(FULL).expect("sample should parse");
        assert!((telemetry.battery_fraction() - (11.5 / 12.0)).abs() < 1e-6);
        assert!((telemetry.wheel_speed_rpm() - 12.5).abs() < 1e-6);
        assert!((telemetry.max_tilt_deg() - 2.25).abs() < 1e-6);
    }
}
