//! USB CDC-ACM console: telemetry out, line commands in.
//!
//! The bench machine has no debug probe, so `defmt` over RTT is invisible
//! during normal operation. This module exposes a serial port on the Pico's
//! USB-C connector; the host sees it as `/dev/cu.usbmodem*`.
//!
//! Protocol (v1), one ASCII line per message:
//!
//! * Host → Pico: `help`, `ping`, `stat`, `stream <hz>`, `trim <deg>`,
//!   `reinit`, `estop`, `clear`, `stats reset`, `reboot`,
//!   `test <units> [units]`, `home <deg>`, `vmax <deg/s>`, `pscale <0..2>`,
//!   `dscale <0..2>`, `slew <units/s>`, `ff [on|off]`, `db <Nm|off>`,
//!   `cal [left right] | [l|r units] | off`, `dither <units|off>`,
//!   `save`, `forget`.
//! * Pico → host: `T key=value ...` telemetry, `ok ...` accepted, `err ...`
//!   rejected, plus a banner on connect.
//!
//! Nothing here is on the control path: a missing host only means lines are
//! dropped, and a slow host cannot stall the control loop. The console task
//! survives a host closing the port: every write has a deadline, so a packet
//! left waiting for a host that stopped draining is abandoned instead of
//! parking the task. Any host can open or close the port without a USB
//! re-enumeration.

use core::fmt::{self, Write};

use embassy_futures::select::{select, Either};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::nvm::{self, Store};
use crate::state::{self, Telemetry};
use jbot_pico_core::mount::Mount;
use jbot_pico_core::nvm::Record;
use jbot_pico_core::spin::{SpinPair, SPIN_TORQUE_LIMIT};

/// USB vendor id used for development boards without an assigned id.
const VID: u16 = 0x1209;
/// USB product id; arbitrary but stable for this firmware.
const PID: u16 = 0x0001;
/// CDC-ACM bulk endpoint size.
const PACKET_SIZE: u16 = 64;
/// Longest accepted command line, including the terminating newline.
const MAX_LINE: usize = 64;
/// Reply buffer; longer replies are truncated rather than dropped.
// Sized for the longest reply (`stat` is ~400 B) with headroom so a new field
// cannot silently truncate the line on the wire.
const REPLY_BUFFER: usize = 512;
/// Banner sent every time the host configures the interface.
const BANNER: &[u8] = b"JBOT pico bridge 0.1 ready\n";
/// Poll interval while the telemetry stream is disabled.
const IDLE_POLL: Duration = Duration::from_millis(200);
/// Time allowed for the final USB packets to leave before a requested reboot.
const REBOOT_FLUSH: Duration = Duration::from_millis(50);
/// Maximum wait for a telemetry packet to be drained. Telemetry may be dropped.
const TELEMETRY_WRITE_TIMEOUT: Duration = Duration::from_millis(100);
/// Maximum wait for a command reply; a command implies a host that reads.
const REPLY_WRITE_TIMEOUT: Duration = Duration::from_millis(1_000);

/// Largest accepted `home` bias away from the captured level, degrees.
///
/// A standing two-wheel machine cannot hold a large commanded pitch: the pitch
/// loop has to trade body angle against holding torque, so a large bias walks
/// the base. Five degrees is enough for a deliberate lean and stays well inside
/// the eight degree dynamic demand limit.
const HOME_LIMIT_DEG: f32 = 5.0;

/// USB device running on the RP2350 peripheral.
pub type Usb = UsbDevice<'static, Driver<'static, USB>>;
/// CDC-ACM serial class carrying the console.
pub type Console = CdcAcmClass<'static, Driver<'static, USB>>;

static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR: StaticCell<[u8; 512]> = StaticCell::new();
static CONTROL_BUFFER: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State<'static>> = StaticCell::new();

/// Build the USB device and its single CDC-ACM interface.
pub fn new(driver: Driver<'static, USB>) -> (Usb, Console) {
    let mut config = Config::new(VID, PID);
    config.manufacturer = Some("JBOT");
    config.product = Some("JBOT pico bridge");
    config.serial_number = Some("JBOT-PICO-01");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 512]),
        CONTROL_BUFFER.init([0; 64]),
    );

    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), PACKET_SIZE);
    (builder.build(), class)
}

/// Fixed-capacity writer for `core::fmt`.
struct LineWriter<'a> {
    buffer: &'a mut [u8],
    len: usize,
}

impl Write for LineWriter<'_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len + text.len();
        if end > self.buffer.len() {
            return Err(fmt::Error);
        }
        self.buffer[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// Push bytes, splitting them into endpoint-sized packets.
async fn write_bytes(class: &mut Console, bytes: &[u8]) -> Result<(), ()> {
    for chunk in bytes.chunks(PACKET_SIZE as usize) {
        class.write_packet(chunk).await.map_err(|_| ())?;
    }
    Ok(())
}

/// Write with a deadline.
///
/// `write_packet` waits until the host drains the IN endpoint. A host that
/// closes the port can leave that wait pending forever, parking the console
/// task even though the endpoint is still enabled. Cancelling the wait is safe:
/// the RP driver only touches the endpoint buffer after it reports ready, so a
/// dropped future leaves no half-written packet behind. The line may be lost,
/// which is the documented telemetry overflow policy.
async fn write_bounded(class: &mut Console, bytes: &[u8], timeout: Duration) -> bool {
    match select(write_bytes(class, bytes), Timer::after(timeout)).await {
        Either::First(result) => result.is_ok(),
        Either::Second(()) => {
            // The cancelled write left its IN endpoint marked `available`: the
            // hardware is still waiting for a host that has already closed the
            // port. Every later write would park on that same flag. Clear it so
            // the next session can write again. The console task is the only
            // owner of these endpoints and EP0 stays owned by the USB stack.
            for index in 1..16usize {
                rp_pac::USB_DPRAM
                    .ep_in_buffer_control(index)
                    .modify(|w| w.set_available(0, false));
            }
            false
        }
    }
}

/// Format one line into `buffer` and return the used slice.
fn format<'a>(buffer: &'a mut [u8], args: fmt::Arguments<'_>) -> &'a [u8] {
    let mut writer = LineWriter { buffer, len: 0 };
    let _ = writer.write_fmt(args);
    let _ = writer.write_str("\n");
    let len = writer.len;
    &writer.buffer[..len]
}

/// Send one formatted reply line.
async fn reply(class: &mut Console, args: fmt::Arguments<'_>) -> bool {
    let mut buffer = [0u8; REPLY_BUFFER];
    let len = format(&mut buffer, args).len();
    write_bounded(class, &buffer[..len], REPLY_WRITE_TIMEOUT).await
}

/// Parse an unsigned decimal.
fn parse_u32(text: &str) -> Option<u32> {
    text.parse().ok()
}

/// Parse a signed float.
fn parse_f32(text: &str) -> Option<f32> {
    text.parse().ok()
}

/// Optional pose reference for a reply; `NaN` means it has not been measured.
fn pose_value(value: Option<f32>) -> f32 {
    value.unwrap_or(f32::NAN)
}

/// What the console task should do after a command.
enum Action {
    /// Keep serving.
    Continue,
    /// Reset the chip after flushing the reply.
    Reboot,
}

/// Settings snapshot that is worth keeping across a power cycle.
///
/// The mount is always meaningful; the zero offsets and the level reference
/// only exist once they have been captured in this session, so an uncaptured
/// value is stored as zero rather than as a stale number from a previous
/// mount.
fn current_record() -> Option<Record> {
    let (zero_pitch_rad, zero_roll_rad) = if state::zero_captured() {
        (state::zero_pitch_rad(), state::zero_roll_rad())
    } else {
        (0.0, 0.0)
    };
    let level_rad = if state::level_captured() {
        state::pitch_trim_rad()
    } else {
        0.0
    };
    Record::new(
        u8::try_from(state::mount()).ok()?,
        zero_pitch_rad,
        zero_roll_rad,
        level_rad,
        state::pose_back_deg(),
        state::pose_face_deg(),
        state::pose_home_deg(),
    )
    .map(|record| record.with_zero_captured(state::zero_captured()))
}

/// Write the current settings record to flash.
///
/// Blocking: the erase parks the XIP cache for tens of milliseconds, so this
/// must only run while the balance law is disarmed. Every caller disarms first
/// or refuses when armed.
fn save_settings(flash: &mut Store) -> bool {
    current_record().is_some_and(|record| nvm::save(flash, record).is_ok())
}

/// Report a failed settings write.
///
/// Success is silent on purpose: the host parses `ok level <deg>` and
/// `ok zero pitch=... roll=...` positionally, so those reply lines must keep
/// their shape. A failed write is a real fault (the settings would not survive
/// the next power cycle), so it is reported as an `err` after the success
/// line.
async fn report_save_failure(class: &mut Console, saved: bool, context: &str) {
    if !saved {
        reply(
            class,
            format_args!("err nvm write failed: {context} not persisted"),
        )
        .await;
    }
}

/// Execute one command line. Replies are written by this function.
async fn handle_command(class: &mut Console, flash: &mut Store, line: &[u8]) -> Action {
    let text = match core::str::from_utf8(line) {
        Ok(text) => text.trim(),
        Err(_) => {
            reply(class, format_args!("err line is not utf8")).await;
            return Action::Continue;
        }
    };

    let mut parts = text.split_whitespace();
    let Some(command) = parts.next() else {
        return Action::Continue;
    };
    let argument = parts.next();

    match command {
        "help" | "?" => {
            reply(
                class,
                format_args!(
                    "ok help: ping | stat | stream <hz> | speed <mps> | steer <deg/s> | trim <deg> | zero | level | pose [back|face|home] [deg] | mount <name> | home [deg] | vmax [deg/s] | tmax [Nm|off] | pscale [0..2] | dscale [0..2] | iscale [0..2] | posg [0..5|off] | velg [0..10|off] | demlim [deg|off] | slew [units/s] | ff [on|off] | db [Nm|off] | cal [left right] | cal l|r <units> | cal off | dither <units|off> | predict <ms|off> | arm | disarm | fall | test <units> [units] | reinit | estop | clear | save | forget | stats reset | reboot"
                ),
            )
            .await;
        }
        "ping" => {
            reply(class, format_args!("ok ping")).await;
        }
        "stat" => {
            let uptime_s = Instant::now().as_millis() as f32 / 1_000.0;
            // Report the *effective* law: gains after the bench power scale,
            // so what the console prints is what the loop runs.
            let pscale = state::pscale();
            let tuning = state::tuning()
                .scaled_by(pscale)
                .with_deadband(state::deadband_nm())
                .with_derivative_scale(state::dscale());
            reply(
                class,
                format_args!(
                    "stat up={:.1} loop={} stream={} trim={:.2} estop={} steps={} imu={} imu_err={} imu_code={} fb={} fb_drop={} rx={} fb_ok={} stop={} zero={:.2} zroll={:.2} level={:.2} mount={} nvm={} kpnm={:.1} kdnm={:.1} kpos={:.2} kv={:.2} ki={:.3} pdem={:.1} ptrm={:.1} armed={} zcapt={} lcapt={} home={:.2} fall={} vmax={:.2} pscale={:.2} dscale={:.2} slew={:.0} db={:.2} cbl={:.0} cbr={:.0} dth={:.0} tmax={:.1} al={} ar={}",
                    uptime_s,
                    state::loop_hz(),
                    state::stream_hz(),
                    state::pitch_trim_rad().to_degrees(),
                    u8::from(state::estop()),
                    state::read(&state::CONTROL_STEPS),
                    state::read(&state::IMU_SAMPLES),
                    state::read(&state::IMU_ERRORS),
                    state::read(&state::IMU_LAST_ERROR),
                    state::read(&state::FB_FRAMES),
                    state::read(&state::FB_DISCARDED),
                    state::read(&state::LINK_BYTES),
                    u8::from(state::link_fresh()),
                    state::stop_reason_label(),
                    state::zero_pitch_rad().to_degrees(),
                    state::zero_roll_rad().to_degrees(),
                    state::pitch_trim_rad().to_degrees(),
                    state::mount(),
                    u8::from(state::nvm_present()),
                    tuning.pitch_kp_nm_per_rad,
                    tuning.pitch_kd_nm_per_rad_s,
                    tuning.position_gain_per_s,
                    tuning.velocity_gain_per_s,
                    tuning.trim_gain_rad_per_m_s,
                    tuning.pitch_demand_limit_rad.to_degrees(),
                    tuning.pitch_trim_limit_rad.to_degrees(),
                    u8::from(state::armed()),
                    u8::from(state::zero_captured()),
                    u8::from(state::level_captured()),
                    state::home_target_rad().map_or_else(
                        || state::pitch_trim_rad().to_degrees(),
                        f32::to_degrees,
                    ),
                    state::read(&state::FALL_EVENTS),
                    state::ref_v_max_rad_s().to_degrees(),
                    pscale,
                    state::dscale(),
                    state::torque_slew_per_s(),
                    state::deadband_nm(),
                    state::cal_left_units(),
                    state::cal_right_units(),
                    state::dither_units(),
                    state::max_torque_nm(),
                    state::feedback_applied().0,
                    state::feedback_applied().1,
                ),
            )
            .await;
        }
        "stream" => match argument {
            None => {
                reply(class, format_args!("ok stream {}", state::stream_hz())).await;
            }
            Some(value) => match parse_u32(value) {
                Some(hz) => {
                    let applied = state::set_stream_hz(hz);
                    reply(class, format_args!("ok stream {}", applied)).await;
                }
                None => {
                    reply(
                        class,
                        format_args!("err stream needs 0..{}", state::MAX_STREAM_HZ),
                    )
                    .await;
                }
            },
        },
        "trim" => match argument {
            None => {
                reply(
                    class,
                    format_args!("ok trim {:.2}", state::pitch_trim_rad().to_degrees()),
                )
                .await;
            }
            Some(value) => match parse_f32(value) {
                Some(deg) if deg.is_finite() && (-90.0..=90.0).contains(&deg) => {
                    state::set_pitch_trim_rad(deg.to_radians());
                    reply(class, format_args!("ok trim {:.2}", deg)).await;
                }
                _ => {
                    reply(class, format_args!("err trim needs degrees in -90..90")).await;
                }
            },
        },
        "speed" => match argument {
            None => {
                reply(
                    class,
                    format_args!("ok speed {:.2}", state::speed_setpoint_mps()),
                )
                .await;
            }
            Some(value) => match parse_f32(value) {
                Some(mps) if mps.is_finite() && (-5.0..=5.0).contains(&mps) => {
                    state::set_speed_setpoint_mps(mps);
                    reply(class, format_args!("ok speed {:.2}", mps)).await;
                }
                _ => {
                    reply(class, format_args!("err speed needs mps in -5..5")).await;
                }
            },
        },
        "steer" => match argument {
            None => {
                reply(
                    class,
                    format_args!("ok steer {:.2}", state::steer_setpoint_rps().to_degrees()),
                )
                .await;
            }
            Some(value) => match parse_f32(value) {
                Some(deg) if deg.is_finite() && (-180.0..=180.0).contains(&deg) => {
                    state::set_steer_setpoint_rps(deg.to_radians());
                    reply(class, format_args!("ok steer {:.2}", deg)).await;
                }
                _ => {
                    reply(
                        class,
                        format_args!("err steer needs degrees/s in -180..180"),
                    )
                    .await;
                }
            },
        },
        "test" => {
            if state::estop() {
                reply(class, format_args!("err test refused: e-stop latched")).await;
            } else if state::armed() {
                reply(
                    class,
                    format_args!("err test refused: disarm the balance law first"),
                )
                .await;
            } else {
                let second = parts.next();
                match (argument, second) {
                    (None, _) => {
                        let applied = state::spin_applied();
                        reply(
                            class,
                            format_args!("ok test l={:.0} r={:.0}", applied.left, applied.right),
                        )
                        .await;
                    }
                    (Some(left), right) => {
                        let right = right.unwrap_or(left);
                        match (parse_f32(left), parse_f32(right)) {
                            (Some(l), Some(r))
                                if l.is_finite()
                                    && r.is_finite()
                                    && l.abs() <= SPIN_TORQUE_LIMIT
                                    && r.abs() <= SPIN_TORQUE_LIMIT =>
                            {
                                state::request_spin(SpinPair { left: l, right: r });
                                reply(class, format_args!("ok test l={:.0} r={:.0}", l, r)).await;
                            }
                            _ => {
                                reply(
                                    class,
                                    format_args!(
                                        "err test needs units in -{:.0}..{:.0}",
                                        SPIN_TORQUE_LIMIT, SPIN_TORQUE_LIMIT
                                    ),
                                )
                                .await;
                            }
                        }
                    }
                }
            }
        }
        "zero" => {
            if !state::imu_fresh() {
                reply(class, format_args!("err zero needs a fresh IMU sample")).await;
            } else {
                let (pitch, roll) = state::latest_mapped_attitude();
                state::set_zero_pitch_rad(pitch.to_radians());
                state::set_zero_roll_rad(roll.to_radians());
                state::set_zero_captured(true);
                state::set_armed(false);
                // The home reference follows the captured calibration; drop
                // any explicit override.
                state::clear_home_target();
                // Persist right away: the operator just proved that the mount
                // maps this pose correctly, which is exactly the state worth
                // surviving a power cycle.
                let saved = save_settings(flash);
                reply(
                    class,
                    format_args!("ok zero pitch={:.2} roll={:.2}", pitch, roll),
                )
                .await;
                report_save_failure(class, saved, "zero").await;
            }
        }
        "level" => {
            if !state::imu_fresh() {
                reply(class, format_args!("err level needs a fresh IMU sample")).await;
            } else {
                let (pitch, _) = state::latest_mapped_attitude();
                let level = pitch - state::zero_pitch_rad().to_degrees();
                state::set_pitch_trim_rad(level.to_radians());
                state::set_level_captured(true);
                state::set_armed(false);
                // The home reference follows the captured level; drop any
                // explicit override.
                state::clear_home_target();
                let saved = save_settings(flash);
                reply(class, format_args!("ok level {:.2}", level)).await;
                report_save_failure(class, saved, "level").await;
            }
        }
        "pose" => match argument {
            None => {
                reply(
                    class,
                    format_args!(
                        "ok pose back={:.2} face={:.2} home={:.2}",
                        pose_value(state::pose_back_deg()),
                        pose_value(state::pose_face_deg()),
                        pose_value(state::pose_home_deg())
                    ),
                )
                .await;
            }
            Some(name) => {
                let captured = match parts.next() {
                    Some(text) => parse_f32(text),
                    None if state::imu_fresh() => Some(state::latest_mapped_attitude().0),
                    None => {
                        reply(class, format_args!("err pose needs a fresh IMU sample")).await;
                        return Action::Continue;
                    }
                };
                match captured {
                    Some(deg) if deg.is_finite() && deg.abs() <= 180.0 => {
                        match name {
                            "back" => state::set_pose_back_deg(Some(deg)),
                            "face" => state::set_pose_face_deg(Some(deg)),
                            "home" => {
                                state::set_pose_home_deg(Some(deg));
                                // The vertical reference doubles as the pitch zero so a
                                // captured attitude survives a reboot without a fresh
                                // calibration. Roll is left untouched: tilts are not
                                // part of this stage.
                                state::set_zero_pitch_rad(deg.to_radians());
                                state::set_zero_captured(true);
                                state::set_armed(false);
                                state::clear_home_target();
                            }
                            _ => {
                                reply(class, format_args!("err pose name is back, face or home"))
                                    .await;
                                return Action::Continue;
                            }
                        }
                        let saved = save_settings(flash);
                        if saved {
                            state::set_nvm_present(true);
                        }
                        reply(class, format_args!("ok pose {name} {deg:.2}")).await;
                        report_save_failure(class, saved, "pose").await;
                    }
                    _ => {
                        reply(class, format_args!("err pose needs degrees in -180..180")).await;
                    }
                }
            }
        },
        "home" => {
            // Set the home reference: a small pitch bias away from the
            // captured level. The balance law ramps there smoothly from the
            // armed pose. The limit is deliberately tight, because a standing
            // machine cannot hold a large commanded pitch while stationary;
            // the vertical equilibrium (captured level) is the default when no
            // angle is given.
            match argument {
                None => {
                    let target = state::pitch_trim_rad();
                    state::set_home_target_rad(target);
                    reply(class, format_args!("ok home {:.2}", target.to_degrees())).await;
                }
                Some(value) => match parse_f32(value) {
                    Some(deg)
                        if deg.is_finite() && (-HOME_LIMIT_DEG..=HOME_LIMIT_DEG).contains(&deg) =>
                    {
                        state::set_home_target_rad(deg.to_radians());
                        reply(class, format_args!("ok home {:.2}", deg)).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!("err home needs degrees in -{HOME_LIMIT_DEG:.0}..{HOME_LIMIT_DEG:.0}"),
                        )
                        .await;
                    }
                },
            }
        }
        "vmax" => {
            // Commanded reference rate for the home ramp, in degrees per
            // second. Zero freezes the ramp in place (the law holds the
            // current reference); the cap is enforced by the reference
            // generator itself.
            match argument {
                None => {
                    reply(
                        class,
                        format_args!("ok vmax {:.2}", state::ref_v_max_rad_s().to_degrees()),
                    )
                    .await;
                }
                Some(value) => match parse_f32(value) {
                    Some(deg_s) if deg_s.is_finite() && (0.0..=60.0).contains(&deg_s) => {
                        state::set_ref_v_max_rad_s(deg_s.to_radians());
                        reply(class, format_args!("ok vmax {:.2}", deg_s)).await;
                    }
                    _ => {
                        reply(class, format_args!("err vmax needs degrees/s in 0..60")).await;
                    }
                },
            }
        }
        "slew" => {
            // Runtime rate limit on the applied wheel torque, in command
            // units per second. Lower values give a softer, slower torque
            // build-up without weakening the balance law itself; the gain is
            // set by `pscale`. Runtime-only, like `pscale`.
            match argument {
                None => {
                    reply(
                        class,
                        format_args!("ok slew {:.0}", state::torque_slew_per_s()),
                    )
                    .await;
                }
                Some(value) => match parse_f32(value) {
                    Some(units_per_s)
                        if units_per_s.is_finite()
                            && (state::MIN_TORQUE_SLEW_PER_S..=state::MAX_TORQUE_SLEW_PER_S)
                                .contains(&units_per_s) =>
                    {
                        let applied = state::set_torque_slew_per_s(units_per_s);
                        reply(class, format_args!("ok slew {:.0}", applied)).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!(
                                "err slew needs units/s in {:.0}..={:.0}",
                                state::MIN_TORQUE_SLEW_PER_S,
                                state::MAX_TORQUE_SLEW_PER_S
                            ),
                        )
                        .await;
                    }
                },
            }
        }
        "tmax" => {
            // Common-mode torque budget in Nm. It drives the Nm-to-unit
            // conversion, the torque limit and the derived guard thresholds
            // at once. Runtime-only: the compiled plant value is restored at
            // boot so a bench measurement cannot leak into a fresh session.
            match argument {
                None => {
                    reply(class, format_args!("ok tmax {:.1}", state::max_torque_nm())).await;
                }
                Some("off") => {
                    let applied = state::set_max_torque_nm(f32::NAN);
                    reply(class, format_args!("ok tmax {applied:.1}")).await;
                }
                Some(value) => match parse_f32(value) {
                    Some(nm)
                        if nm.is_finite()
                            && (state::MIN_MAX_TORQUE_NM..=state::MAX_MAX_TORQUE_NM)
                                .contains(&nm) =>
                    {
                        let applied = state::set_max_torque_nm(nm);
                        reply(class, format_args!("ok tmax {applied:.1}")).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!(
                                "err tmax needs {:.0}..={:.0} Nm or off",
                                state::MIN_MAX_TORQUE_NM,
                                state::MAX_MAX_TORQUE_NM
                            ),
                        )
                        .await;
                    }
                },
            }
        }
        "pscale" => {
            // Bench power scale for the balance law: a runtime multiplier on
            // the angular-output gains (pitch kp/kd and the yaw differential).
            // Runtime-only and resets to 1.0 at boot, so a bench session
            // cannot leak a weakened law into the next power-up. The fall
            // guards and the torque budget do not scale with this value.
            match argument {
                None => {
                    reply(class, format_args!("ok pscale {:.2}", state::pscale())).await;
                }
                Some(value) => match parse_f32(value) {
                    Some(scale)
                        if scale.is_finite()
                            && (0.0..=jbot_pico_core::control::MAX_PSCALE).contains(&scale) =>
                    {
                        let applied = state::set_pscale(scale);
                        reply(class, format_args!("ok pscale {:.2}", applied)).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!(
                                "err pscale needs 0..={:.1} (1.0 = compiled tuning, 0 = law inert)",
                                jbot_pico_core::control::MAX_PSCALE
                            ),
                        )
                        .await;
                    }
                },
            }
        }
        "dscale" => {
            // Derivative-only bench scale. Runtime-only, like `pscale`.
            match argument {
                None => {
                    reply(class, format_args!("ok dscale {:.2}", state::dscale())).await;
                }
                Some(value) => match parse_f32(value) {
                    Some(scale)
                        if scale.is_finite()
                            && (0.0..=jbot_pico_core::control::MAX_DSCALE).contains(&scale) =>
                    {
                        let applied = state::set_dscale(scale);
                        reply(class, format_args!("ok dscale {:.2}", applied)).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!(
                                "err dscale needs 0..={:.1} (1.0 = compiled damping)",
                                jbot_pico_core::control::MAX_DSCALE
                            ),
                        )
                        .await;
                    }
                },
            }
        }
        "iscale" => {
            // Integral-only bench scale. Runtime-only, like `pscale`/`dscale`.
            match argument {
                None => {
                    reply(class, format_args!("ok iscale {:.2}", state::iscale())).await;
                }
                Some(value) => match parse_f32(value) {
                    Some(scale) if scale.is_finite() && (0.0..=2.0).contains(&scale) => {
                        let applied = state::set_iscale(scale);
                        reply(class, format_args!("ok iscale {:.2}", applied)).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!("err iscale needs 0..2 (1.0 = compiled)"),
                        )
                        .await;
                    }
                },
            }
        }
        "posg" | "velg" | "demlim" => {
            // Runtime outer-loop overrides for bench identification. `off`
            // means zero for the gains and zero degrees for the demand cap.
            let name = match command {
                "posg" => "posg",
                "velg" => "velg",
                _ => "demlim",
            };
            match argument {
                None => {
                    let current = match name {
                        "posg" => state::position_gain(),
                        "velg" => state::velocity_gain(),
                        _ => state::demand_limit_rad().to_degrees(),
                    };
                    reply(class, format_args!("ok {name} {current:.2}")).await;
                }
                Some(value) => {
                    let applied = if value == "off" {
                        match name {
                            "posg" => state::set_position_gain(0.0),
                            "velg" => state::set_velocity_gain(0.0),
                            _ => state::set_demand_limit_deg(0.0),
                        }
                    } else {
                        match parse_f32(value) {
                            Some(number) if number.is_finite() && number >= 0.0 => match name {
                                "posg" => state::set_position_gain(number),
                                "velg" => state::set_velocity_gain(number),
                                _ => state::set_demand_limit_deg(number),
                            },
                            _ => {
                                reply(class, format_args!("err {name} needs 0.. or off")).await;
                                return Action::Continue;
                            }
                        }
                    };
                    reply(class, format_args!("ok {name} {applied:.2}")).await;
                }
            }
        }
        "dbg" => {
            // Bench-only bridge to the STM32 debug serial on USART3. `dbg on`
            // forwards its output to this console; the rest of the line is
            // sent verbatim as one debug command.
            match argument {
                None => {
                    reply(
                        class,
                        format_args!(
                            "ok dbg {}",
                            if state::debug_bridge_enabled() {
                                "on"
                            } else {
                                "off"
                            }
                        ),
                    )
                    .await;
                }
                Some("on") => {
                    state::set_debug_bridge(true);
                    reply(class, format_args!("ok dbg on")).await;
                }
                Some("off") => {
                    state::set_debug_bridge(false);
                    reply(class, format_args!("ok dbg off")).await;
                }
                Some(_) => {
                    let payload = jbot_pico_core::text::command_tail(text);
                    crate::debug_bridge::push_tx(payload.as_bytes());
                    crate::debug_bridge::push_tx(b"\r");
                    reply(class, format_args!("ok dbg tx {}", payload.len())).await;
                }
            }
        }
        "ff" => {
            // Runtime gate for the calibrated feedforward term. Off at boot,
            // so an empirical bench curve never leaks into a fresh session.
            match argument {
                None => {
                    reply(
                        class,
                        format_args!("ok ff {}", state::feedforward_enabled()),
                    )
                    .await;
                }
                Some(value) => match value {
                    "on" => {
                        state::set_feedforward_enabled(true);
                        reply(class, format_args!("ok ff on")).await;
                    }
                    "off" => {
                        state::set_feedforward_enabled(false);
                        reply(class, format_args!("ok ff off")).await;
                    }
                    _ => {
                        reply(class, format_args!("err ff needs on or off")).await;
                    }
                },
            }
        }
        "db" => {
            // Runtime inverse of the measured drive dead zone. Off at boot so
            // a bench value cannot leak into a fresh session.
            match argument {
                None => {
                    reply(class, format_args!("ok db {:.2}", state::deadband_nm())).await;
                }
                Some("off") => {
                    state::set_deadband_nm(0.0);
                    reply(class, format_args!("ok db off")).await;
                }
                Some(value) => match value.parse::<f32>() {
                    Ok(nm) if (0.0..=state::MAX_DEADBAND_NM).contains(&nm) => {
                        let applied = state::set_deadband_nm(nm);
                        reply(class, format_args!("ok db {applied:.2}")).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!(
                                "err db needs 0..={:.1} Nm or off",
                                state::MAX_DEADBAND_NM
                            ),
                        )
                        .await;
                    }
                },
            }
        }
        "cal" => {
            // Per-wheel breakaway compensation, in EFeru command units.
            // Runtime-only and off at boot: a threshold measured on the bench
            // must not leak into a fresh session. The control task applies it
            // to both the `test` and the balance path at one choke point.
            match argument {
                None => {
                    reply(
                        class,
                        format_args!(
                            "ok cal l={:.0} r={:.0}",
                            state::cal_left_units(),
                            state::cal_right_units()
                        ),
                    )
                    .await;
                }
                Some("off") => {
                    state::clear_cal();
                    reply(class, format_args!("ok cal off")).await;
                }
                Some(side @ ("l" | "r")) => match parts.next().and_then(parse_f32) {
                    Some(units) if units.is_finite() => {
                        let applied = if side == "l" {
                            state::set_cal_left_units(units)
                        } else {
                            state::set_cal_right_units(units)
                        };
                        reply(class, format_args!("ok cal {side} {applied:.0}")).await;
                    }
                    _ => {
                        reply(class, format_args!("err cal {side} needs units")).await;
                    }
                },
                Some(left) => match (parse_f32(left), parts.next().and_then(parse_f32)) {
                    (Some(l), Some(r)) if l.is_finite() && r.is_finite() => {
                        let l = state::set_cal_left_units(l);
                        let r = state::set_cal_right_units(r);
                        reply(class, format_args!("ok cal l={l:.0} r={r:.0}")).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!("err cal needs 'left right' units, l|r <units>, or off"),
                        )
                        .await;
                    }
                },
            }
        }
        "dither" => {
            // Small common-mode triangle wave at DITHER_HZ, added after the
            // breakaway inverse to keep the drive moving through stiction.
            // Runtime-only and off at boot; the e-stop path never dither.
            match argument {
                None => {
                    reply(
                        class,
                        format_args!("ok dither {:.0}", state::dither_units()),
                    )
                    .await;
                }
                Some("off") => {
                    state::set_dither_units(0.0);
                    reply(class, format_args!("ok dither off")).await;
                }
                Some(value) => match parse_f32(value) {
                    Some(units) if units.is_finite() => {
                        let applied = state::set_dither_units(units);
                        reply(class, format_args!("ok dither {applied:.0}")).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!(
                                "err dither needs 0..={:.0} units or off",
                                state::MAX_DITHER_UNITS
                            ),
                        )
                        .await;
                    }
                },
            }
        }
        "predict" => {
            // Smith-predictor horizon for the delayed hub drive. Runtime-only
            // and off at boot: delay compensation has to be measured first.
            match argument {
                None => {
                    reply(class, format_args!("ok predict {:.0}", state::predict_ms())).await;
                }
                Some("off") => {
                    state::set_predict_ms(0.0);
                    reply(class, format_args!("ok predict off")).await;
                }
                Some(value) => match parse_f32(value) {
                    Some(ms) if ms.is_finite() => {
                        let applied = state::set_predict_ms(ms);
                        reply(class, format_args!("ok predict {applied:.0}")).await;
                    }
                    _ => {
                        reply(
                            class,
                            format_args!(
                                "err predict needs 0..={:.0} ms or off",
                                state::MAX_PREDICT_MS
                            ),
                        )
                        .await;
                    }
                },
            }
        }
        "mount" => {
            match argument {
                None => {
                    let mount = Mount::from_u32(state::mount());
                    reply(
                        class,
                        format_args!("ok mount {} {}", mount.as_u32(), mount.name()),
                    )
                    .await;
                }
                Some(value) => match Mount::from_name(value) {
                    Some(mount) => {
                        state::set_mount(mount.as_u32());
                        // Zero and level were captured against the previous
                        // attitude mapping; they are almost never valid for the
                        // new mount and must be re-captured.
                        state::set_zero_captured(false);
                        state::set_level_captured(false);
                        state::set_armed(false);
                        state::clear_home_target();
                        let saved = save_settings(flash);
                        reply(
                            class,
                            format_args!(
                                "ok mount {} {} (re-capture zero and level)",
                                mount.as_u32(),
                                mount.name()
                            ),
                        )
                        .await;
                        report_save_failure(class, saved, "mount").await;
                    }
                    None => {
                        reply(
                        class,
                        format_args!("err mount needs normal, flipx, flipy, rotxp, rotxm, rotyp or rotym"),
                    )
                    .await;
                    }
                },
            }
        }
        "arm" => {
            if state::estop() {
                reply(class, format_args!("err arm refused: e-stop latched")).await;
            } else if !state::imu_fresh() {
                reply(class, format_args!("err arm refused: IMU sample is stale")).await;
            } else if !state::link_fresh() {
                reply(
                    class,
                    format_args!("err arm refused: no mainboard feedback (fb=0, check the link)"),
                )
                .await;
            } else if !state::zero_captured() {
                reply(class, format_args!("err arm refused: capture zero first")).await;
            } else if !state::level_captured() {
                reply(class, format_args!("err arm refused: capture level first")).await;
            } else {
                state::clear_spin_request();
                state::set_armed(true);
                reply(class, format_args!("ok armed")).await;
            }
        }
        "disarm" => {
            state::clear_spin_request();
            state::set_armed(false);
            reply(class, format_args!("ok disarmed")).await;
        }
        "reinit" => {
            state::REINIT.signal(());
            reply(class, format_args!("ok reinit")).await;
        }
        "fall" => {
            if !state::armed() {
                reply(class, format_args!("err fall needs an armed balance law")).await;
            } else {
                state::SAFE_FALL_REQUEST.signal(());
                reply(class, format_args!("ok safe fall requested")).await;
            }
        }
        "estop" => {
            state::set_estop(true);
            reply(class, format_args!("ok estop")).await;
        }
        "clear" => {
            state::set_estop(false);
            state::set_stop(None);
            reply(class, format_args!("ok clear")).await;
        }
        "save" => {
            // Persist mount, zero offsets and level. A flash erase stalls the
            // whole executor, so it is only allowed with the law disarmed:
            // nothing is commanding torque, and the mainboard keeps its own
            // serial timeout.
            if state::armed() {
                reply(
                    class,
                    format_args!(
                        "err save refused: disarm first (the flash write stalls the loop)"
                    ),
                )
                .await;
            } else if save_settings(flash) {
                state::set_nvm_present(true);
                reply(class, format_args!("ok save nvm=stored")).await;
            } else {
                reply(class, format_args!("err save failed")).await;
            }
        }
        "forget" => {
            if state::armed() {
                reply(class, format_args!("err forget refused: disarm first")).await;
            } else {
                match nvm::forget(flash) {
                    Ok(()) => {
                        state::set_nvm_present(false);
                        // A construction change invalidates geometry references.
                        // The saved pose table must not survive as a plausible number.
                        state::set_pose_back_deg(None);
                        state::set_pose_face_deg(None);
                        state::set_pose_home_deg(None);
                        reply(class, format_args!("ok forget nvm=blank")).await;
                    }
                    Err(()) => {
                        reply(class, format_args!("err forget failed")).await;
                    }
                }
            }
        }
        "stats" => {
            if argument == Some("reset") {
                state::reset_counters();
                reply(class, format_args!("ok stats reset")).await;
            } else {
                reply(class, format_args!("err use: stats reset")).await;
            }
        }
        "reboot" => {
            reply(class, format_args!("ok reboot")).await;
            return Action::Reboot;
        }
        other => {
            reply(class, format_args!("err unknown command: {}", other)).await;
        }
    }

    Action::Continue
}

/// Format one telemetry line.
fn format_telemetry<'a>(buffer: &'a mut [u8], telemetry: &Telemetry) -> &'a [u8] {
    format(
        buffer,
        format_args!(
            "T step={} loop={} imu={} cal={} pitch={:.2} roll={:.2} hdg={:.2} gy={:.2} gx={:.2} gz={:.2} err=0x{:02x} mb={} L={} R={} bat={:.2} tmp={:.1} age={} estop={} vset={:.2} wset={:.2} mnt={} zof={:.2} lvl={:.2} zro={:.2} arm={} tsl={:.0} tsr={:.0} trq={:.0} ref={:.2} dem={:.2} trm={:.2} x={:.3} mode={} gtlt={:.2} opm={} grace={:.2} ff={:.0} cbl={:.0} cbr={:.0} cwl={:.1} cwr={:.1} dth={:.0} dthv={:.1} al={} ar={}",
            telemetry.control_steps,
            telemetry.loop_hz,
            if telemetry.imu_fresh { "ok" } else { "stale" },
            telemetry.imu_system_cal,
            telemetry.pitch_deg,
            telemetry.roll_deg,
            telemetry.heading_deg,
            telemetry.gyro_y_dps,
            telemetry.gyro_x_dps,
            telemetry.gyro_z_dps,
            telemetry.sys_err,
            if telemetry.feedback_present { "ok" } else { "none" },
            telemetry.speed_left_rpm,
            telemetry.speed_right_rpm,
            telemetry.battery_v,
            telemetry.board_temp_c,
            telemetry.feedback_age_ms,
            u8::from(telemetry.estop),
            telemetry.speed_setpoint_mps,
            telemetry.steer_setpoint_rps.to_degrees(),
            telemetry.mount,
            telemetry.zero_offset_deg,
            telemetry.level_deg,
            telemetry.roll_zero_offset_deg,
            u8::from(telemetry.armed),
            telemetry.spin_left_units,
            telemetry.spin_right_units,
            telemetry.balance_torque_common_units,
            telemetry.pitch_ref_deg,
            telemetry.pitch_demand_deg,
            telemetry.slope_trim_deg,
            telemetry.position_m,
            telemetry.balance_mode,
            telemetry.guard_tilt_deg,
            telemetry.op_mode,
            telemetry.raise_grace_s,
            telemetry.feedforward_units,
            telemetry.cal_left_units,
            telemetry.cal_right_units,
            telemetry.cmd_left_units,
            telemetry.cmd_right_units,
            telemetry.dither_units,
            telemetry.dither_applied_units,
            telemetry.applied_left_units,
            telemetry.applied_right_units,
        ),
    )
}

/// Keep the USB stack running. Never returns.
#[embassy_executor::task]
pub async fn usb_task(mut usb: Usb) {
    usb.run().await
}

/// Serve the console: commands in, telemetry out.
#[embassy_executor::task]
pub async fn console_task(mut class: Console, mut flash: Store) {
    let mut packet = [0u8; PACKET_SIZE as usize];
    let mut line = [0u8; MAX_LINE];
    let mut telemetry_buffer = [0u8; REPLY_BUFFER];

    loop {
        class.wait_connection().await;

        // Close any half-written telemetry line left in the host's buffer by a
        // previous session, so the first command of this one cannot be merged
        // into that fragment and silently lost.
        let _ = write_bounded(&mut class, b"\n", REPLY_WRITE_TIMEOUT).await;
        let _ = write_bounded(&mut class, BANNER, REPLY_WRITE_TIMEOUT).await;

        // Say what the flash record contains. This is the first thing the
        // operator sees after a power cycle, and a wrong mount is the
        // failure that has to be impossible to miss.
        let mount = Mount::from_u32(state::mount());
        let text = format(
            &mut telemetry_buffer,
            format_args!(
                "nvm mount={} {} stored={} zcapt={} lcapt={}",
                mount.as_u32(),
                mount.name(),
                u8::from(state::nvm_present()),
                u8::from(state::zero_captured()),
                u8::from(state::level_captured())
            ),
        );
        let len = text.len();
        let _ = write_bounded(&mut class, &telemetry_buffer[..len], REPLY_WRITE_TIMEOUT).await;

        // A DTR-less host is legitimate after a re-enumeration, so only a drop
        // from an asserted state marks a real close. Ending the session here
        // reinitialises the line parser and emits the resync newline.
        let dtr_at_start = class.dtr();

        // A new connection always starts on a clean line boundary.
        let mut line_len = 0usize;
        let mut line_too_long = false;

        loop {
            let hz = state::stream_hz();
            let period = if hz == 0 {
                IDLE_POLL
            } else {
                Duration::from_millis(1_000 / u64::from(hz))
            };

            let event = select(Timer::after(period), class.read_packet(&mut packet)).await;
            match event {
                Either::First(()) => {
                    if dtr_at_start && !class.dtr() {
                        break;
                    }
                    if state::debug_bridge_enabled() {
                        let mut count = 0;
                        while count < telemetry_buffer.len() {
                            match crate::debug_bridge::poll_rx() {
                                Some(byte) => {
                                    telemetry_buffer[count] = byte;
                                    count += 1;
                                }
                                None => break,
                            }
                        }
                        if count > 0 {
                            let _ = write_bounded(
                                &mut class,
                                &telemetry_buffer[..count],
                                TELEMETRY_WRITE_TIMEOUT,
                            )
                            .await;
                        }
                    }
                    if hz != 0 {
                        if let Some(telemetry) = state::TELEMETRY.try_take() {
                            let text = format_telemetry(&mut telemetry_buffer, &telemetry);
                            // A host that closed mid-stream leaves this write
                            // waiting for a drain that never comes. The bound
                            // drops the line; the task stays alive for the next
                            // host, with no USB re-enumeration required.
                            let _ = write_bounded(&mut class, text, TELEMETRY_WRITE_TIMEOUT).await;
                        }
                    }
                }
                Either::Second(Ok(0)) => break,
                Either::Second(Ok(len)) => {
                    for &byte in &packet[..len] {
                        if byte == b'\n' || byte == b'\r' {
                            if line_too_long {
                                reply(&mut class, format_args!("err line too long")).await;
                            } else if line_len > 0 {
                                match handle_command(&mut class, &mut flash, &line[..line_len])
                                    .await
                                {
                                    Action::Continue => {}
                                    Action::Reboot => {
                                        Timer::after(REBOOT_FLUSH).await;
                                        cortex_m::peripheral::SCB::sys_reset();
                                    }
                                }
                            }
                            line_len = 0;
                            line_too_long = false;
                        } else if line_len < MAX_LINE {
                            line[line_len] = byte;
                            line_len += 1;
                        } else {
                            line_too_long = true;
                        }
                    }
                }
                Either::Second(Err(_)) => break,
            }
        }
        // A closed host can leave the endpoint enabled, so wait_connection
        // would return immediately. A short pause keeps a dead session from
        // becoming a tight loop while the next host is opening the port.
        Timer::after(IDLE_POLL).await;
    }
}
