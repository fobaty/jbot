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
//!   `save`, `forget`.
//! * Pico → host: `T key=value ...` telemetry, `ok ...` accepted, `err ...`
//!   rejected, plus a banner on connect.
//!
//! Nothing here is on the control path: a missing host only means lines are
//! dropped, and a slow host cannot stall the 200 Hz loop.

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
    write_bytes(class, &buffer[..len]).await.is_ok()
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
                    "ok help: ping | stat | stream <hz> | speed <mps> | steer <deg/s> | trim <deg> | zero | level | pose [back|face|home] [deg] | mount <name> | home [deg] | vmax [deg/s] | pscale [0..2] | slew [units/s] | arm | disarm | test <units> [units] | reinit | estop | clear | save | forget | stats reset | reboot"
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
            let tuning = state::tuning().scaled_by(pscale);
            reply(
                class,
                format_args!(
                    "stat up={:.1} loop={} stream={} trim={:.2} estop={} steps={} imu={} imu_err={} imu_code={} fb={} fb_drop={} rx={} fb_ok={} stop={} zero={:.2} zroll={:.2} level={:.2} mount={} nvm={} kpnm={:.1} kdnm={:.1} kpos={:.2} kv={:.2} ki={:.3} pdem={:.1} ptrm={:.1} armed={} zcapt={} lcapt={} home={:.2} fall={} vmax={:.2} pscale={:.2} slew={:.0}",
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
                    state::torque_slew_per_s(),
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
            "T step={} loop={} imu={} cal={} pitch={:.2} roll={:.2} hdg={:.2} gy={:.2} gx={:.2} gz={:.2} err=0x{:02x} mb={} L={} R={} bat={:.2} tmp={:.1} age={} estop={} vset={:.2} wset={:.2} mnt={} zof={:.2} lvl={:.2} zro={:.2} arm={} tsl={:.0} tsr={:.0} trq={:.0} ref={:.2} dem={:.2} trm={:.2} x={:.3} mode={} gtlt={:.2} opm={} grace={:.2}",
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
        let _ = write_bytes(&mut class, BANNER).await;

        // Say what the flash record contains. This is the first thing the
        // operator sees after a power cycle, and a wrong mount is the failure
        // that has to be impossible to miss.
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
        let _ = write_bytes(&mut class, &telemetry_buffer[..len]).await;

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
                    if hz != 0 {
                        if let Some(telemetry) = state::TELEMETRY.try_take() {
                            let text = format_telemetry(&mut telemetry_buffer, &telemetry);
                            if write_bytes(&mut class, text).await.is_err() {
                                break;
                            }
                        }
                    }
                }
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
    }
}
