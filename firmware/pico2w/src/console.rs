//! USB CDC-ACM console: telemetry out, line commands in.
//!
//! The bench machine has no debug probe, so `defmt` over RTT is invisible
//! during normal operation. This module exposes a serial port on the Pico's
//! USB-C connector; the host sees it as `/dev/cu.usbmodem*`.
//!
//! Protocol (v1), one ASCII line per message:
//!
//! * Host → Pico: `help`, `ping`, `stat`, `stream <hz>`, `trim <deg>`,
//!   `reinit`, `estop`, `clear`, `stats reset`, `reboot`.
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

use crate::state::{self, Telemetry};

/// USB vendor id used for development boards without an assigned id.
const VID: u16 = 0x1209;
/// USB product id; arbitrary but stable for this firmware.
const PID: u16 = 0x0001;
/// CDC-ACM bulk endpoint size.
const PACKET_SIZE: u16 = 64;
/// Longest accepted command line, including the terminating newline.
const MAX_LINE: usize = 64;
/// Reply buffer; longer replies are truncated rather than dropped.
const REPLY_BUFFER: usize = 256;
/// Banner sent every time the host configures the interface.
const BANNER: &[u8] = b"JBOT pico bridge 0.1 ready\n";
/// Poll interval while the telemetry stream is disabled.
const IDLE_POLL: Duration = Duration::from_millis(200);
/// Time allowed for the final USB packets to leave before a requested reboot.
const REBOOT_FLUSH: Duration = Duration::from_millis(50);

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

/// What the console task should do after a command.
enum Action {
    /// Keep serving.
    Continue,
    /// Reset the chip after flushing the reply.
    Reboot,
}

/// Execute one command line. Replies are written by this function.
async fn handle_command(class: &mut Console, line: &[u8]) -> Action {
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
                    "ok help: ping | stat | stream <hz> | trim <deg> | reinit | estop | clear | stats reset | reboot"
                ),
            )
            .await;
        }
        "ping" => {
            reply(class, format_args!("ok ping")).await;
        }
        "stat" => {
            let uptime_s = Instant::now().as_millis() as f32 / 1_000.0;
            reply(
                class,
                format_args!(
                    "stat up={:.1} loop={} stream={} trim={:.2} estop={} steps={} imu={} imu_err={} imu_code={} fb={} fb_drop={}",
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
            reply(class, format_args!("ok clear")).await;
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
            "T step={} loop={} imu={} cal={} pitch={:.2} gy={:.2} err=0x{:02x} mb={} L={} R={} bat={:.2} tmp={:.1} age={} estop={}",
            telemetry.control_steps,
            telemetry.loop_hz,
            if telemetry.imu_fresh { "ok" } else { "stale" },
            telemetry.imu_system_cal,
            telemetry.pitch_deg,
            telemetry.gyro_y_dps,
            telemetry.sys_err,
            if telemetry.feedback_present { "ok" } else { "none" },
            telemetry.speed_left_rpm,
            telemetry.speed_right_rpm,
            telemetry.battery_v,
            telemetry.board_temp_c,
            telemetry.feedback_age_ms,
            u8::from(telemetry.estop),
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
pub async fn console_task(mut class: Console) {
    let mut packet = [0u8; PACKET_SIZE as usize];
    let mut line = [0u8; MAX_LINE];
    let mut telemetry_buffer = [0u8; REPLY_BUFFER];

    loop {
        class.wait_connection().await;
        let _ = write_bytes(&mut class, BANNER).await;

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
                                match handle_command(&mut class, &line[..line_len]).await {
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
