//! Application state and layout for the JBOT Pico desktop console.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use eframe::egui::{
    self, Align, Color32, FontId, Frame, Layout, Margin, RichText, ScrollArea, Stroke, Vec2,
};

use crate::attitude;
use crate::axiscal::{self, Lean, Measurements, MOUNT_COUNT, MOUNT_NAMES};
use crate::link::{available_ports, looks_like_pico, LinkEvent, LinkHandle};
use crate::protocol::{Message, Stat, Telemetry, MAX_STREAM_HZ};
use crate::theme;

/// Everything the UI can send to the Pico. Kept in one place so the safety
/// behaviour is easy to audit.
const COMMANDS: &[(&str, &str)] = &[("Ping", "ping"), ("Stat", "stat"), ("Help", "help")];

/// Command-line startup options.
pub struct Startup {
    /// Start with synthetic telemetry and no serial port.
    pub demo: bool,
    /// Preferred serial port; otherwise the first Pico-like port is used.
    pub port: Option<String>,
    /// Telemetry rate requested on connect.
    pub stream_hz: u32,
}

/// Event log severity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Level {
    Info,
    Tx,
    Rx,
    Warn,
    Error,
}

impl Level {
    fn color(self) -> Color32 {
        match self {
            Self::Info => theme::MUTED,
            Self::Tx => theme::ACCENT,
            Self::Rx => theme::GREEN,
            Self::Warn => theme::AMBER,
            Self::Error => theme::RED,
        }
    }

    fn tag(self) -> &'static str {
        match self {
            Self::Info => "INFO",
            Self::Tx => " TX ",
            Self::Rx => " RX ",
            Self::Warn => "WARN",
            Self::Error => "ERR ",
        }
    }
}

struct LogEntry {
    at: Instant,
    level: Level,
    text: String,
}

/// State of the guided mount (axis) calibration wizard. The wizard sweeps
/// every firmware mount selector while the operator holds one known pose at
/// a time, then picks the selector whose attitude convention matches
/// reality. See `axiscal` for the scoring rules.
#[derive(Clone)]
enum AxisCal {
    /// No guided calibration in progress.
    Idle,
    /// Waiting for the operator to press "Capture upright".
    PromptBaseline,
    /// Waiting for the operator to press the button for lean `usize`.
    PromptLean(usize),
    /// Sweeping the mount selectors for one pose.
    Capturing {
        kind: CaptureKind,
        mount: usize,
        window_start: Instant,
        samples: Vec<(f32, f32)>,
    },
    /// Sweep finished with a trustworthy detection.
    Done {
        detection: axiscal::Detection,
        scores: [f32; MOUNT_COUNT],
    },
    /// Sweep finished without a trustworthy detection; original mount
    /// restored.
    Failed {
        reason: String,
        scores: [f32; MOUNT_COUNT],
    },
}

impl AxisCal {
    /// True while the wizard owns the pose sweep; other calibration controls
    /// are locked out in that state.
    fn running(&self) -> bool {
        matches!(
            self,
            Self::PromptBaseline | Self::PromptLean(_) | Self::Capturing { .. }
        )
    }
}

/// Which pose is being swept.
#[derive(Clone, Copy)]
enum CaptureKind {
    Baseline,
    Lean(usize),
}

/// Desktop console.
pub struct ConsoleApp {
    link: Option<LinkHandle>,
    connected: bool,
    port: String,
    ports: Vec<String>,

    demo: bool,
    demo_started: Instant,
    last_demo_at: Instant,
    demo_estop: bool,

    telemetry: Option<Telemetry>,
    telemetry_at: Option<Instant>,
    stat: Option<Stat>,
    last_step: Option<u32>,
    garbage_lines: u32,

    stream_hz: u32,
    speed_setpoint: f32,
    steer_setpoint_deg: f32,
    applied_speed: f32,
    applied_steer_deg: f32,
    trim_deg: f32,
    mount: u8,
    zero_offset_deg: Option<f32>,
    roll_zero_offset_deg: Option<f32>,
    level_deg: Option<f32>,
    armed: bool,
    confirm_reboot: bool,
    axis_cal: AxisCal,
    axis_cal_data: Measurements,
    axis_cal_saved_mount: u8,

    events: VecDeque<LogEntry>,
    started: Instant,
}

impl ConsoleApp {
    /// Build the application. The serial port is opened immediately when a
    /// Pico-like device is present unless `--demo` was requested.
    pub fn new(cc: &eframe::CreationContext<'_>, startup: Startup) -> Self {
        configure_style(&cc.egui_ctx);
        let started = Instant::now();
        let ports = available_ports();
        let port = startup
            .port
            .filter(|name| !name.trim().is_empty())
            .or_else(|| ports.iter().find(|name| looks_like_pico(name)).cloned())
            .or_else(|| ports.first().cloned())
            .unwrap_or_default();

        let mut app = Self {
            link: None,
            connected: false,
            port,
            ports,
            demo: startup.demo,
            demo_started: started,
            last_demo_at: started,
            demo_estop: false,
            telemetry: None,
            telemetry_at: None,
            stat: None,
            last_step: None,
            garbage_lines: 0,
            stream_hz: startup.stream_hz.clamp(1, MAX_STREAM_HZ),
            speed_setpoint: 0.0,
            steer_setpoint_deg: 0.0,
            applied_speed: 0.0,
            applied_steer_deg: 0.0,
            trim_deg: 0.0,
            mount: 0,
            zero_offset_deg: None,
            roll_zero_offset_deg: None,
            level_deg: None,
            armed: false,
            confirm_reboot: false,
            axis_cal: AxisCal::Idle,
            axis_cal_data: Measurements::default(),
            axis_cal_saved_mount: 0,
            events: VecDeque::new(),
            started,
        };

        if app.demo {
            app.log(
                Level::Info,
                "demo mode: synthetic BNO055 attitude, no serial port opened",
            );
        } else {
            app.log(Level::Info, "ready: select a serial port and press Connect");
            if !app.port.is_empty() {
                app.connect();
            }
        }
        app
    }

    fn telemetry_age(&self) -> Option<Duration> {
        self.telemetry_at.map(|at| at.elapsed())
    }

    fn telemetry_stale(&self) -> bool {
        self.telemetry_age()
            .is_none_or(|age| age > Duration::from_millis(1_000))
    }

    fn estop_latched(&self) -> bool {
        self.demo_estop
            || self
                .telemetry
                .as_ref()
                .is_some_and(|telemetry| telemetry.estop)
    }

    fn poll_link(&mut self) {
        let events = self
            .link
            .as_mut()
            .map(LinkHandle::drain)
            .unwrap_or_default();
        for event in events {
            self.handle_event(event);
        }
    }

    fn handle_event(&mut self, event: LinkEvent) {
        match event {
            LinkEvent::Connected { port } => {
                self.connected = true;
                self.speed_setpoint = 0.0;
                self.steer_setpoint_deg = 0.0;
                self.applied_speed = 0.0;
                self.applied_steer_deg = 0.0;
                self.log(Level::Info, format!("connected: {port}"));
            }
            LinkEvent::Disconnected { reason } => {
                self.connected = false;
                self.log(Level::Warn, format!("disconnected: {reason}"));
            }
            LinkEvent::Message(message) => self.handle_message(message),
            LinkEvent::Notice(text) => self.log(Level::Warn, text),
        }
    }

    fn handle_message(&mut self, message: Message) {
        match message {
            Message::Telemetry(telemetry) => {
                if let Some(previous) = self.last_step {
                    if telemetry.step < previous && previous.wrapping_sub(telemetry.step) > 1_000 {
                        self.log(
                            Level::Warn,
                            format!(
                                "firmware restart detected (step {previous} -> {})",
                                telemetry.step
                            ),
                        );
                    }
                }
                self.last_step = Some(telemetry.step);
                if telemetry.has_setpoints {
                    self.applied_speed = telemetry.speed_setpoint_mps;
                    self.applied_steer_deg = telemetry.steer_setpoint_deg_s();
                }
                if let Some(mount) = telemetry.mount {
                    self.mount = mount;
                }
                if let Some(offset) = telemetry.zero_offset_deg {
                    self.zero_offset_deg = Some(offset);
                }
                if let Some(offset) = telemetry.roll_zero_offset_deg {
                    self.roll_zero_offset_deg = Some(offset);
                }
                if let Some(level) = telemetry.level_deg {
                    self.level_deg = Some(level);
                }
                if let Some(armed) = telemetry.armed {
                    self.armed = armed;
                }
                self.axis_cal_sample(telemetry.pitch_deg, telemetry.roll_deg);
                self.telemetry = Some(telemetry);
                self.telemetry_at = Some(Instant::now());
            }
            Message::Stat(stat) => {
                self.log(
                    Level::Rx,
                    format!(
                        "stat up={:.1}s loop={}Hz stream={}Hz imu_err={} fb={} drop={} estop={}",
                        stat.uptime_s,
                        stat.loop_hz,
                        stat.stream_hz,
                        stat.imu_errors,
                        stat.fb_frames,
                        stat.fb_discarded,
                        u8::from(stat.estop),
                    ),
                );
                if let Some(zero) = stat.zero_deg {
                    self.zero_offset_deg = Some(zero);
                }
                if let Some(zroll) = stat.zroll_deg {
                    self.roll_zero_offset_deg = Some(zroll);
                }
                if let Some(level) = stat.level_deg {
                    self.level_deg = Some(level);
                    self.trim_deg = level;
                }
                if let Some(mount) = stat.mount {
                    self.mount = mount;
                }
                if let Some(armed) = stat.armed {
                    self.armed = armed;
                }
                self.stat = Some(stat);
            }
            Message::Ok(text) => {
                self.apply_ok_reply(&text);
                self.log(Level::Rx, text);
            }
            Message::Err(text) => self.log(Level::Error, text),
            Message::Banner(text) => self.log(Level::Info, format!("firmware: {text}")),
            Message::Garbage(text) => {
                self.garbage_lines = self.garbage_lines.saturating_add(1);
                if self.garbage_lines <= 5 || self.garbage_lines.is_multiple_of(50) {
                    self.log(
                        Level::Warn,
                        format!("unrecognised line #{}: {text}", self.garbage_lines),
                    );
                }
            }
        }
    }

    fn apply_ok_reply(&mut self, text: &str) {
        if let Some(rest) = text.strip_prefix("ok speed ") {
            if let Ok(value) = rest.trim().parse::<f32>() {
                self.applied_speed = value;
                self.speed_setpoint = value;
            }
        } else if let Some(rest) = text.strip_prefix("ok steer ") {
            if let Ok(value) = rest.trim().parse::<f32>() {
                self.applied_steer_deg = value;
                self.steer_setpoint_deg = value;
            }
        } else if let Some(rest) = text.strip_prefix("ok trim ") {
            if let Ok(value) = rest.trim().parse::<f32>() {
                self.trim_deg = value;
                self.level_deg = Some(value);
            }
        } else if let Some(rest) = text.strip_prefix("ok zero ") {
            for token in rest.split_whitespace() {
                if let Some(value) = token.strip_prefix("pitch=") {
                    if let Ok(parsed) = value.parse::<f32>() {
                        self.zero_offset_deg = Some(parsed);
                    }
                } else if let Some(value) = token.strip_prefix("roll=") {
                    if let Ok(parsed) = value.parse::<f32>() {
                        self.roll_zero_offset_deg = Some(parsed);
                    }
                }
            }
        } else if let Some(rest) = text.strip_prefix("ok level ") {
            if let Ok(value) = rest.trim().parse::<f32>() {
                self.level_deg = Some(value);
                self.trim_deg = value;
            }
        } else if text == "ok armed" {
            self.armed = true;
        } else if text == "ok disarmed" {
            self.armed = false;
        } else if let Some(rest) = text.strip_prefix("ok mount ") {
            if let Some(value) = rest.split_whitespace().next() {
                if let Ok(parsed) = value.parse::<u8>() {
                    self.mount = parsed;
                }
            }
        } else if let Some(rest) = text.strip_prefix("ok stream ") {
            if let Ok(value) = rest.trim().parse::<u32>() {
                self.stream_hz = value;
            }
        }
    }

    fn update_demo(&mut self) {
        if !self.demo {
            return;
        }
        let now = Instant::now();
        if now.duration_since(self.last_demo_at) < Duration::from_millis(20) {
            return;
        }
        self.last_demo_at = now;
        let t = now.duration_since(self.demo_started).as_secs_f32();
        let pitch = 12.0 * (t * 0.55).sin() + 2.0 * (t * 1.7).sin();
        let roll = 22.0 * (t * 0.37 + 0.8).sin();
        let heading = (t * 14.0).rem_euclid(360.0);
        let gyro_y = 12.0 * 0.55 * (t * 0.55).cos();
        let gyro_x = 22.0 * 0.37 * (t * 0.37 + 0.8).cos();
        let gyro_z = 8.0 * (t * 0.9).sin();
        let left_rpm = (85.0 * (t * 0.4).sin()) as i16;
        let right_rpm = (85.0 * (t * 0.4 + 0.4).sin()) as i16;
        let battery_v = 39.4 + 0.35 * (t * 0.07).sin();
        let board_temp_c = 34.0 + 1.5 * (t * 0.05).sin();

        self.telemetry = Some(Telemetry {
            step: (t * 200.0) as u32,
            loop_hz: 200,
            imu_fresh: true,
            cal: 3,
            pitch_deg: pitch,
            roll_deg: roll,
            heading_deg: heading,
            gyro_y_dps: gyro_y,
            gyro_x_dps: gyro_x,
            gyro_z_dps: gyro_z,
            sys_err: 0,
            feedback_present: true,
            speed_left_rpm: left_rpm,
            speed_right_rpm: right_rpm,
            battery_v,
            board_temp_c,
            feedback_age_ms: 12,
            estop: self.demo_estop,
            speed_setpoint_mps: self.applied_speed,
            steer_setpoint_rps: self.applied_steer_deg.to_radians(),
            has_orientation: true,
            has_setpoints: true,
            mount: Some(self.mount),
            zero_offset_deg: Some(self.zero_offset_deg.unwrap_or(1.8)),
            roll_zero_offset_deg: Some(self.roll_zero_offset_deg.unwrap_or(-0.6)),
            level_deg: Some(self.level_deg.unwrap_or(self.trim_deg)),
            armed: Some(false),
        });
        self.telemetry_at = Some(now);
    }

    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let keys = ctx.input(|input| {
            (
                input.key_pressed(egui::Key::ArrowUp),
                input.key_pressed(egui::Key::ArrowDown),
                input.key_pressed(egui::Key::ArrowLeft),
                input.key_pressed(egui::Key::ArrowRight),
                input.key_pressed(egui::Key::Space),
                input.key_pressed(egui::Key::E),
                input.key_pressed(egui::Key::C),
                input.key_pressed(egui::Key::Z),
                input.key_pressed(egui::Key::L),
            )
        });

        if keys.0 {
            self.nudge_speed(0.1);
        }
        if keys.1 {
            self.nudge_speed(-0.1);
        }
        if keys.2 {
            self.nudge_steer(-5.0);
        }
        if keys.3 {
            self.nudge_steer(5.0);
        }
        if keys.4 {
            self.zero();
        }
        if keys.5 {
            self.emergency_stop();
        }
        if keys.6 {
            self.clear_estop();
        }
        if keys.7 {
            self.calibrate_zero();
        }
        if keys.8 {
            self.calibrate_level();
        }
    }

    fn calibrate_zero(&mut self) {
        self.send("zero");
        self.log(
            Level::Info,
            "zero calibration: hold the robot at the vertical reference, capturing current pitch/roll as zero",
        );
    }

    fn calibrate_level(&mut self) {
        self.send("level");
        self.log(
            Level::Info,
            "level calibration: hold the robot at the balance stance, capturing it as the balance reference",
        );
    }

    /// True while the guided mount wizard owns the sweep.
    fn axis_cal_running(&self) -> bool {
        self.axis_cal.running()
    }

    fn axis_cal_sample(&mut self, pitch_deg: f32, roll_deg: f32) {
        if let AxisCal::Capturing { samples, .. } = &mut self.axis_cal {
            samples.push((pitch_deg, roll_deg));
        }
    }

    fn axis_cal_start(&mut self) {
        if !self.demo && !self.connected {
            self.log(
                Level::Error,
                "guided mount calibration needs a live link to the firmware (or demo mode)",
            );
            return;
        }
        self.axis_cal_saved_mount = self.mount;
        self.axis_cal_data = Measurements::default();
        self.send("stream 5");
        self.axis_cal = AxisCal::PromptBaseline;
        self.log(
            Level::Info,
            "guided mount calibration: stand the robot UPRIGHT and still, then press Capture",
        );
    }

    fn axis_cal_cancel(&mut self) {
        let saved = self.axis_cal_saved_mount;
        self.send(format!("mount {saved}"));
        self.send(format!("stream {}", self.stream_hz));
        self.axis_cal = AxisCal::Idle;
        self.log(
            Level::Warn,
            "guided mount calibration cancelled; mount restored",
        );
    }

    fn axis_cal_begin_sweep(&mut self, kind: CaptureKind) {
        self.send("mount 0");
        self.axis_cal = AxisCal::Capturing {
            kind,
            mount: 0,
            window_start: Instant::now(),
            samples: Vec::with_capacity(16),
        };
    }

    /// Advance the wizard's windowed capture; called once per frame.
    fn update_axis_cal(&mut self) {
        const WINDOW: Duration = Duration::from_millis(800);
        let (kind, mount, samples) = match &mut self.axis_cal {
            AxisCal::Capturing {
                kind,
                mount,
                window_start,
                samples,
            } => {
                if window_start.elapsed() < WINDOW {
                    return;
                }
                (*kind, *mount, std::mem::take(samples))
            }
            _ => return,
        };
        let Some(mean) = mean_pair(&samples) else {
            self.axis_cal_restore_after_failure("no telemetry from the firmware during the sweep");
            return;
        };
        match kind {
            CaptureKind::Baseline => self.axis_cal_data.upright[mount] = Some(mean),
            CaptureKind::Lean(index) => self.axis_cal_data.leans[index][mount] = Some(mean),
        }
        if mount + 1 < MOUNT_COUNT {
            let next = mount + 1;
            self.send(format!("mount {next}"));
            self.axis_cal = AxisCal::Capturing {
                kind,
                mount: next,
                window_start: Instant::now(),
                samples: Vec::with_capacity(16),
            };
            return;
        }
        match kind {
            CaptureKind::Baseline => {
                self.axis_cal = AxisCal::PromptLean(0);
                self.log(
                    Level::Info,
                    "upright reference captured; now hold the FORWARD lean and press its button",
                );
            }
            CaptureKind::Lean(index) => {
                if index + 1 < Lean::ALL.len() {
                    self.axis_cal = AxisCal::PromptLean(index + 1);
                } else {
                    self.axis_cal_finish();
                }
            }
        }
    }

    fn axis_cal_restore_after_failure(&mut self, reason: &str) {
        let saved = self.axis_cal_saved_mount;
        self.send(format!("mount {saved}"));
        self.send(format!("stream {}", self.stream_hz));
        self.axis_cal = AxisCal::Failed {
            reason: reason.to_owned(),
            scores: axiscal::scores(&self.axis_cal_data),
        };
        self.log(
            Level::Error,
            "guided mount calibration aborted; mount restored",
        );
    }

    fn axis_cal_finish(&mut self) {
        let scores = axiscal::scores(&self.axis_cal_data);
        let detection = axiscal::detect(&scores);
        self.send(format!("stream {}", self.stream_hz));
        if detection.confident {
            self.send(format!("mount {}", detection.best));
            self.axis_cal = AxisCal::Done { detection, scores };
            self.log(
                Level::Info,
                format!(
                    "guided mount calibration: detected {} (selector {})",
                    MOUNT_NAMES[detection.best], detection.best
                ),
            );
        } else {
            let saved = self.axis_cal_saved_mount;
            self.send(format!("mount {saved}"));
            self.axis_cal = AxisCal::Failed {
                reason: "no selector matched confidently; hold a larger lean (20–30°) and retry"
                    .to_owned(),
                scores,
            };
            self.log(
                Level::Warn,
                "guided mount calibration: no confident match; original mount restored",
            );
        }
    }

    fn nudge_speed(&mut self, delta: f32) {
        if self.estop_latched() {
            return;
        }
        self.speed_setpoint = (self.speed_setpoint + delta).clamp(-2.0, 2.0);
        self.applied_speed = self.speed_setpoint;
        self.send(format!("speed {:.2}", self.applied_speed));
    }

    fn nudge_steer(&mut self, delta: f32) {
        if self.estop_latched() {
            return;
        }
        self.steer_setpoint_deg = (self.steer_setpoint_deg + delta).clamp(-180.0, 180.0);
        self.applied_steer_deg = self.steer_setpoint_deg;
        self.send(format!("steer {:.1}", self.applied_steer_deg));
    }

    fn apply_drive(&mut self) {
        if self.estop_latched() {
            self.log(Level::Warn, "e-stop latched: clear it before driving");
            return;
        }
        self.speed_setpoint = self.speed_setpoint.clamp(-5.0, 5.0);
        self.steer_setpoint_deg = self.steer_setpoint_deg.clamp(-180.0, 180.0);
        self.applied_speed = self.speed_setpoint;
        self.applied_steer_deg = self.steer_setpoint_deg;
        self.send(format!("speed {:.2}", self.applied_speed));
        self.send(format!("steer {:.1}", self.applied_steer_deg));
    }

    fn zero(&mut self) {
        self.speed_setpoint = 0.0;
        self.steer_setpoint_deg = 0.0;
        self.applied_speed = 0.0;
        self.applied_steer_deg = 0.0;
        self.send("speed 0");
        self.send("steer 0");
        self.log(Level::Info, "soft stop: setpoints zeroed");
    }

    fn emergency_stop(&mut self) {
        self.demo_estop = true;
        self.speed_setpoint = 0.0;
        self.steer_setpoint_deg = 0.0;
        self.applied_speed = 0.0;
        self.applied_steer_deg = 0.0;
        self.send("estop");
        self.send("speed 0");
        self.send("steer 0");
        self.log(Level::Error, "EMERGENCY STOP requested");
    }

    fn clear_estop(&mut self) {
        self.demo_estop = false;
        self.send("clear");
        self.log(Level::Info, "e-stop clear requested");
    }

    fn send(&mut self, line: impl Into<String>) {
        let line = line.into();
        if self.demo {
            self.log(Level::Tx, format!("demo -> {line}"));
            return;
        }
        let sent = self.link.as_ref().map(|link| link.send(line.clone()));
        match sent {
            Some(true) => self.log(Level::Tx, line),
            Some(false) => self.log(Level::Warn, format!("command dropped: {line}")),
            None => self.log(Level::Warn, format!("link down: {line}")),
        }
    }

    fn connect(&mut self) {
        if self.demo {
            self.set_demo(false);
        }
        let port = self.port.trim().to_owned();
        if port.is_empty() {
            self.log(Level::Error, "no serial port selected");
            return;
        }
        self.disconnect(false);
        match LinkHandle::connect(port.clone(), self.stream_hz) {
            Ok(link) => {
                self.link = Some(link);
                self.connected = false;
                self.log(Level::Info, format!("opening {port}"));
            }
            Err(error) => self.log(
                Level::Error,
                format!("cannot start the serial thread: {error}"),
            ),
        }
    }

    fn disconnect(&mut self, log: bool) {
        if let Some(mut link) = self.link.take() {
            link.shutdown();
            self.connected = false;
            if log {
                self.log(
                    Level::Info,
                    "link closed: firmware e-stop latched and setpoints zeroed",
                );
            }
        }
    }

    fn set_demo(&mut self, demo: bool) {
        if self.demo == demo {
            return;
        }
        self.demo = demo;
        if demo {
            self.disconnect(false);
            self.demo_started = Instant::now();
            self.last_demo_at = self.demo_started;
            self.demo_estop = false;
            self.speed_setpoint = 0.0;
            self.steer_setpoint_deg = 0.0;
            self.applied_speed = 0.0;
            self.applied_steer_deg = 0.0;
            self.log(Level::Info, "demo mode enabled");
        } else {
            self.telemetry = None;
            self.telemetry_at = None;
            self.log(Level::Info, "demo mode disabled");
        }
    }

    fn refresh_ports(&mut self) {
        self.ports = available_ports();
        if self.port.trim().is_empty() || !self.ports.iter().any(|name| name == &self.port) {
            if let Some(first) = self
                .ports
                .iter()
                .find(|name| looks_like_pico(name))
                .or_else(|| self.ports.first())
            {
                self.port = first.clone();
            }
        }
        self.log(
            Level::Info,
            format!("{} serial port(s) available", self.ports.len()),
        );
    }

    fn log(&mut self, level: Level, text: impl Into<String>) {
        self.events.push_back(LogEntry {
            at: Instant::now(),
            level,
            text: text.into(),
        });
        while self.events.len() > 500 {
            self.events.pop_front();
        }
    }

    fn top_bar(&mut self, root: &mut egui::Ui) {
        egui::Panel::top("top_bar")
            .frame(
                Frame::new()
                    .fill(theme::PANEL)
                    .inner_margin(Margin::symmetric(14, 10)),
            )
            .show(root, |ui| {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("JBOT")
                            .size(22.0)
                            .strong()
                            .color(theme::ACCENT),
                    );
                    ui.label(RichText::new("PICO CONSOLE").size(13.0).color(theme::MUTED));
                    ui.add_space(8.0);
                    ui.separator();

                    let mut selected = self.port.clone();
                    egui::ComboBox::from_id_salt("serial_port")
                        .width(260.0)
                        .selected_text(if selected.is_empty() {
                            "select serial port".to_owned()
                        } else {
                            selected.clone()
                        })
                        .show_ui(ui, |ui| {
                            for name in &self.ports {
                                ui.selectable_value(&mut selected, name.clone(), name.as_str());
                            }
                            if self.ports.is_empty() {
                                ui.label("no serial ports found");
                            }
                        });
                    if selected != self.port {
                        self.port = selected;
                    }

                    if ui.button("Refresh").clicked() {
                        self.refresh_ports();
                    }

                    if self.link.is_some() {
                        if ui.button("Disconnect").clicked() {
                            self.disconnect(true);
                        }
                    } else {
                        let can_connect = !self.demo && !self.port.trim().is_empty();
                        if ui
                            .add_enabled(can_connect, egui::Button::new("Connect"))
                            .clicked()
                        {
                            self.connect();
                        }
                    }

                    let mut demo = self.demo;
                    if ui.checkbox(&mut demo, "Demo").changed() {
                        self.set_demo(demo);
                    }

                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        let (color, text) = if self.demo {
                            (theme::ACCENT, "DEMO DATA")
                        } else if self.connected {
                            (theme::GREEN, "LINKED")
                        } else if self.link.is_some() {
                            (theme::AMBER, "CONNECTING")
                        } else {
                            (theme::MUTED, "OFFLINE")
                        };
                        ui.label(RichText::new(format!("● {text}")).color(color).strong());
                    });
                });
            });
    }

    fn controls_panel(&mut self, root: &mut egui::Ui) {
        egui::Panel::left("controls")
            .resizable(true)
            .default_size(335.0)
            .size_range(egui::Rangef::new(285.0, 440.0))
            .frame(
                Frame::new()
                    .fill(theme::PANEL)
                    .inner_margin(Margin::same(12)),
            )
            .show(root, |ui| {
                ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        self.link_controls(ui);
                        self.drive_controls(ui);
                        self.calibration_controls(ui);
                        self.system_controls(ui);
                    });
            });
    }

    fn link_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "LINK");
        ui.horizontal(|ui| {
            ui.label("Telemetry");
            ui.add(egui::Slider::new(&mut self.stream_hz, 0..=50).suffix(" Hz"));
        });
        if ui.button("Apply stream rate").clicked() {
            self.send(format!("stream {}", self.stream_hz));
        }
        ui.label(
            RichText::new(format!("firmware accepts 0..={MAX_STREAM_HZ} Hz"))
                .size(10.0)
                .color(theme::MUTED),
        );
        ui.horizontal(|ui| {
            for (label, command) in COMMANDS {
                if ui.button(*label).clicked() {
                    self.send(*command);
                }
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Reset stats").clicked() {
                self.send("stats reset");
            }
            if let Some(link) = &self.link {
                ui.label(
                    RichText::new(format!("dropped UI events: {}", link.dropped_events()))
                        .size(10.0)
                        .color(theme::MUTED),
                );
            }
        });
    }

    fn drive_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "DRIVE");
        let latched = self.estop_latched();
        ui.add_enabled_ui(!latched, |ui| {
            ui.add(
                egui::Slider::new(&mut self.speed_setpoint, -2.0..=2.0)
                    .text("Speed")
                    .suffix(" m/s")
                    .step_by(0.05),
            );
            ui.add(
                egui::Slider::new(&mut self.steer_setpoint_deg, -180.0..=180.0)
                    .text("Steer")
                    .suffix(" °/s")
                    .step_by(5.0),
            );
            ui.horizontal(|ui| {
                if ui.button("Apply").clicked() {
                    self.apply_drive();
                }
                if ui.button("Zero").clicked() {
                    self.zero();
                }
                if ui.button("0.5 m/s").clicked() {
                    self.speed_setpoint = 0.5;
                    self.steer_setpoint_deg = 0.0;
                    self.apply_drive();
                }
                if ui.button("1.0 m/s").clicked() {
                    self.speed_setpoint = 1.0;
                    self.steer_setpoint_deg = 0.0;
                    self.apply_drive();
                }
            });
        });
        ui.label(
            RichText::new(format!(
                "applied: {:+.2} m/s / {:+.1} °/s",
                self.applied_speed, self.applied_steer_deg
            ))
            .monospace()
            .size(11.0)
            .color(theme::MUTED),
        );
        if latched {
            ui.label(
                RichText::new("E-stop latched: drive controls locked")
                    .color(theme::RED)
                    .strong(),
            );
        }
        let estop_button = egui::Button::new(
            RichText::new("EMERGENCY STOP")
                .size(16.0)
                .strong()
                .color(Color32::WHITE),
        )
        .fill(theme::RED)
        .min_size(egui::vec2(ui.available_width(), 42.0));
        if ui.add(estop_button).clicked() {
            self.emergency_stop();
        }
        if ui.button("Clear e-stop").clicked() {
            self.clear_estop();
        }
        ui.label(
            RichText::new("E-stop is latched in firmware and survives a USB reconnect.")
                .size(10.0)
                .color(theme::MUTED),
        );
    }

    fn calibration_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "CALIBRATION");
        let busy = self.axis_cal_running();
        ui.add_enabled_ui(!busy, |ui| {
            ui.horizontal(|ui| {
                ui.label("Mount");
                egui::ComboBox::from_id_salt("mount")
                    .width(150.0)
                    .selected_text(mount_name(self.mount))
                    .show_ui(ui, |ui| {
                        for (selector, name) in
                            MOUNT_NAMES.iter().enumerate().map(|(i, n)| (i as u8, *n))
                        {
                            if ui
                                .selectable_value(&mut self.mount, selector, name)
                                .clicked()
                            {
                                self.send(format!("mount {selector}"));
                            }
                        }
                    });
            });
        });
        if busy {
            ui.label(
                RichText::new("Mount locked while the guided calibration sweeps the selectors.")
                    .size(10.0)
                    .color(theme::AMBER),
            );
        }
        self.axis_cal_ui(ui);
        ui.label(
            RichText::new("Re-capture zero and level after changing the mount.")
                .size(10.0)
                .color(theme::MUTED),
        );
        ui.add_enabled_ui(!busy, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Calibrate ZERO").clicked() {
                    self.calibrate_zero();
                }
                if ui.button("Calibrate LEVEL").clicked() {
                    self.calibrate_level();
                }
            });
        });
        ui.add_enabled_ui(!busy, |ui| {
            ui.horizontal(|ui| {
                let arm_button = egui::Button::new(
                    RichText::new(if self.armed { "ARMED" } else { "ARM BALANCE" })
                        .strong()
                        .color(Color32::WHITE),
                )
                .fill(if self.armed {
                    theme::GREEN
                } else {
                    theme::AMBER
                })
                .min_size(egui::vec2(ui.available_width() * 0.62, 34.0));
                if ui.add(arm_button).clicked() {
                    self.send("arm");
                }
                if ui.button("DISARM").clicked() {
                    self.send("disarm");
                }
            });
        });
        ui.label(
            RichText::new("ARM is refused until zero and level are captured this session.")
                .size(10.0)
                .color(theme::MUTED),
        );
        let zero_text = self
            .zero_offset_deg
            .map_or_else(|| "--".to_owned(), |value| format!("{value:+.2}°"));
        let roll_text = self
            .roll_zero_offset_deg
            .map_or_else(|| "--".to_owned(), |value| format!("{value:+.2}°"));
        let level_text = self
            .level_deg
            .map_or_else(|| "--".to_owned(), |value| format!("{value:+.2}°"));
        ui.label(
            RichText::new(format!(
                "zero {zero_text} · roll {roll_text} · level {level_text}"
            ))
            .monospace()
            .size(11.0)
            .color(theme::TEXT),
        );
        ui.label(
            RichText::new(
                "Zero: hold the robot at the vertical reference. Level: hold it at the balance stance.",
            )
            .size(10.0)
            .color(theme::MUTED),
        );
        ui.add(
            egui::Slider::new(&mut self.trim_deg, -10.0..=10.0)
                .text("Level trim")
                .suffix(" °")
                .step_by(0.1),
        );
        ui.horizontal(|ui| {
            if ui.button("Apply trim").clicked() {
                self.send(format!("trim {:.2}", self.trim_deg));
            }
            if ui.button("Reinit BNO055").clicked() {
                self.send("reinit");
            }
        });
        if let Some(telemetry) = &self.telemetry {
            ui.label(
                RichText::new(format!(
                    "CAL {}/3 · SYS_ERR 0x{:02x} · gyro Y {:+.1} °/s",
                    telemetry.cal, telemetry.sys_err, telemetry.gyro_y_dps
                ))
                .monospace()
                .size(10.0)
                .color(theme::MUTED),
            );
        }
        ui.label(
            RichText::new("IMUPLUS mode; the magnetometer stays disabled.")
                .size(10.0)
                .color(theme::MUTED),
        );
    }

    fn system_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "SYSTEM");
        if ui.button("Reboot Pico").clicked() {
            self.confirm_reboot = true;
        }
        if self.confirm_reboot {
            ui.horizontal(|ui| {
                ui.label(RichText::new("Reboot now?").color(theme::AMBER));
                if ui.button("Yes").clicked() {
                    self.confirm_reboot = false;
                    self.send("reboot");
                }
                if ui.button("Cancel").clicked() {
                    self.confirm_reboot = false;
                }
            });
        }
        ui.label(
            RichText::new(
                "On quit the console latches e-stop, zeros speed and steering, then closes the port.",
            )
            .size(10.0)
            .color(theme::MUTED),
        );
        ui.add_space(8.0);
        ui.label(
            RichText::new(
                "Keys: ↑/↓ speed · ←/→ steer · Space zero · E e-stop · C clear · Z zero · L level",
            )
            .size(10.0)
            .color(theme::MUTED),
        );
    }

    fn central_panel(&mut self, root: &mut egui::Ui) {
        egui::CentralPanel::default()
            .frame(Frame::new().fill(theme::BG).inner_margin(Margin::same(12)))
            .show(root, |ui| {
                ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        let telemetry = self.telemetry.clone();
                        let stale = self.telemetry_stale();
                        let width = ui.available_width();
                        let horizon_height = (ui.available_height() * 0.44).clamp(250.0, 390.0);

                        ui.horizontal(|ui| {
                            let compass_width = (width * 0.26).clamp(190.0, 250.0);
                            let horizon_width = (width - compass_width - 12.0).max(300.0);
                            ui.allocate_ui_with_layout(
                                Vec2::new(horizon_width, horizon_height),
                                Layout::top_down(Align::Min),
                                |ui| {
                                    attitude::horizon(
                                        ui,
                                        telemetry.as_ref(),
                                        horizon_height,
                                        stale,
                                    );
                                },
                            );
                            ui.allocate_ui_with_layout(
                                Vec2::new(compass_width, horizon_height),
                                Layout::top_down(Align::Center),
                                |ui| {
                                    attitude::compass(
                                        ui,
                                        telemetry.as_ref(),
                                        compass_width.min(horizon_height) * 0.88,
                                    );
                                    if let Some(telemetry) = &telemetry {
                                        ui.add_space(4.0);
                                        ui.label(
                                            RichText::new(format!(
                                                "HDG {:03.0}° · TILT {:.1}°",
                                                telemetry.heading_deg.rem_euclid(360.0),
                                                telemetry.max_tilt_deg()
                                            ))
                                            .monospace()
                                            .size(10.0)
                                            .color(theme::MUTED),
                                        );
                                        ui.label(
                                            RichText::new(format!(
                                                "GYRO X {:+.1} Y {:+.1} Z {:+.1} °/s",
                                                telemetry.gyro_x_dps,
                                                telemetry.gyro_y_dps,
                                                telemetry.gyro_z_dps
                                            ))
                                            .monospace()
                                            .size(10.0)
                                            .color(theme::MUTED),
                                        );
                                    }
                                },
                            );
                        });

                        ui.add_space(10.0);
                        self.instrument_row(ui, telemetry.as_ref());
                        ui.add_space(10.0);
                        self.status_chips(ui, telemetry.as_ref());
                        ui.add_space(10.0);
                        self.event_log(ui);
                    });
            });
    }

    fn instrument_row(&mut self, ui: &mut egui::Ui, telemetry: Option<&Telemetry>) {
        let gap = 10.0;
        let card_width = ((ui.available_width() - 3.0 * gap) / 4.0).max(165.0);
        let card_height = 118.0;

        let max_tilt = telemetry.map_or(0.0, Telemetry::max_tilt_deg);
        let attitude_color = if max_tilt > 25.0 {
            theme::RED
        } else if max_tilt > 15.0 {
            theme::AMBER
        } else {
            theme::ACCENT
        };

        let battery_v = telemetry.map_or(0.0, |value| value.battery_v);
        let battery_color = if battery_v < 31.0 {
            theme::RED
        } else if battery_v < 34.0 {
            theme::AMBER
        } else {
            theme::GREEN
        };

        let wheel_rpm = telemetry.map_or(0.0, Telemetry::wheel_speed_rpm);
        let loop_hz = telemetry.map_or(0, |value| value.loop_hz);
        let control_color = if (190..=210).contains(&loop_hz) {
            theme::GREEN
        } else if loop_hz > 0 {
            theme::AMBER
        } else {
            theme::MUTED
        };

        let attitude_text = telemetry
            .map(|_| format!("{max_tilt:.1}°"))
            .unwrap_or_else(|| "--".to_owned());
        let battery_text = telemetry
            .map(|_| format!("{battery_v:.2} V"))
            .unwrap_or_else(|| "--".to_owned());
        let wheel_text = telemetry
            .map(|_| format!("{wheel_rpm:.0} rpm"))
            .unwrap_or_else(|| "--".to_owned());
        let control_text = telemetry
            .map(|_| format!("{loop_hz} Hz"))
            .unwrap_or_else(|| "--".to_owned());
        let wheel_subtitle = telemetry.map_or_else(
            || "left / right".to_owned(),
            |value| {
                format!(
                    "L {} / R {} rpm",
                    value.speed_left_rpm, value.speed_right_rpm
                )
            },
        );
        let control_subtitle = telemetry
            .map(|value| format!("IMU CAL {}/3", value.cal))
            .unwrap_or_else(|| "control loop".to_owned());

        ui.horizontal(|ui| {
            ui.allocate_ui_with_layout(
                Vec2::new(card_width, card_height),
                Layout::top_down(Align::Min),
                |ui| {
                    attitude::gauge_card(
                        ui,
                        "ATTITUDE",
                        &attitude_text,
                        "max pitch / roll",
                        (max_tilt / 45.0).clamp(0.0, 1.0),
                        attitude_color,
                    );
                },
            );
            ui.allocate_ui_with_layout(
                Vec2::new(card_width, card_height),
                Layout::top_down(Align::Min),
                |ui| {
                    attitude::gauge_card(
                        ui,
                        "BATTERY",
                        &battery_text,
                        "pack 30-42 V",
                        telemetry.map_or(0.0, Telemetry::battery_fraction),
                        battery_color,
                    );
                },
            );
            ui.allocate_ui_with_layout(
                Vec2::new(card_width, card_height),
                Layout::top_down(Align::Min),
                |ui| {
                    attitude::gauge_card(
                        ui,
                        "WHEELS",
                        &wheel_text,
                        &wheel_subtitle,
                        (wheel_rpm / 400.0).clamp(0.0, 1.0),
                        theme::ACCENT,
                    );
                },
            );
            ui.allocate_ui_with_layout(
                Vec2::new(card_width, card_height),
                Layout::top_down(Align::Min),
                |ui| {
                    attitude::gauge_card(
                        ui,
                        "CONTROL",
                        &control_text,
                        &control_subtitle,
                        (loop_hz as f32 / 220.0).clamp(0.0, 1.0),
                        control_color,
                    );
                },
            );
        });
    }

    fn status_chips(&mut self, ui: &mut egui::Ui, telemetry: Option<&Telemetry>) {
        ui.horizontal_wrapped(|ui| {
            if let Some(telemetry) = telemetry {
                chip(
                    ui,
                    "IMU",
                    if telemetry.imu_fresh { "ok" } else { "stale" }.to_owned(),
                    if telemetry.imu_fresh {
                        theme::GREEN
                    } else {
                        theme::AMBER
                    },
                );
                chip(
                    ui,
                    "CAL",
                    format!("{}/3", telemetry.cal),
                    if telemetry.cal >= 3 {
                        theme::GREEN
                    } else {
                        theme::AMBER
                    },
                );
                chip(
                    ui,
                    "SYS ERR",
                    format!("0x{:02x}", telemetry.sys_err),
                    if telemetry.sys_err == 0 {
                        theme::GREEN
                    } else {
                        theme::RED
                    },
                );
                chip(
                    ui,
                    "FEEDBACK",
                    telemetry.feedback_age_text(),
                    if telemetry.feedback_present {
                        theme::GREEN
                    } else {
                        theme::MUTED
                    },
                );
                chip(
                    ui,
                    "BOARD",
                    format!("{:.1} °C", telemetry.board_temp_c),
                    theme::TEXT,
                );
                chip(
                    ui,
                    "SETPOINTS",
                    format!(
                        "{:+.2} m/s / {:+.0} °/s",
                        telemetry.speed_setpoint_mps,
                        telemetry.steer_setpoint_deg_s()
                    ),
                    theme::TEXT,
                );
                chip(ui, "STEP", telemetry.step.to_string(), theme::MUTED);
                chip(ui, "MOUNT", mount_name(self.mount).to_owned(), theme::TEXT);
                if let Some(level) = self.level_deg {
                    chip(ui, "LEVEL", format!("{level:+.2}°"), theme::TEXT);
                }
                if let Some(zero) = self.zero_offset_deg {
                    chip(ui, "ZERO", format!("{zero:+.2}°"), theme::MUTED);
                }
                chip(
                    ui,
                    "ARM",
                    if self.armed {
                        "YES".to_owned()
                    } else {
                        "no".to_owned()
                    },
                    if self.armed {
                        theme::GREEN
                    } else {
                        theme::MUTED
                    },
                );
            } else {
                ui.label(
                    RichText::new("waiting for telemetry")
                        .monospace()
                        .color(theme::MUTED),
                );
            }
            if let Some(stat) = &self.stat {
                chip(
                    ui,
                    "UPTIME",
                    format!("{:.1} s", stat.uptime_s),
                    theme::MUTED,
                );
            }
        });
    }

    fn event_log(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(
                RichText::new("EVENT LOG")
                    .size(11.0)
                    .strong()
                    .color(theme::ACCENT),
            );
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if ui.button("Clear").clicked() {
                    self.events.clear();
                }
            });
        });

        Frame::new()
            .fill(theme::CARD)
            .stroke(Stroke::new(1.0, theme::BORDER))
            .corner_radius(egui::CornerRadius::same(8))
            .inner_margin(Margin::same(8))
            .show(ui, |ui| {
                ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .max_height(200.0)
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        if self.events.is_empty() {
                            ui.label(
                                RichText::new("no events yet")
                                    .monospace()
                                    .size(11.0)
                                    .color(theme::MUTED),
                            );
                        }
                        for entry in &self.events {
                            let stamp = format_stamp(entry.at, self.started);
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new(stamp)
                                        .monospace()
                                        .size(10.0)
                                        .color(theme::MUTED),
                                );
                                ui.label(
                                    RichText::new(entry.level.tag())
                                        .monospace()
                                        .size(10.0)
                                        .color(entry.level.color()),
                                );
                                ui.label(
                                    RichText::new(&entry.text)
                                        .monospace()
                                        .size(11.0)
                                        .color(entry.level.color()),
                                );
                            });
                        }
                    });
            });
    }

    fn bottom_bar(&mut self, root: &mut egui::Ui) {
        egui::Panel::bottom("status_bar")
            .frame(
                Frame::new()
                    .fill(theme::PANEL)
                    .inner_margin(Margin::symmetric(14, 6)),
            )
            .show(root, |ui| {
                ui.horizontal(|ui| {
                    let age = self.telemetry_age();
                    let age_text = age.map_or_else(
                        || "--".to_owned(),
                        |age| format!("{:.0} ms", age.as_secs_f64() * 1_000.0),
                    );
                    let age_color = match age {
                        Some(age) if age < Duration::from_millis(500) => theme::GREEN,
                        Some(age) if age < Duration::from_secs(2) => theme::AMBER,
                        _ => theme::RED,
                    };
                    ui.label(
                        RichText::new(format!("telemetry {age_text}"))
                            .monospace()
                            .color(age_color),
                    );
                    ui.separator();
                    if let Some(telemetry) = &self.telemetry {
                        ui.label(
                            RichText::new(format!(
                                "step {} · loop {} Hz · estop {}",
                                telemetry.step,
                                telemetry.loop_hz,
                                if telemetry.estop { "ON" } else { "off" }
                            ))
                            .monospace()
                            .size(11.0)
                            .color(theme::MUTED),
                        );
                    } else {
                        ui.label(
                            RichText::new("no telemetry")
                                .monospace()
                                .size(11.0)
                                .color(theme::MUTED),
                        );
                    }
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        ui.label(
                            RichText::new(
                                "E e-stop · C clear · Space zero · ↑/↓ speed · ←/→ steer · Z zero · L level",
                            )
                            .size(10.0)
                            .color(theme::MUTED),
                        );
                    });
                });
            });
    }
}

impl ConsoleApp {
    /// Render the guided mount (axis) calibration wizard.
    fn axis_cal_ui(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Guided axis calibration")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(
                    RichText::new(
                        "Re-soldering or re-mounting the IMU changes which way its axes point. \
                         This wizard sweeps every mount selector while you hold one pose at a \
                         time, then applies the match.",
                    )
                    .size(10.0)
                    .color(theme::MUTED),
                );
                let state = self.axis_cal.clone();
                match state {
                    AxisCal::Idle => {
                        if ui.button("Start guided calibration").clicked() {
                            self.axis_cal_start();
                        }
                    }
                    AxisCal::PromptBaseline => {
                        ui.label("Step 1/5 · Stand the robot UPRIGHT and hold it still");
                        if ui.button("Capture upright").clicked() {
                            self.axis_cal_begin_sweep(CaptureKind::Baseline);
                        }
                        if ui.button("Cancel").clicked() {
                            self.axis_cal_cancel();
                        }
                    }
                    AxisCal::PromptLean(index) => {
                        let lean = Lean::ALL[index];
                        ui.label(format!("Step {}/5 · {}", index + 2, lean.instruction()));
                        if ui.button(lean.button_label()).clicked() {
                            self.axis_cal_begin_sweep(CaptureKind::Lean(index));
                        }
                        if ui.button("Cancel").clicked() {
                            self.axis_cal_cancel();
                        }
                    }
                    AxisCal::Capturing { kind, mount, .. } => {
                        let step = match kind {
                            CaptureKind::Baseline => "upright".to_owned(),
                            CaptureKind::Lean(index) => Lean::ALL[index].button_label().to_owned(),
                        };
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.label(format!(
                                "measuring {step} · mount {}/{}",
                                mount + 1,
                                MOUNT_COUNT
                            ));
                        });
                        ui.label(
                            RichText::new("Hold the pose; the sweep cycles through the selectors.")
                                .size(10.0)
                                .color(theme::MUTED),
                        );
                        if ui.button("Cancel").clicked() {
                            self.axis_cal_cancel();
                        }
                    }
                    AxisCal::Done { detection, scores } => {
                        ui.label(
                            RichText::new(format!(
                                "Detected: {} (selector {})",
                                MOUNT_NAMES[detection.best], detection.best
                            ))
                            .color(theme::GREEN)
                            .strong(),
                        );
                        ui.label(
                            RichText::new(format!(
                                "runner-up: {} · margin {:.1}",
                                MOUNT_NAMES[detection.second],
                                detection.second_score - detection.best_score
                            ))
                            .size(10.0)
                            .color(theme::MUTED),
                        );
                        Self::scores_table(ui, &scores, Some(detection.best));
                        ui.label(
                            RichText::new(
                                "Mount applied. Now hold the robot upright and press \
                                 Calibrate ZERO, then Calibrate LEVEL.",
                            )
                            .size(10.0)
                            .color(theme::MUTED),
                        );
                        if ui.button("Close").clicked() {
                            self.axis_cal = AxisCal::Idle;
                        }
                    }
                    AxisCal::Failed { reason, scores } => {
                        ui.label(RichText::new(format!("Failed: {reason}")).color(theme::RED));
                        let best = scores
                            .iter()
                            .enumerate()
                            .filter(|(_, score)| score.is_finite())
                            .min_by(|a, b| a.1.total_cmp(b.1))
                            .map(|(index, _)| index);
                        Self::scores_table(ui, &scores, best);
                        ui.horizontal(|ui| {
                            if ui.button("Retry").clicked() {
                                self.axis_cal_start();
                            }
                            if ui.button("Close").clicked() {
                                self.axis_cal = AxisCal::Idle;
                            }
                        });
                    }
                }
            });
    }

    /// Render the per-selector score table; the winner (when known) is
    /// highlighted in green. Lower is better.
    fn scores_table(ui: &mut egui::Ui, scores: &[f32; MOUNT_COUNT], best: Option<usize>) {
        egui::Grid::new("axis_cal_scores")
            .num_columns(2)
            .spacing([12.0, 2.0])
            .show(ui, |ui| {
                for (index, score) in scores.iter().enumerate() {
                    let text_color = if best == Some(index) {
                        theme::GREEN
                    } else {
                        theme::MUTED
                    };
                    ui.label(
                        RichText::new(format!("{} ({})", MOUNT_NAMES[index], index))
                            .monospace()
                            .size(10.0)
                            .color(text_color),
                    );
                    ui.label(
                        RichText::new(if score.is_infinite() {
                            "—".to_owned()
                        } else {
                            format!("{score:.1}")
                        })
                        .monospace()
                        .size(10.0)
                        .color(text_color),
                    );
                    ui.end_row();
                }
            });
    }
}

fn mean_pair(samples: &[(f32, f32)]) -> Option<(f32, f32)> {
    if samples.is_empty() {
        return None;
    }
    let mut pitch = 0.0_f32;
    let mut roll = 0.0_f32;
    for &(p, r) in samples {
        pitch += p;
        roll += r;
    }
    let n = samples.len() as f32;
    Some((pitch / n, roll / n))
}

impl eframe::App for ConsoleApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.poll_link();
        self.update_demo();
        self.update_axis_cal();
        self.handle_shortcuts(ui.ctx());
        self.top_bar(ui);
        self.bottom_bar(ui);
        self.controls_panel(ui);
        self.central_panel(ui);
        ui.ctx().request_repaint_after(Duration::from_millis(33));
    }
}

impl Drop for ConsoleApp {
    fn drop(&mut self) {
        self.disconnect(false);
    }
}

fn configure_style(ctx: &egui::Context) {
    ctx.set_theme(egui::ThemePreference::Dark);
    let mut style = (*ctx.style_of(egui::Theme::Dark)).clone();

    let visuals = &mut style.visuals;
    visuals.panel_fill = theme::PANEL;
    visuals.window_fill = theme::CARD;
    visuals.extreme_bg_color = theme::BG;
    visuals.faint_bg_color = theme::CARD_HIGH;
    visuals.selection.bg_fill = Color32::from_rgba_unmultiplied(78, 205, 220, 80);
    visuals.hyperlink_color = theme::ACCENT;
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, theme::BORDER);
    visuals.widgets.inactive.bg_fill = theme::CARD;
    visuals.widgets.hovered.bg_fill = theme::CARD_HIGH;
    visuals.widgets.active.bg_fill = theme::ACCENT;
    visuals.widgets.active.fg_stroke = Stroke::new(1.0, theme::BG);

    style.spacing.item_spacing = Vec2::new(8.0, 6.0);
    style.spacing.button_padding = Vec2::new(10.0, 5.0);
    style
        .text_styles
        .insert(egui::TextStyle::Heading, FontId::proportional(21.0));
    style
        .text_styles
        .insert(egui::TextStyle::Body, FontId::proportional(13.0));
    style
        .text_styles
        .insert(egui::TextStyle::Button, FontId::proportional(12.0));
    style
        .text_styles
        .insert(egui::TextStyle::Small, FontId::proportional(11.0));
    style
        .text_styles
        .insert(egui::TextStyle::Monospace, FontId::monospace(12.0));
    ctx.set_style_of(egui::Theme::Dark, style);
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(6.0);
    ui.label(
        RichText::new(title)
            .size(11.0)
            .strong()
            .color(theme::ACCENT),
    );
    ui.separator();
}

fn mount_name(mount: u8) -> &'static str {
    MOUNT_NAMES
        .get(mount as usize)
        .copied()
        .unwrap_or("unknown")
}

fn chip(ui: &mut egui::Ui, label: &str, value: String, color: Color32) {
    Frame::new()
        .fill(theme::CARD)
        .stroke(Stroke::new(1.0, theme::BORDER))
        .corner_radius(egui::CornerRadius::same(6))
        .inner_margin(Margin::symmetric(8, 4))
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new(label).size(9.0).color(theme::MUTED));
                ui.label(RichText::new(value).size(11.0).monospace().color(color));
            });
        });
}

fn format_stamp(at: Instant, started: Instant) -> String {
    let elapsed = at.saturating_duration_since(started).as_secs_f64();
    let minutes = (elapsed / 60.0).floor() as u64;
    let seconds = elapsed - minutes as f64 * 60.0;
    format!("{minutes:02}:{seconds:06.3}")
}
