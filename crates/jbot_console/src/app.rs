//! Application state and layout for the JBOT Pico desktop console.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use eframe::egui::{
    self, Align, Color32, FontId, Frame, Layout, Margin, RichText, ScrollArea, Stroke, Vec2,
};

use crate::attitude;
use crate::autocal::StillnessTracker;
use crate::axiscal::{self, Lean, Measurements, MOUNT_COUNT, MOUNT_NAMES};
use crate::link::{available_ports, looks_like_pico, LinkEvent, LinkHandle};
use crate::protocol::{Message, Stat, Telemetry, MAX_STREAM_HZ};
use crate::record::{self, EventLog, SessionRecorder};
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

/// State of the auto-calibration wizard: the operator holds the robot in
/// the approximate pose (upright for `zero`, the balance stance for
/// `level`) and the console captures the reference itself once the robot
/// has been completely still for a full window.
#[derive(Clone)]
enum AutoCal {
    /// Not running.
    Idle,
    /// Waiting for the operator to hold the robot UPRIGHT and completely still.
    WaitingUpright {
        /// Stillness detector for the upright pose.
        still: StillnessTracker,
        /// When this phase started.
        started: Instant,
    },
    /// `zero` was sent; the reply is pending.
    AwaitingZero {
        /// When the command was sent.
        started: Instant,
    },
    /// Waiting for the operator to hold the BALANCE STANCE and completely still.
    WaitingStance {
        /// Stillness detector for the balance stance.
        still: StillnessTracker,
        /// When this phase started.
        started: Instant,
    },
    /// `level` was sent; the reply is pending.
    AwaitingLevel {
        /// When the command was sent.
        started: Instant,
    },
    /// Both captures finished; references are live.
    Done,
    /// The wizard aborted; the operator can retry.
    Failed {
        /// Human-readable reason.
        reason: String,
    },
}

impl AutoCal {
    /// True while the wizard owns the calibration flow.
    fn running(&self) -> bool {
        matches!(
            self,
            Self::WaitingUpright { .. }
                | Self::AwaitingZero { .. }
                | Self::WaitingStance { .. }
                | Self::AwaitingLevel { .. }
        )
    }
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
    demo_test_expire: Option<Instant>,

    telemetry: Option<Telemetry>,
    telemetry_at: Option<Instant>,
    stat: Option<Stat>,
    last_step: Option<u32>,
    garbage_lines: u32,
    /// Ring buffer of raw inbound lines for the debug view.
    raw_lines: VecDeque<String>,
    /// Raw command input box for the debug panel.
    raw_input: String,
    /// Power scale slider; 1.0 is the compiled tuning.
    pscale: f32,
    /// Home bias slider, degrees.
    home_bias_deg: f32,
    /// Ramp rate slider, degrees per second.
    vmax_deg_s: f32,
    /// Whether the console polls `stat` automatically.
    auto_poll_stat: bool,
    /// Next time the automatic stat poll fires; armed while connected.
    stat_poll_at: Option<Instant>,
    /// Set by the automatic poll so the paired `stat` TX/RX lines stay out
    /// of the event log (the values still land in the debug panel).
    stat_log_quiet: bool,

    stream_hz: u32,
    speed_setpoint: f32,
    steer_setpoint_deg: f32,
    applied_speed: f32,
    applied_steer_deg: f32,
    test_setpoint: i32,
    applied_test_left: f32,
    applied_test_right: f32,
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
    autocal: AutoCal,
    autocal_saved_stream: u32,

    events: VecDeque<LogEntry>,
    started: Instant,
    recorder: Option<SessionRecorder>,
    event_log: Option<EventLog>,
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
            demo_test_expire: None,
            telemetry: None,
            telemetry_at: None,
            stat: None,
            last_step: None,
            garbage_lines: 0,
            raw_lines: VecDeque::new(),
            raw_input: String::new(),
            pscale: 1.0,
            home_bias_deg: 0.0,
            vmax_deg_s: 10.0,
            auto_poll_stat: true,
            stat_poll_at: None,
            stat_log_quiet: false,
            stream_hz: startup.stream_hz.clamp(1, MAX_STREAM_HZ),
            speed_setpoint: 0.0,
            steer_setpoint_deg: 0.0,
            applied_speed: 0.0,
            applied_steer_deg: 0.0,
            test_setpoint: 0,
            applied_test_left: 0.0,
            applied_test_right: 0.0,
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
            autocal: AutoCal::Idle,
            autocal_saved_stream: 0,
            events: VecDeque::new(),
            started,
            recorder: None,
            event_log: None,
        };

        if let Ok(event_log) =
            EventLog::open(&std::env::current_dir().unwrap_or_else(|_| ".".into()))
        {
            app.event_log = Some(event_log);
            let path = app.event_log.as_ref().map(|log| log.path().to_path_buf());
            if let Some(path) = path {
                app.log(Level::Info, format!("event log: {}", path.display()));
            }
        }

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

    /// Keep the diagnostics panel alive: poll `stat` every two seconds while
    /// connected. Read-only query, no actuator involved.
    fn poll_stat(&mut self) {
        if !self.auto_poll_stat || !self.connected || self.demo {
            return;
        }
        let now = Instant::now();
        let due = self.stat_poll_at.is_none_or(|at| now >= at);
        if due {
            self.stat_log_quiet = true;
            self.send("stat");
            self.stat_poll_at = Some(now + Duration::from_secs(2));
        }
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
                self.stat_poll_at = Some(Instant::now());
                self.speed_setpoint = 0.0;
                self.steer_setpoint_deg = 0.0;
                self.applied_speed = 0.0;
                self.applied_steer_deg = 0.0;
                self.test_setpoint = 0;
                self.applied_test_left = 0.0;
                self.applied_test_right = 0.0;
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("sys", &format!("connected {port}"));
                }
                self.log(Level::Info, format!("connected: {port}"));
            }
            LinkEvent::Disconnected { reason } => {
                self.connected = false;
                self.stat_poll_at = None;
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("sys", &format!("disconnected {reason}"));
                }
                self.log(Level::Warn, format!("disconnected: {reason}"));
            }
            LinkEvent::Message { raw, message } => {
                self.raw_lines.push_back(raw);
                while self.raw_lines.len() > 200 {
                    self.raw_lines.pop_front();
                }
                self.handle_message(*message)
            }
            LinkEvent::Notice(text) => {
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("sys", &text);
                }
                self.log(Level::Warn, text);
            }
        }
    }

    fn handle_message(&mut self, message: Message) {
        match message {
            Message::Telemetry(telemetry) => {
                let record_error = self
                    .recorder
                    .as_mut()
                    .map(|recorder| recorder.record_telemetry(&telemetry));
                if let Some(Err(error)) = record_error {
                    self.log(
                        Level::Error,
                        format!("recording failed: {error}; stopping recorder"),
                    );
                    self.recorder = None;
                }
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
                if let Some(left) = telemetry.test_left_units {
                    self.applied_test_left = left;
                }
                if let Some(right) = telemetry.test_right_units {
                    self.applied_test_right = right;
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
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("rx", &stat.raw);
                }
                let quiet = std::mem::take(&mut self.stat_log_quiet);
                let rx = stat
                    .link_bytes
                    .map_or_else(|| "?".to_owned(), |bytes| bytes.to_string());
                let fb_ok = stat
                    .link_fresh
                    .map_or("?", |fresh| if fresh { "1" } else { "0" });
                let stop = stat.stop_reason.as_deref().unwrap_or("none");
                if !quiet {
                    self.log(
                        Level::Rx,
                        format!(
                            "stat up={:.1}s loop={}Hz stream={}Hz imu_err={} fb={} rx={} drop={} fb_ok={} stop={} estop={}",
                            stat.uptime_s,
                            stat.loop_hz,
                            stat.stream_hz,
                            stat.imu_errors,
                            stat.fb_frames,
                            rx,
                            stat.fb_discarded,
                            fb_ok,
                            stop,
                            u8::from(stat.estop),
                        ),
                    );
                }
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
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("rx", &text);
                }
                match &mut self.autocal {
                    AutoCal::AwaitingZero { .. } if text.starts_with("ok zero") => {
                        self.autocal = AutoCal::WaitingStance {
                            still: StillnessTracker::new(),
                            started: Instant::now(),
                        };
                        self.log(
                            Level::Info,
                            "auto-calibration: zero captured; now hold the BALANCE STANCE, dead still",
                        );
                    }
                    AutoCal::AwaitingLevel { .. } if text.starts_with("ok level") => {
                        self.autocal_done();
                    }
                    _ => {}
                }
                self.apply_ok_reply(&text);
                self.log(Level::Rx, text);
            }
            Message::Err(text) => {
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("rx", &text);
                }
                if self.autocal_running() {
                    self.autocal_fail(&format!("firmware refused: {text}"));
                }
                self.log(Level::Error, text);
            }
            Message::Banner(text) => {
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("rx", &format!("banner {text}"));
                }
                self.log(Level::Info, format!("firmware: {text}"));
            }
            Message::Garbage(text) => {
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("rx", &format!("garbage {text}"));
                }
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
        } else if let Some(rest) = text.strip_prefix("ok pscale ") {
            if let Ok(value) = rest.trim().parse::<f32>() {
                self.pscale = value;
            }
        } else if let Some(rest) = text.strip_prefix("ok vmax ") {
            if let Ok(value) = rest.trim().parse::<f32>() {
                self.vmax_deg_s = value;
            }
        } else if let Some(rest) = text.strip_prefix("ok home ") {
            if let Ok(value) = rest.trim().parse::<f32>() {
                self.home_bias_deg = value;
            }
        }
    }

    fn update_demo(&mut self) {
        if !self.demo {
            return;
        }
        let now = Instant::now();
        if let Some(expire) = self.demo_test_expire {
            if now >= expire {
                self.applied_test_left = 0.0;
                self.applied_test_right = 0.0;
                self.demo_test_expire = None;
                self.log(Level::Info, "demo: spin test auto-stop (5 s)");
            }
        }
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
            test_left_units: Some(self.applied_test_left),
            test_right_units: Some(self.applied_test_right),
            torque_common_units: Some((pitch * 60.0).clamp(-1000.0, 1000.0)),
            pitch_ref_deg: Some(0.0),
            pitch_demand_deg: Some(pitch * 0.5),
            slope_trim_deg: Some(0.5 * (t * 0.07).sin()),
            position_m: Some(0.2 * (t * 0.1).sin()),
            balance_mode: Some(0),
        });
        self.telemetry_at = Some(now);
    }

    /// Synthetic `stat` for demo mode: every field populated so the
    /// diagnostics panel has something to show without the robot.
    fn demo_stat(&self) -> Stat {
        let t = self.demo_started.elapsed().as_secs_f32();
        Stat {
            uptime_s: t,
            loop_hz: 200,
            stream_hz: self.stream_hz,
            trim_deg: self.trim_deg,
            estop: self.demo_estop,
            steps: (t * 200.0) as u32,
            imu_samples: (t * 200.0) as u32,
            imu_errors: 0,
            imu_code: 0,
            fb_frames: (t * 100.0) as u32,
            fb_discarded: 0,
            link_bytes: Some((t * 1800.0) as u32),
            link_fresh: Some(true),
            stop_reason: Some("none".to_owned()),
            zero_deg: Some(self.zero_offset_deg.unwrap_or(1.8)),
            zroll_deg: Some(self.roll_zero_offset_deg.unwrap_or(-0.6)),
            level_deg: Some(self.level_deg.unwrap_or(self.trim_deg)),
            mount: Some(self.mount),
            armed: Some(false),
            home_deg: Some(self.home_bias_deg),
            fall_events: Some(0),
            ref_vmax_deg_s: Some(self.vmax_deg_s),
            kp_nm_per_rad: Some(28.6 * self.pscale),
            kd_nm_per_rad_s: Some(7.2 * self.pscale),
            position_gain_per_s: Some(0.4),
            velocity_gain_per_s: Some(1.8),
            trim_gain_rad_per_m_s: Some(0.010),
            demand_limit_deg: Some(8.0),
            trim_limit_deg: Some(20.0),
            zero_captured: Some(true),
            level_captured: Some(true),
            pscale: Some(self.pscale),
            raw: "demo stat".to_owned(),
        }
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
        let wizard_busy = self.autocal.running();

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
        if keys.7 && !wizard_busy {
            self.calibrate_zero();
        }
        if keys.8 && !wizard_busy {
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

    /// True while the auto-calibration wizard owns the calibration flow.
    fn autocal_running(&self) -> bool {
        self.autocal.running()
    }

    /// Start the auto-calibration wizard: capture `zero` at the upright
    /// pose, then `level` at the balance stance, both triggered by measured
    /// stillness instead of a button press.
    fn autocal_start(&mut self) {
        if !self.demo && !self.connected {
            self.log(
                Level::Error,
                "auto-calibration needs a live link to the firmware (or demo mode)",
            );
            return;
        }
        if self.estop_latched() {
            self.log(Level::Warn, "e-stop latched: clear it before calibrating");
            return;
        }
        self.autocal_saved_stream = self.stream_hz;
        if self.stream_hz != 50 {
            self.send("stream 50");
        }
        self.autocal = AutoCal::WaitingUpright {
            still: StillnessTracker::new(),
            started: Instant::now(),
        };
        self.log(
            Level::Info,
            "auto-calibration: hold the robot UPRIGHT and dead still; capturing zero when it is still",
        );
    }

    /// Abort the wizard and restore the stream rate.
    fn autocal_cancel(&mut self) {
        if !self.autocal.running() {
            return;
        }
        let stream = self.autocal_saved_stream;
        if self.stream_hz != stream {
            self.send(format!("stream {stream}"));
        }
        self.autocal = AutoCal::Idle;
        self.log(Level::Info, "auto-calibration cancelled");
    }

    /// Fail the wizard with a reason and restore the stream rate.
    fn autocal_fail(&mut self, reason: &str) {
        let stream = self.autocal_saved_stream;
        if self.stream_hz != stream {
            self.send(format!("stream {stream}"));
        }
        self.autocal = AutoCal::Failed {
            reason: reason.to_owned(),
        };
        self.log(Level::Error, format!("auto-calibration failed: {reason}"));
    }

    /// Complete the wizard and restore the stream rate.
    fn autocal_done(&mut self) {
        let stream = self.autocal_saved_stream;
        if self.stream_hz != stream {
            self.send(format!("stream {stream}"));
        }
        self.autocal = AutoCal::Done;
        self.log(
            Level::Info,
            "auto-calibration done: zero and level captured while still",
        );
    }

    /// Advance the wizard from the latest telemetry.
    fn update_autocal(&mut self) {
        const PHASE_TIMEOUT: Duration = Duration::from_secs(30);
        const REPLY_TIMEOUT: Duration = Duration::from_secs(4);

        if self.estop_latched() && self.autocal.running() {
            self.autocal_fail("e-stop latched");
            return;
        }

        match &mut self.autocal {
            AutoCal::WaitingUpright { still, started } => {
                if started.elapsed() > PHASE_TIMEOUT {
                    self.autocal_fail(
                        "the robot never became still at the upright pose; hold it motionless and retry",
                    );
                    return;
                }
                let quiet = self.demo
                    || self
                        .telemetry
                        .as_ref()
                        .map(|t| still.update(t.gyro_y_dps, t.gyro_x_dps, t.gyro_z_dps))
                        .unwrap_or(false);
                if quiet {
                    self.send("zero");
                    self.autocal = AutoCal::AwaitingZero {
                        started: Instant::now(),
                    };
                    self.log(
                        Level::Info,
                        "auto-calibration: zero sent; now hold the BALANCE STANCE, dead still",
                    );
                }
            }
            AutoCal::AwaitingZero { started } => {
                if self.demo {
                    self.zero_offset_deg = Some(1.8);
                    self.autocal = AutoCal::WaitingStance {
                        still: StillnessTracker::new(),
                        started: Instant::now(),
                    };
                    return;
                }
                if started.elapsed() > REPLY_TIMEOUT {
                    self.autocal_fail("no reply to `zero`");
                }
            }
            AutoCal::WaitingStance { still, started } => {
                if started.elapsed() > PHASE_TIMEOUT {
                    self.autocal_fail(
                        "the robot never became still at the balance stance; hold it motionless and retry",
                    );
                    return;
                }
                let quiet = self.demo
                    || self
                        .telemetry
                        .as_ref()
                        .map(|t| still.update(t.gyro_y_dps, t.gyro_x_dps, t.gyro_z_dps))
                        .unwrap_or(false);
                if quiet {
                    self.send("level");
                    self.autocal = AutoCal::AwaitingLevel {
                        started: Instant::now(),
                    };
                    self.log(
                        Level::Info,
                        "auto-calibration: level sent; references are captured",
                    );
                }
            }
            AutoCal::AwaitingLevel { started } => {
                if self.demo {
                    self.level_deg = Some(0.3);
                    self.autocal_done();
                    return;
                }
                if started.elapsed() > REPLY_TIMEOUT {
                    self.autocal_fail("no reply to `level`");
                }
            }
            AutoCal::Idle | AutoCal::Done | AutoCal::Failed { .. } => {}
        }
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

    /// Send a bench spin-test request. `0` stops the test. Torque is in
    /// EFeru command units; the firmware clamps to ±300, slews the wheels up
    /// smoothly and auto-stops the request after five seconds.
    fn apply_test(&mut self, units: i32) {
        let clamped = units.clamp(-300, 300);
        if clamped != 0 && self.estop_latched() {
            self.log(Level::Warn, "e-stop latched: clear it before the spin test");
            return;
        }
        if clamped != 0 && self.armed {
            self.log(
                Level::Warn,
                "balance law armed: disarm it before the spin test",
            );
            return;
        }
        self.test_setpoint = clamped;
        if clamped == 0 {
            self.applied_test_left = 0.0;
            self.applied_test_right = 0.0;
            self.demo_test_expire = None;
        } else if self.demo {
            self.applied_test_left = clamped as f32;
            self.applied_test_right = clamped as f32;
            // The firmware drops every accepted request after five seconds;
            // the demo mirrors that so the panel behaves the same.
            self.demo_test_expire = Some(Instant::now() + Duration::from_secs(5));
        }
        self.send(format!("test {clamped}"));
    }

    fn zero(&mut self) {
        self.speed_setpoint = 0.0;
        self.steer_setpoint_deg = 0.0;
        self.applied_speed = 0.0;
        self.applied_steer_deg = 0.0;
        self.test_setpoint = 0;
        self.applied_test_left = 0.0;
        self.applied_test_right = 0.0;
        self.demo_test_expire = None;
        self.send("speed 0");
        self.send("steer 0");
        self.send("test 0");
        self.log(Level::Info, "soft stop: setpoints zeroed");
    }

    fn emergency_stop(&mut self) {
        self.demo_estop = true;
        self.speed_setpoint = 0.0;
        self.steer_setpoint_deg = 0.0;
        self.applied_speed = 0.0;
        self.applied_steer_deg = 0.0;
        self.test_setpoint = 0;
        self.applied_test_left = 0.0;
        self.applied_test_right = 0.0;
        self.demo_test_expire = None;
        self.send("estop");
        self.send("test 0");
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
        let quiet = line == "stat" && std::mem::take(&mut self.stat_log_quiet);
        if self.demo {
            if !quiet {
                self.log(Level::Tx, format!("demo -> {line}"));
            }
            if line.trim_start() == "stat" {
                self.stat = Some(self.demo_stat());
            }
            if let Some(recorder) = &mut self.recorder {
                recorder.record_action("tx", &line);
            }
            return;
        }
        let sent = self.link.as_ref().map(|link| link.send(line.clone()));
        match sent {
            Some(true) => {
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("tx", &line);
                }
                if !quiet {
                    self.log(Level::Tx, line);
                }
            }
            Some(false) => {
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("tx-drop", &line);
                }
                self.log(Level::Warn, format!("command dropped: {line}"));
            }
            None => {
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("tx-down", &line);
                }
                self.log(Level::Warn, format!("link down: {line}"));
            }
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
                if let Some(recorder) = &mut self.recorder {
                    recorder.record_action("sys", "link closed by the console");
                }
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
            if let Some(recorder) = &mut self.recorder {
                recorder.record_action("ui", "demo mode enabled");
            }
        } else {
            self.telemetry = None;
            self.telemetry_at = None;
            self.log(Level::Info, "demo mode disabled");
            if let Some(recorder) = &mut self.recorder {
                recorder.record_action("ui", "demo mode disabled");
            }
        }
    }

    fn toggle_recording(&mut self) {
        if let Some(recorder) = self.recorder.take() {
            let telemetry_path = recorder.telemetry_path().to_path_buf();
            let actions_path = recorder.actions_path().to_path_buf();
            let telemetry_rows = recorder.telemetry_rows();
            let action_rows = recorder.action_rows();
            let mut recorder = recorder;
            recorder.flush();
            self.log(
                Level::Info,
                format!(
                    "recording stopped: {telemetry_rows} telemetry rows -> {}, {action_rows} action rows -> {}",
                    telemetry_path.display(),
                    actions_path.display(),
                ),
            );
            return;
        }
        let dir = match std::env::current_dir() {
            Ok(dir) => dir,
            Err(error) => {
                self.log(
                    Level::Error,
                    format!("cannot determine the working directory: {error}"),
                );
                return;
            }
        };
        match record::start(&dir) {
            Ok(recorder) => {
                self.log(
                    Level::Info,
                    format!(
                        "recording -> {} and {}",
                        recorder.telemetry_path().display(),
                        recorder.actions_path().display(),
                    ),
                );
                self.recorder = Some(recorder);
            }
            Err(error) => {
                self.log(Level::Error, format!("cannot start the recording: {error}"));
            }
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
        let text = text.into();
        if let Some(event_log) = &mut self.event_log {
            event_log.record(level.tag(), &text);
        }
        self.events.push_back(LogEntry {
            at: Instant::now(),
            level,
            text,
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
                        self.motor_test_controls(ui);
                        self.balance_controls(ui);
                        self.calibration_controls(ui);
                        self.home_controls(ui);
                        self.diagnostics_controls(ui);
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

    fn motor_test_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "MOTOR TEST (bench)");
        let latched = self.estop_latched();
        let locked = latched || self.armed;
        ui.label(
            RichText::new("Bench use only: hold the robot or keep the wheels clear.")
                .size(10.0)
                .color(theme::AMBER),
        );
        ui.add_enabled_ui(!locked, |ui| {
            ui.add(
                egui::Slider::new(&mut self.test_setpoint, -300..=300)
                    .text("Torque")
                    .step_by(10.0),
            );
            ui.horizontal(|ui| {
                for preset in [-150, -75, 75, 150] {
                    if ui.button(format!("{preset:+}")).clicked() {
                        self.apply_test(preset);
                    }
                }
            });
            if ui.button("Apply torque").clicked() {
                self.apply_test(self.test_setpoint);
            }
        });
        let stop = egui::Button::new(RichText::new("STOP (0)").strong().color(Color32::WHITE))
            .fill(theme::RED)
            .min_size(egui::vec2(ui.available_width(), 32.0));
        if ui.add(stop).clicked() {
            self.test_setpoint = 0;
            self.apply_test(0);
        }
        ui.label(
            RichText::new(format!(
                "applied: L {:+.0} · R {:+.0} units",
                self.applied_test_left, self.applied_test_right
            ))
            .monospace()
            .size(11.0)
            .color(theme::MUTED),
        );
        if self.applied_test_left != 0.0 || self.applied_test_right != 0.0 {
            ui.label(
                RichText::new("MOTOR TEST ACTIVE — auto-stops 5 s after the last command")
                    .color(theme::AMBER)
                    .strong(),
            );
        }
        if latched {
            ui.label(
                RichText::new("E-stop latched: spin test locked")
                    .color(theme::RED)
                    .strong(),
            );
        } else if self.armed {
            ui.label(
                RichText::new("Balance law armed: spin test locked (disarm first)")
                    .color(theme::AMBER)
                    .strong(),
            );
        }
        ui.label(
            RichText::new("±300 units max, ramped start. ±1000 units = full motor current.")
                .size(10.0)
                .color(theme::MUTED),
        );
    }

    fn calibration_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "CALIBRATION");
        self.autocal_ui(ui);
        let busy = self.axis_cal_running() || self.autocal_running();
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
        let recording = self.recorder.is_some();
        let record_label = match &self.recorder {
            Some(recorder) => format!(
                "STOP recording ({} telem / {} actions)",
                recorder.telemetry_rows(),
                recorder.action_rows()
            ),
            None => "Record session → CSV".to_owned(),
        };
        let record_button = egui::Button::new(RichText::new(record_label).strong())
            .min_size(egui::vec2(ui.available_width(), 30.0));
        if ui.add(record_button).clicked() {
            self.toggle_recording();
        }
        ui.label(
            RichText::new(
                "Writes jbot-telemetry-*.csv (attitude, rates, rpm, torque) and jbot-actions-*.csv (every command sent, every reply, connect events) to the working directory.",
            )
            .size(10.0)
            .color(theme::MUTED),
        );
        if recording {
            ui.label(RichText::new("RECORDING ACTIVE").color(theme::RED).strong());
        }
        ui.add_space(8.0);
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

    fn balance_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "BALANCE");
        let telemetry = self.telemetry.clone();
        let stat = self.stat.clone();
        ui.horizontal_wrapped(|ui| {
            if let Some(telemetry) = &telemetry {
                if let Some(mode) = telemetry.balance_mode {
                    let color = match mode {
                        2 => theme::GREEN,
                        3 => theme::AMBER,
                        4 => theme::RED,
                        _ => theme::MUTED,
                    };
                    chip(ui, "MODE", balance_mode_name(Some(mode)).to_owned(), color);
                }
            }
            if let Some(stat) = &stat {
                if let Some(stop) = &stat.stop_reason {
                    let color = if stop == "none" {
                        theme::GREEN
                    } else {
                        theme::RED
                    };
                    chip(ui, "STOP", stop.clone(), color);
                }
                if let Some(fall) = stat.fall_events {
                    if fall > 0 {
                        chip(ui, "FALL", fall.to_string(), theme::RED);
                    }
                }
            }
        });

        if let Some(t) = &telemetry {
            ui.label(
                RichText::new(format!(
                    "pitch {:+.2}° · ref {} · dem {} · trim {} · x {}",
                    t.pitch_deg,
                    opt_signed(t.pitch_ref_deg, 2),
                    opt_signed(t.pitch_demand_deg, 2),
                    opt_signed(t.slope_trim_deg, 2),
                    opt_signed(t.position_m, 3),
                ))
                .monospace()
                .size(11.0)
                .color(theme::TEXT),
            );
            let trq = t.torque_common_units.unwrap_or(0.0);
            let trq_color = if trq.abs() > 800.0 {
                theme::RED
            } else if trq.abs() > 500.0 {
                theme::AMBER
            } else {
                theme::ACCENT
            };
            ui.label(
                RichText::new(format!("balance torque {trq:+.0} / ±1000 units"))
                    .monospace()
                    .size(10.0)
                    .color(theme::MUTED),
            );
            centered_bar(ui, trq, 1000.0, trq_color);
            if let (Some(left), Some(right)) = (t.test_left_units, t.test_right_units) {
                ui.label(
                    RichText::new(format!("spin test L {left:+.0} · R {right:+.0} units"))
                        .monospace()
                        .size(10.0)
                        .color(theme::MUTED),
                );
            }
        } else {
            ui.label(RichText::new("--").color(theme::MUTED));
        }

        if let Some(stat) = &stat {
            ui.label(
                RichText::new(format!(
                    "effective: kp {} · kd {} · kpos {} · kv {} · ki {}",
                    opt_f32(stat.kp_nm_per_rad, 1),
                    opt_f32(stat.kd_nm_per_rad_s, 1),
                    opt_f32(stat.position_gain_per_s, 2),
                    opt_f32(stat.velocity_gain_per_s, 2),
                    opt_f32(stat.trim_gain_rad_per_m_s, 3),
                ))
                .monospace()
                .size(10.0)
                .color(theme::MUTED),
            );
            ui.label(
                RichText::new(format!(
                    "zero captured {} · level captured {} · mount {} ({})",
                    stat.zero_captured.map_or("?".to_owned(), |v| if v {
                        "yes".to_owned()
                    } else {
                        "NO".to_owned()
                    }),
                    stat.level_captured.map_or("?".to_owned(), |v| if v {
                        "yes".to_owned()
                    } else {
                        "NO".to_owned()
                    }),
                    stat.mount.map_or("?".to_owned(), |m| m.to_string()),
                    stat.mount.map_or("unknown", mount_name),
                ))
                .monospace()
                .size(10.0)
                .color(theme::TEXT),
            );
        } else {
            ui.label(
                RichText::new("press Stat or enable auto-poll for the law internals")
                    .size(10.0)
                    .color(theme::MUTED),
            );
        }
    }

    fn home_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "HOME & RAMP");
        ui.add(
            egui::Slider::new(&mut self.home_bias_deg, -5.0..=5.0)
                .text("Home bias")
                .suffix(" °")
                .step_by(0.1),
        );
        ui.horizontal(|ui| {
            if ui.button("Apply home").clicked() {
                self.send(format!("home {:.2}", self.home_bias_deg));
            }
            if ui.button("Home = level").clicked() {
                self.home_bias_deg = 0.0;
                self.send("home 0");
            }
            if let Some(home) = self.stat.as_ref().and_then(|stat| stat.home_deg) {
                ui.label(
                    RichText::new(format!("home {:+.2}°", home))
                        .monospace()
                        .size(11.0)
                        .color(theme::TEXT),
                );
            }
        });
        ui.label(
            RichText::new(
                "posture bias from the captured level; the reference ramps there smoothly",
            )
            .size(10.0)
            .color(theme::MUTED),
        );
        ui.add(
            egui::Slider::new(&mut self.vmax_deg_s, 0.0..=15.0)
                .text("Ramp rate")
                .suffix(" °/s")
                .step_by(0.5),
        );
        ui.horizontal(|ui| {
            if ui.button("Apply vmax").clicked() {
                self.send(format!("vmax {:.2}", self.vmax_deg_s));
            }
            if let Some(vmax) = self.stat.as_ref().and_then(|stat| stat.ref_vmax_deg_s) {
                ui.label(
                    RichText::new(format!("vmax {:.1} °/s", vmax))
                        .monospace()
                        .size(11.0)
                        .color(theme::TEXT),
                );
            }
        });

        ui.add_space(6.0);
        ui.add(
            egui::Slider::new(&mut self.pscale, 0.0..=2.0)
                .text("Power scale")
                .step_by(0.05),
        );
        ui.horizontal(|ui| {
            if ui.button("Apply pscale").clicked() {
                self.send(format!("pscale {:.2}", self.pscale));
            }
            if ui.button("Reset 1.0").clicked() {
                self.pscale = 1.0;
                self.send("pscale 1");
            }
            if let Some(pscale) = self.stat.as_ref().and_then(|stat| stat.pscale) {
                ui.label(
                    RichText::new(format!("pscale {:.2}", pscale))
                        .monospace()
                        .size(11.0)
                        .color(theme::AMBER),
                );
            }
        });
        ui.label(
            RichText::new(
                "bench scale on pitch kp/kd (and yaw): 1.0 = compiled tuning, 0 = law inert. Unscaled: fall guards, torque budget, trim. Runtime-only; resets to 1.0 at boot. Arm with 0.2–0.3 while holding the robot, then raise in steps.",
            )
            .size(10.0)
            .color(theme::AMBER),
        );
    }

    fn diagnostics_controls(&mut self, ui: &mut egui::Ui) {
        section(ui, "DIAGNOSTICS");
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.auto_poll_stat, "Auto-poll stat (2 s)");
            if ui.button("Poll now").clicked() {
                self.send("stat");
            }
        });

        let telemetry = self.telemetry.clone();
        let stat = self.stat.clone();
        egui::CollapsingHeader::new("All values")
            .default_open(false)
            .show(ui, |ui| {
                let rows = debug_rows(telemetry.as_ref(), stat.as_ref());
                egui::Grid::new("all_values_grid")
                    .num_columns(2)
                    .spacing([10.0, 2.0])
                    .show(ui, |ui| {
                        for (label, value) in rows {
                            ui.label(RichText::new(label).size(10.0).color(theme::MUTED));
                            ui.label(
                                RichText::new(value)
                                    .monospace()
                                    .size(10.0)
                                    .color(theme::TEXT),
                            );
                            ui.end_row();
                        }
                    });
            });

        egui::CollapsingHeader::new("Raw lines (last 200)")
            .default_open(false)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("Copy raw lines").clicked() {
                        let text = self
                            .raw_lines
                            .iter()
                            .cloned()
                            .collect::<Vec<_>>()
                            .join("\n");
                        ui.ctx().copy_text(text);
                    }
                    if ui.button("Copy debug snapshot").clicked() {
                        let rows = debug_rows(self.telemetry.as_ref(), self.stat.as_ref());
                        let text = rows
                            .into_iter()
                            .map(|(label, value)| format!("{label}={value}"))
                            .collect::<Vec<_>>()
                            .join("\n");
                        ui.ctx().copy_text(text);
                    }
                });
                Frame::new()
                    .fill(theme::CARD)
                    .stroke(Stroke::new(1.0, theme::BORDER))
                    .corner_radius(egui::CornerRadius::same(6))
                    .inner_margin(Margin::same(6))
                    .show(ui, |ui| {
                        ScrollArea::vertical()
                            .max_height(140.0)
                            .stick_to_bottom(true)
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                if self.raw_lines.is_empty() {
                                    ui.label(
                                        RichText::new("no raw lines yet")
                                            .monospace()
                                            .size(9.0)
                                            .color(theme::MUTED),
                                    );
                                }
                                for line in &self.raw_lines {
                                    ui.label(
                                        RichText::new(line)
                                            .monospace()
                                            .size(9.0)
                                            .color(theme::TEXT),
                                    );
                                }
                            });
                    });
            });

        ui.horizontal(|ui| {
            let response = ui.add(
                egui::TextEdit::singleline(&mut self.raw_input)
                    .hint_text("raw command, e.g. stat")
                    .desired_width(ui.available_width() - 62.0),
            );
            let send = ui.button("Send").clicked()
                || (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)));
            if send {
                let line = self.raw_input.trim().to_owned();
                if !line.is_empty() {
                    self.send(line);
                    self.raw_input.clear();
                }
            }
        });
        ui.label(
            RichText::new("one firmware line per send; the exact bytes the console itself uses")
                .size(9.0)
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
                chip(
                    ui,
                    "ESTOP",
                    if telemetry.estop {
                        "LATCHED".to_owned()
                    } else {
                        "ok".to_owned()
                    },
                    if telemetry.estop {
                        theme::RED
                    } else {
                        theme::GREEN
                    },
                );
                if let Some(mode) = telemetry.balance_mode {
                    let color = match mode {
                        2 => theme::GREEN,
                        3 | 1 => theme::AMBER,
                        4 => theme::RED,
                        _ => theme::MUTED,
                    };
                    chip(ui, "MODE", balance_mode_name(Some(mode)).to_owned(), color);
                }
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
                if let Some(stop) = &stat.stop_reason {
                    let color = if stop == "none" {
                        theme::GREEN
                    } else {
                        theme::RED
                    };
                    chip(ui, "STOP", stop.clone(), color);
                }
                if let Some(fall) = stat.fall_events {
                    if fall > 0 {
                        chip(ui, "FALL", fall.to_string(), theme::RED);
                    }
                }
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
    /// Render the SET HOME (auto-calibration) wizard.
    fn autocal_ui(&mut self, ui: &mut egui::Ui) {
        ui.label(
            RichText::new(
                "SET HOME — hold the pose, the console captures it once the robot is still.",
            )
            .size(10.0)
            .color(theme::MUTED),
        );
        match &self.autocal {
            AutoCal::Idle => {
                let button = egui::Button::new(
                    RichText::new("SET HOME")
                        .strong()
                        .size(14.0)
                        .color(Color32::WHITE),
                )
                .fill(theme::ACCENT)
                .min_size(egui::vec2(ui.available_width(), 34.0));
                if ui.add(button).clicked() {
                    self.autocal_start();
                }
            }
            AutoCal::Done => {
                ui.label(
                    RichText::new("home set — references captured")
                        .color(theme::GREEN)
                        .strong(),
                );
                let button = egui::Button::new(RichText::new("Re-capture home").strong())
                    .fill(theme::ACCENT)
                    .min_size(egui::vec2(ui.available_width(), 30.0));
                if ui.add(button).clicked() {
                    self.autocal_start();
                }
            }
            AutoCal::Failed { reason } => {
                ui.label(
                    RichText::new(format!("auto-calibration failed: {reason}")).color(theme::AMBER),
                );
                let button = egui::Button::new(RichText::new("Try again").strong())
                    .fill(theme::ACCENT)
                    .min_size(egui::vec2(ui.available_width(), 30.0));
                if ui.add(button).clicked() {
                    self.autocal_start();
                }
            }
            AutoCal::WaitingUpright { still, .. } => {
                ui.label(
                    RichText::new("1/2  Hold the robot UPRIGHT and dead still.")
                        .color(theme::ACCENT)
                        .strong(),
                );
                ui.add(
                    egui::ProgressBar::new(still.progress()).desired_width(ui.available_width()),
                );
                if ui.button("Cancel").clicked() {
                    self.autocal_cancel();
                }
            }
            AutoCal::AwaitingZero { .. } => {
                ui.label(
                    RichText::new("1/2  Capturing zero...")
                        .color(theme::ACCENT)
                        .strong(),
                );
                ui.add(egui::ProgressBar::new(1.0).desired_width(ui.available_width()));
            }
            AutoCal::WaitingStance { still, .. } => {
                ui.label(
                    RichText::new("2/2  Hold the BALANCE STANCE and dead still.")
                        .color(theme::ACCENT)
                        .strong(),
                );
                ui.add(
                    egui::ProgressBar::new(still.progress()).desired_width(ui.available_width()),
                );
                if ui.button("Cancel").clicked() {
                    self.autocal_cancel();
                }
            }
            AutoCal::AwaitingLevel { .. } => {
                ui.label(
                    RichText::new("2/2  Capturing level...")
                        .color(theme::ACCENT)
                        .strong(),
                );
                ui.add(egui::ProgressBar::new(1.0).desired_width(ui.available_width()));
            }
        }
    }

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
        self.update_autocal();
        self.handle_shortcuts(ui.ctx());
        self.poll_stat();
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

/// Balance mode byte from the firmware, as a word.
fn balance_mode_name(mode: Option<u8>) -> &'static str {
    match mode {
        Some(0) => "disarmed",
        Some(1) => "transit",
        Some(2) => "hold",
        Some(3) => "recover",
        Some(4) => "fall",
        _ => "?",
    }
}

/// Format an optional number with fixed precision, `--` when absent.
fn opt_f32(value: Option<f32>, precision: usize) -> String {
    value.map_or_else(|| "--".to_owned(), |v| format!("{:.*}", precision, v))
}

/// Same as [`opt_f32`] but always signed.
fn opt_signed(value: Option<f32>, precision: usize) -> String {
    value.map_or_else(|| "--".to_owned(), |v| format!("{:+.*}", precision, v))
}

/// Horizontal zero-centred bar for a signed value; the fill grows from the
/// centre in the sign direction.
fn centered_bar(ui: &mut egui::Ui, value: f32, max: f32, color: Color32) {
    let width = ui.available_width();
    let (rect, _response) = ui.allocate_exact_size(Vec2::new(width, 16.0), egui::Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(rect, egui::CornerRadius::same(4), theme::CARD);
    painter.rect_stroke(
        rect,
        egui::CornerRadius::same(4),
        Stroke::new(1.0, theme::BORDER),
        egui::StrokeKind::Inside,
    );
    let centre = rect.center().x;
    let half = width * 0.5 - 3.0;
    painter.line_segment(
        [
            egui::pos2(centre, rect.top() + 2.0),
            egui::pos2(centre, rect.bottom() - 2.0),
        ],
        Stroke::new(1.0, theme::MUTED),
    );
    let fraction = (value / max).clamp(-1.0, 1.0);
    let end = centre + fraction * half;
    let fill = if fraction >= 0.0 {
        egui::Rect::from_min_max(
            egui::pos2(centre, rect.top() + 3.0),
            egui::pos2(end, rect.bottom() - 3.0),
        )
    } else {
        egui::Rect::from_min_max(
            egui::pos2(end, rect.top() + 3.0),
            egui::pos2(centre, rect.bottom() - 3.0),
        )
    };
    if fraction.abs() > 0.005 {
        painter.rect_filled(fill, egui::CornerRadius::same(3), color);
    }
}

/// Every diagnostic value the console knows about, as label/value rows.
/// Shared by the on-screen table and the clipboard snapshot.
fn debug_rows(telemetry: Option<&Telemetry>, stat: Option<&Stat>) -> Vec<(&'static str, String)> {
    fn yes_no(value: Option<bool>) -> String {
        value.map_or_else(
            || "--".to_owned(),
            |v| if v { "yes".to_owned() } else { "no".to_owned() },
        )
    }
    let mut rows: Vec<(&'static str, String)> = Vec::with_capacity(56);
    let mut row = |label: &'static str, value: String| rows.push((label, value));

    match telemetry {
        Some(t) => {
            row("tele.step", t.step.to_string());
            row("tele.loop_hz", t.loop_hz.to_string());
            row("tele.imu_fresh", yes_no(Some(t.imu_fresh)));
            row("tele.cal", format!("{}/3", t.cal));
            row("tele.sys_err", format!("0x{:02x}", t.sys_err));
            row("tele.pitch_deg", format!("{:+.2}", t.pitch_deg));
            row("tele.roll_deg", format!("{:+.2}", t.roll_deg));
            row("tele.heading_deg", format!("{:+.2}", t.heading_deg));
            row("tele.gyro_x_dps", format!("{:+.2}", t.gyro_x_dps));
            row("tele.gyro_y_dps", format!("{:+.2}", t.gyro_y_dps));
            row("tele.gyro_z_dps", format!("{:+.2}", t.gyro_z_dps));
            row("tele.speed_left_rpm", t.speed_left_rpm.to_string());
            row("tele.speed_right_rpm", t.speed_right_rpm.to_string());
            row("tele.battery_v", format!("{:.2}", t.battery_v));
            row("tele.board_temp_c", format!("{:.1}", t.board_temp_c));
            row("tele.feedback_age_ms", t.feedback_age_ms.to_string());
            row(
                "tele.estop",
                if t.estop {
                    "LATCHED".to_owned()
                } else {
                    "clear".to_owned()
                },
            );
            row("tele.vset_mps", format!("{:+.3}", t.speed_setpoint_mps));
            row(
                "tele.wset_deg_s",
                format!("{:+.2}", t.steer_setpoint_deg_s()),
            );
            row(
                "tele.mount",
                t.mount
                    .map_or_else(|| "--".to_owned(), |m| format!("{} {}", m, mount_name(m))),
            );
            row("tele.zero_deg", opt_signed(t.zero_offset_deg, 2));
            row("tele.roll_zero_deg", opt_signed(t.roll_zero_offset_deg, 2));
            row("tele.level_deg", opt_signed(t.level_deg, 2));
            row("tele.armed", yes_no(t.armed));
            row("tele.test_l_units", opt_signed(t.test_left_units, 0));
            row("tele.test_r_units", opt_signed(t.test_right_units, 0));
            row("tele.trq_units", opt_signed(t.torque_common_units, 0));
            row("tele.ref_deg", opt_signed(t.pitch_ref_deg, 2));
            row("tele.dem_deg", opt_signed(t.pitch_demand_deg, 2));
            row("tele.trim_deg", opt_signed(t.slope_trim_deg, 2));
            row("tele.x_m", opt_signed(t.position_m, 3));
            row(
                "tele.mode",
                t.balance_mode.map_or_else(
                    || "--".to_owned(),
                    |m| format!("{} {}", m, balance_mode_name(Some(m))),
                ),
            );
        }
        None => row("tele.*", "no telemetry".to_owned()),
    }

    match stat {
        Some(s) => {
            row("stat.uptime_s", format!("{:.1}", s.uptime_s));
            row("stat.loop_hz", s.loop_hz.to_string());
            row("stat.stream_hz", s.stream_hz.to_string());
            row("stat.trim_deg", format!("{:+.2}", s.trim_deg));
            row(
                "stat.estop",
                if s.estop {
                    "LATCHED".to_owned()
                } else {
                    "clear".to_owned()
                },
            );
            row("stat.steps", s.steps.to_string());
            row("stat.imu_samples", s.imu_samples.to_string());
            row("stat.imu_errors", s.imu_errors.to_string());
            row("stat.imu_code", s.imu_code.to_string());
            row("stat.fb_frames", s.fb_frames.to_string());
            row("stat.fb_discarded", s.fb_discarded.to_string());
            row("stat.rx_bytes", opt_f32_u32(s.link_bytes));
            row("stat.fb_ok", yes_no(s.link_fresh));
            row(
                "stat.stop_reason",
                s.stop_reason.clone().unwrap_or_else(|| "--".to_owned()),
            );
            row("stat.zero_deg", opt_signed(s.zero_deg, 2));
            row("stat.zroll_deg", opt_signed(s.zroll_deg, 2));
            row("stat.level_deg", opt_signed(s.level_deg, 2));
            row(
                "stat.mount",
                s.mount
                    .map_or_else(|| "--".to_owned(), |m| format!("{} {}", m, mount_name(m))),
            );
            row("stat.armed", yes_no(s.armed));
            row("stat.home_deg", opt_signed(s.home_deg, 2));
            row(
                "stat.fall_events",
                s.fall_events
                    .map_or_else(|| "--".to_owned(), |v| v.to_string()),
            );
            row("stat.vmax_deg_s", opt_f32(s.ref_vmax_deg_s, 2));
            row("stat.kp_nm_per_rad", opt_f32(s.kp_nm_per_rad, 2));
            row("stat.kd_nm_per_rad_s", opt_f32(s.kd_nm_per_rad_s, 2));
            row("stat.kpos_per_s", opt_f32(s.position_gain_per_s, 3));
            row("stat.kv_per_s", opt_f32(s.velocity_gain_per_s, 3));
            row("stat.ki_rad_per_m_s", opt_f32(s.trim_gain_rad_per_m_s, 4));
            row("stat.dem_limit_deg", opt_f32(s.demand_limit_deg, 1));
            row("stat.trim_limit_deg", opt_f32(s.trim_limit_deg, 1));
            row("stat.zcapt", yes_no(s.zero_captured));
            row("stat.lcapt", yes_no(s.level_captured));
            row("stat.pscale", opt_f32(s.pscale, 2));
        }
        None => row("stat.*", "press Stat or enable auto-poll".to_owned()),
    }

    fn opt_f32_u32(value: Option<u32>) -> String {
        value.map_or_else(|| "--".to_owned(), |v| v.to_string())
    }
    rows
}

fn format_stamp(at: Instant, started: Instant) -> String {
    let elapsed = at.saturating_duration_since(started).as_secs_f64();
    let minutes = (elapsed / 60.0).floor() as u64;
    let seconds = elapsed - minutes as f64 * 60.0;
    format!("{minutes:02}:{seconds:06.3}")
}
