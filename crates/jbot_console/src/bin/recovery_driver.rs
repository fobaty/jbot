//! Command-driven balance-recovery trials on the Pico USB CDC console.
//!
//! Arms the balance law, applies N identical pitch-reference steps with `home`,
//! returns the reference to level after each one and records the recovery.
//! Every run ends with `estop`. The robot must be secured with a rope, hands on
//! the body, wheels on the floor and the test zone at most 0.5 m.
//!
//! Usage: recovery_driver --run [--trials 3] [--angle 5] [--hold-ms 500]
//!                        [--settle-ms 2000] [--port PATH]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
// The first 25 columns match spin_driver so `jbot-analyze` can score these
// recordings with `oscillation`, `law_fit` and `breakaway` directly.
const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,test_right_units,ref_deg,demand_deg,trim_deg,position_m,mode,trial,op_mode,grace_s,arm,ff,cal_l,cal_r,cmd_l,cmd_r,dither,dither_applied";
/// Beyond this angle the attempt is a fall, not a recovery.
const PITCH_LIMIT_DEG: f64 = 25.0;
/// Allowed residual pitch error after a trial, degrees.
const SETTLED_PITCH_DEG: f64 = 3.0;
/// Allowed travel during one trial, metres.
const TRAVEL_LIMIT_M: f64 = 0.5;

fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    line.split_whitespace()
        .find_map(|token| token.strip_prefix(&format!("{key}=")))
}

fn num(line: &str, key: &str) -> f64 {
    field(line, key)
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(0.0)
}

fn telemetry_line(line: &str) -> bool {
    line.starts_with("T ") && field(line, "step").is_some() && field(line, "pitch").is_some()
}

fn row(trial: usize, line: &str, t_ms: u64) -> String {
    let n = |key| num(line, key);
    let fields = [
        t_ms.to_string(),
        (n("step") as u64).to_string(),
        (n("loop") as u64).to_string(),
        u8::from(field(line, "imu") == Some("ok")).to_string(),
        format!("{:.4}", n("pitch")),
        format!("{:.4}", n("roll")),
        format!("{:.4}", n("hdg")),
        format!("{:.4}", n("gy")),
        format!("{:.4}", n("gx")),
        format!("{:.4}", n("gz")),
        n("L").to_string(),
        n("R").to_string(),
        format!("{:.2}", n("bat")),
        (n("age") as u64).to_string(),
        (n("estop") as u64).to_string(),
        format!("{:.2}", n("vset")),
        format!("{:.2}", n("wset")),
        format!("{:.1}", n("trq")),
        format!("{:.1}", n("tsl")),
        format!("{:.1}", n("tsr")),
        format!("{:.4}", n("ref")),
        format!("{:.4}", n("dem")),
        format!("{:.4}", n("trm")),
        format!("{:.4}", n("x")),
        (n("mode") as u64).to_string(),
        trial.to_string(),
        (n("opm") as u64).to_string(),
        format!("{:.3}", n("grace")),
        (n("arm") as u64).to_string(),
        format!("{:.0}", n("ff")),
        format!("{:.0}", n("cbl")),
        format!("{:.0}", n("cbr")),
        format!("{:.1}", n("cwl")),
        format!("{:.1}", n("cwr")),
        format!("{:.0}", n("dth")),
        format!("{:.1}", n("dthv")),
    ];
    fields.join(",")
}

#[derive(Default)]
struct TrialStats {
    origin_m: f64,
    samples: usize,
    max_abs_pitch: f64,
    last_pitch: f64,
    max_travel_m: f64,
    peak_torque: f64,
    wheel_active_samples: usize,
    estop: bool,
    fall_latch: bool,
    disarmed: bool,
    other_mode: bool,
}

impl TrialStats {
    fn observe(&mut self, line: &str) {
        let origin_m = self.origin_m;
        self.samples += 1;
        let pitch = num(line, "pitch").abs();
        self.max_abs_pitch = self.max_abs_pitch.max(pitch);
        self.last_pitch = num(line, "pitch");
        self.max_travel_m = self.max_travel_m.max((num(line, "x") - origin_m).abs());
        self.peak_torque = self.peak_torque.max(num(line, "trq").abs());
        if num(line, "left_rpm").abs() > 2.0 || num(line, "right_rpm").abs() > 2.0 {
            self.wheel_active_samples += 1;
        }
        self.estop |= num(line, "estop") > 0.5;
        // Mode 4 is the fall latch; any other nonzero op mode means the
        // balance law handed control to a different subsystem.
        self.fall_latch |= num(line, "mode") > 3.5;
        self.other_mode |= num(line, "opm") > 0.5;
        self.disarmed |= num(line, "arm") < 0.5;
    }

    /// A body that never turns its wheels is resting on the rope, not balancing.
    fn externally_supported(&self) -> bool {
        self.wheel_active_samples < 20
    }

    fn pass(&self, baseline_pitch: f64) -> bool {
        self.samples > 20
            && !self.externally_supported()
            && !self.estop
            && !self.fall_latch
            && !self.disarmed
            && !self.other_mode
            && self.max_abs_pitch < PITCH_LIMIT_DEG
            && (self.last_pitch - baseline_pitch).abs() < SETTLED_PITCH_DEG
            && self.max_travel_m < TRAVEL_LIMIT_M
    }
}

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    trial: usize,
    duration: Duration,
    mut stats: Option<&mut TrialStats>,
) -> Vec<String> {
    let mut lines = Vec::new();
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline {
        let mut chunk = [0_u8; 1024];
        let read = serial.read(&mut chunk).unwrap_or_default();
        for &byte in &chunk[..read] {
            match byte {
                b'\r' => {}
                b'\n' => {
                    if let Ok(line) = std::str::from_utf8(buffer) {
                        let text = line.trim_end().to_owned();
                        if telemetry_line(&text) {
                            let t_ms = started.elapsed().as_millis() as u64;
                            let _ = writeln!(csv, "{}", row(trial, &text, t_ms));
                            if let Some(stats) = stats.as_deref_mut() {
                                stats.observe(&text);
                            }
                            lines.push(text);
                        } else {
                            lines.push(text);
                        }
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
    lines
}

/// Wait until the machine is truly standing and holding, then return its
/// pitch and position. Arming alone is not enough: an operator still holding
/// the body at an angle produces a baseline that no recovery can match.
fn wait_for_baseline(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    timeout: Duration,
    loose: bool,
) -> Option<(f64, f64)> {
    // The strict gate is the acceptance path. The loose variant is diagnostic
    // only: this drive's breakaway jolts the body past 5 dps at the instant the
    // wheels start turning, which the strict gate forbids by construction.
    let (gy_limit, wheel_hits_needed, stable_ms, x_limit) = if loose {
        (40.0, 3, 200, 0.05)
    } else {
        (5.0, 5, 500, 0.03)
    };
    let deadline = Instant::now() + timeout;
    let mut stable_since: Option<Instant> = None;
    let mut wheel_hits = 0_usize;
    let mut candidate: (f64, f64);
    while Instant::now() < deadline {
        let lines = pump(
            serial,
            buffer,
            csv,
            started,
            0,
            Duration::from_millis(200),
            None,
        );
        for line in lines {
            if !telemetry_line(&line) {
                continue;
            }
            let pitch = num(&line, "pitch");
            let mode = num(&line, "mode");
            let standing = pitch.abs() < 5.0
                && (mode == 1.0 || mode == 2.0)
                && num(&line, "arm") > 0.5
                && num(&line, "estop") < 0.5
                && num(&line, "age") < 300.0
                && num(&line, "x").abs() < x_limit
                && num(&line, "gy").abs() < gy_limit;
            if !standing {
                stable_since = None;
                wheel_hits = 0;
                continue;
            }
            // An operator still holding the body produces a perfectly quiet
            // baseline that no hold test can match. Require the wheels to
            // actually be turning, so the driver waits until the robot stands
            // on its own instead of timing the release against the console.
            if num(&line, "left_rpm").abs() > 2.0 || num(&line, "right_rpm").abs() > 2.0 {
                wheel_hits += 1;
            }
            candidate = (pitch, num(&line, "x"));
            if stable_since.is_none() {
                stable_since = Some(Instant::now());
            }
            if wheel_hits >= wheel_hits_needed
                && stable_since
                    .is_some_and(|since| since.elapsed() >= Duration::from_millis(stable_ms))
            {
                return Some(candidate);
            }
        }
    }
    None
}

/// Capture a new level only while the body is held still. A level taken
/// during operator motion becomes a pitch reference the machine cannot hold.
fn capture_level_when_still(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    timeout: Duration,
    loose: bool,
) -> bool {
    let deadline = Instant::now() + timeout;
    let level_limit = if loose { 5.0 } else { 2.0 };
    let mut window: Vec<(f64, f64)> = Vec::new();
    while Instant::now() < deadline {
        let lines = pump(
            serial,
            buffer,
            csv,
            started,
            0,
            Duration::from_millis(200),
            None,
        );
        for line in lines {
            if !telemetry_line(&line) {
                continue;
            }
            if num(&line, "arm") > 0.5 || num(&line, "estop") > 0.5 || num(&line, "age") > 300.0 {
                window.clear();
                continue;
            }
            let now = started.elapsed().as_secs_f64();
            window.retain(|(t, _)| now - *t <= 1.0);
            window.push((now, num(&line, "pitch")));
            if window.len() < 8 {
                continue;
            }
            let pitch = window.last().map(|(_, p)| *p).unwrap_or(0.0);
            let lo = window.iter().map(|(_, p)| *p).fold(f64::INFINITY, f64::min);
            let hi = window
                .iter()
                .map(|(_, p)| *p)
                .fold(f64::NEG_INFINITY, f64::max);
            if hi - lo < 0.5 {
                // The hold gate scores absolute pitch against level. Capturing
                // a tilted pose would make that criterion impossible, so only
                // accept a genuinely upright, still body.
                if pitch.abs() <= level_limit {
                    let _ = serial.write_all(b"level\n");
                    let _ = serial.flush();
                    return true;
                }
                window.clear();
            }
        }
    }
    false
}

/// Snap the pitch reference to the body's current attitude. A level captured
/// moments earlier can still differ from the attitude at `arm`, and that
/// mismatch is enough to push a free-standing machine over.
fn snap_reference_when_still(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    timeout: Duration,
) -> Option<f64> {
    let deadline = Instant::now() + timeout;
    let mut window: Vec<(f64, f64)> = Vec::new();
    while Instant::now() < deadline {
        let lines = pump(
            serial,
            buffer,
            csv,
            started,
            0,
            Duration::from_millis(200),
            None,
        );
        for line in lines {
            if !telemetry_line(&line) {
                continue;
            }
            if num(&line, "arm") > 0.5 || num(&line, "estop") > 0.5 || num(&line, "age") > 300.0 {
                window.clear();
                continue;
            }
            let now = started.elapsed().as_secs_f64();
            window.retain(|(t, _)| now - *t <= 1.0);
            window.push((now, num(&line, "pitch")));
            if window.len() < 8 {
                continue;
            }
            let lo = window.iter().map(|(_, p)| *p).fold(f64::INFINITY, f64::min);
            let hi = window
                .iter()
                .map(|(_, p)| *p)
                .fold(f64::NEG_INFINITY, f64::max);
            if hi - lo < 0.5 {
                let pitch = window.last().map(|(_, p)| *p).unwrap_or(0.0);
                if pitch.abs() <= 4.9 {
                    let _ = serial.write_all(format!("home {pitch:.2}\n").as_bytes());
                    let _ = serial.flush();
                    return Some(pitch);
                }
                return None;
            }
        }
    }
    None
}

fn pick_port(explicit: Option<String>) -> String {
    if let Some(port) = explicit {
        return port;
    }
    let ports = serialport::available_ports().unwrap_or_default();
    ports
        .iter()
        .find(|port| port.port_name.contains("JBOT_PICO"))
        .map(|port| port.port_name.clone())
        .unwrap_or_else(|| "/dev/cu.usbmodemJBOT_PICO_011".to_owned())
}

fn timestamp_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

/// End a session without parking the console task: stop the stream and wait
/// past the firmware's 1 s reply deadline before the port is closed.
fn finish(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
) {
    let _ = serial.write_all(b"estop\n");
    let _ = serial.flush();
    let _ = serial.write_all(b"stream 0\n");
    let _ = serial.flush();
    let _ = pump(
        serial,
        buffer,
        csv,
        started,
        0,
        Duration::from_millis(3_000),
        None,
    );
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut relevel = false;
    let mut snap_ref = false;
    let mut hold_only = false;
    let mut loose_baseline = false;
    let mut ff_enabled = false;
    let mut cal = None;
    let mut dither = 0.0_f64;
    let mut predict = 0.0_f64;
    let mut port = None;
    // The compiled defaults (pscale 1.0, slew 3000) are not a safe bench
    // profile: every stable armed run in the audit used pscale 0.10-0.30.
    let mut pscale = 0.30_f64;
    let mut slew = 100.0_f64;
    // A fast slew makes the initial breakaway violent enough to trip the
    // baseline gate (gyro > 5 dps while the wheels start turning). Allow a
    // separate, faster rate for the measured hold after the baseline is set.
    let mut slew_after: Option<f64> = None;
    let mut trials = 3_usize;
    let mut angle = 5.0_f64;
    let mut hold_ms = 500_u64;
    let mut settle_ms = 2_000_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--relevel" => relevel = true,
            "--snap-ref" => snap_ref = true,
            "--hold-only" => hold_only = true,
            "--loose-baseline" => loose_baseline = true,
            "--ff" => match args.next().as_deref() {
                Some("on") => ff_enabled = true,
                Some("off") => ff_enabled = false,
                _ => {}
            },
            "--cal" => {
                let left = args.next().and_then(|v| v.parse::<f64>().ok());
                let right = args.next().and_then(|v| v.parse::<f64>().ok());
                if let (Some(l), Some(r)) = (left, right) {
                    cal = Some((l, r));
                } else {
                    eprintln!("--cal needs two numbers (left right)");
                    return;
                }
            }
            "--dither" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    if value.is_finite() && value >= 0.0 {
                        dither = value.min(60.0);
                    }
                } else {
                    eprintln!("--dither needs amplitude in 0..60 units");
                    return;
                }
            }
            "--predict" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    if value.is_finite() && value >= 0.0 {
                        predict = value.min(150.0);
                    }
                }
            }
            "--port" => port = args.next(),
            "--pscale" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    if value.is_finite() && (0.0..=1.0).contains(&value) {
                        pscale = value;
                    }
                }
            }
            "--slew" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    if value.is_finite() {
                        let applied = value.clamp(100.0, 1000.0);
                        if (applied - value).abs() > 1e-6 {
                            println!("warning: slew {value:.0} clamped to {applied:.0} units/s");
                        }
                        slew = applied;
                    } else {
                        println!("warning: non-finite slew ignored, using {slew:.0}");
                    }
                }
            }
            "--slew-after" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    if value.is_finite() {
                        slew_after = Some(value.clamp(100.0, 1000.0));
                    }
                }
            }
            "--trials" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<usize>().ok()) {
                    trials = value.clamp(1, 10);
                }
            }
            "--angle" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    if value.is_finite() && (0.5..=5.0).contains(&value) {
                        angle = value;
                    }
                }
            }
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(100, 5_000);
                }
            }
            "--settle-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    settle_ms = value.clamp(500, 10_000);
                }
            }
            "--help" | "-h" => {
                println!("usage: recovery_driver --run [--snap-ref] [--hold-only] [--ff on|off] [--cal LEFT RIGHT] [--dither 0..60] [--trials 3] [--angle 5] [--hold-ms 500] [--settle-ms 2000] [--pscale 0.30] [--slew 100] [--slew-after 100] [--loose-baseline] [--predict 100] [--port PATH]");
                return;
            }
            other => {
                eprintln!("unknown option: {other}");
                return;
            }
        }
    }
    if !run {
        eprintln!("refusing to run: pass --run after securing the robot");
        return;
    }

    let path = pick_port(port);
    println!("opening {path} at {BAUD} baud");
    let mut serial = match serialport::new(&path, BAUD)
        .timeout(Duration::from_millis(20))
        .dtr_on_open(true)
        .open()
    {
        Ok(port) => port,
        Err(error) => {
            eprintln!("cannot open {path}: {error}");
            return;
        }
    };

    let csv_path = PathBuf::from(format!("csv/jbot-recovery-{}.csv", timestamp_ms()));
    if let Some(parent) = csv_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let mut csv = match File::create(&csv_path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {}: {error}", csv_path.display());
            return;
        }
    };
    let _ = writeln!(csv, "{HEADER}");

    let send = |serial: &mut Box<dyn SerialPort>, line: &str| {
        let _ = serial.write_all(format!("{line}\n").as_bytes());
        let _ = serial.flush();
    };

    let started = Instant::now();
    let mut buffer = Vec::new();
    send(&mut serial, "stream 200");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        0,
        Duration::from_millis(500),
        None,
    );
    send(&mut serial, "clear");
    println!(
        "safe bench profile: pscale {pscale:.2}, slew {slew:.0}, ff {}, cal {}, dither {dither:.0} u, predict {predict:.0} ms",
        if ff_enabled { "on" } else { "off" },
        cal.map_or_else(|| "off".to_owned(), |(l, r)| format!("{l:.0}/{r:.0}"))
    );
    send(&mut serial, if ff_enabled { "ff on" } else { "ff off" });
    send(&mut serial, &format!("pscale {pscale:.2}"));
    send(&mut serial, &format!("slew {slew:.0}"));
    match cal {
        Some((left, right)) => {
            send(&mut serial, &format!("cal {left:.0} {right:.0}"));
        }
        None => send(&mut serial, "cal off"),
    }
    if dither > 0.0 {
        send(&mut serial, &format!("dither {dither:.0}"));
    } else {
        send(&mut serial, "dither off");
    }
    if predict > 0.0 {
        send(&mut serial, &format!("predict {predict:.0}"));
    } else {
        send(&mut serial, "predict off");
    }
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        0,
        Duration::from_millis(300),
        None,
    );
    if snap_ref {
        println!("hold the body upright and still: snapping the reference to its pitch");
        if snap_reference_when_still(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_secs(12),
        )
        .is_none()
        {
            println!("pitch never settled or was outside the home range; aborting");
            finish(&mut serial, &mut buffer, &mut csv, &started);
            let _ = csv.flush();
            return;
        }
    } else if relevel {
        println!("hold the body upright and still: capturing level when pitch settles");
        if !capture_level_when_still(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_secs(12),
            loose_baseline,
        ) {
            println!("pitch never settled; aborting without arming");
            finish(&mut serial, &mut buffer, &mut csv, &started);
            let _ = csv.flush();
            return;
        }
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            0,
            Duration::from_millis(300),
            None,
        );
    }
    // Otherwise keep the persisted level: re-capturing it while the operator
    // still holds the body at an angle poisons the reference.
    send(&mut serial, "arm");
    let arm = pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        0,
        Duration::from_millis(1_000),
        None,
    );
    let armed = arm
        .iter()
        .any(|line| line.contains("arm=1") || line.starts_with("ok armed"));
    println!("armed: {armed}");
    if !armed {
        for line in arm.iter().take(4) {
            println!("  {line}");
        }
        finish(&mut serial, &mut buffer, &mut csv, &started);
        let _ = csv.flush();
        println!("aborted; recording: {}", csv_path.display());
        return;
    }

    let Some((baseline_pitch, baseline_position)) = wait_for_baseline(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_secs(15),
        loose_baseline,
    ) else {
        println!("no stable upright baseline with turning wheels; release the body and retry");
        finish(&mut serial, &mut buffer, &mut csv, &started);
        let _ = csv.flush();
        println!("aborted; recording: {}", csv_path.display());
        return;
    };
    println!("baseline pitch {baseline_pitch:.2} deg, position {baseline_position:.3} m");

    if let Some(rate) = slew_after {
        send(&mut serial, &format!("slew {rate:.0}"));
        println!("hold slew {rate:.0} units/s");
    }

    if hold_only {
        println!("hold-only: recording 10 s with no disturbance");
        let mut stats = TrialStats {
            origin_m: baseline_position,
            ..TrialStats::default()
        };
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            0,
            Duration::from_secs(10),
            Some(&mut stats),
        );
        println!(
            "hold: wheel-active {} samples, pitch max {:.2} deg, travel {:.3} m, torque peak {:.0} units, estop={} latch={} disarmed={}",
            stats.wheel_active_samples,
            stats.max_abs_pitch,
            stats.max_travel_m,
            stats.peak_torque,
            u8::from(stats.estop),
            u8::from(stats.fall_latch),
            u8::from(stats.disarmed),
        );
        finish(&mut serial, &mut buffer, &mut csv, &started);
        let _ = csv.flush();
        println!("estop sent; recording: {}", csv_path.display());
        return;
    }

    let mut passed = 0;
    for trial in 1..=trials {
        let mut stats = TrialStats {
            origin_m: baseline_position,
            ..TrialStats::default()
        };
        let forward = baseline_pitch + angle;
        let target = if forward.abs() <= 5.0 {
            forward
        } else {
            baseline_pitch - angle
        };
        let return_target = baseline_pitch;
        if target.abs() > 5.0 || return_target.abs() > 5.0 {
            println!("home targets outside the 5 degree limit; aborting");
            break;
        }
        println!("trial {trial}/{trials}: home {target:.1} deg for {hold_ms} ms");
        send(&mut serial, &format!("home {target:.2}"));
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            trial,
            Duration::from_millis(hold_ms),
            Some(&mut stats),
        );
        send(&mut serial, &format!("home {return_target:.2}"));
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            trial,
            Duration::from_millis(settle_ms),
            Some(&mut stats),
        );
        let ok = stats.pass(baseline_pitch);
        if ok {
            passed += 1;
        }
        println!(
            "  {} samples, wheel-active {} samples, pitch max {:.2} deg, last {:.2} deg, travel {:.3} m, torque peak {:.0} units, estop={} latch={} other_opm={} disarmed={}",
            stats.samples,
            stats.wheel_active_samples,
            stats.max_abs_pitch,
            stats.last_pitch,
            stats.max_travel_m,
            stats.peak_torque,
            u8::from(stats.estop),
            u8::from(stats.fall_latch),
            u8::from(stats.other_mode),
            u8::from(stats.disarmed),
        );
        if stats.estop || stats.fall_latch || stats.disarmed {
            println!("  guard fired or the law disarmed; stopping the series");
            break;
        }
    }

    finish(&mut serial, &mut buffer, &mut csv, &started);
    let _ = csv.flush();
    println!(
        "passed {passed}/{trials} trials; estop sent; recording: {}",
        csv_path.display()
    );
    println!(
        "score: cargo run -q -p jbot_analysis --bin jbot-analyze -- {}",
        csv_path.display()
    );
}
