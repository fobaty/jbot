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
const HEADER: &str = "trial,t_ms,step,pitch_deg,heading_deg,gy_dps,gz_dps,ref_deg,demand_deg,torque_units,position_m,loop_hz,age_ms,estop,mode,op_mode,grace_s,arm";
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
        trial.to_string(),
        t_ms.to_string(),
        (n("step") as u64).to_string(),
        format!("{:.4}", n("pitch")),
        format!("{:.4}", n("hdg")),
        format!("{:.4}", n("gy")),
        format!("{:.4}", n("gz")),
        format!("{:.4}", n("ref")),
        format!("{:.4}", n("dem")),
        format!("{:.1}", n("trq")),
        format!("{:.4}", n("x")),
        (n("loop") as u64).to_string(),
        (n("age") as u64).to_string(),
        (n("estop") as u64).to_string(),
        (n("mode") as u64).to_string(),
        (n("opm") as u64).to_string(),
        format!("{:.3}", n("grace")),
        (n("arm") as u64).to_string(),
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
        self.estop |= num(line, "estop") > 0.5;
        // Mode 4 is the fall latch; any other nonzero op mode means the
        // balance law handed control to a different subsystem.
        self.fall_latch |= num(line, "mode") > 3.5;
        self.other_mode |= num(line, "opm") > 0.5;
        self.disarmed |= num(line, "arm") < 0.5;
    }

    fn pass(&self, baseline_pitch: f64) -> bool {
        self.samples > 20
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
) -> Option<(f64, f64)> {
    let deadline = Instant::now() + timeout;
    let mut stable_since: Option<Instant> = None;
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
                && num(&line, "age") < 300.0;
            if standing {
                candidate = (pitch, num(&line, "x"));
                if stable_since.is_none() {
                    stable_since = Some(Instant::now());
                }
                if stable_since.is_some_and(|since| since.elapsed() >= Duration::from_millis(500)) {
                    return Some(candidate);
                }
            } else {
                stable_since = None;
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

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut port = None;
    let mut trials = 3_usize;
    let mut angle = 5.0_f64;
    let mut hold_ms = 500_u64;
    let mut settle_ms = 2_000_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
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
                println!("usage: recovery_driver --run [--trials 3] [--angle 5] [--hold-ms 500] [--settle-ms 2000] [--port PATH]");
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
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        0,
        Duration::from_millis(300),
        None,
    );
    // Keep the persisted level. Re-capturing it while the operator still
    // holds the body at an angle poisoned the reference in the first trial.
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
        send(&mut serial, "estop");
        let _ = csv.flush();
        println!("aborted; recording: {}", csv_path.display());
        return;
    }

    let Some((baseline_pitch, baseline_position)) = wait_for_baseline(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_secs(8),
    ) else {
        println!("no stable upright baseline; release the body and retry");
        send(&mut serial, "estop");
        let _ = csv.flush();
        println!("aborted; recording: {}", csv_path.display());
        return;
    };
    println!("baseline pitch {baseline_pitch:.2} deg, position {baseline_position:.3} m");

    let mut passed = 0;
    for trial in 1..=trials {
        let mut stats = TrialStats {
            origin_m: baseline_position,
            ..TrialStats::default()
        };
        println!("trial {trial}/{trials}: home {angle:.1} deg for {hold_ms} ms");
        send(&mut serial, &format!("home {angle:.1}"));
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            trial,
            Duration::from_millis(hold_ms),
            Some(&mut stats),
        );
        send(&mut serial, "home");
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
            "  {} samples, pitch max {:.2} deg, last {:.2} deg, travel {:.3} m, torque peak {:.0} units, estop={} latch={} other_opm={} disarmed={}",
            stats.samples,
            stats.max_abs_pitch,
            stats.last_pitch,
            stats.max_travel_m,
            stats.peak_torque,
            u8::from(stats.estop),
            u8::from(stats.fall_latch),
            u8::from(stats.other_mode),
            u8::from(stats.disarmed),
        );
        if stats.estop || stats.fall_latch {
            println!("  guard fired; stopping the series");
            break;
        }
    }

    send(&mut serial, "estop");
    let _ = csv.flush();
    println!(
        "passed {passed}/{trials} trials; estop sent; recording: {}",
        csv_path.display()
    );
}
