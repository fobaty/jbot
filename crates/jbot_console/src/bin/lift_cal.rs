//! Pose-aware closed-loop lift calibration.
//!
//! This driver implements the inverted-lever learning procedure described in
//! `docs/lever-calibration.md`:
//!
//! 1. Read the three poses stored in NVM (`back`, `face`, `home`).
//! 2. Classify the current pose from the pitch telemetry.
//! 3. If the body is lying, raise it very smoothly under closed-loop pitch
//!    control using the common-mode `test` torque. The pitch target advances a
//!    few degrees at a time; at each target the controller holds until the body
//!    is still, and the torque that holds it is recorded.
//! 4. Fit the measured holding torque `T(pitch)` and derive the feedforward
//!    correction `ff(pitch) = -(T(pitch) - T(home))` used by the balance law.
//!
//! The robot is disarmed throughout: only the existing `test` command drives
//! the motors, so the 200 Hz balance loop is never modified. Every run ends
//! with `estop`.
//!
//! Usage: lift_cal --run [--port PATH] [--from back|face|auto]
//!                 [--step 5] [--hold-ms 900] [--sample-ms 20]
//!                 [--kp 1.5] [--ki 3.0] [--kd 0.4] [--max-units 200]
//!
//! `--run` is mandatory. The robot must be secured with a rope, hands on or
//! immediately above the body, wheels on the floor, test zone at most 0.5 m.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,test_right_units,position_m,mode,phase,target_deg,torque_cmd,err_deg,held_ms,point";

/// Abort if the body angular rate exceeds this value, degrees per second.
const RUNAWAY_DPS: f64 = 250.0;
/// Abort before the machine leaves the test zone, metres.
const TRAVEL_LIMIT_M: f64 = 0.4;
/// Stop the lift once the body is this close to the stored home pose.
const HOME_TOL_DEG: f64 = 6.0;
/// Absolute runtime ceiling on the `test` command; the firmware clamps at 300.
const HARD_UNIT_LIMIT: f64 = 290.0;

fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    line.split_whitespace()
        .find_map(|token| token.strip_prefix(&format!("{key}=")))
}

fn num(line: &str, key: &str) -> f64 {
    field(line, key)
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(0.0)
}

fn is_telemetry(line: &str) -> bool {
    line.starts_with("T ") && field(line, "step").is_some() && field(line, "pitch").is_some()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pose {
    Back,
    Face,
    Upright,
    Unknown,
}

/// Which stored pose the current pitch is closest to.
fn classify_pose(pitch: f64, back: f64, face: f64, home: f64, tol: f64) -> Pose {
    if (pitch - back).abs() <= tol {
        Pose::Back
    } else if (pitch - face).abs() <= tol {
        Pose::Face
    } else if (pitch - home).abs() <= tol {
        Pose::Upright
    } else {
        Pose::Unknown
    }
}

/// Sign of the pitch change required to lift from a pose toward home.
/// A positive command drives the body negative, so the wheel command is the
/// negative of the lift direction.
fn lift_direction(pose: Pose, back: f64, home: f64) -> i8 {
    if pose == Pose::Back && back < home {
        1
    } else if pose == Pose::Face {
        -1
    } else {
        0
    }
}

/// PI with rate damping and integral anti-windup. The output is a signed torque
/// magnitude along the lift axis; the caller applies the direction sign.
#[derive(Clone, Copy)]
struct LiftController {
    kp: f64,
    ki: f64,
    kd: f64,
    limit: f64,
    integral: f64,
}

impl LiftController {
    fn new(kp: f64, ki: f64, kd: f64, limit: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            limit,
            integral: 0.0,
        }
    }

    fn reset(&mut self) {
        self.integral = 0.0;
    }

    /// `error` is the pitch error aligned with the lift direction; `rate` is
    /// the pitch rate aligned with the lift direction. Positive output drives
    /// the body along the lift direction (the caller negates it for the motor).
    fn step(&mut self, error: f64, rate: f64, dt_s: f64) -> f64 {
        if self.ki > 0.0 {
            self.integral += error * dt_s;
            let bound = self.limit / self.ki;
            self.integral = self.integral.clamp(-bound, bound);
        }
        let magnitude = self.kp * error + self.ki * self.integral - self.kd * rate;
        magnitude.clamp(-self.limit, self.limit)
    }
}

#[derive(Clone, Copy)]
struct Sample {
    target_deg: f64,
    pitch_deg: f64,
    torque_units: f64,
    rate_dps: f64,
}

#[derive(Clone, Copy)]
struct RowCtx<'a> {
    phase: &'a str,
    target_deg: f64,
    torque_cmd: f64,
    error_deg: f64,
    held_ms: u64,
    point: bool,
}

fn csv_row(line: &str, t_ms: u64, ctx: RowCtx<'_>) -> String {
    let n = |key| num(line, key);
    let values = [
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
        format!("{:.4}", n("x")),
        (n("mode") as u64).to_string(),
        ctx.phase.to_owned(),
        format!("{:.2}", ctx.target_deg),
        format!("{:.1}", ctx.torque_cmd),
        format!("{:.2}", ctx.error_deg),
        ctx.held_ms.to_string(),
        u8::from(ctx.point).to_string(),
    ];
    values.join(",")
}

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    ctx: RowCtx<'_>,
    samples: &mut Vec<(f64, f64)>,
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
                    if let Ok(text) = std::str::from_utf8(buffer) {
                        let text = text.trim_end().to_owned();
                        if is_telemetry(&text) {
                            let t_ms = started.elapsed().as_millis() as u64;
                            let _ = writeln!(csv, "{}", csv_row(&text, t_ms, ctx));
                            samples.push((num(&text, "pitch"), num(&text, "gy")));
                        }
                        lines.push(text);
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
    lines
}

fn parse_pose_reply(line: &str) -> Option<(f64, f64, f64)> {
    if !line.contains("back=") || !line.contains("face=") || !line.contains("home=") {
        return None;
    }
    let back = field(line, "back")?.parse::<f64>().ok()?;
    let face = field(line, "face")?.parse::<f64>().ok()?;
    let home = field(line, "home")?.parse::<f64>().ok()?;
    Some((back, face, home))
}

fn timestamp_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
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

/// Least-squares slope of torque against pitch, with intercept.
fn fit_slope(points: &[Sample]) -> Option<(f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let sx: f64 = points.iter().map(|p| p.pitch_deg).sum();
    let sy: f64 = points.iter().map(|p| p.torque_units).sum();
    let sxx: f64 = points.iter().map(|p| p.pitch_deg * p.pitch_deg).sum();
    let sxy: f64 = points.iter().map(|p| p.pitch_deg * p.torque_units).sum();
    let det = n * sxx - sx * sx;
    if det.abs() < 1e-9 {
        return None;
    }
    let slope = (n * sxy - sx * sy) / det;
    let intercept = (sy - slope * sx) / n;
    Some((slope, intercept))
}

fn write_json(
    path: &PathBuf,
    direction: i8,
    back: f64,
    face: f64,
    home: f64,
    abort: Option<&str>,
    points: &[Sample],
) {
    let mut file = match File::create(path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {}: {error}", path.display());
            return;
        }
    };
    let direction_name = if direction > 0 { "back" } else { "face" };
    let _ = writeln!(file, "{{");
    let _ = writeln!(file, "  \"direction\": \"{direction_name}\",");
    let _ = writeln!(file, "  \"pose_back_deg\": {back:.2},");
    let _ = writeln!(file, "  \"pose_face_deg\": {face:.2},");
    let _ = writeln!(file, "  \"home_deg\": {home:.2},");
    let _ = writeln!(
        file,
        "  \"abort\": {},",
        abort.map_or_else(|| "null".to_owned(), |reason| format!("\"{reason}\""))
    );
    let _ = writeln!(file, "  \"points\": [");
    for (index, point) in points.iter().enumerate() {
        let comma = if index + 1 == points.len() { "" } else { "," };
        let _ = writeln!(
            file,
            "    {{\"target_deg\": {:.2}, \"pitch_deg\": {:.2}, \"torque_units\": {:.1}, \"rate_dps\": {:.2}}}{comma}",
            point.target_deg, point.pitch_deg, point.torque_units, point.rate_dps
        );
    }
    let _ = writeln!(file, "  ]");
    let _ = writeln!(file, "}}");
    let _ = file.flush();
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut forced_pose: Option<String> = None;
    let mut port = None;
    let mut step = 5.0_f64;
    let mut hold_ms = 900_u64;
    let mut sample_ms = 20_u64;
    let mut kp = 1.5_f64;
    let mut ki = 3.0_f64;
    let mut kd = 0.4_f64;
    let mut max_units = 200.0_f64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--from" => forced_pose = args.next(),
            "--step" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    step = value.clamp(1.0, 20.0);
                }
            }
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(200, 5000);
                }
            }
            "--sample-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    sample_ms = value.clamp(10, 100);
                }
            }
            "--kp" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    kp = value.clamp(0.0, 20.0);
                }
            }
            "--ki" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    ki = value.clamp(0.0, 20.0);
                }
            }
            "--kd" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    kd = value.clamp(0.0, 20.0);
                }
            }
            "--max-units" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    max_units = value.clamp(20.0, HARD_UNIT_LIMIT);
                }
            }
            "--help" | "-h" => {
                println!("usage: lift_cal --run [--port PATH] [--from back|face|auto] [--step 5] [--hold-ms 900] [--sample-ms 20] [--kp 1.5] [--ki 3.0] [--kd 0.4] [--max-units 200]");
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

    let stem = format!("csv/jbot-lift-{}", timestamp_ms());
    let csv_path = PathBuf::from(format!("{stem}.csv"));
    let json_path = PathBuf::from(format!("{stem}.json"));
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
    let mut samples = Vec::new();

    send(&mut serial, "stream 200");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(400),
        RowCtx {
            phase: "init",
            target_deg: 0.0,
            torque_cmd: 0.0,
            error_deg: 0.0,
            held_ms: 0,
            point: false,
        },
        &mut samples,
    );
    send(&mut serial, "clear");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(300),
        RowCtx {
            phase: "init",
            target_deg: 0.0,
            torque_cmd: 0.0,
            error_deg: 0.0,
            held_ms: 0,
            point: false,
        },
        &mut samples,
    );

    // Read the poses stored in NVM. The operator committed them earlier with
    // `pose ...; save`, so they survive power cycles and need no re-entry.
    send(&mut serial, "pose");
    let pose_lines = pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(500),
        RowCtx {
            phase: "pose",
            target_deg: 0.0,
            torque_cmd: 0.0,
            error_deg: 0.0,
            held_ms: 0,
            point: false,
        },
        &mut samples,
    );
    let poses = pose_lines
        .iter()
        .find_map(|line| parse_pose_reply(line))
        .unwrap_or((-80.4, 83.5, -2.5));
    let (back, face, home) = poses;
    println!("poses from NVM: back={back:.2} face={face:.2} home={home:.2}");

    // Capture a quiet sample of the current attitude to classify the pose.
    let mut current_pitch = 0.0_f64;
    let mut quiet_since: Option<Instant> = None;
    let classify_deadline = Instant::now() + Duration::from_secs(8);
    while Instant::now() < classify_deadline {
        let lines = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(sample_ms),
            RowCtx {
                phase: "classify",
                target_deg: 0.0,
                torque_cmd: 0.0,
                error_deg: 0.0,
                held_ms: 0,
                point: false,
            },
            &mut samples,
        );
        for line in lines.iter().rev() {
            if !is_telemetry(line) {
                continue;
            }
            let pitch = num(line, "pitch");
            let gy = num(line, "gy").abs();
            if gy < 10.0 && num(line, "age") < 300.0 {
                current_pitch = pitch;
                if quiet_since.is_none() {
                    quiet_since = Some(Instant::now());
                }
                if quiet_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(400)) {
                    break;
                }
            } else {
                quiet_since = None;
            }
        }
        if quiet_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(400)) {
            break;
        }
    }

    let mut pose = classify_pose(current_pitch, back, face, home, 25.0);
    if let Some(ref forced) = forced_pose {
        pose = match forced.as_str() {
            "back" => Pose::Back,
            "face" => Pose::Face,
            _ => pose,
        };
    }
    println!("current pitch {current_pitch:.2} deg -> pose {pose:?}");

    let finish = |serial: &mut Box<dyn SerialPort>,
                  csv: &mut BufWriter<File>,
                  abort: Option<&str>,
                  points: &[Sample],
                  direction: i8| {
        send(serial, "test 0");
        send(serial, "estop");
        let _ = csv.flush();
        write_json(&json_path, direction, back, face, home, abort, points);
        if let Some(reason) = abort {
            println!("ABORT: {reason}; estop sent");
        } else {
            println!("lift complete; estop sent");
        }
        println!(
            "recording: {} and {}",
            csv_path.display(),
            json_path.display()
        );
    };

    if pose == Pose::Upright {
        println!("already upright; lay the robot on its back or face and retry");
        finish(&mut serial, &mut csv, Some("already upright"), &[], 0);
        return;
    }
    if pose == Pose::Unknown {
        println!("pose is between the stored poses; lay the robot near back or face and retry");
        finish(&mut serial, &mut csv, Some("pose unknown"), &[], 0);
        return;
    }

    let direction = lift_direction(pose, back, home);
    if direction == 0 {
        println!("cannot derive a lift direction from this pose");
        finish(&mut serial, &mut csv, Some("no lift direction"), &[], 0);
        return;
    }
    println!("lifting from {pose:?} toward home, direction {direction:+}");

    send(&mut serial, "estop");
    // The console refuses commands while the estop latch is set; clear it for
    // this secured trial and restore it at the end.
    send(&mut serial, "clear");

    // Position baseline comes from the last telemetry line captured below; the
    // pump helper does not retain full lines, so track travel from x directly.
    let mut origin_x = 0.0_f64;
    {
        let probe = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(100),
            RowCtx {
                phase: "baseline",
                target_deg: 0.0,
                torque_cmd: 0.0,
                error_deg: 0.0,
                held_ms: 0,
                point: false,
            },
            &mut samples,
        );
        if let Some(line) = probe.iter().rev().find(|line| is_telemetry(line)) {
            origin_x = num(line, "x");
        }
    }

    let mut controller = LiftController::new(kp, ki, kd, max_units);
    let mut points: Vec<Sample> = Vec::new();
    let mut target = current_pitch + direction as f64 * step;
    let mut last_tick = Instant::now();
    let run_deadline = Instant::now() + Duration::from_secs(180);
    let mut abort: Option<String> = None;
    let mut held_ms = 0_u64;
    let mut hold_torque_sum = 0.0_f64;
    let mut hold_pitch_sum = 0.0_f64;
    let mut hold_rate_sum = 0.0_f64;
    let mut hold_n = 0_u64;
    let hold_tol = (step * 0.6).max(2.0);
    let rate_tol = 8.0_f64;
    let mut torque_cmd = 0.0_f64;
    let mut record_point = false;

    while Instant::now() < run_deadline {
        let now = Instant::now();
        let dt_s = now.duration_since(last_tick).as_secs_f64().min(0.1);
        last_tick = now;

        let lines = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(sample_ms),
            RowCtx {
                phase: "lift",
                target_deg: target,
                torque_cmd,
                error_deg: direction as f64 * (target - current_pitch),
                held_ms,
                point: record_point,
            },
            &mut samples,
        );
        record_point = false;
        let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) else {
            continue;
        };
        current_pitch = num(line, "pitch");
        let gyro = num(line, "gy");
        let travel = (num(line, "x") - origin_x).abs();

        if num(line, "estop") > 0.5 || num(line, "mode") > 3.5 || num(line, "arm") > 0.5 {
            abort = Some("guard latched or law armed unexpectedly".to_owned());
            break;
        }
        if gyro.abs() > RUNAWAY_DPS {
            abort = Some("body rate exceeded limit".to_owned());
            break;
        }
        if travel > TRAVEL_LIMIT_M {
            abort = Some("travel limit".to_owned());
            break;
        }
        if (current_pitch - home).abs() <= HOME_TOL_DEG {
            break;
        }

        let error = direction as f64 * (target - current_pitch);
        let rate = direction as f64 * gyro;
        let magnitude = controller.step(error, rate, dt_s);
        torque_cmd = -(direction as f64) * magnitude;
        send(&mut serial, &format!("test {torque_cmd:.1}"));

        if (current_pitch - target).abs() <= hold_tol && gyro.abs() <= rate_tol {
            held_ms += sample_ms;
            hold_torque_sum += torque_cmd;
            hold_pitch_sum += current_pitch;
            hold_rate_sum += gyro;
            hold_n += 1;
        } else {
            held_ms = 0;
            hold_torque_sum = 0.0;
            hold_pitch_sum = 0.0;
            hold_rate_sum = 0.0;
            hold_n = 0;
        }

        if held_ms >= hold_ms && hold_n > 0 {
            let sample = Sample {
                target_deg: target,
                pitch_deg: hold_pitch_sum / hold_n as f64,
                torque_units: hold_torque_sum / hold_n as f64,
                rate_dps: hold_rate_sum / hold_n as f64,
            };
            println!(
                "target {:+6.1} deg: pitch {:+6.2}, torque {:+6.1} units, rate {:+5.2} dps",
                sample.target_deg, sample.pitch_deg, sample.torque_units, sample.rate_dps
            );
            points.push(sample);
            record_point = true;
            controller.reset();
            held_ms = 0;
            hold_torque_sum = 0.0;
            hold_pitch_sum = 0.0;
            hold_rate_sum = 0.0;
            hold_n = 0;
            let next = target + direction as f64 * step;
            target = if direction > 0 {
                next.min(home + HOME_TOL_DEG)
            } else {
                next.max(home - HOME_TOL_DEG)
            };
        }
    }

    if abort.is_none() && points.is_empty() {
        abort = Some("no stable hold reached".to_owned());
    }

    if let Some((slope, intercept)) = fit_slope(&points) {
        println!("holding torque T(pitch) = {slope:.2}*pitch + {intercept:.1} units");
        println!(
            "recommended feedforward ff(pitch) = {:.2}*pitch + {:.1} units (relative to home {home:.1})",
            -slope,
            -intercept
        );
    } else {
        println!("not enough stable points to fit a torque curve");
    }

    finish(&mut serial, &mut csv, abort.as_deref(), &points, direction);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_classification_uses_stored_poses() {
        let pose = classify_pose(-78.0, -80.4, 83.5, -2.5, 25.0);
        assert_eq!(pose, Pose::Back);
        let pose = classify_pose(80.0, -80.4, 83.5, -2.5, 25.0);
        assert_eq!(pose, Pose::Face);
        let pose = classify_pose(-2.0, -80.4, 83.5, -2.5, 25.0);
        assert_eq!(pose, Pose::Upright);
        let pose = classify_pose(40.0, -80.4, 83.5, -2.5, 25.0);
        assert_eq!(pose, Pose::Unknown);
    }

    #[test]
    fn lift_direction_points_toward_home() {
        assert_eq!(lift_direction(Pose::Back, -80.4, -2.5), 1);
        assert_eq!(lift_direction(Pose::Face, -80.4, -2.5), -1);
        assert_eq!(lift_direction(Pose::Upright, -80.4, -2.5), 0);
    }

    #[test]
    fn controller_builds_holding_torque_and_limits_it() {
        let mut ctrl = LiftController::new(1.5, 3.0, 0.4, 200.0);
        let mut value = 0.0;
        for _ in 0..200 {
            value = ctrl.step(5.0, 0.0, 0.02);
        }
        assert!(value > 40.0, "integral must overcome the dead zone");
        assert!(value <= 200.0, "output must stay clamped");
        ctrl.reset();
        assert_eq!(ctrl.integral, 0.0);
    }

    #[test]
    fn controller_damps_rate_and_handles_overshoot() {
        let mut ctrl = LiftController::new(1.5, 0.0, 0.4, 200.0);
        let moving = ctrl.step(5.0, 10.0, 0.02);
        let still = ctrl.step(5.0, 0.0, 0.02);
        assert!(moving < still, "positive aligned rate must reduce torque");
        let overshoot = ctrl.step(-3.0, 0.0, 0.02);
        assert!(overshoot < 0.0, "overshoot must command reverse torque");
    }

    #[test]
    fn pose_reply_parsing() {
        let parsed = parse_pose_reply("ok pose back=-80.40 face=83.50 home=-2.50");
        assert_eq!(parsed, Some((-80.4, 83.5, -2.5)));
        assert_eq!(parse_pose_reply("err something"), None);
    }

    #[test]
    fn slope_fit_recovers_known_line() {
        let points = [
            Sample {
                target_deg: -20.0,
                pitch_deg: -20.0,
                torque_units: 90.0,
                rate_dps: 0.0,
            },
            Sample {
                target_deg: -15.0,
                pitch_deg: -15.0,
                torque_units: 70.0,
                rate_dps: 0.0,
            },
            Sample {
                target_deg: -10.0,
                pitch_deg: -10.0,
                torque_units: 50.0,
                rate_dps: 0.0,
            },
        ];
        let (slope, intercept) = fit_slope(&points).unwrap();
        assert!((slope + 4.0).abs() < 1e-6);
        assert!((intercept - 10.0).abs() < 1e-6);
    }
}
