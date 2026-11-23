//! Programmatic identification of the inverted-lever torque curve.
//!
//! The robot stands on its wheels. A command-driven torque ladder drives the
//! body away from vertical in small steps. For every step the driver records
//! the pitch the body settles at, then fits
//!
//!     tau = A * sin(pitch) + B * sign(tau) + C
//!
//! where `A` is the gravity lever (m*g*L), `B` is the loaded dead zone and `C`
//! is a static bias. The result is a feedforward table and suggested pitch
//! poses for back, face and home.
//!
//! Usage: lever_cal --run [--direction back|face] [--max-units 120]
//!                  [--step 10] [--hold-ms 250] [--settle-ms 250]
//!                  [--commit-poses]
//!
//! `--run` is mandatory. `--commit-poses` writes the learned poses to NVM;
//! without it the run is a dry run that only reports suggestions.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,test_right_units,ref_deg,demand_deg,trim_deg,position_m,mode,phase,level,commit";
/// Beyond this tilt the body is considered to have reached a lying pose.
const POSE_PITCH_DEG: f64 = 65.0;
/// Abort before the machine leaves the test zone.
const TRAVEL_LIMIT_M: f64 = 0.4;
/// Command units per newton-metre, measured in WP0.
const UNITS_PER_NM: f64 = 175.0;

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

fn csv_row(line: &str, t_ms: u64, phase: &str, level: f64, commit: bool) -> String {
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
        format!("{:.4}", n("ref")),
        format!("{:.4}", n("dem")),
        format!("{:.4}", n("trm")),
        format!("{:.4}", n("x")),
        (n("mode") as u64).to_string(),
        phase.to_owned(),
        format!("{:.0}", level),
        u8::from(commit).to_string(),
    ];
    values.join(",")
}

#[derive(Clone, Copy, Default)]
struct Point {
    level: f64,
    pitch_rad: f64,
    rate_dps: f64,
    position_m: f64,
}

#[derive(Clone, Copy)]
struct Record<'a> {
    phase: &'a str,
    level: f64,
    commit: bool,
}

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    record: Record<'_>,
    points: &mut Vec<Point>,
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
                            let _ = writeln!(
                                csv,
                                "{}",
                                csv_row(&text, t_ms, record.phase, record.level, record.commit)
                            );
                            points.push(Point {
                                level: record.level,
                                pitch_rad: num(&text, "pitch").to_radians(),
                                rate_dps: num(&text, "gy"),
                                position_m: num(&text, "x"),
                            });
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

/// Solve a small dense system by Gaussian elimination with partial pivoting.
fn solve(mut a: [[f64; 4]; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let mut pivot = col;
        for row in col + 1..3 {
            if a[row][col].abs() > a[pivot][col].abs() {
                pivot = row;
            }
        }
        if a[pivot][col].abs() < 1e-9 {
            return None;
        }
        a.swap(col, pivot);
        let scale = a[col][col];
        for item in a[col].iter_mut() {
            *item /= scale;
        }
        for row in 0..3 {
            if row == col {
                continue;
            }
            let factor = a[row][col];
            let pivot_row = a[col];
            for k in 0..4 {
                a[row][k] -= factor * pivot_row[k];
            }
        }
    }
    Some([a[0][3], a[1][3], a[2][3]])
}

/// Fit tau = A*sin(pitch) + B*sign(tau) + C by least squares.
fn fit(points: &[Point]) -> Option<[f64; 3]> {
    let mut m = [[0.0_f64; 4]; 3];
    for p in points {
        let x = [p.pitch_rad.sin(), p.level.signum(), 1.0];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += x[i] * x[j];
            }
            m[i][3] += x[i] * p.level;
        }
    }
    if let Some(coeffs) = solve(m) {
        return Some(coeffs);
    }
    // A single-direction run has a constant sign column, so B and C are not
    // separable. Fall back to tau = A*sin(pitch) + D.
    let mut s = 0.0_f64;
    let mut ss = 0.0_f64;
    let mut t = 0.0_f64;
    let mut st = 0.0_f64;
    for p in points {
        let x = p.pitch_rad.sin();
        s += x;
        ss += x * x;
        t += p.level;
        st += x * p.level;
    }
    let det = points.len() as f64 * ss - s * s;
    if det.abs() < 1e-9 {
        return None;
    }
    let a = (points.len() as f64 * st - s * t) / det;
    let d = (ss * t - s * st) / det;
    Some([a, 0.0, d])
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

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut commit_poses = false;
    let mut direction = 1.0_f64;
    let mut port = None;
    let mut max_units = 120.0_f64;
    let mut step = 10.0_f64;
    let mut hold_ms = 250_u64;
    let mut settle_ms = 250_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--commit-poses" => commit_poses = true,
            "--port" => port = args.next(),
            "--direction" => match args.next().as_deref() {
                Some("face") => direction = -1.0,
                _ => direction = 1.0,
            },
            "--max-units" => {
                if let Some(v) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    max_units = v.clamp(10.0, 300.0);
                }
            }
            "--step" => {
                if let Some(v) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    step = v.clamp(5.0, 50.0);
                }
            }
            "--hold-ms" => {
                if let Some(v) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = v.clamp(80, 1000);
                }
            }
            "--settle-ms" => {
                if let Some(v) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    settle_ms = v.clamp(80, 2000);
                }
            }
            "--help" | "-h" => {
                println!("usage: lever_cal --run [--direction back|face] [--max-units 120] [--step 10] [--hold-ms 250] [--settle-ms 250] [--commit-poses]");
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

    let csv_path = PathBuf::from(format!("csv/jbot-lever-{}.csv", timestamp_ms()));
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
    let mut points = Vec::new();
    send(&mut serial, "stream 200");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(400),
        Record {
            phase: "init",
            level: 0.0,
            commit: false,
        },
        &mut points,
    );
    send(&mut serial, "clear");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(300),
        Record {
            phase: "init",
            level: 0.0,
            commit: false,
        },
        &mut points,
    );

    // Wait for a quiet, upright, disarmed machine before applying torque.
    let mut home_pitch = 0.0_f64;
    let upright_deadline = Instant::now() + Duration::from_secs(10);
    let mut upright_since: Option<Instant> = None;
    while Instant::now() < upright_deadline {
        let lines = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(200),
            Record {
                phase: "upright",
                level: 0.0,
                commit: false,
            },
            &mut points,
        );
        for line in lines {
            if !is_telemetry(&line) {
                continue;
            }
            let pitch = num(&line, "pitch");
            let ok = pitch.abs() < 12.0
                && num(&line, "gy").abs() < 5.0
                && num(&line, "arm") < 0.5
                && num(&line, "estop") < 0.5
                && num(&line, "age") < 300.0;
            if ok {
                home_pitch = pitch;
                if upright_since.is_none() {
                    upright_since = Some(Instant::now());
                }
                if upright_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500)) {
                    break;
                }
            } else {
                upright_since = None;
            }
        }
        if upright_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500)) {
            break;
        }
    }
    if !upright_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500)) {
        println!("no quiet upright baseline; hold the robot upright and retry");
        send(&mut serial, "estop");
        let _ = csv.flush();
        return;
    }
    println!("home pitch {home_pitch:.2} deg, direction {direction:+.0}");
    println!(
        "RELEASE the body now; keep hands ready to catch, rope taut but slack enough to pitch"
    );
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(500),
        Record {
            phase: "release",
            level: 0.0,
            commit: false,
        },
        &mut points,
    );

    let mut limit_reason = "level range exhausted";
    let mut curve: Vec<Point> = Vec::new();
    let mut counter = step;
    while counter <= max_units + 1e-6 {
        let level = direction * counter;
        let before = points.len();
        send(&mut serial, &format!("test {level:.0}"));
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(hold_ms),
            Record {
                phase: "hold",
                level,
                commit: false,
            },
            &mut points,
        );
        send(&mut serial, "test 0");
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(settle_ms),
            Record {
                phase: "settle",
                level,
                commit: false,
            },
            &mut points,
        );
        let window: Vec<Point> = points[before..].to_vec();
        let steady: Vec<Point> = window
            .iter()
            .copied()
            .filter(|p| p.rate_dps.abs() < 30.0)
            .collect();
        let pitch = if steady.is_empty() {
            window
                .last()
                .map(|p| p.pitch_rad.to_degrees())
                .unwrap_or(0.0)
        } else {
            steady.iter().map(|p| p.pitch_rad.to_degrees()).sum::<f64>() / steady.len() as f64
        };
        let travel = window
            .iter()
            .map(|p| p.position_m.abs())
            .fold(0.0_f64, f64::max);
        println!("level {level:+.0}: pitch {pitch:+7.2} deg, travel {travel:.3} m");
        curve.push(Point {
            level,
            pitch_rad: pitch.to_radians(),
            rate_dps: 0.0,
            position_m: travel,
        });
        counter += step;
        if pitch.abs() >= POSE_PITCH_DEG {
            limit_reason = "lying pose reached";
            break;
        }
        if travel >= TRAVEL_LIMIT_M {
            limit_reason = "travel limit";
            break;
        }
        if window.iter().any(|p| p.rate_dps.abs() > 400.0) {
            limit_reason = "body ran away";
            break;
        }
    }

    // Quasi-static samples are the ones that carry information about the
    // gravity lever and the dead zone.
    let coeffs = fit(&curve);
    let learned_pose = points
        .iter()
        .map(|p| p.pitch_rad.to_degrees())
        .fold(0.0_f64, |acc, p| if p.abs() > acc.abs() { p } else { acc });

    if let Some([a, b, c]) = coeffs {
        if b == 0.0 {
            println!("fit: tau = {a:.1}*sin(pitch) + {c:.1} (one direction)");
            println!(
                "gravity lever {:.2} Nm, offset incl. dead zone {:.2} Nm",
                a / UNITS_PER_NM,
                c / UNITS_PER_NM
            );
            println!("run the opposite direction to separate dead zone from bias");
        } else {
            println!("fit: tau = {a:.1}*sin(pitch) + {b:.1}*sign(tau) + {c:.1}");
            println!(
                "gravity lever {:.2} Nm, dead zone {:.2} Nm, bias {:.2} Nm",
                a / UNITS_PER_NM,
                b / UNITS_PER_NM,
                c / UNITS_PER_NM
            );
        }
        println!("feedforward table (deg -> units):");
        for deg in [2.0_f64, 4.0, 6.0, 8.0, 10.0, 12.0] {
            let theta = deg.to_radians();
            let tau = a * theta.sin() + b * direction + c;
            println!("  {deg:4.0} -> {tau:7.1}");
        }
    } else {
        println!("fit failed: not enough quasi-static samples");
    }
    println!("{limit_reason}; learned pose pitch {learned_pose:.2} deg");

    if commit_poses {
        let pose = if direction > 0.0 { "back" } else { "face" };
        send(&mut serial, &format!("pose {pose} {learned_pose:.1}"));
        send(&mut serial, &format!("pose home {home_pitch:.1}"));
        send(&mut serial, "save");
        println!("committed pose {pose} and home to NVM");
    } else {
        println!(
            "dry run: would commit pose {} {:.1} and home {:.1}",
            if direction > 0.0 { "back" } else { "face" },
            learned_pose,
            home_pitch
        );
    }

    send(&mut serial, "estop");
    let _ = csv.flush();
    println!("estop sent; recording: {}", csv_path.display());
}
