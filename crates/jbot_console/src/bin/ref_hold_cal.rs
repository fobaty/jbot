//! Reference-hold identification of the balance feedforward curve.
//!
//! A wheeled inverted pendulum can only regulate pitch while its wheels touch
//! the floor. Lying on the back or face removes that contact, so motor torque
//! cannot hold an intermediate angle; it only rocks the shell. The operating
//! range for maneuver corrections is the small band around upright. This
//! driver measures the holding torque there directly:
//!
//! 1. Arm the balance law on the safe bench profile (`pscale 0.25`, `slew 100`,
//!    `ff off`) so the raw law torque is observed without the correction.
//! 2. Step the `home` reference through a sequence of small angles.
//! 3. At each angle wait for the body to settle, then record the mean applied
//!    balance torque `trq` from telemetry.
//! 4. Fit `T(pitch)` and report `ff(pitch) = T(pitch)` as the feedforward that
//!    offloads the integral term the law currently uses to hold these angles.
//!
//! Every run ends with `home 0` and `estop`. The robot must be secured with a
//! rope, hands on the body, wheels on the floor, test zone at most 0.5 m.
//!
//! Usage: ref_hold_cal --run [--port PATH] [--pscale 0.25] [--slew 100]
//!                     [--angles -5,-4,-3,-2,-1,0,1,2,3,4,5]
//!                     [--settle-ms 1500] [--hold-ms 800] [--tol 1.0]
//!                     [--torque-source applied|requested] [--db 0.25]
//!                     [--cal LEFT RIGHT] [--dither 30]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,test_right_units,cmd_left_units,cmd_right_units,applied_left_units,applied_right_units,ref_deg,demand_deg,trim_deg,position_m,mode,phase,angle_cmd,hold_torque,settled";

/// Latch or guard state in the `mode` telemetry field.
const MODE_LIMIT: f64 = 3.5;
/// Abort above this body rate during a reference transition, degrees per
/// second. Settled holds still require a rate below 8 dps.
const RUNAWAY_DPS: f64 = 250.0;
/// Abort before the machine leaves the test zone, metres.
const TRAVEL_LIMIT_M: f64 = 0.4;
/// Mirror `hold_sweep`: a lean with motionless wheels is an external support.
const SUPPORT_PITCH_DEG: f64 = 0.8;
const SUPPORT_STILL_SAMPLES: usize = 40;

/// A leaning body whose wheels are not turning is being held from outside.
fn supported_sample(pitch: f64, level: f64, left_rpm: f64, right_rpm: f64) -> bool {
    left_rpm.abs() <= 2.0 && right_rpm.abs() <= 2.0 && (pitch - level).abs() > SUPPORT_PITCH_DEG
}

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
enum Settlement {
    Pending,
    Settled,
    Failed,
}

#[derive(Clone, Copy)]
struct RowCtx<'a> {
    phase: &'a str,
    angle_cmd: f64,
    hold_torque: f64,
    settled: bool,
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
        format!("{:.1}", n("cwl")),
        format!("{:.1}", n("cwr")),
        n("al").to_string(),
        n("ar").to_string(),
        format!("{:.4}", n("ref")),
        format!("{:.4}", n("dem")),
        format!("{:.4}", n("trm")),
        format!("{:.4}", n("x")),
        (n("mode") as u64).to_string(),
        ctx.phase.to_owned(),
        format!("{:.2}", ctx.angle_cmd),
        format!("{:.1}", ctx.hold_torque),
        u8::from(ctx.settled).to_string(),
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

fn parse_angles(text: &str) -> Vec<f64> {
    text.split(',')
        .filter_map(|part| part.trim().parse::<f64>().ok())
        .filter(|value| value.abs() <= 5.0)
        .collect()
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TorqueSource {
    Requested,
    Applied,
}

impl TorqueSource {
    fn parse(text: &str) -> Option<Self> {
        match text {
            "applied" => Some(Self::Applied),
            "requested" => Some(Self::Requested),
            _ => None,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::Requested => "requested",
            Self::Applied => "applied",
        }
    }
}

#[derive(Clone, Copy)]
struct Hold {
    angle: f64,
    pitch: f64,
    torque: f64,
    applied: f64,
}

/// Common-mode holding torque in command units. The mainboard reports the
/// commands it actually applied per wheel; their mean cancels the yaw
/// differential and is the torque the body experienced.
fn hold_torque(line: &str, source: TorqueSource) -> f64 {
    match source {
        TorqueSource::Requested => num(line, "trq"),
        TorqueSource::Applied => (num(line, "al") + num(line, "ar")) / 2.0,
    }
}

fn hold_value(point: &Hold, source: TorqueSource) -> f64 {
    match source {
        TorqueSource::Requested => point.torque,
        TorqueSource::Applied => point.applied,
    }
}

/// Least-squares slope of torque against settled pitch, with intercept.
fn fit_slope(points: &[Hold], source: TorqueSource) -> Option<(f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let sx: f64 = points.iter().map(|p| p.pitch).sum();
    let sy: f64 = points.iter().map(|p| hold_value(p, source)).sum();
    let sxx: f64 = points.iter().map(|p| p.pitch * p.pitch).sum();
    let sxy: f64 = points.iter().map(|p| p.pitch * hold_value(p, source)).sum();
    let det = n * sxx - sx * sx;
    if det.abs() < 1e-9 {
        return None;
    }
    let slope = (n * sxy - sx * sy) / det;
    let intercept = (sy - slope * sx) / n;
    Some((slope, intercept))
}

/// A body is settled when pitch barely moves and the rate is low for a full
/// window. `pitch_span` is the peak-to-peak pitch over the window.
fn settlement(
    pitch: f64,
    expected: f64,
    pitch_span: f64,
    rate: f64,
    window_complete: bool,
    tol: f64,
    failed: bool,
) -> Settlement {
    if failed {
        Settlement::Failed
    } else if window_complete
        && (pitch - expected).abs() <= tol
        && pitch_span <= tol
        && rate.abs() <= 8.0
    {
        Settlement::Settled
    } else {
        Settlement::Pending
    }
}

#[derive(Clone, Copy)]
struct Profile {
    pscale: f64,
    slew: f64,
    db: f64,
    cal_l: f64,
    cal_r: f64,
    dither: f64,
    source: TorqueSource,
}

fn write_json(path: &PathBuf, profile: &Profile, points: &[Hold], abort: Option<&str>) {
    let mut file = match File::create(path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {}: {error}", path.display());
            return;
        }
    };
    let _ = writeln!(file, "{{");
    let _ = writeln!(file, "  \"pscale\": {:.2},", profile.pscale);
    let _ = writeln!(file, "  \"slew\": {:.0},", profile.slew);
    let _ = writeln!(file, "  \"db\": {:.2},", profile.db);
    let _ = writeln!(file, "  \"cal_left_units\": {:.0},", profile.cal_l);
    let _ = writeln!(file, "  \"cal_right_units\": {:.0},", profile.cal_r);
    let _ = writeln!(file, "  \"dither_units\": {:.0},", profile.dither);
    let _ = writeln!(
        file,
        "  \"torque_source\": \"{}\",",
        profile.source.as_str()
    );
    let _ = writeln!(
        file,
        "  \"abort\": {},",
        abort.map_or_else(|| "null".to_owned(), |reason| format!("\"{reason}\""))
    );
    let _ = writeln!(file, "  \"holds\": [");
    for (index, point) in points.iter().enumerate() {
        let comma = if index + 1 == points.len() { "" } else { "," };
        let _ = writeln!(
            file,
            "    {{\"angle_cmd\": {:.2}, \"pitch_deg\": {:.3}, \"torque_units\": {:.2}, \"applied_units\": {:.2}}}{comma}",
            point.angle, point.pitch, point.torque, point.applied
        );
    }
    let _ = writeln!(file, "  ]");
    let _ = writeln!(file, "}}");
    let _ = file.flush();
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut port = None;
    let mut pscale = 0.25_f64;
    let mut slew = 100.0_f64;
    let mut angles_text = "-5,-4,-3,-2,-1,0,1,2,3,4,5".to_owned();
    let mut settle_ms = 1500_u64;
    let mut hold_ms = 800_u64;
    let mut tol = 1.0_f64;
    let mut vmax = 2.0_f64;
    let mut db = 0.0_f64;
    let mut cal_l = 0.0_f64;
    let mut cal_r = 0.0_f64;
    let mut dither = 0.0_f64;
    let mut source = TorqueSource::Applied;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--pscale" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    pscale = value.clamp(0.1, 0.5);
                }
            }
            "--slew" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    slew = value.clamp(20.0, 3000.0);
                }
            }
            "--angles" => {
                if let Some(value) = args.next() {
                    angles_text = value;
                }
            }
            "--settle-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    settle_ms = value.clamp(500, 6000);
                }
            }
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(200, 3000);
                }
            }
            "--tol" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    tol = value.clamp(0.2, 3.0);
                }
            }
            "--vmax" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    vmax = value.clamp(0.2, 60.0);
                }
            }
            "--db" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    db = value.clamp(0.0, 0.6);
                }
            }
            "--cal" => {
                let left = args.next().and_then(|v| v.parse::<f64>().ok());
                let right = args.next().and_then(|v| v.parse::<f64>().ok());
                match (left, right) {
                    (Some(left), Some(right)) => {
                        cal_l = left.clamp(0.0, 150.0);
                        cal_r = right.clamp(0.0, 150.0);
                    }
                    _ => {
                        eprintln!("--cal needs two values: left right");
                        return;
                    }
                }
            }
            "--dither" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    dither = value.clamp(0.0, 60.0);
                }
            }
            "--torque-source" => {
                let Some(value) = args.next().and_then(|v| TorqueSource::parse(&v)) else {
                    eprintln!("--torque-source needs applied or requested");
                    return;
                };
                source = value;
            }
            "--help" | "-h" => {
                println!("usage: ref_hold_cal --run [--port PATH] [--pscale 0.25] [--slew 100] [--angles -5,-4,-3,-2,-1,0,1,2,3,4,5] [--settle-ms 1500] [--hold-ms 800] [--tol 1.0] [--torque-source applied|requested] [--db 0.25] [--cal LEFT RIGHT] [--dither 30]");
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

    let angles = parse_angles(&angles_text);
    if angles.is_empty() {
        eprintln!("no valid angles in -5..5 range");
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

    let stem = format!("csv/jbot-refhold-{}", timestamp_ms());
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
    send(&mut serial, "stream 200");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(400),
        RowCtx {
            phase: "init",
            angle_cmd: 0.0,
            hold_torque: 0.0,
            settled: false,
        },
    );
    send(&mut serial, "clear");

    // Configure a known, safe, reproducible profile. Feedforward is off so the
    // recorded torque is the raw law, not the correction we want to derive.
    send(&mut serial, "ff off");
    send(&mut serial, &format!("pscale {pscale:.2}"));
    send(&mut serial, &format!("slew {slew:.0}"));
    send(&mut serial, &format!("db {db:.2}"));
    if cal_l > 0.0 || cal_r > 0.0 {
        send(&mut serial, &format!("cal {:.0} {:.0}", cal_l, cal_r));
    } else {
        send(&mut serial, "cal off");
    }
    if dither > 0.0 {
        send(&mut serial, &format!("dither {dither:.0}"));
    } else {
        send(&mut serial, "dither off");
    }
    // A slow reference ramp is what makes the deviation smooth enough for a
    // weak bench profile to follow without exciting the pendulum.
    send(&mut serial, &format!("vmax {vmax:.2}"));
    println!(
        "safe profile: pscale {pscale:.2}, slew {slew:.0}, db {db:.2}, cal {cal_l:.0}/{cal_r:.0}, dither {dither:.0}, vmax {vmax:.2} deg/s, ff off"
    );

    // A reflash can leave the level reference uncaptured, which makes `arm`
    // refuse silently. Capture it from the current upright pose first.
    send(&mut serial, "level");
    send(&mut serial, "arm");

    let mut armed = false;
    let mut origin_x = 0.0_f64;
    let mut upright_pitch = 0.0_f64;
    let mut upright_level = 0.0_f64;
    let mut still_run = 0_usize;
    let mut max_still_run = 0_usize;
    let mut quiet = false;
    for _ in 0..12 {
        let probe = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(100),
            RowCtx {
                phase: "baseline",
                angle_cmd: 0.0,
                hold_torque: 0.0,
                settled: false,
            },
        );
        for line in probe.iter().filter(|line| is_telemetry(line)) {
            let pitch = num(line, "pitch");
            let level = num(line, "lvl");
            origin_x = num(line, "x");
            upright_pitch = pitch;
            upright_level = level;
            armed = num(line, "arm") > 0.5;
            if supported_sample(pitch, level, num(line, "L"), num(line, "R")) {
                still_run += 1;
                max_still_run = max_still_run.max(still_run);
            } else {
                still_run = 0;
            }
            quiet = (pitch - level).abs() <= tol && num(line, "gy").abs() <= 8.0;
        }
        if armed && quiet && max_still_run <= SUPPORT_STILL_SAMPLES {
            break;
        }
    }
    let baseline_reason = if !armed {
        Some("arm refused; check zero/level capture, link and e-stop".to_owned())
    } else if !quiet {
        Some(format!(
            "level capture failed: pitch {upright_pitch:.2}, level {upright_level:.2}"
        ))
    } else if max_still_run > SUPPORT_STILL_SAMPLES {
        Some(format!(
            "externally supported baseline: lean {:.2} deg with still wheels",
            upright_pitch - upright_level
        ))
    } else if upright_pitch.abs() > 15.0 {
        Some(format!("not upright at start (pitch {upright_pitch:.1})"))
    } else {
        None
    };
    if let Some(reason) = &baseline_reason {
        println!("{reason}; stand the robot on its wheels and retry");
        send(&mut serial, "home 0");
        send(&mut serial, "estop");
        send(&mut serial, "stream 0");
        let _ = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(1_500),
            RowCtx {
                phase: "close",
                angle_cmd: 0.0,
                hold_torque: 0.0,
                settled: false,
            },
        );
        let _ = csv.flush();
        write_json(
            &json_path,
            &Profile {
                pscale,
                slew,
                db,
                cal_l,
                cal_r,
                dither,
                source,
            },
            &[],
            Some(reason.as_str()),
        );
        return;
    }
    println!("upright baseline pitch {upright_pitch:.2} deg, level {upright_level:.2} deg");

    let mut points: Vec<Hold> = Vec::new();
    let mut abort: Option<String> = None;
    let mut last_pitch = 0.0_f64;

    for angle in &angles {
        let angle = *angle;
        send(&mut serial, &format!("home {angle:.1}"));
        let ramp_ms = (angle.abs() / vmax * 1000.0) as u64;
        let angle_deadline =
            Instant::now() + Duration::from_millis(ramp_ms + settle_ms + hold_ms + 5000);
        let mut window_start = Instant::now();
        let mut window_min = f64::INFINITY;
        let mut window_max = f64::NEG_INFINITY;
        let mut state = Settlement::Pending;
        let mut hold_sum = 0.0_f64;
        let mut hold_applied_sum = 0.0_f64;
        let mut hold_n = 0_u64;

        while Instant::now() < angle_deadline {
            let lines = pump(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(40),
                RowCtx {
                    phase: "angle",
                    angle_cmd: angle,
                    hold_torque: if state == Settlement::Settled {
                        let sum = match source {
                            TorqueSource::Requested => hold_sum,
                            TorqueSource::Applied => hold_applied_sum,
                        };
                        sum / hold_n.max(1) as f64
                    } else {
                        0.0
                    },
                    settled: state == Settlement::Settled,
                },
            );
            let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) else {
                continue;
            };
            let pitch = num(line, "pitch");
            let rate = num(line, "gy");
            // Applied words describe the last mainboard frame. They must not be
            // fitted from stale feedback when the law is still running.
            let feedback_stale = source == TorqueSource::Applied
                && (field(line, "mb") != Some("ok") || num(line, "age") > 300.0);
            let guard = num(line, "estop") > 0.5
                || num(line, "mode") > MODE_LIMIT
                || num(line, "arm") < 0.5
                || rate.abs() > RUNAWAY_DPS
                || (num(line, "x") - origin_x).abs() > TRAVEL_LIMIT_M
                || feedback_stale;
            if guard {
                abort = Some(format!(
                    "guard at angle {angle:+.1}: pitch {pitch:.1}, rate {rate:.1}"
                ));
                break;
            }
            last_pitch = pitch;
            window_min = window_min.min(pitch);
            window_max = window_max.max(pitch);

            match state {
                Settlement::Pending => {
                    // `home` is an offset from the captured level, so the
                    // expected settled pitch in the zeroed frame is level +
                    // angle. Requiring the body to actually reach it prevents
                    // a slow ramp from being mistaken for a settled hold.
                    let expected = num(line, "lvl") + angle;
                    state = settlement(
                        pitch,
                        expected,
                        window_max - window_min,
                        rate,
                        window_start.elapsed() >= Duration::from_millis(settle_ms),
                        tol,
                        false,
                    );
                    if state == Settlement::Settled {
                        println!(
                            "angle {:+.1}: settled at pitch {:+.2}, recording torque",
                            angle, pitch
                        );
                        window_start = Instant::now();
                        hold_sum = 0.0;
                        hold_applied_sum = 0.0;
                        hold_n = 0;
                    }
                }
                Settlement::Settled => {
                    hold_sum += num(line, "trq");
                    hold_applied_sum += hold_torque(line, TorqueSource::Applied);
                    hold_n += 1;
                    if window_start.elapsed() >= Duration::from_millis(hold_ms) {
                        let torque = hold_sum / hold_n.max(1) as f64;
                        let applied = hold_applied_sum / hold_n.max(1) as f64;
                        let measured = if source == TorqueSource::Applied {
                            applied
                        } else {
                            torque
                        };
                        println!(
                            "angle {:+.1}: pitch {:+.2}, {} torque {:+.1} units (requested {:+.1})",
                            angle,
                            pitch,
                            source.as_str(),
                            measured,
                            torque
                        );
                        points.push(Hold {
                            angle,
                            pitch,
                            torque,
                            applied,
                        });
                        break;
                    }
                }
                Settlement::Failed => break,
            }
        }
        if let Some(reason) = &abort {
            println!("{reason}");
            break;
        }
        if state != Settlement::Settled {
            println!(
                "angle {:+.1}: no stable hold (pitch {:+.2}), skipping",
                angle, last_pitch
            );
        }
    }

    if abort.is_none() && points.len() < 3 {
        abort = Some("fewer than three stable holds".to_owned());
    }

    if let Some((slope, intercept)) = fit_slope(&points, source) {
        println!(
            "measured holding torque from {}: T(pitch) = {slope:.2}*pitch + {intercept:.1} units",
            source.as_str()
        );
        println!("feedforward to apply: ff(pitch) = {slope:.2}*pitch + {intercept:.1} units");
        println!("current firmware uses +3.23 units/deg relative to home -2.5 deg");
    } else {
        println!("not enough stable holds to fit a torque curve");
    }

    send(&mut serial, "home 0");
    send(&mut serial, "estop");
    // Stop and drain the stream before closing, otherwise the console task
    // parks on a pending write for the next host.
    send(&mut serial, "stream 0");
    let _ = pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(1_500),
        RowCtx {
            phase: "done",
            angle_cmd: 0.0,
            hold_torque: 0.0,
            settled: false,
        },
    );
    let _ = csv.flush();
    write_json(
        &json_path,
        &Profile {
            pscale,
            slew,
            db,
            cal_l,
            cal_r,
            dither,
            source,
        },
        &points,
        abort.as_deref(),
    );
    if let Some(reason) = &abort {
        println!("ABORT: {reason}; home 0 and estop sent");
    } else {
        println!("calibration complete; home 0 and estop sent");
    }
    println!(
        "recording: {} and {}",
        csv_path.display(),
        json_path.display()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_parser_bounds_and_filters() {
        let parsed = parse_angles("-5,-2,0,2,5,9,abc");
        assert_eq!(parsed, vec![-5.0, -2.0, 0.0, 2.0, 5.0]);
    }

    #[test]
    fn settlement_needs_stable_window() {
        assert_eq!(
            settlement(0.0, 0.0, 2.0, 1.0, true, 1.0, false),
            Settlement::Pending
        );
        assert_eq!(
            settlement(2.0, 0.0, 0.5, 5.0, true, 1.0, false),
            Settlement::Pending
        );
        assert_eq!(
            settlement(0.0, 0.0, 0.5, 5.0, true, 1.0, false),
            Settlement::Settled
        );
        assert_eq!(
            settlement(0.0, 0.0, 0.5, 5.0, false, 1.0, false),
            Settlement::Pending
        );
        assert_eq!(
            settlement(0.0, 0.0, 0.5, 5.0, true, 1.0, true),
            Settlement::Failed
        );
    }

    #[test]
    fn slope_fit_recovers_known_line() {
        let points = [
            Hold {
                angle: -2.0,
                pitch: -2.0,
                torque: -8.0,
                applied: -7.0,
            },
            Hold {
                angle: 0.0,
                pitch: 0.0,
                torque: 0.0,
                applied: 0.0,
            },
            Hold {
                angle: 2.0,
                pitch: 2.0,
                torque: 8.0,
                applied: 7.0,
            },
        ];
        let (slope, intercept) = fit_slope(&points, TorqueSource::Requested).unwrap();
        assert!((slope - 4.0).abs() < 1e-6);
        assert!(intercept.abs() < 1e-6);
        let (slope, intercept) = fit_slope(&points, TorqueSource::Applied).unwrap();
        assert!((slope - 3.5).abs() < 1e-6);
        assert!(intercept.abs() < 1e-6);
    }

    #[test]
    fn applied_torque_is_the_mean_of_both_wheels() {
        let line = "T step=1 al=40 ar=-20 trq=9 cwl=10 cwr=10";
        assert_eq!(hold_torque(line, TorqueSource::Requested), 9.0);
        assert_eq!(hold_torque(line, TorqueSource::Applied), 10.0);
    }

    #[test]
    fn supported_baseline_requires_a_lean_with_still_wheels() {
        assert!(supported_sample(5.0, 0.0, 0.0, 0.0));
        assert!(!supported_sample(5.0, 5.0, 0.0, 0.0));
        assert!(!supported_sample(5.0, 0.0, 30.0, -30.0));
        assert!(!supported_sample(0.2, 0.0, 0.0, 0.0));
    }

    #[test]
    fn torque_source_parses_both_modes() {
        assert_eq!(TorqueSource::parse("applied"), Some(TorqueSource::Applied));
        assert_eq!(
            TorqueSource::parse("requested"),
            Some(TorqueSource::Requested)
        );
        assert_eq!(TorqueSource::parse("other"), None);
        assert_eq!(TorqueSource::Applied.as_str(), "applied");
    }
}
