//! Feedforward calibration by capturing the pitch zero at each tilt.
//!
//! The balance law always regulates toward its captured zero, even when that
//! zero is a tilted attitude. Holding the body at a small physical tilt, then
//! sending `zero` there, makes the law balance at that tilt on its next `arm`.
//! The torque its integral settles on is exactly the holding torque `T(theta)`
//! for that angle. Repeating for several tilts gives the feedforward curve with
//! the existing firmware and no reference-step transient.
//!
//! The operator only holds the body still for a moment per angle; there is no
//! rope release, no repeatable fixture and no reflash.
//!
//! Usage: zero_capture_cal --run [--port PATH] [--pscale 0.25] [--slew 100]
//!                         [--angles -4,-2,0,2,4] [--tol 1.0] [--hold-ms 1200]
//!
//! Safety: rope attached, hands on the body, wheels on the floor, test zone at
//! most 0.5 m. Every angle ends with `estop`; the original zero is restored.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,gy_dps,left_rpm,right_rpm,bat_v,age_ms,estop,torque_units,position_m,mode,phase,target_pitch,angle_deg,hold_torque,settled";

const MODE_LIMIT: f64 = 3.5;
const RUNAWAY_DPS: f64 = 250.0;
const TRAVEL_LIMIT_M: f64 = 0.4;
/// How long the operator has to place the body at each target angle.
const PLACE_TIMEOUT_S: u64 = 40;

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

#[derive(Clone, Copy)]
struct RowCtx<'a> {
    phase: &'a str,
    target_pitch: f64,
    angle: f64,
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
        format!("{:.4}", n("gy")),
        n("L").to_string(),
        n("R").to_string(),
        format!("{:.2}", n("bat")),
        (n("age") as u64).to_string(),
        (n("estop") as u64).to_string(),
        format!("{:.1}", n("trq")),
        format!("{:.4}", n("x")),
        (n("mode") as u64).to_string(),
        ctx.phase.to_owned(),
        format!("{:.2}", ctx.target_pitch),
        format!("{:.2}", ctx.angle),
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
        .filter(|value| value.abs() <= 8.0)
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

#[derive(Clone, Copy)]
struct Hold {
    angle: f64,
    torque: f64,
}

fn fit_slope(points: &[Hold]) -> Option<(f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let sx: f64 = points.iter().map(|p| p.angle).sum();
    let sy: f64 = points.iter().map(|p| p.torque).sum();
    let sxx: f64 = points.iter().map(|p| p.angle * p.angle).sum();
    let sxy: f64 = points.iter().map(|p| p.angle * p.torque).sum();
    let det = n * sxx - sx * sx;
    if det.abs() < 1e-9 {
        return None;
    }
    let slope = (n * sxy - sx * sy) / det;
    let intercept = (sy - slope * sx) / n;
    Some((slope, intercept))
}

fn write_json(path: &PathBuf, pscale: f64, points: &[Hold], abort: Option<&str>) {
    let mut file = match File::create(path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {}: {error}", path.display());
            return;
        }
    };
    let _ = writeln!(file, "{{");
    let _ = writeln!(file, "  \"pscale\": {pscale:.2},");
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
            "    {{\"angle_deg\": {:.2}, \"torque_units\": {:.2}}}{comma}",
            point.angle, point.torque
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
    let mut angles_text = "-4,-2,0,2,4".to_owned();
    let mut tol = 1.0_f64;
    let mut hold_ms = 1200_u64;
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
                    slew = value.clamp(20.0, 200.0);
                }
            }
            "--angles" => {
                if let Some(value) = args.next() {
                    angles_text = value;
                }
            }
            "--tol" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    tol = value.clamp(0.3, 3.0);
                }
            }
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(200, 4000);
                }
            }
            "--help" | "-h" => {
                println!("usage: zero_capture_cal --run [--port PATH] [--pscale 0.25] [--slew 100] [--angles -4,-2,0,2,4] [--tol 1.0] [--hold-ms 1200]");
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
        eprintln!("no valid angles in -8..8 range");
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

    let stem = format!("csv/jbot-zerocal-{}", timestamp_ms());
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
            target_pitch: 0.0,
            angle: 0.0,
            hold_torque: 0.0,
            settled: false,
        },
    );
    send(&mut serial, "clear");
    send(&mut serial, "ff off");
    send(&mut serial, &format!("pscale {pscale:.2}"));
    send(&mut serial, &format!("slew {slew:.0}"));
    println!("safe profile: pscale {pscale:.2}, slew {slew:.0}, ff off");

    // Baseline: hold the robot upright once, capture level, and record the
    // physical pitch that represents "vertical" for this session.
    println!("PLACE the robot upright and still, then wait for the baseline capture");
    let mut baseline_pitch = 0.0_f64;
    let mut zof = 0.0_f64;
    let mut origin_x = 0.0_f64;
    let mut baseline_since: Option<Instant> = None;
    let place_start = Instant::now();
    while place_start.elapsed() < Duration::from_secs(PLACE_TIMEOUT_S) {
        let lines = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(60),
            RowCtx {
                phase: "baseline",
                target_pitch: 0.0,
                angle: 0.0,
                hold_torque: 0.0,
                settled: false,
            },
        );
        let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) else {
            continue;
        };
        let pitch = num(line, "pitch");
        if num(line, "age") > 300.0 || num(line, "estop") > 0.5 {
            continue;
        }
        if (pitch - baseline_pitch).abs() <= tol && num(line, "gy").abs() <= 8.0 {
            if baseline_since.is_none() {
                baseline_since = Some(Instant::now());
            }
            baseline_pitch = pitch;
            zof = num(line, "zof");
            origin_x = num(line, "x");
            if baseline_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500)) {
                break;
            }
        } else {
            baseline_since = None;
            baseline_pitch = pitch;
        }
    }
    if !baseline_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500)) {
        println!("no quiet upright baseline; estop sent");
        send(&mut serial, "estop");
        let _ = csv.flush();
        write_json(&json_path, pscale, &[], Some("no upright baseline"));
        return;
    }
    let vertical_physical = baseline_pitch + zof;
    let original_zof = zof;
    println!(
        "upright baseline pitch {baseline_pitch:.2}, physical vertical {vertical_physical:.2}"
    );

    let mut points: Vec<Hold> = Vec::new();
    let mut abort: Option<String> = None;

    for delta in &angles {
        let delta = *delta;
        // The previous angle ended with `estop`; release the latch before the
        // operator repositions the body, otherwise the guard trips instantly.
        send(&mut serial, "clear");
        // Target telemetry pitch for the operator, in the current zero frame.
        let target_pitch = vertical_physical + delta - zof;
        println!(
            "PLACE the body at pitch {target_pitch:+.1} deg (tilt {delta:+.1} from vertical); hold still"
        );

        let mut placed_since: Option<Instant> = None;
        let place_deadline = Instant::now() + Duration::from_secs(PLACE_TIMEOUT_S);
        while Instant::now() < place_deadline {
            let lines = pump(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(50),
                RowCtx {
                    phase: "place",
                    target_pitch,
                    angle: delta,
                    hold_torque: 0.0,
                    settled: false,
                },
            );
            let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) else {
                continue;
            };
            let pitch = num(line, "pitch");
            if num(line, "estop") > 0.5 || num(line, "mode") > MODE_LIMIT {
                abort = Some("guard while placing".to_owned());
                break;
            }
            if (pitch - target_pitch).abs() <= tol && num(line, "gy").abs() <= 8.0 {
                if placed_since.is_none() {
                    placed_since = Some(Instant::now());
                }
                if placed_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(400)) {
                    break;
                }
            } else {
                placed_since = None;
            }
        }
        if let Some(reason) = &abort {
            println!("{reason}");
            break;
        }
        if !placed_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(400)) {
            println!("tilt {delta:+.1}: no stable placement, skipping");
            continue;
        }

        // Capture this attitude as the new zero, then level at it so the law
        // balances exactly here instead of at the previous trim.
        send(&mut serial, "zero");
        send(&mut serial, "level");
        let mut new_zof = zof;
        for _ in 0..10 {
            let lines = pump(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(80),
                RowCtx {
                    phase: "capture",
                    target_pitch,
                    angle: delta,
                    hold_torque: 0.0,
                    settled: false,
                },
            );
            if let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) {
                new_zof = num(line, "zof");
                if (new_zof - zof).abs() > 1e-6 {
                    break;
                }
            }
        }
        zof = new_zof;
        let angle = zof - vertical_physical;
        println!(
            "tilt {delta:+.1}: captured zero {zof:.2}, physical angle {angle:+.2} deg; RELEASE"
        );

        send(&mut serial, "arm");
        let balance_start = Instant::now();
        let mut settled_since: Option<Instant> = None;
        let mut hold_sum = 0.0_f64;
        let mut hold_n = 0_u64;
        let mut stable = false;
        while balance_start.elapsed() < Duration::from_millis(hold_ms + 6000) {
            let lines = pump(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(40),
                RowCtx {
                    phase: "balance",
                    target_pitch,
                    angle,
                    hold_torque: if hold_n > 0 {
                        hold_sum / hold_n as f64
                    } else {
                        0.0
                    },
                    settled: stable,
                },
            );
            let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) else {
                continue;
            };
            let pitch = num(line, "pitch");
            let rate = num(line, "gy");
            if num(line, "estop") > 0.5
                || num(line, "mode") > MODE_LIMIT
                || rate.abs() > RUNAWAY_DPS
                || (num(line, "x") - origin_x).abs() > TRAVEL_LIMIT_M
            {
                abort = Some(format!(
                    "guard while balancing at {angle:+.1}: pitch {pitch:.1}, rate {rate:.1}"
                ));
                break;
            }
            let armed = num(line, "arm") > 0.5;
            // Count only a continuous quiet window. Leaving tolerance resets
            // the window, so a transient swing cannot contribute samples.
            if armed && pitch.abs() <= tol && rate.abs() <= 8.0 {
                let now = Instant::now();
                if settled_since.is_none() {
                    settled_since = Some(now);
                    hold_sum = 0.0;
                    hold_n = 0;
                }
                stable = true;
                hold_sum += num(line, "trq");
                hold_n += 1;
                if settled_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(hold_ms)) {
                    break;
                }
            } else {
                settled_since = None;
                hold_sum = 0.0;
                hold_n = 0;
                stable = false;
            }
        }
        if let Some(reason) = &abort {
            println!("{reason}");
            break;
        }
        if hold_n == 0 {
            println!("tilt {delta:+.1}: no continuous stable balance, skipping");
        } else {
            let torque = hold_sum / hold_n as f64;
            println!(
                "angle {angle:+.2} deg: torque {torque:+.2} units ({} stable samples)",
                hold_n
            );
            points.push(Hold { angle, torque });
        }
        send(&mut serial, "estop");
    }

    if let Some((slope, intercept)) = fit_slope(&points) {
        println!("measured holding torque T(angle) = {slope:.2}*angle + {intercept:.1} units");
        println!("feedforward to apply: ff(angle) = {slope:.2}*angle + {intercept:.1} units");
        println!("current firmware uses +3.23 units/deg relative to home -2.5 deg");
    } else {
        println!("not enough stable holds to fit a torque curve");
    }

    // Restore the original pitch zero so the session leaves the stored frame
    // unchanged. `pose home` writes the value and persists it.
    send(&mut serial, &format!("pose home {original_zof:.2}"));
    send(&mut serial, "estop");
    let _ = csv.flush();
    write_json(&json_path, pscale, &points, abort.as_deref());
    if let Some(reason) = &abort {
        println!("ABORT: {reason}; estop sent");
    } else {
        println!("calibration complete; original zero restored and estop sent");
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
    fn angle_parser_filters_range() {
        assert_eq!(
            parse_angles("-4,-2,0,2,4,9,x"),
            vec![-4.0, -2.0, 0.0, 2.0, 4.0]
        );
    }

    #[test]
    fn slope_fit_recovers_line() {
        let points = [
            Hold {
                angle: -2.0,
                torque: -8.0,
            },
            Hold {
                angle: 0.0,
                torque: 0.0,
            },
            Hold {
                angle: 2.0,
                torque: 8.0,
            },
        ];
        let (slope, intercept) = fit_slope(&points).unwrap();
        assert!((slope - 4.0).abs() < 1e-6);
        assert!(intercept.abs() < 1e-6);
    }
}
