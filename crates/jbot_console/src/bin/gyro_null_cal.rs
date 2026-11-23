//! Dynamic identification: dead-zone search then gyro torque-null.
//!
//! Phase 1 — dynamic dead-zone search. Starting from a very small common-mode
//! `test` torque, the command grows in small steps while watching wheel RPM
//! and gyro rate. The first level at which the wheels clearly turn and the
//! body starts to respond is the loaded dead zone. No static placement.
//!
//! Phase 2 — torque null. The body is first allowed to settle: every probe
//! waits for an at-rest gyro before applying torque, so `alpha = d(gy)/dt`
//! measures the response to the probe rather than an ongoing swing. From just
//! above the measured dead zone, the torque grows gradually until `alpha`
//! crosses zero. That torque holds the current pitch against gravity and
//! friction. A gentle nudge moves the body a few degrees and the search
//! repeats, building the angle-to-torque curve dynamically.
//!
//! No reflash: only `test` commands and telemetry. Every probe is short, starts
//! very small, and the run ends with `estop`.
//!
//! Usage: gyro_null_cal --run [--port PATH] [--steps 6] [--probe-ms 140]
//!                      [--start-units 3] [--max-units 200] [--tol-alpha 14]
//!
//! Safety: rope attached, hands on the body, wheels on the floor, test zone at
//! most 0.5 m. Aborts on guard, 300 dps rate or 0.4 m travel.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,gy_dps,left_rpm,right_rpm,bat_v,age_ms,estop,torque_units,position_m,mode,phase,probe_units,alpha_dps2,pitch_at_probe,hold_torque,rpm_abs";

const MODE_LIMIT: f64 = 3.5;
const RUNAWAY_DPS: f64 = 300.0;
const TRAVEL_LIMIT_M: f64 = 0.4;
/// Wheel speed above which the drive is considered to have broken away, rpm.
const BREAKAWAY_RPM: f64 = 12.0;
/// Gyro response above which the body is considered to have reacted, dps.
const RESPONSE_GY: f64 = 3.0;
/// Gyro rate below which the body is considered at rest before a probe, dps.
const QUIET_GY: f64 = 8.0;
/// How long the body must stay quiet before a probe, milliseconds.
const QUIET_HOLD_MS: u64 = 400;

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
    probe_units: f64,
    alpha: f64,
    pitch_at_probe: f64,
    hold_torque: f64,
    rpm_abs: f64,
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
        format!("{:.1}", ctx.probe_units),
        format!("{:.2}", ctx.alpha),
        format!("{:.2}", ctx.pitch_at_probe),
        format!("{:.1}", ctx.hold_torque),
        format!("{:.1}", ctx.rpm_abs),
    ];
    values.join(",")
}

#[derive(Clone)]
struct Sample {
    t_ms: f64,
    t_ms_u: u64,
    line: String,
    pitch: f64,
    gyro: f64,
    x: f64,
    mode: f64,
    rpm: f64,
}

/// Read telemetry for `duration` without writing CSV. Callers that compute a
/// derived value (for example `alpha`) use this, then call `record` once the
/// value is known so every row carries it.
fn collect(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    started: &Instant,
    duration: Duration,
) -> Vec<Sample> {
    let mut out = Vec::new();
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
                            let (pitch, gyro, x, mode, rpm) = (
                                num(&text, "pitch"),
                                num(&text, "gy"),
                                num(&text, "x"),
                                num(&text, "mode"),
                                num(&text, "L").abs().max(num(&text, "R").abs()),
                            );
                            out.push(Sample {
                                t_ms: started.elapsed().as_secs_f64() * 1000.0,
                                t_ms_u: started.elapsed().as_millis() as u64,
                                line: text,
                                pitch,
                                gyro,
                                x,
                                mode,
                                rpm,
                            });
                        }
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
    out
}

fn record(csv: &mut BufWriter<File>, samples: &[Sample], ctx: RowCtx<'_>) {
    for sample in samples {
        let _ = writeln!(csv, "{}", csv_row(&sample.line, sample.t_ms_u, ctx));
    }
}

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    ctx: RowCtx<'_>,
) -> Vec<Sample> {
    let samples = collect(serial, buffer, started, duration);
    record(csv, &samples, ctx);
    samples
}

/// Wait until the body is quiet, then return the last sample. A quiet body is
/// required before each torque probe: `alpha` measured on a swinging body is
/// just the slope of an ongoing oscillation, not the response to the probe.
fn wait_quiet(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    max_wait: Duration,
    phase: &str,
) -> Option<Sample> {
    let deadline = Instant::now() + max_wait;
    let mut quiet_since: Option<Instant> = None;
    while Instant::now() < deadline {
        let samples = pump(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(50),
            RowCtx {
                phase,
                probe_units: 0.0,
                alpha: 0.0,
                pitch_at_probe: 0.0,
                hold_torque: 0.0,
                rpm_abs: 0.0,
            },
        );
        if let Some(sample) = samples.last() {
            if sample.gyro.abs() <= QUIET_GY && sample.pitch.abs() <= 15.0 {
                if quiet_since.is_none() {
                    quiet_since = Some(Instant::now());
                }
                if quiet_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(QUIET_HOLD_MS))
                {
                    return Some(sample.clone());
                }
            } else {
                quiet_since = None;
            }
        }
    }
    None
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

/// Angular acceleration in degrees per second squared from a least-squares fit
/// of gyro rate against time; sample timestamps are milliseconds, so the slope
/// needs a unit conversion before it can be compared with `alpha_dps2`.
fn angular_acceleration(samples: &[Sample]) -> Option<f64> {
    if samples.len() < 5 {
        return None;
    }
    let n = samples.len() as f64;
    let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
    for s in samples {
        sx += s.t_ms;
        sy += s.gyro;
        sxx += s.t_ms * s.t_ms;
        sxy += s.t_ms * s.gyro;
    }
    let det = n * sxx - sx * sx;
    if det.abs() < 1e-9 {
        return None;
    }
    Some(1000.0 * (n * sxy - sx * sy) / det)
}

#[derive(Clone, Copy)]
struct Point {
    pitch: f64,
    torque: f64,
}

fn fit_slope(points: &[Point]) -> Option<(f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let sx: f64 = points.iter().map(|p| p.pitch).sum();
    let sy: f64 = points.iter().map(|p| p.torque).sum();
    let sxx: f64 = points.iter().map(|p| p.pitch * p.pitch).sum();
    let sxy: f64 = points.iter().map(|p| p.pitch * p.torque).sum();
    let det = n * sxx - sx * sx;
    if det.abs() < 1e-9 {
        return None;
    }
    let slope = (n * sxy - sx * sy) / det;
    let intercept = (sy - slope * sx) / n;
    Some((slope, intercept))
}

fn write_json(path: &PathBuf, dead_zone: f64, points: &[Point], abort: Option<&str>) {
    let mut file = match File::create(path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {}: {error}", path.display());
            return;
        }
    };
    let _ = writeln!(file, "{{");
    let _ = writeln!(file, "  \"dead_zone_units\": {dead_zone:.1},");
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
            "    {{\"pitch_deg\": {:.2}, \"torque_units\": {:.2}}}{comma}",
            point.pitch, point.torque
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
    let mut steps = 6_u32;
    let mut probe_ms = 140_u64;
    let mut start_units = 3.0_f64;
    let mut max_units = 200.0_f64;
    let mut tol_alpha = 14.0_f64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--steps" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u32>().ok()) {
                    steps = value.clamp(2, 20);
                }
            }
            "--probe-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    probe_ms = value.clamp(80, 400);
                }
            }
            "--start-units" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    start_units = value.clamp(1.0, 20.0);
                }
            }
            "--max-units" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    max_units = value.clamp(40.0, 290.0);
                }
            }
            "--tol-alpha" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    tol_alpha = value.clamp(3.0, 40.0);
                }
            }
            "--help" | "-h" => {
                println!("usage: gyro_null_cal --run [--port PATH] [--steps 6] [--probe-ms 140] [--start-units 3] [--max-units 200] [--tol-alpha 14]");
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

    let stem = format!("csv/jbot-gyronull-{}", timestamp_ms());
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
            probe_units: 0.0,
            alpha: 0.0,
            pitch_at_probe: 0.0,
            hold_torque: 0.0,
            rpm_abs: 0.0,
        },
    );
    send(&mut serial, "clear");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(200),
        RowCtx {
            phase: "init",
            probe_units: 0.0,
            alpha: 0.0,
            pitch_at_probe: 0.0,
            hold_torque: 0.0,
            rpm_abs: 0.0,
        },
    );

    // Quiet upright baseline: start from rest with a known zero travel origin.
    let baseline = wait_quiet(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_secs(30),
        "baseline",
    );
    let Some(baseline) = baseline else {
        println!("no quiet upright baseline; estop sent");
        send(&mut serial, "estop");
        let _ = csv.flush();
        write_json(&json_path, 0.0, &[], Some("no baseline"));
        return;
    };
    let mut pitch = baseline.pitch;
    let mut origin_x = baseline.x;
    println!("baseline pitch {pitch:.2} deg");

    let mut abort: Option<String> = None;

    // ---- Phase 1: dynamic dead-zone search -------------------------------
    // Grow the command from a very small value until the wheels break away
    // and the body visibly reacts. Record that level as the loaded dead zone.
    let mut dead_zone = 0.0_f64;
    let mut magnitude = start_units;
    while magnitude <= max_units {
        send(&mut serial, &format!("test {magnitude:.1}"));
        let samples = collect(
            &mut serial,
            &mut buffer,
            &started,
            Duration::from_millis(probe_ms),
        );
        send(&mut serial, "test 0");
        let max_rpm = samples.iter().map(|s| s.rpm).fold(0.0, f64::max);
        let max_gy = samples.iter().map(|s| s.gyro.abs()).fold(0.0, f64::max);
        let alpha = angular_acceleration(&samples).unwrap_or(0.0);
        record(
            &mut csv,
            &samples,
            RowCtx {
                phase: "deadzone",
                probe_units: magnitude,
                alpha,
                pitch_at_probe: pitch,
                hold_torque: 0.0,
                rpm_abs: max_rpm,
            },
        );
        if let Some(last) = samples.last() {
            pitch = last.pitch;
            if last.gyro.abs() > RUNAWAY_DPS
                || (last.x - origin_x).abs() > TRAVEL_LIMIT_M
                || samples.iter().any(|s| s.mode > MODE_LIMIT)
            {
                abort = Some(format!("guard in dead-zone search: gy {:.1}", last.gyro));
                break;
            }
        }
        println!(
            "dead-zone probe {:+.1}: rpm {:.0}, gy {:.1}, alpha {:.1}",
            magnitude, max_rpm, max_gy, alpha
        );
        if max_rpm >= BREAKAWAY_RPM && max_gy >= RESPONSE_GY {
            dead_zone = magnitude;
            break;
        }
        magnitude *= 1.3;
    }
    if let Some(reason) = &abort {
        println!("{reason}");
        send(&mut serial, "estop");
        let _ = csv.flush();
        write_json(&json_path, dead_zone, &[], Some(reason));
        return;
    }
    if dead_zone <= 0.0 {
        println!("no dead zone found up to {max_units:.0} units; estop sent");
        send(&mut serial, "estop");
        let _ = csv.flush();
        write_json(&json_path, 0.0, &[], Some("no dead zone found"));
        return;
    }
    println!("dynamic dead zone: {dead_zone:.1} units");

    // The dead-zone search leaves the body swinging. Wait for it to come to
    // rest before the first torque-null probe, otherwise `alpha` measures the
    // slope of an ongoing oscillation and every null is false.
    let Some(settled) = wait_quiet(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_secs(10),
        "resettle",
    ) else {
        println!("body did not settle after dead-zone search; estop sent");
        send(&mut serial, "estop");
        let _ = csv.flush();
        write_json(
            &json_path,
            dead_zone,
            &[],
            Some("no resettle after dead zone"),
        );
        return;
    };
    pitch = settled.pitch;
    origin_x = settled.x;
    println!("resettled at pitch {pitch:.2} deg before torque null");

    // ---- Phase 2: gyro torque-null above the dead zone ------------------
    let mut points: Vec<Point> = Vec::new();
    for _ in 0..steps {
        let sign = if pitch < 0.0 { -1.0 } else { 1.0 };
        let mut alpha_prev = f64::NAN;
        let mut tau_prev = 0.0_f64;
        let mut converged: Option<f64> = None;
        let mut magnitude = dead_zone * 1.05;

        loop {
            if magnitude > max_units {
                break;
            }
            // Each probe must start from rest: `alpha` on a moving body is the
            // slope of the existing swing, not the response to this torque.
            let Some(quiet) = wait_quiet(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_secs(5),
                "preprobe",
            ) else {
                abort = Some("body not quiet before torque probe".to_owned());
                break;
            };
            pitch = quiet.pitch;
            let tau = sign * magnitude;
            send(&mut serial, &format!("test {tau:.1}"));
            let samples = collect(
                &mut serial,
                &mut buffer,
                &started,
                Duration::from_millis(probe_ms),
            );
            send(&mut serial, "test 0");
            let alpha = angular_acceleration(&samples).unwrap_or(0.0);
            let max_rpm = samples.iter().map(|s| s.rpm).fold(0.0, f64::max);
            record(
                &mut csv,
                &samples,
                RowCtx {
                    phase: "probe",
                    probe_units: tau,
                    alpha,
                    pitch_at_probe: pitch,
                    hold_torque: 0.0,
                    rpm_abs: max_rpm,
                },
            );
            if let Some(last) = samples.last() {
                pitch = last.pitch;
                if last.gyro.abs() > RUNAWAY_DPS
                    || (last.x - origin_x).abs() > TRAVEL_LIMIT_M
                    || samples.iter().any(|s| s.mode > MODE_LIMIT)
                {
                    abort = Some(format!("body ran away: gy {:.1}", last.gyro));
                    break;
                }
            }
            println!(
                "probe tau {:+.1}: alpha {:+.1} dps2, pitch {:+.2}",
                tau, alpha, pitch
            );
            if alpha.abs() <= tol_alpha {
                converged = Some(tau);
                break;
            }
            if alpha_prev.is_finite() && (alpha * alpha_prev) < 0.0 {
                let t = alpha_prev / (alpha_prev - alpha);
                converged = Some(tau_prev + t * (tau - tau_prev));
                break;
            }
            alpha_prev = alpha;
            tau_prev = tau;
            magnitude *= 1.25;
        }
        if let Some(reason) = &abort {
            println!("{reason}");
            break;
        }
        let Some(tau_hold) = converged else {
            println!("no null at pitch {pitch:.2}; skipping");
            continue;
        };
        println!("NULL pitch {:+.2}: torque {:+.1} units", pitch, tau_hold);
        points.push(Point {
            pitch,
            torque: tau_hold,
        });

        let nudge = -sign * (dead_zone * 0.4).min(max_units * 0.3);
        send(&mut serial, &format!("test {nudge:.1}"));
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(100),
            RowCtx {
                phase: "nudge",
                probe_units: nudge,
                alpha: 0.0,
                pitch_at_probe: pitch,
                hold_torque: 0.0,
                rpm_abs: 0.0,
            },
        );
        send(&mut serial, "test 0");
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(300),
            RowCtx {
                phase: "settle",
                probe_units: 0.0,
                alpha: 0.0,
                pitch_at_probe: pitch,
                hold_torque: tau_hold,
                rpm_abs: 0.0,
            },
        );
    }

    if abort.is_none() && points.len() < 3 {
        abort = Some("fewer than three converged points".to_owned());
    }
    if let Some((slope, intercept)) = fit_slope(&points) {
        println!("holding torque T(pitch) = {slope:.2}*pitch + {intercept:.1} units");
        println!("feedforward: ff(pitch) = {slope:.2}*pitch + {intercept:.1} units");
        println!("current firmware uses +3.23 units/deg relative to home -2.5 deg");
    } else {
        println!("not enough converged points to fit a torque curve");
    }

    send(&mut serial, "estop");
    let _ = csv.flush();
    write_json(&json_path, dead_zone, &points, abort.as_deref());
    if let Some(reason) = &abort {
        println!("ABORT: {reason}; estop sent");
    } else {
        println!("calibration complete; estop sent");
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

    fn sample(t: f64, gyro: f64) -> Sample {
        Sample {
            t_ms: t,
            t_ms_u: t as u64,
            line: String::new(),
            pitch: 0.0,
            gyro,
            x: 0.0,
            mode: 0.0,
            rpm: 0.0,
        }
    }

    #[test]
    fn acceleration_from_constant_rate_is_zero() {
        let samples = vec![
            sample(0.0, 10.0),
            sample(50.0, 10.0),
            sample(100.0, 10.0),
            sample(150.0, 10.0),
            sample(200.0, 10.0),
        ];
        assert!(angular_acceleration(&samples).unwrap().abs() < 1e-6);
    }

    #[test]
    fn acceleration_recovers_linear_ramp() {
        let samples = vec![
            sample(0.0, 0.0),
            sample(50.0, 5.0),
            sample(100.0, 10.0),
            sample(150.0, 15.0),
            sample(200.0, 20.0),
        ];
        assert!((angular_acceleration(&samples).unwrap() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn slope_fit_recovers_line() {
        let points = [
            Point {
                pitch: -2.0,
                torque: -8.0,
            },
            Point {
                pitch: 0.0,
                torque: 0.0,
            },
            Point {
                pitch: 2.0,
                torque: 8.0,
            },
        ];
        let (slope, intercept) = fit_slope(&points).unwrap();
        assert!((slope - 4.0).abs() < 1e-6);
        assert!(intercept.abs() < 1e-6);
    }
}
