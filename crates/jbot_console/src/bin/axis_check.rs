//! Guided sign and axis check for the balance law.
//!
//! A re-mounted IMU or a re-wired drive can silently break one of the four
//! conventions the control law depends on:
//!
//! 1. leaning forward increases `pitch`;
//! 2. the reported pitch rate (`gy`) has the same sign as `d(pitch)/dt`, so the
//!    derivative term damps the motion instead of exciting it;
//! 3. turning counter-clockwise seen from above gives a positive yaw rate `gz`;
//! 4. `test 50 50` rolls forward (`L > 0`, `R < 0`) and `test 100 -100` pivots
//!    with a negative yaw rate.
//!
//! The tool only reads telemetry and, with `--wheels`, applies the bounded
//! spin-test torque. It never arms the balance law and ends with `estop`.
//!
//! Usage: axis_check --run [--port PATH] [--phase-ms 3000] [--wheels]
//!
//! Safety: blocks under the body, rope attached, hands on the body, e-stop
//! reachable, test zone at most 0.5 m.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,phase,pitch_deg,pitch_rate_dps,yaw_rate_dps,left_rpm,right_rpm";
/// Time the operator gets to move the body before a capture window starts.
const LEAD_MS: u64 = 1_500;
/// Minimum lean away from the upright baseline for the sign to be trusted.
const MIN_PITCH_DELTA_DEG: f64 = 3.0;
/// Minimum absolute yaw rate during the counter-clockwise turn.
const MIN_YAW_DPS: f64 = 5.0;
/// Minimum correlation between `d(pitch)/dt` and the reported pitch rate.
const MIN_CORR: f64 = 0.7;
/// Minimum slope of the pitch-rate fit; a mirrored channel reads negative.
const MIN_SLOPE: f64 = 0.3;
/// Wheel speed that proves a roll direction, rpm.
const MIN_WHEEL_RPM: f64 = 2.0;
/// Yaw rate that proves the pivot direction, dps.
const MIN_PIVOT_DPS: f64 = 2.0;

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

/// One telemetry sample reduced to the fields the check needs.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sample {
    /// Host receive time, milliseconds since the run started.
    pub t_ms: f64,
    /// Mount-compensated pitch, degrees.
    pub pitch_deg: f64,
    /// Mount-compensated pitch rate, degrees per second.
    pub pitch_rate_dps: f64,
    /// Mount-compensated yaw rate, degrees per second.
    pub yaw_rate_dps: f64,
    /// Left hall speed, rpm.
    pub left_rpm: f64,
    /// Right hall speed, rpm.
    pub right_rpm: f64,
}

/// Mean of one captured window.
#[derive(Clone, Copy, Debug, Default)]
pub struct PhaseStats {
    /// Mean pitch, degrees.
    pub pitch: f64,
    /// Mean pitch rate, degrees per second.
    pub pitch_rate: f64,
    /// Mean yaw rate, degrees per second.
    pub yaw_rate: f64,
    /// Mean left hall speed, rpm.
    pub left_rpm: f64,
    /// Mean right hall speed, rpm.
    pub right_rpm: f64,
    /// Number of samples in the window.
    pub samples: usize,
}

impl PhaseStats {
    /// Reduce a window to its means.
    pub fn of(samples: &[Sample]) -> Self {
        if samples.is_empty() {
            return Self::default();
        }
        let count = samples.len() as f64;
        let sum = |value: fn(&Sample) -> f64| samples.iter().map(value).sum::<f64>() / count;
        Self {
            pitch: sum(|sample| sample.pitch_deg),
            pitch_rate: sum(|sample| sample.pitch_rate_dps),
            yaw_rate: sum(|sample| sample.yaw_rate_dps),
            left_rpm: sum(|sample| sample.left_rpm),
            right_rpm: sum(|sample| sample.right_rpm),
            samples: samples.len(),
        }
    }
}

/// Wheel-direction evidence from the two spin tests.
#[derive(Clone, Copy, Debug)]
pub struct WheelCheck {
    /// Mean left hall speed during `test 50 50`, rpm.
    pub forward_left_rpm: f64,
    /// Mean right hall speed during `test 50 50`, rpm.
    pub forward_right_rpm: f64,
    /// Mean yaw rate during `test 100 -100`, degrees per second.
    pub pivot_yaw_dps: f64,
}

/// Outcome of the whole check.
#[derive(Clone, Copy, Debug)]
pub struct Report {
    /// Pitch of the upright baseline, degrees.
    pub baseline_pitch: f64,
    /// Pitch change while leaning forward, degrees.
    pub forward_delta: f64,
    /// Pitch change while leaning backward, degrees.
    pub backward_delta: f64,
    /// Correlation of `d(pitch)/dt` against the reported pitch rate.
    pub corr: f64,
    /// Slope of that fit.
    pub slope: f64,
    /// Mean yaw rate while turning counter-clockwise, degrees per second.
    pub yaw_rate: f64,
    /// Wheel evidence when the wheel test ran.
    pub wheel: Option<WheelCheck>,
    /// Forward lean increases pitch and backward lean decreases it.
    pub pitch_ok: bool,
    /// The reported pitch rate matches the angle derivative in sign and scale.
    pub rate_ok: bool,
    /// Counter-clockwise rotation gives a positive yaw rate.
    pub yaw_ok: bool,
    /// Wheel signs when the wheel test ran; `None` when it was skipped.
    pub wheels_ok: Option<bool>,
}

impl Report {
    /// True when every check that ran passed.
    pub fn pass(&self) -> bool {
        self.pitch_ok && self.rate_ok && self.yaw_ok && self.wheels_ok.unwrap_or(true)
    }
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<f64>() / values.len() as f64
}

/// Correlation and slope of `d(pitch)/dt` against the reported pitch rate.
///
/// Returns `None` when fewer than eight usable intervals exist, so a still
/// body cannot produce a verdict.
pub fn pitch_rate_pairing(samples: &[Sample]) -> Option<(f64, f64)> {
    let mut derivative = Vec::new();
    let mut reported = Vec::new();
    for window in samples.windows(2) {
        let dt = (window[1].t_ms - window[0].t_ms) / 1000.0;
        if !(0.001..=0.05).contains(&dt) {
            continue;
        }
        derivative.push((window[1].pitch_deg - window[0].pitch_deg) / dt);
        reported.push(window[1].pitch_rate_dps);
    }
    if derivative.len() < 8 {
        return None;
    }
    let dx = mean(&derivative);
    let ry = mean(&reported);
    let mut cov = 0.0;
    let mut sxx = 0.0;
    let mut syy = 0.0;
    let mut sxy = 0.0;
    for (x, y) in derivative.iter().zip(reported.iter()) {
        let ex = x - dx;
        let ey = y - ry;
        cov += ex * ey;
        sxx += ex * ex;
        syy += ey * ey;
        sxy += x * y;
    }
    if sxx <= 0.0 || syy <= 0.0 {
        return None;
    }
    let corr = cov / (sxx.sqrt() * syy.sqrt());
    let slope = sxy / sxx;
    Some((corr, slope))
}

/// Reduce the captured windows to a verdict.
pub fn evaluate(
    baseline: PhaseStats,
    forward: PhaseStats,
    backward: PhaseStats,
    yaw: PhaseStats,
    pairing: Option<(f64, f64)>,
    wheel: Option<WheelCheck>,
) -> Report {
    let forward_delta = forward.pitch - baseline.pitch;
    let backward_delta = backward.pitch - baseline.pitch;
    let (corr, slope) = pairing.unwrap_or((0.0, 0.0));
    let wheels_ok = wheel.map(|evidence| {
        evidence.forward_left_rpm >= MIN_WHEEL_RPM
            && evidence.forward_right_rpm <= -MIN_WHEEL_RPM
            && evidence.pivot_yaw_dps <= -MIN_PIVOT_DPS
    });
    Report {
        baseline_pitch: baseline.pitch,
        forward_delta,
        backward_delta,
        corr,
        slope,
        yaw_rate: yaw.yaw_rate,
        wheel,
        pitch_ok: forward_delta >= MIN_PITCH_DELTA_DEG && backward_delta <= -MIN_PITCH_DELTA_DEG,
        rate_ok: pairing.is_some() && corr >= MIN_CORR && slope >= MIN_SLOPE,
        yaw_ok: yaw.yaw_rate >= MIN_YAW_DPS,
        wheels_ok,
    }
}

fn print_report(report: &Report) {
    println!("\n=== sign and axis report ===");
    println!(
        "  pitch   baseline {:+.2} deg, forward {:+.2} deg, backward {:+.2} deg  [{}]",
        report.baseline_pitch,
        report.forward_delta,
        report.backward_delta,
        if report.pitch_ok { "PASS" } else { "FAIL" }
    );
    println!(
        "  rate    corr {:+.2}, slope {:+.2}  [{}]",
        report.corr,
        report.slope,
        if report.rate_ok { "PASS" } else { "FAIL" }
    );
    println!(
        "  yaw     counter-clockwise {:+.1} dps  [{}]",
        report.yaw_rate,
        if report.yaw_ok { "PASS" } else { "FAIL" }
    );
    if let Some(wheel) = report.wheel {
        println!(
            "  wheels  forward L {:+.1} R {:+.1} rpm, pivot yaw {:+.1} dps  [{}]",
            wheel.forward_left_rpm,
            wheel.forward_right_rpm,
            wheel.pivot_yaw_dps,
            if report.wheels_ok == Some(true) {
                "PASS"
            } else {
                "FAIL"
            }
        );
    }
    if !report.pitch_ok {
        println!(
            "  -> pitch sign is wrong: forward lean must raise pitch. Re-run the guided\n     \
             axis calibration and re-capture zero and level."
        );
    }
    if !report.rate_ok {
        println!(
            "  -> the pitch rate does not follow d(pitch)/dt. The damper is driving the\n     \
             wrong channel; check the mount selector and the raw axis pairing."
        );
    }
    if !report.yaw_ok {
        println!(
            "  -> yaw rate is inverted: a counter-clockwise turn must give positive gz.\n     \
             The yaw loop would spin the machine instead of damping it."
        );
    }
    if report.wheels_ok == Some(false) {
        println!(
            "  -> wheel signs are wrong: `test 50 50` must roll forward (L>0, R<0) and\n     \
             `test 100 -100` must pivot with negative yaw. Check the mixer and wiring."
        );
    }
    println!("  result: {}", if report.pass() { "PASS" } else { "FAIL" });
}

fn send(serial: &mut Box<dyn SerialPort>, command: &str) {
    println!("> {command}");
    let _ = serial.write_all(command.as_bytes());
    let _ = serial.write_all(b"\n");
    let _ = serial.flush();
}

fn collect(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    phase: &str,
) -> Vec<Sample> {
    let mut out = Vec::new();
    let deadline = Instant::now() + duration;
    let mut chunk = [0_u8; 1024];
    while Instant::now() < deadline {
        let read = serial.read(&mut chunk).unwrap_or_default();
        for &byte in &chunk[..read] {
            match byte {
                b'\r' => {}
                b'\n' => {
                    if let Ok(text) = std::str::from_utf8(buffer) {
                        let text = text.trim_end().to_owned();
                        if is_telemetry(&text) {
                            let sample = Sample {
                                t_ms: started.elapsed().as_secs_f64() * 1000.0,
                                pitch_deg: num(&text, "pitch"),
                                pitch_rate_dps: num(&text, "gy"),
                                yaw_rate_dps: num(&text, "gz"),
                                left_rpm: num(&text, "L"),
                                right_rpm: num(&text, "R"),
                            };
                            let _ = writeln!(
                                csv,
                                "{:.3},{},{:.4},{:.4},{:.4},{},{}",
                                sample.t_ms,
                                phase,
                                sample.pitch_deg,
                                sample.pitch_rate_dps,
                                sample.yaw_rate_dps,
                                sample.left_rpm,
                                sample.right_rpm
                            );
                            out.push(sample);
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

fn phase(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    instruction: &str,
    label: &str,
    window: Duration,
) -> (PhaseStats, Vec<Sample>) {
    println!("\n==> {instruction}");
    println!("    prepare; capturing in {} ms", LEAD_MS);
    let _ = collect(
        serial,
        buffer,
        csv,
        started,
        Duration::from_millis(LEAD_MS),
        "lead",
    );
    println!("    CAPTURING {} ms", window.as_millis());
    let samples = collect(serial, buffer, csv, started, window, label);
    let stats = PhaseStats::of(&samples);
    println!(
        "    mean pitch {:+.2} deg, rate {:+.1} dps, yaw {:+.1} dps, L {:+.0} R {:+.0} rpm ({} samples)",
        stats.pitch,
        stats.pitch_rate,
        stats.yaw_rate,
        stats.left_rpm,
        stats.right_rpm,
        stats.samples
    );
    (stats, samples)
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut port = None;
    let mut phase_ms = 3_000_u64;
    let mut wheels = false;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--phase-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    phase_ms = value.clamp(1_000, 10_000);
                }
            }
            "--wheels" => wheels = true,
            "--help" | "-h" => {
                println!("usage: axis_check --run [--port PATH] [--phase-ms 3000] [--wheels]");
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

    let csv_path = format!("csv/jbot-axis-{}.csv", timestamp_ms());
    if let Some(parent) = std::path::Path::new(&csv_path).parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let mut csv = match File::create(&csv_path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {csv_path}: {error}");
            return;
        }
    };
    let _ = writeln!(csv, "{HEADER}");

    let started = Instant::now();
    let mut buffer = Vec::new();
    let window = Duration::from_millis(phase_ms);

    send(&mut serial, "clear");
    let _ = collect(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(300),
        "setup",
    );
    send(&mut serial, "disarm");
    let _ = collect(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(200),
        "setup",
    );
    send(&mut serial, "stream 100");
    let _ = collect(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(300),
        "setup",
    );

    let (baseline, _) = phase(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        "HOLD the robot UPRIGHT and still",
        "upright",
        window,
    );
    let (forward, forward_samples) = phase(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        "LEAN the robot FORWARD 20-30 deg and hold it",
        "forward",
        window,
    );
    let (backward, backward_samples) = phase(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        "LEAN the robot BACKWARD 20-30 deg and hold it",
        "backward",
        window,
    );
    let (yaw, _) = phase(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        "TURN the body COUNTER-CLOCKWISE seen from above and hold it",
        "yaw",
        window,
    );

    let mut motion = forward_samples;
    motion.extend(backward_samples);
    let pairing = pitch_rate_pairing(&motion);

    let wheel = if wheels {
        println!("\nWheel test: free both wheels, keep the body secured, zone at most 0.5 m");
        send(&mut serial, "test 50 50");
        let forward_wheel = collect(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(1_500),
            "wheel_forward",
        );
        send(&mut serial, "test 0");
        let _ = collect(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(1_000),
            "wheel_settle",
        );
        send(&mut serial, "test 100 -100");
        let pivot = collect(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(1_500),
            "wheel_pivot",
        );
        send(&mut serial, "test 0");
        let forward_stats = PhaseStats::of(&forward_wheel);
        let pivot_stats = PhaseStats::of(&pivot);
        Some(WheelCheck {
            forward_left_rpm: forward_stats.left_rpm,
            forward_right_rpm: forward_stats.right_rpm,
            pivot_yaw_dps: pivot_stats.yaw_rate,
        })
    } else {
        None
    };

    send(&mut serial, "stream 0");
    send(&mut serial, "estop");
    let _ = csv.flush();

    print_report(&evaluate(baseline, forward, backward, yaw, pairing, wheel));
    println!("\nrecording: {csv_path}");
    println!("e-stop is latched; send `clear` before the next run.");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stats(pitch: f64, yaw: f64) -> PhaseStats {
        PhaseStats {
            pitch,
            yaw_rate: yaw,
            samples: 100,
            ..PhaseStats::default()
        }
    }

    fn motion(rate_dps: f64, samples: usize) -> Vec<Sample> {
        (0..samples)
            .map(|index| {
                let t_ms = index as f64 * 10.0;
                Sample {
                    t_ms,
                    pitch_deg: rate_dps * t_ms / 1000.0,
                    pitch_rate_dps: rate_dps,
                    ..Sample::default()
                }
            })
            .collect()
    }

    fn consistent_pairing() -> Option<(f64, f64)> {
        let mut samples = motion(40.0, 40);
        samples.extend(motion(-40.0, 40));
        pitch_rate_pairing(&samples)
    }

    #[test]
    fn consistent_setup_passes() {
        let report = evaluate(
            stats(0.0, 0.0),
            stats(12.0, 0.0),
            stats(-8.0, 0.0),
            stats(0.0, 25.0),
            consistent_pairing(),
            Some(WheelCheck {
                forward_left_rpm: 7.0,
                forward_right_rpm: -8.0,
                pivot_yaw_dps: -30.0,
            }),
        );
        assert!(report.pitch_ok);
        assert!(report.rate_ok);
        assert!(report.yaw_ok);
        assert_eq!(report.wheels_ok, Some(true));
        assert!(report.pass());
    }

    #[test]
    fn inverted_pitch_sign_fails() {
        let report = evaluate(
            stats(0.0, 0.0),
            stats(-12.0, 0.0),
            stats(8.0, 0.0),
            stats(0.0, 25.0),
            consistent_pairing(),
            None,
        );
        assert!(!report.pitch_ok);
        assert!(!report.pass());
    }

    #[test]
    fn mirrored_rate_channel_fails() {
        // The reported rate has the opposite sign to the angle derivative:
        // the classic inversion of a re-mounted sensor.
        let mut pitch = 0.0;
        let mut samples = Vec::new();
        for index in 0..80 {
            let t_ms = index as f64 * 10.0;
            let derivative = if index < 40 { 40.0 } else { -40.0 };
            pitch += derivative * 0.01;
            samples.push(Sample {
                t_ms,
                pitch_deg: pitch,
                pitch_rate_dps: -derivative,
                ..Sample::default()
            });
        }
        let pairing = pitch_rate_pairing(&samples);
        assert!(pairing.expect("samples").0 < 0.0);
        let report = evaluate(
            stats(0.0, 0.0),
            stats(12.0, 0.0),
            stats(-8.0, 0.0),
            stats(0.0, 25.0),
            pairing,
            None,
        );
        assert!(!report.rate_ok);
    }

    #[test]
    fn still_body_has_no_rate_verdict() {
        let samples = motion(0.0, 40);
        assert!(pitch_rate_pairing(&samples).is_none());
    }

    #[test]
    fn inverted_yaw_fails() {
        let report = evaluate(
            stats(0.0, 0.0),
            stats(12.0, 0.0),
            stats(-8.0, 0.0),
            stats(0.0, -25.0),
            consistent_pairing(),
            None,
        );
        assert!(!report.yaw_ok);
    }

    #[test]
    fn wheel_direction_is_checked() {
        let report = evaluate(
            stats(0.0, 0.0),
            stats(12.0, 0.0),
            stats(-8.0, 0.0),
            stats(0.0, 25.0),
            consistent_pairing(),
            Some(WheelCheck {
                forward_left_rpm: -7.0,
                forward_right_rpm: 8.0,
                pivot_yaw_dps: 30.0,
            }),
        );
        assert_eq!(report.wheels_ok, Some(false));
        assert!(!report.pass());
    }

    #[test]
    fn skipped_wheel_test_does_not_block() {
        let report = evaluate(
            stats(0.0, 0.0),
            stats(12.0, 0.0),
            stats(-8.0, 0.0),
            stats(0.0, 25.0),
            consistent_pairing(),
            None,
        );
        assert_eq!(report.wheels_ok, None);
        assert!(report.pass());
    }
}
