//! Host-side identification and parameter recommendation for the JBOT balance
//! controller.
//!
//! The firmware already contains every compensation mechanism needed for a
//! stable balance (per-wheel breakaway inverse, dead band, dither, derivative
//! scaling, feedforward). They are runtime-only and off at boot. What is
//! missing is measured values for those knobs, not another firmware image.
//!
//! This driver measures what can be measured through the Pico USB console and
//! derives a concrete command set. It never writes firmware, never changes a
//! safety contract, and always ends every phase with `home 0` and `estop`.
//!
//! Phases:
//!   1. baseline   - link, IMU and start-pose gates;
//!   2. spin       - free-wheel torque scale and per-wheel breakaway;
//!   3. hold       - settled holding torque around upright (feedforward);
//!   4. search     - optional bounded profile ladder with strict gates.
//!
//! Phase 4 is opt-in because it needs a repeatable release fixture. Phenomena
//! that need external instruments (phase current waveform, dead time, winding
//! temperature, true body inertia) are reported as manual, not guessed.
//!
//! Usage:
//!   identify_all --run --secured [--port PATH] [--out-dir csv]
//!                [--levels 39,48,60,75,99,120,150] [--angles -4,-2,0,2,4]
//!                [--hold-ms 600] [--search] [--trial-ms 4000]
//!                [--spin-only]
//!
//! `--spin-only` is the correct mode for a robot fixed on a stand with both
//! wheels free. It skips the balance phases, which require floor contact.

use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115_200;
/// Wheel inertia from `config/robot.toml`, kg m^2. Used only to convert a
/// measured hall acceleration into a torque for the units/Nm scale.
const WHEEL_INERTIA_KG_M2: f64 = 0.0078;
/// Below this hall speed a wheel is treated as stationary, rpm.
const REST_RPM: f64 = 3.0;
/// Above this hall speed the torque-speed envelope already distorts the
/// acceleration slope, revolutions per second (about 190 rpm).
const LOW_SPEED_RPS: f64 = 20.0;
/// Minimum angular acceleration that proves a wheel moved, rad/s^2.
const MIN_ALPHA_RPS2: f64 = 0.3;
/// A settled pitch window must stay inside this spread, degrees.
const SETTLE_TOL_DEG: f64 = 1.0;
/// Settled body rate, deg/s.
const SETTLE_RATE_DPS: f64 = 8.0;
/// Abort a phase above this body rate, deg/s.
const RUNAWAY_DPS: f64 = 250.0;
/// Abort before the body leaves the test zone, metres.
const TRAVEL_LIMIT_M: f64 = 0.4;
/// A lean with motionless wheels means an external hand is holding the body.
const SUPPORT_PITCH_DEG: f64 = 0.8;
const SUPPORT_STILL_SAMPLES: usize = 40;
/// Latest acceptable mainboard feedback age, ms.
const MAX_FEEDBACK_AGE_MS: f64 = 300.0;

/// Column names match `jbot_analysis::session` so the recording can be scored
/// by `jbot-analyze` and `spin-fit` without a conversion step.
const TELEMETRY_HEADER: &str = concat!(
    "t_ms,phase,step,loop_hz,imu_fresh,mb_ok,pitch_deg,level_deg,roll_deg,heading_deg,",
    "gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,arm,torque_units,",
    "test_left_units,test_right_units,ref_deg,demand_deg,trim_deg,position_m,mode,",
    "applied_left_units,applied_right_units"
);

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Telemetry {
    t_ms: u64,
    step: u64,
    loop_hz: f64,
    imu_ok: bool,
    mb_ok: bool,
    pitch: f64,
    level: f64,
    roll: f64,
    heading: f64,
    rate: f64,
    gx: f64,
    gz: f64,
    left_rpm: f64,
    right_rpm: f64,
    battery: f64,
    age_ms: f64,
    estop: f64,
    arm: f64,
    torque: f64,
    test_left: f64,
    test_right: f64,
    reference: f64,
    demand: f64,
    trim: f64,
    position: f64,
    mode: f64,
    applied_left: f64,
    applied_right: f64,
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

fn parse_telemetry(line: &str) -> Option<Telemetry> {
    if !line.starts_with("T ") || field(line, "step").is_none() || field(line, "pitch").is_none() {
        return None;
    }
    Some(Telemetry {
        t_ms: 0,
        step: num(line, "step") as u64,
        loop_hz: num(line, "loop"),
        imu_ok: field(line, "imu") == Some("ok"),
        mb_ok: field(line, "mb") == Some("ok"),
        pitch: num(line, "pitch"),
        level: num(line, "lvl"),
        roll: num(line, "roll"),
        heading: num(line, "hdg"),
        rate: num(line, "gy"),
        gx: num(line, "gx"),
        gz: num(line, "gz"),
        left_rpm: num(line, "L"),
        right_rpm: num(line, "R"),
        battery: num(line, "bat"),
        age_ms: num(line, "age"),
        estop: num(line, "estop"),
        arm: num(line, "arm"),
        torque: num(line, "trq"),
        test_left: num(line, "tsl"),
        test_right: num(line, "tsr"),
        reference: num(line, "ref"),
        demand: num(line, "dem"),
        trim: num(line, "trm"),
        position: num(line, "x"),
        mode: num(line, "mode"),
        applied_left: num(line, "al"),
        applied_right: num(line, "ar"),
    })
}

fn csv_row(sample: &Telemetry, phase: &str) -> String {
    [
        sample.t_ms.to_string(),
        phase.to_owned(),
        sample.step.to_string(),
        format!("{:.1}", sample.loop_hz),
        u8::from(sample.imu_ok).to_string(),
        u8::from(sample.mb_ok).to_string(),
        format!("{:.4}", sample.pitch),
        format!("{:.4}", sample.level),
        format!("{:.4}", sample.roll),
        format!("{:.4}", sample.heading),
        format!("{:.3}", sample.rate),
        format!("{:.3}", sample.gx),
        format!("{:.3}", sample.gz),
        format!("{:.2}", sample.left_rpm),
        format!("{:.2}", sample.right_rpm),
        format!("{:.2}", sample.battery),
        format!("{:.1}", sample.age_ms),
        (sample.estop as u64).to_string(),
        (sample.arm as u64).to_string(),
        format!("{:.1}", sample.torque),
        format!("{:.1}", sample.test_left),
        format!("{:.1}", sample.test_right),
        format!("{:.3}", sample.reference),
        format!("{:.3}", sample.demand),
        format!("{:.3}", sample.trim),
        format!("{:.4}", sample.position),
        (sample.mode as u64).to_string(),
        format!("{:.1}", sample.applied_left),
        format!("{:.1}", sample.applied_right),
    ]
    .join(",")
}

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    phase: &str,
) -> Vec<Telemetry> {
    let mut samples = Vec::new();
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline {
        let mut chunk = [0_u8; 1024];
        let read = serial.read(&mut chunk).unwrap_or_default();
        for &byte in &chunk[..read] {
            match byte {
                b'\r' => {}
                b'\n' => {
                    if let Ok(text) = std::str::from_utf8(buffer) {
                        if let Some(mut sample) = parse_telemetry(text.trim_end()) {
                            sample.t_ms = started.elapsed().as_millis() as u64;
                            let _ = writeln!(csv, "{}", csv_row(&sample, phase));
                            samples.push(sample);
                        }
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
    samples
}

fn send(serial: &mut Box<dyn SerialPort>, line: &str) {
    let _ = serial.write_all(format!("{line}\n").as_bytes());
    let _ = serial.flush();
}

fn median(mut values: Vec<f64>) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let middle = values.len() / 2;
    if values.len().is_multiple_of(2) {
        Some((values[middle - 1] + values[middle]) / 2.0)
    } else {
        Some(values[middle])
    }
}

/// Least-squares slope with intercept and coefficient of determination.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Fit {
    slope: f64,
    intercept: f64,
    r2: f64,
}

fn fit_line(points: &[(f64, f64)]) -> Option<Fit> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let sx: f64 = points.iter().map(|p| p.0).sum();
    let sy: f64 = points.iter().map(|p| p.1).sum();
    let sxx: f64 = points.iter().map(|p| p.0 * p.0).sum();
    let sxy: f64 = points.iter().map(|p| p.0 * p.1).sum();
    let det = n * sxx - sx * sx;
    if det.abs() < 1e-9 {
        return None;
    }
    let slope = (n * sxy - sx * sy) / det;
    let intercept = (sy - slope * sx) / n;
    let mean = sy / n;
    let mut ss_tot = 0.0;
    let mut ss_res = 0.0;
    for &(x, y) in points {
        ss_tot += (y - mean) * (y - mean);
        let residual = y - (slope * x + intercept);
        ss_res += residual * residual;
    }
    let r2 = if ss_tot > 1e-12 {
        1.0 - ss_res / ss_tot
    } else {
        0.0
    };
    Some(Fit {
        slope,
        intercept,
        r2,
    })
}

fn clamp(value: f64, low: f64, high: f64) -> f64 {
    if value.is_finite() {
        value.clamp(low, high)
    } else {
        low
    }
}

fn parse_list(text: &str) -> Vec<f64> {
    text.split(',')
        .filter_map(|part| part.trim().parse::<f64>().ok())
        .filter(|value| value.is_finite())
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
    serialport::available_ports()
        .unwrap_or_default()
        .iter()
        .find(|port| port.port_name.contains("JBOT_PICO"))
        .map(|port| port.port_name.clone())
        .unwrap_or_else(|| "/dev/cu.usbmodemJBOT_PICO_011".to_owned())
}

#[derive(Clone, Debug, Default, PartialEq)]
struct SpinResult {
    units_per_nm: f64,
    breakaway_left: f64,
    breakaway_right: f64,
    ratios: Vec<f64>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct HoldResult {
    /// Command units of common-mode torque per degree of settled pitch.
    units_per_degree: f64,
    intercept_units: f64,
    r2: f64,
    angles: Vec<(f64, f64)>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Trial {
    pscale: f64,
    dscale: f64,
    slew: f64,
    label: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
struct Recommendation {
    commands: Vec<String>,
    evidence: Vec<String>,
    manual: Vec<String>,
    confidence: &'static str,
}

/// Pure synthesis of runtime commands from whatever measurements succeeded.
/// Missing measurements degrade confidence; they never invent a number that
/// looks measured.
fn synthesize(
    spin: Option<&SpinResult>,
    hold: Option<&HoldResult>,
    best: Option<&Trial>,
) -> Recommendation {
    let mut commands = vec!["clear".to_owned(), "ff off".to_owned()];
    let mut evidence = Vec::new();
    let mut manual = Vec::new();

    let units_per_nm = spin.map_or(175.0, |result| result.units_per_nm);
    match spin {
        Some(spin) => {
            let left = clamp(spin.breakaway_left, 0.0, 150.0);
            let right = clamp(spin.breakaway_right, 0.0, 150.0);
            commands.push(format!("cal {left:.0} {right:.0}"));
            commands.push(format!("dither {:.0}", clamp(left.max(right), 0.0, 60.0)));
            let deadband_nm = spin.breakaway_left.max(spin.breakaway_right) / units_per_nm;
            commands.push(format!("db {:.2}", clamp(deadband_nm, 0.0, 0.6)));
            evidence.push(format!(
                "spin: {:.1} units/Nm, breakaway {:.0}/{:.0} units",
                units_per_nm, left, right
            ));
        }
        None => {
            commands.push("cal off".to_owned());
            commands.push("dither off".to_owned());
            commands.push("db 0.00".to_owned());
            evidence.push("spin: unavailable; compensation left off".to_owned());
        }
    }

    match hold {
        Some(hold) if hold.r2 >= 0.8 && hold.units_per_degree.is_finite() => {
            evidence.push(format!(
                "hold: {:.2} units/deg, intercept {:.1}, R2 {:.2}",
                hold.units_per_degree, hold.intercept_units, hold.r2
            ));
        }
        _ => {
            evidence.push(
                "hold: no reliable torque curve; keep the current +3.23 units/deg".to_owned(),
            );
        }
    }

    let profile = best.copied().unwrap_or(Trial {
        pscale: 0.25,
        dscale: 1.0,
        slew: 100.0,
        label: "safe bench default",
    });
    commands.push(format!("pscale {:.2}", clamp(profile.pscale, 0.0, 2.0)));
    commands.push(format!("dscale {:.2}", clamp(profile.dscale, 0.0, 2.0)));
    commands.push(format!("slew {:.0}", clamp(profile.slew, 20.0, 3000.0)));
    commands.push("vmax 2.00".to_owned());
    commands.push("home 0".to_owned());
    evidence.push(format!(
        "profile: {} (pscale {:.2}, dscale {:.2}, slew {:.0})",
        profile.label, profile.pscale, profile.dscale, profile.slew
    ));

    manual.push(
        "oscilloscope/current probe: phase waveform, 48-count dead time, current window".to_owned(),
    );
    manual
        .push("fixed pivot and repeatable release fixture before any disturbance gate".to_owned());
    manual.push(
        "wheel encoders for low-speed velocity; winding thermocouple for continuous torque"
            .to_owned(),
    );

    let confidence = if spin.is_some() && hold.is_some() && best.is_some() {
        "measured"
    } else if spin.is_some() {
        "partial"
    } else {
        "unverified"
    };
    Recommendation {
        commands,
        evidence,
        manual,
        confidence,
    }
}

fn print_recommendation(recommendation: &Recommendation) {
    println!("confidence: {}", recommendation.confidence);
    println!("evidence:");
    for line in &recommendation.evidence {
        println!("  - {line}");
    }
    println!("apply these console commands (do NOT arm automatically):");
    for command in &recommendation.commands {
        println!("  {command}");
    }
    println!("needs external equipment or a fixture:");
    for line in &recommendation.manual {
        println!("  - {line}");
    }
}

fn write_json(
    path: &Path,
    spin: Option<&SpinResult>,
    hold: Option<&HoldResult>,
    best: Option<&Trial>,
    recommendation: &Recommendation,
) {
    let Ok(file) = File::create(path) else {
        return;
    };
    let mut out = BufWriter::new(file);
    let _ = writeln!(out, "{{");
    let _ = writeln!(
        out,
        "  \"spin\": {},",
        json_option(spin.map(|s| format!(
            "{{\"units_per_nm\": {:.2}, \"breakaway_left\": {:.0}, \"breakaway_right\": {:.0}}}",
            s.units_per_nm, s.breakaway_left, s.breakaway_right
        )))
    );
    let _ = writeln!(
        out,
        "  \"hold\": {},",
        json_option(hold.map(|h| format!(
            "{{\"units_per_degree\": {:.2}, \"intercept_units\": {:.1}, \"r2\": {:.2}}}",
            h.units_per_degree, h.intercept_units, h.r2
        )))
    );
    let _ = writeln!(
        out,
        "  \"best_profile\": {},",
        json_option(best.map(|b| format!(
            "{{\"label\": \"{}\", \"pscale\": {:.2}, \"dscale\": {:.2}, \"slew\": {:.0}}}",
            b.label, b.pscale, b.dscale, b.slew
        )))
    );
    let _ = writeln!(out, "  \"confidence\": \"{}\",", recommendation.confidence);
    let _ = writeln!(out, "  \"commands\": [");
    for (index, command) in recommendation.commands.iter().enumerate() {
        let comma = if index + 1 == recommendation.commands.len() {
            ""
        } else {
            ","
        };
        let _ = writeln!(out, "    \"{command}\"{comma}");
    }
    let _ = writeln!(out, "  ]");
    let _ = writeln!(out, "}}");
    let _ = out.flush();
}

fn json_option(value: Option<String>) -> String {
    value.unwrap_or_else(|| "null".to_owned())
}

/// A quiet, supported start makes every later fit meaningless. Refuse it
/// instead of producing a confident-looking wrong number.
fn wait_baseline(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
) -> Result<Telemetry, String> {
    let mut still_run = 0_usize;
    let mut origin = 0.0_f64;
    let mut have_origin = false;
    for _ in 0..30 {
        let samples = pump(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(100),
            "baseline",
        );
        for sample in samples {
            if !have_origin {
                origin = sample.position;
                have_origin = true;
            }
            if !sample.mb_ok || !sample.imu_ok || sample.age_ms > MAX_FEEDBACK_AGE_MS {
                still_run = 0;
                continue;
            }
            if sample.left_rpm.abs() <= 2.0
                && sample.right_rpm.abs() <= 2.0
                && (sample.pitch - sample.level).abs() > SUPPORT_PITCH_DEG
            {
                still_run += 1;
            } else {
                still_run = 0;
            }
            if sample.estop < 0.5
                && (sample.pitch - sample.level).abs() <= 2.0
                && sample.rate.abs() <= SETTLE_RATE_DPS
                && (sample.position - origin).abs() <= 0.05
                && still_run <= SUPPORT_STILL_SAMPLES
            {
                return Ok(sample);
            }
        }
    }
    Err("baseline rejected: check link, IMU, upright pose and external support".to_owned())
}

/// Free-wheel step: acceleration early in a plateau gives torque through
/// `M = I * alpha`; the smallest moving command gives the breakaway.
fn measure_spin(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    levels: &[f64],
    hold_ms: u64,
) -> Result<SpinResult, String> {
    let mut ratios: Vec<(f64, f64)> = Vec::new();
    let mut left_breakaways = Vec::new();
    let mut right_breakaways = Vec::new();
    for &level in levels {
        send(serial, "clear");
        send(serial, &format!("test {level:.0}"));
        let samples = pump(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(hold_ms),
            "spin",
        );
        if samples.len() < 5 {
            continue;
        }
        // Mirrors `jbot_analysis::spinfit`: fit the early part of the plateau,
        // before the torque-speed envelope cuts the available torque, and use
        // both wheels. A fixed time window would include the 600 units/s
        // command ramp and inflate the scale by an order of magnitude.
        let mut window: Vec<&Telemetry> = Vec::new();
        for sample in &samples {
            let speed_rps = sample.left_rpm.abs().max(sample.right_rpm.abs()) * PI / 30.0;
            if speed_rps > LOW_SPEED_RPS {
                break;
            }
            window.push(sample);
        }
        if window.len() < 8 {
            window = samples.iter().take(15).collect();
        }
        if window.len() >= 8 {
            let to_rps = |rpm: f64| rpm * PI / 30.0;
            let left: Vec<(f64, f64)> = window
                .iter()
                .map(|s| (s.t_ms as f64 / 1000.0, to_rps(s.left_rpm)))
                .collect();
            let right: Vec<(f64, f64)> = window
                .iter()
                .map(|s| (s.t_ms as f64 / 1000.0, to_rps(s.right_rpm)))
                .collect();
            let slopes: Vec<f64> = [fit_line(&left), fit_line(&right)]
                .into_iter()
                .flatten()
                .map(|fit| fit.slope.abs())
                .collect();
            if !slopes.is_empty() {
                let alpha = slopes.iter().sum::<f64>() / slopes.len() as f64;
                if alpha >= MIN_ALPHA_RPS2 {
                    let torque = WHEEL_INERTIA_KG_M2 * alpha;
                    if torque > 1e-4 {
                        ratios.push((level, level / torque));
                    }
                }
            }
        }
        let last = samples.last().expect("non-empty samples");
        if last.left_rpm.abs() > REST_RPM {
            left_breakaways.push(level);
        }
        if last.right_rpm.abs() > REST_RPM {
            right_breakaways.push(level);
        }
        send(serial, "test 0");
        pump(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(400),
            "coast",
        );
    }
    // The affine breakaway inflates the ratio at small commands. Use the
    // highest commands, where a fixed offset is a small fraction of the total,
    // as the asymptotic torque scale (the same convention as WP0).
    let mut by_level = ratios.clone();
    by_level.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    let top: Vec<f64> = by_level
        .into_iter()
        .take(3)
        .map(|(_, ratio)| ratio)
        .collect();
    let units_per_nm = median(top).unwrap_or(0.0);
    let breakaway_left = left_breakaways
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    let breakaway_right = right_breakaways
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    // A free-wheel estimate is only asymptotic once the high commands ran.
    // Fewer plateaus means the console stopped streaming mid-sweep, and a
    // low-command-only fit is inflated by the breakaway term.
    let highest_level = ratios.iter().map(|(level, _)| *level).fold(0.0, f64::max);
    if ratios.len() < 6 || highest_level < 99.0 || !units_per_nm.is_finite() || units_per_nm <= 0.0
    {
        return Err(format!(
            "spin: incomplete sweep ({} plateaus, highest {highest_level:.0}); keeping the WP0 scale",
            ratios.len()
        ));
    }
    Ok(SpinResult {
        units_per_nm,
        breakaway_left: if breakaway_left.is_finite() {
            breakaway_left
        } else {
            0.0
        },
        breakaway_right: if breakaway_right.is_finite() {
            breakaway_right
        } else {
            0.0
        },
        ratios: ratios.into_iter().map(|(_, ratio)| ratio).collect(),
    })
}

/// Settled holding torque around upright. Feedforward is off so the recorded
/// value is the raw law, not the correction we want to derive.
fn measure_hold(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    angles: &[f64],
) -> Result<HoldResult, String> {
    send(serial, "clear");
    send(serial, "ff off");
    send(serial, "pscale 0.25");
    send(serial, "slew 100");
    send(serial, "level");
    send(serial, "arm");
    let mut armed = false;
    let mut origin = 0.0_f64;
    for _ in 0..12 {
        let samples = pump(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(100),
            "arm",
        );
        if let Some(last) = samples.last() {
            origin = last.position;
            armed = last.arm > 0.5;
        }
        if armed {
            break;
        }
    }
    if !armed {
        return Err("hold: arm refused".to_owned());
    }

    let mut points: Vec<(f64, f64)> = Vec::new();
    for &angle in angles {
        send(serial, &format!("home {angle:.1}"));
        let mut window_start = Instant::now();
        let mut window_min = f64::INFINITY;
        let mut window_max = f64::NEG_INFINITY;
        let mut state = 0_u8; // 0 = settling, 1 = recording
        let mut torque_sum = 0.0;
        let mut torque_n = 0_u64;
        let mut pitch_at_hold = 0.0;
        let deadline = Instant::now() + Duration::from_millis(4_000);
        while Instant::now() < deadline {
            let samples = pump(
                serial,
                buffer,
                csv,
                started,
                Duration::from_millis(40),
                "hold",
            );
            let Some(sample) = samples.last().copied() else {
                continue;
            };
            if sample.estop > 0.5
                || sample.mode > 3.5
                || sample.arm < 0.5
                || sample.rate.abs() > RUNAWAY_DPS
                || (sample.position - origin).abs() > TRAVEL_LIMIT_M
            {
                send(serial, "home 0");
                send(serial, "estop");
                return Err(format!("hold: guard at {angle:+.1} degrees"));
            }
            window_min = window_min.min(sample.pitch);
            window_max = window_max.max(sample.pitch);
            match state {
                0 => {
                    let expected = sample.level + angle;
                    if window_start.elapsed() >= Duration::from_millis(1_200)
                        && (sample.pitch - expected).abs() <= SETTLE_TOL_DEG
                        && window_max - window_min <= SETTLE_TOL_DEG
                        && sample.rate.abs() <= SETTLE_RATE_DPS
                    {
                        state = 1;
                        window_start = Instant::now();
                        torque_sum = 0.0;
                        torque_n = 0;
                        pitch_at_hold = sample.pitch;
                    }
                }
                _ => {
                    torque_sum += (sample.applied_left + sample.applied_right) / 2.0;
                    torque_n += 1;
                    if window_start.elapsed() >= Duration::from_millis(600) {
                        if torque_n > 0 {
                            points.push((pitch_at_hold, torque_sum / torque_n as f64));
                        }
                        break;
                    }
                }
            }
        }
    }
    send(serial, "home 0");
    send(serial, "estop");
    let fit = fit_line(&points).ok_or_else(|| "hold: fewer than three stable holds".to_owned())?;
    Ok(HoldResult {
        units_per_degree: fit.slope,
        intercept_units: fit.intercept,
        r2: fit.r2,
        angles: points,
    })
}

/// Optional bounded ladder. It only accepts a profile that settles with the
/// wheels turning and no guard, and it stops at the first guard.
fn search_profile(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    trial_ms: u64,
) -> Option<Trial> {
    let candidates = [
        (0.25, 1.0, 100.0, "hold-0.25"),
        (0.25, 0.6, 100.0, "hold-0.25-d06"),
        (0.30, 0.6, 100.0, "hold-0.30-d06"),
    ];
    let mut best = None;
    for &(pscale, dscale, slew, label) in &candidates {
        send(serial, "clear");
        send(serial, &format!("pscale {pscale:.2}"));
        send(serial, &format!("dscale {dscale:.2}"));
        send(serial, &format!("slew {slew:.0}"));
        send(serial, "home 0");
        send(serial, "arm");
        let mut quiet = false;
        for _ in 0..20 {
            let samples = pump(
                serial,
                buffer,
                csv,
                started,
                Duration::from_millis(50),
                "search-arm",
            );
            if let Some(last) = samples.last() {
                quiet = last.arm > 0.5 && last.rate.abs() <= SETTLE_RATE_DPS;
            }
            if quiet {
                break;
            }
        }
        if !quiet {
            send(serial, "estop");
            break;
        }
        let start = Instant::now();
        let mut settled = 0_u64;
        while start.elapsed() < Duration::from_millis(trial_ms) {
            let samples = pump(
                serial,
                buffer,
                csv,
                started,
                Duration::from_millis(40),
                "search",
            );
            let Some(last) = samples.last().copied() else {
                continue;
            };
            if last.estop > 0.5
                || last.arm < 0.5
                || last.mode > 3.5
                || last.rate.abs() > RUNAWAY_DPS
                || (last.pitch - last.level).abs() > 20.0
            {
                settled = 0;
                break;
            }
            if (last.pitch - last.level).abs() <= 3.0
                && last.rate.abs() <= SETTLE_RATE_DPS
                && (last.left_rpm.abs() > 2.0 || last.right_rpm.abs() > 2.0)
            {
                settled += 40;
            } else {
                settled = 0;
            }
            if settled >= 1_000 {
                break;
            }
        }
        let accepted = settled >= 1_000;
        send(serial, "home 0");
        send(serial, "estop");
        println!(
            "search {label}: {}",
            if accepted { "accepted" } else { "rejected" }
        );
        if accepted {
            best = Some(Trial {
                pscale,
                dscale,
                slew,
                label,
            });
        } else {
            break;
        }
    }
    best
}

fn run(args: &[String]) {
    let mut secured = false;
    let mut search = false;
    let mut spin_only = false;
    let mut port = None;
    let mut out_dir = PathBuf::from("csv");
    let mut levels_text = "39,48,60,75,99,120,150".to_owned();
    let mut angles_text = "-4,-2,0,2,4".to_owned();
    let mut hold_ms = 600_u64;
    let mut trial_ms = 4_000_u64;
    let mut index = 0;
    while index < args.len() {
        match args[index].as_str() {
            "--secured" => secured = true,
            "--search" => search = true,
            "--spin-only" => spin_only = true,
            "--port" => {
                index += 1;
                if index < args.len() {
                    port = Some(args[index].clone());
                }
            }
            "--out-dir" => {
                index += 1;
                if index < args.len() {
                    out_dir = PathBuf::from(&args[index]);
                }
            }
            "--levels" => {
                index += 1;
                if index < args.len() {
                    levels_text = args[index].clone();
                }
            }
            "--angles" => {
                index += 1;
                if index < args.len() {
                    angles_text = args[index].clone();
                }
            }
            "--hold-ms" => {
                index += 1;
                if let Some(value) = args.get(index).and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(200, 3_000);
                }
            }
            "--trial-ms" => {
                index += 1;
                if let Some(value) = args.get(index).and_then(|v| v.parse::<u64>().ok()) {
                    trial_ms = value.clamp(500, 20_000);
                }
            }
            _ => {}
        }
        index += 1;
    }
    if !secured {
        eprintln!("refusing to energise: pass --secured after fixing the robot in the test zone");
        return;
    }
    let levels: Vec<f64> = parse_list(&levels_text)
        .into_iter()
        .map(|value| clamp(value, 20.0, 200.0))
        .collect();
    let angles: Vec<f64> = parse_list(&angles_text)
        .into_iter()
        .map(|value| clamp(value, -5.0, 5.0))
        .collect();
    if levels.is_empty() || angles.is_empty() {
        eprintln!("no valid levels or angles");
        return;
    }
    let _ = std::fs::create_dir_all(&out_dir);

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
    let stem = out_dir.join(format!("jbot-identify-{}", timestamp_ms()));
    let csv_path = PathBuf::from(format!("{}.csv", stem.display()));
    let json_path = PathBuf::from(format!("{}.json", stem.display()));
    let mut csv = match File::create(&csv_path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {}: {error}", csv_path.display());
            return;
        }
    };
    let _ = writeln!(csv, "{TELEMETRY_HEADER}");

    let started = Instant::now();
    let mut buffer = Vec::new();
    send(&mut serial, "stream 200");
    send(&mut serial, "clear");
    let init = pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(300),
        "init",
    );
    // A body fixed on a stand is usually leaning a few degrees with its wheels
    // motionless. The baseline gate would read that as external support. `trim`
    // re-anchors the reference to this pose without writing NVM, which is safe
    // because --spin-only never arms the balance law.
    if spin_only {
        if let Some(sample) = init.last() {
            let pitch = clamp(sample.pitch, -20.0, 20.0);
            send(&mut serial, &format!("trim {pitch:.2}"));
            println!("spin-only: trimmed level to the current pose ({pitch:.2} deg)");
        }
    }
    if let Err(reason) = wait_baseline(&mut serial, &mut buffer, &mut csv, &started) {
        eprintln!("{reason}");
        send(&mut serial, "estop");
        send(&mut serial, "stream 0");
        let _ = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(300),
            "close",
        );
        let _ = csv.flush();
        return;
    }

    let spin = match measure_spin(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        &levels,
        hold_ms,
    ) {
        Ok(result) => {
            println!(
                "spin: {:.1} units/Nm, breakaway {:.0}/{:.0} units",
                result.units_per_nm, result.breakaway_left, result.breakaway_right
            );
            Some(result)
        }
        Err(reason) => {
            println!("spin skipped: {reason}");
            None
        }
    };

    let hold = if spin_only {
        println!("hold skipped: --spin-only (wheels must touch the floor for balance)");
        None
    } else {
        match measure_hold(&mut serial, &mut buffer, &mut csv, &started, &angles) {
            Ok(result) => {
                println!(
                    "hold: {:.2} units/deg, intercept {:.1}, R2 {:.2}",
                    result.units_per_degree, result.intercept_units, result.r2
                );
                Some(result)
            }
            Err(reason) => {
                println!("hold skipped: {reason}");
                None
            }
        }
    };

    let best = if search && !spin_only {
        search_profile(&mut serial, &mut buffer, &mut csv, &started, trial_ms)
    } else {
        None
    };

    send(&mut serial, "home 0");
    send(&mut serial, "estop");
    // Stop the 200 Hz stream and drain before closing. Closing a port while
    // telemetry is still being written parks the Pico console task on some
    // firmware versions and the next session gets no replies at all.
    send(&mut serial, "stream 0");
    let _ = pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(1_500),
        "close",
    );
    let _ = csv.flush();

    let recommendation = synthesize(spin.as_ref(), hold.as_ref(), best.as_ref());
    print_recommendation(&recommendation);
    write_json(
        &json_path,
        spin.as_ref(),
        hold.as_ref(),
        best.as_ref(),
        &recommendation,
    );
    println!(
        "recording: {} and {}",
        csv_path.display(),
        json_path.display()
    );
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|arg| arg == "--help" || arg == "-h") {
        println!("usage: identify_all --run --secured [--port PATH] [--out-dir csv] [--levels 39,48,60,75,99,120,150] [--angles -4,-2,0,2,4] [--hold-ms 600] [--search] [--trial-ms 4000] [--spin-only]");
        return;
    }
    if args.first().map(String::as_str) == Some("--run") {
        run(&args[1..]);
        return;
    }
    eprintln!("nothing to do: pass --run --secured (or --help)");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_parser_reads_the_console_fields() {
        let line = "T step=12 loop=200 imu=ok mb=ok pitch=1.5 lvl=0.5 gy=3.2 L=40 R=-20 trq=9 al=10 ar=-5 ref=0 dem=0 trm=0 x=0.1 mode=1 estop=0 arm=1 age=3 bat=39.5";
        let sample = parse_telemetry(line).expect("telemetry");
        assert_eq!(sample.step, 12);
        assert!(sample.imu_ok && sample.mb_ok);
        assert_eq!(sample.rate, 3.2);
        assert_eq!((sample.applied_left + sample.applied_right) / 2.0, 2.5);
        assert!(parse_telemetry("ok ping").is_none());
    }

    #[test]
    fn line_fit_recovers_a_known_line() {
        let points = [(0.0, 1.0), (1.0, 3.0), (2.0, 5.0)];
        let fit = fit_line(&points).expect("fit");
        assert!((fit.slope - 2.0).abs() < 1e-9);
        assert!((fit.intercept - 1.0).abs() < 1e-9);
        assert!(fit.r2 > 0.99);
    }

    #[test]
    fn median_handles_even_and_odd_sets() {
        assert_eq!(median(vec![3.0, 1.0, 2.0]), Some(2.0));
        assert_eq!(median(vec![4.0, 1.0, 3.0, 2.0]), Some(2.5));
        assert_eq!(median(Vec::new()), None);
    }

    #[test]
    fn synthesis_recommends_measured_compensation() {
        let spin = SpinResult {
            units_per_nm: 175.0,
            breakaway_left: 35.0,
            breakaway_right: 50.0,
            ratios: Vec::new(),
        };
        let hold = HoldResult {
            units_per_degree: 3.23,
            intercept_units: 0.0,
            r2: 0.94,
            angles: Vec::new(),
        };
        let best = Trial {
            pscale: 0.25,
            dscale: 0.6,
            slew: 100.0,
            label: "hold-0.25-d06",
        };
        let result = synthesize(Some(&spin), Some(&hold), Some(&best));
        assert_eq!(result.confidence, "measured");
        assert!(result.commands.iter().any(|c| c == "cal 35 50"));
        assert!(result.commands.iter().any(|c| c == "pscale 0.25"));
        assert!(result.commands.iter().any(|c| c.starts_with("db ")));
    }

    #[test]
    fn synthesis_without_measurements_is_unverified() {
        let result = synthesize(None, None, None);
        assert_eq!(result.confidence, "unverified");
        assert!(result.commands.iter().any(|c| c == "cal off"));
        assert!(result.commands.iter().any(|c| c == "db 0.00"));
    }

    #[test]
    fn csv_header_matches_row_width() {
        let row = csv_row(&Telemetry::default(), "test");
        assert_eq!(TELEMETRY_HEADER.split(',').count(), row.split(',').count());
    }

    #[test]
    fn list_parser_drops_invalid_values() {
        assert_eq!(parse_list("39,48,bad,60"), vec![39.0, 48.0, 60.0]);
    }
}
