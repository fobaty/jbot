//! Bounded automatic profile search for recovery from a tilt to `home`.
//!
//! This driver never changes firmware. It applies runtime-only profiles over
//! the USB console and scores each attempt against fixed safety gates. It
//! preserves the existing `zero`/`level` calibration: it never sends `zero` or
//! `level`, so a tilt relative to an upright reference stays a tilt.
//!
//! The robot must start tilted by at most 30 degrees, held with a rope and
//! hands on the body, inside a 0.5 m test zone. Every profile ends with
//! `home 0` and `estop`. A profile is accepted only when the body settles
//! inside 2 degrees of `home` with the wheels actually turning and no guard.
//!
//! Usage: auto_tune --run [--port PATH] [--profiles 0.25:100,0.30:300]
//!                  [--cal 50 50] [--dither 10] [--db 0] [--vmax 2]
//!                  [--trial-ms 5000] [--dscale 1.0]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115_200;
const HEADER: &str = "t_ms,profile,phase,step,pitch_deg,level_deg,error_deg,gy_dps,left_rpm,right_rpm,torque_units,applied_units,position_m,mode,estop,arm";
const SUPPORT_PITCH_DEG: f64 = 0.8;
const SUPPORT_STILL_SAMPLES: usize = 40;
const RATE_LIMIT_DPS: f64 = 250.0;
const PITCH_LIMIT_DEG: f64 = 30.0;
const TRAVEL_LIMIT_M: f64 = 0.4;
const SETTLE_ERROR_DEG: f64 = 2.0;
const SETTLE_RATE_DPS: f64 = 8.0;
const SETTLE_MS: u64 = 500;

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

fn parse_profiles(text: &str) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for part in text.split(',') {
        let Some((left, right)) = part.split_once(':') else {
            continue;
        };
        let (Ok(pscale), Ok(slew)) = (left.parse::<f64>(), right.parse::<f64>()) else {
            continue;
        };
        let entry = (pscale.clamp(0.1, 1.0), slew.clamp(20.0, 3000.0));
        if !out.contains(&entry) {
            out.push(entry);
        }
    }
    out
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

struct PumpCtx<'a> {
    csv: &'a mut BufWriter<File>,
    started: &'a Instant,
    profile: &'a str,
    phase: &'a str,
}

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    duration: Duration,
    ctx: &mut PumpCtx<'_>,
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
                            let pitch = num(&text, "pitch");
                            let level = num(&text, "lvl");
                            let applied = (num(&text, "al") + num(&text, "ar")) / 2.0;
                            let values = [
                                ctx.started.elapsed().as_millis().to_string(),
                                ctx.profile.to_owned(),
                                ctx.phase.to_owned(),
                                (num(&text, "step") as u64).to_string(),
                                format!("{pitch:.3}"),
                                format!("{level:.3}"),
                                format!("{:.3}", pitch - level),
                                format!("{:.3}", num(&text, "gy")),
                                num(&text, "L").to_string(),
                                num(&text, "R").to_string(),
                                format!("{:.1}", num(&text, "trq")),
                                format!("{applied:.1}"),
                                format!("{:.4}", num(&text, "x")),
                                (num(&text, "mode") as u64).to_string(),
                                (num(&text, "estop") as u64).to_string(),
                                (num(&text, "arm") as u64).to_string(),
                            ];
                            let _ = writeln!(ctx.csv, "{}", values.join(","));
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

#[derive(Clone, Copy)]
struct Verdict {
    label: &'static str,
    peak_error_deg: f64,
    peak_rate_dps: f64,
    travel_m: f64,
    samples: usize,
}

fn score_profile(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    profile: &str,
    trial_ms: u64,
) -> Verdict {
    let mut origin_x = 0.0;
    let mut armed = false;
    let mut start_error = 0.0_f64;
    for _ in 0..10 {
        let lines = pump(
            serial,
            buffer,
            Duration::from_millis(100),
            &mut PumpCtx {
                csv,
                started,
                profile,
                phase: "arm",
            },
        );
        if let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) {
            origin_x = num(line, "x");
            start_error = (num(line, "pitch") - num(line, "lvl")).abs();
            armed = num(line, "arm") > 0.5;
            if armed {
                break;
            }
        }
    }
    if !armed {
        return Verdict {
            label: "arm refused",
            peak_error_deg: 0.0,
            peak_rate_dps: 0.0,
            travel_m: 0.0,
            samples: 0,
        };
    }
    if start_error > PITCH_LIMIT_DEG {
        return Verdict {
            label: "not upright",
            peak_error_deg: start_error,
            peak_rate_dps: 0.0,
            travel_m: 0.0,
            samples: 0,
        };
    }
    println!("ARMED - release the body now");
    pump(
        serial,
        buffer,
        Duration::from_millis(300),
        &mut PumpCtx {
            csv,
            started,
            profile,
            phase: "release",
        },
    );

    let mut peak_error = 0.0_f64;
    let mut peak_rate = 0.0_f64;
    let mut last_travel = 0.0_f64;
    let mut still_run = 0_usize;
    let mut supported = false;
    let mut settle_start: Option<Instant> = None;
    let mut samples = 0_usize;
    let started_trial = Instant::now();
    while started_trial.elapsed() < Duration::from_millis(trial_ms) {
        let lines = pump(
            serial,
            buffer,
            Duration::from_millis(40),
            &mut PumpCtx {
                csv,
                started,
                profile,
                phase: "trial",
            },
        );
        let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) else {
            continue;
        };
        samples += 1;
        let error = (num(line, "pitch") - num(line, "lvl")).abs();
        let rate = num(line, "gy").abs();
        let travel = (num(line, "x") - origin_x).abs();
        let left = num(line, "L").abs();
        let right = num(line, "R").abs();
        peak_error = peak_error.max(error);
        peak_rate = peak_rate.max(rate);
        last_travel = travel;

        if error > SUPPORT_PITCH_DEG && left <= 2.0 && right <= 2.0 {
            still_run += 1;
            if still_run > SUPPORT_STILL_SAMPLES {
                supported = true;
            }
        } else {
            still_run = 0;
        }
        // Mode 3 is `recover`: a tilt start is supposed to enter it. Only the
        // fall latch (mode 4), e-stop, disarm, rate and travel are guards.
        let guarded = num(line, "estop") > 0.5
            || num(line, "arm") < 0.5
            || num(line, "mode") > 3.5
            || rate > RATE_LIMIT_DPS
            || error > PITCH_LIMIT_DEG
            || travel > TRAVEL_LIMIT_M;
        if guarded {
            return Verdict {
                label: if supported {
                    "guard after support"
                } else {
                    "guard"
                },
                peak_error_deg: peak_error,
                peak_rate_dps: peak_rate,
                travel_m: travel,
                samples,
            };
        }
        let settled =
            error <= SETTLE_ERROR_DEG && rate <= SETTLE_RATE_DPS && (left > 2.0 || right > 2.0);
        if settled {
            let start = *settle_start.get_or_insert_with(Instant::now);
            if start.elapsed() >= Duration::from_millis(SETTLE_MS) {
                return Verdict {
                    label: "recovered",
                    peak_error_deg: peak_error,
                    peak_rate_dps: peak_rate,
                    travel_m: travel,
                    samples,
                };
            }
        } else {
            settle_start = None;
        }
    }
    Verdict {
        label: if supported { "supported" } else { "timeout" },
        peak_error_deg: peak_error,
        peak_rate_dps: peak_rate,
        travel_m: last_travel,
        samples,
    }
}

fn write_json(path: &PathBuf, results: &[(String, Verdict)]) {
    let Ok(file) = File::create(path) else {
        return;
    };
    let mut file = BufWriter::new(file);
    let _ = writeln!(file, "[");
    for (index, (profile, verdict)) in results.iter().enumerate() {
        let comma = if index + 1 == results.len() { "" } else { "," };
        let _ = writeln!(
            file,
            "  {{\"profile\": \"{profile}\", \"verdict\": \"{}\", \"peak_error_deg\": {:.2}, \"peak_rate_dps\": {:.1}, \"travel_m\": {:.3}, \"samples\": {}}}{comma}",
            verdict.label,
            verdict.peak_error_deg,
            verdict.peak_rate_dps,
            verdict.travel_m,
            verdict.samples
        );
    }
    let _ = writeln!(file, "]");
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut port = None;
    let mut profiles_text = "0.25:100,0.25:300,0.30:300,0.40:300".to_owned();
    let mut cal = (0.0_f64, 0.0_f64);
    let mut dither = 0.0_f64;
    let mut db = 0.0_f64;
    let mut vmax = 2.0_f64;
    let mut dscale = 1.0_f64;
    let mut trial_ms = 5000_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--profiles" => {
                if let Some(value) = args.next() {
                    profiles_text = value;
                }
            }
            "--cal" => {
                let left = args.next().and_then(|v| v.parse::<f64>().ok());
                let right = args.next().and_then(|v| v.parse::<f64>().ok());
                match (left, right) {
                    (Some(left), Some(right)) => {
                        cal = (left.clamp(0.0, 150.0), right.clamp(0.0, 150.0))
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
            "--db" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    db = value.clamp(0.0, 0.6);
                }
            }
            "--vmax" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    vmax = value.clamp(0.2, 60.0);
                }
            }
            "--dscale" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    dscale = value.clamp(0.1, 2.0);
                }
            }
            "--trial-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    trial_ms = value.clamp(500, 30_000);
                }
            }
            "--help" | "-h" => {
                println!("usage: auto_tune --run [--port PATH] [--profiles 0.25:100,0.30:300] [--cal LEFT RIGHT] [--dither 10] [--db 0] [--vmax 2] [--trial-ms 5000] [--dscale 1.0]");
                return;
            }
            other => {
                eprintln!("unknown option: {other}");
                return;
            }
        }
    }
    if !run {
        eprintln!("refusing to run: pass --run after tilting the robot by at most 30 degrees");
        return;
    }
    let profiles = parse_profiles(&profiles_text);
    if profiles.is_empty() {
        eprintln!("no valid profiles in --profiles");
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

    let stem = format!("csv/jbot-autotune-{}", timestamp_ms());
    let csv_path = PathBuf::from(format!("{stem}.csv"));
    let json_path = PathBuf::from(format!("{stem}.json"));
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
        Duration::from_millis(300),
        &mut PumpCtx {
            csv: &mut csv,
            started: &started,
            profile: "init",
            phase: "init",
        },
    );

    let mut results = Vec::new();
    for (pscale, slew) in profiles {
        let label = format!("p{pscale:.2}_s{slew:.0}");
        println!("profile {label}: pscale {pscale:.2}, dscale {dscale:.2}, slew {slew:.0}");
        send(&mut serial, "clear");
        send(&mut serial, "ff off");
        send(&mut serial, &format!("pscale {pscale:.2}"));
        send(&mut serial, &format!("dscale {dscale:.2}"));
        send(&mut serial, &format!("slew {slew:.0}"));
        send(&mut serial, &format!("db {db:.2}"));
        send(&mut serial, &format!("cal {:.0} {:.0}", cal.0, cal.1));
        if dither > 0.0 {
            send(&mut serial, &format!("dither {dither:.0}"));
        } else {
            send(&mut serial, "dither off");
        }
        send(&mut serial, &format!("vmax {vmax:.2}"));
        send(&mut serial, "home 0");
        send(&mut serial, "arm");

        let verdict = score_profile(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            &label,
            trial_ms,
        );
        println!(
            "  {} -> {} (peak error {:.1} deg, rate {:.0} dps, travel {:.3} m)",
            label, verdict.label, verdict.peak_error_deg, verdict.peak_rate_dps, verdict.travel_m
        );
        let needs_reset = matches!(
            verdict.label,
            "guard" | "guard after support" | "arm refused" | "not upright"
        );
        send(&mut serial, "home 0");
        send(&mut serial, "estop");
        pump(
            &mut serial,
            &mut buffer,
            Duration::from_millis(300),
            &mut PumpCtx {
                csv: &mut csv,
                started: &started,
                profile: &label,
                phase: "estop",
            },
        );
        results.push((label, verdict));
        if needs_reset {
            println!("stopping the grid: stand the robot up and rerun for the next profile");
            break;
        }
    }

    let _ = csv.flush();
    write_json(&json_path, &results);
    let winner = results
        .iter()
        .find(|(_, verdict)| verdict.label == "recovered")
        .map(|(label, _)| label.as_str())
        .unwrap_or("none");
    println!("best profile: {winner}");
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
    fn profiles_are_clamped_and_deduplicated() {
        let parsed = parse_profiles("0.25:100,0.30:300,0.25:100,bad,2.0:1");
        assert_eq!(parsed, vec![(0.25, 100.0), (0.30, 300.0), (1.0, 20.0)]);
    }

    #[test]
    fn telemetry_parser_ignores_replies() {
        assert!(is_telemetry("T step=1 pitch=2.0 lvl=1.0"));
        assert!(!is_telemetry("ok ping"));
        assert_eq!(num("T step=1 lvl=1.5", "lvl"), 1.5);
    }
}
