//! Bounded tilt-recovery tune for a short, precise hold.
//!
//! This is not a 10 s balance driver. For each runtime-only profile the
//! operator tilts the body to the requested angle with the rope attached,
//! arms, and then releases. The driver watches the gyro for body motion and
//! the mainboard wheel feedback for drive authority. A profile passes only
//! when the body stays within tolerance for the requested hold with the
//! wheels turning and without a guard, e-stop or disarm.
//!
//! Every profile ends with `home 0` and `estop`. No firmware default and no
//! persisted settings are changed; `zero`/`level` are never sent.
//!
//! Usage: tilt_recover_tune --run [--port PATH] [--tilt 10] [--hold-ms 2000]
//!                          [--tolerance 3] [--profiles 1,2,3]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115_200;
const HEADER: &str = "t_ms,profile,phase,step,pitch_deg,error_deg,gy_dps,left_rpm,right_rpm,torque_units,position_m,mode,estop,arm,wheel_active";
const START_TILT_TOLERANCE: f64 = 3.0;
const START_RATE_LIMIT: f64 = 30.0;
const RELEASE_RATE: f64 = 20.0;
const RELEASE_RPM: f64 = 5.0;
const RELEASE_TIMEOUT: Duration = Duration::from_millis(2_000);
const WHEEL_ACTIVE_RPM: f64 = 3.0;
const RATE_GUARD: f64 = 250.0;
const ERROR_GUARD: f64 = 30.0;
const TRAVEL_GUARD: f64 = 0.4;

#[derive(Clone, Copy)]
struct Profile {
    name: &'static str,
    pscale: f64,
    dscale: f64,
    slew: f64,
    db: f64,
    cal_left: f64,
    cal_right: f64,
    minimal: bool,
    dither: f64,
    feedforward: bool,
}

const PROFILES: &[Profile] = &[
    Profile {
        name: "full-min",
        pscale: 1.0,
        dscale: 1.0,
        slew: 10_000.0,
        db: 0.0,
        cal_left: 0.0,
        cal_right: 0.0,
        minimal: true,
        dither: 0.0,
        feedforward: false,
    },
    Profile {
        name: "host-min",
        pscale: 0.75,
        dscale: 0.60,
        slew: 1_500.0,
        db: 0.0,
        cal_left: 0.0,
        cal_right: 0.0,
        minimal: true,
        dither: 0.0,
        feedforward: false,
    },
    Profile {
        name: "full-cal",
        pscale: 1.0,
        dscale: 1.0,
        slew: 10_000.0,
        db: 0.20,
        cal_left: 35.0,
        cal_right: 45.0,
        minimal: true,
        dither: 0.0,
        feedforward: false,
    },
    Profile {
        name: "host-cal",
        pscale: 0.75,
        dscale: 0.60,
        slew: 1_500.0,
        db: 0.20,
        cal_left: 35.0,
        cal_right: 45.0,
        minimal: true,
        dither: 0.0,
        feedforward: false,
    },
    Profile {
        name: "full-ff",
        pscale: 1.0,
        dscale: 1.0,
        slew: 10_000.0,
        db: 0.0,
        cal_left: 0.0,
        cal_right: 0.0,
        minimal: false,
        dither: 0.0,
        feedforward: true,
    },
    Profile {
        name: "full-dither",
        pscale: 1.0,
        dscale: 1.0,
        slew: 10_000.0,
        db: 0.0,
        cal_left: 0.0,
        cal_right: 0.0,
        minimal: true,
        dither: 20.0,
        feedforward: false,
    },
];

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
    line.starts_with('T') && field(line, "step").is_some() && field(line, "pitch").is_some()
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
                            let error = pitch - num(&text, "lvl");
                            let left = num(&text, "L");
                            let right = num(&text, "R");
                            let active =
                                left.abs() > WHEEL_ACTIVE_RPM || right.abs() > WHEEL_ACTIVE_RPM;
                            let values = [
                                ctx.started.elapsed().as_millis().to_string(),
                                ctx.profile.to_owned(),
                                ctx.phase.to_owned(),
                                (num(&text, "step") as u64).to_string(),
                                format!("{pitch:.3}"),
                                format!("{error:.3}"),
                                format!("{:.3}", num(&text, "gy")),
                                left.to_string(),
                                right.to_string(),
                                format!("{:.1}", num(&text, "trq")),
                                format!("{:.4}", num(&text, "x")),
                                (num(&text, "mode") as u64).to_string(),
                                (num(&text, "estop") as u64).to_string(),
                                (num(&text, "arm") as u64).to_string(),
                                u8::from(active).to_string(),
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
    peak_error: f64,
    peak_rate: f64,
    wheel_samples: usize,
    sign_changes: usize,
    hold_ms: u64,
}

fn apply_profile(
    send: &impl Fn(&mut Box<dyn SerialPort>, &str),
    serial: &mut Box<dyn SerialPort>,
    profile: Profile,
) {
    send(serial, "clear");
    send(
        serial,
        &format!("ff {}", if profile.feedforward { "on" } else { "off" }),
    );
    send(serial, &format!("db {:.2}", profile.db));
    if profile.cal_left == 0.0 && profile.cal_right == 0.0 {
        send(serial, "cal off");
    } else {
        send(
            serial,
            &format!("cal {:.0} {:.0}", profile.cal_left, profile.cal_right),
        );
    }
    if profile.dither > 0.0 {
        send(serial, &format!("dither {:.0}", profile.dither));
    } else {
        send(serial, "dither off");
    }
    if profile.minimal {
        send(serial, "iscale 0");
        send(serial, "posg off");
        send(serial, "velg off");
        send(serial, "demlim off");
    } else {
        send(serial, "iscale 1");
        send(serial, "posg 0.40");
        send(serial, "velg 1.80");
        send(serial, "demlim 8");
    }
    send(serial, &format!("pscale {:.2}", profile.pscale));
    send(serial, &format!("dscale {:.2}", profile.dscale));
    send(serial, &format!("slew {:.0}", profile.slew));
    send(serial, "predict off");
    send(serial, "vmax 4");
    send(serial, "home 0");
}

struct ScoreCtx<'a> {
    serial: &'a mut Box<dyn SerialPort>,
    buffer: &'a mut Vec<u8>,
    csv: &'a mut BufWriter<File>,
    started: &'a Instant,
    profile: Profile,
    tilt: f64,
    tolerance: f64,
    hold: Duration,
}

fn score_profile(ctx: ScoreCtx<'_>) -> Verdict {
    let ScoreCtx {
        serial,
        buffer,
        csv,
        started,
        profile,
        tilt,
        tolerance,
        hold,
    } = ctx;
    let mut origin_x = 0.0;
    let mut armed = false;
    let mut start_error = 0.0;
    let mut start_rate = 0.0;
    for _ in 0..20 {
        let lines = pump(
            serial,
            buffer,
            Duration::from_millis(100),
            &mut PumpCtx {
                csv,
                started,
                profile: profile.name,
                phase: "arm",
            },
        );
        if let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) {
            origin_x = num(line, "x");
            start_error = (num(line, "pitch") - num(line, "lvl")).abs();
            start_rate = num(line, "gy").abs();
            armed = num(line, "arm") > 0.5;
            if armed {
                break;
            }
        }
    }
    if !armed {
        return Verdict {
            label: "arm refused",
            peak_error: start_error,
            peak_rate: 0.0,
            wheel_samples: 0,
            sign_changes: 0,
            hold_ms: 0,
        };
    }
    if (start_error - tilt).abs() > START_TILT_TOLERANCE || start_rate > START_RATE_LIMIT {
        return Verdict {
            label: "bad start",
            peak_error: start_error,
            peak_rate: 0.0,
            wheel_samples: 0,
            sign_changes: 0,
            hold_ms: 0,
        };
    }

    // The operator releases after arming. Confirm motion from either the gyro
    // or the wheel feedback; the gyro alone cannot prove wheel movement.
    let mut released = false;
    let release_start = Instant::now();
    while release_start.elapsed() < RELEASE_TIMEOUT {
        let lines = pump(
            serial,
            buffer,
            Duration::from_millis(40),
            &mut PumpCtx {
                csv,
                started,
                profile: profile.name,
                phase: "release",
            },
        );
        if let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) {
            let rate = num(line, "gy").abs();
            let rpm = num(line, "L").abs().max(num(line, "R").abs());
            if rate > RELEASE_RATE || rpm > RELEASE_RPM {
                released = true;
                break;
            }
        }
    }
    if !released {
        return Verdict {
            label: "no release",
            peak_error: start_error,
            peak_rate: 0.0,
            wheel_samples: 0,
            sign_changes: 0,
            hold_ms: 0,
        };
    }

    let mut peak_error = 0.0_f64;
    let mut peak_rate = 0.0_f64;
    let mut wheel_samples = 0_usize;
    let mut sign_changes = 0_usize;
    let mut last_sign = 0_i8;
    let mut hold_start: Option<Instant> = None;
    let mut best_hold = 0_u64;
    let trial_started = Instant::now();

    while trial_started.elapsed() < hold + Duration::from_millis(1_500) {
        let lines = pump(
            serial,
            buffer,
            Duration::from_millis(40),
            &mut PumpCtx {
                csv,
                started,
                profile: profile.name,
                phase: "trial",
            },
        );
        let Some(line) = lines.iter().rev().find(|line| is_telemetry(line)) else {
            continue;
        };
        let error = num(line, "pitch") - num(line, "lvl");
        let rate = num(line, "gy").abs();
        let travel = (num(line, "x") - origin_x).abs();
        let active =
            num(line, "L").abs() > WHEEL_ACTIVE_RPM || num(line, "R").abs() > WHEEL_ACTIVE_RPM;
        peak_error = peak_error.max(error.abs());
        peak_rate = peak_rate.max(rate);
        if active {
            wheel_samples += 1;
        }
        let sign = if error > 1.0 {
            1_i8
        } else if error < -1.0 {
            -1_i8
        } else {
            0
        };
        if sign != 0 && last_sign != 0 && sign != last_sign {
            sign_changes += 1;
        }
        if sign != 0 {
            last_sign = sign;
        }

        let guarded = num(line, "estop") > 0.5
            || num(line, "arm") < 0.5
            || num(line, "mode") > 3.5
            || rate > RATE_GUARD
            || error.abs() > ERROR_GUARD
            || travel > TRAVEL_GUARD;
        if guarded {
            return Verdict {
                label: "guard",
                peak_error,
                peak_rate,
                wheel_samples,
                sign_changes,
                hold_ms: best_hold,
            };
        }

        let settled = error.abs() <= tolerance && rate <= 20.0 && active;
        if settled {
            let start = *hold_start.get_or_insert_with(Instant::now);
            let elapsed = start.elapsed();
            best_hold = best_hold.max(elapsed.as_millis() as u64);
            if elapsed >= hold {
                return Verdict {
                    label: "held",
                    peak_error,
                    peak_rate,
                    wheel_samples,
                    sign_changes,
                    hold_ms: elapsed.as_millis() as u64,
                };
            }
        } else {
            hold_start = None;
        }
    }
    Verdict {
        label: "not held",
        peak_error,
        peak_rate,
        wheel_samples,
        sign_changes,
        hold_ms: best_hold,
    }
}

fn write_json(path: &PathBuf, results: &[(String, Verdict)]) {
    let Ok(file) = File::create(path) else {
        return;
    };
    let mut file = BufWriter::new(file);
    let _ = writeln!(file, "[");
    for (index, (name, verdict)) in results.iter().enumerate() {
        let comma = if index + 1 == results.len() { "" } else { "," };
        let _ = writeln!(
            file,
            "  {{\"profile\": \"{name}\", \"verdict\": \"{}\", \"peak_error_deg\": {:.2}, \"peak_rate_dps\": {:.1}, \"wheel_samples\": {}, \"sign_changes\": {}, \"hold_ms\": {}}}{comma}",
            verdict.label, verdict.peak_error, verdict.peak_rate, verdict.wheel_samples,
            verdict.sign_changes, verdict.hold_ms
        );
    }
    let _ = writeln!(file, "]");
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut port = None;
    let mut tilt = 10.0_f64;
    let mut hold_ms = 2_000_u64;
    let mut tolerance = 3.0_f64;
    let mut selection: Vec<usize> = (0..PROFILES.len()).collect();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--tilt" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    tilt = value.clamp(1.0, 30.0);
                }
            }
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(200, 10_000);
                }
            }
            "--tolerance" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    tolerance = value.clamp(0.5, 10.0);
                }
            }
            "--profiles" => {
                if let Some(value) = args.next() {
                    selection = value
                        .split(',')
                        .filter_map(|part| part.parse::<usize>().ok())
                        .filter(|index| *index < PROFILES.len())
                        .collect();
                }
            }
            "--help" | "-h" => {
                println!("usage: tilt_recover_tune --run [--port PATH] [--tilt 10] [--hold-ms 2000] [--tolerance 3] [--profiles 1,2,3]");
                return;
            }
            other => {
                eprintln!("unknown option: {other}");
                return;
            }
        }
    }
    if !run {
        eprintln!("refusing to run: pass --run after placing the robot on the ground with rope and e-stop ready");
        return;
    }
    if selection.is_empty() {
        eprintln!("no valid profile indices");
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

    let stem = format!("csv/jbot-tilt-tune-{}", timestamp_ms());
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
    let hold = Duration::from_millis(hold_ms);
    for index in selection {
        let profile = PROFILES[index];
        println!("\nSet the body to {tilt:.0} deg, rope attached, hands above the body, e-stop reachable.");
        println!("Press Enter to arm profile {index} ({})...", profile.name);
        let mut answer = String::new();
        if std::io::stdin().read_line(&mut answer).unwrap_or_default() == 0 {
            println!("stdin closed; stopping the grid");
            break;
        }
        apply_profile(&send, &mut serial, profile);
        send(&mut serial, "arm");
        println!("ARMED - release the body now");
        let verdict = score_profile(ScoreCtx {
            serial: &mut serial,
            buffer: &mut buffer,
            csv: &mut csv,
            started: &started,
            profile,
            tilt,
            tolerance,
            hold,
        });
        println!(
            "  {} -> {} (peak {:.1} deg, gyro {:.0} dps, wheel samples {}, sign changes {}, hold {} ms)",
            profile.name, verdict.label, verdict.peak_error, verdict.peak_rate,
            verdict.wheel_samples, verdict.sign_changes, verdict.hold_ms
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
                profile: profile.name,
                phase: "estop",
            },
        );
        results.push((profile.name.to_owned(), verdict));
        if results.last().is_some_and(|(_, v)| v.label == "held") {
            println!("profile held the tilt; stopping the grid");
            break;
        }
    }

    let _ = csv.flush();
    write_json(&json_path, &results);
    let winner = results
        .iter()
        .find(|(_, v)| v.label == "held")
        .map(|(n, _)| n.as_str())
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
    fn profile_grid_is_bounded() {
        assert!(PROFILES.len() >= 4);
        for profile in PROFILES {
            assert!((0.1..=1.0).contains(&profile.pscale));
            assert!((0.1..=1.0).contains(&profile.dscale));
            assert!(profile.slew >= 100.0);
        }
    }

    #[test]
    fn telemetry_parser_reads_the_fields() {
        let line =
            "T step=1 pitch=9.80 lvl=-0.10 gy=12.50 L=8 R=-7 trq=42 x=0.010 mode=1 estop=0 arm=1";
        assert!(is_telemetry(line));
        assert!((num(line, "pitch") - 9.8).abs() < 1e-6);
        assert!((num(line, "gy") - 12.5).abs() < 1e-6);
        assert_eq!(num(line, "L"), 8.0);
    }
}
