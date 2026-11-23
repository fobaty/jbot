//! Command-to-torque latency and GP10 cadence from telemetry, no firmware changes.
//!
//! The Pico toggles GP10 once per 5 ms control tick and reports a `step`
//! counter in every telemetry line. This driver sends small, harmless `test`
//! requests with the wheels free, times the Pico's `ok test` reply, and watches
//! `tsl` (applied spin torque) and wheel RPM. It derives:
//!
//! * command -> applied torque in control ticks, by comparing the step at the
//!   reply with the step at the first nonzero `tsl`;
//! * applied torque -> wheel motion, from the first hall feedback response;
//! * GP10 cadence proxy, from `step` against host time and the `loop` field.
//!
//! The spin path shares the host link, control tick and mainboard UART with the
//! balance law, but uses its own fixed 600 units/s ramp. It needs no `arm`.
//! Every run ends with `estop`.
//!
//! Usage: latency_cal --run [--port PATH] [--levels 20,40,60] [--hold-ms 500]
//!
//! Safety: rope attached, hands on the body, wheels free, test zone at most
//! 0.5 m. Aborts unless mainboard feedback is fresh and e-stop is clear.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const TICK_MS: f64 = 5.0;
const HEADER: &str = "t_ms,step,loop_hz,pitch_deg,gy_dps,left_rpm,right_rpm,bat_v,age_ms,estop,tsl_units,tsr_units,trq_units,position_m,mode,opm,phase,level_units";
/// Applied spin torque above which the Pico is considered to have answered.
const APPLIED_EPS: f64 = 0.5;
/// Hall feedback above which the wheels are considered to have moved, rpm.
const RPM_EPS: f64 = 2.0;
/// Mainboard feedback age above which the link is not usable, ms.
const MAX_FB_AGE_MS: f64 = 100.0;

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

#[derive(Clone)]
struct Sample {
    t_ms: f64,
    step: f64,
    line: String,
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

/// Read telemetry into `csv` for `duration`, returning samples and the host
/// time (relative to `started`) of the first command reply seen.
fn listen(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    phase: &str,
    level: f64,
) -> (Vec<Sample>, Option<f64>) {
    let mut out = Vec::new();
    let mut reply_at = None;
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
                            let t_ms = started.elapsed().as_secs_f64() * 1000.0;
                            let n = |key| num(&text, key);
                            let row = [
                                format!("{t_ms:.3}"),
                                (n("step") as u64).to_string(),
                                (n("loop") as u64).to_string(),
                                format!("{:.4}", n("pitch")),
                                format!("{:.4}", n("gy")),
                                n("L").to_string(),
                                n("R").to_string(),
                                format!("{:.2}", n("bat")),
                                (n("age") as u64).to_string(),
                                (n("estop") as u64).to_string(),
                                format!("{:.1}", n("tsl")),
                                format!("{:.1}", n("tsr")),
                                format!("{:.1}", n("trq")),
                                format!("{:.4}", n("x")),
                                (n("mode") as u64).to_string(),
                                (n("opm") as u64).to_string(),
                                phase.to_owned(),
                                format!("{:.1}", level),
                            ];
                            let _ = writeln!(csv, "{}", row.join(","));
                            out.push(Sample {
                                t_ms,
                                step: num(&text, "step"),
                                line: text,
                            });
                        } else if text.starts_with("ok test") && reply_at.is_none() {
                            reply_at = Some(started.elapsed().as_secs_f64() * 1000.0);
                        }
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
    (out, reply_at)
}

/// Interpolate the firmware step counter at `t_ms` using telemetry samples on
/// both sides of it. The reply and the telemetry traverse the same USB channel,
/// so this cancels most of the host-side latency.
fn step_at(samples: &[Sample], t_ms: f64) -> Option<f64> {
    if samples.is_empty() {
        return None;
    }
    if t_ms <= samples[0].t_ms {
        return Some(samples[0].step);
    }
    if t_ms >= samples[samples.len() - 1].t_ms {
        return Some(samples[samples.len() - 1].step);
    }
    for pair in samples.windows(2) {
        if pair[0].t_ms <= t_ms && t_ms <= pair[1].t_ms {
            let span = pair[1].t_ms - pair[0].t_ms;
            if span <= 0.0 {
                return Some(pair[0].step);
            }
            let k = (t_ms - pair[0].t_ms) / span;
            return Some(pair[0].step + k * (pair[1].step - pair[0].step));
        }
    }
    None
}

struct ResultRow {
    level: f64,
    reply_ms: f64,
    torque_ms: f64,
    torque_upper_ms: f64,
    wheel_ms: f64,
    loop_min: f64,
    loop_max: f64,
    tick_rate_hz: f64,
}

/// Delay from the reply (telemetry-resolution lower bound) and from the host
/// send time (upper bound, includes the host-to-device USB leg).
fn analyze(
    level: f64,
    samples: &[Sample],
    reply_at: Option<f64>,
    send_ms: f64,
) -> Option<ResultRow> {
    let reply_at = reply_at?;
    let step_reply = step_at(samples, reply_at)?;
    let step_send = step_at(samples, send_ms)?;
    let torque = samples
        .iter()
        .find(|sample| num(&sample.line, "tsl").abs() > APPLIED_EPS)?;
    let wheel = samples.iter().find(|sample| {
        num(&sample.line, "L")
            .abs()
            .max(num(&sample.line, "R").abs())
            > RPM_EPS
    })?;
    let loops: Vec<f64> = samples
        .iter()
        .map(|sample| num(&sample.line, "loop"))
        .collect();
    let span_ms = samples[samples.len() - 1].t_ms - samples[0].t_ms;
    let delta_steps = samples[samples.len() - 1].step - samples[0].step;
    let tick_rate_hz = if span_ms > 0.0 {
        delta_steps * 1000.0 / span_ms
    } else {
        0.0
    };
    Some(ResultRow {
        level,
        reply_ms: reply_at,
        torque_ms: (torque.step - step_reply) * TICK_MS,
        torque_upper_ms: (torque.step - step_send) * TICK_MS,
        wheel_ms: (wheel.step - torque.step) * TICK_MS,
        loop_min: loops.iter().copied().fold(f64::INFINITY, f64::min),
        loop_max: loops.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        tick_rate_hz,
    })
}

/// Wait until both wheels have coasted to rest so each probe starts clean.
fn wait_rest(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    level: f64,
) -> bool {
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut still_since: Option<Instant> = None;
    while Instant::now() < deadline {
        let (samples, _) = listen(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(50),
            "settle",
            level,
        );
        let Some(last) = samples.last() else {
            continue;
        };
        let rpm = num(&last.line, "L").abs().max(num(&last.line, "R").abs());
        if rpm < RPM_EPS {
            if still_since.is_none() {
                still_since = Some(Instant::now());
            }
            if still_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(300)) {
                return true;
            }
        } else {
            still_since = None;
        }
    }
    false
}

fn write_json(path: &PathBuf, results: &[ResultRow], abort: Option<&str>) {
    let mut file = match File::create(path) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create {}: {error}", path.display());
            return;
        }
    };
    let _ = writeln!(file, "{{");
    let _ = writeln!(
        file,
        "  \"abort\": {},",
        abort.map_or_else(|| "null".to_owned(), |reason| format!("\"{reason}\""))
    );
    let _ = writeln!(file, "  \"results\": [");
    for (index, row) in results.iter().enumerate() {
        let comma = if index + 1 == results.len() { "" } else { "," };
        let _ = writeln!(
            file,
            "    {{\"level_units\": {:.0}, \"reply_ms\": {:.2}, \"cmd_to_torque_ms\": {:.1}, \"cmd_to_torque_upper_ms\": {:.1}, \"torque_to_wheel_ms\": {:.1}, \"loop_min\": {:.0}, \"loop_max\": {:.0}, \"tick_rate_hz\": {:.1}}}{comma}",
            row.level,
            row.reply_ms,
            row.torque_ms,
            row.torque_upper_ms,
            row.wheel_ms,
            row.loop_min,
            row.loop_max,
            row.tick_rate_hz
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
    let mut levels = vec![20.0, 40.0, 60.0];
    let mut hold_ms = 500_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--levels" => {
                if let Some(value) = args.next() {
                    levels = value
                        .split(',')
                        .filter_map(|part| part.trim().parse::<f64>().ok())
                        .filter(|value| (1.0..=200.0).contains(value))
                        .collect();
                }
            }
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(200, 4000);
                }
            }
            "--help" | "-h" => {
                println!(
                    "usage: latency_cal --run [--port PATH] [--levels 20,40,60] [--hold-ms 500]"
                );
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
    if levels.is_empty() {
        eprintln!("no valid torque levels");
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

    let stem = format!("csv/jbot-latency-{}", timestamp_ms());
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
    // The console gates writes on DTR with a two-second grace at session
    // start. The builder asserts DTR on open; allow the full grace before probing.
    let started = Instant::now();
    let mut buffer = Vec::new();
    send(&mut serial, "stream 200");
    send(&mut serial, "clear");
    let (baseline, _) = listen(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(2500),
        "init",
        0.0,
    );

    let link_ok = baseline.last().is_some_and(|sample| {
        let line = &sample.line;
        field(line, "mb") == Some("ok")
            && num(line, "age") < MAX_FB_AGE_MS
            && num(line, "estop") < 0.5
            && num(line, "bat") > 1.0
    });
    let mut abort = None;
    if !link_ok {
        abort = Some("mainboard feedback not fresh or e-stop latched");
        eprintln!("{}; no torque sent", abort.unwrap_or_default());
    }

    let mut results = Vec::new();
    if abort.is_none() {
        for level in &levels {
            send(&mut serial, "test 0");
            if !wait_rest(&mut serial, &mut buffer, &mut csv, &started, *level) {
                println!("level {level:.0} u: wheels did not stop before probe");
            }
            let send_ms = started.elapsed().as_secs_f64() * 1000.0;
            send(&mut serial, &format!("test {level:.0}"));
            let (samples, reply_at) = listen(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(hold_ms),
                "probe",
                *level,
            );
            send(&mut serial, "test 0");
            match analyze(*level, &samples, reply_at, send_ms) {
                Some(row) => {
                    println!(
                        "level {:>3.0} u: reply {:.1} ms, cmd->torque {:.1}..{:.1} ms (0..{} ticks), torque->wheel {:.1} ms ({} ticks), loop {:.0}..{:.0}, steps {:.1} Hz",
                        row.level,
                        row.reply_ms,
                        row.torque_ms,
                        row.torque_upper_ms,
                        row.torque_upper_ms / TICK_MS,
                        row.wheel_ms,
                        row.wheel_ms / TICK_MS,
                        row.loop_min,
                        row.loop_max,
                        row.tick_rate_hz
                    );
                    results.push(row);
                }
                None => println!("level {level:.0} u: no complete response in window"),
            }
        }
    }

    send(&mut serial, "estop");
    // Stop telemetry before the port closes: a write in flight at DTR drop can
    // park the console task until USB re-enumeration.
    send(&mut serial, "stream 0");
    let _ = listen(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(200),
        "done",
        0.0,
    );
    let _ = csv.flush();
    write_json(&json_path, &results, abort);
    println!("csv: {}", csv_path.display());
    println!("json: {}", json_path.display());
}
