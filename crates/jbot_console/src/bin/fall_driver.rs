//! Unattended controlled-fall bench driver for the Pico USB CDC console.
//!
//! Arms the balance law, requests `fall`, records telemetry and always ends
//! with `estop`. The robot must be secured with a rope and the test zone kept
//! small before `--run` is passed.
//!
//! Usage: fall_driver --run [--port PATH] [--record-ms 8000]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,pitch_deg,heading_deg,gy_dps,gz_dps,ref_deg,demand_deg,torque_units,position_m,loop_hz,age_ms,estop,mode,op_mode,grace_s,arm";

fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    line.split_whitespace()
        .find_map(|token| token.strip_prefix(&format!("{key}=")))
}

fn num(line: &str, key: &str) -> f64 {
    field(line, key)
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(0.0)
}

fn telemetry_line(line: &str) -> bool {
    line.starts_with("T ") && field(line, "step").is_some() && field(line, "pitch").is_some()
}

fn row(line: &str, t_ms: u64) -> String {
    let n = |key| num(line, key);
    let fields = [
        t_ms.to_string(),
        (n("step") as u64).to_string(),
        format!("{:.4}", n("pitch")),
        format!("{:.4}", n("hdg")),
        format!("{:.4}", n("gy")),
        format!("{:.4}", n("gz")),
        format!("{:.4}", n("ref")),
        format!("{:.4}", n("dem")),
        format!("{:.1}", n("trq")),
        format!("{:.4}", n("x")),
        (n("loop") as u64).to_string(),
        (n("age") as u64).to_string(),
        (n("estop") as u64).to_string(),
        (n("mode") as u64).to_string(),
        (n("opm") as u64).to_string(),
        format!("{:.3}", n("grace")),
        (n("arm") as u64).to_string(),
    ];
    fields.join(",")
}

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
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
                    if let Ok(line) = std::str::from_utf8(buffer) {
                        let text = line.trim_end().to_owned();
                        if telemetry_line(&text) {
                            let t_ms = started.elapsed().as_millis() as u64;
                            let _ = writeln!(csv, "{}", row(&text, t_ms));
                            if text.contains("arm=1") || text.contains("opm=3") {
                                lines.push(text);
                            }
                        } else if text.starts_with("ok ") || text.starts_with("err ") {
                            lines.push(text);
                        }
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
    lines
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

fn timestamp_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut port = None;
    let mut record_ms = 8_000_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--record-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    record_ms = value;
                }
            }
            "--help" | "-h" => {
                println!("usage: fall_driver --run [--port PATH] [--record-ms 8000]");
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

    let csv_path = PathBuf::from(format!("csv/jbot-fall-{}.csv", timestamp_ms()));
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
        Duration::from_millis(500),
    );
    send(&mut serial, "clear");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(300),
    );
    send(&mut serial, "level");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(300),
    );
    send(&mut serial, "arm");
    let arm = pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(1_000),
    );
    let armed = arm
        .iter()
        .any(|line| line.contains("arm=1") || line.starts_with("ok armed"));
    println!("armed: {armed}");
    if !armed {
        for line in arm.iter().take(4) {
            println!("  {line}");
        }
        send(&mut serial, "estop");
        let _ = csv.flush();
        println!("aborted; recording: {}", csv_path.display());
        return;
    }

    println!("requesting safe fall");
    send(&mut serial, "fall");
    let events = pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(record_ms),
    );
    println!("recorded {} ms, {} notable lines", record_ms, events.len());
    for line in events.iter().take(6) {
        println!("  {line}");
    }

    send(&mut serial, "estop");
    let _ = csv.flush();
    println!("estop sent; recording: {}", csv_path.display());
}
