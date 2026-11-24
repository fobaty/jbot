//! Measure command-to-wheel-motion latency with and without dither.
//!
//! One run performs two identical small steps from rest: first with dither
//! disabled, then with the requested amplitude. Telemetry is sampled at the
//! full 200 Hz so the delay from command change to sustained wheel motion is
//! visible. Intended for the suspended stand.
//!
//! Usage: dither_step [--dither 30] [--cal L R] [--port PATH]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const DEFAULT_PORT: &str = "/dev/cu.usbmodemJBOT_PICO_011";
const STEP_UNITS: i32 = 20;
const SETTLE_MS: u64 = 1_500;
const WINDOW_MS: u64 = 1_200;

#[derive(Clone, Copy, Debug, Default)]
struct Sample {
    has: bool,
    cwl: f64,
    cwr: f64,
    left: f64,
    right: f64,
    dthv: f64,
    age: f64,
    estop: f64,
}

fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let prefix = format!("{key}=");
    line.split_whitespace()
        .find_map(|token| token.strip_prefix(&prefix))
}

fn num(line: &str, key: &str) -> f64 {
    field(line, key)
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(0.0)
}

fn parse(line: &str) -> Option<Sample> {
    if !line.starts_with("T ") || field(line, "step").is_none() {
        return None;
    }
    Some(Sample {
        has: true,
        cwl: num(line, "cwl"),
        cwr: num(line, "cwr"),
        left: num(line, "L"),
        right: num(line, "R"),
        dthv: num(line, "dthv"),
        age: num(line, "age"),
        estop: num(line, "estop"),
    })
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut dither = 30.0_f32;
    let mut cal = (32.0_f32, 28.0_f32);
    let mut port = DEFAULT_PORT.to_owned();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--dither" => {
                dither = args
                    .next()
                    .and_then(|v| v.parse::<f32>().ok())
                    .unwrap_or(dither)
                    .clamp(0.0, 60.0);
            }
            "--cal" => {
                let l = args.next().and_then(|v| v.parse::<f32>().ok());
                let r = args.next().and_then(|v| v.parse::<f32>().ok());
                if let (Some(l), Some(r)) = (l, r) {
                    cal = (l, r);
                }
            }
            "--port" => {
                if let Some(value) = args.next() {
                    port = value;
                }
            }
            other => {
                eprintln!("unknown option: {other}");
                return;
            }
        }
    }

    let mut serial = match serialport::new(&port, 115_200)
        .timeout(Duration::from_millis(50))
        .dtr_on_open(true)
        .open()
    {
        Ok(port) => port,
        Err(error) => {
            println!("open error: {error}");
            return;
        }
    };
    let name = format!(
        "csv/jbot-dither-step-{}.csv",
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_millis())
    );
    let _ = std::fs::create_dir_all("csv");
    let mut csv = BufWriter::new(File::create(&name).expect("create csv"));
    writeln!(
        csv,
        "t_ms,phase,cmd_units,cwl,cwr,left_rpm,right_rpm,dither_applied,age_ms,estop"
    )
    .unwrap();

    let started = Instant::now();
    let mut line_buf = Vec::new();
    let mut latest = Sample::default();

    let wait =
        |ms: u64, port: &mut Box<dyn SerialPort>, latest: &mut Sample, line_buf: &mut Vec<u8>| {
            let deadline = started.elapsed() + Duration::from_millis(ms);
            let mut chunk = [0_u8; 4096];
            while started.elapsed() < deadline {
                if let Ok(n) = port.read(&mut chunk) {
                    if n > 0 {
                        for &byte in &chunk[..n] {
                            match byte {
                                b'\r' => {}
                                b'\n' => {
                                    if let Ok(text) = std::str::from_utf8(line_buf) {
                                        if let Some(sample) = parse(text.trim_end()) {
                                            *latest = sample;
                                        }
                                    }
                                    line_buf.clear();
                                }
                                other => line_buf.push(other),
                            }
                        }
                    }
                }
            }
        };
    let send = |port: &mut Box<dyn SerialPort>, text: &str| {
        let _ = port.write_all(format!("{text}\n").as_bytes());
        let _ = port.flush();
    };

    send(&mut serial, "stream 200");
    wait(500, &mut serial, &mut latest, &mut line_buf);
    send(&mut serial, "clear");
    send(&mut serial, &format!("cal {} {}", cal.0, cal.1));
    send(&mut serial, "test 0");
    wait(700, &mut serial, &mut latest, &mut line_buf);
    if !latest.has || latest.estop != 0.0 || latest.age > 300.0 {
        println!("safety gate failed: {latest:?}");
        send(&mut serial, "stream 0");
        send(&mut serial, "estop");
        return;
    }

    for (phase, amplitude) in [("off", 0.0_f32), ("on", dither)] {
        send(&mut serial, &format!("dither {amplitude:.0}"));
        send(&mut serial, "test 0");
        wait(SETTLE_MS, &mut serial, &mut latest, &mut line_buf);

        let t0 = started.elapsed().as_millis();
        send(&mut serial, &format!("test {STEP_UNITS}"));
        let deadline = started.elapsed() + Duration::from_millis(WINDOW_MS);
        let mut chunk = [0_u8; 4096];
        while started.elapsed() < deadline {
            if let Ok(n) = serial.read(&mut chunk) {
                if n > 0 {
                    for &byte in &chunk[..n] {
                        match byte {
                            b'\r' => {}
                            b'\n' => {
                                if let Ok(text) = std::str::from_utf8(&line_buf) {
                                    if let Some(s) = parse(text.trim_end()) {
                                        latest = s;
                                        let t = started.elapsed().as_millis();
                                        writeln!(
                                            csv,
                                            "{t},{phase},{STEP_UNITS},{:.1},{:.1},{:.1},{:.1},{:.1},{:.0},{:.0}",
                                            s.cwl, s.cwr, s.left, s.right, s.dthv, s.age, s.estop
                                        )
                                        .unwrap();
                                    }
                                }
                                line_buf.clear();
                            }
                            other => line_buf.push(other),
                        }
                    }
                }
            }
        }
        println!(
            "phase={phase}: step at t={t0} ms, last applied=({:.1},{:.1}) rpm=({:.1},{:.1})",
            latest.cwl, latest.cwr, latest.left, latest.right
        );
    }

    send(&mut serial, "test 0");
    wait(500, &mut serial, &mut latest, &mut line_buf);
    send(&mut serial, "stream 0");
    send(&mut serial, "estop");
    let _ = csv.flush();
    println!("recording: {name}");
}
