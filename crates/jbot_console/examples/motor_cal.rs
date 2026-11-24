//! Measure the per-wheel command-to-rpm curve with optional breakaway calibration.
//!
//! Intended for the suspended stand: it ramps `test` torque in small steps in
//! both directions, logs the calibrated wheel commands from firmware telemetry,
//! and reports the first command that actually moves each wheel. Run it once
//! with `cal off` to get the thresholds, then again with `--cal L R` to check
//! that the breakaway disappears and the response becomes proportional.
//!
//! Every run clears the e-stop first and ends with `test 0`, `stream 0`, `estop`.
//!
//! Usage: motor_cal [--cal L R] [--port PATH] [--out PATH]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const DEFAULT_PORT: &str = "/dev/cu.usbmodemJBOT_PICO_011";
const SETTLE_MS: u64 = 150;
const STOP_MS: u64 = 1_500;

#[derive(Clone, Debug, Default)]
struct Sample {
    has: bool,
    cmd_left: f64,
    cmd_right: f64,
    left: f64,
    right: f64,
    gy: f64,
    pitch: f64,
    dither: f64,
    dither_applied: f64,
    age: f64,
    estop: f64,
    arm: f64,
}

fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let prefix = format!("{key}=");
    line.split_whitespace()
        .find_map(|token| token.strip_prefix(&prefix))
}

fn num(line: &str, key: &str) -> f64 {
    field(line, key)
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(0.0)
}

fn parse(line: &str) -> Option<Sample> {
    if !line.starts_with("T ") || field(line, "step").is_none() {
        return None;
    }
    Some(Sample {
        has: true,
        cmd_left: num(line, "cwl"),
        cmd_right: num(line, "cwr"),
        left: num(line, "L"),
        right: num(line, "R"),
        gy: num(line, "gy"),
        pitch: num(line, "pitch"),
        dither: num(line, "dth"),
        dither_applied: num(line, "dthv"),
        age: num(line, "age"),
        estop: num(line, "estop"),
        arm: num(line, "arm"),
    })
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut cal = None;
    let mut dither = 0.0_f32;
    let mut port = DEFAULT_PORT.to_owned();
    let mut out = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--cal" => {
                let left = args.next().and_then(|v| v.parse::<f32>().ok());
                let right = args.next().and_then(|v| v.parse::<f32>().ok());
                if let (Some(l), Some(r)) = (left, right) {
                    cal = Some((l, r));
                } else {
                    eprintln!("--cal needs two numbers");
                    return;
                }
            }
            "--dither" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f32>().ok()) {
                    dither = value.clamp(0.0, 60.0);
                } else {
                    eprintln!("--dither needs amplitude in 0..60 units");
                    return;
                }
            }
            "--port" => {
                if let Some(value) = args.next() {
                    port = value;
                }
            }
            "--out" => {
                out = args.next();
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

    let name = out.unwrap_or_else(|| {
        format!(
            "csv/jbot-motor-cal-{}.csv",
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_or(0, |d| d.as_millis())
        )
    });
    let _ = std::fs::create_dir_all("csv");
    let mut csv = BufWriter::new(File::create(&name).expect("create csv"));
    writeln!(
        csv,
        "t_ms,cmd_units,cal_left,cal_right,cwl,cwr,left_rpm,right_rpm,gy_dps,pitch_deg,dither,dither_applied,age_ms,estop,arm"
    )
    .unwrap();

    let started = Instant::now();
    let mut line_buf = Vec::new();
    let mut latest = Sample::default();

    let mut wait = |ms: u64, port: &mut Box<dyn SerialPort>, latest: &mut Sample| {
        let deadline = started.elapsed() + Duration::from_millis(ms);
        let mut chunk = [0_u8; 4096];
        while started.elapsed() < deadline {
            if let Ok(n) = port.read(&mut chunk) {
                if n > 0 {
                    for &byte in &chunk[..n] {
                        match byte {
                            b'\r' => {}
                            b'\n' => {
                                if let Ok(text) = std::str::from_utf8(&line_buf) {
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
    let record = |csv: &mut BufWriter<File>, cmd: i32, cal: Option<(f32, f32)>, s: &Sample| {
        let t = started.elapsed().as_millis();
        let (cl, cr) = cal.unwrap_or((0.0, 0.0));
        writeln!(
            csv,
            "{t},{cmd},{cl:.0},{cr:.0},{:.1},{:.1},{:.1},{:.1},{:.2},{:.2},{:.0},{:.1},{:.0},{:.0},{:.0}",
            s.cmd_left,
            s.cmd_right,
            s.left,
            s.right,
            s.gy,
            s.pitch,
            s.dither,
            s.dither_applied,
            s.age,
            s.estop,
            s.arm
        )
        .unwrap();
    };

    send(&mut serial, "stream 200");
    wait(500, &mut serial, &mut latest);
    send(&mut serial, "clear");
    send(&mut serial, "test 0");
    if let Some((left, right)) = cal {
        send(&mut serial, &format!("cal {left:.0} {right:.0}"));
    } else {
        send(&mut serial, "cal off");
    }
    if dither > 0.0 {
        send(&mut serial, &format!("dither {dither:.0}"));
    } else {
        send(&mut serial, "dither off");
    }
    wait(700, &mut serial, &mut latest);

    let ready = latest.has
        && latest.estop == 0.0
        && latest.arm == 0.0
        && latest.age < 300.0
        && latest.left.abs() < 12.0
        && latest.right.abs() < 12.0;
    if !ready {
        println!("safety gate failed; no spin: {latest:?}");
        send(&mut serial, "stream 0");
        send(&mut serial, "estop");
        return;
    }
    println!(
        "safety gate ok: estop=0 arm=0 rpm≈0 age={:.0} ms cal={:?}",
        latest.age, cal
    );

    for direction in [1_i32, -1] {
        for step in (5..=80).step_by(5) {
            let cmd = direction * step;
            send(&mut serial, &format!("test {cmd}"));
            wait(SETTLE_MS, &mut serial, &mut latest);
            record(&mut csv, cmd, cal, &latest);
            if step % 20 == 0 || step == 5 {
                println!(
                    "cmd={cmd:+3} applied=({:5.1},{:5.1}) rpm=({:7.1},{:7.1}) gy={:+.2}",
                    latest.cmd_left, latest.cmd_right, latest.left, latest.right, latest.gy
                );
            }
        }
        send(&mut serial, "test 0");
        wait(STOP_MS, &mut serial, &mut latest);
    }

    send(&mut serial, "test 0");
    wait(400, &mut serial, &mut latest);
    send(&mut serial, "stream 0");
    send(&mut serial, "estop");
    let _ = csv.flush();
    println!("done; recording: {name}");
}
