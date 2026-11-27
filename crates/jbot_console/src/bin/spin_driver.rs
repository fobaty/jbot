//! Unattended spin-up torque identification on the Pico USB CDC console.
//!
//! This is a bench tool: it opens the serial port, applies a fixed sequence of
//! `test <units>` commands with the wheels free, records telemetry into `csv/`
//! in the console's schema, and always finishes with `estop`. Firmware limits
//! (±300 units, 5 s auto-stop) still apply.
//!
//! Usage: spin_driver --run [--port /dev/cu.usbmodem...]
//!
//! `--run` is mandatory so an accidental invocation cannot spin the wheels.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,test_right_units,ref_deg,demand_deg,trim_deg,position_m,mode";
const DEFAULT_LEVELS: &[f64] = &[40.0, 50.0, 60.0, 75.0, 100.0, 120.0, 150.0];

/// Minimal view of a `T ...` line, enough for the safety guards and logging.
#[derive(Clone, Debug, Default)]
struct Sample {
    t_ms: u64,
    step: u64,
    loop_hz: f64,
    imu_fresh: bool,
    pitch: f64,
    roll: f64,
    heading: f64,
    gy: f64,
    gx: f64,
    gz: f64,
    left_rpm: f64,
    right_rpm: f64,
    battery: f64,
    age_ms: f64,
    estop: bool,
    vset: f64,
    wset: f64,
    torque: f64,
    test_left: f64,
    test_right: f64,
    reference: f64,
    demand: f64,
    trim: f64,
    position: f64,
    mode: f64,
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

fn flag(line: &str, key: &str, true_word: &str) -> bool {
    field(line, key).is_some_and(|value| value == true_word || value == "1")
}

fn parse_sample(line: &str, t_ms: u64) -> Option<Sample> {
    if !line.starts_with("T ") || field(line, "step").is_none() || field(line, "pitch").is_none() {
        return None;
    }
    Some(Sample {
        t_ms,
        step: num(line, "step") as u64,
        loop_hz: num(line, "loop"),
        imu_fresh: flag(line, "imu", "ok"),
        pitch: num(line, "pitch"),
        roll: num(line, "roll"),
        heading: num(line, "hdg"),
        gy: num(line, "gy"),
        gx: num(line, "gx"),
        gz: num(line, "gz"),
        left_rpm: num(line, "L"),
        right_rpm: num(line, "R"),
        battery: num(line, "bat"),
        age_ms: num(line, "age"),
        estop: flag(line, "estop", "1"),
        vset: num(line, "vset"),
        wset: num(line, "wset"),
        torque: num(line, "trq"),
        test_left: num(line, "tsl"),
        test_right: num(line, "tsr"),
        reference: num(line, "ref"),
        demand: num(line, "dem"),
        trim: num(line, "trm"),
        position: num(line, "x"),
        mode: num(line, "mode"),
    })
}

fn row(sample: &Sample) -> String {
    format!(
        "{},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.1},{:.1},{:.3},{},{},{:.3},{:.3},{:.1},{:.1},{:.1},{:.3},{:.3},{:.3},{:.3},{}",
        sample.t_ms,
        sample.step,
        sample.loop_hz as u64,
        u8::from(sample.imu_fresh),
        sample.pitch,
        sample.roll,
        sample.heading,
        sample.gy,
        sample.gx,
        sample.gz,
        sample.left_rpm,
        sample.right_rpm,
        sample.battery,
        sample.age_ms as u64,
        u8::from(sample.estop),
        sample.vset,
        sample.wset,
        sample.torque,
        sample.test_left,
        sample.test_right,
        sample.reference,
        sample.demand,
        sample.trim,
        sample.position,
        sample.mode as u64,
    )
}

/// Read telemetry for `duration`, parse `T` lines into the CSV and update the
/// latest sample. Always consumes every byte returned by the serial port, so a
/// read timeout cannot leave a stale byte in the line buffer.
fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    latest: &mut Option<Sample>,
    started: &Instant,
    duration: Duration,
) {
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline {
        let mut chunk = [0_u8; 1024];
        let read = serial.read(&mut chunk).unwrap_or_default();
        if read == 0 {
            continue;
        }
        for &byte in &chunk[..read] {
            match byte {
                b'\r' => {}
                b'\n' => {
                    if let Ok(line) = std::str::from_utf8(buffer) {
                        let t_ms = started.elapsed().as_millis() as u64;
                        if let Some(sample) = parse_sample(line.trim_end(), t_ms) {
                            let _ = writeln!(csv, "{}", row(&sample));
                            *latest = Some(sample);
                        }
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
}

fn pick_port(explicit: Option<String>) -> String {
    if let Some(port) = explicit {
        return port;
    }
    serialport::available_ports()
        .ok()
        .and_then(|ports| {
            ports
                .iter()
                .find(|info| info.port_name.contains("usbmodem"))
                .map(|info| info.port_name.clone())
        })
        .unwrap_or_else(|| "/dev/cu.usbmodemJBOT_PICO_011".to_owned())
}

fn timestamp_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |duration| duration.as_millis())
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut run = false;
    let mut clear_estop = false;
    let mut steer = false;
    let mut port = None;
    let mut hold_ms = 1500_u64;
    let mut levels = DEFAULT_LEVELS.to_vec();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--clear-estop" => clear_estop = true,
            "--steer" => steer = true,
            "--port" => port = args.next(),
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value;
                }
            }
            "--levels" => {
                if let Some(value) = args.next() {
                    levels = value
                        .split(',')
                        .filter_map(|item| item.trim().parse::<f64>().ok())
                        .filter(|value| (-300.0..=300.0).contains(value))
                        .collect();
                }
            }
            "--help" | "-h" => {
                println!("usage: spin_driver --run [--clear-estop] [--steer] [--port PATH] [--hold-ms 1500] [--levels 40,50,60,75,100,120,150]");
                return;
            }
            other => {
                eprintln!("unknown option: {other}");
                return;
            }
        }
    }
    if !run {
        eprintln!("refusing to spin: pass --run after securing the robot");
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

    let csv_path = PathBuf::from(format!("csv/jbot-spin-{}.csv", timestamp_ms()));
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
    send(&mut serial, "stream 200");
    send(&mut serial, "stat");
    if clear_estop {
        println!("clearing the latch left by the previous run");
        send(&mut serial, "clear");
    }

    let started = Instant::now();
    let mut buffer = Vec::new();
    let mut latest: Option<Sample> = None;

    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &mut latest,
        &started,
        Duration::from_millis(500),
    );

    // Wake the inactivity counter: commands <= 50 units do not reset the
    // mainboard's 8 minute poweroff timer.
    println!("wake pulse: 60 units for 300 ms");
    send(&mut serial, "test 60");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &mut latest,
        &started,
        Duration::from_millis(300),
    );
    send(&mut serial, "test 0");
    // The flashed firmware has no electric brake, so the wake pulse leaves the
    // wheels coasting for several seconds. Wait for them to actually stop so
    // the safety gate below sees a genuinely idle machine.
    println!("wake pulse: waiting for the wheels to coast down");
    let coast_deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < coast_deadline {
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &mut latest,
            &started,
            Duration::from_millis(200),
        );
        if latest
            .as_ref()
            .is_some_and(|sample| sample.left_rpm.abs() < 10.0 && sample.right_rpm.abs() < 10.0)
        {
            break;
        }
    }

    for &level in &levels {
        // Safety gate: fresh link, no e-stop, disarmed, zero torque and rpm.
        let ready = latest.as_ref().is_some_and(|sample| {
            sample.age_ms < 300.0
                && !sample.estop
                && (sample.mode == 0.0)
                && sample.torque.abs() < 1.0
                && sample.left_rpm.abs() < 10.0
                && sample.right_rpm.abs() < 10.0
        });
        if !ready {
            println!("safety gate failed before level {level:.0}; aborting");
            break;
        }

        println!("level {level:.0}: apply for {} ms", hold_ms);
        if steer {
            // Counter-rotating wheels: with the corrected mixer inverse this
            // reaches the wire as steer = 2*level, speed = 0.
            send(&mut serial, &format!("test {level:.0} -{level:.0}"));
        } else {
            send(&mut serial, &format!("test {level:.0}"));
        }
        pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &mut latest,
            &started,
            Duration::from_millis(hold_ms),
        );
        send(&mut serial, "test 0");
        println!("level {level:.0}: stopped, waiting for zero rpm");

        // The flashed firmware has ELECTRIC_BRAKE_ENABLE off, so free wheels
        // coast for a long time; 4 s was not enough at the 75 unit level.
        let stop_deadline = Instant::now() + Duration::from_secs(20);
        while Instant::now() < stop_deadline {
            pump(
                &mut serial,
                &mut buffer,
                &mut csv,
                &mut latest,
                &started,
                Duration::from_millis(200),
            );
            if latest
                .as_ref()
                .is_some_and(|sample| sample.left_rpm.abs() < 10.0 && sample.right_rpm.abs() < 10.0)
            {
                break;
            }
        }
    }

    send(&mut serial, "estop");
    // Stop the stream and drain long enough for the console's 1 s reply
    // deadline before closing, otherwise a pending write parks the task.
    send(&mut serial, "stream 0");
    pump(
        &mut serial,
        &mut buffer,
        &mut csv,
        &mut latest,
        &started,
        Duration::from_millis(1_500),
    );
    let _ = csv.flush();
    println!("estop sent; recording: {}", csv_path.display());
    println!(
        "run: cargo run -p jbot_analysis --bin spin-fit --release -- {}",
        csv_path.display()
    );
}
