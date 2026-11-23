//! Undisturbed 10 s hold gate with the slew knob isolated.
//!
//! Each profile applies runtime `pscale` and `slew` (no firmware changes),
//! waits for a quiet, healthy baseline, arms the existing law, records a hold
//! window, scores it with the audit's gate and finishes with `estop`. The port
//! stays open for the whole sweep so the USB console is not parked by DTR.
//!
//! Usage: hold_sweep --run [--port PATH] [--pscale 0.25]
//!                   [--slews 100,200,300] [--hold-ms 10000]
//!
//! Safety: blocks under the body, rope attached, hands on the body, e-stop
//! reachable, test zone at most 0.5 m. Any guard, fall latch or disarm ends the
//! profile immediately. No torque is applied unless mainboard feedback is fresh.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,pitch_deg,gy_dps,left_rpm,right_rpm,bat_v,age_ms,estop,arm,tsl_units,tsr_units,trq_units,ref_deg,demand_deg,position_m,mode,opm,profile";
const PITCH_LIMIT_DEG: f64 = 2.0;
const TRAVEL_LIMIT_M: f64 = 0.1;
const TORQUE_LIMIT_UNITS: f64 = 900.0;
const QUIET_GY: f64 = 8.0;
const QUIET_PITCH: f64 = 5.0;
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

#[derive(Clone)]
struct Sample {
    line: String,
}

fn listen(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    profile: &str,
) -> (Vec<Sample>, Vec<String>) {
    let mut out = Vec::new();
    let mut replies = Vec::new();
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
                                (n("arm") as u64).to_string(),
                                format!("{:.1}", n("tsl")),
                                format!("{:.1}", n("tsr")),
                                format!("{:.1}", n("trq")),
                                format!("{:.4}", n("ref")),
                                format!("{:.4}", n("dem")),
                                format!("{:.4}", n("x")),
                                (n("mode") as u64).to_string(),
                                (n("opm") as u64).to_string(),
                                profile.to_owned(),
                            ];
                            let _ = writeln!(csv, "{}", row.join(","));
                            out.push(Sample { line: text });
                        } else if text.starts_with("ok ") || text.starts_with("err ") {
                            replies.push(text);
                        }
                    }
                    buffer.clear();
                }
                other => buffer.push(other),
            }
        }
    }
    (out, replies)
}

struct Gate {
    pscale: f64,
    slew: f64,
    samples: usize,
    max_pitch: f64,
    final_pitch: f64,
    travel: f64,
    peak_torque: f64,
    estop: bool,
    disarmed: bool,
    fall: bool,
    other_mode: bool,
}

impl Gate {
    fn new(pscale: f64, slew: f64) -> Self {
        Self {
            pscale,
            slew,
            samples: 0,
            max_pitch: 0.0,
            final_pitch: 0.0,
            travel: 0.0,
            peak_torque: 0.0,
            estop: false,
            disarmed: false,
            fall: false,
            other_mode: false,
        }
    }

    fn observe(&mut self, line: &str, origin_x: f64) {
        self.samples += 1;
        let pitch = num(line, "pitch");
        self.max_pitch = self.max_pitch.max(pitch.abs());
        self.final_pitch = pitch;
        self.travel = self.travel.max((num(line, "x") - origin_x).abs());
        self.peak_torque = self.peak_torque.max(num(line, "trq").abs());
        self.estop |= num(line, "estop") > 0.5;
        self.disarmed |= num(line, "arm") < 0.5;
        self.fall |= num(line, "mode") > 3.5 || num(line, "opm") > 0.5;
        self.other_mode |= num(line, "mode") > 2.5;
    }

    fn pass(&self) -> bool {
        self.samples > 200
            && !self.estop
            && !self.disarmed
            && !self.fall
            && !self.other_mode
            && self.max_pitch < PITCH_LIMIT_DEG
            && self.final_pitch.abs() < PITCH_LIMIT_DEG
            && self.travel < TRAVEL_LIMIT_M
            && self.peak_torque < TORQUE_LIMIT_UNITS
    }
}

fn wait_baseline(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    profile: &str,
) -> Option<Sample> {
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut quiet_since: Option<Instant> = None;
    while Instant::now() < deadline {
        let (samples, _) = listen(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(50),
            profile,
        );
        let Some(last) = samples.last() else {
            continue;
        };
        let line = &last.line;
        let healthy = field(line, "mb") == Some("ok")
            && num(line, "age") < MAX_FB_AGE_MS
            && num(line, "bat") > 1.0
            && num(line, "estop") < 0.5
            && num(line, "arm") < 0.5;
        let quiet = num(line, "gy").abs() <= QUIET_GY && num(line, "pitch").abs() <= QUIET_PITCH;
        if healthy && quiet {
            if quiet_since.is_none() {
                quiet_since = Some(Instant::now());
            }
            if quiet_since.is_some_and(|t| t.elapsed() >= Duration::from_millis(500)) {
                return Some(last.clone());
            }
        } else {
            quiet_since = None;
        }
    }
    None
}

fn write_json(path: &PathBuf, gates: &[Gate], abort: Option<&str>) {
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
    let _ = writeln!(file, "  \"profiles\": [");
    for (index, gate) in gates.iter().enumerate() {
        let comma = if index + 1 == gates.len() { "" } else { "," };
        let _ = writeln!(
            file,
            "    {{\"pscale\": {:.2}, \"slew\": {:.0}, \"pass\": {}, \"samples\": {}, \"max_pitch_deg\": {:.2}, \"final_pitch_deg\": {:.2}, \"travel_m\": {:.3}, \"peak_torque\": {:.0}, \"estop\": {}, \"fall\": {}}}{comma}",
            gate.pscale,
            gate.slew,
            gate.pass(),
            gate.samples,
            gate.max_pitch,
            gate.final_pitch,
            gate.travel,
            gate.peak_torque,
            u8::from(gate.estop),
            u8::from(gate.fall)
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
    let mut pscale = 0.25_f64;
    let mut slews = vec![100.0, 200.0, 300.0];
    let mut hold_ms = 10_000_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--port" => port = args.next(),
            "--pscale" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    pscale = value.clamp(0.0, 2.0);
                }
            }
            "--slews" => {
                if let Some(value) = args.next() {
                    slews = value
                        .split(',')
                        .filter_map(|part| part.trim().parse::<f64>().ok())
                        .filter(|value| (100.0..=10_000.0).contains(value))
                        .collect();
                }
            }
            "--hold-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    hold_ms = value.clamp(1000, 60_000);
                }
            }
            "--help" | "-h" => {
                println!("usage: hold_sweep --run [--port PATH] [--pscale 0.25] [--slews 100,200,300] [--hold-ms 10000]");
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
    if slews.is_empty() {
        eprintln!("no valid slew values");
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

    let stem = format!("csv/jbot-hold-{}", timestamp_ms());
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
    let (init, _) = listen(
        &mut serial,
        &mut buffer,
        &mut csv,
        &started,
        Duration::from_millis(400),
        "init",
    );
    let mut abort = None;
    if !init.last().is_some_and(|sample| {
        let line = &sample.line;
        field(line, "mb") == Some("ok") && num(line, "bat") > 1.0
    }) {
        abort = Some("mainboard feedback absent");
        eprintln!("{}; no arm sent", abort.unwrap_or_default());
    }

    let mut gates = Vec::new();
    if abort.is_none() {
        for slew in &slews {
            let profile = format!("p{pscale:.2}_s{slew:.0}");
            send(&mut serial, "clear");
            send(&mut serial, &format!("pscale {pscale:.2}"));
            send(&mut serial, &format!("slew {slew:.0}"));
            send(&mut serial, "ff off");
            let (_, replies) = listen(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(200),
                &profile,
            );
            if replies.iter().any(|line| line.starts_with("err ")) {
                println!("profile {profile}: setup refused: {:?}", replies);
                send(&mut serial, "estop");
                continue;
            }
            let Some(baseline) =
                wait_baseline(&mut serial, &mut buffer, &mut csv, &started, &profile)
            else {
                println!("profile {profile}: no quiet baseline; skipped");
                send(&mut serial, "estop");
                continue;
            };
            // `level` is runtime state and does not survive a Pico reboot;
            // the arm gate requires it. Capture it at the quiet baseline pose.
            send(&mut serial, "level");
            let (_, level_replies) = listen(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(200),
                &profile,
            );
            if level_replies.iter().any(|line| line.starts_with("err ")) {
                println!("profile {profile}: level refused: {level_replies:?}");
                send(&mut serial, "estop");
                continue;
            }
            let origin_x = num(&baseline.line, "x");
            send(&mut serial, "arm");
            let (armed, arm_replies) = listen(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(300),
                &profile,
            );
            if !armed
                .last()
                .is_some_and(|sample| num(&sample.line, "arm") > 0.5)
            {
                println!("profile {profile}: arm refused: {:?}", arm_replies);
                send(&mut serial, "estop");
                continue;
            }
            println!("profile {profile}: armed, recording {} ms", hold_ms);
            let (held, _) = listen(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(hold_ms),
                &profile,
            );
            let mut gate = Gate::new(pscale, *slew);
            for sample in &held {
                gate.observe(&sample.line, origin_x);
                if gate.estop || gate.disarmed || gate.fall {
                    break;
                }
            }
            println!(
                "profile {profile}: {} samples, pitch max {:.2} final {:.2}, travel {:.3} m, torque {:.0} u, estop {} fall {} => {}",
                gate.samples,
                gate.max_pitch,
                gate.final_pitch,
                gate.travel,
                gate.peak_torque,
                u8::from(gate.estop),
                u8::from(gate.fall),
                if gate.pass() { "PASS" } else { "FAIL" }
            );
            gates.push(gate);
            send(&mut serial, "estop");
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
    );
    let _ = csv.flush();
    write_json(&json_path, &gates, abort);
    println!("csv: {}", csv_path.display());
    println!("json: {}", json_path.display());
}
