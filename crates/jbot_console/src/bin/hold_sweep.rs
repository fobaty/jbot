//! Undisturbed 10 s hold gate with the slew knob isolated.
//!
//! Each profile applies runtime `pscale`, `slew` and `db` (no firmware changes),
//! waits for a quiet, healthy baseline, arms the existing law, records a hold
//! window, scores it with the audit's gate and finishes with `estop`. The port
//! stays open for the whole sweep so the USB console is not parked by DTR.
//!
//! A quiet body is not enough: the gate also requires wheel motion, otherwise
//! a rope still carrying the weight would score as a successful balance. An
//! opening pitch-rate spike marks the release as `VOID`: a hand swing invalidates
//! the trial before the control law can respond, so it is not a FAIL either.
//!
//! Usage: hold_sweep --run [--port PATH] [--pscale 0.25] [--db 0.00]
//!                   [--slews 100,200,300] [--hold-ms 10000] [--release]
//!
//! After arming, the driver keeps draining telemetry until the support is
//! removed and then starts the scored hold window. Without `--release-file`,
//! release is detected from telemetry (a pitch jump or wheel motion), so no
//! operator terminal is needed. With `--release-file PATH`, the operator
//! creates that file at the release cue instead. `--ready-file PATH` is
//! written once the arm succeeded. Neither mechanism relies on harness stdin.
//!
//! Safety: blocks under the body, rope attached, hands on the body, e-stop
//! reachable, test zone at most 0.5 m. Any guard, fall latch or disarm ends the
//! profile immediately. No torque is applied unless mainboard feedback is fresh.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115200;
const HEADER: &str = "t_ms,step,loop_hz,pitch_deg,lvl_deg,gy_dps,left_rpm,right_rpm,bat_v,age_ms,estop,arm,tsl_units,tsr_units,trq_units,ref_deg,demand_deg,position_m,mode,opm,profile";
const PITCH_LIMIT_DEG: f64 = 2.0;
const TRAVEL_LIMIT_M: f64 = 0.1;
const TORQUE_LIMIT_UNITS: f64 = 900.0;
/// A hold without wheel motion is a rope-supported stance, not a balance.
const MIN_WHEEL_ACTIVE_SAMPLES: usize = 20;
/// Opening hold-window samples used to grade the release. At the 100 Hz stream
/// this is about 300 ms; the fixture targets |gy| <= 2 dps over that span.
const RELEASE_WINDOW_SAMPLES: usize = 30;
/// A hand release swings the body far faster than a fixed-anchor drop. Above
/// this opening rate the trial cannot attribute its outcome to the control law.
const RELEASE_GY_LIMIT_DPS: f64 = 25.0;
/// A fixture is only qualified when the opening is effectively impulse-free.
/// This is stricter than the VOID gate: a release between these thresholds is
/// informative for control but still not a repeatable fixture.
const FIXTURE_GY_LIMIT_DPS: f64 = 2.0;
/// Number of identical releases required by the fixture acceptance test.
const FIXTURE_RUNS: usize = 3;
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
                                format!("{:.4}", n("lvl")),
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
    db: f64,
    samples: usize,
    max_pitch: f64,
    final_pitch: f64,
    travel: f64,
    peak_torque: f64,
    wheel_active_samples: usize,
    /// Peak gyro rate in the release window after the cue.
    release_gy_peak: f64,
    /// Peak gyro rate in the window just before the cue. A hand swing can
    /// happen here, before the wheels move and before scoring starts.
    release_pre_peak: f64,
    estop: bool,
    disarmed: bool,
    fall: bool,
    other_mode: bool,
}

impl Gate {
    fn new(pscale: f64, slew: f64, db: f64) -> Self {
        Self {
            pscale,
            slew,
            db,
            samples: 0,
            max_pitch: 0.0,
            final_pitch: 0.0,
            travel: 0.0,
            peak_torque: 0.0,
            wheel_active_samples: 0,
            release_gy_peak: 0.0,
            release_pre_peak: 0.0,
            estop: false,
            disarmed: false,
            fall: false,
            other_mode: false,
        }
    }

    fn observe(&mut self, line: &str, origin_x: f64) {
        self.samples += 1;
        // Telemetry `pitch` is relative to `zero`; `level` adds a trim that is
        // the real vertical for this session. Score the control error, not the
        // raw sensor angle.
        let pitch = num(line, "pitch") - num(line, "lvl");
        self.max_pitch = self.max_pitch.max(pitch.abs());
        self.final_pitch = pitch;
        self.travel = self.travel.max((num(line, "x") - origin_x).abs());
        self.peak_torque = self.peak_torque.max(num(line, "trq").abs());
        if num(line, "L").abs() > 2.0 || num(line, "R").abs() > 2.0 {
            self.wheel_active_samples += 1;
        }
        if self.samples <= RELEASE_WINDOW_SAMPLES {
            self.release_gy_peak = self.release_gy_peak.max(num(line, "gy").abs());
        }
        self.estop |= num(line, "estop") > 0.5;
        self.disarmed |= num(line, "arm") < 0.5;
        self.fall |= num(line, "mode") > 3.5 || num(line, "opm") > 0.5;
        self.other_mode |= num(line, "mode") > 2.5;
    }

    /// Largest gyro rate spanning both sides of the release cue.
    fn release_disturbance_peak(&self) -> f64 {
        self.release_pre_peak.max(self.release_gy_peak)
    }

    /// A violent release injects a several-degree swing before the loop can
    /// react, so the outcome says nothing about the control law.
    fn release_void(&self) -> bool {
        self.release_disturbance_peak() > RELEASE_GY_LIMIT_DPS
    }

    fn pass(&self) -> bool {
        self.samples > 200
            && !self.release_void()
            && !self.estop
            && !self.disarmed
            && !self.fall
            && !self.other_mode
            && self.max_pitch < PITCH_LIMIT_DEG
            && self.final_pitch.abs() < PITCH_LIMIT_DEG
            && self.travel < TRAVEL_LIMIT_M
            && self.peak_torque < TORQUE_LIMIT_UNITS
            && self.wheel_active_samples >= MIN_WHEEL_ACTIVE_SAMPLES
    }
}

fn wait_baseline(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    profile: &str,
    quiet_pitch: f64,
    quiet_gy: f64,
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
        let quiet = num(line, "gy").abs() <= quiet_gy && num(line, "pitch").abs() <= quiet_pitch;
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

/// Keep telemetry flowing while the operator releases the support. The scored
/// window must not start before that cue, and the port must keep draining so a
/// paused host cannot back-pressure the bounded USB console writes.
///
/// The release can be signalled by a sentinel file or detected from telemetry:
/// removing the support moves the body or starts the wheels. Detection makes
/// the gate independent of operator terminal access and of chat latency.
/// Returns `(released, pre_release_gy_peak)`. The second value is the largest
/// gyro rate seen in the rolling window just before the release cue. A hand
/// swing often happens before the wheels start moving, so scoring only the
/// post-release window would miss the very disturbance the fixture gate must
/// catch.
fn wait_for_release(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    path: Option<&str>,
    baseline_pitch: f64,
    timeout: Duration,
) -> (bool, f64) {
    if let Some(path) = path {
        let _ = std::fs::remove_file(Path::new(path));
    }
    let deadline = Instant::now() + timeout;
    let mut motion_samples = 0_usize;
    let mut pre_gy: Vec<f64> = Vec::with_capacity(RELEASE_WINDOW_SAMPLES);
    while path.is_none_or(|path| !Path::new(path).exists()) {
        if Instant::now() >= deadline {
            eprintln!(
                "release cue timed out after {} ms; aborting",
                timeout.as_millis()
            );
            let _ = serial.write_all(b"estop\n");
            return (false, 0.0);
        }
        let (samples, _) = listen(
            serial,
            buffer,
            csv,
            started,
            Duration::from_millis(50),
            "release_wait",
        );
        for sample in &samples {
            pre_gy.push(num(&sample.line, "gy").abs());
        }
        if pre_gy.len() > RELEASE_WINDOW_SAMPLES {
            let excess = pre_gy.len() - RELEASE_WINDOW_SAMPLES;
            pre_gy.drain(..excess);
        }
        if path.is_none()
            && samples.iter().any(|sample| {
                let line = &sample.line;
                (num(line, "pitch") - baseline_pitch).abs() > 2.5
                    || num(line, "L").abs() > 3.0
                    || num(line, "R").abs() > 3.0
            })
        {
            motion_samples += samples.len();
            if motion_samples >= 5 {
                println!("release detected from telemetry; starting hold window");
                return (true, peak_gy(&pre_gy));
            }
        }
    }
    (true, peak_gy(&pre_gy))
}

fn peak_gy(values: &[f64]) -> f64 {
    values.iter().copied().fold(0.0_f64, f64::max)
}

/// Grade the release fixture, not the control law. Every run must be present,
/// free of the VOID flag and open below the fixture rate limit. A missing run
/// is a failure: three releases are required by `docs/release-fixture.md`.
fn fixture_acceptable(gates: &[Gate]) -> bool {
    gates.len() == FIXTURE_RUNS
        && gates.iter().all(|gate| {
            !gate.release_void() && gate.release_disturbance_peak() <= FIXTURE_GY_LIMIT_DPS
        })
}

fn write_json(path: &PathBuf, gates: &[Gate], abort: Option<&str>, fixture: Option<bool>) {
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
    if let Some(acceptable) = fixture {
        let _ = writeln!(file, "  \"fixture_acceptable\": {},", acceptable);
    }
    let _ = writeln!(file, "  \"profiles\": [");
    for (index, gate) in gates.iter().enumerate() {
        let comma = if index + 1 == gates.len() { "" } else { "," };
        let _ = writeln!(
            file,
            "    {{\"pscale\": {:.2}, \"slew\": {:.0}, \"db\": {:.2}, \"pass\": {}, \"release_void\": {}, \"release_pre_gy_peak\": {:.1}, \"release_gy_peak\": {:.1}, \"samples\": {}, \"max_pitch_deg\": {:.2}, \"final_pitch_deg\": {:.2}, \"travel_m\": {:.3}, \"peak_torque\": {:.0}, \"wheel_active_samples\": {}, \"estop\": {}, \"fall\": {}}}{comma}",
            gate.pscale,
            gate.slew,
            gate.db,
            gate.pass(),
            u8::from(gate.release_void()),
            gate.release_pre_peak,
            gate.release_gy_peak,
            gate.samples,
            gate.max_pitch,
            gate.final_pitch,
            gate.travel,
            gate.peak_torque,
            gate.wheel_active_samples,
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
    let mut fixture_qual = false;
    let mut port = None;
    let mut pscale = 0.25_f64;
    let mut db = 0.0_f64;
    let mut release_file: Option<String> = None;
    let mut ready_file: Option<String> = None;
    let mut quiet_pitch = QUIET_PITCH;
    let mut quiet_gy = QUIET_GY;
    let mut release_timeout_ms = 5_000_u64;
    let mut slews = vec![100.0, 200.0, 300.0];
    let mut hold_ms = 10_000_u64;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--run" => run = true,
            "--fixture-qual" => fixture_qual = true,
            "--release-file" => release_file = args.next(),
            "--ready-file" => ready_file = args.next(),
            "--quiet-pitch" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    quiet_pitch = value.clamp(0.5, QUIET_PITCH);
                }
            }
            "--quiet-gy" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    quiet_gy = value.clamp(1.0, QUIET_GY);
                }
            }
            "--release-timeout-ms" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    release_timeout_ms = value.clamp(1_000, 30_000);
                }
            }
            "--port" => port = args.next(),
            "--pscale" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    pscale = value.clamp(0.0, 2.0);
                }
            }
            "--db" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                    db = value.clamp(0.0, 0.6);
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
                println!("usage: hold_sweep --run [--fixture-qual] [--port PATH] [--pscale 0.25] [--db 0.00] [--slews 100,200,300] [--hold-ms 10000] [--release-file PATH] [--ready-file PATH] [--quiet-pitch 5] [--quiet-gy 8] [--release-timeout-ms 5000]");
                return;
            }
            other => {
                eprintln!("unknown option: {other}");
                return;
            }
        }
    }
    if fixture_qual {
        // Fixture grading only needs the release opening, so use three
        // identical short profiles instead of the full gain ladder.
        let slew = slews.first().copied().unwrap_or(100.0);
        slews = vec![slew; FIXTURE_RUNS];
        hold_ms = 2_000;
    }
    if let Some(path) = &ready_file {
        let _ = std::fs::remove_file(path);
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
    let mut abort: Option<String> = None;
    if !init.last().is_some_and(|sample| {
        let line = &sample.line;
        field(line, "mb") == Some("ok") && num(line, "bat") > 1.0
    }) {
        abort = Some("mainboard feedback absent".to_owned());
        eprintln!("{}; no arm sent", abort.as_deref().unwrap_or_default());
    }

    let mut gates = Vec::new();
    if abort.is_none() {
        for slew in &slews {
            let profile = format!("p{pscale:.2}_s{slew:.0}_db{db:.2}");
            send(&mut serial, "clear");
            send(&mut serial, &format!("pscale {pscale:.2}"));
            send(&mut serial, &format!("slew {slew:.0}"));
            send(&mut serial, "ff off");
            send(&mut serial, &format!("db {db:.2}"));
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
            let Some(baseline) = wait_baseline(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                &profile,
                quiet_pitch,
                quiet_gy,
            ) else {
                println!("profile {profile}: no quiet baseline; skipped");
                send(&mut serial, "estop");
                continue;
            };
            // `zero` and `level` are runtime state. A Pico reboot restores the
            // pitch zero only from a saved pose_home, so after `forget` the arm
            // gate refuses to arm until a fresh zero is captured. Take both at
            // the quiet baseline pose, zero first so the trim is relative to it.
            send(&mut serial, "zero");
            let (_, zero_replies) = listen(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(200),
                &profile,
            );
            if zero_replies.iter().any(|line| line.starts_with("err ")) {
                println!("profile {profile}: zero refused: {zero_replies:?}");
                send(&mut serial, "estop");
                continue;
            }
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
            let trim = armed.last().map_or(0.0, |sample| num(&sample.line, "lvl"));
            println!(
                "profile {profile}: baseline pitch {:.2} deg, level trim {:.2} deg, armed, recording {} ms",
                num(&baseline.line, "pitch"),
                trim,
                hold_ms
            );
            if let Some(ready) = &ready_file {
                let _ = std::fs::write(ready, "armed\n");
            }
            match &release_file {
                Some(path) => {
                    println!("profile {profile}: ARMED — RELEASE NOW, then create {path}")
                }
                None => println!("profile {profile}: ARMED — RELEASE NOW (auto-detected)"),
            }
            let _ = std::io::stdout().flush();
            let (released, release_pre_peak) = wait_for_release(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                release_file.as_deref(),
                num(&baseline.line, "pitch"),
                Duration::from_millis(release_timeout_ms),
            );
            if !released {
                continue;
            }
            let (held, _) = listen(
                &mut serial,
                &mut buffer,
                &mut csv,
                &started,
                Duration::from_millis(hold_ms),
                &profile,
            );
            let mut gate = Gate::new(pscale, *slew, db);
            gate.release_pre_peak = release_pre_peak;
            for sample in &held {
                gate.observe(&sample.line, origin_x);
                if gate.estop || gate.disarmed || gate.fall {
                    break;
                }
            }
            println!(
                "profile {profile}: {} samples, pitch max {:.2} final {:.2}, travel {:.3} m, torque {:.0} u, wheel-active {} samples, release gy pre/post {:.0}/{:.0} dps, estop {} fall {} => {} (authority {})",
                gate.samples,
                gate.max_pitch,
                gate.final_pitch,
                gate.travel,
                gate.peak_torque,
                gate.wheel_active_samples,
                gate.release_pre_peak,
                gate.release_gy_peak,
                u8::from(gate.estop),
                u8::from(gate.fall),
                if gate.release_void() {
                    "VOID - release disturbance"
                } else if gate.pass() {
                    "PASS"
                } else {
                    "FAIL"
                },
                if gate.wheel_active_samples >= MIN_WHEEL_ACTIVE_SAMPLES {
                    "yes"
                } else {
                    "NO - externally supported"
                }
            );
            let down = gate.fall || gate.estop || gate.disarmed;
            gates.push(gate);
            send(&mut serial, "estop");
            if down {
                abort = Some(format!(
                    "robot down after profile {profile}; re-stand before more releases"
                ));
                eprintln!("{}", abort.as_deref().unwrap_or_default());
                break;
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
    );
    let _ = csv.flush();
    let fixture = if fixture_qual {
        let acceptable = fixture_acceptable(&gates);
        println!(
            "FIXTURE {} — {} of {} releases below {:.1} dps",
            if acceptable { "PASS" } else { "FAIL" },
            gates
                .iter()
                .filter(|gate| {
                    !gate.release_void() && gate.release_disturbance_peak() <= FIXTURE_GY_LIMIT_DPS
                })
                .count(),
            FIXTURE_RUNS,
            FIXTURE_GY_LIMIT_DPS
        );
        Some(acceptable)
    } else {
        None
    };
    write_json(&json_path, &gates, abort.as_deref(), fixture);
    println!("csv: {}", csv_path.display());
    println!("json: {}", json_path.display());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A rope-supported body satisfies every angle and travel limit, but it
    /// must not be reported as a balance.
    #[test]
    fn a_still_body_without_wheel_authority_fails() {
        let line = "t_ms=1000 pitch=0.2 lvl=0.0 gy=0.1 L=0 R=0 age=5 mb=ok bat=38 estop=0 arm=1 mode=2 opm=0 x=0 trq=10";
        let mut gate = Gate::new(0.25, 100.0, 0.0);
        for _ in 0..300 {
            gate.observe(line, 0.0);
        }
        assert_eq!(gate.max_pitch, 0.2);
        assert_eq!(gate.wheel_active_samples, 0);
        assert!(!gate.pass());
    }

    #[test]
    fn a_hold_with_wheel_authority_passes() {
        let still = "t_ms=1000 pitch=0.2 lvl=0.0 gy=0.1 L=0 R=0 age=5 mb=ok bat=38 estop=0 arm=1 mode=2 opm=0 x=0 trq=10";
        let rolling = "t_ms=1000 pitch=0.2 lvl=0.0 gy=0.1 L=3 R=-4 age=5 mb=ok bat=38 estop=0 arm=1 mode=2 opm=0 x=0 trq=10";
        let mut gate = Gate::new(0.25, 100.0, 0.0);
        for _ in 0..300 {
            gate.observe(still, 0.0);
        }
        for _ in 0..MIN_WHEEL_ACTIVE_SAMPLES {
            gate.observe(rolling, 0.0);
        }
        assert!(gate.pass());
    }

    /// A hand release swings the body before the loop can react. The trial is
    /// invalid, not a control-law failure.
    #[test]
    fn a_violent_release_voids_the_trial() {
        let swinging = "t_ms=1000 pitch=0.2 lvl=0.0 gy=120 L=3 R=-4 age=5 mb=ok bat=38 estop=0 arm=1 mode=2 opm=0 x=0 trq=10";
        let mut gate = Gate::new(0.60, 1000.0, 0.0);
        for _ in 0..RELEASE_WINDOW_SAMPLES {
            gate.observe(swinging, 0.0);
        }
        assert!(gate.release_void());
        assert!(!gate.pass());
    }

    #[test]
    fn a_gentle_release_does_not_void_the_trial() {
        let line = "t_ms=1000 pitch=0.2 lvl=0.0 gy=8 L=3 R=-4 age=5 mb=ok bat=38 estop=0 arm=1 mode=2 opm=0 x=0 trq=10";
        let mut gate = Gate::new(0.60, 1000.0, 0.0);
        for _ in 0..300 {
            gate.observe(line, 0.0);
        }
        assert!(!gate.release_void());
        assert!(gate.pass());
    }

    fn gate_with_peak(peak: f64) -> Gate {
        let line = format!("t_ms=1000 pitch=0.2 lvl=0.0 gy={peak} L=3 R=-4 age=5 mb=ok bat=38 estop=0 arm=1 mode=2 opm=0 x=0 trq=10");
        let mut gate = Gate::new(0.25, 100.0, 0.0);
        for _ in 0..RELEASE_WINDOW_SAMPLES {
            gate.observe(&line, 0.0);
        }
        gate
    }

    #[test]
    fn three_clean_releases_qualify_the_fixture() {
        let gates: Vec<Gate> = (0..FIXTURE_RUNS).map(|_| gate_with_peak(1.0)).collect();
        assert!(fixture_acceptable(&gates));
    }

    /// Between the fixture limit and the VOID gate a trial still carries
    /// control information, but the fixture is not yet repeatable.
    #[test]
    fn a_release_above_fixture_limit_fails_fixture_qualification() {
        let gates: Vec<Gate> = (0..FIXTURE_RUNS).map(|_| gate_with_peak(8.0)).collect();
        assert!(gates.iter().all(|gate| !gate.release_void()));
        assert!(!fixture_acceptable(&gates));
    }

    #[test]
    fn a_missing_or_violent_release_fails_fixture_qualification() {
        assert!(!fixture_acceptable(&[
            gate_with_peak(1.0),
            gate_with_peak(1.0)
        ]));
        assert!(!fixture_acceptable(&[
            gate_with_peak(1.0),
            gate_with_peak(120.0),
            gate_with_peak(1.0)
        ]));
    }

    /// A hand swing before the wheels start moving is part of the release,
    /// even when it stays below the VOID threshold. Scoring only the
    /// post-release window would hide it and qualify a bad fixture.
    #[test]
    fn a_pre_release_impulse_fails_fixture_qualification() {
        let calm = "t_ms=1000 pitch=0.2 lvl=0.0 gy=0.5 L=0 R=0 age=5 mb=ok bat=38 estop=0 arm=1 mode=2 opm=0 x=0 trq=10";
        let mut gate = Gate::new(0.30, 300.0, 0.0);
        for _ in 0..RELEASE_WINDOW_SAMPLES {
            gate.observe(calm, 0.0);
        }
        gate.release_pre_peak = 16.9;
        assert_eq!(gate.release_gy_peak, 0.5);
        assert!(!gate.release_void());
        assert_eq!(gate.release_disturbance_peak(), 16.9);
        assert!(!fixture_acceptable(&[
            gate,
            gate_with_peak(1.0),
            gate_with_peak(1.0)
        ]));
    }

    #[test]
    fn a_violent_pre_release_impulse_voids_the_trial() {
        let mut gate = gate_with_peak(0.5);
        gate.release_pre_peak = 32.0;
        assert!(gate.release_void());
    }

    #[test]
    fn a_clean_release_on_both_sides_qualifies_the_fixture() {
        let mut gates: Vec<Gate> = (0..FIXTURE_RUNS).map(|_| gate_with_peak(1.0)).collect();
        for gate in &mut gates {
            gate.release_pre_peak = 0.8;
        }
        assert!(fixture_acceptable(&gates));
    }
}
