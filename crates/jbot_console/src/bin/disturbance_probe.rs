//! Single-session disturbance probe for a robot that already stands in its own
//! equilibrium. The wheel-turning baseline of `recovery_driver` rejects such a
//! machine, so this driver arms from a quiet still pose, applies one `home`
//! step, and records the response in the same session.
//!
//! Safety: `--run --secured` is mandatory, every run ends with `home 0` and
//! `estop`, and the firmware watchdog stays authoritative. No firmware change.
//!
//! `--minimal` reduces the runtime control law to the mjbots hoverbot minimum:
//! pitch PD on the mapped gyro plus the speed reference. After the normal knob
//! setup it forces feed-forward, deadband, calibration, and dither off, zeroes
//! `iscale`, and disables the outer position/velocity and demand-limit loops.
//! Firmware defaults and persisted state are not changed.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const BAUD: u32 = 115_200;
const HEADER: &str = concat!(
    "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,",
    "left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,",
    "test_right_units,ref_deg,demand_deg,trim_deg,position_m,mode,phase,arm"
);

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

fn pump(
    serial: &mut Box<dyn SerialPort>,
    buffer: &mut Vec<u8>,
    csv: &mut BufWriter<File>,
    started: &Instant,
    duration: Duration,
    phase: &str,
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
                            let values = [
                                started.elapsed().as_millis().to_string(),
                                num(&text, "step").to_string(),
                                num(&text, "loop").to_string(),
                                u8::from(field(&text, "imu") == Some("ok")).to_string(),
                                num(&text, "pitch").to_string(),
                                num(&text, "roll").to_string(),
                                num(&text, "hdg").to_string(),
                                num(&text, "gy").to_string(),
                                num(&text, "gx").to_string(),
                                num(&text, "gz").to_string(),
                                num(&text, "L").to_string(),
                                num(&text, "R").to_string(),
                                num(&text, "bat").to_string(),
                                num(&text, "age").to_string(),
                                num(&text, "estop").to_string(),
                                num(&text, "vset").to_string(),
                                num(&text, "wset").to_string(),
                                num(&text, "trq").to_string(),
                                num(&text, "tsl").to_string(),
                                num(&text, "tsr").to_string(),
                                num(&text, "ref").to_string(),
                                num(&text, "dem").to_string(),
                                num(&text, "trm").to_string(),
                                num(&text, "x").to_string(),
                                num(&text, "mode").to_string(),
                                phase.to_owned(),
                                num(&text, "arm").to_string(),
                            ];
                            let _ = writeln!(csv, "{}", values.join(","));
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

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|arg| arg == "--help" || arg == "-h") {
        println!("usage: disturbance_probe --run --secured [--port PATH] [--angle 3] [--record-ms 8000] [--pscale 0.25] [--dscale 1.00] [--slew 100] [--db 0.20] [--cal 35 45] [--posg V] [--velg V] [--demlim DEG] [--isolate-outer] [--minimal]");
        return;
    }
    let secured = args.iter().any(|arg| arg == "--secured");
    let isolate_outer = args.iter().any(|arg| arg == "--isolate-outer");
    let minimal = args.iter().any(|arg| arg == "--minimal");
    if !args.iter().any(|arg| arg == "--run") {
        eprintln!("pass --run --secured");
        return;
    }
    if !secured {
        eprintln!("refusing to energise: pass --secured");
        return;
    }
    let value = |key: &str, default: &str| -> String {
        args.iter()
            .position(|arg| arg == key)
            .and_then(|index| args.get(index + 1))
            .cloned()
            .unwrap_or_else(|| default.to_owned())
    };
    let port_path = value("--port", "");
    let angle = value("--angle", "3");
    let record_ms = value("--record-ms", "8000");
    let pscale = value("--pscale", "0.25");
    let dscale = value("--dscale", "1.00");
    let iscale = value("--iscale", "1.00");
    let slew = value("--slew", "100");
    let db = value("--db", "0.20");
    let posg = value("--posg", "");
    let velg = value("--velg", "");
    let demlim = value("--demlim", "");
    let cal: Vec<String> = value("--cal", "35 45")
        .split_whitespace()
        .map(str::to_owned)
        .collect();
    let cal_l = cal.first().cloned().unwrap_or_else(|| "35".to_owned());
    let cal_r = cal.get(1).cloned().unwrap_or_else(|| "45".to_owned());

    let path = pick_port(if port_path.is_empty() {
        None
    } else {
        Some(port_path)
    });
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
    let stem = format!(
        "csv/jbot-disturbance-{}",
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis()
    );
    let mut csv = match File::create(format!("{stem}.csv")) {
        Ok(file) => BufWriter::new(file),
        Err(error) => {
            eprintln!("cannot create csv: {error}");
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
    send(&mut serial, "clear");
    send(&mut serial, "ff on");
    send(&mut serial, &format!("db {db}"));
    send(&mut serial, &format!("cal {cal_l} {cal_r}"));
    send(&mut serial, "dither off");
    send(&mut serial, &format!("pscale {pscale}"));
    send(&mut serial, &format!("dscale {dscale}"));
    send(&mut serial, &format!("iscale {iscale}"));
    send(&mut serial, &format!("slew {slew}"));
    if isolate_outer && !minimal {
        send(&mut serial, "posg off");
        send(&mut serial, "velg off");
        send(&mut serial, "demlim off");
        println!("outer position/velocity loops isolated for this session");
    } else if !minimal {
        if !posg.is_empty() {
            send(&mut serial, &format!("posg {posg}"));
        }
        if !velg.is_empty() {
            send(&mut serial, &format!("velg {velg}"));
        }
        if !demlim.is_empty() {
            send(&mut serial, &format!("demlim {demlim}"));
        }
    }
    if minimal {
        send(&mut serial, "ff off");
        send(&mut serial, "db off");
        send(&mut serial, "cal off");
        send(&mut serial, "dither off");
        send(&mut serial, "iscale 0");
        send(&mut serial, "posg off");
        send(&mut serial, "velg off");
        send(&mut serial, "demlim off");
        println!("minimal control law: pitch PD on mapped gyro + speed reference only");
    }
    send(&mut serial, "level");
    send(&mut serial, "home 0");
    send(&mut serial, "arm");

    let mut armed = false;
    for _ in 0..20 {
        let lines = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(50),
            "arm",
        );
        if lines.iter().any(|line| num(line, "arm") > 0.5) {
            armed = true;
            break;
        }
    }
    if !armed {
        eprintln!("arm refused; estop sent");
        send(&mut serial, "estop");
        let _ = csv.flush();
        return;
    }
    println!("armed; applying home {angle} deg");
    send(&mut serial, &format!("home {angle}"));

    let record = Duration::from_millis(record_ms.parse::<u64>().unwrap_or(8_000));
    let mut peak_pitch = 0.0_f64;
    let mut min_pitch = f64::INFINITY;
    let mut max_pitch = f64::NEG_INFINITY;
    let mut armed_samples = 0_u64;
    let mut max_rpm = 0.0_f64;
    for _ in 0..(record.as_millis() / 50) {
        let lines = pump(
            &mut serial,
            &mut buffer,
            &mut csv,
            &started,
            Duration::from_millis(50),
            "disturbance",
        );
        for line in lines.iter().filter(|line| is_telemetry(line)) {
            let pitch = num(line, "pitch");
            peak_pitch = peak_pitch.max(pitch.abs());
            min_pitch = min_pitch.min(pitch);
            max_pitch = max_pitch.max(pitch);
            armed_samples += u64::from(num(line, "arm") > 0.5);
            max_rpm = max_rpm.max(num(line, "L").abs().max(num(line, "R").abs()));
        }
    }
    send(&mut serial, "home 0");
    send(&mut serial, "estop");
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
    println!(
        "pitch {min_pitch:.2}..{max_pitch:.2} deg, peak {peak_pitch:.2}, armed samples {armed_samples}, max wheel {max_rpm:.0} rpm; csv {stem}.csv"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_parser_reads_the_fields() {
        let line = "T step=1 loop=200 imu=ok pitch=2.0 lvl=1.0 gy=3 L=4 R=-5 trq=9 arm=1 mode=2";
        assert!(is_telemetry(line));
        assert_eq!(num(line, "pitch"), 2.0);
        assert_eq!(num(line, "arm"), 1.0);
        assert!(!is_telemetry("ok ping"));
    }
}
