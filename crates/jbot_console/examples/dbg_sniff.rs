//! Capture the STM32 debug serial forwarded through the Pico USB console.
//!
//! Enables the bench bridge with `dbg on`, silences Pico telemetry, then logs
//! every incoming line with a millisecond timestamp. Run this before
//! power-cycling the mainboard so the boot configuration line is captured.
//!
//! Usage: dbg_sniff [--seconds 30] [--port PATH]

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serialport::SerialPort;

const DEFAULT_PORT: &str = "/dev/cu.usbmodemJBOT_PICO_011";

fn main() {
    let mut args = std::env::args().skip(1);
    let mut seconds = 30_u64;
    let mut port = DEFAULT_PORT.to_owned();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--seconds" => {
                if let Some(value) = args.next().and_then(|v| v.parse::<u64>().ok()) {
                    seconds = value;
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
        "csv/jbot-stm32-debug-{}.txt",
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_millis())
    );
    let _ = std::fs::create_dir_all("csv");
    let mut log = BufWriter::new(File::create(&name).expect("create log"));

    let send = |port: &mut Box<dyn SerialPort>, text: &str| {
        let _ = port.write_all(format!("{text}\n").as_bytes());
        let _ = port.flush();
    };

    send(&mut serial, "stream 0");
    std::thread::sleep(Duration::from_millis(200));
    send(&mut serial, "dbg on");

    let started = Instant::now();
    let mut line = Vec::new();
    let mut chunk = [0_u8; 4096];
    println!("capturing STM32 debug for {seconds}s; reset the mainboard now");
    while started.elapsed() < Duration::from_secs(seconds) {
        if let Ok(n) = serial.read(&mut chunk) {
            for &byte in &chunk[..n] {
                match byte {
                    b'\r' => {}
                    b'\n' => {
                        if !line.is_empty() {
                            if let Ok(text) = std::str::from_utf8(&line) {
                                let t = started.elapsed().as_millis();
                                println!("[{t:6}] {text}");
                                let _ = writeln!(log, "{t},{text}");
                            }
                            line.clear();
                        }
                    }
                    0 => {}
                    other => line.push(other),
                }
            }
        }
    }
    if !line.is_empty() {
        let _ = writeln!(log, "-,{}", String::from_utf8_lossy(&line));
    }

    send(&mut serial, "dbg off");
    send(&mut serial, "stream 0");
    let _ = log.flush();
    println!("recording: {name}");
}
