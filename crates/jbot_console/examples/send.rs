//! Send raw console commands and print every reply.
//!
//! Each argument is one complete command line. Useful for bench calibration
//! and for checking state between scripted runs.
//!
//! Usage: send ["stream 200"] ["zero"] ["stat"]

use std::io::{Read, Write};
use std::time::Duration;

fn main() {
    let commands: Vec<String> = std::env::args().skip(1).collect();
    if commands.is_empty() {
        eprintln!("usage: send [\"command\"] ...");
        return;
    }
    let mut port = match serialport::new("/dev/cu.usbmodemJBOT_PICO_011", 115_200)
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
    for command in commands {
        let _ = port.write_all(format!("{command}\n").as_bytes());
        let _ = port.flush();
        let mut buf = [0_u8; 4096];
        let started = std::time::Instant::now();
        while started.elapsed() < Duration::from_millis(700) {
            if let Ok(n) = port.read(&mut buf) {
                if n > 0 {
                    print!("{}", String::from_utf8_lossy(&buf[..n]));
                }
            }
        }
    }

    // Closing while the 200 Hz stream is active parks the console task: a
    // pending USB write is cancelled and leaves the IN endpoint unavailable.
    // Stop the stream and drain before the port is closed.
    let _ = port.write_all(b"stream 0\n");
    let _ = port.flush();
    let mut buf = [0_u8; 4096];
    let started = std::time::Instant::now();
    while started.elapsed() < Duration::from_millis(1_500) {
        if let Ok(n) = port.read(&mut buf) {
            if n > 0 {
                let _ = n;
            }
        }
    }
}
