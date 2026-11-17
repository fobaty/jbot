//! JBOT Pico 2 W desktop console.
//!
//! A native Rust bench application for the USB CDC console described in
//! `docs/pico-bridge.md` section 8.1. It shows live BNO055 attitude, a horizon
//! and compass, control-loop and power instruments, drive controls, and the
//! firmware event log. The serial port is owned by a dedicated thread; the UI
//! itself never performs blocking I/O.

mod app;
mod attitude;
mod axiscal;
mod link;
mod protocol;
mod theme;

use eframe::egui;

fn main() -> eframe::Result {
    let mut demo = false;
    let mut port = None;
    let mut stream_hz = protocol::DEFAULT_STREAM_HZ;
    let arguments: Vec<String> = std::env::args().skip(1).collect();
    let mut index = 0;

    while index < arguments.len() {
        match arguments[index].as_str() {
            "--demo" => demo = true,
            "--list" => {
                for name in link::available_ports() {
                    println!("{name}");
                }
                return Ok(());
            }
            "--help" | "-h" => {
                print_help();
                return Ok(());
            }
            "--port" => {
                index += 1;
                match arguments.get(index) {
                    Some(value) => port = Some(value.clone()),
                    None => {
                        eprintln!("--port needs a value");
                        std::process::exit(2);
                    }
                }
            }
            "--stream" => {
                index += 1;
                match arguments
                    .get(index)
                    .and_then(|value| value.parse::<u32>().ok())
                {
                    Some(value) => stream_hz = value.clamp(1, protocol::MAX_STREAM_HZ),
                    None => {
                        eprintln!("--stream needs a rate in hertz, for example --stream 20");
                        std::process::exit(2);
                    }
                }
            }
            other => {
                eprintln!("unknown argument: {other}");
                print_help();
                std::process::exit(2);
            }
        }
        index += 1;
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1320.0, 880.0])
            .with_min_inner_size([1000.0, 680.0])
            .with_title("JBOT Pico Console"),
        ..Default::default()
    };

    let startup = app::Startup {
        demo,
        port,
        stream_hz,
    };

    eframe::run_native(
        "JBOT Pico Console",
        options,
        Box::new(move |cc| Ok(Box::new(app::ConsoleApp::new(cc, startup)))),
    )
}

fn print_help() {
    println!(
        "jbot-console - JBOT Pico 2 W desktop console\n\
         \n\
         Usage: jbot-console [OPTIONS]\n\
         \n\
         Options:\n\
           --demo           run with synthetic attitude and telemetry\n\
           --port <path>    serial port (for example /dev/cu.usbmodemJBOT_PICO_011)\n\
           --stream <hz>    telemetry rate requested on connect (default 20)\n\
           --list           list serial ports and exit\n\
           -h, --help       show this help"
    );
}
