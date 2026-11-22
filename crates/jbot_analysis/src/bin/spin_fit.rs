//! Offline torque-scale fit for a recorded spin test.
//!
//! Usage: `cargo run -p jbot_analysis --bin spin-fit -- csv/jbot-telemetry-<epoch>.csv`

use std::path::PathBuf;

use jbot_analysis::session;
use jbot_analysis::spinfit::{self, WHEEL_INERTIA_KGM2};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = match args.next() {
        Some(value) if !matches!(value.as_str(), "-h" | "--help") => PathBuf::from(value),
        _ => {
            println!("usage: spin-fit <telemetry.csv>");
            return;
        }
    };
    let mut inertia = WHEEL_INERTIA_KGM2;
    while let Some(arg) = args.next() {
        if arg == "--inertia" {
            if let Some(value) = args.next().and_then(|v| v.parse::<f64>().ok()) {
                inertia = value;
            }
        }
    }

    let session = match session::read_session(&path) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("{}: {error}", path.display());
            std::process::exit(1);
        }
    };
    let report = spinfit::fit(&session.samples, inertia);

    println!("file: {}", path.display());
    println!(
        "samples: {} ({} spin plateaus)",
        session.samples.len(),
        report.levels.len()
    );
    println!("wheel inertia: {inertia} kg m^2");
    println!();
    println!(
        "{:>8} {:>12} {:>11} {:>13} {:>8}",
        "units", "alpha rad/s2", "torque Nm", "units/Nm", "samples"
    );
    for level in &report.levels {
        println!(
            "{:>8.0} {:>12.2} {:>11.4} {:>13.1} {:>8}",
            level.units, level.alpha_rps2, level.torque_nm, level.units_per_nm, level.samples
        );
    }
    println!();
    println!(
        "breakaway bracket: stuck <= {:?} units, moving >= {:?} units",
        report.stuck_max_units, report.moving_min_units
    );
    if report.levels.is_empty() {
        println!("no spin-up plateau found: check that the motor test was used while disarmed");
    }
}
