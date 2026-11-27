//! Print gain candidates for a 3 degree reference step.
//!
//! Usage: gain-sweep [--delay-ticks 10] [--deadband 0.20] [--slew 1500] [--step 3]

use jbot_analysis::gain_sweep;
use jbot_analysis::recovery_sim::Geometry;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |key: &str, default: &str| -> String {
        args.iter()
            .position(|arg| arg == key)
            .and_then(|index| args.get(index + 1))
            .cloned()
            .unwrap_or_else(|| default.to_owned())
    };
    let delay_ticks = value("--delay-ticks", "10").parse::<usize>().unwrap_or(10);
    let deadband = value("--deadband", "0.20").parse::<f32>().unwrap_or(0.20);
    let slew = value("--slew", "1500").parse::<f32>().unwrap_or(1500.0);
    let step = value("--step", "3").parse::<f32>().unwrap_or(3.0);

    let geometry = Geometry::default();
    let candidates = gain_sweep::sweep(&geometry, delay_ticks, deadband, slew, step);
    println!(
        "plant floor {:.2} Nm/rad, kp nominal {:.2}, delay {} ticks ({:.0} ms), deadband {:.2} Nm",
        geometry.floor_nm_per_rad(),
        4.0 * geometry.floor_nm_per_rad(),
        delay_ticks,
        delay_ticks as f32 * 5.0,
        deadband
    );
    println!("pscale dscale   kp_nm   kd_nm   final_deg peak_deg settle_s peak_Nm");
    for candidate in candidates.iter().take(12) {
        println!(
            "{:6.2} {:6.2} {:7.2} {:7.2} {:10.2} {:8.2} {:8.2} {:7.2}",
            candidate.pscale,
            candidate.dscale,
            candidate.kp_nm_per_rad,
            candidate.kd_nm_per_rad_s,
            candidate.final_pitch_deg,
            candidate.peak_pitch_deg,
            candidate.settle_s,
            candidate.peak_torque_nm
        );
    }
}
