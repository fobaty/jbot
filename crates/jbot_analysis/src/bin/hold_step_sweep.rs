//! Full-cascade gain sweep for a 3 degree `home` step (host only).
//!
//! Uses `recovery_sim::hold_step`, which mirrors the firmware balance law
//! including its position and speed loops, with the measured dead zone and a
//! 50 ms IMU/transport delay. The firmware command slew limiter is not part of
//! `NonlinearPlant`; with 1500 units/s it is fast enough not to dominate.

use jbot_analysis::recovery_sim::{hold_step, Geometry, SimParams};

fn main() {
    let mut params = SimParams::nominal();
    params.plant = Geometry {
        com_height_m: 0.26,
        deadband_nm: 0.20,
        ..Geometry::default()
    };
    params.delay_ticks = 10;

    println!("full-cascade sweep: 3 deg step, 50 ms delay, deadband 0.20 Nm");
    println!("pscale dscale final_deg peak_deg travel_m peak_Nm");
    let mut rows: Vec<(f32, f32, f32, f32, f32, f32)> = Vec::new();
    for p in 3..=12 {
        for d in 1..=10 {
            let pscale = p as f32 / 10.0;
            let dscale = d as f32 / 10.0;
            let (final_deg, peak_deg, travel_m, peak_nm) = hold_step(&params, pscale, dscale, 3.0);
            let score = (final_deg - 3.0).abs() + peak_deg.max(0.0) * 0.1 + travel_m;
            rows.push((score, pscale, dscale, final_deg, peak_deg, peak_nm));
        }
    }
    rows.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (_, pscale, dscale, final_deg, peak_deg, peak_nm) in rows.iter().take(12) {
        println!(
            "{:6.2} {:6.2} {:9.2} {:8.2} {:8.3} {:7.2}",
            pscale, dscale, final_deg, peak_deg, 0.0_f32, peak_nm
        );
    }
}
