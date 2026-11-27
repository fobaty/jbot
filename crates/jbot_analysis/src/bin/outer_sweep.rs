//! Host sweep of the outer position/speed gains for a 3 degree step.

use jbot_analysis::recovery_sim::{hold_step_with_outer, Geometry, SimParams};

fn main() {
    let mut params = SimParams::nominal();
    params.plant = Geometry {
        com_height_m: 0.26,
        deadband_nm: 0.20,
        ..Geometry::default()
    };
    params.delay_ticks = 10;

    println!("outer-loop sweep: pscale 0.60, dscale 1.00, 3 deg step, 50 ms delay");
    println!(" posg  velg demlim final_deg peak_deg travel_m peak_Nm");
    let mut rows = Vec::new();
    for pos_index in 1..=10 {
        let posg = pos_index as f32 / 5.0; // 0.2 .. 2.0
        for vel_index in 1..=12 {
            let velg = vel_index as f32 / 4.0; // 0.25 .. 3.0
            for dem in [2.0_f32, 4.0, 6.0, 8.0] {
                let (final_deg, peak_deg, travel_m, peak_nm) =
                    hold_step_with_outer(&params, 0.60, 1.00, posg, velg, dem, 3.0);
                let score = (final_deg - 3.0).abs() + peak_deg.max(0.0) * 0.2 + travel_m * 10.0;
                rows.push((
                    score, posg, velg, dem, final_deg, peak_deg, travel_m, peak_nm,
                ));
            }
        }
    }
    rows.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (_, posg, velg, dem, final_deg, peak_deg, travel_m, peak_nm) in rows.iter().take(15) {
        println!(
            "{:5.2} {:5.2} {:6.1} {:9.2} {:8.2} {:8.3} {:7.2}",
            posg, velg, dem, final_deg, peak_deg, travel_m, peak_nm
        );
    }
}
