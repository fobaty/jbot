//! Print a feasibility map for large-angle recovery strategies.
//!
//! Usage: `cargo run -p jbot_analysis --bin recovery-sweep --release`
//!
//! Host-only. Nothing here is flashed or sent to the robot.

use jbot_analysis::recovery_sim::{self, SimParams, Strategy};

fn parse_list(arg: &str) -> Vec<f32> {
    arg.split(',')
        .filter_map(|item| item.trim().parse::<f32>().ok())
        .collect()
}

fn main() {
    let params = SimParams::nominal();
    let mut budgets = vec![5.0_f32, 10.0, 20.0, 40.0];
    let mut angles = vec![5.0_f32, 10.0, 20.0, 30.0, 45.0, 60.0, 75.0];
    let mut rates = vec![180.0_f32, 360.0];

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--budgets" => {
                if let Some(value) = args.next() {
                    let parsed = parse_list(&value);
                    if !parsed.is_empty() {
                        budgets = parsed;
                    }
                }
            }
            "--angles" => {
                if let Some(value) = args.next() {
                    let parsed = parse_list(&value);
                    if !parsed.is_empty() {
                        angles = parsed;
                    }
                }
            }
            "--rates" => {
                if let Some(value) = args.next() {
                    let parsed = parse_list(&value);
                    if !parsed.is_empty() {
                        rates = parsed;
                    }
                }
            }
            "--help" | "-h" => {
                println!(
                    "usage: recovery-sweep [--budgets 5,10,20,40] [--angles 5,10,20,30,45,60,75] [--rates 180,360]"
                );
                return;
            }
            other => {
                eprintln!("unknown option {other}");
                return;
            }
        }
    }

    let mut strategies: Vec<Strategy> = rates
        .iter()
        .map(|&rate| Strategy::StandUp { rate_dps: rate })
        .collect();
    for &(accel, reverse, limit) in &[
        (4.0_f32, 2.0_f32, 0.5_f32),
        (8.0, 4.0, 1.0),
        (12.0, 6.0, 1.5),
        (20.0, 10.0, 2.0),
    ] {
        strategies.push(Strategy::Throw {
            accel_speed_rps: accel,
            reverse_speed_rps: reverse,
            phase_limit_s: limit,
        });
    }

    println!(
        "geometry: body={} kg wheels={} kg L={} m r={} m deadband={} Nm delay={} ticks",
        params.plant.body_mass_kg,
        params.plant.wheel_mass_kg,
        params.plant.com_height_m,
        params.plant.wheel_radius_m,
        params.plant.deadband_nm,
        params.delay_ticks
    );
    println!(
        "guards: travel<={} m speed<={} m/s",
        recovery_sim::MAX_TRAVEL_M,
        recovery_sim::MAX_SPEED_MPS
    );
    println!();
    println!(
        "{:<48} {:>7} {:>7} {:>5} {:>9} {:>8} {:>9} {:>9} {:>7} {:>7}",
        "strategy", "Nm", "start", "ok", "pitch_d", "travel", "peakv", "peakNm", "guard", "sec"
    );

    for (strategy, budget, angle, outcome) in
        recovery_sim::sweep(params, &strategies, &budgets, &angles)
    {
        println!(
            "{:<48} {:>7.1} {:>7.1} {:>5} {:>9.2} {:>8.2} {:>9.2} {:>9.2} {:>7} {:>7.1}",
            format!("{strategy:?}"),
            budget,
            angle,
            if outcome.success { "yes" } else { "no" },
            outcome.final_pitch_deg,
            outcome.travel_m,
            outcome.peak_speed_mps,
            outcome.peak_torque_nm,
            if outcome.guard_hit { "hit" } else { "-" },
            outcome.duration_s
        );
    }
}
