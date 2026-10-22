//! Main high-level executable node for JBOT running on Raspberry Pi 5.

use anyhow::Context;
use jbot_common::config::{config_hash, RobotConfig};
use jbot_control::{solve_discrete_lqr, LqrWeights, WheeledInvertedPendulum, WipParameters};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    println!("Starting JBOT High-Level Node (Raspberry Pi 5)...");

    let path = std::env::var("JBOT_CONFIG").unwrap_or_else(|_| "config/robot.toml".to_owned());
    let raw = std::fs::read_to_string(&path)
        .with_context(|| format!("failed to read configuration from {path}"))?;
    let config = RobotConfig::from_toml_str(&raw)
        .with_context(|| format!("invalid configuration in {path}"))?;

    println!(
        "Loaded configuration {path} (schema {}, hash {:016x})",
        config.schema_version,
        config_hash(raw.as_bytes())
    );

    let _drive = config
        .to_differential_drive()
        .context("failed to build differential-drive model")?;
    let _leg = config
        .to_leg_kinematics()
        .context("failed to build four-bar leg model")?;
    let wip = WheeledInvertedPendulum::new(WipParameters::from_config(&config))
        .context("failed to build the reduced-order WIP model")?;
    let discrete = wip
        .discrete(1.0 / config.control.balance_rate_hz)
        .context("failed to discretize the WIP model")?;
    let lqr = solve_discrete_lqr(&discrete, &LqrWeights::default())
        .context("failed to solve the discrete LQR problem")?;

    println!(
        "WIP 2D model: dt {:.6} s, controllable {}, LQR iterations {}, K [{:.5}, {:.5}, {:.5}, {:.5}]",
        discrete.dt_s,
        discrete.is_controllable(),
        lqr.iterations,
        lqr.gain[(0, 0)],
        lqr.gain[(0, 1)],
        lqr.gain[(0, 2)],
        lqr.gain[(0, 3)]
    );

    println!("Estimator, drive, leg, and controller initialized successfully.");
    Ok(())
}
