//! Main high-level executable node for JBOT running on Raspberry Pi 5.

use anyhow::Context;
use jbot_common::config::{config_hash, RobotConfig};

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
    println!("Hardware drivers and middleware are not wired yet.");

    Ok(())
}
