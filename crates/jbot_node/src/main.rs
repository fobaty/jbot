//! Main high-level executable node for JBOT running on Raspberry Pi 5.

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    println!("Starting JBOT High-Level Node (Raspberry Pi 5)...");

    let _estimator = jbot_estimation::Estimator::new();
    let _controller = jbot_control::BalanceController::new(10.0, 1.5);

    println!("Estimator and Controller initialized successfully.");
    Ok(())
}
