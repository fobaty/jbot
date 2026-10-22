//! Hardware abstraction layer for communication with sensors and actuators.

pub struct HardwareInterface {
    _port: String,
}

impl HardwareInterface {
    /// Initialize new hardware interface with given port.
    pub fn new(port: &str) -> Self {
        Self {
            _port: port.to_string(),
        }
    }

    /// Connect to the underlying device (MCU / CAN bridge).
    pub async fn connect(&self) -> Result<(), anyhow::Error> {
        // Placeholder for serial/CAN connection logic
        Ok(())
    }
}
