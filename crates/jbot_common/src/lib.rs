//! Common data structures, kinematic models, and message types for JBOT.

pub mod config;
pub mod kinematics;

pub use config::{
    config_hash, ConfigError, ControlConfig, DriveConfig, HipConfig, LegConfig, MechanicalConfig,
    RobotConfig, CONFIG_SCHEMA_VERSION, MIN_LEG_HEIGHT_RANGE_M,
};
pub use kinematics::{
    AssemblyBranch, BodyTwist, DifferentialDrive, KinematicsError, LegGeometry, LegKinematics,
    LegPose, WheelSpeeds,
};

pub struct RobotState {
    pub timestamp: u64,
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub velocity: f32,
    pub position: f32,
}

impl RobotState {
    /// Create a new neutral robot state.
    pub fn new() -> Self {
        Self {
            timestamp: 0,
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            velocity: 0.0,
            position: 0.0,
        }
    }
}

impl Default for RobotState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_robot_state_new() {
        let state = RobotState::new();
        assert_eq!(state.timestamp, 0);
        assert_eq!(state.pitch, 0.0);
    }
}
