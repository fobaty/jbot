//! Typed robot configuration, validation, and provenance.
//!
//! The schema is deliberately strict: unknown fields, unsupported schema
//! versions, and parameters that make the mechanism unassemblable are rejected
//! before the configuration reaches a controller or a simulator.

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::kinematics::{
    AssemblyBranch, DifferentialDrive, KinematicsError, LegGeometry, LegKinematics,
};

/// Version of the on-disk configuration schema.
pub const CONFIG_SCHEMA_VERSION: u32 = 1;

/// Minimum wheel-axle vertical travel required by the leg concept.
pub const MIN_LEG_HEIGHT_RANGE_M: f64 = 0.1;

/// Complete robot description loaded from `config/robot.toml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotConfig {
    pub schema_version: u32,
    pub mechanical: MechanicalConfig,
    pub leg: LegConfig,
    pub drive: DriveConfig,
    pub hip: HipConfig,
    pub control: ControlConfig,
}

/// Mass-inertial properties of the body and wheels.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MechanicalConfig {
    pub body_mass_kg: f64,
    pub body_pitch_inertia_kg_m2: f64,
    pub wheel_mass_kg: f64,
    pub wheel_radius_m: f64,
    pub wheel_inertia_kg_m2: f64,
    pub track_width_m: f64,
    pub com_height_m: f64,
}

/// Geometry and travel of one planar four-bar leg.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LegConfig {
    pub base_m: f64,
    pub crank_m: f64,
    pub rocker_m: f64,
    pub coupler_m: f64,
    pub fork_de_m: f64,
    pub fork_ce_m: f64,
    pub rocker_branch: AssemblyBranch,
    pub fork_branch: AssemblyBranch,
    pub hip_min_rad: f64,
    pub hip_max_rad: f64,
    pub hip_gear_ratio: f64,
    pub spring_stiffness_nm_per_rad: f64,
    pub spring_travel_rad: f64,
}

/// Wheel actuator limits used by the drive model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriveConfig {
    pub max_wheel_torque_nm: f64,
    pub max_wheel_speed_rad_s: f64,
    pub max_wheel_accel_rad_s2: f64,
}

/// Hip actuator limits used by the height and roll allocation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HipConfig {
    pub max_torque_nm: f64,
    pub max_speed_rad_s: f64,
}

/// Loop timing and safety thresholds shared by node and firmware.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlConfig {
    pub balance_rate_hz: f64,
    pub tilt_limit_rad: f64,
    pub command_timeout_ms: u32,
}

impl RobotConfig {
    /// Parse and validate a TOML document.
    pub fn from_toml_str(input: &str) -> Result<Self, ConfigError> {
        let config: Self = toml::from_str(input)?;
        config.validate()?;
        Ok(config)
    }

    /// Validate every parameter and the geometry it implies.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.schema_version != CONFIG_SCHEMA_VERSION {
            return Err(ConfigError::UnsupportedSchema {
                found: self.schema_version,
                supported: CONFIG_SCHEMA_VERSION,
            });
        }

        self.mechanical.validate()?;
        self.leg.validate()?;
        self.drive.validate()?;
        self.hip.validate()?;
        self.control.validate()?;

        let leg = self.leg.to_kinematics()?;
        let (minimum, maximum) = leg.vertical_range()?;
        let range = maximum - minimum;
        if range < MIN_LEG_HEIGHT_RANGE_M {
            return Err(ConfigError::HeightRangeTooSmall { range_m: range });
        }

        Ok(())
    }

    /// Build the differential-drive model from the mechanical section.
    pub fn to_differential_drive(&self) -> Result<DifferentialDrive, ConfigError> {
        Ok(DifferentialDrive::new(self.mechanical.track_width_m)?)
    }

    /// Build the four-bar leg model from the leg section.
    pub fn to_leg_kinematics(&self) -> Result<LegKinematics, ConfigError> {
        self.leg.to_kinematics()
    }
}

impl MechanicalConfig {
    fn validate(&self) -> Result<(), ConfigError> {
        positive("mechanical.body_mass_kg", self.body_mass_kg)?;
        positive(
            "mechanical.body_pitch_inertia_kg_m2",
            self.body_pitch_inertia_kg_m2,
        )?;
        positive("mechanical.wheel_mass_kg", self.wheel_mass_kg)?;
        positive("mechanical.wheel_radius_m", self.wheel_radius_m)?;
        positive("mechanical.wheel_inertia_kg_m2", self.wheel_inertia_kg_m2)?;
        positive("mechanical.track_width_m", self.track_width_m)?;
        positive("mechanical.com_height_m", self.com_height_m)
    }
}

impl LegConfig {
    /// Build the leg model, including the assembly and travel checks.
    pub fn to_kinematics(&self) -> Result<LegKinematics, ConfigError> {
        let geometry = LegGeometry {
            base_m: self.base_m,
            crank_m: self.crank_m,
            rocker_m: self.rocker_m,
            coupler_m: self.coupler_m,
            fork_de_m: self.fork_de_m,
            fork_ce_m: self.fork_ce_m,
            rocker_branch: self.rocker_branch,
            fork_branch: self.fork_branch,
        };
        Ok(LegKinematics::new(
            geometry,
            self.hip_min_rad,
            self.hip_max_rad,
        )?)
    }

    fn validate(&self) -> Result<(), ConfigError> {
        positive("leg.base_m", self.base_m)?;
        positive("leg.crank_m", self.crank_m)?;
        positive("leg.rocker_m", self.rocker_m)?;
        positive("leg.coupler_m", self.coupler_m)?;
        positive("leg.fork_de_m", self.fork_de_m)?;
        positive("leg.fork_ce_m", self.fork_ce_m)?;
        positive("leg.hip_gear_ratio", self.hip_gear_ratio)?;
        non_negative(
            "leg.spring_stiffness_nm_per_rad",
            self.spring_stiffness_nm_per_rad,
        )?;
        non_negative("leg.spring_travel_rad", self.spring_travel_rad)?;
        if self.spring_stiffness_nm_per_rad > 0.0 && self.spring_travel_rad <= 0.0 {
            return Err(ConfigError::InvalidField {
                field: "leg.spring_travel_rad",
                reason: "must be positive when a spring is fitted".to_owned(),
            });
        }
        Ok(())
    }
}

impl DriveConfig {
    fn validate(&self) -> Result<(), ConfigError> {
        positive("drive.max_wheel_torque_nm", self.max_wheel_torque_nm)?;
        positive("drive.max_wheel_speed_rad_s", self.max_wheel_speed_rad_s)?;
        positive("drive.max_wheel_accel_rad_s2", self.max_wheel_accel_rad_s2)
    }
}

impl HipConfig {
    fn validate(&self) -> Result<(), ConfigError> {
        positive("hip.max_torque_nm", self.max_torque_nm)?;
        positive("hip.max_speed_rad_s", self.max_speed_rad_s)
    }
}

impl ControlConfig {
    fn validate(&self) -> Result<(), ConfigError> {
        positive("control.balance_rate_hz", self.balance_rate_hz)?;
        if !self.tilt_limit_rad.is_finite()
            || self.tilt_limit_rad <= 0.0
            || self.tilt_limit_rad >= std::f64::consts::FRAC_PI_2
        {
            return Err(ConfigError::InvalidValue {
                field: "control.tilt_limit_rad",
                value: self.tilt_limit_rad,
            });
        }
        if self.command_timeout_ms == 0 {
            return Err(ConfigError::InvalidField {
                field: "control.command_timeout_ms",
                reason: "must be positive".to_owned(),
            });
        }
        Ok(())
    }
}

/// Stable 64-bit FNV-1a hash of the raw configuration bytes.
///
/// The hash is logged at startup so a trace can be tied to the exact file that
/// produced it, including comments and whitespace.
pub fn config_hash(bytes: &[u8]) -> u64 {
    const OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    let mut hash = OFFSET_BASIS;
    for &byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

/// Configuration parsing and validation failures.
#[derive(Debug)]
pub enum ConfigError {
    Parse(toml::de::Error),
    UnsupportedSchema { found: u32, supported: u32 },
    InvalidValue { field: &'static str, value: f64 },
    InvalidField { field: &'static str, reason: String },
    HeightRangeTooSmall { range_m: f64 },
    Kinematics(KinematicsError),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Parse(error) => write!(f, "failed to parse configuration: {error}"),
            Self::UnsupportedSchema { found, supported } => write!(
                f,
                "unsupported configuration schema {found}; expected {supported}"
            ),
            Self::InvalidValue { field, value } => {
                write!(f, "invalid configuration value {field}={value}")
            }
            Self::InvalidField { field, reason } => {
                write!(f, "invalid configuration field {field}: {reason}")
            }
            Self::HeightRangeTooSmall { range_m } => write!(
                f,
                "four-bar leg height range {range_m} m is below the {MIN_LEG_HEIGHT_RANGE_M} m minimum"
            ),
            Self::Kinematics(error) => write!(f, "invalid leg geometry: {error}"),
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Parse(error) => Some(error),
            Self::Kinematics(error) => Some(error),
            _ => None,
        }
    }
}

impl From<toml::de::Error> for ConfigError {
    fn from(error: toml::de::Error) -> Self {
        Self::Parse(error)
    }
}

impl From<KinematicsError> for ConfigError {
    fn from(error: KinematicsError) -> Self {
        Self::Kinematics(error)
    }
}

fn positive(field: &'static str, value: f64) -> Result<(), ConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(ConfigError::InvalidValue { field, value })
    }
}

fn non_negative(field: &'static str, value: f64) -> Result<(), ConfigError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(ConfigError::InvalidValue { field, value })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASELINE: &str = include_str!("../../../config/robot.toml");

    fn baseline() -> RobotConfig {
        RobotConfig::from_toml_str(BASELINE).expect("baseline configuration must be valid")
    }

    #[test]
    fn baseline_config_parses_and_builds_models() {
        let config = baseline();
        assert_eq!(config.schema_version, CONFIG_SCHEMA_VERSION);

        let drive = config
            .to_differential_drive()
            .expect("track width is valid");
        assert!((drive.track_width_m() - config.mechanical.track_width_m).abs() <= 1e-12);

        let leg = config.to_leg_kinematics().expect("leg geometry is valid");
        let (minimum, maximum) = leg.vertical_range().expect("leg assembles");
        assert!(maximum - minimum >= MIN_LEG_HEIGHT_RANGE_M);
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let invalid = format!("{BASELINE}\nunexpected_field = 1\n");
        assert!(matches!(
            RobotConfig::from_toml_str(&invalid),
            Err(ConfigError::Parse(_))
        ));
    }

    #[test]
    fn unsupported_schema_is_rejected() {
        let invalid = BASELINE.replace("schema_version = 1", "schema_version = 2");
        let result = RobotConfig::from_toml_str(&invalid);
        assert!(matches!(
            result,
            Err(ConfigError::UnsupportedSchema {
                found: 2,
                supported: CONFIG_SCHEMA_VERSION
            })
        ));
    }

    #[test]
    fn non_positive_mass_is_rejected() {
        let mut config = baseline();
        config.mechanical.body_mass_kg = -1.0;
        assert!(matches!(
            config.validate(),
            Err(ConfigError::InvalidValue {
                field: "mechanical.body_mass_kg",
                ..
            })
        ));
    }

    #[test]
    fn unsupported_spring_configuration_is_rejected() {
        let mut config = baseline();
        config.leg.spring_stiffness_nm_per_rad = 1.0;
        config.leg.spring_travel_rad = 0.0;
        assert!(matches!(
            config.validate(),
            Err(ConfigError::InvalidField {
                field: "leg.spring_travel_rad",
                ..
            })
        ));
    }

    #[test]
    fn unassemblable_leg_is_rejected() {
        let mut config = baseline();
        config.leg.coupler_m = 10.0;
        assert!(matches!(config.validate(), Err(ConfigError::Kinematics(_))));
    }

    #[test]
    fn height_range_is_enforced() {
        let mut config = baseline();
        config.leg.hip_min_rad = -0.95;
        config.leg.hip_max_rad = -0.70;
        assert!(matches!(
            config.validate(),
            Err(ConfigError::HeightRangeTooSmall { .. })
        ));
    }

    #[test]
    fn config_hash_is_stable_and_sensitive() {
        assert_eq!(config_hash(b"abc"), config_hash(b"abc"));
        assert_ne!(config_hash(b"abc"), config_hash(b"abd"));
        assert_eq!(config_hash(b""), 0xcbf2_9ce4_8422_2325);
    }
}
