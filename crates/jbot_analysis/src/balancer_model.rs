//! Planar dynamics, torque capability, and BNO055 orientation limits for JBOT-WheeledBiped.
//!
//! Evaluates gravity torque, motor torque margins, breakaway dead zones, and swing-up
//! transitions from horizontal (laying down) to vertical upright balancing.

use std::f32::consts::PI;

/// Physical and inertial parameters of JBOT-WheeledBiped (Stage 1).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotPlantModel {
    /// Mass of body + electronics + battery (kg).
    pub body_mass_kg: f32,
    /// Combined mass of both wheels (kg).
    pub wheel_mass_kg: f32,
    /// Body pitch inertia about centre of mass (kg m^2).
    pub body_inertia_kgm2: f32,
    /// Combined wheel inertia about axle (kg m^2).
    pub wheel_inertia_kgm2: f32,
    /// Distance from wheel axle to body COM (m).
    pub com_height_m: f32,
    /// Wheel radius (m).
    pub wheel_radius_m: f32,
    /// Maximum common-mode actuator torque (Nm).
    pub max_torque_nm: f32,
    /// Drive torque dead band (Nm).
    pub deadband_nm: f32,
    /// Gravitational acceleration (m/s^2).
    pub gravity_ms2: f32,
}

impl Default for RobotPlantModel {
    fn default() -> Self {
        Self {
            body_mass_kg: 5.0,
            wheel_mass_kg: 2.3,
            body_inertia_kgm2: 0.30,
            wheel_inertia_kgm2: 0.0078,
            com_height_m: 0.25,
            wheel_radius_m: 0.0825,
            max_torque_nm: 2.5,
            deadband_nm: 0.5,
            gravity_ms2: 9.81,
        }
    }
}

impl RobotPlantModel {
    /// Static gravitational torque exerted on the wheels by the tilted body:
    /// `T_grav = m_body * g * L * sin(pitch_rad)`
    pub fn static_gravity_torque(&self, pitch_rad: f32) -> f32 {
        self.body_mass_kg * self.gravity_ms2 * self.com_height_m * pitch_rad.sin()
    }

    /// Counter-torque required to hold the robot at a static pitch angle (Nm).
    pub fn required_hold_torque(&self, pitch_deg: f32) -> f32 {
        let pitch_rad = pitch_deg * PI / 180.0;
        self.static_gravity_torque(pitch_rad)
    }

    /// Maximum stable holding angle (degrees) where motor torque can counteract gravity.
    pub fn max_stable_holding_angle_deg(&self) -> f32 {
        let ratio = self.max_torque_nm / (self.body_mass_kg * self.gravity_ms2 * self.com_height_m);
        if ratio >= 1.0 {
            90.0
        } else {
            ratio.asin() * 180.0 / PI
        }
    }

    /// Check if the robot can statically balance at the given BNO055 pitch angle.
    pub fn can_statically_hold(&self, pitch_deg: f32) -> bool {
        self.required_hold_torque(pitch_deg).abs() <= self.max_torque_nm
    }

    /// Estimate required wheel acceleration/torque profile for swing-up from lying down.
    pub fn evaluate_transition(
        &self,
        current_pitch_deg: f32,
        target_pitch_deg: f32,
    ) -> TransitionAssessment {
        let hold_torque = self.required_hold_torque(current_pitch_deg);
        let max_angle = self.max_stable_holding_angle_deg();
        let static_feasible = current_pitch_deg.abs() <= max_angle;

        let recommendation = if static_feasible {
            "Within static hold limits: standard PD balance controller can maintain position."
        } else {
            "Outside static hold limits (laying down / large tilt): requires dynamic swing-up maneuver (kick-back pulse)."
        };

        TransitionAssessment {
            current_pitch_deg,
            target_pitch_deg,
            required_torque_nm: hold_torque,
            static_feasible,
            max_stable_angle_deg: max_angle,
            recommendation,
        }
    }
}

/// Assessment results for state transitions and holding capability.
#[derive(Clone, Debug, PartialEq)]
pub struct TransitionAssessment {
    pub current_pitch_deg: f32,
    pub target_pitch_deg: f32,
    pub required_torque_nm: f32,
    pub static_feasible: bool,
    pub max_stable_angle_deg: f32,
    pub recommendation: &'static str,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gravity_torque_at_vertical() {
        let model = RobotPlantModel::default();
        let torque = model.required_hold_torque(0.0);
        assert!((torque - 0.0).abs() < 1e-5);
    }

    #[test]
    fn test_max_stable_angle() {
        let model = RobotPlantModel::default();
        let max_angle = model.max_stable_holding_angle_deg();
        // With M=5.0kg, L=0.25m, Tau_max=2.5Nm => sin(theta) = 2.5 / (5.0 * 9.81 * 0.25) = 2.5 / 12.2625 = 0.2038
        // asin(0.2038) ~ 11.75 degrees
        assert!(max_angle > 10.0 && max_angle < 15.0);
        assert!(model.can_statically_hold(5.0));
        assert!(!model.can_statically_hold(45.0));
    }

    #[test]
    fn test_transition_assessment() {
        let model = RobotPlantModel::default();
        let assessment = model.evaluate_transition(90.0, 0.0);
        assert!(!assessment.static_feasible);
        assert_eq!(assessment.current_pitch_deg, 90.0);
    }
}
