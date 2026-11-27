//! Plant identification and parameter derivation from telemetry sessions.
//!
//! Bridges the gap between raw telemetry logs and physical parameters
//! (restoring torque constant, breakaway dead zone, and effective inertia)
//! required for gain tuning.

use crate::metrics::{Breakaway, LawFit};

/// Derived plant parameters from telemetry analysis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdentifiedPlant {
    /// Estimated torque per 1000 units, Nm.
    pub torque_per_1000_nm: f64,
    /// Estimated dead-zone torque lower bound, Nm.
    pub dead_zone_min_nm: f64,
    /// Estimated dead-zone torque upper bound, Nm.
    pub dead_zone_max_nm: f64,
    /// Estimated restoring torque constant (kp), Nm/rad.
    pub gravity_torque_per_rad: f64,
}

impl IdentifiedPlant {
    /// Derive plant parameters from a law fit and breakaway brackets.
    ///
    /// Assumes standard full scale of 1000 units and 5.0 Nm nominal reference.
    pub fn from_fit_and_breakaway(fit: &LawFit, breakaway: &Breakaway) -> Self {
        // Nominal conversion factor in current firmware: 1000 units = 5.0 Nm
        let nominal_scale_nm = 5.0;
        let inferred_kp_nm = fit.kp_units_per_rad * (nominal_scale_nm / 1000.0);

        Self {
            torque_per_1000_nm: nominal_scale_nm,
            dead_zone_min_nm: breakaway.stuck_max_units * (nominal_scale_nm / 1000.0),
            dead_zone_max_nm: breakaway.moving_min_units * (nominal_scale_nm / 1000.0),
            gravity_torque_per_rad: inferred_kp_nm,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plant_parameters_are_derived_correctly() {
        let fit = LawFit {
            kp_units_per_rad: 1200.0,
            kd_units_per_rad_s: 200.0,
            r2: 0.99,
            rms_units: 5.0,
            samples: 100,
            saturated_fraction: 0.0,
            torque_limit_units: 1000.0,
        };
        let breakaway = Breakaway {
            stuck_max_units: 100.0,
            moving_min_units: 300.0,
            samples: 50,
            from_applied: false,
        };
        let plant = IdentifiedPlant::from_fit_and_breakaway(&fit, &breakaway);
        assert!((plant.gravity_torque_per_rad - 6.0).abs() < 1e-9);
        assert!((plant.dead_zone_min_nm - 0.5).abs() < 1e-9);
        assert!((plant.dead_zone_max_nm - 1.5).abs() < 1e-9);
    }
}
