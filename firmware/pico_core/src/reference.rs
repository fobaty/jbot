//! Pitch reference generator for the smooth home-pose transition.
//!
//! The balance reference ("home") must be reached without a torque step. This
//! module owns the reference trajectory between the currently applied angle
//! and the requested target. It is pure computation: no I/O, no allocation,
//! no floating-point modes, so it runs inside the 200 Hz step.
//!
//! Profile: acceleration-limited approach. The desired speed towards the
//! target is `sqrt(2 * a_max * |err|)` in the direction of the error,
//! saturated to `±v_max`. This brakes smoothly into the target instead of
//! riding a trapezoid into a stop. The reference never passes the target:
//! the last integration step is clamped so the overshoot is exactly zero.
//!
//! Usage contract: on arming, call [`PitchReference::reset`] with the
//! *current measured pitch*, so the initial error is zero and the first
//! balance correction ramps in from the present need instead of from zero.
//!
//! Sign convention matches the balance law: positive pitch leans forward.

/// Default maximum reference rate: 10 degrees per second.
pub const DEFAULT_V_MAX_RAD_S: f32 = core::f32::consts::PI / 18.0;
/// Default maximum reference acceleration: 20 degrees per second squared.
pub const DEFAULT_A_MAX_RAD_S: f32 = core::f32::consts::PI / 9.0;
/// Hard cap on the reference rate, used to clamp operator-provided values.
pub const V_MAX_CAP_RAD_S: f32 = core::f32::consts::FRAC_PI_3; // 60 degrees per second
/// Largest absolute reference target: the tilt safety limit from
/// `config/robot.toml` (`tilt_limit_rad`), 30 degrees.
pub const TARGET_LIMIT_RAD: f32 = core::f32::consts::FRAC_PI_6;

/// One step of the reference trajectory.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReferenceStep {
    /// Reference angle to feed the balance law, radians.
    pub ref_rad: f32,
    /// Reference velocity, radians per second. Telemetry use.
    pub velocity_rad_s: f32,
    /// True while the reference is still moving towards the target.
    pub moving: bool,
}

/// Acceleration-limited point-to-point pitch reference.
#[derive(Clone, Copy, Debug)]
pub struct PitchReference {
    /// Currently applied reference.
    ref_rad: f32,
    /// Reference velocity towards the target.
    velocity_rad_s: f32,
    /// Requested target, clamped to `±TARGET_LIMIT_RAD`.
    target_rad: f32,
    /// Rate cap, radians per second. Zero freezes the reference.
    v_max_rad_s: f32,
    /// Acceleration cap, radians per second squared.
    a_max_rad_s: f32,
}

impl Default for PitchReference {
    fn default() -> Self {
        Self::new()
    }
}

impl PitchReference {
    /// A fresh reference at zero with the default rate and acceleration.
    pub const fn new() -> Self {
        Self {
            ref_rad: 0.0,
            velocity_rad_s: 0.0,
            target_rad: 0.0,
            v_max_rad_s: DEFAULT_V_MAX_RAD_S,
            a_max_rad_s: DEFAULT_A_MAX_RAD_S,
        }
    }

    /// Anchor the reference to a measured pose and drop all velocity.
    ///
    /// This is the arming soft-start: the error is zero from the first tick
    /// and grows only at `v_max`. A non-finite pose anchors at zero instead.
    pub fn reset(&mut self, at_rad: f32) {
        self.ref_rad = if at_rad.is_finite() { at_rad } else { 0.0 }
            .clamp(-TARGET_LIMIT_RAD, TARGET_LIMIT_RAD);
        self.velocity_rad_s = 0.0;
    }

    /// Override the reference rate, clamped to `[0.0, V_MAX_CAP_RAD_S]`.
    /// Non-finite values are ignored; zero freezes the reference in place.
    pub fn set_v_max(&mut self, v_max_rad_s: f32) {
        if v_max_rad_s.is_finite() {
            self.v_max_rad_s = v_max_rad_s.clamp(0.0, V_MAX_CAP_RAD_S);
        }
    }

    /// Override the reference acceleration. Non-finite or negative values
    /// are ignored; zero holds the reference stationary.
    pub fn set_a_max(&mut self, a_max_rad_s: f32) {
        if a_max_rad_s.is_finite() && a_max_rad_s >= 0.0 {
            self.a_max_rad_s = a_max_rad_s;
        }
    }

    /// Set the target, clamped to `±TARGET_LIMIT_RAD`. A non-finite target
    /// is ignored and the previous target stays in effect.
    pub fn set_target(&mut self, target_rad: f32) {
        if target_rad.is_finite() {
            self.target_rad = target_rad.clamp(-TARGET_LIMIT_RAD, TARGET_LIMIT_RAD);
        }
    }

    /// Current target, radians.
    pub fn target(&self) -> f32 {
        self.target_rad
    }

    /// Currently applied reference, radians.
    pub fn ref_rad(&self) -> f32 {
        self.ref_rad
    }

    /// Reference velocity, radians per second.
    pub fn velocity(&self) -> f32 {
        self.velocity_rad_s
    }

    /// Advance the profile by `dt_s` and return the reference to apply.
    ///
    /// A non-finite or non-positive `dt_s` leaves the state untouched and
    /// returns the current reference, so a caller bug cannot inject motion.
    pub fn update(&mut self, dt_s: f32) -> ReferenceStep {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return self.step();
        }

        let error = self.target_rad - self.ref_rad;
        // Speed that still brakes to a stop at the target. `micromath` has
        // no exact zero at zero input, so the zero-error case is special-cased.
        let desired = if error == 0.0 {
            0.0
        } else {
            micromath::F32Ext::sqrt(2.0 * self.a_max_rad_s * error.abs())
                .copysign(error)
                .clamp(-self.v_max_rad_s, self.v_max_rad_s)
        };

        let delta_v = (desired - self.velocity_rad_s)
            .clamp(-self.a_max_rad_s * dt_s, self.a_max_rad_s * dt_s);
        self.velocity_rad_s += delta_v;

        let mut next = self.ref_rad + self.velocity_rad_s * dt_s;
        // Never pass the target: clamp the final step.
        if (error >= 0.0 && next > self.target_rad) || (error < 0.0 && next < self.target_rad) {
            next = self.target_rad;
            self.velocity_rad_s = 0.0;
        }
        self.ref_rad = next;

        self.step()
    }

    fn step(&self) -> ReferenceStep {
        const MOVING_EPSILON_RAD_S: f32 = 1.0e-4;
        ReferenceStep {
            ref_rad: self.ref_rad,
            velocity_rad_s: self.velocity_rad_s,
            moving: self.velocity_rad_s.abs() > MOVING_EPSILON_RAD_S
                || (self.target_rad - self.ref_rad).abs() > 1.0e-6,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.005; // 200 Hz

    #[test]
    fn fresh_reference_is_zero_and_stationary() {
        let mut reference = PitchReference::new();
        let step = reference.update(DT);
        assert_eq!(step.ref_rad, 0.0);
        assert_eq!(step.velocity_rad_s, 0.0);
        assert!(!step.moving);
    }

    #[test]
    fn reset_anchors_the_reference_to_the_measured_pose() {
        let mut reference = PitchReference::new();
        reference.set_target(0.0);
        reference.reset(0.3);
        assert_eq!(reference.ref_rad(), 0.3);
        // Error is target - ref = -0.3; it ramps at v_max, not instantly.
        let step = reference.update(DT);
        assert!((step.ref_rad - 0.3).abs() < 0.3 * DT);
        assert!(step.moving);
    }

    #[test]
    fn reset_with_non_finite_pose_anchors_at_zero() {
        let mut reference = PitchReference::new();
        reference.reset(f32::NAN);
        assert_eq!(reference.ref_rad(), 0.0);
        reference.reset(f32::INFINITY);
        assert_eq!(reference.ref_rad(), 0.0);
    }

    #[test]
    fn target_is_clamped_to_the_tilt_limit() {
        let mut reference = PitchReference::new();
        reference.set_target(10.0);
        assert_eq!(reference.target(), TARGET_LIMIT_RAD);
        reference.set_target(-10.0);
        assert_eq!(reference.target(), -TARGET_LIMIT_RAD);
    }

    #[test]
    fn non_finite_target_is_ignored() {
        let mut reference = PitchReference::new();
        reference.set_target(0.1);
        reference.set_target(f32::NAN);
        assert_eq!(reference.target(), 0.1);
    }

    #[test]
    fn reference_reaches_the_target_without_overshoot() {
        let mut reference = PitchReference::new();
        reference.reset(0.0);
        reference.set_target(0.2);
        let mut peak = 0.0f32;
        for _ in 0..2_000 {
            let step = reference.update(DT);
            peak = peak.max(step.ref_rad);
            assert!(
                step.ref_rad <= 0.2 + 1.0e-6,
                "reference overshot: {}",
                step.ref_rad
            );
        }
        assert!((reference.ref_rad() - 0.2).abs() < 1.0e-5);
        assert!(peak > 0.19);
    }

    #[test]
    fn ramp_speed_respects_the_rate_cap() {
        let mut reference = PitchReference::new();
        reference.reset(-TARGET_LIMIT_RAD);
        reference.set_target(TARGET_LIMIT_RAD);
        for _ in 0..400 {
            let step = reference.update(DT);
            assert!(step.velocity_rad_s.abs() <= DEFAULT_V_MAX_RAD_S + 1.0e-6);
        }
    }

    #[test]
    fn approach_is_monotone_until_the_target() {
        let mut reference = PitchReference::new();
        reference.reset(0.0);
        reference.set_target(0.15);
        let mut previous = reference.ref_rad();
        for _ in 0..2_000 {
            let step = reference.update(DT);
            assert!(step.ref_rad >= previous - 1.0e-6);
            previous = step.ref_rad;
        }
    }

    #[test]
    fn mid_ramp_retarget_is_tracked() {
        let mut reference = PitchReference::new();
        reference.reset(0.0);
        reference.set_target(0.3);
        for _ in 0..200 {
            reference.update(DT);
        }
        reference.set_target(-0.1);
        for _ in 0..2_000 {
            reference.update(DT);
        }
        assert!((reference.ref_rad() - -0.1).abs() < 1.0e-5);
    }

    #[test]
    fn zero_rate_cap_freezes_the_reference() {
        let mut reference = PitchReference::new();
        reference.reset(0.0);
        reference.set_target(0.2);
        reference.set_v_max(0.0);
        for _ in 0..100 {
            let step = reference.update(DT);
            assert_eq!(step.ref_rad, 0.0);
        }
    }

    #[test]
    fn non_finite_dt_leaves_the_state_untouched() {
        let mut reference = PitchReference::new();
        reference.reset(0.05);
        reference.set_target(0.2);
        for _ in 0..50 {
            reference.update(DT);
        }
        let before = reference.ref_rad();
        assert_eq!(reference.update(f32::NAN).ref_rad, before);
        assert_eq!(reference.update(0.0).ref_rad, before);
        assert_eq!(reference.update(-1.0).ref_rad, before);
    }

    #[test]
    fn v_max_above_the_cap_is_clamped() {
        let mut reference = PitchReference::new();
        reference.set_v_max(100.0);
        reference.reset(-TARGET_LIMIT_RAD);
        reference.set_target(TARGET_LIMIT_RAD);
        for _ in 0..100 {
            let step = reference.update(DT);
            assert!(step.velocity_rad_s.abs() <= V_MAX_CAP_RAD_S + 1.0e-6);
        }
    }

    #[test]
    fn negative_target_is_reached_from_above() {
        let mut reference = PitchReference::new();
        reference.reset(0.25);
        reference.set_target(-0.2);
        for _ in 0..2_000 {
            reference.update(DT);
        }
        assert!((reference.ref_rad() - -0.2).abs() < 1.0e-5);
    }
}
