//! Guided auto-calibration for the balance reference.
//!
//! The dominant error in the manual capture flow is the operator: the
//! console reports pitch changes of several degrees between nominally
//! identical "hold vertically" attempts, and every degree of error becomes
//! a permanent lean bias for the balance law.
//!
//! The auto-calibrator removes the operator's timing: the operator places
//! the robot approximately (vertically upright for `zero`, at the balance
//! stance for `level`), and the console watches the gyro rates. Once the
//! robot has been quiet for a full window, it issues the capture command
//! itself — no button and no hand-eye coordination required.
//!
//! Stillness is judged per axis at 1.5 deg/s: below the drift of a careful
//! hand-held arm, well above sensor noise, so even gentle pose sway trips
//! the detector and delays the capture.

/// Number of consecutive quiet samples required before a capture fires.
pub const REQUIRED_STILL_TICKS: u32 = 50;
/// Maximum allowed absolute rate on every axis, in degrees per second.
pub const STILL_RATE_LIMIT_DPS: f32 = 1.5;

/// Consecutive-quiet-samples detector.
#[derive(Clone, Copy, Debug)]
pub struct StillnessTracker {
    quiet_ticks: u32,
    required_ticks: u32,
}

impl StillnessTracker {
    /// New tracker with the default window [`REQUIRED_STILL_TICKS`].
    pub const fn new() -> Self {
        Self {
            quiet_ticks: 0,
            required_ticks: REQUIRED_STILL_TICKS,
        }
    }

    /// New tracker with an explicit window (for tests and demos).
    #[cfg(test)]
    pub const fn with_window(required_ticks: u32) -> Self {
        Self {
            quiet_ticks: 0,
            required_ticks,
        }
    }

    /// Feed one sample of the three rates. Returns true once the robot has
    /// been quiet for the whole window.
    pub fn update(&mut self, pitch_rate_dps: f32, roll_rate_dps: f32, yaw_rate_dps: f32) -> bool {
        let quiet = pitch_rate_dps.abs() <= STILL_RATE_LIMIT_DPS
            && roll_rate_dps.abs() <= STILL_RATE_LIMIT_DPS
            && yaw_rate_dps.abs() <= STILL_RATE_LIMIT_DPS;
        if quiet {
            self.quiet_ticks = self.quiet_ticks.saturating_add(1);
        } else {
            self.quiet_ticks = 0;
        }
        self.quiet_ticks >= self.required_ticks
    }

    /// Progress through the window, `0.0..=1.0`, for the UI progress bar.
    pub fn progress(&self) -> f32 {
        if self.required_ticks == 0 {
            return 1.0;
        }
        (self.quiet_ticks as f32 / self.required_ticks as f32).clamp(0.0, 1.0)
    }
}

impl Default for StillnessTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stillness_requires_a_full_window() {
        let mut tracker = StillnessTracker::with_window(5);
        for _ in 0..4 {
            assert!(!tracker.update(0.5, -0.5, 0.1));
        }
        assert!(tracker.update(0.5, -0.5, 0.1));
    }

    #[test]
    fn any_axis_over_the_limit_resets_the_window() {
        let mut tracker = StillnessTracker::with_window(5);
        for _ in 0..4 {
            tracker.update(0.5, 0.0, 0.1);
        }
        // One bad sample resets everything.
        assert!(!tracker.update(2.0, 0.0, 0.1));
        assert!(!tracker.update(0.5, 0.0, 0.1));
        assert!(!tracker.update(0.5, 0.0, 0.1));
        assert!(!tracker.update(0.5, 0.0, 0.1));
        assert!(!tracker.update(0.5, 0.0, 0.1));
        assert!(tracker.update(0.5, 0.0, 0.1));
    }

    #[test]
    fn roll_and_yaw_are_checked_too() {
        let mut tracker = StillnessTracker::with_window(3);
        assert!(!tracker.update(0.0, 5.0, 0.0));
        assert!(!tracker.update(0.0, 0.0, 0.0));
        assert!(!tracker.update(0.0, 0.0, 0.0));
        assert!(tracker.update(0.0, 0.0, 0.0));
    }

    #[test]
    fn progress_tracks_the_window() {
        let mut tracker = StillnessTracker::with_window(4);
        assert_eq!(tracker.progress(), 0.0);
        tracker.update(0.0, 0.0, 0.0);
        tracker.update(0.0, 0.0, 0.0);
        assert!((tracker.progress() - 0.5).abs() < 1e-6);
    }
}
