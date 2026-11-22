//! Small, allocation-free signal conditioners for the control loop.
//!
//! Two pieces only:
//!
//! * [`LowPass`] — a first-order low-pass filter, used on the pitch rate
//!   before the derivative gain sees it, so gyro noise does not turn into
//!   a choppy torque command;
//! * [`SlewLimiter`] — a per-wheel rate limit on the torque command, so a
//!   discontinuous balance correction ramps over a few ticks instead of
//!   slamming the wheels from full forward to full reverse in one step.
//!
//! Neither piece applies on the e-stop path: the emergency stop zeroes the
//! command immediately and resets both conditioners.

use core::f32::consts::TAU;

/// First-order low-pass filter with a fixed cutoff frequency.
#[derive(Clone, Copy, Debug)]
pub struct LowPass {
    /// Fraction of the residual applied per tick, `dt / (RC + dt)`.
    alpha: f32,
    /// Latest filtered value.
    value: f32,
    /// False until the first sample arrives; the first sample passes through.
    initialized: bool,
}

impl LowPass {
    /// Build a filter with the given cutoff in hertz and tick period in seconds.
    /// Non-finite or non-positive arguments fall back to a pass-through filter.
    pub fn new(cutoff_hz: f32, period_s: f32) -> Self {
        let valid =
            cutoff_hz.is_finite() && cutoff_hz > 0.0 && period_s.is_finite() && period_s > 0.0;
        let alpha = if valid {
            let rc = 1.0 / (TAU * cutoff_hz);
            period_s / (rc + period_s)
        } else {
            1.0
        };
        Self {
            alpha,
            value: 0.0,
            initialized: false,
        }
    }

    /// Filter one sample. A non-finite input resets the filter to zero.
    pub fn update(&mut self, input: f32) -> f32 {
        if !input.is_finite() {
            self.reset();
            return 0.0;
        }
        if !self.initialized {
            self.value = input;
            self.initialized = true;
            return input;
        }
        self.value += self.alpha * (input - self.value);
        self.value
    }

    /// Drop all state; the next sample passes through unchanged.
    pub fn reset(&mut self) {
        self.value = 0.0;
        self.initialized = false;
    }
}

/// Per-channel slew-rate limiter for a torque command.
///
/// Starts from zero: after an arm or a reset the torque ramps up instead of
/// jumping to the first computed value. The limit is shared against a target
/// that may change every tick.
#[derive(Clone, Copy, Debug)]
pub struct SlewLimiter {
    /// Maximum change per second.
    max_rate_per_s: f32,
    /// Latest limited value.
    value: f32,
}

impl SlewLimiter {
    /// Build a limiter with the given rate in units per second.
    /// A non-finite or non-positive rate degenerates to "no movement".
    pub fn new(max_rate_per_s: f32) -> Self {
        let max_rate_per_s = if max_rate_per_s.is_finite() && max_rate_per_s > 0.0 {
            max_rate_per_s
        } else {
            0.0
        };
        Self {
            max_rate_per_s,
            value: 0.0,
        }
    }

    /// Advance one tick towards `target` by at most `rate * period_s`.
    /// A non-finite target zeroes the output and the internal value.
    pub fn update(&mut self, target: f32, period_s: f32) -> f32 {
        if !target.is_finite() {
            self.value = 0.0;
            return 0.0;
        }
        let max_delta = if period_s.is_finite() && period_s > 0.0 {
            self.max_rate_per_s * period_s
        } else {
            0.0
        };
        let delta = target - self.value;
        self.value += delta.clamp(-max_delta, max_delta);
        self.value
    }

    /// Replace the maximum change per second without dropping the state, so a
    /// bench session can soften the ramp on the fly. A non-finite or
    /// non-positive rate degenerates to "no movement".
    pub fn set_rate(&mut self, max_rate_per_s: f32) {
        self.max_rate_per_s = if max_rate_per_s.is_finite() && max_rate_per_s > 0.0 {
            max_rate_per_s
        } else {
            0.0
        };
    }

    /// The currently applied value.
    pub fn value(&self) -> f32 {
        self.value
    }

    /// Drop all state; the next ramp starts from zero.
    pub fn reset(&mut self) {
        self.value = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.005;

    #[test]
    fn low_pass_passes_the_first_sample_through() {
        let mut filter = LowPass::new(12.0, DT);
        assert_eq!(filter.update(5.0), 5.0);
    }

    #[test]
    fn low_pass_attenuates_a_step() {
        let mut filter = LowPass::new(12.0, DT);
        filter.update(0.0);
        // A step input is tracked gradually, not instantly.
        let first = filter.update(10.0);
        assert!(first > 0.0 && first < 10.0);
        let mut last = first;
        for _ in 0..200 {
            last = filter.update(10.0);
        }
        assert!((last - 10.0).abs() < 0.01);
    }

    #[test]
    fn low_pass_smooths_high_frequency_jitter() {
        let mut filter = LowPass::new(12.0, DT);
        // A square wave at 100 Hz (half the loop rate) must be crushed to a
        // small residual; the very first sample passes through by design, so
        // measure the steady-state swing only.
        let mut peak = 0.0_f32;
        for tick in 0..400 {
            let square = if tick % 2 == 0 { 1.0 } else { -1.0 };
            let out = filter.update(square).abs();
            if tick >= 2 {
                peak = peak.max(out);
            }
        }
        // The unfiltered square wave is ±1; the filter crushes it below 0.65.
        assert!(peak < 0.65, "residual {peak}");
    }

    #[test]
    fn low_pass_non_finite_input_resets_to_zero() {
        let mut filter = LowPass::new(12.0, DT);
        filter.update(3.0);
        assert_eq!(filter.update(f32::NAN), 0.0);
        assert_eq!(filter.update(f32::INFINITY), 0.0);
        // After the reset the next sample passes through.
        assert_eq!(filter.update(2.0), 2.0);
    }

    #[test]
    fn low_pass_reset_restores_pass_through() {
        let mut filter = LowPass::new(12.0, DT);
        filter.update(1.0);
        filter.update(9.0);
        filter.reset();
        assert_eq!(filter.update(7.0), 7.0);
    }

    #[test]
    fn slew_limiter_ramps_from_zero() {
        let mut limiter = SlewLimiter::new(3_000.0);
        let first = limiter.update(1_000.0, DT);
        // One tick advances by at most rate * period (15 units).
        assert!((first - 15.0).abs() < 1e-4);
        for _ in 0..300 {
            limiter.update(1_000.0, DT);
        }
        assert!((limiter.value() - 1_000.0).abs() < 1e-3);
    }

    #[test]
    fn slew_limiter_clamps_a_reversal_over_ticks() {
        let mut limiter = SlewLimiter::new(3_000.0);
        for _ in 0..200 {
            limiter.update(1_000.0, DT);
        }
        // A full sign reversal takes at least 1000/15 ≈ 67 ticks.
        let first = limiter.update(-1_000.0, DT);
        assert!((first - 985.0).abs() < 1e-4);
        let mut last = first;
        for _ in 0..200 {
            last = limiter.update(-1_000.0, DT);
        }
        assert!((last - (-1_000.0)).abs() < 1e-3);
    }

    #[test]
    fn slew_limiter_non_finite_target_zeroes_output() {
        let mut limiter = SlewLimiter::new(3_000.0);
        limiter.update(500.0, DT);
        assert_eq!(limiter.update(f32::NAN, DT), 0.0);
        assert_eq!(limiter.value(), 0.0);
    }

    #[test]
    fn slew_limiter_zero_rate_never_moves() {
        let mut limiter = SlewLimiter::new(0.0);
        assert_eq!(limiter.update(1_000.0, DT), 0.0);
        limiter.reset();
        assert_eq!(limiter.value(), 0.0);
    }

    #[test]
    fn slew_limiter_set_rate_keeps_state_and_softens_ramp() {
        let mut limiter = SlewLimiter::new(3_000.0);
        for _ in 0..20 {
            limiter.update(1_000.0, DT);
        }
        let before = limiter.value();
        // A slower rate must not drop the already applied value.
        limiter.set_rate(400.0);
        assert!((limiter.value() - before).abs() < 1e-4);
        // Two units per tick at 400 units/s and a 5 ms period.
        let next = limiter.update(1_000.0, DT);
        assert!((next - before - 2.0).abs() < 1e-4);
    }

    #[test]
    fn slew_limiter_set_rate_rejects_non_positive() {
        let mut limiter = SlewLimiter::new(3_000.0);
        limiter.update(500.0, DT);
        let held = limiter.value();
        limiter.set_rate(0.0);
        assert_eq!(limiter.update(1_000.0, DT), held);
        limiter.set_rate(f32::NAN);
        assert_eq!(limiter.update(1_000.0, DT), held);
    }
}
