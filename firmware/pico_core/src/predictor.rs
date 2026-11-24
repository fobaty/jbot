//! Smith-style state predictor for the delayed hub drive.
//!
//! The EFeru FOC loop and the wheel mechanics put roughly 100 ms between a
//! torque command and the body's reaction. A PID that sees only the present
//! measurement therefore reacts to a fall that has already advanced by a
//! substantial fraction of the pendulum time constant, which turns the loop
//! into a limit cycle.
//!
//! At tick `n` the IMU still reflects commands issued up to `n - L`, where `L`
//! is that delay. The commands issued since are already on their way to the
//! wheels and will act over the next `L` ticks. Starting from the fresh
//! measurement and replaying that known command history through a linearized
//! pitch model therefore estimates the state at `n + L`, which is what the
//! controller needs when its own command finally takes effect.
//!
//! The module is pure: no I/O, no allocation, no hardware types. It is inert
//! at a zero delay, which is the firmware default.

use crate::plant::PlantParams;

/// Longest delay the ring can hold, in control ticks (200 ms at 200 Hz).
pub const MAX_DELAY_TICKS: usize = 40;

/// Linear pitch predictor with a finite command history.
#[derive(Clone, Copy, Debug)]
pub struct StatePredictor {
    params: PlantParams,
    delay_ticks: usize,
    history: [f32; MAX_DELAY_TICKS],
    index: usize,
}

impl StatePredictor {
    /// Build an inert predictor for the given plant.
    pub const fn new(params: PlantParams) -> Self {
        Self {
            params,
            delay_ticks: 0,
            history: [0.0; MAX_DELAY_TICKS],
            index: 0,
        }
    }

    /// Configure the prediction horizon in control ticks, clamped to the ring.
    pub fn set_delay_ticks(&mut self, ticks: usize) {
        self.delay_ticks = ticks.min(MAX_DELAY_TICKS - 1);
        if self.delay_ticks == 0 {
            self.reset();
        }
    }

    /// Configured horizon in control ticks.
    pub fn delay_ticks(&self) -> usize {
        self.delay_ticks
    }

    /// Drop the command history, for example after disarming.
    pub fn reset(&mut self) {
        self.history = [0.0; MAX_DELAY_TICKS];
        self.index = 0;
    }

    /// Advance by one tick and return the state projected `delay_ticks` ahead.
    ///
    /// `measured_pitch_rad` and `measured_rate_rad_s` come from the IMU;
    /// `applied_torque_nm` is the common-mode torque actually sent on this
    /// tick, after calibration and slew. A non-finite input returns the
    /// measurement unchanged.
    pub fn update(
        &mut self,
        measured_pitch_rad: f32,
        measured_rate_rad_s: f32,
        applied_torque_nm: f32,
        dt_s: f32,
    ) -> (f32, f32) {
        if self.delay_ticks == 0
            || !measured_pitch_rad.is_finite()
            || !measured_rate_rad_s.is_finite()
            || !applied_torque_nm.is_finite()
            || !dt_s.is_finite()
            || dt_s <= 0.0
        {
            if self.delay_ticks == 0 {
                self.reset();
            }
            return (measured_pitch_rad, measured_rate_rad_s);
        }

        self.history[self.index] = applied_torque_nm;
        self.index = (self.index + 1) % MAX_DELAY_TICKS;

        let a1 = self.params.a1();
        let a2 = self.params.a2();
        let mut pitch = measured_pitch_rad;
        let mut rate = measured_rate_rad_s;
        // The L most recent commands are the ones still in flight. Replaying
        // them from the fresh measurement lands at the state when this tick's
        // command actually reaches the wheels.
        for age in 1..=self.delay_ticks {
            let at = (self.index + MAX_DELAY_TICKS - age) % MAX_DELAY_TICKS;
            let torque = self.history[at];
            rate += (a1 * pitch - a2 * torque) * dt_s;
            pitch += rate * dt_s;
        }

        if pitch.is_finite() && rate.is_finite() {
            (pitch, rate)
        } else {
            (measured_pitch_rad, measured_rate_rad_s)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn predictor(delay: usize) -> StatePredictor {
        let mut p = StatePredictor::new(PlantParams::NOMINAL);
        p.set_delay_ticks(delay);
        p
    }

    #[test]
    fn zero_delay_passes_the_measurement_through() {
        let mut p = predictor(0);
        assert_eq!(p.update(0.1, 0.2, 1.0, 0.005), (0.1, 0.2));
        assert_eq!(p.delay_ticks(), 0);
    }

    #[test]
    fn delay_is_clamped_to_the_ring() {
        let p = predictor(MAX_DELAY_TICKS + 10);
        assert_eq!(p.delay_ticks(), MAX_DELAY_TICKS - 1);
    }

    #[test]
    fn a_falling_body_is_predicted_ahead() {
        let mut p = predictor(20);
        for _ in 0..20 {
            let _ = p.update(0.0, 0.0, 0.0, 0.005);
        }
        let measured_rate = 10.0_f32.to_radians();
        let (pitch, rate) = p.update(0.0, measured_rate, 0.0, 0.005);
        assert!(pitch > 0.0, "pitch {pitch}");
        assert!(rate > measured_rate, "rate {rate}");
    }

    #[test]
    fn a_non_finite_sample_is_passed_through() {
        let mut p = predictor(20);
        for _ in 0..20 {
            let _ = p.update(0.0, 0.0, 0.0, 0.005);
        }
        let (pitch, rate) = p.update(f32::NAN, 1.0, 0.0, 0.005);
        assert!(pitch.is_nan());
        assert_eq!(rate, 1.0);
        let (pitch, rate) = p.update(0.0, 0.0, 0.0, 0.005);
        assert!(pitch.is_finite() && rate.is_finite());
    }

    #[test]
    fn opposing_torque_reduces_the_projected_pitch() {
        let mut p = predictor(20);
        for _ in 0..20 {
            let _ = p.update(0.0, 0.0, 0.0, 0.005);
        }
        let (pitch_free, _) = p.update(0.0, 0.0, 0.0, 0.005);
        let mut q = predictor(20);
        for _ in 0..20 {
            let _ = q.update(0.0, 0.0, 1.0, 0.005);
        }
        let (pitch_braked, _) = q.update(0.0, 0.0, 1.0, 0.005);
        assert!(pitch_braked < pitch_free, "{pitch_braked} vs {pitch_free}");
    }

    #[test]
    fn reset_clears_the_history() {
        let mut p = predictor(20);
        for _ in 0..20 {
            let _ = p.update(0.0, 0.0, 1.0, 0.005);
        }
        p.reset();
        let (pitch, rate) = p.update(0.05, 0.0, 0.0, 0.005);
        assert!(pitch > 0.05, "pitch {pitch}");
        assert!(rate >= 0.0, "rate {rate}");
    }
}
