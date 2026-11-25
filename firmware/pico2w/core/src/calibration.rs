//! Per-wheel breakaway calibration for the EFeru drive.
//!
//! The hoverboard hub motors are direct drive, so the small-command
//! nonlinearity is stiction / Hall-commutation ripple rather than gear backlash.
//! Bench measurements (2026-11-23) put the first wheel motion at about 30-35
//! command units on the left wheel and 40-50 on the right, while both wheels
//! reach nearly equal rpm at 80 units. The channel gains therefore match; only
//! the breakaway threshold differs, and it travels with the physical motor.
//!
//! The inverse below adds the per-wheel breakaway back to a torque request so
//! that a small command produces a proportional wheel response instead of
//! nothing until the threshold. A hard `command + breakaway * sign(command)`
//! would be a relay: at the zero crossing the output jumps by twice the
//! breakaway, which is exactly the overtorque that made an earlier symmetric
//! compensation unusable in the balance loop. The last [`BLEND_UNITS`] before
//! zero are therefore blended linearly, which keeps the inverse continuous and
//! gives it a finite slope at the origin.
//!
//! This module is pure: no I/O, no allocation, no hardware types. It is applied
//! once at the firmware's command choke point so the bench `test` path and the
//! balance law see identical calibration.

/// Half-width of the linear blend around a zero torque request, command units.
///
/// Ten units is narrow enough that the full breakaway is already present at
/// the operating point where balance normally sits, but wide enough that the
/// slope at the origin stays finite (about `1 + breakaway / BLEND_UNITS`).
pub const BLEND_UNITS: f32 = 10.0;
/// Largest per-wheel breakaway the firmware console accepts, command units.
/// Well above the measured 30-50 units, with room for a cold or loaded motor.
pub const MAX_BREAKAWAY_UNITS: f32 = 150.0;

/// Per-wheel breakaway compensation, in EFeru command units.
///
/// Zero (the default and [`Self::ZERO`]) leaves the command untouched. Values
/// are runtime-only in the firmware: a bench measurement must not leak into a
/// fresh power-up.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelCalibration {
    /// Breakaway added to left-wheel torque requests.
    pub breakaway_left_units: f32,
    /// Breakaway added to right-wheel torque requests.
    pub breakaway_right_units: f32,
}

impl WheelCalibration {
    /// No compensation.
    pub const ZERO: Self = Self {
        breakaway_left_units: 0.0,
        breakaway_right_units: 0.0,
    };

    /// Build a calibration from two per-wheel thresholds.
    pub const fn new(left: f32, right: f32) -> Self {
        Self {
            breakaway_left_units: left,
            breakaway_right_units: right,
        }
    }

    /// Apply the two thresholds to a desired wheel-torque pair.
    ///
    /// Returns `(left, right)` in the same units as the inputs. A pair of
    /// non-finite or zero requests is returned unchanged, so a failed balance
    /// step cannot gain torque from the calibration.
    pub fn apply(self, left: f32, right: f32) -> (f32, f32) {
        (
            compensate(left, self.breakaway_left_units),
            compensate(right, self.breakaway_right_units),
        )
    }
}

/// Smooth inverse of one wheel's breakaway threshold.
///
/// For requests beyond [`BLEND_UNITS`] the full threshold is added in the
/// direction of motion. Inside the blend the addition grows linearly with the
/// request, so the result is continuous at the blend boundary and passes
/// through zero. A non-finite command maps to zero; a non-positive threshold
/// is a pass-through.
/// Dither frequency, hertz. The flashed mainboard low-pass filter attenuates
/// faster ripple, so the drive has to be nudged slowly enough to follow; 12 Hz
/// is above the body's balance mode but inside the drive's usable bandwidth.
pub const DITHER_HZ: f32 = 12.0;

/// Unit-amplitude triangular dither wave, in `-1.0..=1.0`.
///
/// `phase` is in cycles and wraps at 1.0. A triangle keeps the spectrum compact
/// and needs no `sin`/`cos` in the control loop. A non-finite phase returns
/// zero so a corrupt accumulator cannot inject torque.
pub fn dither_wave(phase: f32) -> f32 {
    if !phase.is_finite() {
        return 0.0;
    }
    let mut wrapped = phase % 1.0;
    if wrapped < 0.0 {
        wrapped += 1.0;
    }
    1.0 - 4.0 * (wrapped - 0.5).abs()
}

pub fn compensate(command: f32, breakaway: f32) -> f32 {
    if !command.is_finite() {
        return 0.0;
    }
    if command == 0.0 || !breakaway.is_finite() || breakaway <= 0.0 {
        return command;
    }
    let magnitude = command.abs();
    let added = if magnitude >= BLEND_UNITS {
        breakaway
    } else {
        breakaway * magnitude / BLEND_UNITS
    };
    command.signum() * (magnitude + added)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_calibration_is_identity() {
        let cal = WheelCalibration::ZERO;
        assert_eq!(cal.apply(37.0, -91.0), (37.0, -91.0));
        assert_eq!(compensate(5.0, 0.0), 5.0);
        assert_eq!(compensate(-5.0, -10.0), -5.0);
    }

    #[test]
    fn zero_request_never_gains_torque() {
        assert_eq!(compensate(0.0, 50.0), 0.0);
        assert_eq!(
            WheelCalibration::new(35.0, 50.0).apply(0.0, 0.0),
            (0.0, 0.0)
        );
    }

    #[test]
    fn full_breakaway_is_added_beyond_the_blend() {
        // Chosen well inside the linear region of the drive.
        assert_eq!(compensate(20.0, 35.0), 55.0);
        assert_eq!(compensate(-20.0, 50.0), -70.0);
    }

    #[test]
    fn blend_is_continuous_at_the_boundary() {
        let below = compensate(BLEND_UNITS - 0.001, 50.0);
        let above = compensate(BLEND_UNITS + 0.001, 50.0);
        assert!((above - below).abs() < 0.01, "below={below} above={above}");
    }

    #[test]
    fn compensation_is_monotonic_and_odd() {
        let mut previous = 0.0;
        for magnitude in 1..=30 {
            let value = compensate(magnitude as f32, 40.0);
            assert!(value > previous, "not monotonic at {magnitude}");
            assert_eq!(compensate(-(magnitude as f32), 40.0), -value);
            previous = value;
        }
    }

    #[test]
    fn thresholds_are_independent_per_wheel() {
        let cal = WheelCalibration::new(30.0, 50.0);
        assert_eq!(cal.apply(20.0, 20.0), (50.0, 70.0));
    }

    #[test]
    fn dither_wave_is_bounded_and_wraps() {
        for step in 0..100 {
            let phase = step as f32 / 100.0;
            let value = dither_wave(phase);
            assert!((-1.0..=1.0).contains(&value), "phase={phase} value={value}");
            assert!((dither_wave(phase + 1.0) - value).abs() < 1e-6);
        }
        assert!((dither_wave(0.0) + 1.0).abs() < 1e-6);
        assert!((dither_wave(0.5) - 1.0).abs() < 1e-6);
        assert_eq!(dither_wave(f32::NAN), 0.0);
    }

    #[test]
    fn non_finite_commands_are_dropped() {
        assert_eq!(compensate(f32::NAN, 40.0), 0.0);
        assert_eq!(compensate(f32::INFINITY, 40.0), 0.0);
        // 5 units is inside the 10-unit blend: 5 + 40*5/10 = 25.
        assert_eq!(
            WheelCalibration::new(30.0, 40.0).apply(f32::NAN, 5.0),
            (0.0, 25.0)
        );
    }
}
