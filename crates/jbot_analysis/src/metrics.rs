//! The measurements themselves: axis consistency, pitch-law fit, drive
//! breakaway, oscillation, and session summary.
//!
//! Every function is pure and takes samples in file order. Nothing here needs
//! the robot: these are the numbers the operator should have read *before*
//! arming, and the numbers that make a bench report reproducible.

use crate::session::{Sample, Session, MODE_FALL};

/// Shortest gap between samples used for a rate/angle comparison, seconds.
/// Shorter gaps amplify quantisation noise in the angle.
const MIN_DT_S: f64 = 0.001;
/// Longest gap between samples used for a rate/angle comparison, seconds.
/// Longer gaps mean a dropped sample, and the finite difference would be an
/// average over the gap.
const MAX_DT_S: f64 = 0.05;
/// Rule used to call a drive "at rest", rpm.
const REST_RPM: f64 = 5.0;
/// Rule used to call a drive "turning", rpm.
const TURNING_RPM: f64 = 25.0;
/// Fraction of the torque full scale that counts as saturated.
const SATURATION_FRACTION: f64 = 0.995;
/// EFeru command full scale, units. The torque request in the 8-byte command
/// frame is normalised so that ±1000 is ±`I_MOT_MAX`, whatever the configured
/// current limit is; the law clamps to this value.
const EFERU_FULL_SCALE_UNITS: f64 = 1000.0;
/// Pitch beyond which leaving the armed state counts as a latch rather than a
/// deliberate disarm, degrees. Matches the firmware recover threshold.
pub const LATCH_PITCH_DEG: f64 = 15.0;
/// Angle spread below which a session is called "too still to judge" rather
/// than "unpaired", degrees.
const MOTION_THRESHOLD_DEG: f64 = 1.0;
/// Accepted slope band for "the angle integrates its own rate".
///
/// The BNO055 fusion lags the true angle, so a healthy pair reads 0.6..0.9
/// rather than 1.0 and a tight band would reject it. A slope of 0.03 means the
/// rate runs 30 times faster than the angle derivative, which is not a pair at
/// all however well the two correlate.
const SLOPE_BAND: (f64, f64) = (0.3, 3.0);

/// Pearson correlation and least-squares slope of `y` against `x`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fit {
    /// Correlation coefficient, `-1..=1`.
    pub corr: f64,
    /// Slope of the best-fit line through the origin, `y / x`.
    pub slope: f64,
    /// Number of pairs used.
    pub samples: usize,
}

/// What the rate/angle comparison says about one pair of channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisVerdict {
    /// The angle integrates its own rate channel with the right sign.
    Consistent,
    /// The angle follows the *other* axis' rate: the channels are swapped.
    Swapped,
    /// The angle and its own rate have opposite signs.
    Inverted,
    /// The channels moved, but the angle does not follow either rate: the
    /// mapping pairs two unrelated signals.
    Unpaired,
    /// Too few usable pairs, or the motion was too small to judge.
    Unknown,
}

impl AxisVerdict {
    /// Stable one-word label for reports.
    pub fn label(self) -> &'static str {
        match self {
            Self::Consistent => "consistent",
            Self::Swapped => "SWAPPED",
            Self::Inverted => "INVERTED",
            Self::Unpaired => "UNPAIRED",
            Self::Unknown => "unknown",
        }
    }

    /// Whether the axis can be trusted for the balance law.
    pub fn is_trustworthy(self) -> bool {
        matches!(self, Self::Consistent)
    }

    /// Whether the pair is definitely wrong, as opposed to unjudged.
    pub fn is_fault(self) -> bool {
        matches!(self, Self::Swapped | Self::Inverted | Self::Unpaired)
    }
}

/// One axis: its own rate channel, and the other axis' rate channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLine {
    /// `d(angle)/dt` against the matching rate channel.
    pub direct: Option<Fit>,
    /// `d(angle)/dt` against the other rate channel.
    pub cross: Option<Fit>,
    /// Standard deviation of the angle itself, degrees. Below one degree the
    /// session simply did not move enough to judge, and a noisy derivative
    /// must not be mistaken for motion.
    pub angle_std_deg: f64,
}

impl AxisLine {
    /// Judge the pair.
    pub fn verdict(&self) -> AxisVerdict {
        let direct_abs = self.direct.map_or(0.0, |fit| fit.corr.abs());
        let cross = self.cross.map_or(0.0, |fit| fit.corr.abs());
        let cross_is_strong = self
            .cross
            .is_some_and(|fit| fit.samples >= 8 && cross >= 0.7 && cross > direct_abs + 0.15);
        if let Some(direct) = self.direct {
            if direct.samples >= 8 {
                if direct.corr <= -0.7 {
                    return AxisVerdict::Inverted;
                }
                if direct.corr >= 0.7 && (SLOPE_BAND.0..=SLOPE_BAND.1).contains(&direct.slope) {
                    return AxisVerdict::Consistent;
                }
            }
        }
        // A swapped axis can have a near-zero direct correlation, so the
        // cross channel is checked even when the direct one is unusable.
        if cross_is_strong {
            return AxisVerdict::Swapped;
        }
        // Channels that moved but do not pair with each other: a mount whose
        // angle source and rate source disagree, which the balance law cannot
        // survive.
        if self.angle_std_deg >= MOTION_THRESHOLD_DEG {
            AxisVerdict::Unpaired
        } else {
            AxisVerdict::Unknown
        }
    }
}

/// Result of the attitude self-consistency check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisCheck {
    /// Pitch against the pitch rate.
    pub pitch: AxisLine,
    /// Roll against the roll rate.
    pub roll: AxisLine,
}

impl AxisCheck {
    /// Overall verdict.
    ///
    /// A definite fault on either axis wins. Otherwise one clean axis is
    /// enough: a robot that only leans forward during the test proves the
    /// pitch pair, and a still roll axis must not hide that. The per-axis
    /// verdicts remain the authority on which axis was actually judged.
    pub fn verdict(&self) -> AxisVerdict {
        let pitch = self.pitch.verdict();
        let roll = self.roll.verdict();
        for verdict in [pitch, roll] {
            if verdict.is_fault() {
                return verdict;
            }
        }
        if pitch.is_trustworthy() || roll.is_trustworthy() {
            AxisVerdict::Consistent
        } else {
            AxisVerdict::Unknown
        }
    }
}

/// Compare each angle with the derivative of that angle and with the rate
/// channel of the other axis.
///
/// This is the offline equivalent of the physical check "lean forward, watch
/// `pitch`": if `pitch` moves but `gy` stays quiet while `gx` follows it, the
/// attitude mapping is wrong, and the balance law is driving the wrong axis.
pub fn axis_check<'a>(samples: impl Iterator<Item = &'a Sample>) -> AxisCheck {
    let mut pitch: Vec<(f64, f64, f64)> = Vec::new();
    let mut roll: Vec<(f64, f64, f64)> = Vec::new();
    let mut pitch_angles: Vec<f64> = Vec::new();
    let mut roll_angles: Vec<f64> = Vec::new();
    let mut previous: Option<&Sample> = None;

    for sample in samples {
        if sample.imu_fresh {
            pitch_angles.push(sample.pitch_deg);
            roll_angles.push(sample.roll_deg);
        }
        if let Some(last) = previous {
            let dt = (sample.t_ms.saturating_sub(last.t_ms)) as f64 / 1000.0;
            if (MIN_DT_S..=MAX_DT_S).contains(&dt) && last.imu_fresh && sample.imu_fresh {
                let pitch_rate = (sample.pitch_deg - last.pitch_deg) / dt;
                let roll_rate = (sample.roll_deg - last.roll_deg) / dt;
                pitch.push((pitch_rate, sample.gy_dps, sample.gx_dps));
                roll.push((roll_rate, sample.gx_dps, sample.gy_dps));
            }
        }
        previous = Some(sample);
    }

    AxisCheck {
        pitch: line(&pitch, deviation(&pitch_angles)),
        roll: line(&roll, deviation(&roll_angles)),
    }
}

/// Build an [`AxisLine`] from `(derivative, own rate, other rate)` triples.
fn line(pairs: &[(f64, f64, f64)], angle_std_deg: f64) -> AxisLine {
    if pairs.len() < 8 {
        return AxisLine {
            direct: None,
            cross: None,
            angle_std_deg,
        };
    }
    let direct_x: Vec<f64> = pairs.iter().map(|pair| pair.0).collect();
    let direct_y: Vec<f64> = pairs.iter().map(|pair| pair.1).collect();
    let cross_y: Vec<f64> = pairs.iter().map(|pair| pair.2).collect();
    AxisLine {
        direct: fit_through_origin(&direct_x, &direct_y),
        cross: fit_through_origin(&direct_x, &cross_y),
        angle_std_deg,
    }
}

/// Correlation and slope of `y = slope * x`, without an intercept.
///
/// The control law has no bias term, and an angle derivative of zero must
/// correspond to a rate of zero, so the origin is the right anchor.
pub fn fit_through_origin(x: &[f64], y: &[f64]) -> Option<Fit> {
    let count = x.len().min(y.len());
    if count < 8 {
        return None;
    }
    let mut sxx = 0.0;
    let mut syy = 0.0;
    let mut sxy = 0.0;
    for (x, y) in x.iter().zip(y.iter()).take(count) {
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
    }
    if sxx <= f64::EPSILON || syy <= f64::EPSILON {
        return None;
    }
    let slope = sxy / sxx;
    let corr = sxy / (sxx.sqrt() * syy.sqrt());
    corr.is_finite().then_some(Fit {
        corr: corr.clamp(-1.0, 1.0),
        slope,
        samples: count,
    })
}

/// The pitch law recovered from the applied torque.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LawFit {
    /// Common-mode torque per radian of pitch error, EFeru units.
    pub kp_units_per_rad: f64,
    /// Common-mode torque per radian per second of pitch rate, EFeru units.
    pub kd_units_per_rad_s: f64,
    /// Coefficient of determination of the two-predictor fit.
    pub r2: f64,
    /// Residual RMS, EFeru units.
    pub rms_units: f64,
    /// Samples used after dropping saturated ones.
    pub samples: usize,
    /// Fraction of armed samples that sat on the torque limit.
    pub saturated_fraction: f64,
    /// Torque full scale of the EFeru command frame the fit treats as the
    /// clamp, EFeru units (±1000).
    pub torque_limit_units: f64,
}

/// Recover `torque = kp * (pitch - ref - demand) + kd * pitch_rate` from the
/// telemetry of an armed session.
///
/// The fit is only meaningful while the law is actually running: armed,
/// non-fallen, not a spin test, and not sitting on the torque limit (the clamp
/// would hide the true gains).
pub fn law_fit<'a>(samples: impl Iterator<Item = &'a Sample>) -> Option<LawFit> {
    let armed: Vec<&Sample> = samples
        .filter(|sample| sample.armed() && sample.imu_fresh && !sample.spin_test())
        .collect();
    if armed.len() < 16 {
        return None;
    }
    // Saturation is judged against the EFeru full scale, not against the
    // largest command seen in the session: the clamp is a property of the
    // wire format, and an observed maximum confounds "nothing was clamped"
    // with "the session stayed far from the limit".
    let saturation = EFERU_FULL_SCALE_UNITS * SATURATION_FRACTION;

    let usable: Vec<&&Sample> = armed
        .iter()
        .filter(|sample| sample.torque_units.abs() < saturation)
        .collect();
    if usable.len() < 16 {
        return None;
    }

    let mut s11 = 0.0;
    let mut s22 = 0.0;
    let mut s12 = 0.0;
    let mut t1 = 0.0;
    let mut t2 = 0.0;
    let mut sum_y = 0.0;
    for sample in &usable {
        let error = sample.error_deg().to_radians();
        let rate = sample.gy_dps.to_radians();
        let torque = sample.torque_units;
        s11 += error * error;
        s22 += rate * rate;
        s12 += error * rate;
        t1 += error * torque;
        t2 += rate * torque;
        sum_y += torque;
    }
    let determinant = s11 * s22 - s12 * s12;
    if determinant.abs() <= f64::EPSILON * (s11 * s22).max(1.0) {
        // Pitch error and pitch rate are collinear in this window: the gains
        // cannot be separated.
        return None;
    }
    let kp = (t1 * s22 - t2 * s12) / determinant;
    let kd = (t2 * s11 - t1 * s12) / determinant;

    let mean_y = sum_y / usable.len() as f64;
    let mut ss_res = 0.0;
    let mut ss_tot = 0.0;
    for sample in &usable {
        let predicted = kp * sample.error_deg().to_radians() + kd * sample.gy_dps.to_radians();
        let residual = sample.torque_units - predicted;
        ss_res += residual * residual;
        ss_tot += (sample.torque_units - mean_y).powi(2);
    }
    let samples = usable.len();
    let saturated = armed.len() - samples;
    Some(LawFit {
        kp_units_per_rad: kp,
        kd_units_per_rad_s: kd,
        r2: if ss_tot > f64::EPSILON {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        },
        rms_units: (ss_res / samples as f64).sqrt(),
        samples,
        saturated_fraction: saturated as f64 / armed.len() as f64,
        torque_limit_units: EFERU_FULL_SCALE_UNITS,
    })
}

/// Brackets around the drive breakaway torque.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Breakaway {
    /// Largest |torque| observed while both wheels were at rest, units.
    pub stuck_max_units: f64,
    /// Smallest |torque| observed while both wheels were turning, units.
    pub moving_min_units: f64,
    /// Samples the two numbers are based on.
    pub samples: usize,
}

impl Breakaway {
    /// Whether the brackets are tight enough to be useful.
    pub fn is_bracketed(&self) -> bool {
        self.stuck_max_units < self.moving_min_units
    }
}

/// Find the torque at which the drive starts moving.
///
/// The balance law spends most of its time commanding torque below the dead
/// zone while the wheels stand still, and above it while they roll. The
/// largest "still" command and the smallest "rolling" command bracket the
/// breakaway point; with a good session the gap is small.
pub fn breakaway<'a>(samples: impl Iterator<Item = &'a Sample>) -> Option<Breakaway> {
    let mut stuck_max = 0.0_f64;
    let mut moving_min = f64::INFINITY;
    let mut stuck = 0;
    let mut moving = 0;

    for sample in samples {
        if !sample.armed() || sample.spin_test() {
            continue;
        }
        let torque = sample.torque_units.abs();
        let left = sample.left_rpm.abs();
        let right = sample.right_rpm.abs();
        if left < REST_RPM && right < REST_RPM {
            stuck += 1;
            stuck_max = stuck_max.max(torque);
        } else if left > TURNING_RPM && right > TURNING_RPM {
            moving += 1;
            moving_min = moving_min.min(torque);
        }
    }

    if stuck == 0 || moving == 0 || !moving_min.is_finite() {
        return None;
    }
    Some(Breakaway {
        stuck_max_units: stuck_max,
        moving_min_units: moving_min,
        samples: stuck + moving,
    })
}

/// Sign-change statistics of the pitch error while armed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Oscillation {
    /// Length of the armed segments, seconds.
    pub duration_s: f64,
    /// Sign changes of the pitch error per second.
    pub crossings_per_s: f64,
    /// Period implied by two sign changes, seconds.
    pub period_s: Option<f64>,
    /// Peak-to-peak pitch error, degrees.
    pub error_p2p_deg: f64,
}

/// Measure how much the pitch error rings.
///
/// A machine fighting itself shows up as a regular train of sign changes with
/// a stable implied period; a machine that is merely being pushed shows
/// irregular crossings with no period.
pub fn oscillation<'a>(samples: impl Iterator<Item = &'a Sample>) -> Option<Oscillation> {
    let armed: Vec<&Sample> = samples.filter(|sample| sample.armed()).collect();
    if armed.len() < 16 {
        return None;
    }
    let duration_s = (armed[armed.len() - 1].t_ms - armed[0].t_ms) as f64 / 1000.0;
    if duration_s <= 0.0 {
        return None;
    }

    let mut crossings = 0usize;
    let mut previous_sign = 0i8;
    let mut min_error = f64::INFINITY;
    let mut max_error = f64::NEG_INFINITY;
    for sample in &armed {
        let error = sample.error_deg();
        min_error = min_error.min(error);
        max_error = max_error.max(error);
        let sign = if error > 0.0 {
            1
        } else if error < 0.0 {
            -1
        } else {
            0
        };
        if sign != 0 {
            if previous_sign != 0 && sign != previous_sign {
                crossings += 1;
            }
            previous_sign = sign;
        }
    }

    let crossings_per_s = crossings as f64 / duration_s;
    Some(Oscillation {
        duration_s,
        crossings_per_s,
        period_s: (crossings >= 2).then(|| 2.0 * duration_s / crossings as f64),
        error_p2p_deg: max_error - min_error,
    })
}

/// Spread of one attitude channel over a session.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelMotion {
    /// Standard deviation of the angle, degrees.
    pub std_deg: f64,
    /// Standard deviation of the rate channel, degrees per second.
    pub rate_std_dps: f64,
}

/// How the tilt is distributed between the two attitude channels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Motion {
    /// Pitch channel.
    pub pitch: ChannelMotion,
    /// Roll channel.
    pub roll: ChannelMotion,
}

impl Motion {
    /// Fraction of the tilt variance that lives in `pitch`, `0..=1`.
    ///
    /// A forward-lean test on a correctly mapped machine puts most of the
    /// motion in `pitch`. A wrong mount moves `roll` instead, which is the
    /// signature the rate/angle check also looks for.
    pub fn pitch_share(&self) -> f64 {
        let pitch = self.pitch.std_deg * self.pitch.std_deg;
        let roll = self.roll.std_deg * self.roll.std_deg;
        let total = pitch + roll;
        if total <= f64::EPSILON {
            0.0
        } else {
            pitch / total
        }
    }

    /// Whether either channel carried enough motion to be worth judging.
    pub fn has_motion(&self) -> bool {
        self.pitch.rate_std_dps + self.roll.rate_std_dps > 1.0
    }
}

/// Standard deviation of each attitude channel and its rate over the session.
pub fn motion<'a>(samples: impl Iterator<Item = &'a Sample>) -> Motion {
    let mut pitch = Vec::new();
    let mut roll = Vec::new();
    let mut gy = Vec::new();
    let mut gx = Vec::new();
    for sample in samples {
        if !sample.imu_fresh {
            continue;
        }
        pitch.push(sample.pitch_deg);
        roll.push(sample.roll_deg);
        gy.push(sample.gy_dps);
        gx.push(sample.gx_dps);
    }
    Motion {
        pitch: ChannelMotion {
            std_deg: deviation(&pitch),
            rate_std_dps: deviation(&gy),
        },
        roll: ChannelMotion {
            std_deg: deviation(&roll),
            rate_std_dps: deviation(&gx),
        },
    }
}

/// Population standard deviation of a series.
fn deviation(values: &[f64]) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / values.len() as f64;
    variance.sqrt()
}

/// Headline numbers for a session.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Summary {
    /// Wall-clock length of the recording, seconds.
    pub duration_s: f64,
    /// Samples in the file.
    pub rows: usize,
    /// Time the balance law was armed, seconds.
    pub armed_s: f64,
    /// Time the emergency stop was latched, seconds.
    pub estop_s: f64,
    /// Transitions into `mode 4`.
    pub falls: usize,
    /// Transitions out of an armed mode while the body was past the recover
    /// threshold.
    ///
    /// A mid-run latch is a single control tick, and the console samples the
    /// telemetry at 50 Hz from a 200 Hz loop, so the `mode 4` row is usually
    /// overwritten before it is read. The transition is the reliable witness.
    pub suspected_latches: usize,
    /// Samples per balance mode, index `0..=4`.
    pub mode_counts: [usize; 5],
    /// Pitch range, degrees.
    pub pitch_range: (f64, f64),
    /// Roll range, degrees.
    pub roll_range: (f64, f64),
    /// Wheel-speed range, rpm.
    pub rpm_range: (f64, f64),
    /// Common-mode torque range, EFeru units.
    pub torque_range: (f64, f64),
}

/// Summarise a session.
pub fn summary(session: &Session) -> Summary {
    let mut armed_s = 0.0;
    let mut estop_s = 0.0;
    let mut falls = 0;
    let mut suspected_latches = 0;
    let mut mode_counts = [0usize; 5];
    let mut pitch = (f64::INFINITY, f64::NEG_INFINITY);
    let mut roll = (f64::INFINITY, f64::NEG_INFINITY);
    let mut rpm = (f64::INFINITY, f64::NEG_INFINITY);
    let mut torque = (f64::INFINITY, f64::NEG_INFINITY);
    let mut previous_mode = 0u8;

    for (index, sample) in session.samples.iter().enumerate() {
        let dt = if index == 0 {
            0.0
        } else {
            (sample.t_ms.saturating_sub(session.samples[index - 1].t_ms)) as f64 / 1000.0
        };
        if sample.armed() {
            armed_s += dt;
        }
        if sample.estop {
            estop_s += dt;
        }
        if sample.mode == MODE_FALL && previous_mode != MODE_FALL {
            falls += 1;
        }
        if index > 0 {
            let last = &session.samples[index - 1];
            if last.armed() && !sample.armed() && last.pitch_deg.abs() >= LATCH_PITCH_DEG {
                suspected_latches += 1;
            }
        }
        mode_counts[usize::from(sample.mode).min(4)] += 1;
        pitch.0 = pitch.0.min(sample.pitch_deg);
        pitch.1 = pitch.1.max(sample.pitch_deg);
        roll.0 = roll.0.min(sample.roll_deg);
        roll.1 = roll.1.max(sample.roll_deg);
        let mean = sample.forward_rpm();
        rpm.0 = rpm.0.min(mean);
        rpm.1 = rpm.1.max(mean);
        torque.0 = torque.0.min(sample.torque_units);
        torque.1 = torque.1.max(sample.torque_units);
        previous_mode = sample.mode;
    }

    if session.samples.is_empty() {
        pitch = (0.0, 0.0);
        roll = (0.0, 0.0);
        rpm = (0.0, 0.0);
        torque = (0.0, 0.0);
    }

    Summary {
        duration_s: session.end_ms() as f64 / 1000.0,
        rows: session.samples.len(),
        armed_s,
        estop_s,
        falls,
        suspected_latches,
        mode_counts,
        pitch_range: pitch,
        roll_range: roll,
        rpm_range: rpm,
        torque_range: torque,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(t_ms: u64, pitch: f64, roll: f64, gy: f64, gx: f64, mode: u8) -> Sample {
        Sample {
            t_ms,
            imu_fresh: true,
            pitch_deg: pitch,
            roll_deg: roll,
            gy_dps: gy,
            gx_dps: gx,
            mode,
            ..Sample::default()
        }
    }

    /// Build a run where `pitch` integrates `pitch_rate` and `roll`
    /// integrates `roll_rate`, at 200 Hz for `seconds`.
    fn consistent_run(
        seconds: f64,
        pitch_rate: fn(f64) -> f64,
        roll_rate: fn(f64) -> f64,
    ) -> Vec<Sample> {
        let dt = 0.005;
        let steps = (seconds / dt) as usize;
        let mut pitch = 0.0;
        let mut roll = 0.0;
        let mut samples = Vec::with_capacity(steps);
        for step in 0..steps {
            let t = step as f64 * dt;
            let gy = pitch_rate(t);
            let gx = roll_rate(t);
            pitch += gy * dt;
            roll += gx * dt;
            samples.push(sample((t * 1000.0) as u64, pitch, roll, gy, gx, 2));
        }
        samples
    }

    #[test]
    fn consistent_mapping_is_detected() {
        let samples = consistent_run(4.0, |t| 20.0 * (2.0 * t).sin(), |t| 10.0 * (3.0 * t).cos());
        let check = axis_check(samples.iter());
        assert_eq!(check.pitch.verdict(), AxisVerdict::Consistent);
        assert_eq!(check.roll.verdict(), AxisVerdict::Consistent);
        assert_eq!(check.verdict(), AxisVerdict::Consistent);
        let pitch = check.pitch.direct.unwrap();
        assert!((pitch.slope - 1.0).abs() < 0.05, "slope {}", pitch.slope);
    }

    #[test]
    fn a_rate_far_out_of_scale_is_not_consistent() {
        // The mount-sweep signature: the channels correlate, but the reported
        // rate is 30x the angle derivative, so the pair is unusable.
        let samples: Vec<Sample> = consistent_run(4.0, |t| 20.0 * (2.0 * t).sin(), |_| 0.0)
            .into_iter()
            .map(|sample| Sample {
                gy_dps: sample.gy_dps * 30.0,
                ..sample
            })
            .collect();
        assert_eq!(
            axis_check(samples.iter()).pitch.verdict(),
            AxisVerdict::Unpaired
        );
    }

    #[test]
    fn one_clean_axis_is_enough_for_the_overall_verdict() {
        // A forward-lean check moves pitch only; the still roll axis must not
        // downgrade the verdict for the axis that was actually exercised.
        let samples = consistent_run(4.0, |t| 20.0 * (2.0 * t).sin(), |_| 0.0);
        let check = axis_check(samples.iter());
        assert_eq!(check.pitch.verdict(), AxisVerdict::Consistent);
        assert_eq!(check.roll.verdict(), AxisVerdict::Unknown);
        assert_eq!(check.verdict(), AxisVerdict::Consistent);
    }

    #[test]
    fn swapped_mapping_is_detected() {
        // The "pitch" channel integrates the roll rate and vice versa: what a
        // wrong mount does to the control frame.
        let samples = consistent_run(4.0, |t| 10.0 * (3.0 * t).cos(), |t| 20.0 * (2.0 * t).sin());
        let swapped: Vec<Sample> = samples
            .iter()
            .map(|sample| Sample {
                gy_dps: sample.gx_dps,
                gx_dps: sample.gy_dps,
                ..*sample
            })
            .collect();
        let check = axis_check(swapped.iter());
        assert_eq!(check.pitch.verdict(), AxisVerdict::Swapped);
    }

    #[test]
    fn inverted_rate_is_detected() {
        let samples: Vec<Sample> = consistent_run(4.0, |t| 20.0 * (2.0 * t).sin(), |_| 0.0)
            .into_iter()
            .map(|sample| Sample {
                gy_dps: -sample.gy_dps,
                ..sample
            })
            .collect();
        assert_eq!(
            axis_check(samples.iter()).pitch.verdict(),
            AxisVerdict::Inverted
        );
    }

    #[test]
    fn a_still_session_is_not_judged() {
        let samples = consistent_run(2.0, |_| 0.0, |_| 0.0);
        assert_eq!(axis_check(samples.iter()).verdict(), AxisVerdict::Unknown);
    }

    #[test]
    fn law_fit_recovers_the_gains() {
        let mut samples = Vec::new();
        for step in 0..400 {
            let t = step as f64 * 0.005;
            let error = 3.0 * (2.0 * t).sin();
            let rate = 15.0 * (5.0 * t).cos();
            let torque = 5700.0 * error.to_radians() + 280.0 * rate.to_radians();
            samples.push(Sample {
                t_ms: (t * 1000.0) as u64,
                imu_fresh: true,
                pitch_deg: error,
                gy_dps: rate,
                torque_units: torque,
                mode: 2,
                ..Sample::default()
            });
        }
        let fit = law_fit(samples.iter()).expect("fit");
        assert!((fit.kp_units_per_rad - 5700.0).abs() < 1.0, "{fit:?}");
        assert!((fit.kd_units_per_rad_s - 280.0).abs() < 1.0, "{fit:?}");
        assert!(fit.r2 > 0.999);
        assert!(fit.rms_units < 1.0);
        assert_eq!(fit.torque_limit_units, 1000.0);
        assert!(fit.saturated_fraction < 1e-9);
    }

    #[test]
    fn saturated_samples_do_not_poison_the_fit() {
        let mut samples = Vec::new();
        for step in 0..400 {
            let t = step as f64 * 0.005;
            // Large enough that the raw law output exceeds the ±1000 clamp,
            // so the clamp actually engages on part of the window.
            let error = 30.0 * (2.0 * t).sin();
            let rate = 15.0 * (5.0 * t).cos();
            let raw = 5700.0 * error.to_radians() + 280.0 * rate.to_radians();
            samples.push(Sample {
                t_ms: (t * 1000.0) as u64,
                imu_fresh: true,
                pitch_deg: error,
                gy_dps: rate,
                torque_units: raw.clamp(-1000.0, 1000.0),
                mode: 2,
                ..Sample::default()
            });
        }
        let fit = law_fit(samples.iter()).expect("fit");
        assert!(fit.saturated_fraction > 0.0);
        assert!((fit.kp_units_per_rad - 5700.0).abs() < 60.0, "{fit:?}");
    }

    #[test]
    fn disarmed_samples_are_ignored_by_the_fit() {
        let samples: Vec<Sample> = (0..100)
            .map(|step| Sample {
                t_ms: step * 5,
                imu_fresh: true,
                pitch_deg: step as f64,
                torque_units: 0.0,
                mode: 0,
                ..Sample::default()
            })
            .collect();
        assert!(law_fit(samples.iter()).is_none());
    }

    #[test]
    fn breakaway_brackets_the_dead_zone() {
        let mut samples = Vec::new();
        for step in 0..200 {
            let torque = 40.0 + step as f64 * 2.0;
            let moving = torque > 150.0;
            samples.push(Sample {
                t_ms: step * 5,
                imu_fresh: true,
                torque_units: torque,
                left_rpm: if moving { 120.0 } else { 0.0 },
                right_rpm: if moving { 120.0 } else { 0.0 },
                mode: 2,
                ..Sample::default()
            });
        }
        let bracket = breakaway(samples.iter()).expect("breakaway");
        assert!(bracket.stuck_max_units <= 150.0);
        assert!(bracket.moving_min_units > 150.0);
        assert!(bracket.is_bracketed());
    }

    #[test]
    fn oscillation_measures_the_period() {
        let mut samples = Vec::new();
        for step in 0..1000 {
            let t = step as f64 * 0.005;
            samples.push(Sample {
                t_ms: (t * 1000.0) as u64,
                imu_fresh: true,
                pitch_deg: 2.0 * (2.0 * std::f64::consts::PI * 1.5 * t).sin(),
                mode: 2,
                ..Sample::default()
            });
        }
        let ring = oscillation(samples.iter()).expect("oscillation");
        assert!((ring.crossings_per_s - 3.0).abs() < 0.2, "{ring:?}");
        let period = ring.period_s.expect("period");
        assert!((period - 1.0 / 1.5).abs() < 0.05, "{period}");
        assert!((ring.error_p2p_deg - 4.0).abs() < 0.2);
    }

    #[test]
    fn motion_exposes_a_wrong_axis_session() {
        // What a forward-lean test looks like with `mount 0`: the tilt lands
        // in roll, pitch barely moves.
        let samples: Vec<Sample> = (0..200)
            .map(|step| {
                let t = step as f64 * 0.005;
                Sample {
                    t_ms: step * 5,
                    imu_fresh: true,
                    pitch_deg: 0.2 * t.sin(),
                    roll_deg: 25.0 * t.sin(),
                    gy_dps: 1.0 * t.cos(),
                    gx_dps: 80.0 * t.cos(),
                    mode: 0,
                    ..Sample::default()
                }
            })
            .collect();
        let motion = motion(samples.iter());
        assert!(motion.has_motion());
        assert!(motion.pitch_share() < 0.01, "{motion:?}");
    }

    #[test]
    fn motion_is_empty_for_a_still_session() {
        let samples: Vec<Sample> = (0..50)
            .map(|step| Sample {
                t_ms: step * 5,
                imu_fresh: true,
                ..Sample::default()
            })
            .collect();
        let motion = motion(samples.iter());
        assert!(!motion.has_motion());
        assert_eq!(motion.pitch_share(), 0.0);
    }

    #[test]
    fn a_latch_is_inferred_from_the_armed_transition() {
        let samples = vec![
            sample(0, 0.0, 0.0, 0.0, 0.0, 0),
            sample(10, 2.0, 0.0, 0.0, 0.0, 2),
            sample(20, 20.0, 0.0, 0.0, 0.0, 2),
            sample(30, 24.0, 0.0, 0.0, 0.0, 0),
            sample(40, 40.0, 0.0, 0.0, 0.0, 0),
        ];
        let session = Session {
            samples,
            ..Session::default()
        };
        let summary = summary(&session);
        assert_eq!(summary.suspected_latches, 1);
        assert_eq!(summary.falls, 0);
    }

    #[test]
    fn summary_counts_modes_and_falls() {
        let samples = vec![
            sample(0, 0.0, 0.0, 0.0, 0.0, 0),
            sample(10, 1.0, 0.0, 0.0, 0.0, 2),
            sample(20, 2.0, 0.0, 0.0, 0.0, 4),
            sample(30, 3.0, 0.0, 0.0, 0.0, 4),
            sample(40, 0.0, 0.0, 0.0, 0.0, 0),
        ];
        let session = Session {
            samples,
            ..Session::default()
        };
        let summary = summary(&session);
        assert_eq!(summary.falls, 1);
        assert_eq!(summary.mode_counts[4], 2);
        assert_eq!(summary.mode_counts[0], 2);
        assert!((summary.armed_s - 0.01).abs() < 1e-9);
        assert!((summary.pitch_range.1 - 3.0).abs() < 1e-9);
    }
}
