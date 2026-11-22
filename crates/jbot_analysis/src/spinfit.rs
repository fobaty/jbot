//! Offline fit of the EFeru torque scale from spin-up telemetry.
//!
//! The bench console records `test_left_units`, `test_right_units` and both
//! hall speeds at 200 Hz. While the robot is held with its wheels free, each
//! stable torque plateau gives `M = I * alpha` per wheel; dividing the command
//! units by that torque gives units/Nm. No force gauge is needed.
//!
//! Host-only. This does not drive hardware.

use crate::session::Sample;

/// Single-wheel inertia from `config/robot.toml` [mechanical], kg m^2.
pub const WHEEL_INERTIA_KGM2: f64 = 0.0078;

const MIN_PLATEAU_SAMPLES: usize = 15;
const MIN_PLATEAU_S: f64 = 0.15;
const MOVING_ALPHA_RPS2: f64 = 0.3;
const UNIT_TOLERANCE: f64 = 2.0;

/// One stable torque command and the acceleration it produced.
#[derive(Clone, Debug, PartialEq)]
pub struct SpinLevel {
    pub units: f64,
    pub alpha_rps2: f64,
    pub torque_nm: f64,
    pub units_per_nm: f64,
    pub samples: usize,
    pub duration_s: f64,
}

/// Result of a spin-up recording.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SpinReport {
    pub levels: Vec<SpinLevel>,
    /// Largest command that did not move the wheel, EFeru units.
    pub stuck_max_units: Option<f64>,
    /// Smallest command that did move the wheel, EFeru units.
    pub moving_min_units: Option<f64>,
}

/// Contiguous runs of a nearly constant spin-test command.
pub fn plateaus(samples: &[Sample]) -> Vec<Vec<Sample>> {
    let mut result = Vec::new();
    let mut current: Vec<Sample> = Vec::new();
    let mut anchor = 0.0_f64;

    for sample in samples {
        if !sample.spin_test() {
            if !current.is_empty() {
                result.push(std::mem::take(&mut current));
            }
            continue;
        }
        let units = sample.test_left_units.abs();
        if current.is_empty() || (units - anchor).abs() <= UNIT_TOLERANCE {
            if current.is_empty() {
                anchor = units;
            }
            current.push(*sample);
        } else {
            result.push(std::mem::take(&mut current));
            anchor = units;
            current.push(*sample);
        }
    }
    if !current.is_empty() {
        result.push(current);
    }
    result
}

fn rpm_to_rps(rpm: f64) -> f64 {
    rpm * std::f64::consts::PI / 30.0
}

/// Least-squares slope of `(time_s, angular_speed_rps)`.
fn slope(series: &[(f64, f64)]) -> Option<f64> {
    if series.len() < 4 {
        return None;
    }
    let n = series.len() as f64;
    let mean_t = series.iter().map(|(t, _)| t).sum::<f64>() / n;
    let mean_v = series.iter().map(|(_, v)| v).sum::<f64>() / n;
    let numerator = series
        .iter()
        .map(|(t, v)| (t - mean_t) * (v - mean_v))
        .sum::<f64>();
    let denominator = series
        .iter()
        .map(|(t, _)| (t - mean_t).powi(2))
        .sum::<f64>();
    if denominator <= 0.0 {
        None
    } else {
        Some(numerator / denominator)
    }
}

fn acceleration(plateau: &[Sample]) -> Option<f64> {
    if plateau.len() < MIN_PLATEAU_SAMPLES {
        return None;
    }
    let start = plateau.first()?.t_ms;
    let end = plateau.last()?.t_ms;
    if end <= start || (end - start) as f64 / 1000.0 < MIN_PLATEAU_S {
        return None;
    }
    // Drop the ramp-in edges; the middle of the plateau is pure spin-up.
    let from = start + (end - start) / 5;
    let to = end - (end - start) / 5;
    let window: Vec<&Sample> = plateau
        .iter()
        .filter(|sample| sample.t_ms >= from && sample.t_ms <= to)
        .collect();
    if window.len() < MIN_PLATEAU_SAMPLES {
        return None;
    }

    let series = |pick: fn(&Sample) -> f64| -> Vec<(f64, f64)> {
        window
            .iter()
            .map(|sample| (sample.t_ms as f64 / 1000.0, rpm_to_rps(pick(sample))))
            .collect()
    };
    let left = slope(&series(|s| s.left_rpm));
    let right = slope(&series(|s| s.right_rpm));
    let values: Vec<f64> = [left, right].into_iter().flatten().map(f64::abs).collect();
    if values.is_empty() {
        None
    } else {
        Some(values.iter().sum::<f64>() / values.len() as f64)
    }
}

/// Fit units/Nm and the breakaway bracket from a recorded spin test.
pub fn fit(samples: &[Sample], wheel_inertia_kgm2: f64) -> SpinReport {
    let mut report = SpinReport::default();
    let mut stuck: Vec<f64> = Vec::new();
    let mut moving: Vec<f64> = Vec::new();

    for group in plateaus(samples) {
        let Some(alpha) = acceleration(&group) else {
            continue;
        };
        let units = group
            .first()
            .map(|s| s.test_left_units.abs())
            .unwrap_or(0.0);
        if units <= 0.0 || !alpha.is_finite() {
            continue;
        }
        if alpha >= MOVING_ALPHA_RPS2 {
            moving.push(units);
        } else {
            stuck.push(units);
        }
        let torque_nm = wheel_inertia_kgm2.max(0.0) * alpha;
        if torque_nm <= 0.0 {
            continue;
        }
        let duration_s = (group.last().unwrap().t_ms - group.first().unwrap().t_ms) as f64 / 1000.0;
        report.levels.push(SpinLevel {
            units,
            alpha_rps2: alpha,
            torque_nm,
            units_per_nm: units / torque_nm,
            samples: group.len(),
            duration_s,
        });
    }

    report.stuck_max_units = stuck.into_iter().max_by(|a, b| a.total_cmp(b));
    report.moving_min_units = moving.into_iter().min_by(|a, b| a.total_cmp(b));
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(t_ms: u64, units: f64, rpm: f64) -> Sample {
        Sample {
            t_ms,
            test_left_units: units,
            test_right_units: units,
            left_rpm: rpm,
            right_rpm: rpm,
            ..Sample::default()
        }
    }

    fn ramp(units: f64) -> Vec<Sample> {
        (0..40)
            .map(|i| sample(i * 5, units, i as f64 * 5.0))
            .collect()
    }

    #[test]
    fn detects_a_torque_plateau() {
        let groups = plateaus(&ramp(100.0));
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].len(), 40);
    }

    #[test]
    fn separates_levels_and_zero_gaps() {
        let mut samples = ramp(100.0);
        samples.extend((200..210).map(|i| sample(i * 5, 0.0, 200.0)));
        samples.extend((210..250).map(|i| sample(i * 5, 200.0, 200.0 + i as f64)));
        let groups = plateaus(&samples);
        assert_eq!(groups.len(), 2);
    }

    #[test]
    fn fits_units_per_newton_metre() {
        // 5 rpm per 5 ms is 1000 rpm/s = 104.72 rad/s^2.
        let expected_alpha = 1000.0 * std::f64::consts::PI / 30.0;
        let expected_units_per_nm = 200.0 / (WHEEL_INERTIA_KGM2 * expected_alpha);
        let report = fit(&ramp(200.0), WHEEL_INERTIA_KGM2);
        assert_eq!(report.levels.len(), 1);
        let level = &report.levels[0];
        assert!(
            (level.alpha_rps2 - expected_alpha).abs() < 1.0,
            "{}",
            level.alpha_rps2
        );
        assert!((level.units_per_nm - expected_units_per_nm).abs() < 5.0);
        assert_eq!(report.moving_min_units, Some(200.0));
    }

    #[test]
    fn stalled_level_is_recorded_in_dead_zone() {
        let stalled: Vec<Sample> = (0..40).map(|i| sample(i * 5, 40.0, 0.0)).collect();
        let report = fit(&stalled, WHEEL_INERTIA_KGM2);
        assert!(report.levels.is_empty());
        assert_eq!(report.stuck_max_units, Some(40.0));
    }
}
