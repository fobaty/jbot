//! Guided mount (axis) calibration.
//!
//! The BNO055 is rigidly attached to the chassis, but every re-solder or
//! re-mount changes which way the sensor axes point. The firmware compensates
//! with a mount selector (`mount`, `0..=6`), and picking the wrong selector
//! makes the console display a forward lean as a sideways lean — exactly the
//! symptom this module exists to diagnose.
//!
//! The operator performs one physical action at a time (stand upright, lean
//! forward/backward/left/right). After each step the console sweeps the seven
//! selectors and averages the reported pitch/roll for each of them. Deltas
//! from the upright reference cancel any stale zero offset, so the sweep is
//! valid even with a previously captured zero.
//!
//! Scoring is deliberately simple: for every mount candidate, each lean must
//! move the reported attitude along the expected axis and nowhere else. The
//! candidate with the lowest penalty wins; a confident verdict requires both
//! an absolute score cap and a clear margin over the runner-up.

/// Number of built-in mount selectors in the firmware.
pub const MOUNT_COUNT: usize = 7;
/// Number of guided lean actions.
pub const ACTION_COUNT: usize = 4;

/// Human-readable mount selector names, indexed by selector value.
pub const MOUNT_NAMES: [&str; MOUNT_COUNT] = [
    "normal",
    "flip X 180°",
    "flip Y 180°",
    "rot +90° X",
    "rot −90° X",
    "rot +90° Y",
    "rot −90° Y",
];

/// One physical action the operator performs in the wizard.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lean {
    /// Nose down, the robot tips forward.
    Forward,
    /// Nose up, the robot tips backward.
    Backward,
    /// Left side down, the robot tips to the left.
    Left,
    /// Right side down, the robot tips to the right.
    Right,
}

impl Lean {
    /// Every lean, in wizard order.
    pub const ALL: [Lean; ACTION_COUNT] = [Self::Forward, Self::Backward, Self::Left, Self::Right];

    /// Index into the per-action arrays.
    pub fn index(self) -> usize {
        match self {
            Self::Forward => 0,
            Self::Backward => 1,
            Self::Left => 2,
            Self::Right => 3,
        }
    }

    /// One-line operator instruction for the wizard.
    pub fn instruction(self) -> &'static str {
        match self {
            Self::Forward => "Tilt the robot FORWARD about 20–30° and hold it",
            Self::Backward => "Tilt the robot BACKWARD about 20–30° and hold it",
            Self::Left => "Tilt the robot to the LEFT about 20–30° and hold it",
            Self::Right => "Tilt the robot to the RIGHT about 20–30° and hold it",
        }
    }

    /// Short name used on the button the operator presses.
    pub fn button_label(self) -> &'static str {
        match self {
            Self::Forward => "I leaned FORWARD",
            Self::Backward => "I leaned BACKWARD",
            Self::Left => "I leaned LEFT",
            Self::Right => "I leaned RIGHT",
        }
    }

    /// Unit direction in (pitch, roll) space the response must follow.
    /// Pitch is positive leaning forward, roll is positive leaning left.
    fn expected(self) -> (f32, f32) {
        match self {
            Self::Forward => (1.0, 0.0),
            Self::Backward => (-1.0, 0.0),
            Self::Left => (0.0, 1.0),
            Self::Right => (0.0, -1.0),
        }
    }
}

/// Attitude samples collected during the guided sweep, in degrees.
#[derive(Clone, Copy, Default)]
pub struct Measurements {
    /// Mean (pitch, roll) per mount at the upright reference.
    pub upright: [Option<(f32, f32)>; MOUNT_COUNT],
    /// Mean (pitch, roll) per mount while holding each lean.
    pub leans: [[Option<(f32, f32)>; MOUNT_COUNT]; ACTION_COUNT],
}

/// Minimum motion along the expected axis (degrees) for the response to be
/// considered genuine; below that the candidate is penalised heavily.
const MIN_LEAN_DEG: f32 = 4.0;
/// Penalty per missing degree when the lean does not move the expected axis.
const MIN_LEAN_PENALTY: f32 = 30.0;

/// Absolute score a candidate must stay under to be trusted at all.
const CONFIDENT_SCORE: f32 = 90.0;
/// Margin the best candidate must hold over the runner-up.
const MIN_MARGIN: f32 = 40.0;

/// Score one mount selector. Lower is better; `f32::INFINITY` when required
/// samples are missing.
pub fn score_mount(measurements: &Measurements, mount: usize) -> f32 {
    let Some(upright) = measurements.upright[mount] else {
        return f32::INFINITY;
    };
    let mut total = 0.0_f32;
    for lean in Lean::ALL {
        let Some(measured) = measurements.leans[lean.index()][mount] else {
            return f32::INFINITY;
        };
        let dp = measured.0 - upright.0;
        let dr = measured.1 - upright.1;
        let (ex, ey) = lean.expected();
        let along = dp * ex + dr * ey;
        let perp_x = dp - ex * along;
        let perp_y = dr - ey * along;
        let perp = perp_x.hypot(perp_y);
        total += perp * perp;
        if along < MIN_LEAN_DEG {
            total += (MIN_LEAN_DEG - along) * MIN_LEAN_PENALTY;
        }
    }
    total
}

/// Score every mount selector.
pub fn scores(measurements: &Measurements) -> [f32; MOUNT_COUNT] {
    std::array::from_fn(|mount| score_mount(measurements, mount))
}

/// Outcome of the guided sweep.
#[derive(Clone, Copy, Debug)]
pub struct Detection {
    /// Selector with the lowest score.
    pub best: usize,
    /// Score of the best candidate; lower is better.
    pub best_score: f32,
    /// Runner-up selector.
    pub second: usize,
    /// Score of the runner-up.
    pub second_score: f32,
    /// True when the best candidate is below the absolute cap *and* leads the
    /// runner-up by the required margin.
    pub confident: bool,
}

/// Reduce a score table to a verdict.
pub fn detect(scores: &[f32; MOUNT_COUNT]) -> Detection {
    let mut best = 0;
    let mut best_score = f32::INFINITY;
    let mut second = 0;
    let mut second_score = f32::INFINITY;
    for (index, &score) in scores.iter().enumerate() {
        if score < best_score {
            second = best;
            second_score = best_score;
            best = index;
            best_score = score;
        } else if score < second_score {
            second = index;
            second_score = score;
        }
    }
    let confident = best_score <= CONFIDENT_SCORE && second_score - best_score >= MIN_MARGIN;
    Detection {
        best,
        best_score,
        second,
        second_score,
        confident,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mount matrices exactly as implemented by the firmware (see
    // firmware/pico_core/src/mount.rs). The gravity vector in the robot
    // frame is g_robot; the sensor reports g_sensor = Mᵀ * g_robot and the
    // candidate mount re-maps it with its own matrix.
    const MATRICES: [[[f32; 3]; 3]; MOUNT_COUNT] = [
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
        [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
        [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    ];

    fn multiply(matrix: [[f32; 3]; 3], vector: [f32; 3]) -> [f32; 3] {
        let mut result = [0.0; 3];
        for (row, out) in matrix.iter().zip(result.iter_mut()) {
            *out = row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2];
        }
        result
    }

    fn transpose(matrix: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
        let mut result = [[0.0; 3]; 3];
        for row in 0..3 {
            for col in 0..3 {
                result[row][col] = matrix[col][row];
            }
        }
        result
    }

    fn gravity_robot(lean_forward_deg: f32, lean_left_deg: f32) -> [f32; 3] {
        // Only the pure-axis leans are needed by the wizard.
        const G: f32 = 9.81;
        if lean_left_deg.abs() > 0.0 {
            let angle = lean_left_deg.to_radians();
            [0.0, G * angle.sin(), -G * angle.cos()]
        } else {
            let angle = lean_forward_deg.to_radians();
            [G * angle.sin(), 0.0, -G * angle.cos()]
        }
    }

    fn attitude_under_mount(true_mount: usize, candidate: usize, gravity: [f32; 3]) -> (f32, f32) {
        let sensor = multiply(transpose(MATRICES[true_mount]), gravity);
        let mapped = multiply(MATRICES[candidate], sensor);
        let pitch = mapped[0].atan2(-mapped[2]).to_degrees();
        let roll = mapped[1].atan2(-mapped[2]).to_degrees();
        (pitch, roll)
    }

    fn synthesize(true_mount: usize, lean_deg: f32) -> Measurements {
        let mut measurements = Measurements::default();
        for candidate in 0..MOUNT_COUNT {
            measurements.upright[candidate] = Some(attitude_under_mount(
                true_mount,
                candidate,
                gravity_robot(0.0, 0.0),
            ));
            let leans = [
                gravity_robot(lean_deg, 0.0),
                gravity_robot(-lean_deg, 0.0),
                gravity_robot(0.0, lean_deg),
                gravity_robot(0.0, -lean_deg),
            ];
            for (index, lean) in leans.iter().enumerate() {
                measurements.leans[index][candidate] =
                    Some(attitude_under_mount(true_mount, candidate, *lean));
            }
        }
        measurements
    }

    #[test]
    fn detects_every_true_mount() {
        for true_mount in 0..MOUNT_COUNT {
            let measurements = synthesize(true_mount, 25.0);
            let detection = detect(&scores(&measurements));
            assert_eq!(
                detection.best, true_mount,
                "true mount {true_mount} detected as {}",
                MOUNT_NAMES[detection.best]
            );
            assert!(
                detection.confident,
                "true mount {true_mount} not confident: {detection:?}"
            );
        }
    }

    #[test]
    fn tolerates_small_measurement_noise() {
        for true_mount in 0..MOUNT_COUNT {
            let mut measurements = synthesize(true_mount, 25.0);
            // Deterministic pseudo-noise, ~±1.5°, applied uniformly.
            let mut phase = 0.7_f32;
            let mut jitter = || {
                phase += 1.3;
                phase.sin() * 1.5
            };
            for sample in measurements.upright.iter_mut().flatten() {
                sample.0 += jitter();
                sample.1 += jitter();
            }
            for action in measurements.leans.iter_mut() {
                for sample in action.iter_mut().flatten() {
                    sample.0 += jitter();
                    sample.1 += jitter();
                }
            }
            let detection = detect(&scores(&measurements));
            assert_eq!(detection.best, true_mount);
        }
    }

    #[test]
    fn detects_shallow_leans() {
        for true_mount in 0..MOUNT_COUNT {
            let measurements = synthesize(true_mount, 8.0);
            let detection = detect(&scores(&measurements));
            assert_eq!(detection.best, true_mount, "true mount {true_mount}");
        }
    }

    #[test]
    fn missing_samples_score_infinitely() {
        let measurements = Measurements::default();
        for mount in 0..MOUNT_COUNT {
            assert_eq!(score_mount(&measurements, mount), f32::INFINITY);
        }
        assert!(detect(&scores(&measurements)).best_score.is_infinite());
    }

    #[test]
    fn scores_rank_true_mount_first() {
        // Sanity check on the ordering itself, not just the winner.
        let measurements = synthesize(5, 25.0);
        let table = scores(&measurements);
        for candidate in 0..MOUNT_COUNT {
            if candidate != 5 {
                assert!(
                    table[candidate] > table[5],
                    "candidate {candidate} beats the truth"
                );
            }
        }
    }
}
