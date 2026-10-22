//! Planar 1-DOF four-bar leg kinematics.
//!
//! The leg follows the mechanical concept in `docs/architecture.md`:
//!
//! ```text
//! A (hip, driven) -------- B (rocker pivot, passive)
//!        \                     /
//!   crank \                   / rocker
//!          C ------ D
//!           \        \
//!            \        E (wheel axle)
//!             \_______/
//! ```
//!
//! Frame convention: the leg plane uses +x forward and +y up. A is the origin
//! and B lies on the positive x axis. The hip angle is measured from the A->B
//! direction, positive counter-clockwise. The wheel axle lies at E.
//!
//! The fork C-D-E is one rigid body, so E is found by intersecting the circles
//! of the two fork links around C and D. Branch selection is part of the
//! geometry because both a circle-circle intersection and the four-bar closure
//! have two mirror solutions.

use serde::{Deserialize, Serialize};

use super::{require_finite, require_positive, vector_norm, KinematicsError};

/// Which of the two circle-circle intersection solutions forms the assembly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssemblyBranch {
    /// Intersection on the positive normal side of the centre line.
    Positive,
    /// Intersection on the negative normal side of the centre line.
    Negative,
}

impl AssemblyBranch {
    fn sign(self) -> f64 {
        match self {
            Self::Positive => 1.0,
            Self::Negative => -1.0,
        }
    }
}

/// Link lengths and assembly branches of one planar four-bar leg.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LegGeometry {
    /// Ground link A-B (chassis).
    pub base_m: f64,
    /// Upper crank A-C driven by the hip actuator.
    pub crank_m: f64,
    /// Lower rocker B-D.
    pub rocker_m: f64,
    /// Coupler segment C-D of the rigid wheel fork.
    pub coupler_m: f64,
    /// Fork link D-E.
    pub fork_de_m: f64,
    /// Fork link C-E.
    pub fork_ce_m: f64,
    pub rocker_branch: AssemblyBranch,
    pub fork_branch: AssemblyBranch,
}

/// Joint coordinates of one leg at a given hip angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegPose {
    pub hip_angle_rad: f64,
    pub crank_joint_m: [f64; 2],
    pub rocker_joint_m: [f64; 2],
    pub wheel_axle_m: [f64; 2],
}

impl LegGeometry {
    /// Verify that every link length is finite and strictly positive.
    pub fn validate(&self) -> Result<(), KinematicsError> {
        require_positive("base_m", self.base_m)?;
        require_positive("crank_m", self.crank_m)?;
        require_positive("rocker_m", self.rocker_m)?;
        require_positive("coupler_m", self.coupler_m)?;
        require_positive("fork_de_m", self.fork_de_m)?;
        require_positive("fork_ce_m", self.fork_ce_m)
    }

    /// Solve the closed loop for a hip angle without applying joint limits.
    pub fn forward(&self, hip_angle_rad: f64) -> Result<LegPose, KinematicsError> {
        require_finite("hip_angle_rad", hip_angle_rad)?;
        self.validate()?;

        let a = [0.0, 0.0];
        let b = [self.base_m, 0.0];
        let c = [
            a[0] + self.crank_m * hip_angle_rad.cos(),
            a[1] + self.crank_m * hip_angle_rad.sin(),
        ];

        let d = assembly_point(c, self.coupler_m, b, self.rocker_m, self.rocker_branch).ok_or(
            KinematicsError::NoSolution {
                angle_rad: hip_angle_rad,
            },
        )?;

        let e = assembly_point(c, self.fork_ce_m, d, self.fork_de_m, self.fork_branch).ok_or(
            KinematicsError::NoSolution {
                angle_rad: hip_angle_rad,
            },
        )?;

        Ok(LegPose {
            hip_angle_rad,
            crank_joint_m: c,
            rocker_joint_m: d,
            wheel_axle_m: e,
        })
    }
}

/// Four-bar leg with a finite hip travel window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegKinematics {
    geometry: LegGeometry,
    hip_min_rad: f64,
    hip_max_rad: f64,
}

impl LegKinematics {
    /// Number of samples used by workspace queries and inverse kinematics.
    const SAMPLES: usize = 1024;
    /// Accepted distance from the coupler curve for a valid inverse solution.
    const IK_TOLERANCE_M: f64 = 1e-3;

    /// Build a leg model and verify that the linkage assembles across the
    /// configured hip window.
    pub fn new(
        geometry: LegGeometry,
        hip_min_rad: f64,
        hip_max_rad: f64,
    ) -> Result<Self, KinematicsError> {
        geometry.validate()?;
        require_finite("hip_min_rad", hip_min_rad)?;
        require_finite("hip_max_rad", hip_max_rad)?;
        if hip_min_rad >= hip_max_rad {
            return Err(KinematicsError::InvalidJointRange {
                min_rad: hip_min_rad,
                max_rad: hip_max_rad,
            });
        }

        let model = Self {
            geometry,
            hip_min_rad,
            hip_max_rad,
        };

        for angle in [hip_min_rad, 0.5 * (hip_min_rad + hip_max_rad), hip_max_rad] {
            geometry.forward(angle)?;
        }

        Ok(model)
    }

    pub fn geometry(&self) -> &LegGeometry {
        &self.geometry
    }

    pub fn hip_limits_rad(&self) -> (f64, f64) {
        (self.hip_min_rad, self.hip_max_rad)
    }

    /// Solve the leg for a hip angle inside the configured travel window.
    pub fn forward(&self, hip_angle_rad: f64) -> Result<LegPose, KinematicsError> {
        require_finite("hip_angle_rad", hip_angle_rad)?;
        if hip_angle_rad < self.hip_min_rad || hip_angle_rad > self.hip_max_rad {
            return Err(KinematicsError::JointLimit {
                angle_rad: hip_angle_rad,
                min_rad: self.hip_min_rad,
                max_rad: self.hip_max_rad,
            });
        }
        self.geometry.forward(hip_angle_rad)
    }

    /// Find the hip angle whose wheel axle is closest to `target_m`.
    ///
    /// The mechanism has one DOF, so an arbitrary target is usually not
    /// reachable. A target farther than [`Self::IK_TOLERANCE_M`] from the
    /// coupler curve returns [`KinematicsError::OutsideWorkspace`].
    pub fn inverse(&self, target_m: [f64; 2]) -> Result<LegPose, KinematicsError> {
        require_finite("target_x_m", target_m[0])?;
        require_finite("target_y_m", target_m[1])?;

        let mut best_index = 0;
        let mut best_distance = f64::INFINITY;
        let step = (self.hip_max_rad - self.hip_min_rad) / Self::SAMPLES as f64;

        for index in 0..=Self::SAMPLES {
            let angle = self.hip_min_rad + step * index as f64;
            let distance = self.squared_distance(angle, target_m);
            if distance < best_distance {
                best_distance = distance;
                best_index = index;
            }
        }

        if !best_distance.is_finite() {
            return Err(KinematicsError::NoSolution {
                angle_rad: 0.5 * (self.hip_min_rad + self.hip_max_rad),
            });
        }

        let lower_index = best_index.saturating_sub(1);
        let upper_index = (best_index + 1).min(Self::SAMPLES);
        let mut lower = self.hip_min_rad + step * lower_index as f64;
        let mut upper = self.hip_min_rad + step * upper_index as f64;

        // Golden-section search on the bracket around the best coarse sample.
        let inv_phi = 0.5 * (5.0_f64.sqrt() - 1.0);
        let mut left = upper - inv_phi * (upper - lower);
        let mut right = lower + inv_phi * (upper - lower);
        for _ in 0..80 {
            let left_distance = self.squared_distance(left, target_m);
            let right_distance = self.squared_distance(right, target_m);
            if left_distance <= right_distance {
                upper = right;
                right = left;
                left = upper - inv_phi * (upper - lower);
            } else {
                lower = left;
                left = right;
                right = lower + inv_phi * (upper - lower);
            }
        }

        let angle = 0.5 * (lower + upper);
        let pose = self.forward(angle)?;
        let distance = ((pose.wheel_axle_m[0] - target_m[0]).powi(2)
            + (pose.wheel_axle_m[1] - target_m[1]).powi(2))
        .sqrt();

        if distance > Self::IK_TOLERANCE_M {
            return Err(KinematicsError::OutsideWorkspace {
                distance_m: distance,
            });
        }

        Ok(pose)
    }

    /// Derivative of the wheel-axle position with respect to the hip angle.
    pub fn jacobian(&self, hip_angle_rad: f64) -> Result<[f64; 2], KinematicsError> {
        require_finite("hip_angle_rad", hip_angle_rad)?;
        let delta = 1e-6;
        let lower = (hip_angle_rad - delta).max(self.hip_min_rad);
        let upper = (hip_angle_rad + delta).min(self.hip_max_rad);
        let span = upper - lower;

        if span < 1e-12 {
            return Err(KinematicsError::Degenerate {
                angle_rad: hip_angle_rad,
            });
        }

        let lower_pose = self.forward(lower)?;
        let upper_pose = self.forward(upper)?;

        Ok([
            (upper_pose.wheel_axle_m[0] - lower_pose.wheel_axle_m[0]) / span,
            (upper_pose.wheel_axle_m[1] - lower_pose.wheel_axle_m[1]) / span,
        ])
    }

    /// True when the wheel-axle Jacobian is smaller than `tolerance_m_per_rad`.
    pub fn is_singular(
        &self,
        hip_angle_rad: f64,
        tolerance_m_per_rad: f64,
    ) -> Result<bool, KinematicsError> {
        Ok(vector_norm(self.jacobian(hip_angle_rad)?) < tolerance_m_per_rad)
    }

    /// Sample the wheel-axle coupler curve over the configured hip window.
    pub fn coupler_curve(&self, samples: usize) -> Result<Vec<[f64; 2]>, KinematicsError> {
        if samples < 2 {
            return Err(KinematicsError::InvalidSampleCount { samples });
        }

        let step = (self.hip_max_rad - self.hip_min_rad) / (samples - 1) as f64;
        let mut points = Vec::with_capacity(samples);
        for index in 0..samples {
            let angle = self.hip_min_rad + step * index as f64;
            points.push(self.forward(angle)?.wheel_axle_m);
        }
        Ok(points)
    }

    /// Lowest and highest wheel-axle height reached inside the hip window.
    pub fn vertical_range(&self) -> Result<(f64, f64), KinematicsError> {
        let mut minimum = f64::INFINITY;
        let mut maximum = f64::NEG_INFINITY;
        let step = (self.hip_max_rad - self.hip_min_rad) / Self::SAMPLES as f64;

        for index in 0..=Self::SAMPLES {
            let angle = self.hip_min_rad + step * index as f64;
            let height = self.forward(angle)?.wheel_axle_m[1];
            minimum = minimum.min(height);
            maximum = maximum.max(height);
        }

        if minimum.is_finite() && maximum.is_finite() {
            Ok((minimum, maximum))
        } else {
            Err(KinematicsError::NoSolution {
                angle_rad: 0.5 * (self.hip_min_rad + self.hip_max_rad),
            })
        }
    }

    fn squared_distance(&self, hip_angle_rad: f64, target_m: [f64; 2]) -> f64 {
        match self.forward(hip_angle_rad) {
            Ok(pose) => {
                (pose.wheel_axle_m[0] - target_m[0]).powi(2)
                    + (pose.wheel_axle_m[1] - target_m[1]).powi(2)
            }
            Err(_) => f64::INFINITY,
        }
    }
}

/// Pick one of the two intersections of two circles in the leg plane.
fn assembly_point(
    center_a: [f64; 2],
    radius_a: f64,
    center_b: [f64; 2],
    radius_b: f64,
    branch: AssemblyBranch,
) -> Option<[f64; 2]> {
    let dx = center_b[0] - center_a[0];
    let dy = center_b[1] - center_a[1];
    let distance = dx.hypot(dy);

    if !distance.is_finite() || distance <= f64::EPSILON {
        return None;
    }

    let tolerance = 1e-12 * (radius_a + radius_b + distance).max(1.0);
    if distance > radius_a + radius_b + tolerance
        || distance < (radius_a - radius_b).abs() - tolerance
    {
        return None;
    }

    let along =
        (radius_a * radius_a - radius_b * radius_b + distance * distance) / (2.0 * distance);
    let height_squared = radius_a * radius_a - along * along;
    if height_squared < -1e-9 {
        return None;
    }

    let height = height_squared.max(0.0).sqrt();
    let mid_x = center_a[0] + along * dx / distance;
    let mid_y = center_a[1] + along * dy / distance;
    let normal_x = -dy / distance;
    let normal_y = dx / distance;
    let sign = branch.sign();

    Some([
        mid_x + sign * height * normal_x,
        mid_y + sign * height * normal_y,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baseline_leg() -> LegKinematics {
        LegKinematics::new(
            LegGeometry {
                base_m: 0.14,
                crank_m: 0.11,
                rocker_m: 0.13,
                coupler_m: 0.06,
                fork_de_m: 0.18,
                fork_ce_m: 0.17,
                rocker_branch: AssemblyBranch::Negative,
                fork_branch: AssemblyBranch::Negative,
            },
            -1.55,
            -0.70,
        )
        .expect("baseline geometry must assemble")
    }

    #[test]
    fn forward_solves_both_joints_and_the_axle() {
        let leg = baseline_leg();
        let pose = leg.forward(-1.1).expect("angle is inside the joint window");

        assert!(pose.crank_joint_m[1] < 0.0);
        assert!(pose.wheel_axle_m[1] < 0.0);
        assert!(pose.hip_angle_rad == -1.1);
    }

    #[test]
    fn forward_rejects_angles_outside_the_joint_window() {
        let leg = baseline_leg();
        assert!(matches!(
            leg.forward(-2.0),
            Err(KinematicsError::JointLimit { .. })
        ));
        assert!(matches!(
            leg.forward(0.0),
            Err(KinematicsError::JointLimit { .. })
        ));
    }

    #[test]
    fn raw_geometry_reports_unassemblable_angles() {
        let geometry = baseline_leg().geometry().to_owned();
        assert!(matches!(
            geometry.forward(0.0),
            Err(KinematicsError::NoSolution { .. })
        ));
    }

    #[test]
    fn inverse_recovers_the_hip_angle_from_the_axle_position() {
        let leg = baseline_leg();
        for angle in [-1.50, -1.20, -0.90, -0.75] {
            let axle = leg.forward(angle).expect("valid angle").wheel_axle_m;
            let recovered = leg.inverse(axle).expect("axle is on the curve");
            assert!(
                (recovered.hip_angle_rad - angle).abs() <= 1e-4,
                "expected {angle}, got {}",
                recovered.hip_angle_rad
            );
        }
    }

    #[test]
    fn inverse_rejects_points_off_the_coupler_curve() {
        let leg = baseline_leg();
        assert!(matches!(
            leg.inverse([0.0, 0.0]),
            Err(KinematicsError::OutsideWorkspace { .. })
        ));
    }

    #[test]
    fn jacobian_matches_a_finite_difference() {
        let leg = baseline_leg();
        let angle = -1.1;
        let delta = 1e-6;
        let lower = leg.forward(angle - delta).expect("valid angle");
        let upper = leg.forward(angle + delta).expect("valid angle");
        let expected = [
            (upper.wheel_axle_m[0] - lower.wheel_axle_m[0]) / (2.0 * delta),
            (upper.wheel_axle_m[1] - lower.wheel_axle_m[1]) / (2.0 * delta),
        ];
        let actual = leg.jacobian(angle).expect("valid angle");

        assert!((actual[0] - expected[0]).abs() <= 1e-5);
        assert!((actual[1] - expected[1]).abs() <= 1e-5);
        assert!(vector_norm(actual) > 1e-3);
        assert!(!leg.is_singular(angle, 1e-3).expect("valid angle"));
    }

    #[test]
    fn coupler_curve_covers_the_configured_height_range() {
        let leg = baseline_leg();
        let (minimum, maximum) = leg.vertical_range().expect("geometry assembles");
        assert!(maximum - minimum >= 0.1);

        let curve = leg.coupler_curve(64).expect("valid sample count");
        assert_eq!(curve.len(), 64);
        assert_eq!(curve[0], leg.forward(-1.55).expect("valid").wheel_axle_m);
        assert_eq!(curve[63], leg.forward(-0.70).expect("valid").wheel_axle_m);
    }

    #[test]
    fn rejects_degenerate_geometry() {
        let result = LegKinematics::new(
            LegGeometry {
                base_m: 0.14,
                crank_m: 0.11,
                rocker_m: 0.13,
                coupler_m: 10.0,
                fork_de_m: 0.18,
                fork_ce_m: 0.17,
                rocker_branch: AssemblyBranch::Negative,
                fork_branch: AssemblyBranch::Negative,
            },
            -1.55,
            -0.70,
        );

        assert!(matches!(result, Err(KinematicsError::NoSolution { .. })));
    }

    #[test]
    fn rejects_invalid_joint_window_and_sample_count() {
        let geometry = *baseline_leg().geometry();
        assert!(matches!(
            LegKinematics::new(geometry, -0.7, -1.55),
            Err(KinematicsError::InvalidJointRange { .. })
        ));

        let leg = baseline_leg();
        assert!(matches!(
            leg.coupler_curve(1),
            Err(KinematicsError::InvalidSampleCount { samples: 1 })
        ));
    }
}
