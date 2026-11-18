//! Mount-orientation compensation for the BNO055.
//!
//! The sensor is rigidly attached to the chassis with its board plane
//! vertical (mounted on its side): the sensor Z axis is horizontal. Euler
//! conventions are device-specific, so the mount compensation is done on
//! vectors, where it is exact: the gravity vector and the angular rates are
//! re-expressed in the robot frame (X forward, Y left, Z up) with sign
//! flips, and pitch and roll are re-derived from the re-mapped gravity
//! vector with `atan2`.
//!
//! Definitions used throughout:
//!
//! * pitch is positive when the body leans forward (nose down), and is the
//!   only angle the balance law consumes;
//! * roll is positive when the body leans left;
//! * both are exactly zero in the captured zero pose once the zero offsets
//!   are applied.
//!
//! The remap is its own inverse (all flips are sign changes), so the same
//! function maps sensor to robot and back.

use crate::bno055::{AngularRate, Euler};

/// Degrees per radian.
pub const RAD_TO_DEG: f32 = 57.295_78;
/// Radians per degree.
pub const DEG_TO_RAD: f32 = 1.0 / RAD_TO_DEG;

/// Mount orientation of the module on the chassis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mount {
    /// X forward, Y left, Z up, aligned with the robot frame.
    #[default]
    Normal,
    /// Rotated 180 degrees about the robot X (forward) axis: the sensor
    /// Y and Z axes are inverted.
    FlipX,
    /// Rotated 180 degrees about the robot Y (left) axis: the sensor
    /// X and Z axes are inverted.
    FlipY,
    /// Rotated +90 degrees about the robot X axis (right-hand rule); the
    /// sensor Y axis points along robot +Z.
    RotXp,
    /// Rotated -90 degrees about the robot X axis; the sensor Y axis points
    /// along robot -Z.
    RotXm,
    /// Rotated +90 degrees about the robot Y axis (right-hand rule); the
    /// sensor Z axis points along robot +X.
    RotYp,
    /// Rotated -90 degrees about the robot Y axis; the sensor Z axis points
    /// along robot -X.
    RotYm,
}

impl Mount {
    /// Number of selectable mounts.
    pub const COUNT: u32 = 7;

    /// Decode a stored mount selector. Unknown values fall back to
    /// [`Mount::Normal`] so a corrupted setting cannot enable a wrong flip.
    pub fn from_u32(value: u32) -> Self {
        match value {
            1 => Self::FlipX,
            2 => Self::FlipY,
            3 => Self::RotXp,
            4 => Self::RotXm,
            5 => Self::RotYp,
            6 => Self::RotYm,
            _ => Self::Normal,
        }
    }

    /// Stable selector stored in firmware state.
    pub const fn as_u32(self) -> u32 {
        match self {
            Self::Normal => 0,
            Self::FlipX => 1,
            Self::FlipY => 2,
            Self::RotXp => 3,
            Self::RotXm => 4,
            Self::RotYp => 5,
            Self::RotYm => 6,
        }
    }

    /// Stable lower-case name used on the console.
    pub fn name(self) -> &'static str {
        match self {
            Self::Normal => "normal",
            Self::FlipX => "flipx",
            Self::FlipY => "flipy",
            Self::RotXp => "rotxp",
            Self::RotXm => "rotxm",
            Self::RotYp => "rotyp",
            Self::RotYm => "rotym",
        }
    }

    /// Parse a console argument into a mount. Accepts the numeric selector
    /// as well as the stable names, case-insensitively.
    pub fn from_name(text: &str) -> Option<Self> {
        let text = text.trim();
        const NAMES: [(&str, Mount); 7] = [
            ("normal", Mount::Normal),
            ("flipx", Mount::FlipX),
            ("flipy", Mount::FlipY),
            ("rotxp", Mount::RotXp),
            ("rotxm", Mount::RotXm),
            ("rotyp", Mount::RotYp),
            ("rotym", Mount::RotYm),
        ];
        for (name, mount) in NAMES {
            if text.eq_ignore_ascii_case(name) {
                return Some(mount);
            }
        }
        // Also accept short forms like "90x" and "-90y".
        if text == "3" || text.eq_ignore_ascii_case("+90x") || text.eq_ignore_ascii_case("90x") {
            Some(Self::RotXp)
        } else if text == "4" || text.eq_ignore_ascii_case("-90x") {
            Some(Self::RotXm)
        } else if text == "5"
            || text.eq_ignore_ascii_case("+90y")
            || text.eq_ignore_ascii_case("90y")
        {
            Some(Self::RotYp)
        } else if text == "6" || text.eq_ignore_ascii_case("-90y") {
            Some(Self::RotYm)
        } else if text == "0" {
            Some(Self::Normal)
        } else if text == "1" {
            Some(Self::FlipX)
        } else if text == "2" {
            Some(Self::FlipY)
        } else {
            None
        }
    }

    /// True when the attitude has to be derived from the gravity vector
    /// instead of the fused Euler output.
    pub fn remaps_axes(self) -> bool {
        !matches!(self, Self::Normal)
    }

    /// Rotation matrix that re-expresses a sensor-frame vector in the
    /// robot frame. All entries are exact (0, ±1), so the remap is lossless.
    fn matrix(self) -> [[f32; 3]; 3] {
        match self {
            Self::Normal => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            Self::FlipX => [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            Self::FlipY => [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
            Self::RotXp => [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
            Self::RotXm => [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
            Self::RotYp => [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
            Self::RotYm => [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
        }
    }

    /// Transposed rotation matrix; the inverse of [`Self::matrix`].
    #[cfg(test)]
    fn matrix_transpose(self) -> [[f32; 3]; 3] {
        let m = self.matrix();
        [
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ]
    }

    /// Re-express a sensor-frame vector in the robot frame.
    pub fn map_vector(self, vector: [f32; 3]) -> [f32; 3] {
        multiply(self.matrix(), vector)
    }

    /// Re-express a robot-frame vector in the sensor frame. Exposed for the
    /// test-suite and for diagnostics.
    #[cfg(test)]
    fn map_vector_inverse(self, vector: [f32; 3]) -> [f32; 3] {
        multiply(self.matrix_transpose(), vector)
    }

    /// Re-map an angular-rate sample into the robot frame.
    pub fn map_rate(self, rate: AngularRate) -> AngularRate {
        let [x, y, z] = self.map_vector([rate.x_dps, rate.y_dps, rate.z_dps]);
        AngularRate {
            x_dps: x,
            y_dps: y,
            z_dps: z,
        }
    }

    /// Pitch and roll in degrees, derived from the gravity vector reported
    /// by the sensor (direction of gravitational acceleration, m/s^2).
    ///
    /// Pitch is positive when leaning forward, roll positive when leaning
    /// left. Upright yields `(0, 0)` for every mount after the remap.
    pub fn pitch_roll_deg(self, gravity_sensor: [f32; 3]) -> (f32, f32) {
        let [x, y, z] = self.map_vector(gravity_sensor);
        // Fully qualified so the same code path (micromath, ~0.002 rad
        // worst-case error) is exercised in tests and on target.
        let pitch = micromath::F32Ext::atan2(x, -z) * RAD_TO_DEG;
        let roll = micromath::F32Ext::atan2(y, -z) * RAD_TO_DEG;
        (pitch, roll)
    }
}

fn multiply(matrix: [[f32; 3]; 3], vector: [f32; 3]) -> [f32; 3] {
    let mut result = [0.0_f32; 3];
    for (row, out) in matrix.iter().zip(result.iter_mut()) {
        *out = row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2];
    }
    result
}

/// Attitude used by the control law and the telemetry stream, derived from
/// one BNO055 sample for the configured mount.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Attitude {
    /// Pitch in degrees, positive when leaning forward, before the zero
    /// offset is applied.
    pub pitch_deg: f32,
    /// Roll in degrees, positive when leaning left, before the zero offset
    /// is applied.
    pub roll_deg: f32,
    /// Pitch rate in degrees per second in the robot frame.
    pub pitch_rate_dps: f32,
    /// Yaw rate in degrees per second in the robot frame.
    pub yaw_rate_dps: f32,
    /// Roll rate in degrees per second in the robot frame.
    pub roll_rate_dps: f32,
}

/// Derive the control-frame attitude from one sample.
pub fn attitude(mount: Mount, euler: Euler, gyro: AngularRate, gravity_ms2: [f32; 3]) -> Attitude {
    let gyro = mount.map_rate(gyro);
    if mount.remaps_axes() {
        let (pitch_deg, roll_deg) = mount.pitch_roll_deg(gravity_ms2);
        Attitude {
            pitch_deg,
            roll_deg,
            pitch_rate_dps: gyro.y_dps,
            yaw_rate_dps: gyro.z_dps,
            roll_rate_dps: gyro.x_dps,
        }
    } else {
        Attitude {
            pitch_deg: euler.pitch_deg,
            roll_deg: euler.roll_deg,
            pitch_rate_dps: gyro.y_dps,
            yaw_rate_dps: gyro.z_dps,
            roll_rate_dps: gyro.x_dps,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: f32 = 9.806_65;

    fn robot_gravity(pitch_deg: f32, roll_deg: f32) -> [f32; 3] {
        // Compose rotations about Y (pitch) then X (roll) applied to the
        // upright vector (0, 0, -G). Forward lean gives +x, left lean +y.
        let (sp, cp) = pitch_deg.to_radians().sin_cos();
        let (sr, cr) = roll_deg.to_radians().sin_cos();
        // R_x(roll) * R_y(pitch) * (0, 0, -G)
        let after_pitch = [G * sp, 0.0, -G * cp];
        [
            after_pitch[0],
            after_pitch[1] * cr - after_pitch[2] * sr,
            after_pitch[1] * sr + after_pitch[2] * cr,
        ]
    }

    fn sensor_vector(mount: Mount, robot: [f32; 3]) -> [f32; 3] {
        // The inverse map is the transpose for proper rotations.
        mount.map_vector_inverse(robot)
    }

    const ALL_MOUNTS: [Mount; 7] = [
        Mount::Normal,
        Mount::FlipX,
        Mount::FlipY,
        Mount::RotXp,
        Mount::RotXm,
        Mount::RotYp,
        Mount::RotYm,
    ];

    #[test]
    fn map_round_trip_is_identity_for_every_mount() {
        for mount in ALL_MOUNTS {
            let robot = [0.3, -1.2, 9.5];
            let sensor = mount.map_vector(robot);
            let back = mount.map_vector_inverse(sensor);
            assert!((back[0] - robot[0]).abs() < 1e-6);
            assert!((back[1] - robot[1]).abs() < 1e-6);
            assert!((back[2] - robot[2]).abs() < 1e-6);
        }
    }

    #[test]
    fn matrices_are_proper_rotations() {
        for mount in ALL_MOUNTS {
            let m = mount.matrix();
            // M * M^T must be the identity.
            for row in 0..3 {
                for col in 0..3 {
                    let dot: f32 = (0..3).map(|k| m[row][k] * m[col][k]).sum();
                    let expected = if row == col { 1.0 } else { 0.0 };
                    assert!((dot - expected).abs() < 1e-6, "{mount:?} not orthogonal");
                }
            }
            // Determinant must be +1 (no reflection).
            let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            assert!((det - 1.0).abs() < 1e-6, "{mount:?} determinant {det}");
        }
    }

    #[test]
    fn selector_round_trip_and_unknown_fallback() {
        for mount in ALL_MOUNTS {
            assert_eq!(Mount::from_u32(mount.as_u32()), mount);
            assert_eq!(Mount::from_name(mount.name()), Some(mount));
        }
        assert_eq!(Mount::from_u32(9), Mount::Normal);
        assert_eq!(Mount::from_name("bogus"), None);
    }

    #[test]
    fn upright_reads_zero_for_every_mount() {
        for mount in ALL_MOUNTS {
            let robot = robot_gravity(0.0, 0.0);
            let sensor = sensor_vector(mount, robot);
            let (pitch, roll) = mount.pitch_roll_deg(sensor);
            assert!(pitch.abs() < 0.2, "pitch {pitch} for {mount:?}");
            assert!(roll.abs() < 0.2, "roll {roll} for {mount:?}");
        }
    }

    #[test]
    fn forward_lean_is_positive_for_every_mount() {
        for mount in ALL_MOUNTS {
            let robot = robot_gravity(30.0, 0.0);
            let sensor = sensor_vector(mount, robot);
            let (pitch, roll) = mount.pitch_roll_deg(sensor);
            assert!((pitch - 30.0).abs() < 0.2, "pitch {pitch} for {mount:?}");
            assert!(roll.abs() < 0.2, "roll {roll} for {mount:?}");
        }
    }

    #[test]
    fn left_lean_is_positive_for_every_mount() {
        for mount in ALL_MOUNTS {
            let robot = robot_gravity(0.0, 20.0);
            let sensor = sensor_vector(mount, robot);
            let (pitch, roll) = mount.pitch_roll_deg(sensor);
            assert!(pitch.abs() < 0.2, "pitch {pitch} for {mount:?}");
            assert!((roll - 20.0).abs() < 0.2, "roll {roll} for {mount:?}");
        }
    }

    #[test]
    fn gyro_signs_follow_the_mount() {
        let rate = AngularRate {
            x_dps: 1.0,
            y_dps: 2.0,
            z_dps: 3.0,
        };
        let normal = Mount::Normal.map_rate(rate);
        assert_eq!((normal.x_dps, normal.y_dps, normal.z_dps), (1.0, 2.0, 3.0));
        let flipx = Mount::FlipX.map_rate(rate);
        assert_eq!((flipx.x_dps, flipx.y_dps, flipx.z_dps), (1.0, -2.0, -3.0));
        let flipy = Mount::FlipY.map_rate(rate);
        assert_eq!((flipy.x_dps, flipy.y_dps, flipy.z_dps), (-1.0, 2.0, -3.0));
        let rotxp = Mount::RotXp.map_rate(rate);
        assert_eq!((rotxp.x_dps, rotxp.y_dps, rotxp.z_dps), (1.0, -3.0, 2.0));
        let rotxm = Mount::RotXm.map_rate(rate);
        assert_eq!((rotxm.x_dps, rotxm.y_dps, rotxm.z_dps), (1.0, 3.0, -2.0));
        let rotyp = Mount::RotYp.map_rate(rate);
        assert_eq!((rotyp.x_dps, rotyp.y_dps, rotyp.z_dps), (3.0, 2.0, -1.0));
        let rotym = Mount::RotYm.map_rate(rate);
        assert_eq!((rotym.x_dps, rotym.y_dps, rotym.z_dps), (-3.0, 2.0, 1.0));
    }

    #[test]
    fn attitude_falls_back_to_euler_for_normal_mount() {
        let euler = Euler {
            heading_deg: 90.0,
            roll_deg: -5.0,
            pitch_deg: 7.5,
        };
        let gyro = AngularRate {
            x_dps: 0.5,
            y_dps: 1.5,
            z_dps: 2.5,
        };
        let attitude = attitude(Mount::Normal, euler, gyro, [0.0, 0.0, -G]);
        assert!((attitude.pitch_deg - 7.5).abs() < 1e-6);
        assert!((attitude.roll_deg + 5.0).abs() < 1e-6);
        assert!((attitude.pitch_rate_dps - 1.5).abs() < 1e-6);
        assert!((attitude.yaw_rate_dps - 2.5).abs() < 1e-6);
    }
}
