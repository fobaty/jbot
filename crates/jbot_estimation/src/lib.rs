//! Attitude estimation filters for pitch and roll.
//!
//! Both filters consume a 6-axis IMU sample: specific force in m/s^2 and angular
//! rate in rad/s. The accelerometer is assumed to measure gravity as `+g` on the
//! body Z axis when level, which is the usual stationary accelerometer reading.

use std::f32::consts::{FRAC_PI_2, PI};

const MIN_ACCEL_NORM_MPS2: f32 = 1.0e-3;
const MAX_PITCH_RAD: f32 = FRAC_PI_2 - 1.0e-4;

/// One 6-axis IMU sample in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    /// Specific force, m/s^2. Level at rest is `[0, 0, +g]`.
    pub accel_mps2: [f32; 3],
    /// Angular rate, rad/s, ordered as roll, pitch, yaw axes.
    pub gyro_radps: [f32; 3],
}

impl ImuReading {
    pub fn new(accel_mps2: [f32; 3], gyro_radps: [f32; 3]) -> Self {
        Self {
            accel_mps2,
            gyro_radps,
        }
    }
}

/// Estimated roll, pitch, and yaw in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
}

impl Attitude {
    pub fn new(roll_rad: f32, pitch_rad: f32, yaw_rad: f32) -> Self {
        Self {
            roll_rad,
            pitch_rad,
            yaw_rad,
        }
    }
}

impl Default for Attitude {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

/// Errors from filter configuration or invalid IMU input.
#[derive(Debug, Clone, PartialEq)]
pub enum EstimationError {
    InvalidParameter { name: &'static str, value: f32 },
    InvalidTimeStep { dt_s: f32 },
    NonFiniteSample { field: &'static str, value: f32 },
}

impl std::fmt::Display for EstimationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidParameter { name, value } => {
                write!(f, "invalid estimation parameter {name}={value}")
            }
            Self::InvalidTimeStep { dt_s } => {
                write!(f, "invalid estimation time step dt_s={dt_s}")
            }
            Self::NonFiniteSample { field, value } => {
                write!(f, "non-finite IMU sample {field}={value}")
            }
        }
    }
}

impl std::error::Error for EstimationError {}

/// Complementary filter that fuses gyro integration with accelerometer angles.
///
/// The filter uses a first-order blend with `alpha = tau / (tau + dt)`, so its
/// cutoff is independent of the update rate. Roll is wrapped to `[-pi, pi]`.
#[derive(Debug, Clone, PartialEq)]
pub struct ComplementaryFilter {
    attitude: Attitude,
    time_constant_s: f32,
    initialized: bool,
}

impl ComplementaryFilter {
    pub fn new(time_constant_s: f32) -> Result<Self, EstimationError> {
        positive("time_constant_s", time_constant_s)?;
        Ok(Self {
            attitude: Attitude::default(),
            time_constant_s,
            initialized: false,
        })
    }

    pub fn attitude(&self) -> Attitude {
        self.attitude
    }

    pub fn reset(&mut self) {
        self.attitude = Attitude::default();
        self.initialized = false;
    }

    pub fn update(&mut self, reading: &ImuReading, dt_s: f32) -> Result<Attitude, EstimationError> {
        validate_reading(reading)?;
        validate_dt(dt_s)?;

        let accel = accel_angles(reading.accel_mps2);
        if !self.initialized {
            if let Some((roll, pitch)) = accel {
                self.attitude = Attitude::new(roll, pitch, 0.0);
                self.initialized = true;
                return Ok(self.attitude);
            }
        }

        let [gx, gy, _] = reading.gyro_radps;
        let alpha = self.time_constant_s / (self.time_constant_s + dt_s);
        let roll_gyro = self.attitude.roll_rad + gx * dt_s;
        let pitch_gyro = self.attitude.pitch_rad + gy * dt_s;

        let (roll, pitch) = if let Some((roll_acc, pitch_acc)) = accel {
            let roll = wrap_angle(roll_gyro + (1.0 - alpha) * wrap_angle(roll_acc - roll_gyro));
            let pitch = (pitch_gyro + (1.0 - alpha) * (pitch_acc - pitch_gyro))
                .clamp(-MAX_PITCH_RAD, MAX_PITCH_RAD);
            (roll, pitch)
        } else {
            (
                wrap_angle(roll_gyro),
                pitch_gyro.clamp(-MAX_PITCH_RAD, MAX_PITCH_RAD),
            )
        };

        self.attitude = Attitude::new(roll, pitch, self.attitude.yaw_rad);
        self.initialized = true;
        Ok(self.attitude)
    }
}

impl Default for ComplementaryFilter {
    fn default() -> Self {
        Self::new(0.5).expect("default time constant is valid")
    }
}

/// Madgwick 6-axis AHRS filter using gyro propagation and accelerometer correction.
#[derive(Debug, Clone, PartialEq)]
pub struct MadgwickFilter {
    quaternion: [f32; 4],
    beta: f32,
}

impl MadgwickFilter {
    pub fn new(beta: f32) -> Result<Self, EstimationError> {
        non_negative("beta", beta)?;
        Ok(Self {
            quaternion: [1.0, 0.0, 0.0, 0.0],
            beta,
        })
    }

    pub fn quaternion(&self) -> [f32; 4] {
        self.quaternion
    }

    pub fn attitude(&self) -> Attitude {
        let [q0, q1, q2, q3] = self.quaternion;
        Attitude::new(
            (2.0 * (q0 * q1 + q2 * q3)).atan2(1.0 - 2.0 * (q1 * q1 + q2 * q2)),
            (2.0 * (q0 * q2 - q3 * q1)).clamp(-1.0, 1.0).asin(),
            (2.0 * (q0 * q3 + q1 * q2)).atan2(1.0 - 2.0 * (q2 * q2 + q3 * q3)),
        )
    }

    pub fn reset(&mut self) {
        self.quaternion = [1.0, 0.0, 0.0, 0.0];
    }

    pub fn update(&mut self, reading: &ImuReading, dt_s: f32) -> Result<Attitude, EstimationError> {
        validate_reading(reading)?;
        validate_dt(dt_s)?;

        let [gx, gy, gz] = reading.gyro_radps;
        let [mut ax, mut ay, mut az] = reading.accel_mps2;
        let [mut q0, mut q1, mut q2, mut q3] = self.quaternion;

        let mut qdot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        let mut qdot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        let mut qdot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        let mut qdot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        let accel_norm_sq = ax * ax + ay * ay + az * az;
        if accel_norm_sq > MIN_ACCEL_NORM_MPS2 * MIN_ACCEL_NORM_MPS2 {
            let inv_norm = 1.0 / accel_norm_sq.sqrt();
            ax *= inv_norm;
            ay *= inv_norm;
            az *= inv_norm;

            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _4q0 = 4.0 * q0;
            let _4q1 = 4.0 * q1;
            let _4q2 = 4.0 * q2;
            let _8q1 = 8.0 * q1;
            let _8q2 = 8.0 * q2;
            let q0q0 = q0 * q0;
            let q1q1 = q1 * q1;
            let q2q2 = q2 * q2;
            let q3q3 = q3 * q3;

            let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1
                + _8q1 * q1q1
                + _8q1 * q2q2
                + _4q1 * az;
            let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                + _8q2 * q1q1
                + _8q2 * q2q2
                + _4q2 * az;
            let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

            let step_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
            if step_norm_sq > 0.0 {
                let inv_step_norm = 1.0 / step_norm_sq.sqrt();
                qdot0 -= self.beta * s0 * inv_step_norm;
                qdot1 -= self.beta * s1 * inv_step_norm;
                qdot2 -= self.beta * s2 * inv_step_norm;
                qdot3 -= self.beta * s3 * inv_step_norm;
            }
        }

        q0 += qdot0 * dt_s;
        q1 += qdot1 * dt_s;
        q2 += qdot2 * dt_s;
        q3 += qdot3 * dt_s;

        let norm = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3).sqrt();
        let inv_norm = 1.0 / norm;
        self.quaternion = [q0 * inv_norm, q1 * inv_norm, q2 * inv_norm, q3 * inv_norm];
        Ok(self.attitude())
    }
}

impl Default for MadgwickFilter {
    fn default() -> Self {
        Self::new(0.1).expect("default beta is valid")
    }
}

fn accel_angles(accel_mps2: [f32; 3]) -> Option<(f32, f32)> {
    let [ax, ay, az] = accel_mps2;
    let norm_sq = ax * ax + ay * ay + az * az;
    if norm_sq < MIN_ACCEL_NORM_MPS2 * MIN_ACCEL_NORM_MPS2 {
        return None;
    }
    let inv_norm = 1.0 / norm_sq.sqrt();
    let ax = ax * inv_norm;
    let ay = ay * inv_norm;
    let az = az * inv_norm;
    let roll = ay.atan2(az);
    let pitch = (-ax).atan2((ay * ay + az * az).sqrt());
    Some((roll, pitch))
}

fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

fn positive(name: &'static str, value: f32) -> Result<(), EstimationError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(EstimationError::InvalidParameter { name, value })
    }
}

fn non_negative(name: &'static str, value: f32) -> Result<(), EstimationError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(EstimationError::InvalidParameter { name, value })
    }
}

fn validate_dt(dt_s: f32) -> Result<(), EstimationError> {
    if dt_s.is_finite() && dt_s > 0.0 {
        Ok(())
    } else {
        Err(EstimationError::InvalidTimeStep { dt_s })
    }
}

fn validate_reading(reading: &ImuReading) -> Result<(), EstimationError> {
    for (index, value) in reading.accel_mps2.iter().enumerate() {
        finite(
            match index {
                0 => "accel_mps2[0]",
                1 => "accel_mps2[1]",
                _ => "accel_mps2[2]",
            },
            *value,
        )?;
    }
    for (index, value) in reading.gyro_radps.iter().enumerate() {
        finite(
            match index {
                0 => "gyro_radps[0]",
                1 => "gyro_radps[1]",
                _ => "gyro_radps[2]",
            },
            *value,
        )?;
    }
    Ok(())
}

fn finite(field: &'static str, value: f32) -> Result<(), EstimationError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(EstimationError::NonFiniteSample { field, value })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GRAVITY: f32 = 9.806_65;

    fn static_reading(roll_rad: f32, pitch_rad: f32) -> ImuReading {
        let (sr, cr) = roll_rad.sin_cos();
        let (sp, cp) = pitch_rad.sin_cos();
        ImuReading::new(
            [-GRAVITY * sp, GRAVITY * sr * cp, GRAVITY * cr * cp],
            [0.0, 0.0, 0.0],
        )
    }

    #[test]
    fn complementary_converges_to_static_tilt() {
        let roll = 5.0_f32.to_radians();
        let pitch = 10.0_f32.to_radians();
        let reading = static_reading(roll, pitch);
        let mut filter = ComplementaryFilter::new(0.2).expect("valid filter");

        let mut estimate = Attitude::default();
        for _ in 0..1_000 {
            estimate = filter.update(&reading, 0.01).expect("valid update");
        }

        assert!((estimate.roll_rad - roll).abs() < 1.0e-4);
        assert!((estimate.pitch_rad - pitch).abs() < 1.0e-4);
    }

    #[test]
    fn madgwick_converges_to_static_tilt() {
        let roll = 5.0_f32.to_radians();
        let pitch = 10.0_f32.to_radians();
        let reading = static_reading(roll, pitch);
        let mut filter = MadgwickFilter::new(0.2).expect("valid filter");

        let mut estimate = Attitude::default();
        for _ in 0..5_000 {
            estimate = filter.update(&reading, 0.01).expect("valid update");
        }

        assert!((estimate.roll_rad - roll).abs() < 0.005);
        assert!((estimate.pitch_rad - pitch).abs() < 0.005);
    }

    #[test]
    fn complementary_integrates_gyro_without_accel() {
        let mut filter = ComplementaryFilter::new(0.5).expect("valid filter");
        filter
            .update(&static_reading(0.0, 0.0), 0.01)
            .expect("valid update");
        let gyro = ImuReading::new([0.0, 0.0, 0.0], [0.1, 0.0, 0.0]);
        for _ in 0..100 {
            filter.update(&gyro, 0.01).expect("valid update");
        }
        assert!((filter.attitude().roll_rad - 0.1).abs() < 1.0e-3);
    }

    #[test]
    fn madgwick_quaternion_stays_normalized() {
        let mut filter = MadgwickFilter::new(0.1).expect("valid filter");
        let reading = ImuReading::new([0.0, 0.0, GRAVITY], [0.3, -0.2, 0.1]);
        for _ in 0..1_000 {
            filter.update(&reading, 0.01).expect("valid update");
            let norm = filter
                .quaternion()
                .iter()
                .map(|value| value * value)
                .sum::<f32>()
                .sqrt();
            assert!((norm - 1.0).abs() < 1.0e-5);
        }
    }

    #[test]
    fn zero_accel_uses_gyro_only() {
        let mut filter = MadgwickFilter::new(0.2).expect("valid filter");
        let reading = ImuReading::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        for _ in 0..100 {
            filter.update(&reading, 0.01).expect("valid update");
        }
        assert!((filter.quaternion()[0] - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn rejects_invalid_parameters_and_samples() {
        assert!(matches!(
            ComplementaryFilter::new(0.0),
            Err(EstimationError::InvalidParameter {
                name: "time_constant_s",
                ..
            })
        ));
        assert!(matches!(
            MadgwickFilter::new(-0.1),
            Err(EstimationError::InvalidParameter { name: "beta", .. })
        ));

        let mut filter = MadgwickFilter::default();
        let reading = static_reading(0.0, 0.0);
        assert!(matches!(
            filter.update(&reading, 0.0),
            Err(EstimationError::InvalidTimeStep { .. })
        ));
        assert!(matches!(
            filter.update(
                &ImuReading::new([f32::NAN, 0.0, 1.0], [0.0, 0.0, 0.0]),
                0.01
            ),
            Err(EstimationError::NonFiniteSample {
                field: "accel_mps2[0]",
                ..
            })
        ));
    }
}
