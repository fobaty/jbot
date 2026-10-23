//! Deterministic sensor models for simulation.

use std::collections::VecDeque;
use std::fmt;

use jbot_control::StateVector;

use crate::rng::Pcg32;

/// Noise, bias, quantization, and environment parameters for the sensor suite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorConfig {
    pub imu_pitch_noise_rad: f64,
    pub imu_pitch_rate_noise_radps: f64,
    pub imu_pitch_bias_rad: f64,
    pub imu_pitch_rate_bias_radps: f64,
    pub encoder_position_noise_rad: f64,
    pub encoder_velocity_noise_rad_s: f64,
    pub encoder_quantization_rad: f64,
    pub current_noise_a: f64,
    pub voltage_noise_v: f64,
    pub battery_nominal_v: f64,
    pub battery_internal_resistance_ohm: f64,
    pub motor_torque_constant_nm_per_a: f64,
    pub lidar_max_range_m: f64,
    pub lidar_range_noise_m: f64,
    pub lidar_wall_distance_m: Option<f64>,
    pub measurement_delay_steps: usize,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            imu_pitch_noise_rad: 0.0,
            imu_pitch_rate_noise_radps: 0.0,
            imu_pitch_bias_rad: 0.0,
            imu_pitch_rate_bias_radps: 0.0,
            encoder_position_noise_rad: 0.0,
            encoder_velocity_noise_rad_s: 0.0,
            encoder_quantization_rad: 0.0,
            current_noise_a: 0.0,
            voltage_noise_v: 0.0,
            battery_nominal_v: 18.0,
            battery_internal_resistance_ohm: 0.05,
            motor_torque_constant_nm_per_a: 0.1,
            lidar_max_range_m: 10.0,
            lidar_range_noise_m: 0.0,
            lidar_wall_distance_m: None,
            measurement_delay_steps: 0,
        }
    }
}

impl SensorConfig {
    fn validate(&self) -> Result<(), SensorError> {
        non_negative("imu_pitch_noise_rad", self.imu_pitch_noise_rad)?;
        non_negative(
            "imu_pitch_rate_noise_radps",
            self.imu_pitch_rate_noise_radps,
        )?;
        finite("imu_pitch_bias_rad", self.imu_pitch_bias_rad)?;
        finite("imu_pitch_rate_bias_radps", self.imu_pitch_rate_bias_radps)?;
        non_negative(
            "encoder_position_noise_rad",
            self.encoder_position_noise_rad,
        )?;
        non_negative(
            "encoder_velocity_noise_rad_s",
            self.encoder_velocity_noise_rad_s,
        )?;
        non_negative("encoder_quantization_rad", self.encoder_quantization_rad)?;
        non_negative("current_noise_a", self.current_noise_a)?;
        non_negative("voltage_noise_v", self.voltage_noise_v)?;
        positive("battery_nominal_v", self.battery_nominal_v)?;
        non_negative(
            "battery_internal_resistance_ohm",
            self.battery_internal_resistance_ohm,
        )?;
        positive(
            "motor_torque_constant_nm_per_a",
            self.motor_torque_constant_nm_per_a,
        )?;
        positive("lidar_max_range_m", self.lidar_max_range_m)?;
        non_negative("lidar_range_noise_m", self.lidar_range_noise_m)?;
        if let Some(distance) = self.lidar_wall_distance_m {
            non_negative("lidar_wall_distance_m", distance)?;
        }
        Ok(())
    }
}

/// IMU sample for the planar model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub pitch_rad: f64,
    pub pitch_rate_radps: f64,
}

/// Wheel encoder sample for the planar model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSample {
    pub left_position_rad: f64,
    pub right_position_rad: f64,
    pub left_velocity_rad_s: f64,
    pub right_velocity_rad_s: f64,
}

/// Motor current and battery voltage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerSample {
    pub motor_current_a: f64,
    pub battery_voltage_v: f64,
}

/// One timestamped sensor sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorSample {
    pub time_s: f64,
    pub imu: ImuSample,
    pub wheels: WheelSample,
    pub power: PowerSample,
    pub lidar_range_m: f64,
}

/// Stateful deterministic sensor model.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorModel {
    config: SensorConfig,
    wheel_radius_m: f64,
    rng: Pcg32,
    delay_queue: VecDeque<SensorSample>,
}

impl SensorModel {
    pub fn new(config: SensorConfig, wheel_radius_m: f64, seed: u64) -> Result<Self, SensorError> {
        config.validate()?;
        positive("wheel_radius_m", wheel_radius_m)?;
        Ok(Self {
            config,
            wheel_radius_m,
            rng: Pcg32::from_seed(seed),
            delay_queue: VecDeque::with_capacity(config.measurement_delay_steps),
        })
    }

    pub fn config(&self) -> &SensorConfig {
        &self.config
    }

    pub fn reset(&mut self, seed: u64) {
        self.rng = Pcg32::from_seed(seed);
        self.delay_queue.clear();
    }

    pub fn sample(
        &mut self,
        time_s: f64,
        state: StateVector,
        total_wheel_torque_nm: f64,
    ) -> Result<SensorSample, SensorError> {
        if !time_s.is_finite() {
            return Err(SensorError::NonFiniteInput {
                name: "time_s",
                value: time_s,
            });
        }
        const STATE_NAMES: [&str; 4] = ["x_m", "x_dot_mps", "pitch_rad", "pitch_rate_radps"];
        for (index, value) in state.iter().enumerate() {
            if !value.is_finite() {
                return Err(SensorError::NonFiniteInput {
                    name: STATE_NAMES[index],
                    value: *value,
                });
            }
        }
        if !total_wheel_torque_nm.is_finite() {
            return Err(SensorError::NonFiniteInput {
                name: "total_wheel_torque_nm",
                value: total_wheel_torque_nm,
            });
        }

        let pitch_rad = state[2]
            + self.config.imu_pitch_bias_rad
            + self.config.imu_pitch_noise_rad * self.rng.next_normal();
        let pitch_rate_radps = state[3]
            + self.config.imu_pitch_rate_bias_radps
            + self.config.imu_pitch_rate_noise_radps * self.rng.next_normal();

        let wheel_position = state[0] / self.wheel_radius_m
            + self.config.encoder_position_noise_rad * self.rng.next_normal();
        let wheel_velocity = state[1] / self.wheel_radius_m
            + self.config.encoder_velocity_noise_rad_s * self.rng.next_normal();
        let wheel_position = quantize(wheel_position, self.config.encoder_quantization_rad);

        let motor_current_a = total_wheel_torque_nm / self.config.motor_torque_constant_nm_per_a
            + self.config.current_noise_a * self.rng.next_normal();
        let battery_voltage_v = (self.config.battery_nominal_v
            - motor_current_a * self.config.battery_internal_resistance_ohm
            + self.config.voltage_noise_v * self.rng.next_normal())
        .max(0.0);

        let range = match self.config.lidar_wall_distance_m {
            Some(distance) => distance - state[0],
            None => self.config.lidar_max_range_m,
        };
        let lidar_range_m = (range + self.config.lidar_range_noise_m * self.rng.next_normal())
            .clamp(0.0, self.config.lidar_max_range_m);

        let sample = SensorSample {
            time_s,
            imu: ImuSample {
                pitch_rad,
                pitch_rate_radps,
            },
            wheels: WheelSample {
                left_position_rad: wheel_position,
                right_position_rad: wheel_position,
                left_velocity_rad_s: wheel_velocity,
                right_velocity_rad_s: wheel_velocity,
            },
            power: PowerSample {
                motor_current_a,
                battery_voltage_v,
            },
            lidar_range_m,
        };

        if sample.imu.pitch_rad.is_finite()
            && sample.imu.pitch_rate_radps.is_finite()
            && sample.wheels.left_position_rad.is_finite()
            && sample.wheels.right_position_rad.is_finite()
            && sample.wheels.left_velocity_rad_s.is_finite()
            && sample.wheels.right_velocity_rad_s.is_finite()
            && sample.power.motor_current_a.is_finite()
            && sample.power.battery_voltage_v.is_finite()
            && sample.lidar_range_m.is_finite()
        {
            self.delayed(sample)
        } else {
            Err(SensorError::NonFiniteOutput)
        }
    }

    fn delayed(&mut self, sample: SensorSample) -> Result<SensorSample, SensorError> {
        if self.config.measurement_delay_steps == 0 {
            return Ok(sample);
        }

        self.delay_queue.push_back(sample);
        if self.delay_queue.len() > self.config.measurement_delay_steps {
            if let Some(delayed) = self.delay_queue.pop_front() {
                return Ok(delayed);
            }
        }
        if let Some(delayed) = self.delay_queue.front().copied() {
            Ok(delayed)
        } else {
            Ok(sample)
        }
    }
}

/// Reconstruction of the full planar state from one sensor sample.
pub trait StateEstimate {
    fn estimate(&self, sample: &SensorSample) -> StateVector;
}

/// Minimal estimator that maps encoder and IMU channels back to the state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EncoderImuEstimator {
    wheel_radius_m: f64,
}

impl EncoderImuEstimator {
    pub fn new(wheel_radius_m: f64) -> Result<Self, SensorError> {
        positive("wheel_radius_m", wheel_radius_m)?;
        Ok(Self { wheel_radius_m })
    }

    pub fn wheel_radius_m(&self) -> f64 {
        self.wheel_radius_m
    }
}

impl StateEstimate for EncoderImuEstimator {
    fn estimate(&self, sample: &SensorSample) -> StateVector {
        StateVector::new(
            sample.wheels.left_position_rad * self.wheel_radius_m,
            sample.wheels.left_velocity_rad_s * self.wheel_radius_m,
            sample.imu.pitch_rad,
            sample.imu.pitch_rate_radps,
        )
    }
}

/// Errors from sensor configuration and sampling.
#[derive(Debug, Clone, PartialEq)]
pub enum SensorError {
    InvalidConfig { field: &'static str, value: f64 },
    NonFiniteInput { name: &'static str, value: f64 },
    NonFiniteOutput,
}

impl fmt::Display for SensorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig { field, value } => {
                write!(f, "invalid sensor config {field}={value}")
            }
            Self::NonFiniteInput { name, value } => {
                write!(f, "non-finite sensor input {name}={value}")
            }
            Self::NonFiniteOutput => write!(f, "sensor model produced a non-finite value"),
        }
    }
}

impl std::error::Error for SensorError {}

fn finite(name: &'static str, value: f64) -> Result<(), SensorError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(SensorError::InvalidConfig { field: name, value })
    }
}

fn positive(name: &'static str, value: f64) -> Result<(), SensorError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(SensorError::InvalidConfig { field: name, value })
    }
}

fn non_negative(name: &'static str, value: f64) -> Result<(), SensorError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(SensorError::InvalidConfig { field: name, value })
    }
}

fn quantize(value: f64, step: f64) -> f64 {
    if step > 0.0 {
        (value / step).round() * step
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ideal_config() -> SensorConfig {
        SensorConfig {
            battery_nominal_v: 18.0,
            battery_internal_resistance_ohm: 0.05,
            motor_torque_constant_nm_per_a: 0.1,
            lidar_max_range_m: 10.0,
            lidar_wall_distance_m: Some(5.0),
            ..SensorConfig::default()
        }
    }

    #[test]
    fn ideal_sensors_reconstruct_the_state() {
        let mut model = SensorModel::new(ideal_config(), 0.09, 1).expect("valid model");
        let state = StateVector::new(1.0, 2.0, 0.1, 0.2);
        let sample = model.sample(0.5, state, 2.0).expect("sample succeeds");

        assert_eq!(sample.time_s, 0.5);
        assert!((sample.imu.pitch_rad - 0.1).abs() < 1e-12);
        assert!((sample.imu.pitch_rate_radps - 0.2).abs() < 1e-12);
        assert!((sample.wheels.left_position_rad - 1.0 / 0.09).abs() < 1e-12);
        assert!((sample.wheels.left_velocity_rad_s - 2.0 / 0.09).abs() < 1e-12);
        assert!((sample.power.motor_current_a - 20.0).abs() < 1e-12);
        assert!((sample.power.battery_voltage_v - (18.0 - 20.0 * 0.05)).abs() < 1e-12);
        assert!((sample.lidar_range_m - 4.0).abs() < 1e-12);
    }

    #[test]
    fn noise_is_reproducible_for_a_seed() {
        let config = SensorConfig {
            imu_pitch_noise_rad: 0.01,
            imu_pitch_rate_noise_radps: 0.02,
            encoder_position_noise_rad: 0.03,
            encoder_velocity_noise_rad_s: 0.04,
            current_noise_a: 0.05,
            voltage_noise_v: 0.06,
            lidar_range_noise_m: 0.07,
            lidar_wall_distance_m: Some(5.0),
            ..SensorConfig::default()
        };
        let state = StateVector::new(0.1, 0.2, 0.3, 0.4);
        let mut first = SensorModel::new(config, 0.09, 123).expect("valid model");
        let mut second = SensorModel::new(config, 0.09, 123).expect("valid model");

        for step in 0..100 {
            let time_s = step as f64 * 0.001;
            assert_eq!(
                first.sample(time_s, state, 1.0).expect("sample"),
                second.sample(time_s, state, 1.0).expect("sample")
            );
        }
    }

    #[test]
    fn different_seed_changes_the_noise() {
        let config = SensorConfig {
            imu_pitch_noise_rad: 0.01,
            ..SensorConfig::default()
        };
        let state = StateVector::new(0.0, 0.0, 0.0, 0.0);
        let mut first = SensorModel::new(config, 0.09, 1).expect("valid model");
        let mut second = SensorModel::new(config, 0.09, 2).expect("valid model");

        assert_ne!(
            first.sample(0.0, state, 0.0).expect("sample").imu.pitch_rad,
            second
                .sample(0.0, state, 0.0)
                .expect("sample")
                .imu
                .pitch_rad
        );
    }

    #[test]
    fn bias_and_quantization_are_applied() {
        let config = SensorConfig {
            imu_pitch_bias_rad: 0.05,
            encoder_quantization_rad: 0.01,
            ..SensorConfig::default()
        };
        let mut model = SensorModel::new(config, 0.1, 1).expect("valid model");
        let sample = model
            .sample(0.0, StateVector::new(0.123, 0.0, 0.2, 0.0), 0.0)
            .expect("sample succeeds");

        assert!((sample.imu.pitch_rad - 0.25).abs() < 1e-12);
        let position = sample.wheels.left_position_rad;
        assert!((position / 0.01 - (position / 0.01).round()).abs() < 1e-9);
    }

    #[test]
    fn measurement_delay_returns_old_timestamps() {
        let config = SensorConfig {
            measurement_delay_steps: 2,
            ..SensorConfig::default()
        };
        let mut model = SensorModel::new(config, 0.1, 1).expect("valid model");

        for step in 0..4usize {
            let sample = model
                .sample(step as f64, StateVector::zeros(), 0.0)
                .expect("sample succeeds");
            let expected = (step.saturating_sub(2)) as f64;
            assert!((sample.time_s - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn lidar_range_is_clamped() {
        let mut config = ideal_config();
        config.lidar_wall_distance_m = Some(1.0);
        let mut model = SensorModel::new(config, 0.1, 1).expect("valid model");

        let sample = model
            .sample(0.0, StateVector::new(5.0, 0.0, 0.0, 0.0), 0.0)
            .expect("sample succeeds");
        assert_eq!(sample.lidar_range_m, 0.0);
    }

    #[test]
    fn encoder_imu_estimator_reconstructs_the_state() {
        let mut model = SensorModel::new(ideal_config(), 0.09, 1).expect("valid model");
        let sample = model
            .sample(0.0, StateVector::new(1.0, 2.0, 0.1, 0.2), 0.0)
            .expect("sample succeeds");
        let estimator = EncoderImuEstimator::new(0.09).expect("valid estimator");
        let estimate = estimator.estimate(&sample);

        assert!((estimate[0] - 1.0).abs() < 1e-12);
        assert!((estimate[1] - 2.0).abs() < 1e-12);
        assert!((estimate[2] - 0.1).abs() < 1e-12);
        assert!((estimate[3] - 0.2).abs() < 1e-12);
    }

    #[test]
    fn rejects_invalid_config() {
        let config = SensorConfig {
            motor_torque_constant_nm_per_a: 0.0,
            ..SensorConfig::default()
        };
        assert!(matches!(
            SensorModel::new(config, 0.1, 1),
            Err(SensorError::InvalidConfig {
                field: "motor_torque_constant_nm_per_a",
                ..
            })
        ));
        assert!(matches!(
            SensorModel::new(ideal_config(), 0.0, 1),
            Err(SensorError::InvalidConfig {
                field: "wheel_radius_m",
                ..
            })
        ));
    }
}
