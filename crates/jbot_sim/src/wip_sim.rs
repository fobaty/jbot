//! Fixed-step nonlinear simulation of the 2D wheeled inverted pendulum.

use std::collections::VecDeque;
use std::fmt;

use jbot_control::{
    LqrController, StateVector, VelocityController, WheeledInvertedPendulum, WipError,
    WipParameters,
};

use crate::rk4::rk4_step;
use crate::sensors::{SensorError, SensorModel, StateEstimate};

/// Timing, actuator, and fall-detection limits for one simulation run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulationConfig {
    /// Fixed integration and control period, seconds.
    pub dt_s: f64,
    /// Total torque available from both wheels, N*m.
    pub max_total_torque_nm: f64,
    /// Absolute pitch at which the run is considered fallen, radians.
    pub fall_limit_rad: f64,
    /// Ground slope, positive uphill, radians.
    pub slope_rad: f64,
    /// Number of control periods the actuator command is delayed.
    pub command_delay_steps: usize,
    /// Total current limit for both wheels, A. `None` disables the limit.
    pub max_total_current_a: Option<f64>,
    /// Motor torque constant used to convert current to torque, N*m/A.
    pub motor_torque_constant_nm_per_a: f64,
    /// Command slew-rate limit, N*m/s. `None` disables rate limiting.
    pub torque_rate_limit_nm_per_s: Option<f64>,
    /// Actuator dead zone, N*m.
    pub dead_zone_nm: f64,
    /// Coulomb friction torque at the wheels, N*m.
    pub coulomb_friction_nm: f64,
    /// Viscous friction coefficient at the wheels, N*m*s/rad.
    pub viscous_friction_nm_per_rad_s: f64,
}

impl SimulationConfig {
    pub fn new(dt_s: f64, max_total_torque_nm: f64, fall_limit_rad: f64) -> Result<Self, SimError> {
        Self::with_slope(dt_s, max_total_torque_nm, fall_limit_rad, 0.0)
    }

    pub fn with_slope(
        dt_s: f64,
        max_total_torque_nm: f64,
        fall_limit_rad: f64,
        slope_rad: f64,
    ) -> Result<Self, SimError> {
        Self::with_delay(dt_s, max_total_torque_nm, fall_limit_rad, slope_rad, 0)
    }

    pub fn with_delay(
        dt_s: f64,
        max_total_torque_nm: f64,
        fall_limit_rad: f64,
        slope_rad: f64,
        command_delay_steps: usize,
    ) -> Result<Self, SimError> {
        let config = Self {
            dt_s,
            max_total_torque_nm,
            fall_limit_rad,
            slope_rad,
            command_delay_steps,
            max_total_current_a: None,
            motor_torque_constant_nm_per_a: 0.1,
            torque_rate_limit_nm_per_s: None,
            dead_zone_nm: 0.0,
            coulomb_friction_nm: 0.0,
            viscous_friction_nm_per_rad_s: 0.0,
        };
        config.validate()?;
        Ok(config)
    }

    /// Configure actuator non-idealities and revalidate the result.
    #[allow(clippy::too_many_arguments)]
    pub fn with_actuator_limits(
        mut self,
        max_total_current_a: Option<f64>,
        motor_torque_constant_nm_per_a: f64,
        torque_rate_limit_nm_per_s: Option<f64>,
        dead_zone_nm: f64,
        coulomb_friction_nm: f64,
        viscous_friction_nm_per_rad_s: f64,
    ) -> Result<Self, SimError> {
        self.max_total_current_a = max_total_current_a;
        self.motor_torque_constant_nm_per_a = motor_torque_constant_nm_per_a;
        self.torque_rate_limit_nm_per_s = torque_rate_limit_nm_per_s;
        self.dead_zone_nm = dead_zone_nm;
        self.coulomb_friction_nm = coulomb_friction_nm;
        self.viscous_friction_nm_per_rad_s = viscous_friction_nm_per_rad_s;
        self.validate()?;
        Ok(self)
    }

    fn validate(&self) -> Result<(), SimError> {
        if !self.dt_s.is_finite() || self.dt_s <= 0.0 {
            return Err(SimError::InvalidConfig {
                field: "dt_s",
                value: self.dt_s,
            });
        }
        if !self.max_total_torque_nm.is_finite() || self.max_total_torque_nm <= 0.0 {
            return Err(SimError::InvalidConfig {
                field: "max_total_torque_nm",
                value: self.max_total_torque_nm,
            });
        }
        if !self.fall_limit_rad.is_finite()
            || self.fall_limit_rad <= 0.0
            || self.fall_limit_rad >= std::f64::consts::FRAC_PI_2
        {
            return Err(SimError::InvalidConfig {
                field: "fall_limit_rad",
                value: self.fall_limit_rad,
            });
        }
        if !self.slope_rad.is_finite() || self.slope_rad.abs() >= std::f64::consts::FRAC_PI_2 {
            return Err(SimError::InvalidConfig {
                field: "slope_rad",
                value: self.slope_rad,
            });
        }
        if let Some(current_a) = self.max_total_current_a {
            if !current_a.is_finite() || current_a <= 0.0 {
                return Err(SimError::InvalidConfig {
                    field: "max_total_current_a",
                    value: current_a,
                });
            }
        }
        if !self.motor_torque_constant_nm_per_a.is_finite()
            || self.motor_torque_constant_nm_per_a <= 0.0
        {
            return Err(SimError::InvalidConfig {
                field: "motor_torque_constant_nm_per_a",
                value: self.motor_torque_constant_nm_per_a,
            });
        }
        if let Some(rate) = self.torque_rate_limit_nm_per_s {
            if !rate.is_finite() || rate <= 0.0 {
                return Err(SimError::InvalidConfig {
                    field: "torque_rate_limit_nm_per_s",
                    value: rate,
                });
            }
        }
        let non_negative = [
            ("dead_zone_nm", self.dead_zone_nm),
            ("coulomb_friction_nm", self.coulomb_friction_nm),
            (
                "viscous_friction_nm_per_rad_s",
                self.viscous_friction_nm_per_rad_s,
            ),
        ];
        for (field, value) in non_negative {
            if !value.is_finite() || value < 0.0 {
                return Err(SimError::InvalidConfig { field, value });
            }
        }
        Ok(())
    }
}

/// Horizontal force impulse applied to the wheel axle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Impulse {
    /// Application time, seconds. Must align with a simulation step.
    pub at_s: f64,
    /// Impulse magnitude, N*s.
    pub force_ns: f64,
}

/// Mid-run change of the plant parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParameterChange {
    /// Application time, seconds. Must align with a simulation step.
    pub at_s: f64,
    /// New plant parameters from this step onward.
    pub parameters: WipParameters,
}

/// Controller contract used by the simulator.
pub trait StateFeedback {
    /// Total wheel torque requested for the given full state, N*m.
    fn torque(&self, state: StateVector) -> f64;

    /// Torque requested at an absolute simulation time.
    fn torque_at(&self, _time_s: f64, state: StateVector) -> f64 {
        self.torque(state)
    }
}

impl StateFeedback for LqrController {
    fn torque(&self, state: StateVector) -> f64 {
        self.control(state)
    }
}

impl StateFeedback for VelocityController {
    fn torque(&self, state: StateVector) -> f64 {
        self.control(state)
    }

    fn torque_at(&self, time_s: f64, state: StateVector) -> f64 {
        self.control_at(time_s, state)
    }
}

/// One sample at the start of a control period.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceSample {
    pub time_s: f64,
    pub state: StateVector,
    /// Torque after actuator saturation, N*m.
    pub torque_nm: f64,
    pub saturated: bool,
}

/// In-memory simulation trace.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    pub(crate) samples: Vec<TraceSample>,
    pub(crate) dt_s: f64,
}

impl Trace {
    pub fn samples(&self) -> &[TraceSample] {
        &self.samples
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn dt_s(&self) -> f64 {
        self.dt_s
    }

    pub fn final_state(&self) -> Option<StateVector> {
        self.samples.last().map(|sample| sample.state)
    }

    pub fn max_abs_pitch_rad(&self) -> f64 {
        self.samples.iter().fold(0.0_f64, |maximum, sample| {
            maximum.max(sample.state[2].abs())
        })
    }

    pub fn max_abs_torque_nm(&self) -> f64 {
        self.samples.iter().fold(0.0_f64, |maximum, sample| {
            maximum.max(sample.torque_nm.abs())
        })
    }
}

/// Result of one simulation run.
#[derive(Debug, Clone, PartialEq)]
pub struct SimulationOutcome {
    pub trace: Trace,
    /// Time at which the pitch limit was first exceeded, if any.
    pub fell_at_s: Option<f64>,
}

/// Nonlinear 2D WIP simulator with fixed-step RK4 integration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WipSimulator {
    plant: WheeledInvertedPendulum,
    config: SimulationConfig,
}

impl WipSimulator {
    pub fn new(plant: WheeledInvertedPendulum, config: SimulationConfig) -> Result<Self, SimError> {
        config.validate()?;
        Ok(Self { plant, config })
    }

    pub fn plant(&self) -> &WheeledInvertedPendulum {
        &self.plant
    }

    pub fn config(&self) -> &SimulationConfig {
        &self.config
    }

    /// Effective horizontal inertia of the wheels and body at zero pitch.
    pub fn effective_cart_mass_kg(&self) -> f64 {
        effective_cart_mass_kg(self.plant.parameters())
    }

    pub fn run<C: StateFeedback + ?Sized>(
        &self,
        controller: &C,
        initial_state: StateVector,
        duration_s: f64,
        impulses: &[Impulse],
    ) -> Result<SimulationOutcome, SimError> {
        self.run_with_parameter_changes(controller, initial_state, duration_s, impulses, &[])
    }

    pub fn run_with_parameter_changes<C: StateFeedback + ?Sized>(
        &self,
        controller: &C,
        initial_state: StateVector,
        duration_s: f64,
        impulses: &[Impulse],
        parameter_changes: &[ParameterChange],
    ) -> Result<SimulationOutcome, SimError> {
        self.run_inner(
            controller,
            None,
            None,
            initial_state,
            duration_s,
            impulses,
            parameter_changes,
        )
    }

    /// Run the closed loop from delayed, noisy sensor samples.
    pub fn run_with_sensors<C: StateFeedback + ?Sized, E: StateEstimate>(
        &self,
        controller: &C,
        estimator: &E,
        sensors: &mut SensorModel,
        initial_state: StateVector,
        duration_s: f64,
        impulses: &[Impulse],
    ) -> Result<SimulationOutcome, SimError> {
        self.run_with_sensors_and_changes(
            controller,
            estimator,
            sensors,
            initial_state,
            duration_s,
            impulses,
            &[],
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn run_with_sensors_and_changes<C: StateFeedback + ?Sized, E: StateEstimate>(
        &self,
        controller: &C,
        estimator: &E,
        sensors: &mut SensorModel,
        initial_state: StateVector,
        duration_s: f64,
        impulses: &[Impulse],
        parameter_changes: &[ParameterChange],
    ) -> Result<SimulationOutcome, SimError> {
        self.run_inner(
            controller,
            Some(estimator),
            Some(sensors),
            initial_state,
            duration_s,
            impulses,
            parameter_changes,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn run_inner<C: StateFeedback + ?Sized>(
        &self,
        controller: &C,
        estimator: Option<&dyn StateEstimate>,
        mut sensors: Option<&mut SensorModel>,
        initial_state: StateVector,
        duration_s: f64,
        impulses: &[Impulse],
        parameter_changes: &[ParameterChange],
    ) -> Result<SimulationOutcome, SimError> {
        validate_state(initial_state)?;
        let steps = self.step_count(duration_s)?;
        self.validate_impulses(impulses, duration_s)?;
        self.validate_parameter_changes(parameter_changes, duration_s)?;

        let mut plant = self.plant;
        let mut state = initial_state;
        let mut samples = Vec::with_capacity(steps + 1);
        let mut fell_at_s = None;
        let mut command_delay = VecDeque::with_capacity(self.config.command_delay_steps);
        let mut applied_torque_nm = 0.0;
        let torque_limit_nm = match self.config.max_total_current_a {
            Some(current_a) => self
                .config
                .max_total_torque_nm
                .min(current_a * self.config.motor_torque_constant_nm_per_a),
            None => self.config.max_total_torque_nm,
        };
        let rate_limit_nm_per_step = self
            .config
            .torque_rate_limit_nm_per_s
            .map(|rate| rate * self.config.dt_s);

        for step in 0..steps {
            let time_s = step as f64 * self.config.dt_s;

            for change in parameter_changes {
                if (change.at_s - time_s).abs() <= 1e-12 {
                    plant = WheeledInvertedPendulum::new(change.parameters)?;
                }
            }
            for impulse in impulses {
                if (impulse.at_s - time_s).abs() <= 1e-12 {
                    state[1] += impulse.force_ns / effective_cart_mass_kg(plant.parameters());
                }
            }
            validate_state(state)?;

            let control_state = if let Some(model) = sensors.as_deref_mut() {
                let sample = model.sample(time_s, state, applied_torque_nm)?;
                match estimator {
                    Some(estimator) => estimator.estimate(&sample),
                    None => state,
                }
            } else {
                state
            };
            validate_state(control_state)?;

            let requested = controller.torque_at(time_s, control_state);
            if !requested.is_finite() {
                return Err(SimError::NonFiniteControllerOutput { value: requested });
            }
            let delayed_request = if self.config.command_delay_steps == 0 {
                requested
            } else {
                command_delay.push_back(requested);
                if command_delay.len() > self.config.command_delay_steps {
                    command_delay.pop_front().unwrap_or(0.0)
                } else {
                    0.0
                }
            };
            let rate_limited = match rate_limit_nm_per_step {
                Some(max_delta) => {
                    applied_torque_nm
                        + (delayed_request - applied_torque_nm).clamp(-max_delta, max_delta)
                }
                None => delayed_request,
            };
            let shaped = apply_dead_zone(rate_limited, self.config.dead_zone_nm);
            let torque_nm = shaped.clamp(-torque_limit_nm, torque_limit_nm);
            let saturated = shaped < -torque_limit_nm || shaped > torque_limit_nm;
            applied_torque_nm = torque_nm;
            let friction_nm = friction_torque(state, plant.parameters(), &self.config);
            let plant_torque_nm = torque_nm - friction_nm;

            samples.push(TraceSample {
                time_s,
                state,
                torque_nm,
                saturated,
            });

            if state[2].abs() > self.config.fall_limit_rad {
                fell_at_s = Some(time_s);
                break;
            }

            state = rk4_step(state, plant_torque_nm, self.config.dt_s, |stage, input| {
                plant
                    .derivative_with_slope(stage, input, self.config.slope_rad)
                    .map_err(SimError::from)
            })?;
        }

        Ok(SimulationOutcome {
            trace: Trace {
                samples,
                dt_s: self.config.dt_s,
            },
            fell_at_s,
        })
    }

    fn step_count(&self, duration_s: f64) -> Result<usize, SimError> {
        if !duration_s.is_finite() || duration_s <= 0.0 {
            return Err(SimError::InvalidDuration { duration_s });
        }

        let ratio = duration_s / self.config.dt_s;
        let rounded = ratio.round();
        if (ratio - rounded).abs() > 1e-9 {
            return Err(SimError::DurationNotAligned {
                duration_s,
                dt_s: self.config.dt_s,
            });
        }
        if rounded < 1.0 || rounded > usize::MAX as f64 {
            return Err(SimError::InvalidDuration { duration_s });
        }

        Ok(rounded as usize)
    }

    fn validate_parameter_changes(
        &self,
        changes: &[ParameterChange],
        duration_s: f64,
    ) -> Result<(), SimError> {
        for change in changes {
            if !change.at_s.is_finite() || change.at_s < 0.0 || change.at_s >= duration_s {
                return Err(SimError::ParameterChangeOutsideRun {
                    at_s: change.at_s,
                    duration_s,
                });
            }
            let step_ratio = change.at_s / self.config.dt_s;
            if (step_ratio - step_ratio.round()).abs() > 1e-9 {
                return Err(SimError::ParameterChangeNotAligned {
                    at_s: change.at_s,
                    dt_s: self.config.dt_s,
                });
            }
            WheeledInvertedPendulum::new(change.parameters)?;
        }
        Ok(())
    }

    fn validate_impulses(&self, impulses: &[Impulse], duration_s: f64) -> Result<(), SimError> {
        for impulse in impulses {
            if !impulse.at_s.is_finite() || impulse.at_s < 0.0 || impulse.at_s >= duration_s {
                return Err(SimError::ImpulseOutsideRun {
                    at_s: impulse.at_s,
                    duration_s,
                });
            }
            if !impulse.force_ns.is_finite() {
                return Err(SimError::InvalidImpulse {
                    value: impulse.force_ns,
                });
            }
            let step_ratio = impulse.at_s / self.config.dt_s;
            if (step_ratio - step_ratio.round()).abs() > 1e-9 {
                return Err(SimError::ImpulseNotAligned {
                    at_s: impulse.at_s,
                    dt_s: self.config.dt_s,
                });
            }
        }
        Ok(())
    }
}

/// Errors from simulator configuration, inputs, or integration.
#[derive(Debug, Clone, PartialEq)]
pub enum SimError {
    InvalidConfig { field: &'static str, value: f64 },
    InvalidDuration { duration_s: f64 },
    DurationNotAligned { duration_s: f64, dt_s: f64 },
    ImpulseOutsideRun { at_s: f64, duration_s: f64 },
    ImpulseNotAligned { at_s: f64, dt_s: f64 },
    InvalidImpulse { value: f64 },
    ParameterChangeOutsideRun { at_s: f64, duration_s: f64 },
    ParameterChangeNotAligned { at_s: f64, dt_s: f64 },
    NonFiniteState { name: &'static str, value: f64 },
    NonFiniteControllerOutput { value: f64 },
    NonFiniteIntegration,
    Sensor(SensorError),
    Model(WipError),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig { field, value } => {
                write!(f, "invalid simulation config {field}={value}")
            }
            Self::InvalidDuration { duration_s } => {
                write!(f, "invalid simulation duration {duration_s}")
            }
            Self::DurationNotAligned { duration_s, dt_s } => write!(
                f,
                "duration {duration_s} s is not an integer multiple of dt {dt_s} s"
            ),
            Self::ImpulseOutsideRun { at_s, duration_s } => write!(
                f,
                "impulse time {at_s} s is outside the run [0, {duration_s})"
            ),
            Self::ImpulseNotAligned { at_s, dt_s } => {
                write!(f, "impulse time {at_s} s is not aligned to dt {dt_s} s")
            }
            Self::InvalidImpulse { value } => {
                write!(f, "invalid impulse magnitude {value}")
            }
            Self::ParameterChangeOutsideRun { at_s, duration_s } => write!(
                f,
                "parameter change time {at_s} s is outside the run [0, {duration_s})"
            ),
            Self::ParameterChangeNotAligned { at_s, dt_s } => write!(
                f,
                "parameter change time {at_s} s is not aligned to dt {dt_s} s"
            ),
            Self::NonFiniteState { name, value } => {
                write!(f, "non-finite simulation state {name}={value}")
            }
            Self::NonFiniteControllerOutput { value } => {
                write!(f, "non-finite controller output {value}")
            }
            Self::NonFiniteIntegration => {
                write!(f, "RK4 integration produced a non-finite state")
            }
            Self::Sensor(error) => error.fmt(f),
            Self::Model(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for SimError {}

impl From<WipError> for SimError {
    fn from(error: WipError) -> Self {
        Self::Model(error)
    }
}

impl From<SensorError> for SimError {
    fn from(error: SensorError) -> Self {
        Self::Sensor(error)
    }
}

fn apply_dead_zone(value: f64, dead_zone_nm: f64) -> f64 {
    if dead_zone_nm <= 0.0 {
        return value;
    }
    if value.abs() <= dead_zone_nm {
        0.0
    } else {
        value.signum() * (value.abs() - dead_zone_nm)
    }
}

fn friction_torque(
    state: StateVector,
    parameters: &WipParameters,
    config: &SimulationConfig,
) -> f64 {
    let wheel_omega_rad_s = state[1] / parameters.wheel_radius_m;
    let viscous_nm = config.viscous_friction_nm_per_rad_s * wheel_omega_rad_s;
    let coulomb_nm = if wheel_omega_rad_s.abs() > 1e-9 {
        config.coulomb_friction_nm * wheel_omega_rad_s.signum()
    } else {
        0.0
    };
    viscous_nm + coulomb_nm
}

fn effective_cart_mass_kg(parameters: &WipParameters) -> f64 {
    parameters.body_mass_kg
        + parameters.wheel_mass_kg
        + parameters.wheel_inertia_kg_m2 / (parameters.wheel_radius_m * parameters.wheel_radius_m)
}

fn validate_state(state: StateVector) -> Result<(), SimError> {
    const NAMES: [&str; 4] = ["x_m", "x_dot_mps", "pitch_rad", "pitch_rate_radps"];
    for (index, value) in state.iter().enumerate() {
        if !value.is_finite() {
            return Err(SimError::NonFiniteState {
                name: NAMES[index],
                value: *value,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use jbot_control::wip::STANDARD_GRAVITY_MPS2;
    use jbot_control::{solve_discrete_lqr, LqrWeights, WipParameters};

    use super::*;
    use crate::sensors::{EncoderImuEstimator, SensorConfig, SensorModel};

    fn baseline_plant() -> WheeledInvertedPendulum {
        WheeledInvertedPendulum::new(WipParameters {
            body_mass_kg: 6.0,
            wheel_mass_kg: 1.8,
            body_pitch_inertia_kg_m2: 0.35,
            wheel_inertia_kg_m2: 0.008,
            wheel_radius_m: 0.09,
            com_height_m: 0.25,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        })
        .expect("baseline WIP must be valid")
    }

    fn baseline_controller(plant: &WheeledInvertedPendulum) -> LqrController {
        let discrete = plant.discrete(0.001).expect("valid ZOH step");
        let gain = solve_discrete_lqr(&discrete, &LqrWeights::default()).expect("LQR converges");
        LqrController::new(gain)
    }

    fn baseline_simulator(max_total_torque_nm: f64) -> WipSimulator {
        let plant = baseline_plant();
        let config = SimulationConfig::new(0.001, max_total_torque_nm, 30.0_f64.to_radians())
            .expect("valid");
        WipSimulator::new(plant, config).expect("valid simulator")
    }

    #[test]
    fn rk4_matches_exponential_decay() {
        let dt_s = 0.001;
        let mut state = StateVector::new(1.0, 0.0, 0.0, 0.0);

        for _ in 0..1000 {
            state = rk4_step(state, 0.0, dt_s, |stage, _| {
                Ok(StateVector::new(-stage[0], 0.0, 0.0, 0.0))
            })
            .expect("RK4 step succeeds");
        }

        assert!((state[0] - (-1.0_f64).exp()).abs() < 1e-9);
    }

    #[test]
    fn lqr_recovers_from_ten_degree_tilt() {
        let simulator = baseline_simulator(16.0);
        let controller = baseline_controller(simulator.plant());
        let initial = StateVector::new(0.0, 0.0, 10.0_f64.to_radians(), 0.0);

        let outcome = simulator
            .run(&controller, initial, 8.0, &[])
            .expect("run succeeds");
        let final_state = outcome.trace.final_state().expect("trace is not empty");

        assert!(outcome.fell_at_s.is_none());
        assert!(final_state[2].abs() < 1e-3);
        assert!(outcome.trace.max_abs_torque_nm() <= 16.0 + 1e-12);
    }

    #[test]
    fn lqr_holds_on_a_ten_degree_slope() {
        let plant = baseline_plant();
        let config =
            SimulationConfig::with_slope(0.001, 16.0, 30.0_f64.to_radians(), 10.0_f64.to_radians())
                .expect("valid slope config");
        let simulator = WipSimulator::new(plant, config).expect("valid simulator");
        let controller = baseline_controller(simulator.plant());

        let outcome = simulator
            .run(&controller, StateVector::zeros(), 20.0, &[])
            .expect("run succeeds");
        let final_state = outcome.trace.final_state().expect("trace is not empty");

        assert!(outcome.fell_at_s.is_none());
        assert!(final_state[2] > 0.07);
        assert!(final_state[2] < 0.09);
        assert!(final_state[0].abs() < 0.5);
        assert!(outcome.trace.max_abs_torque_nm() <= 16.0 + 1e-12);
    }

    #[test]
    fn lqr_recovers_from_ten_newton_second_impulse() {
        let simulator = baseline_simulator(16.0);
        let controller = baseline_controller(simulator.plant());
        let impulse = Impulse {
            at_s: 0.1,
            force_ns: 10.0,
        };

        let outcome = simulator
            .run(&controller, StateVector::zeros(), 8.0, &[impulse])
            .expect("run succeeds");
        let final_state = outcome.trace.final_state().expect("trace is not empty");

        assert!(outcome.fell_at_s.is_none());
        assert!(outcome.trace.max_abs_pitch_rad() > 0.02);
        assert!(final_state[2].abs() < 1e-3);
        assert!(outcome.trace.max_abs_torque_nm() <= 16.0 + 1e-12);
    }

    #[test]
    fn lqr_recovers_after_mass_and_com_change() {
        let simulator = baseline_simulator(16.0);
        let controller = baseline_controller(simulator.plant());
        let mut changed = *simulator.plant().parameters();
        changed.body_mass_kg *= 1.2;
        changed.com_height_m *= 1.2;
        changed.body_pitch_inertia_kg_m2 *= 1.2 * 1.2 * 1.2;
        let changes = [ParameterChange {
            at_s: 1.0,
            parameters: changed,
        }];

        let outcome = simulator
            .run_with_parameter_changes(&controller, StateVector::zeros(), 8.0, &[], &changes)
            .expect("run succeeds");
        let final_state = outcome.trace.final_state().expect("trace is not empty");

        assert!(outcome.fell_at_s.is_none());
        assert!(final_state[2].abs() < 1e-3);
        assert!(outcome.trace.max_abs_torque_nm() <= 16.0 + 1e-12);
    }

    #[test]
    fn identical_inputs_produce_identical_traces() {
        let simulator = baseline_simulator(16.0);
        let controller = baseline_controller(simulator.plant());
        let initial = StateVector::new(0.0, 0.5, 0.05, 0.0);

        let first = simulator
            .run(&controller, initial, 2.0, &[])
            .expect("first run succeeds");
        let second = simulator
            .run(&controller, initial, 2.0, &[])
            .expect("second run succeeds");

        assert_eq!(first.trace, second.trace);
    }

    #[test]
    fn torque_is_saturated_and_bounded() {
        let simulator = baseline_simulator(1.0);
        let controller = baseline_controller(simulator.plant());
        let initial = StateVector::new(0.0, 0.0, 0.2, 0.0);

        let outcome = simulator
            .run(&controller, initial, 0.1, &[])
            .expect("run succeeds");

        assert!(outcome
            .trace
            .samples()
            .iter()
            .any(|sample| sample.saturated));
        assert!(outcome.trace.max_abs_torque_nm() <= 1.0 + 1e-12);
    }

    struct ConstantTorque(f64);

    impl StateFeedback for ConstantTorque {
        fn torque(&self, _state: StateVector) -> f64 {
            self.0
        }
    }

    fn noisy_sensors(seed: u64) -> SensorModel {
        let config = SensorConfig {
            imu_pitch_noise_rad: 0.005,
            imu_pitch_rate_noise_radps: 0.05,
            encoder_position_noise_rad: 0.002,
            encoder_velocity_noise_rad_s: 0.02,
            current_noise_a: 0.1,
            voltage_noise_v: 0.05,
            lidar_range_noise_m: 0.01,
            lidar_wall_distance_m: Some(5.0),
            ..SensorConfig::default()
        };
        SensorModel::new(config, 0.09, seed).expect("valid sensor model")
    }

    #[test]
    fn command_delay_holds_zero_during_warmup() {
        let plant = baseline_plant();
        let config = SimulationConfig::with_delay(0.001, 16.0, 30.0_f64.to_radians(), 0.0, 2)
            .expect("valid config");
        let simulator = WipSimulator::new(plant, config).expect("valid simulator");

        let outcome = simulator
            .run(&ConstantTorque(1.0), StateVector::zeros(), 0.005, &[])
            .expect("run succeeds");
        let torques: Vec<f64> = outcome
            .trace
            .samples()
            .iter()
            .map(|sample| sample.torque_nm)
            .collect();

        assert_eq!(torques, vec![0.0, 0.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn lqr_is_stable_with_delay_noise_and_saturation() {
        let plant = baseline_plant();
        let config = SimulationConfig::with_delay(0.001, 5.0, 30.0_f64.to_radians(), 0.0, 10)
            .expect("valid config");
        let simulator = WipSimulator::new(plant, config).expect("valid simulator");
        let controller = baseline_controller(simulator.plant());
        let estimator = EncoderImuEstimator::new(0.09).expect("valid estimator");
        let mut sensors = noisy_sensors(2026);
        let impulse = Impulse {
            at_s: 1.0,
            force_ns: 10.0,
        };

        let outcome = simulator
            .run_with_sensors(
                &controller,
                &estimator,
                &mut sensors,
                StateVector::zeros(),
                20.0,
                &[impulse],
            )
            .expect("run succeeds");
        let final_state = outcome.trace.final_state().expect("trace is not empty");

        assert!(outcome.fell_at_s.is_none());
        assert!(outcome
            .trace
            .samples()
            .iter()
            .any(|sample| sample.saturated));
        assert!(
            final_state[2].abs() < 0.02,
            "final pitch {}",
            final_state[2]
        );
        assert!(outcome.trace.max_abs_pitch_rad() < 0.35);
        assert!(outcome.trace.max_abs_torque_nm() <= 5.0 + 1e-12);
    }

    #[test]
    fn lqr_holds_a_ten_degree_slope_for_ten_minutes() {
        let plant = baseline_plant();
        let config = SimulationConfig::with_delay(
            0.001,
            16.0,
            30.0_f64.to_radians(),
            10.0_f64.to_radians(),
            5,
        )
        .expect("valid delay config")
        .with_actuator_limits(None, 0.1, Some(2000.0), 0.02, 0.05, 0.01)
        .expect("valid actuator config");
        let simulator = WipSimulator::new(plant, config).expect("valid simulator");
        let controller = baseline_controller(simulator.plant());
        let estimator = EncoderImuEstimator::new(0.09).expect("valid estimator");
        let mut sensors = noisy_sensors(42);

        let outcome = simulator
            .run_with_sensors(
                &controller,
                &estimator,
                &mut sensors,
                StateVector::zeros(),
                600.0,
                &[],
            )
            .expect("run succeeds");
        let final_state = outcome.trace.final_state().expect("trace is not empty");

        assert!(outcome.fell_at_s.is_none());
        assert!(final_state[2] > 0.06 && final_state[2] < 0.1);
        assert!(final_state[0].abs() < 1.0);
        assert!(outcome.trace.max_abs_pitch_rad() < 0.2);
        assert!(outcome.trace.max_abs_torque_nm() <= 16.0 + 1e-12);
    }

    #[test]
    fn velocity_step_tracks_without_sustained_overshoot() {
        let config = SimulationConfig::new(0.001, 16.0, 30.0_f64.to_radians())
            .expect("valid")
            .with_actuator_limits(Some(100.0), 0.1, None, 0.0, 0.0, 0.0)
            .expect("valid actuator limits");
        let simulator = WipSimulator::new(baseline_plant(), config).expect("valid simulator");
        let inner = baseline_controller(simulator.plant());
        let controller = VelocityController::new(
            inner,
            0.5,
            jbot_control::VelocityGains::default(),
            16.0,
            0.001,
        )
        .expect("valid velocity controller");

        let outcome = simulator
            .run(&controller, StateVector::zeros(), 10.0, &[])
            .expect("run succeeds");
        let final_state = outcome.trace.final_state().expect("trace is not empty");
        let peak_velocity = outcome
            .trace
            .samples()
            .iter()
            .fold(0.0_f64, |peak, sample| peak.max(sample.state[1]));
        let settling_time_s = outcome
            .trace
            .samples()
            .iter()
            .rev()
            .find(|sample| (sample.state[1] - 0.5).abs() > 0.025)
            .map_or(0.0, |sample| sample.time_s + outcome.trace.dt_s());

        assert!(outcome.fell_at_s.is_none());
        assert!(
            (final_state[1] - 0.5).abs() < 0.02,
            "final velocity {}",
            final_state[1]
        );
        assert!(peak_velocity < 0.6, "peak velocity {peak_velocity}");
        assert!(settling_time_s < 4.0, "settling time {settling_time_s}");
        assert!(outcome.trace.max_abs_torque_nm() <= 10.0 + 1e-12);
    }

    #[test]
    fn sensor_path_is_deterministic_for_a_seed() {
        let simulator = baseline_simulator(16.0);
        let controller = baseline_controller(simulator.plant());
        let estimator = EncoderImuEstimator::new(0.09).expect("valid estimator");

        let first = simulator
            .run_with_sensors(
                &controller,
                &estimator,
                &mut noisy_sensors(7),
                StateVector::new(0.0, 0.2, 0.02, 0.0),
                3.0,
                &[],
            )
            .expect("first run succeeds");
        let second = simulator
            .run_with_sensors(
                &controller,
                &estimator,
                &mut noisy_sensors(7),
                StateVector::new(0.0, 0.2, 0.02, 0.0),
                3.0,
                &[],
            )
            .expect("second run succeeds");

        assert_eq!(first.trace, second.trace);
    }

    #[test]
    fn dead_zone_and_rate_limit_shape_the_command() {
        let plant = baseline_plant();
        let config = SimulationConfig::with_delay(0.001, 16.0, 30.0_f64.to_radians(), 0.0, 0)
            .expect("valid config")
            .with_actuator_limits(None, 0.1, Some(1000.0), 0.5, 0.0, 0.0)
            .expect("valid actuator config");
        let simulator = WipSimulator::new(plant, config).expect("valid simulator");

        let outcome = simulator
            .run(&ConstantTorque(10.0), StateVector::zeros(), 0.005, &[])
            .expect("run succeeds");
        let torques: Vec<f64> = outcome
            .trace
            .samples()
            .iter()
            .map(|sample| sample.torque_nm)
            .collect();

        for (actual, expected) in torques.iter().zip([0.5, 1.0, 1.5, 2.0, 2.5]) {
            assert!((actual - expected).abs() < 1e-12, "torque {actual}");
        }
    }

    #[test]
    fn current_limit_caps_the_command() {
        let plant = baseline_plant();
        let config = SimulationConfig::with_delay(0.001, 16.0, 30.0_f64.to_radians(), 0.0, 0)
            .expect("valid config")
            .with_actuator_limits(Some(10.0), 0.1, None, 0.0, 0.0, 0.0)
            .expect("valid actuator config");
        let simulator = WipSimulator::new(plant, config).expect("valid simulator");

        let outcome = simulator
            .run(&ConstantTorque(10.0), StateVector::zeros(), 0.003, &[])
            .expect("run succeeds");

        assert!(outcome
            .trace
            .samples()
            .iter()
            .all(|sample| sample.saturated && (sample.torque_nm - 1.0).abs() < 1e-12));
    }

    #[test]
    fn friction_opposes_wheel_velocity() {
        let plant = baseline_plant();
        let frictionless = WipSimulator::new(
            plant,
            SimulationConfig::new(0.001, 16.0, 30.0_f64.to_radians()).expect("valid config"),
        )
        .expect("valid simulator");
        let with_friction = WipSimulator::new(
            plant,
            SimulationConfig::new(0.001, 16.0, 30.0_f64.to_radians())
                .expect("valid config")
                .with_actuator_limits(None, 0.1, None, 0.0, 0.5, 0.2)
                .expect("valid actuator config"),
        )
        .expect("valid simulator");
        let initial = StateVector::new(0.0, 1.0, 0.0, 0.0);

        let free = frictionless
            .run(&ConstantTorque(0.0), initial, 0.5, &[])
            .expect("run succeeds");
        let damped = with_friction
            .run(&ConstantTorque(0.0), initial, 0.5, &[])
            .expect("run succeeds");

        let free_velocity = free.trace.final_state().expect("trace")[1];
        let damped_velocity = damped.trace.final_state().expect("trace")[1];
        assert!(
            damped_velocity < free_velocity - 0.05,
            "free {free_velocity}, damped {damped_velocity}"
        );
    }

    #[test]
    fn rejects_invalid_actuator_config() {
        let config =
            SimulationConfig::new(0.001, 16.0, 30.0_f64.to_radians()).expect("valid config");
        assert!(matches!(
            config.with_actuator_limits(Some(0.0), 0.1, None, 0.0, 0.0, 0.0),
            Err(SimError::InvalidConfig {
                field: "max_total_current_a",
                ..
            })
        ));
        assert!(matches!(
            config.with_actuator_limits(None, 0.0, None, 0.0, 0.0, 0.0),
            Err(SimError::InvalidConfig {
                field: "motor_torque_constant_nm_per_a",
                ..
            })
        ));
        assert!(matches!(
            config.with_actuator_limits(None, 0.1, Some(-1.0), 0.0, 0.0, 0.0),
            Err(SimError::InvalidConfig {
                field: "torque_rate_limit_nm_per_s",
                ..
            })
        ));
        assert!(matches!(
            config.with_actuator_limits(None, 0.1, None, -0.1, 0.0, 0.0),
            Err(SimError::InvalidConfig {
                field: "dead_zone_nm",
                ..
            })
        ));
    }

    #[test]
    fn rejects_misaligned_or_out_of_range_inputs() {
        let simulator = baseline_simulator(16.0);
        let controller = baseline_controller(simulator.plant());

        assert!(matches!(
            simulator.run(&controller, StateVector::zeros(), 1.0005, &[]),
            Err(SimError::DurationNotAligned { .. })
        ));

        let impulse = Impulse {
            at_s: 0.1005,
            force_ns: 10.0,
        };
        assert!(matches!(
            simulator.run(&controller, StateVector::zeros(), 1.0, &[impulse]),
            Err(SimError::ImpulseNotAligned { .. })
        ));

        let impulse = Impulse {
            at_s: 1.0,
            force_ns: 10.0,
        };
        assert!(matches!(
            simulator.run(&controller, StateVector::zeros(), 1.0, &[impulse]),
            Err(SimError::ImpulseOutsideRun { .. })
        ));

        let change = ParameterChange {
            at_s: 0.1005,
            parameters: *simulator.plant().parameters(),
        };
        assert!(matches!(
            simulator.run_with_parameter_changes(
                &controller,
                StateVector::zeros(),
                1.0,
                &[],
                &[change]
            ),
            Err(SimError::ParameterChangeNotAligned { .. })
        ));

        let change = ParameterChange {
            at_s: 1.0,
            parameters: *simulator.plant().parameters(),
        };
        assert!(matches!(
            simulator.run_with_parameter_changes(
                &controller,
                StateVector::zeros(),
                1.0,
                &[],
                &[change]
            ),
            Err(SimError::ParameterChangeOutsideRun { .. })
        ));

        assert!(matches!(
            SimulationConfig::new(0.001, 0.0, PI / 6.0),
            Err(SimError::InvalidConfig {
                field: "max_total_torque_nm",
                ..
            })
        ));
    }
}
