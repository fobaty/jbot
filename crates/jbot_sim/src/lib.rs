//! Deterministic fixed-step simulation for the reduced-order WIP plant.

pub mod record;
mod report;
pub mod rk4;
pub mod rng;
pub mod sensors;
pub mod wip_sim;

pub use record::{SimulationRecord, TraceFormatError, RECORD_FORMAT_VERSION};
pub use rk4::rk4_step;
pub use rng::Pcg32;
pub use sensors::{
    EncoderImuEstimator, ImuSample, PowerSample, SensorConfig, SensorError, SensorModel,
    SensorSample, StateEstimate, WheelSample,
};
pub use wip_sim::{
    Impulse, ParameterChange, SimError, SimulationConfig, SimulationOutcome, StateFeedback, Trace,
    TraceSample, WipSimulator,
};
