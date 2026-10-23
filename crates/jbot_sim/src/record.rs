//! Versioned simulation records and deterministic replay.

use std::fmt;
use std::io::{BufRead, Write};

use jbot_control::{StateVector, WheeledInvertedPendulum, WipParameters};

use crate::wip_sim::{
    Impulse, ParameterChange, SimError, SimulationConfig, SimulationOutcome, StateFeedback, Trace,
    TraceSample, WipSimulator,
};

/// Current on-disk record version.
pub const RECORD_FORMAT_VERSION: u32 = 5;

const HEADER: &str = "jbot-sim-record";
const SAMPLE_HEADER: &str = "time_s x_m x_dot_mps pitch_rad pitch_rate_radps torque_nm saturated";

/// Full scenario and trace captured in a versioned text format.
#[derive(Debug, Clone, PartialEq)]
pub struct SimulationRecord {
    pub parameters: WipParameters,
    pub config: SimulationConfig,
    pub initial_state: StateVector,
    pub duration_s: f64,
    pub impulses: Vec<Impulse>,
    pub parameter_changes: Vec<ParameterChange>,
    pub outcome: SimulationOutcome,
}

impl SimulationRecord {
    /// Capture a completed run from its simulator and inputs.
    pub fn from_run(
        simulator: &WipSimulator,
        initial_state: StateVector,
        duration_s: f64,
        impulses: &[Impulse],
        outcome: SimulationOutcome,
    ) -> Self {
        Self::from_run_with_changes(simulator, initial_state, duration_s, impulses, &[], outcome)
    }

    pub fn from_run_with_changes(
        simulator: &WipSimulator,
        initial_state: StateVector,
        duration_s: f64,
        impulses: &[Impulse],
        parameter_changes: &[ParameterChange],
        outcome: SimulationOutcome,
    ) -> Self {
        Self {
            parameters: *simulator.plant().parameters(),
            config: *simulator.config(),
            initial_state,
            duration_s,
            impulses: impulses.to_vec(),
            parameter_changes: parameter_changes.to_vec(),
            outcome,
        }
    }

    /// Re-run the scenario with the supplied controller and return the trace.
    pub fn replay<C: StateFeedback + ?Sized>(
        &self,
        controller: &C,
    ) -> Result<SimulationOutcome, SimError> {
        let plant = WheeledInvertedPendulum::new(self.parameters)?;
        let simulator = WipSimulator::new(plant, self.config)?;
        simulator.run_with_parameter_changes(
            controller,
            self.initial_state,
            self.duration_s,
            &self.impulses,
            &self.parameter_changes,
        )
    }

    /// Serialize the record to the versioned text format.
    pub fn write<W: Write>(&self, mut writer: W) -> Result<(), TraceFormatError> {
        writeln!(writer, "{HEADER} {RECORD_FORMAT_VERSION}")?;
        writeln!(writer, "dt_s {}", self.config.dt_s)?;
        writeln!(
            writer,
            "max_total_torque_nm {}",
            self.config.max_total_torque_nm
        )?;
        writeln!(writer, "fall_limit_rad {}", self.config.fall_limit_rad)?;
        writeln!(writer, "slope_rad {}", self.config.slope_rad)?;
        writeln!(
            writer,
            "command_delay_steps {}",
            self.config.command_delay_steps
        )?;
        match self.config.max_total_current_a {
            Some(current_a) => writeln!(writer, "max_total_current_a {current_a}")?,
            None => writeln!(writer, "max_total_current_a none")?,
        }
        writeln!(
            writer,
            "motor_torque_constant_nm_per_a {}",
            self.config.motor_torque_constant_nm_per_a
        )?;
        match self.config.torque_rate_limit_nm_per_s {
            Some(rate) => writeln!(writer, "torque_rate_limit_nm_per_s {rate}")?,
            None => writeln!(writer, "torque_rate_limit_nm_per_s none")?,
        }
        writeln!(writer, "dead_zone_nm {}", self.config.dead_zone_nm)?;
        writeln!(
            writer,
            "coulomb_friction_nm {}",
            self.config.coulomb_friction_nm
        )?;
        writeln!(
            writer,
            "viscous_friction_nm_per_rad_s {}",
            self.config.viscous_friction_nm_per_rad_s
        )?;
        writeln!(writer, "body_mass_kg {}", self.parameters.body_mass_kg)?;
        writeln!(writer, "wheel_mass_kg {}", self.parameters.wheel_mass_kg)?;
        writeln!(
            writer,
            "body_pitch_inertia_kg_m2 {}",
            self.parameters.body_pitch_inertia_kg_m2
        )?;
        writeln!(
            writer,
            "wheel_inertia_kg_m2 {}",
            self.parameters.wheel_inertia_kg_m2
        )?;
        writeln!(writer, "wheel_radius_m {}", self.parameters.wheel_radius_m)?;
        writeln!(writer, "com_height_m {}", self.parameters.com_height_m)?;
        writeln!(writer, "gravity_mps2 {}", self.parameters.gravity_mps2)?;
        writeln!(
            writer,
            "initial_state {} {} {} {}",
            self.initial_state[0],
            self.initial_state[1],
            self.initial_state[2],
            self.initial_state[3]
        )?;
        writeln!(writer, "duration_s {}", self.duration_s)?;
        writeln!(writer, "impulses {}", self.impulses.len())?;
        for impulse in &self.impulses {
            writeln!(writer, "impulse {} {}", impulse.at_s, impulse.force_ns)?;
        }
        writeln!(writer, "parameter_changes {}", self.parameter_changes.len())?;
        for change in &self.parameter_changes {
            writeln!(
                writer,
                "parameter_change {} {} {} {} {} {} {} {}",
                change.at_s,
                change.parameters.body_mass_kg,
                change.parameters.wheel_mass_kg,
                change.parameters.body_pitch_inertia_kg_m2,
                change.parameters.wheel_inertia_kg_m2,
                change.parameters.wheel_radius_m,
                change.parameters.com_height_m,
                change.parameters.gravity_mps2
            )?;
        }
        match self.outcome.fell_at_s {
            Some(time_s) => writeln!(writer, "fell_at_s {time_s}")?,
            None => writeln!(writer, "fell_at_s none")?,
        }
        writeln!(writer, "samples {}", self.outcome.trace.samples.len())?;
        writeln!(writer, "{SAMPLE_HEADER}")?;
        for sample in &self.outcome.trace.samples {
            writeln!(
                writer,
                "{} {} {} {} {} {} {}",
                sample.time_s,
                sample.state[0],
                sample.state[1],
                sample.state[2],
                sample.state[3],
                sample.torque_nm,
                u8::from(sample.saturated)
            )?;
        }
        Ok(())
    }

    /// Parse a record from the versioned text format.
    pub fn read<R: BufRead>(reader: R) -> Result<Self, TraceFormatError> {
        let mut lines = reader.lines().enumerate();

        let header = next_line(&mut lines, "record header")?;
        let mut header_parts = header.split_whitespace();
        let magic = required_token(&mut header_parts, "record header", 0)?;
        if magic != HEADER {
            return Err(TraceFormatError::Parse {
                line: 1,
                reason: "missing jbot-sim-record header".to_owned(),
            });
        }
        let version = parse_usize(
            required_token(&mut header_parts, "record header", 0)?,
            "format_version",
            1,
        )?;
        if version < 1 || version > RECORD_FORMAT_VERSION as usize {
            return Err(TraceFormatError::UnsupportedVersion {
                found: version,
                supported: RECORD_FORMAT_VERSION,
            });
        }
        ensure_end(&mut header_parts, "record header", 1)?;

        let mut dt_s = None;
        let mut max_total_torque_nm = None;
        let mut fall_limit_rad = None;
        let mut slope_rad = None;
        let mut command_delay_steps = None;
        let mut max_total_current_a = None;
        let mut motor_torque_constant_nm_per_a = None;
        let mut torque_rate_limit_nm_per_s = None;
        let mut dead_zone_nm = None;
        let mut coulomb_friction_nm = None;
        let mut viscous_friction_nm_per_rad_s = None;
        let mut body_mass_kg = None;
        let mut wheel_mass_kg = None;
        let mut body_pitch_inertia_kg_m2 = None;
        let mut wheel_inertia_kg_m2 = None;
        let mut wheel_radius_m = None;
        let mut com_height_m = None;
        let mut gravity_mps2 = None;
        let mut initial_state = None;
        let mut duration_s = None;
        let mut impulse_count = None;
        let mut impulses = Vec::new();
        let mut parameter_change_count = None;
        let mut parameter_changes = Vec::new();
        let mut fell_at_s = None;

        let sample_count = loop {
            let (line_number, line) = next_line_with_number(&mut lines, "record metadata")?;
            let trimmed = line.trim();
            if trimmed.is_empty() {
                continue;
            }

            let mut parts = trimmed.split_whitespace();
            let key = required_token(&mut parts, "metadata", line_number)?;
            match key {
                "dt_s" => dt_s = Some(parse_f64_token(&mut parts, "dt_s", line_number)?),
                "max_total_torque_nm" => {
                    max_total_torque_nm = Some(parse_f64_token(
                        &mut parts,
                        "max_total_torque_nm",
                        line_number,
                    )?)
                }
                "fall_limit_rad" => {
                    fall_limit_rad =
                        Some(parse_f64_token(&mut parts, "fall_limit_rad", line_number)?)
                }
                "slope_rad" => {
                    slope_rad = Some(parse_f64_token(&mut parts, "slope_rad", line_number)?)
                }
                "command_delay_steps" => {
                    let count = parse_usize(
                        required_token(&mut parts, "command_delay_steps", line_number)?,
                        "command_delay_steps",
                        line_number,
                    )?;
                    ensure_end(&mut parts, "command_delay_steps", line_number)?;
                    command_delay_steps = Some(count);
                }
                "max_total_current_a" => {
                    let token = required_token(&mut parts, "max_total_current_a", line_number)?;
                    ensure_end(&mut parts, "max_total_current_a", line_number)?;
                    max_total_current_a = Some(if token == "none" {
                        None
                    } else {
                        Some(parse_f64_token_value(
                            token,
                            "max_total_current_a",
                            line_number,
                        )?)
                    });
                }
                "motor_torque_constant_nm_per_a" => {
                    motor_torque_constant_nm_per_a = Some(parse_f64_token(
                        &mut parts,
                        "motor_torque_constant_nm_per_a",
                        line_number,
                    )?);
                }
                "torque_rate_limit_nm_per_s" => {
                    let token =
                        required_token(&mut parts, "torque_rate_limit_nm_per_s", line_number)?;
                    ensure_end(&mut parts, "torque_rate_limit_nm_per_s", line_number)?;
                    torque_rate_limit_nm_per_s = Some(if token == "none" {
                        None
                    } else {
                        Some(parse_f64_token_value(
                            token,
                            "torque_rate_limit_nm_per_s",
                            line_number,
                        )?)
                    });
                }
                "dead_zone_nm" => {
                    dead_zone_nm = Some(parse_f64_token(&mut parts, "dead_zone_nm", line_number)?);
                }
                "coulomb_friction_nm" => {
                    coulomb_friction_nm = Some(parse_f64_token(
                        &mut parts,
                        "coulomb_friction_nm",
                        line_number,
                    )?);
                }
                "viscous_friction_nm_per_rad_s" => {
                    viscous_friction_nm_per_rad_s = Some(parse_f64_token(
                        &mut parts,
                        "viscous_friction_nm_per_rad_s",
                        line_number,
                    )?);
                }
                "body_mass_kg" => {
                    body_mass_kg = Some(parse_f64_token(&mut parts, "body_mass_kg", line_number)?)
                }
                "wheel_mass_kg" => {
                    wheel_mass_kg = Some(parse_f64_token(&mut parts, "wheel_mass_kg", line_number)?)
                }
                "body_pitch_inertia_kg_m2" => {
                    body_pitch_inertia_kg_m2 = Some(parse_f64_token(
                        &mut parts,
                        "body_pitch_inertia_kg_m2",
                        line_number,
                    )?)
                }
                "wheel_inertia_kg_m2" => {
                    wheel_inertia_kg_m2 = Some(parse_f64_token(
                        &mut parts,
                        "wheel_inertia_kg_m2",
                        line_number,
                    )?)
                }
                "wheel_radius_m" => {
                    wheel_radius_m =
                        Some(parse_f64_token(&mut parts, "wheel_radius_m", line_number)?)
                }
                "com_height_m" => {
                    com_height_m = Some(parse_f64_token(&mut parts, "com_height_m", line_number)?)
                }
                "gravity_mps2" => {
                    gravity_mps2 = Some(parse_f64_token(&mut parts, "gravity_mps2", line_number)?)
                }
                "initial_state" => {
                    let mut state = StateVector::zeros();
                    for index in 0..4 {
                        state[index] = parse_f64(&mut parts, "initial_state", line_number)?;
                    }
                    ensure_end(&mut parts, "initial_state", line_number)?;
                    initial_state = Some(state);
                }
                "duration_s" => {
                    duration_s = Some(parse_f64_token(&mut parts, "duration_s", line_number)?)
                }
                "impulses" => {
                    let count = parse_usize(
                        required_token(&mut parts, "impulses", line_number)?,
                        "impulses",
                        line_number,
                    )?;
                    ensure_end(&mut parts, "impulses", line_number)?;
                    impulse_count = Some(count);
                }
                "impulse" => {
                    let at_s = parse_f64(&mut parts, "impulse", line_number)?;
                    let force_ns = parse_f64(&mut parts, "impulse", line_number)?;
                    ensure_end(&mut parts, "impulse", line_number)?;
                    impulses.push(Impulse { at_s, force_ns });
                }
                "parameter_changes" => {
                    let count = parse_usize(
                        required_token(&mut parts, "parameter_changes", line_number)?,
                        "parameter_changes",
                        line_number,
                    )?;
                    ensure_end(&mut parts, "parameter_changes", line_number)?;
                    parameter_change_count = Some(count);
                }
                "parameter_change" => {
                    let at_s = parse_f64(&mut parts, "parameter_change", line_number)?;
                    let parameters = WipParameters {
                        body_mass_kg: parse_f64(&mut parts, "parameter_change", line_number)?,
                        wheel_mass_kg: parse_f64(&mut parts, "parameter_change", line_number)?,
                        body_pitch_inertia_kg_m2: parse_f64(
                            &mut parts,
                            "parameter_change",
                            line_number,
                        )?,
                        wheel_inertia_kg_m2: parse_f64(
                            &mut parts,
                            "parameter_change",
                            line_number,
                        )?,
                        wheel_radius_m: parse_f64(&mut parts, "parameter_change", line_number)?,
                        com_height_m: parse_f64(&mut parts, "parameter_change", line_number)?,
                        gravity_mps2: parse_f64(&mut parts, "parameter_change", line_number)?,
                    };
                    ensure_end(&mut parts, "parameter_change", line_number)?;
                    parameter_changes.push(ParameterChange { at_s, parameters });
                }
                "fell_at_s" => {
                    let token = required_token(&mut parts, "fell_at_s", line_number)?;
                    ensure_end(&mut parts, "fell_at_s", line_number)?;
                    fell_at_s = if token == "none" {
                        Some(None)
                    } else {
                        Some(Some(parse_f64_token_value(
                            token,
                            "fell_at_s",
                            line_number,
                        )?))
                    };
                }
                "samples" => {
                    let count = parse_usize(
                        required_token(&mut parts, "samples", line_number)?,
                        "samples",
                        line_number,
                    )?;
                    ensure_end(&mut parts, "samples", line_number)?;
                    break count;
                }
                other => {
                    return Err(TraceFormatError::Parse {
                        line: line_number,
                        reason: format!("unknown metadata key {other}"),
                    });
                }
            }
        };

        let dt_s = required(dt_s, "dt_s")?;
        let max_total_torque_nm = required(max_total_torque_nm, "max_total_torque_nm")?;
        let fall_limit_rad = required(fall_limit_rad, "fall_limit_rad")?;
        let slope_rad = match slope_rad {
            Some(value) => value,
            None if version == 1 => 0.0,
            None => {
                return Err(TraceFormatError::InvalidScenario(
                    "missing slope_rad".to_owned(),
                ))
            }
        };
        let parameters = WipParameters {
            body_mass_kg: required(body_mass_kg, "body_mass_kg")?,
            wheel_mass_kg: required(wheel_mass_kg, "wheel_mass_kg")?,
            body_pitch_inertia_kg_m2: required(
                body_pitch_inertia_kg_m2,
                "body_pitch_inertia_kg_m2",
            )?,
            wheel_inertia_kg_m2: required(wheel_inertia_kg_m2, "wheel_inertia_kg_m2")?,
            wheel_radius_m: required(wheel_radius_m, "wheel_radius_m")?,
            com_height_m: required(com_height_m, "com_height_m")?,
            gravity_mps2: required(gravity_mps2, "gravity_mps2")?,
        };
        let command_delay_steps = match command_delay_steps {
            Some(value) => value,
            None if version < 4 => 0,
            None => {
                return Err(TraceFormatError::InvalidScenario(
                    "missing command_delay_steps".to_owned(),
                ))
            }
        };
        let max_total_current_a = match max_total_current_a {
            Some(value) => value,
            None if version < 5 => None,
            None => {
                return Err(TraceFormatError::InvalidScenario(
                    "missing max_total_current_a".to_owned(),
                ))
            }
        };
        let torque_rate_limit_nm_per_s = match torque_rate_limit_nm_per_s {
            Some(value) => value,
            None if version < 5 => None,
            None => {
                return Err(TraceFormatError::InvalidScenario(
                    "missing torque_rate_limit_nm_per_s".to_owned(),
                ))
            }
        };
        let default_if_old = |value: Option<f64>, field: &str| -> Result<f64, TraceFormatError> {
            match value {
                Some(value) => Ok(value),
                None if version < 5 => Ok(0.0),
                None => Err(TraceFormatError::InvalidScenario(format!(
                    "missing {field}"
                ))),
            }
        };
        let motor_torque_constant_nm_per_a = match motor_torque_constant_nm_per_a {
            Some(value) => value,
            None if version < 5 => 0.1,
            None => {
                return Err(TraceFormatError::InvalidScenario(
                    "missing motor_torque_constant_nm_per_a".to_owned(),
                ))
            }
        };
        let dead_zone_nm = default_if_old(dead_zone_nm, "dead_zone_nm")?;
        let coulomb_friction_nm = default_if_old(coulomb_friction_nm, "coulomb_friction_nm")?;
        let viscous_friction_nm_per_rad_s = default_if_old(
            viscous_friction_nm_per_rad_s,
            "viscous_friction_nm_per_rad_s",
        )?;
        let config = SimulationConfig::with_delay(
            dt_s,
            max_total_torque_nm,
            fall_limit_rad,
            slope_rad,
            command_delay_steps,
        )
        .and_then(|config| {
            config.with_actuator_limits(
                max_total_current_a,
                motor_torque_constant_nm_per_a,
                torque_rate_limit_nm_per_s,
                dead_zone_nm,
                coulomb_friction_nm,
                viscous_friction_nm_per_rad_s,
            )
        })
        .map_err(|error| TraceFormatError::InvalidScenario(error.to_string()))?;
        let initial_state = required(initial_state, "initial_state")?;
        let duration_s = required(duration_s, "duration_s")?;
        let impulse_count = required(impulse_count, "impulses")?;
        if impulse_count != impulses.len() {
            return Err(TraceFormatError::InvalidScenario(format!(
                "impulse count {impulse_count} does not match {} impulse lines",
                impulses.len()
            )));
        }
        if version >= 3 {
            let parameter_change_count = required(parameter_change_count, "parameter_changes")?;
            if parameter_change_count != parameter_changes.len() {
                return Err(TraceFormatError::InvalidScenario(format!(
                    "parameter change count {parameter_change_count} does not match {} lines",
                    parameter_changes.len()
                )));
            }
        } else {
            parameter_changes.clear();
        }
        let fell_at_s = required(fell_at_s, "fell_at_s")?;

        let (line_number, sample_header) = next_line_with_number(&mut lines, "sample header")?;
        if sample_header.trim() != SAMPLE_HEADER {
            return Err(TraceFormatError::Parse {
                line: line_number,
                reason: "invalid sample header".to_owned(),
            });
        }

        let mut samples = Vec::with_capacity(sample_count.min(1_000_000));
        for index in 0..sample_count {
            let (line_number, line) = next_line_with_number(&mut lines, "sample")?;
            let mut parts = line.split_whitespace();
            let time_s = parse_f64(&mut parts, "time_s", line_number)?;
            let mut state = StateVector::zeros();
            for component in 0..4 {
                state[component] = parse_f64(&mut parts, "state", line_number)?;
            }
            let torque_nm = parse_f64(&mut parts, "torque_nm", line_number)?;
            let saturated = match required_token(&mut parts, "saturated", line_number)? {
                "0" => false,
                "1" => true,
                _ => {
                    return Err(TraceFormatError::Parse {
                        line: line_number,
                        reason: "saturated must be 0 or 1".to_owned(),
                    })
                }
            };
            ensure_end(&mut parts, "sample", line_number)?;

            let expected_time = index as f64 * dt_s;
            if (time_s - expected_time).abs() > 1e-9 {
                return Err(TraceFormatError::InvalidScenario(format!(
                    "sample {index} time {time_s} does not match expected {expected_time}"
                )));
            }
            if state.iter().any(|value| !value.is_finite()) || !torque_nm.is_finite() {
                return Err(TraceFormatError::InvalidScenario(format!(
                    "sample {index} contains a non-finite value"
                )));
            }

            samples.push(TraceSample {
                time_s,
                state,
                torque_nm,
                saturated,
            });
        }

        for (line_number, line) in lines {
            let line = line?;
            if !line.trim().is_empty() {
                return Err(TraceFormatError::Parse {
                    line: line_number + 1,
                    reason: "trailing data after the last sample".to_owned(),
                });
            }
        }

        if samples.is_empty() {
            return Err(TraceFormatError::InvalidScenario(
                "trace must contain at least one sample".to_owned(),
            ));
        }

        Ok(Self {
            parameters,
            config,
            initial_state,
            duration_s,
            impulses,
            parameter_changes,
            outcome: SimulationOutcome {
                trace: Trace { samples, dt_s },
                fell_at_s,
            },
        })
    }
}

/// Errors from record serialization and parsing.
#[derive(Debug)]
pub enum TraceFormatError {
    Io(std::io::Error),
    Parse { line: usize, reason: String },
    UnsupportedVersion { found: usize, supported: u32 },
    InvalidScenario(String),
}

impl fmt::Display for TraceFormatError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => error.fmt(f),
            Self::Parse { line, reason } => write!(f, "trace parse error at line {line}: {reason}"),
            Self::UnsupportedVersion { found, supported } => {
                write!(
                    f,
                    "unsupported trace version {found}, supported {supported}"
                )
            }
            Self::InvalidScenario(reason) => write!(f, "invalid trace scenario: {reason}"),
        }
    }
}

impl std::error::Error for TraceFormatError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            _ => None,
        }
    }
}

impl From<std::io::Error> for TraceFormatError {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error)
    }
}

fn next_line<I>(lines: &mut I, context: &str) -> Result<String, TraceFormatError>
where
    I: Iterator<Item = (usize, std::io::Result<String>)>,
{
    let (_, line) = next_line_with_number(lines, context)?;
    Ok(line)
}

fn next_line_with_number<I>(
    lines: &mut I,
    context: &str,
) -> Result<(usize, String), TraceFormatError>
where
    I: Iterator<Item = (usize, std::io::Result<String>)>,
{
    match lines.next() {
        Some((index, Ok(line))) => Ok((index + 1, line)),
        Some((_, Err(error))) => Err(TraceFormatError::Io(error)),
        None => Err(TraceFormatError::Parse {
            line: 0,
            reason: format!("unexpected end of file while reading {context}"),
        }),
    }
}

fn required_token<'a>(
    parts: &mut impl Iterator<Item = &'a str>,
    context: &str,
    line: usize,
) -> Result<&'a str, TraceFormatError> {
    parts.next().ok_or_else(|| TraceFormatError::Parse {
        line,
        reason: format!("missing token in {context}"),
    })
}

fn parse_f64<'a>(
    parts: &mut impl Iterator<Item = &'a str>,
    field: &str,
    line: usize,
) -> Result<f64, TraceFormatError> {
    let token = required_token(parts, field, line)?;
    parse_f64_token_value(token, field, line)
}

fn parse_f64_token<'a>(
    parts: &mut impl Iterator<Item = &'a str>,
    field: &str,
    line: usize,
) -> Result<f64, TraceFormatError> {
    let value = parse_f64(parts, field, line)?;
    ensure_end(parts, field, line)?;
    Ok(value)
}

fn parse_f64_token_value(token: &str, field: &str, line: usize) -> Result<f64, TraceFormatError> {
    let value = token.parse::<f64>().map_err(|_| TraceFormatError::Parse {
        line,
        reason: format!("invalid {field} value {token}"),
    })?;
    if !value.is_finite() {
        return Err(TraceFormatError::Parse {
            line,
            reason: format!("non-finite {field} value {token}"),
        });
    }
    Ok(value)
}

fn parse_usize(token: &str, field: &str, line: usize) -> Result<usize, TraceFormatError> {
    token.parse::<usize>().map_err(|_| TraceFormatError::Parse {
        line,
        reason: format!("invalid {field} count {token}"),
    })
}

fn ensure_end<'a>(
    parts: &mut impl Iterator<Item = &'a str>,
    context: &str,
    line: usize,
) -> Result<(), TraceFormatError> {
    if parts.next().is_some() {
        return Err(TraceFormatError::Parse {
            line,
            reason: format!("unexpected trailing token in {context}"),
        });
    }
    Ok(())
}

fn required<T>(value: Option<T>, field: &'static str) -> Result<T, TraceFormatError> {
    value.ok_or_else(|| TraceFormatError::InvalidScenario(format!("missing {field}")))
}

#[cfg(test)]
mod tests {
    use jbot_control::wip::STANDARD_GRAVITY_MPS2;
    use jbot_control::{solve_discrete_lqr, LqrController, LqrWeights};

    use super::*;

    fn baseline_record() -> (SimulationRecord, LqrController) {
        let parameters = WipParameters {
            body_mass_kg: 6.0,
            wheel_mass_kg: 1.8,
            body_pitch_inertia_kg_m2: 0.35,
            wheel_inertia_kg_m2: 0.008,
            wheel_radius_m: 0.09,
            com_height_m: 0.25,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        };
        let plant = WheeledInvertedPendulum::new(parameters).expect("baseline plant is valid");
        let config =
            SimulationConfig::with_slope(0.001, 16.0, 30.0_f64.to_radians(), 10.0_f64.to_radians())
                .expect("simulation config is valid");
        let simulator = WipSimulator::new(plant, config).expect("simulator is valid");
        let discrete = plant.discrete(0.001).expect("valid ZOH step");
        let gain = solve_discrete_lqr(&discrete, &LqrWeights::default()).expect("LQR converges");
        let controller = LqrController::new(gain);

        let initial_state = StateVector::new(0.0, 0.0, 10.0_f64.to_radians(), 0.0);
        let impulses = [Impulse {
            at_s: 0.1,
            force_ns: 10.0,
        }];
        let mut changed = *simulator.plant().parameters();
        changed.body_mass_kg *= 1.2;
        changed.com_height_m *= 1.2;
        changed.body_pitch_inertia_kg_m2 *= 1.2 * 1.2 * 1.2;
        let changes = [ParameterChange {
            at_s: 1.0,
            parameters: changed,
        }];
        let outcome = simulator
            .run_with_parameter_changes(&controller, initial_state, 2.0, &impulses, &changes)
            .expect("simulation succeeds");
        let record = SimulationRecord::from_run_with_changes(
            &simulator,
            initial_state,
            2.0,
            &impulses,
            &changes,
            outcome,
        );
        (record, controller)
    }

    #[test]
    fn record_round_trips_and_replays_identically() {
        let (record, controller) = baseline_record();
        let mut buffer = Vec::new();
        record.write(&mut buffer).expect("record writes");
        let text = String::from_utf8(buffer).expect("record is UTF-8");
        assert!(text.starts_with("jbot-sim-record 5\n"));

        let loaded =
            SimulationRecord::read(std::io::Cursor::new(text.as_bytes())).expect("record parses");
        assert_eq!(loaded, record);
        assert_eq!(
            loaded.replay(&controller).expect("replay succeeds"),
            record.outcome
        );
    }

    #[test]
    fn reads_version_one_records_with_zero_slope() {
        let (record, _) = baseline_record();
        let mut buffer = Vec::new();
        record.write(&mut buffer).expect("record writes");
        let text = String::from_utf8(buffer).expect("record is UTF-8");
        let text = text
            .replacen("jbot-sim-record 5", "jbot-sim-record 1", 1)
            .lines()
            .filter(|line| {
                !line.starts_with("slope_rad ")
                    && !line.starts_with("command_delay_steps ")
                    && !line.starts_with("max_total_current_a ")
                    && !line.starts_with("motor_torque_constant_nm_per_a ")
                    && !line.starts_with("torque_rate_limit_nm_per_s ")
                    && !line.starts_with("dead_zone_nm ")
                    && !line.starts_with("coulomb_friction_nm ")
                    && !line.starts_with("viscous_friction_nm_per_rad_s ")
                    && !line.starts_with("parameter_changes ")
                    && !line.starts_with("parameter_change ")
            })
            .collect::<Vec<_>>()
            .join("\n")
            + "\n";

        let loaded =
            SimulationRecord::read(std::io::Cursor::new(text.as_bytes())).expect("v1 parses");
        assert_eq!(loaded.config.slope_rad, 0.0);
    }

    #[test]
    fn reads_version_three_records_with_zero_command_delay() {
        let (record, _) = baseline_record();
        let mut buffer = Vec::new();
        record.write(&mut buffer).expect("record writes");
        let text = String::from_utf8(buffer).expect("record is UTF-8");
        let text = text
            .replacen("jbot-sim-record 5", "jbot-sim-record 3", 1)
            .lines()
            .filter(|line| {
                !line.starts_with("command_delay_steps ")
                    && !line.starts_with("max_total_current_a ")
                    && !line.starts_with("motor_torque_constant_nm_per_a ")
                    && !line.starts_with("torque_rate_limit_nm_per_s ")
                    && !line.starts_with("dead_zone_nm ")
                    && !line.starts_with("coulomb_friction_nm ")
                    && !line.starts_with("viscous_friction_nm_per_rad_s ")
            })
            .collect::<Vec<_>>()
            .join("\n")
            + "\n";

        let loaded =
            SimulationRecord::read(std::io::Cursor::new(text.as_bytes())).expect("v3 parses");
        assert_eq!(loaded.config.command_delay_steps, 0);
    }

    #[test]
    fn record_round_trips_actuator_limits() {
        let parameters = WipParameters {
            body_mass_kg: 6.0,
            wheel_mass_kg: 1.8,
            body_pitch_inertia_kg_m2: 0.35,
            wheel_inertia_kg_m2: 0.008,
            wheel_radius_m: 0.09,
            com_height_m: 0.25,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        };
        let plant = WheeledInvertedPendulum::new(parameters).expect("valid plant");
        let config = SimulationConfig::with_delay(0.001, 16.0, 30.0_f64.to_radians(), 0.0, 3)
            .expect("valid config")
            .with_actuator_limits(Some(30.0), 0.11, Some(500.0), 0.05, 0.1, 0.02)
            .expect("valid actuator config");
        let simulator = WipSimulator::new(plant, config).expect("valid simulator");
        let discrete = plant.discrete(0.001).expect("valid ZOH step");
        let gain = solve_discrete_lqr(&discrete, &LqrWeights::default()).expect("LQR converges");
        let controller = LqrController::new(gain);
        let initial_state = StateVector::zeros();
        let outcome = simulator
            .run(&controller, initial_state, 0.1, &[])
            .expect("run succeeds");
        let record = SimulationRecord::from_run(&simulator, initial_state, 0.1, &[], outcome);

        let mut buffer = Vec::new();
        record.write(&mut buffer).expect("record writes");
        let loaded = SimulationRecord::read(std::io::Cursor::new(buffer)).expect("record parses");

        assert_eq!(loaded.config, config);
        assert_eq!(loaded, record);
    }

    #[test]
    fn reads_version_four_records_with_default_actuator_limits() {
        let (record, _) = baseline_record();
        let mut buffer = Vec::new();
        record.write(&mut buffer).expect("record writes");
        let text = String::from_utf8(buffer).expect("record is UTF-8");
        let text = text
            .replacen("jbot-sim-record 5", "jbot-sim-record 4", 1)
            .lines()
            .filter(|line| {
                !line.starts_with("max_total_current_a ")
                    && !line.starts_with("motor_torque_constant_nm_per_a ")
                    && !line.starts_with("torque_rate_limit_nm_per_s ")
                    && !line.starts_with("dead_zone_nm ")
                    && !line.starts_with("coulomb_friction_nm ")
                    && !line.starts_with("viscous_friction_nm_per_rad_s ")
            })
            .collect::<Vec<_>>()
            .join("\n")
            + "\n";

        let loaded =
            SimulationRecord::read(std::io::Cursor::new(text.as_bytes())).expect("v4 parses");
        assert_eq!(loaded.config.max_total_current_a, None);
        assert_eq!(loaded.config.motor_torque_constant_nm_per_a, 0.1);
        assert_eq!(loaded.config.torque_rate_limit_nm_per_s, None);
        assert_eq!(loaded.config.dead_zone_nm, 0.0);
    }

    #[test]
    fn rejects_unsupported_record_version() {
        let text = "jbot-sim-record 99\n";
        assert!(matches!(
            SimulationRecord::read(std::io::Cursor::new(text.as_bytes())),
            Err(TraceFormatError::UnsupportedVersion {
                found: 99,
                supported: RECORD_FORMAT_VERSION
            })
        ));
    }

    #[test]
    fn rejects_malformed_sample_time() {
        let (record, _) = baseline_record();
        let mut buffer = Vec::new();
        record.write(&mut buffer).expect("record writes");
        let text = String::from_utf8(buffer).expect("record is UTF-8");
        let mut lines: Vec<&str> = text.lines().collect();
        let header_index = lines
            .iter()
            .position(|line| *line == SAMPLE_HEADER)
            .expect("sample header is present");
        lines[header_index + 1] = "0.5 0 0 0 0 0 0";
        let broken = lines.join("\n") + "\n";

        assert!(matches!(
            SimulationRecord::read(std::io::Cursor::new(broken.as_bytes())),
            Err(TraceFormatError::InvalidScenario(_))
        ));
    }
}
