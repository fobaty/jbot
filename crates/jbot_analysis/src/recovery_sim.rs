//! Host-only nonlinear recovery simulation (WP1).
//!
//! This module answers one question before any hardware trial: can a
//! large-angle recovery succeed at the torque the drive actually has? It is
//! deliberately not part of the firmware crate and is never called from the
//! 200 Hz path. Its parameters and control law mirror `config/robot.toml`,
//! `jbot_pico_core::plant::PlantParams::NOMINAL` and
//! `jbot_pico_core::control::{Tuning, BalanceController}`; the derivation is
//! copied here rather than imported so the two Cargo workspaces stay separate.
//! A unit test pins the derived coefficients to the documented values, so a
//! silent drift is caught.
//!
//! The dynamics are the full planar two-body pendulum on coaxial wheels with a
//! no-slip constraint, on level ground:
//!
//! ```text
//! a*x'' + b*cos(theta)*theta'' - b*sin(theta)*theta'^2 = tau / r
//! b*cos(theta)*x'' + c*theta'' - b*g*sin(theta)        = -tau
//! ```
//!
//! with `a = m_body + m_wheel + I_wheel/r^2`, `b = m_body*L`,
//! `c = I_body + m_body*L^2`. Linearising at theta=0 reproduces the
//! coefficients in `jbot_pico_core::plant` exactly; a unit test checks that.
//!
//! Limitations, stated explicitly so the sweep is not over-read:
//! * level ground only - no slope term;
//! * body-to-ground contact is not modelled, so angles much past 80 degrees
//!   are optimistic;
//! * the drive is a dead zone, a transport delay and a linear torque-speed
//!   envelope - no current loop, back-EMF dynamics or thermal model;
//! * battery sag is represented by a fixed scale, not an impedance model.

/// Firmware control period, seconds (200 Hz).
pub const CONTROL_DT_S: f32 = 0.005;

/// Guards from `jbot_pico_core::control`.
pub const MAX_TRAVEL_M: f32 = 3.0;
pub const MAX_SPEED_MPS: f32 = 3.0;

const G: f32 = 9.81;

/// Geometry and drive limits, combined-wheel values from `config/robot.toml`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Geometry {
    pub body_mass_kg: f32,
    pub wheel_mass_kg: f32,
    pub body_inertia_kgm2: f32,
    pub wheel_inertia_kgm2: f32,
    pub com_height_m: f32,
    pub wheel_radius_m: f32,
    pub max_torque_nm: f32,
    pub deadband_nm: f32,
}

impl Default for Geometry {
    fn default() -> Self {
        Self {
            body_mass_kg: 5.0,
            wheel_mass_kg: 4.6,
            body_inertia_kgm2: 0.30,
            wheel_inertia_kgm2: 0.0156,
            com_height_m: 0.25,
            wheel_radius_m: 0.0825,
            max_torque_nm: 5.0,
            deadband_nm: 0.5,
        }
    }
}

impl Geometry {
    fn shape(&self) -> (f32, f32, f32, f32) {
        let r = self.wheel_radius_m;
        let a = self.body_mass_kg + self.wheel_mass_kg + self.wheel_inertia_kgm2 / (r * r);
        let b = self.body_mass_kg * self.com_height_m;
        let c = self.body_inertia_kgm2 + self.body_mass_kg * self.com_height_m * self.com_height_m;
        (a, b, c, a * c - b * b)
    }

    pub fn a1(&self) -> f32 {
        let (a, _, _, d) = self.shape();
        a * self.body_mass_kg * G * self.com_height_m / d
    }

    pub fn a2(&self) -> f32 {
        let (a, b, _, d) = self.shape();
        (a + b / self.wheel_radius_m) / d
    }

    pub fn b1(&self) -> f32 {
        let (_, b, c, d) = self.shape();
        (c / self.wheel_radius_m + b) / d
    }

    pub fn b2(&self) -> f32 {
        let (_, b, _, d) = self.shape();
        b * self.body_mass_kg * G * self.com_height_m / d
    }

    /// Stability floor `A1/A2`, Nm/rad.
    pub fn floor_nm_per_rad(&self) -> f32 {
        self.a1() / self.a2()
    }
}

/// Drive and geometry parameters beyond the firmware's `PlantParams`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimParams {
    pub plant: Geometry,
    /// No-load wheel speed at full duty, rad/s (`config/robot.toml`).
    pub wheel_speed_limit_rps: f32,
    /// Transport delay in control ticks.
    pub delay_ticks: usize,
    /// Apply a linear torque-speed envelope when true.
    pub torque_speed_envelope: bool,
}

impl SimParams {
    pub fn nominal() -> Self {
        Self {
            plant: Geometry::default(),
            wheel_speed_limit_rps: 40.0,
            delay_ticks: 2,
            torque_speed_envelope: true,
        }
    }

    /// Override the common-mode budget while keeping the geometry fixed.
    pub fn with_budget(mut self, budget_nm: f32) -> Self {
        self.plant.max_torque_nm = budget_nm.max(0.0);
        self
    }

    /// Linear accelerations from `jbot_pico_core::plant`, level ground.
    pub fn linear_accel(&self, pitch_rad: f32, torque_nm: f32) -> (f32, f32) {
        let p = self.plant;
        (
            p.a1() * pitch_rad - p.a2() * torque_nm,
            p.b1() * torque_nm - p.b2() * pitch_rad,
        )
    }

    /// Full nonlinear accelerations, level ground.
    pub fn nonlinear_accel(
        &self,
        pitch_rad: f32,
        pitch_rate_rad_s: f32,
        torque_nm: f32,
    ) -> (f32, f32) {
        let r = self.plant.wheel_radius_m;
        let (a, b, c, _) = self.plant.shape();
        let cos = pitch_rad.cos();
        let sin = pitch_rad.sin();
        let det = a * c - b * b * cos * cos;

        let supply = torque_nm / r + b * sin * pitch_rate_rad_s * pitch_rate_rad_s;
        let x_dd = (c * supply + b * cos * torque_nm - b * b * cos * sin * G) / det;
        let pitch_dd = (-b * cos * supply - a * torque_nm + a * b * G * sin) / det;
        (pitch_dd, x_dd)
    }
}

/// Gains derived exactly as `Tuning::from_plant` does, in Nm units.
#[derive(Clone, Copy, Debug)]
struct SimTuning {
    kp: f32,
    kd: f32,
    ki: f32,
    integral_limit_nm: f32,
    integral_window_rad: f32,
    position_gain: f32,
    velocity_gain: f32,
    trim_gain: f32,
    accel_per_rad: f32,
    speed_limit: f32,
    speed_slew: f32,
    accel_limit: f32,
    demand_limit_rad: f32,
    trim_limit_rad: f32,
    torque_limit_nm: f32,
}

impl SimTuning {
    fn from_geometry(p: &Geometry) -> Self {
        let floor = p.floor_nm_per_rad();
        let margin = 4.0_f32;
        let kp = margin * floor;
        let stiffness = (p.a2() * kp - p.a1()).max(1e-3);
        let kd = 2.0 * 1.6 * stiffness.sqrt() / p.a2();
        let accel_per_rad = ((p.b1() * floor - p.b2()) * margin / (margin - 1.0)).max(0.1);
        Self {
            kp,
            kd,
            ki: kp / 20.0,
            integral_limit_nm: 1.0,
            integral_window_rad: 3.0_f32.to_radians(),
            position_gain: 0.4,
            velocity_gain: 1.8,
            trim_gain: 0.01,
            accel_per_rad,
            speed_limit: 1.0,
            speed_slew: 0.5,
            accel_limit: 1.0,
            demand_limit_rad: 8.0_f32.to_radians(),
            trim_limit_rad: 20.0_f32.to_radians(),
            torque_limit_nm: p.max_torque_nm,
        }
    }
}

fn clamp(value: f32, limit: f32) -> f32 {
    value.clamp(-limit, limit)
}

/// One control sample, matching the fields of `BalanceInput` used here.
#[derive(Clone, Copy, Debug)]
struct BalanceSample {
    pitch_rad: f32,
    pitch_rate_rad_s: f32,
    reference_rad: f32,
    position_m: f32,
    speed_mps: f32,
}

/// Local mirror of `BalanceController`, returning Nm directly.
#[derive(Clone, Copy, Debug)]
struct SimBalance {
    origin_m: f32,
    speed_ref_mps: f32,
    demand_rad: f32,
    trim_rad: f32,
    pitch_integral_rad_s: f32,
    anchored: bool,
}

impl SimBalance {
    fn new() -> Self {
        Self {
            origin_m: 0.0,
            speed_ref_mps: 0.0,
            demand_rad: 0.0,
            trim_rad: 0.0,
            pitch_integral_rad_s: 0.0,
            anchored: false,
        }
    }

    fn reset(&mut self, position_m: f32) {
        self.origin_m = position_m;
        self.speed_ref_mps = 0.0;
        self.demand_rad = 0.0;
        self.trim_rad = 0.0;
        self.pitch_integral_rad_s = 0.0;
        self.anchored = position_m.is_finite();
    }

    fn update(&mut self, sample: &BalanceSample, tuning: &SimTuning, dt: f32) -> f32 {
        let BalanceSample {
            pitch_rad,
            pitch_rate_rad_s,
            reference_rad,
            position_m,
            speed_mps,
        } = *sample;
        if !pitch_rad.is_finite()
            || !pitch_rate_rad_s.is_finite()
            || !reference_rad.is_finite()
            || !position_m.is_finite()
            || !speed_mps.is_finite()
            || dt <= 0.0
        {
            self.reset(0.0);
            return 0.0;
        }
        if !self.anchored {
            self.reset(position_m);
        }

        let limit = tuning.torque_limit_nm.max(0.0);
        let target = clamp(0.0, tuning.speed_limit);
        let step = clamp(target - self.speed_ref_mps, tuning.speed_slew * dt);
        self.speed_ref_mps += step;
        self.origin_m += self.speed_ref_mps * dt;

        let offset = position_m - self.origin_m;
        let speed_ref = clamp(
            self.speed_ref_mps - tuning.position_gain * offset,
            tuning.speed_limit,
        );
        let accel_ref = clamp(
            tuning.velocity_gain * (speed_ref - speed_mps),
            tuning.accel_limit,
        );
        let dynamic = clamp(accel_ref / tuning.accel_per_rad, tuning.demand_limit_rad);

        let increment = -tuning.trim_gain * offset * dt;
        let total = dynamic + self.trim_rad;
        let pushing_outwards = (total >= tuning.trim_limit_rad && increment > 0.0)
            || (total <= -tuning.trim_limit_rad && increment < 0.0);
        if !pushing_outwards {
            self.trim_rad = clamp(self.trim_rad + increment, tuning.trim_limit_rad);
        }
        self.demand_rad = clamp(dynamic + self.trim_rad, tuning.trim_limit_rad);

        let error = pitch_rad - reference_rad - self.demand_rad;
        let pd = tuning.kp * error + tuning.kd * pitch_rate_rad_s;
        let integral = tuning.ki * self.pitch_integral_rad_s;
        if error.abs() < tuning.integral_window_rad && (pd + integral).abs() < limit * 0.95 {
            let state_limit = tuning.integral_limit_nm / tuning.ki;
            self.pitch_integral_rad_s =
                (self.pitch_integral_rad_s + error * dt).clamp(-state_limit, state_limit);
        }
        clamp(pd + tuning.ki * self.pitch_integral_rad_s, limit)
    }
}

/// Machine state of the nonlinear simulation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SimState {
    pub position_m: f32,
    pub speed_mps: f32,
    pub pitch_rad: f32,
    pub pitch_rate_rad_s: f32,
    pub applied_torque_nm: f32,
}

/// Nonlinear plant with the same drive defects as the firmware model.
#[derive(Clone, Debug)]
pub struct NonlinearPlant {
    params: SimParams,
    state: SimState,
    delay: [f32; 4],
    delay_index: usize,
}

impl NonlinearPlant {
    pub fn new(params: SimParams, start_pitch_rad: f32) -> Self {
        Self {
            params,
            state: SimState {
                pitch_rad: start_pitch_rad,
                ..SimState::default()
            },
            delay: [0.0; 4],
            delay_index: 0,
        }
    }

    pub fn state(&self) -> SimState {
        self.state
    }

    /// Wheel angular speed from the no-slip constraint, rad/s.
    pub fn wheel_speed_rps(&self) -> f32 {
        self.state.speed_mps / self.params.plant.wheel_radius_m
    }

    /// Advance one control tick with a commanded common-mode torque in Nm.
    pub fn step(&mut self, command_nm: f32, dt_s: f32) -> SimState {
        if !dt_s.is_finite() || dt_s <= 0.0 || !command_nm.is_finite() {
            return self.state;
        }

        let p = &self.params;
        let budget = p.plant.max_torque_nm.max(0.0);
        let deadband = p.plant.deadband_nm.max(0.0);

        let envelope = if p.torque_speed_envelope && p.wheel_speed_limit_rps > 0.0 {
            (1.0 - (self.wheel_speed_rps().abs() / p.wheel_speed_limit_rps)).max(0.0)
        } else {
            1.0
        };
        let magnitude = command_nm.abs().min(budget) * envelope;
        let mut torque = if magnitude <= deadband {
            0.0
        } else {
            (magnitude - deadband).copysign(command_nm)
        };
        torque = torque.clamp(-budget, budget);

        if p.delay_ticks > 0 {
            let index = self.delay_index.min(self.delay.len() - 1);
            let delayed = self.delay[index];
            self.delay[index] = torque;
            self.delay_index = (index + 1) % p.delay_ticks.min(self.delay.len());
            torque = delayed;
        }

        let (pitch_dd, x_dd) =
            p.nonlinear_accel(self.state.pitch_rad, self.state.pitch_rate_rad_s, torque);

        let mut speed = self.state.speed_mps + x_dd * dt_s;
        let mut position = self.state.position_m + speed * dt_s;
        let mut pitch_rate = self.state.pitch_rate_rad_s + pitch_dd * dt_s;
        let mut pitch = self.state.pitch_rad + pitch_rate * dt_s;
        if !speed.is_finite()
            || !position.is_finite()
            || !pitch_rate.is_finite()
            || !pitch.is_finite()
        {
            speed = 0.0;
            position = 0.0;
            pitch_rate = 0.0;
            pitch = 0.0;
        }

        self.state = SimState {
            position_m: position,
            speed_mps: speed,
            pitch_rad: pitch,
            pitch_rate_rad_s: pitch_rate,
            applied_torque_nm: torque,
        };
        self.state
    }
}

/// Result of one simulated recovery attempt.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Outcome {
    pub success: bool,
    pub settled: bool,
    pub final_pitch_deg: f32,
    pub final_speed_mps: f32,
    pub travel_m: f32,
    pub peak_speed_mps: f32,
    pub peak_torque_nm: f32,
    pub duration_s: f32,
    pub guard_hit: bool,
}

/// Which recovery pattern the sweep runs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Strategy {
    /// Linear pitch-reference ramp from the measured angle to zero.
    StandUp { rate_dps: f32 },
    /// Two-phase momentum throw followed by the normal balance loop.
    Throw {
        accel_speed_rps: f32,
        reverse_speed_rps: f32,
        phase_limit_s: f32,
    },
}

impl Strategy {
    pub fn label(self) -> &'static str {
        match self {
            Strategy::StandUp { .. } => "standup",
            Strategy::Throw { .. } => "throw",
        }
    }
}

/// Linear pitch reference of the `StandUp` strategy, radians.
pub fn stand_up_reference(start_rad: f32, rate_dps: f32, elapsed_s: f32) -> f32 {
    let remaining = start_rad.abs() - rate_dps.to_radians() * elapsed_s.max(0.0);
    start_rad.signum() * remaining.max(0.0)
}

/// Run one recovery attempt. `settle_s` is how long the machine must stay
/// near upright for the run to count as settled.
pub fn simulate(
    params: SimParams,
    strategy: Strategy,
    start_pitch_deg: f32,
    settle_s: f32,
) -> Outcome {
    let dt = CONTROL_DT_S;
    let limit_s = 20.0_f32;
    let start = start_pitch_deg.to_radians();
    let mut plant = NonlinearPlant::new(params, start);
    let tuning = SimTuning::from_geometry(&params.plant);
    let mut balance = SimBalance::new();
    balance.reset(0.0);

    let mut outcome = Outcome::default();
    let mut settle_for = 0.0_f32;
    let mut phase = 0_u8; // 0 = prepare, 1 = accelerate, 2 = reverse, 3 = catch
    let mut phase_time = 0.0_f32;
    let throw_sign = if start >= 0.0 { 1.0 } else { -1.0 };

    let steps = (limit_s / dt) as usize;
    for step in 0..steps {
        let state = plant.state();
        let elapsed = step as f32 * dt;
        let mut torque_nm = 0.0_f32;

        match strategy {
            Strategy::StandUp { rate_dps } => {
                let reference = stand_up_reference(start, rate_dps, elapsed);
                torque_nm = balance.update(
                    &BalanceSample {
                        pitch_rad: state.pitch_rad,
                        pitch_rate_rad_s: state.pitch_rate_rad_s,
                        reference_rad: reference,
                        position_m: state.position_m,
                        speed_mps: state.speed_mps,
                    },
                    &tuning,
                    dt,
                );
            }
            Strategy::Throw {
                accel_speed_rps,
                reverse_speed_rps,
                phase_limit_s,
            } => {
                phase_time += dt;
                let wheel_speed = plant.wheel_speed_rps();
                match phase {
                    0 => {
                        phase = 1;
                        phase_time = 0.0;
                    }
                    1 => {
                        torque_nm = throw_sign * params.plant.max_torque_nm;
                        if wheel_speed.abs() >= accel_speed_rps || phase_time >= phase_limit_s {
                            phase = 2;
                            phase_time = 0.0;
                        }
                    }
                    2 => {
                        torque_nm = -throw_sign * params.plant.max_torque_nm;
                        if wheel_speed * throw_sign <= -reverse_speed_rps
                            || phase_time >= phase_limit_s
                        {
                            phase = 3;
                            phase_time = 0.0;
                            balance.reset(state.position_m);
                        }
                    }
                    _ => {
                        torque_nm = balance.update(
                            &BalanceSample {
                                pitch_rad: state.pitch_rad,
                                pitch_rate_rad_s: state.pitch_rate_rad_s,
                                reference_rad: 0.0,
                                position_m: state.position_m,
                                speed_mps: state.speed_mps,
                            },
                            &tuning,
                            dt,
                        );
                    }
                }
            }
        }

        let state = plant.step(torque_nm, dt);
        outcome.travel_m = outcome.travel_m.max(state.position_m.abs());
        outcome.peak_speed_mps = outcome.peak_speed_mps.max(state.speed_mps.abs());
        outcome.peak_torque_nm = outcome.peak_torque_nm.max(state.applied_torque_nm.abs());
        outcome.duration_s = elapsed + dt;

        if state.pitch_rad.abs() > 85.0_f32.to_radians()
            || state.position_m.abs() > MAX_TRAVEL_M
            || state.speed_mps.abs() > MAX_SPEED_MPS
        {
            outcome.guard_hit = true;
            break;
        }

        // Same "close enough" aperture as `ARM_ANCHOR_RAD` in the firmware.
        // A 0.5 Nm dead zone alone leaves about 1.3 deg of lean, and the
        // transport delay adds more, so a 3 deg test would reject a stable
        // stance for a reason that has nothing to do with recovery.
        let upright = state.pitch_rad.abs() < 5.0_f32.to_radians()
            && state.speed_mps.abs() < 0.1
            && state.pitch_rate_rad_s.abs() < 0.3;
        settle_for = if upright { settle_for + dt } else { 0.0 };
        if settle_for >= settle_s {
            outcome.settled = true;
            break;
        }
    }

    let final_state = plant.state();
    outcome.final_pitch_deg = final_state.pitch_rad.to_degrees();
    outcome.final_speed_mps = final_state.speed_mps;
    outcome.success = outcome.settled && !outcome.guard_hit;
    outcome
}

/// Sweep budgets and start angles; the caller chooses the strategies.
pub fn sweep(
    params: SimParams,
    strategies: &[Strategy],
    budgets_nm: &[f32],
    start_angles_deg: &[f32],
) -> Vec<(Strategy, f32, f32, Outcome)> {
    let mut rows = Vec::new();
    for &strategy in strategies {
        for &budget in budgets_nm {
            for &angle in start_angles_deg {
                let attempt = simulate(params.with_budget(budget), strategy, angle, 0.5);
                rows.push((strategy, budget, angle, attempt));
            }
        }
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derived_coefficients_match_the_documented_plant() {
        let p = Geometry::default();
        assert!((p.a1() - 25.49).abs() < 0.5, "A1 {}", p.a1());
        assert!((p.a2() - 4.73).abs() < 0.05, "A2 {}", p.a2());
        assert!((p.b1() - 1.52).abs() < 0.05, "B1 {}", p.b1());
        assert!((p.b2() - 2.68).abs() < 0.05, "B2 {}", p.b2());
        assert!((p.floor_nm_per_rad() - 5.39).abs() < 0.1);
        let tuning = SimTuning::from_geometry(&p);
        assert!((tuning.kp - 21.57).abs() < 0.3, "kp {}", tuning.kp);
        assert!((tuning.kd - 5.92).abs() < 0.3, "kd {}", tuning.kd);
        assert!((tuning.accel_per_rad - 7.33).abs() < 0.3);
    }

    #[test]
    fn nonlinear_matches_linear_near_upright() {
        let p = SimParams::nominal();
        for torque in [-2.0, 0.0, 2.0] {
            let (lin_pitch, lin_x) = p.linear_accel(0.001, torque);
            let (non_pitch, non_x) = p.nonlinear_accel(0.001, 0.0, torque);
            assert!(
                (lin_pitch - non_pitch).abs() < 0.01,
                "pitch {lin_pitch} vs {non_pitch}"
            );
            assert!((lin_x - non_x).abs() < 0.01, "x {lin_x} vs {non_x}");
        }
    }

    #[test]
    fn gravity_accelerates_a_falling_body() {
        let p = SimParams::nominal();
        let (pitch_dd, _) = p.nonlinear_accel(10.0_f32.to_radians(), 0.0, 0.0);
        assert!(pitch_dd > 0.0);
    }

    #[test]
    fn counter_torque_pushes_the_body_up() {
        let p = SimParams::nominal();
        let (pitch_dd, _) = p.nonlinear_accel(10.0_f32.to_radians(), 0.0, 2.0);
        assert!(pitch_dd < 0.0);
    }

    #[test]
    fn dead_zone_blocks_small_commands() {
        let mut plant = NonlinearPlant::new(SimParams::nominal(), 0.0);
        let state = plant.step(0.2, CONTROL_DT_S);
        assert_eq!(state.applied_torque_nm, 0.0);
    }

    #[test]
    fn stand_up_reference_walks_to_zero_from_both_sides() {
        assert_eq!(stand_up_reference(0.5, 180.0, 0.0), 0.5);
        assert_eq!(stand_up_reference(0.5, 180.0, 10.0), 0.0);
        assert_eq!(stand_up_reference(-0.5, 180.0, 10.0), 0.0);
        assert!(stand_up_reference(-0.5, 180.0, 0.1) < 0.0);
    }

    #[test]
    fn nominal_law_holds_a_small_lean() {
        let outcome = simulate(
            SimParams::nominal(),
            Strategy::StandUp { rate_dps: 180.0 },
            5.0,
            0.5,
        );
        assert!(outcome.success, "outcome {outcome:?}");
        assert!(outcome.final_pitch_deg.abs() < 5.0);
    }

    #[test]
    fn sweep_is_deterministic() {
        let p = SimParams::nominal();
        let strategies = [Strategy::StandUp { rate_dps: 180.0 }];
        let a = sweep(p, &strategies, &[5.0], &[10.0, 20.0]);
        let b = sweep(p, &strategies, &[5.0], &[10.0, 20.0]);
        assert_eq!(a, b);
    }
}
