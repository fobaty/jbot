//! Planar simulation model of the stage-1 wheeled biped.
//!
//! The model is the linearized inverted pendulum on two coaxial wheels, the
//! standard control-design model for this machine:
//!
//! ```text
//! pitch_dd = A1 * (pitch - slope) - A2 * torque + S1 * slope
//! x_dd     = B1 * torque - B2 * (pitch - slope) - S2 * slope
//! ```
//!
//! `pitch` is the body angle in the gravity frame, positive when the body
//! leans forward; `x` is the base position along the ground, positive forward;
//! `torque` is the common-mode wheel torque in newton-metres. `slope` is the
//! ground angle and enters through both the gravity torque on the body and the
//! gravity force along the slope, which is why the base must be pushed uphill
//! to stand still on an incline.
//!
//! The coefficients follow from the Lagrangian of the two-body system
//! (`body` + `wheels`) with the no-slip constraint, using
//!
//! ```text
//! a = m_body + m_wheel + I_wheel / r^2
//! b = m_body * L
//! c = I_body + m_body * L^2
//! d = a * c - b^2
//! A1 = a * m_body * g * L / d
//! A2 = (a + b / r) / d
//! B1 = (c / r + b) / d
//! B2 = b * m_body * g * L / d
//! S1 = b * (m_body + m_wheel) * g / d
//! S2 = c * (m_body + m_wheel) * g / d
//! ```
//!
//! The drive is modelled as a torque dead zone (`deadband_nm`) followed by a
//! pure delay of `delay_ticks` samples, because both effects dominate the
//! bench behaviour of the donor hardware: below roughly 80-110 command units
//! the wheels do not move at all, and the mainboard applies the command one
//! to two ticks late.
//!
//! This module is pure computation and is only used by host tests and the
//! bench-analysis tools. It is compiled into the firmware image as well, but
//! nothing in the 200 Hz path calls it.
//!
//! Sign conventions match [`crate::control`] and [`crate::reference`].

/// Gravity, m/s^2.
const G: f32 = 9.81;

/// Physical parameters of the machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantParams {
    /// Mass of the body without the wheels, kg.
    pub body_mass_kg: f32,
    /// Combined mass of both wheels, kg.
    pub wheel_mass_kg: f32,
    /// Body inertia about its centre of mass, kg m^2.
    pub body_inertia_kgm2: f32,
    /// Combined wheel inertia about the axle, kg m^2.
    pub wheel_inertia_kgm2: f32,
    /// Distance from the axle to the body centre of mass, m.
    pub com_height_m: f32,
    /// Wheel radius, m.
    pub wheel_radius_m: f32,
    /// Maximum common-mode torque, Nm.
    pub max_torque_nm: f32,
    /// Torque dead zone of the drive, Nm.
    pub deadband_nm: f32,
}

impl PlantParams {
    /// Nominal parameters used for control design. They are estimates from
    /// the donor hardware and must be replaced by bench measurements before
    /// the gains are trusted; only the resulting *ratios* matter, because the
    /// tuning is expressed relative to them.
    pub const NOMINAL: Self = Self {
        body_mass_kg: 8.0,
        wheel_mass_kg: 2.5,
        body_inertia_kgm2: 0.30,
        wheel_inertia_kgm2: 0.05,
        com_height_m: 0.18,
        wheel_radius_m: 0.0825,
        max_torque_nm: 5.0,
        deadband_nm: 0.5,
    };

    /// `A1`: pitch stiffness from gravity, 1/s^2.
    pub fn a1(self) -> f32 {
        let (a, _, _, d) = self.shape();
        a * self.body_mass_kg * G * self.com_height_m / d
    }

    /// `A2`: pitch response to wheel torque, 1/(kg m).
    pub fn a2(self) -> f32 {
        let (a, b, _c, d) = self.shape();
        (a + b / self.wheel_radius_m) / d
    }

    /// `B1`: base acceleration per unit wheel torque, 1/(kg m).
    pub fn b1(self) -> f32 {
        let (_a, b, c, d) = self.shape();
        (c / self.wheel_radius_m + b) / d
    }

    /// `B2`: base acceleration per radian of body lean, 1/s^2.
    pub fn b2(self) -> f32 {
        let (_a, b, _c, d) = self.shape();
        b * self.body_mass_kg * G * self.com_height_m / d
    }

    /// `S1`: extra pitch acceleration per radian of slope, 1/s^2.
    pub fn s1(self) -> f32 {
        let (_a, b, _c, d) = self.shape();
        b * (self.body_mass_kg + self.wheel_mass_kg) * G / d
    }

    /// `S2`: extra base acceleration per radian of slope, 1/s^2.
    pub fn s2(self) -> f32 {
        let (_a, _b, c, d) = self.shape();
        c * (self.body_mass_kg + self.wheel_mass_kg) * G / d
    }

    /// Smallest proportional pitch gain that stabilises the pitch mode,
    /// Nm/rad. Below this value the closed loop tips over.
    pub fn pitch_gain_floor_nm(self) -> f32 {
        self.a1() / self.a2()
    }

    /// Torque needed to hold a static lean of `pitch_rad`, Nm.
    pub fn hold_torque_nm(self, pitch_rad: f32) -> f32 {
        self.pitch_gain_floor_nm() * pitch_rad
    }

    /// Largest lean from which the maximum torque can still hold the body at
    /// rest, rad.
    ///
    /// At rest the pitch equation is `0 = A1 * pitch - A2 * torque`, so the
    /// limit is `torque / (A1 / A2)`, capped at a right angle. Note that the
    /// effective stiffness `A1 / A2` is lower than the simple pendulum value
    /// `m g L` because part of the gravity moment accelerates the wheels
    /// through the no-slip constraint. This is a model quantity; the firmware
    /// uses [`crate::control::Tuning::lift_limit_rad`], which is the same
    /// expression for the identified plant.
    pub fn lift_limit_rad(self) -> f32 {
        let floor = self.pitch_gain_floor_nm().max(1e-6);
        (self.max_torque_nm / floor).clamp(0.0, core::f32::consts::FRAC_PI_2)
    }

    /// Intermediates `(a, b, c, d)` of the Lagrangian.
    fn shape(self) -> (f32, f32, f32, f32) {
        let r = self.wheel_radius_m;
        let a = self.body_mass_kg + self.wheel_mass_kg + self.wheel_inertia_kgm2 / (r * r);
        let b = self.body_mass_kg * self.com_height_m;
        let c = self.body_inertia_kgm2 + self.body_mass_kg * self.com_height_m * self.com_height_m;
        (a, b, c, a * c - b * b)
    }
}

impl Default for PlantParams {
    fn default() -> Self {
        Self::NOMINAL
    }
}

/// Observed state of the simulated machine.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlantState {
    /// Base position along the ground, m.
    pub position_m: f32,
    /// Base speed, m/s.
    pub speed_mps: f32,
    /// Body pitch in the gravity frame, rad.
    pub pitch_rad: f32,
    /// Body pitch rate, rad/s.
    pub pitch_rate_rad_s: f32,
    /// Torque actually applied by the wheels, Nm, after the dead zone.
    pub applied_torque_nm: f32,
}

/// Number of delayed samples kept by the drive model.
const DELAY_LEN: usize = 4;

/// Simulated machine.
#[derive(Clone, Copy, Debug)]
pub struct Plant {
    params: PlantParams,
    state: PlantState,
    slope_rad: f32,
    delay_ticks: usize,
    delay: [f32; DELAY_LEN],
    delay_index: usize,
}

impl Plant {
    /// Build a plant at rest, upright, on level ground.
    pub fn new(params: PlantParams) -> Self {
        Self {
            params,
            state: PlantState::default(),
            slope_rad: 0.0,
            delay_ticks: 0,
            delay: [0.0; DELAY_LEN],
            delay_index: 0,
        }
    }

    /// Set the drive delay in control ticks, clamped to the buffer length.
    pub fn with_delay(mut self, ticks: usize) -> Self {
        self.delay_ticks = ticks.min(DELAY_LEN - 1);
        self
    }

    /// Place the machine at rest with the given pitch.
    pub fn reset(&mut self, pitch_rad: f32, slope_rad: f32) {
        self.state = PlantState {
            pitch_rad,
            ..PlantState::default()
        };
        self.slope_rad = slope_rad;
        self.delay = [0.0; DELAY_LEN];
        self.delay_index = 0;
    }

    /// Current state.
    pub fn state(&self) -> PlantState {
        self.state
    }

    /// Ground slope, rad.
    pub fn slope(&self) -> f32 {
        self.slope_rad
    }

    /// Change the ground slope without disturbing the state, rad.
    ///
    /// This models driving onto a ramp: the body keeps its pitch and speed at
    /// the instant the terrain changes, exactly like a real machine crossing
    /// the ramp lip.
    pub fn set_slope(&mut self, slope_rad: f32) {
        if slope_rad.is_finite() {
            self.slope_rad = slope_rad;
        }
    }

    /// Advance by `dt_s` with a commanded common-mode torque in Nm.
    ///
    /// A non-finite or non-positive `dt_s` leaves the state untouched.
    pub fn step(&mut self, command_nm: f32, dt_s: f32) -> PlantState {
        if !dt_s.is_finite() || dt_s <= 0.0 || !command_nm.is_finite() {
            return self.state;
        }

        // Drive dead zone, then transport delay. Below the dead zone the wheels
        // do not turn at all, so the body sees no reaction torque.
        let deadband = self.params.deadband_nm.max(0.0);
        let magnitude = command_nm.abs().min(self.params.max_torque_nm + deadband);
        let mut torque = if magnitude <= deadband {
            0.0
        } else {
            (magnitude - deadband).copysign(command_nm)
        };
        torque = torque.clamp(-self.params.max_torque_nm, self.params.max_torque_nm);

        if self.delay_ticks > 0 {
            let delayed = self.delay[self.delay_index];
            self.delay[self.delay_index] = torque;
            self.delay_index = (self.delay_index + 1) % self.delay_ticks;
            torque = delayed;
        }

        let p = self.params;
        let slope = self.slope_rad;
        let relative = self.state.pitch_rad - slope;
        let pitch_dd = p.a1() * relative - p.a2() * torque + p.s1() * slope;
        let x_dd = p.b1() * torque - p.b2() * relative - p.s2() * slope;

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

        self.state = PlantState {
            position_m: position,
            speed_mps: speed,
            pitch_rad: pitch,
            pitch_rate_rad_s: pitch_rate,
            applied_torque_nm: torque,
        };
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.005;

    #[test]
    fn nominal_coefficients_match_the_lagrangian() {
        let p = PlantParams::NOMINAL;
        // Values derived from the Lagrangian in the module header.
        assert!((p.a1() - 31.8).abs() < 0.5, "a1 = {}", p.a1());
        assert!((p.a2() - 4.46).abs() < 0.05, "a2 = {}", p.a2());
        assert!((p.b1() - 1.04).abs() < 0.02, "b1 = {}", p.b1());
        assert!((p.b2() - 2.57).abs() < 0.05, "b2 = {}", p.b2());
        // 5 Nm total against a 7.13 Nm/rad stability floor.
        assert!((p.pitch_gain_floor_nm() - 7.13).abs() < 0.1);
        assert!((p.lift_limit_rad().to_degrees() - 40.2).abs() < 0.5);
    }

    #[test]
    fn a_resting_body_falls_forward_without_torque() {
        let mut plant = Plant::new(PlantParams::NOMINAL);
        plant.reset(0.01, 0.0);
        for _ in 0..200 {
            plant.step(0.0, DT);
        }
        assert!(plant.state().pitch_rad > 0.05);
        assert!(plant.state().pitch_rate_rad_s > 0.0);
    }

    #[test]
    fn forward_torque_pitches_the_body_backwards() {
        let mut plant = Plant::new(PlantParams::NOMINAL);
        plant.reset(0.0, 0.0);
        plant.step(2.0, DT);
        assert!(plant.state().pitch_rad < 0.0);
        assert!(plant.state().pitch_rate_rad_s < 0.0);
    }

    #[test]
    fn the_dead_zone_blocks_small_commands() {
        let mut plant = Plant::new(PlantParams::NOMINAL);
        plant.reset(0.0, 0.0);
        let state = plant.step(0.4, DT);
        assert_eq!(state.applied_torque_nm, 0.0);
    }

    #[test]
    fn a_slope_pushes_the_machine_downhill_when_idle() {
        let mut plant = Plant::new(PlantParams::NOMINAL);
        plant.reset(0.0, 0.05);
        for _ in 0..100 {
            plant.step(0.0, DT);
        }
        assert!(plant.state().speed_mps < 0.0);
    }

    #[test]
    fn holding_a_slope_needs_uphill_torque() {
        // Solve the static equilibrium of
        //   pitch_dd = A1*(pitch - slope) - A2*torque + S1*slope
        //   x_dd     = B1*torque - B2*(pitch - slope) - S2*slope
        // for the torque and the pitch that make both zero. Without the drive
        // dead zone the state must then stay put.
        let mut params = PlantParams::NOMINAL;
        params.deadband_nm = 0.0;
        let p = params;
        let slope = 0.07_f32;
        let denominator = p.b1() - p.b2() * p.a2() / p.a1();
        let torque = slope * (p.s2() - p.b2() * p.s1() / p.a1()) / denominator;
        let relative = (p.a2() * torque - p.s1() * slope) / p.a1();
        assert!(torque > 0.0);

        let mut plant = Plant::new(params);
        plant.reset(slope + relative, slope);
        for _ in 0..2000 {
            plant.step(torque, DT);
        }
        assert!(
            plant.state().speed_mps.abs() < 0.02,
            "{}",
            plant.state().speed_mps
        );
        assert!((plant.state().pitch_rad - (slope + relative)).abs() < 0.02);

        // The same slope with no torque must slide downhill.
        let mut drifting = Plant::new(params);
        drifting.reset(slope + relative, slope);
        for _ in 0..2000 {
            drifting.step(0.0, DT);
        }
        assert!(drifting.state().speed_mps.abs() > 0.2);
    }

    #[test]
    fn non_finite_dt_is_ignored() {
        let mut plant = Plant::new(PlantParams::NOMINAL);
        plant.reset(0.1, 0.0);
        let before = plant.state();
        assert_eq!(plant.step(1.0, f32::NAN), before);
        assert_eq!(plant.step(1.0, 0.0), before);
        assert_eq!(plant.step(f32::NAN, DT), before);
    }
}
