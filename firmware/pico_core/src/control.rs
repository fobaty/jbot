//! Balance law for the stage-1 wheeled biped.
//!
//! Structure (one 200 Hz step), from the outside in:
//!
//! 1. the **speed command** is acceleration-limited and the **position loop**
//!    tracks it: a non-zero operator speed drives the origin forward and the
//!    position loop corrects the tracking error, while a zero command holds
//!    the arming position;
//! 2. the **speed loop** turns the speed error into a base acceleration
//!    reference;
//! 3. the acceleration reference is converted into a **pitch demand** with the
//!    identified closed-loop sensitivity of the base acceleration to the pitch
//!    reference, so the outer loops steer the machine by leaning it;
//! 4. the **pitch loop** is a PID on `pitch - reference - demand` and produces
//!    the common-mode wheel torque; the slow integral term removes the lean
//!    the drive dead zone would otherwise leave behind;
//! 5. the **yaw loop** produces the differential wheel torque and can only use
//!    the torque headroom that balance leaves unused.
//!
//! Why the position loop is not optional: a two-wheel balancer with a pure
//! pitch loop is neutrally stable in position. Any lean offset becomes a
//! constant acceleration, so the machine drifts until it runs out of room.
//! Closing the loop on position is what makes the machine stand still, and it
//! is also what makes it stand still on a slope: the trim integrator supplies
//! the steady pitch offset the slope demands (`pitch ≈ 1.6 * slope`).
//!
//! Two facts drive the design and must not be argued away:
//!
//! * **A non-zero commanded pitch cannot be held while standing still.** The
//!   only static equilibrium of the closed loop is the one where the wheels
//!   push against the terrain, so a commanded pitch bias makes the machine
//!   creep and the position loop pulls it back. `home` is therefore a posture
//!   *bias*, not a pose the machine can park at on level ground.
//! * **Recovery is bounded by torque.** With the pitch equation
//!   `pitch_dd = A1 * pitch - A2 * torque` the body can only be lifted from
//!   rest while `pitch < torque_max / (A1 / A2)`. The nominal machine lifts
//!   from about 10.6 degrees per newton-metre of total wheel torque.
//!
//! Gains are expressed in SI units and derived from [`PlantParams`], so a
//! measured plant (mass, centre-of-mass height, wheel inertia, torque budget)
//! re-scales the whole law instead of leaving a table of magic numbers.
//!
//! Sign convention: positive pitch means the body leans forward in the
//! direction of travel, and positive wheel torque drives the wheels forward.
//! Leaning forward therefore demands positive common torque: the wheels move
//! under the centre of mass. Positive yaw rate means a counter-clockwise turn
//! seen from above, which is produced by a positive right / negative left
//! differential.

use crate::eferu::{Command, TORQUE_LIMIT};
use crate::plant::PlantParams;

/// Torque limit expressed as a float, in EFeru command units.
pub const TORQUE_LIMIT_F32: f32 = TORQUE_LIMIT as f32;

/// Largest body pitch treated as a recoverable fall, radians (45 degrees).
/// Beyond this the machine is on the ground and the law must latch off.
pub const LYING_RAD: f32 = core::f32::consts::FRAC_PI_4;

/// Base travel from the arming position beyond which the machine is treated
/// as out of control, metres.
///
/// A recovery from a 30 degree lean needs about two metres of roll-back at the
/// nominal torque budget, so anything past three metres is a runaway rather
/// than a recovery and must latch the safety state.
pub const RANGE_LIMIT_M: f32 = 3.0;

/// Base speed beyond which the machine is treated as out of control, m/s.
///
/// A recovery from thirty degrees peaks near 1.4 m/s, so this only catches a
/// runaway and never a legitimate recovery.
pub const FALL_SPEED_MPS: f32 = 3.0;

/// Hard cap on the fall threshold derived from the torque budget, radians
/// (30 degrees). A larger budget does not make a larger control error safe,
/// because the outer loops are not designed for it.
const FALL_ERROR_CAP_RAD: f32 = core::f32::consts::FRAC_PI_6;

/// Fraction of the static lift limit accepted as a recoverable control error.
///
/// The static limit is where a full-torque command exactly cancels gravity at
/// zero pitch rate. Recovering from rest needs less than that, because the
/// body has to be accelerated up to the neutral point first. Simulation
/// recovers from 0.75 of the static limit at a 3 Nm budget and from about 0.63
/// at 8-12 Nm, so 0.70 keeps a margin on both ends.
const FALL_FRACTION: f32 = 0.70;

/// Balance tuning, SI units.
///
/// Build it with [`Tuning::from_plant`] so every gain follows the measured
/// machine. [`Tuning::SAFE`] leaves the law inert.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Tuning {
    /// Pitch loop proportional gain, Nm per radian.
    pub pitch_kp_nm_per_rad: f32,
    /// Pitch loop derivative gain, Nm per radian per second.
    pub pitch_kd_nm_per_rad_s: f32,
    /// Pitch loop integral gain, Nm per radian per second.
    ///
    /// The drive ignores small torque requests, so a proportional-only pitch
    /// loop parks the body at a lean offset of roughly `deadband / kp`. This
    /// slow integrator winds that offset away and lets the machine hold the
    /// captured angle instead of leaning on the dead zone.
    pub pitch_ki_nm_per_rad_s: f32,
    /// Clamp on the integral term, Nm. Large enough to break the drive
    /// stiction, small enough that a wind-up cannot drive the machine.
    pub pitch_integral_limit_nm: f32,
    /// Pitch tracking error below which the integral term may accumulate,
    /// radians. During a large recovery or a hard command the error is
    /// outside this window and the integrator is frozen, so it cannot wind up
    /// while the pitch loop is saturated.
    pub pitch_integral_window_rad: f32,
    /// Position loop gain, 1/s: metres of offset to metres per second.
    pub position_gain_per_s: f32,
    /// Speed loop gain, 1/s: metres per second of error to m/s^2.
    pub velocity_gain_per_s: f32,
    /// Position trim integrator gain, radians per (metre second).
    pub trim_gain_rad_per_m_s: f32,
    /// Closed-loop base acceleration per radian of pitch demand, m/s^2/rad.
    pub accel_per_rad_mps2: f32,
    /// Speed reference cap, m/s.
    pub speed_limit_mps: f32,
    /// Largest rate of change of the commanded speed, m/s^2. The operator
    /// speed setpoint is a step; this turns it into a gentle ramp so the
    /// outer loops never demand a violent lean.
    pub speed_slew_mps2: f32,
    /// Base acceleration reference cap, m/s^2.
    pub accel_limit_mps2: f32,
    /// Cap on the dynamic pitch demand, radians.
    pub pitch_demand_limit_rad: f32,
    /// Cap on the total pitch demand, including the slope trim, radians.
    pub pitch_trim_limit_rad: f32,
    /// Proportional pitch gain that just stabilises the pitch mode,
    /// `A1 / A2`, Nm per radian.
    pub pitch_floor_nm_per_rad: f32,
    /// Total common-mode torque available from the drive, Nm.
    pub torque_limit_nm: f32,
    /// Command units per newton-metre of common-mode torque.
    pub units_per_nm: f32,
    /// Torque dead zone of the drive, Nm. Zero disables the compensation.
    pub deadband_nm: f32,
    /// Yaw loop gain, command units per radian per second.
    pub yaw_kp_units: f32,
}

impl Tuning {
    /// All-zero tuning: the controller is inert and the robot must stay in
    /// `IDLE`. Used before the plant has been measured.
    pub const SAFE: Self = Self {
        pitch_kp_nm_per_rad: 0.0,
        pitch_kd_nm_per_rad_s: 0.0,
        pitch_ki_nm_per_rad_s: 0.0,
        pitch_integral_limit_nm: 0.0,
        pitch_integral_window_rad: 0.0,
        position_gain_per_s: 0.0,
        velocity_gain_per_s: 0.0,
        trim_gain_rad_per_m_s: 0.0,
        accel_per_rad_mps2: 0.0,
        speed_limit_mps: 0.0,
        speed_slew_mps2: 0.0,
        accel_limit_mps2: 0.0,
        pitch_demand_limit_rad: 0.0,
        pitch_trim_limit_rad: 0.0,
        pitch_floor_nm_per_rad: 0.0,
        torque_limit_nm: 0.0,
        units_per_nm: 0.0,
        deadband_nm: 0.0,
        yaw_kp_units: 0.0,
    };

    /// Derive the tuning from the identified machine.
    ///
    /// `pitch_margin` is the proportional pitch gain as a multiple of the
    /// stability floor `A1 / A2`; below 1.0 the pitch loop cannot stand up.
    /// `damping_ratio` sets the pitch loop damping (1.6 is critically damped
    /// with margin for the mainboard transport delay).
    ///
    /// The remaining loops are set for a decade of separation from the pitch
    /// loop: position 0.4 1/s, speed 1.8 1/s against a pitch loop near
    /// 1.5 Hz, and a slow trim integrator so it cannot pump the standing
    /// oscillation.
    pub fn from_plant(plant: &PlantParams, pitch_margin: f32, damping_ratio: f32) -> Self {
        let floor_nm = plant.pitch_gain_floor_nm();
        let kp = pitch_margin.max(1.0) * floor_nm;
        // Closed pitch loop: pitch_dd + A2*kd*pitch_d + (A2*kp - A1)*pitch = 0,
        // so the damping ratio is A2*kd / (2*sqrt(A2*kp - A1)).
        let stiffness = (plant.a2() * kp - plant.a1()).max(1e-3);
        let kd = 2.0 * damping_ratio.max(0.1) * micromath::F32Ext::sqrt(stiffness) / plant.a2();
        // Steady-state base acceleration per radian of pitch demand. The pitch
        // loop settles where `A1 * pitch = A2 * kp * error`, which puts the
        // body at `r = kp / (kp - floor)` times the demand and leaves the
        // wheels carrying `b1 * kp * (r - 1) - b2 * r` of acceleration.
        let margin = (kp / floor_nm).max(1.0);
        let accel_per_rad = (plant.b1() * floor_nm - plant.b2()) * margin / (margin - 1.0);
        let units_per_nm = if plant.max_torque_nm > 0.0 {
            TORQUE_LIMIT_F32 / plant.max_torque_nm
        } else {
            0.0
        };
        Self {
            pitch_kp_nm_per_rad: kp,
            pitch_kd_nm_per_rad_s: kd,
            // A decade below the pitch loop: the integrator removes the
            // steady lean the drive dead zone leaves behind without pumping
            // the stance. The window keeps it frozen during a recovery.
            pitch_ki_nm_per_rad_s: kp / 20.0,
            pitch_integral_limit_nm: 1.0,
            pitch_integral_window_rad: 3.0_f32.to_radians(),
            position_gain_per_s: 0.4,
            velocity_gain_per_s: 1.8,
            trim_gain_rad_per_m_s: 0.01,
            accel_per_rad_mps2: if accel_per_rad > 0.1 {
                accel_per_rad
            } else {
                0.1
            },
            speed_limit_mps: 1.0,
            speed_slew_mps2: 0.5,
            accel_limit_mps2: 1.0,
            pitch_demand_limit_rad: 8.0_f32.to_radians(),
            pitch_trim_limit_rad: 20.0_f32.to_radians(),
            pitch_floor_nm_per_rad: floor_nm,
            torque_limit_nm: plant.max_torque_nm,
            units_per_nm,
            // Off by default: an inverse of the drive dead zone behaves like a
            // relay and pumps the machine exactly on the slope where the
            // holding torque equals the dead zone. The pitch integrator above
            // removes the resulting lean without that relay behaviour.
            deadband_nm: 0.0,
            // Differential torque per radian per second of yaw-rate error.
            // A 1 rad/s error asks for 0.5 Nm per wheel, a gentle turn.
            yaw_kp_units: 0.5 * units_per_nm,
        }
    }

    /// True when at least the pitch loop is active.
    pub fn is_active(self) -> bool {
        self.pitch_kp_nm_per_rad > 0.0 && self.units_per_nm > 0.0 && self.torque_limit_nm > 0.0
    }

    /// Lean angle from which the torque budget can still lift the body from
    /// rest, radians.
    pub fn lift_limit_rad(self) -> f32 {
        if !self.is_active() || self.pitch_floor_nm_per_rad <= 0.0 {
            return 0.0;
        }
        // pitch_dd = A1*pitch - A2*torque is zero at pitch = torque / (A1/A2).
        (self.torque_limit_nm / self.pitch_floor_nm_per_rad).min(core::f32::consts::FRAC_PI_2)
    }

    /// Control error beyond which the pose is treated as a fall, radians.
    pub fn fall_error_rad(self) -> f32 {
        (FALL_FRACTION * self.lift_limit_rad()).min(FALL_ERROR_CAP_RAD)
    }

    /// Scale the angular-output gains by `factor`.
    ///
    /// A pure force multiplier for supervised bench tuning: the proportional,
    /// derivative and integral pitch gains and the yaw differential gain all
    /// scale together, so the closed-loop shape (the damping ratio the gains
    /// were derived for) is preserved while the torque the law is allowed to
    /// ask for grows or shrinks. The outer loops, the stability floor, the
    /// torque budget, the integral clamp and the fall thresholds are
    /// deliberately untouched: the fail-safe behaviour must not depend on how
    /// hard the machine corrects, only on what is allowed to happen.
    ///
    /// A zero factor makes the law inert ([`Tuning::is_active`] goes false and
    /// the step produces neutral torque), which is a safe parking spot for a
    /// hand-held start. Non-finite factors behave as zero; factors are capped
    /// at [`MAX_PSCALE`].
    pub fn scaled_by(self, factor: f32) -> Self {
        let factor = if factor.is_finite() {
            factor.clamp(0.0, MAX_PSCALE)
        } else {
            0.0
        };
        Self {
            pitch_kp_nm_per_rad: self.pitch_kp_nm_per_rad * factor,
            pitch_kd_nm_per_rad_s: self.pitch_kd_nm_per_rad_s * factor,
            pitch_ki_nm_per_rad_s: self.pitch_ki_nm_per_rad_s * factor,
            yaw_kp_units: self.yaw_kp_units * factor,
            ..self
        }
    }
}

/// Largest [`Tuning::scaled_by`] factor; the USB console polices the same
/// bound for its `pscale` command.
pub const MAX_PSCALE: f32 = 2.0;

/// High-level operating mode of the balance supervisor.
///
/// The live telemetry keeps its own historical `balance_mode` field; this enum
/// is the explicit operational mode consumed by the guard and by future trick
/// controllers. Codes are stable and must never be reordered.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BalanceMode {
    /// Upright balance and position hold.
    #[default]
    Balance,
    /// Commanded translation.
    Drive,
    /// Commanded in-place pivot.
    Spin,
    /// Deliberate stand-up attempt; the guard grants a short grace period.
    Raise,
    /// Intentional, controlled descent.
    SafeFall,
    /// Unrecoverable fault; torque stays zero until a fresh arm.
    Fault,
}

impl BalanceMode {
    /// Short stable label for logs and telemetry.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Balance => "balance",
            Self::Drive => "drive",
            Self::Spin => "spin",
            Self::Raise => "raise",
            Self::SafeFall => "safe_fall",
            Self::Fault => "fault",
        }
    }

    /// Numeric code for telemetry. Stable: never reorder.
    pub const fn code(self) -> u8 {
        match self {
            Self::Balance => 0,
            Self::Drive => 1,
            Self::Spin => 2,
            Self::Raise => 3,
            Self::SafeFall => 4,
            Self::Fault => 5,
        }
    }

    /// Inverse of [`Self::code`]; `None` for an unknown code.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Balance),
            1 => Some(Self::Drive),
            2 => Some(Self::Spin),
            3 => Some(Self::Raise),
            4 => Some(Self::SafeFall),
            5 => Some(Self::Fault),
            _ => None,
        }
    }

    /// Whether this mode grants a guard grace period.
    pub const fn grants_guard_grace(self) -> bool {
        matches!(self, Self::Raise | Self::SafeFall)
    }

    /// Default grace duration for the mode, seconds. Zero when no grace is
    /// granted.
    pub const fn default_grace_s(self) -> f32 {
        match self {
            Self::Raise => RAISE_GRACE_S,
            Self::SafeFall => SAFE_FALL_GRACE_S,
            _ => 0.0,
        }
    }
}

/// How long [`BalanceMode::Raise`] relaxes the guard, seconds.
pub const RAISE_GRACE_S: f32 = 0.8;

/// Pose-error and lying allowance granted during the raise grace, radians.
/// About 60 degrees: enough for a deliberate lean, short of a full lie-down.
pub const RAISE_GRACE_ERROR_RAD: f32 = 1.05;

/// Controlled-lean target of a guided tip-over, radians (about 12 degrees).
/// This is the angle the reduced torque budget can still shape, not a resting
/// angle: beyond it gravity takes over and the body lands on its own.
pub const SAFE_FALL_TARGET_RAD: f32 = 0.21;
/// Time over which the balance torque fades to zero during a guided tip-over.
pub const SAFE_FALL_FADE_S: f32 = 1.5;
/// Rate at which the controlled-fall reference leans away, radians per second
/// (about 15 degrees per second). Slow enough for the operator to abort.
pub const SAFE_FALL_RATE_RAD_S: f32 = 0.262;
/// How long the guard tolerates the growing pose error of a controlled fall.
pub const SAFE_FALL_GRACE_S: f32 = 6.0;

/// Why the supervisor refused a fresh arm request or ended a balance attempt.
///
/// The supervisor is pure so the decision can be tested without hardware and
/// so the control task can only ever forward the torque it produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    /// No recent wheel-speed feedback: the outer loops cannot run.
    StaleFeedback,
    /// The body is on the ground, beyond any recoverable pitch.
    Lying,
    /// The pose error exceeds what the torque budget can lift.
    PoseError,
    /// Travel since arming exceeded [`RANGE_LIMIT_M`].
    Range,
    /// Base speed exceeded [`FALL_SPEED_MPS`].
    Speed,
    /// A controlled fall reached its target angle and was ended deliberately.
    SafeFall,
}

impl StopReason {
    /// Short stable label for logs and telemetry.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::StaleFeedback => "stale_feedback",
            Self::Lying => "lying",
            Self::PoseError => "pose_error",
            Self::Range => "range",
            Self::Speed => "speed",
            Self::SafeFall => "safe_fall",
        }
    }

    /// Numeric code for storage and telemetry. Stable: never reorder.
    pub const fn code(self) -> u8 {
        match self {
            Self::StaleFeedback => 1,
            Self::Lying => 2,
            Self::PoseError => 3,
            Self::Range => 4,
            Self::Speed => 5,
            Self::SafeFall => 6,
        }
    }

    /// Inverse of [`Self::code`]; `None` means no recorded stop.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            1 => Some(Self::StaleFeedback),
            2 => Some(Self::Lying),
            3 => Some(Self::PoseError),
            4 => Some(Self::Range),
            5 => Some(Self::Speed),
            6 => Some(Self::SafeFall),
            _ => None,
        }
    }
}

/// Safety-relevant snapshot of one control step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SafetyState {
    /// Zeroed pitch, radians.
    pub pitch_rad: f32,
    /// Reference the pose error is measured against: the target while arming,
    /// the applied ramped reference while running.
    pub reference_rad: f32,
    /// Travel since arming, metres.
    pub position_m: f32,
    /// Base speed from the wheel feedback, metres per second.
    pub speed_mps: f32,
    /// True when a wheel-speed frame arrived within the feedback window.
    pub feedback_fresh: bool,
}

impl SafetyState {
    /// Pose error against the reference, radians.
    pub fn pose_error_rad(&self) -> f32 {
        self.pitch_rad - self.reference_rad
    }

    /// Why a fresh arm request must be refused, if it must.
    ///
    /// Travel and speed are not checked: arming resets the travel origin, and
    /// a machine at rest cannot already be a runaway. Feedback is checked
    /// first because it is the condition an operator can actually fix.
    pub fn arm_refusal(&self, tuning: &Tuning) -> Option<StopReason> {
        if !self.feedback_fresh {
            Some(StopReason::StaleFeedback)
        } else if self.pitch_rad.abs() > LYING_RAD {
            Some(StopReason::Lying)
        } else if self.pose_error_rad().abs() > tuning.fall_error_rad() {
            Some(StopReason::PoseError)
        } else {
            None
        }
    }

    /// Why a running balance attempt must be ended, if it must.
    pub fn fall_cause(&self, tuning: &Tuning) -> Option<StopReason> {
        self.mode_guard_cause(tuning, BalanceMode::Balance, self.pitch_rad, 0.0)
    }

    /// Mode-aware fall decision.
    ///
    /// `guard_tilt_rad` is the separately filtered tilt used only by the guard,
    /// so the control path keeps its own 12 Hz-filtered rate. During
    /// [`BalanceMode::Raise`] with grace left, the pose-error and lying limits
    /// are widened so a deliberate stand-up attempt is not latched as a fall;
    /// every other mode is identical to [`Self::fall_cause`]. A non-finite guard
    /// tilt falls back to the unfiltered pose instead of disabling the guard.
    pub fn mode_guard_cause(
        &self,
        tuning: &Tuning,
        mode: BalanceMode,
        guard_tilt_rad: f32,
        grace_remaining_s: f32,
    ) -> Option<StopReason> {
        let guard_tilt = if guard_tilt_rad.is_finite() {
            guard_tilt_rad
        } else {
            self.pitch_rad
        };
        let grace_active = mode.grants_guard_grace() && grace_remaining_s > 0.0;
        let lying_limit = if grace_active {
            RAISE_GRACE_ERROR_RAD
        } else {
            LYING_RAD
        };
        let pose_limit = if grace_active {
            tuning.fall_error_rad().max(RAISE_GRACE_ERROR_RAD)
        } else {
            tuning.fall_error_rad()
        };
        let error = guard_tilt - self.reference_rad;
        if guard_tilt.abs() > lying_limit {
            Some(StopReason::Lying)
        } else if error.abs() > pose_limit {
            Some(StopReason::PoseError)
        } else if self.position_m.abs() > RANGE_LIMIT_M {
            Some(StopReason::Range)
        } else if self.speed_mps.abs() > FALL_SPEED_MPS {
            Some(StopReason::Speed)
        } else {
            None
        }
    }
}

/// Inputs consumed by one balance step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BalanceInput {
    /// Pitch angle in the gravity frame, radians, positive leaning forward.
    pub pitch_rad: f32,
    /// Pitch rate, radians per second, positive falling forward.
    pub pitch_rate_rad_s: f32,
    /// Ramped home reference, radians.
    pub pitch_ref_rad: f32,
    /// Base position since arming, metres, positive forward.
    pub position_m: f32,
    /// Base speed from the wheel feedback, metres per second.
    pub speed_mps: f32,
    /// Operator forward/backward speed setpoint, metres per second. Positive
    /// drives forward. Zero holds the arming position.
    pub speed_setpoint_mps: f32,
    /// Requested yaw rate minus measured yaw rate, radians per second.
    pub yaw_rate_error_rad_s: f32,
    /// Torque scaling for the battery state; 1.0 at nominal pack voltage.
    pub battery_scale: f32,
}

impl Default for BalanceInput {
    fn default() -> Self {
        // A valid pack is 1.0. Deriving the default would leave the scale at
        // zero and silently halve the torque budget of any caller that only
        // overrides the angle.
        Self {
            pitch_rad: 0.0,
            pitch_rate_rad_s: 0.0,
            pitch_ref_rad: 0.0,
            position_m: 0.0,
            speed_mps: 0.0,
            speed_setpoint_mps: 0.0,
            yaw_rate_error_rad_s: 0.0,
            battery_scale: 1.0,
        }
    }
}

/// Wheel torque pair in EFeru command units, before rounding.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelTorque {
    /// Left wheel torque request.
    pub left: f32,
    /// Right wheel torque request.
    pub right: f32,
}

impl WheelTorque {
    /// Neutral request.
    pub const ZERO: Self = Self {
        left: 0.0,
        right: 0.0,
    };
}

/// Stateful balance controller.
///
/// The controller carries the position origin captured when the law is armed,
/// the acceleration-limited speed reference, the trim integrator that learns
/// the terrain, and the pitch-error integrator that removes the drive dead
/// zone lean. All of them are dropped by [`BalanceController::reset`], which
/// the firmware calls on arming and on every safety latch.
#[derive(Clone, Copy, Debug, Default)]
pub struct BalanceController {
    /// Base position at the moment of arming, metres.
    origin_m: f32,
    /// Acceleration-limited operator speed reference, m/s.
    speed_ref_mps: f32,
    /// Pitch offset demanded by the position loop, radians.
    demand_rad: f32,
    /// Slow trim that compensates a slope, radians.
    trim_rad: f32,
    /// Integral of the pitch tracking error, radian seconds.
    pitch_integral_rad_s: f32,
    /// True once the origin has been captured.
    anchored: bool,
}

impl BalanceController {
    /// A fresh controller with no state.
    pub const fn new() -> Self {
        Self {
            origin_m: 0.0,
            speed_ref_mps: 0.0,
            demand_rad: 0.0,
            trim_rad: 0.0,
            pitch_integral_rad_s: 0.0,
            anchored: false,
        }
    }

    /// Drop all state and anchor the position loop at `position_m`.
    pub fn reset(&mut self, position_m: f32) {
        self.origin_m = if position_m.is_finite() {
            position_m
        } else {
            0.0
        };
        self.speed_ref_mps = 0.0;
        self.demand_rad = 0.0;
        self.trim_rad = 0.0;
        self.pitch_integral_rad_s = 0.0;
        self.anchored = position_m.is_finite();
    }

    /// Total pitch offset currently demanded by the outer loops, radians.
    pub fn demand_rad(&self) -> f32 {
        self.demand_rad
    }

    /// Slope trim currently learned, radians.
    pub fn trim_rad(&self) -> f32 {
        self.trim_rad
    }

    /// Origin the position loop tracks, metres. When a speed is commanded the
    /// origin advances with the ramped reference, so the difference
    /// `position - origin` is a tracking error rather than the total travel.
    pub fn origin_m(&self) -> f32 {
        self.origin_m
    }

    /// Run one control step.
    ///
    /// `period_s` is the loop period. Non-finite inputs, a non-positive
    /// period, or an inactive tuning zero the command and drop the state.
    pub fn update(&mut self, input: &BalanceInput, tuning: &Tuning, period_s: f32) -> WheelTorque {
        if !input.pitch_rad.is_finite()
            || !input.pitch_rate_rad_s.is_finite()
            || !input.pitch_ref_rad.is_finite()
            || !input.position_m.is_finite()
            || !input.speed_mps.is_finite()
            || !input.speed_setpoint_mps.is_finite()
            || !input.yaw_rate_error_rad_s.is_finite()
            || !period_s.is_finite()
            || period_s <= 0.0
            || !tuning.is_active()
        {
            self.reset(0.0);
            return WheelTorque::ZERO;
        }

        if !self.anchored {
            self.reset(input.position_m);
        }

        let battery = if input.battery_scale.is_finite() {
            input.battery_scale.clamp(0.5, 2.0)
        } else {
            1.0
        };
        let torque_limit_nm = (tuning.torque_limit_nm * battery).max(0.0);

        // 1. Operator speed command -> acceleration-limited speed reference.
        //    The origin advances with the reference, so the position loop
        //    measures tracking error while driving and still holds a fixed
        //    point when the command is zero.
        let speed_target = clamp(input.speed_setpoint_mps, tuning.speed_limit_mps);
        let speed_step = clamp(
            speed_target - self.speed_ref_mps,
            tuning.speed_slew_mps2 * period_s,
        );
        self.speed_ref_mps += speed_step;
        self.origin_m += self.speed_ref_mps * period_s;

        // 2. Position error -> speed reference. Being ahead of the origin
        //    demands a negative speed, that is, rolling back. The ramped
        //    operator speed is the feed-forward the position loop corrects
        //    around, which is what lets the machine drive and still hold a
        //    straight line.
        let offset_m = input.position_m - self.origin_m;
        let speed_ref = clamp(
            self.speed_ref_mps - tuning.position_gain_per_s * offset_m,
            tuning.speed_limit_mps,
        );
        // 3. Speed error -> acceleration reference.
        let accel_ref = clamp(
            tuning.velocity_gain_per_s * (speed_ref - input.speed_mps),
            tuning.accel_limit_mps2,
        );
        // 4. Acceleration reference -> dynamic pitch demand.
        let dynamic = clamp(
            accel_ref / tuning.accel_per_rad_mps2,
            tuning.pitch_demand_limit_rad,
        );
        // 5. The trim integrator learns the terrain: it grows while the base
        //    keeps sliding back. It is frozen while the *total* demand sits on
        //    its clamp and the increment would push further into it, so a long
        //    slide cannot wind the trim past what the terrain needs.
        let increment = -tuning.trim_gain_rad_per_m_s * offset_m * period_s;
        let total = dynamic + self.trim_rad;
        let pushing_outwards = (total >= tuning.pitch_trim_limit_rad && increment > 0.0)
            || (total <= -tuning.pitch_trim_limit_rad && increment < 0.0);
        if !pushing_outwards {
            self.trim_rad = clamp(self.trim_rad + increment, tuning.pitch_trim_limit_rad);
        }
        self.demand_rad = clamp(dynamic + self.trim_rad, tuning.pitch_trim_limit_rad);

        // 6. Pitch loop, with the drive dead zone compensation when enabled.
        let error = input.pitch_rad - input.pitch_ref_rad - self.demand_rad;
        let pd_nm = tuning.pitch_kp_nm_per_rad * error
            + tuning.pitch_kd_nm_per_rad_s * input.pitch_rate_rad_s;
        // The drive ignores small torque requests, so a proportional-only
        // pitch loop parks the body at `deadband / kp` of lean. Integrate the
        // pitch error, but only near equilibrium and only while the total
        // output is not saturated, so the term learns the breakaway torque
        // without winding up during a recovery or a hard command.
        let integral_nm = tuning.pitch_ki_nm_per_rad_s * self.pitch_integral_rad_s;
        if tuning.pitch_ki_nm_per_rad_s > 0.0
            && error.abs() < tuning.pitch_integral_window_rad
            && (pd_nm + integral_nm).abs() < torque_limit_nm * 0.95
        {
            let state_limit = tuning.pitch_integral_limit_nm / tuning.pitch_ki_nm_per_rad_s;
            self.pitch_integral_rad_s =
                (self.pitch_integral_rad_s + error * period_s).clamp(-state_limit, state_limit);
        }
        let mut torque_nm = pd_nm + tuning.pitch_ki_nm_per_rad_s * self.pitch_integral_rad_s;
        torque_nm += deadband_compensation(torque_nm, tuning.deadband_nm);
        torque_nm = torque_nm.clamp(-torque_limit_nm, torque_limit_nm);

        let common = clamp(torque_nm * tuning.units_per_nm, TORQUE_LIMIT_F32);
        if !common.is_finite() {
            self.reset(input.position_m);
            return WheelTorque::ZERO;
        }

        // 7. Balance has priority: reserve the common-mode torque, then let
        //    yaw use only the remaining headroom on both wheels. A positive
        //    differential drives the right wheel forward and the left wheel
        //    backward, which is a counter-clockwise turn: the same sign as a
        //    positive yaw-rate error.
        let headroom = (TORQUE_LIMIT_F32 - common.abs()).max(0.0);
        let differential = clamp(tuning.yaw_kp_units * input.yaw_rate_error_rad_s, headroom);

        WheelTorque {
            left: common - differential,
            right: common + differential,
        }
    }
}

/// Smooth inverse of the torque dead zone, newton-metres in and out.
///
/// Ramped pitch reference for a controlled fall.
///
/// The reference starts at the measured pitch so arming the mode cannot cause
/// an instantaneous command jump, then leans away at a bounded rate in the
/// direction the body is already falling. Reaching the target does not remove
/// the reference: the operator or the supervisor ends the mode.
#[derive(Clone, Copy, Debug)]
pub struct SafeFallReference {
    direction: f32,
    angle_rad: f32,
    rate_rad_s: f32,
    target_rad: f32,
    started: bool,
}

impl SafeFallReference {
    /// Build a ramp with the given rate and target. Non-finite or non-positive
    /// arguments fall back to the documented defaults.
    pub fn new(rate_rad_s: f32, target_rad: f32) -> Self {
        let rate = if rate_rad_s.is_finite() && rate_rad_s > 0.0 {
            rate_rad_s
        } else {
            SAFE_FALL_RATE_RAD_S
        };
        let target = if target_rad.is_finite() && target_rad > 0.0 {
            target_rad.min(SAFE_FALL_TARGET_RAD)
        } else {
            SAFE_FALL_TARGET_RAD
        };
        Self {
            direction: 1.0,
            angle_rad: 0.0,
            rate_rad_s: rate,
            target_rad: target,
            started: false,
        }
    }

    /// Advance one tick and return the pitch reference in radians.
    /// An invalid sample returns the previous reference without advancing.
    pub fn update(&mut self, pitch_rad: f32, period_s: f32) -> f32 {
        if !pitch_rad.is_finite() || !period_s.is_finite() || period_s <= 0.0 {
            return self.direction * self.angle_rad;
        }
        if !self.started {
            self.direction = if pitch_rad < 0.0 { -1.0 } else { 1.0 };
            self.angle_rad = pitch_rad.abs().min(self.target_rad);
            self.started = true;
            return self.direction * self.angle_rad;
        }
        self.angle_rad = (self.angle_rad + self.rate_rad_s * period_s).min(self.target_rad);
        self.direction * self.angle_rad
    }

    /// True once the ramp has reached its target angle.
    pub fn done(&self) -> bool {
        self.started && self.angle_rad >= self.target_rad
    }

    /// Drop the ramp so a fresh mode entry re-anchors to the measured pitch.
    pub fn reset(&mut self) {
        self.direction = 1.0;
        self.angle_rad = 0.0;
        self.started = false;
    }
}

/// Smoothly decaying torque multiplier for a guided tip-over.
///
/// Cutting balance torque in one step makes the body drop from whatever angle
/// it happens to hold. Fading it linearly lets the operator see the loss of
/// support coming and keeps the commanded torque continuous until zero.
#[derive(Clone, Copy, Debug)]
pub struct FadeGain {
    value: f32,
    rate_per_s: f32,
}

impl FadeGain {
    /// Build a gain that decays from 1.0 to zero over `duration_s`.
    /// A non-finite or non-positive duration freezes the gain at 1.0.
    pub fn new(duration_s: f32) -> Self {
        let rate = if duration_s.is_finite() && duration_s > 0.0 {
            1.0 / duration_s
        } else {
            0.0
        };
        Self {
            value: 1.0,
            rate_per_s: rate,
        }
    }

    /// Advance one tick and return the current multiplier.
    pub fn update(&mut self, period_s: f32) -> f32 {
        if !period_s.is_finite() || period_s <= 0.0 {
            return self.value;
        }
        self.value = (self.value - self.rate_per_s * period_s).max(0.0);
        self.value
    }

    /// Current multiplier, in `0.0..=1.0`.
    pub fn value(&self) -> f32 {
        self.value
    }

    /// True once the torque has fully faded out.
    pub fn done(&self) -> bool {
        self.value <= 0.0
    }

    /// Restore the gain to 1.0 for a fresh mode entry.
    pub fn reset(&mut self) {
        self.value = 1.0;
    }
}

/// The drive ignores commands below `deadband` and then applies the excess, so
/// the inverse has to add the dead zone back. A hard sign step would act like a
/// relay and pump the machine, so the last bit before zero is blended.
fn deadband_compensation(torque_nm: f32, deadband_nm: f32) -> f32 {
    if !deadband_nm.is_finite() || deadband_nm <= 0.0 || !torque_nm.is_finite() {
        return 0.0;
    }
    if torque_nm.abs() >= deadband_nm {
        deadband_nm.copysign(torque_nm)
    } else {
        let ratio = torque_nm / deadband_nm;
        deadband_nm * ratio * ratio.abs()
    }
}

/// Convert a wheel torque pair into the EFeru steer/speed command.
///
/// Bench-verified on the flashed `VARIANT_USART` mainboard (stock mixer, no
/// `INVERT_L_DIRECTION` / `INVERT_R_DIRECTION`): the wire `speed` channel
/// drives both wheels the same way, and `steer` counter-rotates them. The
/// console `test` path feeds this function, which makes the mapping visible
/// (session `jbot-telemetry-1795141040.csv`, 2026-11-19):
///
/// * `test 50 50` reaches the wire as `speed = 50, steer = 0` and rolls the
///   machine forward; the hall feedback reads `L = +8`, `R = -8` rpm.
/// * `test 100 -100` pivots the machine in place; the heading changes and the
///   gyro reads `gz ~= -30 dps`. That session used the old inverse and the wire
///   saw `steer = 100` (half authority due to `STEER_COEFFICIENT = 0.5`); the
///   corrected inverse now sends `steer = 200` for the same request.
///
/// A same-sign physical [`WheelTorque`] is therefore a straight roll, which
/// is exactly what the plain mixer inverse produces. The right wheel must not
/// be negated here: doing so turns the balance law's common-mode torque into
/// a `steer` command and makes the machine spin on the spot.
///
/// [`crate::eferu::Feedback::forward_speed_mps`] follows the same convention:
/// a forward roll reads opposite-sign hall speeds on this unit.
pub fn to_command(torque: WheelTorque) -> Command {
    let left = torque_to_i16(torque.left);
    let right = torque_to_i16(torque.right);
    Command::from_wheel_torque(left, right)
}

fn torque_to_i16(value: f32) -> i16 {
    if !value.is_finite() {
        return 0;
    }
    let clamped = value.clamp(-TORQUE_LIMIT_F32, TORQUE_LIMIT_F32);
    // `f32::round` lives in `std`, so round half away from zero manually.
    let rounded = if clamped >= 0.0 {
        clamped + 0.5
    } else {
        clamped - 0.5
    };
    rounded as i16
}

fn clamp(value: f32, limit: f32) -> f32 {
    value.clamp(-limit.abs(), limit.abs())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plant::{Plant, PlantParams};

    const DT: f32 = 0.005;

    fn tuning() -> Tuning {
        Tuning::from_plant(&PlantParams::NOMINAL, 4.0, 1.6)
    }

    #[test]
    fn scaled_by_preserves_shape_and_fail_safe() {
        let base = tuning();
        let half = base.scaled_by(0.5);
        assert!((half.pitch_kp_nm_per_rad - 0.5 * base.pitch_kp_nm_per_rad).abs() < 1e-6);
        assert!((half.pitch_kd_nm_per_rad_s - 0.5 * base.pitch_kd_nm_per_rad_s).abs() < 1e-6);
        assert!((half.yaw_kp_units - 0.5 * base.yaw_kp_units).abs() < 1e-6);
        // Fall thresholds, budget and floor do not move with the effort scale.
        assert_eq!(half.fall_error_rad(), base.fall_error_rad());
        assert_eq!(half.lift_limit_rad(), base.lift_limit_rad());
        assert_eq!(half.torque_limit_nm, base.torque_limit_nm);
        assert_eq!(half.pitch_floor_nm_per_rad, base.pitch_floor_nm_per_rad);
        // Outer loops and the unit conversion untouched.
        assert_eq!(half.position_gain_per_s, base.position_gain_per_s);
        assert_eq!(half.velocity_gain_per_s, base.velocity_gain_per_s);
        assert_eq!(half.units_per_nm, base.units_per_nm);
        assert_eq!(half.pitch_demand_limit_rad, base.pitch_demand_limit_rad);
    }

    #[test]
    fn scale_one_is_identity_and_zero_is_inert() {
        let base = tuning();
        assert_eq!(base.scaled_by(1.0), base);
        assert!(!base.scaled_by(0.0).is_active());
        assert!(base.scaled_by(1.0).is_active());
    }

    #[test]
    fn scale_is_capped_and_non_finite_is_zero() {
        let base = tuning();
        let capped = base.scaled_by(99.0);
        assert_eq!(
            capped.pitch_kp_nm_per_rad,
            MAX_PSCALE * base.pitch_kp_nm_per_rad
        );
        assert!(capped.is_active());
        assert!(!base.scaled_by(f32::NAN).is_active());
        assert!(!base.scaled_by(f32::INFINITY).is_active());
        assert_eq!(base.scaled_by(-1.0), base.scaled_by(0.0));
    }

    /// One simulated closed-loop run. Rows are
    /// `(time, position, speed, pitch, pitch_rate, applied_torque)`.
    type Row = (f32, f32, f32, f32, f32, f32);

    /// Closed-loop scenario description.
    struct Setup {
        /// Pitch at the moment of arming, radians.
        start_pitch_rad: f32,
        /// Terrain slope, radians.
        slope_rad: f32,
        /// Commanded home pitch, radians.
        home_rad: f32,
        /// Simulated duration, seconds.
        seconds: f32,
        /// Plant parameters before the dead zone override.
        plant: PlantParams,
        /// Mainboard transport delay, control ticks.
        delay: usize,
        /// Drive dead zone, Nm.
        deadband_nm: f32,
        /// Terrain change partway through the run: `(time_s, slope_rad)`.
        slope_step: Option<(f32, f32)>,
        /// Operator speed command, m/s. Zero holds the arming position.
        speed_cmd_mps: f32,
        /// Captured `zero` offset, radians. The controller sees
        /// `pitch - zero_rad`; zero on a slope models capturing the reference
        /// pose in place, which is the documented workflow for steep terrain.
        zero_rad: f32,
    }

    impl Default for Setup {
        fn default() -> Self {
            Self {
                start_pitch_rad: 0.0,
                slope_rad: 0.0,
                home_rad: 0.0,
                seconds: 20.0,
                plant: PlantParams::NOMINAL,
                // 0.5 Nm is the bench estimate for the donor drive.
                delay: 0,
                deadband_nm: 0.5,
                slope_step: None,
                speed_cmd_mps: 0.0,
                zero_rad: 0.0,
            }
        }
    }

    fn simulate(setup: Setup, tuning: Tuning) -> Vec<Row> {
        let mut params = setup.plant;
        params.deadband_nm = setup.deadband_nm;
        let mut plant = Plant::new(params).with_delay(setup.delay);
        plant.reset(setup.start_pitch_rad, setup.slope_rad);
        let start_zeroed = setup.start_pitch_rad - setup.zero_rad;

        let mut controller = BalanceController::new();
        let mut reference = crate::reference::PitchReference::new();
        reference.set_target(setup.home_rad);
        // Arming behaviour: a small pose error is anchored to the measured
        // pose and ramped home. A large one starts at the target, so the law
        // has to actually lift the body instead of slowly chasing it.
        if (start_zeroed - setup.home_rad).abs() < 5.0_f32.to_radians() {
            reference.reset(start_zeroed);
        } else {
            reference.reset(setup.home_rad);
        }
        controller.reset(0.0);

        let mut log = Vec::new();
        for step in 0..(setup.seconds / DT) as usize {
            if let Some((at_s, slope_rad)) = setup.slope_step {
                if step == (at_s / DT) as usize {
                    plant.set_slope(slope_rad);
                }
            }
            let state = plant.state();
            let reference_step = reference.update(DT);
            let input = BalanceInput {
                pitch_rad: state.pitch_rad - setup.zero_rad,
                pitch_rate_rad_s: state.pitch_rate_rad_s,
                pitch_ref_rad: reference_step.ref_rad,
                position_m: state.position_m,
                speed_mps: state.speed_mps,
                speed_setpoint_mps: setup.speed_cmd_mps,
                yaw_rate_error_rad_s: 0.0,
                battery_scale: 1.0,
            };
            let torque = controller.update(&input, &tuning, DT);
            let common = 0.5 * (torque.left + torque.right);
            plant.step(common / tuning.units_per_nm, DT);
            let after = plant.state();
            log.push((
                step as f32 * DT,
                after.position_m,
                after.speed_mps,
                after.pitch_rad,
                after.pitch_rate_rad_s,
                after.applied_torque_nm,
            ));
        }
        log
    }

    fn peak_after(log: &[Row], from_s: f32, index: usize) -> f32 {
        log.iter()
            .filter(|row| row.0 >= from_s)
            .map(|row| match index {
                1 => row.1.abs(),
                2 => row.2.abs(),
                3 => row.3.abs(),
                4 => row.4.abs(),
                _ => row.5.abs(),
            })
            .fold(0.0_f32, f32::max)
    }

    fn last(log: &[Row], index: usize) -> f32 {
        let row = *log.last().expect("log is not empty");
        match index {
            1 => row.1,
            2 => row.2,
            3 => row.3,
            4 => row.4,
            _ => row.5,
        }
    }

    #[test]
    fn derived_gains_match_the_design() {
        let t = tuning();
        // 4x the stability floor of 5.39 Nm/rad.
        assert!(
            (t.pitch_kp_nm_per_rad - 4.0 * 5.39).abs() < 0.3,
            "{}",
            t.pitch_kp_nm_per_rad
        );
        // Damping ratio 1.6 on a better-than-1 Hz pitch mode.
        assert!(
            (t.pitch_kd_nm_per_rad_s - 5.94).abs() < 0.3,
            "{}",
            t.pitch_kd_nm_per_rad_s
        );
        // (b1*floor - b2) * margin / (margin - 1) = 4.85 * 4/3.
        assert!(
            (t.accel_per_rad_mps2 - 7.33).abs() < 0.2,
            "{}",
            t.accel_per_rad_mps2
        );
        assert!(t.is_active());
        assert!(!Tuning::SAFE.is_active());
    }

    #[test]
    fn accel_per_rad_is_the_measured_sensitivity() {
        // Hold a fixed pitch demand and measure the base acceleration once the
        // pitch loop has settled. This is the number the outer loops divide by.
        let base = tuning();
        let frozen = Tuning {
            position_gain_per_s: 0.0,
            velocity_gain_per_s: 0.0,
            trim_gain_rad_per_m_s: 0.0,
            ..base
        };
        // The identification demand settles at a holding torque close to the
        // drive dead zone, which would swallow part of it. Measure the plant
        // property, not the drive nonlinearity.
        let mut params = PlantParams::NOMINAL;
        params.deadband_nm = 0.0;
        let mut plant = Plant::new(params);
        plant.reset(0.0, 0.0);

        let demand = 3.0_f32.to_radians();
        let mut controller = BalanceController::new();
        controller.reset(0.0);
        controller.trim_rad = demand;
        let mut speed_at = 0.0;
        for step in 0..400 {
            let state = plant.state();
            let input = BalanceInput {
                pitch_rad: state.pitch_rad,
                pitch_rate_rad_s: state.pitch_rate_rad_s,
                pitch_ref_rad: 0.0,
                position_m: state.position_m,
                speed_mps: state.speed_mps,
                speed_setpoint_mps: 0.0,
                yaw_rate_error_rad_s: 0.0,
                battery_scale: 1.0,
            };
            let torque = controller.update(&input, &frozen, DT);
            plant.step(0.5 * (torque.left + torque.right) / base.units_per_nm, DT);
            if step == 200 {
                speed_at = plant.state().speed_mps;
            }
        }
        let measured = (plant.state().speed_mps - speed_at) / (200.0 * DT);
        let expected = base.accel_per_rad_mps2 * demand;
        assert!(
            (measured / expected - 1.0).abs() < 0.2,
            "measured {measured} expected {expected}"
        );
    }

    #[test]
    fn safe_tuning_produces_no_torque() {
        let input = BalanceInput {
            pitch_rad: 0.5,
            pitch_rate_rad_s: 1.0,
            pitch_ref_rad: 0.0,
            position_m: 1.0,
            speed_mps: 1.0,
            speed_setpoint_mps: 0.0,
            yaw_rate_error_rad_s: 1.0,
            battery_scale: 1.0,
        };
        let mut controller = BalanceController::new();
        assert_eq!(
            controller.update(&input, &Tuning::SAFE, DT),
            WheelTorque::ZERO
        );
    }

    #[test]
    fn forward_lean_produces_forward_torque() {
        let input = BalanceInput {
            pitch_rad: 0.1,
            ..BalanceInput::default()
        };
        let mut controller = BalanceController::new();
        let output = controller.update(&input, &tuning(), DT);
        assert!(output.left > 0.0);
        assert!((output.left - output.right).abs() < 1e-3);
    }

    #[test]
    fn battery_scale_shrinks_the_torque_budget() {
        // A 0.2 rad lean asks for about 4.3 Nm, which fits inside the full
        // 5 Nm budget but not inside half of it, so the reduced scale binds.
        let input = BalanceInput {
            pitch_rad: 0.2,
            ..BalanceInput::default()
        };
        let mut full = BalanceController::new();
        let strong = full.update(&input, &tuning(), DT);
        let mut weak = BalanceController::new();
        let low_battery = BalanceInput {
            battery_scale: 0.5,
            ..input
        };
        let reduced = weak.update(&low_battery, &tuning(), DT);
        assert!(reduced.left.abs() < strong.left.abs());
        assert!(reduced.left > 0.0);
    }

    #[test]
    fn non_finite_inputs_are_rejected() {
        let mut controller = BalanceController::new();
        let bad = [
            BalanceInput {
                pitch_rad: f32::NAN,
                ..BalanceInput::default()
            },
            BalanceInput {
                speed_mps: f32::INFINITY,
                ..BalanceInput::default()
            },
            BalanceInput {
                position_m: f32::NAN,
                ..BalanceInput::default()
            },
        ];
        for input in bad {
            assert_eq!(controller.update(&input, &tuning(), DT), WheelTorque::ZERO);
        }
        assert_eq!(
            controller.update(&BalanceInput::default(), &tuning(), 0.0),
            WheelTorque::ZERO
        );
    }

    #[test]
    fn holds_level_from_a_small_lean() {
        let log = simulate(
            Setup {
                start_pitch_rad: 1.0_f32.to_radians(),
                seconds: 20.0,
                ..Setup::default()
            },
            tuning(),
        );
        assert!(
            peak_after(&log, 6.0, 3) < 3.5_f32.to_radians(),
            "{}",
            peak_after(&log, 6.0, 3).to_degrees()
        );
        assert!(
            peak_after(&log, 6.0, 1) < 0.15,
            "{}",
            peak_after(&log, 6.0, 1)
        );
        assert!(last(&log, 3).abs() < 3.0_f32.to_radians());
    }

    #[test]
    fn recovers_from_twenty_degrees() {
        let log = simulate(
            Setup {
                start_pitch_rad: 20.0_f32.to_radians(),
                seconds: 25.0,
                ..Setup::default()
            },
            tuning(),
        );
        // Upright again, with the excursion bounded to a fraction of a metre.
        assert!(
            last(&log, 3).abs() < 3.0_f32.to_radians(),
            "{}",
            last(&log, 3).to_degrees()
        );
        assert!(
            peak_after(&log, 0.0, 1) < 0.8,
            "{}",
            peak_after(&log, 0.0, 1)
        );
        assert!(last(&log, 2).abs() < 0.1, "{}", last(&log, 2));
    }

    #[test]
    fn recovers_from_thirty_degrees() {
        // Within the 28 degree fall threshold at the nominal 5 Nm budget. The
        // roll-back is about 1.7 m: the body has to swing back to gain the
        // pitch rate that carries it upright.
        let log = simulate(
            Setup {
                start_pitch_rad: 30.0_f32.to_radians(),
                seconds: 30.0,
                ..Setup::default()
            },
            tuning(),
        );
        assert!(
            last(&log, 3).abs() < 3.0_f32.to_radians(),
            "{}",
            last(&log, 3).to_degrees()
        );
        assert!(
            peak_after(&log, 0.0, 1) < 2.5,
            "{}",
            peak_after(&log, 0.0, 1)
        );
        assert!(last(&log, 2).abs() < 0.15, "{}", last(&log, 2));
    }

    #[test]
    fn holds_on_a_slope() {
        // The machine is zeroed on level ground, then armed on a slope at the
        // static equilibrium pose (about 1.6 times the slope). Only the slopes
        // that fit inside the firmware `RANGE_LIMIT_M = 3 m` guard are tested
        // here: the trim has to learn the offset before the position loop can
        // hold, so a steeper slope rolls back further at first. Steep terrain
        // is handled by capturing `zero`/`level` on the surface, which is the
        // separate test below.
        //
        // 8 degrees is deliberately absent: the holding torque there (about
        // 0.6 Nm) sits on the edge of the estimated 0.5 Nm drive dead zone and
        // the drive chatters. See docs/design_balance_home.md.
        for (degrees, travel_limit_m) in [(4.0_f32, 1.0_f32), (6.0, 2.0)] {
            let slope = degrees.to_radians();
            let log = simulate(
                Setup {
                    start_pitch_rad: 1.602 * slope,
                    slope_rad: slope,
                    seconds: 45.0,
                    ..Setup::default()
                },
                tuning(),
            );
            let pitch = last(&log, 3);
            let expected = 1.6 * slope;
            assert!(
                (pitch - expected).abs() < 3.0_f32.to_radians(),
                "slope {degrees}: pitch {} expected {}",
                pitch.to_degrees(),
                expected.to_degrees()
            );
            assert!(
                last(&log, 2).abs() < 0.05,
                "slope {degrees}: speed {}",
                last(&log, 2)
            );
            assert!(
                peak_after(&log, 0.0, 1) < travel_limit_m,
                "slope {degrees}: travel {}",
                peak_after(&log, 0.0, 1)
            );
        }
    }

    #[test]
    fn holds_a_steep_slope_when_zeroed_on_it() {
        // The documented workflow for steep terrain: capture `zero`/`level`
        // while standing on the surface, so the commanded pose is the local
        // equilibrium. The machine must then hold 10 and 12 degrees with
        // almost no travel, instead of spending the 9-17 m an on-flat-ground
        // zero would need (which the range guard would abort anyway).
        for degrees in [10.0_f32, 12.0] {
            let slope = degrees.to_radians();
            let equilibrium = 1.602 * slope;
            let log = simulate(
                Setup {
                    start_pitch_rad: equilibrium,
                    slope_rad: slope,
                    zero_rad: equilibrium,
                    seconds: 45.0,
                    ..Setup::default()
                },
                tuning(),
            );
            let pitch = last(&log, 3);
            assert!(
                (pitch - equilibrium).abs() < 2.0_f32.to_radians(),
                "slope {degrees}: pitch {} expected {}",
                pitch.to_degrees(),
                equilibrium.to_degrees()
            );
            assert!(
                last(&log, 2).abs() < 0.05,
                "slope {degrees}: speed {}",
                last(&log, 2)
            );
            assert!(
                peak_after(&log, 0.0, 1) < 1.5,
                "slope {degrees}: travel {}",
                peak_after(&log, 0.0, 1)
            );
        }
    }

    #[test]
    fn holds_when_the_ground_tilts() {
        // Drive onto a ramp while balancing: the machine must re-learn the new
        // slope offset and come to rest on it. Six degrees is the steepest
        // entry that stays inside a metre of travel with the nominal torque
        // budget; steeper terrain is outside the stage-1 envelope and is
        // documented in docs/design_balance_home.md.
        for degrees in [4.0_f32, 6.0] {
            let slope = degrees.to_radians();
            let log = simulate(
                Setup {
                    slope_step: Some((5.0, slope)),
                    seconds: 40.0,
                    ..Setup::default()
                },
                tuning(),
            );
            let expected = 1.6 * slope;
            let pitch = last(&log, 3);
            assert!(
                (pitch - expected).abs() < 3.0_f32.to_radians(),
                "slope {degrees}: pitch {} expected {}",
                pitch.to_degrees(),
                expected.to_degrees()
            );
            assert!(
                last(&log, 2).abs() < 0.05,
                "slope {degrees}: speed {}",
                last(&log, 2)
            );
            assert!(
                peak_after(&log, 4.0, 1) < 1.5,
                "slope {degrees}: travel {}",
                peak_after(&log, 4.0, 1)
            );
        }
    }

    #[test]
    fn home_step_returns_smoothly() {
        let log = simulate(
            Setup {
                home_rad: 5.0_f32.to_radians(),
                seconds: 20.0,
                ..Setup::default()
            },
            tuning(),
        );
        // The acceleration-limited ramp keeps the excursion under a metre and
        // the pitch within the commanded bias plus a small overshoot.
        assert!(
            peak_after(&log, 0.0, 1) < 0.8,
            "{}",
            peak_after(&log, 0.0, 1)
        );
        assert!(peak_after(&log, 0.0, 3) < 6.5_f32.to_radians());
        assert!(last(&log, 3).abs() < 4.0_f32.to_radians());
    }

    #[test]
    fn survives_a_drive_dead_zone() {
        let log = simulate(
            Setup {
                start_pitch_rad: 2.0_f32.to_radians(),
                deadband_nm: 0.8,
                ..Setup::default()
            },
            tuning(),
        );
        assert!(
            peak_after(&log, 6.0, 3) < 5.5_f32.to_radians(),
            "{}",
            peak_after(&log, 6.0, 3).to_degrees()
        );
        assert!(last(&log, 1).abs() < 0.1, "{}", last(&log, 1));
    }

    #[test]
    fn survives_transport_delay() {
        let log = simulate(
            Setup {
                start_pitch_rad: 2.0_f32.to_radians(),
                delay: 3,
                ..Setup::default()
            },
            tuning(),
        );
        assert!(
            peak_after(&log, 6.0, 3) < 4.5_f32.to_radians(),
            "{}",
            peak_after(&log, 6.0, 3).to_degrees()
        );
        assert!(last(&log, 1).abs() < 0.1);
    }

    #[test]
    fn survives_a_heavier_machine() {
        let log = simulate(
            Setup {
                start_pitch_rad: 2.0_f32.to_radians(),
                plant: PlantParams {
                    body_mass_kg: 10.4,
                    wheel_mass_kg: 3.2,
                    ..PlantParams::NOMINAL
                },
                seconds: 25.0,
                ..Setup::default()
            },
            tuning(),
        );
        assert!(
            peak_after(&log, 8.0, 3) < 6.0_f32.to_radians(),
            "{}",
            peak_after(&log, 8.0, 3).to_degrees()
        );
        assert!(last(&log, 1).abs() < 0.2);
    }

    #[test]
    fn speed_command_drives_and_stays_upright() {
        // The operator `speed` setpoint is a step. The law ramps it, leans the
        // machine to accelerate, and then tracks it while balancing.
        for command in [0.4_f32, -0.3] {
            let log = simulate(
                Setup {
                    speed_cmd_mps: command,
                    seconds: 25.0,
                    ..Setup::default()
                },
                tuning(),
            );
            let tail = &log[log.len() - 400..];
            let settled = tail.iter().map(|row| row.2).sum::<f32>() / tail.len() as f32;
            assert!(
                (settled - command).abs() < 0.1,
                "command {command}: settled speed {settled}"
            );
            assert!(
                peak_after(&log, 0.0, 3) < 10.0_f32.to_radians(),
                "command {command}: peak pitch {}",
                peak_after(&log, 0.0, 3).to_degrees()
            );
            // Travel follows the sign of the command, not the arming point.
            let travel = last(&log, 1);
            assert!(travel * command > 2.0, "command {command}: travel {travel}");
        }
    }

    #[test]
    fn yaw_error_turns_the_wheels_the_right_way() {
        // A positive yaw-rate error asks for a counter-clockwise turn, which
        // is a positive differential: right wheel forward, left wheel back.
        // This is the bench-verified sign, not the old zero-gain placeholder.
        let t = tuning();
        assert!(t.yaw_kp_units > 0.0, "{}", t.yaw_kp_units);
        let input = BalanceInput {
            yaw_rate_error_rad_s: 0.5,
            ..BalanceInput::default()
        };
        let mut controller = BalanceController::new();
        let output = controller.update(&input, &t, DT);
        assert!(output.right > 0.0, "{}", output.right);
        assert!(output.left < 0.0, "{}", output.left);
        assert!((output.left + output.right).abs() < 1e-3);
    }

    #[test]
    fn yaw_uses_only_the_headroom_balance_leaves() {
        // A saturated balance correction leaves no differential: the yaw loop
        // must not push a wheel past the wire torque limit.
        let input = BalanceInput {
            pitch_rad: 0.5,
            yaw_rate_error_rad_s: 100.0,
            ..BalanceInput::default()
        };
        let mut controller = BalanceController::new();
        let output = controller.update(&input, &tuning(), DT);
        assert!(output.left.abs() <= TORQUE_LIMIT_F32 + 0.5);
        assert!(output.right.abs() <= TORQUE_LIMIT_F32 + 0.5);
    }

    #[test]
    fn pitch_integrator_removes_the_dead_zone_lean() {
        // Without the integral term a proportional-only law parks at
        // `deadband / kp` of lean. The slow integrator must wind that away so
        // the machine holds the captured angle instead of leaning on the
        // drive stiction. A dead-zoned drive always limit-cycles around the
        // hold point, so the test asserts the mean lean over a settled window
        // rather than the instantaneous sample.
        let log = simulate(
            Setup {
                start_pitch_rad: 1.0_f32.to_radians(),
                deadband_nm: 0.5,
                seconds: 40.0,
                ..Setup::default()
            },
            tuning(),
        );
        let tail = &log[log.len() - 2000..];
        let mean_pitch = tail.iter().map(|row| row.3).sum::<f32>() / tail.len() as f32;
        let mean_speed = tail.iter().map(|row| row.2).sum::<f32>() / tail.len() as f32;
        assert!(
            mean_pitch.abs() < 1.0_f32.to_radians(),
            "mean lean {}",
            mean_pitch.to_degrees()
        );
        assert!(mean_speed.abs() < 0.05, "mean speed {mean_speed}");
        assert!(last(&log, 1).abs() < 0.2, "{}", last(&log, 1));
    }

    #[test]
    fn lift_limit_follows_the_torque_budget() {
        let t = tuning();
        // 5 Nm against a 5.39 Nm/rad floor is ~53.1 degrees.
        let weak = Tuning {
            torque_limit_nm: 5.0,
            ..t
        };
        assert!((weak.lift_limit_rad().to_degrees() - 53.1).abs() < 1.5);
        let strong = Tuning {
            torque_limit_nm: 12.0,
            ..t
        };
        assert!(strong.lift_limit_rad() > 1.5);
        // The fall threshold never exceeds the control-design cap.
        assert!((strong.fall_error_rad().to_degrees() - 30.0).abs() < 0.1);
        assert!(weak.fall_error_rad() <= strong.fall_error_rad());
        assert!(weak.fall_error_rad().to_degrees() > 20.0);
    }

    #[test]
    fn fall_threshold_leaves_room_for_the_lift() {
        let t = tuning();
        // The threshold must stay clearly below the static limit, because
        // lifting from rest needs torque to accelerate the body first.
        assert!(t.fall_error_rad() <= 0.75 * t.lift_limit_rad());
        let small = Tuning {
            torque_limit_nm: 3.0,
            ..t
        };
        // 3 Nm lifts about 31.9 degrees, so the threshold is about 22.3.
        assert!((small.fall_error_rad().to_degrees() - 22.3).abs() < 1.0);
    }

    fn safety() -> SafetyState {
        SafetyState {
            pitch_rad: 0.0,
            reference_rad: 0.0,
            position_m: 0.0,
            speed_mps: 0.0,
            feedback_fresh: true,
        }
    }

    #[test]
    fn arm_accepts_a_recoverable_pose() {
        let t = tuning();
        // A level machine with a fresh link and no travel is armable, whatever
        // its recorded travel is: arming resets the origin.
        assert_eq!(safety().arm_refusal(&t), None);
        assert_eq!(
            SafetyState {
                position_m: 99.0,
                ..safety()
            }
            .arm_refusal(&t),
            None
        );
        // Just inside the pose budget is still accepted.
        let inside = tuning().fall_error_rad() * 0.99;
        assert_eq!(
            SafetyState {
                pitch_rad: inside,
                ..safety()
            }
            .arm_refusal(&t),
            None
        );
    }

    #[test]
    fn arm_refuses_a_blind_link() {
        let t = tuning();
        assert_eq!(
            SafetyState {
                feedback_fresh: false,
                ..safety()
            }
            .arm_refusal(&t),
            Some(StopReason::StaleFeedback)
        );
    }

    #[test]
    fn arm_refuses_a_lying_body_and_a_pose_beyond_the_budget() {
        let t = tuning();
        // Forty degrees is outside the lift budget but not yet on the ground.
        assert_eq!(
            SafetyState {
                pitch_rad: 40.0_f32.to_radians(),
                ..safety()
            }
            .arm_refusal(&t),
            Some(StopReason::PoseError)
        );
        // Past the lying limit the body is down and the reason says so.
        assert_eq!(
            SafetyState {
                pitch_rad: 50.0_f32.to_radians(),
                ..safety()
            }
            .arm_refusal(&t),
            Some(StopReason::Lying)
        );
    }

    #[test]
    fn fall_guard_reports_the_first_violation() {
        let t = tuning();
        assert_eq!(safety().fall_cause(&t), None);
        assert_eq!(
            SafetyState {
                position_m: RANGE_LIMIT_M + 0.01,
                ..safety()
            }
            .fall_cause(&t),
            Some(StopReason::Range)
        );
        assert_eq!(
            SafetyState {
                speed_mps: -(FALL_SPEED_MPS + 0.01),
                ..safety()
            }
            .fall_cause(&t),
            Some(StopReason::Speed)
        );
        // The pose checks win over the runaway checks because they describe
        // what actually went wrong.
        assert_eq!(
            SafetyState {
                pitch_rad: 50.0_f32.to_radians(),
                position_m: 10.0,
                speed_mps: 10.0,
                ..safety()
            }
            .fall_cause(&t),
            Some(StopReason::Lying)
        );
    }

    #[test]
    fn fall_guard_uses_pose_error_against_the_given_reference() {
        let t = tuning();
        // The same pose is a fall against one reference and fine against
        // another: this is why the caller passes the target when arming and
        // the applied reference while running.
        let pose = 20.0_f32.to_radians();
        assert_eq!(
            SafetyState {
                pitch_rad: pose,
                reference_rad: -pose,
                ..safety()
            }
            .fall_cause(&t),
            Some(StopReason::PoseError)
        );
        // A reference that is still ramping towards the pose keeps the error
        // inside the threshold, so a transition is not mistaken for a fall.
        let ramping = SafetyState {
            pitch_rad: pose,
            reference_rad: pose - 0.9 * t.fall_error_rad(),
            ..safety()
        };
        assert_eq!(ramping.fall_cause(&t), None);
    }

    #[test]
    fn balance_modes_have_stable_codes() {
        let modes = [
            BalanceMode::Balance,
            BalanceMode::Drive,
            BalanceMode::Spin,
            BalanceMode::Raise,
            BalanceMode::SafeFall,
            BalanceMode::Fault,
        ];
        for mode in modes {
            assert!(!mode.as_str().is_empty());
            assert!(mode.as_str().is_ascii());
            assert_eq!(BalanceMode::from_code(mode.code()), Some(mode));
        }
        assert_eq!(BalanceMode::default(), BalanceMode::Balance);
        assert_eq!(BalanceMode::from_code(6), None);
        assert!(BalanceMode::Raise.grants_guard_grace());
        assert!(BalanceMode::SafeFall.grants_guard_grace());
        assert!(!BalanceMode::Balance.grants_guard_grace());
        assert_eq!(BalanceMode::Balance.default_grace_s(), 0.0);
        assert_eq!(BalanceMode::Raise.default_grace_s(), RAISE_GRACE_S);
        assert_eq!(BalanceMode::SafeFall.default_grace_s(), SAFE_FALL_GRACE_S);
    }

    #[test]
    fn raise_grace_suppresses_a_deliberate_large_pose_error() {
        let t = tuning();
        let tilt = 0.60_f32;
        let state = SafetyState {
            pitch_rad: tilt,
            reference_rad: 0.0,
            ..safety()
        };
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Balance, tilt, 0.0),
            Some(StopReason::PoseError)
        );
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Raise, tilt, RAISE_GRACE_S),
            None
        );
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Raise, tilt, 0.0),
            Some(StopReason::PoseError)
        );
    }

    #[test]
    fn raise_grace_widens_the_lying_limit_but_not_past_a_full_fall() {
        let t = tuning();
        let tilt = 0.90_f32;
        let state = SafetyState {
            pitch_rad: tilt,
            reference_rad: 0.0,
            ..safety()
        };
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Balance, tilt, 0.0),
            Some(StopReason::Lying)
        );
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Raise, tilt, 0.5),
            None
        );
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Raise, LYING_RAD + 0.4, 0.5),
            Some(StopReason::Lying)
        );
    }

    #[test]
    fn guard_uses_its_own_tilt_and_falls_back_safely() {
        let t = tuning();
        let state = SafetyState {
            pitch_rad: 0.0,
            reference_rad: 0.0,
            ..safety()
        };
        assert_eq!(state.fall_cause(&t), None);
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Balance, 0.60, 0.0),
            Some(StopReason::PoseError)
        );
        // A missing guard sample must not disable the guard.
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::Balance, f32::NAN, 0.0),
            None
        );
    }

    #[test]
    fn safe_fall_grace_holds_a_large_intentional_lean() {
        let t = tuning();
        let tilt = 0.60_f32;
        let state = SafetyState {
            pitch_rad: tilt,
            reference_rad: 0.0,
            ..safety()
        };
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::SafeFall, tilt, 0.0),
            Some(StopReason::PoseError)
        );
        assert_eq!(
            state.mode_guard_cause(&t, BalanceMode::SafeFall, tilt, SAFE_FALL_GRACE_S),
            None
        );
        // Grace never suppresses range or speed, which mean the machine ran away.
        assert_eq!(
            SafetyState {
                position_m: RANGE_LIMIT_M + 0.1,
                ..state
            }
            .mode_guard_cause(&t, BalanceMode::SafeFall, tilt, SAFE_FALL_GRACE_S),
            Some(StopReason::Range)
        );
    }

    #[test]
    fn safe_fall_reference_ramps_slowly_from_the_measured_pitch() {
        let mut ramp = SafeFallReference::new(SAFE_FALL_RATE_RAD_S, SAFE_FALL_TARGET_RAD);
        let first = ramp.update(0.20, DT);
        assert!((first - 0.20).abs() < 1e-6, "no jump on entry: {first}");
        let second = ramp.update(0.20, DT);
        assert!(second > first);
        assert!((second - first - SAFE_FALL_RATE_RAD_S * DT).abs() < 1e-6);
        for _ in 0..1000 {
            ramp.update(0.20, DT);
        }
        assert!(ramp.done());
        assert!((ramp.update(0.20, DT) - SAFE_FALL_TARGET_RAD).abs() < 1e-6);
    }

    #[test]
    fn safe_fall_reference_leans_the_way_the_body_falls() {
        let mut ramp = SafeFallReference::new(1.0, SAFE_FALL_TARGET_RAD);
        let first = ramp.update(-0.10, DT);
        assert!(first < 0.0, "negative pitch must ramp negative: {first}");
        for _ in 0..300 {
            ramp.update(-0.10, DT);
        }
        assert!((ramp.update(-0.10, DT) + SAFE_FALL_TARGET_RAD).abs() < 1e-6);
        ramp.reset();
        assert!(!ramp.done());
        let first = ramp.update(0.10, DT);
        assert!(first > 0.0);
    }

    #[test]
    fn safe_fall_reference_ignores_invalid_samples() {
        let mut ramp = SafeFallReference::new(1.0, SAFE_FALL_TARGET_RAD);
        ramp.update(0.10, DT);
        let held = ramp.update(0.10, DT);
        assert_eq!(ramp.update(f32::NAN, DT), held);
        assert_eq!(ramp.update(0.10, 0.0), held);
    }

    #[test]
    fn fade_gain_decays_linearly_to_zero() {
        let mut gain = FadeGain::new(SAFE_FALL_FADE_S);
        assert_eq!(gain.value(), 1.0);
        let first = gain.update(DT);
        assert!((first - (1.0 - DT / SAFE_FALL_FADE_S)).abs() < 1e-6);
        for _ in 0..400 {
            gain.update(DT);
        }
        assert!(gain.done());
        assert_eq!(gain.update(DT), 0.0);
        gain.reset();
        assert_eq!(gain.value(), 1.0);
    }

    #[test]
    fn fade_gain_rejects_invalid_arguments() {
        let mut gain = FadeGain::new(0.0);
        assert_eq!(gain.update(DT), 1.0);
        assert!(!gain.done());
        let mut gain = FadeGain::new(SAFE_FALL_FADE_S);
        assert_eq!(gain.update(f32::NAN), 1.0);
        assert_eq!(gain.update(-DT), 1.0);
    }

    #[test]
    fn stop_reasons_have_stable_labels() {
        let reasons = [
            StopReason::StaleFeedback,
            StopReason::Lying,
            StopReason::PoseError,
            StopReason::Range,
            StopReason::Speed,
            StopReason::SafeFall,
        ];
        for reason in reasons {
            assert!(!reason.as_str().is_empty());
            assert!(reason.as_str().is_ascii());
            // Codes survive a round trip and never collide with "no stop".
            assert_ne!(reason.code(), 0);
            assert_eq!(StopReason::from_code(reason.code()), Some(reason));
        }
        assert_eq!(StopReason::from_code(0), None);
        assert_eq!(StopReason::from_code(42), None);
    }

    #[test]
    fn to_command_maps_physical_pair_through_bench_wiring() {
        // Same-sign physical torque (both wheels roll forward) must reach the
        // wire as a pure `speed` command; the stock mixer then drives both
        // motors the same way.
        let pair = to_command(WheelTorque {
            left: 100.0,
            right: 100.0,
        });
        assert_eq!(pair, Command::new(0, 100));
        // A counter-rolling physical pair (yaw) reaches the wire as a pure
        // `steer` command and pivots the machine.
        let steer = to_command(WheelTorque {
            left: 100.0,
            right: -100.0,
        });
        assert_eq!(steer, Command::new(200, 0));
        // Mixer round trip with the flashed STEER_COEFFICIENT = 0.5:
        // cmdL = speed + steer/2 = left, cmdR = speed - steer/2 = right.
        for (left, right) in [(0, 0), (100, 100), (-150, 200), (37, -91)] {
            let command = to_command(WheelTorque {
                left: left as f32,
                right: right as f32,
            });
            let cmd_left = i32::from(command.speed) + i32::from(command.steer) / 2;
            let cmd_right = i32::from(command.speed) - i32::from(command.steer) / 2;
            assert!((cmd_left - left).abs() <= 1);
            assert!((cmd_right - right).abs() <= 1);
        }
    }

    #[test]
    fn deadband_compensation_is_continuous_at_zero() {
        assert_eq!(deadband_compensation(0.0, 0.5), 0.0);
        assert!((deadband_compensation(0.5, 0.5) - 0.5).abs() < 1e-6);
        assert!((deadband_compensation(-0.5, 0.5) + 0.5).abs() < 1e-6);
        assert!((deadband_compensation(2.0, 0.5) - 0.5).abs() < 1e-6);
        assert_eq!(deadband_compensation(2.0, 0.0), 0.0);
        // Monotone and bounded by the dead zone.
        let mut previous = f32::NEG_INFINITY;
        for step in 0..=200 {
            let value = deadband_compensation(step as f32 * 0.005 - 0.5, 0.5);
            assert!(value >= previous - 1e-6);
            assert!(value.abs() <= 0.5 + 1e-6);
            previous = value;
        }
    }
}
