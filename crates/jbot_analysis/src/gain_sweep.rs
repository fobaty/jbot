//! Host-only gain sweep for the balance loop.
//!
//! This is a design aid, not a substitute for HIL. It reproduces the dominant
//! effects measured on the bench: the linearized wheeled-pendulum plant, the
//! proportional/derivative pitch law with `pscale`/`dscale`, a quantized IMU
//! delay, the drive dead zone, the command slew limiter and the torque budget.
//! It intentionally leaves out the outer position/speed loops so the pitch
//! candidate is not contaminated by a second tuning problem.

use crate::recovery_sim::{Geometry, CONTROL_DT_S};

/// One candidate profile and its simulated response to a 3 degree reference step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Candidate {
    pub pscale: f32,
    pub dscale: f32,
    pub kp_nm_per_rad: f32,
    pub kd_nm_per_rad_s: f32,
    pub final_pitch_deg: f32,
    pub peak_pitch_deg: f32,
    pub settle_s: f32,
    pub peak_torque_nm: f32,
}

fn clamp(value: f32, limit: f32) -> f32 {
    value.clamp(-limit, limit)
}

/// Smooth per-wheel breakaway inverse, mirrored from `jbot_pico_core::calibration`.
fn compensate(command: f32, deadband: f32) -> f32 {
    if command == 0.0 || deadband <= 0.0 {
        return command;
    }
    let magnitude = command.abs();
    let blend = 0.06_f32; // 10 command units at 175 units/Nm
    let added = if magnitude >= blend {
        deadband
    } else {
        deadband * magnitude / blend
    };
    command + added.copysign(command)
}

/// Simulate one reference step and report the response.
pub fn evaluate(
    geometry: &Geometry,
    pscale: f32,
    dscale: f32,
    delay_ticks: usize,
    deadband_nm: f32,
    slew_units_per_s: f32,
    step_deg: f32,
) -> Candidate {
    let dt = CONTROL_DT_S;
    let floor = geometry.floor_nm_per_rad();
    let kp = 4.0 * floor * pscale;
    let stiffness = (geometry.a2() * kp - geometry.a1()).max(1e-3);
    let kd = 2.0 * 1.6 * stiffness.sqrt() / geometry.a2() * dscale;

    let budget = geometry.max_torque_nm;
    let units_per_nm = 1000.0_f32 / budget.max(1e-3);
    let mut pitch = 0.0_f32;
    let mut rate = 0.0_f32;
    let mut integral_nm = 0.0_f32;
    let mut applied_units = 0.0_f32;
    let mut seen: Vec<(f32, f32)> = Vec::new();
    let mut torque_history: Vec<f32> = Vec::new();
    let mut peak_pitch = 0.0_f32;
    let mut peak_torque = 0.0_f32;
    let steps: usize = 1_600;
    let step = step_deg.to_radians();
    let mut settled_for = 0.0_f32;
    let mut settle_at = 0.0_f32;

    for index in 0..steps {
        seen.push((pitch, rate));
        let (seen_pitch, seen_rate) = seen[index.saturating_sub(delay_ticks)];
        // The law drives the wheels to move under a lean: torque has the same
        // sign as the lean, matching `control.rs`.
        let error = seen_pitch - step;
        // Mirrors the firmware's pitch integrator: a decade below the P loop,
        // clamped so it removes the dead-zone lean without pumping the stance.
        integral_nm = clamp(integral_nm + (kp / 20.0) * error * dt, 1.0);
        let command_nm = clamp(kp * error + kd * seen_rate + integral_nm, budget);
        let target_units = compensate(command_nm, deadband_nm) * units_per_nm;
        let max_delta = slew_units_per_s * dt;
        applied_units += clamp(target_units - applied_units, max_delta);
        let applied_nm = applied_units / units_per_nm;
        torque_history.push(applied_nm);
        peak_torque = peak_torque.max(applied_nm.abs());
        let delayed = torque_history[index.saturating_sub(2)];
        let (pitch_acc, _) = (
            geometry.a1() * pitch - geometry.a2() * delayed,
            geometry.b1() * delayed - geometry.b2() * pitch,
        );
        rate += pitch_acc * dt;
        pitch += rate * dt;
        peak_pitch = peak_pitch.max(pitch.abs());
        if (pitch - step).abs() <= 0.5_f32.to_radians() && rate.abs() <= 0.05 {
            settled_for += dt;
            if settled_for >= 0.5 && settle_at == 0.0 {
                settle_at = index as f32 * dt;
            }
        } else {
            settled_for = 0.0;
        }
    }

    Candidate {
        pscale,
        dscale,
        kp_nm_per_rad: kp,
        kd_nm_per_rad_s: kd,
        final_pitch_deg: pitch.to_degrees(),
        peak_pitch_deg: peak_pitch.to_degrees(),
        settle_s: settle_at,
        peak_torque_nm: peak_torque,
    }
}

/// Sweep `pscale` and `dscale`, returning candidates sorted by settling time
/// and peak excursion. A zero settle time means the step was not recovered.
pub fn sweep(
    geometry: &Geometry,
    delay_ticks: usize,
    deadband_nm: f32,
    slew_units_per_s: f32,
    step_deg: f32,
) -> Vec<Candidate> {
    let mut result = Vec::new();
    for p_index in 3..=12 {
        let pscale = p_index as f32 / 10.0;
        for d_index in 1..=10 {
            let dscale = d_index as f32 / 10.0;
            result.push(evaluate(
                geometry,
                pscale,
                dscale,
                delay_ticks,
                deadband_nm,
                slew_units_per_s,
                step_deg,
            ));
        }
    }
    result.sort_by(|a, b| {
        let a_ok = a.settle_s > 0.0;
        let b_ok = b.settle_s > 0.0;
        b_ok.cmp(&a_ok)
            .then_with(|| a.settle_s.total_cmp(&b.settle_s))
            .then_with(|| a.peak_pitch_deg.total_cmp(&b.peak_pitch_deg))
    });
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dead_zone_inverse_is_continuous() {
        assert_eq!(compensate(0.0, 0.2), 0.0);
        assert!((compensate(1.0, 0.2) - 1.2).abs() < 1e-6);
        assert!((compensate(-1.0, 0.2) + 1.2).abs() < 1e-6);
    }

    #[test]
    fn sweep_prefers_a_settled_profile() {
        let candidates = sweep(&Geometry::default(), 10, 0.2, 1500.0, 3.0);
        assert!(!candidates.is_empty());
        assert!(candidates[0].settle_s > 0.0);
        assert!(candidates[0].peak_pitch_deg < 30.0);
    }
}
