//! Sensor freshness handling for the control loop and its watchdog.
//!
//! The BNO055 publishes fused Euler at 100 Hz while the balance task runs at
//! 200 Hz. Feeding the same fused angle into two consecutive steps adds almost
//! half a sample of phase lag to the pitch loop, so the mapped gyro rate carries
//! the propagated angle across the intervening step. The rate is only an
//! integrator between samples: a fresh sample always replaces the propagated
//! value, so fusion drift cannot accumulate.
//!
//! The hardware watchdog is health-gated. A halted control task stops feeding
//! and the board resets, but a benign feedback gap while disarmed must not
//! reset a board that is still sending neutral commands.

/// Propagate a pitch angle between IMU samples, in degrees.
///
/// `fresh_sample` must be true on the tick a new fused sample was received;
/// then the sample wins outright. Otherwise the last mapped gyro rate advances
/// the previous propagated angle by one control period. Freshness of the
/// sample itself is the caller's responsibility: this function never uses a
/// stale rate unless the caller chooses to pass one in.
pub fn propagate_pitch_deg(
    propagated_deg: f32,
    sample_pitch_deg: f32,
    rate_dps: f32,
    dt_s: f32,
    fresh_sample: bool,
) -> f32 {
    if fresh_sample {
        sample_pitch_deg
    } else {
        propagated_deg + rate_dps * dt_s
    }
}

/// Whether a completed control step may feed the hardware watchdog.
///
/// The IMU is the primary loop sensor: without a live attitude there is no
/// meaningful control step to keep alive. Wheel-speed feedback is required
/// only while armed, because the position loop depends on it; during boot and
/// while disarmed a tolerated feedback gap still sends safe neutral commands,
/// so the board must not reset.
pub fn watchdog_feed_allowed(imu_fresh: bool, feedback_fresh: bool, armed: bool) -> bool {
    imu_fresh && (feedback_fresh || !armed)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT_S: f32 = 0.005;

    #[test]
    fn fresh_sample_replaces_the_propagated_pitch() {
        // A nonzero rate must not leak into the sample tick: that would turn
        // the fused angle into a double-integrated estimate.
        assert_eq!(propagate_pitch_deg(12.0, 1.25, -30.0, DT_S, true), 1.25);
    }

    #[test]
    fn intervening_ticks_accumulate_the_mapped_rate() {
        let first = propagate_pitch_deg(1.25, 1.25, -8.0, DT_S, false);
        assert!((first - 1.21).abs() < 1e-6);
        let second = propagate_pitch_deg(first, 1.25, -8.0, DT_S, false);
        assert!((second - 1.17).abs() < 1e-6);
        // A still body holds the last sample; a fresh sample then re-anchors.
        assert_eq!(propagate_pitch_deg(2.5, 1.25, 0.0, DT_S, false), 2.5);
        assert_eq!(propagate_pitch_deg(second, 0.5, -8.0, DT_S, true), 0.5);
    }

    #[test]
    fn watchdog_requires_a_live_imu() {
        assert!(!watchdog_feed_allowed(false, true, true));
        assert!(!watchdog_feed_allowed(false, true, false));
        assert!(!watchdog_feed_allowed(false, false, false));
    }

    #[test]
    fn watchdog_tolerates_a_feedback_gap_while_disarmed() {
        assert!(watchdog_feed_allowed(true, false, false));
        assert!(watchdog_feed_allowed(true, true, false));
    }

    #[test]
    fn watchdog_withholds_when_armed_and_link_is_stale() {
        assert!(!watchdog_feed_allowed(true, false, true));
        assert!(watchdog_feed_allowed(true, true, true));
    }
}
