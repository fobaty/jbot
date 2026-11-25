//! Bench spin test: limited direct wheel torque for wiring and sign checks.
//!
//! This is a bring-up aid, not a drive mode. A requested wheel-torque pair is
//! applied directly, bypassing the balance law, with three guards:
//!
//! * the request is clamped to [`SPIN_TORQUE_LIMIT`];
//! * the applied torque slews toward the target at [`SPIN_SLEW_PER_S`], so the
//!   wheels ramp up instead of jumping;
//! * the request expires [`SPIN_TIMEOUT_MS`] after it was accepted, so a
//!   forgotten request cannot keep the wheels turning.
//!
//! The firmware applies the pair only while the balance law is disarmed and
//! the e-stop is clear; both conditions ramp the applied torque to zero.

/// Highest magnitude of a wheel-torque request, in EFeru command units.
/// Deliberately far below [`crate::eferu::TORQUE_LIMIT`].
pub const SPIN_TORQUE_LIMIT: f32 = 300.0;
/// Maximum rate of change of the applied torque, command units per second.
pub const SPIN_SLEW_PER_S: f32 = 600.0;
/// Shelf life of one spin-test request, in milliseconds.
pub const SPIN_TIMEOUT_MS: u64 = 5_000;

/// One requested wheel-torque pair, in EFeru command units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpinPair {
    /// Left wheel torque request.
    pub left: f32,
    /// Right wheel torque request.
    pub right: f32,
}

/// Clamp a torque request to the supported range. Non-finite values map to zero.
fn clamp_request(value: f32) -> f32 {
    if value.is_finite() {
        value.clamp(-SPIN_TORQUE_LIMIT, SPIN_TORQUE_LIMIT)
    } else {
        0.0
    }
}

/// Slew `applied` toward `target` by at most `max_delta`.
fn slew(applied: f32, target: f32, max_delta: f32) -> f32 {
    let delta = target - applied;
    if delta.abs() <= max_delta {
        target
    } else {
        applied + delta.signum() * max_delta
    }
}

/// Spin-test state machine. One step is one control-loop tick.
#[derive(Clone, Copy, Debug)]
pub struct SpinTest {
    target_left: f32,
    target_right: f32,
    applied_left: f32,
    applied_right: f32,
    /// Monotonic millisecond instant at which the current request expires.
    expire_at_ms: u64,
    /// True between an accepted request and its expiry or a stop.
    request_active: bool,
}

impl SpinTest {
    /// A spin test with nothing applied.
    pub const fn new() -> Self {
        Self {
            target_left: 0.0,
            target_right: 0.0,
            applied_left: 0.0,
            applied_right: 0.0,
            expire_at_ms: 0,
            request_active: false,
        }
    }

    /// Accept a new request. `now_ms` is monotonic milliseconds.
    /// Clamps each side and starts a fresh [`SPIN_TIMEOUT_MS`] window.
    pub fn request(&mut self, left: f32, right: f32, now_ms: u64) {
        self.target_left = clamp_request(left);
        self.target_right = clamp_request(right);
        self.expire_at_ms = now_ms.saturating_add(SPIN_TIMEOUT_MS);
        self.request_active = true;
    }

    /// Stop the test: targets go to zero immediately, applied torque slews down.
    pub fn stop(&mut self) {
        self.target_left = 0.0;
        self.target_right = 0.0;
        self.request_active = false;
    }

    /// The wheel-torque pair applied on this tick.
    pub fn applied(&self) -> SpinPair {
        SpinPair {
            left: self.applied_left,
            right: self.applied_right,
        }
    }

    /// Advance one tick.
    ///
    /// `balance_armed` and `estop` force the targets to zero; `estop` also
    /// zeroes the applied torque immediately. Otherwise the applied torque
    /// slews toward the target, and an expired request is dropped first.
    pub fn step(&mut self, dt_s: f32, now_ms: u64, balance_armed: bool, estop: bool) -> SpinPair {
        if estop {
            self.target_left = 0.0;
            self.target_right = 0.0;
            self.applied_left = 0.0;
            self.applied_right = 0.0;
            self.request_active = false;
            return self.applied();
        }
        if balance_armed {
            // Ramp down smoothly: the balance law owns the wheels now.
            self.target_left = 0.0;
            self.target_right = 0.0;
        } else if self.request_active && now_ms >= self.expire_at_ms {
            self.target_left = 0.0;
            self.target_right = 0.0;
            self.request_active = false;
        }

        let max_delta = SPIN_SLEW_PER_S * dt_s.max(0.0);
        self.applied_left = slew(self.applied_left, self.target_left, max_delta);
        self.applied_right = slew(self.applied_right, self.target_right, max_delta);
        self.applied()
    }
}

impl Default for SpinTest {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.005;

    #[test]
    fn a_new_test_applies_nothing() {
        let mut spin = SpinTest::new();
        let applied = spin.step(DT, 0, false, false);
        assert_eq!(
            applied,
            SpinPair {
                left: 0.0,
                right: 0.0
            }
        );
    }

    #[test]
    fn requests_are_clamped_to_the_limit() {
        let mut spin = SpinTest::new();
        spin.request(10_000.0, -10_000.0, 0);
        spin.stop(); // targets clamped on the way in
        spin.request(10_000.0, -10_000.0, 0);
        for _ in 0..10_000 {
            spin.step(DT, 0, false, false);
        }
        let applied = spin.applied();
        assert_eq!(applied.left, SPIN_TORQUE_LIMIT);
        assert_eq!(applied.right, -SPIN_TORQUE_LIMIT);
    }

    #[test]
    fn non_finite_requests_map_to_zero() {
        let mut spin = SpinTest::new();
        spin.request(f32::NAN, f32::INFINITY, 0);
        for _ in 0..10_000 {
            spin.step(DT, 0, false, false);
        }
        assert_eq!(
            spin.applied(),
            SpinPair {
                left: 0.0,
                right: 0.0
            }
        );
    }

    #[test]
    fn applied_torque_ramps_instead_of_jumping() {
        let mut spin = SpinTest::new();
        spin.request(SPIN_TORQUE_LIMIT, SPIN_TORQUE_LIMIT, 0);
        let first = spin.step(DT, 0, false, false);
        // One tick moves at most SPIN_SLEW_PER_S * DT.
        assert!(first.left > 0.0 && first.left <= SPIN_SLEW_PER_S * DT + 1e-6);
        // After a second it must have arrived.
        for step in 1..200 {
            spin.step(DT, step * 5, false, false);
        }
        assert_eq!(spin.applied().left, SPIN_TORQUE_LIMIT);
    }

    #[test]
    fn a_request_expires_after_the_timeout() {
        let mut spin = SpinTest::new();
        spin.request(200.0, 200.0, 1_000);
        // Spin up fully.
        for step in 0..400 {
            spin.step(DT, 1_000 + step * 5, false, false);
        }
        assert_eq!(spin.applied().left, 200.0);
        // Past the timeout the applied torque slews back to zero.
        for step in 0..400 {
            spin.step(DT, 1_000 + SPIN_TIMEOUT_MS + step * 5, false, false);
        }
        assert_eq!(spin.applied().left, 0.0);
    }

    #[test]
    fn stop_ramps_the_request_to_zero() {
        let mut spin = SpinTest::new();
        spin.request(200.0, 200.0, 0);
        for step in 0..400 {
            spin.step(DT, step * 5, false, false);
        }
        spin.stop();
        for step in 0..400 {
            spin.step(DT, 400 * 5 + step * 5, false, false);
        }
        assert_eq!(
            spin.applied(),
            SpinPair {
                left: 0.0,
                right: 0.0
            }
        );
    }

    #[test]
    fn estop_zeroes_applied_torque_immediately() {
        let mut spin = SpinTest::new();
        spin.request(200.0, 200.0, 0);
        spin.step(DT, 0, false, false);
        let applied = spin.step(DT, 5, false, true);
        assert_eq!(
            applied,
            SpinPair {
                left: 0.0,
                right: 0.0
            }
        );
    }

    #[test]
    fn arming_the_balance_law_ramps_the_spin_test_down() {
        let mut spin = SpinTest::new();
        spin.request(200.0, 200.0, 0);
        for step in 0..400 {
            spin.step(DT, step * 5, false, false);
        }
        assert_eq!(spin.applied().left, 200.0);
        for step in 0..400 {
            spin.step(DT, 2_000 + step * 5, true, false);
        }
        assert_eq!(
            spin.applied(),
            SpinPair {
                left: 0.0,
                right: 0.0
            }
        );
    }
}
