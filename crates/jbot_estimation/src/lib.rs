//! State estimation filters (EKF, Madgwick) for robot posture tracking.

pub struct Estimator {
    filtered_pitch: f32,
}

impl Estimator {
    /// Create a new state estimator instance.
    pub fn new() -> Self {
        Self {
            filtered_pitch: 0.0,
        }
    }

    /// Update state estimate with new IMU and wheel encoder measurements.
    pub fn update(&mut self, raw_pitch: f32, dt: f32) -> f32 {
        // Simple complementary filter placeholder
        const ALPHA: f32 = 0.98;
        self.filtered_pitch =
            ALPHA * (self.filtered_pitch + raw_pitch * dt) + (1.0 - ALPHA) * raw_pitch;
        self.filtered_pitch
    }
}

impl Default for Estimator {
    fn default() -> Self {
        Self::new()
    }
}
