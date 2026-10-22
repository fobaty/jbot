//! Navigation stack integration, local planners, and costmaps.

pub struct PathPlanner {
    target_velocity: f32,
}

impl PathPlanner {
    /// Create a new path planner instance.
    pub fn new() -> Self {
        Self {
            target_velocity: 0.0,
        }
    }

    /// Set desired linear velocity.
    pub fn set_target(&mut self, velocity: f32) {
        self.target_velocity = velocity;
    }
}

impl Default for PathPlanner {
    fn default() -> Self {
        Self::new()
    }
}
