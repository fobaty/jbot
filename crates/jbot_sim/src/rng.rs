//! Small deterministic PRNG for reproducible simulation noise.

/// PCG32 generator with a caller-controlled seed and stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pcg32 {
    state: u64,
    increment: u64,
}

impl Pcg32 {
    pub fn new(seed: u64, stream: u64) -> Self {
        let mut generator = Self {
            state: 0,
            increment: (stream << 1) | 1,
        };
        generator.next_u32();
        generator.state = generator.state.wrapping_add(seed);
        generator.next_u32();
        generator
    }

    pub fn from_seed(seed: u64) -> Self {
        Self::new(seed, 0xda3e_39cb_94b9_5bdb)
    }

    pub fn next_u32(&mut self) -> u32 {
        let old_state = self.state;
        self.state = old_state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(self.increment);
        let xorshifted = (((old_state >> 18) ^ old_state) >> 27) as u32;
        let rotation = (old_state >> 59) as u32;
        xorshifted.rotate_right(rotation)
    }

    /// Uniform value in the open interval `(0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u32() as f64 + 0.5) * (1.0 / 4_294_967_296.0)
    }

    /// Standard normal sample with mean zero and unit variance.
    pub fn next_normal(&mut self) -> f64 {
        let first = self.next_f64();
        let second = self.next_f64();
        (-2.0 * first.ln()).sqrt() * (std::f64::consts::TAU * second).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_produces_the_same_sequence() {
        let mut first = Pcg32::from_seed(42);
        let mut second = Pcg32::from_seed(42);

        for _ in 0..1000 {
            assert_eq!(first.next_u32(), second.next_u32());
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut first = Pcg32::from_seed(1);
        let mut second = Pcg32::from_seed(2);

        assert_ne!(first.next_u32(), second.next_u32());
    }

    #[test]
    fn uniform_samples_stay_inside_the_open_unit_interval() {
        let mut generator = Pcg32::from_seed(7);

        for _ in 0..10_000 {
            let value = generator.next_f64();
            assert!(value > 0.0 && value < 1.0, "value {value}");
        }
    }

    #[test]
    fn normal_samples_are_finite() {
        let mut generator = Pcg32::from_seed(11);

        for _ in 0..10_000 {
            assert!(generator.next_normal().is_finite());
        }
    }
}
