//! Offline analysis of recorded JBOT console sessions.
//!
//! The console writes two CSV files per session (`crates/jbot_console`):
//! `jbot-telemetry-<epoch>.csv` with the machine signal and
//! `jbot-actions-<epoch>.csv` with the operator commands. This crate reads
//! them and answers the questions that the bench cannot answer by eye:
//!
//! * is the attitude mapping self-consistent, or are `pitch` and `roll`
//!   reporting the wrong axes (the "lost its axes" failure);
//! * what pitch law did the loop actually run, and how much of it was
//!   saturated;
//! * at what torque does the drive break away from standstill.
//!
//! Everything here is host-side file analysis. Nothing in this crate runs on
//! the robot, and no result here is a substitute for the bench checks in
//! `docs/design_balance_home.md` §8.

pub mod metrics;
pub mod session;

pub use session::{Action, Error, Sample, Segment, Session};

pub mod balancer_model;
pub mod plant;
