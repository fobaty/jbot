//! I/O-free logic for the STM32F103 torque node.
//!
//! This crate is compiled and tested on the host and linked into the no_std
//! firmware. It owns the wire contract, fixed-point control primitives, the
//! safety state machine and the pure FOC transforms. No hardware access here.

#![no_std]
#![forbid(unsafe_code)]

pub mod alerts;
pub mod battery;
pub mod config;
pub mod console;
pub mod diagnostics;
pub mod fixed;
pub mod foc;
pub mod foc_generated;
pub mod mixer;
pub mod monitors;
pub mod motor;
pub mod parameters;
pub mod protocol;
pub mod safety;
pub mod sensing;
pub mod thermal;
pub mod trace;
