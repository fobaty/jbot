//! Hardware-independent logic for the JBOT Pico 2 W firmware.
//!
//! Everything in this crate is deterministic, allocation-free, and free of I/O,
//! so it can be unit-tested on the host and reused by the `no_std` firmware.
//! The firmware crate owns the peripheral drivers; this crate owns the wire
//! format, the sensor decoding, and the control math.

#![cfg_attr(not(test), no_std)]

pub mod bno055;
pub mod control;
pub mod eferu;
pub mod filters;
pub mod mount;
pub mod nvm;
pub mod plant;
pub mod reference;
pub mod spin;
