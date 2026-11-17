//! Shared colour palette for the desktop console.
//!
//! The console is a bench instrument, so the palette keeps the chrome dark and
//! uses saturated colours only for live data and safety states.

use eframe::egui::Color32;

/// Application background.
pub const BG: Color32 = Color32::from_rgb(9, 13, 20);
/// Panel and side-bar background.
pub const PANEL: Color32 = Color32::from_rgb(15, 21, 32);
/// Card background.
pub const CARD: Color32 = Color32::from_rgb(21, 29, 43);
/// Slightly lighter card background for nested sections.
pub const CARD_HIGH: Color32 = Color32::from_rgb(27, 37, 54);
/// Border and separator colour.
pub const BORDER: Color32 = Color32::from_rgb(47, 62, 84);
/// Primary text.
pub const TEXT: Color32 = Color32::from_rgb(225, 233, 244);
/// Secondary text and labels.
pub const MUTED: Color32 = Color32::from_rgb(137, 153, 176);
/// Live-data accent.
pub const ACCENT: Color32 = Color32::from_rgb(78, 205, 220);
/// Healthy state.
pub const GREEN: Color32 = Color32::from_rgb(88, 214, 141);
/// Warning state.
pub const AMBER: Color32 = Color32::from_rgb(245, 184, 74);
/// Fault and emergency state.
pub const RED: Color32 = Color32::from_rgb(239, 92, 92);

/// Sky above the horizon.
pub const SKY: Color32 = Color32::from_rgb(36, 91, 142);
/// Ground below the horizon.
pub const GROUND: Color32 = Color32::from_rgb(112, 78, 49);
/// Horizon line and pitch ladder.
pub const HORIZON: Color32 = Color32::from_rgb(240, 246, 255);
/// Fixed aircraft reference symbol.
pub const AIRCRAFT: Color32 = Color32::from_rgb(255, 208, 92);
