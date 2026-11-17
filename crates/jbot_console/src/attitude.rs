//! Painting for the attitude indicator, compass, and instrument cards.
//!
//! Everything here is pure rendering: the caller supplies a telemetry snapshot
//! and the widgets draw it. No I/O, no shared state, no allocation in a hot
//! path beyond the strings the labels need.

use eframe::egui::{self, Align2, Color32, FontId, Margin, Pos2, Rect, Sense, Shape, Stroke, Vec2};

use crate::protocol::Telemetry;
use crate::theme;

/// Artificial horizon with pitch ladder and bank scale.
pub fn horizon(ui: &mut egui::Ui, telemetry: Option<&Telemetry>, height: f32, stale: bool) {
    let width = ui.available_width().max(280.0);
    let (rect, _) = ui.allocate_exact_size(egui::vec2(width, height), Sense::hover());
    let chrome = ui.painter_at(rect);
    chrome.rect_filled(rect, egui::CornerRadius::same(14), theme::CARD);
    chrome.rect_stroke(
        rect,
        egui::CornerRadius::same(14),
        Stroke::new(1.0, theme::BORDER),
        egui::StrokeKind::Inside,
    );

    let inner = rect.shrink(10.0);
    let world = ui.painter_at(inner);

    let Some(telemetry) = telemetry else {
        chrome.text(
            inner.center(),
            Align2::CENTER_CENTER,
            "NO TELEMETRY",
            FontId::proportional(20.0),
            theme::MUTED,
        );
        chrome.text(
            inner.center() + egui::vec2(0.0, 26.0),
            Align2::CENTER_CENTER,
            "connect to the Pico or enable demo mode",
            FontId::proportional(12.0),
            theme::MUTED,
        );
        return;
    };

    let pitch = telemetry.pitch_deg.clamp(-90.0, 90.0);
    let roll = if telemetry.has_orientation {
        telemetry.roll_deg
    } else {
        0.0
    };
    let center = inner.center();
    let pixels_per_degree = (inner.height() * 0.5 / 45.0).max(1.0);
    let rotation = -roll.to_radians();
    let horizon_y = center.y + pitch * pixels_per_degree;

    let big = inner.width().max(inner.height()) * 3.0;
    let corner = |x: f32, y: f32| rotate(pos2(x, y), center, rotation);
    let sky = vec![
        corner(center.x - big, horizon_y - big),
        corner(center.x + big, horizon_y - big),
        corner(center.x + big, horizon_y),
        corner(center.x - big, horizon_y),
    ];
    let ground = vec![
        corner(center.x - big, horizon_y),
        corner(center.x + big, horizon_y),
        corner(center.x + big, horizon_y + big),
        corner(center.x - big, horizon_y + big),
    ];
    world.add(Shape::convex_polygon(sky, theme::SKY, Stroke::NONE));
    world.add(Shape::convex_polygon(ground, theme::GROUND, Stroke::NONE));

    // Pitch ladder: positive marks are above the horizon, negative below.
    let half_length = inner.width() * 0.30;
    let gap = inner.width() * 0.055;
    for mark in (-90..=90).step_by(10) {
        if mark == 0 {
            continue;
        }
        let y = center.y + (pitch - mark as f32) * pixels_per_degree;
        if y < inner.top() - 40.0 || y > inner.bottom() + 40.0 {
            continue;
        }
        let length = if mark % 20 == 0 {
            half_length
        } else {
            half_length * 0.58
        };
        let stroke = Stroke::new(1.4, Color32::from_white_alpha(190));
        world.line_segment(
            [corner(center.x - length, y), corner(center.x - gap, y)],
            stroke,
        );
        world.line_segment(
            [corner(center.x + gap, y), corner(center.x + length, y)],
            stroke,
        );
        if mark % 20 == 0 {
            let label_at = corner(center.x - length - 6.0, y);
            world.text(
                label_at,
                Align2::RIGHT_CENTER,
                format!("{mark}"),
                FontId::monospace(11.0),
                Color32::from_white_alpha(210),
            );
            let label_at = corner(center.x + length + 6.0, y);
            world.text(
                label_at,
                Align2::LEFT_CENTER,
                format!("{mark}"),
                FontId::monospace(11.0),
                Color32::from_white_alpha(210),
            );
        }
    }

    // Horizon line.
    world.line_segment(
        [
            corner(center.x - half_length * 1.25, horizon_y),
            corner(center.x + half_length * 1.25, horizon_y),
        ],
        Stroke::new(2.4, theme::HORIZON),
    );

    draw_bank_scale(&world, center, inner, roll);
    draw_aircraft_symbol(&world, center, inner);

    chrome.text(
        inner.left_top() + egui::vec2(10.0, 8.0),
        Align2::LEFT_TOP,
        format!("PITCH {:+.1}°", telemetry.pitch_deg),
        FontId::monospace(12.0),
        Color32::from_white_alpha(225),
    );
    chrome.text(
        inner.right_top() + egui::vec2(-10.0, 8.0),
        Align2::RIGHT_TOP,
        if telemetry.has_orientation {
            format!("ROLL {:+.1}°", telemetry.roll_deg)
        } else {
            "ROLL --".to_owned()
        },
        FontId::monospace(12.0),
        Color32::from_white_alpha(225),
    );
    chrome.text(
        inner.left_bottom() + egui::vec2(10.0, -8.0),
        Align2::LEFT_BOTTOM,
        if telemetry.has_orientation {
            format!("HDG {:03.0}°", normalized_heading(telemetry.heading_deg))
        } else {
            "HDG ---".to_owned()
        },
        FontId::monospace(12.0),
        Color32::from_white_alpha(225),
    );
    chrome.text(
        inner.right_bottom() + egui::vec2(-10.0, -8.0),
        Align2::RIGHT_BOTTOM,
        "BNO055 / IMUPLUS",
        FontId::monospace(11.0),
        Color32::from_white_alpha(170),
    );

    if stale || !telemetry.imu_fresh {
        let badge = Rect::from_center_size(
            inner.center_bottom() + egui::vec2(0.0, -18.0),
            egui::vec2(78.0, 22.0),
        );
        chrome.rect_filled(badge, egui::CornerRadius::same(6), theme::RED);
        chrome.text(
            badge.center(),
            Align2::CENTER_CENTER,
            "STALE",
            FontId::monospace(12.0),
            Color32::WHITE,
        );
    }

    if !telemetry.has_orientation {
        chrome.text(
            inner.center_top() + egui::vec2(0.0, 8.0),
            Align2::CENTER_TOP,
            "ROLL / HEADING NOT REPORTED - UPDATE PICO FIRMWARE",
            FontId::monospace(10.0),
            theme::AMBER,
        );
    }
}

/// Compass rose showing the fused heading.
pub fn compass(ui: &mut egui::Ui, telemetry: Option<&Telemetry>, diameter: f32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(diameter, diameter), Sense::hover());
    let painter = ui.painter_at(rect);
    let center = rect.center();
    let radius = diameter * 0.44;

    painter.circle_filled(center, radius, theme::CARD);
    painter.circle_stroke(center, radius, Stroke::new(1.5, theme::BORDER));

    for degrees in (0..360).step_by(15) {
        let angle = (degrees as f32).to_radians();
        let direction = egui::vec2(angle.sin(), -angle.cos());
        let outer = center + direction * radius;
        let tick_length = if degrees % 45 == 0 { 11.0 } else { 5.0 };
        let inner = center + direction * (radius - tick_length);
        painter.line_segment([inner, outer], Stroke::new(1.0, theme::MUTED));
        if degrees % 90 == 0 {
            let label = match degrees {
                0 => "N",
                90 => "E",
                180 => "S",
                _ => "W",
            };
            painter.text(
                center + direction * (radius - 25.0),
                Align2::CENTER_CENTER,
                label,
                FontId::proportional(13.0),
                theme::TEXT,
            );
        }
    }

    let heading = telemetry.filter(|value| value.has_orientation);
    if let Some(telemetry) = heading {
        let angle = normalized_heading(telemetry.heading_deg).to_radians();
        let direction = egui::vec2(angle.sin(), -angle.cos());
        painter.line_segment(
            [
                center - direction * radius * 0.20,
                center + direction * radius * 0.62,
            ],
            Stroke::new(3.0, theme::ACCENT),
        );
        painter.circle_filled(center + direction * radius * 0.62, 5.0, theme::RED);
        painter.text(
            center,
            Align2::CENTER_CENTER,
            format!("{:03.0}°", normalized_heading(telemetry.heading_deg)),
            FontId::monospace(19.0),
            theme::TEXT,
        );
    } else {
        painter.text(
            center,
            Align2::CENTER_CENTER,
            "--",
            FontId::monospace(22.0),
            theme::MUTED,
        );
    }

    painter.circle_filled(center, 4.0, theme::TEXT);
    painter.text(
        center + egui::vec2(0.0, radius * 0.45),
        Align2::CENTER_CENTER,
        "HEADING",
        FontId::proportional(10.0),
        theme::MUTED,
    );
}

/// Compact instrument card with an arc gauge.
pub fn gauge_card(
    ui: &mut egui::Ui,
    title: &str,
    value: &str,
    subtitle: &str,
    fraction: f32,
    color: Color32,
) {
    egui::Frame::new()
        .fill(theme::CARD)
        .stroke(Stroke::new(1.0, theme::BORDER))
        .corner_radius(egui::CornerRadius::same(10))
        .inner_margin(Margin::same(12))
        .show(ui, |ui| {
            ui.set_min_height(96.0);
            ui.horizontal(|ui| {
                arc_gauge(ui, 72.0, fraction, color);
                ui.add_space(8.0);
                ui.vertical(|ui| {
                    ui.label(
                        egui::RichText::new(title)
                            .size(10.0)
                            .color(theme::MUTED)
                            .strong(),
                    );
                    ui.add_space(2.0);
                    ui.label(
                        egui::RichText::new(value)
                            .size(21.0)
                            .color(theme::TEXT)
                            .strong(),
                    );
                    ui.label(egui::RichText::new(subtitle).size(10.0).color(theme::MUTED));
                });
            });
        });
}

fn arc_gauge(ui: &mut egui::Ui, size: f32, fraction: f32, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(Vec2::splat(size), Sense::hover());
    let painter = ui.painter_at(rect);
    let center = rect.center();
    let radius = size * 0.38;
    let start = 135.0_f32.to_radians();
    let sweep = 270.0_f32.to_radians();
    const STEPS: usize = 48;

    let background: Vec<Pos2> = (0..=STEPS)
        .map(|step| {
            let angle = start + sweep * step as f32 / STEPS as f32;
            center + egui::vec2(angle.cos(), angle.sin()) * radius
        })
        .collect();
    painter.add(Shape::line(
        background,
        Stroke::new(5.0, Color32::from_gray(46)),
    ));

    let fraction = fraction.clamp(0.0, 1.0);
    if fraction > 0.0 {
        let steps = ((STEPS as f32) * fraction).ceil() as usize;
        let value: Vec<Pos2> = (0..=steps)
            .map(|step| {
                let angle = start + sweep * step as f32 / STEPS as f32;
                center + egui::vec2(angle.cos(), angle.sin()) * radius
            })
            .collect();
        painter.add(Shape::line(value, Stroke::new(5.0, color)));
    }

    let needle_angle = start + sweep * fraction;
    let tip = center + egui::vec2(needle_angle.cos(), needle_angle.sin()) * (radius - 7.0);
    painter.line_segment([center, tip], Stroke::new(2.0, theme::TEXT));
    painter.circle_filled(center, 3.0, theme::TEXT);
}

fn draw_bank_scale(painter: &egui::Painter, center: Pos2, inner: Rect, roll: f32) {
    let radius = (inner.height() * 0.44).min(inner.width() * 0.44);
    let top = center + egui::vec2(0.0, -radius);
    let arc: Vec<Pos2> = (0..=24)
        .map(|step| {
            let angle = (-60.0 + 120.0 * step as f32 / 24.0).to_radians();
            rotate(top, center, angle)
        })
        .collect();
    painter.add(Shape::line(
        arc,
        Stroke::new(2.0, Color32::from_white_alpha(150)),
    ));

    for tick in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60] {
        let angle = (tick as f32).to_radians();
        let outer = rotate(top, center, angle);
        let inner_tick = rotate(center + egui::vec2(0.0, -(radius - 9.0)), center, angle);
        painter.line_segment(
            [outer, inner_tick],
            Stroke::new(1.4, Color32::from_white_alpha(180)),
        );
    }

    let roll_rad = roll.to_radians();
    let tip = rotate(center + egui::vec2(0.0, -(radius - 1.0)), center, roll_rad);
    let left = rotate(
        center + egui::vec2(0.0, -(radius - 14.0)),
        center,
        roll_rad - 0.055,
    );
    let right = rotate(
        center + egui::vec2(0.0, -(radius - 14.0)),
        center,
        roll_rad + 0.055,
    );
    painter.add(Shape::convex_polygon(
        vec![tip, left, right],
        theme::AIRCRAFT,
        Stroke::NONE,
    ));
}

fn draw_aircraft_symbol(painter: &egui::Painter, center: Pos2, inner: Rect) {
    let wing = inner.width().min(inner.height()) * 0.17;
    let y = center.y + 9.0;
    let stroke = Stroke::new(4.0, theme::AIRCRAFT);
    painter.line_segment(
        [
            center + egui::vec2(-wing, y),
            center + egui::vec2(-wing * 0.22, y),
        ],
        stroke,
    );
    painter.line_segment(
        [
            center + egui::vec2(wing * 0.22, y),
            center + egui::vec2(wing, y),
        ],
        stroke,
    );
    painter.line_segment(
        [
            center + egui::vec2(0.0, y + 2.0),
            center + egui::vec2(0.0, y + 13.0),
        ],
        Stroke::new(2.5, theme::AIRCRAFT),
    );
    painter.circle_filled(center + egui::vec2(0.0, y), 3.5, theme::AIRCRAFT);
}

fn rotate(point: Pos2, center: Pos2, angle: f32) -> Pos2 {
    let offset = point - center;
    let (sin, cos) = angle.sin_cos();
    center
        + egui::vec2(
            offset.x * cos - offset.y * sin,
            offset.x * sin + offset.y * cos,
        )
}

fn normalized_heading(heading: f32) -> f32 {
    heading.rem_euclid(360.0)
}

fn pos2(x: f32, y: f32) -> Pos2 {
    Pos2::new(x, y)
}
