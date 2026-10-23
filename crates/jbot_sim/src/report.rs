//! Text and SVG reports generated directly from a simulation trace.

use std::fmt::Write as _;
use std::io::{self, Write};

use crate::wip_sim::{Trace, TraceSample};

const WIDTH: f64 = 900.0;
const PLOT_LEFT: f64 = 70.0;
const PLOT_RIGHT: f64 = 880.0;
const PLOT_HEIGHT: f64 = 150.0;

impl Trace {
    /// Write one CSV row per sample, including the true state and applied torque.
    pub fn write_csv<W: Write>(&self, mut writer: W) -> io::Result<()> {
        writeln!(
            writer,
            "time_s,x_m,x_dot_mps,pitch_rad,pitch_rate_radps,torque_nm,saturated"
        )?;
        for sample in self.samples() {
            writeln!(
                writer,
                "{},{},{},{},{},{},{}",
                sample.time_s,
                sample.state[0],
                sample.state[1],
                sample.state[2],
                sample.state[3],
                sample.torque_nm,
                u8::from(sample.saturated)
            )?;
        }
        Ok(())
    }

    /// Write a self-contained SVG with pitch, velocity, and torque panels.
    pub fn write_svg<W: Write>(&self, mut writer: W) -> io::Result<()> {
        let mut svg = String::new();
        let duration_s = self
            .samples()
            .last()
            .map(|sample| sample.time_s)
            .unwrap_or(0.0);
        let _ = writeln!(
            svg,
            r##"<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="590" viewBox="0 0 {WIDTH} 590">"##
        );
        svg.push_str(r##"<rect width="100%" height="100%" fill="#ffffff"/>"##);
        let _ = writeln!(
            svg,
            r##"<text x="20" y="20" font-family="sans-serif" font-size="16">jbot_sim trace ({duration_s:.3} s)</text>"##
        );

        panel(&mut svg, self.samples(), 30.0, "pitch", "deg", |sample| {
            sample.state[2].to_degrees()
        });
        panel(
            &mut svg,
            self.samples(),
            220.0,
            "velocity",
            "m/s",
            |sample| sample.state[1],
        );
        panel(&mut svg, self.samples(), 410.0, "torque", "N*m", |sample| {
            sample.torque_nm
        });
        svg.push_str("</svg>\n");
        writer.write_all(svg.as_bytes())
    }
}

fn panel(
    svg: &mut String,
    samples: &[TraceSample],
    top: f64,
    title: &str,
    unit: &str,
    value: impl Fn(&TraceSample) -> f64,
) {
    let _ = writeln!(
        svg,
        r##"<rect x="{PLOT_LEFT}" y="{top}" width="{}" height="{PLOT_HEIGHT}" fill="#fafafa" stroke="#888"/>"##,
        PLOT_RIGHT - PLOT_LEFT
    );
    let _ = writeln!(
        svg,
        r##"<text x="{PLOT_LEFT}" y="{}" font-family="sans-serif" font-size="13">{title} [{unit}]</text>"##,
        top - 6.0
    );

    if samples.is_empty() {
        return;
    }

    let limit = samples
        .iter()
        .map(|sample| value(sample).abs())
        .fold(1e-9_f64, f64::max);
    let end_time = samples.last().map(|sample| sample.time_s).unwrap_or(0.0);
    let x_scale = if end_time > 0.0 {
        (PLOT_RIGHT - PLOT_LEFT) / end_time
    } else {
        0.0
    };
    let mid_y = top + PLOT_HEIGHT / 2.0;
    let y_scale = PLOT_HEIGHT / 2.0 / limit;

    let _ = writeln!(
        svg,
        r##"<line x1="{PLOT_LEFT}" y1="{mid_y}" x2="{PLOT_RIGHT}" y2="{mid_y}" stroke="#ddd"/>"##
    );
    let _ = writeln!(
        svg,
        r##"<text x="8" y="{}">{:.3}</text>"##,
        top + 12.0,
        limit
    );
    let _ = writeln!(
        svg,
        r##"<text x="8" y="{}">{:.3}</text>"##,
        top + PLOT_HEIGHT - 4.0,
        -limit
    );

    let stride = (samples.len() / 2000).max(1);
    let points: Vec<String> = samples
        .iter()
        .step_by(stride)
        .map(|sample| {
            let x = PLOT_LEFT + sample.time_s * x_scale;
            let y = mid_y - value(sample) * y_scale;
            format!("{x:.2},{y:.2}")
        })
        .collect();
    let _ = writeln!(
        svg,
        r##"<polyline fill="none" stroke="#1f77b4" stroke-width="1.2" points="{}"/>"##,
        points.join(" ")
    );
}

#[cfg(test)]
mod tests {
    use jbot_control::wip::STANDARD_GRAVITY_MPS2;
    use jbot_control::{StateVector, WheeledInvertedPendulum, WipParameters};

    use super::*;
    use crate::wip_sim::{SimulationConfig, StateFeedback, WipSimulator};

    struct NoTorque;

    impl StateFeedback for NoTorque {
        fn torque(&self, _state: StateVector) -> f64 {
            0.0
        }
    }

    fn trace() -> Trace {
        let plant = WheeledInvertedPendulum::new(WipParameters {
            body_mass_kg: 6.0,
            wheel_mass_kg: 1.8,
            body_pitch_inertia_kg_m2: 0.35,
            wheel_inertia_kg_m2: 0.008,
            wheel_radius_m: 0.09,
            com_height_m: 0.25,
            gravity_mps2: STANDARD_GRAVITY_MPS2,
        })
        .expect("valid plant");
        let config =
            SimulationConfig::new(0.001, 16.0, 30.0_f64.to_radians()).expect("valid config");
        WipSimulator::new(plant, config)
            .expect("valid simulator")
            .run(&NoTorque, StateVector::new(0.0, 1.0, 0.01, 0.0), 0.1, &[])
            .expect("run succeeds")
            .trace
    }

    #[test]
    fn csv_contains_a_header_and_every_sample() {
        let trace = trace();
        let mut buffer = Vec::new();
        trace.write_csv(&mut buffer).expect("csv writes");
        let text = String::from_utf8(buffer).expect("utf8");
        let lines: Vec<&str> = text.lines().collect();

        assert_eq!(
            lines[0],
            "time_s,x_m,x_dot_mps,pitch_rad,pitch_rate_radps,torque_nm,saturated"
        );
        assert_eq!(lines.len(), trace.len() + 1);
    }

    #[test]
    fn svg_contains_the_three_panels() {
        let trace = trace();
        let mut buffer = Vec::new();
        trace.write_svg(&mut buffer).expect("svg writes");
        let text = String::from_utf8(buffer).expect("utf8");

        assert!(text.starts_with("<svg"));
        assert!(text.ends_with("</svg>\n"));
        assert!(text.contains("pitch [deg]"));
        assert!(text.contains("velocity [m/s]"));
        assert!(text.contains("torque [N*m]"));
        assert_eq!(text.matches("<polyline").count(), 3);
    }
}
