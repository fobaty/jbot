//! Combine inverted-lever calibration recordings into a feedforward table.
//!
//! Each `lever_cal` CSV contains a torque ladder in one direction. This tool
//! groups the quasi-static samples of every level, finds the loaded dead zone
//! and interpolates an angle -> torque table from both directions. The result
//! is written as JSON next to the first input file.
//!
//! Usage: lever-fit <lever_csv> [more_lever_csv ...]

use std::collections::BTreeMap;
use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Default)]
struct Point {
    level: f64,
    pitch_deg: f64,
    rate_dps: f64,
}

fn column(names: &[&str], want: &str) -> Option<usize> {
    names.iter().position(|name| *name == want)
}

fn read_file(path: &Path) -> Option<(Vec<Point>, f64, f64)> {
    let file = File::open(path).ok()?;
    let mut lines = BufReader::new(file).lines();
    let header = lines.next()?.ok()?;
    let names: Vec<&str> = header.split(',').collect();
    let (Some(level_i), Some(pitch_i), Some(phase_i), Some(rate_i)) = (
        column(&names, "level"),
        column(&names, "pitch_deg"),
        column(&names, "phase"),
        column(&names, "gy_dps"),
    ) else {
        eprintln!("{}: missing lever_cal columns", path.display());
        return None;
    };
    let mut by_level: BTreeMap<i32, (f64, f64, usize)> = BTreeMap::new();
    let mut min_pitch = f64::INFINITY;
    let mut max_pitch = f64::NEG_INFINITY;
    for line in lines.map_while(Result::ok) {
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() <= rate_i || (fields[phase_i] != "hold" && fields[phase_i] != "settle") {
            continue;
        }
        let Ok(level) = fields[level_i].parse::<f64>() else {
            continue;
        };
        let Ok(pitch) = fields[pitch_i].parse::<f64>() else {
            continue;
        };
        let Ok(rate) = fields[rate_i].parse::<f64>() else {
            continue;
        };
        if pitch < min_pitch {
            min_pitch = pitch;
        }
        if pitch > max_pitch {
            max_pitch = pitch;
        }
        let entry = by_level
            .entry(level.round() as i32)
            .or_insert((0.0, 0.0, 0));
        entry.0 += pitch;
        entry.1 += rate;
        entry.2 += 1;
    }
    let points: Vec<Point> = by_level
        .into_iter()
        .filter(|(_, (_, _, n))| *n > 0)
        .map(|(level, (pitch_sum, rate_sum, n))| Point {
            level: level as f64,
            pitch_deg: pitch_sum / n as f64,
            rate_dps: rate_sum / n as f64,
        })
        .collect();
    println!("{}: {} levels", path.display(), points.len());
    Some((points, min_pitch, max_pitch))
}

/// Linear interpolation of torque for a requested pitch. `points` is sorted
/// by pitch; repeated levels are averaged by the caller.
fn interpolate(points: &[Point], pitch: f64) -> Option<f64> {
    if points.len() < 2 {
        return None;
    }
    if pitch <= points[0].pitch_deg {
        return Some(points[0].level);
    }
    if pitch >= points[points.len() - 1].pitch_deg {
        return Some(points[points.len() - 1].level);
    }
    for pair in points.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if (a.pitch_deg..=b.pitch_deg).contains(&pitch) {
            let span = b.pitch_deg - a.pitch_deg;
            if span.abs() < 1e-9 {
                return Some((a.level + b.level) / 2.0);
            }
            let t = (pitch - a.pitch_deg) / span;
            return Some(a.level + t * (b.level - a.level));
        }
    }
    None
}

fn main() {
    let paths: Vec<PathBuf> = env::args()
        .skip(1)
        .map(PathBuf::from)
        .filter(|path| path.to_string_lossy() != "--fit-only")
        .collect();
    if paths.is_empty() {
        eprintln!("usage: lever-fit <lever_csv> [more_lever_csv ...]");
        return;
    }
    let mut points = Vec::new();
    let mut min_pitch = f64::INFINITY;
    let mut max_pitch = f64::NEG_INFINITY;
    for path in &paths {
        if let Some((mut file_points, lo, hi)) = read_file(path) {
            points.append(&mut file_points);
            min_pitch = min_pitch.min(lo);
            max_pitch = max_pitch.max(hi);
        }
    }
    if points.len() < 4 {
        eprintln!("not enough lever data");
        return;
    }

    let smallest_level = points
        .iter()
        .map(|p| p.level.abs())
        .fold(f64::INFINITY, f64::min);
    let home_points: Vec<f64> = points
        .iter()
        .filter(|p| (p.level.abs() - smallest_level).abs() < 1e-6)
        .map(|p| p.pitch_deg)
        .collect();
    let home = home_points.iter().sum::<f64>() / home_points.len().max(1) as f64;
    let mut dead_zone = 0.0_f64;
    for p in &points {
        if (p.pitch_deg - home).abs() < 3.0 {
            dead_zone = dead_zone.max(p.level.abs());
        }
    }

    let mut by_pitch: BTreeMap<i32, Vec<f64>> = BTreeMap::new();
    for p in &points {
        // Inside the dead zone, or while the body is still rotating fast, the
        // command does not determine the pitch: those samples must not shape
        // the feedforward curve.
        if (p.pitch_deg - home).abs() <= 5.0 || p.rate_dps.abs() >= 30.0 {
            continue;
        }
        by_pitch
            .entry(p.pitch_deg.round() as i32)
            .or_default()
            .push(p.level);
    }
    let mut curve: Vec<Point> = by_pitch
        .into_iter()
        .map(|(pitch, levels)| Point {
            pitch_deg: pitch as f64,
            level: levels.iter().sum::<f64>() / levels.len() as f64,
            rate_dps: 0.0,
        })
        .collect();
    curve.push(Point {
        pitch_deg: home,
        level: 0.0,
        rate_dps: 0.0,
    });
    curve.sort_by(|a, b| a.pitch_deg.total_cmp(&b.pitch_deg));

    let mut table = String::new();
    for deg in [
        -12.0_f64, -10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0,
    ] {
        if let Some(tau) = interpolate(&curve, deg) {
            table.push_str(&format!(
                "    {{ \"deg\": {deg:.0}, \"units\": {tau:.1} }},\n"
            ));
        }
    }
    table.pop();
    table.pop();

    let max_back = min_pitch;
    let max_face = max_pitch;

    let out = paths[0]
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join("lever-calibration.json");
    let mut file = match File::create(&out) {
        Ok(file) => file,
        Err(error) => {
            eprintln!("cannot create {}: {error}", out.display());
            return;
        }
    };
    let _ = writeln!(file, "{{");
    let _ = writeln!(file, "  \"dead_zone_units\": {dead_zone:.1},");
    let _ = writeln!(file, "  \"pose_back_deg\": {max_back:.1},");
    let _ = writeln!(file, "  \"pose_face_deg\": {max_face:.1},");
    let _ = writeln!(file, "  \"home_deg\": {home:.1},");
    let _ = writeln!(file, "  \"feedforward\": [");
    let _ = writeln!(file, "{table}");
    let _ = writeln!(file, "  ]");
    let _ = writeln!(file, "}}");

    println!(
        "dead zone {dead_zone:.0} units, back {max_back:.1} deg, face {max_face:.1} deg, home {home:.1} deg"
    );
    println!("wrote {}", out.display());
}
