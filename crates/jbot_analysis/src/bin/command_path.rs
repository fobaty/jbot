//! Offline measurement of the command path from recorded console CSVs.
//!
//! The firmware reports both the requested wheel commands (`cwl`/`cwr`, or the
//! explicit `test` words in `ref_hold_cal` CSVs) and the commands the mainboard
//! says it applied (`al`/`ar`). A step in the requested value is followed by a
//! step in the applied value; the gap is the observable command-path delay.
//! This tool reports that delay and its spread so a latency claim is backed by
//! a recording instead of a stopwatch.
//!
//! Usage: command_path <csv> [more_csv ...] [--threshold 5] [--tolerance 2] [--timeout-ms 2000]

use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug)]
struct Sample {
    t_ms: u64,
    requested: f64,
    applied: f64,
}

#[derive(Clone, Copy, Debug)]
struct Step {
    before: f64,
    after: f64,
    t_request_ms: u64,
    delay_ms: Option<f64>,
    tracking_error_units: Option<f64>,
}

#[derive(Clone, Copy, Debug, Default)]
struct Summary {
    steps: usize,
    resolved: usize,
    min_ms: f64,
    mean_ms: f64,
    max_ms: f64,
    jitter_ms: f64,
    max_error_units: f64,
}

fn column(names: &[&str], candidates: &[&str]) -> Option<usize> {
    candidates
        .iter()
        .find_map(|candidate| names.iter().position(|name| name == candidate))
}

fn pair(names: &[&str], left: &[&str], right: &[&str]) -> Option<(usize, usize)> {
    Some((column(names, left)?, column(names, right)?))
}

fn read_file(path: &Path) -> Option<Vec<Sample>> {
    let file = File::open(path).ok()?;
    let mut lines = BufReader::new(file).lines();
    let header = lines.next()?.ok()?;
    let names: Vec<&str> = header.trim_end().split(',').collect();
    let requested = pair(
        &names,
        &["cwl", "cmd_left_units", "test_left_units"],
        &["cwr", "cmd_right_units", "test_right_units"],
    );
    let applied = pair(
        &names,
        &["al", "applied_left_units"],
        &["ar", "applied_right_units"],
    );
    let (Some((req_l, req_r)), Some((app_l, app_r))) = (requested, applied) else {
        eprintln!(
            "{}: need a requested pair (cwl/cwr, cmd_left_units/cmd_right_units or test_left_units/test_right_units) and an applied pair (al/ar)",
            path.display()
        );
        return None;
    };
    let t_i = column(&names, &["t_ms"])?;
    let mut samples = Vec::new();
    for line in lines.map_while(Result::ok) {
        let fields: Vec<&str> = line.trim_end().split(',').collect();
        if fields.len() <= req_r.max(app_r).max(t_i) {
            continue;
        }
        let (Ok(t_ms), Ok(req_left), Ok(req_right), Ok(app_left), Ok(app_right)) = (
            fields[t_i].parse::<u64>(),
            fields[req_l].parse::<f64>(),
            fields[req_r].parse::<f64>(),
            fields[app_l].parse::<f64>(),
            fields[app_r].parse::<f64>(),
        ) else {
            continue;
        };
        samples.push(Sample {
            t_ms,
            requested: (req_left + req_right) / 2.0,
            applied: (app_left + app_right) / 2.0,
        });
    }
    Some(samples)
}

/// A step resolves when the applied command sits inside `tolerance` of the
/// requested value for three consecutive samples. The first of those samples
/// marks the arrival time; requiring a short run avoids a single lucky frame.
fn analyze(samples: &[Sample], threshold: f64, tolerance: f64, timeout_ms: u64) -> Vec<Step> {
    let mut steps = Vec::new();
    let mut baseline = samples.first().map_or(0.0, |sample| sample.requested);
    let mut pending: Option<(u64, f64, usize)> = None;
    for (index, sample) in samples.iter().enumerate() {
        if let Some((t_request, target, consecutive)) = pending {
            if (sample.requested - target).abs() >= threshold {
                steps.push(Step {
                    before: baseline,
                    after: target,
                    t_request_ms: t_request,
                    delay_ms: None,
                    tracking_error_units: None,
                });
                pending = None;
                baseline = sample.requested;
                continue;
            }
            if (sample.applied - target).abs() <= tolerance {
                let consecutive = consecutive + 1;
                if consecutive >= 3 {
                    let first = index + 1 - consecutive;
                    steps.push(Step {
                        before: baseline,
                        after: target,
                        t_request_ms: t_request,
                        delay_ms: Some(samples[first].t_ms as f64 - t_request as f64),
                        tracking_error_units: Some((sample.applied - target).abs()),
                    });
                    baseline = target;
                    pending = None;
                } else {
                    pending = Some((t_request, target, consecutive));
                }
                continue;
            }
            if sample.t_ms.saturating_sub(t_request) > timeout_ms {
                steps.push(Step {
                    before: baseline,
                    after: target,
                    t_request_ms: t_request,
                    delay_ms: None,
                    tracking_error_units: Some((sample.applied - target).abs()),
                });
                baseline = target;
                pending = None;
                continue;
            }
            pending = Some((t_request, target, 0));
            continue;
        }
        if (sample.requested - baseline).abs() >= threshold {
            pending = Some((sample.t_ms, sample.requested, 0));
        }
    }
    if let Some((t_request, target, _)) = pending {
        steps.push(Step {
            before: baseline,
            after: target,
            t_request_ms: t_request,
            delay_ms: None,
            tracking_error_units: None,
        });
    }
    steps
}

fn summarize(steps: &[Step]) -> Summary {
    let delays: Vec<f64> = steps.iter().filter_map(|step| step.delay_ms).collect();
    let mut summary = Summary {
        steps: steps.len(),
        resolved: delays.len(),
        ..Summary::default()
    };
    if let Some(&first) = delays.first() {
        summary.min_ms = delays.iter().fold(first, |min, &delay| min.min(delay));
        summary.max_ms = delays.iter().fold(first, |max, &delay| max.max(delay));
        summary.mean_ms = delays.iter().sum::<f64>() / delays.len() as f64;
        let variance = delays
            .iter()
            .map(|delay| (delay - summary.mean_ms).powi(2))
            .sum::<f64>()
            / delays.len() as f64;
        summary.jitter_ms = variance.sqrt();
    }
    summary.max_error_units = steps
        .iter()
        .filter_map(|step| step.tracking_error_units)
        .fold(0.0, f64::max);
    summary
}

fn main() {
    let mut paths = Vec::new();
    let mut threshold = 5.0_f64;
    let mut tolerance = 2.0_f64;
    let mut timeout_ms = 2000_u64;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--threshold" => {
                if let Some(value) = args.next().and_then(|value| value.parse::<f64>().ok()) {
                    threshold = value.max(0.1);
                }
            }
            "--tolerance" => {
                if let Some(value) = args.next().and_then(|value| value.parse::<f64>().ok()) {
                    tolerance = value.max(0.0);
                }
            }
            "--timeout-ms" => {
                if let Some(value) = args.next().and_then(|value| value.parse::<u64>().ok()) {
                    timeout_ms = value.max(1);
                }
            }
            "--help" | "-h" => {
                println!("usage: command_path <csv> [more_csv ...] [--threshold 5] [--tolerance 2] [--timeout-ms 2000]");
                return;
            }
            other if other.starts_with('-') => {
                eprintln!("unknown option: {other}");
                return;
            }
            other => paths.push(PathBuf::from(other)),
        }
    }
    if paths.is_empty() {
        eprintln!("usage: command_path <csv> [more_csv ...] [--threshold 5] [--tolerance 2]");
        return;
    }
    let mut all_steps = Vec::new();
    for path in &paths {
        let Some(samples) = read_file(path) else {
            continue;
        };
        let steps = analyze(&samples, threshold, tolerance, timeout_ms);
        println!(
            "{}: {} samples, {} command steps",
            path.display(),
            samples.len(),
            steps.len()
        );
        all_steps.extend(steps);
    }
    let summary = summarize(&all_steps);
    println!(
        "resolved {}/{} steps: min {:.0} ms, mean {:.0} ms, max {:.0} ms, jitter (sd) {:.0} ms",
        summary.resolved,
        summary.steps,
        summary.min_ms,
        summary.mean_ms,
        summary.max_ms,
        summary.jitter_ms
    );
    println!(
        "largest applied-vs-requested error at resolution: {:.1} units",
        summary.max_error_units
    );
    for step in &all_steps {
        match step.delay_ms {
            Some(delay) => println!(
                "  t={} ms: {:+.0} -> {:+.0} units, applied after {:.0} ms",
                step.t_request_ms, step.before, step.after, delay
            ),
            None => println!(
                "  t={} ms: {:+.0} -> {:+.0} units, unresolved within {} ms",
                step.t_request_ms, step.before, step.after, timeout_ms
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn samples(entries: &[(u64, f64, f64)]) -> Vec<Sample> {
        entries
            .iter()
            .map(|&(t_ms, requested, applied)| Sample {
                t_ms,
                requested,
                applied,
            })
            .collect()
    }

    #[test]
    fn measures_a_clean_step() {
        let data = samples(&[
            (0, 0.0, 0.0),
            (10, 0.0, 0.0),
            (20, 10.0, 0.0),
            (100, 10.0, 5.0),
            (110, 10.0, 10.0),
            (120, 10.0, 10.0),
            (130, 10.0, 10.0),
        ]);
        let steps = analyze(&data, 5.0, 2.0, 2000);
        assert_eq!(steps.len(), 1);
        assert_eq!(steps[0].t_request_ms, 20);
        assert_eq!(steps[0].delay_ms, Some(90.0));
    }

    #[test]
    fn reports_unresolved_when_applied_never_follows() {
        let data = samples(&[(0, 0.0, 0.0), (10, 20.0, 0.0), (50, 20.0, 0.0)]);
        let steps = analyze(&data, 5.0, 2.0, 20);
        assert_eq!(steps.len(), 1);
        assert_eq!(steps[0].delay_ms, None);
    }

    #[test]
    fn splits_two_steps_instead_of_merging_them() {
        let data = samples(&[
            (0, 0.0, 0.0),
            (10, 10.0, 0.0),
            (20, 10.0, 10.0),
            (30, 10.0, 10.0),
            (40, 10.0, 10.0),
            (50, -10.0, 10.0),
            (60, -10.0, -10.0),
            (70, -10.0, -10.0),
            (80, -10.0, -10.0),
        ]);
        let steps = analyze(&data, 5.0, 2.0, 2000);
        assert_eq!(steps.len(), 2);
        assert_eq!(steps[0].after, 10.0);
        assert_eq!(steps[1].after, -10.0);
    }
}
