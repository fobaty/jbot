//! `jbot-analyze`: offline report for recorded console sessions.
//!
//! Usage: `jbot-analyze [--actions <actions.csv>] <telemetry.csv>...`
//!
//! The analyzer never guesses silently: an axis that cannot be judged is
//! reported as `unknown` with the reason, not smoothed over.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use jbot_analysis::metrics::{self, AxisVerdict, Breakaway, Fit, Oscillation};
use jbot_analysis::session::{self, Segment, Session};

/// EFeru command units per newton-metre at the nominal 5 Nm plant. Reported as
/// an approximation only: the plant has never been identified.
const NOMINAL_UNITS_PER_NM: f64 = 200.0;

fn main() -> ExitCode {
    let mut actions_path: Option<PathBuf> = None;
    let mut files: Vec<PathBuf> = Vec::new();
    let mut arguments = std::env::args().skip(1);

    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "-h" | "--help" => {
                usage();
                return ExitCode::SUCCESS;
            }
            "--actions" => match arguments.next() {
                Some(path) => actions_path = Some(PathBuf::from(path)),
                None => {
                    eprintln!("error: --actions needs a path");
                    return ExitCode::FAILURE;
                }
            },
            other if other.starts_with('-') => {
                eprintln!("error: unknown option {other}");
                usage();
                return ExitCode::FAILURE;
            }
            other => files.push(PathBuf::from(other)),
        }
    }

    if files.is_empty() {
        usage();
        return ExitCode::FAILURE;
    }

    let mut failed = false;
    for (index, file) in files.iter().enumerate() {
        if index > 0 {
            println!();
        }
        if let Err(error) = report(file, actions_path.as_deref()) {
            eprintln!("{}: {error}", file.display());
            failed = true;
        }
    }

    if failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

fn usage() {
    eprintln!("usage: jbot-analyze [--actions <actions.csv>] <telemetry.csv>...");
    eprintln!();
    eprintln!("Reports axis consistency, the recovered pitch law, the drive");
    eprintln!("breakaway torque and the pitch-error ringing of a recording.");
    eprintln!("The actions file is picked up automatically from the telemetry");
    eprintln!("file name; --actions overrides it.");
}

/// Print the report for one telemetry file.
fn report(path: &Path, explicit_actions: Option<&Path>) -> Result<(), session::Error> {
    let session = session::read_session(path)?;
    let summary = metrics::summary(&session);

    println!("{}", path.display());
    println!(
        "  rows {} ({} truncated), {:.1} s, modes 0:{} 1:{} 2:{} 3:{} 4:{}",
        summary.rows,
        session.truncated_rows,
        summary.duration_s,
        summary.mode_counts[0],
        summary.mode_counts[1],
        summary.mode_counts[2],
        summary.mode_counts[3],
        summary.mode_counts[4],
    );
    println!(
        "  armed {:.2} s, estop {:.2} s, falls {}",
        summary.armed_s, summary.estop_s, summary.falls
    );
    println!(
        "  pitch {:.1}..{:.1} deg, roll {:.1}..{:.1} deg, rpm {:.0}..{:.0}, torque {:.0}..{:.0} units",
        summary.pitch_range.0,
        summary.pitch_range.1,
        summary.roll_range.0,
        summary.roll_range.1,
        summary.rpm_range.0,
        summary.rpm_range.1,
        summary.torque_range.0,
        summary.torque_range.1,
    );

    let actions = actions_for(path, explicit_actions)?;
    let segments = match &actions {
        Some(actions) => {
            println!(
                "  actions {} ({} rows)",
                display_path(&actions.0),
                actions.1.len()
            );
            session::mount_segments(&actions.1, session.end_ms())
        }
        None => {
            println!("  actions none (single segment)");
            vec![Segment {
                start_ms: 0,
                end_ms: session.end_ms().saturating_add(1),
                label: "mount (not recorded)".to_owned(),
            }]
        }
    };

    if segments.len() > 1 {
        println!();
        println!("  axis check by segment");
        for segment in &segments {
            let check = metrics::axis_check(segment.samples(&session));
            println!(
                "    {:>7.1}..{:<7.1} s  {:<16}  pitch {:<12} roll {:<12} {}",
                segment.start_ms as f64 / 1000.0,
                segment.end_ms as f64 / 1000.0,
                segment.label,
                check.pitch.verdict().label(),
                check.roll.verdict().label(),
                verdict_hint(check.verdict()),
            );
        }
    }

    println!();
    print_axis_check(&session);
    println!();
    print_law_fit(&session);
    println!();
    print_breakaway(&session);
    println!();
    print_oscillation(&session);

    let mut warnings: Vec<String> = Vec::new();
    if summary.falls > 0 {
        warnings.push(format!(
            "{} fall latch(es): the law stopped itself, check `stop` in the actions file",
            summary.falls
        ));
    }
    if session.truncated_rows > 0 {
        warnings.push(format!(
            "{} truncated row(s): the recorder did not flush before exit",
            session.truncated_rows
        ));
    }
    if summary.armed_s <= 0.0 {
        warnings.push("no armed samples: the pitch law cannot be judged".to_owned());
    }
    if !warnings.is_empty() {
        println!();
        for warning in warnings {
            println!("  note: {warning}");
        }
    }

    Ok(())
}

fn display_path(path: &Path) -> String {
    path.file_name().map_or_else(
        || path.display().to_string(),
        |name| name.to_string_lossy().into(),
    )
}

/// Print the whole-session axis check.
fn print_axis_check(session: &Session) {
    let check = metrics::axis_check(session.samples.iter());
    println!("  axis check (angle derivative against gyro, 1..50 ms gaps)");
    line(
        "pitch",
        "gy",
        "gx",
        check.pitch.direct,
        check.pitch.cross,
        check.pitch.verdict(),
    );
    line(
        "roll",
        "gx",
        "gy",
        check.roll.direct,
        check.roll.cross,
        check.roll.verdict(),
    );
    println!(
        "    verdict: {}{}",
        check.verdict().label(),
        verdict_hint(check.verdict())
    );
}

fn verdict_hint(verdict: AxisVerdict) -> &'static str {
    match verdict {
        AxisVerdict::Consistent => "",
        AxisVerdict::Swapped => " - pitch/roll channels do not belong together",
        AxisVerdict::Inverted => " - angle and rate disagree in sign",
        AxisVerdict::Unknown => " - too little motion to judge",
    }
}

fn line(
    axis: &str,
    own: &str,
    other: &str,
    direct: Option<Fit>,
    cross: Option<Fit>,
    verdict: AxisVerdict,
) {
    match direct {
        Some(fit) => println!(
            "    {axis:<5} d/dt vs {own}: corr {:+.2} slope {:+.2} (n {}), vs {other}: corr {:+.2}  {}",
            fit.corr,
            fit.slope,
            fit.samples,
            cross.map_or(0.0, |fit| fit.corr),
            verdict.label()
        ),
        None => println!(
            "    {axis:<5} d/dt vs {own}: not enough samples  {}",
            verdict.label()
        ),
    }
}

fn print_law_fit(session: &Session) {
    match metrics::law_fit(session.samples.iter()) {
        Some(fit) => {
            println!("  pitch law (armed, unsaturated)");
            println!(
                "    kp {:+.0} units/rad ({:+.1} Nm/rad nominal), kd {:+.0} units/(rad/s) ({:+.2} Nm/(rad/s) nominal)",
                fit.kp_units_per_rad,
                fit.kp_units_per_rad / NOMINAL_UNITS_PER_NM,
                fit.kd_units_per_rad_s,
                fit.kd_units_per_rad_s / NOMINAL_UNITS_PER_NM,
            );
            println!(
                "    R2 {:.4}, residual rms {:.0} units, {} samples, saturated {:.1} % of armed",
                fit.r2,
                fit.rms_units,
                fit.samples,
                fit.saturated_fraction * 100.0,
            );
            println!(
                "    torque limit {:.0} units ({:.2} Nm nominal), sign {}",
                fit.torque_limit_units,
                fit.torque_limit_units / NOMINAL_UNITS_PER_NM,
                if fit.kp_units_per_rad >= 0.0 {
                    "positive (error drives torque)"
                } else {
                    "NEGATIVE (the law fights itself)"
                },
            );
        }
        None => println!("  pitch law: not measurable (not enough armed, unsaturated samples)"),
    }
}

fn print_breakaway(session: &Session) {
    match metrics::breakaway(session.samples.iter()) {
        Some(Breakaway {
            stuck_max_units,
            moving_min_units,
            samples,
        }) => {
            let bracket = if moving_min_units > stuck_max_units {
                "bracketed"
            } else {
                "overlapping (the drive was commanded while rolling)"
            };
            println!("  drive breakaway ({samples} armed samples)");
            println!(
                "    stuck up to {stuck_max_units:.0} units, moving from {moving_min_units:.0} units -> {bracket}",
            );
            println!(
                "    implied dead zone {:.0}..{:.0} units ({:.2}..{:.2} Nm nominal)",
                stuck_max_units,
                moving_min_units.max(stuck_max_units),
                stuck_max_units / NOMINAL_UNITS_PER_NM,
                moving_min_units.max(stuck_max_units) / NOMINAL_UNITS_PER_NM,
            );
        }
        None => println!("  drive breakaway: not measurable (no armed rolling samples)"),
    }
}

fn print_oscillation(session: &Session) {
    match metrics::oscillation(session.samples.iter()) {
        Some(Oscillation {
            duration_s,
            crossings_per_s,
            period_s,
            error_p2p_deg,
        }) => {
            println!("  pitch error while armed ({duration_s:.2} s)");
            match period_s {
                Some(period) => println!(
                    "    p2p {error_p2p_deg:.1} deg, {crossings_per_s:.2} sign changes/s, implied period {period:.2} s",
                ),
                None => println!(
                    "    p2p {error_p2p_deg:.1} deg, {crossings_per_s:.2} sign changes/s (no period: not oscillating)",
                ),
            }
        }
        None => println!("  pitch error: no armed samples"),
    }
}

/// Locate the actions file that belongs to a telemetry file.
fn actions_for(
    telemetry: &Path,
    explicit: Option<&Path>,
) -> Result<Option<(PathBuf, Vec<session::Action>)>, session::Error> {
    let candidate = explicit.map(Path::to_path_buf).or_else(|| {
        let name = telemetry.file_name()?.to_string_lossy();
        if !name.contains("telemetry") {
            return None;
        }
        let sibling = name.replace("telemetry", "actions");
        let path = telemetry.with_file_name(sibling);
        path.exists().then_some(path)
    });
    match candidate {
        Some(path) => Ok(Some((path.clone(), session::read_actions(&path)?))),
        None => Ok(None),
    }
}
