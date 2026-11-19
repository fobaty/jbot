//! Reading the CSV files written by the console recorder.
//!
//! The formats are owned by `crates/jbot_console/src/record.rs`; this module
//! parses them by column *name*, so a client that adds a column does not break
//! the analyzer, and a client that renames one fails loudly instead of
//! silently reading the wrong field.

use std::fmt;
use std::path::Path;

/// Columns the analyzer cannot work without.
const REQUIRED_COLUMNS: [&str; 16] = [
    "t_ms",
    "step",
    "loop_hz",
    "imu_fresh",
    "pitch_deg",
    "roll_deg",
    "heading_deg",
    "gy_dps",
    "gx_dps",
    "gz_dps",
    "left_rpm",
    "right_rpm",
    "bat_v",
    "age_ms",
    "estop",
    "torque_units",
];

/// Columns that later firmware added; a recording without them is still
/// readable, with the fields defaulted to zero.
const OPTIONAL_COLUMNS: [&str; 9] = [
    "vset",
    "wset",
    "test_left_units",
    "test_right_units",
    "ref_deg",
    "demand_deg",
    "trim_deg",
    "position_m",
    "mode",
];

/// Balance-mode value the firmware reports while disarmed.
pub const MODE_DISARMED: u8 = 0;
/// Balance-mode value reported after a latched fall.
pub const MODE_FALL: u8 = 4;

/// One telemetry sample, in the units the recorder writes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sample {
    /// Milliseconds since the recording started.
    pub t_ms: u64,
    /// Control ticks since the firmware booted.
    pub step: u64,
    /// Measured control-loop rate, hertz.
    pub loop_hz: f64,
    /// The attitude sample used for this tick was fresh.
    pub imu_fresh: bool,
    /// Zeroed pitch, degrees.
    pub pitch_deg: f64,
    /// Zeroed roll, degrees.
    pub roll_deg: f64,
    /// Raw heading, degrees.
    pub heading_deg: f64,
    /// Pitch rate used by the law, degrees per second.
    pub gy_dps: f64,
    /// Roll rate, degrees per second.
    pub gx_dps: f64,
    /// Yaw rate, degrees per second.
    pub gz_dps: f64,
    /// Left wheel speed, rpm.
    pub left_rpm: f64,
    /// Right wheel speed, rpm.
    pub right_rpm: f64,
    /// Pack voltage, volts.
    pub bat_v: f64,
    /// Age of the last mainboard feedback frame, milliseconds.
    pub age_ms: f64,
    /// Emergency stop latch.
    pub estop: bool,
    /// Speed setpoint, metres per second.
    pub vset: f64,
    /// Steering setpoint, radians per second.
    pub wset: f64,
    /// Applied balance-law common-mode torque, EFeru units.
    pub torque_units: f64,
    /// Applied spin-test torque, left wheel, EFeru units.
    pub test_left_units: f64,
    /// Applied spin-test torque, right wheel, EFeru units.
    pub test_right_units: f64,
    /// Applied (ramped) pitch reference, degrees.
    pub ref_deg: f64,
    /// Outer-loop pitch demand, degrees.
    pub demand_deg: f64,
    /// Learned slope trim, degrees.
    pub trim_deg: f64,
    /// Travel since arming, metres.
    pub position_m: f64,
    /// Balance mode: 0 disarmed, 1 transit, 2 hold, 3 recover, 4 fall.
    pub mode: u8,
}

impl Sample {
    /// Signed pitch error the law acts on, degrees.
    pub fn error_deg(&self) -> f64 {
        self.pitch_deg - self.ref_deg - self.demand_deg
    }

    /// Mean of the two wheel speeds, rpm.
    pub fn mean_rpm(&self) -> f64 {
        0.5 * (self.left_rpm + self.right_rpm)
    }

    /// True while the balance law was driving an armed, non-fallen machine.
    pub fn armed(&self) -> bool {
        (1..=3).contains(&self.mode)
    }

    /// True while a bench spin test was applying torque.
    pub fn spin_test(&self) -> bool {
        self.test_left_units != 0.0 || self.test_right_units != 0.0
    }
}

/// A parsed telemetry file.
#[derive(Debug, Clone, Default)]
pub struct Session {
    /// Samples in file order.
    pub samples: Vec<Sample>,
    /// Rows dropped because the file ended mid-line.
    pub truncated_rows: usize,
    /// Columns the recording does not carry, in schema order.
    ///
    /// Sessions recorded before the pitch reference existed (`ref_deg`,
    /// `demand_deg`, `trim_deg`, `position_m`, `mode`) are still readable;
    /// the fields default to zero and the report says which sections cannot
    /// run.
    pub missing: Vec<&'static str>,
}

impl Session {
    /// Last timestamp in the file, milliseconds.
    pub fn end_ms(&self) -> u64 {
        self.samples.last().map_or(0, |sample| sample.t_ms)
    }
}

/// One operator action from the actions file.
#[derive(Debug, Clone, PartialEq)]
pub struct Action {
    /// Milliseconds since the recording started.
    pub t_ms: u64,
    /// `tx` for a command sent to the firmware, `rx` for a reply.
    pub kind: String,
    /// Command text, or the reply line.
    pub detail: String,
}

/// A contiguous slice of a session.
#[derive(Debug, Clone, PartialEq)]
pub struct Segment {
    /// First timestamp, milliseconds.
    pub start_ms: u64,
    /// End timestamp, milliseconds, exclusive.
    pub end_ms: u64,
    /// Human label, for example `mount rotyp`.
    pub label: String,
}

impl Segment {
    /// Samples of `session` that fall inside this segment.
    pub fn samples<'a>(&self, session: &'a Session) -> impl Iterator<Item = &'a Sample> {
        let start_ms = self.start_ms;
        let end_ms = self.end_ms;
        session
            .samples
            .iter()
            .filter(move |sample| sample.t_ms >= start_ms && sample.t_ms < end_ms)
    }
}

/// Why a file could not be read.
#[derive(Debug)]
pub enum Error {
    /// The filesystem call failed.
    Io(std::io::Error),
    /// The content does not match the recorder format.
    Parse {
        /// 1-based line number.
        line: usize,
        /// What is wrong with it.
        message: String,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "{error}"),
            Self::Parse { line, message } => write!(formatter, "line {line}: {message}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error)
    }
}

impl Error {
    fn parse(line: usize, message: impl Into<String>) -> Self {
        Self::Parse {
            line,
            message: message.into(),
        }
    }
}

/// Read and parse a telemetry file.
pub fn read_session(path: &Path) -> Result<Session, Error> {
    parse_session(&std::fs::read_to_string(path)?)
}

/// Read and parse an actions file.
pub fn read_actions(path: &Path) -> Result<Vec<Action>, Error> {
    parse_actions(&std::fs::read_to_string(path)?)
}

/// Parse telemetry CSV text.
pub fn parse_session(text: &str) -> Result<Session, Error> {
    let mut lines = text.lines().enumerate();
    let header = lines.next().ok_or_else(|| Error::parse(1, "empty file"))?.1;
    let header = header.trim_start_matches('\u{feff}');
    let names: Vec<&str> = header.split(',').map(str::trim).collect();
    let columns = Columns::resolve(&names)?;

    let mut session = Session {
        missing: OPTIONAL_COLUMNS
            .iter()
            .filter(|name| !names.contains(name))
            .copied()
            .collect(),
        ..Session::default()
    };
    for (index, line) in lines {
        let number = index + 1;
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() < names.len() {
            // The recorder buffers rows; a power cut leaves a short last line.
            session.truncated_rows += 1;
            continue;
        }
        session.samples.push(columns.sample(&fields, number)?);
    }
    Ok(session)
}

/// Parse actions CSV text.
pub fn parse_actions(text: &str) -> Result<Vec<Action>, Error> {
    let mut lines = text.lines().enumerate();
    let header = lines.next().ok_or_else(|| Error::parse(1, "empty file"))?.1;
    let header = header.trim_start_matches('\u{feff}');
    let names: Vec<&str> = header.split(',').map(str::trim).collect();
    let t_ms = position(&names, "t_ms")?;
    let kind = position(&names, "kind")?;
    let detail = position(&names, "detail")?;

    let mut actions = Vec::new();
    for (index, line) in lines {
        let number = index + 1;
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() < names.len() {
            continue;
        }
        let Ok(t_ms) = fields[t_ms].trim().parse::<u64>() else {
            return Err(Error::parse(number, "t_ms is not an integer"));
        };
        actions.push(Action {
            t_ms,
            kind: fields[kind].trim().to_owned(),
            detail: fields[detail].trim().to_owned(),
        });
    }
    Ok(actions)
}

/// Split a session at every `mount` command in the actions file.
///
/// The mapping is the parameter that invalidates an axis check, so a session
/// that swept mounts 0..5 has to be judged per segment. Without any `mount`
/// command the whole session is one segment labelled `mount (not recorded)`.
pub fn mount_segments(actions: &[Action], end_ms: u64) -> Vec<Segment> {
    let mut cuts: Vec<(u64, String)> = actions
        .iter()
        .filter(|action| action.kind == "tx")
        .filter_map(|action| {
            let argument = action.detail.strip_prefix("mount ")?.trim();
            Some((action.t_ms, format!("mount {argument}")))
        })
        .collect();
    cuts.sort_by_key(|(t_ms, _)| *t_ms);

    let mut segments = Vec::with_capacity(cuts.len() + 1);
    let mut start = 0;
    let mut label = String::from("mount (not recorded)");
    for (t_ms, next) in cuts {
        if t_ms > start {
            segments.push(Segment {
                start_ms: start,
                end_ms: t_ms,
                label: label.clone(),
            });
        }
        start = t_ms;
        label = next;
    }
    segments.push(Segment {
        start_ms: start,
        end_ms: end_ms.saturating_add(1),
        label,
    });
    segments
}

/// Resolved column positions.
///
/// The first block is required: without it nothing can be analysed. The
/// second block is optional and defaults to zero, so a recording from before
/// the pitch reference existed still yields an axis and motion report.
struct Columns {
    t_ms: usize,
    step: usize,
    loop_hz: usize,
    imu_fresh: usize,
    pitch_deg: usize,
    roll_deg: usize,
    heading_deg: usize,
    gy_dps: usize,
    gx_dps: usize,
    gz_dps: usize,
    left_rpm: usize,
    right_rpm: usize,
    bat_v: usize,
    age_ms: usize,
    estop: usize,
    vset: Option<usize>,
    wset: Option<usize>,
    torque_units: usize,
    test_left_units: Option<usize>,
    test_right_units: Option<usize>,
    ref_deg: Option<usize>,
    demand_deg: Option<usize>,
    trim_deg: Option<usize>,
    position_m: Option<usize>,
    mode: Option<usize>,
}

impl Columns {
    fn resolve(names: &[&str]) -> Result<Self, Error> {
        for required in REQUIRED_COLUMNS {
            position(names, required)?;
        }
        Ok(Self {
            t_ms: position(names, "t_ms")?,
            step: position(names, "step")?,
            loop_hz: position(names, "loop_hz")?,
            imu_fresh: position(names, "imu_fresh")?,
            pitch_deg: position(names, "pitch_deg")?,
            roll_deg: position(names, "roll_deg")?,
            heading_deg: position(names, "heading_deg")?,
            gy_dps: position(names, "gy_dps")?,
            gx_dps: position(names, "gx_dps")?,
            gz_dps: position(names, "gz_dps")?,
            left_rpm: position(names, "left_rpm")?,
            right_rpm: position(names, "right_rpm")?,
            bat_v: position(names, "bat_v")?,
            age_ms: position(names, "age_ms")?,
            estop: position(names, "estop")?,
            vset: optional(names, "vset"),
            wset: optional(names, "wset"),
            torque_units: position(names, "torque_units")?,
            test_left_units: optional(names, "test_left_units"),
            test_right_units: optional(names, "test_right_units"),
            ref_deg: optional(names, "ref_deg"),
            demand_deg: optional(names, "demand_deg"),
            trim_deg: optional(names, "trim_deg"),
            position_m: optional(names, "position_m"),
            mode: optional(names, "mode"),
        })
    }

    fn sample(&self, fields: &[&str], line: usize) -> Result<Sample, Error> {
        Ok(Sample {
            t_ms: unsigned(fields, self.t_ms, "t_ms", line)?,
            step: unsigned(fields, self.step, "step", line)?,
            loop_hz: number(fields, self.loop_hz, "loop_hz", line)?,
            imu_fresh: flag(fields, self.imu_fresh, "imu_fresh", line)?,
            pitch_deg: number(fields, self.pitch_deg, "pitch_deg", line)?,
            roll_deg: number(fields, self.roll_deg, "roll_deg", line)?,
            heading_deg: number(fields, self.heading_deg, "heading_deg", line)?,
            gy_dps: number(fields, self.gy_dps, "gy_dps", line)?,
            gx_dps: number(fields, self.gx_dps, "gx_dps", line)?,
            gz_dps: number(fields, self.gz_dps, "gz_dps", line)?,
            left_rpm: number(fields, self.left_rpm, "left_rpm", line)?,
            right_rpm: number(fields, self.right_rpm, "right_rpm", line)?,
            bat_v: number(fields, self.bat_v, "bat_v", line)?,
            age_ms: number(fields, self.age_ms, "age_ms", line)?,
            estop: flag(fields, self.estop, "estop", line)?,
            vset: optional_number(fields, self.vset),
            wset: optional_number(fields, self.wset),
            torque_units: number(fields, self.torque_units, "torque_units", line)?,
            test_left_units: optional_number(fields, self.test_left_units),
            test_right_units: optional_number(fields, self.test_right_units),
            ref_deg: optional_number(fields, self.ref_deg),
            demand_deg: optional_number(fields, self.demand_deg),
            trim_deg: optional_number(fields, self.trim_deg),
            position_m: optional_number(fields, self.position_m),
            mode: match self.mode {
                Some(index) => mode(fields, index, line)?,
                None => 0,
            },
        })
    }
}

/// Column position, or `None` when the recording predates the field.
fn optional(names: &[&str], want: &str) -> Option<usize> {
    names.iter().position(|name| *name == want)
}

fn position(names: &[&str], want: &str) -> Result<usize, Error> {
    names
        .iter()
        .position(|name| *name == want)
        .ok_or_else(|| Error::parse(1, format!("missing column `{want}`")))
}

fn number(fields: &[&str], index: usize, want: &str, line: usize) -> Result<f64, Error> {
    fields[index].trim().parse::<f64>().map_err(|_| {
        Error::parse(
            line,
            format!("`{want}` is not a number: `{}`", fields[index]),
        )
    })
}

/// Field value when the column exists, otherwise zero.
fn optional_number(fields: &[&str], index: Option<usize>) -> f64 {
    index
        .and_then(|index| fields.get(index))
        .and_then(|value| value.trim().parse::<f64>().ok())
        .unwrap_or(0.0)
}

fn unsigned(fields: &[&str], index: usize, want: &str, line: usize) -> Result<u64, Error> {
    fields[index].trim().parse::<u64>().map_err(|_| {
        Error::parse(
            line,
            format!("`{want}` is not an integer: `{}`", fields[index]),
        )
    })
}

fn flag(fields: &[&str], index: usize, want: &str, line: usize) -> Result<bool, Error> {
    match fields[index].trim() {
        "0" | "false" => Ok(false),
        "1" | "true" => Ok(true),
        other => Err(Error::parse(
            line,
            format!("`{want}` is not a flag: `{other}`"),
        )),
    }
}

fn mode(fields: &[&str], index: usize, line: usize) -> Result<u8, Error> {
    let value = fields[index].trim();
    value
        .parse::<u8>()
        .ok()
        .filter(|mode| *mode <= MODE_FALL)
        .ok_or_else(|| Error::parse(line, format!("`mode` is out of range: `{value}`")))
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) const HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,test_right_units,ref_deg,demand_deg,trim_deg,position_m,mode";

    pub(crate) fn row(t_ms: u64, pitch: f64, roll: f64, gy: f64, gx: f64, mode: u8) -> String {
        format!(
            "{t_ms},{t_ms},200,1,{pitch},{roll},0,{gy},{gx},0,0,0,40.0,5,0,0,0,0,0,0,0,0,0,0,{mode}"
        )
    }

    #[test]
    fn parses_a_row() {
        let text = format!("{HEADER}\n{}\n", row(10, -1.5, 0.25, 3.0, -0.5, 2));
        let session = parse_session(&text).unwrap();
        assert_eq!(session.samples.len(), 1);
        let sample = session.samples[0];
        assert_eq!(sample.t_ms, 10);
        assert_eq!(sample.mode, 2);
        assert!(sample.armed());
        assert!((sample.pitch_deg + 1.5).abs() < 1e-9);
        assert!(sample.imu_fresh);
        assert!(!sample.estop);
    }

    #[test]
    fn truncated_tail_is_counted() {
        let text = format!("{HEADER}\n{}\n11,12,200\n", row(10, 0.0, 0.0, 0.0, 0.0, 0));
        let session = parse_session(&text).unwrap();
        assert_eq!(session.samples.len(), 1);
        assert_eq!(session.truncated_rows, 1);
    }

    #[test]
    fn missing_column_is_named() {
        let text = "t_ms,step\n0,0\n";
        let error = parse_session(text).unwrap_err();
        let message = error.to_string();
        assert!(message.contains("missing column"), "{message}");
    }

    #[test]
    fn bad_number_is_located() {
        let header = HEADER;
        let mut fields: Vec<String> = row(10, 0.0, 0.0, 0.0, 0.0, 0)
            .split(',')
            .map(str::to_owned)
            .collect();
        fields[4] = "nan?".to_owned();
        let text = format!("{header}\n{}\n", fields.join(","));
        let error = parse_session(&text).unwrap_err();
        assert!(error.to_string().starts_with("line 2:"), "{error}");
    }

    #[test]
    fn extra_columns_are_ignored() {
        let text = format!("{HEADER},future\n{},1\n", row(10, 1.0, 0.0, 0.0, 0.0, 1));
        let session = parse_session(&text).unwrap();
        assert_eq!(session.samples.len(), 1);
    }

    #[test]
    fn legacy_recording_without_reference_still_parses() {
        let header = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,torque_units";
        let text = format!("{header}\n0,0,200,1,1.0,0.0,0,0,0,0,0,0,40.0,5,0,0\n");
        let session = parse_session(&text).unwrap();
        assert_eq!(session.samples.len(), 1);
        assert_eq!(session.samples[0].mode, 0);
        assert!(session.missing.contains(&"ref_deg"));
        assert!(session.missing.contains(&"mode"));
    }

    #[test]
    fn current_recording_reports_no_missing_columns() {
        let text = format!("{HEADER}\n{}\n", row(10, 0.0, 0.0, 0.0, 0.0, 0));
        let session = parse_session(&text).unwrap();
        assert!(session.missing.is_empty(), "{:?}", session.missing);
    }

    #[test]
    fn segments_split_at_mount_commands() {
        let actions = vec![
            Action {
                t_ms: 100,
                kind: "tx".to_owned(),
                detail: "mount rotyp".to_owned(),
            },
            Action {
                t_ms: 500,
                kind: "tx".to_owned(),
                detail: "arm".to_owned(),
            },
            Action {
                t_ms: 700,
                kind: "tx".to_owned(),
                detail: "mount flipx".to_owned(),
            },
        ];
        let segments = mount_segments(&actions, 1000);
        assert_eq!(segments.len(), 3);
        assert_eq!(segments[0].start_ms, 0);
        assert_eq!(segments[0].end_ms, 100);
        assert_eq!(segments[1].label, "mount rotyp");
        assert_eq!(segments[2].label, "mount flipx");
        assert_eq!(segments[2].end_ms, 1001);
    }

    #[test]
    fn no_mount_command_is_one_segment() {
        let segments = mount_segments(&[], 50);
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].end_ms, 51);
    }
}
