//! Session recording for bench analysis.
//!
//! One recorder produces two CSV files with a shared millisecond clock:
//!
//! * `jbot-telemetry-<unix secs>.csv` — one row per `T ...` sample: attitude,
//!   rates, wheel speeds, setpoints, applied torque. The machine signal.
//! * `jbot-actions-<unix secs>.csv` — one row per operator-visible event:
//!   every command sent to the firmware, every non-telemetry reply received
//!   (`ok`, `err`, `stat`, banner, garbage), connection state changes. The
//!   human signal.
//!
//! Both files share the same `t_ms` base (milliseconds since the recording
//! started), so a plot can overlay "what the operator did" on "what the
//! robot did".

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use crate::protocol::Telemetry;

/// Header row for the telemetry file.
pub const TELEMETRY_HEADER: &str = "t_ms,step,loop_hz,imu_fresh,pitch_deg,roll_deg,heading_deg,gy_dps,gx_dps,gz_dps,left_rpm,right_rpm,bat_v,age_ms,estop,vset,wset,torque_units,test_left_units,test_right_units,ref_deg,demand_deg,trim_deg,position_m,mode";
/// Header row for the actions file.
pub const ACTIONS_HEADER: &str = "t_ms,kind,detail";

/// Open a new session in `dir` and return the recorder.
pub fn start(dir: &Path) -> std::io::Result<SessionRecorder> {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_secs())
        .unwrap_or(0);
    let telemetry_path = dir.join(format!("jbot-telemetry-{secs}.csv"));
    let actions_path = dir.join(format!("jbot-actions-{secs}.csv"));

    let telemetry = BufWriter::new(File::create(&telemetry_path)?);
    let actions = BufWriter::new(File::create(&actions_path)?);

    let mut recorder = SessionRecorder {
        telemetry,
        actions,
        telemetry_path,
        actions_path,
        telemetry_rows: 0,
        action_rows: 0,
        started: Instant::now(),
    };
    recorder.write_action_raw(0, "session", "recording started");
    Ok(recorder)
}

/// One open recording session (telemetry + actions).
pub struct SessionRecorder {
    telemetry: BufWriter<File>,
    actions: BufWriter<File>,
    telemetry_path: PathBuf,
    actions_path: PathBuf,
    telemetry_rows: u64,
    action_rows: u64,
    started: Instant,
}

impl SessionRecorder {
    /// Path of the telemetry CSV.
    pub fn telemetry_path(&self) -> &Path {
        &self.telemetry_path
    }

    /// Path of the actions CSV.
    pub fn actions_path(&self) -> &Path {
        &self.actions_path
    }

    /// Telemetry rows written so far.
    pub fn telemetry_rows(&self) -> u64 {
        self.telemetry_rows
    }

    /// Action rows written so far.
    pub fn action_rows(&self) -> u64 {
        self.action_rows
    }

    fn elapsed_ms(&self) -> u64 {
        self.started.elapsed().as_millis() as u64
    }

    /// Append one telemetry sample; flushes every 100 rows.
    pub fn record_telemetry(&mut self, telemetry: &Telemetry) -> std::io::Result<()> {
        let t_ms = self.elapsed_ms();
        if self.telemetry_rows == 0 {
            writeln!(self.telemetry, "{TELEMETRY_HEADER}")?;
        }
        let row = format_row(t_ms as u128, telemetry);
        writeln!(self.telemetry, "{row}")?;
        self.telemetry_rows += 1;
        if self.telemetry_rows.is_multiple_of(100) {
            self.telemetry.flush()?;
        }
        Ok(())
    }

    /// Append one action row: `kind` is `tx`, `rx`, `sys` or `ui`;
    /// `detail` is escaped for CSV.
    pub fn record_action(&mut self, kind: &str, detail: &str) {
        let t_ms = self.elapsed_ms();
        self.write_action_raw(t_ms, kind, detail);
    }

    fn write_action_raw(&mut self, t_ms: u64, kind: &str, detail: &str) {
        if self.action_rows == 0 {
            let _ = writeln!(self.actions, "{ACTIONS_HEADER}");
        }
        let _ = writeln!(self.actions, "{t_ms},{kind},{}", csv_escape(detail));
        self.action_rows += 1;
        if self.action_rows.is_multiple_of(100) {
            let _ = self.actions.flush();
        }
    }

    /// Flush both files.
    pub fn flush(&mut self) {
        let _ = self.telemetry.flush();
        let _ = self.actions.flush();
        self.record_action("session", "recording stopped");
        let _ = self.actions.flush();
    }
}

/// A continuously appended, human-readable event log.
///
/// The session CSV files are for deliberate bench recordings; this log never
/// stops recording: every line the console prints into its own event panel is
/// also appended to `jbot-events.log` with a millisecond timestamp. It exists
/// so post-incident analysis does not depend on the operator remembering to
/// press REC.
pub struct EventLog {
    writer: BufWriter<File>,
    path: PathBuf,
    disabled: bool,
}

impl EventLog {
    /// Open (append) `jbot-events.log` inside `dir`; writes a session banner.
    pub fn open(dir: &Path) -> std::io::Result<Self> {
        let path = dir.join("jbot-events.log");
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)?;
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|duration| duration.as_millis())
            .unwrap_or(0);
        writeln!(file, "--- session {now} ---")?;
        Ok(Self {
            writer: BufWriter::new(file),
            path,
            disabled: false,
        })
    }

    /// Absolute path of the log file.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Append one line. A write failure disables further writes but never
    /// propagates — the UI must not die because a log file broke.
    pub fn record(&mut self, tag: &str, text: &str) {
        if self.disabled {
            return;
        }
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|duration| duration.as_millis())
            .unwrap_or(0);
        let line = format_line(now, tag, text);
        if writeln!(self.writer, "{line}")
            .and_then(|()| self.writer.flush())
            .is_err()
        {
            self.disabled = true;
        }
    }
}

/// Format one event-log line: `unix_ms TAG text`.
pub fn format_line(now_ms: u128, tag: &str, text: &str) -> String {
    format!("{now_ms} {tag} {text}")
}

/// Escape one CSV cell: quote when it contains a comma, a quote or a newline.
fn csv_escape(cell: &str) -> String {
    let needs_quotes = cell.contains([',', '"', '\n', '\r']);
    if !needs_quotes {
        return cell.to_owned();
    }
    let mut out = String::with_capacity(cell.len() + 2);
    out.push('"');
    for ch in cell.chars() {
        if ch == '"' {
            out.push('"');
        }
        out.push(ch);
    }
    out.push('"');
    out
}

/// One telemetry CSV row (without the terminating newline).
///
/// Optional fields stay empty when the firmware does not send them, so an old
/// log and a new firmware still parse column by column.
pub fn format_row(t_ms: u128, telemetry: &Telemetry) -> String {
    let opt_f32 = |value: Option<f32>| value.map_or_else(String::new, |v| format!("{v:.2}"));
    let opt_f32_3 = |value: Option<f32>| value.map_or_else(String::new, |v| format!("{v:.3}"));
    let mode = telemetry
        .balance_mode
        .map_or_else(String::new, |value| value.to_string());
    format!(
        "{},{},{},{},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{},{},{:.2},{},{},{:.2},{:.2},{},{},{},{},{},{},{},{}",
        t_ms,
        telemetry.step,
        telemetry.loop_hz,
        u8::from(telemetry.imu_fresh),
        telemetry.pitch_deg,
        telemetry.roll_deg,
        telemetry.heading_deg,
        telemetry.gyro_y_dps,
        telemetry.gyro_x_dps,
        telemetry.gyro_z_dps,
        telemetry.speed_left_rpm,
        telemetry.speed_right_rpm,
        telemetry.battery_v,
        telemetry.feedback_age_ms,
        u8::from(telemetry.estop),
        telemetry.speed_setpoint_mps,
        telemetry.steer_setpoint_rps,
        opt_f32(telemetry.torque_common_units),
        opt_f32(telemetry.test_left_units),
        opt_f32(telemetry.test_right_units),
        opt_f32(telemetry.pitch_ref_deg),
        opt_f32(telemetry.pitch_demand_deg),
        opt_f32(telemetry.slope_trim_deg),
        opt_f32_3(telemetry.position_m),
        mode,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_header_has_as_many_columns_as_a_row() {
        let telemetry = Telemetry::default();
        let row = format_row(0, &telemetry);
        assert_eq!(TELEMETRY_HEADER.split(',').count(), row.split(',').count());
    }

    #[test]
    fn row_contains_the_signal_values() {
        let telemetry = Telemetry {
            step: 42,
            pitch_deg: 3.25,
            speed_left_rpm: -10,
            torque_common_units: Some(120.5),
            ..Telemetry::default()
        };
        let row = format_row(1234, &telemetry);
        assert!(row.starts_with("1234,42,"));
        assert!(row.contains("3.25"));
        assert!(row.contains("-10"));
        assert!(row.contains("120.50"));
    }

    #[test]
    fn missing_optional_fields_become_empty_cells() {
        let telemetry = Telemetry::default();
        let row = format_row(0, &telemetry);
        assert!(row.trim_end().ends_with(",,"));
    }

    #[test]
    fn csv_escape_quotes_only_when_needed() {
        assert_eq!(csv_escape("plain"), "plain");
        assert_eq!(csv_escape("with,comma"), "\"with,comma\"");
        assert_eq!(csv_escape("with\"quote"), "\"with\"\"quote\"");
        assert_eq!(csv_escape("with\nnewline"), "\"with\nnewline\"");
    }

    #[test]
    fn actions_header_matches_row_shape() {
        assert_eq!(ACTIONS_HEADER.split(',').count(), 3);
    }

    #[test]
    fn event_log_line_is_machine_and_human_readable() {
        let line = format_line(1_750_000_000_000, "INFO", "speed 0.5");
        assert_eq!(line, "1750000000000 INFO speed 0.5");
    }

    #[test]
    fn event_log_appends_across_opens() {
        let dir = std::env::temp_dir().join(format!("jbot-test-eventlog-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("jbot-events.log");
        let _ = std::fs::remove_file(&path);
        {
            let mut log = EventLog::open(&dir).expect("open");
            log.record("INFO", "first");
        }
        {
            let mut log = EventLog::open(&dir).expect("reopen");
            log.record("ERR ", "second");
        }
        let content = std::fs::read_to_string(&path).expect("read log");
        assert!(content.contains("first"));
        assert!(content.contains("second"));
        assert!(content.contains("--- session "));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
