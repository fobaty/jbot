//! Read-only live view of the bench console over local HTTP.
//!
//! The GUI still owns the serial port and still writes CSV recordings. This
//! module only publishes what the UI already knows, so an external tool can
//! read live telemetry without taking the port or waiting for a CSV to close.
//!
//! Security model: the listener binds to 127.0.0.1, accepts GET only, and never
//! exposes a command endpoint. Arming, test torque and estop stay in the GUI.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::protocol::{Stat, Telemetry};

/// Default localhost port for the live view.
pub const DEFAULT_PORT: u16 = 8712;
const SAMPLE_CAPACITY: usize = 4096; // ~20 s at 200 Hz
const EVENT_CAPACITY: usize = 200;

/// One timestamped telemetry sample in the ring buffer.
#[derive(Clone, Debug)]
pub struct LiveSample {
    pub t_ms: u64,
    pub telemetry: Telemetry,
}

#[derive(Default)]
struct Inner {
    telemetry: Option<Telemetry>,
    stat: Option<Stat>,
    samples: VecDeque<LiveSample>,
    events: VecDeque<String>,
    recorder: Option<String>,
    started: Option<Instant>,
}

/// Shared, cloneable handle to the console's live state.
#[derive(Clone, Default)]
pub struct LiveState {
    inner: Arc<Mutex<Inner>>,
}

impl LiveState {
    pub fn set_telemetry(&self, telemetry: Telemetry) {
        let mut inner = self.inner.lock().expect("live state poisoned");
        let started = *inner.started.get_or_insert_with(Instant::now);
        let t_ms = started.elapsed().as_millis() as u64;
        inner.samples.push_back(LiveSample {
            t_ms,
            telemetry: telemetry.clone(),
        });
        while inner.samples.len() > SAMPLE_CAPACITY {
            inner.samples.pop_front();
        }
        inner.telemetry = Some(telemetry);
    }

    pub fn set_stat(&self, stat: Stat) {
        self.inner.lock().expect("live state poisoned").stat = Some(stat);
    }

    pub fn push_event(&self, entry: String) {
        let mut inner = self.inner.lock().expect("live state poisoned");
        inner.events.push_back(entry);
        while inner.events.len() > EVENT_CAPACITY {
            inner.events.pop_front();
        }
    }

    pub fn set_recorder(&self, path: Option<String>) {
        self.inner.lock().expect("live state poisoned").recorder = path;
    }

    fn with_inner<T>(&self, f: impl FnOnce(&Inner) -> T) -> T {
        let inner = self.inner.lock().expect("live state poisoned");
        f(&inner)
    }

    fn state_json(&self) -> String {
        self.with_inner(|inner| {
            let telemetry = inner
                .telemetry
                .as_ref()
                .map(telemetry_json)
                .unwrap_or_else(|| "null".to_owned());
            let stat = inner
                .stat
                .as_ref()
                .map(stat_json)
                .unwrap_or_else(|| "null".to_owned());
            let recorder = json_option_str(inner.recorder.as_deref());
            format!("{{\"telemetry\":{telemetry},\"stat\":{stat},\"recording\":{recorder},\"samples\":{}}}", inner.samples.len())
        })
    }

    fn samples_json(&self, last: usize) -> String {
        self.with_inner(|inner| {
            let skip = inner.samples.len().saturating_sub(last);
            let rows: Vec<String> = inner
                .samples
                .iter()
                .skip(skip)
                .map(|sample| sample_json(sample.t_ms, &sample.telemetry))
                .collect();
            format!("[{}]", rows.join(","))
        })
    }

    fn events_json(&self, last: usize) -> String {
        self.with_inner(|inner| {
            let skip = inner.events.len().saturating_sub(last);
            let rows: Vec<String> = inner
                .events
                .iter()
                .skip(skip)
                .map(|entry| escape(entry))
                .collect();
            format!("[{}]", rows.join(","))
        })
    }
}

/// Start the localhost listener. Returns an error if the port is busy.
pub fn spawn(state: LiveState, port: u16) -> std::io::Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    std::thread::spawn(move || {
        for connection in listener.incoming() {
            match connection {
                Ok(mut stream) => {
                    let _ = stream.set_read_timeout(Some(Duration::from_secs(1)));
                    handle(&mut stream, &state);
                }
                Err(_) => break,
            }
        }
    });
    Ok(())
}

fn handle(stream: &mut TcpStream, state: &LiveState) {
    let mut buffer = [0_u8; 4096];
    let read = match stream.read(&mut buffer) {
        Ok(size) => size,
        Err(_) => return,
    };
    let request = String::from_utf8_lossy(&buffer[..read]);
    let first_line = request.lines().next().unwrap_or("");
    let mut parts = first_line.split_whitespace();
    let method = parts.next().unwrap_or("");
    let target = parts.next().unwrap_or("/");
    if method != "GET" {
        respond(stream, 405, "application/json", "{\"error\":\"GET only\"}");
        return;
    }

    let (path, query) = target.split_once('?').unwrap_or((target, ""));
    let last = query
        .split('&')
        .find_map(|item| item.strip_prefix("last="))
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(200)
        .min(SAMPLE_CAPACITY);

    let (status, content_type, body) = match path {
        "/api/state" => (200, "application/json", state.state_json()),
        "/api/samples" => (200, "application/json", state.samples_json(last)),
        "/api/events" => (200, "application/json", state.events_json(last)),
        "/api/status" => (
            200,
            "application/json",
            "{\"service\":\"jbot-console-live\",\"bound\":\"127.0.0.1\"}".to_owned(),
        ),
        "/" => (
            200,
            "text/plain; charset=utf-8",
            "jbot-console live view (read-only)\n  /api/state\n  /api/samples?last=N\n  /api/events?last=N\n  /api/status\n".to_owned(),
        ),
        _ => (404, "application/json", "{\"error\":\"not found\"}".to_owned()),
    };
    respond(stream, status, content_type, &body);
}

fn respond(stream: &mut TcpStream, status: u16, content_type: &str, body: &str) {
    let reason = match status {
        200 => "OK",
        404 => "Not Found",
        405 => "Method Not Allowed",
        _ => "Error",
    };
    let header = format!(
        "HTTP/1.1 {status} {reason}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    );
    let _ = stream.write_all(header.as_bytes());
    let _ = stream.write_all(body.as_bytes());
    let _ = stream.flush();
}

fn escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for character in text.chars() {
        match character {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c.is_control() => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn f(value: f64) -> String {
    if value.is_finite() {
        format!("{value:.6}")
    } else {
        "null".to_owned()
    }
}

fn b(value: bool) -> &'static str {
    if value {
        "true"
    } else {
        "false"
    }
}

fn opt_f(value: Option<f32>) -> String {
    value.map_or_else(|| "null".to_owned(), |v| f(f64::from(v)))
}

fn opt_u(value: Option<u32>) -> String {
    value.map_or_else(|| "null".to_owned(), |v| v.to_string())
}

fn opt_bool(value: Option<bool>) -> &'static str {
    match value {
        Some(true) => "true",
        Some(false) => "false",
        None => "null",
    }
}

fn json_option_str(value: Option<&str>) -> String {
    value.map_or_else(|| "null".to_owned(), escape)
}

fn telemetry_json(t: &Telemetry) -> String {
    format!(
        "{{\"step\":{},\"loop_hz\":{},\"imu_fresh\":{},\"cal\":{},\"pitch_deg\":{},\"roll_deg\":{},\"heading_deg\":{},\"gy_dps\":{},\"gx_dps\":{},\"gz_dps\":{},\"sys_err\":{},\"feedback_present\":{},\"feedback_age_ms\":{},\"left_rpm\":{},\"right_rpm\":{},\"battery_v\":{},\"board_temp_c\":{},\"estop\":{},\"vset\":{},\"wset\":{},\"torque_units\":{},\"test_left_units\":{},\"test_right_units\":{},\"ref_deg\":{},\"demand_deg\":{},\"trim_deg\":{},\"position_m\":{},\"mode\":{},\"armed\":{},\"mount\":{},\"zero_offset_deg\":{},\"level_deg\":{}}}",
        t.step,
        t.loop_hz,
        b(t.imu_fresh),
        t.cal,
        f(f64::from(t.pitch_deg)),
        f(f64::from(t.roll_deg)),
        f(f64::from(t.heading_deg)),
        f(f64::from(t.gyro_y_dps)),
        f(f64::from(t.gyro_x_dps)),
        f(f64::from(t.gyro_z_dps)),
        t.sys_err,
        b(t.feedback_present),
        t.feedback_age_ms,
        f(f64::from(t.speed_left_rpm)),
        f(f64::from(t.speed_right_rpm)),
        f(f64::from(t.battery_v)),
        f(f64::from(t.board_temp_c)),
        b(t.estop),
        f(f64::from(t.speed_setpoint_mps)),
        f(f64::from(t.steer_setpoint_rps)),
        opt_f(t.torque_common_units),
        opt_f(t.test_left_units),
        opt_f(t.test_right_units),
        opt_f(t.pitch_ref_deg),
        opt_f(t.pitch_demand_deg),
        opt_f(t.slope_trim_deg),
        opt_f(t.position_m),
        t.balance_mode.map_or_else(|| "null".to_owned(), |m| m.to_string()),
        opt_bool(t.armed),
        t.mount.map_or_else(|| "null".to_owned(), |m| m.to_string()),
        opt_f(t.zero_offset_deg),
        opt_f(t.level_deg),
    )
}

fn sample_json(t_ms: u64, t: &Telemetry) -> String {
    let body = telemetry_json(t);
    format!("{{\"t_ms\":{},{}}}", t_ms, &body[1..body.len() - 1])
}

fn stat_json(s: &Stat) -> String {
    format!(
        "{{\"uptime_s\":{},\"loop_hz\":{},\"stream_hz\":{},\"steps\":{},\"imu_samples\":{},\"imu_errors\":{},\"imu_code\":{},\"fb_frames\":{},\"fb_discarded\":{},\"fb_ok\":{},\"estop\":{},\"stop\":{},\"zero_deg\":{},\"zroll_deg\":{},\"level_deg\":{},\"mount\":{},\"armed\":{},\"home_deg\":{},\"fall_events\":{},\"pscale\":{},\"raw\":{}}}",
        f(f64::from(s.uptime_s)),
        s.loop_hz,
        s.stream_hz,
        s.steps,
        s.imu_samples,
        s.imu_errors,
        s.imu_code,
        s.fb_frames,
        s.fb_discarded,
        opt_bool(s.link_fresh),
        b(s.estop),
        json_option_str(s.stop_reason.as_deref()),
        opt_f(s.zero_deg),
        opt_f(s.zroll_deg),
        opt_f(s.level_deg),
        s.mount.map_or_else(|| "null".to_owned(), |m| m.to_string()),
        opt_bool(s.armed),
        opt_f(s.home_deg),
        opt_u(s.fall_events),
        opt_f(s.pscale),
        escape(&s.raw),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_buffer_keeps_the_newest_samples() {
        let state = LiveState::default();
        for i in 0..10 {
            state.set_telemetry(Telemetry {
                step: i,
                ..Telemetry::default()
            });
        }
        let json = state.samples_json(4);
        assert!(json.contains("\"step\":6"));
        assert!(json.contains("\"step\":9"));
        assert!(!json.contains("\"step\":5"));
    }

    #[test]
    fn json_escapes_event_text() {
        assert_eq!(escape("a\"b\\c\nd"), "\"a\\\"b\\\\c\\nd\"");
    }

    #[test]
    fn state_without_data_is_valid_json_shape() {
        let json = LiveState::default().state_json();
        assert!(json.contains("\"telemetry\":null"));
        assert!(json.contains("\"stat\":null"));
        assert!(json.contains("\"samples\":0"));
    }
}
