//! Serial transport for the Pico USB console.
//!
//! A dedicated thread owns the blocking serial port, so the UI never performs
//! I/O. Commands are latest-value lines; telemetry is loss-tolerant and the
//! event queue is bounded. Every disconnect, user close, or write failure
//! latches the firmware e-stop and zeroes both setpoints before the port is
//! released.

use std::io::{self, ErrorKind, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TryRecvError};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use serialport::SerialPort;

use crate::protocol::{self, Message};

/// Pico CDC-ACM link speed. The USB CDC transport ignores the value, but the
/// serial API still requires one.
const BAUD: u32 = 115_200;
/// Blocking read timeout; also the worst-case shutdown latency.
const READ_TIMEOUT: Duration = Duration::from_millis(25);
/// Delay before retrying a failed or closed port.
const RETRY_DELAY: Duration = Duration::from_millis(750);
/// Read chunk size.
const READ_CHUNK: usize = 512;
/// Longest accepted line before the parser resynchronises.
const MAX_LINE: usize = 4096;
/// Capacity of the command queue.
const COMMAND_QUEUE: usize = 64;
/// Capacity of the UI event queue.
const EVENT_QUEUE: usize = 1024;

/// Something the serial thread reports to the UI.
#[derive(Clone, Debug)]
pub enum LinkEvent {
    /// The port opened and the initial commands were written.
    Connected {
        /// Port name that opened.
        port: String,
    },
    /// The port closed or failed.
    Disconnected {
        /// Human-readable reason.
        reason: String,
    },
    /// One parsed protocol line.
    Message(Message),
    /// Transport-level notice.
    Notice(String),
}

/// Handle to the serial thread.
pub struct LinkHandle {
    commands: SyncSender<String>,
    events: Receiver<LinkEvent>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU32>,
    join: Option<JoinHandle<()>>,
}

impl LinkHandle {
    /// Open a link to `port` and request `stream_hz` telemetry.
    ///
    /// The thread keeps retrying while the port is unavailable; the UI decides
    /// when to give up by calling [`LinkHandle::shutdown`].
    pub fn connect(port: impl Into<String>, stream_hz: u32) -> io::Result<Self> {
        let port = port.into();
        let (command_tx, command_rx) = sync_channel(COMMAND_QUEUE);
        let (event_tx, event_rx) = sync_channel(EVENT_QUEUE);
        let stop = Arc::new(AtomicBool::new(false));
        let dropped = Arc::new(AtomicU32::new(0));

        let thread_stop = Arc::clone(&stop);
        let thread_dropped = Arc::clone(&dropped);
        let join = thread::Builder::new()
            .name("jbot-serial".to_owned())
            .spawn(move || {
                run_serial(
                    port,
                    stream_hz,
                    command_rx,
                    event_tx,
                    thread_stop,
                    thread_dropped,
                );
            })?;

        Ok(Self {
            commands: command_tx,
            events: event_rx,
            stop,
            dropped,
            join: Some(join),
        })
    }

    /// Queue one protocol line. Returns false when the link is closed or the
    /// queue is full.
    pub fn send(&self, line: impl Into<String>) -> bool {
        // The queue is drained on every read timeout, so a full queue only
        // happens when the port is stalled. Dropping the newest command is
        // safer than replaying a stale backlog after a reconnect.
        self.commands.try_send(line.into()).is_ok()
    }

    /// Drain every available event without blocking.
    pub fn drain(&mut self) -> Vec<LinkEvent> {
        let mut events = Vec::new();
        while let Ok(event) = self.events.try_recv() {
            events.push(event);
        }
        events
    }

    /// Number of events dropped because the UI queue was full.
    pub fn dropped_events(&self) -> u32 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Stop the thread and wait for the safety commands to be written.
    pub fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

impl Drop for LinkHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// List serial ports with Pico-like names first.
pub fn available_ports() -> Vec<String> {
    let mut ports: Vec<String> = serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|port| port.port_name)
        .collect();
    ports.sort_by_key(|name| (!looks_like_pico(name), name.clone()));
    ports
}

/// True for USB CDC/ACM names used by the Pico and common USB-UART bridges.
pub fn looks_like_pico(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    lower.contains("usbmodem")
        || lower.contains("jbot")
        || lower.contains("ttyacm")
        || lower.contains("ttyusb")
}

fn run_serial(
    port_name: String,
    stream_hz: u32,
    commands: Receiver<String>,
    events: SyncSender<LinkEvent>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU32>,
) {
    let mut last_open_error = String::new();

    while !stop.load(Ordering::Relaxed) {
        let opened = serialport::new(&port_name, BAUD)
            .timeout(READ_TIMEOUT)
            .open();
        let mut port = match opened {
            Ok(port) => port,
            Err(error) => {
                let text = error.to_string();
                if text != last_open_error {
                    send_event(
                        &events,
                        &dropped,
                        LinkEvent::Notice(format!("open {port_name}: {text}")),
                    );
                    last_open_error = text;
                }
                sleep_interruptible(&stop, RETRY_DELAY);
                continue;
            }
        };
        last_open_error.clear();

        // Never replay commands queued while the port was gone.
        while commands.try_recv().is_ok() {}

        send_event(
            &events,
            &dropped,
            LinkEvent::Connected {
                port: port_name.clone(),
            },
        );

        // Known-safe initial state: telemetry on, setpoints zero, trim queried.
        let _ = write_line(port.as_mut(), &format!("stream {stream_hz}"));
        let _ = write_line(port.as_mut(), "speed 0");
        let _ = write_line(port.as_mut(), "steer 0");
        let _ = write_line(port.as_mut(), "trim");
        let _ = write_line(port.as_mut(), "ping");
        let _ = write_line(port.as_mut(), "stat");

        let mut buffer = [0u8; READ_CHUNK];
        let mut line = LineBuffer::new();
        let mut reason = String::new();

        loop {
            if stop.load(Ordering::Relaxed) {
                reason = "closed by the console".to_owned();
                break;
            }

            match port.read(&mut buffer) {
                Ok(0) => {}
                Ok(len) => line.feed(&buffer[..len], &events, &dropped),
                Err(error)
                    if matches!(
                        error.kind(),
                        ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted
                    ) => {}
                Err(error) => {
                    reason = error.to_string();
                    break;
                }
            }

            loop {
                match commands.try_recv() {
                    Ok(command) => {
                        if let Err(error) = write_line(port.as_mut(), &command) {
                            reason = error.to_string();
                            break;
                        }
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        stop.store(true, Ordering::Relaxed);
                        reason = "console closed".to_owned();
                        break;
                    }
                }
            }

            if !reason.is_empty() {
                break;
            }
        }

        // Safety: latch the firmware e-stop and zero the setpoints on every
        // link end, including an unexpected USB disconnect.
        let _ = write_line(port.as_mut(), "estop");
        let _ = write_line(port.as_mut(), "speed 0");
        let _ = write_line(port.as_mut(), "steer 0");
        let _ = port.flush();

        send_event(&events, &dropped, LinkEvent::Disconnected { reason });

        if stop.load(Ordering::Relaxed) {
            break;
        }
        sleep_interruptible(&stop, RETRY_DELAY);
    }
}

fn write_line(port: &mut dyn SerialPort, line: &str) -> io::Result<()> {
    port.write_all(line.as_bytes())?;
    port.write_all(b"\n")?;
    port.flush()
}

fn send_event(events: &SyncSender<LinkEvent>, dropped: &AtomicU32, event: LinkEvent) {
    if events.try_send(event).is_err() {
        dropped.fetch_add(1, Ordering::Relaxed);
    }
}

fn sleep_interruptible(stop: &AtomicBool, total: Duration) {
    const STEP: Duration = Duration::from_millis(50);
    let mut left = total;
    while !stop.load(Ordering::Relaxed) && !left.is_zero() {
        let chunk = left.min(STEP);
        thread::sleep(chunk);
        left = left.saturating_sub(chunk);
    }
}

/// Accumulates bytes into lines and resynchronises after an oversized line.
struct LineBuffer {
    bytes: Vec<u8>,
    skipping: bool,
}

impl LineBuffer {
    fn new() -> Self {
        Self {
            bytes: Vec::with_capacity(256),
            skipping: false,
        }
    }

    fn feed(&mut self, chunk: &[u8], events: &SyncSender<LinkEvent>, dropped: &AtomicU32) {
        for &byte in chunk {
            if byte == b'\n' || byte == b'\r' {
                if self.skipping {
                    self.skipping = false;
                    self.bytes.clear();
                } else if !self.bytes.is_empty() {
                    let text = String::from_utf8_lossy(&self.bytes).trim().to_owned();
                    self.bytes.clear();
                    if let Some(message) = protocol::classify(&text) {
                        send_event(events, dropped, LinkEvent::Message(message));
                    }
                }
                continue;
            }

            if self.skipping {
                continue;
            }
            if self.bytes.len() < MAX_LINE {
                self.bytes.push(byte);
            } else {
                self.bytes.clear();
                self.skipping = true;
                send_event(
                    events,
                    dropped,
                    LinkEvent::Notice("oversized line discarded".to_owned()),
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drain(rx: &Receiver<LinkEvent>) -> Vec<LinkEvent> {
        let mut events = Vec::new();
        while let Ok(event) = rx.try_recv() {
            events.push(event);
        }
        events
    }

    #[test]
    fn line_buffer_handles_split_chunks_and_delimiters() {
        let (tx, rx) = sync_channel(8);
        let dropped = AtomicU32::new(0);
        let mut buffer = LineBuffer::new();

        buffer.feed(b"T step=1 pitch=2.0\r", &tx, &dropped);
        buffer.feed(b"\nok ping\n\n", &tx, &dropped);

        let events = drain(&rx);
        assert!(matches!(
            events[0],
            LinkEvent::Message(Message::Telemetry(_))
        ));
        assert!(matches!(events[1], LinkEvent::Message(Message::Ok(_))));
        assert_eq!(events.len(), 2);
    }

    #[test]
    fn line_buffer_resynchronises_after_an_oversized_line() {
        let (tx, rx) = sync_channel(8);
        let dropped = AtomicU32::new(0);
        let mut buffer = LineBuffer::new();

        let long = vec![b'x'; MAX_LINE + 8];
        buffer.feed(&long, &tx, &dropped);
        buffer.feed(b"tail\nok ping\n", &tx, &dropped);

        let events = drain(&rx);
        assert!(matches!(events[0], LinkEvent::Notice(_)));
        assert!(matches!(events[1], LinkEvent::Message(Message::Ok(_))));
    }

    #[test]
    fn pico_names_are_recognised() {
        assert!(looks_like_pico("/dev/cu.usbmodemJBOT_PICO_011"));
        assert!(looks_like_pico("/dev/ttyACM0"));
        assert!(looks_like_pico("/dev/ttyUSB0"));
        assert!(!looks_like_pico("/dev/tty.Bluetooth-Incoming-Port"));
    }
}
