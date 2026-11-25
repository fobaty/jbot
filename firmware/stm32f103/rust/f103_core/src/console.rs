//! Hardware-independent debug console grammar, formatting and blackbox.
//!
//! This module owns no peripherals. The firmware feeds it bytes and telemetry;
//! it returns ASCII replies. Keeping the grammar here makes the command set and
//! its output golden-testable on the host, with no MCU in the loop.

use crate::parameters::{Parameters, Tunable};
use crate::protocol::ProtocolMode;
use crate::safety::{Fault, SafeMode};
use crate::sensing::ADC_FIELDS;

pub const LINE_MAX: usize = 64;
pub const REPLY_MAX: usize = 256;
pub const BLACKBOX_DEPTH: usize = 32;

/// Accumulates one line of ASCII. CR is ignored, LF commits the line, and an
/// overlong line is dropped until the next LF so a stuck sender cannot wedge
/// the parser.
#[derive(Clone)]
pub struct LineBuffer<const N: usize> {
    bytes: [u8; N],
    len: usize,
    overflow: bool,
    committed: bool,
}

impl<const N: usize> LineBuffer<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            overflow: false,
            committed: false,
        }
    }

    /// Returns the committed line, or `None` for a byte that did not end a line.
    /// The buffer resets lazily on the next push so the returned slice stays
    /// valid for the caller's whole branch.
    pub fn push(&mut self, byte: u8) -> Option<&str> {
        if self.committed {
            self.reset();
        }
        if byte == b'\r' {
            return None;
        }
        if byte == b'\n' {
            self.committed = true;
            if self.overflow {
                return Some("");
            }
            return core::str::from_utf8(&self.bytes[..self.len]).ok();
        }
        if self.overflow {
            return None;
        }
        if self.len < N {
            self.bytes[self.len] = byte;
            self.len += 1;
        } else {
            self.overflow = true;
            self.len = 0;
        }
        None
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.overflow = false;
        self.committed = false;
    }
}

impl<const N: usize> Default for LineBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// A bounded single-producer/single-consumer byte ring used for both USART3
/// directions. The producer is the ISR, the consumer is the main loop.
#[derive(Clone)]
pub struct ByteRing<const N: usize> {
    bytes: [u8; N],
    head: usize,
    len: usize,
}

impl<const N: usize> ByteRing<N> {
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            head: 0,
            len: 0,
        }
    }

    /// Returns false when the ring is full and the byte is dropped.
    pub fn push(&mut self, byte: u8) -> bool {
        if self.len == N {
            return false;
        }
        let tail = (self.head + self.len) % N;
        self.bytes[tail] = byte;
        self.len += 1;
        true
    }

    pub fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.bytes[self.head];
        self.head = (self.head + 1) % N;
        self.len -= 1;
        Some(byte)
    }

    /// Discard queued bytes, e.g. when disabling a binary stream so a text
    /// console reply cannot be prefixed by a stale frame.
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

impl<const N: usize> Default for ByteRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Parsed debug commands. Tuning keys are clamped in `Tunable::set` and are
/// only accepted while disarmed; safety contracts have no console setter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command<'a> {
    Empty,
    Help,
    Get,
    GetParams,
    WatchToggle,
    SetWatch(u8),
    SetProtocol(ProtocolMode),
    SetParam(Tunable, u32),
    /// Zero disables the binary trace, N streams one frame every N FOC ticks.
    Trace(u8),
    /// Bench-only arming of the generated current loop. Never touches MOE.
    Arm,
    Disarm,
    Init,
    Save,
    Dump,
    Adc,
    /// Any unrecognised or forbidden command; echoed back to the operator.
    Rejected(&'a str),
}

pub fn parse_command(line: &str) -> Command<'_> {
    let mut parts = line.split_whitespace();
    let Some(head) = parts.next() else {
        return Command::Empty;
    };
    if head.eq_ignore_ascii_case("$HELP") || head.eq_ignore_ascii_case("?") {
        return Command::Help;
    }
    if head.eq_ignore_ascii_case("$GET") {
        return Command::Get;
    }
    if head.eq_ignore_ascii_case("$GETP") {
        return Command::GetParams;
    }
    if head.eq_ignore_ascii_case("$TRACE") {
        return match parts.next().and_then(|value| value.parse::<u8>().ok()) {
            Some(decimation) => Command::Trace(decimation),
            None => Command::Rejected(line),
        };
    }
    if head.eq_ignore_ascii_case("$ARM") {
        return Command::Arm;
    }
    if head.eq_ignore_ascii_case("$DISARM") {
        return Command::Disarm;
    }
    if head.eq_ignore_ascii_case("$WATCH") {
        return Command::WatchToggle;
    }
    if head.eq_ignore_ascii_case("$INIT") {
        return Command::Init;
    }
    if head.eq_ignore_ascii_case("$SAVE") {
        return Command::Save;
    }
    if head.eq_ignore_ascii_case("$DUMP") {
        return Command::Dump;
    }
    if head.eq_ignore_ascii_case("$ADC") {
        return Command::Adc;
    }
    if head.eq_ignore_ascii_case("$SET") {
        let (Some(key), Some(value)) = (parts.next(), parts.next()) else {
            return Command::Rejected(line);
        };
        if key.eq_ignore_ascii_case("WATCH") {
            if let Ok(value) = value.parse::<u8>() {
                return Command::SetWatch(value);
            }
        }
        if key.eq_ignore_ascii_case("PROTOCOL") {
            if value.eq_ignore_ascii_case("LEGACY") {
                return Command::SetProtocol(ProtocolMode::Legacy);
            }
            if value.eq_ignore_ascii_case("HARDENED") {
                return Command::SetProtocol(ProtocolMode::Hardened);
            }
        }
        if let (Some(tunable), Ok(number)) = (Tunable::from_name(key), value.parse::<u32>()) {
            return Command::SetParam(tunable, number);
        }
        return Command::Rejected(line);
    }
    Command::Rejected(line)
}

/// In-place ASCII formatter. Writes are silently truncated at the buffer end:
/// a console reply must never overwrite adjacent memory or allocate.
pub struct Formatter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> Formatter<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }

    pub fn as_str(&self) -> &str {
        // Every writer only emits ASCII digits, letters and punctuation.
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }

    pub fn push(&mut self, byte: u8) {
        if self.len < self.buf.len() {
            self.buf[self.len] = byte;
            self.len += 1;
        }
    }

    pub fn put_str(&mut self, value: &str) {
        for byte in value.bytes() {
            self.push(byte);
        }
    }

    fn put_uint(&mut self, mut value: u32) {
        let mut digits = [0u8; 10];
        let mut start = digits.len();
        loop {
            start -= 1;
            digits[start] = b'0' + (value % 10) as u8;
            value /= 10;
            if value == 0 {
                break;
            }
        }
        for digit in &digits[start..] {
            self.push(*digit);
        }
    }

    pub fn put_i32(&mut self, value: i32) {
        if value < 0 {
            self.push(b'-');
            // i32::MIN maps to 2^31, which still fits in u32.
            self.put_uint(value.unsigned_abs());
        } else {
            self.put_uint(value as u32);
        }
    }

    pub fn put_hex4(&mut self, value: u16) {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        for shift in [12, 8, 4, 0] {
            self.push(HEX[((value >> shift) & 0xF) as usize]);
        }
    }

    pub fn put_bool(&mut self, value: bool) {
        self.put_str(if value { "1" } else { "0" });
    }

    pub fn put_u16_list(&mut self, values: &[u16; 3]) {
        for (index, value) in values.iter().enumerate() {
            if index > 0 {
                self.push(b',');
            }
            self.put_i32(*value as i32);
        }
    }

    pub fn line(&mut self) {
        self.push(b'\n');
    }
}

impl SafeMode {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Freewheel => "FREEWHEEL",
            Self::RampToZero => "RAMP_ZERO",
            Self::ActiveBrake => "ACTIVE_BRAKE",
        }
    }
}

/// Point-in-time view assembled by the main loop from atomics and safe state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Telemetry {
    pub now_us: u32,
    pub foc_ticks: u32,
    pub foc_overcurrent: u32,
    pub foc_skipped: u32,
    pub calibration_ready: bool,
    pub protocol: ProtocolMode,
    pub mode: SafeMode,
    pub armed: bool,
    pub fault: Option<Fault>,
    pub faults: u32,
    pub rejected: u32,
    pub link_age_us: Option<u32>,
    pub steer: i16,
    pub speed: i16,
    pub target_left: i16,
    pub target_right: i16,
    pub duty_l: [u16; 3],
    pub duty_r: [u16; 3],
    pub dc_left: i32,
    pub dc_right: i32,
    pub battery_raw: i16,
    pub temperature_decic: i16,
    pub diag_left: u8,
    pub diag_right: u8,
    pub foc_cycles: u32,
    pub foc_max_cycles: u32,
    pub foc_misses: u32,
    pub params_from_flash: bool,
}

fn fault_label(fault: Option<Fault>) -> &'static str {
    match fault {
        None => "NONE",
        Some(Fault::LinkTimeout) => "LINK_TIMEOUT",
        Some(Fault::CommandRejected) => "CMD_REJECTED",
        Some(Fault::MotorDiagnostic) => "MOTOR",
        Some(Fault::Overcurrent) => "OVERCURRENT",
        Some(Fault::LoopOverrun) => "LOOP_OVERRUN",
        Some(Fault::BatteryCritical) => "BATTERY_CRITICAL",
        Some(Fault::ThermalCritical) => "THERMAL_CRITICAL",
        Some(Fault::Inactivity) => "INACTIVITY",
    }
}

pub fn format_help(f: &mut Formatter<'_>) {
    f.put_str(
        "HELP GET GETP WATCH SET WATCH <0-255> SET PROTOCOL LEGACY|HARDENED \
         SET <KEY> <VALUE> TRACE <0-255> ARM DISARM INIT SAVE DUMP ADC",
    );
    f.line();
}

pub fn format_get(f: &mut Formatter<'_>, t: &Telemetry) {
    f.put_str("GET tick=");
    f.put_i32(t.foc_ticks as i32);
    f.put_str(" oc=");
    f.put_i32(t.foc_overcurrent as i32);
    f.put_str(" skip=");
    f.put_i32(t.foc_skipped as i32);
    f.put_str(" cal=");
    f.put_bool(t.calibration_ready);
    f.put_str(" proto=");
    f.put_str(t.protocol.label());
    f.put_str(" mode=");
    f.put_str(t.mode.label());
    f.put_str(" arm=");
    f.put_bool(t.armed);
    f.put_str(" fault=");
    f.put_str(fault_label(t.fault));
    f.put_str(" faults=");
    f.put_i32(t.faults as i32);
    f.put_str(" rej=");
    f.put_i32(t.rejected as i32);
    f.put_str(" lnk=");
    match t.link_age_us {
        Some(age) => f.put_i32(age as i32),
        None => f.put_str("-"),
    }
    f.put_str(" steer=");
    f.put_i32(t.steer as i32);
    f.put_str(" speed=");
    f.put_i32(t.speed as i32);
    f.put_str(" dl=");
    f.put_u16_list(&t.duty_l);
    f.put_str(" dr=");
    f.put_u16_list(&t.duty_r);
    f.put_str(" bat=");
    f.put_i32(t.battery_raw as i32);
    f.put_str(" temp=");
    f.put_i32(t.temperature_decic as i32);
    f.put_str(" diag=");
    f.put_hex4(u16::from(t.diag_left) << 8 | u16::from(t.diag_right));
    f.put_str(" cyc=");
    f.put_i32(t.foc_cycles as i32);
    f.put_str(" max=");
    f.put_i32(t.foc_max_cycles as i32);
    f.put_str(" miss=");
    f.put_i32(t.foc_misses as i32);
    f.put_str(" nvm=");
    f.put_str(if t.params_from_flash {
        "FLASH"
    } else {
        "DEFAULT"
    });
    f.line();
}

/// One line per tunable: `P <NAME> <ACTIVE_VALUE>`. Scripts parse this to
/// discover the complete, bounded tuning surface without hard-coding keys.
pub fn format_params(f: &mut Formatter<'_>, params: &Parameters) {
    for (name, key) in Tunable::ALL {
        f.put_str("P ");
        f.put_str(name);
        f.push(b' ');
        f.put_i32(key.get(params) as i32);
        f.line();
    }
}

pub fn format_adc(f: &mut Formatter<'_>, fields: &[u16; ADC_FIELDS]) {
    f.put_str("ADC");
    for field in fields {
        f.push(b' ');
        f.put_i32(*field as i32);
    }
    f.line();
}

pub fn format_watch(f: &mut Formatter<'_>, t: &Telemetry) {
    f.put_str("W ");
    f.put_i32(t.now_us as i32);
    f.push(b' ');
    f.put_i32(t.steer as i32);
    f.push(b' ');
    f.put_i32(t.speed as i32);
    f.push(b' ');
    f.put_i32(t.target_left as i32);
    f.push(b' ');
    f.put_i32(t.target_right as i32);
    f.push(b' ');
    f.put_i32(t.dc_left);
    f.push(b' ');
    f.put_i32(t.dc_right);
    f.push(b' ');
    f.put_i32(t.foc_overcurrent as i32);
    f.push(b' ');
    f.put_i32(t.foc_skipped as i32);
    f.line();
}

/// One compact 5 ms sample. Flags: bit0 armed, bit1 calibration ready, bit2
/// fault latched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample {
    pub tick: u32,
    pub steer: i16,
    pub speed: i16,
    pub target_left: i16,
    pub target_right: i16,
    pub dc_left: i32,
    pub dc_right: i32,
    pub flags: u8,
}

impl Sample {
    pub const ZERO: Self = Self {
        tick: 0,
        steer: 0,
        speed: 0,
        target_left: 0,
        target_right: 0,
        dc_left: 0,
        dc_right: 0,
        flags: 0,
    };
}

/// Fixed-depth blackbox. `get(0)` is the oldest retained sample, so dumping in
/// index order yields a chronological trace after wraparound.
#[derive(Clone)]
pub struct Blackbox<const N: usize> {
    samples: [Sample; N],
    head: usize,
    len: usize,
}

impl<const N: usize> Blackbox<N> {
    pub const fn new() -> Self {
        Self {
            samples: [Sample::ZERO; N],
            head: 0,
            len: 0,
        }
    }

    pub fn push(&mut self, sample: Sample) {
        let tail = (self.head + self.len) % N;
        self.samples[tail] = sample;
        if self.len < N {
            self.len += 1;
        } else {
            self.head = (self.head + 1) % N;
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn get(&self, index: usize) -> Option<Sample> {
        if index >= self.len {
            return None;
        }
        Some(self.samples[(self.head + index) % N])
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

impl<const N: usize> Default for Blackbox<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub fn format_sample(f: &mut Formatter<'_>, index: usize, s: &Sample) {
    f.put_str("D ");
    f.put_i32(index as i32);
    f.push(b' ');
    f.put_i32(s.tick as i32);
    f.push(b' ');
    f.put_i32(s.steer as i32);
    f.push(b' ');
    f.put_i32(s.speed as i32);
    f.push(b' ');
    f.put_i32(s.target_left as i32);
    f.push(b' ');
    f.put_i32(s.target_right as i32);
    f.push(b' ');
    f.put_i32(s.dc_left);
    f.push(b' ');
    f.put_i32(s.dc_right);
    f.push(b' ');
    f.put_hex4(s.flags as u16);
    f.line();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn telemetry() -> Telemetry {
        Telemetry {
            now_us: 1_234,
            foc_ticks: 9_876,
            foc_overcurrent: 2,
            foc_skipped: 3,
            calibration_ready: true,
            protocol: ProtocolMode::Legacy,
            mode: SafeMode::RampToZero,
            armed: false,
            fault: Some(Fault::LinkTimeout),
            faults: 4,
            rejected: 5,
            link_age_us: Some(1_000),
            steer: -120,
            speed: 340,
            target_left: -460,
            target_right: 580,
            duty_l: [100, 1_000, 1_900],
            duty_r: [900, 1_100, 1_300],
            dc_left: -17,
            dc_right: 23,
            battery_raw: 1_400,
            temperature_decic: 349,
            diag_left: 1,
            diag_right: 0,
            foc_cycles: 1_100,
            foc_max_cycles: 1_800,
            foc_misses: 2,
            params_from_flash: false,
        }
    }

    #[test]
    fn line_buffer_commits_ignores_cr_and_drops_overflow() {
        let mut line = LineBuffer::<4>::new();
        assert_eq!(line.push(b'a'), None);
        assert_eq!(line.push(b'\r'), None);
        assert_eq!(line.push(b'b'), None);
        assert_eq!(line.push(b'\n'), Some("ab"));
        assert_eq!(line.push(b'x'), None);
        assert_eq!(line.push(b'y'), None);
        assert_eq!(line.push(b'z'), None);
        assert_eq!(line.push(b'w'), None);
        assert_eq!(line.push(b'v'), None);
        assert_eq!(line.push(b'\n'), Some(""));
        assert_eq!(line.push(b'o'), None);
        assert_eq!(line.push(b'k'), None);
        assert_eq!(line.push(b'\n'), Some("ok"));
    }

    #[test]
    fn command_parser_covers_the_full_grammar() {
        assert_eq!(parse_command("   "), Command::Empty);
        assert_eq!(parse_command("$help"), Command::Help);
        assert_eq!(parse_command("?"), Command::Help);
        assert_eq!(parse_command("$GET"), Command::Get);
        assert_eq!(parse_command("$watch"), Command::WatchToggle);
        assert_eq!(parse_command("$SET WATCH 7"), Command::SetWatch(7));
        assert_eq!(parse_command("$set watch 0"), Command::SetWatch(0));
        assert_eq!(
            parse_command("$SET PROTOCOL LEGACY"),
            Command::SetProtocol(ProtocolMode::Legacy)
        );
        assert_eq!(
            parse_command("$set protocol hardened"),
            Command::SetProtocol(ProtocolMode::Hardened)
        );
        assert_eq!(
            parse_command("$SET PROTOCOL OTHER"),
            Command::Rejected("$SET PROTOCOL OTHER")
        );
        assert_eq!(parse_command("$GETP"), Command::GetParams);
        assert_eq!(
            parse_command("$set iqkp 1500"),
            Command::SetParam(Tunable::IqKp, 1_500)
        );
        assert_eq!(parse_command("$TRACE 0"), Command::Trace(0));
        assert_eq!(parse_command("$TRACE 16"), Command::Trace(16));
        assert_eq!(parse_command("$TRACE"), Command::Rejected("$TRACE"));
        assert_eq!(parse_command("$arm"), Command::Arm);
        assert_eq!(parse_command("$DISARM"), Command::Disarm);
        assert_eq!(parse_command("$INIT"), Command::Init);
        assert_eq!(parse_command("$SAVE"), Command::Save);
        assert_eq!(parse_command("$DUMP"), Command::Dump);
        assert_eq!(parse_command("$ADC"), Command::Adc);
        assert_eq!(
            parse_command("$SET MODE FREEWHEEL"),
            Command::Rejected("$SET MODE FREEWHEEL")
        );
        assert_eq!(
            parse_command("$SET WATCH 999"),
            Command::Rejected("$SET WATCH 999")
        );
        assert_eq!(parse_command("$NOPE"), Command::Rejected("$NOPE"));
    }

    #[test]
    fn formatter_handles_signs_and_truncation() {
        let mut buf = [0u8; 8];
        let mut f = Formatter::new(&mut buf);
        f.put_i32(i32::MIN);
        f.push(b' ');
        f.put_i32(-7);
        f.push(b' ');
        f.put_hex4(0xAB3);
        f.line();
        assert_eq!(f.as_str(), "-2147483");
    }

    #[test]
    fn formatter_renders_get_and_watch() {
        let t = telemetry();
        let mut get_buf = [0u8; REPLY_MAX];
        {
            let mut f = Formatter::new(&mut get_buf);
            format_get(&mut f, &t);
            assert!(f.as_str().starts_with(
                "GET tick=9876 oc=2 skip=3 cal=1 proto=LEGACY8 mode=RAMP_ZERO arm=0 fault=LINK_TIMEOUT"
            ));
            assert!(f.as_str().contains("steer=-120 speed=340"));
            assert!(f.as_str().contains("cyc=1100 max=1800 miss=2"));
        }
        let mut watch_buf = [0u8; REPLY_MAX];
        let mut f = Formatter::new(&mut watch_buf);
        format_watch(&mut f, &t);
        assert_eq!(f.as_str(), "W 1234 -120 340 -460 580 -17 23 2 3\n");
    }

    #[test]
    fn byte_ring_wraps_in_fifo_order() {
        let mut ring = ByteRing::<3>::new();
        assert!(ring.push(1));
        assert!(ring.push(2));
        assert!(ring.push(3));
        assert!(!ring.push(4));
        assert_eq!(ring.pop(), Some(1));
        assert!(ring.push(4));
        assert_eq!(ring.pop(), Some(2));
        assert_eq!(ring.pop(), Some(3));
        assert_eq!(ring.pop(), Some(4));
        assert_eq!(ring.pop(), None);
    }

    #[test]
    fn blackbox_reports_oldest_first_after_wrap() {
        let mut blackbox = Blackbox::<3>::new();
        for tick in 1..=5 {
            let mut s = Sample::ZERO;
            s.tick = tick;
            blackbox.push(s);
        }
        assert_eq!(blackbox.len(), 3);
        let ticks: [u32; 3] = core::array::from_fn(|i| blackbox.get(i).unwrap().tick);
        assert_eq!(ticks, [3, 4, 5]);
        let mut buf = [0u8; REPLY_MAX];
        let mut f = Formatter::new(&mut buf);
        format_sample(&mut f, 0, &blackbox.get(0).unwrap());
        assert_eq!(f.as_str(), "D 0 3 0 0 0 0 0 0 0000\n");
        blackbox.clear();
        assert_eq!(blackbox.len(), 0);
    }
}
