//! Wire contracts: a hardened COBS/CRC16 frame and the stock EFeru frames.
//!
//! The decoder validates CRC and length before a `Frame` exists, so validation
//! at the application boundary can check version, sequence, age and semantics
//! in that order (see `docs/skills/rust_robotics_guidelines.md` section 3).

use crate::config::{
    COMMAND_LIMIT, COMMAND_TIMEOUT_US, LEGACY_START_FRAME, PROTOCOL_VERSION, WIRE_MAX,
};
use crate::trace::{TraceData, TRACE_PAYLOAD_LEN};

pub const PAYLOAD_MAX: usize = 96;
pub const MSG_COMMAND: u8 = 1;
pub const MSG_FEEDBACK: u8 = 2;
pub const MSG_TRACE: u8 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandError {
    BadLength,
    BadVersion,
    BadCrc,
    BadSequence,
    Stale,
    OutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Setpoint {
    pub steer: i16,
    pub speed: i16,
}

impl Setpoint {
    pub const ZERO: Self = Self { steer: 0, speed: 0 };
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub version: u8,
    pub msg_type: u8,
    pub flags: u8,
    pub seq: u16,
    pub timestamp_us: u32,
    pub payload: [u8; PAYLOAD_MAX],
    pub payload_len: u8,
}

impl Frame {
    pub fn command(seq: u16, now_us: u32, steer: i16, speed: i16) -> Self {
        let mut payload = [0u8; PAYLOAD_MAX];
        payload[0..2].copy_from_slice(&steer.to_le_bytes());
        payload[2..4].copy_from_slice(&speed.to_le_bytes());
        Self {
            version: PROTOCOL_VERSION,
            msg_type: MSG_COMMAND,
            flags: 0,
            seq,
            timestamp_us: now_us,
            payload,
            payload_len: 4,
        }
    }

    /// Encodes into `out` as crc16(CCITT-FALSE) over header+payload, then COBS
    /// with a 0x00 delimiter. Returns the number of bytes written.
    pub fn encode(&self, out: &mut [u8; WIRE_MAX]) -> Option<usize> {
        let len = self.payload_len as usize;
        if len > PAYLOAD_MAX {
            return None;
        }
        let mut decoded = [0u8; WIRE_MAX];
        let mut n = 0usize;
        decoded[n] = self.version;
        n += 1;
        decoded[n] = self.msg_type;
        n += 1;
        decoded[n] = self.flags;
        n += 1;
        decoded[n..n + 2].copy_from_slice(&self.seq.to_le_bytes());
        n += 2;
        decoded[n..n + 4].copy_from_slice(&self.timestamp_us.to_le_bytes());
        n += 4;
        decoded[n] = self.payload_len;
        n += 1;
        decoded[n..n + len].copy_from_slice(&self.payload[..len]);
        n += len;
        let crc = crc16(&decoded[..n]);
        decoded[n..n + 2].copy_from_slice(&crc.to_le_bytes());
        n += 2;
        let encoded = cobs_encode(&decoded[..n], out)?;
        if encoded + 1 > out.len() {
            return None;
        }
        out[encoded] = 0;
        Some(encoded + 1)
    }
}

/// Length of the hardened feedback payload. Mirrored in `jbot_pico_core`.
pub const FEEDBACK_PAYLOAD_LEN: usize = 25;

/// Feedback carried by a hardened frame. The first seven fields mirror the
/// legacy 18-byte frame so the Pico control loop can use one `Feedback` type;
/// the rest expose the diagnostics the stock wire contract cannot carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FeedbackData {
    pub cmd1: i16,
    pub cmd2: i16,
    pub speed_right_rpm: i16,
    pub speed_left_rpm: i16,
    pub battery_cv: i16,
    pub board_temp_decic: i16,
    pub led: u16,
    pub err_code: u8,
    pub hall_left: u8,
    pub hall_right: u8,
    pub dc_left: i16,
    pub dc_right: i16,
    pub duty_left: u16,
    pub duty_right: u16,
}

impl Frame {
    /// Build a hardened feedback frame. `seq` echoes the last accepted command
    /// so the Pico can measure control-loop latency from its own timestamp.
    pub fn feedback(seq: u16, now_us: u32, data: &FeedbackData) -> Self {
        let mut payload = [0u8; PAYLOAD_MAX];
        let fields: [i16; 6] = [
            data.cmd1,
            data.cmd2,
            data.speed_right_rpm,
            data.speed_left_rpm,
            data.battery_cv,
            data.board_temp_decic,
        ];
        for (index, value) in fields.iter().enumerate() {
            payload[index * 2..index * 2 + 2].copy_from_slice(&value.to_le_bytes());
        }
        payload[12..14].copy_from_slice(&data.led.to_le_bytes());
        payload[14] = data.err_code;
        payload[15] = data.hall_left;
        payload[16] = data.hall_right;
        payload[17..19].copy_from_slice(&data.dc_left.to_le_bytes());
        payload[19..21].copy_from_slice(&data.dc_right.to_le_bytes());
        payload[21..23].copy_from_slice(&data.duty_left.to_le_bytes());
        payload[23..25].copy_from_slice(&data.duty_right.to_le_bytes());
        Self {
            version: PROTOCOL_VERSION,
            msg_type: MSG_FEEDBACK,
            flags: 0,
            seq,
            timestamp_us: now_us,
            payload,
            payload_len: FEEDBACK_PAYLOAD_LEN as u8,
        }
    }

    /// Build a hardened frame carrying one compact FOC trace snapshot.
    pub fn trace(seq: u16, now_us: u32, data: &TraceData) -> Self {
        Self {
            version: PROTOCOL_VERSION,
            msg_type: MSG_TRACE,
            flags: data.flags,
            seq,
            timestamp_us: now_us,
            payload: {
                let mut payload = [0u8; PAYLOAD_MAX];
                payload[..TRACE_PAYLOAD_LEN].copy_from_slice(&data.to_payload());
                payload
            },
            payload_len: TRACE_PAYLOAD_LEN as u8,
        }
    }
}

/// Encode one trace frame into `out`, returning the byte count.
pub fn encode_trace(
    seq: u16,
    now_us: u32,
    data: &TraceData,
    out: &mut [u8; WIRE_MAX],
) -> Option<usize> {
    Frame::trace(seq, now_us, data).encode(out)
}

/// Decode a trace frame payload. Returns `None` for any other message type.
pub fn decode_trace_data(frame: &Frame) -> Option<TraceData> {
    if frame.msg_type != MSG_TRACE || frame.payload_len as usize != TRACE_PAYLOAD_LEN {
        return None;
    }
    let mut payload = [0u8; TRACE_PAYLOAD_LEN];
    payload.copy_from_slice(&frame.payload[..TRACE_PAYLOAD_LEN]);
    Some(TraceData::from_payload(&payload))
}

/// Decode the hardened feedback payload into the shared telemetry structure.
pub fn decode_feedback_data(frame: &Frame) -> Result<FeedbackData, CommandError> {
    if frame.msg_type != MSG_FEEDBACK || frame.payload_len as usize != FEEDBACK_PAYLOAD_LEN {
        return Err(CommandError::BadLength);
    }
    let p = &frame.payload;
    let word = |at: usize| i16::from_le_bytes([p[at], p[at + 1]]);
    Ok(FeedbackData {
        cmd1: word(0),
        cmd2: word(2),
        speed_right_rpm: word(4),
        speed_left_rpm: word(6),
        battery_cv: word(8),
        board_temp_decic: word(10),
        led: u16::from_le_bytes([p[12], p[13]]),
        err_code: p[14],
        hall_left: p[15],
        hall_right: p[16],
        dc_left: word(17),
        dc_right: word(19),
        duty_left: u16::from_le_bytes([p[21], p[22]]),
        duty_right: u16::from_le_bytes([p[23], p[24]]),
    })
}

/// Encode one hardened feedback frame into `out`, returning the byte count.
pub fn encode_hardened_feedback(
    seq: u16,
    now_us: u32,
    data: &FeedbackData,
    out: &mut [u8; WIRE_MAX],
) -> Option<usize> {
    Frame::feedback(seq, now_us, data).encode(out)
}

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn cobs_encode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut code_index = 0usize;
    let mut write = 0usize;
    let mut code = 1u8;
    out[0] = 1;
    write += 1;
    for &byte in input {
        if byte == 0 || code == 0xFF {
            out[code_index] = code;
            if write >= out.len() {
                return None;
            }
            code_index = write;
            out[write] = 0;
            write += 1;
            code = 1;
            if byte == 0 {
                continue;
            }
        }
        if write >= out.len() {
            return None;
        }
        out[write] = byte;
        write += 1;
        code += 1;
    }
    out[code_index] = code;
    Some(write)
}

fn cobs_decode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut read = 0usize;
    let mut write = 0usize;
    while read < input.len() {
        let code = input[read] as usize;
        if code == 0 {
            return None;
        }
        read += 1;
        let count = code - 1;
        if read + count > input.len() || write + count > out.len() {
            return None;
        }
        out[write..write + count].copy_from_slice(&input[read..read + count]);
        write += count;
        read += count;
        if code != 0xFF && read < input.len() {
            if write >= out.len() {
                return None;
            }
            out[write] = 0;
            write += 1;
        }
    }
    Some(write)
}

/// Streaming decoder. Arbitrary chunk boundaries, garbage and oversize frames
/// never panic and always resynchronise at the next 0x00 delimiter.
pub struct FrameDecoder {
    encoded: [u8; WIRE_MAX],
    len: usize,
    discarding: bool,
    pub crc_errors: u32,
    pub oversize: u32,
    pub resyncs: u32,
    pub frames: u32,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self {
            encoded: [0u8; WIRE_MAX],
            len: 0,
            discarding: false,
            crc_errors: 0,
            oversize: 0,
            resyncs: 0,
            frames: 0,
        }
    }

    pub fn feed<F: FnMut(Frame)>(&mut self, bytes: &[u8], on_frame: &mut F) {
        for &byte in bytes {
            if byte == 0 {
                if self.discarding {
                    self.discarding = false;
                    self.resyncs = self.resyncs.wrapping_add(1);
                    self.len = 0;
                    continue;
                }
                if self.len > 0 {
                    if let Some(frame) = self.finish() {
                        self.frames = self.frames.wrapping_add(1);
                        on_frame(frame);
                    }
                }
                self.len = 0;
                continue;
            }
            if self.discarding {
                continue;
            }
            if self.len == self.encoded.len() {
                self.oversize = self.oversize.wrapping_add(1);
                self.len = 0;
                self.discarding = true;
                continue;
            }
            self.encoded[self.len] = byte;
            self.len += 1;
        }
    }

    fn finish(&mut self) -> Option<Frame> {
        let mut decoded = [0u8; WIRE_MAX];
        let n = match cobs_decode(&self.encoded[..self.len], &mut decoded) {
            Some(n) => n,
            None => {
                self.crc_errors = self.crc_errors.wrapping_add(1);
                return None;
            }
        };
        if n < 13 {
            self.crc_errors = self.crc_errors.wrapping_add(1);
            return None;
        }
        let expected = u16::from_le_bytes([decoded[n - 2], decoded[n - 1]]);
        if crc16(&decoded[..n - 2]) != expected {
            self.crc_errors = self.crc_errors.wrapping_add(1);
            return None;
        }
        let payload_len = decoded[9] as usize;
        if payload_len > PAYLOAD_MAX || n != 12 + payload_len {
            self.crc_errors = self.crc_errors.wrapping_add(1);
            return None;
        }
        let mut payload = [0u8; PAYLOAD_MAX];
        payload[..payload_len].copy_from_slice(&decoded[10..10 + payload_len]);
        Some(Frame {
            version: decoded[0],
            msg_type: decoded[1],
            flags: decoded[2],
            seq: u16::from_le_bytes([decoded[3], decoded[4]]),
            timestamp_us: u32::from_le_bytes([decoded[5], decoded[6], decoded[7], decoded[8]]),
            payload,
            payload_len: decoded[9],
        })
    }
}

/// Selected control-link contract. Both decoders are always compiled; the mode
/// is a configuration value so the pinned Pico and the hardened endpoint can
/// coexist on the bench without reflashing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolMode {
    Legacy,
    Hardened,
}

impl ProtocolMode {
    /// Keeps the currently flashed Pico link alive until the mirrored hardened
    /// contract has passed golden-vector and HIL checks.
    pub const DEFAULT: Self = Self::Legacy;

    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Legacy),
            1 => Some(Self::Hardened),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Legacy => 0,
            Self::Hardened => 1,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Legacy => "LEGACY8",
            Self::Hardened => "HARDENED",
        }
    }
}

/// Streaming command decoder for either wire contract.
///
/// Hardened frames are COBS-encoded and delimited by 0x00. Legacy frames are a
/// fixed 8-byte structure introduced by 0xABCD with no delimiter, so resync is
/// best-effort: a bad checksum drops the candidate and scanning restarts.
pub struct CommandDecoder {
    mode: ProtocolMode,
    frames: FrameDecoder,
    legacy: [u8; 8],
    legacy_len: usize,
    pub legacy_rejected: u32,
}

impl Default for CommandDecoder {
    fn default() -> Self {
        Self::new(ProtocolMode::DEFAULT)
    }
}

impl CommandDecoder {
    pub fn new(mode: ProtocolMode) -> Self {
        Self {
            mode,
            frames: FrameDecoder::new(),
            legacy: [0; 8],
            legacy_len: 0,
            legacy_rejected: 0,
        }
    }

    pub fn mode(&self) -> ProtocolMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: ProtocolMode) {
        if self.mode != mode {
            self.mode = mode;
            self.legacy_len = 0;
        }
    }

    /// Feed a chunk from the UART. Valid commands are delivered as a
    /// `(Setpoint, Option<seq>)` pair. Hardened frames carry their own sequence;
    /// legacy frames carry none, so the caller synthesises one to preserve the
    /// `SafeController` one-shot contract.
    pub fn feed<F: FnMut(Setpoint, Option<u16>)>(
        &mut self,
        bytes: &[u8],
        now_us: u32,
        on_command: &mut F,
    ) {
        match self.mode {
            ProtocolMode::Hardened => self.feed_hardened(bytes, now_us, on_command),
            ProtocolMode::Legacy => self.feed_legacy(bytes, on_command),
        }
    }

    fn feed_hardened<F: FnMut(Setpoint, Option<u16>)>(
        &mut self,
        bytes: &[u8],
        now_us: u32,
        on_command: &mut F,
    ) {
        self.frames.feed(bytes, &mut |frame| {
            if let Ok(setpoint) = validate(&frame, now_us, None) {
                on_command(setpoint, Some(frame.seq));
            }
        });
    }

    fn feed_legacy<F: FnMut(Setpoint, Option<u16>)>(&mut self, bytes: &[u8], on_command: &mut F) {
        for &byte in bytes {
            if self.legacy_len == 0 {
                let start = LEGACY_START_FRAME as u8;
                if byte == start {
                    self.legacy[0] = byte;
                    self.legacy_len = 1;
                }
                continue;
            }
            if self.legacy_len == 1 && byte != (LEGACY_START_FRAME >> 8) as u8 {
                // Restart scanning without losing a start byte in the stream.
                self.legacy_len = 0;
                if byte == LEGACY_START_FRAME as u8 {
                    self.legacy[0] = byte;
                    self.legacy_len = 1;
                }
                continue;
            }
            self.legacy[self.legacy_len] = byte;
            self.legacy_len += 1;
            if self.legacy_len == self.legacy.len() {
                match decode_legacy_command(&self.legacy) {
                    Ok(setpoint) => on_command(setpoint, None),
                    Err(_) => self.legacy_rejected = self.legacy_rejected.wrapping_add(1),
                }
                self.legacy_len = 0;
            }
        }
    }
}

/// Accept only the newest valid command. Sequence gaps are diagnostics, not
/// permission to apply an older frame.
pub fn validate(
    frame: &Frame,
    now_us: u32,
    last_seq: Option<u16>,
) -> Result<Setpoint, CommandError> {
    if frame.payload_len as usize != 4 {
        return Err(CommandError::BadLength);
    }
    if frame.version != PROTOCOL_VERSION {
        return Err(CommandError::BadVersion);
    }
    if frame.msg_type != MSG_COMMAND {
        return Err(CommandError::BadLength);
    }
    if let Some(previous) = last_seq {
        let distance = frame.seq.wrapping_sub(previous) as i16;
        if distance <= 0 {
            return Err(CommandError::BadSequence);
        }
    }
    if now_us.wrapping_sub(frame.timestamp_us) > COMMAND_TIMEOUT_US {
        return Err(CommandError::Stale);
    }
    let steer = i16::from_le_bytes([frame.payload[0], frame.payload[1]]);
    let speed = i16::from_le_bytes([frame.payload[2], frame.payload[3]]);
    if steer.unsigned_abs() > COMMAND_LIMIT as u16 || speed.unsigned_abs() > COMMAND_LIMIT as u16 {
        return Err(CommandError::OutOfRange);
    }
    Ok(Setpoint { steer, speed })
}

/// Stock 8-byte command: 0xABCD, steer, speed, XOR checksum.
pub fn encode_legacy_command(steer: i16, speed: i16) -> [u8; 8] {
    let checksum = LEGACY_START_FRAME ^ steer as u16 ^ speed as u16;
    let mut out = [0u8; 8];
    out[0..2].copy_from_slice(&LEGACY_START_FRAME.to_le_bytes());
    out[2..4].copy_from_slice(&steer.to_le_bytes());
    out[4..6].copy_from_slice(&speed.to_le_bytes());
    out[6..8].copy_from_slice(&checksum.to_le_bytes());
    out
}

pub fn decode_legacy_command(bytes: &[u8; 8]) -> Result<Setpoint, CommandError> {
    let start = u16::from_le_bytes([bytes[0], bytes[1]]);
    let steer = i16::from_le_bytes([bytes[2], bytes[3]]);
    let speed = i16::from_le_bytes([bytes[4], bytes[5]]);
    let checksum = u16::from_le_bytes([bytes[6], bytes[7]]);
    if start != LEGACY_START_FRAME {
        return Err(CommandError::BadVersion);
    }
    if checksum != start ^ steer as u16 ^ speed as u16 {
        return Err(CommandError::BadCrc);
    }
    if steer.unsigned_abs() > COMMAND_LIMIT as u16 || speed.unsigned_abs() > COMMAND_LIMIT as u16 {
        return Err(CommandError::OutOfRange);
    }
    Ok(Setpoint { steer, speed })
}

/// Stock 18-byte feedback; `cmd1`/`cmd2` are i16 in the pinned C source.
pub fn encode_legacy_feedback(
    cmd1: i16,
    cmd2: i16,
    speed_r: i16,
    speed_l: i16,
    battery_cv: i16,
    board_temp_decic: i16,
    led: u16,
) -> [u8; 18] {
    let fields = [
        LEGACY_START_FRAME as i16,
        cmd1,
        cmd2,
        speed_r,
        speed_l,
        battery_cv,
        board_temp_decic,
        led as i16,
    ];
    let mut checksum = 0u16;
    for value in fields {
        checksum ^= value as u16;
    }
    let mut out = [0u8; 18];
    for (index, value) in fields.iter().enumerate() {
        out[index * 2..index * 2 + 2].copy_from_slice(&value.to_le_bytes());
    }
    out[16..18].copy_from_slice(&checksum.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_check_value() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn cobs_round_trip_with_embedded_zeros() {
        let input = [1u8, 0, 2, 3, 0, 0, 4];
        let mut encoded = [0u8; 64];
        let n = cobs_encode(&input, &mut encoded).unwrap();
        let mut decoded = [0u8; 64];
        let m = cobs_decode(&encoded[..n], &mut decoded).unwrap();
        assert_eq!(&decoded[..m], &input);
    }

    #[test]
    fn frame_survives_arbitrary_chunk_boundaries() {
        let frame = Frame::command(7, 123_456, 120, -80);
        let mut wire = [0u8; WIRE_MAX];
        let n = frame.encode(&mut wire).unwrap();
        let mut decoder = FrameDecoder::new();
        let mut received = None;
        for chunk in wire[..n].chunks(3) {
            decoder.feed(chunk, &mut |f| received = Some(f));
        }
        assert_eq!(decoder.frames, 1);
        let got = received.expect("frame");
        assert_eq!(got.seq, 7);
        assert_eq!(got.timestamp_us, 123_456);
        assert_eq!(
            validate(&got, 123_500, None).unwrap(),
            Setpoint {
                steer: 120,
                speed: -80
            }
        );
    }

    #[test]
    fn corrupted_and_oversize_frames_never_panic() {
        let mut decoder = FrameDecoder::new();
        decoder.feed(&[0x03, 0x11, 0x22, 0x99, 0], &mut |_| {});
        assert_eq!(decoder.crc_errors, 1);
        let mut long = [0x42u8; WIRE_MAX + 6];
        long[WIRE_MAX + 5] = 0;
        decoder.feed(&long, &mut |_| {});
        assert_eq!(decoder.oversize, 1);
        assert_eq!(decoder.resyncs, 1);
    }

    #[test]
    fn stale_duplicate_and_out_of_range_are_rejected() {
        let frame = Frame::command(9, 1_000, 2_000, 0);
        assert_eq!(validate(&frame, 1_010, None), Err(CommandError::OutOfRange));
        let good = Frame::command(9, 1_000, 100, 0);
        assert_eq!(validate(&good, 200_000, None), Err(CommandError::Stale));
        assert_eq!(
            validate(&good, 1_010, Some(9)),
            Err(CommandError::BadSequence)
        );
        assert_eq!(
            validate(&good, 1_010, Some(10)),
            Err(CommandError::BadSequence)
        );
    }

    #[test]
    fn hardened_golden_vector() {
        let frame = Frame::command(0x0102, 0x0304_0506, 100, -200);
        let mut wire = [0u8; WIRE_MAX];
        let n = frame.encode(&mut wire).unwrap();
        assert_eq!(
            &wire[..n],
            &[
                0x03, 0x02, 0x01, 0x09, 0x02, 0x01, 0x06, 0x05, 0x04, 0x03, 0x04, 0x64, 0x05, 0x38,
                0xFF, 0xC4, 0x40, 0x00,
            ]
        );
    }

    #[test]
    fn hardened_feedback_golden_vector() {
        let data = FeedbackData {
            cmd1: 100,
            cmd2: -100,
            speed_right_rpm: 200,
            speed_left_rpm: -200,
            battery_cv: 4_150,
            board_temp_decic: 349,
            led: 0,
            err_code: 0x01,
            hall_left: 3,
            hall_right: 5,
            dc_left: 12,
            dc_right: -13,
            duty_left: 110,
            duty_right: 120,
        };
        let mut wire = [0u8; WIRE_MAX];
        let n = encode_hardened_feedback(0x0103, 0x0304_0510, &data, &mut wire).unwrap();
        assert_eq!(
            &wire[..n],
            &[
                0x03, 0x02, 0x02, 0x09, 0x03, 0x01, 0x10, 0x05, 0x04, 0x03, 0x19, 0x64, 0x04, 0x9C,
                0xFF, 0xC8, 0x07, 0x38, 0xFF, 0x36, 0x10, 0x5D, 0x01, 0x01, 0x05, 0x01, 0x03, 0x05,
                0x0C, 0x04, 0xF3, 0xFF, 0x6E, 0x02, 0x78, 0x03, 0x43, 0x68, 0x00,
            ]
        );
        let mut decoder = FrameDecoder::new();
        let mut received = None;
        decoder.feed(&wire[..n], &mut |frame| received = Some(frame));
        let frame = received.unwrap();
        assert_eq!(frame.seq, 0x0103);
        assert_eq!(decode_feedback_data(&frame).unwrap(), data);
    }

    #[test]
    fn protocol_mode_switch_accepts_both_contracts() {
        let legacy = encode_legacy_command(120, -80);
        let frame = Frame::command(4, 900, -30, 60);
        let mut hardened = [0u8; WIRE_MAX];
        let n = frame.encode(&mut hardened).unwrap();

        let mut decoder = CommandDecoder::default();
        assert_eq!(decoder.mode(), ProtocolMode::Legacy);
        let mut received = [Setpoint::ZERO; 2];
        let mut count = 0;
        decoder.feed(&legacy, 1_000, &mut |s, seq| {
            assert_eq!(seq, None);
            received[count] = s;
            count += 1;
        });
        assert_eq!(count, 1);
        assert_eq!(
            received[0],
            Setpoint {
                steer: 120,
                speed: -80
            }
        );

        decoder.set_mode(ProtocolMode::Hardened);
        decoder.feed(&hardened[..n], 1_000, &mut |s, seq| {
            assert_eq!(seq, Some(4));
            received[count] = s;
            count += 1;
        });
        assert_eq!(count, 2);
        assert_eq!(
            received[1],
            Setpoint {
                steer: -30,
                speed: 60
            }
        );

        assert_eq!(ProtocolMode::from_u8(0), Some(ProtocolMode::Legacy));
        assert_eq!(ProtocolMode::from_u8(1), Some(ProtocolMode::Hardened));
        assert_eq!(ProtocolMode::from_u8(2), None);
        assert_eq!(ProtocolMode::Hardened.as_u8(), 1);
    }

    #[test]
    fn legacy_decoder_resynchronises_after_garbage() {
        let mut decoder = CommandDecoder::new(ProtocolMode::Legacy);
        let mut received = None;
        let mut stream = [0u8; 16];
        // A well-framed candidate with a deliberately wrong checksum.
        stream[..8].copy_from_slice(&[0xCD, 0xAB, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00]);
        stream[8..].copy_from_slice(&encode_legacy_command(9, -9));
        decoder.feed(&stream, 0, &mut |s, _| received = Some(s));
        assert_eq!(decoder.legacy_rejected, 1);
        assert_eq!(
            received,
            Some(Setpoint {
                steer: 9,
                speed: -9
            })
        );
    }

    #[test]
    fn arbitrary_byte_streams_are_tolerated() {
        let mut decoder = CommandDecoder::new(ProtocolMode::Hardened);
        let mut state = 0x1234_5678u32;
        let mut accepted = 0usize;
        for _ in 0..4_000 {
            // A deterministic LCG keeps this a regression test, not a fuzzer.
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let byte = (state >> 24) as u8;
            decoder.feed(&[byte], 0, &mut |_, _| accepted += 1);
        }
        // A random stream must not authenticate a command by accident; only
        // the counters are allowed to move.
        assert!(accepted <= 1);

        let mut legacy = CommandDecoder::new(ProtocolMode::Legacy);
        state = 0xDEAD_BEEF;
        for _ in 0..4_000 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let byte = (state >> 24) as u8;
            legacy.feed(&[byte], 0, &mut |_, _| accepted += 1);
        }
        assert!(accepted <= 2);
    }

    #[test]
    fn legacy_golden_vectors() {
        assert_eq!(
            encode_legacy_command(0x1234, 0x5678),
            [0xCD, 0xAB, 0x34, 0x12, 0x78, 0x56, 0x81, 0xEF]
        );
        assert_eq!(
            encode_legacy_feedback(100, -100, 200, -200, 4150, 349, 0),
            [
                0xCD, 0xAB, 0x64, 0x00, 0x9C, 0xFF, 0xC8, 0x00, 0x38, 0xFF, 0x36, 0x10, 0x5D, 0x01,
                0x00, 0x00, 0xAE, 0xBA
            ]
        );
    }
}
