//! Hardened control-link codec shared with the STM32 torque node.
//!
//! This is a byte-for-byte mirror of `f103_core::protocol`. Keeping the two
//! implementations separate modules (rather than a shared crate) preserves the
//! existing independent workspaces while the golden vectors below pin the
//! contract on both sides.

/// Protocol version from the STM32 firmware configuration.
pub const PROTOCOL_VERSION: u8 = 2;
pub const MSG_COMMAND: u8 = 1;
pub const MSG_FEEDBACK: u8 = 2;
pub const PAYLOAD_MAX: usize = 32;
pub const WIRE_MAX: usize = 256;
pub const COMMAND_TIMEOUT_US: u32 = 100_000;
/// Length of the hardened feedback payload. Mirrored in `f103_core::protocol`.
pub const FEEDBACK_PAYLOAD_LEN: usize = 25;
pub const START_FRAME: u16 = 0xABCD;
pub const TORQUE_LIMIT: i16 = 1_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolMode {
    Legacy,
    Hardened,
}

impl ProtocolMode {
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

    /// Build a hardened feedback frame. The first seven fields mirror the
    /// legacy 18-byte frame so the control loop can use one `Feedback` type.
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

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
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

/// Hardened feedback telemetry. The wire layout is byte-identical to
/// `f103_core::protocol::FeedbackData`.
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

impl FeedbackData {
    /// Narrow the hardened frame to the same `Feedback` the legacy parser
    /// produces, so the control task needs no protocol-specific branch. The
    /// hardened producer reports its raw input echo in `cmd1`/`cmd2`; it does
    /// not measure the post-filter command, so these fields are only an
    /// approximation of the applied wheel command in that mode.
    pub fn to_eferu(self) -> crate::eferu::Feedback {
        crate::eferu::Feedback {
            applied_left_units: self.cmd1,
            applied_right_units: self.cmd2,
            speed_right_rpm: self.speed_right_rpm,
            speed_left_rpm: self.speed_left_rpm,
            battery_voltage_centivolts: self.battery_cv,
            board_temp_decideg_c: self.board_temp_decic,
            led: self.led,
        }
    }
}

/// Decode a hardened feedback frame. Returns `None` for the wrong message type
/// or payload length; CRC and framing were already checked by `FrameDecoder`.
pub fn decode_feedback_data(frame: &Frame) -> Option<FeedbackData> {
    if frame.msg_type != MSG_FEEDBACK || frame.payload_len as usize != FEEDBACK_PAYLOAD_LEN {
        return None;
    }
    let p = &frame.payload;
    let word = |at: usize| i16::from_le_bytes([p[at], p[at + 1]]);
    Some(FeedbackData {
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

/// Streaming decoder with the same counters and resync behaviour as the STM32.
pub struct FrameDecoder {
    encoded: [u8; WIRE_MAX],
    decoded: [u8; WIRE_MAX],
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
            encoded: [0; WIRE_MAX],
            decoded: [0; WIRE_MAX],
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
        let n = match cobs_decode(&self.encoded[..self.len], &mut self.decoded) {
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
        let decoded = &self.decoded;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    BadLength,
    BadVersion,
    BadSequence,
    Stale,
    OutOfRange,
}

/// Same validation order as the STM32: length, version, message type, sequence,
/// age, then value range.
pub fn validate(frame: &Frame, now_us: u32, last_seq: Option<u16>) -> Result<Setpoint, FrameError> {
    if frame.payload_len as usize != 4 {
        return Err(FrameError::BadLength);
    }
    if frame.version != PROTOCOL_VERSION {
        return Err(FrameError::BadVersion);
    }
    if frame.msg_type != MSG_COMMAND {
        return Err(FrameError::BadLength);
    }
    if let Some(previous) = last_seq {
        if (frame.seq.wrapping_sub(previous) as i16) <= 0 {
            return Err(FrameError::BadSequence);
        }
    }
    if now_us.wrapping_sub(frame.timestamp_us) > COMMAND_TIMEOUT_US {
        return Err(FrameError::Stale);
    }
    let steer = i16::from_le_bytes([frame.payload[0], frame.payload[1]]);
    let speed = i16::from_le_bytes([frame.payload[2], frame.payload[3]]);
    if steer.unsigned_abs() > TORQUE_LIMIT as u16 || speed.unsigned_abs() > TORQUE_LIMIT as u16 {
        return Err(FrameError::OutOfRange);
    }
    Ok(Setpoint { steer, speed })
}

/// Emits either wire contract. The legacy path is byte-identical to
/// `eferu::Command::encode`; the hardened path uses COBS + CRC16.
pub struct CommandEncoder {
    mode: ProtocolMode,
}

impl Default for CommandEncoder {
    fn default() -> Self {
        Self::new(ProtocolMode::DEFAULT)
    }
}

impl CommandEncoder {
    pub const fn new(mode: ProtocolMode) -> Self {
        Self { mode }
    }

    pub const fn mode(&self) -> ProtocolMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: ProtocolMode) {
        self.mode = mode;
    }

    pub fn encode(
        &self,
        seq: u16,
        now_us: u32,
        steer: i16,
        speed: i16,
        out: &mut [u8],
    ) -> Option<usize> {
        match self.mode {
            ProtocolMode::Legacy => {
                let frame = encode_legacy_command(steer, speed);
                out[..frame.len()].copy_from_slice(&frame);
                Some(frame.len())
            }
            ProtocolMode::Hardened => Frame::command(seq, now_us, steer, speed).encode(out),
        }
    }
}

pub fn encode_legacy_command(steer: i16, speed: i16) -> [u8; 8] {
    let checksum = START_FRAME ^ steer as u16 ^ speed as u16;
    let mut out = [0u8; 8];
    out[0..2].copy_from_slice(&START_FRAME.to_le_bytes());
    out[2..4].copy_from_slice(&steer.to_le_bytes());
    out[4..6].copy_from_slice(&speed.to_le_bytes());
    out[6..8].copy_from_slice(&checksum.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_value_is_shared() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn legacy_golden_vector_matches_stm32() {
        assert_eq!(
            encode_legacy_command(0x1234, 0x5678),
            [0xCD, 0xAB, 0x34, 0x12, 0x78, 0x56, 0x81, 0xEF]
        );
    }

    #[test]
    fn hardened_golden_vector_matches_stm32() {
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
    fn hardened_feedback_golden_vector_matches_stm32() {
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
        let n = Frame::feedback(0x0103, 0x0304_0510, &data)
            .encode(&mut wire)
            .unwrap();
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
        let decoded = decode_feedback_data(&received.unwrap()).unwrap();
        assert_eq!(decoded, data);
        assert_eq!(decoded.to_eferu().battery_voltage_v(), 41.50);
    }

    #[test]
    fn encoder_switch_is_mirrored() {
        let mut encoder = CommandEncoder::default();
        let mut wire = [0u8; WIRE_MAX];
        assert_eq!(encoder.mode(), ProtocolMode::Legacy);
        assert_eq!(encoder.encode(1, 0, 120, -80, &mut wire), Some(8));
        encoder.set_mode(ProtocolMode::Hardened);
        let n = encoder.encode(4, 900, -30, 60, &mut wire).unwrap();
        let mut decoder = FrameDecoder::new();
        let mut received = None;
        decoder.feed(&wire[..n], &mut |frame| received = Some(frame));
        assert_eq!(decoder.frames, 1);
        assert_eq!(
            validate(&received.unwrap(), 900, None).unwrap(),
            Setpoint {
                steer: -30,
                speed: 60
            }
        );
        assert_eq!(ProtocolMode::from_u8(2), None);
        assert_eq!(ProtocolMode::Hardened.as_u8(), 1);
    }

    #[test]
    fn arbitrary_byte_streams_never_panic() {
        let mut decoder = FrameDecoder::new();
        let mut state = 0x1357_9BDFu32;
        let mut frames = 0usize;
        for _ in 0..4_000 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let byte = (state >> 24) as u8;
            decoder.feed(&[byte], &mut |_| frames += 1);
        }
        // CRC and length checks make an accidental command essentially
        // impossible; the test pins that no random byte sequence panics.
        assert!(frames <= 1);
        let _ = (decoder.crc_errors, decoder.oversize, decoder.resyncs);
    }

    #[test]
    fn garbage_and_chunk_boundaries_are_tolerated() {
        let frame = Frame::command(7, 123_456, 120, -80);
        let mut wire = [0u8; WIRE_MAX];
        let n = frame.encode(&mut wire).unwrap();
        let mut decoder = FrameDecoder::new();
        let mut received = None;
        for chunk in wire[..n].chunks(2) {
            decoder.feed(chunk, &mut |f| received = Some(f));
        }
        assert_eq!(decoder.frames, 1);
        assert_eq!(received.unwrap().seq, 7);
        decoder.feed(&[0x03, 0x11, 0x22, 0x99, 0], &mut |_| {});
        assert_eq!(decoder.crc_errors, 1);
    }
}
