//! Wire format for the EFeru mainboard link.
//!
//! The STM32F103 keeps the upstream EFeru C firmware. On the left cable it runs
//! `CONTROL_SERIAL_USART2` plus `FEEDBACK_SERIAL_USART2`:
//!
//! * command, Pico -> mainboard, 8 bytes: `start u16`, `steer i16`, `speed i16`,
//!   `checksum u16`, little-endian, checksum is the XOR of `start ^ steer ^ speed`;
//! * feedback, mainboard -> Pico, 18 bytes: `start`, `cmd1`, `cmd2`,
//!   `speedR`, `speedL`, `batVoltage`, `boardTemp`, `cmdLed`, `checksum`, all
//!   16-bit little-endian, checksum is the XOR of every preceding word.
//!
//! `cmd1`/`cmd2` in the feedback are the parsed *inputs* (`steer`, `speed`), not
//! the per-wheel torque that the F103 finally applies. They are still useful as
//! a link-acceptance measurement because the F103 echoes what it accepted.
//!
//! Torque requests use `TRQ_MODE`: `+/-1000` maps to `+/-I_MOT_MAX`. The F103
//! applies its own mixer unless `TANK_STEERING` is enabled, so a desired wheel
//! pair is converted with [`Command::from_wheel_torque`]. That inverse assumes
//! `SPEED_COEFFICIENT = STEER_COEFFICIENT = 16384` (both 1.0); the production
//! mainboard profile must keep those values, otherwise the yaw differential is
//! scaled by `STEER_COEFFICIENT`.
//!
//! The F103 accepts a command only when exactly 8 bytes arrived between two
//! idle periods, so the sender must leave a small inter-frame gap.

/// Length of a Pico-to-mainboard command frame.
pub const CMD_FRAME_LEN: usize = 8;
/// Length of a mainboard-to-Pico feedback frame without odometry.
pub const FB_FRAME_LEN: usize = 18;
/// Frame marker shared by both directions.
pub const START_FRAME: u16 = 0xABCD;
/// Magnitude of the torque request that maps to `I_MOT_MAX`.
pub const TORQUE_LIMIT: i16 = 1000;

/// Errors returned when a frame cannot be decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// The slice does not have the expected frame length.
    BadLength,
    /// The start marker is not [`START_FRAME`].
    BadStart,
    /// The checksum does not match the payload.
    BadChecksum,
}

/// Torque command sent to the mainboard.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Command {
    /// Steering input, `-1000..=1000`.
    pub steer: i16,
    /// Speed input, `-1000..=1000`.
    pub speed: i16,
}

impl Command {
    /// Neutral command: zero torque on both wheels.
    pub const ZERO: Self = Self { steer: 0, speed: 0 };

    /// Build a command from raw `steer`/`speed` inputs, clamping each to the
    /// supported torque range.
    pub fn new(steer: i16, speed: i16) -> Self {
        Self {
            steer: clamp_torque(steer),
            speed: clamp_torque(speed),
        }
    }

    /// Build a command from a desired left/right wheel torque pair.
    ///
    /// The EFeru mixer computes `cmdL = speed + steer` and
    /// `cmdR = speed - steer` with unit coefficients, so the inverse is the
    /// mean for the common mode and half the difference for the differential.
    /// The conversion truncates toward zero; the residual is below one unit.
    pub fn from_wheel_torque(left: i16, right: i16) -> Self {
        let left = i32::from(clamp_torque(left));
        let right = i32::from(clamp_torque(right));
        let speed = (left + right) / 2;
        let steer = (left - right) / 2;
        Self::new(steer as i16, speed as i16)
    }

    /// XOR checksum used by the stock EFeru command frame.
    pub fn checksum(self) -> u16 {
        START_FRAME ^ (self.steer as u16) ^ (self.speed as u16)
    }

    /// Encode the command as the 8-byte wire frame.
    pub fn encode(self) -> [u8; CMD_FRAME_LEN] {
        let mut frame = [0u8; CMD_FRAME_LEN];
        frame[0..2].copy_from_slice(&START_FRAME.to_le_bytes());
        frame[2..4].copy_from_slice(&self.steer.to_le_bytes());
        frame[4..6].copy_from_slice(&self.speed.to_le_bytes());
        frame[6..8].copy_from_slice(&self.checksum().to_le_bytes());
        frame
    }

    /// Decode an 8-byte command frame. Used by tests and bench harnesses.
    pub fn decode(frame: &[u8]) -> Result<Self, FrameError> {
        if frame.len() != CMD_FRAME_LEN {
            return Err(FrameError::BadLength);
        }
        let start = u16::from_le_bytes([frame[0], frame[1]]);
        if start != START_FRAME {
            return Err(FrameError::BadStart);
        }
        let steer = i16::from_le_bytes([frame[2], frame[3]]);
        let speed = i16::from_le_bytes([frame[4], frame[5]]);
        let checksum = u16::from_le_bytes([frame[6], frame[7]]);
        let command = Self { steer, speed };
        if checksum != command.checksum() {
            return Err(FrameError::BadChecksum);
        }
        Ok(command)
    }
}

/// Feedback frame received from the mainboard.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Feedback {
    /// Steering input the mainboard accepted, `-1000..=1000`.
    pub cmd_steer: i16,
    /// Speed input the mainboard accepted, `-1000..=1000`.
    pub cmd_speed: i16,
    /// Right wheel speed, rpm.
    pub speed_right_rpm: i16,
    /// Left wheel speed, rpm.
    pub speed_left_rpm: i16,
    /// Pack voltage in 0.01 V units: 4200 means 42.00 V.
    pub battery_voltage_centivolts: i16,
    /// Board temperature in tenths of a degree Celsius: 349 means 34.9 °C.
    ///
    /// EFeru computes this value from `TEMP_CAL_*_DEG_C`, which are themselves
    /// specified in °C × 10, so the wire field is decidegrees rather than whole
    /// degrees.
    pub board_temp_decideg_c: i16,
    /// Sideboard LED state, unused in the JBOT harness.
    pub led: u16,
}

impl Feedback {
    /// XOR checksum over every preceding 16-bit word.
    pub fn checksum(self) -> u16 {
        START_FRAME
            ^ (self.cmd_steer as u16)
            ^ (self.cmd_speed as u16)
            ^ (self.speed_right_rpm as u16)
            ^ (self.speed_left_rpm as u16)
            ^ (self.battery_voltage_centivolts as u16)
            ^ (self.board_temp_decideg_c as u16)
            ^ self.led
    }

    /// Encode the feedback frame. Used by tests and the bench injector.
    pub fn encode(self) -> [u8; FB_FRAME_LEN] {
        let mut frame = [0u8; FB_FRAME_LEN];
        let words = [
            START_FRAME,
            self.cmd_steer as u16,
            self.cmd_speed as u16,
            self.speed_right_rpm as u16,
            self.speed_left_rpm as u16,
            self.battery_voltage_centivolts as u16,
            self.board_temp_decideg_c as u16,
            self.led,
            self.checksum(),
        ];
        for (slot, word) in frame.as_chunks_mut::<2>().0.iter_mut().zip(words) {
            slot.copy_from_slice(&word.to_le_bytes());
        }
        frame
    }

    /// Decode an 18-byte feedback frame.
    pub fn decode(frame: &[u8]) -> Result<Self, FrameError> {
        if frame.len() != FB_FRAME_LEN {
            return Err(FrameError::BadLength);
        }
        let mut words = [0u16; FB_FRAME_LEN / 2];
        for (word, slot) in words.iter_mut().zip(frame.as_chunks::<2>().0) {
            *word = u16::from_le_bytes([slot[0], slot[1]]);
        }
        if words[0] != START_FRAME {
            return Err(FrameError::BadStart);
        }
        let feedback = Self {
            cmd_steer: words[1] as i16,
            cmd_speed: words[2] as i16,
            speed_right_rpm: words[3] as i16,
            speed_left_rpm: words[4] as i16,
            battery_voltage_centivolts: words[5] as i16,
            board_temp_decideg_c: words[6] as i16,
            led: words[7],
        };
        if words[8] != feedback.checksum() {
            return Err(FrameError::BadChecksum);
        }
        Ok(feedback)
    }

    /// Pack voltage in volts.
    pub fn battery_voltage_v(self) -> f32 {
        f32::from(self.battery_voltage_centivolts) * 0.01
    }

    /// Board temperature in degrees Celsius.
    pub fn board_temp_celsius(self) -> f32 {
        f32::from(self.board_temp_decideg_c) * 0.1
    }

    /// Wheel speed converted from rpm to metres per second.
    pub fn wheel_speed_mps(self, wheel_radius_m: f32) -> (f32, f32) {
        // rpm -> rad/s is 2*pi/60, and v = omega * r.
        let scale = core::f32::consts::TAU * wheel_radius_m / 60.0;
        (
            f32::from(self.speed_left_rpm) * scale,
            f32::from(self.speed_right_rpm) * scale,
        )
    }
}

/// Byte-stream parser that resynchronises on the frame marker.
///
/// The mainboard transmits whole frames separated by an idle period, but a
/// byte-stream parser does not need that assumption: it slides over the buffer
/// until a complete frame with a valid checksum is found.
#[derive(Clone, Copy)]
pub struct FeedbackParser {
    buf: [u8; 64],
    len: usize,
    /// Number of bytes discarded while searching for a valid frame.
    pub discarded_bytes: u32,
}

impl FeedbackParser {
    /// Create an empty parser.
    pub const fn new() -> Self {
        Self {
            buf: [0; 64],
            len: 0,
            discarded_bytes: 0,
        }
    }

    /// Drop any partially received frame.
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Push one received byte and return a frame when one completes.
    pub fn push(&mut self, byte: u8) -> Option<Feedback> {
        if self.len == self.buf.len() {
            self.drop_front(1);
        }
        self.buf[self.len] = byte;
        self.len += 1;

        while self.len >= FB_FRAME_LEN {
            if let Ok(feedback) = Feedback::decode(&self.buf[..FB_FRAME_LEN]) {
                self.drop_front(FB_FRAME_LEN);
                return Some(feedback);
            }
            self.drop_front(1);
        }
        None
    }

    fn drop_front(&mut self, count: usize) {
        if count >= self.len {
            self.len = 0;
            return;
        }
        self.buf.copy_within(count..self.len, 0);
        self.len -= count;
        self.discarded_bytes = self.discarded_bytes.saturating_add(count as u32);
    }
}

impl Default for FeedbackParser {
    fn default() -> Self {
        Self::new()
    }
}

fn clamp_torque(value: i16) -> i16 {
    value.clamp(-TORQUE_LIMIT, TORQUE_LIMIT)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_command_round_trips() {
        let frame = Command::ZERO.encode();
        assert_eq!(frame.len(), CMD_FRAME_LEN);
        assert_eq!(Command::decode(&frame), Ok(Command::ZERO));
    }

    #[test]
    fn command_clamps_to_torque_limit() {
        let command = Command::new(5000, -5000);
        assert_eq!(command.steer, TORQUE_LIMIT);
        assert_eq!(command.speed, -TORQUE_LIMIT);
    }

    #[test]
    fn command_rejects_bad_checksum() {
        let mut frame = Command::new(10, -20).encode();
        frame[7] ^= 0xFF;
        assert_eq!(Command::decode(&frame), Err(FrameError::BadChecksum));
    }

    #[test]
    fn wheel_torque_matches_eferu_mixer() {
        // The F103 computes cmdL = speed + steer and cmdR = speed - steer.
        for (left, right) in [(-1000, -1000), (0, 0), (500, -500), (37, 91)] {
            let command = Command::from_wheel_torque(left, right);
            let mixed_left = i32::from(command.speed) + i32::from(command.steer);
            let mixed_right = i32::from(command.speed) - i32::from(command.steer);
            assert!((mixed_left - i32::from(left)).abs() <= 1);
            assert!((mixed_right - i32::from(right)).abs() <= 1);
        }
    }

    #[test]
    fn decodes_captured_bench_frame() {
        // Captured from the bench mainboard at 115200 8N1: idle wheels, 41.53 V
        // pack, 34.9 °C board temperature.
        let frame = [
            0xCD, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x10, 0x5D, 0x01,
            0x00, 0x00, 0xA9, 0xBA,
        ];
        let feedback = Feedback::decode(&frame).expect("valid captured frame");
        assert_eq!(feedback.cmd_steer, 0);
        assert_eq!(feedback.cmd_speed, 0);
        assert_eq!(feedback.speed_left_rpm, 0);
        assert_eq!(feedback.speed_right_rpm, 0);
        assert_eq!(feedback.led, 0);
        assert!((feedback.battery_voltage_v() - 41.53).abs() < 1e-6);
        assert!((feedback.board_temp_celsius() - 34.9).abs() < 1e-6);
    }

    #[test]
    fn feedback_round_trips() {
        let feedback = Feedback {
            cmd_steer: -12,
            cmd_speed: 300,
            speed_right_rpm: -45,
            speed_left_rpm: 46,
            battery_voltage_centivolts: 4198,
            board_temp_decideg_c: 314,
            led: 0xF00F,
        };
        let frame = feedback.encode();
        assert_eq!(frame.len(), FB_FRAME_LEN);
        assert_eq!(Feedback::decode(&frame), Ok(feedback));
        assert!((feedback.battery_voltage_v() - 41.98).abs() < 1e-6);
        assert!((feedback.board_temp_celsius() - 31.4).abs() < 1e-6);
    }

    #[test]
    fn feedback_rejects_bad_start_and_length() {
        let frame = Feedback::default().encode();
        assert_eq!(Feedback::decode(&frame[..17]), Err(FrameError::BadLength));
        let mut bad = frame;
        bad[0] ^= 0xFF;
        assert_eq!(Feedback::decode(&bad), Err(FrameError::BadStart));
    }

    #[test]
    fn parser_resynchronises_after_garbage() {
        let feedback = Feedback {
            cmd_speed: 123,
            battery_voltage_centivolts: 4200,
            board_temp_decideg_c: 251,
            ..Feedback::default()
        };
        let frame = feedback.encode();
        let mut parser = FeedbackParser::new();
        for byte in [0x00, 0xFF, 0x12] {
            assert_eq!(parser.push(byte), None);
        }
        let mut found = None;
        for byte in frame {
            found = found.or(parser.push(byte));
        }
        assert_eq!(found, Some(feedback));
        assert!(parser.discarded_bytes >= 3);
    }

    #[test]
    fn parser_recovers_two_back_to_back_frames() {
        let first = Feedback {
            speed_left_rpm: 10,
            ..Feedback::default()
        };
        let second = Feedback {
            speed_left_rpm: -10,
            ..Feedback::default()
        };
        let mut parser = FeedbackParser::new();
        let mut frames = [None; 2];
        let mut index = 0;
        for byte in first.encode().into_iter().chain(second.encode()) {
            if let Some(frame) = parser.push(byte) {
                frames[index] = Some(frame);
                index += 1;
            }
        }
        assert_eq!(frames, [Some(first), Some(second)]);
    }
}
