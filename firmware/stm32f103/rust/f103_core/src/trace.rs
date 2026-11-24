//! Machine-readable FOC trace snapshot carried in one hardened frame.
//!
//! Every field uses its full native width so a calibration script sees the
//! actual fixed-point state: the PI integrators are 32-bit Q16 values and can
//! reach tens of millions, while electrical speed and phase amplitudes need
//! the whole i16 range. No lossy bit packing is used here.

use crate::foc_generated::MotorTrace;

/// Size of the trace payload, matched to `protocol::PAYLOAD_MAX`.
pub const TRACE_PAYLOAD_LEN: usize = 88;

pub const FLAG_ARMED: u8 = 1 << 0;
pub const FLAG_CALIBRATION_READY: u8 = 1 << 1;
pub const FLAG_FAULT_LATCHED: u8 = 1 << 2;

const MOTOR_BYTES: usize = 32;

/// One controller snapshot plus the shared drive measurements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TraceData {
    pub tick: u32,
    pub flags: u8,
    pub left: MotorTrace,
    pub right: MotorTrace,
    pub dc_left: i16,
    pub dc_right: i16,
    pub duty_left: [u16; 3],
    pub duty_right: [u16; 3],
}

impl TraceData {
    /// Sets one flag by bit index: 0 armed, 1 calibration ready, 2 fault latched.
    pub fn set_flag(&mut self, bit: u8, value: bool) {
        if bit >= 8 {
            return;
        }
        let mask = 1u8 << bit;
        if value {
            self.flags |= mask;
        } else {
            self.flags &= !mask;
        }
    }

    /// Serialises the snapshot into the documented little-endian layout:
    ///
    /// | offset | field |
    /// |---|---|
    /// | 0 | `tick` u32 |
    /// | 4 | `flags` u8, 5..8 reserved |
    /// | 8, 10 | `dc_left`, `dc_right` i16 |
    /// | 12, 14, 16 / 18, 20, 22 | duty U/V/W u16 per motor |
    /// | 24..56 / 56..88 | left / right motor block |
    ///
    /// A motor block is `mode`, `scheduler`, `err_code`, one reserved byte,
    /// then `angle_q6`, `speed_rpm`, `target`, `iq`, `id`, `vq`, `vd`,
    /// `phase_a/b/c` as i16, and `integral_q`, `integral_d` as i32.
    pub fn to_payload(&self) -> [u8; TRACE_PAYLOAD_LEN] {
        let mut out = [0u8; TRACE_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.tick.to_le_bytes());
        out[4] = self.flags;
        out[8..10].copy_from_slice(&self.dc_left.to_le_bytes());
        out[10..12].copy_from_slice(&self.dc_right.to_le_bytes());
        for (index, duty) in self
            .duty_left
            .iter()
            .chain(self.duty_right.iter())
            .enumerate()
        {
            let at = 12 + index * 2;
            out[at..at + 2].copy_from_slice(&duty.to_le_bytes());
        }
        write_motor(&mut out[24..56], &self.left);
        write_motor(&mut out[56..88], &self.right);
        out
    }

    /// Reverses `to_payload`. Every field is stored at its native width, so
    /// the round trip is exact for the full i16/i32 range.
    pub fn from_payload(payload: &[u8; TRACE_PAYLOAD_LEN]) -> Self {
        let word = |at: usize| i16::from_le_bytes([payload[at], payload[at + 1]]);
        let mut duty_left = [0u16; 3];
        let mut duty_right = [0u16; 3];
        for (index, slot) in duty_left.iter_mut().enumerate() {
            let at = 12 + index * 2;
            *slot = u16::from_le_bytes([payload[at], payload[at + 1]]);
        }
        for (index, slot) in duty_right.iter_mut().enumerate() {
            let at = 18 + index * 2;
            *slot = u16::from_le_bytes([payload[at], payload[at + 1]]);
        }
        Self {
            tick: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            flags: payload[4],
            dc_left: word(8),
            dc_right: word(10),
            duty_left,
            duty_right,
            left: read_motor(&payload[24..56]),
            right: read_motor(&payload[56..88]),
        }
    }
}

fn write_motor(out: &mut [u8], motor: &MotorTrace) {
    debug_assert_eq!(out.len(), MOTOR_BYTES);
    out[0] = motor.mode;
    out[1] = motor.scheduler;
    out[2] = motor.err_code;
    let fields: [i16; 9] = [
        motor.angle_q6,
        motor.speed_rpm,
        motor.target,
        motor.iq,
        motor.id,
        motor.vq,
        motor.vd,
        motor.phase_a,
        motor.phase_b,
    ];
    for (index, value) in fields.iter().enumerate() {
        let at = 4 + index * 2;
        out[at..at + 2].copy_from_slice(&value.to_le_bytes());
    }
    out[22..24].copy_from_slice(&motor.phase_c.to_le_bytes());
    out[24..28].copy_from_slice(&motor.integral_q.to_le_bytes());
    out[28..32].copy_from_slice(&motor.integral_d.to_le_bytes());
}

fn read_motor(payload: &[u8]) -> MotorTrace {
    let word = |at: usize| i16::from_le_bytes([payload[at], payload[at + 1]]);
    MotorTrace {
        mode: payload[0],
        scheduler: payload[1],
        err_code: payload[2],
        angle_q6: word(4),
        speed_rpm: word(6),
        target: word(8),
        iq: word(10),
        id: word(12),
        vq: word(14),
        vd: word(16),
        phase_a: word(18),
        phase_b: word(20),
        phase_c: word(22),
        integral_q: i32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]),
        integral_d: i32::from_le_bytes([payload[28], payload[29], payload[30], payload[31]]),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{decode_trace_data, Frame, FrameDecoder};

    fn sample() -> TraceData {
        TraceData {
            tick: 0x0102_0304,
            flags: FLAG_ARMED | FLAG_CALIBRATION_READY,
            left: MotorTrace {
                mode: 3,
                scheduler: 4,
                angle_q6: -1,
                speed_rpm: -6_400,
                target: -1_000,
                iq: -32_768,
                id: 32_767,
                vq: -14_400,
                vd: 14_400,
                integral_q: -26_000_000,
                integral_d: 65_536_000,
                phase_a: 16_600,
                phase_b: -1,
                phase_c: -16_600,
                err_code: 7,
            },
            right: MotorTrace {
                mode: 2,
                scheduler: 3,
                angle_q6: 352,
                speed_rpm: 6_400,
                target: 1_000,
                iq: 10_841,
                id: -10_961,
                vq: 5_286,
                vd: -1_893,
                integral_q: 1_232,
                integral_d: -65_536_000,
                phase_a: -1,
                phase_b: 5_064,
                phase_c: 4_232,
                err_code: 0,
            },
            dc_left: -1,
            dc_right: 23,
            duty_left: [110, 1_900, 2_000],
            duty_right: [1_000, 1_100, 0],
        }
    }

    #[test]
    fn payload_golden_vector() {
        let payload = sample().to_payload();
        assert_eq!(payload.len(), TRACE_PAYLOAD_LEN);
        assert_eq!(payload[0..8], [0x04, 0x03, 0x02, 0x01, 0x03, 0, 0, 0]);
        // Spot-check the fields that the previous packed layout truncated.
        assert_eq!(&payload[8..10], &(-1i16).to_le_bytes());
        assert_eq!(&payload[10..12], &23i16.to_le_bytes());
        assert_eq!(&payload[24 + 6..24 + 8], &(-6_400i16).to_le_bytes());
        assert_eq!(&payload[24 + 24..24 + 28], &(-26_000_000i32).to_le_bytes());
        assert_eq!(&payload[56 + 28..56 + 32], &(-65_536_000i32).to_le_bytes());
    }

    #[test]
    fn payload_round_trips_every_field() {
        let data = sample();
        assert_eq!(TraceData::from_payload(&data.to_payload()), data);
    }

    #[test]
    fn extreme_fields_round_trip() {
        let mut data = TraceData::default();
        data.left.integral_q = i32::MIN;
        data.right.integral_d = i32::MAX;
        data.left.iq = i16::MIN;
        data.right.vq = i16::MAX;
        data.duty_left = [u16::MAX; 3];
        assert_eq!(TraceData::from_payload(&data.to_payload()), data);
    }

    #[test]
    fn flags_are_set_independently() {
        let mut data = TraceData::default();
        data.set_flag(0, true);
        data.set_flag(2, true);
        assert_eq!(data.flags, FLAG_ARMED | FLAG_FAULT_LATCHED);
        data.set_flag(0, false);
        assert_eq!(data.flags, FLAG_FAULT_LATCHED);
    }

    #[test]
    fn frame_trace_survives_chunk_boundaries() {
        let data = sample();
        let mut wire = [0u8; crate::config::WIRE_MAX];
        let n = crate::protocol::encode_trace(0x0203, 0x0405_0607, &data, &mut wire).unwrap();
        let frame = Frame::trace(0x0203, 0x0405_0607, &data);
        assert_eq!(frame.msg_type, crate::protocol::MSG_TRACE);
        assert_eq!(frame.payload_len as usize, TRACE_PAYLOAD_LEN);

        let mut decoder = FrameDecoder::new();
        let mut received = None;
        for chunk in wire[..n].chunks(7) {
            decoder.feed(chunk, &mut |f| received = Some(f));
        }
        assert_eq!(decoder.frames, 1);
        let got = received.expect("trace frame");
        assert_eq!(got.seq, 0x0203);
        assert_eq!(got.timestamp_us, 0x0405_0607);
        assert_eq!(decode_trace_data(&got), Some(data));
    }
}
