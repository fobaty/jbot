#!/usr/bin/env python3
"""Host-side mirror of the STM32 hardened protocol and FOC trace codec.

The wire format is pinned by `f103_core::protocol` and `f103_core::trace`.
This module is deliberately dependency-free so a calibration laptop needs no
package install. Run `python3 f103_trace.py --selftest` to check it against the
golden vectors before connecting hardware.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

PROTOCOL_VERSION = 2
MSG_COMMAND = 1
MSG_FEEDBACK = 2
MSG_TRACE = 3
PAYLOAD_MAX = 96
TRACE_PAYLOAD_LEN = 88
FEEDBACK_PAYLOAD_LEN = 25

FLAG_ARMED = 1 << 0
FLAG_CALIBRATION_READY = 1 << 1
FLAG_FAULT_LATCHED = 1 << 2


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, no xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Standard COBS with an ending 0x00 delimiter, matching the firmware."""
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0 or code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            if byte == 0:
                continue
        out.append(byte)
        code += 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    """Inverse of `cobs_encode`; returns None for any malformed code run."""
    out = bytearray()
    read = 0
    while read < len(data):
        code = data[read]
        if code == 0:
            return None
        read += 1
        count = code - 1
        if read + count > len(data):
            return None
        out += data[read : read + count]
        read += count
        if code != 0xFF and read < len(data):
            out.append(0)
    return bytes(out)


@dataclass
class Frame:
    version: int
    msg_type: int
    flags: int
    seq: int
    timestamp_us: int
    payload: bytes
    payload_len: int

    def encode(self) -> bytes:
        decoded = bytearray()
        decoded += bytes([self.version, self.msg_type, self.flags])
        decoded += struct.pack("<H", self.seq)
        decoded += struct.pack("<I", self.timestamp_us)
        decoded += bytes([self.payload_len])
        decoded += self.payload[: self.payload_len]
        decoded += struct.pack("<H", crc16(bytes(decoded)))
        return cobs_encode(bytes(decoded)) + b"\x00"


def decode_frame(encoded: bytes) -> Frame | None:
    """Decode one delimiter-terminated COBS frame; None on any validation error."""
    if not encoded:
        return None
    decoded = cobs_decode(encoded)
    if decoded is None or len(decoded) < 12:
        return None
    payload_len = decoded[9]
    if payload_len > PAYLOAD_MAX or len(decoded) != 12 + payload_len:
        return None
    got_crc = struct.unpack_from("<H", decoded, 10 + payload_len)[0]
    if crc16(decoded[: 10 + payload_len]) != got_crc:
        return None
    return Frame(
        version=decoded[0],
        msg_type=decoded[1],
        flags=decoded[2],
        seq=struct.unpack_from("<H", decoded, 3)[0],
        timestamp_us=struct.unpack_from("<I", decoded, 5)[0],
        payload=bytes(decoded[10 : 10 + payload_len]),
        payload_len=payload_len,
    )


@dataclass
class MotorTrace:
    mode: int = 0
    scheduler: int = 0
    angle_q6: int = 0
    speed_rpm: int = 0
    target: int = 0
    iq: int = 0
    id: int = 0
    vq: int = 0
    vd: int = 0
    integral_q: int = 0
    integral_d: int = 0
    phase_a: int = 0
    phase_b: int = 0
    phase_c: int = 0
    err_code: int = 0

    def to_payload(self) -> bytes:
        out = bytearray(32)
        out[0] = self.mode
        out[1] = self.scheduler
        out[2] = self.err_code
        fields = (
            self.angle_q6,
            self.speed_rpm,
            self.target,
            self.iq,
            self.id,
            self.vq,
            self.vd,
            self.phase_a,
            self.phase_b,
        )
        for index, value in enumerate(fields):
            struct.pack_into("<h", out, 4 + index * 2, value)
        struct.pack_into("<h", out, 22, self.phase_c)
        struct.pack_into("<i", out, 24, self.integral_q)
        struct.pack_into("<i", out, 28, self.integral_d)
        return bytes(out)

    @classmethod
    def from_payload(cls, p: bytes, off: int) -> "MotorTrace":
        def i16(at: int) -> int:
            return struct.unpack_from("<h", p, off + at)[0]

        def i32(at: int) -> int:
            return struct.unpack_from("<i", p, off + at)[0]

        return cls(
            mode=p[off],
            scheduler=p[off + 1],
            err_code=p[off + 2],
            angle_q6=i16(4),
            speed_rpm=i16(6),
            target=i16(8),
            iq=i16(10),
            id=i16(12),
            vq=i16(14),
            vd=i16(16),
            phase_a=i16(18),
            phase_b=i16(20),
            phase_c=i16(22),
            integral_q=i32(24),
            integral_d=i32(28),
        )


@dataclass
class TraceData:
    tick: int = 0
    flags: int = 0
    left: MotorTrace = field(default_factory=MotorTrace)
    right: MotorTrace = field(default_factory=MotorTrace)
    dc_left: int = 0
    dc_right: int = 0
    duty_left: tuple[int, int, int] = (0, 0, 0)
    duty_right: tuple[int, int, int] = (0, 0, 0)

    def to_payload(self) -> bytes:
        out = bytearray(TRACE_PAYLOAD_LEN)
        struct.pack_into("<I", out, 0, self.tick)
        out[4] = self.flags
        struct.pack_into("<h", out, 8, self.dc_left)
        struct.pack_into("<h", out, 10, self.dc_right)
        for index, duty in enumerate(self.duty_left + self.duty_right):
            struct.pack_into("<H", out, 12 + index * 2, duty)
        out[24:56] = self.left.to_payload()
        out[56:88] = self.right.to_payload()
        return bytes(out)

    @classmethod
    def from_payload(cls, p: bytes) -> "TraceData":
        if len(p) != TRACE_PAYLOAD_LEN:
            raise ValueError("trace payload must be 88 bytes")
        return cls(
            tick=struct.unpack_from("<I", p, 0)[0],
            flags=p[4],
            dc_left=struct.unpack_from("<h", p, 8)[0],
            dc_right=struct.unpack_from("<h", p, 10)[0],
            duty_left=struct.unpack_from("<3H", p, 12),
            duty_right=struct.unpack_from("<3H", p, 18),
            left=MotorTrace.from_payload(p, 24),
            right=MotorTrace.from_payload(p, 56),
        )

    @classmethod
    def from_frame(cls, frame: Frame) -> "TraceData | None":
        if frame.version != PROTOCOL_VERSION or frame.msg_type != MSG_TRACE:
            return None
        if frame.payload_len != TRACE_PAYLOAD_LEN:
            return None
        return cls.from_payload(frame.payload)


class StreamDemux:
    """Split the USART3 stream into ASCII lines and COBS frames.

    The firmware never interleaves the two at byte level: while `$TRACE` is
    active the TXE ISR drains only the binary ring, and console replies are
    flushed after `$TRACE 0`. The bridge therefore switches the demux mode
    explicitly around a trace session instead of guessing from byte values.
    """

    def __init__(self) -> None:
        self._text = bytearray()
        self._frame = bytearray()
        self._frames = False

    def start_frames(self) -> None:
        self._frame.clear()
        self._text.clear()
        self._frames = True

    def stop_frames(self) -> None:
        self._frame.clear()
        self._frames = False

    def feed(self, chunk: bytes) -> tuple[list[str], list[Frame]]:
        if self._frames:
            frames: list[Frame] = []
            for byte in chunk:
                if byte == 0:
                    if self._frame:
                        frame = decode_frame(bytes(self._frame))
                        if frame is not None:
                            frames.append(frame)
                        self._frame.clear()
                    continue
                if len(self._frame) < 512:
                    self._frame.append(byte)
            return [], frames

        lines: list[str] = []
        for byte in chunk:
            if byte == 0:
                continue
            if byte == 0x0A:
                if self._text:
                    lines.append(self._text.decode("ascii", "replace"))
                    self._text.clear()
            elif byte != 0x0D:
                self._text.append(byte)
        return lines, []


# Golden vectors from f103_core::protocol and ::trace tests.
GOLDEN_HARDENED_COMMAND = bytes.fromhex("0302010902010605040304640538FFC44000")
GOLDEN_HARDENED_FEEDBACK = bytes.fromhex(
    "030202090301100504031964049CFFC80738FF36105D0101050103050C"
    "04F3FF6E027803436800"
)
GOLDEN_LEGACY_COMMAND = bytes.fromhex("CDAB3412785681EF")
GOLDEN_LEGACY_FEEDBACK = bytes.fromhex("CDAB64009CFFC80038FF36105D010000AEBA")
CRC_CHECK = 0x29B1


def selftest() -> None:
    assert crc16(b"123456789") == CRC_CHECK, "CRC check value"
    assert cobs_decode(cobs_encode(b"\x00\x01\x02\x00")) == b"\x00\x01\x02\x00"

    frame = decode_frame(GOLDEN_HARDENED_COMMAND[:-1])
    assert frame is not None, "golden command frame must decode"
    assert (frame.version, frame.msg_type, frame.flags) == (2, 1, 0)
    assert frame.seq == 0x0102 and frame.timestamp_us == 0x03040506
    assert frame.payload_len == 4
    assert frame.payload == struct.pack("<hh", 100, -200), frame.payload.hex()

    cmd = Frame(2, 1, 0, 0x0102, 0x03040506, struct.pack("<hh", 100, -200), 4)
    assert cmd.encode() == GOLDEN_HARDENED_COMMAND, cmd.encode().hex()

    fb_payload = bytearray(25)
    for index, value in enumerate((100, -100, 200, -200, 4150, 349)):
        struct.pack_into("<h", fb_payload, index * 2, value)
    struct.pack_into("<H", fb_payload, 12, 0)
    fb_payload[14:17] = bytes([0x01, 3, 5])
    struct.pack_into("<hh", fb_payload, 17, 12, -13)
    struct.pack_into("<HH", fb_payload, 21, 110, 120)
    fb_frame = Frame(2, 2, 0, 0x0103, 0x03040510, bytes(fb_payload), 25)
    assert fb_frame.encode() == GOLDEN_HARDENED_FEEDBACK, fb_frame.encode().hex()

    sample = TraceData(
        tick=0x01020304,
        flags=FLAG_ARMED | FLAG_CALIBRATION_READY,
        left=MotorTrace(
            mode=3,
            scheduler=4,
            angle_q6=-1,
            speed_rpm=-6400,
            target=-1000,
            iq=-32768,
            id=32767,
            vq=-14400,
            vd=14400,
            integral_q=-26_000_000,
            integral_d=65_536_000,
            phase_a=16_600,
            phase_b=-1,
            phase_c=-16_600,
            err_code=7,
        ),
        right=MotorTrace(
            mode=2,
            scheduler=3,
            angle_q6=352,
            speed_rpm=6400,
            target=1000,
            iq=10841,
            id=-10961,
            vq=5286,
            vd=-1893,
            integral_q=1232,
            integral_d=-65_536_000,
            phase_a=-1,
            phase_b=5064,
            phase_c=4232,
            err_code=0,
        ),
        dc_left=-1,
        dc_right=23,
        duty_left=(110, 1900, 2000),
        duty_right=(1000, 1100, 0),
    )
    payload = sample.to_payload()
    assert payload[:8] == bytes.fromhex("0403020103000000"), payload[:8].hex()
    assert TraceData.from_payload(payload) == sample, "trace round trip"
    trace_frame = Frame(2, 3, sample.flags, 0x0203, 0x04050607, payload, 88)
    decoded = decode_frame(trace_frame.encode()[:-1])
    assert decoded is not None and TraceData.from_frame(decoded) == sample

    demux = StreamDemux()
    wire = trace_frame.encode()
    demux.start_frames()
    frames = []
    for chunk in (wire[:7], wire[7:40], wire[40:]):
        _, got = demux.feed(chunk)
        frames.extend(got)
    assert len(frames) == 1 and TraceData.from_frame(frames[0]) == sample
    demux.stop_frames()
    lines, _ = demux.feed(b"TRACE 0\n")
    assert lines == ["TRACE 0"], lines
    print(
        "selftest: OK (crc, cobs, command/feedback goldens, trace round trip, "
        "chunked frames, text mode)"
    )


if __name__ == "__main__":
    import sys

    if "--selftest" in sys.argv:
        selftest()
    else:
        print(__doc__)
