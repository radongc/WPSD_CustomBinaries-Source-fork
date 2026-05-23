"""
P25 network protocol parser/emitter.

Speaks MMDVMHost's wire format as defined in MMDVMHost/P25Network.cpp.
A transmission is a sequence of small UDP packets, each with a 1-byte
record marker as the first byte.

LDU1 = 9 packets (records 0x62..0x6A), one IMBE voice frame per packet.
LDU2 = 9 packets (records 0x6B..0x73), one IMBE voice frame per packet.
End  = 1 packet  (record 0x80, 17 bytes of zeros).

Each IMBE voice frame is 11 bytes (88 bits) of encoded audio = ~20 ms.
So 9 frames per LDU = ~180 ms. Alternating LDU1/LDU2 gives ~360 ms per
LDU pair.

Record layout (offsets and lengths from P25Network.cpp `writeLDU1` /
`writeLDU2`):

  LDU1:
    0x62 (22 B): hdr[0..9], imbe@10  [voice frame 0]
    0x63 (14 B): hdr[0],    imbe@1   [voice frame 1]
    0x64 (17 B): hdr[0], lcf, mfid, hdr[3..4], imbe@5  [voice frame 2]
    0x65 (17 B): hdr[0], dstId(3B), hdr[4], imbe@5     [voice frame 3]
    0x66 (17 B): hdr[0], srcId(3B), hdr[4], imbe@5     [voice frame 4]
    0x67 (17 B): hdr[0..4], imbe@5                     [voice frame 5]
    0x68 (17 B): hdr[0..4], imbe@5                     [voice frame 6]
    0x69 (17 B): hdr[0..4], imbe@5                     [voice frame 7]
    0x6A (16 B): hdr[0], lsd1, lsd2, hdr[3], imbe@4    [voice frame 8]

  LDU2:
    0x6B (22 B): hdr[0..9], imbe@10  [voice frame 0]
    0x6C (14 B): hdr[0],    imbe@1   [voice frame 1]
    0x6D (17 B): hdr[0], mi[0..2], hdr[4], imbe@5      [voice frame 2]
    0x6E (17 B): hdr[0], mi[3..5], hdr[4], imbe@5      [voice frame 3]
    0x6F (17 B): hdr[0], mi[6..8], hdr[4], imbe@5      [voice frame 4]
    0x70 (17 B): hdr[0], algId, kid(2B), hdr[4], imbe@5 [voice frame 5]
    0x71 (17 B): hdr[0..4], imbe@5                     [voice frame 6]
    0x72 (17 B): hdr[0..4], imbe@5                     [voice frame 7]
    0x73 (17 B): hdr[0..4], imbe@5                     [voice frame 8]

  End (0x80, 17 B): all zeros after the marker.
"""

from dataclasses import dataclass, field
from typing import Optional


# Record markers in LDU1 / LDU2 / end-of-transmission.
LDU1_MARKERS = list(range(0x62, 0x6B))   # 0x62..0x6A inclusive (9 records)
LDU2_MARKERS = list(range(0x6B, 0x74))   # 0x6B..0x73 inclusive (9 records)
END_MARKER = 0x80

# IMBE voice frame size on the wire.
IMBE_FRAME_BYTES = 11

# Per-record packet length and IMBE-payload offset within the packet.
# Mirrors REC62..REC6A / REC6B..REC73 in MMDVMHost/P25Network.cpp.
_RECORD_TABLE = {
    # LDU1
    0x62: (22, 10),
    0x63: (14, 1),
    0x64: (17, 5),
    0x65: (17, 5),
    0x66: (17, 5),
    0x67: (17, 5),
    0x68: (17, 5),
    0x69: (17, 5),
    0x6A: (16, 4),
    # LDU2
    0x6B: (22, 10),
    0x6C: (14, 1),
    0x6D: (17, 5),
    0x6E: (17, 5),
    0x6F: (17, 5),
    0x70: (17, 5),
    0x71: (17, 5),
    0x72: (17, 5),
    0x73: (17, 5),
}


@dataclass
class TransmissionMeta:
    """Per-transmission metadata extracted from LDU1/LDU2 control fields."""

    src_id: Optional[int] = None      # 24-bit source unit ID (from 0x66)
    dst_id: Optional[int] = None      # 24-bit destination TG/ID (from 0x65)
    lcf: Optional[int] = None         # Link Control Format (from 0x64)
    mfid: Optional[int] = None        # Manufacturer ID (from 0x64)
    alg_id: Optional[int] = None      # Encryption algorithm ID (from 0x70)
    kid: Optional[int] = None         # Key ID (from 0x70)
    lsd1: Optional[int] = None        # Low-speed data byte 1 (from 0x6A)
    lsd2: Optional[int] = None        # Low-speed data byte 2 (from 0x6A)
    mi: bytearray = field(default_factory=lambda: bytearray(9))  # Msg Indicator


def parse_record(pkt: bytes, meta: TransmissionMeta) -> Optional[bytes]:
    """
    Inspect a single UDP packet from MMDVMHost.

    Updates `meta` in-place with any control fields present. Returns the
    11-byte IMBE voice frame contained in the packet, or `None` if the
    packet is the end-of-transmission marker (0x80) or unrecognized.
    """
    if not pkt:
        return None
    marker = pkt[0]
    if marker == END_MARKER:
        return None
    record = _RECORD_TABLE.get(marker)
    if record is None:
        return None
    expected_len, imbe_off = record
    if len(pkt) < expected_len:
        return None

    # Pull control fields out of the records that carry them.
    if marker == 0x64:
        meta.lcf = pkt[1]
        meta.mfid = pkt[2]
    elif marker == 0x65:
        meta.dst_id = (pkt[1] << 16) | (pkt[2] << 8) | pkt[3]
    elif marker == 0x66:
        meta.src_id = (pkt[1] << 16) | (pkt[2] << 8) | pkt[3]
    elif marker == 0x6A:
        meta.lsd1 = pkt[1]
        meta.lsd2 = pkt[2]
    elif marker == 0x6D:
        meta.mi[0:3] = pkt[1:4]
    elif marker == 0x6E:
        meta.mi[3:6] = pkt[1:4]
    elif marker == 0x6F:
        meta.mi[6:9] = pkt[1:4]
    elif marker == 0x70:
        meta.alg_id = pkt[1]
        meta.kid = (pkt[2] << 8) | pkt[3]

    return bytes(pkt[imbe_off:imbe_off + IMBE_FRAME_BYTES])


def is_end_marker(pkt: bytes) -> bool:
    return bool(pkt) and pkt[0] == END_MARKER


def is_ldu1_start(pkt: bytes) -> bool:
    """First packet of an LDU1 block carries the 0x62 marker."""
    return bool(pkt) and pkt[0] == 0x62


def is_ldu2_start(pkt: bytes) -> bool:
    return bool(pkt) and pkt[0] == 0x6B


def build_end_record() -> bytes:
    """The 17-byte 0x80-prefixed end-of-transmission packet."""
    return bytes([END_MARKER]) + bytes(16)


def build_ldu1(imbe_frames: list[bytes], meta: TransmissionMeta) -> list[bytes]:
    """
    Emit the 9 LDU1 UDP packets for one ~180 ms voice block.

    `imbe_frames` must be exactly 9 IMBE frames of 11 bytes each.
    `meta` provides the src/dst/lcf/mfid/lsd fields that the records carry.
    """
    assert len(imbe_frames) == 9, "LDU1 must have exactly 9 IMBE frames"

    packets: list[bytes] = []
    src = meta.src_id or 0
    dst = meta.dst_id or 0
    lcf = meta.lcf or 0
    mfid = meta.mfid or 0
    lsd1 = meta.lsd1 or 0
    lsd2 = meta.lsd2 or 0

    for marker, (length, imbe_off) in zip(LDU1_MARKERS,
                                          [(22, 10), (14, 1), (17, 5),
                                           (17, 5), (17, 5), (17, 5),
                                           (17, 5), (17, 5), (16, 4)]):
        buf = bytearray(length)
        buf[0] = marker
        if marker == 0x64:
            buf[1] = lcf
            buf[2] = mfid
        elif marker == 0x65:
            buf[1] = (dst >> 16) & 0xFF
            buf[2] = (dst >> 8) & 0xFF
            buf[3] = dst & 0xFF
        elif marker == 0x66:
            buf[1] = (src >> 16) & 0xFF
            buf[2] = (src >> 8) & 0xFF
            buf[3] = src & 0xFF
        elif marker == 0x6A:
            buf[1] = lsd1
            buf[2] = lsd2

        frame_idx = marker - 0x62
        buf[imbe_off:imbe_off + IMBE_FRAME_BYTES] = imbe_frames[frame_idx]
        packets.append(bytes(buf))

    return packets


def build_ldu2(imbe_frames: list[bytes], meta: TransmissionMeta) -> list[bytes]:
    """Emit the 9 LDU2 UDP packets for one ~180 ms voice block."""
    assert len(imbe_frames) == 9, "LDU2 must have exactly 9 IMBE frames"

    packets: list[bytes] = []
    mi = bytes(meta.mi) if meta.mi else bytes(9)
    alg = meta.alg_id or 0x80  # 0x80 = CLEAR in Harris/Motorola P25 convention
    kid = meta.kid or 0

    lengths_and_offsets = [(22, 10), (14, 1), (17, 5), (17, 5), (17, 5),
                           (17, 5), (17, 5), (17, 5), (17, 5)]
    for marker, (length, imbe_off) in zip(LDU2_MARKERS, lengths_and_offsets):
        buf = bytearray(length)
        buf[0] = marker
        if marker == 0x6D:
            buf[1:4] = mi[0:3]
        elif marker == 0x6E:
            buf[1:4] = mi[3:6]
        elif marker == 0x6F:
            buf[1:4] = mi[6:9]
        elif marker == 0x70:
            buf[1] = alg
            buf[2] = (kid >> 8) & 0xFF
            buf[3] = kid & 0xFF

        frame_idx = marker - 0x6B
        buf[imbe_off:imbe_off + IMBE_FRAME_BYTES] = imbe_frames[frame_idx]
        packets.append(bytes(buf))

    return packets
