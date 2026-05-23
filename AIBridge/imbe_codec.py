"""
IMBE codec wrapper.

P25 Phase 1 uses IMBE (Improved MultiBand Excitation) at 7200 bps. Each
voice frame is 88 bits (11 bytes) of encoded audio = ~20 ms of speech.

We need both directions:
  decode: 11 bytes IMBE  -> 160 samples PCM int16 @ 8 kHz (20 ms)
  encode: 160 samples PCM -> 11 bytes IMBE

The underlying codec is patent-encumbered, so we use the open-source
`mbelib` (a clean-room implementation) plus a small thin wrapper. mbelib
ships as a C library; we expose it via ctypes.

The choice between mbelib and `py-imbe-vocoder` is left as a runtime
preference. mbelib is the more battle-tested choice and is what OP25 and
DSD use; py-imbe-vocoder bindings exist but are less consistently
maintained across platforms.

For the prototype we ship a `MockCodec` that returns silence on decode
and zero IMBE frames on encode. That lets the end-to-end protocol path
be tested before pulling in the real codec dep. Wire the real codec by
setting `AIBRIDGE_CODEC=mbelib` in the environment (or via config).
"""

from __future__ import annotations
import logging
import os
from typing import Protocol

log = logging.getLogger("aibridge.imbe")

# IMBE frame size on the wire.
IMBE_FRAME_BYTES = 11

# IMBE voice frame at 8 kHz, 20 ms: exactly 160 PCM samples.
IMBE_FRAME_SAMPLES = 160

# All PCM in this project is 8 kHz, mono, signed 16-bit, little-endian.
PCM_SAMPLE_RATE = 8000
PCM_SAMPLE_WIDTH = 2  # bytes


class Codec(Protocol):
    """Codec interface — either mock or real."""

    def decode(self, imbe_11b: bytes) -> bytes:
        """11 bytes IMBE → 320 bytes PCM (160 samples × 2 bytes)."""
        ...

    def encode(self, pcm_320b: bytes) -> bytes:
        """320 bytes PCM (160 samples) → 11 bytes IMBE."""
        ...


class MockCodec:
    """
    No-op codec used during protocol bring-up.

    decode: returns 160 zero-PCM samples (silence).
    encode: returns 11 zero bytes.

    Lets us verify the UDP gateway plumbing without pulling in mbelib.
    """

    def decode(self, imbe_11b: bytes) -> bytes:
        if len(imbe_11b) != IMBE_FRAME_BYTES:
            log.warning("MockCodec.decode: unexpected frame size %d", len(imbe_11b))
        return b"\x00" * (IMBE_FRAME_SAMPLES * PCM_SAMPLE_WIDTH)

    def encode(self, pcm_320b: bytes) -> bytes:
        expected = IMBE_FRAME_SAMPLES * PCM_SAMPLE_WIDTH
        if len(pcm_320b) != expected:
            log.warning("MockCodec.encode: unexpected PCM size %d (want %d)",
                        len(pcm_320b), expected)
        return b"\x00" * IMBE_FRAME_BYTES


class MbelibCodec:
    """
    Wraps mbelib via ctypes. Stubbed until we wire the actual library.

    To finish this:
      1) apt-get install libmbe-dev  (or build mbelib from source)
      2) ctypes.CDLL("libmbe.so") and bind the relevant entry points
         (mbe_processImbe4400Data + mbe_synthesizeSpeechf)
      3) Maintain per-codec state for the synthesizer (it's stateful
         frame-to-frame because of the energy/pitch smoother)

    Left as a TODO so the rest of the bridge can be built/tested first.
    """

    def __init__(self) -> None:
        raise NotImplementedError(
            "MbelibCodec not yet implemented. Set AIBRIDGE_CODEC=mock "
            "to test the protocol layer, or finish the mbelib binding "
            "in imbe_codec.py."
        )


def get_codec() -> Codec:
    """Factory: pick a codec based on env / config."""
    choice = os.environ.get("AIBRIDGE_CODEC", "mock").lower()
    if choice == "mock":
        log.info("Using MockCodec (silent passthrough)")
        return MockCodec()
    if choice == "mbelib":
        return MbelibCodec()
    raise ValueError(f"Unknown AIBRIDGE_CODEC={choice!r}")


def pcm_to_imbe_frames(pcm: bytes, codec: Codec) -> list[bytes]:
    """
    Slice a longer PCM stream into 20 ms chunks and encode each one.

    Pads the last chunk with silence if PCM length is not a multiple of
    320 bytes (one IMBE frame's worth at 8 kHz / 16-bit).
    """
    chunk = IMBE_FRAME_SAMPLES * PCM_SAMPLE_WIDTH
    out: list[bytes] = []
    for i in range(0, len(pcm), chunk):
        block = pcm[i:i + chunk]
        if len(block) < chunk:
            block = block + b"\x00" * (chunk - len(block))
        out.append(codec.encode(block))
    return out


def imbe_frames_to_pcm(frames: list[bytes], codec: Codec) -> bytes:
    """Decode a list of IMBE frames into a single PCM byte stream."""
    return b"".join(codec.decode(f) for f in frames)
