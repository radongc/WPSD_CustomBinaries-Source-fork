"""
IMBE codec wrapper.

P25 Phase 1 uses IMBE (Improved MultiBand Excitation) at 7200 bps. Each
voice frame is 88 bits (11 bytes) of encoded audio = ~20 ms of speech.

We need both directions:
  decode: 11 bytes IMBE  -> 160 samples PCM int16 @ 8 kHz (20 ms)
  encode: 160 samples PCM -> 11 bytes IMBE

The underlying codec is patent-encumbered, so we use mbelib for decode
(a clean-room implementation, also used by OP25/DSD). Encode is currently
a no-op until we wire in an encoder library — radio still hears the same
quiet hum as MockCodec on TX, but RX side is fully decoded.

Codec selection by env var:
  AIBRIDGE_CODEC=mock     -> silent passthrough (Phase 1 plumbing test)
  AIBRIDGE_CODEC=mbelib   -> real decode via mbelib + zero-byte encode
"""

from __future__ import annotations
import ctypes
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


# Default install path for the C++ wrapper built from imbe_native/.
_DEFAULT_WRAPPER_PATH = "/opt/aibridge/libaibridge_imbe.so"


class MbelibCodec:
    """
    Real IMBE codec via mbelib (decode) + the AIBridge C++ wrapper.

    The wrapper (`libaibridge_imbe.so`, built from `imbe_native/`) exposes
    four C functions:
      aibridge_imbe_create()  -> opaque ctx pointer (mbelib parm state)
      aibridge_imbe_destroy(ctx)
      aibridge_imbe_decode(ctx, imbe_11b*, pcm_160*)
      aibridge_imbe_encode(ctx, pcm_160*, imbe_11b*)   # currently no-op

    Per-stream state lives in the ctx — mbelib needs prev-frame
    parameters across calls because the synthesizer interpolates pitch
    and energy. One Codec instance == one decoding stream.
    """

    def __init__(self, lib_path: str = _DEFAULT_WRAPPER_PATH) -> None:
        self._lib = ctypes.CDLL(lib_path)

        self._lib.aibridge_imbe_create.restype = ctypes.c_void_p
        self._lib.aibridge_imbe_create.argtypes = []

        self._lib.aibridge_imbe_destroy.restype = None
        self._lib.aibridge_imbe_destroy.argtypes = [ctypes.c_void_p]

        self._lib.aibridge_imbe_decode.restype = None
        self._lib.aibridge_imbe_decode.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_int16),
        ]
        self._lib.aibridge_imbe_encode.restype = None
        self._lib.aibridge_imbe_encode.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int16),
            ctypes.POINTER(ctypes.c_uint8),
        ]

        self._ctx = self._lib.aibridge_imbe_create()
        if not self._ctx:
            raise RuntimeError("aibridge_imbe_create returned NULL")

        # Reusable scratch buffers (one frame each) to avoid allocs per call.
        self._pcm_buf = (ctypes.c_int16 * IMBE_FRAME_SAMPLES)()
        self._imbe_buf = (ctypes.c_uint8 * IMBE_FRAME_BYTES)()

    def __del__(self) -> None:
        # Best-effort: __del__ ordering during interpreter shutdown is fragile.
        try:
            if getattr(self, "_ctx", None) and getattr(self, "_lib", None):
                self._lib.aibridge_imbe_destroy(self._ctx)
                self._ctx = None
        except Exception:
            pass

    def decode(self, imbe_11b: bytes) -> bytes:
        if len(imbe_11b) != IMBE_FRAME_BYTES:
            log.warning("MbelibCodec.decode: unexpected frame size %d", len(imbe_11b))
            return b"\x00" * (IMBE_FRAME_SAMPLES * PCM_SAMPLE_WIDTH)
        in_buf = (ctypes.c_uint8 * IMBE_FRAME_BYTES).from_buffer_copy(imbe_11b)
        self._lib.aibridge_imbe_decode(self._ctx, in_buf, self._pcm_buf)
        return bytes(self._pcm_buf)

    def encode(self, pcm_320b: bytes) -> bytes:
        expected = IMBE_FRAME_SAMPLES * PCM_SAMPLE_WIDTH
        if len(pcm_320b) != expected:
            log.warning("MbelibCodec.encode: unexpected PCM size %d", len(pcm_320b))
            return b"\x00" * IMBE_FRAME_BYTES
        in_buf = (ctypes.c_int16 * IMBE_FRAME_SAMPLES).from_buffer_copy(pcm_320b)
        self._lib.aibridge_imbe_encode(self._ctx, in_buf, self._imbe_buf)
        return bytes(self._imbe_buf)


def get_codec() -> Codec:
    """Factory: pick a codec based on env / config."""
    choice = os.environ.get("AIBRIDGE_CODEC", "mock").lower()
    if choice == "mock":
        log.info("Using MockCodec (silent passthrough)")
        return MockCodec()
    if choice == "mbelib":
        log.info("Using MbelibCodec (decode: mbelib; encode: OP25 imbe_vocoder)")
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
