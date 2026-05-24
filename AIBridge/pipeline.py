"""
Voice pipeline: PCM in → STT → LLM → TTS → PCM out.

Implementations are pluggable. The defaults are:
  STT: whisper.cpp via subprocess (small.en model)
  LLM: Anthropic Claude API (Haiku for speed)
  TTS: Piper via subprocess (en_US-amy-medium voice or whatever's installed)

A "mock" implementation of each is provided that lets the rest of the
bridge be exercised without those deps present.

Audio convention everywhere here: 8 kHz, mono, signed 16-bit, little-endian
PCM. Matches the IMBE codec's expectations on both sides. If a real STT or
TTS produces different sample rates, the wrapper here is responsible for
the resample.
"""

from __future__ import annotations
import logging
import os
import subprocess
import tempfile
import time
import wave
from dataclasses import dataclass
from typing import Optional, Protocol

log = logging.getLogger("aibridge.pipeline")


PCM_SAMPLE_RATE = 8000


# ─── interfaces ────────────────────────────────────────────────────────────

class STT(Protocol):
    def transcribe(self, pcm: bytes) -> str: ...


class LLM(Protocol):
    def respond(self, user_text: str) -> str: ...


class TTS(Protocol):
    def synthesize(self, text: str) -> bytes:
        """Returns 8 kHz / 16-bit / mono PCM bytes."""
        ...


# ─── mock implementations ──────────────────────────────────────────────────

class MockSTT:
    def transcribe(self, pcm: bytes) -> str:
        # Approximate length so the mock response feels right.
        secs = len(pcm) / (PCM_SAMPLE_RATE * 2)
        log.info("MockSTT: %.1fs of audio → 'hello there'", secs)
        return "hello there"


class MockLLM:
    def __init__(self, system_prompt: str = "") -> None:
        self.system_prompt = system_prompt

    def respond(self, user_text: str) -> str:
        log.info("MockLLM: %r → echo", user_text)
        return f"You said: {user_text}. This is the mock bot."


class MockTTS:
    """
    Generates a short tone-pip so we can hear something arrive on the radio
    even before real TTS is wired up.
    """
    def synthesize(self, text: str) -> bytes:
        import math
        log.info("MockTTS: %d chars → 1 sec sine wave", len(text))
        n = PCM_SAMPLE_RATE  # 1 second
        amp = 8000  # quiet; full int16 range is ±32767
        freq = 440
        samples = bytearray()
        for i in range(n):
            v = int(amp * math.sin(2 * math.pi * freq * i / PCM_SAMPLE_RATE))
            samples += int(v).to_bytes(2, "little", signed=True)
        return bytes(samples)


# ─── real implementations (lightweight subprocess wrappers) ────────────────

@dataclass
class WhisperCppSTT:
    """
    Shells out to the whisper.cpp `whisper-cli` binary (cmake-built layout
    used by current whisper.cpp; the old `make main` flow no longer exists).

    Requires:
      - whisper.cpp built on the Pi (default location: /opt/whisper.cpp)
      - a model downloaded (default: ggml-small.en.bin)
    """
    binary: str = "/opt/whisper.cpp/build/bin/whisper-cli"
    model_path: str = "/opt/whisper.cpp/models/ggml-small.en.bin"
    threads: int = 4

    def transcribe(self, pcm: bytes) -> str:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            wav_path = f.name
        try:
            with wave.open(wav_path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(PCM_SAMPLE_RATE)
                wf.writeframes(pcm)
            t0 = time.monotonic()
            result = subprocess.run(
                [self.binary, "-m", self.model_path, "-f", wav_path,
                 "-t", str(self.threads), "-nt", "-otxt"],
                capture_output=True, text=True, check=False, timeout=30,
            )
            elapsed = time.monotonic() - t0
            log.info("whisper.cpp: %.2fs, rc=%d", elapsed, result.returncode)
            if result.returncode != 0:
                log.warning("whisper.cpp stderr: %s", result.stderr.strip())
                return ""
            return result.stdout.strip()
        finally:
            try:
                os.unlink(wav_path)
            except OSError:
                pass


@dataclass
class ClaudeLLM:
    """
    Anthropic Claude API client. Uses the modern Anthropic Python SDK.

    Pip install: `anthropic` (>=0.40.0)
    """
    api_key: str
    model: str = "claude-haiku-4-5-20251001"
    system_prompt: str = (
        "You are a helpful AI assistant accessible by amateur radio over "
        "a P25 voice link. Keep responses short, plain, and easy to follow "
        "when heard aloud — under 40 words. No code blocks, no markdown."
    )
    max_tokens: int = 200

    def __post_init__(self) -> None:
        # Lazy import so the module can be loaded without anthropic installed.
        from anthropic import Anthropic
        self._client = Anthropic(api_key=self.api_key)

    def respond(self, user_text: str) -> str:
        if not user_text.strip():
            return "I didn't catch that. Try again?"
        t0 = time.monotonic()
        resp = self._client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=self.system_prompt,
            messages=[{"role": "user", "content": user_text}],
        )
        elapsed = time.monotonic() - t0
        text = "".join(b.text for b in resp.content if b.type == "text").strip()
        log.info("Claude: %.2fs, %d chars", elapsed, len(text))
        return text


@dataclass
class PiperTTS:
    """
    Shells out to the Piper CLI. Piper outputs PCM on stdout when given
    `--output-raw`. Voice models are 22050 Hz typically; we resample to 8 kHz
    via sox (apt-get install sox).
    """
    binary: str = "piper"
    voice_path: str = "/opt/piper/voices/en_US-amy-medium.onnx"
    voice_rate: int = 22050

    def synthesize(self, text: str) -> bytes:
        t0 = time.monotonic()
        piper = subprocess.Popen(
            [self.binary, "--model", self.voice_path, "--output-raw"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        # gain -6: drop 6 dB to leave headroom — Piper output is hot enough
        # that IMBE encoding clips on peaks, which the radio renders as the
        # "loud and muffled" artifact. -6 dB sounds about right empirically.
        # rate -v: highest-quality resampler. Default sox rate is lower
        # quality, costing high-frequency detail when downsampling 22050 → 8000.
        sox = subprocess.Popen(
            ["sox", "-t", "raw", "-r", str(self.voice_rate), "-e", "signed",
             "-b", "16", "-c", "1", "-",
             "-t", "raw", "-r", str(PCM_SAMPLE_RATE), "-e", "signed",
             "-b", "16", "-c", "1", "-",
             "gain", "-6", "rate", "-v"],
            stdin=piper.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        if piper.stdout:
            piper.stdout.close()  # so sox sees EOF when piper finishes
        piper_stdin = piper.stdin
        assert piper_stdin is not None
        piper_stdin.write(text.encode("utf-8"))
        piper_stdin.close()
        pcm, _ = sox.communicate(timeout=15)
        piper.wait(timeout=15)
        elapsed = time.monotonic() - t0
        log.info("Piper+sox: %.2fs, %d bytes PCM", elapsed, len(pcm))
        return pcm


# ─── pipeline orchestrator ─────────────────────────────────────────────────

@dataclass
class Pipeline:
    stt: STT
    llm: LLM
    tts: TTS

    def run(self, pcm_in: bytes) -> Optional[bytes]:
        """
        Full PCM-in → PCM-out round trip. Returns None if any stage fails
        or yields empty (so the caller can skip TX).
        """
        if not pcm_in:
            return None

        try:
            text_in = self.stt.transcribe(pcm_in)
        except Exception:
            log.exception("STT failed")
            return None
        log.info("STT → %r", text_in)
        if not text_in.strip():
            return None  # nothing said, nothing to answer

        try:
            text_out = self.llm.respond(text_in)
        except Exception:
            log.exception("LLM failed")
            return None
        log.info("LLM → %r", text_out)
        if not text_out.strip():
            return None

        try:
            pcm_out = self.tts.synthesize(text_out)
        except Exception:
            log.exception("TTS failed")
            return None
        log.info("TTS → %d bytes PCM", len(pcm_out))
        return pcm_out


def build_pipeline(cfg: dict) -> Pipeline:
    """
    Construct the pipeline from a config dict. Each section can be set
    to "mock" to use the no-op variant — handy for offline testing.
    """
    stt_cfg = cfg.get("stt", {})
    if stt_cfg.get("kind", "whisper") == "mock":
        stt: STT = MockSTT()
    else:
        stt = WhisperCppSTT(
            binary=stt_cfg.get("binary", "/opt/whisper.cpp/main"),
            model_path=stt_cfg.get("model_path", "/opt/whisper.cpp/models/ggml-small.en.bin"),
            threads=int(stt_cfg.get("threads", 4)),
        )

    llm_cfg = cfg.get("llm", {})
    if llm_cfg.get("kind", "claude") == "mock":
        llm: LLM = MockLLM(system_prompt=llm_cfg.get("system_prompt", ""))
    else:
        llm = ClaudeLLM(
            api_key=llm_cfg.get("api_key") or os.environ.get("ANTHROPIC_API_KEY", ""),
            model=llm_cfg.get("model", "claude-haiku-4-5-20251001"),
            system_prompt=llm_cfg.get("system_prompt", ClaudeLLM.system_prompt),
            max_tokens=int(llm_cfg.get("max_tokens", 200)),
        )

    tts_cfg = cfg.get("tts", {})
    if tts_cfg.get("kind", "piper") == "mock":
        tts: TTS = MockTTS()
    else:
        tts = PiperTTS(
            binary=tts_cfg.get("binary", "piper"),
            voice_path=tts_cfg.get("voice_path", "/opt/piper/voices/en_US-amy-medium.onnx"),
            voice_rate=int(tts_cfg.get("voice_rate", 22050)),
        )

    return Pipeline(stt=stt, llm=llm, tts=tts)
