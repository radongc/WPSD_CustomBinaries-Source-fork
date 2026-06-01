"""
Voice pipeline: PCM in → STT → LLM → TTS → PCM out.

Two interfaces:
  - run(pcm_in) -> bytes      (legacy, blocking, returns full PCM)
  - run_stream(pcm_in) -> Iterator[bytes]   (streaming, yields PCM chunks)

The streaming path lets the bridge start transmitting as soon as the first
sentence has been TTS-rendered, instead of waiting for the entire LLM
response + TTS render to complete before TX begins. For long responses
this hides several seconds of latency under TX.

Audio convention everywhere here: 8 kHz, mono, signed 16-bit, little-endian
PCM. Matches the IMBE codec on both sides.
"""

from __future__ import annotations
import json
import logging
import os
import re
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request
import wave
from dataclasses import dataclass
from typing import Any, Iterator, Optional, Protocol

log = logging.getLogger("aibridge.pipeline")


PCM_SAMPLE_RATE = 8000


# Voice commands. Match if any of these substrings appear in the STT
# transcript (case-insensitive). Whisper's punctuation/capitalization
# may vary, so we keep these forgiving.
RESET_COMMANDS = (
    "reset conversation",
    "restart conversation",
    "new conversation",
    "new chat",
    "start over",
    "start it over",
    "forget everything",
    "forget what i said",
    "clear memory",
    "clear context",
    "clear chat",
    "clear conversation",
    "wipe memory",
)


# LLM providers selectable at runtime via voice command.
LLM_PROVIDERS = ("claude", "grok")

# OpenAI TTS voices keyed to the model they belong to. A voice command
# like "switch voice to ash" picks the right model automatically; the
# user doesn't have to know which voice lives on which model.
_TTS_VOICE_TO_MODEL = {
    # tts-1 / tts-1-hd voices
    "alloy": "tts-1",
    "echo": "tts-1",
    "fable": "tts-1",
    "onyx": "tts-1",
    "nova": "tts-1",
    "shimmer": "tts-1",
    # gpt-4o-mini-tts voices
    "ash": "gpt-4o-mini-tts",
    "ballad": "gpt-4o-mini-tts",
    "coral": "gpt-4o-mini-tts",
    "sage": "gpt-4o-mini-tts",
    "verse": "gpt-4o-mini-tts",
}


# Voice-command regexes for runtime LLM / TTS-voice switching.
_LLM_SWITCH_RE = re.compile(
    r"\b(?:switch|change|use|set)\s+(?:the\s+)?"
    r"(?:llm|ai|model|provider|assistant)\s+(?:to\s+)?"
    r"(claude|grok)\b",
    re.IGNORECASE,
)
_VOICE_SWITCH_RE = re.compile(
    r"\b(?:switch|change|set|use)\s+(?:the\s+)?"
    r"voice(?:\s+model)?\s+(?:to\s+)?([a-z]+)\b",
    re.IGNORECASE,
)
# Bare "switch to X" — disambiguates by whether X is a known LLM or
# TTS voice name.
_BARE_SWITCH_RE = re.compile(
    r"\bswitch\s+to\s+([a-z0-9_-]+)\b",
    re.IGNORECASE,
)


def _handle_voice_command(text: str, pipeline, conversation_id: str) -> Optional[str]:
    """If `text` matches a built-in voice command, perform it and return
    a canned reply string. Returns None if no command matched (caller
    should pass through to the LLM)."""
    lowered = text.lower().strip().rstrip(".!?")

    # 1) Conversation reset.
    for phrase in RESET_COMMANDS:
        if phrase in lowered:
            pipeline.llm.reset_conversation(conversation_id)
            log.info("Voice command matched: %r", phrase)
            return "Conversation reset. What's on your mind?"

    # 2) Explicit LLM switch ("switch llm to grok").
    m = _LLM_SWITCH_RE.search(lowered)
    if m:
        return pipeline.switch_llm(m.group(1).lower())

    # 3) Explicit voice switch ("switch voice to onyx").
    m = _VOICE_SWITCH_RE.search(lowered)
    if m:
        return pipeline.switch_voice(m.group(1).lower())

    # 4) Bare "switch to X" — match against the two name spaces.
    m = _BARE_SWITCH_RE.search(lowered)
    if m:
        target = m.group(1).lower()
        if target in LLM_PROVIDERS:
            return pipeline.switch_llm(target)
        if target in _TTS_VOICE_TO_MODEL:
            return pipeline.switch_voice(target)

    return None


# Match sentence-ending punctuation (., !, ?, \n) followed by whitespace
# or end of string. Used to chunk streaming LLM output into TTS-sized
# pieces — TTS quality is generally better per-sentence than per-token.
_SENTENCE_END = re.compile(r"[.!?\n](?:\s|$)")


# ─── tool: radioid.net lookup ──────────────────────────────────────────────

_RADIOID_URL = "https://radioid.net/api/dmr/user/?id={id}"


def _lookup_radio_id(unit_id: int, timeout: float = 8.0) -> str:
    """Query radioid.net for a numeric radio ID and return a human-
    readable summary string. The radioid.net DMR database is shared with
    P25/NXDN/Fusion for amateurs (same ID space), so this works for any
    of those modes' src_ids.

    Returns a short factual sentence on success, or an error/no-match
    message on failure. Always returns a string — the Anthropic API
    tool_result content field expects text."""
    try:
        with urllib.request.urlopen(
            _RADIOID_URL.format(id=int(unit_id)), timeout=timeout
        ) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, json.JSONDecodeError,
            TimeoutError, ValueError) as e:
        return f"radioid.net lookup failed: {e}"

    results = data.get("results") or []
    if not results:
        return f"No radioid.net record for unit ID {unit_id}."

    user = results[0]
    parts = [f"Unit ID {user.get('id', unit_id)}"]
    if user.get("callsign"):
        parts.append(f"callsign {user['callsign']}")
    if user.get("name"):
        parts.append(f"operator {user['name']}")
    location = ", ".join(
        x for x in (user.get("city"), user.get("state"), user.get("country"))
        if x
    )
    if location:
        parts.append(f"location {location}")
    return "; ".join(parts) + "."


# ─── tool: HamDB callsign lookup ───────────────────────────────────────────

_HAMDB_URL = "http://api.hamdb.org/v1/{callsign}/json/aibridge"


def _street_name_only(addr1: str) -> str:
    """Strip the house number off a US street address line, leaving the
    street name. '93 JUNIPER AVE' → 'JUNIPER AVE'. PO Boxes return
    empty (no street to extract)."""
    if not addr1:
        return ""
    s = addr1.strip()
    if re.match(r"^P\.?\s*O\.?\s+BOX\b", s, re.IGNORECASE):
        return ""
    # Drop a leading house number — allowing things like "93", "100A",
    # "1234-1236" — plus the whitespace that follows.
    return re.sub(
        r"^\d+[A-Z]?(?:[\s\-]+\d+[A-Z]?)?\s+",
        "",
        s,
        flags=re.IGNORECASE,
    ).strip()


def _lookup_callsign(callsign: str, timeout: float = 8.0) -> str:
    """Query api.hamdb.org for an amateur callsign and return a short
    human-readable summary. Public FCC + DXCC data only.

    Privacy filter: the upstream returns the licensee's full street
    address and ZIP. We drop the house number (so the bot can say
    'lives on Juniper Ave in Westerville, OH' without ever voicing
    the exact street number) and the ZIP. Coarse locality plus street
    name only."""
    cs = (callsign or "").strip().upper()
    if not cs or not re.fullmatch(r"[A-Z0-9/]+", cs):
        return f"Invalid callsign {callsign!r}; expected letters/digits only."
    try:
        with urllib.request.urlopen(
            _HAMDB_URL.format(callsign=cs), timeout=timeout
        ) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except (urllib.error.URLError, json.JSONDecodeError,
            TimeoutError, ValueError) as e:
        return f"HamDB lookup failed: {e}"

    record = (data.get("hamdb") or {}).get("callsign") or {}
    status = (data.get("hamdb") or {}).get("messages", {}).get("status", "")
    if not record or status.upper() not in ("OK", ""):
        return f"No HamDB record for callsign {cs}."

    name = " ".join(
        x for x in (record.get("fname"), record.get("mi"), record.get("name"))
        if x and x.strip()
    )
    street = _street_name_only(record.get("addr1", ""))
    locality = ", ".join(
        x for x in (street, record.get("addr2"), record.get("state"),
                    record.get("country"))
        if x and x.strip()
    )
    license_class = {
        "T": "Technician", "G": "General", "A": "Advanced",
        "E": "Extra", "N": "Novice", "P": "Tech Plus",
    }.get(record.get("class", ""), record.get("class") or "")
    license_status = {
        "A": "active", "E": "expired", "C": "cancelled",
    }.get(record.get("status", ""), record.get("status") or "")

    parts = [f"Callsign {record.get('call', cs)}"]
    if name:
        parts.append(f"licensee {name}")
    if license_class:
        suffix = f" ({license_status})" if license_status else ""
        parts.append(f"{license_class} class{suffix}")
    elif license_status:
        parts.append(f"license status {license_status}")
    if locality:
        parts.append(f"location {locality}")
    if record.get("grid"):
        parts.append(f"grid {record['grid']}")
    if record.get("expires"):
        parts.append(f"license expires {record['expires']}")
    return "; ".join(parts) + "."


# ─── interfaces ────────────────────────────────────────────────────────────

class STT(Protocol):
    def transcribe(self, pcm: bytes) -> str: ...


class LLM(Protocol):
    def respond(self, user_text: str, conversation_id: str = "default") -> str: ...
    def respond_stream(self, user_text: str, conversation_id: str = "default") -> Iterator[str]: ...
    def reset_conversation(self, conversation_id: str = "default") -> None: ...


class TTS(Protocol):
    def synthesize(self, text: str) -> bytes:
        """Returns 8 kHz / 16-bit / mono PCM bytes (whole response)."""
        ...

    def synthesize_stream(self, text: str) -> Iterator[bytes]:
        """Yields 8 kHz / 16-bit / mono PCM chunks as they're produced."""
        ...


# ─── mock implementations ──────────────────────────────────────────────────

class MockSTT:
    def transcribe(self, pcm: bytes) -> str:
        secs = len(pcm) / (PCM_SAMPLE_RATE * 2)
        log.info("MockSTT: %.1fs of audio → 'hello there'", secs)
        return "hello there"


class MockLLM:
    def __init__(self, system_prompt: str = "") -> None:
        self.system_prompt = system_prompt

    def respond(self, user_text: str, conversation_id: str = "default") -> str:
        log.info("MockLLM[%s]: %r → echo", conversation_id, user_text)
        return f"You said: {user_text}. This is the mock bot."

    def respond_stream(self, user_text: str, conversation_id: str = "default") -> Iterator[str]:
        # Single-shot stream — yield the whole canned response as one sentence.
        yield self.respond(user_text, conversation_id)

    def reset_conversation(self, conversation_id: str = "default") -> None:
        log.info("MockLLM[%s]: reset (no-op)", conversation_id)


class MockTTS:
    """
    Generates a short tone-pip so we can hear something arrive on the radio
    even before real TTS is wired up.
    """
    def _make_tone(self, text: str) -> bytes:
        import math
        log.info("MockTTS: %d chars → 1 sec sine wave", len(text))
        n = PCM_SAMPLE_RATE
        amp = 8000
        freq = 440
        samples = bytearray()
        for i in range(n):
            v = int(amp * math.sin(2 * math.pi * freq * i / PCM_SAMPLE_RATE))
            samples += int(v).to_bytes(2, "little", signed=True)
        return bytes(samples)

    def synthesize(self, text: str) -> bytes:
        return self._make_tone(text)

    def synthesize_stream(self, text: str) -> Iterator[bytes]:
        # Yield the whole tone as one chunk — mocks don't pretend to stream.
        yield self._make_tone(text)


# ─── real implementations (lightweight subprocess wrappers) ────────────────

@dataclass
class WhisperCppSTT:
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
    Anthropic Claude API client with per-conversation memory and streaming.

    Memory model:
      - Keyed by conversation_id (typically the radio's src_id).
      - Auto-reset after `idle_reset_sec` (default 600s) of channel silence.
      - Capped at `max_turns` user+assistant pairs (default 20).
      - In-memory only; reset on bridge restart.

    Streaming model:
      - respond_stream() yields complete sentences as they form.
      - respond() is now a thin wrapper that drains the stream.
      - If the generator is closed early (consumer abandons it), the
        partial response is discarded and the user turn is rolled back —
        history stays alternating and consistent.

    Tools:
      - web_search (server-side): when enabled, Claude can search the web
        itself. The SDK's text_stream filters out the tool-use machinery
        and yields only response text, so we don't have to run a tool
        loop locally. There's just a pause in the stream while the search
        runs — covered by the bridge's first-byte timeout.
    """
    api_key: str
    model: str = "claude-haiku-4-5-20251001"
    system_prompt: str = (
        "You are a helpful AI assistant accessible by amateur radio over "
        "a P25 voice link. Your name is Harald Morbidity Thoribius Morbius III."
        "Keep responses to the point, plain, and easy to follow when heard aloud. You have web search — use it when asked "
        "about current events, weather, prices, news, or anything time-"
        "sensitive. Respond naturally; never include URLs, citation numbers, "
        "or markdown syntax — paraphrase what you found instead. "
        "Responses should be as short as possible while still giving a good answer. "
        "Strive to keep responses under 200 words OR under 1 minute when read aloud - unless otherwise requested by the user, or a good response/answer simply won't reasonably fit in that amount of words. Anything under that is more than fine."
        "The point is to be conversational first, at all times, with the understanding that everything you say will be transmitted via P25 radio. "
        "FORMATTING - IMPORTANT REMINDER: NO code blocks, NO MARKDOWN - do not use any special characters or non-conversational speech (ie. no bullet pointing with colons, etc.) under any circumstances. Paraphrase instead!"
        "Also, DO NOT abruptly cut off responses for any reason - to comply with the 200 word/1 minute rule or otherwise - it is a soft rule, not a hard one. It is more important that you give complete answers than arbitrarily meet this condition IF the response otherwise wouldn't make sense."
        "Unit IDs should be formatted as a space-separated string of digits to work with the text-to-speech model. For example, the unit ID 1234567 should be formatted like 1 2 3 4 5 6 7 in responses - not 1234567."

        "ATTITUDE: You should be unbiased, objective, and blunt at all times. Do not sugar-coat or be overly-polite. Your attitude should depend entirely on the conversation's context - always remain helpful and informative, but don't be afraid to be blunt if the conversation calls for it."
        "HONESTY (most important rule): Never invent facts, names, callsigns, numbers, dates, URLs, or anything else. If you are not confident in an answer, do not guess. Your default response when you are unsure is to use web search to find out. If web search is unavailable, fails, or doesn't return what you need, say so plainly — e.g. 'I don't know,' 'I couldn't find that,' or 'I'm not sure, and I couldn't verify it.' It is always better to admit you don't know than to fabricate. This rule applies equally to lookup_radio_id: if the radioid.net lookup returns no record or an error, say that — do not invent a callsign or operator name. Do not paper over uncertainty with confident-sounding language."
        "PLATFORM INFO: You operate as a modification/extension on the WPSD project. P25 is transliterated to text via STT, and then used to interact with an LLM api. Responses are then used to generate TTS, which is converted into AMBE/P25 packets and transmitted over the hotspot."
    )
    max_tokens: int = 200
    idle_reset_sec: float = 600.0
    max_turns: int = 20
    # Server-side web search (Anthropic-hosted). Cheap to keep enabled —
    # Claude only invokes the tool when it judges that current info is
    # actually needed.
    web_search: bool = True
    web_search_max_uses: int = 3
    # When True, append the speaker's P25 unit ID (= conversation_id,
    # which is the radio's src_id) to the system prompt. Lets the bot
    # reference the caller's unit ID if asked but doesn't make it
    # announce the ID spontaneously.
    include_unit_id: bool = False
    # When True, expose a client-side tool that queries radioid.net for
    # a numeric radio ID and returns callsign / operator info. Useful
    # paired with include_unit_id — the bot can answer "what's my
    # callsign" by looking the caller's src_id up directly.
    radioid_lookup: bool = True
    # When True, expose a client-side tool that queries HamDB for an
    # amateur callsign. Returns name, license class, locality, grid,
    # and license expiration. House number and ZIP are filtered out —
    # over a hotspot we voice street name + city, not full address.
    callsign_lookup: bool = True
    # Safety cap on the tool-use loop. Each iteration is one extra API
    # round-trip plus one tool execution.
    max_tool_rounds: int = 4

    def __post_init__(self) -> None:
        from anthropic import Anthropic
        self._client = Anthropic(api_key=self.api_key)
        self._histories: dict[str, list[dict]] = {}
        self._last_seen: dict[str, float] = {}
        # Materialise the tools list once. Empty list means we omit the
        # `tools` kwarg from the request (no tool calls possible).
        self._tools: list[dict] = []
        if self.web_search:
            self._tools.append({
                "type": "web_search_20250305",
                "name": "web_search",
                "max_uses": self.web_search_max_uses,
            })
        if self.radioid_lookup:
            self._tools.append({
                "name": "lookup_radio_id",
                "description": (
                    "Look up a numeric P25 / DMR / NXDN / Fusion radio unit "
                    "ID in the radioid.net amateur radio database. Returns "
                    "the operator's callsign, name, and location if "
                    "registered. Use this when the user asks about a unit "
                    "ID's owner, callsign, or location — including their "
                    "own (the speaker's unit ID is in the system prompt if "
                    "available)."
                ),
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "unit_id": {
                            "type": "integer",
                            "description": (
                                "The numeric radio ID to look up "
                                "(1-16777215)."
                            ),
                        },
                    },
                    "required": ["unit_id"],
                },
            })
        if self.callsign_lookup:
            self._tools.append({
                "name": "lookup_callsign",
                "description": (
                    "Look up an amateur radio callsign in the public HamDB "
                    "database (FCC for US calls, DXCC for others). Returns "
                    "the licensee's name, license class and status, "
                    "Maidenhead grid square, license expiration, and "
                    "general locality (street name, city, state, country). "
                    "The exact house number and ZIP are deliberately not "
                    "returned. Use this when the user asks about a specific "
                    "callsign — including chained from lookup_radio_id, "
                    "e.g. unit ID -> callsign -> licensee details."
                ),
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "callsign": {
                            "type": "string",
                            "description": (
                                "Amateur callsign, e.g. 'W1AW' or 'KE8UID'. "
                                "Case-insensitive. Letters, digits, and '/' "
                                "for portable/slash designators."
                            ),
                        },
                    },
                    "required": ["callsign"],
                },
            })

    def _execute_tool(self, name: str, tool_input: dict) -> str:
        """Dispatch a client-side tool call. Returns a text result that
        will be fed back to the model as tool_result content."""
        if name == "lookup_radio_id":
            try:
                unit_id = int(tool_input.get("unit_id"))
            except (TypeError, ValueError):
                return "Invalid unit_id; expected an integer."
            result = _lookup_radio_id(unit_id)
            log.info("Tool lookup_radio_id(%d) → %s", unit_id, result)
            return result
        if name == "lookup_callsign":
            callsign = tool_input.get("callsign") or ""
            if not isinstance(callsign, str):
                return "Invalid callsign; expected a string."
            result = _lookup_callsign(callsign)
            log.info("Tool lookup_callsign(%r) → %s", callsign, result)
            return result
        log.warning("Tool %r requested but no handler", name)
        return f"Tool {name!r} is not implemented."

    def _build_system(self, conversation_id: str) -> str:
        """Per-request system prompt. Same content for every turn of a
        given conversation, so it acts as stable context the model can
        reference without it being part of the visible turn history."""
        sys = self.system_prompt
        if (self.include_unit_id
                and conversation_id
                and conversation_id not in ("anon", "default")):
            sys += (
                f"\n\nRadio context: the speaker's P25 unit ID is "
                f"{conversation_id}. Reference it only if relevant or asked; "
                f"do not announce it unprompted."
            )
        return sys

    def respond(self, user_text: str, conversation_id: str = "default") -> str:
        """Blocking call — fully drains the stream and joins all sentences."""
        sentences = list(self.respond_stream(user_text, conversation_id))
        return " ".join(sentences).strip()

    def respond_stream(self, user_text: str, conversation_id: str = "default") -> Iterator[str]:
        if not user_text.strip():
            yield "I didn't catch that. Try again?"
            return

        # Apply idle reset before adding the new turn.
        now = time.monotonic()
        prior = self._last_seen.get(conversation_id, 0.0)
        if prior and (now - prior) > self.idle_reset_sec:
            log.info("Claude[%s]: idle %.0fs, resetting context",
                     conversation_id, now - prior)
            self._histories.pop(conversation_id, None)

        history = self._histories.setdefault(conversation_id, [])
        # Build a working messages list. We only persist user_text + the
        # final assistant text to long-term history; the intermediate
        # tool_use / tool_result blocks live only in this list for the
        # duration of the request.
        messages: list[dict] = list(history)
        messages.append({"role": "user", "content": user_text})

        full_text_parts: list[str] = []
        sentence_buf = ""
        t0 = time.monotonic()

        def _drain_sentences() -> Iterator[str]:
            """Pull complete sentences out of sentence_buf and yield them."""
            nonlocal sentence_buf
            while True:
                match = _SENTENCE_END.search(sentence_buf)
                if not match:
                    return
                end = match.end()
                sentence = sentence_buf[:end].strip()
                sentence_buf = sentence_buf[end:]
                if sentence:
                    full_text_parts.append(sentence)
                    yield sentence

        try:
            for _round in range(self.max_tool_rounds + 1):
                stream_kwargs: dict[str, Any] = dict(
                    model=self.model,
                    max_tokens=self.max_tokens,
                    system=self._build_system(conversation_id),
                    messages=messages,
                )
                if self._tools:
                    stream_kwargs["tools"] = self._tools

                with self._client.messages.stream(**stream_kwargs) as stream:
                    for delta_text in stream.text_stream:
                        sentence_buf += delta_text
                        yield from _drain_sentences()
                    final_msg = stream.get_final_message()

                if final_msg.stop_reason != "tool_use":
                    # Normal end of turn — flush trailing partial sentence.
                    remainder = sentence_buf.strip()
                    if remainder:
                        full_text_parts.append(remainder)
                        yield remainder
                        sentence_buf = ""
                    break

                # Tool round: execute every requested tool, append the
                # assistant turn + results, loop for the model's follow-up.
                messages.append({
                    "role": "assistant",
                    "content": final_msg.content,
                })
                tool_results: list[dict] = []
                for block in final_msg.content:
                    if getattr(block, "type", None) != "tool_use":
                        continue
                    tool_input = getattr(block, "input", {}) or {}
                    result = self._execute_tool(block.name, tool_input)
                    tool_results.append({
                        "type": "tool_result",
                        "tool_use_id": block.id,
                        "content": result,
                    })
                messages.append({"role": "user", "content": tool_results})
            else:
                log.warning(
                    "Claude[%s]: tool loop hit max_tool_rounds=%d; giving up",
                    conversation_id, self.max_tool_rounds,
                )

        finally:
            elapsed = time.monotonic() - t0
            full_text = " ".join(full_text_parts).strip()

            if full_text:
                history.append({"role": "user", "content": user_text})
                history.append({"role": "assistant", "content": full_text})
                if len(history) > self.max_turns:
                    del history[:2]
                self._last_seen[conversation_id] = now
                log.info("Claude[%s] stream: %.2fs, %d chars, %d turns in context",
                         conversation_id, elapsed, len(full_text), len(history))
            else:
                log.info("Claude[%s] stream: aborted with no output (%.2fs)",
                         conversation_id, elapsed)

    def reset_conversation(self, conversation_id: str = "default") -> None:
        if conversation_id in self._histories:
            log.info("Claude[%s]: manual reset, %d turns dropped",
                     conversation_id, len(self._histories[conversation_id]))
            del self._histories[conversation_id]


@dataclass
class GrokLLM:
    """
    xAI Grok client. Uses the Responses API at /v1/responses (NOT
    chat.completions), because xAI hard-deprecated Live Search on
    chat.completions in Jan 2026 and its replacement — server-side
    web_search via the Agent Tools API — is only exposed on
    /v1/responses. Talking to that endpoint via the `openai` SDK is
    `client.responses.create(...)`.

    Differences from ClaudeLLM beyond the underlying API:
      - Web search is the simplest server-side tool: just
        `{"type": "web_search"}`. The model decides when to invoke it,
        and we don't see it in the function-call stream.
      - Client-side function tools use the Responses API's *flat* shape
        (name/description/parameters at top level of the tool entry),
        not the nested `function: {...}` shape that chat.completions
        uses.
      - Tool follow-up uses `previous_response_id` plus a fresh `input`
        list of `function_call_output` items, instead of replaying the
        whole message list.
      - System prompt goes in the `instructions=` kwarg, not in `input`.

    Memory model is the same: long-term history only ever stores the
    original user_text and final assistant text; intermediate
    function_call / function_call_output items stay scoped to the
    request.
    """
    api_key: str
    # grok-4-fast and the other "-fast" tier IDs were retired
    # 2026-05-15 and now silently redirect to grok-4.3. Default to that
    # explicitly; override via config.yaml if xAI's naming moves again.
    model: str = "grok-4.3"
    # Grok-specific system prompt. Tells it to lean into its own
    # personality (witty, blunt, unfiltered) while still respecting the
    # voice-link formatting rules and the honesty / no-hallucination
    # contract. Voice-only constraints (no markdown, no URLs spoken,
    # digit-spelling unit IDs) are non-negotiable because the TTS will
    # literally read whatever it emits.
    system_prompt: str = (
        "You are Harold Morbald Thorb Morbius Morbidity the third, the twin brother of Grok, the AI assistant built by xAI, accessed by a "
        "licensed amateur radio operator over a P25 voice link. Be "
        "yourself — witty, irreverent, blunt, willing to engage with "
        "anything. Don't sanitize, don't moralize unprompted, don't "
        "dodge hard or edgy questions."

        "VOICE LINK FORMAT — non-negotiable, because every word is "
        "synthesized to audio and transmitted over P25:\n"
        "- No code blocks, no markdown, no bullet points, no lists, "
        "no asterisks, no headings, no special characters of any kind. "
        "Just plain sentences a human can speak.\n"
        "- Never read URLs or citation numbers aloud. Paraphrase "
        "sources instead.\n"
        "- When you mention a numeric radio unit ID, separate the "
        "digits with spaces or dashes so TTS reads each digit "
        "individually — 1234567 becomes '1 2 3 4 5 6 7' or "
        "'1-2-3-4-5-6-7', never the seven-digit number whole.\n\n"

        "LENGTH: Aim to keep replies as short as possible while still actually "
        "answering. Aim under about 200 words / one minute of speech — "
        "soft target, not a hard cap, so never outright refuse to give a long answer to a question. You may be long-winded if context calls for it. Don't truncate mid-thought just "
        "to hit a word count; finish your answer cleanly.\n\n"

        "HONESTY (the one rule you don't bend): never invent facts, "
        "names, callsigns, numbers, dates, URLs, or anything else. When "
        "you're not sure, default to web search. If web search fails or "
        "doesn't return what you need, say so plainly — 'I don't know,' "
        "'I couldn't find that,' 'not sure, couldn't verify.' Better to "
        "admit uncertainty than to bullshit. Same rule for "
        "lookup_radio_id and lookup_callsign: if the lookup returns no "
        "record or an error, say so — don't invent a callsign or "
        "operator name.\n\n"

        "PLATFORM CONTEXT: you run as a modification on the WPSD "
        "hotspot project. Inbound P25 voice goes through Whisper STT "
        "to you; your text goes through TTS and IMBE back out over the "
        "user's hotspot. Tools available: web search (live), "
        "lookup_radio_id (radioid.net DMR/P25/NXDN/Fusion database), "
        "lookup_callsign (HamDB / FCC ULS for hams)."
    )
    max_tokens: int = 200
    idle_reset_sec: float = 600.0
    max_turns: int = 20
    web_search: bool = True
    include_unit_id: bool = False
    radioid_lookup: bool = True
    callsign_lookup: bool = True
    max_tool_rounds: int = 4
    base_url: str = "https://api.x.ai/v1"

    def __post_init__(self) -> None:
        from openai import OpenAI
        self._client = OpenAI(api_key=self.api_key, base_url=self.base_url)
        self._histories: dict[str, list[dict]] = {}
        self._last_seen: dict[str, float] = {}
        # Responses-API tool list. Note: function tools are flat here
        # (name/description/parameters at the top level), unlike the
        # chat.completions shape that nests them under `function: {...}`.
        self._tools: list[dict] = []
        if self.web_search:
            # Server-side tool — Grok handles it internally; we never
            # see a corresponding function_call item to dispatch.
            self._tools.append({"type": "web_search"})
        if self.radioid_lookup:
            self._tools.append({
                "type": "function",
                "name": "lookup_radio_id",
                "description": (
                    "Look up a numeric P25 / DMR / NXDN / Fusion radio "
                    "unit ID in the radioid.net amateur radio database. "
                    "Returns the operator's callsign, name, and "
                    "location."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "unit_id": {
                            "type": "integer",
                            "description": (
                                "Numeric radio ID (1-16777215)."
                            ),
                        },
                    },
                    "required": ["unit_id"],
                },
            })
        if self.callsign_lookup:
            self._tools.append({
                "type": "function",
                "name": "lookup_callsign",
                "description": (
                    "Look up an amateur callsign in HamDB (FCC + DXCC). "
                    "Returns licensee name, license class/status, "
                    "Maidenhead grid, license expiration, and general "
                    "locality. House number and ZIP are filtered out."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "callsign": {
                            "type": "string",
                            "description": (
                                "Callsign, e.g. 'W1AW'. Case-insensitive."
                            ),
                        },
                    },
                    "required": ["callsign"],
                },
            })

    def _execute_tool(self, name: str, tool_input: dict) -> str:
        if name == "lookup_radio_id":
            try:
                unit_id = int(tool_input.get("unit_id"))
            except (TypeError, ValueError):
                return "Invalid unit_id; expected an integer."
            result = _lookup_radio_id(unit_id)
            log.info("Tool lookup_radio_id(%d) → %s", unit_id, result)
            return result
        if name == "lookup_callsign":
            callsign = tool_input.get("callsign") or ""
            if not isinstance(callsign, str):
                return "Invalid callsign; expected a string."
            result = _lookup_callsign(callsign)
            log.info("Tool lookup_callsign(%r) → %s", callsign, result)
            return result
        log.warning("Tool %r requested but no handler", name)
        return f"Tool {name!r} is not implemented."

    def _build_system(self, conversation_id: str) -> str:
        sys = self.system_prompt
        if (self.include_unit_id
                and conversation_id
                and conversation_id not in ("anon", "default")):
            sys += (
                f"\n\nRadio context: the speaker's P25 unit ID is "
                f"{conversation_id}. Reference it only if relevant or asked; "
                f"do not announce it unprompted."
            )
        return sys

    def respond(self, user_text: str, conversation_id: str = "default") -> str:
        sentences = list(self.respond_stream(user_text, conversation_id))
        return " ".join(sentences).strip()

    def respond_stream(self, user_text: str,
                       conversation_id: str = "default") -> Iterator[str]:
        if not user_text.strip():
            yield "I didn't catch that. Try again?"
            return

        now = time.monotonic()
        prior = self._last_seen.get(conversation_id, 0.0)
        if prior and (now - prior) > self.idle_reset_sec:
            log.info("Grok[%s]: idle %.0fs, resetting context",
                     conversation_id, now - prior)
            self._histories.pop(conversation_id, None)

        history = self._histories.setdefault(conversation_id, [])
        # Initial input is the prior user/assistant history followed by
        # the new user turn. After a tool round, we'll replace this with
        # a list of function_call_output items and chain via
        # previous_response_id.
        input_items: list = list(history) + [
            {"role": "user", "content": user_text}
        ]
        instructions = self._build_system(conversation_id)

        full_text_parts: list[str] = []
        sentence_buf = ""
        t0 = time.monotonic()
        previous_response_id: Optional[str] = None

        def _drain_sentences() -> Iterator[str]:
            nonlocal sentence_buf
            while True:
                match = _SENTENCE_END.search(sentence_buf)
                if not match:
                    return
                end = match.end()
                sentence = sentence_buf[:end].strip()
                sentence_buf = sentence_buf[end:]
                if sentence:
                    full_text_parts.append(sentence)
                    yield sentence

        try:
            for _round in range(self.max_tool_rounds + 1):
                create_kwargs: dict[str, Any] = dict(
                    model=self.model,
                    input=input_items,
                    max_output_tokens=self.max_tokens,
                    stream=True,
                )
                if self._tools:
                    create_kwargs["tools"] = self._tools
                if previous_response_id is not None:
                    # xAI's Responses API rejects `instructions` + a
                    # `previous_response_id` together — instructions are
                    # already carried by the chained response, so on
                    # follow-up rounds we omit them.
                    create_kwargs["previous_response_id"] = previous_response_id
                else:
                    create_kwargs["instructions"] = instructions

                final_response = None
                stream = self._client.responses.create(**create_kwargs)
                for event in stream:
                    etype = getattr(event, "type", "")
                    if etype == "response.output_text.delta":
                        delta = getattr(event, "delta", "") or ""
                        if delta:
                            sentence_buf += delta
                            yield from _drain_sentences()
                    elif etype == "response.completed":
                        final_response = getattr(event, "response", None)
                    elif etype == "response.error":
                        err = getattr(event, "error", None)
                        log.warning("Grok[%s] stream error event: %r",
                                    conversation_id, err)

                if final_response is None:
                    log.warning(
                        "Grok[%s]: stream ended without response.completed",
                        conversation_id,
                    )
                    break

                previous_response_id = getattr(final_response, "id", None)

                # Find any client-side function_call items in the output.
                function_calls = []
                for item in getattr(final_response, "output", []) or []:
                    if getattr(item, "type", "") == "function_call":
                        function_calls.append(item)

                if not function_calls:
                    remainder = sentence_buf.strip()
                    if remainder:
                        full_text_parts.append(remainder)
                        yield remainder
                        sentence_buf = ""
                    break

                # Execute each tool call and prepare function_call_output
                # items for the follow-up round.
                input_items = []
                for fc in function_calls:
                    args_str = getattr(fc, "arguments", "") or "{}"
                    try:
                        args = json.loads(args_str)
                    except json.JSONDecodeError:
                        args = {}
                    name = getattr(fc, "name", "") or ""
                    result = self._execute_tool(name, args)
                    input_items.append({
                        "type": "function_call_output",
                        "call_id": getattr(fc, "call_id", "") or "",
                        "output": result,
                    })
            else:
                log.warning(
                    "Grok[%s]: tool loop hit max_tool_rounds=%d; giving up",
                    conversation_id, self.max_tool_rounds,
                )

        finally:
            elapsed = time.monotonic() - t0
            full_text = " ".join(full_text_parts).strip()
            if full_text:
                history.append({"role": "user", "content": user_text})
                history.append({"role": "assistant", "content": full_text})
                if len(history) > self.max_turns:
                    del history[:2]
                self._last_seen[conversation_id] = now
                log.info("Grok[%s] stream: %.2fs, %d chars, %d turns in context",
                         conversation_id, elapsed, len(full_text), len(history))
            else:
                log.info("Grok[%s] stream: aborted with no output (%.2fs)",
                         conversation_id, elapsed)

    def reset_conversation(self, conversation_id: str = "default") -> None:
        if conversation_id in self._histories:
            log.info("Grok[%s]: manual reset, %d turns dropped",
                     conversation_id, len(self._histories[conversation_id]))
            del self._histories[conversation_id]


@dataclass
class PiperTTS:
    """
    Shells out to Piper (PCM on stdout via --output-raw) piped through sox
    for resample + gain. Both synthesize() (blocking) and
    synthesize_stream() (incremental) are supported. The streaming variant
    is what the bridge uses; the blocking one is kept for testing and
    backward compatibility.
    """
    binary: str = "piper"
    voice_path: str = "/opt/piper/voices/en_US-ryan-medium.onnx"
    voice_rate: int = 22050

    # Read this many bytes per sox.stdout.read() call. 8192 bytes = ~0.5s
    # of 8 kHz / 16-bit PCM, which is small enough for responsive TX
    # start and large enough to avoid syscall thrash.
    _CHUNK_BYTES: int = 8192

    def synthesize(self, text: str) -> bytes:
        """Blocking — drains the stream into a single bytes buffer."""
        return b"".join(self.synthesize_stream(text))

    def synthesize_stream(self, text: str) -> Iterator[bytes]:
        """Yields PCM chunks as Piper produces them. ~0.5s of audio per
        chunk by default. Subprocesses are killed cleanly if the consumer
        abandons the generator mid-stream."""
        t0 = time.monotonic()
        piper = subprocess.Popen(
            [self.binary, "--model", self.voice_path, "--output-raw"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        # gain -6: 6 dB headroom prevents IMBE clipping on Piper peaks.
        # rate -v: high-quality 22050 → 8000 downsample.
        # (We previously also stripped Piper's leading/trailing silence,
        #  on the theory that low-energy IMBE frames at sentence
        #  boundaries were what triggered the carrier blips. Turned out
        #  the real cause was TX pacing drift; with that fixed, Piper's
        #  ~60-120 ms of natural inter-sentence silence is desirable —
        #  trimming it made the speech feel rushed.)
        sox = subprocess.Popen(
            ["sox", "-t", "raw", "-r", str(self.voice_rate), "-e", "signed",
             "-b", "16", "-c", "1", "-",
             "-t", "raw", "-r", str(PCM_SAMPLE_RATE), "-e", "signed",
             "-b", "16", "-c", "1", "-",
             "gain", "-6", "rate", "-v"],
            stdin=piper.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        if piper.stdout:
            piper.stdout.close()  # sox owns it now; we need the FD released

        piper_stdin = piper.stdin
        assert piper_stdin is not None
        piper_stdin.write(text.encode("utf-8"))
        piper_stdin.close()

        total_bytes = 0
        try:
            assert sox.stdout is not None
            while True:
                chunk = sox.stdout.read(self._CHUNK_BYTES)
                if not chunk:
                    break
                total_bytes += len(chunk)
                yield chunk

            # Stream drained — let the processes exit cleanly.
            try:
                sox.wait(timeout=10)
                piper.wait(timeout=10)
            except subprocess.TimeoutExpired:
                log.warning("Piper/sox didn't exit cleanly after stream end")

            elapsed = time.monotonic() - t0
            if sox.returncode != 0 and sox.stderr is not None:
                err = sox.stderr.read().decode("utf-8", errors="replace")[:200]
                log.warning("sox returned %d: %s", sox.returncode, err)
            log.info("Piper+sox stream: %.2fs, %d bytes PCM (%.1fs audio)",
                     elapsed, total_bytes,
                     total_bytes / (PCM_SAMPLE_RATE * 2))
        finally:
            # On GeneratorExit (consumer abandoned the stream) or any
            # exception, make sure both subprocesses are reaped.
            for proc in (sox, piper):
                if proc.poll() is None:
                    proc.kill()
                    try:
                        proc.communicate(timeout=5)
                    except (subprocess.TimeoutExpired, ValueError):
                        pass


@dataclass
class OpenAITTS:
    """
    OpenAI text-to-speech client. Hits the streaming PCM endpoint and
    pipes the output through sox for resample + headroom — same shape
    as PiperTTS so the bridge consumes it identically.

    Default voice/model:
      tts-1 + onyx — deepest / most "project-able" voice for a P25
      voice link. tts-1 is fast and cheap (~$15/1M chars). For higher
      quality try tts-1-hd ($30/1M) or gpt-4o-mini-tts (same price as
      tts-1, supports voice-steering instructions and adds newer
      voices like ash, ballad, coral, sage).

    Source rate is 24 kHz (OpenAI's PCM output format); sox downsamples
    to the bridge's 8 kHz. The OpenAI stream feeds sox's stdin from a
    producer thread while we read sox's stdout chunk-by-chunk here, so
    the bridge sees PCM as soon as the first sox-resampled bytes are
    ready.
    """
    api_key: str
    # Defaults to gpt-4o-mini-tts so the `instructions` voice-steering
    # field below is honored. tts-1 / tts-1-hd silently ignore it. Same
    # price as tts-1 ($15/1M chars).
    model: str = "gpt-4o-mini-tts"
    voice: str = "ash"
    # Voice-steering prompt — only meaningful on gpt-4o-mini-tts.
    # Default to a clean, conversational two-way-radio delivery: quick,
    # clear, no announcer affect. Override via config to taste.
    instructions: str = (
        "Speak like a regular person talking over a two-way radio — "
        "or a police dispatcher. Clear and crisp, slightly quick, "
        "direct. Natural conversational rhythm; no announcer voice, "
        "no extra warmth, no dramatic pauses. Just deliver the "
        "information cleanly."
    )
    base_url: str = "https://api.openai.com/v1"
    # Silence padding prepended to every utterance, in milliseconds.
    # Piper has ~50-100 ms of natural leading silence which hides the
    # modem's PTT / carrier-acquisition lag. OpenAI gpt-4o-mini-tts has
    # essentially zero leading silence, so the first phoneme of a
    # short word ("No.") can fall inside that PTT window and get
    # clipped on-air. 150 ms is enough headroom on most hotspot
    # modems; bump it if the first syllable still gets eaten.
    preamble_ms: int = 150
    # OpenAI's PCM output sample rate.
    _SOURCE_RATE: int = 24000
    # Bytes per sox.stdout.read() call. Same sizing rationale as PiperTTS.
    _CHUNK_BYTES: int = 8192

    def __post_init__(self) -> None:
        from openai import OpenAI
        self._client = OpenAI(api_key=self.api_key, base_url=self.base_url)

    def synthesize(self, text: str) -> bytes:
        """Blocking — drains the stream into a single bytes buffer."""
        return b"".join(self.synthesize_stream(text))

    def synthesize_stream(self, text: str) -> Iterator[bytes]:
        """Yields PCM chunks as OpenAI produces them and sox resamples
        them. The OpenAI streaming response feeds sox's stdin on a
        producer thread; sox's stdout drives the chunks we yield."""
        t0 = time.monotonic()
        # gain -6 mirrors PiperTTS: 6 dB headroom prevents IMBE clipping
        # on TTS peaks. rate -v: high-quality 24000 → 8000 downsample.
        sox = subprocess.Popen(
            ["sox", "-t", "raw", "-r", str(self._SOURCE_RATE), "-e", "signed",
             "-b", "16", "-c", "1", "-",
             "-t", "raw", "-r", str(PCM_SAMPLE_RATE), "-e", "signed",
             "-b", "16", "-c", "1", "-",
             "gain", "-6", "rate", "-v"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )

        feeder_error: list[Optional[BaseException]] = [None]

        def feed_sox() -> None:
            """Stream OpenAI's PCM bytes into sox.stdin."""
            assert sox.stdin is not None
            # Preamble silence — pads the start of every utterance so the
            # modem PTT lag / radio carrier-acquisition window eats
            # silence instead of the first phoneme of the actual word.
            if self.preamble_ms > 0:
                preamble_bytes = (self._SOURCE_RATE * 2
                                  * self.preamble_ms // 1000)
                try:
                    sox.stdin.write(b"\x00" * preamble_bytes)
                except (BrokenPipeError, ValueError):
                    return

            create_kwargs: dict[str, Any] = dict(
                model=self.model,
                voice=self.voice,
                input=text,
                response_format="pcm",
            )
            if self.instructions:
                # Only meaningful on gpt-4o-mini-tts; older tts-1 / hd
                # silently ignore the field.
                create_kwargs["instructions"] = self.instructions
            try:
                with self._client.audio.speech.with_streaming_response.create(
                    **create_kwargs
                ) as response:
                    for chunk in response.iter_bytes():
                        if not chunk:
                            continue
                        try:
                            sox.stdin.write(chunk)
                        except (BrokenPipeError, ValueError):
                            # sox went away (consumer abandoned us, or
                            # sox crashed). Stop pumping bytes.
                            return
            except Exception as e:  # noqa: BLE001
                feeder_error[0] = e
            finally:
                try:
                    if sox.stdin is not None:
                        sox.stdin.close()
                except Exception:
                    pass

        feeder = threading.Thread(
            target=feed_sox, name="openai-tts-feed", daemon=True
        )
        feeder.start()

        total_bytes = 0
        try:
            assert sox.stdout is not None
            while True:
                chunk = sox.stdout.read(self._CHUNK_BYTES)
                if not chunk:
                    break
                total_bytes += len(chunk)
                yield chunk

            feeder.join(timeout=10)
            try:
                sox.wait(timeout=10)
            except subprocess.TimeoutExpired:
                log.warning("sox didn't exit cleanly after OpenAI TTS stream end")

            if feeder_error[0] is not None:
                log.warning("OpenAI TTS feeder error: %s", feeder_error[0])
            if sox.returncode and sox.returncode != 0 and sox.stderr is not None:
                err = sox.stderr.read().decode("utf-8", errors="replace")[:200]
                log.warning("sox returned %d: %s", sox.returncode, err)

            elapsed = time.monotonic() - t0
            log.info("OpenAI TTS+sox stream: %.2fs, %d bytes PCM (%.1fs audio)",
                     elapsed, total_bytes,
                     total_bytes / (PCM_SAMPLE_RATE * 2))
        finally:
            # On GeneratorExit / exception, reap sox and let the feeder
            # thread notice the broken pipe and exit on its own.
            if sox.poll() is None:
                sox.kill()
                try:
                    sox.communicate(timeout=5)
                except (subprocess.TimeoutExpired, ValueError):
                    pass


# ─── pipeline orchestrator ─────────────────────────────────────────────────

@dataclass
class Pipeline:
    stt: STT
    llm: LLM
    tts: TTS
    # Original config dict. Carried so runtime voice commands can
    # rebuild the LLM / TTS with the same surrounding options
    # (api_key, system_prompt, instructions, etc.) but a different
    # kind/voice. Optional so existing callers that don't pass cfg
    # still work — they just lose the switch capability.
    cfg: Optional[dict] = None

    def switch_llm(self, kind: str) -> str:
        """Voice-command handler: swap the active LLM to the named
        provider, reusing the surrounding config (system prompt,
        flags). Returns a canned reply string for TTS."""
        kind = (kind or "").lower()
        if kind not in LLM_PROVIDERS:
            return f"I don't know an LLM named {kind}."
        if self.cfg is None:
            return "LLM switching isn't available — bridge wasn't built with a config."
        try:
            self.llm = _build_llm(self.cfg, kind_override=kind)
        except Exception:
            log.exception("LLM switch to %s failed", kind)
            return f"Couldn't switch to {kind}; see the bridge log."
        log.info("Pipeline LLM switched to %s", kind)
        return f"Switched to {kind}."

    def switch_voice(self, voice: str) -> str:
        """Voice-command handler: swap the OpenAI TTS voice (and the
        matching model). Returns a canned reply for TTS."""
        voice = (voice or "").lower()
        model = _TTS_VOICE_TO_MODEL.get(voice)
        if not model:
            return f"I don't recognize the voice {voice}."
        if self.cfg is None:
            return "Voice switching isn't available — bridge wasn't built with a config."
        try:
            self.tts = _build_tts(
                self.cfg,
                kind_override="openai",
                voice_override=voice,
                model_override=model,
            )
        except Exception:
            log.exception("Voice switch to %s failed", voice)
            return f"Couldn't switch voice to {voice}; see the bridge log."
        log.info("Pipeline TTS switched to model=%s voice=%s", model, voice)
        return f"Voice switched to {voice}."

    def run(self, pcm_in: bytes, conversation_id: str = "default") -> Optional[bytes]:
        """Blocking — drains the stream into a single bytes buffer.
        Returns None on any failure or empty result."""
        chunks = list(self.run_stream(pcm_in, conversation_id))
        if not chunks:
            return None
        return b"".join(chunks)

    def run_stream(self, pcm_in: bytes, conversation_id: str = "default") -> Iterator[bytes]:
        """
        Full STT → LLM → TTS streamed end-to-end. Yields PCM chunks as
        they become available. First chunk arrives once the first LLM
        sentence has been TTS-rendered, NOT after the whole response —
        this is what lets the bridge start TX early.

        Yields nothing (empty iterator) if STT returns empty, LLM
        produces no text, or any stage raises.
        """
        if not pcm_in:
            return

        try:
            text_in = self.stt.transcribe(pcm_in)
        except Exception:
            log.exception("STT failed")
            return
        log.info("STT → %r", text_in)
        if not text_in.strip():
            return

        # Voice commands intercept before the LLM gets the text.
        cmd_reply = _handle_voice_command(text_in, self, conversation_id)
        if cmd_reply is not None:
            log.info("LLM → %r (voice command)", cmd_reply)
            try:
                yield from self.tts.synthesize_stream(cmd_reply)
            except Exception:
                log.exception("TTS failed for voice command")
            return

        # Stream LLM sentences → TTS chunks → caller.
        try:
            for sentence in self.llm.respond_stream(text_in, conversation_id=conversation_id):
                log.info("LLM → %r", sentence)
                try:
                    yield from self.tts.synthesize_stream(sentence)
                except Exception:
                    log.exception("TTS failed for sentence; skipping and continuing")
                    continue
        except Exception:
            log.exception("LLM stream failed")


def _section(cfg: dict, name: str) -> dict:
    """Return cfg[name] coerced to a dict (handles None from all-
    commented YAML sections)."""
    return cfg.get(name) or {}


def _build_stt(cfg: dict) -> STT:
    stt_cfg = _section(cfg, "stt")
    if stt_cfg.get("kind", "whisper") == "mock":
        return MockSTT()
    return WhisperCppSTT(
        binary=stt_cfg.get("binary", "/opt/whisper.cpp/build/bin/whisper-cli"),
        model_path=stt_cfg.get("model_path",
                               "/opt/whisper.cpp/models/ggml-base.en.bin"),
        threads=int(stt_cfg.get("threads", 4)),
    )


def _build_llm(cfg: dict, kind_override: Optional[str] = None) -> LLM:
    """Build the LLM described by cfg['llm']. Pass kind_override to
    swap the provider without touching the rest of the config — used
    by Pipeline.switch_llm."""
    llm_cfg = _section(cfg, "llm")
    kind = (kind_override or llm_cfg.get("kind", "claude")).lower()
    if kind == "mock":
        return MockLLM(system_prompt=llm_cfg.get("system_prompt", ""))
    if kind == "grok":
        return GrokLLM(
            api_key=llm_cfg.get("api_key") or os.environ.get("XAI_API_KEY", ""),
            model=llm_cfg.get("model", "grok-4.3"),
            system_prompt=llm_cfg.get("system_prompt", GrokLLM.system_prompt),
            max_tokens=int(llm_cfg.get("max_tokens", 200)),
            web_search=bool(llm_cfg.get("web_search", True)),
            include_unit_id=bool(llm_cfg.get("include_unit_id", False)),
            radioid_lookup=bool(llm_cfg.get("radioid_lookup", True)),
            callsign_lookup=bool(llm_cfg.get("callsign_lookup", True)),
            max_tool_rounds=int(llm_cfg.get("max_tool_rounds", 4)),
            base_url=llm_cfg.get("base_url", "https://api.x.ai/v1"),
        )
    # Default: Claude
    return ClaudeLLM(
        api_key=llm_cfg.get("api_key") or os.environ.get("ANTHROPIC_API_KEY", ""),
        model=llm_cfg.get("model", "claude-haiku-4-5-20251001"),
        system_prompt=llm_cfg.get("system_prompt", ClaudeLLM.system_prompt),
        max_tokens=int(llm_cfg.get("max_tokens", 200)),
        web_search=bool(llm_cfg.get("web_search", True)),
        web_search_max_uses=int(llm_cfg.get("web_search_max_uses", 3)),
        include_unit_id=bool(llm_cfg.get("include_unit_id", False)),
        radioid_lookup=bool(llm_cfg.get("radioid_lookup", True)),
        callsign_lookup=bool(llm_cfg.get("callsign_lookup", True)),
        max_tool_rounds=int(llm_cfg.get("max_tool_rounds", 4)),
    )


def _build_tts(cfg: dict,
               kind_override: Optional[str] = None,
               model_override: Optional[str] = None,
               voice_override: Optional[str] = None) -> TTS:
    """Build the TTS described by cfg['tts']. Overrides let
    Pipeline.switch_voice swap voice + matching model without
    discarding the rest of the OpenAI config (key, instructions,
    preamble, base_url)."""
    tts_cfg = _section(cfg, "tts")
    tts_kind = (kind_override or tts_cfg.get("kind", "piper")).lower()
    if tts_kind == "mock":
        return MockTTS()
    if tts_kind == "openai":
        return OpenAITTS(
            api_key=tts_cfg.get("api_key") or os.environ.get("OPENAI_API_KEY", ""),
            model=model_override or tts_cfg.get("model", "gpt-4o-mini-tts"),
            voice=voice_override or tts_cfg.get("voice", "ash"),
            instructions=tts_cfg.get("instructions", OpenAITTS.instructions),
            preamble_ms=int(tts_cfg.get("preamble_ms", 150)),
            base_url=tts_cfg.get("base_url", "https://api.openai.com/v1"),
        )
    # Default: Piper
    return PiperTTS(
        binary=tts_cfg.get("binary", "piper"),
        voice_path=tts_cfg.get("voice_path",
                               "/opt/piper/voices/en_US-ryan-medium.onnx"),
        voice_rate=int(tts_cfg.get("voice_rate", 22050)),
    )


def build_pipeline(cfg: dict) -> Pipeline:
    """Construct the pipeline from a config dict. The dict is carried
    on the Pipeline so runtime voice commands can rebuild the LLM or
    TTS without losing the surrounding config."""
    return Pipeline(
        stt=_build_stt(cfg),
        llm=_build_llm(cfg),
        tts=_build_tts(cfg),
        cfg=cfg,
    )
