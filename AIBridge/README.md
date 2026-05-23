# AIBridge — voice AI assistant over P25 hotspot

A Python daemon that lets you talk to an LLM through a P25 radio via a WPSD
hotspot. You key up, speak, release PTT. A few seconds later the radio comes
back with a synthesized voice reply.

## Architecture

```
   ┌────────┐  RF P25   ┌──────────┐  UDP (P25 net proto)  ┌──────────────┐
   │ Radio  │──────────▶│ MMDVMHost│──────────────────────▶│  AIBridge    │
   │ (P25)  │           │ (WPSD)   │                       │  (this repo) │
   └────────┘           └──────────┘                       └──────┬───────┘
        ▲                     ▲                                   │
        │                     │                                   ▼
        │                     │                          ┌─────────────────┐
        │                     │                          │ IMBE → PCM      │
        │                     │                          │ → STT (Whisper) │
        │                     │                          │ → LLM (Claude)  │
        │                     │                          │ → TTS (Piper)   │
        │                     │                          │ → PCM → IMBE    │
        │                     │                          └────────┬────────┘
        │                     │                                   │
        │                     └────── UDP (P25 net proto) ◀───────┘
        │
   RF P25 (TX from MMDVMHost back to your radio)
```

We impersonate the P25 reflector/gateway that MMDVMHost normally talks to.
MMDVMHost is told to send its decoded P25 traffic to `127.0.0.1:42020` (this
bridge), and to receive from us. No MMDVMHost source changes required.

The wire format is MMDVMHost's standard P25 network protocol: a series of
short UDP packets with record markers `0x62`–`0x6A` (LDU1, one packet per
IMBE voice frame plus header data) and `0x6B`–`0x73` (LDU2), terminated by
a `0x80` end-of-transmission record. Each LDU carries 9 IMBE voice frames
(11 bytes each = 88 bits of encoded voice). One LDU = ~180 ms of audio.

## State machine

```
        ┌────────────┐
        │   IDLE     │◀──────────────────────┐
        └─────┬──────┘                       │
              │ first LDU1 packet arrives    │
              ▼                              │
        ┌────────────┐                       │
        │   RX       │──── timeout / abort ──┤
        │ (buffering)│                       │
        └─────┬──────┘                       │
              │ 0x80 terminator received     │
              ▼                              │
        ┌────────────┐                       │
        │ PROCESSING │── new RX arrives ─────┤
        │ STT/LLM/TTS│   (cancel and yield)  │
        └─────┬──────┘                       │
              │ TTS PCM ready                │
              ▼                              │
        ┌────────────┐                       │
        │   TX       │── new RX arrives ─────┤
        │ (emitting) │   (abort TX, yield)   │
        └─────┬──────┘                       │
              │ all PCM sent + final 0x80    │
              └──────────────────────────────┘
```

Half-duplex collision: if a user keys up while we're processing or
transmitting, we abort immediately and yield the channel. They take
priority.

## Components

| File | What it does |
|---|---|
| `bridge.py` | Main daemon, state machine, UDP socket I/O |
| `p25_protocol.py` | Parse/emit LDU1, LDU2, and end records |
| `imbe_codec.py` | IMBE encode/decode wrapper (mbelib / py-imbe) |
| `pipeline.py` | STT → LLM → TTS orchestration |
| `config.example.yaml` | Configuration template |
| `requirements.txt` | Python deps |
| `aibridge.service` | systemd unit file |
| `install.sh` | One-shot installer for the Pi |

## Setup overview (full instructions in `install.sh`)

1. Clone this repo on your Pi (you probably already have it for WPSD).
2. Switch to the `aibot` branch.
3. Run `AIBridge/install.sh` — installs Python deps, Whisper model, Piper voice, mbelib.
4. Edit `/etc/aibridge/config.yaml`:
   - Anthropic API key
   - Whisper model size and Piper voice
   - System prompt for the bot
5. Edit MMDVMHost config so the P25 Network points at `127.0.0.1:42020` (this
   daemon). See `MMDVMHOST_CONFIG.md`.
6. `sudo systemctl enable --now aibridge`
7. Key up on your radio, say "Hello", release PTT, wait a few seconds.

## Status

**Prototype scaffolding.** Currently the protocol-handler skeleton works end-to-end
with mock STT/LLM/TTS (echo bot). Real STT/LLM/TTS wiring + IMBE codec
integration is in progress. See `STATUS.md` for the latest state.
