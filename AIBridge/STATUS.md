# AIBridge — current status

## Working

- P25 wire-protocol parser/emitter for LDU1/LDU2/end records (`p25_protocol.py`)
- UDP gateway daemon with IDLE/RX/PROCESSING/TX state machine and half-duplex
  collision handling (`bridge.py`) — verified end-to-end on real RF
- Pluggable STT/LLM/TTS pipeline with both real and mock backends (`pipeline.py`)
- Real-mode integrations: whisper.cpp (STT), Claude API (LLM), Piper+sox (TTS)
- Mock mode end-to-end so the gateway + state machine can be exercised without
  the codec or model deps installed
- **IMBE decode** via mbelib + C++ wrapper (`imbe_native/`) — radio audio
  reaches Whisper as real PCM. Set `AIBRIDGE_CODEC=mbelib` in the service
  unit (default in `aibridge.service`).
- Installer (`install.sh`), systemd unit, MMDVMHost config doc

## TODO before first real test on radio

- [ ] **IMBE encode** — `MbelibCodec.encode` currently returns 11 zero bytes
      (no-op), so the bot's TX still arrives at the radio as silence/hum.
      mbelib has no encoder; need to wire OP25's `imbe_vocoder` (full C++
      encoder + decoder) and the b0-b7 ↔ 88-bit packing per TIA-102.BABA.
      Or swap in a DVSI AMBE-3000 USB dongle.
- [ ] **MMDVMHost peer auto-detection edge case** — current code uses the
      source address of whatever sent the first packet. If MMDVMHost is
      configured with separate TX/RX ports, this will TX back to the wrong
      port. Workaround: set explicit `mmdvm.host` / `mmdvm.port` in config.
- [ ] **TX framing details** — first-packet voice header (HDU) and proper
      LDU1/LDU2 alternation cadence. `bridge.py` currently alternates LDU1/LDU2
      on each 9-frame chunk; correct under the protocol but the very first
      packet of a transmission should be the LDU1 header. Verify against a
      MMDVMHost RX-side trace.
- [ ] **Inter-packet pacing** — 20 ms sleep per IMBE frame in `_transmit_pcm`
      paces the stream at real-time. Test whether MMDVMHost's modem prefers
      bursts (no sleep) or paced (current). May vary by modem.
- [ ] **Pipeline-busy filler** — emit a short "working" tone or beep on
      PTT-release before the real response, so the user knows the bot heard.

## TODO for production

- [ ] Multi-turn conversation memory in `ClaudeLLM` (currently single-shot)
- [ ] Push-to-talk word ("OK Claude") gate so the bot doesn't reply to every
      transmission on the channel
- [ ] Voice activity detection on the input so we drop empty / unintelligible
      RX without spending an API call on it
- [ ] User-ID-based access control (only respond if src_id in allowlist)
- [ ] Better error voice prompts ("network is down", "I didn't catch that")
      instead of silent drops
