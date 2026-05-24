#!/usr/bin/env python3
"""
AIBridge main daemon.

A "fake P25 gateway" that speaks MMDVMHost's wire protocol over UDP.
Point MMDVMHost's [P25 Network] section at this daemon and it will:

  1. Buffer incoming IMBE voice frames from the radio (via MMDVMHost)
     until the end-of-transmission marker arrives.
  2. Decode IMBE → PCM and run the STT/LLM/TTS pipeline.
  3. Re-encode the response PCM as IMBE and stream it back to MMDVMHost
     as a fresh P25 transmission.

Half-duplex collisions: if new RX traffic arrives while we're processing
or transmitting, we abort and yield the channel.

Usage:
    python3 bridge.py /etc/aibridge/config.yaml

Or via systemd (see aibridge.service).
"""

from __future__ import annotations
import argparse
import logging
import os
import queue
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

# Make the AIBridge package importable when run as a script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import p25_protocol as p25  # noqa: E402
import imbe_codec  # noqa: E402
import pipeline as pl  # noqa: E402


log = logging.getLogger("aibridge")


# Hard-coded for now; everything else comes from config.
INTER_PACKET_DELAY_S = 0.020  # 20 ms per IMBE frame at 8 kHz


class State(Enum):
    IDLE = "idle"
    RX = "rx"
    PROCESSING = "processing"
    TX = "tx"


@dataclass
class RxBuffer:
    """One in-progress receive transmission."""
    imbe_frames: list[bytes] = field(default_factory=list)
    meta: p25.TransmissionMeta = field(default_factory=p25.TransmissionMeta)
    started: float = 0.0
    last_packet: float = 0.0

    def add_frame(self, frame: bytes) -> None:
        self.imbe_frames.append(frame)
        self.last_packet = time.monotonic()

    def duration_s(self) -> float:
        # 20 ms per IMBE frame.
        return len(self.imbe_frames) * 0.020


class Bridge:
    """Owns the UDP socket, state machine, and worker threads."""

    def __init__(self, cfg: dict) -> None:
        # YAML quirk: a section with all-commented keys parses as None, not
        # {}, so cfg.get("foo", {}) returns None and breaks `in` / .get on
        # it. Coerce every section to a dict.
        def section(name: str) -> dict:
            return cfg.get(name) or {}

        self.cfg = cfg
        listen_cfg = section("listen")
        self.listen_addr = listen_cfg.get("host", "0.0.0.0")
        self.listen_port = int(listen_cfg.get("port", 42020))
        self.mmdvm_addr: Optional[tuple[str, int]] = None  # filled on first packet

        # If MMDVMHost's RX side and TX side differ, allow explicit override.
        mmdvm_cfg = section("mmdvm")
        if "host" in mmdvm_cfg and "port" in mmdvm_cfg:
            self.mmdvm_addr = (mmdvm_cfg["host"], int(mmdvm_cfg["port"]))

        # Bot identity used in TX-side P25 LDU1 control fields.
        bot_cfg = section("bot")
        self.bot_src_id = int(bot_cfg.get("src_id", 1234567))   # who we appear as
        self.bot_dst_id = int(bot_cfg.get("dst_id", 1))         # default TG to talk on
        self.bot_lcf = int(bot_cfg.get("lcf", 0))
        self.bot_mfid = int(bot_cfg.get("mfid", 0))

        # State.
        self.state = State.IDLE
        self.state_lock = threading.Lock()
        self.rx_buffer: Optional[RxBuffer] = None

        # Cross-thread signalling.
        self.tx_abort = threading.Event()  # set to interrupt an in-flight TX
        self.work_q: queue.Queue[RxBuffer] = queue.Queue()

        # Codec + pipeline.
        self.codec: imbe_codec.Codec = imbe_codec.get_codec()
        self.pipeline = pl.build_pipeline(cfg)

        # Socket.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_addr, self.listen_port))
        log.info("Listening on udp://%s:%d", self.listen_addr, self.listen_port)

    # ─── RX path ──────────────────────────────────────────────────────────

    def rx_loop(self) -> None:
        """Read UDP packets from MMDVMHost. Drive the state machine."""
        # Idle-detection: if RX stalls for this long without a 0x80, force end.
        idle_timeout = 0.4  # seconds

        while True:
            self.sock.settimeout(0.1)
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                # While in RX state, check for stall.
                with self.state_lock:
                    if (self.state == State.RX
                            and self.rx_buffer is not None
                            and time.monotonic() - self.rx_buffer.last_packet > idle_timeout):
                        log.warning("RX stalled for %.1fs — forcing end of transmission",
                                    idle_timeout)
                        self._finalize_rx()
                continue

            # Remember who's talking to us so we can talk back.
            if self.mmdvm_addr is None:
                self.mmdvm_addr = addr
                log.info("MMDVMHost peer = %s:%d", addr[0], addr[1])

            self._handle_packet(data)

    def _handle_packet(self, pkt: bytes) -> None:
        """Dispatch one UDP packet from MMDVMHost into the state machine."""
        if not pkt:
            return
        marker = pkt[0]

        with self.state_lock:
            if marker == p25.END_MARKER:
                if self.state == State.RX:
                    self._finalize_rx()
                return

            if self.state in (State.PROCESSING, State.TX):
                # Caller (real user) keying up while we were busy: abort us.
                log.info("RX during %s — aborting our turn", self.state.value)
                self.tx_abort.set()
                self.state = State.RX
                self.rx_buffer = RxBuffer(started=time.monotonic(),
                                          last_packet=time.monotonic())

            if self.state == State.IDLE:
                if marker not in (0x62, 0x6B):
                    # Started mid-stream; still accept, but log.
                    log.debug("RX entry on marker 0x%02X (not LDU1/LDU2 start)", marker)
                log.info("RX start")
                self.state = State.RX
                self.rx_buffer = RxBuffer(started=time.monotonic(),
                                          last_packet=time.monotonic())

            if self.state == State.RX and self.rx_buffer is not None:
                frame = p25.parse_record(pkt, self.rx_buffer.meta)
                if frame is not None:
                    self.rx_buffer.add_frame(frame)

    def _finalize_rx(self) -> None:
        """Hand the finished RX buffer to the worker and return to IDLE."""
        buf = self.rx_buffer
        self.rx_buffer = None
        self.state = State.IDLE
        if buf is None or not buf.imbe_frames:
            log.info("RX ended with empty buffer; ignoring")
            return
        log.info("RX end: %d frames (%.2fs), src=%s dst=%s",
                 len(buf.imbe_frames), buf.duration_s(),
                 buf.meta.src_id, buf.meta.dst_id)
        self.work_q.put(buf)

    # ─── Worker path (STT/LLM/TTS) ────────────────────────────────────────

    def worker_loop(self) -> None:
        """Pop completed RX buffers, run the pipeline, then trigger TX."""
        while True:
            buf = self.work_q.get()
            with self.state_lock:
                if self.state == State.RX:
                    # A newer RX came in after we queued; skip this one.
                    log.info("Skipping pipeline run — newer RX active")
                    continue
                self.state = State.PROCESSING
                self.tx_abort.clear()

            # Use the transmitting radio's src_id as the conversation key so
            # the LLM gives each radio its own memory thread. Fall back to
            # "anon" if MMDVMHost didn't surface the source ID for some reason.
            conv_id = str(buf.meta.src_id) if buf.meta.src_id else "anon"

            try:
                pcm_in = imbe_codec.imbe_frames_to_pcm(buf.imbe_frames, self.codec)
                chunks = self.pipeline.run_stream(pcm_in, conversation_id=conv_id)
                self._transmit_pcm_stream(chunks)
            except Exception:
                log.exception("Pipeline / TX failed")

            with self.state_lock:
                if self.state in (State.PROCESSING, State.TX):
                    self.state = State.IDLE

    # ─── TX path ──────────────────────────────────────────────────────────

    # PCM framing constants. 8 kHz / 16-bit / mono → 320 bytes per 20 ms IMBE
    # frame, and P25 LDU1/LDU2 carries 9 frames each (180 ms of audio).
    _FRAME_PCM_BYTES = 320
    _LDU_FRAMES = 9
    _LDU_PCM_BYTES = _LDU_FRAMES * _FRAME_PCM_BYTES   # 2880
    # How long to wait for the very first PCM byte before giving up. TTS
    # warmup + first LLM sentence on a Pi can comfortably take 5+ sec.
    _PREBUFFER_TIMEOUT_S = 10.0

    def _transmit_pcm_stream(self, chunks_iter) -> None:
        """
        Stream PCM out as P25 LDU1/LDU2 packets to MMDVMHost, consuming
        from a PCM-chunk iterator (pipeline.run_stream).

        A producer thread drains the iterator into a queue so LLM/TTS
        latency can't stall TX pacing. If the queue underruns mid-call
        we pad the current LDU with silence rather than letting the
        radio drop the call.

        We don't transition to State.TX (and don't key the radio) until
        the first PCM byte actually arrives — avoids leading dead air
        during the LLM/TTS warmup.
        """
        if self.mmdvm_addr is None:
            log.warning("No MMDVMHost peer known yet; can't TX")
            self._close_iter(chunks_iter)
            return

        pcm_q: queue.Queue = queue.Queue()
        producer_done_sentinel = object()
        producer_stop = threading.Event()

        def producer() -> None:
            try:
                for chunk in chunks_iter:
                    if producer_stop.is_set() or self.tx_abort.is_set():
                        break
                    if chunk:
                        pcm_q.put(chunk)
            except Exception:
                log.exception("PCM producer thread crashed")
            finally:
                pcm_q.put(producer_done_sentinel)
                # Closing the upstream generator triggers its finally
                # blocks (rolls back LLM history, kills Piper/sox).
                self._close_iter(chunks_iter)

        prod = threading.Thread(target=producer, name="pcm-producer", daemon=True)
        prod.start()

        silence_frame = b"\x00" * self._FRAME_PCM_BYTES
        buf = bytearray()
        producer_done = False

        try:
            # ─── Prebuffer: wait for first PCM before keying up. ────────
            deadline = time.monotonic() + self._PREBUFFER_TIMEOUT_S
            while not buf and not producer_done:
                if self.tx_abort.is_set():
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    log.warning("TX prebuffer timeout (%.1fs); nothing to send",
                                self._PREBUFFER_TIMEOUT_S)
                    return
                try:
                    item = pcm_q.get(timeout=min(remaining, 0.5))
                except queue.Empty:
                    continue
                if item is producer_done_sentinel:
                    producer_done = True
                    break
                buf.extend(item)

            if not buf:
                log.info("TX stream: no PCM produced; nothing to send")
                return

            # We have audio — key up.
            with self.state_lock:
                if self.tx_abort.is_set():
                    return
                self.state = State.TX

            meta = p25.TransmissionMeta(
                src_id=self.bot_src_id,
                dst_id=self.bot_dst_id,
                lcf=self.bot_lcf,
                mfid=self.bot_mfid,
                alg_id=0x80,  # CLEAR
            )
            log.info("TX stream start")
            t0 = time.monotonic()
            toggle_ldu1 = True
            total_frames = 0

            # ─── Steady-state TX loop. ─────────────────────────────────
            while True:
                if self.tx_abort.is_set():
                    log.info("TX aborted mid-stream")
                    return

                # Top buf up toward one LDU's worth of PCM.
                while len(buf) < self._LDU_PCM_BYTES and not producer_done:
                    try:
                        item = pcm_q.get(timeout=0.05)
                    except queue.Empty:
                        break  # underrun — silence-pad below
                    if item is producer_done_sentinel:
                        producer_done = True
                        break
                    buf.extend(item)

                if len(buf) >= self._LDU_PCM_BYTES:
                    # Full LDU of real audio.
                    pcm_ldu = bytes(buf[:self._LDU_PCM_BYTES])
                    del buf[:self._LDU_PCM_BYTES]
                elif producer_done:
                    # Final partial LDU (silence-padded), or genuinely
                    # nothing left — end the call.
                    if not buf:
                        break
                    pcm_ldu = bytes(buf) + b"\x00" * (self._LDU_PCM_BYTES - len(buf))
                    buf.clear()
                else:
                    # Transient underrun mid-stream (e.g. waiting for the
                    # LLM to produce the next sentence and TTS to render
                    # it). Send a full silence LDU to keep the call up;
                    # KEEP buf so the partial audio joins the next real
                    # LDU instead of being padded prematurely.
                    pcm_ldu = b"\x00" * self._LDU_PCM_BYTES

                frames = imbe_codec.pcm_to_imbe_frames(pcm_ldu, self.codec)
                # Defensive: the encoder should give us exactly 9 frames
                # for 9 frames' worth of PCM, but pad if not.
                while len(frames) < self._LDU_FRAMES:
                    frames.append(self.codec.encode(silence_frame))
                frames = frames[:self._LDU_FRAMES]

                packets = (p25.build_ldu1(frames, meta)
                           if toggle_ldu1
                           else p25.build_ldu2(frames, meta))
                toggle_ldu1 = not toggle_ldu1
                total_frames += self._LDU_FRAMES

                for pkt in packets:
                    if self.tx_abort.is_set():
                        log.info("TX aborted mid-stream")
                        return
                    self.sock.sendto(pkt, self.mmdvm_addr)
                    # Pace at roughly real-time so MMDVMHost's modem
                    # queue doesn't have to absorb the whole call in
                    # one gulp.
                    time.sleep(INTER_PACKET_DELAY_S)

            self.sock.sendto(p25.build_end_record(), self.mmdvm_addr)
            elapsed = time.monotonic() - t0
            log.info("TX stream end: %d frames (%.2fs audio, %.2fs wall)",
                     total_frames, total_frames * 0.020, elapsed)
        except Exception:
            log.exception("TX stream failed")
        finally:
            # Signal producer to stop, drain queue so it can exit, then
            # join. Producer's own finally will close the iterator.
            producer_stop.set()
            drain_deadline = time.monotonic() + 5.0
            while prod.is_alive() and time.monotonic() < drain_deadline:
                try:
                    pcm_q.get(timeout=0.2)
                except queue.Empty:
                    pass
            prod.join(timeout=5.0)

    @staticmethod
    def _close_iter(it) -> None:
        """Best-effort close of an iterator (no-op if it isn't a generator)."""
        close = getattr(it, "close", None)
        if close is None:
            return
        try:
            close()
        except Exception:
            log.debug("iterator close raised", exc_info=True)

    # ─── lifecycle ────────────────────────────────────────────────────────

    def run(self) -> None:
        rx = threading.Thread(target=self.rx_loop, name="rx", daemon=True)
        worker = threading.Thread(target=self.worker_loop, name="worker", daemon=True)
        rx.start()
        worker.start()
        log.info("AIBridge running. Ctrl-C to stop.")
        try:
            while True:
                time.sleep(60)
        except KeyboardInterrupt:
            log.info("Shutting down")


def load_config(path: str) -> dict:
    if not path or not os.path.exists(path):
        log.warning("No config at %s — running with defaults (mock everything)", path)
        return {
            "stt": {"kind": "mock"},
            "llm": {"kind": "mock"},
            "tts": {"kind": "mock"},
        }
    import yaml  # pyyaml
    with open(path) as f:
        return yaml.safe_load(f) or {}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("config", nargs="?", default="/etc/aibridge/config.yaml")
    ap.add_argument("--log-level", default=os.environ.get("AIBRIDGE_LOG", "INFO"))
    args = ap.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    cfg = load_config(args.config)
    Bridge(cfg).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
