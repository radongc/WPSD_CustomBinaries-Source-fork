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

        # Optional transparent passthrough to a regular P25 gateway
        # (P25Gateway, etc.) for traffic that isn't on the bot's TG.
        # Every packet MMDVMHost sends us is mirrored to upstream, and
        # replies from upstream are forwarded to MMDVMHost — so non-bot
        # talkgroups behave exactly as if MMDVMHost were pointed at the
        # gateway directly. The bot only processes calls whose dst_id
        # matches bot.dst_id; everything else passes through.
        upstream_cfg = section("upstream")
        self.upstream_addr: Optional[tuple[str, int]] = None
        self.upstream_sock: Optional[socket.socket] = None
        if upstream_cfg.get("host") and upstream_cfg.get("port"):
            self.upstream_addr = (
                upstream_cfg["host"], int(upstream_cfg["port"])
            )
            self.upstream_sock = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM
            )
            self.upstream_sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
            )
            # Ephemeral local port — upstream gateway replies to the
            # source address it sees, which is whatever this socket
            # ends up bound to.
            self.upstream_sock.bind((self.listen_addr, 0))
            log.info(
                "Passthrough enabled: forwarding non-bot traffic to %s:%d",
                *self.upstream_addr,
            )
        else:
            log.info(
                "Passthrough disabled (no upstream configured); "
                "bridge intercepts every TG"
            )

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

        # Transparent passthrough: every MMDVMHost packet is mirrored to
        # the upstream P25 gateway (if configured), regardless of TG. The
        # bot still inspects them below and only claims calls whose
        # dst_id matches bot.dst_id — non-bot calls just pass through
        # untouched, exactly as if the bridge weren't here.
        if self.upstream_sock and self.upstream_addr:
            try:
                self.upstream_sock.sendto(pkt, self.upstream_addr)
            except OSError as e:
                log.debug("upstream forward failed: %s", e)

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
        # Bot only acts on calls addressed to its configured TG. Anything
        # else has already been forwarded to upstream by _handle_packet,
        # so we just drop the buffered frames here.
        if buf.meta.dst_id is not None and buf.meta.dst_id != self.bot_dst_id:
            log.info("RX end: %d frames (%.2fs) on dst=%s — not bot's TG %d, "
                     "passthrough only",
                     len(buf.imbe_frames), buf.duration_s(),
                     buf.meta.dst_id, self.bot_dst_id)
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

    # Build a cushion of audio before keying up. Once TX starts we consume
    # PCM at exactly real-time, but Piper produces in bursts with brief
    # gaps between sentences (subprocess warmup). If the buffer goes
    # empty mid-call we have to send silence LDUs, and some modems / radios
    # treat our synthesised silence frames as a loss-of-signal and drop
    # the carrier briefly. ~1.5 sec of headroom is enough to absorb a
    # typical inter-sentence gap on a Pi.
    _PREBUFFER_TARGET_BYTES = 24000   # ~1.5 sec of 8 kHz / 16-bit PCM
    # How long we'll wait for the FIRST PCM byte from the pipeline.
    # STT + a slow LLM round-trip can comfortably consume 15-20 sec on
    # a Pi; this needs to cover that worst case or short replies get
    # silently dropped. Only fires if the whole pipeline genuinely
    # never produces anything.
    _FIRST_BYTE_TIMEOUT_S = 60.0
    # Once the producer is alive and yielding, how long we'll keep
    # waiting to top the prebuffer up to TARGET. Short — past the
    # first byte the producer is steady-state and bytes arrive fast.
    _PREBUFFER_FILL_S = 3.0

    def _transmit_pcm_stream(self, chunks_iter) -> None:
        """
        Stream PCM out as P25 LDU1/LDU2 packets to MMDVMHost, consuming
        from a PCM-chunk iterator (pipeline.run_stream).

        A producer thread drains the iterator into a queue so LLM/TTS
        latency can't stall TX pacing. We prebuffer ~1.5 sec of audio
        before keying up so brief inter-sentence Piper gaps don't drain
        the queue mid-call. If the queue does underrun mid-stream we
        send a silence LDU to keep the carrier up.
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
            # ─── Prebuffer ──────────────────────────────────────────────
            # Two phases. Phase 1 waits generously for the FIRST PCM byte
            # (STT + a slow LLM round-trip can take many seconds). Once
            # we have any audio at all we switch to Phase 2: a short
            # deadline to top the buffer up to TARGET. This avoids the
            # silent-drop failure mode where a slow LLM call eats a
            # single combined deadline before any audio is produced.
            first_byte_deadline = time.monotonic() + self._FIRST_BYTE_TIMEOUT_S
            while not buf and not producer_done:
                if self.tx_abort.is_set():
                    return
                remaining = first_byte_deadline - time.monotonic()
                if remaining <= 0:
                    log.warning("TX: no PCM after %.0fs; giving up",
                                self._FIRST_BYTE_TIMEOUT_S)
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
                # Producer finished without ever producing anything.
                log.info("TX stream: pipeline produced no PCM; nothing to send")
                return

            # Phase 2: fill toward TARGET on the shorter deadline.
            fill_deadline = time.monotonic() + self._PREBUFFER_FILL_S
            while (len(buf) < self._PREBUFFER_TARGET_BYTES
                   and not producer_done):
                if self.tx_abort.is_set():
                    return
                remaining = fill_deadline - time.monotonic()
                if remaining <= 0:
                    log.info("TX prebuffer fill deadline; starting with %d/%d bytes",
                             len(buf), self._PREBUFFER_TARGET_BYTES)
                    break
                try:
                    item = pcm_q.get(timeout=min(remaining, 0.5))
                except queue.Empty:
                    continue
                if item is producer_done_sentinel:
                    producer_done = True
                    break
                buf.extend(item)

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
            log.info("TX stream start (prebuffered %d bytes / %.2fs)",
                     len(buf), len(buf) / 16000.0)
            t0 = time.monotonic()
            # Absolute send schedule. We compute when each packet *should*
            # leave (t0 + N * 20 ms) and sleep until that wall time, instead
            # of sleeping 20 ms per packet. Per-packet sleeps over-run by
            # 1-2 ms each on Linux, plus the LDU build between iterations
            # adds another few ms; over 30+ LDUs that drift adds up to
            # several hundred ms, the modem's TX FIFO drains, and the
            # carrier blips while the modem waits for the next packet.
            # Absolute scheduling self-corrects: if we fall behind, the
            # next packet ships immediately.
            packets_sent = 0
            toggle_ldu1 = True
            total_frames = 0
            silence_ldus = 0   # diagnostic — mid-stream underrun count

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
                    silence_ldus += 1

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
                    packets_sent += 1
                    # Sleep until the next packet's absolute deadline,
                    # not for a fixed duration. If we already missed it
                    # (LDU build took too long, last sleep over-ran)
                    # the next packet ships immediately.
                    deadline = t0 + packets_sent * INTER_PACKET_DELAY_S
                    slack = deadline - time.monotonic()
                    if slack > 0:
                        time.sleep(slack)

            self.sock.sendto(p25.build_end_record(), self.mmdvm_addr)
            elapsed = time.monotonic() - t0
            log.info("TX stream end: %d frames (%.2fs audio, %.2fs wall), "
                     "%d silence LDUs from underruns",
                     total_frames, total_frames * 0.020, elapsed, silence_ldus)
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

    # ─── upstream passthrough ─────────────────────────────────────────────

    def upstream_loop(self) -> None:
        """Forward packets coming back from the upstream P25 gateway out
        to MMDVMHost. Stateless byte pipe — we never inspect or alter
        the packet, we just shovel it in the other direction."""
        if self.upstream_sock is None:
            return
        while True:
            self.upstream_sock.settimeout(0.5)
            try:
                data, _addr = self.upstream_sock.recvfrom(2048)
            except socket.timeout:
                continue
            if self.mmdvm_addr is None:
                # No MMDVMHost peer learned yet; drop. Once any RX has
                # arrived we'll know where to send.
                continue
            try:
                self.sock.sendto(data, self.mmdvm_addr)
            except OSError as e:
                log.debug("upstream→mmdvm forward failed: %s", e)

    # ─── lifecycle ────────────────────────────────────────────────────────

    def run(self) -> None:
        rx = threading.Thread(target=self.rx_loop, name="rx", daemon=True)
        worker = threading.Thread(target=self.worker_loop, name="worker", daemon=True)
        rx.start()
        worker.start()
        if self.upstream_sock is not None:
            up = threading.Thread(
                target=self.upstream_loop, name="upstream", daemon=True
            )
            up.start()
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
