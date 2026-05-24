#!/usr/bin/env bash
# AIBridge installer for Raspberry Pi (Raspberry Pi OS, Debian-based).
#
# Run from the AIBridge/ directory of the cloned repo:
#   sudo ./install.sh
#
# Idempotent — safe to re-run after `git pull`.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR=/opt/aibridge
CONFIG_DIR=/etc/aibridge

# Detect the user who invoked sudo (pi-star on WPSD, pi on plain Raspberry
# Pi OS, etc.). Falls back to "root" if the script wasn't run with sudo.
RUN_USER="${SUDO_USER:-root}"
RUN_GROUP="$(id -gn "$RUN_USER" 2>/dev/null || echo "$RUN_USER")"
echo "Installing for user: $RUN_USER (group: $RUN_GROUP)"

# ─── apt packages ──────────────────────────────────────────────────────────
echo "[1/7] Installing apt packages"
apt-get update -qq
apt-get install -y --no-install-recommends \
    python3 python3-pip python3-yaml \
    sox build-essential cmake git

# ─── python packages ───────────────────────────────────────────────────────
echo "[2/7] Installing Python packages"
pip3 install --break-system-packages -r "$REPO_DIR/requirements.txt"

# ─── whisper.cpp ───────────────────────────────────────────────────────────
echo "[3/7] Building whisper.cpp + downloading small.en model"
if [ ! -d /opt/whisper.cpp ]; then
    git clone --depth 1 https://github.com/ggml-org/whisper.cpp.git /opt/whisper.cpp
fi
( cd /opt/whisper.cpp
  # whisper.cpp uses cmake now (the old `make main` target was removed).
  # The binary lands at build/bin/whisper-cli.
  #
  # On 32-bit ARM (armhf — including Pi-Star/WPSD userland even on a 64-bit
  # kernel), 64-bit atomic ops (__atomic_*_8) are not inlined and require
  # libatomic at link time. Upstream's CMakeLists doesn't request it on
  # this platform, and CMAKE_*_LINKER_FLAGS won't help here because GNU ld
  # needs -latomic AFTER the object files (CMake's linker-flag vars prepend).
  # Patch ggml's own CMakeLists to link atomic via target_link_libraries,
  # which places it in the right position in the link command.
  GGML_CMAKE=ggml/src/CMakeLists.txt
  if [ -f "$GGML_CMAKE" ] && ! grep -q 'target_link_libraries(ggml-base.*atomic)' "$GGML_CMAKE"; then
      cat >> "$GGML_CMAKE" <<'PATCH'

# AIBridge patch: ensure libatomic linkage on 32-bit ARM where 64-bit
# atomic ops are not inlined by GCC. Harmless on other platforms.
if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    target_link_libraries(ggml-base PUBLIC atomic)
endif()
PATCH
      echo "Patched $GGML_CMAKE to link libatomic on 32-bit targets"
  fi

  if [ ! -x build/bin/whisper-cli ]; then
      rm -rf build  # clean any prior failed configuration
      cmake -B build -DCMAKE_BUILD_TYPE=Release
      cmake --build build -j"$(nproc)" --config Release
  fi
  # base.en is the latency sweet spot on a Pi: ~3-4x faster than small.en
  # for ~5% WER tradeoff on clean speech. small.en is still a good upgrade
  # if a Pi 5 has headroom, so we leave the download for it commented in.
  if [ ! -f models/ggml-base.en.bin ]; then
      bash models/download-ggml-model.sh base.en
  fi
  # bash models/download-ggml-model.sh small.en   # uncomment to also fetch small
)

# ─── mbelib + IMBE codec wrapper ───────────────────────────────────────────
echo "[4/7] Building mbelib + AIBridge IMBE codec wrapper"
if [ ! -d /opt/mbelib ]; then
    git clone --depth 1 https://github.com/szechyjs/mbelib.git /opt/mbelib
fi
if [ ! -f /usr/local/lib/libmbe.so ]; then
    ( cd /opt/mbelib
      mkdir -p build && cd build
      cmake -DCMAKE_BUILD_TYPE=Release ..
      make -j"$(nproc)"
      make install
    )
    ldconfig
fi

# OP25's imbe_vocoder gives us the encoder (mbelib is decode-only).
# We clone OP25 just to copy out its imbe_vocoder subdirectory, then
# build it as a standalone static library that our wrapper links against.
if [ ! -d /opt/op25 ]; then
    git clone --depth 1 https://github.com/osmocom/op25.git /opt/op25
fi
IMBE_VOC_SRC=/opt/imbe_vocoder
if [ ! -d "$IMBE_VOC_SRC" ]; then
    cp -r /opt/op25/op25/gr-op25_repeater/lib/imbe_vocoder "$IMBE_VOC_SRC"
    # Write a minimal CMakeLists for standalone static-library build.
    cat > "$IMBE_VOC_SRC/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.10)
project(imbe_vocoder CXX)
set(CMAKE_CXX_STANDARD 11)
file(GLOB SOURCES "*.cc")
add_library(imbe_vocoder STATIC ${SOURCES})
target_include_directories(imbe_vocoder PUBLIC .)
set_property(TARGET imbe_vocoder PROPERTY POSITION_INDEPENDENT_CODE ON)
EOF
fi
if [ ! -f "$IMBE_VOC_SRC/build/libimbe_vocoder.a" ]; then
    cmake -B "$IMBE_VOC_SRC/build" -S "$IMBE_VOC_SRC" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$IMBE_VOC_SRC/build" -j"$(nproc)"
fi

# Copy mbelib's IMBE bit-layout tables into our wrapper dir so the
# encode side can mirror exactly what mbelib's decoder expects. We sed in
# 'static' on the const declarations to avoid multiple-definition issues
# if libmbe.so's TUs ever export the same symbols.
MBELIB_CONST_SRC=/opt/mbelib/imbe7200x4400_const.h
MBELIB_CONST_DEST="$REPO_DIR/imbe_native/mbelib_imbe_const.h"
if [ -f "$MBELIB_CONST_SRC" ]; then
    sed -e 's/^const float \(quantstep\|standdev\|B2\|ba\)/static const float \1/' \
        -e 's/^const int \(bo\|hoba\|ImbeJi\)/static const int \1/' \
        "$MBELIB_CONST_SRC" > "$MBELIB_CONST_DEST"
fi

( cd "$REPO_DIR/imbe_native"
  make clean
  make IMBE_VOC_DIR="$IMBE_VOC_SRC"
)

# ─── piper ─────────────────────────────────────────────────────────────────
echo "[5/7] Installing Piper TTS + en_US-amy-medium voice"
# Test by actually executing piper rather than just checking PATH. On
# Pi-Star/WPSD the kernel is aarch64 but the userland is armhf — if a
# previous install dropped the wrong arch's binary, "command -v piper"
# would succeed but exec'ing it returns ENOENT (the 64-bit dynamic
# linker isn't present in the armhf filesystem).
if ! piper --help >/dev/null 2>&1; then
    PIPER_VER=2023.11.14-2
    # Use the userland architecture (dpkg) rather than the kernel arch.
    USERLAND_ARCH="$(dpkg --print-architecture 2>/dev/null || uname -m)"
    case "$USERLAND_ARCH" in
        armhf|armv7l)   PIPER_TGZ="piper_linux_armv7l.tar.gz" ;;
        arm64|aarch64)  PIPER_TGZ="piper_linux_aarch64.tar.gz" ;;
        amd64|x86_64)   PIPER_TGZ="piper_linux_x86_64.tar.gz" ;;
        *) echo "Unknown userland arch: $USERLAND_ARCH"; exit 1 ;;
    esac
    echo "  → using Piper build: $PIPER_TGZ (userland $USERLAND_ARCH)"
    # Wipe any prior install so a wrong-arch leftover gets replaced.
    rm -rf /opt/piper /usr/local/bin/piper
    mkdir -p /opt/piper
    cd /tmp
    curl -fL -o "$PIPER_TGZ" \
        "https://github.com/rhasspy/piper/releases/download/${PIPER_VER}/${PIPER_TGZ}"
    tar -xzf "$PIPER_TGZ" -C /opt/piper --strip-components=1
    ln -sf /opt/piper/piper /usr/local/bin/piper
fi
mkdir -p /opt/piper/voices
# Default voice: lessac-medium. Slightly more natural prosody than amy
# at the same render cost. To swap, edit /etc/aibridge/config.yaml
# tts.voice_path; other recommended voices to try:
#   en_US-libritts_r-medium  (smoother/expressive)
#   en_US-ryan-high          (deeper male, higher quality, ~2x slower)
#   en_GB-jenny_dioco-medium (British, very pleasant)
if [ ! -f /opt/piper/voices/en_US-lessac-medium.onnx ]; then
    curl -fL -o /opt/piper/voices/en_US-lessac-medium.onnx \
        https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
    curl -fL -o /opt/piper/voices/en_US-lessac-medium.onnx.json \
        https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
fi

# ─── install the bridge itself ─────────────────────────────────────────────
echo "[6/7] Installing AIBridge under $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"
cp -f "$REPO_DIR"/*.py "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/bridge.py"
# Copy the freshly-built IMBE codec wrapper too.
cp -f "$REPO_DIR/imbe_native/libaibridge_imbe.so" "$INSTALL_DIR/"

mkdir -p "$CONFIG_DIR"
if [ ! -f "$CONFIG_DIR/config.yaml" ]; then
    cp "$REPO_DIR/config.example.yaml" "$CONFIG_DIR/config.yaml"
    echo "  → wrote default $CONFIG_DIR/config.yaml — edit it (API key, IDs, etc.)"
fi
# Always (re-)set ownership and mode so the systemd service running as
# RUN_USER can read the file. 600 keeps the API key from other users.
chown "$RUN_USER:$RUN_GROUP" "$CONFIG_DIR/config.yaml"
chmod 600 "$CONFIG_DIR/config.yaml"

# ─── systemd unit ──────────────────────────────────────────────────────────
echo "[7/7] Installing systemd unit"
sed -e "s|^User=.*|User=$RUN_USER|" \
    -e "s|^Group=.*|Group=$RUN_GROUP|" \
    "$REPO_DIR/aibridge.service" > /etc/systemd/system/aibridge.service
systemctl daemon-reload
echo
echo "Done."
echo
echo "Next steps:"
echo "  1. Edit $CONFIG_DIR/config.yaml — set ANTHROPIC_API_KEY, src_id, dst_id"
echo "  2. Edit MMDVMHost ini — point [P25 Network] at 127.0.0.1:42020"
echo "     (see AIBridge/MMDVMHOST_CONFIG.md)"
echo "  3. systemctl enable --now aibridge"
echo "  4. journalctl -u aibridge -f       # watch it"
