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
echo "[1/6] Installing apt packages"
apt-get update -qq
apt-get install -y --no-install-recommends \
    python3 python3-pip python3-yaml \
    sox build-essential cmake git
# NOTE: mbelib is not in standard Debian/RPi repos. When we wire the real
# IMBE codec, we'll add a "build mbelib from source" step here. Until then
# the mock codec needs no native deps.

# ─── python packages ───────────────────────────────────────────────────────
echo "[2/6] Installing Python packages"
pip3 install --break-system-packages -r "$REPO_DIR/requirements.txt"

# ─── whisper.cpp ───────────────────────────────────────────────────────────
echo "[3/6] Building whisper.cpp + downloading small.en model"
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
  if [ ! -f models/ggml-small.en.bin ]; then
      bash models/download-ggml-model.sh small.en
  fi
)

# ─── piper ─────────────────────────────────────────────────────────────────
echo "[4/6] Installing Piper TTS + en_US-amy-medium voice"
if ! command -v piper >/dev/null 2>&1; then
    PIPER_VER=2023.11.14-2
    PIPER_TGZ="piper_linux_aarch64.tar.gz"
    if [ "$(uname -m)" = "armv7l" ]; then
        PIPER_TGZ="piper_linux_armv7l.tar.gz"
    fi
    mkdir -p /opt/piper
    cd /tmp
    curl -fL -o "$PIPER_TGZ" \
        "https://github.com/rhasspy/piper/releases/download/${PIPER_VER}/${PIPER_TGZ}"
    tar -xzf "$PIPER_TGZ" -C /opt/piper --strip-components=1
    ln -sf /opt/piper/piper /usr/local/bin/piper
fi
mkdir -p /opt/piper/voices
if [ ! -f /opt/piper/voices/en_US-amy-medium.onnx ]; then
    curl -fL -o /opt/piper/voices/en_US-amy-medium.onnx \
        https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx
    curl -fL -o /opt/piper/voices/en_US-amy-medium.onnx.json \
        https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx.json
fi

# ─── install the bridge itself ─────────────────────────────────────────────
echo "[5/6] Installing AIBridge under $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"
cp -f "$REPO_DIR"/*.py "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/bridge.py"

mkdir -p "$CONFIG_DIR"
if [ ! -f "$CONFIG_DIR/config.yaml" ]; then
    cp "$REPO_DIR/config.example.yaml" "$CONFIG_DIR/config.yaml"
    chmod 600 "$CONFIG_DIR/config.yaml"
    echo "  → wrote default $CONFIG_DIR/config.yaml — edit it (API key, IDs, etc.)"
fi

# ─── systemd unit ──────────────────────────────────────────────────────────
echo "[6/6] Installing systemd unit"
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
