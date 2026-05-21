#!/usr/bin/env bash
# install_inference.sh
# Sets up the .venv_inference venv for running VLA inference with
# Isaac-GR00T PolicyClient against a remote or local policy server.
#
# Installs gear_sonic[inference] which pulls in the Isaac-GR00T library,
# PyZMQ, msgpack, Pinocchio, and other inference dependencies.
#
# Usage:  bash install_scripts/install_inference.sh   (run from repo root)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 0. System dependencies ────────────────────────────────────────────────────
ARCH="$(uname -m)"
echo "[OK] Architecture: $ARCH"

# ── 1. Ensure uv is installed and available ──────────────────────────────────
if ! command -v uv &>/dev/null; then
    echo "[INFO] uv not found – installing via official installer …"
    curl -LsSf https://astral.sh/uv/install.sh | sh

    if [ -f "$HOME/.local/bin/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.local/bin/env"
    elif [ -f "$HOME/.cargo/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.cargo/env"
    else
        export PATH="$HOME/.local/bin:$PATH"
    fi

    if ! command -v uv &>/dev/null; then
        echo "[ERROR] uv installation succeeded but binary not found on PATH."
        echo "        Please add ~/.local/bin (or ~/.cargo/bin) to your PATH and re-run."
        exit 1
    fi
fi
echo "[OK] uv $(uv --version)"

# ── 2. Install a uv-managed Python 3.10 (includes dev headers / Python.h) ────
echo "[INFO] Installing uv-managed Python 3.10 (includes development headers) …"
uv python install 3.10
MANAGED_PY="$(uv python find --no-project 3.10)"
echo "[OK] Using Python: $MANAGED_PY"

# ── 3. Clean previous venv (if any) ──────────────────────────────────────────
cd "$REPO_ROOT"
echo "[INFO] Removing old .venv_inference (if present) …"
rm -rf .venv_inference

# ── 4. Create venv & install inference extra ─────────────────────────────────
echo "[INFO] Creating .venv_inference with uv-managed Python 3.10 …"
uv venv .venv_inference --python "$MANAGED_PY" --prompt gear_sonic_inference
# shellcheck disable=SC1091
source .venv_inference/bin/activate

# ── 5. Vendor Isaac-GR00T as a local editable install ────────────────────────
# We install Isaac-GR00T from a local clone (under external_dependencies/) rather
# than as a transitive Git dependency of gear_sonic[inference]. Reason:
# upstream Isaac-GR00T's pyproject.toml uses [tool.uv.sources] to map flash-attn
# and torchcodec to local-file wheels (for aarch64). uv refuses path-to-wheel
# entries in transitive Git dependencies, even when their marker would never
# match on this platform. Installing from a local directory bypasses that check.
mkdir -p external_dependencies
if [ ! -d external_dependencies/Isaac-GR00T ]; then
    echo "[INFO] Cloning Isaac-GR00T (shallow) into external_dependencies/Isaac-GR00T …"
    git clone --depth 1 https://github.com/NVIDIA/Isaac-GR00T.git external_dependencies/Isaac-GR00T
else
    echo "[OK] external_dependencies/Isaac-GR00T already present — reusing existing clone."
fi

# Pre-install flash-attn from its prebuilt wheel on x86_64. Upstream's
# [tool.uv.sources] maps flash-attn to this exact URL, but uv pip mode ignores
# that table, so we pin the URL explicitly here. The wheel is for cu12 + torch
# 2.7 + cp310, which matches Isaac-GR00T's pinned torch==2.7.1 + Python 3.10.
# Without this, pip would fall back to building flash-attn from source — which
# requires nvcc and takes 20+ minutes.
if [ "$ARCH" = "x86_64" ]; then
    FLASH_ATTN_WHL="https://github.com/Dao-AILab/flash-attention/releases/download/v2.7.4.post1/flash_attn-2.7.4.post1+cu12torch2.7cxx11abiFALSE-cp310-cp310-linux_x86_64.whl"
    echo "[INFO] Pre-installing flash-attn prebuilt wheel (avoids 20-min source build) …"
    uv pip install "$FLASH_ATTN_WHL"
fi

echo "[INFO] Installing Isaac-GR00T (editable, from local clone) — pulls torch/transformers/etc, may take a few minutes …"
uv pip install -e ./external_dependencies/Isaac-GR00T

echo "[INFO] Installing gear_sonic[inference] …"
uv pip install -e "gear_sonic[inference]"

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Setup complete!  Activate the venv with:"
echo ""
echo "    source .venv_inference/bin/activate"
echo ""
echo "  You should see (gear_sonic_inference) in your prompt."
echo ""
echo "  Then run VLA inference with:"
echo "    python gear_sonic/scripts/run_vla_inference.py --help"
echo "══════════════════════════════════════════════════════════════"
