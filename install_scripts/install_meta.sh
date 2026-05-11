#!/usr/bin/env bash
# install_meta.sh
# Sets up the .venv_teleop venv for Meta Quest Pro VR teleop on any x86_64 or
# arm64 machine (desktop, laptop, or G1 onboard).
#
# Parallel to install_pico.sh: same venv (.venv_teleop) and gear_sonic[teleop]
# install, but step 5 swaps the XRoboToolkit SDK out for quest_xr_shim, which
# receives Meta Quest tracking over JSON-over-TCP :63901.
#
# At runtime, the manager picks the backend via the XR_BACKEND env var
# (see pico_manager_thread_server.py). XR_BACKEND defaults to "quest", so no
# further config is needed after this script.
#
# Usage:  bash install_scripts/install_meta.sh   (run from repo root)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 0. Print detected architecture ───────────────────────────────────────────
ARCH="$(uname -m)"
echo "[OK] Architecture: $ARCH"

# ── 1. Ensure uv is installed and available ──────────────────────────────────
if ! command -v uv &>/dev/null; then
    echo "[INFO] uv not found – installing via official installer …"
    curl -LsSf https://astral.sh/uv/install.sh | sh

    # Source the uv env so it's available in this session
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
echo "[INFO] Removing old .venv_teleop (if present) …"
rm -rf .venv_teleop

# ── 4. Create venv & install teleop extra ─────────────────────────────────────
echo "[INFO] Creating .venv_teleop with uv-managed Python 3.10 …"
uv venv .venv_teleop --python "$MANAGED_PY" --prompt gear_sonic_teleop
# shellcheck disable=SC1091
source .venv_teleop/bin/activate
echo "[INFO] Installing gear_sonic[teleop] …"
uv pip install -e "gear_sonic[teleop]"

# ── 5. Install quest_xr_shim (Meta Quest Pro JSON-over-TCP backend) ──────────
# In-tree pure-Python package; no CMake / pybind11 build required.
echo "[INFO] Installing quest_xr_shim …"
uv pip install -e gear_sonic/utils/teleop/quest_xr_shim

# ── 6 & 7: sim extra + unitree_sdk2_python ────────────────────────────────────
# On the onboard Jetson Orin (user==unitree + aarch64) these are not needed:
#   • sim extra depends on mujoco which may lack aarch64 wheels
#   • unitree_sdk2_python requires CycloneDDS C lib (already on the robot)
# They are only installed on desktop / x86 dev machines.
if [ "$ARCH" = "aarch64" ] && [ "$(whoami)" = "unitree" ]; then
    echo "[SKIP] Skipping sim extra & unitree_sdk2_python (onboard Jetson Orin)"
else
    # ── 6. Install sim extra (for run_sim_loop.py / sim2sim testing)
    echo "[INFO] Installing sim extra …"
    uv pip install -e "gear_sonic[sim]"

    # ── 7. Install unitree_sdk2_python (needed by the sim2sim bridge)
    echo "[INFO] Installing unitree_sdk2_python …"
    uv pip install -e external_dependencies/unitree_sdk2_python
fi

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Setup complete!  Activate the venv with:"
echo ""
echo "    source .venv_teleop/bin/activate"
echo ""
echo "  You should see (gear_sonic_teleop) in your prompt."
echo ""
echo "  Then start the manager:"
echo "    python gear_sonic/scripts/pico_manager_thread_server.py \\"
echo "      --manager --vis_vr3pt --force-vr-3pt"
echo ""
echo "  The shim listens for the Quest Unity APK on tcp://0.0.0.0:63901."
echo "══════════════════════════════════════════════════════════════"
