# quest_xr_shim

Drop-in replacement for `xrobotoolkit_sdk` that lets you drive SONIC + N1.7
data collection with a **Meta Quest Pro + 2 controllers** instead of the
official **PICO 4 + foot-trackers** rig.

The shim exposes the exact 17 `xrt.*` symbols
[`pico_manager_thread_server.py`](../../../scripts/pico_manager_thread_server.py)
calls, but its `init()` starts a TCP/JSON listener on **port 63901** that
receives frames from a Quest Unity APK. Downstream code (`run_data_exporter`,
LeRobot parquet writers, C++ deploy) is unchanged.

For the full design rationale — including why this is not the same as
`xr_teleoperate` and why we cannot use POSE / FROZEN modes — see
[quest_pro_shim_设计.md](../../../../quest_pro_shim_设计.md) at the repo root.

## Architecture

```
Meta Quest Pro (Unity APK) ──TCP/JSON :63901──► quest_xr_shim
                                                      │
                                              ┌───────┴────────┐
                                              │ 17 xrt.* APIs  │
                                              │ 24×7 SMPL fake │
                                              └───────┬────────┘
                                                      ▼
                                  pico_manager_thread_server.py (unchanged)
                                                      ▼
                                       run_data_collection / N1.7 finetune
```

Only **4 of 24 SMPL joints** carry real data:

| SMPL idx | Joint     | Source                                              |
| :------: | --------- | --------------------------------------------------- |
| 0        | Pelvis    | `head_pos + (0, -0.7, 0)` in Unity; yaw-only orient |
| 12       | Neck      | Quest head pose                                     |
| 22       | L-Wrist   | Quest left controller                               |
| 23       | R-Wrist   | Quest right controller                              |
| others   | —         | zeros + identity quat (ignored by `_process_3pt_pose`) |

## Install

Two parallel install scripts share the same `.venv_teleop` venv:

```bash
bash install_scripts/install_meta.sh   # Meta Quest Pro (this shim)
bash install_scripts/install_pico.sh   # Legacy PICO 4 + foot-trackers
```

Pick one; do not run both (the second overwrites the first's `.venv_teleop`).
The manager picks the right backend at runtime via the `XR_BACKEND` env var
(default `quest`).

Standalone install (already inside an active venv):

```bash
uv pip install -e gear_sonic/utils/teleop/quest_xr_shim
```

## Run

```bash
# 1. Start the data-collection / manager script with the VR_3PT lock on.
python gear_sonic/scripts/pico_manager_thread_server.py \
    --manager --vis_vr3pt --force-vr-3pt
# (waits for a Quest client on tcp://0.0.0.0:63901)

# 2. Launch the Quest Unity APK on the headset; point it at the host's IP.
# 3. Press A+B+X+Y on the right+left controllers to start the session.
```

Env-var knobs:

| Var               | Default        | Purpose                                           |
| ----------------- | -------------- | ------------------------------------------------- |
| `XR_BACKEND`      | `quest`        | Set to `pico` to use the real XRoboToolkit SDK (requires `install_pico.sh`). |
| `QUEST_XR_HOST`   | `0.0.0.0`      | Interface the TCP listener binds to.              |
| `QUEST_XR_PORT`   | `63901`        | TCP port the Unity APK connects to.              |

## API coverage

All 17 `xrt.*` symbols used by `pico_manager_thread_server.py`:

```python
init()                       is_body_data_available()      get_time_stamp_ns()
get_body_joints_pose()       # synthesized 24×7
get_left_trigger()           get_right_trigger()
get_left_grip()              get_right_grip()
get_A_button()               get_B_button()
get_X_button()               get_Y_button()
get_left_menu_button()       get_right_menu_button()
get_left_axis()              get_right_axis()
get_left_axis_click()        get_right_axis_click()
```

Button mapping: A/B = right controller primary/secondary; X/Y = left
controller primary/secondary (matches both PICO and Quest conventions).

## Quest JSON frame format

One JSON object per frame, newline-delimited or back-to-back:

```json
{
  "predictTime": 123955604.744,
  "appState": {"focus": true},
  "Head": {"pose": "x,y,z,qx,qy,qz,qw", "status": 3},
  "Controller": {
    "left":  {"pose": "...", "axisX": 0, "axisY": 0, "axisClick": false,
               "grip": 0, "trigger": 0,
               "primaryButton": false, "secondaryButton": false, "menuButton": false},
    "right": {"...": "same shape as left"}
  },
  "timeStampNs": 1768814265964327168,
  "Input": 2
}
```

`pose` strings are Unity Y-up left-handed frame with scalar-last quaternion
`(qx, qy, qz, qw)`. An untracked controller emits `"0,0,0,0,0,0,-1"`; the
shim detects this sentinel and falls back to the last valid frame.

The Unity→robot frame transform is **not** done here — it happens downstream
in [`_compute_rel_transform`](../../../scripts/pico_manager_thread_server.py)
(the existing `Q` matrix). The shim only delivers Unity-frame poses.

## Known limitations

| Limit                                            | Why                                                       |
| ------------------------------------------------ | --------------------------------------------------------- |
| No squat / step / single-leg / lying tasks       | 3PT method intrinsic limitation                           |
| Bending / leaning corrupts upper-body action     | Pelvis is back-derived from head, assumes upright posture |
| No ankle/knee joints                             | Same as above                                             |
| Coordinate frames must be eyeballed once         | Run with `--vis_vr3pt` and verify wrists track G1's       |

## Package layout

```
quest_xr_shim/
├── pyproject.toml
├── README.md
└── quest_xr_shim/
    ├── __init__.py     # 17 xrt.* APIs, lazy TCP server
    ├── tcp_server.py   # JSON-over-TCP receiver on :63901
    ├── smpl_fake.py    # 24×7 SMPL synthesis + pelvis derivation
    └── coords.py       # Quest pose-string parsing + sentinel detection
```
