# Autopilot trajectories

Walk segments captured/replayed by `gear_sonic/scripts/auto_pilot.py`.

Files (written by the autopilot daemon when the operator presses the record
toggle in VR):
- `walk_to_table.json`   — forward path from start anchor to in front of the table
- `return_to_start.json` — reverse path back to the start anchor

## Record (operator drives via Quest/Pico)

Pre-req: `launch_data_collection.py --sim --autopilot` is running — the daemon
is already in `autopilot` window, listening for record/replay edges from the
manager. Enter `PLANNER_VR_3PT` (Quest backend lands here from OFF via the
`A+B+X+Y` start combo).

Button bindings (gated to `PLANNER_VR_3PT`):

| Action                                          | Binding                         |
|-------------------------------------------------|---------------------------------|
| Record forward + walk straight fwd + lock pose  | `right_grip` + tap **A**        |
| Record return  + walk straight bwd + lock pose  | `right_grip` + tap **B**        |
| Replay forward toggle                           | Left stick click                |
| Replay return  toggle                           | Right stick click               |

`right_grip + A/B` and stick clicks need right_grip held to disambiguate from
`A+X` / `B+Y` / data-collection combos.

## init_pose.json — manually committed

`init_pose.json` is a static file in this directory, hand-tuned to a
"hands-held-forward at chest height, hands open" pose for the G1 robot. It is
loaded once at PlannerStreamer init and re-loaded on each override engage, so
**no capture step is needed** — just commit the file and it gets used.

Schema (see `_layout` field in the JSON for the live description):
  * `vr_3pt_position`: 9 floats — `[L_wrist_xyz, R_wrist_xyz, Neck_xyz]` in
    robot frame relative to pelvis, in meters
  * `vr_3pt_orientation`: 12 floats — same 3 keypoints' quat in `wxyz` order
  * `lh_joints` / `rh_joints`: 7 floats each, zeros = open hand

To re-tune, edit the JSON values directly and restart the manager (or just
trigger any record/replay toggle — the file is hot-reloaded on rising edge).

If you change robot models and want an XML-derived starting point, there is
a one-off helper that FKs the model-default qpos and dumps a fresh JSON:
`.venv_sim/bin/python gear_sonic/scripts/generate_init_pose.py`.
The committed JSON above takes precedence — only run the helper if you want
to overwrite it.

If `init_pose.json` doesn't exist yet, record+walk and replay fall back to
"capture current pose at engage time" (the old behavior).

A record toggle does THREE things at once:
  * pins `lx, ly` to a clean straight-line vector (`(0,+1)` for forward,
    `(0,-1)` for return) — bypasses Quest stick bias
  * pins upper body + hands to **init_pose.json** (loaded once at startup;
    falls back to "current pose at engage" if the file is missing)
  * locks **yaw** to the heading at the moment of the toggle press — `rx`
    input is ignored until the toggle stops
  * starts/stops the planner-frame recording in the daemon

So the operator: faces the robot in the target direction, takes hands off the
controllers if desired, presses once. The robot snaps to the init pose and
walks straight in the locked direction. Press again to stop everything.

Workflow:
1. Position the robot at the start anchor (manual driving or just spawn pose).
2. `right_grip + A` → robot starts walking straight forward AND daemon starts
   buffering frames. Steer yaw with right stick if needed.
3. When the robot is at the table: `right_grip + A` → stops. `walk_to_table.json`
   is saved. Robot stops too.
4. `right_grip + B` → robot starts walking straight backward AND daemon starts
   buffering frames for return.
5. When the robot is back at start: `right_grip + B` → stops.
   `return_to_start.json` is saved.

Interlocks:
  * If you press **A** while a return record is active (or **B** while forward
    is active), the press is ignored.
  * Exiting `PLANNER_VR_3PT` mid-record sends a stop toggle so the buffer is
    flushed to JSON and the walk override is cleared (no phantom drift).
  * During an active record, replay button presses are ignored by the daemon.

The daemon will NOT publish `reset_cmd` between record segments — keep the
operator/sim state coherent yourself.

## Replay

Replay is automatic once both JSON files exist:
- Left stick click → play forward (walks to table)
- Right stick click → play return (walks back; emits `reset_cmd` at end unless
  `--no-sim-reset`)

See [autopilot_数采辅助设计.md](../../../调研文档/autopilot_数采辅助设计.md) for
the topic-level wire protocol.
