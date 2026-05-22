# Autopilot 数采辅助实现摘要

> 配套设计文档：[autopilot_数采辅助设计.md](autopilot_数采辅助设计.md)
> 本文件只罗列**实际落地**的改动 + 操作员手册，不复述设计动机。

---

## 1. 新增文件

| 文件 | 用途 |
|------|------|
| [gear_sonic/scripts/auto_pilot.py](../gear_sonic/scripts/auto_pilot.py) | 统一 daemon：处理回放 + 录制，进程级状态机 |
| [gear_sonic/scripts/generate_init_pose.py](../gear_sonic/scripts/generate_init_pose.py) | 离线工具：从机器人 XML 的 home qpos FK 出 init_pose.json |
| gear_sonic/data/autopilot_trajs/init_pose.json | 手动调好的"端在胸前"上半身锁定姿态（committed） |
| gear_sonic/data/autopilot_trajs/README.md | 操作员说明（committed） |
| gear_sonic/data/autopilot_trajs/walk_to_table.json | 操作员录的 forward 轨迹（gitignored） |
| gear_sonic/data/autopilot_trajs/return_to_start.json | 操作员录的 return 轨迹（gitignored） |

## 2. 修改文件

- **[gear_sonic/scripts/pico_manager_thread_server.py](../gear_sonic/scripts/pico_manager_thread_server.py)**
  - PlannerStreamer 订阅 `autopilot_cmd`（CONFLATE），active=True 时 override `movement/facing/speed/mode`
  - 新增 `_walk_straight_mode` 字段：set 后强制 `lx,ly=(0,±1)`
  - 新增 init_pose 加载 + 热重载：override 起始边沿装入 `_autopilot_snapshot`、yaw 锁定
  - 死人开关：`|stick| > 0.3` 时发 `autopilot_request` cancel
  - Manager 主循环新增按键：
    - 左/右摇杆点击 → `autopilot_request`（回放 forward/return toggle）
    - `right_grip + A/B` → `record_request` + 切 `walk_straight_mode`（录制 + 直行 + 锁姿 + 锁 yaw 三合一）
  - 安全网：模式切出 PLANNER_VR_3PT 时自动发停录 toggle、清 walk_straight、解 yaw 锁
  - 限频：`Sending VR 3-point pose` 由 20Hz 改 5s 一次

- **[gear_sonic/scripts/run_sim_loop.py](../gear_sonic/scripts/run_sim_loop.py)**
  - 起 daemon 线程订阅 `reset_cmd`，收到后调 `sim_wrapper.sim.reset()`

- **[gear_sonic/scripts/launch_data_collection.py](../gear_sonic/scripts/launch_data_collection.py)**
  - 新增 `--autopilot` / `--autopilot-port` flag，启动时在独立 tmux 窗口跑 `auto_pilot.py --play`

- **[gear_sonic/utils/mujoco_sim/configs.py](../gear_sonic/utils/mujoco_sim/configs.py)**
  - SimLoopConfig 新增 `enable_autopilot_reset` / `autopilot_host` / `autopilot_port`

## 3. 拓扑

```
Quest ──► pico_manager (PUB :5556)
              │ planner          ─► C++ deploy + auto_pilot (录制时缓存帧)
              │ autopilot_request ─► auto_pilot (replay toggle)
              │ record_request    ─► auto_pilot (record toggle)
              │ manager_state     ─► data_exporter
              │
              ▼ (planner_streamer 内部)
auto_pilot (PUB :5558)
              │ autopilot_cmd ─► pico_manager.PlannerStreamer (CONFLATE)
              └ reset_cmd     ─► run_sim_loop (daemon thread)
```

## 4. 按键映射（操作员手册）

全部门控在 `PLANNER_VR_3PT` 模式下生效。

| 操作 | 绑定 |
|------|------|
| 录 forward + 直行向前 + 锁姿 + 锁 yaw | `right_grip + 点击 A` |
| 录 return  + 直行向后 + 锁姿 + 锁 yaw | `right_grip + 点击 B` |
| 回放 forward toggle | 左摇杆点击 |
| 回放 return  toggle | 右摇杆点击 |

互锁：录制中忽略回放、回放中忽略录制；A 在 return 录制期间忽略，B 在 forward 录制期间忽略。

## 5. init_pose.json

手动维护的 9+12+7+7 float 文件，schema 见文件本体的 `_layout` 字段。`PlannerStreamer.__init__` 加载，override 起始边沿热重载——编辑 JSON 后无需重启 manager，下次 toggle 自动生效。

## 6. 持久日志

daemon 把所有状态变化写到 `gear_sonic/data/autopilot_trajs/.daemon_trace.log`，每 3 秒一行 heartbeat，落盘/engage/disengage 都有横线 banner。tmux 滚走也能事后 grep。

## 7. 验证状态

- ✅ 静态：四文件语法 / import 干净
- ✅ ZMQ 协议回环：fake manager → daemon → JSON 落盘（forward + return 都过）
- ✅ 仿真：操作员实际录到 walk_to_table.json + return_to_start.json
- ⚠️ 回放 stop toggle 真机 / 实测有疑问（用户报告："按右摇杆 return 后再按一次没停"），待复现 + 修复
- ⏳ 真机：未验证；预期 init_pose 直接通用，前提是 G1 XML 与硬件运动学一致
