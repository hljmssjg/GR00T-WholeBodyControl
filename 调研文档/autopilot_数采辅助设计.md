# Autopilot 数采辅助方案（teleop walk + reset）

> 目标：Quest VR-3pt 模式下数采"走到桌前 → 抓 → 放"任务时，把长距走路段和 episode 间回位**自动化**，
> 减轻操作员负担、提高 episode 吞吐。
> 约束：不改 C++ deploy、不改 data_exporter、不动数据集 schema。

---

## 1. 现状与切入点

teleop → 机器人的命令链：

```
Quest controllers ──> pico_manager_thread_server.py (PUB :5556)
                          ├── topic=planner        → C++ deploy 订阅（真正驱动 walker/arms）
                          └── topic=manager_state  → run_data_exporter 订阅（录制 toggle）
```

关键事实：
- 走路命令 = `planner.movement/facing/speed`，唯一发出点是
  [`PlannerStreamer.run_once`](../gear_sonic/scripts/pico_manager_thread_server.py#L1710)。
- 已有先例 [line 1739-1742](../gear_sonic/scripts/pico_manager_thread_server.py#L1739-L1742)：
  `stick_click_forward` 直接覆盖 `lx,ly = 0,1.0` —— 注入 autopilot 命令的天然入口。
- 录制 toggle 走 `manager_state`，和走路命令是**两条独立 topic**，autopilot 段落是否进数据集只取决于操作员按 `LeftGrip+A` 的时机。
- Sim 端 `sim_env.reset()` 已存在 ([base_sim.py:642](../gear_sonic/utils/mujoco_sim/base_sim.py#L642))，但无外部触发口。

设计原则：**单 publisher**。autopilot 不直接发 `planner`，只发新 topic `autopilot_cmd`，manager 唯一改写 `lx/ly/rx`。

---

## 2. UX

| 按键 | 行为 |
|---|---|
| LeftGrip + A | 录制 start / stop（已有） |
| LeftGrip + B | abort 当前 episode（已有） |
| **左 stick click** | toggle autopilot **forward**（走到桌前） |
| **右 stick click** | toggle autopilot **return**（走回起点） |
| 摇杆推动 \|raw_mag\| > 0.3 | deadman：立即 disengage，推动量作为手动控制 |
| A+B+X+Y | 紧急停止（已有） |

互锁：autopilot 同时只能有一个活跃。forward 活跃时按右 click 被忽略，反之亦然。再按一次同一 click = 取消并回到手动 VR_3PT。

录制 vs 不录制由按键顺序自然解耦：
- forward：先 `LeftGrip+A` 起录 → 左 click（**进数据集**）
- return：先 `LeftGrip+A` 停录 → 右 click（**不进数据集**，纯重置）

---

## 3. 上半身在 autopilot 期间冻结

操作员在 autopilot 期间不需要保持手姿态。机制：

```python
if autopilot_engaging_this_frame:
    snapshot = current vr_3pt pose + hand joints
if autopilot_engaged:
    movement / facing 来自 trajectory
    vr_3pt_position / orientation / hand_q 全部用 snapshot
if autopilot_just_disengaged:
    recalibrate_for_vr3pt()    # 已有，line 1690
    丢弃 snapshot
```

收益：操作员可放下手、调整头显；disengage 时无跳变（重标定把操作员现位置当新零点）。
`stream_mode` 全程是 `PLANNER_VR_3PT`，数据集 schema 零改动。

---

## 4. 取消 / IDLE

不需要单独 IDLE 键。autopilot 退出 + 操作员不推杆 ≡ `movement=0` + `mode=IDLE`，walker 自然站立。

三种取消语义：

| 方式 | 触发 | 之后 |
|---|---|---|
| Toggle off | 再按同一 click | 手动 VR_3PT，机器人原地站立 |
| Deadman | 摇杆推 \|raw_mag\|>0.3 | 手动 VR_3PT，且推动量直接生效（立即接管方向） |
| Emergency stop | A+B+X+Y | `StreamMode.OFF`，policy 停 |

---

## 5. 回位

**Sim**：右 click 回放 return 轨迹结束后，autopilot 额外发 `reset_cmd` →
`run_sim_loop` 调 `sim_env.reset()` + 把 jug freejoint 写回初始位。

**真机**：反向轨迹回放，依赖 SONIC walker 跟踪。无 odometry，预期累计漂移 ±20–50cm，
末端用 SLOW_WALK 收尾 + 现场预留安全区。漂移过大再降级为"半自动 + TTS 提示手动走回"。

---

## 6. 轨迹格式：现场录制

不手写 JSON。`auto_pilot.py --record forward` 现场跑一遍，把 manager 发出的
`movement / facing / speed / mode` 时序原样存为 traj 文件；`--play` 直接回放。

好处：调参 = 重录；轨迹自动匹配当前 SONIC walker 行为（手写 JSON 很难校准速度曲线）。

文件：
- `gear_sonic/data/autopilot_trajs/walk_to_table.json`
- `gear_sonic/data/autopilot_trajs/return_to_start.json`

---

## 7. 文件改动清单

| # | 文件 | 类型 | 内容 |
|---|------|------|------|
| 1 | `gear_sonic/scripts/auto_pilot.py` | 新增 | `--record` 抓 traj，`--play` 监听 stick click 事件回放，PUB topic `autopilot_cmd` 到 :5558 |
| 2 | `gear_sonic/scripts/pico_manager_thread_server.py` | 改 ~50 行 | (a) `PlannerStreamer` 加 `autopilot_sub`；(b) `run_once` 加 override + snapshot + deadman；(c) 左/右 stick click 互锁，替换现有 `stick_click_forward` |
| 3 | `gear_sonic/scripts/run_sim_loop.py` | 改 ~30 行 | ZMQ SUB `reset_cmd` → `sim_env.reset()` + jug 归位 |
| 4 | `gear_sonic/scripts/launch_data_collection.py` | 改 ~20 行 | `--autopilot` flag 多起一个 pane 跑 `auto_pilot.py --play` |
| 5 | `gear_sonic/data/autopilot_trajs/*.json` | 新增 | 现场录的两条 traj |

零改动：data_exporter、C++ deploy、Isaac-GR00T 训练 pipeline、数据集 schema。

---

## 8. 风险与兜底

| 风险 | 影响 | 兜底 |
|---|---|---|
| 真机无 odometry，return 漂移 | 起点偏移影响下一条 demo 一致性 | 末端 SLOW_WALK；漂移过大 → 降级为"半自动 + 手动走回" |
| autopilot 走路段过于一致，VLA 过拟合 | 部署换初始位走偏 | traj 添加 ±10% 时长/速度 jitter；后期补录人工 walk demo |
| 操作员误触 stick click | 中途乱动 | deadman 立即接管；toggle 取消 |
| 双 publisher 抢占 :5556 | 命令错乱 | 单源原则：autopilot 仅发 `autopilot_cmd`，manager 唯一改写 |
| sim reset 后 jug 穿透瞬时 | 不稳定 | reset 后空跑 0.5s 稳定再 unlock 下一条 |
| disengage 瞬间手臂跳变 | 数据/安全问题 | 复用现有 `recalibrate_for_vr3pt()` |

---

## 9. 验证里程碑

| # | 内容 | 通过标准 |
|---|------|---------|
| V1 | `auto_pilot.py --record` 单独跑通 | traj 文件生成，`debug_planner_sub` 能看到回放命令 |
| V2 | sim 里左 click → 走到桌前 | 末端误差 ±20cm（M4 等价） |
| V3 | sim 里右 click → reset，jug + robot 回初始位 | qpos 归零、jug z 稳定 |
| V4 | sim 完整一条 demo：录 → 左 click → 手动 pick → 停录 → 右 click | `process_dataset.py` 能解析 |
| V5 | 真机 1m 短距 forward autopilot | 不摔，deadman 可中断 |
| V6 | 真机 return autopilot 跑通 | 漂移 ≤30cm，episode 间隔 < 30s |

V1–V4 先在 sim 闭环跑通，再上真机 V5–V6。
