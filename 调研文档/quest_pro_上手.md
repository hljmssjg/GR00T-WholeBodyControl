# Quest Pro 上手清单

> 设计文档:[quest_pro_shim_设计.md](quest_pro_shim_设计.md) | shim 包:[gear_sonic/utils/teleop/quest_xr_shim/](../gear_sonic/utils/teleop/quest_xr_shim/)

每一步**做完都要先验证再往下走**,出问题时能精确定位。

---

## 0. 前置条件

| 项 | 检查 |
|---|---|
| 主机 OS | Ubuntu 22.04/24.04 x86_64,或 Jetson Orin aarch64 |
| 主机 ↔ Quest 网络 | 同局域网,主机 IP 能从头盔 ping 通 |
| Quest Unity APK | 已装,能发设计文档 §3 那种 JSON,目标 `主机IP:63901`,TCP 模式 |
| 操作员姿态 | 站立,会保持上半身直立(下半身靠 SONIC 规划) |
| 头显佩戴 | **必须戴在头上,不能挂脖子上**。`smpl_fake.py` 里 neck (joint 12) = HMD 姿态,挂脖子上会让 `calib_inv` 含 pitch,wrist 旋转轴会被相似变换扭歪 |

**没有 Unity APK 一切免谈**——这是阻塞项,先解决。

---

## 1. 装环境(2 个 venv)

```bash
cd /home/jg/baseline/GR00T-WholeBodyControl
bash install_scripts/install_meta.sh            # → .venv_teleop  (含 quest_xr_shim)
bash install_scripts/install_data_collection.sh # → .venv_data_collection
```

**验证**: `ls -d .venv_teleop .venv_data_collection`,两个目录都在。

---

## 2. 验 shim(不连头盔)

```bash
source .venv_teleop/bin/activate
python -c "import quest_xr_shim as xrt; xrt.init(); import time; time.sleep(2)"
```

**期望**: 看到 `[quest_xr_shim] Listening for Quest client on tcp://0.0.0.0:63901`,Ctrl+C 退出。

失败 → 检查 [gear_sonic/utils/teleop/quest_xr_shim/](../gear_sonic/utils/teleop/quest_xr_shim/) 是否被 `uv pip install -e` 装上(`uv pip list | grep quest`)。

---

## 3. 验 Quest 连接(连头盔,不录数据)

```bash
python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt --force-vr-3pt
```

**逐项确认**:

| 阶段 | 预期输出 | 失败原因 |
|---|---|---|
| 启动 | `Listening for Quest client on tcp://0.0.0.0:63901` | shim 未装 |
| PyVista 窗口 | 弹出,显示参考坐标系 | pyvista/Qt 缺失;无 DISPLAY |
| 头盔启动 APK | `[quest_xr_shim] Quest connected from (...)` | 防火墙;APK IP 配错;端口被占 |
| "waiting for body data..." 停止 | `is_body_data_available()` 转 True | APK 没在发,或 head pose 全是 sentinel |

---

## 4. 验坐标系 + 标定(关键)

站好(T-pose 或双手自然下垂),**右控制器 A + 左控制器 X + 右 B + 左 Y 一起按下**:

- 触发 `calibrate_now`,此刻姿态记为零位
- 状态机进 `PLANNER_VR_3PT`,PyVista 里 G1 出现并跟手腕

**眼睛检查**(分两类——PyVista 里和 PyVista 外):

**PyVista 里能看到的(上半身追踪)**:

1. 抬左手 → G1 左手抬(不是右手)
2. 抬右手 → G1 右手抬

**PyVista 里看不到的(base/yaw 是 ZMQ 发给 sim 的,不进 visualizer)**:

3. **左摇杆推前** → G1 base 前移。不起 sim 的最快验证:跑 [debug_planner_sub.py](../gear_sonic/scripts/debug_planner_sub.py) 监听 `tcp://*:5556` 的 `planner` topic,推杆时 `movement` 出现非零 world-frame XY、松手归零。
4. **右摇杆推右** → `yaw_accumulator` 累积 yaw。同上看 `facing`:顺时针累积旋转(满杆约 -2.9 rad/s),**松手停在新角度不归零**(累积器,非速率)。

**档位切换**(只在 StreamMode 是 PLANNER_* 时生效,即 [PlannerLoop.run_once](../gear_sonic/scripts/pico_manager_thread_server.py#L1715-L1725)):

- **A+B 单独按**(松开 X 和 Y)→ `self.mode += 1`(IDLE → SLOW_WALK → WALK → RUN → ... → INJURED_WALK,cap 19)
- **X+Y 单独按**(松开 A 和 B)→ `self.mode -= 1`(底到 IDLE=0)
- **A+B+X+Y 同帧**:加一减一**互抵**,自身不动档,但外层会把 StreamMode 切了(OFF ↔ VR_3PT)

注意:wire 上的 `mode` 字段 ≠ `self.mode`。**左摇杆在死区时,wire `mode` 被强制写成 IDLE=0、`speed=-1`**,无论当前选的是什么档([pico_manager_thread_server.py:1735-1743](../gear_sonic/scripts/pico_manager_thread_server.py#L1735-L1743))。所以"按了 A+B 升档但 sub 看到 mode 还是 0"是正常的,推杆出死区后才会看到真档位。

PyVista 里 G1 的 base 永远固定在原点+面向同一方向——它只渲染相对 root 的 3-point 追踪。

**任何一项错位**: 先看 §4.1 排查顺序,**不要先动 `Q` 矩阵**。

紧急停止: 再按一次 `A+B+X+Y` → OFF。

---

## 4.1 wrist 旋转轴错位的排查与修法(2026-05-12 bring-up 复盘)

> 下游风险评估见 [quest_pro_postmul_下游风险.md](quest_pro_postmul_下游风险.md)。


第一次跑 Step 4 时观察到:抬手位置都对,但 wrist 旋转轴串台——physical-Z(上)旋转 → G1 wrist 绕 Y 转;physical-Y → G1 X 转。两只手都一样。

**正确排查顺序(先硬件、后代码)**:

1. **检查 HMD 是不是真戴在头上**。`smpl_fake.py` 里 `joint 12 (neck) = head_pose`,挂脖子上 → HMD 屏幕朝上 → neck quat 含 ~90° pitch → `_calibration_neck_quat_inv` 把这个 pitch 锁死,后续每帧 wrist 都被这个 pitch 左乘,X/Z 轴会互换。**戴头上能修掉大部分轴错位**(尤其是 X/Z 互换)。
2. **检查 controller 握法 + 标定姿态**。OFFSETS 假设了一个特定的初始 wrist 朝向([pico_manager_thread_server.py:164-171](../gear_sonic/scripts/pico_manager_thread_server.py#L164-L171)),歪握或者半蹲标定都会偏。
3. 排除完硬件再考虑代码。

**代码侧改动(commit `cdbd8b1`)**: `_apply_calibration` 里 wrist 的 `rot_offset` 从 premul 改成 postmul。

- 旧 (premul): `calibrated = rot_offset * calib_inv * wrist`。世界帧每个旋转 delta 被 `(rot_offset * calib_inv)` 相似变换扭一遍,即使 HMD 戴正,`rot_offset` 这一层(`g1_lwrist_rot * lwrist_corrected_inv`)仍会绕走轴。
- 新 (postmul): `calibrated = (calib_inv * wrist) * rot_offset`。`rot_offset` 只用来让标定瞬间对齐 G1 初始 wrist 姿态,不再扭曲后续 delta。
- **门控在 `XR_BACKEND != "pico"`**(`use_intrinsic_wrist_offset=_IS_QUEST_BACKEND`),Pico 路径完全保留旧 premul,避免回归。
- 改动位置: [pico_manager_thread_server.py:894-925](../gear_sonic/scripts/pico_manager_thread_server.py#L894-L925)(`ThreePointPose.__init__`)、[`_capture_calibration`](../gear_sonic/scripts/pico_manager_thread_server.py#L1084-L1099) 和 [`_apply_calibration`](../gear_sonic/scripts/pico_manager_thread_server.py#L1123-L1152)。

**实测**(HMD 戴正 + postmul gate):L/R wrist 三轴旋转都正确。回退 postmul 仍会出现 wrist 轴跑偏,说明这一层改动是必要的、不是 HMD 单点问题。

**§4.3 / §4.4 验证完成(2026-05-12)**:不起 sim,改走 ZMQ subscriber 路径——写了 [debug_planner_sub.py](../gear_sonic/scripts/debug_planner_sub.py),订阅 `tcp://localhost:5556` 的 `planner` topic,打印 `mode/movement/facing/speed`。实测:

- 左摇杆推前 → `movement` 在 world frame 出现非零 XY、松手归零;反推 local frame 一致 ✓
- 右摇杆推右 → `facing` 顺时针累积旋转(满杆约 -2.9 rad/s),松手停住不归零 ✓
- wire `mode` 始终是 0 是预期行为(见上节关于 deadzone 强制 IDLE 的说明,以及 A+B+X+Y 同帧 ±1 互抵的 bring-up 教训)

---

## 4.5 用 sim 走一遍数采全链路(dry-run,不接真机)

§4 只验了 manager 内部的坐标/标定 + planner ZMQ 输出,数采链路下游(C++ deploy → sim → run_data_exporter → parquet)还没动过。先在 sim 里把整条链路跑通,再上真机。

**前置**: `.venv_sim` 已装好(`ls -d .venv_sim/bin/activate`),`gear_sonic_deploy/` 已编译。

```bash
python gear_sonic/scripts/launch_data_collection.py \
    --sim \
    --pico-vis-vr3pt \
    --task-prompt "sim_dryrun" \
    --no-text-to-speech
```

会起一个 `sonic_data_collection` tmux session:窗口 0(4 pane: deploy / data exporter / pico manager / camera viewer)+ 窗口 1(sim)。`tmux attach -t sonic_data_collection`,`Ctrl-b w` 切窗口。

**逐项确认**(出问题在哪个 pane 就在哪个 pane 看 trace,不要直接 kill session):

| 检查 | 期望 | 失败处置 |
|---|---|---|
| 窗口 1 sim | MuJoCo viewer 弹出,G1 站在原点 | `.venv_sim` 没装好;mujoco GL 驱动;无 DISPLAY |
| pane 0 deploy | `./deploy.sh sim` 不报错,等 zmq_manager 输入 | `gear_sonic_deploy` 未编译;deploy_checkpoint/obs_config 路径错 |
| pane 2 pico manager | §3 那串 listening 日志,Quest 连上 | 同 §3 |
| 4 键标定 → 进 PLANNER_VR_3PT | sim 里 G1 跟着手腕动、左摇杆推前 base 前移、右摇杆推右 yaw 累积 | 现象和 §4 不一致:回 §4 / §4.1 排查,**不要在 sim 里调** |
| pane 1 data exporter | 周期性打 frame 计数,无 sentinel 警告 | run_data_exporter 没拿到 camera 流;`--camera-port` 冲突 |
| 触发录制 → 停止 | exporter pane 打出 parquet 路径 | 看 exporter trace,通常是 dataset_name 目录权限 |

**录 1 条 < 30 秒的 episode**,落盘后 `Ctrl-b &` 关 session,直接进 §6 校验 parquet。**§6 通过 = dry-run 通过**,这条 episode 别留着当训练数据。

dry-run 不通过 → 不要进 §5。sim 跑不通的链路接真机只会更难 debug,而且真机失败的损失更大(操作员姿态、安全员、场地占用)。

---

## 5. 真跑数据采集

§4.5 走通后,把 `--sim` 去掉(其它参数照旧):

```bash
python gear_sonic/scripts/launch_data_collection.py [task_args]
```

它会用 tmux 同时拉起 manager + run_data_exporter (+ sim,如果配的话)。具体参数看 [launch_data_collection.py](../gear_sonic/scripts/launch_data_collection.py) 自带的 docstring。

录 **1 条短 episode** 先(< 30 秒),不要直接录长的。

---

## 6. 验 parquet 输出

```bash
source .venv_data_collection/bin/activate
python - <<'PY'
import pyarrow.parquet as pq, sys
t = pq.read_table(sys.argv[1])
print("schema:", t.schema)
print("rows:", t.num_rows)
print("motion_token sample:", t["action.motion_token"][0].as_py()[:8], "...")
PY <episode.parquet 路径>
```

**期望**: `action.motion_token` 是 **64 维 float**,**不是全零**。

全零 → encoder ONNX 没跑通,或 VR_3PT 输入全是 sentinel,回 step 4 查可视化。

---

## 已知坑

| 现象 | 原因 | 处置 |
|---|---|---|
| `is_body_data_available()` 一直 False | APK 没发 / 防火墙挡 :63901 | `sudo ufw allow 63901/tcp` |
| G1 手腕在原点抽搐 | head/controller 长期发 sentinel | 检查 APK 追踪状态,佩戴方式 |
| G1 走的方向反 | Unity 左手系 ↔ 机器人右手系 `Q` 矩阵 | 调 `Q` 的对应行符号 |
| wrist 旋转轴串台(X/Y/Z 互错) | HMD 没戴对 / `rot_offset` premul 扭曲 delta | 先戴正 HMD;若仍偏,见 §4.1,代码已在 commit `cdbd8b1` 加 postmul 门控 |
| 录数据时 parquet 没生成 | run_data_exporter 没起 / .venv_data_collection 缺 | 看 tmux 各窗口报错 |
| 录到的 `action.motion_token` 全零 | encoder ONNX 路径错;输入是无效 SMPL | 看 manager 日志里 token 是否被打印 |
| 按 A+B+X+Y 进 VR_3PT 后档位没升 | 4 键同帧:A+B 加 1、X+Y 减 1,互抵 | 进 VR_3PT 后**全部松手 1 秒**,再单独按 A+B(松开 X 和 Y) |
| sub 看到 wire `mode` 一直 0 | (a) 左摇杆在死区 → 强制 IDLE;(b) `self.mode` 没升档 | 检查摇杆是否推出死区,manager terminal 是否打过 `[PlannerLoop] Mode -> N` |

---

## 索引

| 干什么 | 文件 |
|---|---|
| Quest 后端实现 | [gear_sonic/utils/teleop/quest_xr_shim/](../gear_sonic/utils/teleop/quest_xr_shim/) |
| 主程序(import / state machine) | [gear_sonic/scripts/pico_manager_thread_server.py](../gear_sonic/scripts/pico_manager_thread_server.py) |
| 数据采集 orchestrator | [gear_sonic/scripts/launch_data_collection.py](../gear_sonic/scripts/launch_data_collection.py) |
| 安装 Quest 后端 | [install_scripts/install_meta.sh](../install_scripts/install_meta.sh) |
| 安装 LeRobot 导出 | [install_scripts/install_data_collection.sh](../install_scripts/install_data_collection.sh) |
| 设计 + JSON 协议 | [quest_pro_shim_设计.md](quest_pro_shim_设计.md) |
| 不起 sim 验证 base/yaw | [gear_sonic/scripts/debug_planner_sub.py](../gear_sonic/scripts/debug_planner_sub.py) |
