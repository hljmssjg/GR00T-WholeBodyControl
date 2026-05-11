# Quest Pro 替换 PICO 数据采集 — Shim 设计

> 目标:用 Meta Quest Pro + 2 手柄(无脚环)替换官方 PICO 4 + 脚环方案,接入 SONIC + N1.7 数据采集管线,**改动控制在 1 行 import + 1 个 CLI flag**。

---

## 1. 核心思路

NVIDIA 整个 VR 输入入口是一行:
```python
# gear_sonic/scripts/pico_manager_thread_server.py:75
import xrobotoolkit_sdk as xrt
```

**写一个新名 Python 包 `quest_xr_shim`**,API 与 `xrobotoolkit_sdk` 完全一致,内部用 TCP 协议接收 Quest Unity APK 发来的 JSON。改一行 import 即可切换:
```python
import quest_xr_shim as xrt   # ← 一字之差
```
下游 `run_data_exporter.py`、C++ deploy 全部零改动。

**为什么不用同名劫持**(占用 `xrobotoolkit_sdk` 这个名字):
- 命名污染 —— `xrobotoolkit_sdk` 是 PICO 官方包名
- 真假 SDK 同 venv 互斥,不利切回 PICO 调试
- 新来同事看 venv 包列表会以为跑的是真 SDK
- 多 1 行 import 改动换来名字诚实 + 真假并存 + 自解释,值

---

## 2. 为什么不能直接用 xr_teleoperate

| 维度 | xr_teleoperate | NVIDIA SONIC + N1.7 |
|---|---|---|
| 动作抽象 | 末端位姿 → IK → 关节 | SMPL → encoder ONNX → **64-D token** → policy ONNX → 关节 |
| 数据集格式 | 自家 `episode_writer` | LeRobot v2.1 (parquet+mp4) |
| 下半身 | 不管 | SONIC 规划器生成(VR_3PT)或操作员脚环(POSE) |
| 录到 parquet 的主目标字段 | `action.qpos` | `action.motion_token`(64 维) |

直接搬 xr_teleoperate 会**丢掉 64-D token**,微调 N1.7 用不了。

**复用方式**: 只把 xr_teleoperate 的 TCP 协议文件**拷贝**到本仓库的 shim 目录(不跨仓库引用),其它都不要。

---

## 3. Quest Unity APK 实测 JSON(所有字段齐全 ✅)

```json
{
  "predictTime": 123955604.744,
  "appState": {"focus": true},
  "Head": {
    "pose": "x,y,z,qx,qy,qz,qw",
    "status": 3
  },
  "Controller": {
    "left": {
      "pose": "x,y,z,qx,qy,qz,qw",
      "axisX": 0.0, "axisY": 0.0,
      "axisClick": false,
      "grip": 0.0, "trigger": 0.0,
      "primaryButton": false, "secondaryButton": false,
      "menuButton": false
    },
    "right": { /* 同 left */ }
  },
  "timeStampNs": 1768814265964327168,
  "Input": 2
}
```

**坐标系**: Unity Y-up 左手系,scalar-last 四元数 `(qx,qy,qz,qw)`。

**关键细节**: 未追踪的手柄会发 `pose="0,0,0,0,0,0,-1"`,shim 要识别并沿用上一帧有效值。

---

## 4. API 完整映射表

NVIDIA 调用了 17 个 `xrt.*` API,Quest JSON 全部能覆盖:

| `xrt.*` API | 返回 | 来源 |
|---|---|---|
| `init()` | — | 起 TCPServer on `:63901` |
| `is_body_data_available()` | bool | 收到首条 JSON 后 True |
| `get_time_stamp_ns()` | int | JSON `timeStampNs` |
| `get_body_joints_pose()` | list[24×7] | **伪造**,见 §5 |
| `get_{left,right}_trigger()` | float | `Controller.{l,r}.trigger` |
| `get_{left,right}_grip()` | float | `Controller.{l,r}.grip` |
| `get_A_button()` | bool | `Controller.right.primaryButton` |
| `get_B_button()` | bool | `Controller.right.secondaryButton` |
| `get_X_button()` | bool | `Controller.left.primaryButton` |
| `get_Y_button()` | bool | `Controller.left.secondaryButton` |
| `get_{left,right}_menu_button()` | bool | `Controller.{l,r}.menuButton` |
| `get_{left,right}_axis()` | (x,y) | `[Controller.{l,r}.axisX, axisY]` |
| `get_{left,right}_axis_click()` | bool | `Controller.{l,r}.axisClick` |

**A/B/X/Y 按钮约定**: A,B 在右控制器(primary/secondary);X,Y 在左控制器(primary/secondary)。Quest 和 PICO 都遵循此约定。

---

## 5. 24×7 SMPL 伪造方案

NVIDIA 的 [`_process_3pt_pose`](../gear_sonic/scripts/pico_manager_thread_server.py#L200) 只读 4 个关节索引:`[0, 22, 23, 12]`。其它 20 个填零不影响。

| SMPL 索引 | 关节 | Shim 取值 |
|---|---|---|
| 0 | Pelvis | **派生**: `head_pos + [0, -0.7, 0]`(Unity Y 向下 0.7m);姿态取 head 的 yaw,pitch/roll 归零 |
| 12 | Neck | `Head.pose` 直接喂(也可减去小颈椎偏移) |
| 22 | Left Wrist | `Controller.left.pose` 直接喂 |
| 23 | Right Wrist | `Controller.right.pose` 直接喂 |
| 其它 20 | — | 零位置 + 单位四元数 `(0,0,0,1)` |

**Pelvis 派生的约束**: 假定操作员保持站立。倾身/弯腰会让"虚拟骨盆"与真实头位脱钩,encoder 看到的输入失真,policy 表现漂移。

---

## 6. NVIDIA 端必要改动(共 2 处)

### 6.1 改 1 行 import(`pico_manager_thread_server.py:75`)

```python
# 原
import xrobotoolkit_sdk as xrt
# 改
import quest_xr_shim as xrt
```

可选优雅版(支持环境变量切换):
```python
import os
if os.environ.get("XR_BACKEND", "quest") == "pico":
    import xrobotoolkit_sdk as xrt
else:
    import quest_xr_shim as xrt
```

### 6.2 锁定 VR_3PT 模式(加 CLI flag)

原状态机在 POSE / FROZEN / VR_3PT 间切换。POSE 和 FROZEN 都要 full 24-joint SMPL(脚环来的),Quest 给不出。

最小改动 — 加 `--force-vr-3pt` CLI flag:
- 默认 False:走原状态机(给真 PICO 用户)
- True:[`run_pico_manager`](../gear_sonic/scripts/pico_manager_thread_server.py#L1803) 入口处把 `current_mode` 初始化为 `PLANNER_VR_3PT`,屏蔽状态机里通往 POSE/FROZEN 的跳转
- 保留 `start_combo` (A+B+X+Y) → OFF 的紧急停止

---

## 7. 落地结构(全部本仓库内,不跨 repo 引用)

```
GR00T-WholeBodyControl/
└── gear_sonic/utils/teleop/quest_xr_shim/
    ├── pyproject.toml           # 包元数据,使 quest_xr_shim 可 pip install
    ├── quest_xr_shim/
    │   ├── __init__.py          # 17 个 API + 24×7 伪造 + Pelvis 派生
    │   ├── tcp_server.py        # 从 xr_teleoperate 拷过来(JSON over TCP :63901)
    │   ├── coords.py            # Unity → 机器人坐标系转换
    │   └── smpl_fake.py         # 24×7 数组构造逻辑
    └── README.md
```

**所有外部代码一律拷贝进来**:
- `xr_teleoperate/teleop/xrobotoolkit_server.py` → `tcp_server.py`(裁剪后)
- 不留 `from xr_teleoperate ...` 之类的引用

安装方式:在 [install_meta.sh](../install_scripts/install_meta.sh) 第 5 步把"装真 xrobotoolkit_sdk"换成:
```bash
uv pip install -e gear_sonic/utils/teleop/quest_xr_shim
```

---

## 8. 已知限制

| 限制 | 原因 | 缓解 |
|---|---|---|
| 无法蹲/跨步/起身/单腿 | 3PT 法硬伤,与 shim 无关 | 接受 —— 仅采桌面/站立任务 |
| 操作员前倾/弯腰会失真 | Pelvis 是从 Head 反推的,假定身体直立 | 录数据时提示操作员保持直立 |
| 无脚踝/膝盖动作 | 同上 | 同上 |
| 坐标系需校对 | Unity Y-up 左手系 ≠ 机器人 X-forward Z-up 右手系 | 首跑开 `--vis_vr3pt`,看 3D 可视化里手腕/头位与 G1 是否对齐;参考 [`_compute_rel_transform`](../gear_sonic/scripts/pico_manager_thread_server.py#L169) 的 `Q` 矩阵 |

---

## 9. 待办清单

- [ ] 在 `gear_sonic/utils/teleop/quest_xr_shim/` 下建包结构(`pyproject.toml` + 子模块)
- [ ] 拷贝 `xrobotoolkit_server.py` → `tcp_server.py`,裁剪只保留 TCP+JSON 解析
- [ ] 写 `__init__.py`:17 个 API + 安全 fallback(`_safe_mat` 风格)
- [ ] 写 `smpl_fake.py`:24×7 数组构造,Pelvis 派生
- [ ] 写 `coords.py`:Unity → 机器人坐标系(参考 NVIDIA `Q` 矩阵)
- [ ] 改 `pico_manager_thread_server.py` import 一行
- [ ] 加 `--force-vr-3pt` CLI flag 与状态机分支
- [ ] 改 `install_meta.sh`:装 shim 替代真 SDK
- [ ] 实测: 戴头盔双手柄激活,开 `--vis_vr3pt` 验证坐标系
- [ ] 实测: 跑完整 `launch_data_collection.py`,确认 parquet 里 `action.motion_token` 有数据
- [ ] 录一条短 episode,导入 LeRobot 看 schema 与官方一致

---

## 10. 关键文件索引(全部本仓库内)

| 用途 | 路径 |
|---|---|
| NVIDIA VR 输入主程序 | [gear_sonic/scripts/pico_manager_thread_server.py](../gear_sonic/scripts/pico_manager_thread_server.py) |
| `xrt.*` 调用清单 | 同上,grep `xrt\.` |
| 3PT 处理函数 | [`_process_3pt_pose`](../gear_sonic/scripts/pico_manager_thread_server.py#L200) 第 200 行 |
| Unity→机器人坐标转换 | [`_compute_rel_transform`](../gear_sonic/scripts/pico_manager_thread_server.py#L169) 第 169 行 |
| 数据采集启动脚本 | [gear_sonic/scripts/launch_data_collection.py](../gear_sonic/scripts/launch_data_collection.py) |
| ZMQ 消息格式 | [gear_sonic/utils/teleop/zmq/zmq_planner_sender.py](../gear_sonic/utils/teleop/zmq/zmq_planner_sender.py) |
| Shim 待建目录 | `gear_sonic/utils/teleop/quest_xr_shim/`(§9 待办) |
| 安装脚本 | [install_scripts/install_meta.sh](../install_scripts/install_meta.sh) |
| 上游总体调研 | [数据采集调研笔记.md](数据采集调研笔记.md) |
