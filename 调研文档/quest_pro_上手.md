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

**眼睛检查 4 件事**:

1. 抬左手 → G1 左手抬(不是右手)
2. 抬右手 → G1 右手抬
3. 向前走 → G1 向前(不是后/左/右)
4. 转身 → G1 跟着转

**任何一项错位**: Unity↔机器人坐标变换的 `Q` 矩阵需要调,见 [`_compute_rel_transform`](../gear_sonic/scripts/pico_manager_thread_server.py#L169)(`Q = [[-1,0,0],[0,0,1],[0,1,0]]`)。

紧急停止: 再按一次 `A+B+X+Y` → OFF。

---

## 5. 真跑数据采集

坐标系对上后:

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
| 录数据时 parquet 没生成 | run_data_exporter 没起 / .venv_data_collection 缺 | 看 tmux 各窗口报错 |
| 录到的 `action.motion_token` 全零 | encoder ONNX 路径错;输入是无效 SMPL | 看 manager 日志里 token 是否被打印 |

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
