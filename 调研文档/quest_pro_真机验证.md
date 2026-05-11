# Quest Pro 真机验证 — 步骤卡

> 前置:venv 已装 + shim 已修(详见 [quest_pro_上手.md](quest_pro_上手.md))。本文只覆盖 §4 标定 → §6 数据落盘。

主机 IP: **192.168.101.178**(有线)或 **192.168.101.179**(WiFi),都能到。APK 填任一,端口 `63901`,TCP。

---

## 1. 启 manager(终端 A,前台跑)

```bash
cd /home/jg/baseline/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt --force-vr-3pt
```

逐项确认(每一步看到再往下):

| 看见 | 说明 |
|---|---|
| `[quest_xr_shim] Listening for Quest client on tcp://0.0.0.0:63901` | shim 监听 OK |
| `[quest_xr_shim] Quest connected from ('192.168.101.x', PORT)` | APK 连上来了 |
| `[Manager] ZMQ socket bound to port 5556` + `Available modes:` | body data 流过,manager 正常 |
| `[PicoReader] dt_ts: 11.xx ms, fps: ~90` | 帧率正常 |
| **PyVista 窗口弹出** | 三色坐标轴(还没标定,G1 未出现) |

任一步卡死/报错 → Ctrl+C,贴最后 10 行。

---

## 2. 标定 + 坐标系验证(§4 核心)

**站好**(T-pose 或双手自然下垂),**右控制器 A + B 与 左控制器 X + Y 同按**。

触发后立刻:
- terminal 出 `Calibrated.` / 状态机进 `PLANNER_VR_3PT`
- PyVista 里出现 **G1 模型**并跟你手腕动

**眼睛检查 4 件事**(在 PyVista 看 G1):

| 你做的 | G1 应该 | 错了说明 |
|---|---|---|
| 抬左手 | 左手抬 | 左右镜像 |
| 抬右手 | 右手抬 | 左右镜像 |
| 向前走一步 | 朝前移 | 前后/左右轴反 |
| 原地转身 | 跟着转 | yaw 轴方向反 |

**紧急停止**:再按一次 A+B+X+Y → manager 进 OFF。

---

## 3. 4 件事里有错 → 改 Q 矩阵

文件:[gear_sonic/scripts/pico_manager_thread_server.py:169](../gear_sonic/scripts/pico_manager_thread_server.py#L169)

```python
Q = np.array([[-1, 0, 0],
              [ 0, 0, 1],
              [ 0, 1, 0]])
```

Unity(左手系,Y-up) ↔ G1(右手系,Z-up)的坐标变换。

| 现象 | 改哪 |
|---|---|
| 左右手镜像 | 第 1 行的 `-1` 翻成 `1`(或反过来) |
| 前进/后退反 | 第 2 行第 3 列的 `1` 翻成 `-1` |
| 转身方向反 | 行交换 / 加负号,常需要 trial-and-error |

改完保存,Ctrl+C 重启 manager(shim 监听端口会断,APK 自己重连或你手动再点 Start)。

---

## 4. 真录数据(§5)

**先把 §1-§3 全跑通**。Manager 不用先开,launch 脚本会拉起。

```bash
# 终端 A(新)
cd /home/jg/baseline/GR00T-WholeBodyControl
python gear_sonic/scripts/launch_data_collection.py
```

它用 tmux 同时开 `manager` + `run_data_exporter`。tmux 窗口切换:`Ctrl+B` 然后 `0/1/2` 选 pane。

录 **1 条短 episode(< 30 秒)** 先,不要直接录长的:
1. tmux 里看到两个 venv 都 ready
2. 戴头盔,A+B+X+Y 标定
3. 做动作(30s 内)
4. 再按 A+B+X+Y 停录
5. exporter 会写 parquet 到 datasets dir

录完 Ctrl+B then `:kill-session` 退 tmux。

---

## 5. 验 parquet(§6)

```bash
# 找最新一条 episode
EP=$(find ~/datasets ~/lerobot_data /tmp/datasets -name "*.parquet" 2>/dev/null | head -1)
echo "$EP"

source .venv_data_collection/bin/activate
python - <<PY
import pyarrow.parquet as pq, sys
t = pq.read_table("$EP")
print("schema:", t.schema)
print("rows:", t.num_rows)
print("motion_token sample:", t["action.motion_token"][0].as_py()[:8], "...")
PY
```

**期望**:
- `action.motion_token` 是 **64 维 float**
- **不全是 0**(全 0 = encoder ONNX 没跑通或 VR_3PT 全是 sentinel)

---

## 已知坑

| 现象 | 原因 | 处置 |
|---|---|---|
| `is_body_data_available` 一直 False | APK 没发 / 防火墙挡 :63901 | `sudo ufw status` 看,通常不活跃 |
| fps < 30 | 网络抖 / WiFi 弱 / 后台进程抢 CPU | 关掉无关程序;头盔靠路由器近些 |
| G1 手腕在原点抽搐 | head/controller 持续发 sentinel | 头盔追踪丢,重新戴正 |
| 录时 parquet 没生成 | run_data_exporter 没起 | 看 tmux 各 pane 日志 |
| `action.motion_token` 全零 | encoder ONNX 路径错 / 输入是无效 SMPL | manager 日志里看 token 输出 |

---

## 快速命令汇总

```bash
# 启 manager(终端 A)
cd /home/jg/baseline/GR00T-WholeBodyControl && source .venv_teleop/bin/activate && \
  python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt --force-vr-3pt

# 查端口/连接(终端 B)
ss -tn | grep 63901
ip -4 -br addr | grep -v ^lo

# 录数据
python gear_sonic/scripts/launch_data_collection.py

# 杀全部
pkill -f pico_manager_thread_server
pkill -f run_data_exporter
tmux kill-server
```
