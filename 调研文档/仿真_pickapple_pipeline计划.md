# 仿真 Pick-Jug Pipeline 计划（N1.7 + SONIC）

> 目标：在 MuJoCo 仿真里完整跑通 **数采 → 微调 → 部署** 三步，
> Task = "走到桌前，抓水壶，转身，走回起点"。
> 给 VLA 的 prompt 用**单意图短句**（`fetch the jug back to start`），完整序列靠 demo + 视觉锚点学。
> 约束：**尽量不改官方代码**。
>
> **2026-05-18 物体迭代**：苹果（球形）抓起来太滑、太小，G1 dex hand 摩擦不够。
> 改用 **robocasa locomanip/jug_a01**（带把手的塑料水壶），repo 内现成 mesh + texture，
> 把手让 G1 可以勾握，grasp 一次成功率高得多。pick-apple 版 xml 保留作 fallback。

---

## 0. 用户已确认的选项

| 决策点 | 选择 |
|---|---|
| 场景切换方式 | **直接改 YAML 的 `ROBOT_SCENE` 一行**（最小侵入） |
| Teleop 设备 | **Quest Pro**（复用本仓 `gear_sonic/utils/teleop/quest_xr_shim/`） |
| 抓取目标物 | **robocasa `jug_a01`**（塑料水壶，带把手；replaces earlier apple，因 grasp 难度太高） |
| 抓取摩擦 | `friction="3.0 0.10 0.005"` + `condim` 默认（box 碰撞已经覆盖滑/扭/滚） |
| 起点随机化 | **固定起点**（最简） |
| 视觉锚点 | **在起点地面加一个色块 geom**（帮 VLA 学"什么时候停"） |

---

## 1. 当前仓库链路（不改）

```
run_sim_loop.py  ──> SimulatorFactory  ──> DefaultEnv.init_scene()
                                              │
                                              └──> 读 config["ROBOT_SCENE"]
                                                     │
                                                     ▼
                            gear_sonic/utils/mujoco_sim/wbc_configs/
                                g1_29dof_sonic_model12.yaml
                                  ROBOT_SCENE: "...scene_43dof.xml"
                                                     │
                                                     ▼
                       gear_sonic/data/robot_model/model_data/g1/
                          scene_43dof.xml  =  <include robot.xml> + 地面 + 灯
                                              ↑
                                         "empty world"
```

关键事实：
- `scene_43dof.xml` 只是 **wrapper**，靠 `<include file="g1_29dof_with_hand.xml"/>` 引入机器人本体。
- 数据采集只录 **机器人状态 + ego 相机 + teleop SMPL pose**，不录物体姿态 → 加物体不会破坏 schema。
- `launch_data_collection.py --sim` 和 `launch_inference.py --sim` 都会自动起 `run_sim_loop.py`，所以**只要换它读的 scene，整个 pipeline 自动跟着切**。

---

## 2. 文件改动清单

| # | 文件 | 类型 | 内容 |
|---|------|------|------|
| 1 | `gear_sonic/data/robot_model/model_data/g1/scene_43dof_pick_jug.xml` | **新建（当前）** | wrapper：`<include robot.xml>` + 桌子 + jug (mesh+collision boxes) + 起点锚点 |
| 2 | `gear_sonic/data/robot_model/model_data/g1/scene_43dof_pick_apple.xml` | **保留 fallback** | 早期苹果版本，jug 不稳就回退 |
| 3 | `gear_sonic/data/robot_model/model_data/g1/meshes/jug_a01.obj` | **新增（拷贝）** | 从 `decoupled_wbc/dexmg/gr00trobocasa/.../jug_a01/` 拷过来，~416KB |
| 4 | `gear_sonic/data/robot_model/model_data/g1/meshes/Jug_A.png` | **新增（拷贝）** | jug 贴图，~300KB |
| 5 | `gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml` | **改1行** | `ROBOT_SCENE:` 改指向 #1，老路径以注释保留可一键回退 |

> Python 代码零变更。所有改动都在 asset / config 层。
> 资产为什么拷而不是 include：被引 xml `g1_29dof_with_hand.xml` 里设了 `meshdir="meshes"`，
> mesh 路径相对 meshdir，texture 路径相对 scene xml 自身目录，两个基准不一样，相对路径会很难写。

---

## 3. Scene 设计细节

### 3.1 布局（俯视）

```
                  Y
                  │
                  │
        ┌────────┐│
        │ table  ││   桌面中心 x = 3.00, y = 0  (桌前缘 x=2.70)
        │ 🧴 jug ││   jug @ (2.85, 0, 0.73)，落桌后稳定在 z≈0.72
        │        ││
        └────────┘│
        ▒▒▒▒▒▒▒▒▒▒│
        ▒▒▒▒▒▒▒▒▒▒│  ← carpet runner @ x ∈ [-0.05, 2.65], y ∈ [±0.40]
        ▒▒▒▒▒▒▒▒▒▒│    2.7m × 0.8m warm-tone checker（15×4 格 ~18cm/格）
        ▒▒▒▒▒▒▒▒▒▒│    teleop 演示轨道 + VLA 中距 scale
        ▒▒▒▒▒▒▒▒▒▒│
        ▒▒robot▒▒▒│    robot @ origin, 朝 +X 时正好踩在毯子近端
        ▒▒▒▒▒▒▒▒▒▒│
                  │
   ░░░░anchor░░░░ │   x = -0.15, y = 0   起点地砖锚点（远距方向参考）
                  │
   ▓▓▓▓ wall ▓▓▓▓ │   x = -0.60, y ∈ [-0.75, +0.75], z ∈ [0, 1.5]
                  │   起点后方 cyan/white checker 墙 + 中心 30cm 红靶
                  ▼ ──────► X

robot ↔ 桌前缘 = 2.70m (= 3× 原方案 0.90m,**长走任务**)
```

### 3.2 物体参数

| 物体 | 尺寸 | 质量 | 材质 / 颜色 | 用途 |
|---|---|---|---|---|
| 桌面 (box) | 0.6 × 0.6 × 0.04 m，顶面 z=0.72 | 静态 | 木色 | 任务平面 |
| 桌腿 (4 × box) | 0.04 × 0.04 × 0.70 m | 静态 | 木色 | 撑桌 |
| **Jug** (mesh + 6 box col) | scale=0.7 → ~6.7cm 基底直径 × 19.6cm 高，带"把手"（实为壶嘴） | 0.12 kg | 塑料贴图（`Jug_A.png`），friction `3.0 0.10 0.005` | 抓取目标（**bimanual pinch** 策略） |
| 起点地面锚点 (box) | 0.20 × 0.20 × 0.001 m，z≈0 | 静态、`contype=0 conaffinity=0` | 鲜艳青色 | 远距方向参考（近距相机盲区时失效） |
| **起点立墙** (box) | 0.05 × 1.5 × 1.5 m，pos (-0.60, 0, 0.75) | 静态、默认碰撞 | cyan/white **checker 5×5**（每格 30cm，提供 scale 特征） | 近距停止信号 + 距离估计 |
| **墙心红靶** (box) | 0.006 × 0.30 × 0.30 m，墙面前 3mm | 静态、无碰撞 | 纯红 | 单一 fiducial，角度大小单调对应距离（3m→5.7°，0.3m→53°） |
| **行走地毯** (box) | 2.7 × 0.8 × 0.002 m，pos (1.30, 0, 0.001) | 静态、无碰撞 | 暖色（棕/橙）checker 15×4（每格 ~18-20cm） | 走路轨道：teleop 演示边界 + VLA 学边缘对齐 |

Jug 挂 `<freejoint/>`，初始位 `(1.05, 0.0, 0.73)`（落桌后稳定在 z≈0.72）。
碰撞用 6 个 box 近似 robocasa 原版（3 个旋转 60° 拼六边形 + 把手 + 颈 + 盖），
visual 用 `jug_a01.obj` mesh + `Jug_A.png` 贴图（`group="1"`），collision boxes `group="3"`（viewer 默认不画）。

**关键设计修订**：
- 经 mesh 分析，jug_a01 的"把手"实际是**壶嘴**（spout），间隙 ~9mm，手指穿不过去 → **改 bimanual pinch grasp**（双掌从 ±Y 夹 body 中段，friction 3.0 完全够托 0.12kg），不依赖把手。
- jug 已 scale 到 0.7（更适配 G1 手距），handle/spout 朝外朝内不影响 ±Y 夹持。

**起点立墙设计原因**：
- G1 ego camera 在 ~1.2m 高度，**地面 ~1m 内是视觉盲区**。robot 走回起点时，floor anchor 在最后 1m 已掉出画面。
- 立墙 1.5m × 1.5m at x=-0.60，robot 站原点朝 -X 时墙占满 ~100° HFOV，从任意距离都看得见。
- **纯色墙不行**（近距 robot 只看到一片均匀 cyan，没有 scale 信息）→ 改成 **cyan/white checker（5×5）**：
  每格 30cm，VLA 数像素就能反推距离。
- **墙中心叠加一个 30cm 红色靶心**：单一 fiducial，强信号；从 3m 远占 5.7° HFOV 到 0.3m 处占 53° HFOV，
  尺寸单调对应距离，VLA 学一个 monotonic mapping 即可。
- 三层冗余：远距方向（floor anchor）→ 中距 scale（checker）→ 近距 fiducial（红靶）。

**地毯设计原因**：
- teleop 演示时给操作员**明确轨道**（沿地毯走），50 条 demo 一致性高，VLA 容易学
- VLA 学到的是"两侧地毯边缘对称 = 在轨道正中"——简单的左右对齐误差信号
- 地毯近端 x=-0.05 紧贴起点 anchor 远端 x=-0.05，**robot 站原点正好踩在毯子第一行格子上**，
  起步即"上轨道"，停止即"下轨道"，VLA 学到清晰的状态切换边界
- 暖色 checker（棕/橙）跟冷色墙（cyan/白）+ 冷色 anchor（cyan）形成**冷暖对比**，VLA 容易区分。

### 3.3 桌子高度与距离的取舍

- 标准餐桌 75cm，但 G1 直立高度 ~130cm，原地大幅弯腰可能让 SONIC walker 不稳。
- **桌面 z=0.72**（先用，观察弯腰稳定性。不稳就降到 65cm）。
- jug 把手在桌面之上 ~0.14m 处（scale=0.7 后），世界 z≈0.86，robot 直立伸手可达。
- **走距 2.70m**（2026-05-18 改）：SONIC walker 单程约 ~5-6s，往返加抓握约 15-25s/demo。
  长走距让任务更接近真实场景，但 walker 累计漂移风险增大 —— M4 验证时若失稳，回滚到 1.5m。

---

## 4. 任务定义

**Prompt 给 VLA（数采和部署用同一个）**：
```
fetch the jug back to start
```

> 选短句的原因：N1.7 训练数据 prompt 几乎都是单原子动作；多段 chained instruction（`walk ..., pick ..., turn ..., return ...`）落到 OOD，且 action chunk ~1s 没有 stage 状态，模型不会自己做阶段推进。完整子任务序列**完全靠 demo + 视觉锚点学**：

1. 从起点出发 → 走到桌前
2. 弯腰 / 伸手 → jug 把手
3. 抓握 jug（dex hand 勾把手）
4. 直起身、转身 180°
5. 走回起点（看到锚点色块）
6. 站定

> 备选方案（当前打法失败再考虑）：录制时按阶段切 prompt（`walk to the table` → `pick up the jug` → `turn around` → `walk back`），推理时加 orchestrator 切 prompt。工作量大，先不上。

---

## 5. 完整执行 Pipeline

### 5.1 数采（sim + Quest Pro）

```bash
# 一键起 4 个 tmux pane + sim 窗口
python gear_sonic/scripts/launch_data_collection.py \
    --sim \
    --task-prompt "fetch the jug back to start"
```

**期望 pane 布局**（已存在的 launcher 行为）：
```
┌──────────────────┬──────────────────┐
│ C++ Deploy       │ Data Exporter    │
│ (sim mode)       │ (.venv_data_…)   │
├──────────────────┼──────────────────┤
│ PICO/Quest Teleop│ Camera Viewer    │
│ (.venv_teleop)   │ (.venv_data_…)   │
└──────────────────┴──────────────────┘
+ window 1: MuJoCo sim loop
```

**Quest Pro 接入点**：仓内已有 `gear_sonic/utils/teleop/quest_xr_shim/`，复用现有 `pico_manager_thread_server.py` 路径，pane 内 `XR_BACKEND=quest` 起即可（具体看你之前的 commit `4ae9f7c` 加的 shim）。

**录制控制**：
| 按键 | 行为 |
|---|---|
| Quest 左 Grip + A | 开始 / 结束 episode 并保存 |
| Quest 左 Grip + B | 丢弃当前 episode |
| `c` / `x` (keyboard pub @ 5580) | 同上 |

**目标量**：50 条 demo，每条 30–60s（起点出发 → 完整回起点）。

**输出**：`outputs/<timestamp>-G1-...` 标准 LeRobot v2.1 目录。

### 5.2 后处理

**Step 1 — 浏览 & 标记坏 episode**（可选；采集时按 `x` 也会自动标）

```bash
python gear_sonic/scripts/run_episode_browser.py \
    --dataset outputs/<dataset>
# 浏览器打开 http://127.0.0.1:8765/，每页 30 条，✕ 按钮把 ep 写入
# info.json:discarded_episode_indices（不动文件，可撤销）
```

**Step 2 — 清洗 + 丢弃 discarded**

> ⚠️ **VR-3pt 模式（Quest Pro `stream_mode=5`）必须加 `--no-remove-stale-smpl`**。
> 这种模式 `teleop.smpl_pose` 全零是正常的（没在跑全身 SMPL 流），默认开的 SMPL 清洗会把每条 ep 都判为 stale 全部删光，最后报 `No valid episodes after processing.`。
> SMPL 全身流（Pico 等）才用默认。

```bash
# VR-3pt 模式（Quest Pro 3 点）：
.venv_data_collection/bin/python gear_sonic/scripts/process_dataset.py \
    --dataset-path outputs/<dataset> \
    --no-remove-stale-smpl

# SMPL 全身流模式：
.venv_data_collection/bin/python gear_sonic/scripts/process_dataset.py \
    --dataset-path outputs/<dataset>
```

不论哪种，会：
- 跳过 `discarded_episode_indices` 里的 ep（`--drop-discarded`，默认 on）；
- （仅 SMPL 模式）移除 `teleop.smpl_pose` 全零帧 + 冻结 lead-in；
- 重新编号 0..N-1，重写 `episodes.jsonl` / `episodes_stats.jsonl` / `info.json`。

非原地输出（保留原始数据）：加 `--output-path outputs/<dataset>_clean`。

多次 session 合并：
```bash
.venv_data_collection/bin/python gear_sonic/scripts/process_dataset.py \
    --dataset-path outputs/s1 outputs/s2 outputs/s3 \
    --output-path outputs/merged
```

跑完得到的目录就是 §5.3 的 `--dataset-path` 入参。

### 5.3 微调（外部 Isaac-GR00T repo，不动本仓）

```bash
cd Isaac-GR00T
export NUM_GPUS=4
uv run python gr00t/experiment/launch_finetune.py \
    --base-model-path nvidia/GR00T-N1.7-3B \
    --dataset-path /abs/path/to/outputs/<merged-dataset> \
    --embodiment-tag UNITREE_G1_SONIC \
    --modality-config-path gr00t/configs/data/embodiment_configs.py \
    --num-gpus $NUM_GPUS \
    --output-dir /abs/path/to/output \
    --save-steps 5000 --max-steps 20000 \
    --use-wandb --global-batch-size 32 \
    --color-jitter-params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
    --dataloader-num-workers 4
```

### 5.4 部署（同一个 sim 场景）

GPU 机：
```bash
cd Isaac-GR00T
uv run python gr00t/eval/run_gr00t_server.py \
    --model-path /abs/path/to/output/checkpoint-20000 \
    --embodiment-tag UNITREE_G1_SONIC \
    --device cuda:0 --port 5550
```

本机：
```bash
python gear_sonic/scripts/launch_inference.py \
    --sim \
    --policy-host <gpu_ip> --policy-port 5550 \
    --prompt "fetch the jug back to start"
```

复用同一个 scene xml + WBC yaml → 训练 / 部署视觉分布一致。

---

## 6. 验证里程碑（按这个顺序逐项过）

| # | 验证项 | 命令 | 通过标准 |
|---|---|---|---|
| M1 | scene xml 能加载 | `python gear_sonic/scripts/run_sim_loop.py --enable-onscreen` | MuJoCo 窗口打开，能看到 G1 + 桌子 + jug + 起点锚点 |
| M2 | jug 物理稳定 | 同上，让 sim 跑 30s 不操作 | jug 不穿透桌面、不飞走（已离线验证：2s 内落桌后 z=0.7196 稳定） |
| M3 | ego 相机能看见 jug | `--enable-image-publish --enable-offscreen` + camera viewer | ego_view 流里有 jug 和桌子 |
| M4 | Quest Pro teleop 走到桌前 | `launch_data_collection.py --sim` | 能 teleop 走到 0.9m 外（桌前缘） |
| M5 | 抓起 jug | M4 基础上尝试勾把手 | jug 离开桌面、跟随手运动 ≥ 3s |
| M6 | 录 5 条 demo 跑通后处理 | `process_dataset.py` | parquet + mp4 完整、modality.json 合法 |
| M7 | 录满 50 条 | 同 M4-M6 | 数据集 ready for finetune |
| M8 | 微调 loss 下降 | Isaac-GR00T wandb | loss 单调下降并 plateau |
| M9 | sim 部署完整走通 | `launch_inference.py --sim` | 自主完成完整序列 ≥ 1 次 |

---

## 7. 已知风险 + 兜底

| 风险 | 影响 | 兜底 |
|---|---|---|
| G1 dex hand 在 MuJoCo 不稳定（`scene_43dof.xml` 注释明说） | M5 抓不稳 | 加 mocap weld 假抓（~50 行辅助脚本，新建文件，不改官方） |
| jug 把手 collision 是单 box，可能勾不住 | M5 抓不稳 | 提高把手 friction 到 5.0；或把手 box 改成更细长 |
| SONIC walker 弯腰过深失衡 | M4 走到桌前就倒 | 桌面降到 65cm；或把任务改成"碰一下"先打通（jug 高 19.6cm 缩放后，把手中点在桌面之上 14cm，弯腰需求其实不大） |
| **2.7m 走距累计漂移** | M4 走到桌前已偏离碰不到 jug | 回滚到 1.5m；或加 carpet 中心线视觉强化 |
| 50 条 demo 不够泛化（固定起点） | M9 部署失败 | 加随机化（jug x/y ±10cm）补录 50 条 |
| Quest Pro shim 在 sim 路径偶发卡顿 | 录制中断 | 切回 PICO，或检查 `XR_BACKEND` 环境变量 |
| 转身回来"什么时候停"难学 | 部署时走过头 | **三层冗余**：地面 anchor（远距方向）+ 1.5m cyan/white checker 墙（中距 scale）+ 30cm 中心红靶（近距 fiducial） |

---

## 8. 第一步动手范围（M1 验证为止）—— **已完成 2026-05-18**

DONE:
1. ✅ 写 `scene_43dof_pick_jug.xml`（含桌子 + jug freejoint + 起点锚点）
2. ✅ 拷 `jug_a01.obj` + `Jug_A.png` 到 `meshes/`
3. ✅ 改 `g1_29dof_sonic_model12.yaml` 第 3 行 `ROBOT_SCENE:` 指向 pick_jug，老路径以注释保留
4. ✅ 离线 mujoco 加载 + 2s 物理仿真验证（jug 落桌后稳定 z=0.7196，无穿透）

下一步：
```bash
python gear_sonic/scripts/run_sim_loop.py --enable-onscreen
```
**视觉确认**桌子、jug（含贴图）、起点锚点位置都合理后，进 M3（ego 相机看 jug）。

---

## 9. 附：回退方式

随时回退，按需切 yaml 第 3 行 `ROBOT_SCENE:`：
- 回到 **苹果版**：`scene_43dof_pick_apple.xml`（jug 抓不稳时）
- 回到 **empty world**：`scene_43dof.xml`（彻底排查时）

三个 xml 都保留在仓库，互不影响。`meshes/jug_a01.obj` 和 `meshes/Jug_A.png` 只被 pick_jug.xml 引用，回退后是死文件但不影响加载。
