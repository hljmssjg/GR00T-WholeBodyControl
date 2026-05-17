# 仿真 Pick-Apple Pipeline 计划（N1.7 + SONIC）

> 目标：在 MuJoCo 仿真里完整跑通 **数采 → 微调 → 部署** 三步，
> Task = "走到桌前，抓苹果，转身，走回起点"。
> 约束：**尽量不改官方代码**。

---

## 0. 用户已确认的选项

| 决策点 | 选择 |
|---|---|
| 场景切换方式 | **直接改 YAML 的 `ROBOT_SCENE` 一行**（最小侵入） |
| Teleop 设备 | **Quest Pro**（复用本仓 `gear_sonic/utils/teleop/quest_xr_shim/`） |
| 抓取方案 | **先按原样试**：苹果做大、加重、加高摩擦 |
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
| 1 | `gear_sonic/data/robot_model/model_data/g1/scene_43dof_pick_apple.xml` | **新建** | wrapper：`<include robot.xml>` + 桌子 + 苹果 + 起点锚点 |
| 2 | `gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml` | **改1行** | `ROBOT_SCENE:` 改指向 #1，老路径以注释保留可一键回退 |

> Python 代码零变更。两次改动都在 asset / config 层。

---

## 3. Scene 设计细节

### 3.1 布局（俯视）

```
                  Y
                  │
                  │
        ┌────────┐│
        │ table  ││   x = 0.85, y = 0   （桌子前缘距 robot ~0.55m）
        │  ●apple││
        │        ││
        └────────┘│
                  │
   ░░░░anchor░░░░ │   x = -0.15, y = 0   起点地砖锚点
                  │
              robot @ origin, 朝 +X
                  │
                  ▼ ──────► X
```

### 3.2 物体参数

| 物体 | 尺寸 | 质量 | 材质 / 颜色 | 用途 |
|---|---|---|---|---|
| 桌面 (box) | 0.6 × 0.6 × 0.04 m，顶面 z=0.72 | 静态 | 木色 | 任务平面 |
| 桌腿 (4 × box) | 0.04 × 0.04 × 0.70 m | 静态 | 木色 | 撑桌 |
| **苹果** (sphere) | r = 0.04 m（直径 8cm，比真苹果略大） | 0.08 kg | 红，friction `2.0 0.05 0.001` | 抓取目标 |
| 起点锚点 (box) | 0.20 × 0.20 × 0.001 m，z≈0 | 静态、`contype=0 conaffinity=0` | 鲜艳青色 | 纯视觉标记，不参与物理 |

苹果挂 `<freejoint/>`，初始位 `(0.85, 0.0, 0.78)`。

### 3.3 桌子高度的取舍

- 标准餐桌 75cm，但 G1 直立高度 ~130cm，原地大幅弯腰可能让 SONIC walker 不稳。
- **先用 72cm 桌面**（z=0.72），观察弯腰稳定性。不稳就降到 65cm。

---

## 4. 任务定义

**Prompt 给 VLA（数采和部署用同一个）**：
```
walk to the table, pick up the apple, turn around, return to start
```

子阶段不写到代码里，**完全靠 prompt + 视觉 + demo 学**：

1. 从起点出发 → 走到桌前
2. 弯腰 / 伸手 → 苹果
3. 抓握（dex hand）
4. 直起身、转身 180°
5. 走回起点（看到锚点色块）
6. 站定

---

## 5. 完整执行 Pipeline

### 5.1 数采（sim + Quest Pro）

```bash
# 一键起 4 个 tmux pane + sim 窗口
python gear_sonic/scripts/launch_data_collection.py \
    --sim \
    --task-prompt "walk to the table, pick up the apple, turn around, return to start"
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

```bash
# 去除 SMPL 卡顿帧
python gear_sonic/scripts/process_dataset.py \
    --dataset-path outputs/<dataset>
```

可选：多次 session 合并：
```bash
python gear_sonic/scripts/process_dataset.py \
    --dataset-path outputs/s1 outputs/s2 outputs/s3 \
    --output-path outputs/merged
```

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
    --prompt "walk to the table, pick up the apple, turn around, return to start"
```

复用同一个 scene xml + WBC yaml → 训练 / 部署视觉分布一致。

---

## 6. 验证里程碑（按这个顺序逐项过）

| # | 验证项 | 命令 | 通过标准 |
|---|---|---|---|
| M1 | scene xml 能加载 | `python gear_sonic/scripts/run_sim_loop.py --enable-onscreen` | MuJoCo 窗口打开，能看到 G1 + 桌子 + 苹果 + 起点锚点 |
| M2 | 苹果物理稳定 | 同上，让 sim 跑 30s 不操作 | 苹果不穿透桌面、不飞走 |
| M3 | ego 相机能看见苹果 | `--enable-image-publish --enable-offscreen` + camera viewer | ego_view 流里有苹果和桌子 |
| M4 | Quest Pro teleop 走到桌前 | `launch_data_collection.py --sim` | 能 teleop 走到 0.85m 外 |
| M5 | 抓起苹果 | M4 基础上尝试抓 | 苹果离开桌面、跟随手运动 ≥ 3s |
| M6 | 录 5 条 demo 跑通后处理 | `process_dataset.py` | parquet + mp4 完整、modality.json 合法 |
| M7 | 录满 50 条 | 同 M4-M6 | 数据集 ready for finetune |
| M8 | 微调 loss 下降 | Isaac-GR00T wandb | loss 单调下降并 plateau |
| M9 | sim 部署完整走通 | `launch_inference.py --sim` | 自主完成完整序列 ≥ 1 次 |

---

## 7. 已知风险 + 兜底

| 风险 | 影响 | 兜底 |
|---|---|---|
| G1 dex hand 在 MuJoCo 不稳定（`scene_43dof.xml` 注释明说） | M5 抓不稳 | 加 mocap weld 假抓（~50 行辅助脚本，新建文件，不改官方） |
| SONIC walker 弯腰过深失衡 | M4 走到桌前就倒 | 桌面降到 65cm；或把任务改成"碰一下"先打通 |
| 50 条 demo 不够泛化（固定起点） | M9 部署失败 | 加随机化（苹果 x/y ±10cm）补录 50 条 |
| Quest Pro shim 在 sim 路径偶发卡顿 | 录制中断 | 切回 PICO，或检查 `XR_BACKEND` 环境变量 |
| 转身回来"什么时候停"难学 | 部署时走过头 | 锚点色块已经设计好；不够就再加墙面 marker |

---

## 8. 第一步动手范围（M1 验证为止）

只做两件事，不碰 teleop / 数采：

1. 写 `scene_43dof_pick_apple.xml`：
   - `<include file="g1_29dof_with_hand.xml"/>`
   - 抄 `scene_43dof.xml` 的 visual / asset / 地面
   - 加 table body（5 个静态 box）
   - 加 apple body（freejoint + sphere）
   - 加起点 anchor geom（薄 box，禁用碰撞）
2. 改 `g1_29dof_sonic_model12.yaml` 第 3 行 `ROBOT_SCENE:` 指向新文件，老路径以注释保留。

完成后跑：
```bash
python gear_sonic/scripts/run_sim_loop.py --enable-onscreen
```
**视觉确认**桌子、苹果、起点锚点位置都合理后，再继续 M2 之后。

---

## 9. 附：回退方式

随时回 empty world：把 yaml 里的 `ROBOT_SCENE:` 改回 `scene_43dof.xml` 一行即可。新 xml 文件留着不删，不影响任何东西。
