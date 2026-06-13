# 真机失败诊断与纠正 Agent 方案

## 一、目标和边界

目标不是做一个只会总结日志的聊天 Agent，而是构建可验证的闭环：

```text
episode -> 时间对齐 -> 阶段切分 -> 异常检测 -> 根因诊断
        -> 纠正建议 -> 安全验证 -> 新 episode -> 经验沉淀
```

第一阶段只做离线 postmortem，不让 Agent 直接控制真机。每条诊断必须给出：

- 失败阶段和首次异常时间。
- 根因类别、置信度和支持/反对证据。
- 可以执行的纠正动作。
- 验证纠正是否有效的指标。
- 信息不足时明确输出 `unknown`，而不是猜测。

当前 `2026-06-12-15-58-12` 只有 4 个 episode（3 failure、1 success），适合
搭建分析系统和 benchmark，不足以训练可靠的通用失败检测模型。

## 二、建议架构

### 1. Episode Builder

读取 LeRobot parquet、视频、outcome、events、inference trace 和进程日志，
统一到以 Unix 时间为主键的 timeline，并保留每路数据的原始时间和数据年龄。

标准输出建议为：

```text
analysis/<run>/episode_000000/
├── timeline.parquet
├── phases.json
├── metrics.json
├── evidence.jsonl
└── report.json
```

### 2. Phase Segmenter

先人工辅助标注任务阶段：

```text
search -> approach -> reach -> grasp -> lift -> transport -> place -> verify
```

每个阶段记录 `start_time`、`end_time`、完成条件和失败条件。后续再用视觉模型、
状态机或时序模型自动切分。

### 3. Deterministic Metric Extractor

优先计算不依赖大模型的指标：

- 推理时延 P50/P95/P99、`selected_start_index` 和 action chunk 剩余长度。
- `action.wbc - observation.state` 的逐关节 RMSE、峰值和持续超阈值时间。
- 关节速度、力矩、温度、电机错误码和 base/torso 倾角峰值。
- action jerk、motion token 跳变和 resume blend 前后不连续性。
- camera/state/SONIC age、image-state delta、state index 跳帧率。
- 各进程 error/exception/timeout，以及进程提前退出。

阈值应从成功 episode 的分布和硬件安全限制得到，而不是由 LLM 自行生成。

### 4. Hybrid Failure Detector

采用三路证据，而不是单一模型：

1. 规则检测器：硬件错误、数据陈旧、通信中断、越界、跟踪误差和失稳。
2. 视觉/进度检测器：目标是否可见、是否抓稳、是否掉落、是否完成阶段目标。
3. 对比检测器：失败 episode 与同任务成功 episode 在相同阶段的指标差异。

输出统一的 `failure_event`：

```json
{
  "time_unix_s": 0.0,
  "phase": "grasp",
  "type": "control.tracking_error",
  "severity": "high",
  "confidence": 0.91,
  "evidence_ids": ["metric:joint_rmse", "video:ego:frame_123"]
}
```

### 5. Root Cause Reasoner

LLM/VLM 只消费压缩后的证据，不直接吞整份原始日志。建议用故障树约束推理：

```text
strategy
latency
control
hardware
perception
system
unknown
```

诊断采用“假设竞争”格式：每个候选根因同时列支持证据、反对证据和缺失证据。
如果不能排除多个根因，输出候选集合，不强行给唯一答案。

### 6. Correction Planner

纠正动作来自审核过的动作库：

| 根因 | 允许建议的纠正 |
|---|---|
| strategy | 增加特定失败状态的纠正示范；调整任务分解；重新微调 |
| latency | 降低推理耗时；缩短 horizon；调整 compensation；限速 |
| control | 检查映射/尺度；调整 WBC；限制动作变化率 |
| hardware | 停止测试；检查故障码、温度、碰撞和执行器 |
| perception | 改相机/光照/标定；补腕部或第三视角；增加视觉数据 |
| system | 修复进程、配置、时钟或通信后复现 |

Agent 不能直接修改安全阈值，也不能未经审核自动执行真机恢复动作。

### 7. Validator 和 Case Memory

每次纠正形成一个 case：

```text
症状 + 证据 + 根因 + 修改 + 验证条件 + 验证结果
```

验证顺序为：离线回放、仿真/影子模式、低速真机、正常真机。只有后续 episode
同时满足成功条件和安全约束，case 才标记为 `verified`。

## 三、第一版失败 taxonomy

建议采用可扩展的层级标签：

- `strategy.wrong_direction`
- `strategy.no_progress`
- `strategy.premature_transition`
- `perception.target_missing`
- `perception.state_misaligned`
- `latency.inference_slow`
- `latency.stale_input`
- `control.tracking_error`
- `control.unstable_action`
- `hardware.motor_fault`
- `hardware.collision_or_fall`
- `system.process_error`
- `system.communication_error`
- `operator.abort`
- `unknown`

每个 episode 还需标注：

- `failure_phase`
- `first_anomaly_time`
- `visible_symptom`
- `root_cause`（可以为 unknown）
- `recoverable`
- `suggested_correction`

## 四、实施顺序

### P0：先完成 benchmark

1. 给现有 4 个 episode 标注阶段、首次异常点、表面症状和候选根因。
2. 补外部第三视角视频、接触/足底信号和统一时钟。
3. 固化 `report.json` schema 和人工复核界面。

### P1：实现离线诊断 MVP

1. 实现 Episode Builder 和指标提取。
2. 实现规则检测器和 success-vs-failure 对比报告。
3. 接入 VLM 做关键帧/短视频的阶段与可见失败判断。
4. 接入 LLM，用结构化证据生成受约束的根因报告。

### P2：扩大数据并学习检测器

按 taxonomy 主动采集失败和人工纠正数据。建议先达到每个主要失败类型至少
20--50 个案例，再比较规则、传统时序模型和 VLA hidden-feature detector。
数据量不足时不要把 episode 随机按帧拆分，必须按 run/场景划分训练和测试集。

### P3：在线监控和恢复

先实现 `continue / slow_down / pause / ask_human / emergency_stop` 五种有限动作。
只有经过离线回放和真机 shadow mode 验证后，才加入自动 backtrack、re-grasp
或重新规划。

### P4：纠正学习闭环

把人工接管或成功恢复片段转成 corrective demonstrations，定期重训 VLA；
使用固定回归集比较成功率、安全违规率、检测提前量和误报率，防止修好一个
case 却破坏其他 case。

## 五、评估指标

- 根因分类 macro-F1 和 `unknown` 校准质量。
- 首次失败发生到检测报警的延迟。
- 真失败报警率、成功 episode 误报率。
- Top-1/Top-3 根因命中率。
- 纠正建议经人工判断可执行的比例。
- 纠正后的成功率提升。
- 每成功恢复一次所需的人工干预次数。
- 安全违规、急停和硬件保护触发次数。

最终指标不是“解释看起来合理”，而是纠正后在独立复现实验中成功率提高。

## 六、文献调研优先级

### 第一组：直接对应本项目

1. [REFLECT: Summarizing Robot Experiences for Failure Explanation and Correction](https://arxiv.org/abs/2306.15724)
   学习多传感器经历如何压缩成层级摘要，再交给 LLM 做失败解释和纠正。
2. [AHA: A Vision-Language-Model for Detecting and Reasoning Over Failures in Robotic Manipulation](https://arxiv.org/abs/2410.00371)
   学习失败数据生成、开放式失败解释，以及视觉失败模型如何服务下游恢复。
3. [SAFE: Multitask Failure Detection for Vision-Language-Action Models](https://arxiv.org/abs/2506.09937)
   学习利用 VLA 内部特征输出在线失败概率，以及 conformal prediction 校准。
4. [Hide-and-Seek in Trajectories: Discovering Failure Signals for VLA Runtime Monitoring](https://arxiv.org/abs/2605.30834)
   学习只有 episode 级标签时，如何定位轨迹中的失败时刻。

### 第二组：在线监控与恢复

5. [DoReMi: Grounding Language Model by Detecting and Recovering from Plan-Execution Misalignment](https://arxiv.org/abs/2307.00329)
   把计划写成可监控约束，并在执行偏离时触发恢复。
6. [Inner Monologue: Embodied Reasoning through Planning with Language Models](https://arxiv.org/abs/2207.05608)
   使用成功检测、场景描述和人类反馈形成闭环规划。
7. [Robots That Ask For Help: Uncertainty Alignment for Large Language Model Planners](https://arxiv.org/abs/2307.01928)
   使用 conformal prediction 决定何时继续、何时请求人工帮助。

### 第三组：从失败中收集纠正数据

8. [HG-DAgger: Interactive Imitation Learning with Human Experts](https://arxiv.org/abs/1810.02890)
   学习在人类接管下收集 novice policy 的危险状态和纠正动作。
9. [IntervenGen: Interventional Data Generation for Robust and Data-Efficient Robot Imitation Learning](https://arxiv.org/abs/2405.01472)
   学习如何从少量人工干预扩增纠正数据。
10. [FailSafe: Reasoning and Recovery from Failures in Vision-Language-Action Models](https://arxiv.org/abs/2510.01642)
    关注失败与可执行恢复动作的成对数据，而不只生成文字解释。

调研每篇论文时统一记录：输入模态、标签成本、是否在线、检测延迟、恢复动作、
真机实验、开源情况，以及能否接入 GR00T N1.7 + SONIC。

## 七、当前最小可行交付

第一版不训练新模型，先交付一个命令：

```bash
python -m gear_sonic.failure_analysis \
  --run outputs/2026-06-12-15-58-12
```

它应生成每个 episode 的 metrics、关键时间线、候选根因和证据引用。人工复核后
写回标签，形成后续学习型 detector 和 correction policy 的训练集。
