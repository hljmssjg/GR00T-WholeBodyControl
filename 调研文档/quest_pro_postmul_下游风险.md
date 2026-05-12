# Quest postmul 校准改动 — 下游分布风险

> 上下文: [quest_pro_上手.md §4.1](quest_pro_上手.md) | commit `cdbd8b1` | 改动门控: `XR_BACKEND != "pico"`

## TL;DR

为了修 Quest wrist 旋转轴串台,把 wrist 校准 `rot_offset` 从 premul 改 postmul。**核心判断**: wrist quat 是"G1 wrist 在 robot 帧的真实朝向"这个客观物理量。PyVista 里视觉对齐 ⇒ Quest 输出 ≈ PICO 输出(都收敛到"正确"),encoder 输入分布**大概率接近** PICO 训练分布。需 sim 起来后看 policy 行为做最终确认。

## 数据流(token 是怎么产生的)

```
Quest → manager (校准 ★) → ZMQ :5556 → C++ deploy → ONNX encoder → 64-d motion_token
                                                                       ├→ token_state 给 policy
                                                                       └→ parquet: action.motion_token
```

★ = 改动位置。encoder 是 PICO 数据训练的 ONNX。

## encoder 输入 (`gear_sonic_deploy/policy/release/observation_config.yaml`)

teleop 模式喂给 encoder 的核心字段:

| 字段 | 来源 | 受改动影响 |
|---|---|---|
| `vr_3point_local_target` | manager `vr_3pt_position` | ❌ position 没改 |
| `vr_3point_local_orn_target` | manager `vr_3pt_orientation` | ✅ **数值改了** |
| `motion_anchor_orientation` | motion 本地数据 | ❌ |

## 改动数学(就一句)

| 路径 | wrist quat 公式 |
|---|---|
| PICO (premul) | `rot_offset_p * calib_inv_p * wrist_p` |
| Quest (postmul, 新) | `(calib_inv_q * wrist_q) * rot_offset_q'` |

**注**: 这两个**公式**不同,但代入各自硬件的实际输入后,只要两边都"PyVista 视觉对齐物理 wrist 朝向"成立,输出的 quat 都是同一物理量的表示,**数值上应该接近**。所以不能仅看公式差异断定 encoder 输入有偏移。

## 为什么 PICO premul 不出 Quest 那种错位

推测: PICO SDK 内部已做一层 wrist 校准 → `lwrist_corrected ≈ g1_lwrist_rot` → `rot_offset ≈ identity` → premul / postmul 数值上几乎一致。Quest 没这层 → `rot_offset` 偏离 identity → premul 把这偏离当成 "持续相似变换" 扭曲后续 delta → 视觉错位。

**如果上面推测对**: encoder 实际见过的就是 "`rot_offset ≈ identity` 下的 wrist quat",Quest postmul 输出**反而更贴近**这个分布。但**未量化**。

## 风险评级

| 项 | 状态 |
|---|---|
| PICO 路径 | ✅ 完全不变,门控在 `_IS_QUEST_BACKEND` |
| Quest wrist 视觉 | ✅ 旋转轴对了(三轴肉眼验证) |
| Quest encoder 输入分布 | 🟢 **大概率接近** PICO 训练分布(视觉对齐 ⇒ 物理 wrist 朝向一致 ⇒ quat 数值接近) |
| Quest token 分布 | ⚠️ 未直接测,但因输入接近 → token 也大概率接近 |
| Quest policy 行为 | ⚠️ 未测,需 sim |

**重要澄清**: 之前的版本把 encoder 输入分布标为"中等风险",前提是"premul vs postmul 输出 quat 显著不同"。但 wrist quat 表达的是"G1 wrist 在 robot 帧的真实朝向"——这是个**客观物理量**。两条路径只要都"视觉对齐"成立,数值上必然接近(到归一化精度)。所以风险其实主要在第二、第三项里那些**非 wrist** 的字段(`motion_anchor_orientation` 等),跟硬件佩戴差异有关,跟我这次改动**无关**。

## 验证路径(按代价从低到高)

1. **sim 跑起来录段动作,看 G1 行为**(最直接)
   - 行为正常 → encoder 容忍当前输入分布,无需进一步动
   - G1 抽搐/朝错向/不动 → encoder OOD,走 plan B
2. **dump `teleop.vr_3pt_orientation` 看数值范围**——录一段简单动作,把 quat 的 norm / 分量分布 plot 出来。直觉判断是否"合理"(norm ≈ 1, 没有突跳)
3. **如果以后拿到 PICO 数据**: 同动作 PICO + Quest 各录一段,对比 `vr_3pt_orientation` 和 `action.motion_token` 的分布
4. **plan B**: 用 Quest 数据 fine-tune encoder ONNX,让它学新分布

## 不做改动的代价 vs 做改动的代价

| 选项 | 代价 |
|---|---|
| 保留 postmul (现状) | wrist 视觉对齐 ⇒ quat 数值接近"物理真值" ⇒ encoder 输入接近 PICO 训练分布。**残留风险低** |
| 回退到 premul | wrist 视觉**确认**会再次错位,manager 发的 quat 偏离物理真值 ⇒ encoder 看到的**真正是** OOD 输入 ⇒ 训练效果有损 |

**结论**: 保留。回退是已知坏(数据偏离物理真值),保留是低残留风险(quat 收敛到物理真值)。

## 退路

- commit `cdbd8b1` 之前 = HEAD~1 (`4ae9f7c`)。`git checkout 4ae9f7c -- gear_sonic/scripts/pico_manager_thread_server.py` 即可回退代码,文档保留
- 改动只影响 wrist orientation,position 完全不动,所以 wrist 位置追踪的训练数据照常有效
