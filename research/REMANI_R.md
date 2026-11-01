# REMANI-R

## 定义

REMANI-R 是面向整体式移动机械臂的可恢复性感知时空规划系统。

它在 REMANI 的 Hybrid A*、机械臂采样、MINCO 轨迹和 L-BFGS
优化框架上，加入四项闭环能力：

1. 底盘制动距离约束；
2. 六轴机械臂回收时间约束；
3. 轨迹到达时刻动态障碍查询；
4. 执行期安全审计、时间修复和时间压缩。

## 系统链路

```text
Hybrid A* / RRT
      ↓
MINCO 整体式轨迹
      ↓
L-BFGS 空间-时间联合优化
      ↓
0.01 s 碰撞 + 可恢复性全检
      ↓
FSM 执行期滚动审计
      ↓
局部重规划 / 安全时间修复 / 急停
```

## 核心创新

### 1. 整体制动包络

底盘速度、控制延迟和机械臂展开度共同决定停车距离：

```text
stop_distance = v² / (2a) + |v|dt
whole_body_stop_distance =
    stop_distance × (1 + 0.25 × arm_extension)
```

机械臂越展开，机器人需要预留越大的制动空间。

### 2. 机械臂恢复时间流形

每个轨迹状态同时检查底盘停车和机械臂回收：

```text
stop_time = |v| / a + dt
arm_time  = max_i time(q_i → q_i_recovery)
slack     = recovery_horizon - stop_time - arm_time
```

FastArmer 的回收姿态：

```text
[0.0, -1.2217, 1.9199, 0.0, -1.9199, 0.0]
```

该约束把“当前不碰撞”升级为“中途任何时刻都能停车并收回机械臂”。

### 3. 轨迹到达时刻动态障碍

动态球按恒速模型预测：

```text
center(t) = center_0 + velocity × (t - t_0)
distance(p, t) = ||p - center(t)|| - radius
```

优化器查询机器人到达每个轨迹采样点时的障碍位置，并将静态 ESDF
距离与动态球距离取最小值。动态距离和空间梯度均为解析计算，动态
查询直接进入现有 ESDF 查询层。

### 4. 路径保持型时间修复

轨迹路径安全、仅可恢复性不足时，R 从首次失败的多项式段开始拉长时间：

```text
coefficient_k *= stretch^k
duration      /= stretch
```

该方法保持路径几何及多项式连续性，同时降低速度和加速度。
随后在 `0.05 s` 粗检下压缩膨胀的时间分配，最终轨迹必须通过
`0.01 s` 全检。

## 性能工程

- 静态场景优化预算：`80 ms`
- 动态场景优化预算：`60 ms`
- 稀疏优化采样：每段 `12` 点
- 低速区加密采样：`8` 点
- 滚动重规划复用剩余优化轨迹
- A*、碰撞检测和梯度计算复用预分配缓冲区
- 大静态点云限量发布
- 动态障碍采用解析距离查询
- 无可视化订阅者时跳过 mesh 和前端可视化构造

## 实测结果

### 最终动态回归

20 个地面障碍、30 个悬浮障碍、两个横穿球体，seed 30-34：

| 指标 | REMANI-R |
|---|---:|
| 任务成功 | 5/5 |
| 平均规划时间 | 276.8 ms |
| P95 规划时间 | 357.7 ms |
| 平均轨迹时长 | 18.72 s |
| 安全指标违规 | 0 |
| 运行时重规划 | 0 |
| 急停 | 0 |

### 与原版 REMANI 同三 seed 对比

| 方法 | 平均规划时间 | P95 | 安全违规 | 急停 |
|---|---:|---:|---:|---:|
| REMANI | 214.0 ms | 289.5 ms | 9 | 1 |
| REMANI-R | 258.4 ms | 305.5 ms | 0 | 0 |

### 三模式消融

| 模式 | 最差制动余量 | 最差恢复时间余量 |
|---|---:|---:|
| 原版 REMANI | -0.2004 m | -0.9854 s |
| 制动包络 | -0.0577 m | -0.6833 s |
| 制动 + 恢复时间流形 | -0.0518 m | -0.5051 s |
| Full REMANI-R | -0.0577 m | -0.0235 s |

### 最终静态回归

| 场景 | 成功 | 平均规划时间 | P95 | 安全违规 |
|---|---:|---:|---:|---:|
| `exp0`，5 seeds | 5/5 | 243.7 ms | 286.4 ms | 0 |
| `exp1_random`，5 seeds | 5/5 | 235.3 ms | 293.7 ms | 0 |

## 公平动态对比

三方使用同一张 8 × 8 m 地图、同一机器人、同一起终点、同一物体
位置/半径/速度轨迹。原版 REMANI 和 TopAY 使用当前位置，REMANI-R
使用位置与速度预测未来到达时刻。

| 方法 | 动态模型 | 完成 | 急停 | 结果 |
|---|---|---:|---:|---|
| 原版 REMANI | 当前位置 | 2/2 | 1 | 反应式重规划后恢复 |
| REMANI-R | 位置 + 速度预测 | 2/2 | 0 | 提前避让 |
| TopAY | 当前占据地图 | 0/2 | 0 | 无可行初始拓扑路径 |

公平版视频：

- `research/media/fair_original_dynamic.mp4`
- `research/media/fair_remani_r_dynamic.mp4`
- `research/media/fair_topay_dynamic.mp4`

协议文件：

- `research/docs/fair_dynamic_benchmark.md`

## 外部基线

TopAY 原生 8 × 8 m、100 个随机案例：

| 指标 | 结果 |
|---|---:|
| 成功案例 | 52/100 |
| 成功案例平均规划时间 | 122.17 ms |
| 成功案例平均轨迹时长 | 8.90722 s |

GCDF 公开同协议结果在 120 个障碍下为：GCDF `86.8% / 0.2871 s`，
GCDF + TopAY init `91.6% / 0.1957 s`。数据表见
`research/external/GCDF_COMPARISON.md`。

## 对外表述

```text
REMANI-R 将移动机械臂的停车距离、机械臂回收时间、
轨迹到达时刻动态障碍、运行时安全审计和时间修复，
统一到 REMANI 的整体式规划闭环中。
```

## 入口

- `remani_planner/plan_manage/config/remani_planner_param.yaml`
- `remani_planner/plan_env/src/grid_map.cpp`
- `remani_planner/traj_opt/src/poly_traj_optimizer.cpp`
- `remani_planner/plan_manage/src/remani_replan_fsm.cpp`
- `remani_planner/plan_manage/launch/dynamic_exp0.launch`
- `research/docs/DYNAMIC_PLANNING.md`
- `research/docs/REVERSIBLE_PLANNING.md`
- `research/docs/PERFORMANCE.md`
- `research/docs/EXPERIMENTS.md`
- `research/external/TOPAY_COMPARISON.md`
- `research/external/GCDF_COMPARISON.md`
