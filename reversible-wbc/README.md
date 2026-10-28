# Reversible Whole-Body Control

这套工程把“可逆性由结构保证，学习负责扩张可逆域”落成了可运行的
移动操作闭环。核心路径不依赖学习模型保证身体安全。

## 已实现

- 不可变安全壳：速度、加速度、关节行程、横向耦合、摩擦、支撑域和
  制动距离；
- 有限幅恢复动作：停车、收臂、基座回正与载荷联合补偿；
- 五成员恢复性集成模型与留出集阈值标定；
- 主动探边、认知不确定性门控和近边界速度收缩；
- 八个动作敏感参数的部署后在线更新；
- 边界证书、壳触发和不可逆事件三指标；
- 仓库载物全过程动力学仿真与标准/压力成对实验；
- 失败窗口采矿、课程数据生成与混合重训；
- 离散壳规格、单元测试、CSV/JSON/SVG 报告。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/rwb_tests
```

## 完整实验

```bash
./scripts/run_full_experiment.sh
```

脚本依次完成：初始训练、压力评测、失败课程采矿、混合重训、标准集与
压力集独立种子评测、壳规格和测试。

也可以分开执行：

```bash
./build/rwb_train \
  --samples 24000 --policy-samples 8000 --seed 17 \
  --output models/stage1.model

./build/rwb_benchmark \
  --model models/stage1.model --episodes 40 \
  --seed 73021 --out outputs/stress_round1 --stress

./build/rwb_curriculum \
  --telemetry outputs/stress_round1/telemetry \
  --out outputs/stress_round1/curriculum.csv

./build/rwb_train \
  --samples 24000 --policy-samples 8000 --seed 53 \
  --stress --mixed \
  --curriculum outputs/stress_round1/curriculum.csv \
  --output models/recoverability.model
```

## 实测结果

40 对独立种子：

| 指标 | 标准基线 | 标准新系统 | 压力基线 | 压力新系统 |
|---|---:|---:|---:|---:|
| 任务成功率 | 100.0% | 100.0% | 12.5% | 100.0% |
| 不可逆事件率 | 0.0% | 0.0% | 87.5% | 0.0% |
| 壳触发率 | 0.0% | 0.19% | 0.0% | 1.57% |
| 边界扩张/交互 | 0.000 | 0.983 | 0.000 | 0.981 |
| p95 控制延迟 | 0.21 us | 0.50 us | 0.42 us | 1.25 us |

标准集共有 889 次主动交互并新增 874 个边界证书；压力集共有 821 次
主动交互并新增 805 个边界证书。

## 代码结构

| 路径 | 内容 |
|---|---|
| `src/safety_shell.cpp` | 冻结安全壳与恢复动作 |
| `src/dynamics.cpp` | 载物动力学和恢复试验 |
| `src/monitor.cpp` | 集成模型、标定和在线更新 |
| `src/policy.cpp` | 规划、探边、门控和证书 |
| `src/closed_loop.cpp` | 100 Hz 闭环与完整遥测 |
| `apps/rwb_train.cpp` | 初始训练与混合重训 |
| `apps/rwb_curriculum.cpp` | 失败课程采矿 |
| `apps/rwb_benchmark.cpp` | 成对评测 |

架构细节见 `docs/architecture.md`，实验定义见
`docs/experiment_protocol.md`，真机接入见
`docs/hardware_transfer.md`。

