# 实施闭环对照

| 文档任务 | 实现 | 实测产物 |
|---|---|---|
| 选择可逆且可测的仓库载物场景 | 双关节移动操作平台载物、货架、摩擦、扰动 | `src/dynamics.cpp` |
| 建立不可变身体安全壳 | 速度、加速度、关节、横向、摩擦、支撑、制动 | `src/safety_shell.cpp` |
| 冻结壳并验证极端命令 | 97,200 个离散边界组合，0 个命令越界反例 | `outputs/shell_spec.txt` |
| 建立可恢复性监视器 | 五成员集成、17 维特征、留出阈值标定 | `src/monitor.cpp` |
| 主动探边 | 五档风险尺度、边界目标与不确定性评分 | `src/policy.cpp` |
| 近不可逆窄带控制 | 风险区恢复、边界窄带降速转向 | `src/policy.cpp` |
| 轻量部署后学习 | 八个敏感参数、3% 更新比例、0.16 限幅 | `src/monitor.cpp` |
| 接恢复动作与执行门控 | 停车、收臂、回正、联合载荷补偿 | `src/safety_shell.cpp` |
| 测三个数 | 壳触发率、边界扩张率、不可逆事件率 | `src/metrics.cpp` |
| 失败日志驱动课程 | 失败前 120 步与边界证书采矿 | `apps/rwb_curriculum.cpp` |
| 课程样本混合重训 | 标准在线状态 + 压力失败样本 | `apps/rwb_train.cpp` |
| 独立种子压力验收 | 40 对种子，基线 87.5% 事件率，新系统 0% | `docs/experiment_protocol.md` |
| 场景扩张 | 标准档与高载荷、低摩擦、强扰动压力档 | `apps/rwb_benchmark.cpp` |
| 真机接入契约 | 1 kHz 壳、100 Hz 规划、硬件量与验收步骤 | `docs/hardware_transfer.md` |

