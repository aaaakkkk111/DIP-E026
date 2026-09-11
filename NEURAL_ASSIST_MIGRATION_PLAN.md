# STM32 神经残差 TEST 模式移植计划

## 目标和隔离边界

在 `stm32_Balance_Car_L` 的 20 个厂家模式之后追加 `TEST_Mode`。原模式枚举值保持不变。神经网络只在完成模式选择且 `mode == TEST_Mode` 时初始化，并且只在 TEST 控制分支中推理。

TEST 的基础控制器使用独立状态，按照以下滞回规则选择厂家参数：

- `|Angle_Balance| >= 10°`：Weight_M 参数；
- `|Angle_Balance| <= 7°`：Normal 参数；
- 7°到10°：保持当前参数档位。

神经输出是左右电机各 `±350 PWM` 的残差。最终顺序固定为：基础PID、死区补偿、基础限幅、残差、最终限幅、安全停机、电机寄存器。

## 分阶段实施和验收

| 阶段 | 工作 | 验收条件 | 当前状态 |
|---|---|---|---|
| 0 | 保存厂家基线并比较源码 | 明确已有修改；保留只读厂家副本 | 完成 |
| 1 | 追加 TEST 模式和独立混合PID | 原20个枚举值不变；TEST可选；10°/7°滞回 | 完成，并已同步到正式工程 |
| 2 | 导出确定性PPO actor | 只含13-32-32-2 actor；记录模型SHA-256 | 完成 |
| 3 | 主机C/Python一致性 | 1000组输入最大误差不超过 `1e-5` | 完成，最大误差 `3.57627869e-7` |
| 4 | Keil/ARMCC完整构建 | 新增源码零编译错误；链接生成AXF和HEX | 正式工程源码编译通过；当前 `TOOL_VARIANT=mdk_lite`，53,632字节镜像触发L6050U并阻止链接 |
| 5 | 5 ms时序测量 | actor最大时间建议小于1 ms，硬上限2 ms；整个ISR不得超过5 ms | DWT记录已加入，等待实机运行 |
| 6 | 影子运行 | `TEST_ASSIST_APPLY_RESIDUAL=0`；记录基础PWM、建议残差、档位和周期数 | 代码默认处于影子模式，尚未烧录 |
| 7 | 小残差上电 | 先限幅50 PWM，再按50/100/200/350逐级验证 | 未开始 |
| 8 | 完整策略验收 | 350 PWM范围内无超时、异常切换或安全保护绕过 | 未开始 |

## 当前实现文件

移植源码已经同步到 `D:\DIP\Bluetooth\stm32_Balance_Car_L`。修改前源码和原固件产物保存在 `D:\DIP\Simulation\stm32_pre_test_backup`；本次受限链接没有生成可烧录的新HEX。

- `APP/TestAssist/test_assist.c/.h`：TEST独立PID、观测构造、模式监督、残差合成、DWT计时和影子遥测；
- `APP/TestAssist/neural_actor.c/.h`：从PPO模型导出的确定性浮点参考actor；
- `tools/export_neural_actor.py`：可重复的权重导出器；
- `tools/test_neural_actor_parity.py`：C/Python数值测试。

浮点actor先用于建立严格参考结果和取得真实周期数。STM32F103RC没有硬件FPU；如果实测超过时序预算，下一步将以参考actor为基准转换int8或定点实现，并重复相同的逐向量测试。

## 影子遥测

Keil Watch窗口读取 `g_test_assist_telemetry`：

- `inference_cycles`：最近一次actor耗费周期；
- `maximum_inference_cycles`：启动以来最大周期；
- `base_pwm_left/right`：混合PID基础输出；
- `residual_pwm_left/right`：网络建议残差；
- `assisted_pwm_left/right`：假设启用残差后的输出；
- `commanded_pwm_left/right`：实际送往后续安全检查的输出；
- `weight_profile`：0为Normal，1为Weight；
- `residual_applied`：当前固定为0。

在影子阶段，`commanded_pwm`必须始终等于`base_pwm`，网络结果不能改变电机。

## 下一轮实机执行顺序

1. 在 Keil License Management 中启用无 32 KB 限制的 MDK 许可，并对正式工程执行 Rebuild；必须得到新的 AXF/HEX，且新增文件零 warning/error。
2. 保持 `TEST_ASSIST_APPLY_RESIDUAL=0`，烧录影子固件。首次测试把车轮架空，并保留随时断电条件。
3. 在 Keil Debug/Watch 中添加 `g_test_assist_telemetry`，选择第21项 `TEST Neural`。确认 `inference_count`持续递增，证明网络确实在200 Hz控制路径内运行。
4. 运行静止、手动前后倾斜和低速前后指令三组检查。每次都确认 `commanded_pwm_left/right == base_pwm_left/right`，以及10°进入Weight、7°回到Normal。
5. 72 MHz 下，`maximum_inference_cycles < 72000`作为建议门槛（1 ms），`< 144000`作为硬门槛（2 ms）；同时用逻辑分析仪或调试GPIO确认5 ms中断没有丢周期。
6. 核对角度、角速度、左右编码器和左右残差的符号。任一符号不一致时，仅修正观测映射，重新执行C/Python固定向量测试和影子测试。
7. 全部通过后才把残差限幅按50、100、200、350 PWM四档逐步放开；每一档重新验证倾倒、低压、停止和模式退出保护。

## 启用条件

在以下条件全部满足前，不修改 `TEST_ASSIST_APPLY_RESIDUAL`：

1. 使用无容量限制的Keil许可或等价授权工具链生成完整AXF/HEX；
2. 核对actor模型SHA-256为 `9b92948f15cf7ad4da3ef3f69453277a920563365ac5519cf3eb1b231e7c2026`；
3. 实机确认传感器和编码器符号与仿真一致；
4. 确认最大推理时间满足预算；
5. 完成支架上的影子运行并检查全部遥测；
6. 确认倾倒、低压和停止保护仍在残差之后执行。
