# DIP-E026

This repository is for Group members of NTU EEE DIP-E026 to manage their code.

## MuJoCo 二轮平衡机器人 Demo

这是一个可直接运行的二轮自平衡小车示例，世界坐标约定为：`X` 前方、`Y` 左方、`Z` 上方。项目不依赖外部模型文件，包含：

- `balance_bot.xml`：具有自由底盘、两只独立驱动轮、地面接触与 IMU 传感器的 MuJoCo 模型；
- `balance_bot.py`：数据驱动最优姿态控制、自然语言指令系统、短时日志分析与安全闭环；
- `learned_lqr.py`：从 MuJoCo 短时采样中辨识动力学并求解最优 LQR 策略；
- `test_balance_bot.py`：模型平衡、双语输出、指令限幅和大倾角不自动重置的无界面测试；
- 格式化实时状态栏：运行模式、位置、测量/目标速度、姿态、左右电机力矩和短时窗口摘要。

## 安装与运行

需要 Python 3.10+ 与能创建 OpenGL 窗口的桌面环境。

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python balance_bot.py
```

如果 `python` 或 `py` 没有加入 `PATH`，可用本机解释器的绝对路径创建环境：

```powershell
& "C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe" -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python balance_bot.py
```

运行后会出现两个同进程窗口：MuJoCo 官方查看器与 **BalanceBot 仿真实验台**。在实验台顶部的输入框中输入中文或英文指令后，按 Enter 或点击“发送”：

程序默认使用英文界面。界面最上方的 `Languages` 选项始终使用英文显示，可在 `中文` 与 `English` 之间即时切换；界面语言不会限制指令语言，中英文指令始终都可识别。命令行可使用 `--language zh` 切换为中文。

```text
前进 0.3
左转 0.4
后退 0.2
停止
稳定
状态
重置
```

速度单位为 `m/s`，转向速率单位为 `rad/s`。命令会先经过限幅：默认最高 `1.20 m/s`、`1.50 rad/s`；`稳定` 将上限进一步降为 `0.75 m/s`、`0.90 rad/s`。

显示已恢复为 MuJoCo 官方 passive viewer。WASD 不再绑定任何机器人或显示操作；机器人快捷键只使用方向键，并在 passive 模式下驱动控制目标：

| 按键 | 动作 |
| --- | --- |
| `↑` | 目标前进速度增加 `0.15 m/s` |
| `↓` | 目标前进速度减少 `0.15 m/s` |
| `←` | 目标左转速率增加 `0.25 rad/s` |
| `→` | 目标右转速率增加 `0.25 rad/s` |

使用指令面板中的“停止”“稳定”“重置”和“状态”命令来执行相应操作。关闭 MuJoCo 查看器或指令面板即可结束对应界面。

## 抗扰动实验台

实验台提供下面的实时交互功能：

| 控件 | 作用 |
| --- | --- |
| 地面与轮胎摩擦 | 在 `0.20` 至 `2.50` 间改变接触摩擦 |
| 地面坡度 | 在 `-10°` 至 `+10°` 间改变沿前进方向的坡度 |
| IMU 噪声 / 延迟 | 注入角速度噪声及 `0--120 ms` 的观测延迟 |
| 电机延迟 | 注入 `0--120 ms` 的左右轮执行器延迟 |
| 轻推前方 / 后方 | 对底盘施加持续 `0.15 s` 的 `±25 N` 水平推力 |
| 摄像头跟随 | 切换查看器相机是否平滑跟随小车 |
| 清零指标 / 重置实验 | 清零最大偏差与恢复时间统计，或手动重置机器人状态 |
| Relearning current situation / 重新学习当前环境 | 从零策略开始，在当前环境中探索、收集实际轨迹、在线更新并验证 LQR |
| Stop relearning / 停止重新学习 | 结束学习；若已经找到有效策略，保留并继续使用最新策略 |

实时指标包括最大俯仰角、最大速度、最近恢复时间和恢复次数。普通仿真不再自动重置；机器人跌倒后需点击“重置实验”或输入 `重置/reset`。建议从低摩擦或小坡度开始，每次只增加一个扰动条件，观察小车是否恢复平衡。

## 可视化在线学习

点击 `Relearning current situation`（中文界面显示“重新学习当前环境”）后，查看器中的机器人不使用启动时的预学习策略，而是经历真实的在线学习回合：

1. 第一个回合从约 `9.5°` 的倾斜状态开始，随后使用受限随机轮端力矩探索；其晃动、倾倒和训练回合重启是正常且刻意保留的学习现象。
2. 每 `20 ms` 记录一次实际的 `(状态, 力矩, 下一状态)` 转移。累计至少 90 条后，程序直接用这些环境内数据辨识 `A`、`B`，并生成候选 LQR 策略。
3. 候选策略通过可控性和闭环谱半径检查后，机器人进入 4 秒稳定验证；失败轨迹会加入数据集后继续辨识，成功后自动转为稳定平衡。

“实时稳定性指标”会显示当前回合、真实转移样本数、更新次数、闭环谱半径和阶段说明。建议先开启“摄像头跟随”，然后再点击学习按钮，这样能直观看到“初期控制差 → 重置/采样 → 稳定平衡”的整个过程。

无界面复现该过程可运行：

```powershell
python balance_bot.py --headless --relearn-current-situation --duration 20 --no-realtime
```

无桌面环境或用于快速验证时：

```powershell
python balance_bot.py --headless --duration 10
```

加上 `--no-realtime` 可取消实时节拍，以最快速度运行。

也可在启动时直接交付指令，适合自动化演示：

```powershell
python balance_bot.py --headless --duration 5 --command "前进 0.3" --command "左转 0.3"
```

## 稳定闭环架构

每一个物理步（2 ms）都遵循下列数据流，复现参考图的职责划分：

```text
随机 MuJoCo 短轨迹 -> 最小二乘辨识 (A, B) -> 离散 LQR -> 学得反馈矩阵 K
                                                     |
Sensors -> StateEstimator -> LearnedLQRPolicy -> Motors -> BalanceBot
                           ^                    |
                           |                    v
                  LogBuffer (3 s) <- ShortWindowAnalyzer -> SafetyLimits
```

- **StateEstimator**：对速度、俯仰角速度和航向角速度进行轻量滤波；俯仰角保持直通，避免为平衡环引入额外延迟。
- **LogBuffer**：只保存最近 3 秒数据，范围受限在 1--5 秒内。
- **LearnedLQRPolicy**：启动时从平衡点周围的随机状态和轮端力矩采集 800 条短轨迹，以最小二乘辨识离散状态空间模型 `x(k+1) = A x(k) + B u(k) + c`；随后解离散 Riccati 方程得到反馈矩阵 `K`。状态为俯仰角、俯仰角速度、前进速度和航向角速度，动作是左右轮的共模/差模力矩。
- **OnlineLQRSession**：可视化学习模式不使用前述启动策略，而是把查看器中每一回合产生的真实状态转移追加到数据集；每次跌倒或回合结束后重新辨识、检查候选策略并进行稳定验证。
- **ShortWindowAnalyzer**：读取短窗口的俯仰 RMS、峰值、角速度与力矩占用率；检测到可恢复的大姿态偏差时，只收紧速度/转向限幅，绝不修改已学习的策略。
- **SafetySupervisor**：所有文本命令和分析器更新都必须经过限幅；俯仰超过 10° 时冻结运动目标进入恢复，但不会自动重置 MuJoCo 状态。

`ShortWindowAnalyzer` 是一个离线、确定性的安全监测层：它的唯一输出是受限的 `TuningUpdate(speed_limit, yaw_rate_limit)`，且从不拿到 MuJoCo 电机句柄或策略权重。因此它只能经由 `SafetySupervisor` 收紧运动范围；当前 demo 不需要 API Key 或网络即可运行。

## 控制结构

核心控制规律位于 `learned_lqr.py`，每隔 10 个物理步更新一次力矩目标：

```text
u = -K · (x - x_target)
common, differential = u
left  = clamp(common - differential)
right = clamp(common + differential)
```

这里的 `K` 并非手工 PD/PID 参数：它由启动阶段的随机采样和离散 LQR 优化自动得出。需要适应当前运行环境时，使用 `Relearning current situation` 从可见的实际轨迹重新辨识；状态栏会显示样本数、闭环谱半径和可控性秩。这个“最优”是对所辨识的平衡点附近线性模型、给定状态/力矩代价而言的局部最优，并不等价于任意大姿态或未见环境中的全局最优。

实际硬件上应先从安全的仿真到实物迁移流程开始：验证执行器方向和 IMU 坐标系，采集真实短轨迹重新辨识动力学，并将力矩、速度和倾角安全阈值设为保守值。

## 测试

```powershell
python -m unittest -v test_balance_bot.py
```
