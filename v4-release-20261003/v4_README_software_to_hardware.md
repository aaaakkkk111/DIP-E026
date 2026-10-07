# 平衡小车自适应控制模型 v4：从软件设计到硬件烧录的完整说明 / Balance Car Adaptive Control Model v4: A Complete Guide from Software Design to Hardware Flashing

版本日期：2026 年 10 月 3 日（在 10 月 2 日版基础上经实车调试修改，修改内容见第 12 节）
对应固件：本文件夹里的 `stm32_Balance_Car_L_v4.hex`
文件校验（SHA-256）：`cfb226cc901c0e8a6153d7bc1b3101889bb8ed8ae8ad7bb252342a58ec8289f9`

Version date: October 3, 2026 (modified from the October 2 version after debugging on the real car; the changes are described in Section 12)
Matching firmware: `stm32_Balance_Car_L_v4.hex` in this folder
File checksum (SHA-256): `cfb226cc901c0e8a6153d7bc1b3101889bb8ed8ae8ad7bb252342a58ec8289f9`

> **本版和 10-02 版的区别（详见第 12 节）**：10-02 版只在数字孪生里验证过。10-03 上真车后，
> 根据实车现象和串口数据修了 7 个问题：开机就抖、空载档仍抖、降档慢（两处）、正常开车也升档、
> 升档后又慢慢加硬、被推后反复升档。修改后实车上：频繁前后开 0 次升档（原来 2 分钟 9 次），
> 被推一下只升档 1 次、0.5 秒回到空载档（原来要抖 2～3 秒）。

> **Differences between this version and the 10-02 version (see Section 12 for details)**: the 10-02 version was only validated in the digital twin. After it went onto the real car on 10-03,
> 7 problems were fixed based on what the real car did and on the serial-port data: chatter right after power-on, chatter still present in the empty gear, slow downshift (two places), upshift during normal driving,
> gradual stiffening after an upshift, and repeated upshifts after being pushed. After the changes, on the real car: frequent forward-and-back driving causes 0 upshifts (previously 9 in 2 minutes),
> and a single push causes only 1 upshift, with the car back in the empty gear within 0.5 seconds (previously it chattered for 2 to 3 seconds).

---

## 目录 / Table of Contents

0. 这个文件夹里有什么，怎么烧录、怎么用
1. 要解决的问题，以及为什么这样解决
2. 术语表（全文所有专有名词都在这里解释）
3. 小车硬件和原厂固件是怎么控制平衡的
4. 数字孪生：在电脑里搭一辆「虚拟小车」
5. 设计历程：从交叉熵搜索（CEM）开始的每一步思考和实验
6. 源码逐段说明：每一部分做什么、为什么这样设计
7. 从 Python 到 C：移植、逐拍对拍、编译出 hex 的方法
8. 验证结果（全部带原厂基线）
9. 已知局限，以及上实车之前必须做的标定
10. 所有相关文件的位置
11. 分工：人做了什么、大语言模型做了什么、提示词
12. 本版修改：2026-10-03 实车调试的目的、过程、问题与解决方法
13. 汇总：v4 与原厂两个模式的对比（跑分和跑分规则）

0. What is in this folder, how to flash it, and how to use it
1. The problem to solve, and why it is solved this way
2. Glossary (every specialized term in this document is explained here)
3. How the car hardware and the factory firmware control balance
4. The digital twin: building a "virtual car" in the computer
5. Design history: every step of reasoning and experiment, starting from the cross-entropy method (CEM)
6. Source code walkthrough, section by section: what each part does and why it is designed this way
7. From Python to C: porting, tick-by-tick comparison, and how to compile the hex file
8. Validation results (all with factory baselines)
9. Known limitations, and the calibration that must be done before running on the real car
10. Locations of all related files
11. Division of work: what the human did, what the large language model did, and the prompts
12. Changes in this version: purpose, process, problems and solutions of the 2026-10-03 real-car debugging
13. Summary: comparison of v4 with the two factory modes (benchmark scores and scoring rules)

---

## 0. 这个文件夹里有什么，怎么烧录、怎么用 / 0. What is in this folder, how to flash it, and how to use it

### 0.1 文件清单 / 0.1 File list

| 文件 | 内容 |
|---|---|
| `stm32_Balance_Car_L_v4.hex` | **要烧录的固件**。由 Keil 编译完整工程得到，包含原厂全部功能，再加上新的「模式 27：Adapt V4」。本版就是实车测试版 realtest4。 |
| `v4_README_software_to_hardware.md` | 本说明。 |
| `source_snapshot/v4_core.c`、`v4_core.h` | v4 的全部决策逻辑（纯计算，不碰任何硬件），是本说明第 6 节逐段讲解的对象。 |
| `source_snapshot/v4_adapt.c`、`v4_adapt.h` | 把 v4 接进原厂固件的「胶水层」：读遥控指令、把算出的增益写进原厂 PID 变量。 |
| `source_snapshot/switch_v4.py`、`switch_v3.py` | 同一套逻辑的 Python 版本。所有仿真实验都是用它们做的，C 版本是从它们逐行移植的。 |
| `source_snapshot/firmware_hooks/` | 原厂固件里为接入 v4 而改动过的 8 个文件（编译时用的就是这些）。 |
| `real_car_logs/` | 10-03 实车调试的串口记录（CSV）、自动生成的档位事件说明 `v4_events.txt`、录制脚本 `v4_logger.py`，见第 12 节。 |
| `appendix_all_user_prompts.md` | 用户在整个项目里发给大语言模型的全部提示词原文（393 条，含 10-03 新增的 63 条），见第 11 节。 |

| File | Contents |
|---|---|
| `stm32_Balance_Car_L_v4.hex` | **The firmware to flash**. Built by compiling the complete Keil project; it contains all factory functions plus the new "mode 27: Adapt V4". This version is the real-car test build realtest4. |
| `v4_README_software_to_hardware.md` (v4 guide: from software to hardware) | This document. |
| `source_snapshot/v4_core.c` (source snapshot), `v4_core.h` | All of v4's decision logic (pure computation, touches no hardware); this is what Section 6 of this document explains section by section. |
| `source_snapshot/v4_adapt.c`, `v4_adapt.h` | The "glue layer" that connects v4 to the factory firmware: it reads remote-control commands and writes the computed gains into the factory PID variables. |
| `source_snapshot/switch_v4.py`, `switch_v3.py` | The Python version of the same logic. All simulation experiments were done with these; the C version was ported from them line by line. |
| `source_snapshot/firmware_hooks/` (firmware integration points) | The 8 files in the factory firmware that were modified to connect v4 (these are the ones used for compilation). |
| `real_car_logs/` (real-car logs) | Serial-port logs (CSV) from the 10-03 real-car debugging, the automatically generated gear-event description `v4_events.txt`, and the recording script `v4_logger.py`; see Section 12. |
| `appendix_all_user_prompts.md` (appendix: all user prompts) | The original text of every prompt the user sent to the large language model during the whole project (393 prompts, including 63 new ones from 10-03); see Section 11. |

`source_snapshot/` 只是存档，方便对照阅读。真正参与编译的源码在 Keil 工程里（第 10 节有位置）。

The source snapshot is only an archive, kept for convenient side-by-side reading. The source code actually compiled is in the Keil project (its location is given in Section 10).

### 0.2 烧录和使用步骤 / 0.2 Flashing and usage steps

1. 用你平时的烧录方式（ST-Link、串口下载工具等）把 `stm32_Balance_Car_L_v4.hex` 烧进小车的 STM32F103RCT6 主控。hex 文件的起始地址是 0x0800 0000，也就是 STM32 闪存的起点，和原厂固件相同。
2. 上电后屏幕进入模式选择界面。**用手转动车轮**切换模式（固件 `car_mode()` 检测编码器读数变化超过 250 就切到下一个模式），转到屏幕显示 **「27.Adapt V4」** 后，按 **KEY1** 确认。
3. 屏幕提示「put down key start!」。把小车竖直放在地上，再按一次 **KEY1**，小车开始自平衡。
4. 之后用手机蓝牙 App 正常操控前、后、左、右。v4 会自己判断负载、自己换档，不需要手动选择「正常模式」还是「负重模式」。
5. 屏幕第 3 行会显示 v4 的内部状态，格式为 `f0.00L osc 1.2 u0`：
   - `f0.00`：当前档位系数 f（0 = 原厂正常档，也就是本版的空载档；0.40 = 带载档；1 = 原厂负重档，具体见第 2 节）；
   - `L` 或 `P`：L 表示已经判定好负载并锁定（Latched），P 表示正在探测（Probing）；
   - `osc 1.2`：抖振读数，单位度每秒；
   - `u0`：开机以来因大倾角而升档的次数。

1. Use your usual flashing method (ST-Link, a serial download tool, etc.) to flash `stm32_Balance_Car_L_v4.hex` into the car's STM32F103RCT6 main controller. The hex file's start address is 0x0800 0000, which is the start of the STM32 flash memory, the same as the factory firmware.
2. After power-on, the screen shows the mode selection interface. **Turn a wheel by hand** to change modes (the firmware function `car_mode()` moves to the next mode when the encoder reading changes by more than 250). When the screen shows **"27.Adapt V4"**, press **KEY1** to confirm.
3. The screen shows "put down key start!". Stand the car upright on the ground and press **KEY1** once more; the car starts balancing by itself.
4. After that, drive forward, backward, left and right as usual with the Bluetooth phone app. v4 judges the load and changes gears by itself; there is no need to choose "Normal mode" or "Weight mode" manually.
5. Line 3 of the screen shows v4's internal state, in the format `f0.00L osc 1.2 u0`:
   - `f0.00`: the current gain coefficient f (0 = factory Normal gains, which is the empty gear in this version; 0.40 = the loaded gear; 1 = factory Weight gains; see Section 2 for details);
   - `L` or `P`: L means the load has been judged and latched (Latched); P means probing is in progress (Probing);
   - `osc 1.2`: the chatter reading, in degrees per second;
   - `u0`: the number of upshifts caused by large tilt since power-on.

**实车情况**：判断负载用的分界值（第 6 节的 `OSC_SPLIT` = 23）是在电脑仿真里定的。10-03 的实车记录里，空车在最硬档上的抖振读数每次都在 57～70，远高于 23，所以空载判断在真车上是可靠的（几十次重新判断全部判对）。**带负载的实车判断本轮没有重新测**，如果要带负载使用，建议先按第 9 节做一次实车标定。

**Real-car status**: the threshold used to judge the load (`OSC_SPLIT` = 23 in Section 6) was set in the computer simulation. In the 10-03 real-car logs, the empty car's chatter reading in the stiffest gear was between 57 and 70 every time, far above 23, so the empty-load judgment is reliable on the real car (dozens of re-judgments were all correct). **The loaded judgment on the real car was not re-tested in this round.** If you want to use the car with a load, it is recommended to first do a real-car calibration as described in Section 9.

**串口记录**：用 `real_car_logs/v4_logger.py` 可以录下每一拍的档位和 v4 内部状态，用法见第 12.2 节。注意要等车开始平衡后再开始录。

**Serial-port logging**: `real_car_logs/v4_logger.py` can record the gear and v4's internal state at every tick; see Section 12.2 for usage. Note: start recording only after the car has started balancing.

---

## 1. 要解决的问题，以及为什么这样解决 / 1. The problem to solve, and why it is solved this way

### 1.1 原厂的问题 / 1.1 The problem with the factory firmware

这辆两轮自平衡小车的原厂固件只有两套固定的控制参数：

The factory firmware of this two-wheel self-balancing car has only two fixed sets of control parameters:

- **正常模式（Normal）**：适合空车。车上加了 1 千克以上的东西，它就站不稳，会倒。
- **负重模式（Weight_M）**：适合带货。空车用它会剧烈高频抖动（仿真里抖振读数约 33 度每秒，正常模式只有约 0.5）。

- **Normal mode (Normal)**: suited to an empty car. With 1 kilogram or more added on the car, it cannot stand steadily and falls over.
- **Weight mode (Weight_M)**: suited to carrying cargo. Used on an empty car, it produces violent high-frequency shaking (in simulation the chatter reading is about 33 degrees per second, versus only about 0.5 in Normal mode).

用户必须**自己判断**车上有没有货，再手动选模式。选错了，要么倒，要么一直抖。即使每次都选对（我们称为「理想手动」），两档之间的负载（比如 1 千克）也没有一档真正合适：正常模式会倒，负重模式停下后一直抖、不收敛。

The user must **judge for themselves** whether there is cargo on the car and then select the mode manually. With the wrong choice, the car either falls over or keeps shaking. Even if the choice is right every time (we call this "ideal manual"), for loads between the two gears (for example 1 kilogram) neither gear is really suitable: Normal mode falls over, and Weight mode keeps shaking after stopping and does not converge.

### 1.2 目标 / 1.2 Goal

做一个**自己会判断负载、自己会换档**的控制器，并且在各种情况下都比原厂好或至少不差：不同负重、行驶后停车、被推、上下坡、负载中途改变、急刹、转弯。

Build a controller that **judges the load by itself and changes gears by itself**, and that is better than, or at least no worse than, the factory firmware in every situation: different loads, stopping after driving, being pushed, going up and down slopes, the load changing mid-run, hard braking, and turning.

### 1.3 为什么不用「一个神经网络包办一切」 / 1.3 Why not "one neural network that does everything"

项目早期尝试过端到端强化学习（用近端策略优化算法 PPO 训练一个神经网络直接输出电机指令）。七轮训练的结论是：

Early in the project, end-to-end reinforcement learning was tried (training a neural network with the proximal policy optimization algorithm, PPO, to output motor commands directly). The conclusions after seven rounds of training were:

- 每个工况本身都很「干净」：平衡是经典控制问题，负载是一维的质量变化，冲击是短暂瞬态。把它们混在一个网络里学，反而每一项都学不好；
- 网络输给了专门的负载估计器；在 240 局配对测试里，4 个固定常数（68 局摔）赢了网络（77 局摔）；
- 网络部署到真车最怕「仿真和真车的差别」，而没有传统控制兜底时，这个差别会让策略整个失效。

- Each operating condition is "clean" on its own: balancing is a classic control problem, the load is a one-dimensional change in mass, and a shock is a brief transient. Mixing them together for one network to learn actually makes it learn every one of them poorly;
- The network lost to a dedicated load estimator; in 240 paired test episodes, 4 fixed constants (68 episodes with falls) beat the network (77 episodes with falls);
- The biggest risk when deploying the network on the real car is "the difference between simulation and the real car", and without traditional control as a fallback, this difference can make the whole policy fail.

所以从 2026 年 9 月 29 日起，控制结构固定为**四层**：

So, starting from September 29, 2026, the control structure was fixed as **four layers**:

| 层 | 作用 | v4 里对应的部分 |
|---|---|---|
| 第 0 层（基座） | 原厂串级比例-积分-微分控制器（串级 PID）：平衡、速度、转向 | 原封不动保留原厂代码 |
| 第 1 层 | 负载判断 + 在两套原厂参数之间插值 | v4 的抖振探测、锁存、两档 |
| 第 2 层 | 冲击、大倾角的应急处理 | v4 的大倾角升档、快速撤回 |
| 第 3 层（可选） | 机器学习残差修正 | 没有做，前三层已经达到目标 |

| Layer | Role | Corresponding part in v4 |
|---|---|---|
| Layer 0 (base) | Factory cascaded proportional-integral-derivative controller (cascaded PID): balance, speed, steering | The factory code is kept completely unchanged |
| Layer 1 | Load judgment + interpolation between the two factory parameter sets | v4's chatter probing, latching, and two gears |
| Layer 2 | Emergency handling of shocks and large tilt | v4's large-tilt upshift and quick revert |
| Layer 3 (optional) | Machine-learning residual correction | Not done; the first three layers already reach the goal |

v4 只做第 1、2 层，**而且只改「用哪套参数」，不改原厂的控制公式**。这是一个重要的安全性设计：最坏情况下，v4 等价于「原厂负重模式」或「原厂正常模式」中的某一档，不会产生原厂从来没有过的控制行为。

v4 implements only layers 1 and 2, **and it only changes "which parameter set is used"; it does not change the factory control formulas**. This is an important safety design: in the worst case, v4 is equivalent to one of the gears "factory Weight mode" or "factory Normal mode", and it cannot produce control behavior that the factory firmware never had.

---

## 2. 术语表 / 2. Glossary

| 术语 | 含义 |
|---|---|
| 倾角 | 车身前后倾斜的角度，单位度。0 度为竖直，前倾为正。固件变量 `Angle_Balance`。 |
| 角速度 / 陀螺读数 | 车身倾斜的快慢。固件变量 `Gyro_Balance` 是陀螺仪芯片（MPU6050）寄存器的**原始读数**，单位记作 LSB（最低有效位），16.4 个 LSB 等于 1 度每秒。 |
| 拍 | 固件控制循环执行一次，叫一拍。小车每 5 毫秒一拍，每秒 200 拍。 |
| PWM | 脉冲宽度调制，即电机驱动的占空比，决定电机出力大小。本车上限约 ±2600～2800。 |
| 增益 | 控制公式里的系数。比如平衡增益越大，车身一歪，电机反应越猛。 |
| 串级 PID | 比例（P）-积分（I）-微分（D）控制器。原厂有三个环：平衡环（比例+微分）、速度环（比例+积分）、转向环（比例+微分），三者相加得到电机输出。 |
| 正常档 / 负重档 | 原厂两套增益。正常档平衡比例增益 96，负重档 192（数值见第 3 节）。 |
| 档位系数 f | v4 的核心输出，0 到 1 之间的一个数。f=0 用正常档全套增益，f=1 用负重档全套增益，中间值按比例插值。例如 f=0.4 时，平衡比例增益 = 96 + (192−96)×0.4 = 134.4。 |
| 轻档 / 重档 | v4 判断出负载后锁定的档位：空载用轻档，带载用重档 f=0.40。轻档在 10-02 版是 f=0.12，本版改为 f=0（等于原厂模式 1，见 12.3 问题 2）。 |
| 最硬档 | f=1，即原厂负重档。v4 把它当作「最安全、并且能判断负载」的位置（理由见第 5 节）。 |
| 抖振 / 抖振读数 | 车身 8～16 赫兹的高频小幅振动。用陀螺读数做带通滤波后取能量的平方根，单位度每秒。它是 v4 判断负载的依据：在最硬档下，空车会剧烈抖（读数 24～37），带载车很安静（读数 0.3～3）。 |
| 探测 | 在最硬档下停着，等抖振读数稳定，然后判断轻或重。 |
| 锁存 | 判断一次就记住，之后不再反复判断（理由见 5.3 节，这是 v3 最关键的改动）。 |
| 升档 | 把 f 调到 1（最硬档）。v4 在车身倾斜过大时立刻升档。 |
| 降档 | 在最硬档重新探测后，把 f 锁回轻档或重档。 |
| 预测倾角 θ_pred | 当前倾角偏离平衡位置的量，加上「角速度 × 0.1 秒」，也就是估计 0.1 秒后会歪到多少。用它触发升档，比只看当前倾角提前约 0.1 秒反应。 |
| 平衡角 th_ref | 车实际能站稳的角度。它不一定是 0 度：负载重心偏、在坡上，平衡角都会变。v4 用缓慢跟踪的方式自己学出来。 |
| 门限 | 触发升档的预测倾角大小。静止时 3～8 度，行驶时 16 度。 |
| 上膛 | 锁存或升档后，车要先连续平稳一段时间，升档触发才生效，避免刚换完档的余振马上又触发换档。 |
| 停稳门 | 停车后，车要先真正晃停（连续平稳 0.6 秒），才启用静止时的小门限。 |
| 快速撤回 | 升档后 0.3 秒，如果发现只是一次小扰动，直接退回原来的档位，不重新探测。 |
| 慢速逃生 | 锁存后如果车长期歪着（吃力），临时把 f 往上加，之后慢慢衰减回锁存值。 |
| 无扰切换 | 换档时按比例缩放速度环积分，让电机输出不因增益突变而跳一下。 |
| 数字孪生 | 在电脑物理引擎 MuJoCo 里搭建的虚拟小车，质量、尺寸、电机、传感器、原厂固件逻辑都尽量按真车复刻。所有设计和测试都先在孪生上做。 |
| 种子 | 随机数的起点。换一个种子，随机扰动、随机化的车参数都会不同，用来检验结果不是碰巧。 |
| 随机化孪生 | 每个种子把孪生的若干物理参数（电机延迟、摩擦、力矩、振动强度等）在合理范围内随机取值，相当于「一台和标称车略有不同的车」。 |
| CEM（交叉熵方法） | 一种参数搜索方法。每一代随机生成一批候选参数，在孪生里打分，挑出最好的一批（精英），用它们的均值和分散程度生成下一代，反复迭代。 |
| 逐拍对拍 | 把同一串传感器输入同时喂给 Python 版和 C 版，逐拍比较输出是否完全一致，用来证明 C 移植没有错。 |

| Term | Meaning |
|---|---|
| Tilt angle | The forward/backward tilt angle of the car body, in degrees. 0 degrees is upright; forward tilt is positive. Firmware variable `Angle_Balance`. |
| Angular velocity / gyro reading | How fast the car body is tilting. The firmware variable `Gyro_Balance` is the **raw reading** of the gyroscope chip (MPU6050) register, in units written as LSB (least significant bit); 16.4 LSB equals 1 degree per second. |
| Tick | One execution of the firmware control loop is called a tick. The car runs one tick every 5 milliseconds, 200 ticks per second. |
| PWM | Pulse-width modulation, i.e. the duty cycle of the motor drive, which determines how much force the motor produces. The limit on this car is about ±2600 to 2800. |
| Gain | A coefficient in a control formula. For example, the larger the balance gain, the more strongly the motors react when the car body tilts. |
| Cascaded PID | Proportional (P) - integral (I) - derivative (D) controller. The factory firmware has three loops: the balance loop (proportional + derivative), the speed loop (proportional + integral), and the steering loop (proportional + derivative); the three are added together to give the motor output. |
| Normal gear / Weight gear | The two factory gain sets. The Normal gear's balance proportional gain is 96; the Weight gear's is 192 (values in Section 3). |
| Gain coefficient f | v4's core output, a number between 0 and 1. f=0 uses the full factory Normal gain set, f=1 uses the full factory Weight gain set, and values in between interpolate proportionally. For example, at f=0.4 the balance proportional gain = 96 + (192−96)×0.4 = 134.4. |
| Light gear / heavy gear | The gear v4 latches after judging the load: the light gear for an empty car, the heavy gear f=0.40 for a loaded car. In the 10-02 version the light gear was f=0.12; in this version it was changed to f=0 (equal to factory mode 1; see 12.3, problem 2). |
| Stiffest gear | f=1, i.e. factory Weight gains. v4 treats it as the position that is "safest and also able to judge the load" (reasons in Section 5). |
| Chatter / chatter reading | High-frequency, small-amplitude vibration of the car body at 8 to 16 hertz. It is computed by band-pass filtering the gyro reading and taking the square root of the energy, in degrees per second. It is the basis for v4's load judgment: in the stiffest gear, an empty car chatters violently (reading 24 to 37), while a loaded car is very quiet (reading 0.3 to 3). |
| Probe / probing | Staying in the stiffest gear, waiting for the chatter reading to settle, and then judging light or heavy. |
| Latch | Judge once and remember the result, without judging again and again afterward (reason in Section 5.3; this is the most important change in v3). |
| Upshift | Setting f to 1 (the stiffest gear). v4 upshifts immediately when the car body tilts too far. |
| Downshift | After re-probing in the stiffest gear, latching f back to the light gear or the heavy gear. |
| Predicted tilt θ_pred | The current tilt's deviation from the balance position, plus "angular velocity × 0.1 seconds", i.e. an estimate of how far the car will be tilted 0.1 seconds later. Using it to trigger upshifts reacts about 0.1 seconds earlier than looking only at the current tilt. |
| Balance-angle reference th_ref | The angle at which the car can actually stand steadily. It is not necessarily 0 degrees: an off-center load or a slope both change the balance angle. v4 learns it by itself through slow tracking. |
| Threshold | The predicted-tilt magnitude that triggers an upshift. 3 to 8 degrees when stationary, 16 degrees when driving. |
| Arming | After a latch or an upshift, the car must first stay steady continuously for a period of time before the upshift trigger becomes active; this prevents the residual vibration right after a gear change from immediately triggering another gear change. |
| Stop-calm gate | After stopping, the car must first truly stop swaying (steady continuously for 0.6 seconds) before the small stationary threshold is enabled. |
| Quick revert | 0.3 seconds after an upshift, if it turns out to have been only a small disturbance, the controller goes straight back to the previous gear without re-probing. |
| Slow escape | After latching, if the car stays tilted for a long time (straining), f is temporarily raised and then slowly decays back to the latched value. |
| Bumpless transfer | When changing gears, the speed-loop integral is scaled proportionally so that the motor output does not jump because of the sudden gain change. |
| Digital twin | A virtual car built in the MuJoCo physics engine on the computer; its mass, dimensions, motors, sensors and factory firmware logic replicate the real car as closely as possible. All design and testing is done on the twin first. |
| Seed | The starting point of the random numbers. With a different seed, the random disturbances and the randomized car parameters are all different; this is used to check that a result is not a coincidence. |
| Randomized twin | For each seed, several physical parameters of the twin (motor delay, friction, torque, vibration strength, etc.) are randomly chosen within reasonable ranges, which is equivalent to "a car slightly different from the nominal car". |
| CEM (cross-entropy method) | A parameter search method. Each generation randomly produces a batch of candidate parameters, scores them in the twin, picks the best batch (the elites), and uses their mean and spread to generate the next generation, iterating repeatedly. |
| Tick-by-tick comparison | Feeding the same sequence of sensor inputs to both the Python version and the C version and comparing the outputs tick by tick for exact agreement; this is used to prove that the C port has no errors. |

---

## 3. 小车硬件和原厂固件是怎么控制平衡的 / 3. The car hardware and how the factory firmware keeps it balanced

### 3.1 硬件 / 3.1 Hardware

- 主控：STM32F103RCT6 单片机。
- 姿态传感器：MPU6050（陀螺仪 + 加速度计），固件融合出倾角 `Angle_Balance` 和角速度 `Gyro_Balance`。
- 两个带编码器的直流减速电机，轮半径 33.5 毫米。编码器每 5 毫秒读一次，得到轮子转了多少。
- 蓝牙模块接收手机 App 指令。屏幕为 OLED。两个按键，KEY1 用来确认模式和启动。

- Main controller: STM32F103RCT6 microcontroller.
- Attitude sensor: MPU6050 (gyroscope + accelerometer). The firmware fuses its data into the tilt angle `Angle_Balance` and the angular rate `Gyro_Balance`.
- Two geared DC motors with encoders; wheel radius 33.5 millimeters. The encoders are read every 5 milliseconds to get how far the wheels have turned.
- A Bluetooth module receives commands from the phone App. The display is an OLED screen. There are two keys; KEY1 is used to confirm the mode and to start.

### 3.2 原厂的 5 毫秒控制循环（`APP/app_control.c`） / 3.2 The factory 5-millisecond control loop (`APP/app_control.c`)

每 5 毫秒，陀螺仪芯片发出数据就绪中断，固件依次执行：

Every 5 milliseconds the gyroscope chip raises a data-ready interrupt, and the firmware executes the following in order:

1. `Get_Angle()`：更新倾角和角速度；
2. 读左右编码器；
3. **（v4 插在这里）`V4_Tick()`：算出档位系数 f，把对应的六个增益写进原厂变量**；
4. `Balance_PD()`：平衡环，`输出 = −Balance_Kp/100 × (平衡中值 − 倾角) − (0 − 角速度) × Balance_Kd/100`；
5. `Velocity_PI()`：速度环。把左右轮编码器读数之和低通滤波，累加成积分 `Encoder_Integral`，遥控前进/后退时再往积分里加一个固定量 `Movement`，输出 `−滤波值 × Velocity_Kp/100 − 积分 × Velocity_Ki/100`；
6. `Turn_PD()`：转向环，遥控左转/右转时给一个固定目标 `Turn_Target`；
7. 三者相加，经过死区补偿和限幅，输出到左右电机的 PWM。

1. `Get_Angle()`: update the tilt angle and the angular rate;
2. Read the left and right encoders;
3. **(v4 is inserted here) `V4_Tick()`: compute the gain coefficient f and write the corresponding six gains into the factory variables**;
4. `Balance_PD()`: the balance loop, `output = −Balance_Kp/100 × (balance midpoint − tilt) − (0 − angular rate) × Balance_Kd/100`;
5. `Velocity_PI()`: the velocity loop. The sum of the left and right encoder readings is low-pass filtered and accumulated into the integral `Encoder_Integral`; when the remote commands forward/backward, a fixed amount `Movement` is also added to the integral. The output is `−filtered value × Velocity_Kp/100 − integral × Velocity_Ki/100`;
6. `Turn_PD()`: the steering loop; when the remote commands turn left/right, it is given a fixed target `Turn_Target`;
7. The three outputs are summed, passed through dead-band compensation and limiting, and sent as PWM to the left and right motors.

### 3.3 原厂两套增益 / 3.3 The two factory gain sets

| 增益（固件单位，即公式里除以 100 之前的数） | 正常档 | 负重档 |
|---|---|---|
| 平衡比例 `Balance_Kp` | 9600 | 19200 |
| 平衡微分 `Balance_Kd` | 48 | 150 |
| 速度比例 `Velocity_Kp` | 6200 | 9450 |
| 速度积分 `Velocity_Ki` | 31.00 | 47.25 |
| 转向比例 `Turn_Kp` | 1700 | 1400 |
| 转向微分 `Turn_Kd` | 20 | 20 |

| Gain (firmware units, i.e. the number before dividing by 100 in the formula) | Normal gains | Weight gains |
|---|---|---|
| Balance proportional `Balance_Kp` | 9600 | 19200 |
| Balance derivative `Balance_Kd` | 48 | 150 |
| Velocity proportional `Velocity_Kp` | 6200 | 9450 |
| Velocity integral `Velocity_Ki` | 31.00 | 47.25 |
| Steering proportional `Turn_Kp` | 1700 | 1400 |
| Steering derivative `Turn_Kd` | 20 | 20 |

原厂的负重模式还会在公式最后乘三个倍率（`Balance_K`、`Velocity_K = 1.35`、`Turn_K`）。v4 运行在模式 27，固件不会再乘这些倍率，所以上表负重档的数值是**已经把倍率折算进去之后**的等效值（取自孪生里与原厂逐项核对过的 `pid_gains("Weight_M")`）。

The factory weight mode also multiplies the formulas by three factors at the end (`Balance_K`, `Velocity_K = 1.35`, `Turn_K`). v4 runs in mode 27, where the firmware no longer applies these factors, so the Weight gains values in the table above are equivalent values **with the factors already folded in** (taken from `pid_gains("Weight_M")` in the digital twin, which was checked item by item against the factory firmware).

### 3.4 遥控指令 / 3.4 Remote-control commands

App 每次只发一个离散状态：前进、后退、左转、右转、停止。前进时 `Movement = Car_Target_Velocity = 30`（约 0.48 米每秒），松手立刻变 0，**没有渐变**，所以实际操作里的每一次停车都是「急刹」。左转右转时 `Turn_Target = ±Car_Turn_Amplitude_speed = ±36`。

The App sends only one discrete state at a time: forward, backward, turn left, turn right, stop. When going forward, `Movement = Car_Target_Velocity = 30` (about 0.48 meters per second); when the button is released it becomes 0 immediately, **with no ramp**, so in actual operation every stop is a "hard brake". When turning left or right, `Turn_Target = ±Car_Turn_Amplitude_speed = ±36`.

---

## 4. 数字孪生：在电脑里搭一辆「虚拟小车」 / 4. The digital twin: building a "virtual car" in the computer

所有设计都先在孪生上验证，原因很直接：真车摔一次要扶起来、可能损坏，而且一组参数要跑几百局才能看出统计差别，只有仿真做得到。

Every design is verified on the twin first, for a simple reason: each time the real car falls, someone has to pick it up and it may be damaged, and a parameter set needs hundreds of runs before statistical differences show up, which only simulation can provide.

- **物理引擎**：MuJoCo。车身、两个轮子、电机力矩上限、齿轮弹性、摩擦、陀螺噪声都在模型里。
- **固件逻辑**：孪生里运行的是逐行照抄原厂 C 代码的 Python 版本（平衡环、速度环、转向环、死区补偿、5 毫秒节拍、输出延迟一拍），所以「原厂正常档 / 负重档」的仿真结果可以直接当作原厂基线。
- **参数来源**：2026 年 9 月 23 日的真车测量（开环扫频、死区测试等），记录在 `e026 keil/realcar/PARAMS.md`。
- **修过的孪生错误**（每一个都曾经让结论出错，都已修正）：
  1. 反向死区补偿：真固件正转补 1480、反转补 1455，孪生原来两个方向都补 1480。修正后，原厂负重档带载时的抖振在孪生里被夸大了 3～10 倍的问题消失。
  2. 齿轮积分数值发散：孪生把齿轮弹性分成若干小步积分，步数不够时，轮子一离地（下台阶）就数值爆炸，伪装成「下台阶是物理极限」长达一周。改成按刚度自动计算所需步数后，原厂负重档能直接过 10 毫米台阶。

- **Physics engine**: MuJoCo. The body, the two wheels, the motor torque limit, gear elasticity, friction and gyroscope noise are all in the model.
- **Firmware logic**: the twin runs a Python version copied line by line from the factory C code (balance loop, velocity loop, steering loop, dead-band compensation, 5-millisecond tick, one-tick output delay), so the simulation results for "factory Normal gains / Weight gains" can be used directly as the factory baseline.
- **Parameter source**: real-car measurements from September 23, 2026 (open-loop frequency sweeps, dead-band tests, etc.), recorded in `e026 keil/realcar/PARAMS.md`.
- **Twin errors that were fixed** (each one once led to a wrong conclusion; all have been corrected):
  1. Reverse dead-band compensation: the real firmware compensates 1480 in the forward direction and 1455 in reverse, but the twin originally compensated 1480 in both directions. After the fix, the problem of the twin exaggerating the chatter of the loaded factory Weight gains by 3 to 10 times disappeared.
  2. Numerical divergence of the gear integration: the twin integrates the gear elasticity in several small sub-steps. When there were too few sub-steps, the simulation blew up numerically as soon as a wheel left the ground (stepping down a step), which masqueraded as "stepping down is a physical limit" for a whole week. After changing it to compute the required number of sub-steps automatically from the stiffness, the factory Weight gains can drive straight over a 10-millimeter step.

孪生的局限也要说清楚：真车站立时的抖振比孪生强 3～5 倍，转弯时车身俯仰晃动的幅度也未经真车验证。所以**所有阈值类参数上真车前都要重新标定**（第 9 节）。

The limitations of the twin must also be stated clearly: the chatter of the real car while standing is 3 to 5 times stronger than in the twin, and the amplitude of body pitch oscillation while turning has not been verified on the real car. Therefore **all threshold-type parameters must be recalibrated before going onto the real car** (Section 9).

---

## 5. 设计历程：从交叉熵搜索（CEM）开始的每一步思考和实验 / 5. Design history: every step of reasoning and experiment, starting from the cross-entropy method (CEM)

这一节按时间顺序写，每一步都写「当时看到什么 → 怎么想的 → 做了什么 → 结果如何」。很多设计看起来奇怪，原因都在这里。

This section is written in chronological order, and each step records "what was observed at the time → how we reasoned → what was done → what the result was". Many design choices look strange; the reasons are all here.

### 5.1 交叉熵搜索（CEM）：用来找参数的工具 / 5.1 Cross-entropy method (CEM): the tool for finding parameters

**是什么**：参数搜索方法。设有 6 个待定参数，先给每个参数一个范围和初始猜测：

**What it is**: a parameter search method. Suppose there are 6 parameters to determine; first give each parameter a range and an initial guess:

1. 按当前的均值和分散程度随机生成 20 组候选参数；
2. 每组参数在孪生里跑一套固定的考题，得到一个「罚分」（越低越好）；
3. 挑出罚分最低的若干组（精英），用它们的均值和分散程度作为下一代的分布；
4. 重复，直到连续若干代都没有改善（「平台停止」）。

1. Randomly generate 20 candidate parameter sets from the current mean and spread;
2. Run each parameter set through a fixed set of test tasks in the twin to get a "penalty" (lower is better);
3. Pick the several sets with the lowest penalty (the elite), and use their mean and spread as the distribution for the next generation;
4. Repeat until several consecutive generations show no improvement ("plateau stop").

**考题和罚分**（以 v3 的搜索 `cem_v3.py` 为例）：

**Test tasks and penalty** (using the v3 search `cem_v3.py` as an example):

- 5 个工况：空载、1 千克、2 千克、4 千克、空载被推 3 牛；每个工况用 3 个种子；
- 每局任务：停-走-停，三段航点（前进 0.9 米、倒车到 0.3 米、再前进到 1.2 米），每段要开到距离目标 5 厘米内并停住 5 秒；
- 罚分 = 变稳时间（秒）+ 0.5 × 抖振读数；摔倒按没完成的段数罚 30 分；
- 各工况按重要性加权（空载 2、1 千克 2、2 千克 1、4 千克 1.5、被推 1）。权重是根据「当时的短板在哪」调整的，例如锁存以后 1 千克成了短板，就把它的权重从 1 提到 2。

- 5 conditions: empty, 1 kilogram, 2 kilograms, 4 kilograms, and empty with a 3-newton push; each condition uses 3 seeds;
- Task per run: stop-go-stop with three waypoint segments (forward 0.9 meters, reverse to 0.3 meters, forward again to 1.2 meters); each segment must reach within 5 centimeters of the target and hold still for 5 seconds;
- Penalty = settle time (seconds) + 0.5 × chatter reading; a fall is penalized 30 points per unfinished segment;
- The conditions are weighted by importance (empty 2, 1 kilogram 2, 2 kilograms 1, 4 kilograms 1.5, push 1). The weights were adjusted according to "where the weak spot was at the time"; for example, after latching was added, 1 kilogram became the weak spot, so its weight was raised from 1 to 2.

**工程上的处理**：
- 并行：16 个进程同时评估，一代从约 6 分钟（串行、每代 10 组）降到约 5 分钟（并行、每代 20 组），同样样本量快约 7 倍。并行版和串行版的结果逐格对照完全一致，才开始使用。
- 最终参数取**精英的均值**，不取「20 组里最好的那一组」。3 个种子下最好那组有运气成分，同一组参数重跑，罚分会从 54.50 变到 55.99，这个噪声比后期每代的改善（0.1～0.6）还大，所以到了噪声区就停。

**Engineering details**:
- Parallelism: 16 processes evaluate simultaneously. One generation went from about 6 minutes (serial, 10 sets per generation) to about 5 minutes (parallel, 20 sets per generation), which is about 7 times faster for the same number of samples. The parallel version was only put into use after its results matched the serial version exactly, cell by cell.
- The final parameters are the **mean of the elite**, not "the single best set out of 20". With only 3 seeds, the best set includes an element of luck: re-running the same parameter set changes the penalty from 54.50 to 55.99. This noise is larger than the per-generation improvement in later generations (0.1 to 0.6), so the search stops once it reaches the noise zone.

### 5.2 第一版、第二版：参数怎么搜都修不好 / 5.2 Versions 1 and 2: no parameter search could fix them

| 版本 | 结构 | 空载抖振 | 问题 |
|---|---|---|---|
| v1 | 开机固定探测 3 秒，然后慢慢调 f | 20.4 | 探测期间的剧烈抖动全被算进了判断 |
| v2 | 停车时判定，两段式跳档，**之后每一拍持续重新判断** | 9.0 | 见下 |

| Version | Structure | Empty chatter | Problem |
|---|---|---|---|
| v1 | Fixed 3-second probe at power-on, then slowly adjust f | 20.4 | The violent shaking during probing was all counted into the decision |
| v2 | Decide when stopped, two-stage gear jump, **then keep re-deciding on every tick** | 9.0 | See below |

CEM 在 v2 上搜了两轮，罚分从 70.77 降到 44.88（降了 37%），**空载抖振却始终降不下来**（9.0，原厂正常档是 0.6）。

CEM ran two rounds of search on v2. The penalty dropped from 70.77 to 44.88 (a 37% reduction), **but the empty chatter never came down** (9.0, versus 0.6 for the factory Normal gains).

逐拍追踪后找到根因。空车在第 61 拍时判断是正确的（f=0），但 f 一降、增益变软，车的抖振就从 17.44 掉到 5.90。而判断规则是「抖振小 = 有负载」，于是又把 f 推回 0.65，最后稳定在 0.43，空车一直用半个负重档。

Tick-by-tick tracing found the root cause. At tick 61 the empty car was judged correctly (f=0), but as soon as f dropped and the gains softened, the car's chatter fell from 17.44 to 5.90. The decision rule was "low chatter = loaded", so f was pushed back up to 0.65 and finally settled at 0.43: the empty car kept running on half of the Weight gains.

**结论**：抖振和负载的对应关系是在最硬档（f=1）下测出来的，只在 f=1 时成立。v2 却在任意 f 下使用它，这等于「标尺本身在动」。这是**结构问题，搜参数永远修不好**。

**Conclusion**: the mapping between chatter and load was measured at the stiffest gear (f=1) and only holds at f=1. v2 used it at arbitrary f, which amounts to "the measuring stick itself is moving". This is a **structural problem that no parameter search can ever fix**.

### 5.3 第三版（v3）：锁存，加上可衰减的安全逃生 / 5.3 Version 3 (v3): latching, plus a decaying safety escape

**改动一：锁存。** 只在最硬档、停车时判断**一次**，判断完就记住，不再用已经被自己改过的抖振重新判断。空载抖振 9.0 → 0.5。

**Change 1: latch.** Decide **once**, only at the stiffest gear and while stopped; remember the result and never re-decide using chatter that the controller has already altered itself. Empty chatter 9.0 → 0.5.

**改动二：安全逃生从「覆盖」改成「叠加并衰减」。** 第一版的逃生是：车长期歪着就把 f 永久改成 1。结果空车被推一下 3 牛，就被永久钉在负重档，抖振 33.5，相当于「人选错了档」。改成「在锁存值之上临时加一点，之后按 1.5 秒的时间常数衰减回去」之后，这一格的抖振从 33.5 变成 0.5。

**Change 2: the safety escape changed from "override" to "add on top and decay".** In the first version, the escape was: if the car stays tilted for a long time, permanently set f to 1. As a result, an empty car pushed once at 3 newtons was permanently pinned to the Weight gains, with chatter 33.5, which is equivalent to "a person picking the wrong gear". After changing it to "temporarily add a little on top of the latched value, then decay back with a 1.5-second time constant", the chatter in this cell went from 33.5 to 0.5.

CEM 在 v3 上的结果：罚分 38.78 → 27.10（−30%）。

CEM result on v3: penalty 38.78 → 27.10 (−30%).

**探测时机问题（9 月 30 日发现）**：v3 的参数是按「开局就走、停下才探测」调的，那时抖振读数在行驶中已经稳定。但实际上车是「上电就站着」，0.2 秒时读数还没升上来，1/2/4 千克全被判成半载。修正方法：探测要等**连续静止至少 1.2 秒**，并且最近 0.3 秒窗口内读数的「(最大−最小)/平均」不超过 0.15，即读数已经稳定，最多等 3 秒。判断也改成两档（轻/重），因为读数稳定以后只剩两团：轻的 20～33，重的 0.3～0.6，差了 40 倍以上，中间值没有意义。

**Probe-timing problem (found on September 30)**: the v3 parameters were tuned for "drive right away at the start, probe only after stopping", by which time the chatter reading had already stabilized during driving. In reality, however, the car "stands still right after power-on"; at 0.2 seconds the reading has not risen yet, and 1/2/4 kilograms were all judged as half load. Fix: probing waits for **at least 1.2 seconds of continuous standstill**, and requires that "(max − min) / mean" of the readings in the most recent 0.3-second window does not exceed 0.15, i.e. the reading has stabilized; it waits at most 3 seconds. The decision was also changed to two gears (light/heavy), because once the reading has stabilized there are only two clusters: light at 20 to 33 and heavy at 0.3 to 0.6, more than 40 times apart, so intermediate values are meaningless.

**另外两条搜索结论**：
- 第 2 层冲击门（检测到冲击时临时调增益）：CEM 三代零改善，平台停止。在测试的所有工况里，它挂上和不挂，结果逐位相同，因此不纳入算法。
- 下台阶：曾做过两轮「上限实验」，直接告诉车台阶在哪，让它提前调整，CEM 依然零改善。后来发现这是孪生齿轮积分发散造成的假象（第 4 节第 2 条），修好后原厂就能过 10 毫米台阶。

**Two more search conclusions**:
- Layer 2 shock gate (temporarily adjusting gains when a shock is detected): CEM showed zero improvement over three generations and hit the plateau stop. In every tested condition, the results with and without it were bit-for-bit identical, so it was not included in the algorithm.
- Stepping down a step: two rounds of "upper-bound experiments" were done, telling the car directly where the step was so it could adjust in advance, and CEM still showed zero improvement. It was later found that this was an artifact caused by the twin's gear integration divergence (Section 4, item 2); after the fix, the factory firmware can drive over a 10-millimeter step.

### 5.4 第四版（v4）的想法：升档和受冲击是同一件事 / 5.4 The idea behind version 4 (v4): upshifting and being hit are the same thing

v3 能判断负载，但被推、被撞时只能靠慢速逃生（要歪一段时间才起作用）。用户提出：

v3 can determine the load, but when pushed or hit it can only rely on the slow escape (which only takes effect after the car has been tilted for a while). The user proposed:

> 「升档和受冲击其实是一样的，都是倾角过大。只不过升档需要持续，冲激快速回调会振荡，会通过降档机制回来。」

> "Upshifting and being hit are really the same thing: both mean the tilt is too large. The only difference is that an upshift needs to be sustained, while a quick snap-back after an impulse would oscillate, and that will come back down through the downshift mechanism."

这个思路把两个问题统一了：

This idea unifies the two problems:

- **不管是负载变重还是被推，症状都是倾角过大，说明当前增益不够。** 统一的动作就是：立刻升到最硬档（f=1）。
- 最硬档有两个好处。一是它是最安全的一档。二是**它正好是能判断负载的标定点**。所以升档以后，顺便重新探测一次负载，读数自然会告诉我们该降回哪一档：轻车在最硬档上会一直抖，判轻、降回轻档；重车在最硬档上很安静，判重、留在重档。
- 这样「升档」和「重新判断负载」就合成了一个动作，而且判断总是在正确的标定点上做，不会重蹈 v2 的覆辙。

- **Whether the load became heavier or the car was pushed, the symptom is that the tilt is too large, which means the current gains are insufficient.** The unified action is: upshift immediately to the stiffest gear (f=1).
- The stiffest gear has two advantages. First, it is the safest gear. Second, **it is exactly the calibration point at which the load can be determined**. So after an upshift, the load is probed again along the way, and the reading naturally tells us which gear to drop back to: a light car keeps shaking at the stiffest gear, so it is judged light and downshifts back to the light gear; a heavy car is very quiet at the stiffest gear, so it is judged heavy and stays in the heavy gear.
- In this way "upshift" and "re-determining the load" merge into a single action, and the decision is always made at the correct calibration point, so the mistake of v2 is not repeated.

**怎么检测「倾角过大」**：用预测倾角 θ_pred = (倾角 − 平衡角) + 0.1 × 角速度。加上角速度项，是为了在车「正在快速倒」时提前约 0.1 秒反应。

**How "tilt too large" is detected**: using the predicted tilt θ_pred = (tilt − balance angle) + 0.1 × angular rate. The angular-rate term is added so that the controller reacts about 0.1 seconds earlier when the car "is falling fast".

**实验中逐步发现、逐步修正的问题**（每一条都对应源码里的一个机制）：

**Problems found and fixed one by one during experiments** (each corresponds to a mechanism in the source code):

1. **已在最硬档时还会反复触发**：最硬档下空车会自激抖动，角速度可达每秒 100 度以上，θ_pred 一直超过门限，每拍都触发重新探测，探测永远完成不了。实测 32 组候选全部卡死在最硬档。修正：只有在「已锁存且不在最硬档」时才允许升档（源码 `can_up`）。
2. **刚降档时的余振又把档升回去**：锁存或升档后，要求连续平稳 0.2 秒才「上膛」（源码 `armed`）。
3. **上膛判断写错导致永远不触发**：第一版每拍按「当前是否平稳」重新判断上膛。被推时，θ_pred 一路涨上去，先超过平稳界限、再到达门限，在到达门限的前一拍就把自己「卸膛」了。修正：上膛后保持住，只有锁存或升档才会解除。
4. **推击后重新探测把 2 千克判成轻**：推完 1.22 秒就锁存了，读数 8.32 是推击残余，不是稳态。修正：事件后的重新探测，要求 0.3 秒窗口内**每个**读数都超过 20 才判轻（`EVENT_SPLIT`）。
5. **f 大于 1 的紧急档**（用户提出：倾角速度过大时用比负重档更硬的增益，尽量逼近物理极限）：做了实验，救回次数与不开相同或更差。原因是到了需要它的时候，电机输出已经顶到 ±2800 的上限，增益再大也拿不出更多力矩。因此关闭。物理极限在孪生里是：空载 7 牛、2 千克 18 牛以上的推击，所有控制器都摔。
6. **门限怎么定**：静止时站立的正常晃动最大约 2 度，被推 6 牛后 20 毫秒就到 5 度，所以静止门限取 3 度。行驶中正常倾斜可到 8 度、刚停车可到 9 度，静止门限会不停误触发，所以行驶中用 16 度。

1. **Repeated triggering while already at the stiffest gear**: at the stiffest gear an empty car self-oscillates, with angular rates above 100 degrees per second, so θ_pred stays above the threshold, a new probe is triggered on every tick, and the probe can never complete. In testing, all 32 candidates got stuck at the stiffest gear. Fix: upshifting is allowed only when "latched and not at the stiffest gear" (source `can_up`).
2. **Residual oscillation right after a downshift upshifts the car again**: after a latch or an upshift, 0.2 seconds of continuous calm is required before "arming" (source `armed`).
3. **A wrongly written arming check meant it never triggered**: the first version re-evaluated arming on every tick based on "whether it is calm right now". When pushed, θ_pred rises steadily, first crossing the calm limit and then reaching the threshold, so the controller "disarmed" itself one tick before reaching the threshold. Fix: once armed, it stays armed and is cleared only by a latch or an upshift.
4. **Re-probing after a push judged 2 kilograms as light**: the latch happened only 1.22 seconds after the push, and the reading of 8.32 was push residue, not steady state. Fix: re-probing after an event requires **every** reading in the 0.3-second window to exceed 20 before judging light (`EVENT_SPLIT`).
5. **Emergency gear with f greater than 1** (proposed by the user: when the tilt rate is too large, use gains even stiffer than the Weight gains to get as close to the physical limit as possible): experiments showed the number of rescues was the same or worse than without it. The reason is that by the time it is needed, the motor output is already saturated at the ±2800 limit, and larger gains cannot produce more torque. It was therefore disabled. The physical limit in the twin is: with pushes above 7 newtons empty or 18 newtons at 2 kilograms, every controller falls.
6. **How the thresholds were set**: the normal sway while standing still is at most about 2 degrees, and a 6-newton push reaches 5 degrees within 20 milliseconds, so the standstill threshold is 3 degrees. While driving, the normal lean can reach 8 degrees, and right after stopping it can reach 9 degrees; the standstill threshold would trigger falsely all the time, so 16 degrees is used while driving.

### 5.5 v4 第二轮：在「不同的车」和坡上检验 / 5.5 v4 round 2: testing on "different cars" and on slopes

在随机化孪生（每个种子相当于一台略有不同的车）上发现两个问题：

On the randomized twin (each seed is equivalent to a slightly different car), two problems were found:

1. **坡上停车误升档**：平衡角原来一律用 2 秒时间常数慢慢跟踪，并且偏差超过 4 度就不跟。坡上停车后平衡姿态变了，静止门限启用时平衡角还没跟上（带载坡上的偏移本身就有约 4.8 度，被 4 度的限制挡住，永远跟不上）。修正：**双速跟踪**。行驶中和刚停车时快速跟踪（0.25 秒），不设限制；静止满 1 秒后慢速跟踪（2 秒），并设 4 度限制，防止推击被当成平衡角变化吸收掉。
2. **晃动大的车误升档**：有些车本身晃得多，固定 3 度门限会误触发（随机化车上空载抖振 3.0～4.4，而 v3 只有 1.5）。真车站立时的晃动比孪生大 3～5 倍，只会更严重。修正：**门限跟着本车的噪声走**，门限 = max(3 度, 6 × 本车静止时预测倾角的均方根)。

1. **False upshift when stopping on a slope**: the balance angle was originally always tracked slowly with a 2-second time constant, and tracking stopped if the deviation exceeded 4 degrees. After stopping on a slope the balance posture changes, and when the standstill threshold became active the balance angle had not caught up yet (the offset of a loaded car on a slope is itself about 4.8 degrees, which is blocked by the 4-degree limit, so it can never catch up). Fix: **dual-speed tracking**. While driving and right after stopping, track fast (0.25 seconds) with no limit; after 1 full second of standstill, track slowly (2 seconds) with the 4-degree limit, so that a push is not absorbed as a change in the balance angle.
2. **False upshift on cars that sway a lot**: some cars naturally sway more, and the fixed 3-degree threshold triggers falsely (empty chatter on randomized cars is 3.0 to 4.4, while v3 has only 1.5). The real car sways 3 to 5 times more than the twin while standing, so the problem would only be worse. Fix: **the threshold follows the car's own noise**: threshold = max(3 degrees, 6 × the root mean square of this car's predicted tilt at standstill).

还加了**快速撤回**：小扰动（比如静止时被推 5 度）触发升档后，原来要等 1.2 秒以上的重新探测，空车在最硬档上抖，回稳要 1.31 秒。改为：升档 0.3 秒后检查，如果这段时间里低通滤波后的预测倾角峰值没有超过 6 度，就判定只是小事件、负载没变，直接退回原来的档位。回稳时间降到 0.60～0.67 秒。

A **quick revert** was also added: after a small disturbance (for example, being pushed 5 degrees while standing still) triggers an upshift, the car originally had to wait more than 1.2 seconds for the re-probe, during which the empty car shook at the stiffest gear, and it took 1.31 seconds to settle. It was changed to: check 0.3 seconds after the upshift; if the peak of the low-pass-filtered predicted tilt during that time did not exceed 6 degrees, judge it a small event with no change in load and revert directly to the previous gear. The settling time dropped to 0.60 to 0.67 seconds.

**参数怎么选的**：门限下限 {2, 3} 度 × 噪声倍数 {0, 4, 6} × 撤回门限 {关, 6, 10} 度，共 18 组，加上 v3 作为对照，在标称车和随机化车上各跑推击、正常行驶、小扰动三类测试（脚本 `tune_v4_r2.py`）。选中 3 度 / 6 倍 / 6 度：正常行驶 15 局零误升档，随机化车空载抖振 1.55（和 v3 相同），推击得分 27/32（v3 是 13，改之前是 23）。

**How the parameters were chosen**: threshold floor {2, 3} degrees × noise multiplier {0, 4, 6} × revert threshold {off, 6, 10} degrees, 18 sets in total, plus v3 as a control. Each was run on the nominal car and on randomized cars through three kinds of tests: push, normal driving and small disturbance (script `tune_v4_r2.py`). The chosen setting was 3 degrees / 6 times / 6 degrees: zero false upshifts in 15 normal-driving runs, empty chatter of 1.55 on randomized cars (the same as v3), and a push score of 27/32 (v3 scored 13; before the change it was 23).

### 5.6 v4 第三轮：「任何情况下都要逻辑自洽地收敛」 / 5.6 v4 round 3: "converge in a logically consistent way under all circumstances"

用户的要求是：任何情况下，控制系统都要能逻辑自洽地收敛。这一轮确立了一条统一原则：

The user's requirement was: under all circumstances, the control system must converge in a logically consistent way. This round established one unifying principle:

> **只在最硬档判断负载；只要负载可能变了、或者当前档位可能不够，就回到最硬档重新判断。最硬档对任何负载都安全，判断一定能在有限时间内完成。**

> **Determine the load only at the stiffest gear; whenever the load may have changed, or the current gear may be insufficient, go back to the stiffest gear and decide again. The stiffest gear is safe for any load, so the decision is guaranteed to finish in finite time.**

按这个原则做的修改，以及为什么有的方案没采用：

The changes made according to this principle, and why some proposals were not adopted:

1. **停稳门（采用）**：急刹测试发现，第二轮 v4 在 0.6 米每秒急刹后误升档（空载 8 次里 6 次）。而刹车后 1 秒内预测倾角峰值只有 10～12 度，远低于行驶门限 16 度。**原因不是门限太低，而是门限切换的时机错了**：停车满 1 秒就切到 3 度门限，可车急刹后要 1.1～1.5 秒才晃停。坡上停车误升档也是同一个原因。修正：停车后，车要先连续 0.6 秒预测倾角小于 2.5 度，才启用静止门限；之后保持到下次起步。平稳时间比较了 0.2/0.4/0.6 秒，急刹误升档分别是 5/144、2/144、0/144。「停车后 0.3 秒被推」的保护不受影响，因为那段时间由 16 度行驶门限兜底。
2. **转向也算指令（采用）**：原地转弯时车身会晃，原来只看前后速度指令，把转弯当成静止，于是噪声估计把转弯晃动学了进去，门限被抬到 16.8 度。Python 和固件都改了。
3. **开机判负载「有把握才判轻」（采用）**：随机化 16 台车实测，分界值 3.0 不可靠：2 千克有 3 台读 18～19、4 千克有 1 台读 8.2，全被判成轻，其中一台下台阶时摔了。两种错判的代价不对等：**重判轻会摔，轻判重只是偏硬一点、照样收敛**。所以分界值应该贴着空载那一团放：空载最低 24.3，带载最高 22.1，取 23。改后 16 台全部判对；齿轮刚度敏感性测试里 v3 从 41 升到 44（满分 48）。
4. **噪声门限防失控（采用）**：随机场地测试发现，车**真的歪了**（静止时歪 −20 度、持续好几秒），也被当成「噪声」学进去，门限从 54 度一路涨到 118 度，保护等于自己把自己关掉了。修正：只从门限以内的晃动学噪声，并把门限封顶在 8 度（正常运行里见过的最高值约 8.7 度）。v4 在 64 局里的摔倒从 22 局降到 19 局。
5. **停车复查（测试后不采用）**：每次行驶后停稳，都回最硬档重新判断。原本是想解决「负载静静地改变了却不知道」。但在随机测试里开了它，摔倒 19 → 22 局、抖振 8 → 41 次，加载后判对的次数没有变化（都是 22 次）。原因是：在轻档和中档下，站着时的抖振、平衡角、速度环积分对 0～4 千克**全部重叠**，没有任何被动信号能分辨负载，只能靠回最硬档；而频繁回最硬档的代价比收益大。
6. **重载类行驶门限降低（测试后不采用）**：降到 6 度能救回「2 千克行驶中被推 8 牛」，但正常行驶误升档 18 次，急刹也会触发（2 千克 0.45 米每秒急刹 8 次全部误升档）。带载行驶中，「被推」和「正常开车、急刹」靠倾角分辨不开。
7. **两档数值重新调整（测试后不采用）**：分界改成 23 以后，轻档只剩空载、重档覆盖 1～4 千克，于是重新搜了轻档 {0, 0.06, 0.12} × 重档 {0.40, 0.55, 0.70, 0.85}。重档调硬能少摔约 4/64 局，这在统计波动范围内（64 局约 ±3 局），代价是 1 千克在更硬档上抖，持续抖振 8 → 约 50 次。按用户的规则「没有明显进步就用原来的」，保留 0.12 / 0.40。

1. **Stop-calm gate (adopted)**: hard-brake tests showed that the round-2 v4 upshifted falsely after a hard brake from 0.6 meters per second (6 out of 8 times empty). Yet the peak predicted tilt within 1 second after braking was only 10 to 12 degrees, far below the 16-degree driving threshold. **The cause was not that the threshold was too low, but that the threshold switched at the wrong time**: after 1 full second of being stopped it switched to the 3-degree threshold, but after a hard brake the car needs 1.1 to 1.5 seconds to stop swaying. False upshifts when stopping on a slope had the same cause. Fix: after stopping, the car must first have a predicted tilt below 2.5 degrees for 0.6 consecutive seconds before the standstill threshold becomes active; it then stays active until the next start. Calm durations of 0.2/0.4/0.6 seconds were compared, giving 5/144, 2/144 and 0/144 false upshifts on hard brakes respectively. Protection against "being pushed 0.3 seconds after stopping" is not affected, because during that time the 16-degree driving threshold acts as the backstop.
2. **Steering also counts as a command (adopted)**: the body sways when turning in place. Originally only the forward/backward speed command was considered, so turning was treated as standstill, the noise estimate learned the turning sway, and the threshold was pushed up to 16.8 degrees. Both the Python code and the firmware were changed.
3. **Power-on load decision: "judge light only when certain" (adopted)**: in tests on 16 randomized cars, the split value 3.0 was unreliable: 3 cars at 2 kilograms read 18 to 19 and 1 car at 4 kilograms read 8.2, and all were judged light; one of them fell when stepping down a step. The costs of the two kinds of misjudgment are not equal: **judging heavy as light causes a fall, while judging light as heavy is only a bit too stiff and still converges**. So the split value should sit right next to the empty cluster: the lowest empty reading is 24.3 and the highest loaded reading is 22.1, so 23 was chosen. After the change, all 16 cars were judged correctly; in the gear-stiffness sensitivity test, v3 rose from 41 to 44 (out of 48).
4. **Preventing runaway of the noise threshold (adopted)**: random-terrain tests showed that when the car **really was tilted** (tilted −20 degrees at standstill for several seconds), this was also learned as "noise", and the threshold climbed from 54 degrees all the way to 118 degrees, so the protection effectively switched itself off. Fix: learn noise only from sway within the threshold, and cap the threshold at 8 degrees (the highest value seen in normal operation is about 8.7 degrees). Falls for v4 over 64 runs dropped from 22 to 19.
5. **Re-check after stopping (tested, not adopted)**: after every drive, once the car has stopped and settled, go back to the stiffest gear and decide again. This was meant to solve "the load changed quietly without our knowing". But turning it on in the random tests changed falls 19 → 22 runs and sustained-chatter events 8 → 41, while the number of correct decisions after loading did not change (22 both times). The reason is: in the light and middle gears, the standing chatter, balance angle and velocity-loop integral **all overlap** for 0 to 4 kilograms; no passive signal can distinguish the load, so the only way is to go back to the stiffest gear, and going back to it frequently costs more than it gains.
6. **Lowering the driving threshold for heavy-load classes (tested, not adopted)**: lowering it to 6 degrees could rescue "2 kilograms pushed at 8 newtons while driving", but normal driving produced 18 false upshifts, and hard brakes also triggered it (2 kilograms, hard brake from 0.45 meters per second: all 8 times were false upshifts). When driving loaded, "being pushed" and "normal driving or hard braking" cannot be told apart by tilt.
7. **Re-tuning the values of the two gears (tested, not adopted)**: after the split was changed to 23, the light gear covered only the empty car and the heavy gear covered 1 to 4 kilograms, so the light gear {0, 0.06, 0.12} × heavy gear {0.40, 0.55, 0.70, 0.85} were searched again. A stiffer heavy gear reduced falls by about 4/64 runs, which is within statistical fluctuation (about ±3 runs over 64), at the cost of 1 kilogram shaking at the stiffer gear, with sustained chatter going 8 → about 50 times. Following the user's rule "if there is no clear improvement, keep the original", 0.12 / 0.40 was retained.

### 5.7 测试方法本身也走过弯路（值得记住的教训） / 5.7 The testing method itself also took wrong turns (lessons worth remembering)

- **悬崖格的小样本**：空载 6 牛推击曾经从 4/4 变成 2/4，看起来像退步。逐项排查后把试验次数扩到 8 次，各种版本都在 4/8～5/8 之间摆动。处在「摔与不摔」边缘的格子，4 次一组的结果分不出差别。
- **随机场地测试撞墙**：第一版随机测试里，原厂和 v4 大面积摔倒。用户指出「在 MuJoCo 里小车开得很好」，查出孪生场地四周 ±4 米有墙，闭眼随机开会撞上去。改成开阔场地后，只按键开车的对照组零摔倒。**教训：新测试里连基线都大面积失败，先查测试环境，再怀疑控制器。**
- **测试动作要符合实车**：最初的随机测试有「缓停」（位置环减速）和「斜向」（前进和转弯同时按满）。用户指出实车只有前后左右四个键、松手即停，这两种动作都改掉了。

- **Small samples in cliff-edge cells**: the empty 6-newton push cell once went from 4/4 to 2/4, which looked like a regression. After checking item by item, the number of trials was increased to 8, and every version fluctuated between 4/8 and 5/8. For cells sitting on the edge between "falls" and "does not fall", groups of 4 trials cannot distinguish any difference.
- **Random-terrain tests hitting walls**: in the first version of the random tests, both the factory firmware and v4 fell in large numbers. The user pointed out that "the car drives fine in MuJoCo", and it turned out that the twin's arena had walls at ±4 meters on all sides, which blind random driving would run into. After switching to an open arena, the control group that only drove with the keys had zero falls. **Lesson: if even the baseline fails widely in a new test, check the test environment first before suspecting the controller.**
- **Test maneuvers must match the real car**: the initial random tests included "gentle stop" (deceleration by a position loop) and "diagonal" (forward and turn pressed fully at the same time). The user pointed out that the real car has only four keys (forward, backward, left, right) and stops as soon as the key is released, so both maneuvers were removed.

---

## 6. 源码逐段说明：每一部分做什么、为什么这样设计 / 6. Source code walkthrough: what each part does and why it is designed this way

v4 在固件里分成两个文件，这样拆分有明确的目的：

In the firmware, v4 is split into two files, and the split has a clear purpose:

- `v4_core.c` / `v4_core.h`：**纯逻辑**。输入只有三个数：倾角（度）、陀螺原始读数（LSB）、「当前有没有遥控指令」；输出只有一个数：档位系数 f。不读写任何固件全局变量，不碰硬件。这样同一份文件既能放进 Keil 编译到单片机，也能用电脑上的 gcc 编译，和 Python 版逐拍对拍（第 7 节）。
- `v4_adapt.c` / `v4_adapt.h`：**胶水层**。从固件全局变量里读出上面三个输入，调用 `V4_Step()`，再把 f 换算成六个增益写回原厂变量。

- `v4_core.c` / `v4_core.h`: **pure logic**. It takes only three inputs: the tilt angle (degrees), the raw gyro reading (LSB), and "is there a remote-control command right now"; it produces only one output: the gain coefficient f. It does not read or write any firmware global variable and does not touch the hardware. This way the same file can be compiled in Keil for the microcontroller and also compiled with gcc on a PC for a tick-by-tick comparison against the Python version (Section 7).
- `v4_adapt.c` / `v4_adapt.h`: **glue layer**. It reads the three inputs above from firmware global variables, calls `V4_Step()`, then converts f into six gains and writes them back into the factory variables.

### 6.1 `v4_core.h`：状态结构体 / 6.1 `v4_core.h`: the state structure

`V4_State` 保存 v4 的全部记忆，每个字段的作用：

`V4_State` holds all of v4's memory. The role of each field:

| 字段 | 作用 |
|---|---|
| `hp, bp, en, osc` | 抖振滤波器的三级状态（高通、带通、能量）和最终读数。 |
| `latched` | 是否已经判断好负载并锁存。 |
| `event_probe` | 本次探测是不是「事件之后」的重新探测（判轻用更严的标准）。 |
| `move_latched` | 是否是在行驶中判轻的（只用于记录）。 |
| `f, f_latched` | 当前档位系数，以及锁存的档位（f 可能因慢速逃生而高于锁存值）。 |
| `n_still, n_move` | 连续静止、连续行驶的拍数。 |
| `win[60]`、`mwin[100]` | 最近 0.3 秒（静止）、0.5 秒（行驶）的抖振读数环形缓冲，用来判断「读数是否已稳定」和「是否每个读数都超过分界」。 |
| `struggle, boost` | 慢速逃生：长期偏离程度的低通值、临时加上去的 f。 |
| `th_ref, th_pred` | 平衡角、预测倾角。 |
| `n_cmd_still, arm_cnt, armed, was_latched` | 静止计时、上膛计数、是否上膛、上一拍是否锁存（用于识别「刚锁存」这一刻）。 |
| `rec, calm` | 是否处于「大倾角恢复中」、恢复后的连续平稳拍数。 |
| `om_lp, noise, on_still` | 低通后的角速度（给快速撤回用）、静止噪声的均方估计、当前静止门限。 |
| `qr_active, qr_n, qr_peak, qr_prev` | 快速撤回：是否在检查期、已过拍数、期间峰值、升档前的档位。 |
| `stop_calm, stop_calm_n, n_drive` | 停稳门：是否已停稳、连续平稳拍数、本段行驶拍数。 |
| `rec_ticks, rec_entries, rearms, qr_reverts, escape_ticks, stop_probes` | 只用于统计和屏幕显示的计数器。 |
| `fwin[60]`、`n_unl`、`fast_latches` | （10-03 新增）快速判轻：从上次解锁起单独记录的 0.3 秒抖振窗口、解锁后的拍数、快速判轻次数（12.3 问题 3、4）。 |
| `edge_n`、`edge_skips` | （10-03 新增）宽限期计时：距离上次按键变化（或降档宽限开始）的拍数；被宽限期挡掉的升档拍数（只用于统计）（12.3 问题 5、7）。 |
| `up_n`、`relatch_graces` | （10-03 新增）距离上次升档的拍数；降档宽限期触发次数（12.3 问题 7）。 |

| Field | Role |
|---|---|
| `hp, bp, en, osc` | The three stages of the chatter filter state (high-pass, band-pass, energy) and the final reading. |
| `latched` | Whether the load has already been decided and latched. |
| `event_probe` | Whether the current probe is a re-probe "after an event" (a stricter criterion is used for deciding "light"). |
| `move_latched` | Whether the "light" decision was made while driving (for logging only). |
| `f, f_latched` | The current gain coefficient, and the latched gain level (f may be higher than the latched value because of slow escape). |
| `n_still, n_move` | Number of consecutive ticks standing still / driving. |
| `win[60]`, `mwin[100]` | Ring buffers of chatter readings over the last 0.3 seconds (standing still) and 0.5 seconds (driving), used to judge "has the reading settled" and "is every reading above the split". |
| `struggle, boost` | Slow escape: the low-pass value of the long-term deviation, and the temporary amount added to f. |
| `th_ref, th_pred` | Balance angle, predicted tilt. |
| `n_cmd_still, arm_cnt, armed, was_latched` | Still-time counter, arming counter, whether armed, whether latched on the previous tick (used to detect the moment "just latched"). |
| `rec, calm` | Whether the car is "in recovery from a large tilt", and the number of consecutive calm ticks after recovery. |
| `om_lp, noise, on_still` | Low-pass-filtered angular rate (used by quick revert), mean-square estimate of standing-still noise, the current standing-still threshold. |
| `qr_active, qr_n, qr_peak, qr_prev` | Quick revert: whether the check period is active, ticks elapsed, peak value during the period, gain level before the upshift. |
| `stop_calm, stop_calm_n, n_drive` | Stop-calm gate: whether the car has come to a calm stop, number of consecutive calm ticks, number of ticks in the current driving segment. |
| `rec_ticks, rec_entries, rearms, qr_reverts, escape_ticks, stop_probes` | Counters used only for statistics and the screen display. |
| `fwin[60]`, `n_unl`, `fast_latches` | (Added 10-03) Fast "light" decision: a separate 0.3-second chatter window recorded since the last unlatch, the number of ticks since unlatching, and the number of fast "light" decisions (Section 12.3, issues 3 and 4). |
| `edge_n`, `edge_skips` | (Added 10-03) Grace period timer: number of ticks since the last key change (or since the start of the post-shock grace period); number of upshift ticks blocked by the grace period (statistics only) (Section 12.3, issues 5 and 7). |
| `up_n`, `relatch_graces` | (Added 10-03) Number of ticks since the last upshift; number of times the post-shock (relatch) grace period was triggered (Section 12.3, issue 7). |

所有字段都是固定大小，总共约 1 千字节，不使用动态内存。这对单片机很重要，内存占用编译时就确定，不会出现运行中内存不足。

All fields have a fixed size, about 1 kilobyte in total, and no dynamic memory is used. This matters on a microcontroller: memory usage is fixed at compile time, so the program cannot run out of memory at run time.

### 6.2 `v4_core.c`：常数和取值理由 / 6.2 `v4_core.c`: constants and the reasons for their values

每个常数在源码里都注明了它来自 Python 的哪个参数。

In the source code, each constant is annotated with the Python parameter it comes from.

**抖振和探测（来自 v3）**

**Chatter and probing (inherited from v3)**

| 常数 | 值 | 理由 |
|---|---|---|
| `HP_A, LP_A, EN_A` | 0.2008, 0.3345, 0.01639 | 8 赫兹高通、16 赫兹低通、约 0.3 秒的能量平均。系数照抄原厂已有的负载检测代码 `load_adapt.c`，保证读数和以前录过的数据可比。8～16 赫兹正好是负重档下空车自激抖动的频段。 |
| `PROBE_MIN_N` | 240 拍 = 1.2 秒 | 探测至少等这么久。上电时读数要 1 秒左右才升上来（5.3 节）。 |
| `PROBE_MAX_N` | 600 拍 = 3 秒 | 读数一直不稳也最多等 3 秒，保证探测一定会结束（逻辑自洽要求「有限时间内收敛」）。 |
| `CONV_TOL` | 0.15 | 0.3 秒窗口内 (最大−最小)/平均 ≤ 0.15，认为读数已稳定。 |
| `OSC_SPLIT` | 23 | 开机判负载的分界：读数 ≥ 23 判轻（空载），否则判重。理由见 5.6 第 3 条：有把握才判轻。**实车需要重新标定**。 |
| `F_LIGHT, F_HEAVY` | **0**（10-02 版是 0.12）, 0.40 | 轻档、重档的档位系数。0.12 和 0.40 由 CEM 搜索得到。**10-03 实车上 0.12 让空车一直抖**（实车空载在 f=0 时抖振 4.5～7.4，f=0.25～0.5 时 53.8～58.3，孪生低估约 30 倍），所以轻档改为 0，也就是空载时完全使用原厂模式 1 的 PID（12.3 问题 2）。重档 0.40（平衡比例 134）比原厂负重档（192）软，因为原厂负重档对 4 千克也偏硬（v3 在 4 千克上抖振 0.4，原厂负重档 1.0）。 |
| `MOVE_SPLIT, MOVE_MIN_N` | 20, 1 秒 | 行驶中也允许判轻，但条件很严：在最硬档上连续行驶 1 秒以上，且最近 0.5 秒每个读数都 ≥ 20。行驶中只允许判轻、不判重，因为空车在最硬档上开很抖，越早降下来越好；带载车留在最硬档没有坏处，等停车再判。 |
| `EVENT_SPLIT` | 20（实际取 max(23, 20) = 23） | 事件后重新探测时，判轻要求窗口内每个读数都超过分界，防止推击残余被误判（5.4 第 4 条）。 |
| `RATE_UP, STRUGGLE_DEG, STRUGGLE_LPF, BOOST_DECAY` | 0.022/拍、7.1 度、0.01、时间常数 1.5 秒 | 慢速逃生：偏离平衡角的低通值超过 7.1 度，就每拍把 f 加 0.022；不再吃力后按 1.5 秒时间常数衰减回去。数值由 CEM 搜索得到。 |

| Constant | Value | Reason |
|---|---|---|
| `HP_A, LP_A, EN_A` | 0.2008, 0.3345, 0.01639 | 8 hertz high-pass, 16 hertz low-pass, and an energy average over about 0.3 seconds. The coefficients are copied from the factory's existing load-detection code `load_adapt.c`, so the readings are comparable with previously recorded data. The 8 to 16 hertz band is exactly the band of the self-excited oscillation of an empty car on the Weight gains. |
| `PROBE_MIN_N` | 240 ticks = 1.2 seconds | Probing waits at least this long. After power-on the reading takes about 1 second to rise (Section 5.3). |
| `PROBE_MAX_N` | 600 ticks = 3 seconds | Even if the reading never settles, probing waits at most 3 seconds, so probing is guaranteed to end (logical consistency requires "convergence in finite time"). |
| `CONV_TOL` | 0.15 | If (maximum − minimum)/mean ≤ 0.15 within the 0.3-second window, the reading is considered settled. |
| `OSC_SPLIT` | 23 | The split for the power-on load decision: a reading ≥ 23 is judged light (empty), otherwise heavy. For the reason, see item 3 of Section 5.6: decide "light" only when confident. **Must be recalibrated on the real car.** |
| `F_LIGHT, F_HEAVY` | **0** (0.12 in the 10-02 version), 0.40 | Gain coefficients for the light level and the heavy level. 0.12 and 0.40 were found by the CEM search. **On the real car on 10-03, 0.12 made the empty car chatter constantly** (on the real car, empty, the chatter is 4.5 to 7.4 at f=0 and 53.8 to 58.3 at f=0.25 to 0.5; the twin underestimates it by about 30 times), so the light level was changed to 0, which means that when empty the car uses exactly the factory mode 1 PID (Section 12.3, issue 2). The heavy level 0.40 (balance proportional gain 134) is softer than the factory Weight gains (192), because the factory Weight gains are on the stiff side even for 4 kilograms (on 4 kilograms, v3 has chatter 0.4 versus 1.0 for the factory Weight gains). |
| `MOVE_SPLIT, MOVE_MIN_N` | 20, 1 second | A "light" decision is also allowed while driving, but the conditions are strict: the car has been driving continuously on the stiffest level for more than 1 second, and every reading in the last 0.5 seconds is ≥ 20. While driving, only "light" may be decided, never "heavy", because an empty car chatters badly when driving on the stiffest level, so the sooner it downshifts the better; a loaded car loses nothing by staying on the stiffest level, and the decision can wait until it stops. |
| `EVENT_SPLIT` | 20 (effectively max(23, 20) = 23) | When re-probing after an event, a "light" decision requires every reading in the window to exceed the split, so that residual motion from a push is not misjudged (item 4 of Section 5.4). |
| `RATE_UP, STRUGGLE_DEG, STRUGGLE_LPF, BOOST_DECAY` | 0.022/tick, 7.1 degrees, 0.01, time constant 1.5 seconds | Slow escape: when the low-pass value of the deviation from the balance angle exceeds 7.1 degrees, f is increased by 0.022 every tick; once the car is no longer struggling, the boost decays back with a 1.5-second time constant. The values were found by the CEM search. |

**大倾角升档（来自 v4）**

**Large-tilt upshift (new in v4)**

| 常数 | 值 | 理由 |
|---|---|---|
| `REC_S` | 0.10 秒 | 预测倾角的提前量。 |
| `REC_ON_STILL` | 3 度 | 静止门限下限（5.4 第 6 条）。 |
| `REC_ON_MOVE` | 16 度 | 行驶门限（5.4 第 6 条）。急刹测试中，空载到 2 千克刹车后的预测倾角峰值是 6～12 度，有余量。 |
| `STILL_N` | 200 拍 = 1 秒 | 指令归零满 1 秒才算「静止」。 |
| `ARM_DEG, ARM_N` | 2.5 度、0.2 秒 | 上膛条件（5.4 第 2、3 条）。0.2 秒原来是 0.5 秒，改短是因为 2 千克要 2.5～2.9 秒才锁存，再等 0.5 秒上膛，站 3 秒就被推的那几局正好落在空窗里。 |
| `REC_OFF_DEG, REC_OFF_DPS, REC_OFF_TICKS` | 3 度、60 度/秒、4 拍 | 退出「恢复中」状态的条件。 |
| `REF_RATE, REF_RATE_FAST, REF_CALM_DEG` | 2 秒、0.25 秒、4 度 | 平衡角双速跟踪（5.5 第 1 条）。 |
| `NOISE_K, NOISE_RATE, NOISE_CAP` | 6 倍、2 秒平均、上限 8 度 | 噪声自适应门限（5.5 第 2 条、5.6 第 4 条）。 |
| `QR_N, QR_PEAK, OMLP_A` | 0.3 秒、6 度、3 赫兹低通 | 快速撤回（5.5）。用 3 赫兹低通后的角速度算峰值，是为了滤掉最硬档下空车的高频自激，只看车身真正的大幅运动。 |
| `STOP_CALM, STOP_CALM_N` | 开、0.6 秒 | 停稳门（5.6 第 1 条）。 |
| `REPROBE_ON_STOP` | 关 | 停车复查，测试后不采用（5.6 第 5 条），代码保留以便实车再评估。 |
| `REC_ON_MOVE_HEAVY` | 关 | 重载类行驶门限，测试后不采用（5.6 第 6 条），代码保留。 |
| `CLASS_SPLIT_F` | 0.26 | 区分「轻档类」和「重档类」的 f 界线（轻档和 0.40 之间）。 |
| `FAST_LIGHT_N` | 100 拍 = 0.5 秒 | （10-03 新增）快速判轻：解锁满 0.5 秒后，最近 0.3 秒每个抖振读数都 ≥ 23 就立刻判轻，静止和行驶都适用（12.3 问题 3、4）。 |
| `EDGE_GRACE_N` | 300 拍 = 1.5 秒 | （10-03 新增）按键宽限期：蓝牙按键变化后 1.5 秒内，升档只看倾角偏差、不加角速度项（12.3 问题 5）。 |
| `STRUGGLE_IN_REC` | 0 | （10-03 新增）0 = 「恢复中」时慢速脱困暂停累计（12.3 问题 6）；1 = 10-02 版的旧行为。 |
| `RELATCH_GRACE`、`RELATCH_GRACE_N`、`RELATCH_GRACE_WITHIN_N` | 开、400 拍 = 2 秒、600 拍 = 3 秒 | （10-03 新增）降档宽限期：升档后 3 秒内重新锁定档位时，之后 2 秒升档只看倾角偏差，并且重新等车停稳才用静止门限（12.3 问题 7）。 |
| `GN, GH` | 第 3.3 节的两套增益 | 插值的两个端点。 |

| Constant | Value | Reason |
|---|---|---|
| `REC_S` | 0.10 seconds | Look-ahead time of the predicted tilt. |
| `REC_ON_STILL` | 3 degrees | Lower bound of the standing-still threshold (item 6 of Section 5.4). |
| `REC_ON_MOVE` | 16 degrees | Driving threshold (item 6 of Section 5.4). In the hard-braking tests, the peak predicted tilt after braking, from empty to 2 kilograms, was 6 to 12 degrees, so there is margin. |
| `STILL_N` | 200 ticks = 1 second | The car counts as "standing still" only after the command has been zero for a full 1 second. |
| `ARM_DEG, ARM_N` | 2.5 degrees, 0.2 seconds | Arming conditions (items 2 and 3 of Section 5.4). The 0.2 seconds was originally 0.5 seconds. It was shortened because with 2 kilograms latching takes 2.5 to 2.9 seconds; waiting another 0.5 seconds to arm meant that the runs where the car was pushed after standing for 3 seconds fell exactly into the unprotected gap. |
| `REC_OFF_DEG, REC_OFF_DPS, REC_OFF_TICKS` | 3 degrees, 60 degrees/second, 4 ticks | Conditions for leaving the "in recovery" state. |
| `REF_RATE, REF_RATE_FAST, REF_CALM_DEG` | 2 seconds, 0.25 seconds, 4 degrees | Two-speed tracking of the balance angle (item 1 of Section 5.5). |
| `NOISE_K, NOISE_RATE, NOISE_CAP` | 6 times, 2-second average, upper limit 8 degrees | Noise-adaptive threshold (item 2 of Section 5.5, item 4 of Section 5.6). |
| `QR_N, QR_PEAK, OMLP_A` | 0.3 seconds, 6 degrees, 3 hertz low-pass | Quick revert (Section 5.5). The peak is computed from the 3 hertz low-pass angular rate in order to filter out the high-frequency self-excited oscillation of an empty car on the stiffest level and look only at genuinely large body motion. |
| `STOP_CALM, STOP_CALM_N` | On, 0.6 seconds | Stop-calm gate (item 1 of Section 5.6). |
| `REPROBE_ON_STOP` | Off | Re-probe on stop; not adopted after testing (item 5 of Section 5.6). The code is kept so it can be re-evaluated on the real car. |
| `REC_ON_MOVE_HEAVY` | Off | Driving threshold for the heavy class; not adopted after testing (item 6 of Section 5.6). The code is kept. |
| `CLASS_SPLIT_F` | 0.26 | The f boundary that separates the "light-level class" from the "heavy-level class" (between the light level and 0.40). |
| `FAST_LIGHT_N` | 100 ticks = 0.5 seconds | (Added 10-03) Fast "light" decision: once 0.5 seconds have passed since unlatching, if every chatter reading in the last 0.3 seconds is ≥ 23, "light" is decided immediately; this applies both standing still and driving (Section 12.3, issues 3 and 4). |
| `EDGE_GRACE_N` | 300 ticks = 1.5 seconds | (Added 10-03) Key-change grace period: within 1.5 seconds after a Bluetooth key change, the upshift decision uses only the tilt deviation, without the angular-rate term (Section 12.3, issue 5). |
| `STRUGGLE_IN_REC` | 0 | (Added 10-03) 0 = slow escape stops accumulating while "in recovery" (Section 12.3, issue 6); 1 = the old behavior of the 10-02 version. |
| `RELATCH_GRACE`, `RELATCH_GRACE_N`, `RELATCH_GRACE_WITHIN_N` | On, 400 ticks = 2 seconds, 600 ticks = 3 seconds | (Added 10-03) Post-shock (relatch) grace period: when the gain level is latched again within 3 seconds after an upshift, for the next 2 seconds the upshift decision uses only the tilt deviation, and the car must again come to a calm stop before the standing-still threshold is used (Section 12.3, issue 7). |
| `GN, GH` | The two gain sets from Section 3.3 | The two endpoints of the interpolation. |

为了做对拍测试，部分常数用 `#ifndef` 包起来，电脑编译时可以用 `-D` 临时改值；Keil 编译时用的就是文件里写的默认值。

For the tick-by-tick comparison tests, some constants are wrapped in `#ifndef`, so their values can be temporarily overridden with `-D` when compiling on a PC; the Keil build uses the default values written in the file.

### 6.3 `v4_core.c`：函数逐个说明 / 6.3 `v4_core.c`: the functions one by one

**`V4_GainsFor(f, 增益)`**：把 f 线性插值成六个增益：增益 = 正常档 + (负重档 − 正常档) × f。用线性插值，是因为两个端点都是原厂验证过的稳定参数，中间值在孪生里全部稳定，而且只有一个自由度 f，判断逻辑可以很简单。

**`V4_GainsFor(f, gains)`**: linearly interpolates f into six gains: gain = Normal + (Weight − Normal) × f. Linear interpolation is used because both endpoints are stable parameter sets validated by the factory, every intermediate value is stable in the twin, and there is only one degree of freedom, f, so the decision logic can stay simple.

**`V4_Init()`**：所有状态清零，**f 和锁存档位都设为 1（最硬档）**。开机从最硬档起步有两个原因：一是最硬档对任何负载都站得住，误判成重最多只是抖一下，误判成轻可能会摔；二是最硬档正是判断负载的标定点，开机后在这里探测，结果才可信。

**`V4_Init()`**: clears all state to zero, and **sets both f and the latched level to 1 (the stiffest level)**. There are two reasons to start from the stiffest level at power-on: first, the stiffest level can hold any load upright, so a wrong "heavy" decision at worst causes some chatter, while a wrong "light" decision can cause a fall; second, the stiffest level is exactly the calibration point for the load decision, so probing there after power-on is the only way to get a trustworthy result.

**`push / buf_min / buf_max / buf_mean`**：环形缓冲的写入和统计，固定大小、不分配内存。

**`push / buf_min / buf_max / buf_mean`**: writing to and computing statistics over the ring buffers; fixed size, no memory allocation.

**`rearm()`（升档）**：f 和锁存档位都设为 1，解除锁存，清空探测窗口，标记「事件后探测」，升档计数加 1。之后的重新探测会自动完成「降档」。

**`rearm()` (upshift)**: sets both f and the latched level to 1, clears the latch, empties the probe windows, marks the next probe as "after an event", and increments the upshift counter. The subsequent re-probe performs the "downshift" automatically.

**`V4_CmdEdge()`（10-03 新增）**：蓝牙按键状态变化时由 `v4_adapt.c` 调用，把宽限期计时清零，开始 1.5 秒按键宽限期。

**`V4_CmdEdge()` (added 10-03)**: called by `v4_adapt.c` when the Bluetooth key state changes; it resets the grace period timer to zero and starts the 1.5-second key-change grace period.

**`revert(原档位)`（快速撤回）**：恢复升档前的档位并重新锁存，不重新探测（负载没变），取消恢复状态，重新开始上膛计时。

**`revert(previous level)` (quick revert)**: restores the gain level from before the upshift and latches it again without re-probing (the load has not changed), cancels the recovery state, and restarts the arming timer.

**`V4_Step(状态, 倾角, 陀螺读数, 是否有指令)`**：每拍调用一次，按顺序执行：

**`V4_Step(state, tilt, gyro reading, command present)`**: called once per tick; it executes the following steps in order:

1. **换算和低通**：陀螺读数除以 16.4 得到度每秒；更新 3 赫兹低通角速度。
2. **平衡角初始化**：第一拍把平衡角设为当前倾角。
3. **预测倾角**：θ_pred = (倾角 − 平衡角) + 0.1 × 角速度。
4. **静止判断**：指令归零满 1 秒为「静止」。
5. **停稳门**：有指令时清零；没有指令时，计数连续平稳的拍数，满 0.6 秒且已静止，才算「停稳」。静止状态 = 指令静止 且 已停稳。（关闭的停车复查也写在这里。）
6. **静止门限**：先算当前门限 = max(3, min(8, 6 × 噪声均方根))；只有在已锁存、已上膛、静止、不在恢复中，**并且预测倾角在当前门限以内**时，才用这一拍更新噪声估计；然后重新计算门限。「只学门限以内的」是防止门限失控的关键（5.6 第 4 条）。
7. **行驶门限**：不在静止状态时，门限至少为 16 度。
8. **快速撤回检查**：升档后的 0.3 秒内记录低通预测倾角的峰值；到 0.3 秒时，如果峰值 < 6 度且还没重新锁存，执行 `revert`。
9. **能否升档**：只有已锁存、并且不在最硬档时才能升档（5.4 第 1 条）。
10. **上膛**：刚锁存的那一拍清除上膛（**10-03：如果这次锁存发生在升档后 3 秒内，同时开始 2 秒降档宽限期，并把静止计时和停稳门清零**）；未上膛时，连续 0.2 秒预测倾角 < 2.5 度才上膛；上膛后保持。不能升档或未上膛时，门限设为无穷大。
11. **恢复状态机**：
    - 不在恢复中：先更新平衡角（行驶中快跟、静止慢跟并设 4 度限制）；如果预测倾角超过门限，进入恢复状态，执行 `rearm()` 升档，并启动快速撤回检查。**（10-03）在宽限期内（按键变化后 1.5 秒、降档宽限 2 秒），拿来和门限比较的是倾角偏差本身，不加 0.1 秒 × 角速度那一项。**
    - 在恢复中：倾角偏差 < 3 度且角速度 < 60 度/秒连续 4 拍，退出恢复。恢复期间不更新平衡角，避免把倒下的过程当成新的平衡位置。
12. **抖振读数**：陀螺读数经过 8 赫兹高通、16 赫兹低通，平方后做约 0.3 秒的平均，开方再除以 16.4 得到度每秒。
13. **静止 / 行驶窗口**：静止时把读数写入 0.3 秒窗口，行驶时写入 0.5 秒窗口；状态一切换就清空另一个窗口。
14. **行驶中判轻**：只有在最硬档、未锁存、连续行驶满 1 秒、0.5 秒窗口满、窗口最小值 ≥ 20 时，锁存为轻档。

   *14b.* **快速判轻（10-03 新增）**：未锁存、在最硬档、解锁满 0.5 秒、最近 0.3 秒（`fwin`）每个读数都 ≥ 23，立刻锁存为轻档，不论静止还是行驶。
15. **静止探测**：未锁存时，静止满 3 秒，或者静止满 1.2 秒且窗口读数已稳定，就判断：事件后的探测要求窗口最小值 ≥ 23；开机探测要求当前读数 ≥ 23。满足判轻（本版为 0），否则判重（0.40），然后锁存。
16. **慢速逃生**：已锁存时，倾角偏离平衡角的低通值超过 7.1 度，就增加临时提升；否则衰减。**（10-03）处于「恢复中」时低通值暂停累计。**f = 锁存档位 + 临时提升，限制在 0～1 之间。
17. 返回 f。

1. **Conversion and low-pass**: divide the gyro reading by 16.4 to get degrees per second; update the 3 hertz low-pass angular rate.
2. **Balance-angle initialization**: on the first tick, set the balance angle to the current tilt.
3. **Predicted tilt**: θ_pred = (tilt − balance angle) + 0.1 × angular rate.
4. **Standing-still check**: the car is "standing still" once the command has been zero for a full 1 second.
5. **Stop-calm gate**: reset to zero whenever there is a command; when there is no command, count consecutive calm ticks, and only after a full 0.6 seconds, and with the car already standing still, does it count as "stopped calmly". Standing-still state = command still AND stopped calmly. (The disabled re-probe-on-stop logic is also written here.)
6. **Standing-still threshold**: first compute the current threshold = max(3, min(8, 6 × noise root mean square)); this tick is used to update the noise estimate only if the controller is latched, armed, standing still, not in recovery, **and the predicted tilt is within the current threshold**; then the threshold is recomputed. "Learn only from samples within the threshold" is the key to keeping the threshold from running away (item 4 of Section 5.6).
7. **Driving threshold**: when not in the standing-still state, the threshold is at least 16 degrees.
8. **Quick revert check**: during the 0.3 seconds after an upshift, record the peak of the low-pass predicted tilt; at 0.3 seconds, if the peak is < 6 degrees and the level has not been latched again yet, execute `revert`.
9. **Upshift allowed?**: an upshift is allowed only when latched and not already on the stiffest level (item 1 of Section 5.4).
10. **Arming**: on the tick where latching has just happened, clear the armed flag (**10-03: if this latch happens within 3 seconds after an upshift, also start the 2-second post-shock (relatch) grace period and reset the still timer and the stop-calm gate to zero**); when not armed, arm only after the predicted tilt has been < 2.5 degrees for 0.2 consecutive seconds; once armed, stay armed. When an upshift is not allowed or the controller is not armed, the threshold is set to infinity.
11. **Recovery state machine**:
    - Not in recovery: first update the balance angle (fast tracking while driving, slow tracking with a 4-degree limit while standing still); if the predicted tilt exceeds the threshold, enter the recovery state, call `rearm()` to upshift, and start the quick revert check. **(10-03) During a grace period (1.5 seconds after a key change, 2 seconds of post-shock grace), the value compared with the threshold is the tilt deviation itself, without the 0.1 seconds × angular rate term.**
    - In recovery: leave recovery after the tilt deviation is < 3 degrees and the angular rate is < 60 degrees/second for 4 consecutive ticks. The balance angle is not updated during recovery, so that the process of falling over is not taken as a new balance position.
12. **Chatter reading**: the gyro reading passes through an 8 hertz high-pass and a 16 hertz low-pass, is squared and averaged over about 0.3 seconds, and the square root is divided by 16.4 to get degrees per second.
13. **Standing-still / driving windows**: when standing still, the reading is written into the 0.3-second window; when driving, into the 0.5-second window; as soon as the state switches, the other window is cleared.
14. **"Light" decision while driving**: latch the light level only when on the stiffest level, not latched, driving continuously for a full 1 second, the 0.5-second window is full, and the window minimum is ≥ 20.

   *14b.* **Fast "light" decision (added 10-03)**: when not latched, on the stiffest level, at least 0.5 seconds since unlatching, and every reading in the last 0.3 seconds (`fwin`) is ≥ 23, latch the light level immediately, whether standing still or driving.
15. **Standing-still probe**: when not latched, after standing still for a full 3 seconds, or for a full 1.2 seconds with the window reading settled, make the decision: a probe after an event requires the window minimum to be ≥ 23; the power-on probe requires the current reading to be ≥ 23. If satisfied, decide "light" (0 in this version), otherwise decide "heavy" (0.40), and then latch.
16. **Slow escape**: when latched, if the low-pass value of the tilt's deviation from the balance angle exceeds 7.1 degrees, increase the temporary boost; otherwise let it decay. **(10-03) While "in recovery", the low-pass value stops accumulating.** f = latched level + temporary boost, clamped to the range 0 to 1.
17. Return f.

**为什么升档判断（第 1～11 步）放在抖振和探测（第 12～16 步）之前**：升档要在**这一拍**就生效。如果放在后面，同一拍的探测会用旧状态，升档效果晚一拍（5 毫秒）。

**Why the upshift decision (steps 1 to 11) comes before chatter and probing (steps 12 to 16)**: the upshift must take effect **on this tick**. If it came afterwards, the probe in the same tick would use the old state, and the upshift would take effect one tick (5 milliseconds) late.

### 6.4 `v4_adapt.c`：接进原厂固件 / 6.4 `v4_adapt.c`: hooking into the factory firmware

**`cmd_moving()`：判断当前是否有遥控指令。** 完全照抄原厂 `Velocity_PI()` 和 `Turn_PD()` 生成指令的规则：前进/后退类状态（包括手柄斜向）看 `Car_Target_Velocity`，避障/跟随模式用它们的固定速度，其他模式看 `Move_X`；左右转类状态看 `Car_Turn_Amplitude_speed`，定速原地转直接算有指令，最后再看 `Move_Z`。**转向也算指令**是第三轮加的（5.6 第 2 条）。只要固件真的在执行某种运动指令，v4 就不会把车当成静止。

**`cmd_moving()`: decides whether there is currently a remote-control command.** It copies exactly the rules by which the factory `Velocity_PI()` and `Turn_PD()` generate commands: forward/backward states (including diagonal joystick directions) look at `Car_Target_Velocity`, the obstacle-avoidance/following modes use their fixed speeds, and other modes look at `Move_X`; left/right turn states look at `Car_Turn_Amplitude_speed`, constant-speed spinning in place counts directly as a command, and finally `Move_Z` is checked. **Counting turning as a command too** was added in the third round (item 2 of Section 5.6). As long as the firmware is actually executing some motion command, v4 will not treat the car as standing still.

**`apply(f)`：写增益。** 先算出六个增益；如果速度积分增益要变，先把速度环积分 `Encoder_Integral` 乘以「旧积分增益 / 新积分增益」，再写入新增益。原因：速度环输出含有「−积分 × 积分增益 / 100」这一项，积分增益突然变大而积分值不变，电机输出会跳一下，最多约 1280 PWM，而换档往往正好发生在车已经有麻烦的时候。缩放之后，换档那一瞬间输出连续不跳变（这叫无扰切换，和原厂负载模式 26 的做法一致）。

**`apply(f)`: writes the gains.** It first computes the six gains; if the velocity integral gain is going to change, it first multiplies the velocity-loop integral `Encoder_Integral` by "old integral gain / new integral gain", then writes the new gains. Reason: the velocity-loop output contains the term "−integral × integral gain / 100"; if the integral gain jumps up while the integral value stays the same, the motor output jumps, by up to about 1280 PWM, and gain changes often happen exactly when the car is already in trouble. With the scaling, the output stays continuous without a jump at the moment of the gain change (this is called bumpless transfer, and it matches what the factory load mode 26 does).

**`V4_Reset()`**：选中模式 27 时调用。初始化状态，然后以 f=1（最硬档）写入增益。写之前先把积分增益设为 0，让第一次 `apply` 不去缩放积分（这时积分还没开始累加）。

**`V4_Reset()`**: called when mode 27 is selected. It initializes the state, then writes the gains with f=1 (the stiffest level). Before writing, it sets the integral gain to 0 so that the first `apply` does not scale the integral (at that point the integral has not started accumulating).

**`V4_Tick()`**：每 5 毫秒调用一次。读 `Angle_Balance`、`Gyro_Balance`、`cmd_moving()`，调用 `V4_Step()`，更新屏幕显示用的变量，写增益。10-03 增加了两件事：
- **电机关闭时保持开机状态**：原厂 `Turn_Off()` 判定电机关闭（还没开始、被拿起、倾角超过 40 度、电池低于 9.6 伏）时，每拍都把 v4 重新初始化，不计时、不探测（12.3 问题 1）。
- **按键变化检测**：记住上一拍的蓝牙按键状态 `g_newcarstate`，一变化就调用 `V4_CmdEdge()`（12.3 问题 5）。
- 另外导出 `v4_th_ref`（平衡角基准）和 `v4_rec`（是否恢复中），只给串口日志用。

**`V4_Tick()`**: called once every 5 milliseconds. It reads `Angle_Balance`, `Gyro_Balance`, and `cmd_moving()`, calls `V4_Step()`, updates the variables used for the screen display, and writes the gains. Two things were added on 10-03:
- **Stay in the power-on state while the motors are off**: whenever the factory `Turn_Off()` decides the motors are off (not started yet, picked up, tilt above 40 degrees, battery below 9.6 volts), v4 is re-initialized on every tick, with no timing and no probing (Section 12.3, issue 1).
- **Key-change detection**: it remembers the Bluetooth key state `g_newcarstate` from the previous tick and calls `V4_CmdEdge()` as soon as it changes (Section 12.3, issue 5).
- In addition, it exports `v4_th_ref` (the balance-angle reference) and `v4_rec` (whether in recovery), used only for the serial log.

### 6.5 原厂固件里改动的 8 处（`source_snapshot/firmware_hooks/`） / 6.5 The 8 changes made to the factory firmware (`source_snapshot/firmware_hooks/` (source snapshot / firmware hook points))

| 文件 | 改动 | 目的 |
|---|---|---|
| `USER/myenum.h` | 模式枚举末尾加 `Adapt_V4`（第 27 个模式） | 让模式选择能选到它。 |
| `APP/mode/app_mode.c` | 模式 27 加入模式列表；`Set_control_speed()` 给它和正常模式相同的遥控速度（30 / 36）；进入模式时调用 `V4_Reset()` | 遥控手感和原厂一致；每次进入模式都从最硬档重新开始。 |
| `BSP/bsp.c` | 模式 27 初始化蓝牙和超声波 | 和正常模式一样能用 App 遥控。 |
| `APP/OLED_Show/oled_show.c` | 模式名称显示「27.Adapt V4」 | 选模式时能看到。 |
| `USER/main.c` | 屏幕显示 `f / L或P / osc / u` | 实车调试时能直接看到 v4 状态。 |
| `APP/app_control.c` | 在 `Balance_PD()` 之前调用 `V4_Tick()` | 本拍算出的增益本拍就用上。 |
| `USER/AllHeader.h` | 包含 `v4_adapt.h` | 让上面几个文件能调用 v4 的函数。 |
| `APP/PID/tune_io.c` | 模式 27 时串口日志记录抖振读数、档位（f×100，已锁存再加 1000，**10-03 起恢复中再加 2000**），以及**平衡角基准 ×100（10-03 新增，占用模式 27 下不用的 `k_c` 列）** | 实车录波、标定和诊断用（12.2 节）。 |

| File | Change | Purpose |
|---|---|---|
| `USER/myenum.h` | Add `Adapt_V4` at the end of the mode enumeration (the 27th mode) | So that mode selection can reach it. |
| `APP/mode/app_mode.c` | Add mode 27 to the mode list; `Set_control_speed()` gives it the same remote-control speeds as the Normal mode (30 / 36); call `V4_Reset()` when entering the mode | Remote-control feel matches the factory firmware; every time the mode is entered it starts again from the stiffest level. |
| `BSP/bsp.c` | Initialize Bluetooth and the ultrasonic sensor for mode 27 | The car can be remote-controlled from the App just as in the Normal mode. |
| `APP/OLED_Show/oled_show.c` | Display the mode name "27.Adapt V4" | Visible when selecting the mode. |
| `USER/main.c` | Screen shows `f / L或P / osc / u` (f / L or P / osc / u) | The v4 state can be seen directly while debugging on the real car. |
| `APP/app_control.c` | Call `V4_Tick()` before `Balance_PD()` | The gains computed in this tick are used in this same tick. |
| `USER/AllHeader.h` | Include `v4_adapt.h` | So that the files above can call the v4 functions. |
| `APP/PID/tune_io.c` | In mode 27, the serial log records the chatter reading, the gain level (f×100, plus 1000 when latched, **plus 2000 when in recovery since 10-03**), and **the balance-angle reference ×100 (added 10-03, using the `k_c` column, which is unused in mode 27)** | For recording, calibration and diagnosis on the real car (Section 12.2). |

另外，Keil 工程文件 `USER/stm32_Balance_Car.uvprojx` 里加入了 `v4_core.c` 和 `v4_adapt.c` 两个源文件。

In addition, the two source files `v4_core.c` and `v4_adapt.c` were added to the Keil project file `USER/stm32_Balance_Car.uvprojx`.

这些原厂文件用的是简体中文 GBK 编码，并且换行符混用（Windows 和 Unix 两种）。修改时按字节打补丁，没有整文件重新保存，确保原有的中文注释和格式一个字节都不变。

These factory files use the Simplified Chinese GBK encoding and mix line endings (both Windows and Unix). The changes were applied as byte-level patches rather than by re-saving whole files, to make sure the existing Chinese comments and formatting stay unchanged down to the byte.

### 6.6 可解释性对照表：每一段代码为什么存在、去掉会怎样 / 6.6 Explainability table: why each piece of code exists and what happens without it

这张表的目的是让源码里**每一个机制都能追溯到一个具体问题和一条实测证据**。「去掉会怎样」一列都是实际做过的实验结果，不是推测。行号指 `source_snapshot/v4_core.c`。

The purpose of this table is to make **every mechanism in the source code traceable to a specific problem and a piece of measured evidence**. The "what happens without it" column consists entirely of results from experiments that were actually run, not speculation. Line numbers refer to `source_snapshot/v4_core.c` (source snapshot).

| 机制 | 代码行 | 要解决的问题 | 去掉它会怎样（实测证据） |
|---|---|---|---|
| 开机从最硬档起步（`V4_Init` 里 f=1） | 91 | 开机时不知道车上有没有货；而且只有在最硬档下，抖振才和负载对得上。 | 从软档起步：带载车开机就可能倒（原厂正常档带 1 千克以上全部摔）。在软档下判断负载，等于 v2 的错误（标尺在动）。 |
| 线性插值两套原厂增益（`V4_GainsFor`） | 76–84 | 需要介于两档之间的档位，并且每个档位都要安全。 | 只有原厂两档：1 千克没有合适的档（正常档倒、负重档停下一直抖）。插值的端点是原厂验证过的参数，中间值在孪生里全部稳定。 |
| 抖振读数（8～16 赫兹带通能量） | 244–248 | 需要一个能区分负载的信号。 | 没有被动信号能代替：在轻档和中档下，站立抖振、平衡角、速度环积分对 0～4 千克全部重叠（`load_signal.py`）。 |
| 探测：至少静止 1.2 秒、读数稳定才判断，最多 3 秒 | 273–282 | 上电后读数要约 1 秒才升上来。 | 原来 0.2 秒就判断：上电站着时 1/2/4 千克读数 5.8/4.7/4.4，全被判成半载（BUGS 第十二节）。没有 3 秒上限，读数一直波动时探测永远不会结束。 |
| 锁存（判断一次就记住） | 283–296 | 抖振和负载的对应只在最硬档成立；f 一变，抖振也跟着变。 | v2 每拍重新判断：空车被推回 f=0.43，抖振 9.0；锁存后为 0.5（5.2 节）。 |
| 分界 23「有把握才判轻」 | 290 | 换一台车，带载读数可能升高到接近空载。 | 分界 3.0 时，随机化 16 台车里有 4 台带载车被判成轻，其中一台下台阶摔倒；改成 23 后 16 台全部判对。 |
| 行驶中只判轻，并且要 0.5 秒内每个读数 ≥ 20 | 265–271 | 空车在最硬档上开起来很抖，应尽早降档；带载车留在最硬档没有坏处。 | 行驶中也判重：行驶时空车读数可能降到 14，会被误判为重，一直偏硬。 |
| 事件后探测用更严的判轻标准 | 286–288 | 推击后的残余晃动会被误读为「轻」。 | 2 千克推 15 牛后，读数 8.32 是残余，在旧标准下被判成轻（f=0.12），车上实际有 2 千克。 |
| 慢速逃生（叠加并衰减） | 298–311 | 锁存后如果负载判错，或车长期吃力，需要兜底。 | 改成永久覆盖：空车被推一下 3 牛就被钉在负重档，抖振 33.5；叠加并衰减后为 0.5。 |
| 预测倾角（倾角偏差 + 0.1 × 角速度） | 157 | 车正在快速倒下时，只看倾角会晚。 | 只看倾角：要等真的歪到门限才反应，晚约 0.1 秒；推 6 牛后 20 毫秒倾角才 5 度。 |
| 大倾角升档（`rearm`） | 121–129、228–233 | 被推、加载、下台阶的共同症状是倾角过大，说明当前档位不够。 | 没有它（即 v3）：空载 6 牛 0/4、2 千克 15 牛 0/4；有它为 2/4、2/4，与原厂负重档持平。2 千克回稳包络多救回两格。 |
| 只在「已锁存且不在最硬档」时允许升档（`can_up`） | 211 | 最硬档下空车自激，角速度很大，预测倾角会一直超过门限。 | 没有这一条：32 组候选全部卡死在最硬档；正常行驶 8 局里误升档 1231～12228 次，空车抖振 33.7（等于原厂负重档）。 |
| 上膛（连续 0.2 秒平稳才生效，生效后保持） | 212–221 | 刚锁存、刚降档时的余振会马上触发升档。 | 不上膛：空车没人推也会「锁存→升档→锁存→升档」循环，12 秒里 9 次。上膛不保持：被推时会在到达门限前一拍自己解除，永远不触发。 |
| 静止门限 3 度，行驶门限 16 度 | 154、193–198 | 静止晃动最大约 2 度；行驶中约 8 度，刚停约 9 度。 | 用同一个门限：够早（能救 6 牛）就会在行驶中不停误触发，车一直停在最硬档。 |
| 停稳门（停车后先平稳 0.6 秒） | 160–177 | 急刹、坡上停车后车要 1.1～1.5 秒才晃停，1 秒时就切换小门限太早。 | 没有它：0.6 米每秒急刹，空载 8 次误升档 6 次；坡上停车驻停抖振 2.9。有它：急刹 0/144，坡上 1.1。 |
| 平衡角双速跟踪 | 223–227 | 坡上、带偏心负载时，平衡角不是 0 度，需要跟踪；但推击不能被当成平衡角变化吸收掉。 | 只用慢速并设 4 度限制：坡上平衡角永远跟不上，误升档，空载坡道抖振 5.6。不设限制：推击会被慢慢吸收，门限失效。 |
| 噪声自适应门限 | 179–191 | 有的车本身晃得多，固定 3 度门限会误触发；真车晃动比孪生大 3～5 倍。 | 固定 3 度：随机化车上空载抖振 3.0～4.4（误升档）；自适应后为 1.55，与不升档的 v3 相同。 |
| 噪声只学门限以内的样本，门限封顶 8 度 | 183–190 | 车真的歪了也会被当成「噪声」学进去。 | 没有它：门限从 54 度涨到 118 度，保护失效，随机测试摔倒 22 局；有它为 19 局。 |
| 快速撤回（0.3 秒内峰值小于 6 度就退回原档） | 200–209、133–142 | 小扰动触发升档后，要等 1.2 秒以上的重新探测，空车在最硬档上抖。 | 没有它：静止被推倾斜 5 度，回稳要 1.31～1.50 秒；有它为 0.60～0.67 秒。 |
| 退出恢复状态的条件（3 度、60 度/秒、4 拍） | 234–241 | 恢复期间不能更新平衡角，否则会把倒下的过程学成新的平衡位置；但恢复状态也必须能结束。 | 恢复中更新平衡角：平衡角会跟着倒下的方向漂，门限失效。 |
| 转向也算指令（`v4_adapt.c` 的 `cmd_moving`） | 接入层 46–53 | 原地转弯时车身会晃，不能当成静止。 | 不算：转弯晃动被学进噪声估计，门限被抬到 16.8 度。 |
| 换档时缩放速度环积分（`v4_adapt.c` 的 `apply`） | 接入层 60–61 | 速度积分增益突变而积分值不变，电机输出会跳一下。 | 不缩放：换档瞬间电机指令最多跳约 1280 PWM，而换档往往正好发生在车有麻烦的时候。 |
| 增益写在 `Balance_PD` 之前（`app_control.c`） | 接入点 | 本拍算出的档位要本拍生效。 | 写在之后：所有档位变化晚 5 毫秒生效（原厂模式 26 就是这样）。 |

| Mechanism | Code lines | Problem it solves | What happens without it (measured evidence) |
|---|---|---|---|
| Start from the stiffest level at power-on (f=1 in `V4_Init`) | 91 | At power-on it is unknown whether the car carries cargo; and the chatter matches the load only on the stiffest level. | Starting from a soft level: a loaded car may fall right after power-on (on the factory Normal gains, every case with 1 kilogram or more falls). Deciding the load on a soft level repeats the v2 mistake (the measuring stick is moving). |
| Linear interpolation between the two factory gain sets (`V4_GainsFor`) | 76–84 | Gain levels between the two factory levels are needed, and every level must be safe. | With only the two factory levels: there is no suitable level for 1 kilogram (on the Normal gains it falls; on the Weight gains it chatters constantly when stopped). The interpolation endpoints are factory-validated parameters, and every intermediate value is stable in the twin. |
| Chatter reading (8 to 16 hertz band-pass energy) | 244–248 | A signal that can distinguish loads is needed. | No passive signal can replace it: on the light and middle levels, standing chatter, balance angle and velocity-loop integral all overlap across 0 to 4 kilograms (`load_signal.py`). |
| Probing: decide only after standing still for at least 1.2 seconds with a settled reading, at most 3 seconds | 273–282 | After power-on the reading takes about 1 second to rise. | Originally the decision was made after 0.2 seconds: standing after power-on, the readings for 1/2/4 kilograms were 5.8/4.7/4.4, and all were judged as half load (BUGS, Section 12). Without the 3-second cap, probing would never end if the reading kept fluctuating. |
| Latch (decide once and remember) | 283–296 | The correspondence between chatter and load holds only on the stiffest level; as soon as f changes, the chatter changes too. | v2 re-decided on every tick: an empty car was pushed back to f=0.43, with chatter 9.0; with the latch it is 0.5 (Section 5.2). |
| Split of 23, "decide light only when confident" | 290 | On a different car, the loaded reading may rise close to the empty reading. | With a split of 3.0, 4 of 16 randomized cars carrying a load were judged light, and one of them fell when going down a step; after changing to 23, all 16 were judged correctly. |
| While driving, decide only "light", and only if every reading within 0.5 seconds is ≥ 20 | 265–271 | An empty car chatters badly when driving on the stiffest level and should downshift as soon as possible; a loaded car loses nothing by staying on the stiffest level. | Deciding "heavy" while driving too: an empty car's reading while driving can drop to 14, so it would be misjudged as heavy and stay too stiff. |
| A stricter "light" criterion for probes after an event | 286–288 | Residual wobble after a push can be misread as "light". | After a 15 newton push with 2 kilograms, the reading of 8.32 was residual motion and was judged light under the old criterion (f=0.12), while the car was actually carrying 2 kilograms. |
| Slow escape (added on top, then decays) | 298–311 | After latching, a fallback is needed if the load was misjudged or the car struggles for a long time. | Changed to a permanent override: an empty car pushed once with 3 newtons gets pinned on the Weight gains, chatter 33.5; with add-and-decay it is 0.5. |
| Predicted tilt (tilt deviation + 0.1 × angular rate) | 157 | When the car is falling quickly, looking only at the tilt reacts too late. | Looking only at the tilt: it reacts only once the car has actually leaned to the threshold, about 0.1 seconds late; 20 milliseconds after a 6 newton push the tilt is only 5 degrees. |
| Large-tilt upshift (`rearm`) | 121–129, 228–233 | Being pushed, being loaded and going down a step share one symptom: an excessive tilt, which shows the current gain level is not enough. | Without it (that is, v3): empty with 6 newtons 0/4, 2 kilograms with 15 newtons 0/4; with it 2/4 and 2/4, on par with the factory Weight gains. On the 2-kilogram recovery envelope it rescues two more cells. |
| Upshift allowed only when "latched and not on the stiffest level" (`can_up`) | 211 | On the stiffest level an empty car self-oscillates, the angular rate is large, and the predicted tilt would exceed the threshold all the time. | Without this rule: all 32 candidates got stuck on the stiffest level; in 8 normal-driving runs there were 1231 to 12228 false upshifts, and the empty car's chatter was 33.7 (equal to the factory Weight gains). |
| Arming (takes effect only after 0.2 consecutive calm seconds, and stays on once armed) | 212–221 | Residual oscillation right after latching or downshifting would immediately trigger an upshift. | Without arming: even with nobody pushing, an empty car cycles "latch → upshift → latch → upshift", 9 times in 12 seconds. If arming is not held: when pushed, it disarms itself one tick before reaching the threshold and never triggers. |
| Standing-still threshold 3 degrees, driving threshold 16 degrees | 154, 193–198 | Standing-still wobble is at most about 2 degrees; while driving about 8 degrees, and just after stopping about 9 degrees. | Using a single threshold: one early enough (to rescue a 6 newton push) keeps triggering falsely while driving, and the car stays on the stiffest level all the time. |
| Stop-calm gate (after stopping, first stay calm for 0.6 seconds) | 160–177 | After hard braking or stopping on a slope, the car takes 1.1 to 1.5 seconds to stop wobbling, so switching to the small threshold at 1 second is too early. | Without it: hard braking from 0.6 meters per second, empty, 6 false upshifts in 8 runs; parked chatter after stopping on a slope 2.9. With it: hard braking 0/144, slope 1.1. |
| Two-speed tracking of the balance angle | 223–227 | On a slope or with an off-center load, the balance angle is not 0 degrees and must be tracked; but a push must not be absorbed as a change of balance angle. | Only slow tracking with a 4-degree limit: on a slope the balance angle never catches up, causing false upshifts, empty chatter on the slope 5.6. Without the limit: a push is slowly absorbed and the threshold stops working. |
| Noise-adaptive threshold | 179–191 | Some cars wobble more by nature, and a fixed 3-degree threshold triggers falsely; the real car wobbles 3 to 5 times more than the twin. | Fixed 3 degrees: on randomized cars, empty chatter 3.0 to 4.4 (false upshifts); with adaptation it is 1.55, the same as v3, which never upshifts. |
| Noise is learned only from samples within the threshold, and the threshold is capped at 8 degrees | 183–190 | Otherwise a car that is really leaning would also be learned as "noise". | Without it: the threshold grew from 54 degrees to 118 degrees, the protection failed, and 22 runs fell in the randomized tests; with it, 19 runs. |
| Quick revert (return to the previous level if the peak stays below 6 degrees within 0.3 seconds) | 200–209, 133–142 | After a small disturbance triggers an upshift, the re-probe takes 1.2 seconds or more, and the empty car chatters on the stiffest level meanwhile. | Without it: when pushed while standing still to a 5-degree tilt, settling takes 1.31 to 1.50 seconds; with it, 0.60 to 0.67 seconds. |
| Conditions for leaving the recovery state (3 degrees, 60 degrees/second, 4 ticks) | 234–241 | The balance angle must not be updated during recovery, otherwise the fall would be learned as a new balance position; but the recovery state must also be able to end. | Updating the balance angle during recovery: the balance angle drifts in the direction of the fall, and the threshold stops working. |
| Turning also counts as a command (`cmd_moving` in `v4_adapt.c`) | Glue layer 46–53 | The body wobbles when turning in place, so this must not be treated as standing still. | Not counting it: turning wobble is learned into the noise estimate, and the threshold is raised to 16.8 degrees. |
| Scale the velocity-loop integral when changing gains (`apply` in `v4_adapt.c`) | Glue layer 60–61 | If the velocity integral gain changes abruptly while the integral value stays the same, the motor output jumps. | Without scaling: at the moment of the gain change the motor command jumps by up to about 1280 PWM, and gain changes often happen exactly when the car is in trouble. |
| Gains written before `Balance_PD` (`app_control.c`) | Hook point | The gain level computed in this tick must take effect in this tick. | Written afterwards: every gain change takes effect 5 milliseconds late (this is how the factory mode 26 does it). |

**没有采用、但代码保留的机制**（默认关闭，可以用 `-D` 打开做实验）：

**Mechanisms that were not adopted but whose code is kept** (off by default; they can be turned on with `-D` for experiments):

| 机制 | 为什么不采用 |
|---|---|
| 停车复查（`REPROBE_ON_STOP`） | 随机测试中开启后摔倒 19 → 22 局、持续抖振 8 → 41 次，加载后判对的次数没有增加。 |
| 重载类行驶门限（`REC_ON_MOVE_HEAVY`） | 设为 6 度能救回带载行驶中被推，但正常行驶误升档 18 次，急刹也会误触发。 |
| 增益大于负重档的紧急档（f > 1，只在 Python 版里） | 需要它时电机已经输出到上限，救回次数和不开相同或更差。 |

| Mechanism | Why it was not adopted |
|---|---|
| Re-probe on stop (`REPROBE_ON_STOP`) | When enabled in the randomized tests, falls went from 19 → 22 runs and sustained sustained-chatter events from 8 → 41, while the number of correct decisions after loading did not increase. |
| Driving threshold for the heavy class (`REC_ON_MOVE_HEAVY`) | Set to 6 degrees, it can rescue a loaded car pushed while driving, but it causes 18 false upshifts in normal driving and also triggers falsely during hard braking. |
| Emergency level with gains above the Weight gains (f > 1, only in the Python version) | When it is needed, the motors are already at their output limit, so the number of rescues is the same as without it, or worse. |

---

## 7. 从 Python 到 C：移植、逐拍对拍、编译出 hex 的方法 / 7. From Python to C: porting, tick-by-tick comparison, and building the hex

### 7.1 为什么先写 Python 再写 C / 7.1 Why Python first and C second

在孪生里做实验必须用 Python（孪生是 Python 写的），而且调参要跑成千上万局，Python 改起来快。等逻辑定型后再逐行移植成 C。风险在于两份代码可能不一致，所以必须做对拍。

Experiments in the digital twin have to be done in Python (the twin is written in Python), and tuning requires running thousands of episodes, so Python is faster to change. Once the logic was final, it was ported to C line by line. The risk is that the two versions of the code might not match, so a tick-by-tick comparison is mandatory.

### 7.2 逐拍对拍（`v4_port_check.py`） / 7.2 Tick-by-tick comparison (`v4_port_check.py`)

1. 在孪生里跑 Python 版 v4，**每一拍**都记下它看到的三个输入（倾角、陀螺读数、是否有指令）和它的输出（f、是否锁存、升档次数、抖振读数、撤回次数）；
2. 用电脑上的 gcc 把 `v4_core.c` 和一个小驱动程序 `v4_host.c` 编译成 `v4_host.exe`；
3. 把同一串输入喂给 `v4_host.exe`，逐拍比较输出。

1. Run the Python version of v4 in the twin and, **on every tick**, record the three inputs it sees (tilt angle, gyro reading, whether a command is active) and its outputs (f, whether it is latched, upshift count, chatter reading, revert count);
2. Use gcc on the PC to compile `v4_core.c` together with a small driver program `v4_host.c` into `v4_host.exe`;
3. Feed the same input sequence to `v4_host.exe` and compare the outputs tick by tick.

输入完全相同，所以任何差别都只可能来自移植错误。测试了 12 个场景：上电站立（空载、2 千克）、站立被推（5、8、15 牛）、开局就走（空载、2 千克）、站立中加载再走、站立被倾斜 5 度（空载、2 千克）、0.6 米每秒急刹（空载、2 千克）。

The inputs are identical, so any difference can only come from a porting error. 12 scenarios were tested: standing after power-on (empty, 2 kilograms), pushed while standing (5, 8, 15 newtons), driving from the start (empty, 2 kilograms), load added while standing and then driving, tilted 5 degrees while standing (empty, 2 kilograms), and hard braking from 0.6 meters per second (empty, 2 kilograms).

**结果：全部场景的 f 最大差为 0，锁存状态、升档次数、撤回次数完全一致，抖振读数最大差约 0.00001 度每秒**（浮点运算的末位差异）。另外用 `-D` 选项编译出「快速撤回打开」「停车复查打开」等变体，同样逐拍一致，证明默认关闭的代码路径也移植正确。

**Result: across all scenarios the maximum difference in f is 0, the latch state, upshift count and revert count match exactly, and the maximum difference in the chatter reading is about 0.00001 degrees per second** (last-digit differences in floating-point arithmetic). In addition, variants such as "quick revert enabled" and "re-check after stopping enabled" were compiled with the `-D` option and also matched tick by tick, which shows that the code paths that are disabled by default were ported correctly too.

**10-03 注意**：第 12 节的修改只做在 C 代码里，Python 版 `switch_v4.py` 没有同步，所以上面的对拍结果只适用于 10-02 版。本版以 C 代码为准；10-03 的修改是用「离线重放」（同一份 `v4_core.c` 在电脑上编译，喂实车记录）和实车测试验证的，见 12.2 节。

**10-03 note**: The changes in Section 12 were made only in the C code; the Python version `switch_v4.py` was not updated to match, so the comparison results above apply only to the 10-02 version. For this version the C code is authoritative; the 10-03 changes were verified with offline replay (the same `v4_core.c` compiled on the PC and fed real-car logs) and with tests on the real car, see Section 12.2.

### 7.3 编译出 hex / 7.3 Building the hex

用 Keil MDK（`C:\Keil_v5\UV4\UV4.exe`）命令行编译整个工程：

Build the whole project from the command line with Keil MDK (`C:\Keil_v5\UV4\UV4.exe`):

```
UV4.exe -b USER\stm32_Balance_Car.uvprojx -j0 -o build.log
```

结果（本版）：0 个错误，0 个警告；程序大小：代码 55284 字节，只读数据 6632 字节，读写数据 1140 字节，零初始化数据约 20.6 千字节。程序结束地址 0x0800F2C4。

Result (this version): 0 errors, 0 warnings; program size: code 55284 bytes, read-only data 6632 bytes, read-write data 1140 bytes, zero-initialized data about 20.6 kilobytes. The program end address is 0x0800F2C4.

**闪存注意**：这块板子的芯片标的是 256 千字节闪存，但实测只有前 64 千字节能可靠使用（结束地址必须低于 0x08010000，否则所有模式都会出现一个轮子不转）。本版结束地址离这个界限还有 3388 字节。以后加功能重新编译时，务必检查编译输出的程序大小和 hex 的结束地址。

**Flash note**: The chip on this board is rated for 256 kilobytes of flash, but measurements show that only the first 64 kilobytes can be used reliably (the end address must be below 0x08010000; otherwise one wheel does not turn in every mode). In this version the end address is 3388 bytes below that limit. When adding features and rebuilding in the future, always check the program size in the build output and the end address of the hex.

生成 `OBJ\stm32_Balance_Car_L.hex`，复制到本文件夹即为 `stm32_Balance_Car_L_v4.hex`。

The build produces `OBJ\stm32_Balance_Car_L.hex`; copied into this folder, it is `stm32_Balance_Car_L_v4.hex`.

---

## 8. 验证结果（全部带原厂基线） / 8. Validation results (all with factory baselines)

以下全部是 10-02 版的孪生仿真结果，原始数据见第 10 节的目录。**10-03 版的实车结果见第 12.4 节**；本版的修改只影响「空载档已锁定」和「刚升档」时的行为，孪生里的这些表没有重新跑。

Everything below is digital-twin simulation results for the 10-02 version; the raw data is in the directories listed in Section 10. **Real-car results for the 10-03 version are in Section 12.4**; the changes in this version only affect behavior when "the empty-load gain is already latched" and "just after an upshift", and these tables were not re-run in the twin.

### 8.1 主对比表：停-走-停任务，12 种工况 / 8.1 Main comparison table: stop-go-stop task, 12 conditions

12 种工况：空载 / 1 / 2 / 4 千克平地、空载和 2 千克下 10 毫米台阶、空载被推 3 牛和 8 牛、2 千克被推 8 牛、空载上坡 8 度、2 千克上坡和下坡 8 度。

The 12 conditions: empty / 1 / 2 / 4 kilograms on flat ground, empty and 2 kilograms going down a 10 millimeter step, empty pushed with 3 newtons and 8 newtons, 2 kilograms pushed with 8 newtons, empty going up an 8 degree slope, and 2 kilograms going up and down an 8 degree slope.

| 完成局数 | 站立开局（48 局） | 开局就走（48 局） | 随机化 8 台车（96 局） | 齿轮刚度改为 22 赫兹（48 局） |
|---|---|---|---|---|
| 原厂正常档 | 16 | 18 | 34 | 16 |
| 原厂负重档 | 41 | 40 | 83 | 41 |
| **v4** | **44** | **44** | **88** | **44** |

| Episodes completed | Start standing (48 episodes) | Drive from the start (48 episodes) | 8 randomized cars (96 episodes) | Gear stiffness changed to 22 hertz (48 episodes) |
|---|---|---|---|---|
| Factory Normal gains | 16 | 18 | 34 | 16 |
| Factory Weight gains | 41 | 40 | 83 | 41 |
| **v4** | **44** | **44** | **88** | **44** |

没完成的几局集中在「空载被推 8 牛」，所有控制器都摔，属于物理极限。

The episodes that were not completed are concentrated in "empty, pushed with 8 newtons", where every controller falls; this is a physical limit.

### 8.2 行驶表：开得稳、停得稳 / 8.2 Driving table: drive steadily, stop steadily

前进 1 米、停 4 秒、倒回原点、停 4 秒；每格 4 台车。「原厂对档」是事先按负载选好档（≥1 千克用负重档）。

Drive forward 1 meter, stop for 4 seconds, reverse back to the origin, stop for 4 seconds; 4 cars per cell. "Factory, gear matched to the load" means the gains were chosen in advance according to the load (Weight gains for ≥1 kilogram).

| 负载 | 原厂对档：行驶抖振 / 停车收敛 | v4：行驶抖振 / 停车收敛 |
|---|---|---|
| 空载 | 3.0 / 1.19 秒 | 3.1 / 1.20 秒（打平） |
| 1 千克 | 11.2 / **始终不稳** | **3.2 / 0.69 秒** |
| 2 千克 | 6.6 / 0.77 秒 | **3.2 / 0.60 秒** |
| 4 千克 | 5.6 / 0.75 秒 | **3.7 / 0.66 秒** |
| 2 千克坡道 8 度 | 6.0 / 0.76 秒，驻停抖振 2.9 | **4.5 / 0.63 秒，驻停抖振 1.1** |

| Load | Factory, gear matched to the load: driving chatter / stop settling | v4: driving chatter / stop settling |
|---|---|---|
| Empty | 3.0 / 1.19 seconds | 3.1 / 1.20 seconds (tie) |
| 1 kilogram | 11.2 / **never steady** | **3.2 / 0.69 seconds** |
| 2 kilograms | 6.6 / 0.77 seconds | **3.2 / 0.60 seconds** |
| 4 kilograms | 5.6 / 0.75 seconds | **3.7 / 0.66 seconds** |
| 2 kilograms on an 8 degree slope | 6.0 / 0.76 seconds, chatter while parked 2.9 | **4.5 / 0.63 seconds, chatter while parked 1.1** |

### 8.3 推击（站稳后推 0.12 秒；挺过来的次数 / 4） / 8.3 Push (0.12 second push after standing steady; number of survivals / 4)

| | 空载 5 牛 | 空载 6 牛 | 2 千克 12 牛 | 2 千克 15 牛 |
|---|---|---|---|---|
| 原厂正常档 | 2 | 0 | 站不住 | 站不住 |
| 原厂负重档 | 4 | 2 | 4 | 4 |
| v4 | 4 | 2 | 4 | 2 |

| | Empty, 5 newtons | Empty, 6 newtons | 2 kilograms, 12 newtons | 2 kilograms, 15 newtons |
|---|---|---|---|---|
| Factory Normal gains | 2 | 0 | Cannot stand | Cannot stand |
| Factory Weight gains | 4 | 2 | 4 | 4 |
| v4 | 4 | 2 | 4 | 2 |

空载 6 牛、2 千克 15 牛处在「摔与不摔」的边缘，4 次一组的结果有明显的抽样波动。

Empty with 6 newtons and 2 kilograms with 15 newtons sit on the edge between falling and not falling, so results from groups of 4 trials show clear sampling noise.

### 8.4 急刹（匀速后速度指令瞬间归零，每格 8 次） / 8.4 Hard braking (speed command dropped instantly to zero after constant speed, 8 trials per cell)

v4 在空载、1 千克、2 千克，0.3～0.6 米每秒急刹时，误升档 0 次，收敛时间和原厂对档相当或更快（2 千克 0.6 米每秒：v4 1.00 秒，原厂负重档 1.17 秒）。4 千克 0.6 米每秒有 2 次升档，是急起步时倾斜超过 16 度触发的，属于正当保护。

For hard braking from 0.3 to 0.6 meters per second with empty, 1 kilogram and 2 kilograms, v4 made 0 false upshifts, and its settling time was equal to or faster than the factory with the gear matched to the load (2 kilograms at 0.6 meters per second: v4 1.00 seconds, factory Weight gains 1.17 seconds). At 4 kilograms and 0.6 meters per second there were 2 upshifts; they were triggered by tilt exceeding 16 degrees during the hard start and are legitimate protection.

### 8.5 随机场地测试（64 局，每局 24 条随机指令） / 8.5 Random field test (64 episodes, 24 random commands per episode)

按实车四个键随机操作，中间随机插入被推、改负载、改坡度；开阔场地；一半标称车、一半随机化车。

Random operation of the real car's four keys, with pushes, load changes and slope changes inserted at random; open field; half nominal cars, half randomized cars.

| | 摔倒局 | 不收敛 | 持续抖振 | 停车收敛（中位） |
|---|---|---|---|---|
| 原厂正常档 | 64/64（带载就摔） | 5 | 7 | 1.33 秒 |
| 原厂负重档 | 15/64 | 126 | 259 | 1.04 秒 |
| 原厂对档（负载一变同一拍就换，人做不到） | 17/64 | 52 | 98 | 1.04 秒 |
| **v4** | 19/64 | **8** | **8** | **0.99 秒** |

| | Episodes with a fall | Did not settle | Sustained chatter | Stop settling (median) |
|---|---|---|---|---|
| Factory Normal gains | 64/64 (falls as soon as loaded) | 5 | 7 | 1.33 seconds |
| Factory Weight gains | 15/64 | 126 | 259 | 1.04 seconds |
| Factory, gear matched to the load (switched on the same tick the load changes, which a human cannot do) | 17/64 | 52 | 98 | 1.04 seconds |
| **v4** | 19/64 | **8** | **8** | **0.99 seconds** |

v4 的不收敛和持续抖振只有原厂负重档的几十分之一。摔倒的一半以上（10/19）是「行驶中被推」，那些局原厂也摔了。

v4's counts of "did not settle" and "sustained chatter" are only a small fraction (about 1/16 to 1/32) of those of the factory Weight gains. More than half of the falls (10/19) were "pushed while driving", and the factory controller also fell in those episodes.

**换档表现**：

**Gear-change behavior**:

- 升档 228 次，94% 成功（升档后 3 秒内没摔），升档后收敛中位 0.85 秒；
- 降档 272 次，降档后收敛中位 0 秒（降档发生在车已稳定之后，本身不扰动车）；
- 回弹（降档后 2 秒内又被升回）31 次：其中 29 次由新的推击、改坡度或转弯引起，30 次之后又降了回来（中位 2.5 秒）；没有任何事件的回弹只有 2 次，都在 0.3 秒内被快速撤回。

- 228 upshifts, 94% successful (no fall within 3 seconds after the upshift), median settling after an upshift 0.85 seconds;
- 272 downshifts, median settling after a downshift 0 seconds (downshifts happen after the car is already steady and do not disturb the car themselves);
- 31 rebounds (upshifted again within 2 seconds after a downshift): 29 of them were caused by a new push, a slope change or a turn, and 30 later downshifted again (median 2.5 seconds); only 2 rebounds had no event behind them, and both were undone by quick revert within 0.3 seconds.

---

## 9. 已知局限，以及上实车之前必须做的标定 / 9. Known limitations, and the calibration required before running on the real car

### 9.1 已知局限 / 9.1 Known limitations

1. **负载悄悄改变时，档位靠「事件」纠正**：轻档和中档下没有任何信号能分辨负载，只有升档回到最硬档才能重新判断。随机测试里，加载后判对 22/40 次（中位 6 秒），卸载后判对 18/43 次。加载后一直没纠正的情况里，车会因为偏软而晃起来触发升档，所以很少摔（40 次里摔了 2 次）；卸载后没纠正只是偏硬。真车上加卸负载时一定会碰到车，这一碰本身就是事件，纠正机会可能比孪生多。
2. **两档不能区分 2 千克和 4 千克**：在最硬档下两者的抖振读数重叠，4 千克只能和 2 千克共用 f=0.40。
3. **行驶中被推**：满速行驶时被推，余量本来就小，v4 和原厂一样会摔。
4. **原地满档转弯**：孪生里车身前后晃 ±14 度，随机测试里有 9 次升档出在这里。真车上实际晃多大，需要实测。
5. **（10-03）宽限期内对轻碰不敏感**：按键后 1.5 秒内、推车降档后 2 秒内，倾斜不到 16 度的快速轻碰不会升档。这时用的是空载档（等于原厂模式 1），表现和原厂一样。
6. **（10-03）被推后有 0.5 秒硬档轻抖**：升档到最硬档、再判回空载档需要 0.5 秒，空车在这期间会轻微抖动。
7. **（10-03）带负载的实车表现本轮没有重新测**，Python 版也没有同步本轮修改（见 12.5 节）。

1. **When the load changes quietly, the gain is corrected only by an "event"**: in the light and medium gains there is no signal that can tell the load apart; only an upshift back to the stiffest gain allows a new decision. In the random test, the load was identified correctly 22/40 times after loading (median 6 seconds) and 18/43 times after unloading. In the cases where loading was never corrected, the car is too soft, starts to sway and triggers an upshift, so it rarely falls (2 falls out of 40); when unloading is not corrected, the car is merely too stiff. On the real car, adding or removing a load always involves touching the car, and that touch is itself an event, so there may be more chances to correct than in the twin.
2. **The two-gain scheme cannot distinguish 2 kilograms from 4 kilograms**: at the stiffest gain their chatter readings overlap, so 4 kilograms has to share f=0.40 with 2 kilograms.
3. **Pushed while driving**: when pushed while driving at full speed, the margin is already small, and v4 falls just like the factory controller.
4. **Full-speed turning in place**: in the twin the body sways ±14 degrees forward and backward, and 9 upshifts in the random test happened here. How much the real car actually sways needs to be measured.
5. **(10-03) Insensitive to light bumps during the grace periods**: within 1.5 seconds after a key press and within 2 seconds after a downshift following a push, a quick light bump with a tilt under 16 degrees does not cause an upshift. During this time the empty-load gain is in use (equal to factory mode 1), so the behavior is the same as the factory controller.
6. **(10-03) 0.5 seconds of slight stiff-gain shaking after a push**: upshifting to the stiffest gain and then switching back to the empty gear takes 0.5 seconds, and an empty car shakes slightly during that time.
7. **(10-03) Loaded real-car behavior was not re-tested in this round**, and the Python version was not updated with this round's changes either (see Section 12.5).

### 9.2 实车标定步骤 / 9.2 Real-car calibration steps

1. 用 `e026 keil\realcar\v4_cal.ps1` 录波。它把小车锁在模式 26 的最高档（等价于 f=1），对每种负载录 20 秒（一半静止、一半来回开），再录一段工作档静止数据。**空载请在不同电量下录 2～3 次**。
2. 用 `e026\balance_bot_windows\balance_bot\scripts\twin_fit\v4_cal_analyze.py <录波目录>` 分析。按「有把握才判轻」的规则算出 `OSC_SPLIT = 空载静止读数 × 0.9`；如果有任何带载读数超过分界，它会报错，说明这台车两档分不开。它同时给出 `MOVE_SPLIT`、`EVENT_SPLIT`、`REC_ON_STILL`、`ARM_DEG` 的建议值，直接打印成可以粘进 `v4_core.c` 的 `#define` 行。
3. 把这些值改进 `v4_core.c`，在电脑上重新运行 `v4_port_check.py`（Python 侧同步改值）确认一致，再用 Keil 重新编译，得到新的 hex。

1. Record data with `e026 keil\realcar\v4_cal.ps1`. It locks the car at the highest gain of mode 26 (equivalent to f=1), records 20 seconds for each load (half standing still, half driving back and forth), and then records a segment of standing-still data at the working gain. **For the empty car, please record 2 to 3 times at different battery charge levels**.
2. Analyze with `e026\balance_bot_windows\balance_bot\scripts\twin_fit\v4_cal_analyze.py <录波目录>` (`<录波目录>` = recording directory). Following the rule "decide light only when confident", it computes `OSC_SPLIT = 空载静止读数 × 0.9` (empty-car standing-still reading × 0.9); if any loaded reading exceeds the split, it reports an error, which means the two gains cannot be told apart on this car. It also gives suggested values for `MOVE_SPLIT`, `EVENT_SPLIT`, `REC_ON_STILL` and `ARM_DEG`, printed directly as `#define` lines that can be pasted into `v4_core.c`.
3. Put these values into `v4_core.c`, rerun `v4_port_check.py` on the PC (with the same values changed on the Python side) to confirm they match, and then rebuild with Keil to get the new hex.

---

## 10. 所有相关文件的位置 / 10. Locations of all related files

**固件（Keil 工程，真正参与编译的源码）**

**Firmware (Keil project, the source code that is actually compiled)**

- **本版的工程在 `C:\Users\jiang li\Downloads\e026 keil\v4_work_20261003\`**。原来的 `stm32_Balance_Car_L\` 在 10-03 被改成了朋友强化学习模型的测试工程（只剩模式 1 和 28），不能再用来编译 v4。v4_work_20261003 是从它复制出来、再还原 v4 的原厂接入文件得到的；在加本版修改之前，先编译了一次，和实车测试版 realtest2 的 hex 逐字节相同，证明还原正确。
- v4 逻辑：`v4_work_20261003\APP\PID\v4_core.c`、`v4_core.h`
- 接入层：`v4_work_20261003\APP\PID\v4_adapt.c`、`v4_adapt.h`
- 工程文件：`v4_work_20261003\USER\stm32_Balance_Car.uvprojx`
- 编译输出：`v4_work_20261003\OBJ\stm32_Balance_Car_L.hex`
- 10-03 的各个测试版 hex：`e026 keil\v4_test_*_20261003\`（上午）、`e026 keil\v4_diag_20261003\`（下午，含 realtest3、realtest4 和诊断版）
- 对拍用的电脑端驱动：`APP\PID\v4_host.c`（不在固件工程里）
- 修改前的备份：`C:\Users\jiang li\Downloads\e026 keil\v4_backup\`

- **The project for this version is in `C:\Users\jiang li\Downloads\e026 keil\v4_work_20261003\`**. The original `stm32_Balance_Car_L\` was turned into a test project for a friend's reinforcement learning model on 10-03 (only modes 1 and 28 remain) and can no longer be used to build v4. v4_work_20261003 was copied from it, with v4's integration files in the factory code restored; before this version's changes were added, it was built once and the hex was byte-for-byte identical to that of the real-car test build realtest2, which proves the restore was correct.
- v4 logic: `v4_work_20261003\APP\PID\v4_core.c`, `v4_core.h`
- Integration layer: `v4_work_20261003\APP\PID\v4_adapt.c`, `v4_adapt.h`
- Project file: `v4_work_20261003\USER\stm32_Balance_Car.uvprojx`
- Build output: `v4_work_20261003\OBJ\stm32_Balance_Car_L.hex`
- The various 10-03 test build hex files: `e026 keil\v4_test_*_20261003\` (morning), `e026 keil\v4_diag_20261003\` (afternoon, including realtest3, realtest4 and the diagnostic builds)
- PC-side driver for the tick-by-tick comparison: `APP\PID\v4_host.c` (not part of the firmware project)
- Backup from before the changes: `C:\Users\jiang li\Downloads\e026 keil\v4_backup\`

**Python 版和测试（`C:\Users\jiang li\Downloads\e026\balance_bot_windows\balance_bot\scripts\rl\`）**

**Python version and tests (`C:\Users\jiang li\Downloads\e026\balance_bot_windows\balance_bot\scripts\rl\`)**

- `switch_v4.py`（v4 主体）、`switch_v3.py`（v4 继承的底层：抖振、探测、锁存、两档）
- `v4_port_check.py`（C 与 Python 逐拍对拍）
- `final_table.py`（主对比表）、`drive_test.py`（行驶表）、`push_table.py`、`push_settle.py`（推击）、`tilt_envelope.py`（回稳包络）、`hard_brake.py`（急刹）、`cross_settle.py`（交叉工况）、`random_field.py`（随机场地）
- 调参：`cem_v3.py`（v3 的 CEM 搜索）、`tune_v4_r2.py`（第二轮）、`tune_v4_r3.py`（第三轮开关）、`tune_v4_map.py`（两档数值网格）
- 过程记录：`LAYERED_RESULT.md`（每一版的最终状态和对比表）、`BUGS.md`（每个问题的发现和修复，第十三到十五节是 v4）

- `switch_v4.py` (main body of v4), `switch_v3.py` (the lower layer v4 inherits: chatter, probing, latch, two gains)
- `v4_port_check.py` (tick-by-tick comparison of C and Python)
- `final_table.py` (main comparison table), `drive_test.py` (driving table), `push_table.py`, `push_settle.py` (push), `tilt_envelope.py` (recovery envelope), `hard_brake.py` (hard braking), `cross_settle.py` (cross conditions), `random_field.py` (random field)
- Tuning: `cem_v3.py` (CEM search for v3), `tune_v4_r2.py` (second round), `tune_v4_r3.py` (third round, switches), `tune_v4_map.py` (value grid for the two gains)
- Process records: `LAYERED_RESULT.md` (final state and comparison tables for each version), `BUGS.md` (discovery and fix of each problem; Sections 13 to 15 cover v4)

**验证原始数据（`C:\Users\jiang li\Downloads\e026\balance_bot_windows\balance_bot\runs_e2e\`）**

**Raw validation data (`C:\Users\jiang li\Downloads\e026\balance_bot_windows\balance_bot\runs_e2e\`)**

- `val_final\`：本版最终回归（主对比表、推击、急刹、交叉、行驶等全部日志）
- `val_v4r4\`：随机场地测试、换档统计、两档数值网格

- `val_final\`: final regression for this version (all logs: main comparison table, push, hard braking, cross conditions, driving, and so on)
- `val_v4r4\`: random field test, gear-change statistics, value grid for the two gains

**实车标定**

**Real-car calibration**

- 录波：`C:\Users\jiang li\Downloads\e026 keil\realcar\v4_cal.ps1`
- 分析：`C:\Users\jiang li\Downloads\e026\balance_bot_windows\balance_bot\scripts\twin_fit\v4_cal_analyze.py`
- 9 月 23 日的真车测量记录：`C:\Users\jiang li\Downloads\e026 keil\realcar\PARAMS.md`

- Recording: `C:\Users\jiang li\Downloads\e026 keil\realcar\v4_cal.ps1`
- Analysis: `C:\Users\jiang li\Downloads\e026\balance_bot_windows\balance_bot\scripts\twin_fit\v4_cal_analyze.py`
- Real-car measurement records from September 23: `C:\Users\jiang li\Downloads\e026 keil\realcar\PARAMS.md`

---

## 11. 分工：人做了什么、大语言模型做了什么、提示词 / 11. Division of work: what the human did, what the large language model did, and the prompts

本项目由用户（人）和一个大语言模型协作完成。模型是 Anthropic 公司的 Claude，通过 Claude Code（在用户电脑上运行、可以读写文件和执行命令的命令行工具）工作；整理本文件时使用的版本是 Claude Opus 5.5，会话早期部分具体使用的模型版本，记录里没有核实。

This project was done jointly by the user (a human) and a large language model. The model is Claude, made by Anthropic, working through Claude Code (a command-line tool that runs on the user's computer and can read and write files and run commands). The version used while preparing this file is Claude Opus 5.5; the exact model version used in the early part of the sessions was not verified in the records.

### 11.1 人（用户）做了什么 / 11.1 What the human (the user) did

**定目标、定约束、定取舍规则**

**Set the goal, the constraints and the rules for accepting or rejecting changes**

- 定下总目标：小车在不同工况及其衔接下快速稳定、不摔，并且超过原厂。
- 决定放弃纯强化学习、改用可解释的分层方法（2026-09-29：「现在只靠rl解决全部太难了，暂停这些。我现在要给你更可以实现和解释的指导」，并贴入四层架构的分析；「之后对于小车的控制模型，都基于刚刚提到的分层」；「你来做吧，不用rl和nn了」）。
- 指定用交叉熵搜索辅助调参（2026-09-30：「你做吧，要求满足不同工况及它们的衔接快速稳定不摔，超过原厂，用cem辅助试试」）。
- 规定汇报格式：所有对比表都要带原厂基线（「之后的结果都要把多层结合和基线对比列出来」「之后的表格对比都要有基线的」）。
- 规定取舍规则（「有进步就改进，没进步或差别不大就用原来的」）。
- 规定以收敛为核心要求（「我希望任何情况下小车的控制系统都能逻辑自洽地收敛」）。

- Set the overall goal: the car must settle quickly and not fall under different operating conditions and the transitions between them, and it must beat the factory firmware.
- Decided to drop pure reinforcement learning and switch to an explainable layered method (2026-09-29: "Solving everything with RL alone is too hard right now, pause all of that. I will now give you guidance that is more achievable and explainable", together with a pasted analysis of the four-layer architecture; "From now on, the car's control models are all based on the layering I just described"; "You do it, no more RL or neural networks").
- Specified using the cross-entropy method (CEM) to help with tuning (2026-09-30: "Go ahead. The requirement is that it settles quickly and does not fall under different conditions and their transitions, beats the factory firmware; try using CEM to help").
- Set the reporting format: every comparison table must include the factory baseline ("From now on, all results must list the combined multi-layer method against the baseline", "From now on, all comparison tables must include the baseline").
- Set the acceptance rule ("If it improves, adopt the change; if it does not improve or the difference is small, keep the original").
- Made convergence the core requirement ("I want the car's control system to converge in a logically self-consistent way in every situation").

**提出关键设计思路**（这些是 v4 结构的来源）

**Proposed the key design ideas** (these are the origin of the v4 structure)

- 用「大倾角下回归稳定的能力」来统一处理落地、冲击（「换个思路，车在落地后会有倾角，我们只看车在大倾角下回归稳定的能力行吗」）。
- **升档和受冲击是同一件事**（「我觉得升档和受冲击其实是一样的，都是倾角过大，只不过升档需要持续，冲激快速回调会振荡，会通过降档机制回来」）。这是 v4 大倾角升档、借升档重新探测负载的核心想法。
- 尝试增益大于负重档的紧急控制以逼近物理极限，并要求同时考虑回正后降档。
- 转向也要算作指令（「左右转动也要算成指令」）。
- 要求关注降档稳定性（「这个回弹还会降回去吗，降档稳定很重要」）。

- Use "the ability to return to stability from a large tilt" as one unified way to handle landings and shocks ("Let's think differently: after landing the car will be tilted, can we just look at the car's ability to return to stability from a large tilt?").
- **An upshift and a shock are the same thing** ("I think an upshift and a shock are really the same: both are an excessive tilt, except that an upshift needs to be sustained, while a shock snaps back quickly and oscillates, and it will come back down through the downshift mechanism"). This is the core idea behind v4's large-tilt upshift and using the upshift to probe the load again.
- Try emergency control with gains larger than the factory Weight gains to get closer to the physical limit, and at the same time take the downshift after recovery into account.
- Turning also counts as a command ("Turning left and right must also count as a command").
- Asked for attention to downshift stability ("Will it come back down after this rebound? Downshift stability is very important").

**用实车知识纠正测试方法**

**Corrected the test methods using knowledge of the real car**

- 指出实车只有前后左右四个键、松手就停（「缓停和急刹的区别，因为实际操控小车只有前后左右四个选项」），据此删掉了测试中不存在的「缓停」和「斜向」。
- 指出随机测试结论不对（「这不应该，在e026下的mujuco环境下小车行驶地很好」），据此查出测试场地撞墙的问题。
- 要求用随机指令替代逐个设计工况（「一步一步告诉你工况太麻烦了，我需要你给小车随机指令……然后检测出问题的环节主要在哪」）。
- 叫停过于复杂的方案（「不要测量『倾角/轮加速度』的比值这些，太复杂了」）。
- 要求核实孪生参数来源（「你先确定 PARAMS.md 是你23号得到的数据」），并要求在修正后的孪生上重新验证全部结果。

- Pointed out that the real car has only four keys (forward, back, left, right) and stops when the key is released ("about the difference between a gentle stop and a hard brake: when actually driving the car there are only four options, forward, back, left, right"); based on this, the non-existent "gentle stop" and "diagonal" manoeuvres were removed from the tests.
- Pointed out that the conclusion of the random test was wrong ("This should not be; in the MuJoCo environment under e026 the car drives very well"); this led to finding that the car was hitting the walls of the test arena.
- Asked for random commands instead of designing operating conditions one by one ("Telling you the conditions step by step is too tedious, I need you to give the car random commands …and then detect which stage is mainly causing the problems").
- Stopped overly complicated approaches ("Don't measure things like the ratio 'tilt / wheel acceleration', it's too complicated").
- Asked to verify the source of the digital twin's parameters ("First make sure PARAMS.md is the data you obtained on the 23rd"), and asked for all results to be re-verified on the corrected twin.

**实车工作**

**Work on the real car**

- 在真车上完成烧录、按键操作、死区测试、开环扫频、行驶录波等测量（2026-09-16、09-23），这些数据是数字孪生参数的来源。
- 10-03：在真车上测试 10-02 版和后续 5 个测试版，描述现象（空载抖、降档慢、正常开车升档、推车后长时间小幅振荡），按要求做推车和驾驶测试，最后确认定版。

- Did the flashing, key operation, dead band tests, open-loop frequency sweeps, driving recordings and other measurements on the real car (2026-09-16, 09-23); these data are the source of the digital twin's parameters.
- 10-03: tested the 10-02 version and the 5 following test versions on the real car, described what was observed (chatter when empty, slow downshift, upshifts during normal driving, long small-amplitude oscillation after a push), did the push and driving tests as requested, and finally confirmed the release version.

### 11.2 大语言模型做了什么 / 11.2 What the large language model did

**编程**

**Programming**

- 编写全部 Python 代码：v1～v4 各版切换器（`switch_v2.py`、`switch_v3.py`、`switch_v4.py`）、交叉熵搜索脚本（`cem_v2.py`、`cem_v3.py` 等，含 16 进程并行和平台停止）、各类调参脚本（`tune_v4_*.py`）、全部测试脚本（主对比表、行驶、推击、急刹、交叉工况、回稳包络、随机场地）。
- 修改数字孪生：加入反向死区补偿、改正齿轮积分步数的自动计算等。
- 把 Python 逻辑移植成单片机 C 代码（`v4_core.c/.h`、`v4_adapt.c/.h`），并按字节修改原厂固件的 8 处接入点（保持 GBK 编码和原有格式不变）。
- 编写逐拍对拍工具（`v4_port_check.py`、`v4_host.c`）和实车标定脚本（`v4_cal.ps1`、`v4_cal_analyze.py`）。

- Wrote all the Python code: the switchers of each version v1 to v4 (`switch_v2.py`, `switch_v3.py`, `switch_v4.py`), the cross-entropy method scripts (`cem_v2.py`, `cem_v3.py` and others, including 16-process parallelism and stopping on a plateau), the various tuning scripts (`tune_v4_*.py`), and all test scripts (main comparison table, driving, push, hard braking, cross conditions, recovery envelope, random arena).
- Modified the digital twin: added reverse dead-band compensation, fixed the automatic calculation of the number of gear integration steps, and so on.
- Ported the Python logic to microcontroller C code (`v4_core.c/.h`, `v4_adapt.c/.h`), and edited the 8 hook-in points of the factory firmware byte by byte (keeping the GBK encoding and the original formatting unchanged).
- Wrote the tick-by-tick comparison tools (`v4_port_check.py`, `v4_host.c`) and the real-car calibration scripts (`v4_cal.ps1`, `v4_cal_analyze.py`).

**实验和分析**

**Experiments and analysis**

- 设计考题、罚分和评测指标，跑完全部仿真实验（累计上万局），读结果，做逐拍追踪找根因。例如：v2 的闭环污染、升档反复触发、上膛自我解除、急刹误升档的真正原因（切换时机而非门限）、噪声门限失控、分界 3.0 在别的车上不成立。
- 每次改动后做回归测试，并按用户的规则决定采用或放弃（例如停车复查、重载行驶门限、两档数值调整都是测试后放弃的）。

- Designed the test scenarios, the penalties and the evaluation metrics, ran all the simulation experiments (more than ten thousand episodes in total), read the results, and traced tick by tick to find root causes. Examples: the closed-loop contamination in v2, repeated upshift triggering, arming disarming itself, the real cause of false upshifts on hard braking (the switching timing, not the threshold), the noise threshold running away, and the split value 3.0 not holding on other cars.
- Ran regression tests after every change, and decided to adopt or drop it according to the user's rule (for example, the re-check after stopping, the heavy-load driving threshold and the adjustment of the two gain values were all dropped after testing).

**编译和文档**

**Compilation and documentation**

- 用 Keil 命令行编译出 hex，核对编译结果（0 错误 0 警告）。
- 10-03：在固件串口日志里加入 v4 诊断量，写录制脚本，自己操作串口录实车数据；用同一份 C 代码做离线重放，比较改法；找出第 12 节的 7 个问题的原因并修改。
- 编写过程记录（`LAYERED_RESULT.md`、`BUGS.md`）和本说明。

- Built the hex file with the Keil command line and checked the build result (0 errors, 0 warnings).
- 10-03: added v4 diagnostic values to the firmware's serial log, wrote the recording script, and operated the serial port itself to record real-car data; used the same C code for offline replay to compare candidate fixes; found the causes of the 7 problems in Section 12 and fixed them.
- Wrote the process records (`LAYERED_RESULT.md`, `BUGS.md`) and this document.

**模型犯过、由用户发现或纠正的错误**（如实记录）

**Mistakes made by the model and found or corrected by the user** (recorded as they happened)

- 曾长期走端到端强化学习路线，成效不足，由用户叫停并改为分层方案。
- 曾在未核实的情况下说孪生参数「没有更新」，经用户追问后核对，说法有误，已更正。
- 新写的随机场地测试曾把「撞墙」当成控制器问题分析，由用户指出后才查清。
- 测试里用过实车并不存在的操作（缓停、斜向同时按满），由用户指出后删除。
- 曾把「增益大于负重档」的实验表述得像是有效，用户追问后澄清：它没有带来任何改善，已关闭。
- 10-03 第一次录制时一连上串口就开始高速录，导致第二次按 KEY1 检测不到，由用户反馈后改为车站稳后再录。
- 10-03 第一次重放时用了「开机状态」作起点，而录制开始时车早已锁定空载，导致重放结果错误；发现后改为从「已锁定空载」开始重放。

- Followed the end-to-end reinforcement learning route for a long time without enough results; the user stopped it and switched to the layered approach.
- Once stated, without checking, that the twin's parameters had "not been updated"; after the user asked again and it was checked, the statement turned out to be wrong and was corrected.
- The newly written random-arena test at first analysed "hitting the wall" as a controller problem; this was only cleared up after the user pointed it out.
- The tests used manoeuvres that do not exist on the real car (gentle stop, diagonal with two keys fully pressed); these were removed after the user pointed it out.
- Once described the "gains larger than the factory Weight gains" experiment as if it had worked; after the user asked, it was clarified that it brought no improvement at all, and it has been switched off.
- On 10-03, in the first recording, high-rate recording started as soon as the serial port was connected, so the second KEY1 press was not detected; after the user reported this, recording was changed to start only after the car is standing steadily.
- On 10-03, the first replay used the "power-on state" as its starting point, while at the start of the recording the car had long since latched as empty, so the replay results were wrong; once found, the replay was changed to start from "already latched as empty".

### 11.3 v4 设计过程中的关键提示词和对应结果 / 11.3 Key prompts during the v4 design and their results

时间为世界协调时（北京时间加 8 小时）。全部提示词原文见 `appendix_all_user_prompts.md`。

Times are in Coordinated Universal Time (Beijing time is UTC plus 8 hours). The original text of all prompts is in `appendix_all_user_prompts.md` (appendix: all user prompts).

| 时间 | 用户提示词（原文） | 模型据此做了什么 | 结果 |
|---|---|---|---|
| 09-29 12:35 | 「现在只靠rl解决全部太难了，暂停这些。我现在要给你更可以实现和解释的指导」（附四层架构分析） | 停止强化学习，按四层架构重组 | 架构固定为四层 |
| 09-29 14:59 | 「你来做吧，不用rl和nn了，但是之前提到的工况，我的目标仍然是进化到比原厂好」 | 开始做可解释的负载切换器，替代神经网络 | v1、v2 切换器 |
| 09-30 02:54 | 「你做吧，要求满足不同工况及它们的衔接快速稳定不摔，超过原厂，用cem辅助试试」 | 写交叉熵搜索，16 进程并行 | v2 罚分降 37%，发现闭环污染，改出 v3 锁存 |
| 09-30 04:12 | 「继续优化抖振和恢复平稳速度等，结合小车运动或者停止情况考虑。并且你要着重优化切换逻辑……」 | 优化 v3 锁存和安全逃生 | v3：空载抖振 9.0 → 0.5 |
| 09-30 15:18 | 「换个思路，车在落地后会有倾角，我们只看车在大倾角下回归稳定的能力行吗」 | 做回稳包络测试，研究大倾角恢复 | 回稳包络测试 |
| 09-30 16:45 | 「可以试一下，因为我觉得升档和受冲击其实是一样的，都是倾角过大……」 | 实现大倾角升档 + 借升档重新探测 | v4 第一版 |
| 10-01 09:28 | 「好，把实车以外能做的都做了，另外，尝试在倾角速度过大时使用f大于1的控制方法……同时回正降档也要考虑」 | f>1 实验；第二轮：双速平衡角、噪声门限、快速撤回；C 移植 | f>1 无效，关闭；第二轮采用 |
| 10-01 11:04～11:28 | 「考虑小车实际运行情况，在任何非收敛情况下收敛时间怎么样了」「行驶，急刹时的倾角会不会被误判」 | 推击收敛、交叉工况、行驶表、急刹测试 | 发现急刹/坡上误升档，加停稳门 |
| 10-01（插话） | 「我希望任何情况下小车的控制系统都能逻辑自洽地收敛」 | 确立「只在最硬档判负载，有疑问就回最硬档」原则 | 第三轮 |
| 10-01（插话） | 「一步一步告诉你工况太麻烦了，我需要你给小车随机指令……」 | 写随机场地测试 | 发现噪声门限失控、分界值不可靠 |
| 10-01（插话） | 「左右转动也要算成指令」 | Python 和固件都改 | 转弯不再误学噪声 |
| 10-01（插话） | 「这不应该，在e026下的mujuco环境下小车行驶地很好」 | 查出测试场地撞墙 | 改用开阔场地 |
| 10-01 13:37 | 「……降档平均收敛时间，升档成功率和收敛时间」 | 加换档统计 | 升档成功 94%，降档后收敛中位 0 秒 |
| 10-01 14:36 | 「1，和2做吧，有进步就改进，没进步或差别不大就用原来的的」 | 两档数值网格 + 全套回归 | 保留原值；回归无退步 |
| 10-01 16:30 | 「把最新模型搞成hex，放到e026 keil 的新建文件夹下……」 | 编译 hex、建本文件夹、写本说明 | 本文件夹 |
| 10-01（插话） | 「源码的目的一定要详细，必须要可解释，文档文件最后需要标注llm的所做事情和我的提示词，人的分工」 | 补写 6.6 节可解释性对照表和本节，提取全部提示词 | 本节及附录 |
| 10-03 05:44 | 「小车实测结果出来了，空载开启状态下小车有明显振荡，只有我按一下小车让它稳定才行」 | 查出电机关闭时也在探测；轻档改 0 | 12.3 问题 1、2 |
| 10-03 05:52 | 「三秒后不抖了，但是有没有办法缩短降档时间」 | 快速判轻 | 12.3 问题 3 |
| 10-03 06:14 | 「还有一个问题，小车在空载正常行驶时会因为角度偏大而导致升档带来高频振动，这个震动放缓消失的时间往往很慢……」 | 快速判轻扩展到行驶中 | 12.3 问题 4，测试版 realtest2 |
| 10-03 06:27 | 「升档后一旦恢复平稳就立刻回原档这一点，如果我就是在小车上放了负载呢」 | 放弃「一稳就回原档」 | 12.3 问题 4 |
| 10-03 09:20 | 「现在回到v4，抛开rl， v4即便是在正常模式下行驶，没有受到外力也会偶尔升档然后慢慢回档……你能不能从串口读取数据优化这一点」 | 加诊断列、录实车数据、离线重放 | 12.3 问题 5、6，realtest3 |
| 10-03（插话） | 「可以肉眼观察到小车频繁前后运动时会振荡」 | 与数据对照，确认是刹车误升档 | 12.3 问题 5 |
| 10-03 09:52 | 「是这样的，推车后，车会在向前行进时小幅振荡，回程图中没有振荡，但是回来时急刹会有长时间的小幅振荡，完全降档要2，3秒」 | 查出降档后自刹车的循环升档 | 12.3 问题 7，realtest4 |
| 10-03 10:01 | 「可以了，把这个定为最新正式模型」 | 建本文件夹 | 本版 |
| 10-03（插话） | 「更新readme放在同一文件夹下，把修改过程目的，问题与解决方法说清楚」 | 写第 12 节，同步更新各节 | 本说明 |

| Time | User prompt (original wording) | What the model did in response | Result |
|---|---|---|---|
| 09-29 12:35 | "Solving everything with RL alone is too hard right now, pause all of that. I will now give you guidance that is more achievable and explainable" (with the four-layer architecture analysis attached) | Stopped reinforcement learning and reorganised the work around the four-layer architecture | Architecture fixed at four layers |
| 09-29 14:59 | "You do it, no more RL or neural networks, but for the operating conditions mentioned before, my goal is still to evolve into something better than the factory firmware" | Started building an explainable load switcher to replace the neural network | v1 and v2 switchers |
| 09-30 02:54 | "Go ahead. The requirement is that it settles quickly and does not fall under different conditions and their transitions, beats the factory firmware; try using CEM to help" | Wrote the cross-entropy method (CEM) search, 16 processes in parallel | v2 penalty down 37%; closed-loop contamination found; v3 latch developed |
| 09-30 04:12 | "Keep optimising chatter and the speed of returning to calm, taking into account whether the car is moving or stopped. And you must focus on optimising the switching logic…" | Optimised the v3 latch and the safety escape | v3: empty chatter 9.0 → 0.5 |
| 09-30 15:18 | "Let's think differently: after landing the car will be tilted, can we just look at the car's ability to return to stability from a large tilt?" | Built the recovery-envelope test and studied recovery from large tilts | Recovery-envelope test |
| 09-30 16:45 | "Worth a try, because I think an upshift and a shock are really the same: both are an excessive tilt…" | Implemented the large-tilt upshift + probing again via the upshift | First version of v4 |
| 10-01 09:28 | "OK, do everything that can be done without the real car. Also, try a control method with f greater than 1 when the tilt rate is too large… and also consider the downshift after recovery" | f>1 experiment; second round: dual-speed balance angle, noise threshold, quick revert; C port | f>1 had no effect, switched off; second round adopted |
| 10-01 11:04～11:28 | "Considering how the car actually runs, how is the convergence time in any non-converging situation?" "When driving, will the tilt during hard braking be misjudged?" | Push convergence, cross conditions, driving table, hard-braking test | Found false upshifts on hard braking / on slopes; added the stop-calm gate |
| 10-01 (interjection) | "I want the car's control system to converge in a logically self-consistent way in every situation" | Established the principle "judge the load only in the stiffest gear; when in doubt, go back to the stiffest gear" | Third round |
| 10-01 (interjection) | "Telling you the conditions step by step is too tedious, I need you to give the car random commands…" | Wrote the random-arena test | Found the noise threshold running away and the split value being unreliable |
| 10-01 (interjection) | "Turning left and right must also count as a command" | Changed both the Python code and the firmware | Turning no longer causes noise to be learned by mistake |
| 10-01 (interjection) | "This should not be; in the MuJoCo environment under e026 the car drives very well" | Found that the car was hitting the walls of the test arena | Switched to an open arena |
| 10-01 13:37 | "…average convergence time after downshift, upshift success rate and convergence time" | Added gear-change statistics | Upshift success 94%; median convergence after downshift 0 seconds |
| 10-01 14:36 | "Do 1 and 2. If it improves, adopt the change; if it does not improve or the difference is small, keep the original" | Grid over the two gain values + full regression | Original values kept; no regression |
| 10-01 16:30 | "Build the latest model into a hex and put it in a new folder under e026 keil…" | Built the hex, created this folder, wrote this document | This folder |
| 10-01 (interjection) | "The purpose of the source code must be detailed and explainable; at the end the documentation must state what the LLM did, my prompts, and the human's share of the work" | Added the explainability cross-reference table in Section 6.6 and this section; extracted all prompts | This section and the appendix |
| 10-03 05:44 | "The real-car test results are in: when started empty, the car oscillates clearly, and only stabilises after I press on it once" | Found that probing was running even while the motors were off; light gear changed to 0 | 12.3 problems 1 and 2 |
| 10-03 05:52 | "It stops chattering after three seconds, but is there a way to shorten the downshift time?" | Fast "light" decision | 12.3 problem 3 |
| 10-03 06:14 | "Another problem: when the car is driving normally while empty, a slightly large angle causes an upshift that brings high-frequency vibration, and this vibration often takes a long time to die down…" | Extended the fast "light" decision to driving | 12.3 problem 4, test version realtest2 |
| 10-03 06:27 | "About going straight back to the original gear as soon as it is calm after an upshift: what if I actually did put a load on the car?" | Dropped "go back to the original gear as soon as it is calm" | 12.3 problem 4 |
| 10-03 09:20 | "Now back to v4, forget RL. Even when driving in normal mode with no external force, v4 occasionally upshifts and then slowly shifts back… can you read data from the serial port to improve this?" | Added diagnostic columns, recorded real-car data, offline replay | 12.3 problems 5 and 6, realtest3 |
| 10-03 (interjection) | "I can see with my own eyes that the car oscillates when it moves back and forth frequently" | Compared with the data, confirmed false upshifts on braking | 12.3 problem 5 |
| 10-03 09:52 | "Here is what happens: after a push, the car oscillates slightly while moving forward; there is no oscillation on the way back in the plot, but on coming back the hard braking causes a long small-amplitude oscillation, and it takes 2 or 3 seconds to fully downshift" | Found the cyclic upshift caused by the car braking itself after a downshift | 12.3 problem 7, realtest4 |
| 10-03 10:01 | "That's good, make this the latest release model" | Created this folder | This version |
| 10-03 (interjection) | "Update the readme and put it in the same folder; explain clearly the purpose of the changes, the problems and the solutions" | Wrote Section 12 and updated the other sections to match | This document |

### 11.4 一句话概括分工 / 11.4 The division of work in one sentence

**人决定做什么、什么算好、什么时候停**：目标、架构方向、关键设计思路、取舍规则、实车知识、实车测量。**模型负责怎么做、以及证明做得对**：编程、实验设计、运行仿真、找根因、移植到固件、编译、记录。v4 里最核心的结构性想法（分层、升档即重新探测、转向算指令、逻辑自洽收敛、只看实车四键操作）来自用户；具体机制（锁存、上膛、停稳门、噪声门限防失控、有把握才判轻的分界等）是模型在实验中发现问题后逐个设计并用数据验证的。

**The human decided what to do, what counts as good, and when to stop**: the goal, the architectural direction, the key design ideas, the acceptance rules, knowledge of the real car, and the real-car measurements. **The model was responsible for how to do it, and for proving it was done right**: programming, experiment design, running simulations, finding root causes, porting to the firmware, compiling, and keeping records. The most central structural ideas in v4 (layering, upshift means probing again, turning counts as a command, logically self-consistent convergence, considering only the real car's four-key operation) came from the user; the concrete mechanisms (latch, arming, stop-calm gate, preventing the noise threshold from running away, a split that only decides "light" when confident, and so on) were designed one by one by the model after it found problems in experiments, and were verified with data.

---

## 12. 本版修改：2026-10-03 实车调试的目的、过程、问题与解决方法 / 12. Changes in this version: purpose, process, problems and solutions from the 2026-10-03 real-car debugging

### 12.1 为什么要改 / 12.1 Why changes were needed

10-02 版的全部验证都是在数字孪生（电脑里的虚拟小车）里做的。10-03 第一次在真车上跑，出现了孪生里没有出现过的问题：空载会抖、降档慢、正常开车也会莫名其妙升档、被推后要抖好几秒。这一版的目的只有一个：

All verification of the 10-02 version was done in the digital twin (the virtual car in the computer). On 10-03, when it was run on the real car for the first time, problems appeared that had never shown up in the twin: it chattered when empty, downshifted slowly, upshifted for no apparent reason during normal driving, and chattered for several seconds after being pushed. This version has only one purpose:

> **空载、没有外力时，v4 的表现要和原厂模式 1 一样安静；被推时照常升档抗冲击，推完要尽快回到空载档。**

> **When empty and with no external force, v4 must be as quiet as factory mode 1; when pushed, it should upshift as usual to resist the shock, and after the push it must return to the empty gear as quickly as possible.**

修改分两轮。上午第一轮修了"开机就抖""降档慢"，产出测试版 realtest2。下午第二轮修了"无外力也升档""升档后慢慢回档""被推后反复升档"，产出 realtest3、realtest4。realtest4 经用户实车确认后定为本版。

The changes were made in two rounds. The first round, in the morning, fixed "chatter right after power-on" and "slow downshift", producing test version realtest2. The second round, in the afternoon, fixed "upshifts with no external force", "slowly shifting back after an upshift" and "repeated upshifts after a push", producing realtest3 and realtest4. After the user confirmed it on the real car, realtest4 became this version.

每一处修改都按同一个流程来：**先在实车上看到现象，再从串口数据找到原因，然后在电脑上用录下的数据重放、比较改法，最后回到实车确认。**

Every change followed the same process: **first observe the behaviour on the real car, then find the cause in the serial-port data, then replay the recorded data on the computer to compare candidate fixes, and finally confirm on the real car.**

### 12.2 方法：从串口读数据，而不是靠猜 / 12.2 Method: read data from the serial port instead of guessing

**录什么。** 原厂固件已有一个 200 赫兹的串口日志（`tune_io.c`，每 5 毫秒一行，13 列）。本版把模式 27 下没用的两列换成了 v4 的内部状态。有了它们，每一次升档是哪条规则、在什么条件下触发的，都能直接看出来：

**What to record.** The factory firmware already has a 200 Hz serial log (`tune_io.c`, one line every 5 milliseconds, 13 columns). This version replaces two columns that are unused in mode 27 with v4's internal state. With them, one can see directly which rule triggered each upshift and under what conditions:

| 列 | 内容 |
|---|---|
| `ang_c` | 倾角 × 100（度） |
| `gyro` | 陀螺原始读数（除以 16.4 得到度每秒） |
| `k_c`（本版新含义） | v4 的**平衡角基准** × 100。升档规则拿当前倾角和它比。 |
| `lvl`（本版扩充） | 档位 f × 100，已锁定再加 1000，**处于"恢复中"再加 2000** |
| `cs`、`tv` | 蓝牙按键状态（0 = 没按，1 前进，2 后退，3 左，4 右）、目标速度 |
| 其余 | 左右电机 PWM、左右编码器、抖振读数等，和以前一样 |

| Column | Content |
|---|---|
| `ang_c` | Tilt × 100 (degrees) |
| `gyro` | Raw gyroscope reading (divide by 16.4 to get degrees per second) |
| `k_c` (new meaning in this version) | v4's **balance-angle reference** × 100. The upshift rule compares the current tilt with it. |
| `lvl` (extended in this version) | Gain coefficient f × 100, plus 1000 if latched, **plus 2000 if "in recovery"** |
| `cs`, `tv` | Bluetooth key state (0 = none pressed, 1 forward, 2 back, 3 left, 4 right), target speed |
| Others | Left and right motor PWM, left and right encoders, chatter reading and so on, same as before |

**怎么录。** 电脑端脚本 `real_car_logs/v4_logger.py` 连接串口（230400 波特率），把每一行存成 CSV。它还自动把档位变化写成一句话，放进 `v4_events.txt`：
- UPSHIFT = 倾角规则升档；
- LATCH = 判定负载并锁定；
- BOOST = 慢速加硬。

**How to record.** The PC-side script `real_car_logs/v4_logger.py` (`real_car_logs/` = real-car recordings) connects to the serial port (230400 baud) and saves each line as CSV. It also automatically writes each gear change as one sentence into `v4_events.txt`:
- UPSHIFT = upshift by the tilt rule;
- LATCH = load decided and latched;
- BOOST = slow stiffening.

录制中发现的两个注意事项：
1. **要等车开始平衡以后再开始录。** 录制时主循环忙着往串口发数据，查看按键的频率变低，第二次按 KEY1 会检测不到。第一次录制就因此卡住（`v4_log_20261003_172739.csv`），之后改成车站稳后再发 `r` 命令开始录。
2. 230400 波特率下约有 35% 的行来不及发出而被丢掉（每行带序号，丢了能看出来）。档位变化的那一行都抓到了，不影响结论；分析时用序号把时间对齐。

Two points to note that were found during recording:
1. **Wait until the car has started balancing before starting to record.** While recording, the main loop is busy sending data to the serial port and checks the keys less often, so the second KEY1 press is not detected. The first recording got stuck because of this (`v4_log_20261003_172739.csv`); after that, the `r` command to start recording was sent only once the car was standing steadily.
2. At 230400 baud, about 35% of the lines cannot be sent in time and are dropped (each line carries a sequence number, so drops are visible). All lines with gear changes were captured, so the conclusions are not affected; during analysis the sequence numbers are used to align the timing.

**怎么验证改法：离线重放。** 把记录里每一拍的倾角、陀螺读数、按键状态，喂给电脑上用 gcc 编译的**同一份** `v4_core.c`，看它每一拍做出什么决定。
- **可信度检查：** 用原版代码重放第一段记录，得到 9 次升档，时刻和真车完全一致。
- **用途：** 改一处代码，重放一遍，就能看出这段真实数据上会少几次升档。
- **局限：** 重放是"开环"的。真车升档以后，接下来的数据就是硬档下的抖动，新代码在真车上本来不会进入那种状态，但重放只能拿这些数据继续算。所以重放能证明"这次触发会不会发生"，不能证明"后面整段会怎样"。后者必须上车确认（见问题 7）。

**How to verify a fix: offline replay.** The tilt, gyroscope reading and key state of every tick in the recording are fed into **the same** `v4_core.c`, compiled with gcc on the computer, to see what decision it makes at each tick.
- **Credibility check:** replaying the first recording with the original code gives 9 upshifts, at exactly the same moments as on the real car.
- **Use:** change one piece of code, replay once, and you can see how many fewer upshifts there would be on this real data.
- **Limitation:** the replay is "open-loop". After the real car upshifts, the following data are the chatter in the stiff gear; on the real car the new code would never have entered that state, but the replay can only keep computing with these data. So the replay can prove "whether this particular trigger would happen", but not "what the whole rest of the segment would look like". The latter must be confirmed on the car (see problem 7).

### 12.3 问题与解决方法（按发现顺序） / 12.3 Problems and solutions (in the order they were found)

#### 第一轮（上午，测试版 realtest2） / Round 1 (morning, test version realtest2)

**问题 1：空载开机明显振荡，要用手按一下车才稳。**

**Problem 1: clear oscillation when powered on empty; it only became steady after pressing on the car by hand.**

- **原因：** 原厂的控制中断从初始化陀螺仪起就一直在运行，比第二次按 KEY1 早得多。车拿在手里、电机还没转的那段时间，v4 当成了"静止探测"。电机没转就没有抖振，3 秒后判成"带载"，锁在 f=0.40。按下开始后，空车在 0.40 档上剧烈抖。用户按一下车，触发升档和重新探测，才判回空载。
- **解决：** `v4_adapt.c` 的 `V4_Tick()` 里，只要电机处于关闭状态（还没开始、被拿起、倾角超过 40°、电池电压低于 9.6 伏），v4 就保持开机状态、不计时。真正开始平衡时才开始探测；倒下再扶起，也会从头重新判断负载。

- **Cause:** the factory control interrupt runs continuously from the moment the gyroscope is initialised, much earlier than the second KEY1 press. During the time the car is held in the hand with the motors not yet running, v4 treated it as "stationary probing". With the motors not running there is no chatter, so after 3 seconds it decided "loaded" and latched at f=0.40. After start was pressed, the empty car chattered violently in the 0.40 gear. Only when the user pressed on the car, triggering an upshift and probing again, did it decide "empty".
- **Solution:** in `V4_Tick()` of `v4_adapt.c`, whenever the motors are off (not yet started, picked up, tilt over 40°, battery voltage below 9.6 volts), v4 stays in its power-on state and does not count time. Probing starts only when balancing really starts; after falling over and being set upright again, it also judges the load again from scratch.

**问题 2：空载档 f=0.12 在真车上仍然抖。**

**Problem 2: the empty gear f=0.12 still chattered on the real car.**

- **证据：** 模式 26 的实车记录里，空车在 f=0 时抖振读数 4.5～7.4，在 f=0.25～0.5 时是 53.8～58.3。孪生把这个抖振低估了大约 30 倍，所以孪生里 0.12 看起来没问题。
- **解决：** 空载档 `F_LIGHT` 从 0.12 改为 **0**，也就是空载时完全使用原厂模式 1 的 PID。

- **Evidence:** in the mode 26 real-car recordings, the empty car had a chatter reading of 4.5～7.4 at f=0, and 53.8～58.3 at f=0.25～0.5. The twin underestimated this chatter by about 30 times, so 0.12 looked fine in the twin.
- **Solution:** the empty gear `F_LIGHT` was changed from 0.12 to **0**, meaning that when empty, the factory mode 1 PID is used entirely.

**问题 3：开机后要 3 秒才不抖（降档慢）。**

**Problem 3: after power-on it took 3 seconds to stop chattering (slow downshift).**

- **原因：** 静止探测要等抖振读数"稳定"（0.3 秒内最大和最小之差不超过平均值的 15%）才判断。真车读数一直在跳，永远不算稳定，只能等满 3 秒上限。
- **解决：** 新增"快速判轻"（`FAST_LIGHT_N` = 0.5 秒）。解锁 0.5 秒以后，只要最近 0.3 秒里**每一个**抖振读数都 ≥ 23（只有空车在最硬档上才会这么抖），立刻判空载，不必等读数稳定。孪生里空载降档从 1.2 秒缩到 0.6 秒，带载误判没有增加。

- **Cause:** stationary probing waits for the chatter reading to be "stable" (the difference between maximum and minimum within 0.3 seconds is no more than 15% of the mean) before deciding. On the real car the reading keeps jumping and never counts as stable, so it can only wait for the 3-second upper limit.
- **Solution:** added the fast "light" decision (`FAST_LIGHT_N` = 0.5 seconds). From 0.5 seconds after unlatching, as soon as **every** chatter reading in the last 0.3 seconds is ≥ 23 (only an empty car in the stiffest gear chatters this much), it decides "empty" immediately without waiting for the reading to be stable. In the twin, the empty downshift was shortened from 1.2 seconds to 0.6 seconds, with no increase in loaded cars being misjudged.

**问题 4：行驶中升档后，高频震动消失得很慢。**

**Problem 4: after an upshift while driving, the high-frequency vibration died away very slowly.**

- **原因：** 行驶中判空载原本要求连续开满 1 秒，而且 0.5 秒窗口里还留着升档之前的低读数；停车后又要等读数稳定，一样是 3 秒。
- **解决：** 快速判轻扩展到行驶中。从解锁那一刻起单独记一份读数窗口（`fwin`），不管车在开还是停，满足条件就立刻判空载。带负载的车在最硬档上很安静，读数到不了 23，所以不会被误判成空载。
- **考虑过但没采用的做法：** "升档后车一稳就直接回原来的档"。用户指出："如果我就是在小车上放了负载呢？"——这时车会回到站不住的空载档。孪生里这样做也不更快，所以没有采用。

- **Cause:** deciding "empty" while driving originally required 1 full second of continuous driving, and the 0.5-second window still held low readings from before the upshift; after stopping it had to wait for the reading to be stable, again 3 seconds.
- **Solution:** the fast "light" decision was extended to driving. From the moment of unlatching, a separate window of readings (`fwin`) is kept, and whether the car is driving or stopped, it decides "empty" immediately once the condition is met. A loaded car is very quiet in the stiffest gear and its reading does not reach 23, so it will not be misjudged as empty.
- **Considered but not adopted:** "after an upshift, go straight back to the original gear as soon as the car is calm". The user pointed out: "What if I actually did put a load on the car?" — in that case the car would go back to an empty gear in which it cannot stand. In the twin this was not faster either, so it was not adopted.

#### 第二轮（下午，测试版 realtest3、realtest4） / Round 2 (afternoon, test versions realtest3 and realtest4)

**问题 5：没有外力、正常开车也会升档，前后频繁开时能看到振荡。**

**Problem 5: upshifts during normal driving with no external force; oscillation visible when driving back and forth frequently.**

- **数据：** 2 分钟正常驾驶升档 9 次。每一次都发生在蓝牙按键按下或松开后 0.05～0.40 秒，而且多数是刚松手、车在刹车的时候。
- **原因：** 升档规则看的是"预测倾角 = 倾角偏离平衡角基准的量 + 0.1 秒 × 车身摆动角速度"，行驶中超过 16° 就升档。起步和刹车时车身摆得很快（最快每秒 265°），但实际只倾斜几度。例如第一次升档时，倾角只偏了 0.7°，角速度那一项却有 26.5°。在这段记录的 110 次按键变化里：
  - 只看倾角偏差，最大 11.5°，一次都不会超过 16°；
  - 加上角速度那一项，最大 31.5°，有 11 次超过 16°。
- **解决：** 新增"按键宽限期"（`EDGE_GRACE_N` = 1.5 秒）。`v4_adapt.c` 检测到蓝牙按键状态变化时，调用新函数 `V4_CmdEdge()`；之后 1.5 秒内，升档只比较倾角偏差本身，不加角速度那一项，门槛不变。重放用了 1.5 秒，是因为记录里由角速度触发的升档，最晚出现在按键后 1.42 秒。

- **Data:** 9 upshifts in 2 minutes of normal driving. Every one happened 0.05～0.40 seconds after a Bluetooth key was pressed or released, and most of them just after the key was released, while the car was braking.
- **Cause:** the upshift rule looks at "predicted tilt = deviation of the tilt from the balance-angle reference + 0.1 seconds × body angular rate", and upshifts while driving when this exceeds 16°. When starting off and braking the body swings fast (up to 265° per second), but it actually tilts only a few degrees. For example, at the first upshift the tilt deviated by only 0.7°, but the angular-rate term was 26.5°. Over the 110 key changes in this recording:
  - looking only at the tilt deviation, the maximum was 11.5°, and it never exceeded 16°;
  - adding the angular-rate term, the maximum was 31.5°, and it exceeded 16° 11 times.
- **Solution:** added the key-change grace period (`EDGE_GRACE_N` = 1.5 seconds). When `v4_adapt.c` detects a change in the Bluetooth key state, it calls the new function `V4_CmdEdge()`; for the next 1.5 seconds, the upshift compares only the tilt deviation itself, without the angular-rate term, with the threshold unchanged. The replay used 1.5 seconds because, in the recording, the latest upshift triggered by angular rate occurred 1.42 seconds after a key change.

**问题 6：升档后档位又被"慢慢加硬"，4～5 秒才回到空载档（用户说的"慢慢回档"）。**

**Problem 6: after an upshift the gear was "slowly stiffened" again, and it took 4～5 seconds to return to the empty gear (what the user called "slowly shifting back").**

- **数据：** 同一段记录里有 4 段"已锁定、档位却在 0 以上"的时间，档位最高加到 0.6～1.0，合计 14.8 秒。这段时间 PWM 在 ±2800 之间来回顶满，也就是在抖。
- **原因：** 一条连锁反应。
  1. 先发生一次误升档（问题 5）。
  2. 0.5 秒后快速判轻、回到空载档，但"恢复中"标志还没清除，这段时间平衡角基准被冻结、不更新。
  3. 车身这时还在大幅摆动（最大到 19°）。"慢速脱困"规则拿摆动去和冻结的基准比，误以为车一直斜着，于是把档位往上加。
  4. 空车加硬后又抖，再按 1.5 秒的时间常数慢慢退回。
- **解决：** 处于"恢复中"时，慢速脱困规则暂停累计（`STRUGGLE_IN_REC` = 0）。车刚受过冲击，来回摆是正常的，不应当被当作"一直斜着"。
- **重放比较**（同一段 2 分钟记录）：

  | 版本 | 无外力升档 | 慢速加硬时长（重放） |
  |---|---|---|
  | 原版 | 9 次（和真车一致） | 15.8 秒 |
  | 只加按键宽限期 | 3 次 | 4.8 秒 |
  | 只暂停恢复中的慢速脱困 | 9 次 | 0 秒 |
  | 两处都改（realtest3） | 3 次 | 0 秒 |

  剩下的 3 次，触发时真车正处在原版误升档后的硬档抖动里，数据已经被旧行为污染，只能上车看。

- **realtest3 实车结果：** 约 47 秒里前后切换按键 33 次，**0 次升档**（原版在类似开法下 2 分钟 9 次）。但出现了问题 7。

- **Data:** in the same recording there were 4 periods that were "latched but with the gear above 0"; the gear went up to as high as 0.6～1.0, for 14.8 seconds in total. During these periods the PWM swung back and forth to its ±2800 limits, that is, it was chattering.
- **Cause:** a chain reaction.
  1. First, a false upshift happens (problem 5).
  2. 0.5 seconds later, the fast "light" decision returns it to the empty gear, but the "in recovery" flag has not yet been cleared, and during this time the balance-angle reference is frozen and not updated.
  3. The body is still swinging strongly at this point (up to 19°). The slow escape rule compares the swing with the frozen reference, wrongly concludes that the car is tilted all the time, and so raises the gear.
  4. The empty car chatters again after being stiffened, and then slowly falls back with a 1.5-second time constant.
- **Solution:** while "in recovery", the slow escape rule pauses its accumulation (`STRUGGLE_IN_REC` = 0). The car has just taken a shock, so swinging back and forth is normal and should not be treated as "tilted all the time".
- **Replay comparison** (the same 2-minute recording):

  | Version | Upshifts without external force | Slow stiffening duration (replay) |
  |---|---|---|
  | Original | 9 (matches the real car) | 15.8 seconds |
  | Key-change grace period only | 3 | 4.8 seconds |
  | Pausing slow escape in recovery only | 9 | 0 seconds |
  | Both changes (realtest3) | 3 | 0 seconds |

  The remaining 3 were triggered while the real car was in the stiff-gear chatter that followed a false upshift of the original version; those data are already contaminated by the old behaviour, so this could only be checked on the car.

- **realtest3 real-car result:** about 47 seconds with 33 back-and-forth key changes, **0 upshifts** (the original version had 9 in 2 minutes of similar driving). But problem 7 appeared.

**问题 7：被推一下后，要抖 2～3 秒才完全降档（循环升档）。**

**Problem 7: after one push, it chattered for 2～3 seconds before fully downshifting (cyclic upshifts).**

- **用户描述：** 推车后，车向前时小幅振荡；回程时急刹，会有长时间的小幅振荡，完全降档要 2～3 秒。
- **数据：** 其中一次推车后，17 秒里升档 11 次，每次都在上一次降档后 0.8～1.4 秒，间隔几乎固定在 1.4 秒左右。人推车不会这么规律，说明是车自己在循环。
- **原因：**
  1. 被推后车向前冲，然后速度环把车拉回原位、自己刹住，车身猛地一摆。
  2. 这次刹车没有按键，问题 5 的宽限期管不到。
  3. 因为一直没按键，v4 认为车"静止"，用的是最严格的静止门槛（3～8°），而且这个门槛含角速度项。刹车那一摆马上超过门槛，又升档、0.5 秒后再降档，下一次往回刹车再升……
- **解决：** 新增"降档宽限期"（`RELATCH_GRACE`）。升档后 3 秒内重新锁定档位时：
  - 之后 2 秒（`RELATCH_GRACE_N`），升档只看倾角偏差，不加角速度项；
  - 把"按键静止计时"和"停稳门"都清零，所以这段时间按"行驶中"处理，门槛是 16°；
  - 车真正停稳以后，才恢复静止时的严格门槛。

  记录里降档后再升档的时刻，最晚是降档后 1.97 秒，所以宽限期取 2 秒；这些时刻的倾角偏差最大只有 5.1°，离 16° 很远。
- **为什么没有用重放证明：** 这段记录里真车每次都真的升了档，后面是硬档抖动，新代码在真车上本来就不会进入这种状态（见 12.2 的局限）。所以这一步直接上车验证。

- **User's description:** after a push, the car oscillates slightly while moving forward; on the way back it brakes hard and there is a long small-amplitude oscillation, and it takes 2～3 seconds to fully downshift.
- **Data:** after one of the pushes there were 11 upshifts in 17 seconds, each 0.8～1.4 seconds after the previous downshift, with the interval almost fixed at about 1.4 seconds. A human pushing the car would not be this regular, which shows the car was cycling by itself.
- **Cause:**
  1. After being pushed, the car surges forward, then the speed loop pulls the car back to its original position and brakes by itself, and the body swings sharply.
  2. No key is involved in this braking, so the grace period from problem 5 does not cover it.
  3. Since no key has been pressed, v4 considers the car "stationary" and uses the strictest stationary threshold (3～8°), and this threshold includes the angular-rate term. The swing from braking immediately exceeds the threshold, so it upshifts again, downshifts again 0.5 seconds later, then upshifts again on the next braking on the way back…
- **Solution:** added the post-shock (relatch) grace period (`RELATCH_GRACE`). When the gear is latched again within 3 seconds after an upshift:
  - for the following 2 seconds (`RELATCH_GRACE_N`), the upshift looks only at the tilt deviation, without the angular-rate term;
  - both the "key idle timer" and the stop-calm gate are reset to zero, so this period is treated as "driving", with a threshold of 16°;
  - only after the car has really come to a calm stop is the strict stationary threshold restored.

  In the recording, the latest re-upshift after a downshift occurred 1.97 seconds after the downshift, so the grace period was set to 2 seconds; at those moments the tilt deviation was at most 5.1°, far from 16°.
- **Why replay was not used as proof:** in this recording the real car really did upshift every time, followed by stiff-gear chatter, a state the new code would never enter on the real car (see the limitation in 12.2). So this step was verified directly on the car.

### 12.4 实车结果（realtest4 = 本版） / 12.4 Real-car results (realtest4 = this version)

| 情况 | 原厂模式 1 | 10-02 版之后的 realtest2 | realtest3 | **本版（realtest4）** |
|---|---|---|---|---|
| 频繁前后开、没有外力 | 不换档（单一增益，就是本版的空载档） | 2 分钟误升档 9 次 | 47 秒、33 次按键，0 次 | **33 秒、26 次按键，0 次** |
| 升档后又慢慢加硬 | — | 4 段，共 14.8 秒 | — | **0** |
| 被推一下 | 不换档 | 未单独测 | 一次推车连续升档 2～11 次 | **每次只升 1 次**（5 次推车，5 次升档） |
| 被推后回到空载档 | — | — | 2～3 秒以上 | **0.5 秒** |

| Situation | Factory mode 1 | realtest2 (after the 10-02 version) | realtest3 | **This version (realtest4)** |
|---|---|---|---|---|
| Frequent back-and-forth driving, no external force | No gear changes (single gain set, which is this version's empty gear) | 9 false upshifts in 2 minutes | 47 seconds, 33 key presses, 0 | **33 seconds, 26 key presses, 0** |
| Slow stiffening again after an upshift | — | 4 periods, 14.8 seconds in total | — | **0** |
| One push | No gear changes | Not tested separately | 2～11 consecutive upshifts per push | **Only 1 upshift each time** (5 pushes, 5 upshifts) |
| Return to the empty gear after a push | — | — | 2～3 seconds or more | **0.5 seconds** |

用户实车观察确认后定为正式版。全部原始记录在 `real_car_logs/` 里：

After the user confirmed it by observing the real car, it was made the release version. All raw recordings are in `real_car_logs/` (real-car recordings):

| 记录文件 | 用的固件 | 内容 |
|---|---|---|
| `v4_log_20261003_172739.csv` | realtest2 + 诊断列 | 第一次录制，因 KEY1 检测不到而中止 |
| `v4_log_20261003_172942.csv` | realtest2 + 诊断列 | 2 分钟正常驾驶，问题 5、6 的数据来源 |
| `v4_log_20261003_174605.csv` | realtest3 | 驾驶加推车，问题 7 的数据来源 |
| `v4_log_20261003_175721.csv` | realtest4 | 推车 5 次加驾驶，本版的验收数据 |

| Recording file | Firmware used | Content |
|---|---|---|
| `v4_log_20261003_172739.csv` | realtest2 + diagnostic columns | First recording, aborted because KEY1 was not detected |
| `v4_log_20261003_172942.csv` | realtest2 + diagnostic columns | 2 minutes of normal driving; data source for problems 5 and 6 |
| `v4_log_20261003_174605.csv` | realtest3 | Driving plus pushes; data source for problem 7 |
| `v4_log_20261003_175721.csv` | realtest4 | 5 pushes plus driving; acceptance data for this version |

### 12.5 这些修改的代价 / 12.5 The cost of these changes

1. **宽限期里不看摆动快慢：**
   - 适用时段：按键后 1.5 秒内，以及推车引起的升档降档后 2 秒内。
   - 影响：如果车只是被快速碰一下、倾斜不到 16°，就不会升档。这时车用的是空载档，也就是原厂模式 1 的增益，表现和原厂一样，不会比原厂差。
   - 照常升档的情况：推得狠、把车推斜超过 16°。
   - 不受影响的情况：静止站稳时被推，判断方法完全没变。
2. **被推后那 0.5 秒仍然是最硬档。** 空车在最硬档上会轻微抖一下，就是"推车后向前时小幅振荡"。这是"受冲击就切到最硬档、再重新判断负载"这个设计本身带来的。以后如果想去掉，可以试"先升到中间档"，代价是带重物时抗推变弱。
3. **带负载的实车表现，本轮没有重新测。** 本轮的修改都只在"空载档已锁定"或"刚升档"时起作用，逻辑上不改变带载判断，但还没有实测确认。
4. **Python 版没有同步。** 本轮修改只做在 C 代码（`v4_core.c`、`v4_adapt.c`）里，`switch_v4.py` 还是 10-02 的逻辑，所以第 7 节的 C 与 Python 逐拍对拍不再成立。以后如果还要在孪生里做实验，要先把这些修改移植回 Python。现在以 C 代码为准。

1. **During grace periods, the speed of the swing is ignored:**
   - When it applies: within 1.5 seconds after a key change, and within 2 seconds after the downshift that follows a push-induced upshift.
   - Effect: if the car is only bumped quickly and tilts less than 16°, it will not upshift. In that case the car is using the empty gear, that is, the factory mode 1 gains, so it behaves the same as the factory firmware and is no worse.
   - Cases that still upshift as usual: a hard push that tilts the car more than 16°.
   - Cases not affected: being pushed while standing still and steady; the decision method there is completely unchanged.
2. **For 0.5 seconds after a push, the car is still in the stiffest gear.** An empty car chatters slightly in the stiffest gear; this is the "slight oscillation while moving forward after a push". It comes from the design itself: "on a shock, switch to the stiffest gear, then judge the load again". If this is to be removed in the future, one could try "upshift to a middle gear first", at the cost of weaker push resistance when carrying a heavy load.
3. **The loaded behaviour on the real car was not re-tested in this round.** The changes in this round only take effect when "the empty gear is latched" or "just after an upshift", so logically they do not change the loaded decision, but this has not yet been confirmed by measurement.
4. **The Python version was not kept in sync.** The changes in this round were made only in the C code (`v4_core.c`, `v4_adapt.c`); `switch_v4.py` still has the 10-02 logic, so the tick-by-tick comparison between C and Python in Section 7 no longer holds. If experiments are to be done in the twin again in the future, these changes must first be ported back to Python. For now, the C code is authoritative.

### 12.6 改动在代码里的位置 / 12.6 Where the changes are in the code

| 改动 | 文件和位置 | 开关（`#define`，电脑编译时可以用 `-D` 改） |
|---|---|---|
| 电机关闭时保持开机状态 | `v4_adapt.c`，`V4_Tick()` 开头 | — |
| 空载档改为 0 | `v4_core.c`，`F_LIGHT` | — |
| 快速判轻（静止和行驶都适用） | `v4_core.c`，`fwin` 窗口和"fast light"一段 | `FAST_LIGHT_N`（0 = 关） |
| 按键宽限期 | `v4_adapt.c` 检测按键变化并调用 `V4_CmdEdge()`；`v4_core.c` 中 `trig` 的计算 | `EDGE_GRACE_N`（0 = 关） |
| 恢复中暂停慢速脱困 | `v4_core.c`，慢速脱困一段 | `STRUGGLE_IN_REC`（1 = 恢复旧行为） |
| 降档宽限期 | `v4_core.c`，"刚锁定"那一段 | `RELATCH_GRACE`（0 = 关）、`RELATCH_GRACE_N`、`RELATCH_GRACE_WITHIN_N` |
| 串口诊断列 | `tune_io.c`（`k_c`、`lvl`），`v4_adapt.c` 导出 `v4_th_ref`、`v4_rec` | — |

| Change | File and location | Switch (`#define`; can be changed with `-D` when compiling on the computer) |
|---|---|---|
| Stay in the power-on state while the motors are off | `v4_adapt.c`, start of `V4_Tick()` | — |
| Empty gear changed to 0 | `v4_core.c`, `F_LIGHT` | — |
| Fast "light" decision (applies both stationary and driving) | `v4_core.c`, the `fwin` window and the "fast light" section | `FAST_LIGHT_N` (0 = off) |
| Key-change grace period | `v4_adapt.c` detects key changes and calls `V4_CmdEdge()`; the calculation of `trig` in `v4_core.c` | `EDGE_GRACE_N` (0 = off) |
| Pause slow escape while in recovery | `v4_core.c`, the slow escape section | `STRUGGLE_IN_REC` (1 = restore the old behaviour) |
| Post-shock (relatch) grace period | `v4_core.c`, the "just latched" section | `RELATCH_GRACE` (0 = off), `RELATCH_GRACE_N`, `RELATCH_GRACE_WITHIN_N` |
| Serial diagnostic columns | `tune_io.c` (`k_c`, `lvl`), `v4_adapt.c` exports `v4_th_ref`, `v4_rec` | — |

---

## 13. 汇总：v4 与原厂两个模式的对比（跑分和跑分规则） / 13. Summary: v4 compared with the two factory modes (benchmark scores and scoring rules)

参加对比的三方：

The three contenders in the comparison:

| 名称 | 是什么 | 增益 |
|---|---|---|
| **原厂正常档** | 原厂模式 1，固定一套增益 | 平衡比例 96、平衡微分 0.48、速度比例 62、速度积分 0.31、转向 17 / 0.2（固件里存的是乘以 100 的值） |
| **原厂负重档** | 原厂负重模式，固定一套增益 | 平衡比例 192、平衡微分 1.50、速度比例 94.5、速度积分 0.4725、转向 14 / 0.2 |
| **v4（本版）** | 模式 27，自己判断负载、自己换档 | 在上面两套之间按档位系数 f 线性插值：空载 f=0（等于原厂正常档），带载 f=0.40，受冲击时临时 f=1（等于原厂负重档） |

| Name | What it is | Gains |
|---|---|---|
| **Factory Normal gains** | Factory mode 1, one fixed set of gains | Balance proportional 96, balance derivative 0.48, speed proportional 62, speed integral 0.31, steering 17 / 0.2 (the firmware stores these values multiplied by 100) |
| **Factory Weight gains** | Factory weight-carrying mode, one fixed set of gains | Balance proportional 192, balance derivative 1.50, speed proportional 94.5, speed integral 0.4725, steering 14 / 0.2 |
| **v4 (this release)** | Mode 27; it detects the load and shifts gains by itself | Linear interpolation between the two sets above using the gain coefficient f: empty f=0 (equal to the factory Normal gains), loaded f=0.40, temporarily f=1 during a shock (equal to the factory Weight gains) |

原厂没有自动换档，要靠人在开机前选模式。人选错了（带货用正常档，或者空车用负重档）会怎样，下面的表也都列出来了。

The factory firmware has no automatic gain shifting; a person has to choose the mode before power-on. The tables below also show what happens when the person chooses wrong (Normal gains while carrying cargo, or Weight gains with an empty car).

### 13.1 仿真跑分（数字孪生） / 13.1 Simulation benchmark scores (digital twin)

**跑分规则**（和交叉熵搜索 `cem_v3.py` 用的是同一个目标函数，**分数越低越好**）：

**Scoring rules** (the same objective function that the cross-entropy method (CEM) search `cem_v3.py` uses; **lower scores are better**):

1. **任务**：「停-走-停」。小车依次开到 3 个目标点（前进到 0.9 米，倒车到 0.3 米，再前进到 1.2 米）。每一段都要开到离目标 5 厘米以内，并在那里停住 5 秒，才算这一段完成。
2. **每一局的扣分**：
   - 三段都完成：扣分 = **变稳时间（秒）+ 0.5 × 抖振读数（度/秒）**。变稳时间指到达目标后车身安静下来所用的时间，一直没安静下来按 7 秒算；抖振读数是陀螺仪 8～16 赫兹频段的均方根，也就是高频抖动有多厉害。
   - 中途摔倒：扣分 = **30 × 没完成的段数占比**。例如第一段就摔了，扣 30 分。
3. **5 个工况**：空载、1 千克、2 千克、4 千克、空载被推 3 牛（推 0.12 秒）。每个工况跑 3 个随机种子，取平均。
4. **加权求和**：空载 ×2、1 千克 ×2、2 千克 ×1、4 千克 ×1.5、空载被推 ×1。空载和 1 千克权重高，因为空载是最常用的情况，1 千克是原厂两档都不合适的地方。

1. **Task**: "stop-go-stop". The car drives to 3 target points in turn (forward to 0.9 m, reverse to 0.3 m, then forward to 1.2 m). A segment counts as complete only when the car gets within 5 cm of the target and stays stopped there for 5 seconds.
2. **Penalty for each run**:
   - All three segments completed: penalty = **settle time (seconds) + 0.5 × chatter reading (degrees/second)**. The settle time is the time the body takes to become calm after reaching the target; if it never becomes calm, it counts as 7 seconds. The chatter reading is the root mean square of the gyroscope signal in the 8–16 Hz band, that is, how strong the high-frequency shaking is.
   - Fall partway through: penalty = **30 × fraction of segments not completed**. For example, a fall in the first segment costs 30 points.
3. **5 conditions**: empty, 1 kg, 2 kg, 4 kg, and empty with a 3 N push (push lasts 0.12 s). Each condition is run with 3 random seeds and averaged.
4. **Weighted sum**: empty ×2, 1 kg ×2, 2 kg ×1, 4 kg ×1.5, empty with push ×1. Empty and 1 kg have higher weights because empty is the most common case, and 1 kg is where neither factory gain set fits.

| 控制器 | **总分** | 空载（×2） | 1 千克（×2） | 2 千克（×1） | 4 千克（×1.5） | 空载被推 3 牛（×1） |
|---|---|---|---|---|---|---|
| 原厂正常档（模式 1） | 111.45 | 3.74 | 12.61 | 30.00（全摔） | 30.00（全摔） | 3.75 |
| 原厂负重档 | 115.41 | 23.85 | 17.36 | 3.74 | 3.59 | 23.85 |
| **v4（本版）** | **31.83** | 3.81 | 5.93 | 3.54 | 3.34 | 3.80 |

| Controller | **Total score** | Empty (×2) | 1 kg (×2) | 2 kg (×1) | 4 kg (×1.5) | Empty, pushed 3 N (×1) |
|---|---|---|---|---|---|---|
| Factory Normal gains (mode 1) | 111.45 | 3.74 | 12.61 | 30.00 (fell every run) | 30.00 (fell every run) | 3.75 |
| Factory Weight gains | 115.41 | 23.85 | 17.36 | 3.74 | 3.59 | 23.85 |
| **v4 (this release)** | **31.83** | 3.81 | 5.93 | 3.54 | 3.34 | 3.80 |

（表中各工况数字是加权前、3 个种子平均后的扣分；总分 = 各工况扣分 × 权重之和。2026-10-04 在数字孪生里重新跑。跑分脚本、动态库源码、接线核对脚本和原始日志都复制在本文件夹的 `source_snapshot/benchmark/`；原件在 `e026\balance_bot_windows\balance_bot\scripts\rl\` 和 `e026\balance_bot_windows\balance_bot\runs_e2e\`。）

(The per-condition numbers in the table are penalties before weighting, averaged over the 3 seeds; total score = sum of each condition's penalty × its weight. Re-run in the digital twin on 2026-10-04. The benchmark scripts, the dynamic-library source code, the wiring-check script and the raw logs are all copied into `source_snapshot/benchmark/` (source snapshot / benchmark) in this folder; the originals are in `e026\balance_bot_windows\balance_bot\scripts\rl\` and `e026\balance_bot_windows\balance_bot\runs_e2e\`.)

#### 分数由哪几部分构成，每一部分是什么意思 / What the score is made of, and what each part means

每一局的扣分只来自三样东西，三者相加：

The penalty of each run comes from only three things, added together:

| 组成 | 怎么算 | 代表小车的什么表现 |
|---|---|---|
| **变稳时间**（秒，每 1 秒扣 1 分） | 每一段从「给出新目标点」开始计时，到车**开到目标 5 厘米以内、并且车身安静下来**为止。「安静」的标准是：倾角离平衡位置不到 1 度，并且车身角速度小于每秒 20 度，连续保持 0.25 秒。三段取平均。某一段始终没安静下来，那一段不计；三段都没安静下来，按 7 秒算。 | **快不快、稳不稳**。这个时间里大部分是开车本身的时间：三个控制器开同样的路，最快也要约 3.1～3.5 秒。所以要看的是**超出约 3.3 秒的那部分**，那才是到了目标后晃来晃去、迟迟停不稳的时间；7 秒说明一直没停稳。 |
| **0.5 × 抖振读数**（度/秒） | 每一段到达后停住的那 5 秒里，取陀螺仪 8～16 赫兹频段的均方根（和固件 `load_adapt.c` 用的是同一组滤波器），三段平均，再乘 0.5。 | **高频抖得厉害不厉害**。人眼看到的「车在嗡嗡地抖」就是这个频段。正常站稳时大约 0.5；超过 5 就能明显看到抖；30 以上是剧烈自激抖动。 |
| **摔倒**（最多 30 分） | 30 × 没完成的段数 ÷ 3。第一段就摔扣 30 分，完成一段后摔扣 20 分，完成两段后摔扣 10 分。摔倒的那一局不再计变稳和抖振。 | **能不能完成任务**。第一段就摔的 30 分，相当于多 30 秒停不稳，或者抖振读数 60，所以摔倒是压倒性的惩罚。 |

| Component | How it is computed | What car behavior it represents |
|---|---|---|
| **Settle time** (seconds, 1 point per second) | For each segment, timing starts when "a new target point is given" and ends when the car **is within 5 cm of the target and the body has become calm**. "Calm" means: tilt within 1 degree of the balance position and body angular rate below 20 degrees per second, held continuously for 0.25 s. The three segments are averaged. A segment that never becomes calm is left out; if none of the three segments becomes calm, it counts as 7 seconds. | **How fast and how steady**. Most of this time is the driving itself: all three controllers drive the same route, and even the fastest needs about 3.1–3.5 s. So what matters is **the part beyond about 3.3 s**, which is the time spent swaying around after reaching the target and failing to stop steadily; 7 seconds means it never stopped steadily. |
| **0.5 × chatter reading** (degrees/second) | During the 5 seconds the car stays stopped after reaching each segment's target, take the root mean square of the gyroscope signal in the 8–16 Hz band (the same filters as in the firmware `load_adapt.c`), average over the three segments, then multiply by 0.5. | **How strong the high-frequency shaking is**. What the human eye sees as "the car buzzing and shaking" is this band. A normal steady stand is about 0.5; above 5 the shaking is clearly visible; above 30 is violent self-excited oscillation. |
| **Fall** (at most 30 points) | 30 × number of segments not completed ÷ 3. A fall in the first segment costs 30 points, a fall after completing one segment costs 20 points, and a fall after completing two segments costs 10 points. A run with a fall no longer counts settle time or chatter. | **Whether the task can be completed**. The 30 points for a fall in the first segment equal 30 extra seconds of not settling, or a chatter reading of 60, so a fall is an overwhelming penalty. |

每个工况跑 3 局（3 个随机种子），三局扣分取平均，就是上表各工况的数字；再乘以权重相加，得到总分。

Each condition is run 3 times (3 random seeds); the average penalty of the three runs is the per-condition number in the table above. Multiplying these by the weights and adding them gives the total score.

**各控制器的分数明细**（「变稳」「抖振」两列是三局里完成了的那几局的平均值；「扣分构成」是三局平均后的变稳分 + 抖振分 + 摔倒分）：

**Score breakdown per controller** (the "settle" and "chatter" columns are averages over the runs that were completed out of the three; "penalty breakdown" is the settle points + chatter points + fall points, each averaged over the three runs):

| 控制器 | 工况 | 摔倒 | 变稳时间 | 抖振读数 | 扣分构成（变稳 + 抖振 + 摔倒） | 工况扣分 | × 权重 | 计入总分 |
|---|---|---|---|---|---|---|---|---|
| 原厂正常档 | 空载 | 0/3 | 3.48 秒 | 0.52 | 3.48 + 0.26 + 0 | 3.74 | ×2 | 7.48 |
| 原厂正常档 | 1 千克 | **2/3**（一局走完 1 段摔、一局走完 2 段摔） | 6.80 秒 | 2.07 | 2.27 + 0.34 + 10.00 | 12.61 | ×2 | 25.22 |
| 原厂正常档 | 2 千克 | **3/3**（都在第一段摔） | — | — | 0 + 0 + 30.00 | 30.00 | ×1 | 30.00 |
| 原厂正常档 | 4 千克 | **3/3**（都在第一段摔） | — | — | 0 + 0 + 30.00 | 30.00 | ×1.5 | 45.00 |
| 原厂正常档 | 空载被推 3 牛 | 0/3 | 3.50 秒 | 0.51 | 3.50 + 0.25 + 0 | 3.75 | ×1 | 3.75 |
| | | | | | | | **总分** | **111.45** |
| 原厂负重档 | 空载 | 0/3 | **7 秒（一直没停稳）** | **33.70** | 7.00 + 16.85 + 0 | 23.85 | ×2 | 47.70 |
| 原厂负重档 | 1 千克 | 0/3 | **7 秒（一直没停稳）** | **20.73** | 7.00 + 10.36 + 0 | 17.36 | ×2 | 34.72 |
| 原厂负重档 | 2 千克 | 0/3 | 3.35 秒 | 0.78 | 3.35 + 0.39 + 0 | 3.74 | ×1 | 3.74 |
| 原厂负重档 | 4 千克 | 0/3 | 3.29 秒 | 0.60 | 3.29 + 0.30 + 0 | 3.59 | ×1.5 | 5.39 |
| 原厂负重档 | 空载被推 3 牛 | 0/3 | **7 秒（一直没停稳）** | **33.70** | 7.00 + 16.85 + 0 | 23.85 | ×1 | 23.85 |
| | | | | | | | **总分** | **115.41** |
| v4 | 空载 | 0/3 | 3.55 秒 | 0.52 | 3.55 + 0.26 + 0 | 3.81 | ×2 | 7.62 |
| v4 | 1 千克 | 0/3 | 4.09 秒 | 3.68 | 4.09 + 1.84 + 0 | 5.93 | ×2 | 11.86 |
| v4 | 2 千克 | 0/3 | 3.29 秒 | 0.50 | 3.29 + 0.25 + 0 | 3.54 | ×1 | 3.54 |
| v4 | 4 千克 | 0/3 | 3.11 秒 | 0.46 | 3.11 + 0.23 + 0 | 3.34 | ×1.5 | 5.01 |
| v4 | 空载被推 3 牛 | 0/3 | 3.54 秒 | 0.52 | 3.54 + 0.26 + 0 | 3.80 | ×1 | 3.80 |
| | | | | | | | **总分** | **31.83** |

| Controller | Condition | Falls | Settle time | Chatter reading | Penalty breakdown (settle + chatter + fall) | Condition penalty | × Weight | Counted in total |
|---|---|---|---|---|---|---|---|---|
| Factory Normal gains | Empty | 0/3 | 3.48 s | 0.52 | 3.48 + 0.26 + 0 | 3.74 | ×2 | 7.48 |
| Factory Normal gains | 1 kg | **2/3** (one run fell after completing 1 segment, one run fell after completing 2 segments) | 6.80 s | 2.07 | 2.27 + 0.34 + 10.00 | 12.61 | ×2 | 25.22 |
| Factory Normal gains | 2 kg | **3/3** (all fell in the first segment) | — | — | 0 + 0 + 30.00 | 30.00 | ×1 | 30.00 |
| Factory Normal gains | 4 kg | **3/3** (all fell in the first segment) | — | — | 0 + 0 + 30.00 | 30.00 | ×1.5 | 45.00 |
| Factory Normal gains | Empty, pushed 3 N | 0/3 | 3.50 s | 0.51 | 3.50 + 0.25 + 0 | 3.75 | ×1 | 3.75 |
| | | | | | | | **Total** | **111.45** |
| Factory Weight gains | Empty | 0/3 | **7 s (never settled)** | **33.70** | 7.00 + 16.85 + 0 | 23.85 | ×2 | 47.70 |
| Factory Weight gains | 1 kg | 0/3 | **7 s (never settled)** | **20.73** | 7.00 + 10.36 + 0 | 17.36 | ×2 | 34.72 |
| Factory Weight gains | 2 kg | 0/3 | 3.35 s | 0.78 | 3.35 + 0.39 + 0 | 3.74 | ×1 | 3.74 |
| Factory Weight gains | 4 kg | 0/3 | 3.29 s | 0.60 | 3.29 + 0.30 + 0 | 3.59 | ×1.5 | 5.39 |
| Factory Weight gains | Empty, pushed 3 N | 0/3 | **7 s (never settled)** | **33.70** | 7.00 + 16.85 + 0 | 23.85 | ×1 | 23.85 |
| | | | | | | | **Total** | **115.41** |
| v4 | Empty | 0/3 | 3.55 s | 0.52 | 3.55 + 0.26 + 0 | 3.81 | ×2 | 7.62 |
| v4 | 1 kg | 0/3 | 4.09 s | 3.68 | 4.09 + 1.84 + 0 | 5.93 | ×2 | 11.86 |
| v4 | 2 kg | 0/3 | 3.29 s | 0.50 | 3.29 + 0.25 + 0 | 3.54 | ×1 | 3.54 |
| v4 | 4 kg | 0/3 | 3.11 s | 0.46 | 3.11 + 0.23 + 0 | 3.34 | ×1.5 | 5.01 |
| v4 | Empty, pushed 3 N | 0/3 | 3.54 s | 0.52 | 3.54 + 0.26 + 0 | 3.80 | ×1 | 3.80 |
| | | | | | | | **Total** | **31.83** |

（各项按显示的两位小数四舍五入，相加可能差 0.01～0.02。明细日志：`source_snapshot/benchmark/score_release_v4_detail.log`，是专门为拆分数字重跑的一次，三方总分和第一次完全一致，说明结果可以复现。）

(Each item is rounded to the two decimal places shown, so sums may differ by 0.01–0.02. Detailed log: `source_snapshot/benchmark/score_release_v4_detail.log`, a re-run done specifically to break down the numbers; all three total scores matched the first run exactly, which shows the results are reproducible.)

**从明细能看出的含义：**

**What the breakdown shows:**

- **原厂正常档丢分全在「摔倒」**：带 2 千克、4 千克每局都在第一段就摔，这两项占了它总分的 75 分；1 千克三局摔了两局。空车时它其实很好：变稳 3.48 秒、抖振 0.52，是全场空载最好的。
- **原厂负重档丢分全在「抖」**：空车用这套增益会剧烈自激，抖振 33.7，车一直停不稳（变稳按 7 秒封顶）；1 千克也抖到 20.7。带 2 千克、4 千克时它很好。
- **v4 没有一局摔倒，也没有一项一直抖**：空载时它用的就是原厂正常档的增益，所以和原厂正常档几乎一样（3.81 对 3.74，多出的 0.07 是开机先在硬档上探测负载的代价）；带 2 千克、4 千克比原厂负重档还稳一点（带载档 f=0.40 比负重档软，抖振更小、停得更快）。
- **v4 最弱的一项是 1 千克**（5.93 分）：抖振 3.68，比其他工况大。原因是 1 千克落在两档的中间，被判成带载后用 f=0.40，对 1 千克略硬。但这已经比原厂两档在 1 千克时都好得多：原厂正常档三局摔两局，原厂负重档一直抖。

- **The factory Normal gains lose all their points to "falls"**: with 2 kg and 4 kg, every run falls in the first segment, and these two items account for 75 points of its total score; with 1 kg, two of the three runs fall. With an empty car it is actually very good: settle time 3.48 s and chatter 0.52, the best empty result of all.
- **The factory Weight gains lose all their points to "shaking"**: an empty car on this gain set goes into violent self-excited oscillation, with chatter 33.7, and the car never stops steadily (settle time capped at 7 s); with 1 kg it still shakes at 20.7. With 2 kg and 4 kg it is very good.
- **v4 has no run with a fall and no item with persistent shaking**: when empty it uses exactly the factory Normal gains, so it is almost the same as the factory Normal gains (3.81 versus 3.74; the extra 0.07 is the cost of first probing the load on the stiff gains at power-on); with 2 kg and 4 kg it is slightly steadier than the factory Weight gains (the loaded setting f=0.40 is softer than the Weight gains, so chatter is smaller and it stops faster).
- **v4's weakest item is 1 kg** (5.93 points): chatter 3.68, higher than in the other conditions. The reason is that 1 kg falls between the two gain sets; once it is judged as loaded, v4 uses f=0.40, which is slightly stiff for 1 kg. But this is already much better than either factory gain set at 1 kg: the factory Normal gains fall in two of three runs, and the factory Weight gains shake the whole time.

**总分怎么读：**

**How to read the total score:**

- **原厂正常档**：空载很好（3.74），但带 2 千克、4 千克三局全摔，每局扣满 30 分；1 千克也勉强。
- **原厂负重档**：带载很好，但空车在这套增益上剧烈高频抖动（抖振读数 33.7），空载和空载被推两项都扣到 23.85。
- **v4**：空载（3.81）几乎追平原厂正常档（3.74），带载三项都比原厂负重档还好，1 千克（两个原厂档都不合适的地方）从 12.61～17.36 降到 5.93。总分 31.83，大约是原厂任何一档的 **四分之一**。
- **跑分的就是车上那份代码**：v4 这一行用的是本版固件的 C 代码 `v4_core.c`，编译成电脑能调用的动态库，接进孪生里运行，不是 Python 模仿版。同时还跑了两个核对行：Python 版 v4，以及关掉 10-03 修改的 C 代码，三者分数逐位相同（都是 31.83）。
  - **Python 版和关掉修改的 C 代码相同**：说明 C 代码和孪生的接口接对了。
  - **本版和关掉修改的版本相同**：先核实了修改确实接上了（每局约 11 次按键变化信号确实送进了 C 代码，被推的那局正常升档 1 次）。在孪生里，刹车和起步时车身的摆动比真车小得多，原本就不会误升档，10-03 的修改没有东西可拦，所以分数不变。这也说明 10-03 的修改在仿真里没有副作用。它们的作用只有在真车上才看得到，见 13.3。

- **Factory Normal gains**: very good when empty (3.74), but with 2 kg and 4 kg all three runs fall, each costing the full 30 points; 1 kg is also marginal.
- **Factory Weight gains**: very good when loaded, but an empty car on this gain set shakes violently at high frequency (chatter reading 33.7), so the empty and empty-pushed items both cost 23.85.
- **v4**: empty (3.81) almost matches the factory Normal gains (3.74); all three loaded items are better than even the factory Weight gains; 1 kg (where neither factory gain set fits) drops from 12.61–17.36 to 5.93. The total score of 31.83 is about **one quarter** of either factory gain set.
- **The benchmark runs the same code that runs on the car**: the v4 row uses this release's firmware C code `v4_core.c`, compiled into a dynamic library the computer can call and plugged into the twin; it is not a Python imitation. Two check rows were run at the same time: the Python version of v4, and the C code with the 10-03 changes switched off. All three scores are identical digit for digit (all 31.83).
  - **The Python version equals the C code with the changes switched off**: this shows that the interface between the C code and the twin is wired correctly.
  - **This release equals the version with the changes switched off**: it was first verified that the changes really were connected (about 11 key-change signals per run really did reach the C code, and the pushed run upshifted normally once). In the twin, the body sway during braking and starting is much smaller than on the real car, so it never upshifts by mistake in the first place; the 10-03 changes have nothing to block, so the score does not change. This also shows that the 10-03 changes have no side effects in simulation. Their effect can only be seen on the real car; see 13.3.

**12 工况完成局数**（第 8.1 节同一张表的考题：12 种工况 × 4 个种子 = 48 局；三段全部完成才算一局完成，**越高越好**）：

**Completed runs across 12 conditions** (the same test set as the table in Section 8.1: 12 conditions × 4 seeds = 48 runs; a run counts as completed only if all three segments are completed; **higher is better**):

| 控制器 | 先站 3 秒再走（48 局） | 开局就走（48 局） | 8 台参数随机的车（96 局） | 齿轮刚度改为 22 赫兹（48 局） |
|---|---|---|---|---|
| 原厂正常档 | 16 | 18 | 34 | 16 |
| 原厂负重档 | 41 | 40 | 83 | 41 |
| **v4** | **44** | **44** | **88** | **44** |

| Controller | Stand 3 s, then drive (48 runs) | Drive from the start (48 runs) | 8 cars with randomized parameters (96 runs) | Gear stiffness changed to 22 Hz (48 runs) |
|---|---|---|---|---|
| Factory Normal gains | 16 | 18 | 34 | 16 |
| Factory Weight gains | 41 | 40 | 83 | 41 |
| **v4** | **44** | **44** | **88** | **44** |

（这张表是 10-02 版的结果，见第 8.1 节。没完成的局集中在「空载被推 8 牛」和「下台阶」，三方都摔，属于物理极限。）

(This table is from the 10-02 version; see Section 8.1. The uncompleted runs are concentrated in "empty, pushed 8 N" and "stepping down a ledge", where all three controllers fall; these are physical limits.)

### 13.2 仿真分项对比（10-02 版的结果，见第 8 节） / 13.2 Simulation comparison by item (results of the 10-02 version, see Section 8)

| 项目 | 规则 | 原厂正常档 | 原厂负重档 | v4 |
|---|---|---|---|---|
| 随机场地摔倒局 | 64 局，每局 24 条随机四键指令，中间随机插入推车、改负载、改坡度；越少越好 | 64/64（带载就摔） | 15/64 | 19/64 |
| 随机场地不收敛次数 | 停车后一直没有安静下来的次数；越少越好 | 5 | 126 | **8** |
| 随机场地持续抖振次数 | 停车时抖振一直很大的次数；越少越好 | 7 | 259 | **8** |
| 随机场地停车收敛（中位） | 停车后安静下来所用时间；越短越好 | 1.33 秒 | 1.04 秒 | **0.99 秒** |
| 推击挺过来次数（空载 5 牛 / 空载 6 牛 / 2 千克 12 牛 / 2 千克 15 牛，各 4 次） | 站稳后推 0.12 秒；越多越好 | 2 / 0 / 站不住 / 站不住 | 4 / 2 / 4 / 4 | 4 / 2 / 4 / 2 |
| 行驶抖振 / 停车收敛：空载 | 前进 1 米、停 4 秒、倒回、停 4 秒 | 3.0 / 1.19 秒 | （空车用负重档会剧烈抖） | 3.1 / 1.20 秒 |
| 行驶抖振 / 停车收敛：1 千克 | 同上 | 不适用（正常档带载会摔） | 11.2 / 始终不稳 | **3.2 / 0.69 秒** |
| 行驶抖振 / 停车收敛：2 千克 | 同上 | 不适用（正常档带载会摔） | 6.6 / 0.77 秒 | **3.2 / 0.60 秒** |
| 行驶抖振 / 停车收敛：4 千克 | 同上 | 不适用（正常档带载会摔） | 5.6 / 0.75 秒 | **3.7 / 0.66 秒** |

| Item | Rule | Factory Normal gains | Factory Weight gains | v4 |
|---|---|---|---|---|
| Runs with a fall on the random course | 64 runs, each with 24 random four-key commands, with pushes, load changes and slope changes inserted at random; fewer is better | 64/64 (falls whenever loaded) | 15/64 | 19/64 |
| Non-convergence count on the random course | Number of times the car never became calm after stopping; fewer is better | 5 | 126 | **8** |
| Persistent chatter count on the random course | Number of times the chatter stayed high while stopped; fewer is better | 7 | 259 | **8** |
| Stop convergence on the random course (median) | Time to become calm after stopping; shorter is better | 1.33 s | 1.04 s | **0.99 s** |
| Pushes survived (empty 5 N / empty 6 N / 2 kg 12 N / 2 kg 15 N, 4 times each) | Push for 0.12 s after standing steadily; more is better | 2 / 0 / cannot stand / cannot stand | 4 / 2 / 4 / 4 | 4 / 2 / 4 / 2 |
| Driving chatter / stop convergence: empty | Forward 1 m, stop 4 s, back, stop 4 s | 3.0 / 1.19 s | (an empty car on the Weight gains shakes violently) | 3.1 / 1.20 s |
| Driving chatter / stop convergence: 1 kg | Same as above | Not applicable (the Normal gains fall when loaded) | 11.2 / never steady | **3.2 / 0.69 s** |
| Driving chatter / stop convergence: 2 kg | Same as above | Not applicable (the Normal gains fall when loaded) | 6.6 / 0.77 s | **3.2 / 0.60 s** |
| Driving chatter / stop convergence: 4 kg | Same as above | Not applicable (the Normal gains fall when loaded) | 5.6 / 0.75 s | **3.7 / 0.66 s** |

### 13.3 实车对比（2026-10-03，本版） / 13.3 Real-car comparison (2026-10-03, this release)

| 项目 | 原厂正常档（模式 1） | 原厂负重档 | v4（本版） |
|---|---|---|---|
| 空载站立、正常开车时的增益 | 正常档 | 负重档（空车会持续高频抖） | 和原厂正常档完全相同（f=0） |
| 频繁前后开、没有外力时额外的换档抖动 | 无（不换档） | 无（不换档，但本身一直抖） | **0 次**（33 秒内 26 次按键）；改之前的 10-02 版是 2 分钟 9 次 |
| 被推一下 | 用正常档硬扛 | 用负重档硬扛 | 升到负重档抗冲击，0.5 秒后自动回到空载档；每次推车只升 1 次 |
| 带负载 | 要人手动选负重模式，忘了选就容易摔 | 合适 | 自动判断（实车带载本轮未重测） |

| Item | Factory Normal gains (mode 1) | Factory Weight gains | v4 (this release) |
|---|---|---|---|
| Gains when standing empty and driving normally | Normal gains | Weight gains (an empty car shakes continuously at high frequency) | Exactly the same as the factory Normal gains (f=0) |
| Extra gain-shift shaking when driving back and forth frequently with no external force | None (no gain shifting) | None (no gain shifting, but it shakes all the time anyway) | **0 times** (26 key presses in 33 seconds); the earlier 10-02 version had 9 times in 2 minutes |
| Being pushed once | Rides it out on the Normal gains | Rides it out on the Weight gains | Upshifts to the Weight gains to resist the shock, and automatically returns to the empty setting after 0.5 s; only 1 upshift per push |
| Carrying a load | A person must select the weight mode by hand; forgetting it easily leads to a fall | Suitable | Decides automatically (loaded tests on the real car were not repeated in this round) |

### 13.4 一句话结论 / 13.4 One-line conclusions

- **空载时**：v4 用的就是原厂正常档的增益，没有外力时表现和原厂模式 1 相同。被推时它会临时升到负重档抗冲击，0.5 秒后回来，这是原厂正常档做不到的。
- **带负载时**：原厂要人提前选对负重模式；v4 自动判断，而且在仿真里带载的每一项指标都好于原厂负重档：抖振更小，收敛更快，1 千克时原厂两档都不合适，v4 能稳。
- **跑分**：v4 总分 31.83，原厂正常档 111.45，原厂负重档 115.41（越低越好）。v4 约为原厂任何一档的四分之一，差距来自原厂每一档都有它不适合的负载，而 v4 自己换档。
- **局限**：仿真和真车有差距（例如真车的抖振比仿真大约 30 倍），所以仿真分数只能说明设计方向，最终以 13.3 的实车结果为准。

- **When empty**: v4 uses exactly the factory Normal gains, so with no external force it behaves the same as factory mode 1. When pushed, it temporarily upshifts to the Weight gains to resist the shock and comes back after 0.5 s, which the factory Normal gains cannot do.
- **When loaded**: the factory firmware requires a person to choose the weight mode correctly in advance; v4 decides automatically, and in simulation every loaded metric is better than the factory Weight gains: smaller chatter, faster convergence, and at 1 kg, where neither factory gain set fits, v4 stays steady.
- **Benchmark score**: v4 total 31.83, factory Normal gains 111.45, factory Weight gains 115.41 (lower is better). v4 is about one quarter of either factory gain set; the gap comes from each factory gain set having loads it does not suit, while v4 shifts gains by itself.
- **Limitations**: there is a gap between simulation and the real car (for example, chatter on the real car is about 30 times larger than in simulation), so the simulation scores only indicate the design direction; the real-car results in 13.3 are what finally count.

### 13.5 公式汇总 / 13.5 Formula summary

#### 13.5.1 跑分公式（13.1 节的总分） / 13.5.1 Benchmark formula (the total score of Section 13.1)

记号：
- 控制器 c；工况 k（5 个）；随机种子 s ∈ {0, 1, 2}
- 一局 = 「停-走-停」任务，3 段，第 j 段的目标点依次为 0.9 米、0.3 米、1.2 米

Notation:
- Controller c; condition k (5 of them); random seed s ∈ {0, 1, 2}
- One run = the "stop-go-stop" task, 3 segments; the target point of segment j is 0.9 m, 0.3 m, 1.2 m in turn

**第一步：每一段的变稳时间 T_j。**

**Step 1: the settle time T_j of each segment.**

    T_j = 从第 j 段开始（给出新目标点）的时刻，
          到「已到达目标（离目标 < 5 厘米）之后，第一次满足
                |θ − θ_g| < 1°  且  |ω| < 20°/s ，并连续保持 0.25 秒」
          的那 0.25 秒的起点，所经过的秒数

    θ   = 车身倾角（度）
    θ_g = 重力方向对应的平衡倾角（平地为 0）
    ω   = 车身俯仰角速度（度/秒）

    这一段始终不满足条件，则 T_j 记为「无」。

*English:*

    T_j = the number of seconds elapsed from the moment segment j starts (a new target point is given)
          to the start of the first 0.25 s window that, "after the target has been reached (distance to target < 5 cm), satisfies
                |θ − θ_g| < 1°  and  |ω| < 20°/s , held continuously for 0.25 s"

    θ   = body tilt angle (degrees)
    θ_g = balance tilt angle corresponding to the direction of gravity (0 on flat ground)
    ω   = body pitch angular rate (degrees/second)

    If the condition is never satisfied in this segment, T_j is recorded as "none".

**第二步：每一局的变稳时间 T。**

**Step 2: the settle time T of each run.**

    T = 三段中「不是无」的 T_j 的平均值；三段都是无时，T = 7 秒

*English:*

    T = the average of the T_j values that are "not none" among the three segments; if all three are none, T = 7 seconds

**第三步：每一局的抖振读数 O。**

**Step 3: the chatter reading O of each run.**

    O = 三段的 O_j 的平均值

    O_j 的算法：取第 j 段到达后停住的 5 秒里的陀螺读数 g（度/秒），
    依次做 8 赫兹高通和 16 赫兹低通（与固件 load_adapt.c 同一组系数）：
        h ← h + 0.200849 × (g − h)
        b ← b + 0.334511 × ((g − h) − b)
    再取 b 的均方根：
        O_j = √( b 在这 5 秒里的平方平均 )

*English:*

    O = the average of O_j over the three segments

    How O_j is computed: take the gyroscope readings g (degrees/second) during the 5 seconds the car stays stopped after reaching segment j's target,
    and apply an 8 Hz high-pass and then a 16 Hz low-pass filter (the same coefficients as in the firmware load_adapt.c):
        h ← h + 0.200849 × (g − h)
        b ← b + 0.334511 × ((g − h) − b)
    then take the root mean square of b:
        O_j = √( mean of b squared over these 5 seconds )

**第四步：每一局的扣分 P(c, k, s)。**

**Step 4: the penalty P(c, k, s) of each run.**

    三段全部完成：   P = T + 0.5 × O
    中途摔倒：       P = 30 × (1 − 已完成段数 / 3)

*English:*

    All three segments completed:   P = T + 0.5 × O
    Fall partway through:           P = 30 × (1 − completed segments / 3)

**第五步：每个工况的扣分（3 个种子平均）。**

**Step 5: the penalty of each condition (average over 3 seeds).**

    P̄(c, k) = [ P(c,k,0) + P(c,k,1) + P(c,k,2) ] / 3

**第六步：总分（越低越好）。**

**Step 6: the total score (lower is better).**

    J(c) = Σ_k  w_k × P̄(c, k)

    w_空载 = 2，w_1千克 = 2，w_2千克 = 1，w_4千克 = 1.5，w_空载被推3牛 = 1
    （权重之和 7.5）

*English:*

    J(c) = Σ_k  w_k × P̄(c, k)

    w_empty = 2, w_1kg = 2, w_2kg = 1, w_4kg = 1.5, w_empty_pushed_3N = 1
    (the weights sum to 7.5)

**代入示例**：v4 的总分

**Worked example**: the total score of v4

    2 × 3.81 + 2 × 5.93 + 1 × 3.54 + 1.5 × 3.34 + 1 × 3.80 = 31.83

**理论下限**：开车本身约需 3.3 秒，抖振不可能为 0，所以每个工况最低约 3.3 分，总分下限约为 7.5 × 3.3 ≈ 25 分。

**Theoretical lower bound**: the driving itself takes about 3.3 seconds and chatter cannot be 0, so each condition scores at least about 3.3 points, and the lower bound of the total score is about 7.5 × 3.3 ≈ 25 points.

#### 13.5.2 交叉熵搜索（CEM）用的目标公式 / 13.5.2 The objective formula used by the cross-entropy method (CEM) search

CEM 搜参数时用的就是上面同一个公式（`cem_v3.py`），只是把被评价的对象从「控制器 c」换成了「一组待定参数 θ」：

When CEM searches for parameters it uses exactly the same formula as above (`cem_v3.py`); the only difference is that the thing being evaluated changes from "controller c" to "a set of candidate parameters θ":

    J(θ) = Σ_k  w_k × (1/3) × Σ_{s=0,1,2}  P(θ, k, s)

    θ   = 6 个切换器参数：探测时长、空载读数界、满载读数界、跳档比例、逃生速率、逃生门限
    P   = 13.5.1 第四步的单局扣分
    w_k = 13.5.1 第六步的同一组权重

*English:*

    J(θ) = Σ_k  w_k × (1/3) × Σ_{s=0,1,2}  P(θ, k, s)

    θ   = the 6 switcher parameters: probing duration, empty reading bound, full-load reading bound, gain-jump ratio, escape rate, escape threshold
    P   = the single-run penalty from Step 4 of 13.5.1
    w_k = the same set of weights as in Step 6 of 13.5.1

CEM 是**最小化** J(θ)。如果写成强化学习里习惯的「奖励」形式，就是

CEM **minimizes** J(θ). Written in the "reward" form customary in reinforcement learning, this is

    奖励 R(θ) = −J(θ)

*English:*

    Reward R(θ) = −J(θ)

即：变稳每慢 1 秒奖励少 1；抖振读数每大 1，奖励少 0.5；第一段就摔，奖励少 30。

That is: each extra second of settle time lowers the reward by 1; each extra unit of chatter reading lowers the reward by 0.5; a fall in the first segment lowers the reward by 30.

**CEM 的迭代过程**：

**The CEM iteration**:

    1. 采样：从正态分布 N(μ, σ²) 里随机抽 N = 20 组参数 θ_1 … θ_20，
             每个参数截断在各自的搜索范围内；
    2. 评分：每组参数跑 5 个工况 × 3 个种子 = 15 局，算出 J(θ_i)；
    3. 选精英：取 J 最小的 E = 5 组；
    4. 更新：μ ← 精英的平均值，σ ← 精英的标准差；
    5. 停止：连续 3 代最好的 J 都没有改善就停；
             最终参数取精英的平均值 μ（不取单个最好的那组，因为单组有运气成分）。

*English:*

    1. Sample: draw N = 20 parameter sets θ_1 … θ_20 at random from the normal distribution N(μ, σ²),
             with each parameter clipped to its own search range;
    2. Score: run each parameter set on 5 conditions × 3 seeds = 15 runs and compute J(θ_i);
    3. Select elites: take the E = 5 sets with the smallest J;
    4. Update: μ ← mean of the elites, σ ← standard deviation of the elites;
    5. Stop: stop when the best J has not improved for 3 consecutive generations;
             the final parameters are the elites' mean μ (not the single best set, because a single set involves luck).
