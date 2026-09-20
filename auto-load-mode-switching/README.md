# 自适应负载 PID 调度（模式 26） / Adaptive Load PID Scheduling (Mode 26)

> 平衡车在不知道自己装了多重的情况下，自动在五套 PID 增益之间切换。
> A balancing robot that switches between five PID gain sets without ever being told how much it is carrying.

**目标平台 / Target:** Yahboom STM32F103 平衡车，200 Hz 控制环
**状态 / Status:** 实车验证通过（空车 ↔ 约 0.5 kg 负载双向切换）
**日期 / Date:** 2026-09-20

---

## 目录 / Contents

1. [问题 / The Problem](#1-问题--the-problem)
2. [实测车辆参数 / Measured Car Parameters](#2-实测车辆参数--measured-car-parameters)
3. [为什么需要切换增益 / Why Gains Must Change](#3-为什么需要切换增益--why-gains-must-change)
4. [核心难点 / The Core Difficulty](#4-核心难点--the-core-difficulty)
5. [信号设计 / Signal Design](#5-信号设计--signal-design)
6. [增益梯子 / The Gain Ladder](#6-增益梯子--the-gain-ladder)
7. [决策逻辑 / Decision Logic](#7-决策逻辑--decision-logic)
8. [无扰切换 / Bumpless Transfer](#8-无扰切换--bumpless-transfer)
9. [控制权限 / Control Authority](#9-控制权限--control-authority)
10. [固件接入 / Firmware Integration](#10-固件接入--firmware-integration)
11. [串口控制台 / Serial Console](#11-串口控制台--serial-console)
12. [实测结果 / Measured Results](#12-实测结果--measured-results)
13. [失败过的方案 / What Failed and Why](#13-失败过的方案--what-failed-and-why)
14. [已知限制 / Known Limits](#14-已知限制--known-limits)
15. [文件清单 / File Manifest](#15-文件清单--file-manifest)

---

## 1. 问题 / The Problem

**中文**

平衡车装上负载后，质心被抬高、转动惯量增大，原来的 PID 增益不再够用，车会摔。
但如果一开始就用适合重载的大增益，空车时又会剧烈振荡 —— 因为增益相对于空车的
需求过剩了。

一套固定的增益无法同时覆盖 0 kg 和 4 kg。厂家的做法是让用户手动选"普通模式"
还是"负重模式"。本算法让车自己判断。

**English**

Loading the car raises its centre of mass and increases its pitch inertia. Gains
that held the empty chassis no longer hold, and it falls. Use gains sized for the
load instead and the empty car shakes violently, because those gains are now far
larger than it needs.

One fixed gain set cannot cover 0 kg and 4 kg. The stock firmware asks the user to
pick "normal" or "load-bearing" by hand. This algorithm lets the car decide.

---

## 2. 实测车辆参数 / Measured Car Parameters

所有数值来自实车测量，不是仿真估计。
All values measured on the real car, not estimated from simulation.

| 参数 / Parameter | 值 / Value | 来源 / Source |
|---|---|---|
| 整车质量 / Total mass | 0.942 kg（车身 0.872 + 轮 2×0.035）| 官方整备质量 |
| 轮半径 / Wheel radius | 0.0335 m（周长 210.49 mm）| 规格 |
| 轮距 / Track | 0.1670 m | 规格 |
| 车身质心（轮轴上方）/ Body COM above axle | **0.0340 m** | 2026-09-13 悬挂法实测 |
| 车身外形 / Body box | 84 × 194 × 106.1 mm | 实测 |
| 额定载重 / Rated payload | 4 kg，位于轮轴上方 0.105 m | 规格 |
| 编码器 / Encoder | 1320 计数每轮圈（11 线 × 4 倍频 × 30 减速比）| 固件常数 |
| 陀螺量程 / Gyro range | ±2000 dps，16.4 LSB per °/s | 固件配置 |
| 控制周期 / Control period | 5 ms（200 Hz），由 MPU6050 中断驱动 | 固件 |
| **电机死区 / Motor dead band** | **正向 1480，反向 1455 计数** | 开环扫描 + 回放拟合 |
| PWM 周期 / PWM period | 2880（25 kHz），限幅 ±2800 | 固件 |

由此算出的动力学量 / Derived dynamics:

| 量 / Quantity | 空车 / Empty | 0.5 kg | 2 kg | 4 kg |
|---|---|---|---|---|
| 俯仰惯量 C / Pitch inertia (kg·m²) | 0.00234 | 0.00786 | 0.02439 | 0.04644 |
| 恢复力矩需求 mgl / Restoring torque (N·m/rad) | 0.291 | 0.806 | 2.351 | **4.411** |
| 不稳定极点 / Unstable pole (rad/s) | 11.15 | ~10 | 9.82 | 9.75 |
| 自由倒下时间常数 / Fall time constant | **90 ms** | ~100 ms | 102 ms | 103 ms |

**关键观察 / Key observation：载重主要不是增加质量，而是抬高质心。**
4 kg 让质量变 5.6 倍，但质心从 34 mm 抬到 92 mm（2.7 倍），两者相乘使所需恢复
力矩变成 **15 倍**，而电机力矩一点没变。

A payload does not mainly add mass, it raises the centre of mass. At 4 kg the mass
is 5.6x but the COM moves from 34 mm to 92 mm, and the product means **15x** more
restoring torque is needed from a motor that has not changed at all.

---

## 3. 为什么需要切换增益 / Why Gains Must Change

线性化的俯仰动力学 / Linearised pitch dynamics:

```
C · θ̈ = m·g·l · θ − 2τ
```

控制器提供 `τ ∝ Kp · θ`，所以稳定的必要条件是
The controller supplies `τ ∝ Kp·θ`, so stability requires

```
2·k_motor·Kp  >  m·g·l
```

**最小 Kp 正比于 m·g·l。** 代入实测值：
**Minimum Kp scales exactly with m·g·l.** With the measured numbers:

| 载重 / Load | m·g·l (N·m/rad) | 原厂 Kp=9600 提供 | 裕度 / Margin |
|---|---|---|---|
| 0 kg | 0.291 | 4.463 | **15.3×** |
| 1 kg | 1.321 | 4.463 | 3.4× |
| 2 kg | 2.351 | 4.463 | 1.9× |
| 4 kg | 4.411 | 4.463 | **1.01×** ← 临界 / on the edge |

4 kg 时原厂增益只剩 1% 裕度 —— 理论上刚好不倒，实际必摔。这就是必须升档的
物理依据，不是经验拍脑袋。

At 4 kg the stock gains have 1 percent of margin left. That is the physical reason
the ladder must go up, not a rule of thumb.

---

## 4. 核心难点 / The Core Difficulty

**中文**

最初的设计思路是"看症状"：车抖了说明增益过大，车倒了说明增益不足。
这个思路在实车上反复失败，原因是：

> **增益过大和增益不足会产生大量相同的外在症状。**

两者都会造成：大幅低频摆动、倾角超限、PWM 饱和。实测数据：

- 空车 + 增益过大（2 档）：0.3–2 Hz 摆幅冲到 **4.98°**
- 这个值远超"增益不足"的判定阈值 2.5°

所以任何只看幅度的判据都会把两种相反的故障混为一谈，导致升档和降档互相打架。

**唯一可靠的区分特征是高频抖动**：

```
增益过大  →  慢摆  +  8–16 Hz 抖动   （osc 高）
增益不足  →  慢摆  +  安静           （osc 低）
```

**English**

The first design read symptoms: shaking means too much gain, falling means too
little. It failed repeatedly on the car, because

> **over-gaining and under-gaining share most of their symptoms.**

Both produce large low-frequency swings, excursions past the angle limit, and PWM
saturation. Measured: an empty car at level 2 (over-gained) reached **4.98 deg** of
0.3–2 Hz swing, far above the 2.5 deg threshold meant to detect under-gaining.

Any rule based on amplitude alone therefore confuses two opposite faults, and the
up-shift and down-shift rules fight each other.

**The one reliable discriminator is high-frequency chatter:**

```
too much gain  ->  slow swing  +  8-16 Hz chatter   (osc high)
too little     ->  slow swing  +  quiet             (osc low)
```

**这条规则贯穿整个算法：所有升档路径都必须带"osc 低"这个前提。**
**This rule runs through the whole algorithm: every up-shift path is gated on osc being low.**

---

## 5. 信号设计 / Signal Design

### 5.1 传感器来源 / Sensor Sources

MPU6050 提供陀螺仪和加速度计。固件用卡尔曼滤波融合：
The MPU6050 supplies gyro and accelerometer; the firmware fuses them with a Kalman
filter:

```c
Gyro_Balance  = -Gyro_X;                          // 原始 LSB，无滤波 / raw LSB, unfiltered
Angle_Balance = KF_X(accel_y, accel_z, -gyro_x);  // 融合角度，度 / fused angle, deg
```

两个判据**故意使用不同的源**：
The two detectors deliberately use different sources:

| 判据 / Detector | 用哪个 / Source | 为什么 / Why |
|---|---|---|
| `la_osc`（抖动）| **原始陀螺** / raw gyro | 抖动是高频现象。原始陀螺没有滤波延迟；`Angle_Balance` 过了卡尔曼，高频已被衰减 / Chatter is high frequency. The raw gyro has no filter lag; the fused angle has already had its high frequencies attenuated |
| `la_lean`（摆动）| **融合角度** / fused angle | 倾倒是准静态现象。这个频段的信息主要来自加速度计 —— 陀螺积分会漂，只有加速度计提供绝对重力参考 / Leaning is quasi-static. At those frequencies the information comes from the accelerometer; integrating the gyro drifts |

### 5.2 滤波器实现 / Filter Implementation

所有滤波器都是一阶 IIR，形式 `y += a·(x − y)`。
All filters are one-pole IIRs of the form `y += a*(x - y)`.

拐点频率与系数的换算（fs = 200 Hz）：
Corner frequency to coefficient (fs = 200 Hz):

```
r = 2π·fc / fs        a = r / (1 + r)
```

| 用途 / Purpose | 频段 / Band | 系数 / Coefficients |
|---|---|---|
| 抖动带通 / chatter band-pass | 8 – 16 Hz | `LA_A_HP = 0.200849`, `LA_A_LP = 0.334511` |
| 摆动带通 / lean band-pass | 0.3 – 2 Hz | `LA_A_LEAN_HP = 0.009337`, `LA_A_LEAN_LP = 0.059120` |
| 能量平滑 / energy smoothing | ≈ 0.3 s | `LA_A_EN = 0.016393` |

带通实现 = 高通（减去慢速均值）后接低通：
A band-pass is a high-pass (subtract the slow mean) followed by a low-pass:

```c
mean += A_HP * (x - mean);          // 慢速均值 / slow mean
hp    = x - mean;                   // 高通 / high-passed
band += A_LP * (hp - band);         // 带通输出 / band-passed
energy += A_EN * (band*band - energy);
rms = sqrt(energy);
```

### 5.3 为什么是 8–16 Hz / Why 8-16 Hz

这个频段**不是猜的，是测出来的**。
This band was **measured, not guessed.**

实车扫频（0.5–25 Hz，两个幅值）得到的频率响应：
Swept-sine response measured on the car (0.5–25 Hz, two amplitudes):

| 频率 / Hz | 增益@300 | 增益@500 | 比值 / Ratio |
|---|---|---|---|
| 2.5 | 266 | 266 | 1.00 ← 线性区 / linear |
| 3.0 | 273 | 273 | 1.00 |
| **12.0** | **442** | 280 | 0.63 ← 共振峰 / resonance |
| 18.0 | 71 | 198 | **2.80** ← 间隙 / backlash |
| 22.0 | 25 | 74 | **2.99** |

（增益单位：°/s per 1000 PWM）

然后把 `Balance_Kd` 从 48 提到 120，测空车的自激抖动：
Then Balance_Kd was raised from 48 to 120 and the resulting self-excited chatter
measured on the empty car:

```
主频 / dominant frequency: 13.96 Hz     三次谐波 / third harmonic: 41.6 Hz
```

| 频段 / Band | Kd=48 | Kd=120 | 分离 / Separation |
|---|---|---|---|
| 0.4–3 Hz | 1.53 | 10.01 | 6.5× |
| 3–8 Hz | 6.40 | 8.70 | 1.4× |
| **8–16 Hz** | **4.55** | **94.93** | **20.9×** |
| 16–25 Hz | 2.59 | 10.75 | 4.2× |
| 26.5–45.5 Hz | 1.21 | 19.10 | 15.8× |

**8–16 Hz 给出全场最强的 20.9 倍分离度**，因为它正好罩住 13.96 Hz 这个真实的
极限环频率。

**The 8-16 Hz band gives the strongest separation of any band, 20.9x**, because it
sits on the 13.96 Hz limit cycle the car actually produces.

> **历史注记 / Historical note：** 仿真孪生曾预测抖动在 31 Hz，早期版本据此使用
> 26.5–45.5 Hz 带通。实车主频是 13.96 Hz —— 那个 26.5–45.5 Hz 带通抓到的其实
> 是它的**三次谐波**（41.6 Hz），一个又弱又间接的信号。
>
> The simulation twin predicted 31 Hz and an early version used a 26.5-45.5 Hz
> band accordingly. The real fundamental is 13.96 Hz; that band was picking up its
> **third harmonic**, a weak and indirect signal.

---

## 6. 增益梯子 / The Gain Ladder

五档，两端分别是原厂 Normal 和原厂 Weight_M（后者的负载系数已乘入）：
Five levels, spanning stock Normal to stock Weight_M with its load factors already
multiplied in:

| 档 / Level | Balance_Kp | Balance_Kd | Velocity_Kp | Velocity_Ki | Turn_Kp |
|---|---|---|---|---|---|
| 0 | 9600 | 48 | 6200 | 31 | 1700 |
| 1 | 12000 | 73.5 | 7012 | 35 | 1625 |
| **2**（开机 / boot）| 14400 | 99 | 7825 | 39 | 1550 |
| 3 | 16800 | 124.5 | 8637 | 43 | 1475 |
| 4 | 19200 | 150 | 9450 | 47 | 1400 |

> ⚠️ **重复相乘的陷阱 / Double-multiplication trap**
>
> `pid_control.c` 里 `Balance_K = 2.0` / `Velocity_K = 1.35` 是在 PID 函数**内部**
> 乘上去的，但**只在 `mode == Weight_M` 时生效**。模式 26 不是 Weight_M，所以这些
> 系数天然不生效，上表的数值原样使用。**绝不要把本调度器合并进 Weight_M**，否则
> 会被乘两次。这个坑项目里已经踩过两次。
>
> `pid_control.c` multiplies `Balance_K = 2.0` and `Velocity_K = 1.35` inside the
> PID functions, but only `if (mode == Weight_M)`. Mode 26 is not Weight_M, so they
> do not apply and the table above is used verbatim. **Never fold this scheduler
> into Weight_M** or everything gets multiplied twice. The project has hit this
> twice already.

### 顶档 19200 的依据 / Where 19200 comes from

不是从仿真抄的，是从这台车的物理算出来的：
Not copied from simulation — computed from this car:

```
4 kg 时 m·g·l = 4.411 N·m/rad
要 2 倍裕度需要 Kp ≈ 18978
梯子顶档设 19200
```

### 开机档位为什么是 2 / Why boot at level 2

原方案开机走顶档（4 档），理由是"增益不足会摔，增益过大只是抖"。
**但那个理由成立的前提是只能往下走。**

实测升档响应约 1 秒（负载 t=30.0 s 放上，档位 t=31.04 s 变化），顶档就不值那个
代价了 —— 4 档空车非常暴力，要花几秒才能走出来。2 档实测能扛住测试负载且 **0%
饱和**，空车下也温和得多。

The original scheme booted at the top because under-gaining drops a load while
over-gaining only shakes. **That argument held only while coming down was the
sole option.** With the up-shift measured at about 1 s, the top is no longer worth
its cost: level 4 on an empty car is violent and takes seconds to escape. Level 2
held the test load at **0 percent saturation** and is far gentler when empty.

---

## 7. 决策逻辑 / Decision Logic

三条路径，优先级从高到低。**每一条都基于测量，没有定时器。**
Three paths, highest priority first. **Every one is measurement-driven; there are
no timers.**

```
┌─ 1. 应急升档 / Express lane up ──────────────────────────────┐
│  条件: |0.3-2Hz 倾角| > 4°   AND   osc < 25                  │
│  动作: 立刻跳到顶档，禁止降档 3 秒                            │
│  绕过: 判定时间、锁定时间                                     │
│  用途: 负载突然加上、车即将失去平衡                           │
└──────────────────────────────────────────────────────────────┘
          ↓ 不满足 / not met
┌─ 2. 抖动降档 / Chatter down-shift ───────────────────────────┐
│  条件: osc > 25，持续 1 秒                                    │
│  动作: 降一档                                                 │
│  加速: osc > 50 时降两档，判定缩到 0.3 秒                     │
│  用途: 增益明显过大，尽快离开                                 │
└──────────────────────────────────────────────────────────────┘
          ↓ 不满足 / not met
┌─ 3. 常规升档 / Normal up-shift ──────────────────────────────┐
│  条件: 0.3-2Hz 倾角 RMS > 1.0°   AND   osc < 10              │
│  动作: 持续 0.6 秒后升一档                                    │
│  用途: 增益不足但还没到危险，慢慢补                           │
└──────────────────────────────────────────────────────────────┘
```

**不对称设计 / Deliberate asymmetry**

| 方向 / Direction | 速度 / Speed | 理由 / Reason |
|---|---|---|
| 升档 / up | 快（0–0.6 s）| 安全方向。慢 = 摔车 / The safety direction. Slow means falling |
| 降档 / down | 中（0.3–1 s）| 抖动难受但不危险 / Shaking is unpleasant, not dangerous |

换档后锁定 1–2 秒，期间不再判定，让闭环稳定下来。
After any shift there is a 1–2 s lock during which no decision is made, letting the
loop settle.

---

## 8. 无扰切换 / Bumpless Transfer

**中文**

`Velocity_PI()` 的输出包含一个积分项：

```c
velocity = -Encoder_bias*Velocity_Kp/100 - Encoder_Integral*Velocity_Ki/100;
```

换档时 `Velocity_Ki` 从 31 跳到 47，而积分值不变。积分限幅是 ±8000，所以最坏
情况下输出会**瞬间跳变 1280 PWM** —— 接近全部可用余量。

**换档这个动作本身变成了一次巨大的阶跃扰动**，而它恰好发生在车已经处于边缘的
时刻。这能解释"有时候切换很顺、有时候直接摔"。

解法是标准的无扰切换：换档时按比例缩放积分，让积分项的乘积保持连续。

**English**

`Velocity_PI()` output contains an integral term. Changing `Velocity_Ki` from 31 to
47 without touching the integral steps the motor command by up to **1280 PWM** —
nearly the entire control budget — at the exact moment the car is already in
trouble.

The fix is textbook bumpless transfer: rescale the integral so the product stays
continuous.

```c
if (la_applied >= 0 && i != la_applied && LA_VKI[i] > 0.0f)
    Encoder_Integral *= LA_VKI[la_applied] / LA_VKI[i];
```

这需要把 `Encoder_Integral` 从 `Velocity_PI()` 的函数内 static 提升到文件作用域。
This requires lifting `Encoder_Integral` out of `Velocity_PI()` to file scope.

---

## 9. 控制权限 / Control Authority

**中文**

这台车的死区补偿吃掉了大部分 PWM 量程：

```
PWM 限幅        ±2800
死区补偿         1480 / 1455
控制器可用余量   约 1320
```

载重测试中发现，3 档的 `Kd = 124.5` 意味着**每 1°/s 角速度产生 20.4 PWM**。实测
陀螺 RMS 是 40–60 °/s，**D 项单独就要 816–1224**，把余量全部吃光 —— 车一直在
限幅里打转，这才是它抖得停不下来的原因，不是倾角太大。

两项改动把权限拿回来：

1. **PWM 限幅 2600 → 2800**（周期 2880，97% 占空比，H 桥没问题）
2. **死区补偿改成非对称的实测值** —— 电机正反向死区是 1480 / 1454.5，之前两边
   都垫 1500，反向多垫了 45 计数，全是纯粹的零点跳变

效果（同一负载）：

| 指标 / Metric | 改动前 / Before | 改动后 / After |
|---|---|---|
| 饱和率 / Saturation | 7–80 % | **0 %** |
| 倾角标准差 / Angle std | 0.6–4.9° | **0.54–0.94°** |
| osc | 20–62 | **0.78–1.20** |
| 停留档位 / Level | 3 | **2** |

**English**

Dead band compensation consumes most of the PWM range, leaving about 1320 counts
for control. At level 3 the D term alone (20.4 PWM per deg/s against a measured
40–60 deg/s) demands 816–1224 counts, so the loop lives in saturation. Raising the
limit to 2800 and using the motor's real asymmetric dead band eliminated clipping
entirely under the same load.

---

## 10. 固件接入 / Firmware Integration

### 10.1 文件 / Files

将 `src/load_adapt.c` 和 `src/load_adapt.h` 放入 `APP/PID/`，并在 Keil 工程的
`APP` 组中加入 `load_adapt.c`。
Place `src/load_adapt.c` and `src/load_adapt.h` in `APP/PID/` and add
`load_adapt.c` to the `APP` group of the Keil project.

### 10.2 模式枚举 / Mode Enum

`USER/myenum.h`，加在 `Mode_Max` 之前：

```c
    Load_Adapt,   // adaptive load estimate + gain schedule, mode 26
    Mode_Max
```

### 10.3 三个调用点 / Three Call Sites

**① 模式选择时初始化 / Init at mode select** — `APP/mode/app_mode.c` 的 `Set_PID()`:

```c
else if(mode == Load_Adapt)
{
    LA_Reset();
}
```

**② 控制环每拍 / Every control tick** — `APP/app_control.c` 的
`EXTI15_10_IRQHandler()`，位置在 `Motor_Left/Right` 限幅之后、`Set_Pwm()` 之后:

```c
motors_live = (Turn_Off(Angle_Balance,battery)==0);
if(motors_live)
    Set_Pwm(Motor_Left,Motor_Right);

if(mode == Load_Adapt)
    LA_Tick(motors_live ? Motor_Left  : 0,
            motors_live ? Motor_Right : 0,
            Encoder_Left, Encoder_Right);
```

> 传入的是**实际写进比较寄存器的值** —— `Turn_Off()` 切断输出时传 0，避免把
> "没输出"误当成力矩。
> The values passed are what actually reached the timer: 0 when `Turn_Off()` cut
> the output, so an inactive motor is not mistaken for torque.

**③ 外设初始化 / Peripheral init** — `BSP/bsp.c` 的 `bsp_mode_init()`，把
`Load_Adapt` 加入初始化蓝牙和超声波的分支（遥控是调试必需的）。

### 10.4 需要的其他改动 / Other Required Changes

| 文件 / File | 改动 / Change |
|---|---|
| `APP/PID/pid_control.c` | `Encoder_Integral` 从函数内 static 提到文件作用域 |
| `APP/app_control.c` | PWM 限幅 `2600` → `2800`（两处）|
| `APP/app_motor.c` | 死区补偿改为 `motor_ignore_pulse = 1480` / `motor_ignore_rev = 1455`，`PWM_Ignore()` 按方向选用 |
| `USER/AllHeader.h` | `#include "load_adapt.h"` |
| `APP/OLED_Show/oled_show.c` | 加 `case Load_Adapt:` 显示模式名 |

### 10.5 开销 / Cost

每拍约 30 次浮点乘加 + 2 次 `sqrt`。F103 无硬件浮点，软件浮点下约 8–12 μs，
200 Hz 下占 CPU 约 0.2%。RAM 约 60 字节。

About 30 float multiply-adds plus two `sqrt` per tick: roughly 8–12 us in software
floating point, about 0.2 percent of the CPU at 200 Hz, and some 60 bytes of RAM.

---

## 11. 串口控制台 / Serial Console

**USART1，230400 8N1**（见 `src/tune_io.c`）。命令加不加回车都能识别（50 ms 空闲
自动执行）。

`USART1 at 230400 8N1`. Commands are accepted on CR/LF or after 50 ms of silence.

| 命令 / Command | 作用 / Action |
|---|---|
| `?` | 打印完整状态 / print full state |
| `n <档>` | 强制锁定档位，`-1` = 自动 / force level, -1 = auto |
| `o <值>` | 降档阈值 `LA_OSC_HI` |
| `w <值>` | 升档阈值 `LA_LEAN_HI` |
| `f <值>` | 升档的 osc 门限 `LA_OSC_LO` |
| `g <值>` | 应急阈值 `LA_ANG_PANIC` |
| `h <档>` | 开机档位 `LA_BOOT_LEVEL` |
| `d <值>` | 死区补偿（自动保持 25 计数的正反向差）|
| `r` / `s` | 开始 / 停止记录 |
| `z <n>` | 每 n 拍记一行（抽样，降低丢包）|

状态行示例 / Example state line:

```
#state mode=26 lvl=0/-1 osc=4.34 lean=0.17 sat=0.00 k=14.47
       oHi=25.00 lHi=1.00 oLo=10.00 pan=4.00 npan=4 boot=2
       dead=1480/1455 vbat=12.1
```

CSV 列 / CSV columns:

```
seq, gyro, ang_c, ml, mr, el, er, inj, osc_c, k_c, lvl
```

- `seq` 样本序号，不连续即丢包 / sample counter; a gap means dropped rows
- `gyro` 原始 LSB（÷16.4 = °/s）
- `ang_c` 倾角 ×100（厘度）
- `ml` `mr` 实际 PWM
- `el` `er` 编码器增量
- `osc_c` 抖动 RMS ×100
- `k_c` 载重估计 ×100（仅记录，不参与决策 / logged only, does not decide）
- `lvl` 当前档位 / current level

---

## 12. 实测结果 / Measured Results

### 12.1 静止站立 / Standing Still

| 指标 / Metric | 空车 0 档 / Empty L0 | 空车 Kd=120 / Empty Kd=120 |
|---|---|---|
| 倾角标准差 / Angle std | **0.25°** | 3.95° |
| 陀螺 RMS / Gyro RMS | 8.8 °/s | 99.2 °/s |
| PWM 饱和 / Saturation | 0 % | 65.9 % |
| 主频 / Dominant freq | 4.29 Hz | 13.96 Hz |

### 12.2 双向切换 / Bidirectional Switching

`data/shift_test.txt`（2026-09-19，负载约 0.5 kg）:

```
t = 31.04 s   0 → 1    放上负载约 1 秒后 / about 1 s after loading
t = 45.05 s   1 → 2
t = 52.28 s   2 → 0    卸载约 2 秒后，一次跳两档 / 2 s after unloading, two levels at once
```

载重状态下（35–50 s，档位 1–2）：
While loaded (35–50 s, levels 1–2):

```
|PWM| 1575-1606     饱和 0 %     倾角 std 0.54-0.94°     osc 0.78-1.20
```

> 注意载重时 `osc` 只有 0.8–1.2，比空车 0 档的 7.4 还低 —— **负载把抖动压住了**，
> 这是检测原理成立的直接证据。
>
> Note that `osc` while loaded is *lower* than the empty car at level 0. The payload
> damps the chatter, which is exactly why the detector works.

### 12.3 降档速度 / Down-shift Speed

`data/auto_test.txt`（空车，从 4 档起步）:

```
t = 4.30 s   4 → 2      快速路径，跳两档 / fast path, two levels
t = 5.60 s   2 → 0      间隔 1.31 s
从起立到安静约 2.5 秒 / about 2.5 s from standing up to quiet
```

---

## 13. 失败过的方案 / What Failed and Why

这一节比正文更有价值 —— 六个版本，每个都是被实车否掉的。
This section is more useful than the rest. Six versions, each killed by the car.

### v1 · 升档信号被增益过大误触发 / Up-shift tripped by over-gain

用 `|低通(倾角)|` 做升档判据。增益过大时该值冲到 **4.98°**，远超阈值 2.5°。
**教训：用了会被自己触发的信号。**

Used `|low-pass(angle)|`. Over-gaining pushed it to 4.98 deg, far past the 2.5 deg
threshold. **Lesson: the signal was triggered by the very thing it was meant to
rule out.**

修法：改成 0.3–2 Hz 带通（去掉静态偏置），并加 `osc < 10` 门限。
Fix: band-pass 0.3–2 Hz to drop the static offset, and gate on `osc < 10`.

### v2 · 应急通道死锁 / Express lane deadlock

应急升档用**瞬时倾角**超 4° 判定。4 档空车本身就抖 ±2–4°，于是每一拍都触发，
每次触发都重置"禁止降档 3 秒"，**永远降不下来**。

The express lane used the raw instantaneous angle. Level 4 on an empty car swings
2–4 deg of pure chatter, which tripped it every tick, and each trip re-armed the
3 s no-down timer. The ladder could never step down again.

修法：改用 0.3–2 Hz 带通值；并且只在**真正发生升档时**才置禁降标志。
Fix: use the band-passed value, and arm the no-down flag only when the level
actually moves.

### v3 · 应急通道与降档打架 / Express lane fighting the down-shift

即使换成带通值，4 档空车的**慢摆**仍然超 4°。每降一档就被应急拽回 4 档，
**降一步、退一步、等 3 秒**，所以又慢又一直抖。

Even band-passed, the slow swing of a violently over-gained empty car exceeded
4 deg. Every step down was immediately undone, so the car shook for a long time.

修法：应急通道也加 `osc < LA_OSC_HI` 门限。
Fix: gate the express lane on `osc` being low as well.

### v4 · 互斥条件留出死区 / Mutually exclusive conditions leave a gap

```
降档需要  osc > 25
应急需要  osc < 25
```

如果振荡的 8–16 Hz 分量恰好落在 25 以下，**两条路都不触发** —— 车卡在高档位抖。

If the oscillation's 8-16 Hz content landed below 25, neither path fired and the
car parked at a high level, shaking.

### v5 · 定时下降 / Blind decay timer ❌

加了"8 秒没动静就自动降一档"作为兜底。**这是退步** —— 定时器在零信息的情况下
做决定，装着负载、档位正确时它照样会降，然后车开始晃、再升回来。

A timer that decides with no information at all. With a load aboard and the level
correct, it would still step down, the car would start to wallow, and it would step
back up. **Rejected.**

### v6 · 在线载重估计 / Online payload estimator ❌

思路很好：死区补偿让车一直处于极限环，**那个抖动就是持续激励**。在这个激励里
做回归 `α = k·u`，斜率 `k ∝ 1/C`，而 C 对载重极其敏感（空车 0.0023，2 kg 时
0.024）。

离线在已有数据上验证：空车 k = 1.97 / 2.36，载重 k = 1.05，**2 倍分离**。

**但上车完全不可用。** 同一个空车状态下 k 在 3.6 到 27.4 之间乱跳，和载重完全
重叠，75 秒里换了 19 次档。

**原因是我的离线验证不公平：** 离线用的是 FFT **砖墙带通**，固件里只能跑**两个
一阶 IIR**，阻带衰减差一个数量级。而 `α = (gyro − gyro_prev)×200` 这个微分把
陀螺量化噪声放大 200 倍 —— 砖墙滤波器能切干净，一阶 IIR 切不掉，回归分母里全是
噪声。

**教训：离线验证必须使用和固件完全相同的滤波器结构，否则是在用一个实现不出来的
滤波器证明可行性。**

The idea was sound: the dead-band limit cycle permanently excites the plant, so a
running regression of angular acceleration on drive recovers a number proportional
to `1/C`, and `C` is what a payload changes. Offline on recorded data it separated
empty (k = 1.97, 2.36) from loaded (k = 1.05) by about 2x.

On the car it was useless — k wandered from 3.6 to 27.4 on an unchanging empty
chassis and overlapped the loaded range completely, producing 19 level changes in
75 s.

The offline check had used an **FFT brick-wall band-pass**; the firmware can only
afford **two cascaded one-pole IIRs**, whose stopband rejection is an order of
magnitude worse. Differentiating the gyro multiplies its quantisation noise by 200,
and the IIR pair leaves far too much of it in the regression denominator.

**Lesson: validate offline with the exact filter structure the firmware will use,
or you are proving feasibility with a filter you cannot implement.**

`k` 仍在计算和记录（CSV 的 `k_c`），但**不参与决策**。
`k` is still computed and logged, but decides nothing.

### 贯穿所有失败的一条主线 / The thread through all of them

> 我反复试图用**同一个量的不同侧面**去做两个相反的决策，而这个量对"增益过大"
> 和"增益不足"都会增大。每堵一个漏洞就在别处开一个新的。
>
> 最终成立的设计承认了这一点：**只有高频抖动能单向区分**，所以它成为唯一的
> 仲裁者 —— 所有升档路径都必须先确认"不抖"。
>
> I kept trying to make two opposite decisions from two faces of the same quantity,
> one that grows under both faults. Every patch opened a new hole somewhere else.
> The design that finally worked accepts this: only high-frequency chatter
> discriminates in one direction, so it became the sole arbiter, and every up-shift
> path must first confirm the car is quiet.

---

## 14. 已知限制 / Known Limits

### 14.1 调度是"适应"，不是"救援" / Adaptation, not rescue

```
倒立摆自由倒下时间常数      90 ms
档位调节最快反应            1000-3000 ms
```

整条自适应回路比被控对象**慢 10–30 倍**。

| 情况 / Situation | 能否救 / Can it save the car |
|---|---|
| 车在平衡点附近，扰动缓慢 | ✅ 能（实测 1 秒响应）|
| 车已经在发散 | ❌ 不能 —— 等它反应过来车早倒了 |

**正确用法是：让每一档本身就有能力守住它对应的载重区间，调度只负责选对档位。**

The adaptation loop is 10–30x slower than the plant. It can pick the right level
for a slowly changing load; it cannot rescue a car that is already diverging. Each
level must be able to hold its own load range on its own.

### 14.2 未验证的部分 / Not yet validated

- **梯子的 3–4 档对真实重载（2–4 kg）的效果**，只有解析结论，没有实测
  Levels 3–4 against a real 2–4 kg payload: analysis only, never measured
- **`LA_LEAN_HI = 1.0`** 是在约 0.5 kg 负载下确定的，更重的负载没试过
- 只在硬地面测试过，地毯等软表面未知

### 14.3 依赖死区极限环 / Depends on the dead-band limit cycle

检测器依赖车**一直在抖**这个事实（|PWM| 恒在补偿量附近，每秒换向 28.8 次）。
如果以后把死区补偿调到完美、极限环消失，`la_osc` 的信噪比会下降。

The detector relies on the car always being in a small limit cycle. If the dead-band
compensation were ever made perfect and the limit cycle vanished, the signal-to-noise
ratio of `la_osc` would drop.

### 14.4 硬件注意 / Hardware note

这块板子的 Flash 在 `0x08010000` 附近以上不可靠。镜像越界会让 RW 初值损坏，
全局变量开机就是垃圾。**必须用 -O1 编译**，加功能前检查镜像结束地址。
当前镜像结束于 `0x0800E4EB`，余量约 7 KB。

Flash on this board is unreliable above roughly `0x08010000`. An image that crosses
it loads garbage into every initialised global. **Build with -O1** and check the
image end address before adding features.

---

## 15. 文件清单 / File Manifest

```
adaptive_load_mode26/
├── README.md                  本文件 / this file
├── M26_ladder.hex             可直接烧录的固件 / ready-to-flash firmware
├── src/
│   ├── load_adapt.c           算法本体 / the algorithm
│   ├── load_adapt.h
│   ├── tune_io.c              串口遥测与调参 / serial telemetry and tuning
│   └── tune_io.h
├── tools/
│   ├── shift_test.ps1         双向切换测试（空车→加载→卸载）
│   ├── auto_test.ps1          降档速度测试
│   └── osc_sweep.ps1          逐档测抖动强度
└── data/
    ├── base_empty.txt         空车基线 32 s
    ├── kd120.txt              Kd=120 抖动测试（定出 13.96 Hz）
    ├── sweep_A_300.txt        扫频 0.5-25 Hz，幅值 300
    ├── sweep_A_500.txt        扫频 0.5-25 Hz，幅值 500（对比出间隙非线性）
    ├── auto_test.txt          降档速度实测
    └── shift_test.txt         双向切换实测
```

### 脚本用法 / Script Usage

```powershell
powershell -ExecutionPolicy Bypass -File ".\tools\shift_test.ps1"
```

脚本会提示车上操作，三个阶段自动计时：空车 20 s → 加载 30 s → 卸载 25 s。
输出存在脚本旁边，不受 PowerShell 当前目录影响。

The script prompts for the physical steps and times three phases automatically.
Output is written next to the script, independent of the shell's working directory.
