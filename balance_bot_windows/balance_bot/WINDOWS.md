# Windows + MuJoCo 快速上手

不需要 WSL、不需要 Ubuntu、不需要 ROS 2。整个工程是纯 Python，
MuJoCo 和 PyQt6 在 Windows 上都有官方轮子。

---

## 1. 装

装 **Python 3.10 ~ 3.12**（python.org，安装时务必勾选 **Add python.exe to PATH**），
然后在工程目录里双击 `setup_windows.bat`，或者在 PowerShell 里：

```powershell
cd C:\Users\jiang li\balance_bot
powershell -ExecutionPolicy Bypass -File setup_windows.ps1
```

装完会自动跑两套自检，应该看到 `47 passed, 0 failed` 和 `50 passed, 0 failed`。

要训练神经网络策略的话加一个参数（会多装约 800 MB 的 torch）：

```powershell
powershell -ExecutionPolicy Bypass -File setup_windows.ps1 --train
```

双击装的（cmd 版）用同一个开关：

```
setup_windows.bat --train
```

**每次开新终端先激活**：

```powershell
.\.venv\Scripts\Activate.ps1      # PowerShell
.venv\Scripts\activate.bat        # cmd.exe
```

## 2. 跑

```powershell
python scripts\sim_gui.py                                      # 交互控制台（推荐先看这个）
python scripts\bench_twin_baseline.py --backend mujoco --episodes 40    # MuJoCo 里测 baseline
python scripts\bench_twin_baseline.py --imu lag --episodes 40           # 换回旧的姿态模型对比
python tests\test_twin_baseline.py                                      # 50 项自检
```

`sim_gui.py` 是一个窗口，MuJoCo 画面直接画在里面（拖动转视角、滚轮缩放）：

| 面板 | 能干什么 |
|---|---|
| 控制模型 | 原厂 LQR / 原厂 PID（7 套增益任选）/ CEM 调参 / PPO 策略 |
| 姿态滤波 | 固定 `KF.c` 卡尔曼（板子真跑的）。旧的 10 ms 一阶滞后不再出现在界面上，只保留 `--imu lag` 开关 |
| 环境与传感器 | 传感器噪声、**地板滑度**（改的是接触摩擦系数）、侧风与风向、扭矩噪声、电池电压 |
| 冲击输入 | 力度 + 作用时间 + 方向（前/后/左/右/随机，空格键） |
| 驾驶 | WASD / 方向键 / 摇杆，映射到**固件自己的目标值** |
| 存活步数记录 | 本次 / 最佳 / 摔倒次数 / 每回合明细，**倒下自动扶起** |

> 电压滑条拉到 9.6 V 以下，固件的 `Turn_Off()` 会自己把电机断掉——和真车一样。
> 地板滑度拉到 μ≈0.1 附近，原厂 PID 先崩，LQR 还能站；这是个能看出架构差别的档位。

Windows 上路径用反斜杠，其余命令和之前完全一样。

> ROS 2 / Gazebo 那一侧已经不在这个工程里了。孪生、基准、调参、可视化
> 都不依赖它。

---

## 3. 原厂参数（全部）

### 3.1 车体（`6.LQR/Matlab/parameter_LQR.m`）

| 量 | 值 | 备注 |
|---|---|---|
| 车身质量 `M` | 0.930 kg | 整备 1.000 kg 减去两个轮子 |
| 单轮质量 `m` | 0.035 kg | |
| 轮子半径 `r` | 0.0336 m | 直径 67.2 mm |
| 质心高度 `L` | 0.0383 m | 轴心到质心 |
| 俯仰惯量 `J` | 7.110e-4 kg·m² | 绕质心 |
| 偏航惯量 | 1.185e-3 kg·m² | 含两轮偏置 |
| 轮距 `d` | 0.1612 m | |
| **自然频率** | **12.98 rad/s** | 时间常数 77 ms |

### 3.2 固件常数

| 量 | 值 | 出处 |
|---|---|---|
| 控制频率 | 200 Hz（5 ms 中断） | `app_motor.h Control_Frequency` |
| PWM 频率 / 周期 | 25 kHz / 2880 计数 | `bsp.c BalanceCar_PWM_Init(2880,0)` |
| PWM 限幅 | ±2600 | `app_control.c PWM_Limit` |
| PWM 死区补偿 | +1300（45.1 % 占空比） | `app_motor.c MOTOR_IGNORE_PULSE` |
| 速度→PWM 系数 | 2400 计数 / (m/s) | `app_control.c Ratio_accel` |
| 编码器 | 4 × 11 × 30 = **1320** 线/圈 | `app_motor.h` |
| LQR 环却按 | **1560** 线/圈 | `app_control.c` — 固件自相矛盾，见 TWIN_BASELINE.md 4.2 |
| 关电机角度 | ±40° | `app_motor.c Turn_Off` |
| 关电机电压 | 9.6 V | 同上 |
| 机械零位（LQR） | 0.0349 rad = 2.0° | `app_control.c Target_angle_x` |
| 机械零位（PID） | 1.0° | `main.c Mid_Angle` |
| 陀螺量程 | ±2000 °/s → 939.8 LSB/(rad/s) | `Get_Angle()` 里的 /939.8 |
| 加速度量程 | ±2 g → 1671.84 LSB/(m/s²) | `Get_Angle()` 里的 /1671.84 |
| 姿态算法 | **卡尔曼**（`GET_Angle_Way = 2`） | `main.c` |
| 卡尔曼整定 | `Q = 1e-10·I`，`R = 1e-4`，`Ts = 5 ms` | `APP/KF/KF.c` |
| → 收敛增益 | `K∞ = [0.003311, −0.000998]` | 迭代 Riccati |
| → **估计器时间常数** | **3.0 秒**（≈ 纯陀螺积分器） | 同上 |
| 片上 DLPF | **98 Hz / 94 Hz**，群延迟 2.8 / 3.0 ms | `DMP_Init()` → `mpu_set_lpf(100)` |
| MPU 采样率 | 200 Hz | `DEFAULT_MPU_HZ = 200` |
| 遥控前进 / 后退 | 0.5 / −0.3 m/s | `Target_x_speed`，`Flag_velocity = 2` |
| 遥控转向 | ±2.0 rad/s | `±4 / Flag_velocity` |
| 原地旋转时 K5/K6 | 22.3607 | `enTLEFT` / `enTRIGHT` |

> 那个 3.0 秒不是笔误。`Q` 比 `R` 小七个数量级，所以对任何比几秒更快的东西，
> 板上滤波器就是个纯陀螺积分器——**俯仰角上没有可观的滞后**，真正糟糕的是
> 零偏修正慢。孪生上一版把它近似成 10 ms 一阶滞后，方向是错的，详见
> TWIN_BASELINE.md 第 3.5 节。

### 3.3 LQR 增益（`app_control.c`）

```
K1 = -62.0484     x_pose       位移
K2 = -73.3232     x_speed      速度误差
K3 = -361.4617    angle_x      俯仰误差
K4 = -35.9024     gyro_x       俯仰角速度
K5 = 15.8114      angle_z      偏航角
K6 = 15.8114      gyro_z       偏航率误差
```

### 3.4 PID 增益（`pid_control.c`，固件里存的是 ×100 的值，这里已经除回来）

> **注意：全树 24 份 `pid_control.c`，增益互相差到 40%。** 下面是孪生的默认
> 那一套（`4.Balanced_Car_base/04.bluetooth_control`，选它是因为只有这一版是
> 人拿遥控在开）。另外 6 套见 TWIN_BASELINE.md 第 6.1 节，代码里是
> `balance_bot.firmware.pid_gains("lidar")`，GUI 里是个下拉框。

```
直立环   balance_kp  = 96.0     （固件写 9600）
        balance_kd  = 0.48     （固件写 48，单位是 PWM计数/原始LSB = 7.9 计数/(°/s)）
速度环   velocity_kp = 62.0     （固件写 6200）
        velocity_ki = 0.31     （固件写 31）
转向环   turn_kp     = 14.0     （固件写 1400）
        turn_kd     = 0.20     （固件写 20，只在前进/后退时启用）
其他     mid_angle_deg  = 1.0
        integral_limit = ±8000
        encoder_lpf    = 0.84   （bias = 0.84*bias + 0.16*new）
```

### 3.5 电机模型（从固件标定反推，不是猜的）

| 量 | 值 | 怎么来的 |
|---|---|---|
| 死区 | 45.1 % 占空比 = 5.42 V | `1300/2880` |
| 反电动势 `b = n·Ke` | 0.3360 V·s/rad | 由「满占空比 = 0.658 m/s」定住 |
| 库仑摩擦 `τ_c` | 0.0903 N·m/轮 | 由死区定住 |
| 转矩系数 `a = n·Kt/R` | 0.01667 N·m/V | = 堵转/12V |
| **堵转力矩** | **0.20 N·m/轮** | **唯一自由参数**，有实测数据先标它 |
| 空载转速 | 19.6 rad/s = 0.658 m/s | 反推 |
| **净轮上力** | **6.5 N** | `2(τ_stall−τ_c)/r`，车重 9.8 N |

---

## 4. 训练方法

两条路，产物不一样，**别混为一谈**。

### 路线 A：搜固件那几个常数（推荐先做这个）

**方法**：交叉熵法（CEM）。搜索变量是**相对固件值的对数倍数**，
所以 `x = 0` 恰好就是原厂参数——任何提升都是实打实赢过了一个能用的控制器。
每轮采样一批候选，取前 25 % 精英，用精英的均值和标准差更新分布，标准差每轮 ×0.92。

**目标函数**：`平均存活步数 − 40 × RMS俯仰角`。存活是主项，
俯仰角只在存活相同时用来分胜负。

**产物**：6 个数字，可以直接替换 `app_control.c` 里的 K1~K6，能烧进板子。

```powershell
python scripts\tune_twin_baseline.py --firmware stm32_lqr --out tuned_lqr.json
python scripts\tune_twin_baseline.py --firmware stm32_pid --out tuned_pid.json
python scripts\bench_twin_baseline.py --params tuned_lqr.json --episodes 40
```

### 路线 B：PPO 增益调度（神经网络，烧不进 STM32）

策略每 40 ms 改写一次增益，是个 33→64→64→9 的 MLP。
适合研究「增益该怎么随工况变」，但要跑在板子上得先想清楚怎么部署。

```powershell
python scripts\train_stm32_rl.py --steps 3000000 --n-envs 8
```

**不要用 `balance_bot.train_sb3`** —— 那个训的是底层通用机器人（33 维观测、
9 个级联 PID 增益），导出的 npz 格式 STM32 运行时读不了，放进来 UI 会静默
忽略。两者的对照表在 README 第 3 节。

> Windows 上 `--n-envs > 1` 会用 spawn 而不是 fork，环境工厂必须可 pickle。
> 我已经把闭包改成了 `EnvFactory` 类，所以这条在 Windows 上能用。
> 另外注意：**工程里不附带训练好的权重**。原来那几个 `.npz` 要么是在通用
> 机器人参数上训的，要么是在旧的一阶滞后姿态模型上训的，都不能直接用在
> 这辆 STM32 车上，已经清掉了 —— 要对比就按上面这条重训。

---

## 5. 训练参数（路线 A 实际用的）

```
--iters              12        CEM 轮数
--pop                14        每轮候选数
--elite              0.25      精英比例（取前 25 %）
--sigma              0.35      初始标准差（对数倍数空间）
--span               4.0       搜索范围 = 固件值的 1/4 ~ 4 倍
--episodes           4         每个候选每个难度跑几个回合
--episode-seconds    15        回合长度（评测时是 20 s）
--difficulties       0.75 1.0  在这两档上搜
--seed0              70000     调参种子
```

**评测种子是 20000（`bench_twin_baseline.py` 的默认值），和调参的 70000 不相交。**
这不是形式主义：调参集上分数 189 → 357（+89 %），held-out 上只有 +48 %，
那个落差就是过拟合的量。同种子调完再同种子报，测的是背题不是控制。

`mid_angle_deg` 故意不参与搜索——它是 IMU 装歪多少度的物理量，
让优化器动它只是在拟合仿真自己的安装偏置，换到真车上没意义。

一轮约 15 分钟（解析模型，单核）。

---

## 6. 训练出来的参数

> **`tuned_lqr.json` 已于 2026-09-10 删除。** 下面这些数字只作为方法论记录
> 保留，**不要拿去用**——文件不在了，也没法复现。原始数值存档在
> `runs_stm32/DELETED_2026-09-10.md`。另外注意下表引的 K 值和被删那份文件里
> 的并不一致（文件里是 K1 -105.33 / K2 -173.18），说明它们来自更早的一次
> CEM 运行，文档当时没跟着更新。

原 `tuned_lqr.json`：

```
K1 =  -80.6927     （固件 -62.0484，×1.30）
K2 = -131.0275     （固件 -73.3232，×1.79）
K3 = -361.5323     （固件 -361.4617，×1.00  ← 一动没动）
K4 =  -23.0705     （固件 -35.9024，×0.64）
K5 =   14.7841     （固件  15.8114，×0.94）
K6 =   35.2366     （固件  15.8114，×2.23）
```

> ⚠ **下面这组数字全部作废，只留方法论。** 它是在旧的一阶滞后姿态模型上
> 搜出来、也在它上面评的。此后改动的还有：姿态链路换成板子真跑的 `KF.c`
> （TWIN_BASELINE.md 3.5）、电机模型换成三参数直流直线、`tau_max` 0.20→0.40、
> 整备质量 1.000→0.942 kg、轮距/轮径按 CAD 重定、MuJoCo 积分器 RK4→Euler。
> 任何一条都足以让这张表失效。

**held-out 成绩**（40 回合/档，评测种子 20000，解析模型，旧姿态模型）：

| 难度 | 固件 LQR 原型 | CEM 调参 | 固件 PID 原型 |
|---|---|---|---|
| 0.50 | 460.5 | 476.6 | **500.0** |
| 0.75 | 249.3 | 330.0 | **464.5** |
| 1.00 | 149.3 | 221.5 | **349.4** |

怎么读：

- `K3 ×1.00` —— 俯仰刚度一动没动，说明固件那个值本来就对。
- `K4 ×0.64` —— 俯仰阻尼降下来了。**这条解释已经作废**：当时的说法是
  「那个角速度是角度差分出来的、本身带滞后」，但滞后是旧占位模型发明的，
  真板子的卡尔曼几乎没有滞后（TWIN_BASELINE.md 3.5）。CEM 拟合的是仿真的缺陷，
  不是真车的特性——要在新模型上重搜。
- `K2 ×1.79` —— 速度反馈加重。
- **`K6 ×2.23` 是假的。** 偏航率那个信号被固件的 10⁶ 缩放 bug 打死了
  （TWIN_BASELINE.md 4.1），K6 乘多少都是空转，CEM 只是在一个没有梯度的维度上随机漂移。
- **调完的 LQR 仍然打不过原厂 PID**（330 vs 465）。这辆车上是**架构问题不是
  参数问题**，继续搜参数天花板就在那。要超过 PID 得动结构——最明显的一处是
  把 LQR 那个「角度一阶差分当角速度」换成真陀螺（PID 工程本来就是这么干的）。

> 上表是**解析模型**跑的。MuJoCo 的绝对值会有出入（接触模型不同），
> 趋势应该一致——加 `--backend mujoco` 在你机器上跑一遍才算数。
