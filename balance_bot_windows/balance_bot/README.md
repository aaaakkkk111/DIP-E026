# balance-bot — STM32 平衡小车数字孪生 + PPO 增益调度
# balance-bot — STM32 balance-car digital twin + PPO gain scheduling

> **English summary.** A line-by-line reproduction of the Yahboom STM32 balance
> car's firmware (cascade PID and LQR, the `KF.c` attitude Kalman filter, the
> MPU6050 on-chip DLPF, the PWM dead band, the 40° cut-out) running on either an
> analytic rigid-body model or MuJoCo. The physical parameters are fitted
> against 200 Hz recordings captured from the real car over UART: on the mode-1
> standstill benchmark the twin scores **0.018** (0 = every metric inside the
> real car's own run-to-run range), validated on eight seeds that took no part
> in the fitting. On top of it sit a CEM search over the six firmware constants,
> PPO gain scheduling, and a situation-adaptive PID that quantises to 7 KB of C
> for the MCU. Start with `python scripts/sim_gui.py`; the detailed documents
> below are in Chinese.
>
> **现状一句话.** 孪生按真车 200 Hz 录波拟合，模式 1 静止基准综合误差 **0.018**
> （8 个没参与拟合的种子）；上面接了 CEM 调参、PPO 增益调度和情境自适应 PID。
> 入口是 `python scripts/sim_gui.py`。

| 文档 / Document | 内容 / Contents |
|---|---|
| [TWIN_BASELINE.md](TWIN_BASELINE.md) | 孪生怎么建的、每个常数的出处、冻结的训练基线（第 10 节）/ how the twin was built, every constant's provenance, the frozen training baseline (§10) |
| [ADAPTIVE_PID.md](ADAPTIVE_PID.md) | 情境自适应 PID：8 个特征、4 个情境、定点上车 / situation-adaptive PID: 8 features, 4 scenarios, fixed-point deployment |
| [WINDOWS.md](WINDOWS.md) | 全部参数表 + 训练方法 / full parameter tables and training recipes |
| 运行说明.txt | 中文快速上手 / Chinese quick start |

两件事装在同一个工程里：

1. **[TWIN_BASELINE.md](TWIN_BASELINE.md) —— 亚博 STM32 平衡小车的数字孪生。** 固件逐行复现
   （LQR 与串级 PID 两套、`KF.c` 的姿态卡尔曼、MPU6050 片上 DLPF、PWM 死区
   与 40° 断电保护），每个常数都标了出处。用来给调参和 RL 提供一个
   **可比的 baseline**，指标是存活步数。
2. **底层的通用双轮倒立摆仿真** —— 一套真实的三环级联 PID 在 200 Hz 保持平衡，
   PPO 在 25 Hz 实时改写它的九个 Kp/Ki/Kd。孪生就长在这套东西上面，
   所以两者共用同一个动力学、同一套扰动注入、同一份回合逻辑。

两个后端，**同一套控制器、同一套观测、同一个策略文件**：

| 后端 | 用途 | 速度 |
|---|---|---|
| `analytic` | 解析动力学，训练 + 打分 | ~450 agent-step/s（单核） |
| `mujoco` | 真实接触（会打滑、会失去接触），3D 画面 | 实时 20× 以上 |

> 换后端只覆写两个方法（`_plant_reset` / `_plant_step`），控制器、观测向量、
> 奖励、回合逻辑逐字节共用。所以两个后端表现不一致时，那就是真的 sim-to-sim
> 迁移误差，不是两套代码写岔了 —— TWIN_BASELINE.md 2.1 节就抓到了这么一处，而且方向
> 是反的。

---

## 1. 五分钟跑起来

Windows 上双击 `setup_windows.bat`（或 `powershell -ExecutionPolicy Bypass
-File setup_windows.ps1`），然后：

```powershell
.\.venv\Scripts\Activate.ps1

python scripts\sim_gui.py                      # 交互控制台（先看这个）
python scripts\bench_twin_baseline.py --episodes 40     # 两套原厂固件的 baseline
python scripts\tune_gui.py                     # 增益滑条调参
```

也可以直接双击上一层目录的 `run_gui.bat` / `run_bench.bat` / `run_tune.bat`。

`sim_gui.py` 是一个窗口，MuJoCo 画面直接画在里面（拖动转视角、滚轮缩放）：

* **W A S D / 方向键** 或圆形摇杆 → 前进后退、原地转向，用的是**固件自己的
  目标值**（前进 0.5 m/s、后退 0.3 m/s、转向 2 rad/s）
* **空格** 或方向按钮 → 按设定的力度、时间、方向踹小车一脚
* **控制模型**下拉 → 原厂 LQR / 原厂 PID（0.Large program 的 20 个模式任选）/ CEM 调参 /
  PPO 策略。**姿态滤波固定为 `KF.c` 卡尔曼**——板子跑什么，孪生就跑什么；
  旧的一阶滞后近似只在 `--imu lag` 命令行开关里，用来复现本文档的旧数字
* **地板滑度**改的是接触摩擦系数（地面 + 两个轮子），轮子会真的打滑空转
* 倒下**自动扶起**，每回合的存活步数都记进列表
* **P** 暂停，**R** 复位

详细文档：**[TWIN_BASELINE.md](TWIN_BASELINE.md)**（孪生怎么建的、从源码里查出了什么）、
**[WINDOWS.md](WINDOWS.md)**（全部参数表 + 训练方法）、
**运行说明.txt**（中文快速上手）。

---

## 2. 这套东西到底在算什么

### 2.1 动力学（`balance_bot/dynamics.py`）

平面纵向动力学 + 解耦的偏航通道。记
`A = 2m_w + m_b + 2I_w/r²`，`B = m_b·l`，`C = I_b + m_b·l²`：

```
[ A          B·cosθ ] [ v̇  ]   [ τ_sum/r + B·sinθ·θ̇² + F_ext − drag ]
[ B·cosθ     C      ] [ θ̈  ] = [ m_b·g·l·sinθ − τ_sum + M_ext − b·θ̇ ]

I_z·ψ̈ = (τ_R − τ_L)·(track/2)/r − b_yaw·ψ̇ + M_ext
```

RK4，400 Hz。符号约定是刻意选的：**正的轮子力矩把车往前推，同时把车身
往后仰** —— 这就是这类机器人的非最小相位特性（要往前走必须先往后退一下）。
自检里 `tests/test_core.py` 专门验证了这条，因为符号搞反是这类仿真最常见、
最难发现的 bug。

线性化后开环极点在 `±12.3`，右半平面有极点，确实是不稳定系统。

### 2.2 三环级联 PID（`balance_bot/controller.py`）

```
v_ref  ──▶[ 速度环 PID ]──▶ θ_ref ──▶[ 俯仰环 PID ]──▶ τ_sum ──┐
                                                              ├─▶ τ_L, τ_R
ψ̇_ref ─────────────────────▶[ 偏航率环 PID ]──▶ τ_diff ───────┘
```

九个增益 = `kp/ki/kd_pitch`, `kp/ki/kd_vel`, `kp/ki/kd_yaw`。

nominal 值不是拍脑袋定的：在线性化模型上解连续时间 LQR
（`controller.lqr_reference_gains`，得到 `K = [0.354, 3.835, 0.582]`），
再换算成等价的级联 PID 增益，最后用扫参微调积分项。所以基线本身就是个
能用的控制器 —— PPO 要赢它是要花点力气的。

**一个容易踩的坑**：积分限幅是按**输出单位**做的（`i_pitch_out_max = 0.7 N·m`），
不是限死累加器本身。如果限死累加器，Ki 很小的时候积分项还没到能消除稳态误差
就饱和了，PPO 看到的 Ki 响应就是非单调的乱七八糟的东西 —— 我一开始就是这么写的，
扫参结果一直显示 `ki_vel = 0` 最优，改成输出单位限幅之后 `ki_vel ≈ 0.15` 才
把 0.6 m/s 的稳态误差从 0.27 m/s 压到 0.00。

### 2.3 PPO 学什么（`balance_bot/env.py`）

* **动作**：9 维 `[-1,1]`，**对数均匀**映射到九个增益的上下界。
  用对数是因为控制增益天然是乘性的 —— 差两倍比差一个绝对量重要得多。
* **观测**（33 维）：俯仰/俯仰角速度/速度/偏航率及其参考与误差、三个积分项、
  左右轮力矩、饱和标志、最近障碍距离、8 路射线、以及**当前的九个归一化增益**
  （策略必须知道自己上一步给了什么）。
* **奖励**：直立 + 存活 − 速度跟踪误差 − 偏航跟踪误差 − 力矩 − 俯仰角速度
  − 增益抖动 − 障碍接近度；摔倒 −40 并终止，撞上 −15 并终止。
* **域随机化**：质量 ±25%、质心高度 ±20%、转动惯量 ±30%、摩擦、电机增益误差、
  传感器噪声、IMU 零漂、0–4 拍延迟、地面打滑、风力、以及随机冲击（最大 20 N，
  足够把固定 PID 掀翻）。难度 0→1 按课程逐步拉满。

**关键设计**：策略输出层的 bias 初始化成 nominal 增益对应的 action，
所以训练**从一个已经会平衡的控制器起步**，PPO 的预算全花在「什么时候该偏离」
上，而不是从零重新发明一个能站住的控制器。这是 20 万步收敛和永远不收敛的区别。

**另一个关键设计**：训练时没人开车，所以脚本化的速度指令上面挂了一层
反应式避障（`BalanceCore._avoid`）。没有它，小车再怎么会平衡也会直直撞墙，
而一个策略无法影响的终止就是纯粹的梯度噪声。这层在你用 UI 开车时自动旁路。

---

## 3. 训练与调参

两条路，产物不一样，别混为一谈。

### 路线 A：搜固件那几个常数（能烧进板子）

交叉熵法，搜索变量是**相对固件值的对数倍数**，所以 `x = 0` 恰好就是原厂参数
——任何提升都是实打实赢过了一个能用的控制器。产物是 6 个数字，可以直接替换
`app_control.c` 里的 K1~K6。

```powershell
python scripts\tune_twin_baseline.py --firmware stm32_lqr --out tuned_lqr.json
python scripts\bench_twin_baseline.py --params tuned_lqr.json --episodes 40
```

**调参种子 70000 起，评测种子 20000 起，互不相交。** 不这么做测的是背题不是
控制 —— WINDOWS.md 第 5 节有这个落差的实测值。`--imu` 在两条命令里必须一致，
否则测的是两个不同的孪生。

### 路线 B：PPO 增益调度（神经网络，烧不进 STM32）

策略每 40 ms 改写一次增益。适合研究「增益该怎么随工况变」，但要跑在板子上得
先想清楚怎么部署。

```powershell
python scripts\train_stm32_rl.py --steps 3000000 --n-envs 8   # 需要 torch + sb3
```

**这条和 `balance_bot.train_sb3` 不是一回事，别用错。** 后者训的是底层那套
通用机器人（33 维观测、9 个级联 PID 增益），导出格式 STM32 运行时读不了，
放进来 UI 会静默忽略。两者刻意分开：

| | `scripts/train_stm32_rl.py` | `balance_bot.train_sb3` |
|---|---|---|
| 训谁 | TWIN_BASELINE.md 里那辆 STM32 车 | 底层通用双轮机器人 |
| 观测 | 13 维 | 33 维（含 8 路射线） |
| 动作 | 6 个固件增益的对数倍数 | 9 个级联 PID 增益 |
| 导出 | `stm32_policy` 读的扁平 npz | `policy_io.GainPolicy` 格式 |
| UI 能用 | ✅ 自动列进下拉框 | ❌ |

`--imu` 会写进 npz，评测时 `bench_twin_baseline.py --imu` 必须一致 —— 否则测的是两个
不同的孪生。运行时只需要 numpy 就能加载和推理，单次推理约 20 µs，
机器人端不用装深度学习框架。

> 工程里**不附带**训练好的权重。之前那几个 `policy_stm32_*.npz` 是在旧的
> 一阶滞后姿态模型上训的，姿态链路换成 `KF.c` 之后观测分布变了，留着只会
> 误导人，所以清掉了。要用就按上面重训。

---

## 4. 工程结构

```
balance_bot/
├── balance_bot/
│   ├── params.py          机器人/仿真/增益空间/扰动/奖励 —— 单一事实来源
│   ├── dynamics.py        解析动力学 + RK4 + 线性化
│   ├── controller.py      三环级联 PID + 输出单位限幅的抗积分饱和 + LQR 参考
│   ├── arena.py           场地、障碍、向量化射线投射
│   ├── disturb.py         噪声/延迟/电机误差/冲击 + 训练用域随机化
│   ├── env.py             BalanceCore（纯 numpy）+ Gymnasium 包装
│   ├── mjcf.py            从 RobotParams 生成 MuJoCo 模型
│   ├── nn.py              手写反向传播的 MLP + Adam + running norm
│   ├── policy_io.py       .npz 策略格式（运行时只依赖 numpy）
│   ├── train_sb3.py       通用机器人的 SB3 PPO（不是 STM32 那条）
│   ├── train_numpy.py     通用机器人的零依赖 PPO
│   ├── backends/mujoco_backend.py
│   ├── firmware/          ← STM32 孪生：固件复现，见 TWIN_BASELINE.md
│   │   ├── robot.py           车体参数与固件常数（每个都标了出处）
│   │   ├── controllers.py     LQR / 串级 PID 两套控制律 + 20 个原厂模式
│   │   ├── motor.py           从固件自己的 PWM 标定反推的电机模型
│   │   ├── imu.py             MPU6050：KF.c 的卡尔曼 + 片上 98 Hz DLPF
│   │   └── twin.py            把上面几件接到解析模型 / MuJoCo 上
│   ├── stm32_env.py       STM32 车的 Gymnasium 环境（PPO 调增益）
│   ├── stm32_policy.py    只依赖 numpy 的策略推理
│   └── ui/
│       ├── sim_gui.py         交互控制台：MuJoCo 画面嵌在窗口里
│       ├── twin_baseline_panel.py      调参面板（增益滑条）
│       ├── panel.py           通用面板控件，被 twin_baseline_panel 复用
│       ├── widgets.py         自绘控件（示波器、摇杆，不依赖 pyqtgraph）
│       └── qtcompat.py        PyQt6 / PySide6 / PyQt5 三选一的兼容层
├── scripts/
│   ├── sim_gui.py         交互控制台入口
│   ├── bench_twin_baseline.py      baseline 打分（存活步数）
│   ├── tune_twin_baseline.py       CEM 自动搜固件增益（产物能烧进板子）
│   ├── tune_gui.py        滑条手调
│   ├── train_stm32_rl.py  STM32 车的 PPO 增益调度（导出 UI 能读的 npz）
│   ├── train_adaptive_pid.py       情境自适应 PID 的 PPO（见 ADAPTIVE_PID.md）
│   ├── export_nn_c.py     把策略量化导成能进 Keil 的 C
│   ├── ideal_pid.py       纯数学的理想线性模型（无死区、无延迟、无噪声）
│   └── twin_fit/          真车录波回放与拟合工具（需要 E026_REAL_DATA）
├── tests/
│   ├── test_core.py       46 项：通用机器人 + MuJoCo 接触模型
│   ├── test_twin_baseline.py       89 项：STM32 固件复现 + 冻结的训练基线
│   └── test_adaptive.py   23 项：情境自适应 PID 框架（有 gcc 时真编 C 对拍）
├── examples/              两个参数 JSON 的样例
├── TWIN_BASELINE.md                孪生是怎么建的、从源码里查出了什么
└── WINDOWS.md             全部参数表 + 训练方法
```

## 5. 自检

```bash
python tests/test_core.py            # 46 项：通用机器人 + MuJoCo 接触模型
python tests/test_twin_baseline.py   # 89 项：STM32 固件复现 + 冻结的训练基线
python tests/test_adaptive.py        # 23 项：情境自适应 PID 框架
```

`test_core.py` 覆盖时间步整除、符号约定、右半平面极点、
PID 收敛与跟踪、抗积分饱和限幅、射线投射几何、观测有界性、
同种子可复现性、策略序列化往返、MJCF 合法性、MuJoCo 接触模型下的打滑行为。

`test_twin_baseline.py` 是**孪生对固件的承诺**：每个常数的出处、电机标定的自洽性、
两套工程输出级顺序的差异、两个照抄的固件 bug、`KF_X()` 的逐拍复现与它的
收敛增益。哪一条跪了，就说明某个数字和源码对不上了，baseline 也就不作数了。

> 写这套自检是有回报的：它当场抓到射线阵列整体转了 180°（`ray 0` 指向正后方），
> 而那层反应式避障正是靠 `ray 0` 判断前方 —— 也就是说训练时小车是照着障碍物
> 开过去的。这种 bug 光看回报曲线永远看不出来。

---

## 6. 调参提示

* **改机器人参数** → 只改 `params.py::RobotParams`（STM32 车是
  `firmware/robot.py::STM32_CAR`）。MJCF 和 Arena 都从它生成。
* **PID 不稳** → 先 `python tests/test_core.py` 确认基线还在，
  再看 `controller.lqr_reference_gains` 给的参考增益。
* **PPO 不收敛** → 先确认固定基线的失败率在 20%–50% 之间。基线永远不摔，
  增益调度就没有可赢的空间；基线永远摔，梯度就全是噪声。
  用 `scripts/bench_twin_baseline.py` 看这个数。
* **难度 0 和 0.25 档两套固件都是满分**，不要在那两档上做优化。

---

## 7. 没有实测的部分

解析模型和 MuJoCo 这两条路是真跑通、真训练、真对比过的。

* ~~没有真车实测数据~~ **已有。** 2026-09-16 起有 6 个真车 200 Hz 录波，孪生
  按其中的模式 1 静止基准拟合（`τ_stall` 拟合为 0.568 N·m）。但标定只覆盖
  **轮速量程的 5.5%、力矩权限的 2.7%、0 负重、只有模式 1**——出了这个角落
  定性对、定量偏乐观。逐条见 TWIN_BASELINE.md 第 10.4 / 10.6 节。
  Real recordings exist now, but they cover one corner of the envelope only.
* **ROS 2 / Gazebo 那一侧已经不在这个工程里了**（原来也只做过 XML 校验，
  没有实机跑过）。
* 其余未建模的部分逐条列在 TWIN_BASELINE.md 第 8 节。
