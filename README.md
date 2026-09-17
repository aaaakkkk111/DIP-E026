# E026 — Digital Twin of the Yahboom STM32 Balance Car
# E026 — 亚博 STM32 平衡小车数字孪生

A physics-accurate digital twin of a real two-wheel self-balancing car, fitted
against real-car recordings, plus the RL tooling built on top of it.

一台真实两轮平衡小车的数字孪生：物理模型按真车实测数据拟合，上面搭了强化学习
的训练与部署工具链。

> **Branch note / 分支说明** — this branch carries the digital-twin work and is
> independent of `main`; it is not a merge of the other team branches.
> 本分支是数字孪生这条线的独立内容，不混杂 `main` 与其他团队分支。

---

## What this is / 这是什么

**EN.** The firmware of a Yahboom STM32 balance car is reproduced line by line
in Python (cascade PID and LQR, the `KF.c` attitude Kalman filter, the MPU6050
on-chip DLPF, the PWM dead band, the 40° cut-out), and driven against either an
analytic rigid-body model or MuJoCo. The physical parameters are **not
guesses**: they were fitted by replaying real recordings captured from the car
over UART at 200 Hz and matching the twin's output to the real one, metric by
metric.

**中文.** 亚博 STM32 平衡小车的固件在 Python 里逐行复现（串级 PID 与 LQR 两套、
`KF.c` 姿态卡尔曼、MPU6050 片上 DLPF、PWM 死区、40° 断电保护），下面接解析刚体
模型或 MuJoCo。物理参数**不是估的**：用 200 Hz 串口录下的真车数据原样回放，
让孪生输出逐项对上真车，拟合出来的。

### How close is it? / 有多像真车

Mode 1 standstill (factory gains, dead-band compensation 1500, unloaded), scored
on 8 noise seeds that took no part in the fitting:

模式 1 静止（原厂增益、死区补偿 1500、空车），用 8 个没参与拟合的噪声种子评测：

| Metric / 指标 | Twin / 孪生 | Real car / 真车 |
|---|---|---|
| Tilt std / 倾角标准差 | 0.26° | 0.22–0.27° |
| Gyro RMS / 陀螺 RMS | 10.1 °/s | 8.6–10.3 °/s |
| PWM reversals / 换向率 | 26.5 /s | 22–28.8 /s |
| Displacement p-p / 位移峰峰 | 2.66 mm | 2.1–3.3 mm |
| Main frequency / 主频 | 4.34 Hz | 4.29 Hz |

Composite error **0.018** (0 = every metric inside the real car's own
run-to-run range; the starting point was 0.99).

综合误差 **0.018**（0 = 每一项都落在真车三次复现的范围内，起点是 0.99）。

**Three findings that the fitting forced out / 拟合逼出来的三个结构性发现:**

1. **The Kalman bias state must be seeded.** On power-up the firmware's filter
   learns the +2 °/s gyro bias within a few ticks; the twin reset it to zero
   every episode, which showed up as a 40 mm wander and a 12.5 Hz oscillation.
   卡尔曼零偏必须播种，否则表现为 40 mm 游走加 12.5 Hz 振荡。
2. **MuJoCo rolling friction needs `condim 6`.** The coefficient was in the MJCF
   all along but `condim 3` contacts simply ignore it.
   MJCF 里写了滚动摩擦系数，但 `condim=3` 的接触根本不算它。
3. **The two motors are not a matched pair** — 99 counts of dead-band
   difference. The real car's left and right encoders differ by 20 %; a
   symmetric twin puts all its energy at the 4.3 Hz fundamental and can never
   reproduce the real 3–6 Hz / 6–12 Hz split.
   左右电机死区差 98.6 计数；对称的孪生凑不出真车的频谱分配。

---

## Repository layout / 目录结构

```
balance_bot_windows/balance_bot/
├── balance_bot/            核心库 / the library
│   ├── firmware/           固件的逐行复现 / the firmware, reproduced
│   ├── backends/           MuJoCo 与解析后端 / plant backends
│   ├── ui/sim_gui.py       交互 GUI / interactive GUI
│   └── stm32_*.py          训练环境 / RL environments
├── scripts/                训练、评测、导出、真车拟合 / train, bench, export, fit
├── tests/                  三套自检 / three self-check suites
├── keil_out/               生成的 C，可直接进 Keil 工程 / generated C for Keil
├── TWIN_BASELINE.md        孪生怎么建的（深入，中文）/ how the twin was built
├── ADAPTIVE_PID.md         情境自适应 PID 框架 / situation-adaptive PID
└── WINDOWS.md              全部参数表 + 训练方法 / full parameter tables
```

---

## Quick start / 快速上手

```powershell
cd balance_bot_windows\balance_bot
.\setup_windows.bat                 # 建虚拟环境、装依赖 / venv + deps
.\.venv\Scripts\Activate.ps1

python scripts\sim_gui.py           # 交互窗口，先看这个 / start here
python tests\test_twin_baseline.py  # 89 项自检 / 89 self-checks
```

The GUI gives you the car in MuJoCo with the factory firmware running on it:
drive it with WASD, kick it with the space bar, load it up to 4 kg, change the
floor friction, and switch between the 20 factory gain modes, a CEM-tuned set,
an ideal math-only model, or a trained policy.

GUI 里是跑着原厂固件的小车：WASD 开车、空格踹一脚、加最多 4 kg 载重、改地面
摩擦，控制器可在 20 个原厂模式、CEM 调参结果、纯数学理想模型和训练好的策略之间切换。

---

## What you can do with it / 能拿它做什么

| | EN | 中文 |
|---|---|---|
| **Baseline** | Score any controller against the factory firmware on the same plant, same seeds | 在同一个被控对象、同一批种子上，和原厂固件比 |
| **Tune** | CEM search over the six firmware constants; the result drops into `app_control.c` | CEM 搜六个固件常数，结果可直接写回 `app_control.c` |
| **RL** | PPO rewrites the PID gains at 25 Hz while the firmware runs at 200 Hz | PPO 在 25 Hz 改写增益，固件仍在 200 Hz 跑 |
| **Deploy** | Export a quantised policy as C that fits in 7 KB of flash | 把量化后的策略导成 C，占 7 KB flash |
| **Verify** | Replay real recordings through the twin and compare metric by metric | 真车录波原样回放，逐项对比 |

---

## Honest limits / 已知限制

**EN.** The twin is calibrated in one corner of the operating envelope: mode 1,
standstill, no payload — **5.5 % of the wheel-speed range and 2.7 % of the
torque authority**. Outside that corner it is qualitatively right and
quantitatively optimistic: under excitation its oscillation amplitude is 60–70 %
of the real car's and its resonance sits 20 % low; payload, slopes, slip and
impacts are extrapolation. `TWIN_BASELINE.md` §10 records exactly what was
measured, what was assumed, and which three real-car measurements would close
the biggest gaps.

**中文.** 孪生只在一个角落里标定过：模式 1、静止、空车——**轮速量程的 5.5%、
力矩权限的 2.7%**。出了这个角落，定性对、定量偏乐观：受激时振荡幅度只有真车
60–70%，共振频率低 20%；负重、坡道、打滑、冲击都是外推。`TWIN_BASELINE.md`
第 10 节写清楚了哪些是实测、哪些是假设，以及补哪三个测量最划算。

The real-car recordings live outside this repository. Point the fitting tools at
them with `E026_REAL_DATA=<path>/deadband_test`.

真车录波不在本仓库里，用 `E026_REAL_DATA=<路径>/deadband_test` 指过去。

---

## Self-checks / 自检

```powershell
python tests\test_core.py            # 46 通过 — 动力学、后端、控制器
python tests\test_twin_baseline.py   # 89 通过 — 固件复现 + 冻结的训练基线
python tests\test_adaptive.py        # 23 通过 — 自适应 PID 框架（有 gcc 时会真编 C 对拍）
```

Every assertion is a contract with the firmware source: if a constant stops
matching, the baselines stop meaning anything.

每一条断言都是和固件源码的契约：某个常数一旦对不上，基线数字就失去意义。

---

## Hardware / 硬件

Yahboom two-wheel balance car — STM32F103RC (256 KB flash, 48 KB RAM), MPU6050,
JGB37-520 geared motors (0.49 N·m stall, 333 rpm no-load), 1320 counts/rev
encoders, 67 mm treaded rubber wheels, 942 g unloaded, rated for 4 kg payload.
The stock firmware runs its control loop at 200 Hz.

亚博两轮平衡小车 —— STM32F103RC、MPU6050、JGB37-520 减速电机、1320 计数/圈
编码器、67 mm 花纹橡胶胎、空车 942 g、额定载重 4 kg，控制环 200 Hz。
