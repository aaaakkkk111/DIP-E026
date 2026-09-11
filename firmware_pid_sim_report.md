# STM32平车固件PID的MuJoCo复现

## 本阶段范围

本阶段只复现实车自带控制器与接口，不包含强化学习、神经网络策略、残差控制或LLM调参。

入口文件：

- `firmware_pid_car.xml`：按实车量级重建的MuJoCo刚体、车轮、上层平台和可变负载。
- `firmware_pid_sim.py`：Normal/Weight_M固件PID、编码器量化、PWM接口、电机近似模型与操作面板。
- `test_firmware_pid_sim.py`：控制周期、参数、PWM、负载和闭环稳定性回归测试。

## 已落实的实车边界

| 项目 | 仿真实现 | 来源/状态 |
|---|---|---|
| 控制周期 | 5 ms（200 Hz） | 厂家中断代码，精确复现 |
| 物理步长 | 1 ms，每5步运行一次控制器 | 为5 ms控制提供整数调度 |
| 轮胎直径 | 67 mm | 厂家 `app_motor.h` |
| 编码器 | 11线、4倍频、30:1，总计1320 count/轮端转 | 厂家 `app_motor.h` |
| PWM | 25 kHz，ARR=2879 | 厂家 `bsp.c` / `motor.c` |
| 死区补偿 | 非零命令加/减1300 | 厂家 `app_motor.c` |
| 最终限幅 | -2600至+2600 | 厂家 `app_control.c` |
| 电池保护 | 低于9.6 V停机 | 厂家 `app_motor.c` |
| 倾倒保护 | 绝对倾角超过40度停机 | 厂家Normal/Weight模式 |
| 电机 | 12 V、30:1、333 rpm、堵转4.8 kg·cm | 产品资料；以一阶直流电机近似进入MuJoCo |
| 可调负载 | 0至4.0 kg，固定在上层平台 | 产品标称4 kg负重；质量和惯量实时更新 |

供应的STEP以毫米为单位，全部几何点的包络约为217 x 152 x 248 mm。MuJoCo使用简化碰撞几何，保留67 mm车轮、约152 mm整车宽度、上层载物平台和约1.01 kg空载质量，处于实车约1.0至1.2 kg的范围。STEP没有材料密度，空载质量和部件质心仍属于工程估计。

数学模型资料用于核对双轮车的前进、俯仰与转向自由度以及左右轮输入关系。MuJoCo直接求解非线性刚体动力学，因此没有把资料中的小角度线性状态空间方程再次叠加到物理引擎中。

## 固件控制复现

每个5 ms控制周期依次执行：

```text
姿态和编码器采样
  -> Balance_PD
  -> Velocity_PI（0.84/0.16低通、积分限幅±8000）
  -> Turn_PD
  -> 左右轮PWM合成
  -> ±1300死区补偿
  -> 最终±2600限幅
  -> 倾倒/低压保护
  -> PWM到轮端扭矩模型
```

Normal参数：

```text
Balance Kp/Kd = 9600 / 48
Velocity Kp/Ki = 6200 / 31
Turn Kp/Kd = 1700 / 20
```

Weight_M参数：

```text
Balance Kp/Kd = 9600 / 75，输出再乘2.0
Velocity Kp/Ki = 7000 / 35，输出再乘1.35
Turn Kp/Kd = 1400 / 20
```

上述数值已逐项对照可执行工程 `stm32_Balance_Car_L` 中的 `APP/mode/app_mode.c::Set_PID()`；Weight_M的2.0与1.35输出倍率也逐项对照 `APP/PID/pid_control.c`。因此本轮没有用仿真专用增益替换官方值，而是通过刚性接触和合理扰动量级修复“受力后无法回正”。

## 验证结果

10秒无界面测试结果：

| 工况 | 最大绝对倾角 | 最后2秒倾角RMS | 保护结果 |
|---|---:|---:|---|
| Normal，0 kg，初始3度 | 2.626度 | 1.543度 | 未触发 |
| Weight_M，1 kg，初始2度 | 1.626度 | 0.205度 | 未触发 |
| Weight_M，4 kg，初始0.5度 | 0.765度 | 0.401度 | 未触发 |
| Normal，4 kg，初始0.5度 | 89.825度 | 11.965度 | 约0.806秒触发倾倒保护 |

还验证了Normal和Weight_M下0.10 m/s目标速度，以及0.30 rad/s差分转向接口。以上结果证明软件闭环与模式差异能够运行，不代表已经完成实车参数辨识。

本轮对1.010 kg空载模型施加最大档4 N、0.10 s四向扰动，8秒验证结果如下：

| 方向 | 最大绝对俯仰角 | 结束俯仰角 | 是否触发保护 |
|---|---:|---:|---|
| 前推 | 18.45度 | 1.19度 | 否 |
| 后推 | 21.51度 | 1.39度 | 否 |
| 左推 | 2.32度 | 1.69度 | 否 |
| 右推 | 2.32度 | 1.32度 | 否 |

刚性地面在该测试结束时的轮胎接触沉陷约0.00011 mm。这里的“刚性”指地面几何固定在worldbody、无质量和自由度，同时接触约束使用2 ms时间常数及高阻抗；MuJoCo仍通过约束求解器处理接触，并非数学上无限刚度。

## 使用方法

图形界面：

```powershell
Set-Location 'D:\DIP\Simulation'
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' .\firmware_pid_sim.py
```

控制面板可以实时调整：

- `Firmware mode`：Normal或Weight_M；
- `Payload`：0至4 kg；
- `Target speed`：-0.50至+0.50 m/s；
- `Target yaw`：-1.0至+1.0 rad/s；
- `Battery voltage`：8.0至12.6 V；
- `Tire friction`：0.40至1.60；该值缺少厂家标定，默认1.15，可在实测后替换；
- “前、后、左、右、停止”按钮和键盘方向键/空格控制运动目标；
- “镜头跟随 / Camera follow”默认开启，使观察中心平滑追踪底盘；跟随时仍可旋转和缩放视角，关闭后恢复自由镜头；
- 前、后、左、右四向外力，提供1 N、2 N、4 N三档并持续0.10 s；
- 实时显示目标/实际速度与转向、PID增益和分量、编码器、PWM、轮端力矩及外力。

界面中的硬件对应区持续显示1/5 ms物理/控制周期、67 mm轮径、126 mm轮距、1320 count/轮端转、12 V 30:1电机、25 kHz PWM和当前模型质量。轮径、编码器、减速比、PWM与保护阈值来自厂家资料和源码；126 mm轮距来自供应的STEP装配。轮胎摩擦、精确质心和电机动态时间常数没有厂家标定，因此明确保留为工程估计，而不是伪装成已知硬件参数。

无界面示例：

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' .\firmware_pid_sim.py --headless --duration 10 --mode Normal --payload-kg 0
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' .\firmware_pid_sim.py --headless --duration 10 --mode Weight_M --payload-kg 2.0 --initial-pitch-deg 0.5
```

运行回归测试：

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m unittest -v test_firmware_pid_sim.py
```

## 仍需实物测量的参数

在开始残差策略训练前，至少应测量：整车空载质量、空载质心、平台负重质心、左右轮实际滚动周长、轮距、起转PWM、正反转死区、电池电压下的轮速/PWM曲线和电机响应延迟。当前模型已经把这些参数集中在 `firmware_pid_sim.py` 与 `firmware_pid_car.xml` 中，测量后可以逐项替换。
