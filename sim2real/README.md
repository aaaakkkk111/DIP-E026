# MuJoCo → STM32 平衡车：执行方案与通信验收

核查日期：2026-09-04。项目：`D:\DIP\Simulation`。

推荐路线：**PC 运行 MuJoCo、实验台、记录和参数优化；STM32 本地运行 200 Hz 状态估计与控制。Type-C / CH340K / USART1 负责双向遥测、目标值和参数包。** 移植的是控制规律、状态定义、参数和验证流程。MuJoCo 物理引擎保留在 PC。

本目录已提供：源码核查结果、完整实施顺序、一个独立的串口验收 Keil 工程、对应 PC 验收脚本。**串口工程只实现 PING/INFO，电机控制脚保持低电平；正式遥测、目标控制、参数更新、实车 LQR 和 HIL 尚需按下文集成。没有提供已验证可平衡的实车固件。**

目前完成 Python 协议自测与 Keil 工程引用检查；未完成 ARM 编译、烧录、实物收发或平衡测试。不能把这些离线检查表述为“实车已能通信”。

## 1. 本次解析到的实际硬件与代码

原始资料目录始终只读。`reference` 是解压到 Simulation 内的副本。

| 项目 | 核查结果 | 本地依据 |
|---|---|---|
| 工程目标 MCU | STM32F103RC；原理图还标注 APM32E103RET6 替代料号，实物需看丝印 | `reference/6.LQR/STM32_code/USER/LQR.uvprojx:17`；YB-EST01-V2.0 原理图 |
| MCU 能力 | STM32F103RC 为 Cortex-M3，最高 72 MHz；本工程定义 Flash 256 KB、RAM 48 KB | 同一 uvprojx 的 Cpu 配置；[ST 官方产品页](https://www.st.com/en/microcontrollers-microprocessors/stm32f103rc.html) |
| 主链路 | Type-C → CH340K → USART1：PA9 TX、PA10 RX，115200、8N1、无流控 | `BSP/Usart1/usart.c:35`，`BSP/bsp.c:22`；串口电路说明 |
| 当前双向通信障碍 | `USART_ITConfig(USART1, USART_IT_RXNE, DISABLE)`；现有中断只是逐字节回显 | `BSP/Usart1/usart.c:65,106` |
| 自动下载电路 | CH340K 的 DTR/RTS 接入 RESET/BOOT0 电路；打开端口可能影响启动状态 | YB-EST01-V2.0 原理图 |
| 姿态采样 | MPU6050 INT 接 PA12，EXTI15_10，每 5 ms 更新；默认 `GET_Angle_Way=2`，卡尔曼滤波 | `APP/app_control.c:22`，`USER/main.c:18`，`BSP/MPU6050/mpu6050.c:10` |
| 左右编码器 | TIM3：PA6/PA7；TIM4：PB6/PB7；右侧读取后取反 | `BSP/Enconder/encoder.c`，`APP/app_control.c:32` |
| 电机 | AT8236 驱动；TIM8 CH1..4 → PC6..PC9 | 原理图；`BSP/Motor/motor.h:6` |
| PWM | 初始化参数 2880、0，实际 ARR=2879；72 MHz 定时器时钟下为 25 kHz | `BSP/bsp.c:18`、`BSP/Motor/motor.c` |
| 蓝牙 | UART5：PC12 TX、PD2 RX，代码默认 9600 baud | `BSP/Bluetooth/bsp_bluetooth.c` |
| 现有保护 | 停机标志、超过 ±40°、电池低于 9.6 V 时输出为零；电压采样平均有明显延迟 | `APP/app_motor.c:109`、`BSP/Timer/bsp_timer.c:88` |

`0.Large program.zip` 的目录清单显示标准库 Keil、HAL Keil、HAL STM32CubeIDE 三个版本。当前深入核查并解压的是 `6.LQR.zip` 与 `4.Balanced_Car_base.zip`。前者便于定位 LQR，后者的 `04.bluetooth_control` 适合作为 PID 通信基线。应优先使用你已在实车上验证能够平衡的工程；“厂家提供”不等于本车当前配置已经验证。

多个扩展名为 `.zip` 的源码包实际是 **RAR5**。已用 UnRAR 解压；Python zipfile、PowerShell Expand-Archive 会失败。无需修改或重命名原始压缩包。[厂家资料入口](https://www.yahboom.com/study/SBR-STM32)也保留了源码、接口、下载教程与相关课程。

## 2. 与同项目其他对话的衔接

已读取任务“实现MuJoCo二轮平衡机器人Demo”的相关记录，并用当前文件交叉核对：

- 已有双语实验台、目标与测量值分开显示、短窗口日志、扰动实验、在线辨识与 LQR。可以保留这些 PC 功能。
- `controller_params.json` 为 revision 4：Q=(65,11,5,1.5)，R=(1.5,0.9)，电机上限 4 N·m，控制间隔 4 个物理步。**该配置由 `llm_step_optimizer.py` 使用。**
- `balance_bot.py:548` 创建默认 `LearnedLQRPolicy(model)`；默认 10 步、800 个样本，Q/R 定义在 `learned_lqr.py`。直接运行它不会加载上述 revision 4。
- 物理步长 2 ms，因此原始 Demo 控制间隔是 20 ms；JSON 优化入口当前是 8 ms。二者都不是实车的 5 ms。重设计时明确使用 `control_period_us=5000`，重新辨识/离散化和求 K。
- 之前记录的 ±25 N、±40 N 推力恢复结果来自仿真，不是硬件测量。不能据此选择实车电机力矩或宣布迁移成功。
- 仿真的“Relearning current situation”允许无策略探索、跌倒、回合重置。硬件模式改为“采集数据 → PC 离线辨识 → 验证候选 → 停车切换”。不能把该探索回路接到真实电机。
- 实车 `reset` 定义为清除实验记录/在停机状态清故障；不能解释为把真实位姿重置成 MuJoCo 初始姿态。

## 3. 先修正这些迁移障碍

1. **模型尺寸不匹配。** 当前 `balance_bot.xml` 轮半径 0.16 m、轮距 0.31 m、总几何质量 3.8 kg。厂家 Matlab 脚本示例为半径 0.0336 m、轮距 0.1612 m、总质量 1 kg。它们只是厂家示例，真实装配后的轮径、质量、重心和惯量需测量。不能只换 K。
2. **编码器计数矛盾。** `app_motor.h` 为 4 倍频 × 11 线 × 30 减速比 = 1320 counts/轮转，`app_control.c:36,43` 却硬编码 1560。必须手动转轮验证有效计数后统一；不能猜一个数。
3. **速度量化。** `(Encoder_Left+Encoder_Right)/2` 先做整数除法。改成 `0.5f*(...)*distance_per_count/dt`。
4. **航向换算错误。** `Wheel_spacing=161.0` 的单位是 mm；现有表达式 `/Wheel_spacing/1000` 又除了一次 1000。应先得到 m/s，再 `yaw_rate=(v_right-v_left)/(Wheel_spacing*0.001f)`。按现有单位解释，原式结果相差 10^6 倍。
5. **速度字段单位不可靠。** `Velocity_Left/Right` 注释有 mm/s，实际代码 `/10` 得到 cm/s。新接口统一 m/s，不能直接透传旧变量并命名为 m/s。
6. **LQR 动作不同。** 厂家版本是 6 状态，输出 `L_accel/R_accel`，再用 `Ratio_accel=2400` 转 PWM；当前 MuJoCo 是 4 状态，输出共模/差模轮端力矩。六个厂家系数和 2×4 的仿真矩阵不互换。
7. **LQR 分支限幅顺序错误。** 先限到 ±2600，再加 ±1300 死区补偿，可能写到 ±3900，超过 ARR=2879。补偿、映射之后必须再限幅。基础 PID 分支已先补偿再限幅，不要把这个错误泛化到所有版本。
8. **控制器可能忽略平衡输入。** 两个 Python 辨识入口求出 `(x_eq,u_eq)` 后只保留 `x_eq`；如实车存在持续输入偏置，应使用 `u=u_eq-K(x-x_ref)` 或完整跟踪前馈设计，并保存经过验证的工作点。
9. **真值传感器不可直接复用。** MuJoCo 使用底盘四元数和 `velocimeter` 真值，实车依赖 IMU 与编码器。仿真策略也应使用相同的观测构造、滤波、量化和延迟。`learned_lqr.py:85`、`llm_step_optimizer.py:314` 的 `/0.16` 还必须改成模型参数。
10. **实时链路需要隔离。** 原 `fputc` 是阻塞发送；TIM6 内还有蓝牙上报等任务。正式平衡中移出中断内的 printf、OLED、字符串拼接、阻塞收发。NVIC 分组 2 下抢占优先级只用 0..3；原 TIM6 设置 4 需要核对修正。

## 4. 执行顺序 A：先把 PC ↔ 小车的链路验收做完

### A1. 准备及连接

1. 确认 MCU 实物丝印和板号；本验收工程按 STM32F103RC、8 MHz 外部晶振、72 MHz 系统时钟构建。若实物是 APM32 或其他板型，先核对其工具链/启动文件兼容性。
2. 保存当前可平衡固件的工程或厂家对应版本，作为恢复基线。烧录验收程序会替换当前应用。
3. 断开两侧电机插头，接 Type-C **数据线**到 PC；需要时使用原配电池给主板供电。USB 能枚举 CH340 不能证明 MCU 与传感器已经供电。不要把充电插口当作运行供电口。
4. 检查设备管理器“端口（COM 和 LPT）”中的 CH340 串口。本地驱动包位于 `D:\DIP\STM32平衡车_V2\10.附件\软件工具\CH341SER.zip`，仅从源目录读取；需要解压时放到 Simulation。也可用 [WCH 官方 CH340/CH341 驱动](https://www.wch-ic.com/downloads/CH341SER_ZIP.html)。
5. 关闭 FlyMCU、串口助手以及占用同一 COM 的进程。一个串口只由一个进程打开。

### A2. 编译串口验收固件

已生成工程：

`D:\DIP\Simulation\sim2real\firmware_uart_bench\USER\UART_BENCH.uvprojx`

- 用 Keil MDK 打开。继承厂家的 **ARM Compiler 5.06 update 7** 配置；如缺少该编译器，先安装/指定相应工具链。厂商 MDK 资源包在 `10.附件\软件工具\MDK-ARM.zip`；不自动运行安装程序。
- Rebuild，要求 0 error；生成的是 `sim2real\firmware_uart_bench\OBJ\UART_BENCH.hex`。本次没有生成 HEX，也不能把 `reference` 中厂家旧 HEX 当成新结果。
- 参考厂家“程序的下载”教程，用 SWD/ST-Link 或板载 Type-C + FlyMCU 下载本次编译结果；退出下载程序并恢复正常应用启动。打开 PC 脚本前不要停留在调试暂停状态。
- 本工程仅编译 CMSIS、所需标准库、通信程序；PC6..PC9 被设为低电平 GPIO，不启动 TIM8、IMU、平衡控制或蓝牙。断开电机是本阶段接线条件。
- `bench_src` 是通信源模板；`prepare_bench.py` 可从 reference 再生成工程，但会拒绝覆盖现有目录。已生成工程内的 `USER/pc_link.c` 才是当前 Keil 编译对象，后续编辑不要混淆副本。

### A3. PC 环境与可直接运行的测试

在 PowerShell 中运行；`COM7` 必须替换为实际枚举结果：

```powershell
Set-Location 'D:\DIP\Simulation'
py -3 -m venv .venv-sim2real
.\.venv-sim2real\Scripts\python.exe -m pip install pyserial==3.5
.\.venv-sim2real\Scripts\python.exe .\sim2real\serial_bench.py --self-test
.\.venv-sim2real\Scripts\python.exe .\sim2real\serial_bench.py --list
.\.venv-sim2real\Scripts\python.exe .\sim2real\serial_bench.py --port COM7 --duration 60
.\.venv-sim2real\Scripts\python.exe .\sim2real\serial_bench.py --port COM7 --duration 600 --rate 50 --output .\sim2real\link_10min.json
```

若 `py -3` 找不到 Python，安装 Python 3.10+ 或换成你本机实际 Python 可执行文件。当前会话协议自测使用了 Codex 自带 Python，没有修改项目原有 Python 环境。

脚本先检查 `S2RB` 固件签名，再连续发送包含递增编号及随机数据的 48-byte PING，由 MCU 逐字节内容原样回送。它匹配序列号和完整 payload，统计丢包、异常响应、CRC 错误、MCU UART 错误、RX 溢出和往返延迟。没有发送 ARM、PWM 或运动命令。

正常门槛：10 分钟、请求 50 Hz（约 30000 帧，实际发送率会记录）、`sent == matched`、全部错误计数为 0、RTT P99 ≤ 50 ms、程序返回码 0。50 ms 是本方案的上位机通信验收目标，不是芯片厂商保证，也不是平衡环允许的延迟。建议实际发送率至少 45 Hz，否则排查 PC 负载并重测。

如果 Type-C 自动下载电路影响运行：脚本在打开端口前设置 DTR/RTS=False，但部分驱动可能短暂切换这些信号；先结束下载工具、保持正常启动、重新开端口测试。仍失败则用 SWD 固定烧录流程，或使用空闲 UART2 + 3.3 V USB-TTL（模块 TX→PA3、模块 RX←PA2、共地、不接模块 VCC），此时同步修改两端 UART 配置，并断开占用 UART2 的 K210。

[pySerial 官方文档](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)说明了 read 超时可能返回不足请求长度的数据、write_timeout，以及打开串口时 RTS/DTR 可能出现跳变；因此脚本使用持续分帧解析，不能假设一次 read 就是一帧。

### A4. 已实现的帧格式

所有多字节数为 little-endian；不直接发送 C struct 内存。

| 字段 | 长度 | 含义 |
|---|---:|---|
| SOF | 2 | AA 55 |
| version | 1 | 1 |
| type | 1 | PING=01、INFO=02、PONG=81、INFO_REPLY=82 |
| sequence | 2 | 请求序号；回应回显；16 位回绕 |
| payload_length | 2 | 0..64 |
| tick_us | 4 | MCU 时间；验收固件分辨率 1 ms，按 µs 编码、uint32 回绕 |
| payload | N | PING 数据原样回送；INFO 请求为空 |
| CRC16 | 2 | CRC-16/CCITT-FALSE；poly=1021、init=FFFF、无反射、xorout=0；覆盖 version 至 payload，CRC 按低字节先发 |

INFO_REPLY 的 20-byte payload：ASCII `S2RB` + `u32 firmware_version` + `u32 rx_overflow` + `u32 uart_errors` + `u32 crc_errors`。`123456789` 的 CRC 校验值为 `29B1`。

MCU 使用 512-byte RX 中断环形缓冲、DMA1 Channel4 TX；发送缓冲在 DMA 完成前不复用。验收固件没有实时平衡中断，因此它通过测试还不能证明正式平衡固件也能承受通信负载。

## 5. 执行顺序 B：保留本地平衡，加入正式遥测和指令

通过 A 阶段后，在 Simulation 新建 `firmware_balance` 工作副本，选取已经能在本车平衡的 PID 工程。若还没有这一基线，先依据基础平衡教程完成它。厂家 LQR 版本包含第 3 节问题，不建议作为未经修正的直接下地基线。

正式固件的调度：

```text
MPU6050 数据就绪 / 200 Hz
  → 取 IMU + 编码器快照 → 状态估计 → 读取最新有效目标
  → PID / 已验证 LQR → 最终 PWM 限幅 → 写电机 → 保存遥测快照

主循环 / 后台
  → UART 收包及 CRC → 校验命令 → 双缓冲发布目标 → DMA 发遥测
  → 低频 OLED、日志、通信统计

独立时基 + 看门狗
  → 检查 IMU 陈旧、控制超期、低压、倾倒、通信超时
```

- 先保留 5 ms 内环。代码耗时用 GPIO 翻转+逻辑分析仪或独立计时器测量；目标最坏耗时 < 2.5 ms，绝不漏掉 5 ms 截止期。ARM Cortex-M3 没有硬件 FPU，8 次左右的 float 乘加通常不是主要开销，仍需实测完整 ISR 耗时。
- 正式版本推荐 USART1 RX 改为 DMA1 Channel5 循环接收；用 IDLE/半满/满事件记录生产位置，主循环解析。必须追踪生产计数/溢出，不能只比较读写索引而漏掉整圈覆盖。启动前检查其他外设 DMA 分配冲突；本地 `RM0008...EN.pdf` 有 DMA1 request mapping。
- 建议优先级（分组 2）：控制=0、通信错误/接收事件=1、TX完成=2、低频任务=3。通信中断只搬数据/标记事件，不进行字符串处理。独立超时保护不能只放在 IMU 中断里，否则 IMU 停止就失效。
- 关闭 USART1 文本 printf；调试信息放独立通道或作为有类型的帧。`bsp_usart2.c` 的存在不代表应混用 USART2 与 Type-C。
- 加入协议处理时，同时移除旧的 USART1_IRQHandler，保证一个中断向量只有一个实现。正式控制的状态快照使用短临界区/双缓冲，避免角度来自本次采样、编码器来自上次采样。
- PC 是唯一运动目标发布者；禁用蓝牙、超声波等旧任务对 `g_newcarstate` 或目标值的自动覆盖。保留本地 KEY1 作为使能/停机输入。

建议正式协议扩展如下；**这些类型尚未在本次验收固件实现**：

| 类型 | 载荷/行为 |
|---|---|
| HELLO | 协议版本、固件版本、MCU 型号、boot_id、状态/动作单位、周期、参数版本；建立新 session，默认停机 |
| ARM / DISARM | 必须 session 正确；ARM 还需本地按键、姿态正常、IMU 新鲜、无故障；DISARM 令 PWM=0 并锁存 |
| SET_TARGET | `u32 session_id, u32 command_id, u32 observed_mcu_tick, f32 v, f32 yaw_rate, u16 valid_ms, u16 reserved`；24 bytes |
| ACK / NACK | 回显 session/command_id、状态码和**实际接受**的 v、yaw_rate、参数版本；不能只回复“收到” |
| TELEMETRY | 下述 40 bytes，100 Hz；帧时间戳为采样时间 |
| PARAM_STAGE / COMMIT / READBACK | 分包暂存、整包 CRC、版本和范围校验，DISARM 状态原子切换；回读核对，保留已验证版本 |
| FAULT / STATS | 倾倒、低压、IMU陈旧、截止期错失、掉线、CRC/RX溢出/TX丢帧统计 |

40-byte TELEMETRY 的明确布局：

```text
u32 sample_id
f32 pitch_rad, pitch_rate_rad_s, forward_m_s, yaw_rate_rad_s
i16 encoder_left_delta, encoder_right_delta
i16 pwm_left_applied, pwm_right_applied
u16 battery_mV, fault_flags
u32 last_applied_command_id
u16 control_exec_us
u8 mode, reserved
```

带宽预算按 8N1 的每字节 10 bit 计算：115200 baud 每方向 11520 byte/s；54-byte 遥测 ×100 Hz =5400 byte/s，占上行约 47%；38-byte 目标帧 ×20 Hz =760 byte/s。全双工两方向分别核算。识别数据需要 200 Hz 全量记录时，54×200=10800 byte/s，已经占 94%，应先将双方升到 230400 并重做验收，或采用 MCU 缓存后下载。不能把 100 Hz 遥测插值成 200 Hz 来声称获得真实 5 ms 转移数据。

可靠性规则：CRC 错误、长度错误、非法数值（含 NaN/Inf）、未知 session、过期或重复命令均不进入控制器；重复事务返回既有 ACK。SET_TARGET 引用最近的 MCU 遥测时间，MCU 用自己的时钟检查年龄和有效期，拒收串口队列中积压的旧指令；不要用未同步的 PC 时间直接减 MCU 时间。

PC 以 20 Hz 发送当前目标，序号持续变化；无新命令在 100 ms 内目标过期归零，150 ms 通信超时标记 LINK_LOSS。**通信断开但 IMU/控制仍正常时，本地控制器继续零速平衡**。IMU 陈旧 >15 ms、严重倾倒、低压或控制异常时切断 PWM 并锁存故障；恢复连接不会自动 ARM。15/100/150 ms 为初始工程阈值，需在台架验证。硬件看门狗只在控制和安全任务都完成后喂狗。

倾角初期建议：仅在距已标定平衡角 ±5° 内允许 ARM；超过约 10° 清运动目标；超过约 25°停机锁存。初始移动范围可取 ±0.10 m/s、±0.30 rad/s，设置速度斜坡。它们是测试起点，不能作为未经试验的稳定性保证。不要照搬 Demo 的 ±1.20 m/s、±1.50 rad/s 上限。

## 6. 执行顺序 C：校准模型与统一状态、动作

### C1. 传感器与机械参数

1. 电机脱开/车轮架空，启动遥测但保持 DISARM。静止采样数秒，记录陀螺零偏和噪声；原始有符号 16-bit 数据正确扩展，比例系数以实际量程寄存器为准。
2. 定义 X 前、Y 左、Z 上。手动前倾时 pitch>0、pitch_rate>0；俯视左转时 yaw_rate>0。厂家命名 `gyro_x` 与 MuJoCo `gyro[1]` 指不同传感器安装轴，必须校准安装旋转/符号，不能按数组下标机械复制。
3. 在 ±2000°/s 配置下，原始 gyro counts /16.4 得到 °/s，再 ×π/180 得 rad/s。若量程改变必须改变比例。优先使用校准的陀螺角速度，避免差分俯仰角放大噪声。
4. 每轮缓慢正转、反转各 10 圈，累计编码器计数/10 得 N_left、N_right，重复确认。计数由控制周期读取后复位时，要累计每次增量，不能转一圈后只读末尾 CNT。
5. 测量负载下滚动周长/轮半径、轮距、总质量和重心位置；Matlab 参数只作初值。平衡零角也应实测，不能直接使用厂家 0.0349 rad。

统一计算（r、b 用 m，dt 用 s，编码器方向已校正）：

```text
v_left  = 2πr × delta_left  / (N_left  × dt)
v_right = 2πr × delta_right / (N_right × dt)
v       = 0.5 × (v_left + v_right)
yaw_encoder = (v_right - v_left) / b
x = [pitch_rad, pitch_rate_rad_s, v_m_s, yaw_rate_rad_s]
```

这是近直立的编码器里程速度定义。编码器测得的是轮相对机身转动；如加入机身俯仰转速补偿，两端都要一致。航向速率优先用校准 gyro_z，编码器用于校核；不能把轮滑误差当成真实车体运动。仿真中应构造同样的虚拟 IMU/编码器观测。

### C2. 电机动作：推荐重新设计为有符号 PWM 占空比

当前 `<motor gear="1">` 对转动关节给力矩，`ctrlrange=-4..4` 不是 PWM 比例。MuJoCo 的控制、执行器输出、传动映射含义见 [MuJoCo 官方计算文档](https://mujoco.readthedocs.io/en/stable/computation/index.html#actuation-model)及 [XML motor 定义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-motor)。

本车现有代码没有闭合的电流/力矩控制环，因此建议新策略动作直接定义为归一化实际占空比：

```text
d_left, d_right ∈ [-d_max, d_max]
u_common = (d_left+d_right)/2
u_diff   = (d_right-d_left)/2
d_left   = u_common-u_diff
d_right  = u_common+u_diff
CCR = round(abs(d) × (ARR+1))，最终受批准占空比与硬件周期共同限幅
```

完整流程是：电机方向/死区测量 → 辨识电压、转速与动作的关系 → 在 MuJoCo 为 PWM 动作加入电机动态 → 在这个动作定义下重新生成 K → 导出 STM32。**不得把原力矩 K 的输出直接除以 4 后当 PWM，也不得沿用原动作尺度下的 R 权重。**

电机模型可从 `L·di/dt = d·Vbat - R·i - Ke·N·omega`、`tau_wheel = eta·N·Kt·i - loss` 起步，加入限流/饱和、死区、反向间隙、延迟；参数需厂家电机规格和实验拟合。未测电流时不要将估计力矩命名为实测力矩。可将模型力矩送到现有 MuJoCo motor，也可采用自定义执行器，但两端策略接口均用 PWM 动作。

开始测量时断开地面负载、使用支架、逐步提高短脉冲，记录左右轮起转/反转死区和电池电压；不要做持续堵转。下地平衡所需输出权威必须由基线验证，不能随意把平衡控制最大占空比压到不足以回正的数值。

若数据中的动作记录的是**死区补偿和限幅后的实际 PWM**，策略部署和仿真执行器应使用同一输入定义，不可部署时再次叠加死区而改变已辨识动作。

### C3. 从真实数据重新辨识、导出固定控制器

1. 用已验证 PID 在支撑条件下运行，连续记录 200 Hz 的 `(x_k, u_applied_k, x_{k+1}, dt, Vbat, fault)`；记录施加于 k→k+1 区间的实际动作，避免错位一拍。
2. 在基线周围叠加小幅、限时、独立的共模/差模激励，提供足够可辨识性；不要在无控制器状态随机试错。通信丢样、故障/超期、动作定义改变的段落不组成连续转移对；饱和段应标记并单独评估。
3. 分开训练集与验证集，拟合 5 ms 的 `x_next=A*x+B*u+c`。`learned_lqr.py:106` 的 from_transitions 可复用计算思路，但其动作/单位/Q/R/工作点处理必须按新设计修改。回归矩阵满秩、可控秩=4 和 `rho(A-BK)<1` 只是一部分检查；同时检查条件数、独立轨迹预测误差、不同电压/载荷上的闭环表现。
4. 在与实际尺寸、PWM 电机模型、传感器处理一致的 MuJoCo 模型上，重放工况并测试质量、重心、摩擦、噪声、延迟和低电压变化。物理步可改为 1 ms、控制每 5 步；重新生成 K，不能把原 20 ms/8 ms 矩阵拿来按 5 ms 执行。
5. 导出 `float K[2][4]`、x_eq、u_eq、单位/符号、动作定义、周期、模型版本、参数版本与 CRC。STM32 每周期仅做 `u=u_eq-K*(x-x_ref)`、共差模分配、限幅和保护，矩阵求解留在 PC。
6. 用相同输入样本离线比较 Python 与 C 的输出，检查 float 精度、符号、状态顺序、左右轮顺序、限幅；随后在 MCU 上做 SHADOW：计算新 LQR 并记录，电机仍由 PID 控制。
7. 候选通过仿真和影子评估后，停机切换到 LQR，在支架/保护条件下短时验证。混合两个“各自稳定”的控制器不自动保证过渡稳定；不要把简单混合当成理论保证。

如需要跟踪非零速度，不应只机械覆盖 x_eq 的速度项；根据目标速度/航向率计算可实现参考工作点及前馈，或增加经验证的积分跟踪设计。任何新增积分状态都需要同时改变模型、K 维度和 STM32 接口。

## 7. PC 软件接入现有实验台

建议新增 Backend 接口，而不是在每次 `mj_step` 中等待串口：

```text
SimBackend:      step(), read_state(), set_target()
HardwareBackend: read_state(), set_target(), arm(), disarm(), get_link_status()
```

- `HardwareBackend` 用独立串口工作线程，发送受限目标、维护 session 和 ACK，接收完整遥测后原子更新最新状态。界面线程不读写串口。
- 界面沿用“请求值 / 板端接受值 / 测量值”三者分开显示。只有收到 ACK 并看到 `last_applied_command_id` 更新，才标注为已经执行。
- 仿真和实车日志加 `source=simulation/hardware`、固件/模型/参数版本、MCU采样时间、PC接收时间、丢帧及故障字段；不能把模拟力矩当成实车力矩。
- 实车模式下“摩擦/坡度/虚拟推力/位姿 reset/从零策略 relearn”禁用或明确只作用于旁边的仿真。MuJoCo 可显示由里程估计的数字小车，但屏幕轨迹只是估计，不能当成外部定位真值。
- LLM 保留为离线分析与参数建议工具；固件端始终独立检查范围、版本和使能条件，网络/API 延迟不进入 5 ms 控制环。

可选 HIL：另设电机强制关闭的 HIL 模式，PC 逐步发送模拟状态，STM32 返回实际 C 控制器输出，再由 PC 推进 MuJoCo 一个 5 ms 控制间隔。先采用锁步验证数值与协议，不能据此宣称 Windows 串口已经达到硬实时 200 Hz。HIL 状态帧与实车传感器必须由明确模式隔离。

## 8. 必须通过的验收表

| 阶段 | 怎么做 | 通过条件 |
|---|---|---|
| 协议软件 | `serial_bench.py --self-test` | 已通过 CRC 标准向量、任意分块/拼接、坏 CRC 恢复、非法长度、噪声和帧头嵌入 |
| 电气/驱动 | 电机脱开，枚举 COM，PING/INFO | 正确 S2RB 签名、完整往返；只是本阶段通过 |
| 持续收发 | 10 分钟约 50 Hz PING | 无未匹配响应/丢包/CRC/UART/缓冲溢出，P99≤50 ms；记录实际速率 |
| 正式混合负载 | 本地 200 Hz 控制 +100 Hz遥测 +20 Hz目标，至少10分钟 | MCU采样序号连续；PC发现任何掉样均记录；RX无溢出；控制截止期错失=0；P99满足目标 |
| 数据真实性 | 手动前倾、左转、旋转左右轮并比对 | 单位、方向、计数和时间戳一致；目标 ACK 与实际应用编号一致 |
| 帧异常 | 注入 CRC 错误、非法长度、NaN、重复/过期命令 | 不改变目标/增益；计数器增加；下一有效帧自动恢复 |
| 断连 | 拔 USB、停止 PC 进程、让 PC 停发目标 | 目标按有效期归零；健康本地环继续零速平衡；状态转 LINK_LOSS |
| MCU故障 | 受控测试 IMU数据停止/控制超期 | 独立超时令 PWM关闭，故障锁存；不能依赖已停止的 IMU ISR |
| 重连/重启 | PC重连、MCU reset、重复打开COM | 旧命令不执行，重新握手，必须重新本地使能；验证 Type-C 的 DTR/RTS影响 |
| 控制器等价 | Python与C对同一批状态计算 | 在预定浮点误差内一致，饱和和故障输出相同 |
| 下地闭环 | 静止→0.05 m/s→0.10 m/s→小转向，逐级短时测试 | 无持续振荡/故障；能停止；再增加载荷和轻微扰动 |

串口自测不能代替正式混合负载测试；PC/STM32成功传字节也不能代替命令已正确执行。实车稳定性和通信可靠性必须分别留下日志证据。

## 9. 当前还需要你的实物信息

源码和原理图已在指定目录找到，没有因缺资料更改源目录。本会话通过 .NET 串口枚举未返回 COM 名称；WMI 查询受运行权限限制，不能据此判断小车一定没有连接。

要继续完成实车集成/验收，请提供：

- **已装小车的主板与 MCU 丝印照片、所装配件/电池配置**，用于核对 STM32/APM32、重心与接口占用；上传照片即可。
- **当前可平衡的固件工程或所用厂家案例路径**，用于保留硬件已验证基线；如为修改版，请复制到 `D:\DIP\Simulation`。
- **接上 Type-C 后的 COM 号/设备管理器截图，以及本机 Keil 编译器实际路径或版本**，用于运行测试和编译。已检查 PATH、Codex bundled dependencies、常见 C:/D: Keil_v5 路径，未找到可用 ARM 编译器；可安装厂家资源包内所需工具链或提供实际安装位置。

这些信息决定接线和编译对象，不能从仿真文件推断。先完成 A 阶段便可以明确回答本车和本机是否已建立可靠双向链路。
