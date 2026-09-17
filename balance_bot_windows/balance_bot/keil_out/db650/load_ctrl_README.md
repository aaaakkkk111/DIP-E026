# load_ctrl —— 负载自动切档（电机死区 650 版）

> **这一份是给电机实际死区 ≈ 650 counts 的车用的。**
> 先测你车的死区（方法见文末），再挑对应的目录：
> `db650/` `db900/` `db1300/`。测出来 >1300 的话见文末最后一节。


自动生成于 2026-09-16 14:55，由 `scripts/export_keil.py`。别手改 .c/.h，改这个脚本。

## 加进工程

1. 把 `load_ctrl.c` 和 `load_ctrl.h` 放进 `APP/PID/`
2. Keil 里右键 `APP` 组 → Add Existing Files → 选 `load_ctrl.c`
3. `app_control.c` 顶部加 `#include "load_ctrl.h"`
4. 在 `EXTI15_10_IRQHandler` 里加**一行**，位置是 `Get_Angle()` 之后、
   `Balance_PD()` 之前：

```c
    Get_Angle(GET_Angle_Way);
    Encoder_Left  = Read_Encoder(MOTOR_ID_ML);
    Encoder_Right = -Read_Encoder(MOTOR_ID_MR);
    Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);

    LD_Tick();                    /* <<< 加这一行 */

    Balance_Pwm = Balance_PD(Angle_Balance, Gyro_Balance);
```

5. `main()` 里初始化一次：`LD_Reset();`

`pid_control.c` 顶部那六个 `float` **不用动**——`LD_Tick()` 每拍会覆写它们。
文件本身也不重复定义任何全局，所以不会有重复符号。

**05.weight_control 工程注意**：`Balance_K` / `Velocity_K` / `Turn_K` 保持
1.0。这两套增益已经把负载算进去了，再乘一遍等于把 D 项也放大——那正是原厂
负载模式在空车上剧烈振荡的原因。

## 编译注意

- 注释全是 ASCII。原厂那些 .c 是 GB2312，Keil 按系统代码页读，混 UTF-8 中文
  会乱码。所以中文说明单独放在这个文件里。
- C89 写法：没有 `//`、没有块中声明。armcc 默认模式能过。
- 用了 `sqrt()`（double），不是 `sqrtf`——老版本 armcc 的 `sqrtf` 不一定有。
  每 5 ms 一次，开销可以忽略。

## 阈值标定（必做）

.c 里那两个阈值是**孪生里的数**，余量只有 1.8 倍。真车的陀螺噪声底、机械
间隙、电机响应都不一样。四次读数：

| 步骤 | 做什么 | 读 `ld_rms` |
|---|---|---|
| 1 | 空车站稳（此时在 NORMAL） | `R_empty_N` |
| 2 | 装 2 kg 站稳 | `R_load_N` |
| 3 | 手动置 `ld_heavy = 1`，仍装 2 kg | `R_load_H` |
| 4 | 卸货变空车，**扶住车**（会剧烈抖） | `R_empty_H` |

```
LD_LOADED_BELOW = sqrt(R_empty_N * R_load_N)
LD_EMPTY_ABOVE  = sqrt(R_empty_H * R_load_H)
```

两个变量是非 const 的全局，可以用 Keil 的 Watch 窗口在线改，不用重烧。

**不标的后果**：真车 rms 通常比孪生高，检测器会一直判「空车」、一直停在
NORMAL —— 而 NORMAL 在 4 kg 行驶时会摔。**这一步不能省。**

## 上车清单

每一步不过就停。

0. 先确认编译优化是 **-O1**。-O0 时 0x08010000 以上的 flash 存不住 RW 初值，
   全局变量开机就是乱的。烧完读回 `Balance_Kp`，对一下是不是 28800。
1. **先不接检测器**（不调 `LD_Tick`），手动把六个增益设成 G_NORMAL：
   空车扶着开机、放手、前后左右各开一遍。
2. 手动设成 G_HEAVY，**装 2 kg**，重复。这一步第一次把 Kd 提到 120，
   务必先扶着、确认不抖再放手。
3. G_HEAVY + 4 kg，重复。
4. 做上面的标定，填两个阈值。
5. 接上 `LD_Tick()`。空车开机：应该先抖一两秒（HEAVY 用在空车上），然后自己
   切到 NORMAL 安静下来。**这是正常的，不是故障。**
6. 装 2 kg，停稳几秒，应该自己切到 HEAVY。

## 现象对照

| 现象 | 原因 | 处理 |
|---|---|---|
| 30 Hz 嗡嗡声，NORMAL 下 | Kd 偏大 | `Balance_Kd` 48 → 34，别动 Kp |
| 30 Hz 嗡嗡声，HEAVY 下 | 货不够 2 kg，或真车这个模态比孪生早 | 先查载重；确实够就退回 NORMAL |
| 慢速前后晃，1 秒以上一个来回 | 速度环 | 调 `Velocity_Kp` |
| 起步就往前扑 | Kd 不够 | 该切 HEAVY 了 |
| 档位来回跳 | 阈值没标好 | 重做标定 |
| 停不住，游走几厘米 | 位置环带宽的物理极限 | 见下 |

## 已知没解决的

**车不会真正静止，一直在约 48 mm 范围内游走。**

固件没有位置反馈：`Encoder_Integral` 是速度指令的开环积分，两边都在累积，
误差无处消除。`Velocity_Ki` 只能把游走从 74 mm 压到 48 mm，再往下会压掉
检测器需要的抖动信号。

要根治得加外环：量位置误差 → 算速度指令 → 写 `Move_X`。那是另一件事。

## 怎么测你这台车的死区

源码里 `APP/app_motor.c` 第 5 行：

```c
#define MOTOR_IGNORE_PULSE (1300)//死区  1450 25Khz   此值需要看静止状态微调
```

**厂家自己写明这个数要一车一调**，而且 20 多个工程全用同一个默认值 1300，
没有一台是标定过的。1300/2880 = 45% 占空比，对普通减速电机来说偏大。

十分钟测法：

1. 车**离地**架起来，轮子悬空
2. 临时改成 `#define MOTOR_IGNORE_PULSE (0)`，并注释掉平衡环的调用
3. 直接 `Set_Pwm(n, n)`，n 从 0 每次加 25
4. 记下**轮子刚开始转**的那个 n —— 左右轮分开测，很可能不一样

## 测出来 >1300 怎么办

**改 `MOTOR_IGNORE_PULSE`，不要改 PID。** 死区超过固件补偿量之后，
任何增益都救不回来（实测死区 1700 配 kp 96/144/192/288/384 全部 0/8）。
机制是欠补偿：固件补 1300、电机实际要 1700，小指令出不来力矩，
同时 1700+信号被 2600 的上限截断，可用范围只剩 900 counts。

实测：死区 1700 + 补偿 1300 是 **0/6**；把补偿也改成 1700，立刻变 **5/6**。

所以：把 `MOTOR_IGNORE_PULSE` 改成你实测的值，然后用 `db1300/` 这一套。
