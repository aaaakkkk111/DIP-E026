# 串口调试 —— 三行接进去

USART1 已经在 `bsp.c` 里初始化成 115200 了（`uart_init(115200)`），`printf`
也已经重定向到它。这两个文件只是把遥测和在线改参数挂上去。

**先做这个，再走上车清单。** 它把每一步从「改代码 → 编译 → 烧录 → 复位」
变成「敲一行命令」。

---

## 接线（三处必接，一处选接）

### 1. `APP/app_control.c` —— 200 Hz 中断里

```c
#include "debug_uart.h"          /* 文件顶部 */

    /* ... 在 EXTI15_10_IRQHandler 里 ... */
    LD_Tick();
    DBG_Tick();                  /* <<< 加这行，在 LD_Tick 后面 */
    Balance_Pwm = Balance_PD(Angle_Balance, Gyro_Balance);
```

### 2. `USER/main.c` —— 主循环里

```c
#include "debug_uart.h"          /* 文件顶部 */

int main(void)
{
    /* ... 原有初始化 ... */
    DBG_Init();                  /* <<< 加这行，在 uart_init 之后 */

    while(1)
    {
        DBG_Poll();              /* <<< 加这行 */
        /* ... 原有代码 ... */
    }
}
```

### 3. `BSP/Usart1/usart.c` —— 接收中断里

```c
void USART1_IRQHandler(void)
{
    uint8_t Rx1_Temp = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        Rx1_Temp = USART_ReceiveData(USART1);
        DBG_RxByte(Rx1_Temp);        /* <<< 加这行 */
        USART1_Send_U8(Rx1_Temp);    /* 原来的回显，想留就留 */
    }
}
```

### 4.（选接）低频前后晃的读数

不接也能跑，只是 `osc` / `rev` 两列恒为 0。**你说的「振荡太离谱」就是这两列
要量的东西**，`rms` 那一列量不到它 —— `rms` 是 26~46 Hz 的带通，一秒晃一次
的车在那个频段上是平的。

在 `DBG_Tick()` 旁边加一行，把两个轮子的位置和传进去：

```c
    LD_Tick();
    LD_Pos((float)(Encoder_Left + Encoder_Right));   /* <<< 选接 */
    DBG_Tick();
```

变量名按你工程里实际的来 —— 任何「随走的距离单调增加」的计数器都行。
读数是相对的，只能同一台车前后比，不能跨车比。

> 为什么 `LD_Pos` 要传参数而不是自己去读全局：好几个 Yahboom 版本里位置
> 累加量是 `Velocity()` 函数里的 `static` 局部变量，extern 不到。

把 `debug_uart.c` 加进 Keil 工程（右键 `APP` 组 → Add Existing Files）。

---

## 为什么不会拖慢控制环

中断里**不做任何发送**，只往环形缓冲写字节；发送在主循环的 `DBG_Poll()`
里一次一个字节。

- 文本模式：每 20 拍一次 `sprintf`，其余 19 拍什么都不做
- 二进制模式：每拍写 10 个字节进缓冲，没有格式化

缓冲满了就丢，**永不阻塞中断**。丢包率在录数据时会打印出来。

---

## 命令

串口助手连 115200，敲命令回车。

| 命令 | 作用 | 例 |
|---|---|---|
| `?` | 打印所有当前值 | `?` |
| `p` | Balance_Kp | `p 28800` |
| `d` | Balance_Kd | `d 48` |
| `v` | Velocity_Kp | `v 8200` |
| `i` | Velocity_Ki | `i 69` |
| `l` | LD_LOADED_BELOW（检测阈值） | `l 7.5` |
| `e` | LD_EMPTY_ABOVE（检测阈值） | `e 21.9` |
| `m` | 0=检测器决定 1=锁 NORMAL 2=锁 HEAVY | `m 1` |
| `t` | 0=关 1=文本10Hz 2=二进制200Hz | `t 1` |

数值用固件的 ×100 整数形式（和 `pid_control.c` 一样），只有 `l` 和 `e`
是 deg/s、可以带小数。

### `p/d/v/i` 改的是「当前生效的那一档」，不是全局变量

`LD_Tick()` 每个 tick 把六个增益从档位表里重写一遍。所以直接写
`Balance_Kp` 这类全局量，**5 毫秒后就被覆盖掉**了。

> 这是个真 bug，第一版就是这么写的：命令回 `ok`，车一点变化都没有。
> 编译验证：
> ```
> 直接写全局   Balance_Kp = 12345
> 过一个 tick  Balance_Kp = 28800   <- 没了
> 改成写表     Balance_Kp = 12345   <- 活下来
> ```

现在 `p/d/v/i` 写的是档位表里**当前生效的那一档**。`?` 会把两档都打出来，
并在正在编辑的那一档后面标 `<- editing`：

```
NORMAL  p 22400  d 48  v 6400  i 54   <- editing
HEAVY   p 28800  d 120  v 11600  i 90
```

**所以调参前先 `m 1` 或 `m 2` 锁住档位**，否则检测器可能在你手底下切档，
下一条 `p` 就落到另一张表里去了。

`m` 也一并修了：旧版只改 `Balance_Kd`，`Velocity_Kp/Ki` 还留在另一档的
值上 —— 那是个从没被测过的混合档。现在 `m` 由 `LD_Tick()` 自己认，六个
增益整档切。

**改动不保存。**复位就回到编译时的值——所以不用担心敲错数字把车弄坏。

---

## PC 端

```
pip install pyserial
```

```bat
python scripts\capture_uart.py --list              :: 找串口
python scripts\capture_uart.py COM5                :: 交互终端
python scripts\capture_uart.py COM5 --record 60 --out empty.npz
python scripts\capture_uart.py --fft empty.npz     :: 看频谱
```

### 自动走一遍增益阶梯

```bat
python scripts\capture_uart.py COM5 --tune
python scripts\capture_uart.py COM5 --tune --rungs 288:82:0.69,240:82:0.69,224:82:0.69
python scripts\capture_uart.py COM5 --tune --force 2 --secs 30
```

每一格自动改增益 → 等 4 秒站定 → 收 20 秒遥测 → 算 `rms` / `osc` / `rev` /
倾角，最后打一张表。车放地上，人在旁边扶着。Ctrl-C 或者跑完都会自动恢复
第一格的值并交回自动切档（`m 0`）。

**这就是孪生做不了的那一步。** 孪生排不了「停振快慢、稳不稳」这种暂态
质量 —— 实测上升时间差 3 倍、超调从 0% 到 85%。存活率它算得准，暂态它
算不准，而你抱怨的正是暂态。所以最后一格必须在真车上选。

把那张表贴回来就行。

---

## 这东西主要解决三件事

### 1. 阈值标定不用再接调试器

原来要在 Keil 的 Watch 窗口看 `ld_rms`，车得挂着调试线。现在：

```
t 1
```

车无线跑，PC 上直接刷数。四次读数（空车/2kg × NORMAL/HEAVY）几分钟搞定。

### 2. 上车清单从几小时变几分钟

清单里「手动固定 G_NORMAL」「手动固定 G_HEAVY」原来各要改代码重烧，现在：

```
m 1        锁 NORMAL，试
m 2        锁 HEAVY，试
m 0        放开，让检测器自己判
```

三套死区版本也不用挑了，直接 `p` / `d` / `v` / `i` 敲进去比。

### 3. 最要紧的一件：验证带通频段选对没有

`load_ctrl` 的带通是 **26.5–45.5 Hz**，因为**孪生里**空车抖动主频是 31 Hz。

**真车主频是多少，从来没人量过。** 如果真车在 20 Hz 或 50 Hz，这个带通会把
信号滤掉，检测器直接失效——而这是烧上去之前唯一能提前排掉的重大风险。

```
m 2                                    锁 HEAVY（空车下会抖，那正是要测的）
python scripts\capture_uart.py COM5 --record 30 --out empty_heavy.npz
```

扶着车，别让它摔。脚本会直接告诉你主频和能量分布：

```
  陀螺 rms 40.0 deg/s     主频 31.0 Hz
  26.5~45.5 Hz（load_ctrl 的带通）占总能量 99.8%
```

主频落在 26.5–45.5 之外的话，脚本会报警，那三套文件的带通系数都得重算。

---

## 顺带能解决项目里最老的一个问题

孪生至今复现不了你说的那个 **40 mm / 1.4 秒**来回摆。我试过 300 组五参数
联合搜索，幅值和倾角能对上，**周期从来没低于 0.16 秒**。结论是缺**结构**
不是缺参数——但缺什么结构，光靠仿真猜不出来。

录一分钟真车原始波形（`t 2`，200 Hz，陀螺+倾角+左右 PWM）就能直接看出来。
这比再做十轮仿真都有用，也是让所有那些「孪生里测的」数字变得可信的唯一办法。

```
m 0
python scripts\capture_uart.py COM5 --record 60 --out real_idle.npz
```
