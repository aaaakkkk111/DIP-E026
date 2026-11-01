# Web v3.5 / RA1.9-AUTO6-TXFIX-VP100-NOGUARD

按用户要求移除调参专用的动态门槛。本版本需要更新固件和网页两端。

- 网页不再因当前倾角超过25°或计算输出接近2595而阻止APPLY。
- 网页不再因试验倾角超过25°或低电压而主动回滚，审核前/APPLY前的电压门槛也已移除。
- 固件RouteA_ControlCapture只记录遥测，不再按倾角、输出饱和、低电压的连续三个控制周期请求回滚。
- 自动运行设置已删除“最低电压”选项；电池、角度、饱和率仍正常显示/记录并发送给LLM。

继续使用3.TEST MODE、UART5 9600、快遥测5Hz、指标2.5Hz、默认采集3秒/评价1秒、六参数范围和原经验提示词。PREP/APPLY/GET实际读回、训练锁、CRC、事务TTL、心跳超时、用户停止和综合评价J≤1.10规则保持原逻辑。

注意范围：删除的是Route A调参门槛。官方Turn_Off电机启停逻辑（包括原有倾倒/欠压/Stop_Flag条件）和PWM±2600限幅未修改；超过这些条件时电机仍可关闭。网页不会再把25°、饱和或低电压本身当成中止调参的理由，但综合评分仍可因实际指标恶化而回滚。

固件文件：releases/stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100_NOGUARD.hex
Keil工程：firmware/firmware/USER/stm32_Balance_Car.uvprojx
网页文件：releases/routea_bluefy.html（与releases/index.html相同）
启动日志应显示Web v3.5；固件读回应显示RA1.9-AUTO6-TXFIX-VP100-NOGUARD。

测试覆盖用户50/45基线、75/70候选、27.09°时的完整PREP→APPLY→PID读回，以及饱和/低电压条件下不再动态撤销事务。测试为软件和实际ARM二进制仿真，尚未实车验证新版本。
