# RA3.3-DIRECT：P3/T3

ASCII：`$BODY,Cxxxx#`，CRC-16/CCITT-FALSE（0xFFFF，0x1021）。UART5 9600、8N1，接收帧含首尾字符必须小于80字节。

## 命令体（省略$和CRC尾）

- `P3,PROBE`：只确认链路，不改参数、状态或运动。
- `P3,GET`：STATUS实际PID、profile、事务S、训练锁L和平衡启动B。
- `P3,MSET,N,AP,AD,VP,VI,TP,TD,M1`：普通人工更新，N必须等于实际配置；六项为数字位置字段，无AP文字前缀。M1平衡、M2速度、M4转向，组合为按位或，M7全部。
- `P3,PRESET,N` / `P3,PRESET,L`：显式人工预设，使用固件内的官方六项和固定倍率。
- `P3,TRAIN,START` / `P3,TRAIN,STOP`：建立/解除人工参数锁；STOP时如有活动候选，先在控制边界回滚。
- `P3,PREP,id,ttl,N,AP,AD,VP,VI,TP,TD,M1`：LLM普通候选，配置必须不变。
- `P3,PPRESET,id,ttl,L`：LLM官方预设候选，固件生成六项；只有APPLY后才切换参数及倍率。
- `P3,APPLY,id` / `P3,HB,id` / `P3,ACCEPT,id` / `P3,ROLLBACK,id` / `P3,STOP,id`。
- `P3,DIAG,L+|L-|R+|R-|STOP`：只允许尚未启动平衡且未训练时的原有电机诊断。

旧MSET/PREP未带M字段时默认M7；不得用旧网页跨配置调参，新网页要求固件版本RA3.3-DIRECT。普通更新中未选环的数字是填充项，不写入对应系数；最终合并值必须合法。数字仅限无符号整数或最多两位小数，禁止指数、NaN、Infinity、空值和尾随文本。

## 赋值与确认

主循环解析校验和预缩放；AP/VP/TP乘100存入候选。下一次5 ms控制入口调用同一ra_store_raw，只对选中的两项直接赋值。无插值、无积分重置或Ki重算，不改运动命令、机械中值。回滚直接写回完整的上组确认原始内部参数和profile。配置倍率仅随PRESET/PPRESET和相应回滚改变。

人工先返回ACK MSET_QUEUED；控制边界完成后，主循环发送ACK MSET_APPLIED。网页必须再GET并核对实际profile/六项，方可确认成功。排队ACK不是生效确认。事务期间重复人工写入返回BUSY，训练中返回LOCKED。

LLM PREP/PPRESET只暂存S1，ACK PREP；APPLY ACK只代表请求，下一次控制边界进入S2并赋值。网页看到S2后GET，只有STATUS也为S2、profile和候选六项一致才开始稳定期与评价。ROLLBACK先排队，完成返回ROLLED_BACK。状态S0空闲、S1暂存、S2实际试验。

活动事务心跳超过2秒、TTL到期、连续3次倾角绝对值>25度/电压<9.6V、连续40次PWM饱和，会请求回滚。网页另有危险基线否决与试验倾角20度回滚。内存/栈损坏PID_MEMORY或MEMORY_LATCH锁停，本次上电不允许继续调参。

## 遥测

- 约5Hz：`T3,F,ms,A...,G...,O...,N...,EL...,ER...,ML...,MR...,S...,L...,P...,U0`。
- 约1Hz：`T3,S,ms,V...,A2...,G2...,PK...,X...,AP...,AD...,VP...,VI...,TP...,TD...,P...,D...,B...,RA3.3-DIRECT,SF...,DT...,CT...`。

A倾角度、G陀螺仪raw、O=osc、N=lean。每5ms：lean +=0.0025*(angle-lean)，osc +=0.025*(abs(angle-lean)-osc)，使用更新后的lean。特征只上报不选PID。EL/ER编码器，ML/MR PWM；A2/G2为200点均方，PK绝对倾角峰值，X为|PWM|≥2595的样本比例。SF启动以来最小剩余栈字节，DT最大控制IRQ入口间隔微秒，CT最大控制IRQ执行微秒。U固定为0，保留字段兼容解析。D发送队列丢弃次数。

## 官方格式兼容路径（模式21）

完整13字段：`$运动,原地转向,PID操作,自动上报,更新平衡,更新速度,更新转向,AP96.00,AD48.00,VP62.00,VI31.00,TP17.00,TD20.00#`。

使用独立有界解析器，不进入原Protocol()。控制环更新仍走同一直接赋值函数，每个实际更新环返回一个$OK#。训练中拒绝参数修改。查询返回$AP...,AD...,VP...,VI...,TP...,TD...#一次；不像原APP配合的旧实现重复两次。PID操作2显式恢复本工具Normal。自动上报1切换到约5Hz旧遥测LV/RV/AC/GY/CSB/VT，2关闭旧上报并恢复T3；任意有效P3命令也恢复T3。此处AC/GY仍为raw，未验证LV/RV物理单位，不能用本路径宣称完整官方APP兼容。

移动/停止帧更新标志均0，始终使用固定合法填充值，避免故障PID把停止帧撑破80字节缓冲；它不改实际PID。
