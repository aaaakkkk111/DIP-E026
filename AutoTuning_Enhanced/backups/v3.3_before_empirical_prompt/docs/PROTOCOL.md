# 协议扩展

保持原P1/T1、ASCII `$BODY,Cxxxx#`、CRC16/CCITT-FALSE、UART5 9600 8N1、接收帧含首尾小于80字节。
P1 GET/MSET/TRAIN/PREP/APPLY/HB/ACCEPT/ROLLBACK/DIAG原字段和状态0/1/2不变；MSET与PREP可写六项，均由原ra_write在控制边界整组赋值。
AP/VP/TP固件乘100，AD/VI/TD直接写；不清零积分或滤波。
AP20..288、AD5..200、VP10..72、VI/TP/TD1..100；非有限值拒绝。LLM候选取消“其余四项等于champion”的限制。

新增指标帧：`$T1,E,时间ms,Oosc,Llean,Blean_band,W预热标志,Cxxxx#`。
原T1,F和T1,S字段不变。新增E约2.5Hz；W=0预热，连续600个有效5ms采样后W=1。非法角度/gyro重置滤波。
网页样本附带最近E，仅当600ms内且W1才作为有效指标；连续自动基线/试验需至少90%指标有效。

每5ms：
```
m += .200849*(gyro-m); h=gyro-m;
b += .334511*(h-b); Eg += .016393*(b*b-Eg);
q += .009337*(angle-q); x=angle-q;
l += .059120*(x-l); Ea += .016393*(l*l-Ea);
osc=sqrt(max(Eg,0))/16.4; lean=sqrt(max(Ea,0)); lean_band=l;
```
首个有效采样以当前angle/gyro初始化慢均值。MPU6050原代码设置±2000°/s（16.4 LSB/(°/s)），Gyro_Balance为原始计数，因此osc按°/s显示，gyro遥测仍raw；lean与lean_band为度。
有限阶滤波只有宽松的频带选择性，并非理想8–16Hz/0.3–2Hz砖墙滤波器。指标不直接触发本地PID切档，未使用固定阈值自动升降档。

内部模式ID仍为Normal0、Weight_M3、Test_Mode20；菜单映射为1/2/3，不存在其他可选模式。
Normal/TEST启动PID96/48/62/31/17/20；Load96/75/70/35/14/20。原Load控制倍率2/1.35/1保留。TEST不根据手机命令切换硬件模式。

网页只对版本RA1.9-AUTO6启用写操作；旧RA1.8反馈可读但只读，不能将两套网页固件混用。手机不连接时STM32继续执行当前控制器，不进行LLM优化。

RA1.9-AUTO6-TXFIX：已完成事务的最后一个ID仅用于忽略迟到HB，不修改新事务状态或心跳；未知ID仍返回ERR,TRIAL。网页兼容该版本前缀，结束ACK之后仍以GET/PID一致性确认最终结果。

RA1.9-AUTO6-TXFIX-VP100：VP范围改为10..100，仍按显示值×100写Velocity_Kp。其他协议、保护和控制时序不变。网页按固件版本标记选10..72或10..100。
