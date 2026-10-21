# RA3.0-SEQ1 逐项更新协议

UART5保持9600 8N1。ASCII完整帧 `$BODY,Cxxxx#`，CRC-16/CCITT-FALSE覆盖BODY。接收含首尾字符小于80字节；ONE短帧通常约30~35字节。

## 顺序

1. `P3,BSEQ,id,30000,M|T,N|L`：人工M/训练T，目标配置N/L；建立事务并保存当前完整确认PID。返回标准ACK SEQ_BEGIN。
2. `P3,ONE,id,step,AP|AD|VP|VI|TP|TD,value`：每帧仅一键一值。step从1连续递增，最多6项。只接受有限、范围内、最多两位小数的十进制数字；未知键、跳步、空值、指数和超长报文拒绝。未选系数不改。
3. ONE_QUEUED仅表示排队；下一次控制边界只写目标全局变量，主循环返回 `P3,SEQACK,ONE_APPLIED,id,step,key,value`。
4. 网页发 `P3,SGET,id,step`，返回 `P3,STATUS,AP,AD,VP,VI,TP,TD,PN|PL,S...,L...,B...,Qid,Istep`。必须对应同一id/step，并且六项实际值、当前profile和状态匹配预期，才能确认本项。
5. 确认后等待1000ms，再发下一项。所有数值只发送已改变的项，六项全变也必须六个独立ONE。
6. `P3,FSEQ,id`：全部项结束后提交目标N/L配置。返回SEQ_FINISH_QUEUED；控制边界切换profile，再返回标准ACK SEQ_DONE。网页以SGET,id,7核对全部最终实际值。
7. 人工事务结束即保存完整新确认参数并回S0；训练事务保留S2，等待窗口评价后ACCEPT或ROLLBACK。部分序列不允许ACCEPT。

示例ONE：

`$P3,ONE,12345,1,AP,95.00,C571E#`

## 其他命令

- GET查询（无关联Q0/I0）、PROBE通信测试、DIAG电机诊断（仅尚未启动平衡）。
- TRAIN START/STOP建立/解除人工参数锁。
- HB,id每500ms；固件活动状态超过2秒没心跳或30秒TTL到期，恢复整组序列前的确认参数；FSEQ后为评价重新计30秒TTL。
- ACCEPT,id：只能完整序列结束后接受；ROLLBACK,id：恢复整组PID与原N/L，保留积分、滤波；STOP,id停止运动目标。
- 旧MSET/PREP/APPLY整包更新返回SEQ_REQUIRED，不再走六参数浮点解析。
- 同一步已完成的完全相同ONE可重发完成ACK，不重复赋值；网页本版不盲目自动重试。序列传输期间的HB只刷新时间，不额外回ACK，降低9600baud占用；评价阶段保留ACK。

## 遥测

沿用RA3.0 T3,F约5Hz、T3,S约1Hz，以及osc/lean计算和统计。P3 STATUS、GET和T3慢帧仍显示完整六项，这些是设备读回数据，不是网页批量写入。

AP/VP/TP由固件在主循环预乘100一次，AD/VI/TD保持协议值；控制ISR只做一项赋值。N/L固定倍率与原RA3.0一致，仅在FSEQ末尾显式切换，切换倍率不是LLM随意写入的参数。

不增加实时启动锁停。异常PID读回返回PID_DATA，网页不显示该值为有效PID，并取消后续写入。部分更新试验期间仍采用原RA3.0的倾角/电压/PWM风险回滚。回滚是安全恢复动作，由STM32本地恢复完整确认向量，不等待手机逐项发回。
