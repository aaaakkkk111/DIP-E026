# Web v3.2 / RA1.9-AUTO6-TXFIX验收

1. 烧录releases/stm32_Balance_Car_RA1_9_AUTO6_TXFIX.hex，Bluefy打开?v=AUTO6-3.2；确认日志版本与固件版本。
2. 3.TEST MODE按现有KEY1流程启动平衡，确认5Hz快遥测、osc/lean、GET/PID正常；Standard/Load作为原对照。
3. 自动调参高级设置默认采集3秒、评价1秒。停止后改为其他1..20秒数并保存，刷新应保留。应用后1秒过渡、轮间默认5秒及LLM等待不计入两个窗口，未修改。
4. 运行至少连续3轮：候选应用必须核对ACK及整组PID；ACCEPT后核对IDLE及实际PID再进入下一轮，不应再因尾随HB的TRIAL,0中断。
5. 普通评价不佳仍回滚并进入下一轮；真实保护/通信异常仍停止。未知事务错误不能被无条件忽略。
6. 导出完整原始收发日志和JSON，报告HEX/Web版本、电压、模式、轮数及是否抖动/漂移/倾倒。1秒评价只是短窗判断，不能据此宣称长期改善。

本次机器验证：网页50项、真实ARM指令6项；Keil0错误2条原有警告。未连接实车、未调用真实付费LLM。完整提示词见LLM_PROMPTS_WEB_V3_2.md。
