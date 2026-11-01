# Web v3.6 · 中文 / English

在页首选择中文或English。选择保存在本机；也可用?lang=en或?lang=zh直接打开指定语言。语言切换仅更新显示，不断开BLE、不停止自动调参、不改PID、不清空输入和API Key。

两个界面共用同一套状态、按钮、通信、六PID写入、自动调参、3秒/1秒窗口和接受/回滚逻辑。静态内容、动态状态、错误、确认框、曲线标签、训练进度及系统事务消息均提供英文。英文模式下后续LLM请求只把解释字段的输出语言改为英文，经验、数据、候选格式和评价规则保持一致。已有模型解释和原始JSON/收发报文保留原文；切换语言不翻译或改写历史原始数据。

此版本只更新网页，无需重新烧录；继续使用RA1.9-AUTO6-TXFIX-VP100-NOGUARD。固件及HEX校验和保持不变。

发布地址：https://aaaakkkk111.github.io/DIP-E026/?v=I18N-3.6
直接英文：https://aaaakkkk111.github.io/DIP-E026/?v=I18N-3.6&lang=en
单文件：releases/routea_bluefy.html（中英文均在同一个文件内，无外部语言包或翻译服务依赖）。
