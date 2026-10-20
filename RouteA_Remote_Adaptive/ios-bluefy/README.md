# Bluefy Remote Adaptive Console

源文件位于src，运行node build.mjs生成单文件
routea_remote_adaptive_bluefy.html。该文件可复制到iPhone后用支持本地文件导入的
Bluefy版本打开，也可以从HTTPS地址在Bluefy中打开。

页面使用FFE0/FFE1 BLE UART，UUID可在设置页修改。自动模式要求页面保持前台并能访问
配置的OpenAI-compatible接口。Mock模式不需要蓝牙或API Key。

测试命令：node tests/core.test.cjs。
