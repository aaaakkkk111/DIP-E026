# BalanceBot Route A — iOS / Bluefy 控制台

这是 Android Route A v1.4 功能的单页 Web Bluetooth 移植版，目标设备是
iPhone 15 Plus（iOS 17.5.1）和 Bluefy。它不修改 STM32 固件，按当前
`RA1.5-TEST-MODE` 的 FFE0/FFE1、CRC、遥测和训练事务运行。

直接交付文件是 `routea_bluefy.html`（`index.html` 内容相同）。CSS、协议库和
应用逻辑都已内嵌，部署或传输时只需这一份 HTML。`src`、`template.html`、
`build.mjs` 是可维护源文件。

## 为什么建议 HTTPS，而不是直接点本地文件

Bluefy 为 iOS 提供 Web Bluetooth，但其官方说明要求网页运行在 HTTPS 安全
上下文。iOS“文件”App 可以预览 HTML，不等于 Bluefy 能从 `file://` 页面授予
Bluetooth 权限。若本地打开后页面顶部显示“不安全上下文”或“没有 Web
Bluetooth”，页面本身没有损坏，应改用 HTTPS。

最稳定的部署方式是 GitHub Pages：

1. 把整个 `ios-bluefy` 目录提交到 GitHub 仓库，或者只上传 `index.html`。
2. 在仓库 **Settings → Pages** 选择 **Deploy from a branch**，选择包含该文件的
   分支和目录。若只能选择仓库根目录或 `/docs`，把 `index.html` 放进对应目录。
3. 等待 GitHub 给出 `https://<用户名>.github.io/<仓库>/<目录>/` 地址。
4. 在 iPhone 上安装 **Bluefy – Web BLE Browser**，把这个 HTTPS 地址粘贴到
   Bluefy 地址栏打开。不要用 Safari、iOS Chrome 或“文件”预览来连接。
5. 可把 URL 加入 Bluefy 收藏。更新网页后刷新即可，不需要重新安装 App。

如 Bluefy 的分享菜单确实允许“用 Bluefy 打开”本地 HTML，也可以尝试 AirDrop
`routea_bluefy.html` 到 iPhone 后从“文件”分享给 Bluefy。只有页面设置页日志显示
`secure=true, bluetooth=true` 才进入实车连接；否则仍需 HTTPS。

## 首次连接

1. 烧录并启动现有 Route A v1.5 固件。在 OLED 上选择 **21.TEST MODE**。模式
   1–20 不运行 Route A 服务，网页无法得到 `P1/T1` 状态。
2. 关闭安卓手机的蓝牙或官方 App，避免 JDY-23 被另一个中心设备占用。打开
   iPhone 蓝牙，并允许 Bluefy 使用蓝牙。
3. 在网页“设置”页保持默认值：Service `FFE0`、Characteristic `FFE1`、20 字节
   分片、18 ms 间隔。点 **连接小车**，在系统列表中选择 `YahBoom_BL`（部分模块
   名称前有空格）。
4. 成功标准：顶部变为“BLE 已连接”；日志出现 `FFE0/FFE1 ready`；网页自动发送
   `P1,GET`；PID 页出现六个参数；状态页出现固件版本、倾角和约 5 Hz 遥测。
5. 若只显示已连接但没有 PID，先点 PID 页的“读取”。仍无响应时保存日志；这说明
   GATT 已连但写入或通知链路未闭合。

## 页面结构与操作

### 状态

显示倾角、角速度 raw、左右编码器、电机命令、PWM CCR、电池、固件状态、训练锁、
接收频率和丢包。曲线显示最近 30 秒角度与缩放后的角速度。页面不擅自把固件的
gyro raw 值换算成物理单位。

### 遥控

方向键采用“按住移动、松开停止”。同一时刻只接受一个指针；按下另一方向前会先
停止。页面进入后台、手指取消、连接断开或点“安全断开”都会尝试发送完整停止帧。
悬浮的红色 **紧急停止** 在所有分页可见。

架空电机诊断只在按下小车平衡启动键之前使用。把车架起、两轮离地，再分别点左轮
和右轮正反转；平衡启动、训练锁定或候选试验期间按钮会禁用，固件也会二次拒绝。

### PID

先读取当前值。输入六个协议显示值后，页面显示相对当前值的百分比，并按固件实际
范围检查：AP 20–288、AD 5–200、VP 10–72、VI 1–100、TP 1–100、TD 1–100。
点“应用人工 PID”后，要等 `ACK,MSET` 并读回才算生效。训练锁定后不能人工写入。

### 训练

1. 先用人工 PID 建立可稳定平衡的初始状态，然后按小车 Key1 启动平衡；网页状态
   必须显示“平衡已启动”。
2. 点 **开始训练并锁定**，等待 `ACK,TRAIN_START`。此后人工 PID 被禁用。
3. 点 **开始采集**，让小车完成约 10–30 秒的固定测试，再点 **结束并计算**。
4. 选择 **本地 Harness** 重现仿真的确定性规则，或在设置中填写兼容接口后点
   **调用 LLM**。模型只可建议 AP/AD，VP/VI/TP/TD 必须保持不变，单次变化不得
   超过页面上限和固件 5% 硬上限。模型输出只进入审核框，不会自动发送。
5. 审核候选后点 **PREPARE**；收到固件 ACK 后，APPLY 才可用。确认安全后点
   **APPLY**。页面每 500 ms 发送心跳。
6. 观察试验，满意时 **ACCEPT**；任何异常立即按 **ROLLBACK** 或红色停止。心跳
   超时、TTL、倾角、电压或持续 PWM 饱和也会由固件本地回滚。
7. 点 **停止训练并解锁** 才重新允许人工 PID。

浏览器直连 LLM 时，API 服务必须允许来自网页的 CORS 请求。API Key 只保存在当前
页面内存，不写入 localStorage，也不进入导出日志。若服务端拒绝 CORS，应把 Base
URL 指向用户自己的 HTTPS 代理；本地 Harness 和粘贴严格 JSON 不依赖云端。

### 设置与记录

可以修改 UUID、设备名称前缀、分片参数和 LLM 接口。通信设置保存在当前站点的
localStorage；API Key 不保存。CSV 导出解析后的快速遥测，JSON 导出 PID、指标、
候选、遥测和原始收发日志。

“启动演示模式”不连接实车，用生成的 5 Hz 遥测验证分页、曲线、PID、基线、Harness
和事务按钮。演示通过不能替代 iPhone 与实车的 BLE 验收。

## 实车验收顺序

1. 车轮架空：连接后 PID 自动读回，状态遥测连续 30 秒，无大量 CRC 错误。
2. 平衡启动前：四个电机诊断方向均正确，停止后 PWM 回零。
3. 落地但扶住：按 Key1 后 `balanceStarted=1`，官方基线参数可维持已验证稳定性。
4. 遥控：逐个按住方向键 0.5 秒并松开，每次都能立即停止；多指操作不产生叠加命令。
5. 人工 PID：小幅改动一次，收到 MSET ACK，自动 GET 读回完全一致。
6. 训练锁：TRAIN START 后人工 PID 按钮禁用；固件状态也返回 `L1`。
7. 候选事务：基线不少于 50 个快速样本；PREP 后显示 PREPARED；APPLY 后显示
   TRIAL；ROLLBACK 恢复原 PID。先验收回滚，再尝试 ACCEPT。
8. 断连保护：活动候选期间关闭 Bluefy，固件应在约 2 秒心跳超时后回滚。该机制不
   等同于任何情况下立即停电机，所以始终保留人工扶车和物理断电手段。

## 开发与测试

```text
node build.mjs
node tests/core.test.cjs
node --check src/app.js
```

测试覆盖 CRC、分片/粘包解析、PID 状态、快速遥测、指标、完整 PID 更新帧、移动帧、
LLM schema 和 5%/balance-stage 限制。当前交付已在桌面浏览器完成界面与 Mock 检查；
由于本机没有 iPhone 和小车无线链路，不能声称已经完成 Bluefy 实车射频验证。
