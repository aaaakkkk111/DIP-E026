# BalanceBot 蓝牙链路验收台

## 当前 UART 回环固件的专用页面

如果小车烧录的是 `D:\DIP\Bluetooth\Bluetooth` 工程，请打开
`official-echo-test.html`。该工程只会把 UART5 收到的字节原样发回，不能响应
平衡车 `$...#` 控制、PID 或遥测命令；专用页面会验证 FFE0/FFE1 和逐字节回环，
并明确记录连接失败发生在哪个 GATT 阶段。

本目录将官方 `bluetooth.html` 扩展为 PC 端 BLE 验收和调试页面。机器人端代码及 UART5 的 9600 baud 配置均未修改。

## 能力

- 通过 Web Bluetooth 选择并授权 BLE 设备，连接 FFE0 服务和 FFE1 特征。
- 在 FFE1 上写入现有 `$...#` 指令并订阅通知。
- 用 `$OK#` 和 `$LV...#` 自动验证 PC → STM32 → PC 双向链路。
- 解析 LV、RV、AC、GY、CSB、VT 实时遥测并绘制低负载波形。
- 提供官方协议已有的基础运动控制和 AP/AD/VP/VI PID 调试。
- 按 `$` 与 `#` 重新组帧，允许 BLE 通知拆包、粘包和前导噪声。
- 保存并导出本次会话的原始收发日志。

## 启动

Web Bluetooth 需要 Chrome/Edge 和安全上下文。推荐在本目录启动标准库 HTTP 服务：

```powershell
Set-Location 'D:\DIP\Simulation\bluetooth'
node .\server.js
```

然后用 Windows 版 Chrome 或 Edge 打开：

```text
http://127.0.0.1:8765/bluetooth.html
```

当前 UART 回环固件应打开：

```text
http://127.0.0.1:8765/official-echo-test.html
```

也可以双击 `start-server.cmd` 启动服务。

点击“选择设备并连接”，在浏览器设备选择器中选择 `YahBoom_BL`。部分出厂模块的名称实际带一个前导空格，即 ` YahBoom_BL`；页面已同时兼容这两种名称。如果模块名称仍为默认值，也可选择 `JDY-23`。页面还匹配广播中的 FFE0 服务，减少误选其他 BLE 设备的可能。

如果首次连接在服务发现前断开，页面会自动重试两次，并在每次重连后立即重新获取服务与特征。日志会记录断开发生在连接、FFE0 服务发现、FFE1 特征发现还是通知订阅阶段。仍然失败时，关闭可能占用连接的手机 App，给小车蓝牙模块重新上电，再重新选择设备。

## 首次验收

1. 给小车正常供电并关闭手机端蓝牙应用，避免占用 BLE 连接。
2. 架空车轮或断开电机，连接设备。
3. 点击“开始链路验收”。页面先发送停止，再发送官方开启自动上报命令。
4. 收到 `$OK#` 证明 PC 下行指令到达 STM32，收到 `$LV...#` 证明 STM32 数据经 JDY-23 返回 PC。
5. 验收完成后可以继续观察低频遥测；点击“停止实时上报”关闭。

完整平衡车固件的 `app_bluetooth.c` 会返回 `$OK#` 和遥测。当前
`D:\DIP\Bluetooth\Bluetooth` 工程没有链接该文件，而是在 UART5 中断中把每个
接收字节写回 UART5，因此只能完成回环验收。这属于机器人固件能力差异，不是
网页接收失败。

## 数据速率

网页不会也不能修改 JDY-23 与 STM32 之间的串口波特率。BLE 通知全部进入组帧器；传感器卡片和 Canvas 波形最多每 100 ms 刷新一次，以降低界面开销。官方 LQR 固件通常约每 2 秒产生一帧自动上报，实际频率由当前烧录固件决定。

## 文件

- `bluetooth.html`：验收与控制界面。
- `protocol.js`：官方协议常量、组帧器和解析器。
- `app.js`：Web Bluetooth、验收流程、控制和波形。
- `styles.css`：自适应界面样式。
- `server.py`：无第三方依赖的 localhost 服务。
- `server.js` / `start-server.cmd`：无需安装 Python 的 localhost 服务。
- `official-echo-test.html`：针对当前 UART5 回环固件的最小诊断页。
- `tests/protocol.test.js`：协议拆包、粘包和字段解析测试。

离线测试：

```powershell
node .\tests\protocol.test.js
```

## Windows 原生 BLE 探针

当 Android 可以通信，但 Chrome/Edge 都在 FFE0 服务发现阶段断开时，先关闭手机
蓝牙并给小车重新上电，再运行 `native-probe\run.cmd`。探针绕过浏览器，直接调用
Windows BLE API，扫描 YahBoom/JDY、发现 FFE0/FFE1、订阅通知，并向当前 UART5
回环固件发送测试字符串。

- `PASS`：Windows 原生 GATT 可用，问题位于浏览器路径。
- `GATT/FFE0 discovery failed`：Windows/蓝牙适配器底层兼容问题。
- `No YahBoom/JDY advertisement`：模块当时未广播，常见原因是手机仍保持连接。

`native-probe\run-v2.cmd` 使用当前 Windows 的新 GATT 接口，设置
`GattSession.MaintainConnection=true` 并强制使用 `BluetoothCacheMode.Uncached`
发现 FFE0，用于确认清除缓存后底层连接是否仍失败。
