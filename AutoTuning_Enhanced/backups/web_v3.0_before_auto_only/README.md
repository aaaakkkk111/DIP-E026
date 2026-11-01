# AutoTuning Enhanced · RA1.9-AUTO6 / Web v3.0

独立工作副本，基线为用户确认已经实车验证的 RA1.8-PID-NO-PCT-CAP + 网页v2.5。
新版本自身尚未实车验证；不继承旧版本的实车验证结论。未使用RA3.0-SEQ1固件实现。

## 文件

- `firmware/firmware/USER/stm32_Balance_Car.uvprojx`：Keil工程。
- `releases/stm32_Balance_Car_RA1_9_AUTO6.hex`：经过本次ARM机器码测试的独立ARMCC构建HEX。
- `firmware/firmware/OBJ/stm32_Balance_Car_L.hex`：Keil重建输出，同源但布局可能不同。
- `releases/routea_bluefy.html`：独立网页，配套RA1.9-AUTO6。
- `releases/AutoTuning_Enhanced_source.zip`：源码、构建脚本和测试。
- `docs/PROTOCOL.md`、`docs/ACCEPTANCE.md`：协议和验收。
- `baseline/`：原版源码校验值；`tests/arm_results.json`：本次ARM模拟回归结果。

## 首次使用

1. 烧录 `releases/stm32_Balance_Car_RA1_9_AUTO6.hex`。需要回到原版时使用原目录的RA1.8 HEX。
2. 转动轮子选择 **3.TEST MODE**，按原有KEY1流程确认模式与启动平衡。其他菜单只有1.Standard和2.Load。
3. 在iPhone Bluefy打开 `https://aaaakkkk111.github.io/DIP-E026/?v=AUTO6-3.0`，刷新并在设置日志确认Web v3.0、固件RA1.9-AUTO6。
4. 连接BLE，Service FFE0 / Characteristic FFE1，保持20字节分片、18ms间隔；UART仍为9600 8N1。GET/PID确认后才允许控制。
5. 状态页查看实际六参数、角度、编码器、PWM、电压，以及osc、lean、lean_band。滤波启动需3秒预热。计算200Hz，快遥测5Hz，指标2.5Hz，慢遥测1Hz。
6. 首先验证人工小改：AP96→95→96；其余参数分别小改并恢复，每次必须看到ACK和整组六参数读回一致。参数显示值不会在网页重复乘100。
7. 训练页的原有单轮手动操作继续保留。连续自动：配置LLM接口/模型/会话Key，选择本地规则或独立LLM审核，勾选连续会话授权，点击“启动连续自动调参”。无须每轮重复人工采集、PREP、APPLY。
8. 自动流程锁定人工PID，采集新基线→LLM建议→审核→PREP/APPLY→1秒过渡→评价→接受或回滚→等待→下一轮。停止时点击“停止并解锁”，并检查锁状态。
9. 手机保持前台。紧急停止指停止运动目标/回滚，平衡控制可能仍运行；断线并不保证电机立即断电，现场使用硬件停止。
10. 设置页导出CSV或完整JSON；包含实际读回、指标、候选、事务响应和历史结果；Key仅存当前会话，不写入导出。

## 连续模式规则

- 默认基线8秒、评价10秒、轮间等待5秒；基线/评价可设5..20秒，轮间2..60秒。
- LLM可修改全部AP/AD/VP/VI/TP/TD，不设相对百分比硬上限；绝对范围仍为AP20..288、AD5..200、VP10..72、VI1..100、TP/TD1..100。这是软件保护范围，不是任意组合稳定的保证。
- 每个候选仍须通过固定JSON schema、绝对范围及本地数据质量检查。独立LLM审核可选。
- 静止平衡无法充分识别TP/TD等转向参数，提示词明确不允许虚构证据；允许修改六项不等于要求每轮全改。
- 自动接受要求角度RMS/峰值、gyro RMS、饱和率和osc/lean不超过恶化容限，并至少一个具有非微小基线的RMS/振荡指标改善5%。这是明确的窗口判据，不证明真实系统全局最优或因果关系。
- 无改善回滚；连续三轮失败/审核拒绝暂停。hold保持当前已确认参数，等待后重新采集。LLM提出rollback而无活动候选时保持当前已确认参数，不回退更早已接受结果。
- 固件保护回滚、断线、后台、异常PID或接口错误会停止连续流程；不自动重连后恢复调参。保护停止后可能仍显示训练锁，须确认停止训练解锁。
- 事务心跳500ms，固件超时2秒；PREP开始30秒期限，未放宽原固件角度/电压/饱和回滚。
- 只有收到ACK及实际GET读回一致才报告应用。人工及LLM均保留RA1.8控制边界整组直接赋值，不清空PI积分/滤波，不引入逐项延时写入。

## 模拟与构建

设置页“启动演示模式”，训练页建议来源选“模拟演示（仅模拟设备）”，勾选授权即可无Key演示闭环；模拟器不因候选自动变好，可能回滚三次并停止，这是模拟结果。

网页：`node web/build.mjs`。
测试：`node --test web/tests/core.test.cjs web/tests/app.test.cjs web/tests/enhanced.test.cjs web/tests/proxy.test.cjs web/diagnostics/get.test.cjs`。
独立ARMCC构建：`python firmware/build.py`，默认现有工具链 `D:/DIP/MDK-ARM/core/ARM/ARMCC/Bin`，O1、microlib、64KiB链接保护。
也可在Keil打开uvprojx Rebuild；4KiB栈，原编译器ARMCC5.06u7。
ARM测试：`python -m pip install --target tests/deps unicorn==2.1.4` 后 `python tests/arm_regression.py`。使用baseline/RA1.8_original.axf作为只读对照，不需重新编译原版。
本地预览：`node web/server.mjs`，访问 `http://127.0.0.1:8787/`；该代理支持TLS配置，继承原版实现。
GitHub Pages仅托管静态网页，不提供LLM代理。直连API需要该服务允许CORS，否则配置自己的HTTPS代理。

## 验证边界

网页自动化使用模拟GATT/LLM和本机模拟代理，不调用真实付费模型；ARM测试运行实际AXF指令，硬件I2C、PWM、编码器使用替身。
5ms来自原MPU6050/DMP 200Hz配置，新增滤波已做机器码数值测试；未测量实机最坏中断耗时，也未测量真实无线延迟。
原版固件与网页源码SHA256逐文件复核不变。版本发布仅覆盖授权仓库的网页入口，不修改原始工程或同组其他仓库。
