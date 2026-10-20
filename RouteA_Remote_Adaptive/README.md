# Route A RA3.3-DIRECT

工作区：`D:\DIP\RouteA_Remote_Adaptive_Workspace`。本次仅修改本工作区，官方资料和 `leveling/firmware` 保持原样。历史版本在 backups/20261020_before_direct_write。

## 本轮变更

将模式21的在线调参尽量贴近官方直接赋值路径：主循环完成收包、CRC、范围校验和AP/VP/TP的×100换算，下一次5 ms控制中断入口一次写入所选控制环。删除500 ms插值过程，不调用动态状态清零，不按新旧Ki重算积分，不因普通调参改变运动命令、机械中值或Normal/Load倍率。P3、官方格式兼容更新和回滚共用同一个底层赋值函数。模式1—20的分支和PID算法文件不作本轮改动。

保留原有4 KB栈、静态收发缓冲、严格有界数字解析、内存故障锁停和SF/DT/CT诊断。本轮不把“清零”认定为历次倾倒的唯一根因；历史日志确有CRC正确但PID异常的数据。本版消除额外更新过程，并继续保留相应故障检测。

## 文件

- Keil：`firmware/USER/stm32_Balance_Car.uvprojx`（ARMCC 5.06u7 build960，O1）。
- 烧录：`releases/stm32_Balance_Car_L.hex`。
- Bluefy：`releases/index.html`，独立文件同目录 `routea_remote_adaptive_bluefy.html`。
- 协议：PROTOCOL.md；验收：PID_UPDATE_DIRECT.md；构建数据：FINAL_VERIFICATION.md。
- 发布目标：仓库aaaakkkk111/DIP-E026，分支codex/routea-mobile-v1.5，根目录index.html。
- 网页地址：https://aaaakkkk111.github.io/DIP-E026/?v=RA3.3-DIRECT 。

## 操作

1. 烧录上面的HEX，断电重启，选择21.TEST MODE，按键结束选模，再按KEY1启动平衡。
2. Bluefy打开网页，确认页头为RA3.3-DIRECT。先关闭官方APP的蓝牙连接。
3. 连接小车，等待状态页固件也显示RA3.3-DIRECT。固件未同步或仍是旧版时，网页禁止写PID和启动自动训练。
4. Normal初始显示值为96/48/62/31/17/20。人工页点击“测试通信”，然后原值写回“更新平衡环”。
5. 三个单环按钮只写对应两项；“更新全部PID”写六项。只有完成ACK及GET实际读回一致，才显示已应用并解除写入锁。通信响应丢失时保持锁定，先点击同步；不得反复盲发。
6. Normal/Load按钮是独立的预设操作，切换六项及固定倍率；普通PID更新不能偷偷切换配置。
7. 自动页点击开始前，在设置页填写模型服务Base URL、模型和API Key。API Key不持久化、不导出。保留前台运行。
8. 自动闭环为基线采样→LLM→PREP/PPRESET→APPLY→TRIAL状态和GET读回匹配→稳定期→两次评价→ACCEPT/ROLLBACK。人工PID按钮在训练期间锁定。
9. 停止自动训练会请求回滚活动试验并解锁。红色停止按钮发送零运动命令，维持平衡控制；它不是电机电源切断。
10. 设置页Mock模式可不接实车演示，导出JSON包含原始收发和决策记录，CSV包含快速遥测。

## 参数和限制

| 预设 | AP | AD | VP | VI | TP | TD | 平衡/速度/转向倍率 |
|---|---:|---:|---:|---:|---:|---:|---|
| Normal，官方模式1 |96|48|62|31|17|20|1 / 1 / 1|
| Load，官方模式4 |96|75|70|35|14|20|2 / 1.35 / 1|

协议显示值直接输入网页，AP/VP/TP仅由固件×100一次。没有相对百分比修改幅度上限；绝对保护范围AP20..288、AD5..200、VP10..72、VI1..100、TP/TD1..100仍有效，这是本工具范围，不是官方通用硬限制。直接赋值不会消除大参数跃变本身造成的控制输出突变，不保证任意参数能平衡或LLM一定改善。

UART5保持9600 8N1。快速遥测约5Hz，慢速约1Hz。活动试验心跳超时2秒、TTL最多30秒，风险回滚保留。停止LLM或连接中断不保证车辆静止或恢复直立，物理安全仍需现场控制。

## 本地测试

```powershell
node --test ios-bluefy/tests/*.test.cjs
node tests/run_native_tests.cjs
python tests/check_firmware_resources.py
node ios-bluefy/build.mjs
```

已完成Keil重建、原生C控制状态连续性测试、网页流程测试和完整Mock闭环。未完成本轮实车、iOS Bluefy触控或真实LLM服务验收。Mock通过不代表实车调参必然稳定。旧PID_UPDATE_FIX/STACKFIX文档保留为历史记录，当前行为以本README和PROTOCOL为准。
