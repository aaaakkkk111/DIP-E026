# AutoTuning Enhanced · 固件 RA1.9-AUTO6-TXFIX-VP100 / Web v3.4

本次只更新网页LLM提示词：在原提示词后追加空载、平地实测的条件性经验。固件和HEX与v3.3逐字节相同，已烧录VP100固件时无需重新烧录。本次网页/资料备份在backups/v3.3_before_empirical_prompt。
工作区来自用户已实车验证的RA1.8 + 网页v2.5。RA1.9/当前网页的新行为仍需实车验收。
原始两个工程目录没有改动；Web v3.0完整备份在 `backups/web_v3.0_before_auto_only`。

## 使用

1. 小车选择3.TEST MODE，按现有KEY1流程启动平衡。另有1.Standard、2.Load作为对照。
2. iPhone Bluefy打开 https://aaaakkkk111.github.io/DIP-E026/?v=EMPIRICAL-3.4 ，刷新后日志应为Web v3.4，固件应为RA1.9-AUTO6-TXFIX-VP100。
3. 连接并确认GET/PID、实时快慢遥测，以及osc/lean已预热。UART仍9600 8N1，FFE0/FFE1、20字节/18ms不变。
4. 设置页填写API Base URL、Model和当前会话Key，调参页点“启动自动调参”。点击启动即授权本次前台会话连续运行，不需要额外勾选或每轮确认。
5. 系统自动采集新基线→请求LLM→格式/范围检查→PREP/APPLY→整组实际读回→1秒过渡→观察→自动ACCEPT或ROLLBACK→下一轮。
6. 点“停止并解锁”结束。后台切换、连接异常、非法PID或协议结果不确定时停止；不会自动重连后继续调参。若通信已经中断，停止与解锁不能保证收到小车确认，请现场检查。
7. 人工PID仍在独立PID页，自动训练期间锁定。已移除LLM的人工采样、粘贴JSON、人工审核及PREP/APPLY/ACCEPT按钮。
8. 设置页可导出完整日志JSON与CSV。Key不写入本地设置或导出，网页保持前台。

## 简化界面

调参页只有启动/停止两个主要操作；五阶段指示、实际采样/观察进度、LLM等待时间、当前与候选六参数对照、建议理由、每轮相对代价图、接受/回滚记录均自动更新。
最近一次hold理由保留显示。计数和图表按本次启动会话统计，导出仍保留完整历史。
高级设置折叠：采集1..20秒、评价1..20秒、轮间2..60秒；默认3/1/5秒。停止自动调参后修改高级设置中的采集/评价秒数，保存设置即可持久化；运行中锁定。原有1秒应用后过渡及5秒轮间等待保持不变。默认只作格式、范围与实时状态检查，可选择额外独立LLM自动检查。
网页不能驱动小车自动制造冲击或执行恢复扰动，目前自动实验仍为quiet_balance。静止数据不能充分辨识全部转向性能，LLM不得虚构证据。

## 激进方案及接受规则

六参数可同时大幅改变，无相对百分比上限、无“只能小步/只改一环”要求。提示词允许有物理依据的大幅协同方案或接近绝对边界的方案。
固件绝对范围保持AP20..288、AD5..200、VP10..100、VI/TP/TD1..100；NaN、Infinity、非法报文、未确认读回仍拒绝。
不再要求至少一个指标改善5%，不再要求每一项分别通过原恶化阈值，也不再在连续三轮未改善/审核拒绝时自动停止。普通回滚后继续采集并把历史结果交给模型修正方向。

接受使用**各轮自己的基线**归一化综合代价：

`J = Σ weight[k] * (trial[k] + floor[k]) / (baseline[k] + floor[k])`

| 指标 | weight | floor |
|---|---:|---:|
| 角度RMS（度） | .30 | .25 |
| gyro RMS（raw） | .15 | 5 |
| osc均值（°/s） | .25 | 1 |
| lean均值（度） | .25 | .15 |
| PWM饱和率（0..1） | .05 | .05 |

`J≤1.10`允许ACCEPT；`J>1.10`ROLLBACK。参数改动幅度不参与这一拒绝判断。
J=1表示与本轮基线持平；1到1.10属于接受探索容差内的取舍，**不代表已经改善**。允许单项变差而其他项改善，floor抑制接近零基线的比值放大。
该窗口判据允许探索而非保证最优；连续接受有容差的方案也可能逐步恶化，需结合实际曲线和导出记录评价。
按所设秒数检查实际时间覆盖及至少ceil(秒数×3)帧有效样本（最低2帧），最大帧间隔1.5秒；osc/lean仍需至少80%样本有效。5Hz下1秒评价通常约6帧，只作短窗判断，不能保证长期稳定。固件原倾角/低电压/饱和、心跳2秒、事务30秒回滚不变；真实故障回滚或不确定协议结果停止流程。

## 交付

- `releases/routea_bluefy.html`、`releases/index.html`：独立网页。
- `releases/AutoTuning_Enhanced_source.zip`：当前源代码与测试。
- `firmware/firmware/USER/stm32_Balance_Car.uvprojx`：Keil工程；`releases/stm32_Balance_Car_RA1_9_AUTO6_TXFIX_VP100.hex`：当前固件烧录文件，与v3.3相同。原名AUTO6.hex仅作回退。
- `docs/ACCEPTANCE.md`：当前验收；`docs/VERIFICATION.json`：机器验证；`releases/PUBLICATION.json`：发布记录。

## 模拟与开发

设置页启动演示模式，在自动运行设置中选“模拟演示”，再启动；无需Key，不调用外部LLM。模拟器不会因为候选幅度大就自动改善或自动失败，模拟结果不能证明实车稳定。
构建：`node web/build.mjs`。
测试：`node --test web/tests/core.test.cjs web/tests/app.test.cjs web/tests/proxy.test.cjs web/diagnostics/get.test.cjs web/tests/windows-trial0.test.cjs web/tests/vp100.test.cjs web/tests/empirical-prompt.test.cjs`。
代理：`node web/server.mjs`，访问http://127.0.0.1:8787；手机需可访问信任的HTTPS代理。GitHub Pages仅托管静态网页，不提供LLM后端；直连服务需要支持CORS。
固件构建/原ARM测试保留：`python firmware/build.py`；安装 `unicorn==2.1.4` 到tests/deps后运行 `python tests/arm_txfix.py` 和 `python tests/arm_vp100.py`。此前VP100固件的10项ARM回归和Keil构建记录继续适用；本次固件逐字节未改，不重复编译。
本次网页测试使用模拟GATT/LLM和本机代理，没有调用真实付费模型，也没有操作真实小车或测量5ms实机最坏耗时。

## 本次日志定位

原始日志11:16:56.098已收到ACCEPT ACK，但之前排队的HB随后到达已经IDLE的固件，产生TRIAL,0，错误地拒绝GET并触发网页停止。日志后续PID仍为已接受72/68/62/31/17/20，不能据此认为ACCEPT失败。
网页在发送ACCEPT/ROLLBACK之前停止并清理待发HB；不打断已经写到一半的蓝牙帧。结束ACK已收到时，迟到TRIAL,0不会拒绝GET，仍必须读回预期PID、IDLE与训练锁。固件只忽略最近已完成事务的迟到HB，未知ID继续拒绝，也不能续命新事务。
只修改route_a.c的版本/已结束事务记录/迟到HB处理，以及网页相关收尾与采样窗口。PID控制公式、积分状态、默认PID、5Hz/9600、范围、保护、提示词和综合接受规则保持原样。
完整提示词在docs/LLM_PROMPTS_WEB_V3_4.md；机器可读原文在docs/llm_prompts.json。

## VP100与旧固件兼容

网页v3.3最大支持VP100。检测到RA1.9-AUTO6-TXFIX-VP100标记后，人工输入及发给LLM的parameter_ranges均为10..100；连接原RA1.9固件或未收到新版本标记时，仍为10..72。普通旧固件手动调参/停止路径继续可用，>72候选会在发送前拒绝。只换网页不能突破旧固件的72限制，需烧录本次VP100.hex。
100是本次设置的软件实验边界，不是实车稳定性结论，也不是速度上限。其他五参数范围不变，VP100版本的控制和范围保持不变；v3.4仅追加条件性实验经验，没有新增本地多档切换。
下一轮手动测试建议：AP96、AD48、VI31、TP17、TD20固定；VP先比较72、76、80、84。每次恢复原基准，记录前后活动范围和同样窗口的osc/lean，暂不叠加AP150或VI1。分析见docs/experiments/20261024_analysis.md。三次抄数均值不是整段时间均值，T6倾角两列疑似填反，原始表未修正。

## v3.4条件性经验提示词

调参和可选LLM审核都收到同一经验块。保留原system提示词完整前缀和原JSON schema，不新增动作命令。经验来源是空载、平地的人工实验，记录的是各候选三个瞬时读数的均值，不是统计显著结论。最新T6使用VP94，不能误写成旧VP72。载重、坡度和电压变化时，近期实测和试验历史优先；不得将AP150、AD48、VI1等当作固定标准值或强制恢复目标。允许继续提出有依据的大幅/多参数候选。
只保留历史人工活动范围，不新增前后移动量化，也不得从现有稀疏计数虚构当前行程。没有改变手动PID、保护/回滚、VP100兼容、5Hz或3秒/1秒时长。
完整调参system见docs/LLM_SYSTEM_tuning.txt，完整审核见docs/LLM_SYSTEM_review.txt，新增部分见docs/EMPIRICAL_PRIOR_V3_4.txt。docs/LLM_REQUEST_TUNING_V3_4_SIMULATED.json和REVIEW对应文件为模拟运行时实际构造的请求体，仅用于验证请求，不是实车优化证据。
