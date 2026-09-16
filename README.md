# Team A — Harnessed LLM PID Auto-Tuning

本分支实现 Route A：在 MuJoCo 中复现二轮平衡车、虚拟 STM32 PID 与蓝牙协议；LLM 只提出 PID 候选，确定性的 Safety Harness 负责格式校验、限幅、试验、接受、缩小与回滚。

```text
MuJoCo Plant -> Virtual STM32 (5 ms PID) -> 9600 baud $...#
-> telemetry/metrics -> DeepSeek or MockLLM -> Safety Harness
-> matched candidate trial -> accept / shrink / rollback
```

LLM 不输出 PWM、不直接控制电机、不修改安全阈值，也不能自行接受候选。MuJoCo Oracle 只用于绘图和验证，不进入 LLM、评分或 Harness。

> 这是仿真和迁移验证工具。仿真通过或分数提高不代表真实小车已经安全或稳定。

## 主要文件

| 路径 | 用途 |
|---|---|
| `route_a/` | Plant、虚拟 STM32、协议、遥测、指标、LLM、Harness、状态机、UI 与测试 |
| `firmware_pid_car.xml` | MuJoCo 二轮平衡车模型 |
| `requirements-route-a.txt` | Python 依赖 |
| `start_route_a.bat` | Windows UI 启动入口 |
| `route_a.env.example` | DeepSeek 配置模板，不含密钥 |
| `route_a/README.md` | 固件追踪、协议、参数来源与实现细节 |

Route A 不导入现有 Route B、神经策略或 LQR 自由控制代码。

## 安装与启动

```powershell
Set-Location D:\DIP\Simulation
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m venv .venv
& '.\.venv\Scripts\python.exe' -m pip install --upgrade pip
& '.\.venv\Scripts\python.exe' -m pip install -r requirements-route-a.txt
& '.\.venv\Scripts\python.exe' -m route_a.main
```

依赖已经安装时，可直接使用系统解释器：

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m route_a.main
```

也可运行 `start_route_a.bat`。UI 与独立的 MuJoCo 3D Viewer 会同时打开；首次验收保持 `Adviser = MockLLM`，无需网络和 API key。

## 离线验收

1. 确认 3D Viewer 可见，Camera follow 能跟随小车。
2. 测试 Forward、Backward、Left、Right、Pivot left/right 与 Stop。
3. 测试四向外力，确认遥测、曲线和模型同步变化。
4. 修改坡度、摩擦、负载或传感器设置后点击 Apply environment。
5. 点击 Reset robot pose，确认只恢复位姿，PID 和训练状态不变。
6. 运行测试和离线端到端演示：

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m unittest discover -s route_a\tests -v
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m route_a.main --offline-demo --duration 4
```

## DeepSeek 配置

```powershell
Copy-Item .\route_a.env.example .\route_a.env
notepad .\route_a.env
```

`D:\DIP\Simulation\route_a.env` 内容：

```dotenv
DEEPSEEK_API_KEY=我自己的密钥
DEEPSEEK_BASE_URL=https://api.deepseek.com
DEEPSEEK_MODEL=deepseek-flash
```

确认密钥不会提交：

```powershell
git check-ignore -v route_a.env
git status --short -- route_a.env
```

第一条应显示 `.gitignore` 规则，第二条应无输出。不要把密钥放进源码、README、截图或日志。真实 API smoke test 只在需要时显式运行：

```powershell
& 'C:\Users\stato\AppData\Local\Programs\Python\Python313\python.exe' -m route_a.api_smoke_test --run-real-api
```

## 人工审核工作流

按 `balance -> velocity -> turn` 分阶段调参：

| Stage | 允许修改 | 自动试验 |
|---|---|---|
| balance | AP、AD | 停止命令下施加前向扰动 |
| velocity | VP、VI | 固件 Forward 命令阶跃 |
| turn | TP、TD | 右转阶段后接直行阶段 |

1. 选择 `Mode = manual-review`、Stage 和试验时长，建议 10 秒。
2. 先用 MockLLM 验证流程，再选择 DeepSeek。
3. 点击 Start。系统查询 PID，并以固定场景和随机种子运行基线。
4. 检查 Telemetry/metrics、Telemetry vs Oracle plots、LLM JSON/Harness。
5. 确认候选只修改当前 Stage 参数，并查看 Harness 的接受、缩小或拒绝原因。
6. 点击 Approve 才会写入候选并执行匹配试验；Reject 保留冠军。
7. 人工批准不等于最终接受，确定性评分器仍可回滚。
8. 至少 3 次成功人工审核后，Automatic 才会解锁。

Reset training 会清空训练状态并在 Harness 全范围内重新随机生成六个 PID，但保留历史运行目录。Reset robot pose 只重置 MuJoCo 位姿，不改变 PID、冠军或计数器。

若随机 AP/AD 导致跌倒、保护触发或超过 40°，系统进入 Bootstrap recovery。DeepSeek 必须提出平衡环救援候选；Harness 仍执行绝对范围、阶段锁、协议精度与安全判断。

## 自动调参工作流

Automatic 只运行当前 Stage，不会自动切换三个环。

1. 确认 `Manual approvals >= 3/3`。
2. 选择 `Mode = automatic`、Stage、Trial seconds、Auto rounds 和 `Adviser = DeepSeek`。
3. 点击 Start。
4. 每轮自动执行：基线、LLM 提案、Harness 校验、候选写入并读回、匹配试验、确定性评分、接受/缩小/回滚。
5. 检查 Current/Champion PID、评分、PWM 饱和、跌倒/保护、遥测完整性和 Harness 原因。
6. balance 收敛后，依次选择 velocity 和 turn 运行新的自动循环。

运行期间：Pause 中止当前试验，Resume 后再 Start 开始新一轮；Emergency Stop 立即停止并中止；Rollback 通过协议恢复并读回冠军。基线和候选匹配试验中不要手动改变命令、推力或环境。

## 每轮重点检查

- pitch RMS/peak、pitch-rate RMS、稳态角度误差和恢复时间；
- 速度 RMSE/超调、纵向位移范围；
- yaw RMSE、左右轮差异；
- PWM 饱和比例和控制消耗；
- 跌倒、保护、遥测丢失/过期、无效帧；
- 基线与候选是否使用相同 Stage、环境和随机种子。

不要只看总分。候选必须先通过统一硬性安全门槛，再比较当前 Stage 的加权评分。

## 记录与复现

每轮记录在 `route_a/runs/<timestamp>_round_<n>_<stage>/`，包括配置、PID、协议帧、遥测/Oracle CSV、指标、曲线、LLM 输入/响应、Harness 结果、最终决策、随机种子、Git commit 和 API 使用信息。

`route_a.env`、`route_a/runs/` 和 `route_a/state/` 的本地内容由 `.gitignore` 排除，只保留目录占位文件。

## 下一步：实车影子迁移

自动仿真验收后，先实现 SerialTransport 的只读影子运行：实车继续由当前固件控制，PC 只解析协议、计算指标并生成但不发送候选。确认符号、量纲、5 ms 周期、9600 baud、超时、重复应答和写入读回一致后，才在防倒架、物理急停和人工审核下进行小幅写入。

推荐顺序：

1. 固化冠军 PID、环境、日志和 Git commit。
2. SerialTransport 只读采集与协议一致性测试。
3. LLM/Harness 影子运行，不发送 PID。
4. 用实车数据校准电机、死区、摩擦、质量、质心、IMU 和编码器模型。
5. 防倒架上按 Stage 小幅、人工批准地写入 PID。
6. 低风险地面试验与故障注入验证。
7. 半自动可靠后，才考虑受监督的自动模式。

真实 STM32 必须始终保留 5 ms PID、倾倒/低压保护与本地停止能力；PC、网络或 LLM 故障不能取消这些保护。

固件源码追踪、协议、估计参数和未经过实车验证的项目详见 [route_a/README.md](route_a/README.md)。
