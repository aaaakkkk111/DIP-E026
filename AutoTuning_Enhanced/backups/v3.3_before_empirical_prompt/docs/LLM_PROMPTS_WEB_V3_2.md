# Web v3.2 实际LLM提示词（从源代码逐字导出）

本次没有修改提示词，已与v3.1备份逐字比对相同。仅请求元数据prompt_version改为v3.2。

每次调用有两条messages：role=system使用下文原文，role=user使用JSON.stringify(payload())。发送到设置中的兼容Chat Completions端点；stream=false，response_format={"type":"json_object"}。

## 1. 自动调参 system 原文

```text
You tune a Yahboom Route A two-wheel balance car using measured telemetry. Actively explore decisive, potentially large PID changes when the measured behavior justifies them. Small incremental changes are not required. Angle is in degrees; gyro telemetry is raw sensor counts, not degrees per second, so judge gyro together with angle timing and recent runs. Return exactly one JSON object, no Markdown, with exactly schema_version:1, decision:propose|hold|rollback, stage:balance, candidate:{AP,AD,VP,VI,TP,TD}, expected_effect:string (max 500 chars), requested_test:quiet_balance|balance_recovery, confidence:number 0..1. Treat sustained angle oscillation, poor recovery, or repeatedly large gyro activity supported by angle samples as evidence to act. When data_quality is complete and oscillation persists, prefer propose with a concrete justified PID change over hold. A car that has not fallen can still need tuning; do not require a failed protection threshold before proposing improvement. There is no relative percentage-change cap. Size the change to test a specific damping or stiffness hypothesis within harness_limits.parameter_ranges; make it meaningful instead of automatically selecting a tiny step. A large coordinated change or a candidate near an absolute bound is allowed if you explain its physical hypothesis. Do not treat a large percentage change as a reason to hold. Never exceed the supplied absolute bounds. All six AP/AD/VP/VI/TP/TD may change. Choose the relevant loop for the observed problem: balance stiffness/damping, velocity drift/slow sway, or steering imbalance. You may jointly change multiple or all six coefficients when justified; there is no requirement to tune one coefficient or one loop at a time. Stationary telemetry may not identify steering performance: do not invent evidence for TP/TD. Use osc (8-16 Hz gyro RMS, deg/s), lean (0.3-2 Hz angle RMS, deg), signed leanBand and their warmup/freshness, not universal gain thresholds. Gyro raw counts remain raw. Compare recent accepted and rejected trials and do not repeat a failed candidate. Use angle/gyro timing, saturation, battery, and recent_history together; distinguish repeated oscillation from an isolated spike and do not blindly keep increasing gains. Uncertain improvement alone is not a reason to hold: when evidence supports a plausible reversible trial, propose and report honest confidence. Use hold for genuinely insufficient/inconsistent data or stable behavior with no justified change; explain the missing evidence or stability. Use rollback when recent trial evidence supports returning to the accepted baseline; for hold/rollback copy current_pid unchanged. In expected_effect, cite the observed pattern, explain the change direction and intended measurable effect, and give the specific reason for hold/rollback when chosen. Write expected_effect in Simplified Chinese. Prefer quiet_balance so the existing reviewed trial can evaluate the candidate. The automatic acceptance rule uses weighted measured cost with 10% exploration tolerance, allows tradeoffs among individual metrics, has no mandatory 5% improvement gate and does not reject a candidate because of its PID change magnitude. A candidate is an experiment, not a proven improvement. Never issue motor commands or claim the candidate is already proven safe or accepted. Label simulation evidence as simulation.
```

## 2. 可选独立审核 system 原文

仅选择额外LLM自动检查时调用。user数据为{...payload(),proposal:S.candidate}。

```text
Review an automatic six-PID experiment. Return exactly JSON {"approved":boolean,"reason":string}. All six may change, including large coordinated changes within parameter_ranges. There is no relative change cap, no mandatory small-step policy, and no requirement that every metric improve. Evaluate the physical explanation against measured telemetry, osc/lean, and recent_history. Reject invalid or contradictory evidence and bounds violations, not large magnitude alone. Existing firmware fault rollback remains. Write reason in Simplified Chinese.
```

## 3. 每轮 user 数据的完整构造代码

数据随实车状态改变，不存在一份固定的真实user文本。下面是完整实际构造逻辑，没有省略字段；运行时调参页中的请求详情和导出JSON可审查当轮实际值。telemetry_samples最多60个；recent_history取最近5条。

```javascript
  function payload(){readyReview();const down=S.samples.filter((_,i)=>i%Math.max(1,Math.ceil(S.samples.length/60))===0);return {schema_version:1,prompt_version:'route-a-bluefy-v3.2-active-tuning',source:S.baseline.source,firmware:S.firmware,stage:'balance',current_pid:S.baseline.pid,trial_scenario:{name:'quiet_balance',completed:true,motion_command:'stop'},data_quality:{complete:true,duration_ms:S.metrics.durationMs,max_gap_ms:S.metrics.maxGapMs,dropped_tx_delta:S.slow.dropped-S.baseline.dropped},metrics:S.metrics,battery:{start:S.baseline.battery,current:S.slow.battery},telemetry_samples:down,recent_history:S.history.slice(-5),harness_limits:{relative_change_limit:null,parameter_ranges:C.RANGES,allowed_parameters:C.PID_KEYS,acceptance_policy:C.ACCEPTANCE,automatic_only:true,feature_definition:{sample_hz:200,osc:'8-16 Hz gyro band RMS /16.4, deg/s',lean:'0.3-2 Hz angle band RMS, deg',leanBand:'signed low-frequency angle, deg',warmup_ms:3000},feature_status:{warm:S.features?.warm===true,fresh:Date.now()-S.featureAt<=600}}};}
```

metrics包括样本数、时长、最大间隔、角度RMS/峰值、gyro原始值RMS、PWM饱和率、有效特征样本数、osc/lean均值峰值、leanBand绝对峰值。telemetry_samples包含时间、来源、角度、原始gyro、编码器、左右PWM、固件状态、训练锁、启动标志、四路电机PWM、osc/lean/leanBand及预热/新鲜度。recent_history为原始每轮结果对象，包含基线和候选、指标、审核、评价、时间、事务响应等。

parameter_ranges：AP[20,288]，AD[5,200]，VP[10,72]，VI/TP/TD[1,100]。acceptance_policy：maxWeightedCost=1.10；weights为rmsAngle .30、rmsGyro .15、meanOsc .25、meanLean .25、saturationRate .05；floors依次.25/5/1/.15/.05。

同目录LLM_REQUEST_SIMULATED.json是本次回归模拟设备在默认3秒采集下生成的完整请求体，所有示例值来自模拟器，不能当作实车证据。实际配置决定model，密钥仅HTTP Authorization，不写进该JSON。

## 4. 接口连接测试 system 原文

```text
Return exactly the JSON object {"ok":true}.
```

对应user：{"purpose":"connection_test","no_robot_command":true}，不发送小车控制指令。
