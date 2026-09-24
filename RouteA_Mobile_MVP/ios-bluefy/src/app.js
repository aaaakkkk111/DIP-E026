(function () {
  "use strict";
  const C = window.RouteACore;
  const $ = (id) => document.getElementById(id);
  const state = {
    device: null, server: null, characteristic: null, connected: false, mock: false, mockTimer: null,
    parser: new C.StreamParser(256), currentPid: null, candidate: null, metrics: null, samples: [],
    trainingLocked: false, balanceStarted: false, firmwareState: 0, firmware: "—",
    trialState: "IDLE", trialId: 0, heartbeat: null, baseline: false, activeMotion: null,
    fast: null, slow: null, rxTimes: [], frameOk: 0, frameError: 0, logs: [], telemetry: [],
    chart: [], sendChain: Promise.resolve(), mockClock: 0, mockPid: { ...C.DEFAULT_PID }
  };
  const ranges = { AP: [20, 288], AD: [5, 200], VP: [10, 72], VI: [1, 100], TP: [1, 100], TD: [1, 100] };
  let toastTimer;

  function nowText() { return new Date().toLocaleTimeString("zh-CN", { hour12: false }); }
  function toast(message) { const el = $("toast"); el.textContent = message; el.classList.add("show"); clearTimeout(toastTimer); toastTimer = setTimeout(() => el.classList.remove("show"), 2500); }
  function log(direction, text) {
    const item = { time: new Date().toISOString(), direction, text };
    state.logs.push(item); if (state.logs.length > 1500) state.logs.shift();
    const shown = state.logs.slice(-120).map((x) => `${x.time.slice(11, 23)} ${x.direction.padEnd(4)} ${x.text}`).join("\n");
    $("log").textContent = shown; $("log").scrollTop = $("log").scrollHeight; $("logCount").textContent = String(state.logs.length);
  }
  function connectionLabel(kind, text) { const p = $("connectionPill"); p.className = `pill ${kind}`; p.querySelector("b").textContent = text; }
  function trialLabel() { $("trialState").textContent = state.trialState; document.querySelectorAll("[data-step]").forEach((el) => el.classList.remove("done")); if (state.trainingLocked) document.querySelector('[data-step="lock"]').classList.add("done"); if (["BASELINE","REVIEW","PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState)) document.querySelector('[data-step="baseline"]').classList.add("done"); if (["REVIEW","PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState)) document.querySelector('[data-step="review"]').classList.add("done"); if (["PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState)) document.querySelector('[data-step="trial"]').classList.add("done"); }

  function initPidUi() {
    $("pidGrid").innerHTML = C.PID_KEYS.map((key) => `<div class="pid-item"><label>${key}<b id="live${key}">--</b></label><input id="edit${key}" inputmode="decimal" type="number" step="0.01" min="${ranges[key][0]}" max="${ranges[key][1]}" placeholder="${C.DEFAULT_PID[key].toFixed(2)}"></div>`).join("");
    C.PID_KEYS.forEach((key) => $("edit" + key).addEventListener("input", updatePidDelta));
  }
  function readPidInputs() { const raw = {}; C.PID_KEYS.forEach((key) => raw[key] = $("edit" + key).value); return C.normalizePid(raw); }
  function setPidUi(pid, copyEdits) { state.currentPid = C.normalizePid(pid); C.PID_KEYS.forEach((key) => { $("live" + key).textContent = state.currentPid[key].toFixed(2); if (copyEdits) $("edit" + key).value = state.currentPid[key].toFixed(2); }); updatePidDelta(); updateUi(); }
  function updatePidDelta() {
    if (!state.currentPid) { $("pidDelta").textContent = "先读取当前 PID，才允许人工写入。"; return; }
    try { const p = readPidInputs(); $("pidDelta").innerHTML = C.PID_KEYS.map((key) => { const d = state.currentPid[key] ? (p[key] - state.currentPid[key]) * 100 / Math.abs(state.currentPid[key]) : 0; return `<b>${key}</b> ${d >= 0 ? "+" : ""}${d.toFixed(2)}%`; }).join("　"); }
    catch (e) { $("pidDelta").textContent = e.message; }
  }

  function loadSettings() {
    let s = {}; try { s = JSON.parse(localStorage.getItem("routea-bluefy-settings") || "{}"); } catch (_) {}
    for (const [id, fallback] of [["serviceUuid","0000ffe0-0000-1000-8000-00805f9b34fb"],["charUuid","0000ffe1-0000-1000-8000-00805f9b34fb"],["namePrefix","YahBoom_BL"],["chunkSize","20"],["chunkDelay","18"],["llmBase","https://api.deepseek.com"],["llmModel","deepseek-chat"],["maxPct","5"]]) $(id).value = s[id] ?? fallback;
  }
  function saveSettings() { const s = {}; ["serviceUuid","charUuid","namePrefix","chunkSize","chunkDelay","llmBase","llmModel","maxPct"].forEach((id) => s[id] = $(id).value.trim()); localStorage.setItem("routea-bluefy-settings", JSON.stringify(s)); toast("设置已保存"); }

  async function connect() {
    if (state.mock) stopMock();
    if (!navigator.bluetooth) { toast("当前浏览器没有 Web Bluetooth，请在 Bluefy 中打开 HTTPS 页面"); switchPage("settings"); return; }
    const service = $("serviceUuid").value.trim().toLowerCase(), characteristic = $("charUuid").value.trim().toLowerCase();
    try {
      connectionLabel("connecting", "选择设备");
      const prefix = $("namePrefix").value.trim();
      const filters = prefix ? [{ namePrefix: prefix }, { namePrefix: " Yah" }] : [{ services: [service] }];
      state.device = await navigator.bluetooth.requestDevice({ filters, optionalServices: [service] });
      state.device.addEventListener("gattserverdisconnected", onDisconnected);
      $("deviceName").textContent = state.device.name || "BLE 设备";
      connectionLabel("connecting", "发现服务");
      state.server = await state.device.gatt.connect();
      const svc = await state.server.getPrimaryService(service);
      state.characteristic = await svc.getCharacteristic(characteristic);
      state.characteristic.addEventListener("characteristicvaluechanged", onNotification);
      await state.characteristic.startNotifications();
      state.connected = true; state.mock = false; connectionLabel("online", "BLE 已连接");
      log("SYS", `FFE0/FFE1 ready: ${state.device.name || "unknown"}`); toast("双向链路已就绪，正在读取 PID"); updateUi();
      await send(C.getPid());
    } catch (e) { log("ERR", `${e.name || "Error"}: ${e.message}`); connectionLabel("offline", "连接失败"); toast(`连接失败：${e.message}`); await cleanupConnection(false); }
  }
  async function cleanupConnection(requestGattDisconnect) {
    stopHeartbeat(); releaseMotion(true); state.connected = false; state.characteristic = null; state.server = null;
    if (requestGattDisconnect && state.device?.gatt?.connected) state.device.gatt.disconnect();
    connectionLabel("offline", "未连接"); updateUi();
  }
  function onDisconnected() { log("SYS", "BLE disconnected"); cleanupConnection(false); toast("蓝牙已断开；请确认小车已经停止"); }
  async function safeDisconnect() {
    try { await safeStop(); if (["PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState)) await send(C.command("ROLLBACK", state.trialId)); await delay(120); } catch (_) {}
    if (state.mock) stopMock(); else await cleanupConnection(true);
  }
  function onNotification(event) {
    const bytes = new Uint8Array(event.target.value.buffer, event.target.value.byteOffset, event.target.value.byteLength);
    for (const raw of state.parser.feed(bytes)) handleFrame(raw);
  }
  function delay(ms) { return new Promise((resolve) => setTimeout(resolve, ms)); }
  async function writeChunks(text) {
    if (state.mock) return mockReceiveCommand(text);
    if (!state.connected || !state.characteristic) throw new Error("BLE UART 尚未就绪");
    const bytes = new TextEncoder().encode(text), size = Math.max(1, Math.min(20, Number($("chunkSize").value) || 20)), pause = Math.max(0, Math.min(100, Number($("chunkDelay").value) || 0));
    for (let i = 0; i < bytes.length; i += size) {
      const chunk = bytes.slice(i, i + size), ch = state.characteristic;
      if (typeof ch.writeValue === "function") await ch.writeValue(chunk);
      else if (typeof ch.writeValueWithResponse === "function") await ch.writeValueWithResponse(chunk);
      else if (typeof ch.writeValueWithoutResponse === "function") await ch.writeValueWithoutResponse(chunk);
      else throw new Error("FFE1 不支持写入");
      if (pause && i + size < bytes.length) await delay(pause);
    }
  }
  function send(text) {
    log("TX", text);
    const task = state.sendChain.then(() => writeChunks(text));
    state.sendChain = task.catch((e) => { log("ERR", `write: ${e.message}`); toast(`发送失败：${e.message}`); });
    return task;
  }

  function handleFrame(raw) {
    log("RX", raw); state.rxTimes.push(Date.now()); state.rxTimes = state.rxTimes.filter((t) => Date.now() - t < 5000); $("lastRx").textContent = nowText();
    if (raw.startsWith("$P1,") || raw.startsWith("$T1,")) { if (!C.verify(raw)) { state.frameError += 1; updateUi(); return; } state.frameOk += 1; }
    else state.frameOk += 1;
    const pid = C.parsePidStatus(raw); if (pid) { setPidUi(pid.pid, !state.currentPid); applyFirmwareFlags(pid); toast("PID 已同步"); }
    const fast = C.parseFast(raw); if (fast) {
      state.fast = fast; state.firmwareState = fast.state; state.trainingLocked = fast.trainingLocked; state.balanceStarted = fast.balanceStarted;
      state.telemetry.push({ at: new Date().toISOString(), ...fast }); if (state.telemetry.length > 10000) state.telemetry.shift();
      state.chart.push({ at: Date.now(), angle: fast.angle, gyro: fast.gyro }); state.chart = state.chart.filter((p) => Date.now() - p.at <= 60000);
      if (state.baseline) { state.samples.push(fast); $("sampleCount").textContent = `${state.samples.length} 样本`; }
      if (fast.state === 2 && state.trialState === "APPLYING") state.trialState = "TRIAL";
      if (fast.state === 0 && ["PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState)) { state.trialState = "IDLE"; state.trialId = 0; stopHeartbeat(); toast("固件已结束试验或完成回滚"); }
    }
    const slow = C.parseSlow(raw); if (slow) { state.slow = slow; state.firmware = slow.firmware; state.trainingLocked = slow.trainingLocked; state.balanceStarted = slow.balanceStarted; setPidUi(slow.pid, false); }
    if (raw.includes("P1,READY,1")) state.balanceStarted = true;
    if (raw.includes("P1,ACK,TRAIN_START,")) { state.trainingLocked = true; toast("训练已锁定；人工 PID 已禁用"); setTimeout(() => send(C.getPid()), 120); }
    if (raw.includes("P1,ACK,TRAIN_STOP,")) { state.trainingLocked = false; state.trialState = "IDLE"; stopHeartbeat(); setTimeout(() => send(C.getPid()), 120); }
    if (raw.includes("P1,ACK,MSET,")) setTimeout(() => send(C.getPid()), 120);
    if (raw.includes("P1,ACK,PREP,") && state.trialState === "PREPARING") { state.trialState = "PREPARED"; toast("PREPARE 已获固件确认"); }
    if (raw.includes("P1,ACK,APPLY,") && state.trialState === "APPLYING") startHeartbeat();
    if (raw.includes("P1,ACK,ACCEPT,")) { state.trialState = "IDLE"; state.trialId = 0; stopHeartbeat(); toast("候选已接受为新 champion"); setTimeout(() => send(C.getPid()), 120); }
    if (raw.includes("P1,ACK,ROLLBACK,")) { toast("固件已接收回滚请求"); }
    const err = raw.match(/P1,ERR,([^,]+)/); if (err) toast(`固件拒绝：${err[1]}`);
    updateUi(); draw();
  }
  function applyFirmwareFlags(p) { state.firmwareState = p.state; state.trainingLocked = p.trainingLocked; state.balanceStarted = p.balanceStarted; }

  function updateUi() {
    const connected = state.connected || state.mock;
    $("rxHz").textContent = `${(state.rxTimes.length / 5).toFixed(1)} Hz`;
    $("frameCounts").textContent = `${state.frameOk} / ${state.frameError}`;
    $("angleHero").textContent = state.fast ? `${state.fast.angle.toFixed(2)}°` : "--.-°";
    $("gyroHero").textContent = state.fast ? `角速度 ${state.fast.gyro.toFixed(1)} raw` : "角速度 --.-";
    $("battery").textContent = state.slow ? `${state.slow.battery.toFixed(2)} V` : "-- V";
    $("encoders").textContent = state.fast ? `${state.fast.el} / ${state.fast.er}` : "-- / --";
    $("motors").textContent = state.fast ? `${state.fast.ml} / ${state.fast.mr}` : "-- / --";
    $("firmware").textContent = state.firmware; $("dropped").textContent = state.slow ? String(state.slow.dropped) : "—";
    $("balanceState").textContent = state.balanceStarted ? "已启动" : "未启动"; $("lockState").textContent = state.trainingLocked ? "已锁定" : "未锁定";
    $("fwState").textContent = ["IDLE / 基线", "PREPARED", "TRIAL"][state.firmwareState] || `未知 ${state.firmwareState}`;
    $("driveReady").textContent = connected ? (state.balanceStarted ? "平衡运行中" : "连接已就绪") : "等待连接";
    $("manualPidButton").disabled = !connected || !state.currentPid || state.trainingLocked || state.firmwareState !== 0;
    $("trainStart").disabled = !connected || !state.currentPid || !state.balanceStarted || state.trainingLocked || state.trialState !== "IDLE";
    $("trainStop").disabled = !connected || !state.trainingLocked;
    $("baselineStart").disabled = !state.trainingLocked || state.trialState !== "IDLE";
    $("baselineFinish").disabled = state.trialState !== "BASELINE" || state.samples.length === 0;
    $("prepareButton").disabled = !state.candidate || !state.trainingLocked || state.trialState !== "REVIEW";
    $("applyButton").disabled = state.trialState !== "PREPARED";
    $("acceptButton").disabled = state.trialState !== "TRIAL";
    $("rollbackButton").disabled = !["PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState);
    document.querySelectorAll("[data-motion]").forEach((b) => b.disabled = !connected);
    document.querySelectorAll("[data-diag]").forEach((b) => b.disabled = !connected || state.balanceStarted || state.trainingLocked || state.firmwareState !== 0);
    trialLabel();
  }

  function validateManualPid() {
    const p = readPidInputs(); for (const key of C.PID_KEYS) if (p[key] < ranges[key][0] || p[key] > ranges[key][1]) throw new Error(`${key} 超出固件保护范围 ${ranges[key][0]}..${ranges[key][1]}`); return p;
  }
  async function manualPid() { try { if (!state.currentPid) throw new Error("请先读取 PID"); if (state.trainingLocked) throw new Error("训练锁定期间不能人工调参"); const p = validateManualPid(); if (!confirm("确认把六个 PID 写入 STM32？发送后仍需等待 ACK 和读回。")) return; await send(C.manualSet(p)); } catch (e) { toast(e.message); } }
  async function trainStart() { if (!state.currentPid || !state.balanceStarted || state.trialState !== "IDLE") return toast("需要 PID 已同步、平衡已启动且无活动试验"); await send(C.training("START")); }
  async function trainStop() { if (["PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState) && !confirm("停止训练会触发回滚。继续？")) return; await safeStop(); await send(C.training("STOP")); }
  function baselineStart() { if (!state.trainingLocked || state.trialState !== "IDLE") return toast("请先开始训练并锁定人工调参"); state.samples = []; state.metrics = null; state.baseline = true; state.trialState = "BASELINE"; $("sampleCount").textContent = "0 样本"; $("metricsBox").textContent = "采集中…保持小车处于指定测试状态"; updateUi(); }
  function baselineFinish() { try { state.baseline = false; state.metrics = C.metrics(state.samples); state.trialState = "REVIEW"; const m = state.metrics; $("metricsBox").innerHTML = `倾角 RMS <b>${m.rmsAngle.toFixed(3)}°</b><br>倾角峰值 <b>${m.peakAngle.toFixed(3)}°</b><br>角速度 RMS <b>${m.rmsGyro.toFixed(3)} raw</b><br>PWM 饱和率 <b>${(m.saturationRate * 100).toFixed(2)}%</b>`; updateUi(); } catch (e) { toast(e.message); } }
  function showProposal(p, source) { state.candidate = p; $("proposalJson").value = JSON.stringify(p, null, 2); $("proposalSource").textContent = source; const c = p.candidate; $("candidateBox").innerHTML = `<b>${p.decision.toUpperCase()}</b> · 置信度 ${(p.confidence * 100).toFixed(0)}%<br>${C.PID_KEYS.map((k) => `${k} ${c[k].toFixed(2)}`).join("　")}<br>${p.expected_effect}`; updateUi(); }
  function harness() { try { if (state.trialState !== "REVIEW" || !state.metrics) throw new Error("请先完成基线采集"); showProposal(C.validateProposal(C.harness(state.currentPid, state.metrics), state.currentPid, Number($("maxPct").value)), "本地 Harness"); } catch (e) { toast(e.message); } }
  function validateJson() { try { if (!state.currentPid || state.trialState !== "REVIEW") throw new Error("请先完成基线采集"); showProposal(C.validateProposal($("proposalJson").value, state.currentPid, Number($("maxPct").value)), "手动 JSON"); toast("候选 JSON 验证通过"); } catch (e) { state.candidate = null; updateUi(); toast(`候选被拒绝：${e.message}`); } }
  async function requestLlm() {
    try {
      if (!state.trainingLocked || state.trialState !== "REVIEW" || !state.metrics) throw new Error("请先锁定训练并完成基线采集");
      const key = $("llmKey").value.trim(); if (!key) throw new Error("请在设置页填写 API Key");
      const maxPct = Math.min(5, Math.max(.1, Number($("maxPct").value) || 5));
      const system = "PROMPT_VERSION: route-a-pid-v4-ios. You are a PID tuning adviser for a two-wheel balance robot. Return exactly one JSON object with exactly: schema_version, decision, stage, candidate, expected_effect, requested_test, confidence. decision is propose, hold, or rollback. stage is balance. candidate always contains numeric AP, AD, VP, VI, TP, TD. Change only AP/AD; keep VP/VI/TP/TD exactly equal to current_pid. Stay within harness_limits. requested_test is balance_recovery or quiet_balance. Never output Markdown, PWM, motor commands, or claim acceptance.";
      const user = { schema_version: 1, current_pid: state.currentPid, stage: "balance", trial_scenario: { name: "balance_recovery", completed: true, motion_command: "stop" }, data_quality: { complete: state.metrics.samples >= 10 }, metrics: { pitch_rms_deg: state.metrics.rmsAngle, pitch_peak_deg: state.metrics.peakAngle, pitch_rate_rms_raw: state.metrics.rmsGyro, pwm_saturation_fraction: state.metrics.saturationRate, sample_count: state.metrics.samples }, recent_history: [], harness_limits: { max_relative_change: maxPct / 100, firmware_hard_cap_percent: 5 } };
      toast("正在请求 LLM…");
      const base = $("llmBase").value.trim().replace(/\/$/, "");
      const response = await fetch(`${base}/chat/completions`, { method: "POST", headers: { "Authorization": `Bearer ${key}`, "Content-Type": "application/json" }, body: JSON.stringify({ model: $("llmModel").value.trim(), response_format: { type: "json_object" }, messages: [{ role: "system", content: system }, { role: "user", content: JSON.stringify(user) }] }) });
      if (!response.ok) throw new Error(`HTTP ${response.status}: ${(await response.text()).slice(0, 180)}`);
      const root = await response.json(); const raw = root.choices?.[0]?.message?.content; if (!raw) throw new Error("响应中没有 choices[0].message.content");
      showProposal(C.validateProposal(raw, state.currentPid, maxPct), "LLM"); toast("LLM 候选已通过本地验证，尚未发给小车");
    } catch (e) { log("ERR", `LLM: ${e.message}`); toast(`LLM 请求失败：${e.message}`); }
  }
  async function prepare() { if (!state.candidate || state.candidate.decision !== "propose" || state.trialState !== "REVIEW") return toast("没有可 PREPARE 的 propose 候选"); if (!confirm("确认把候选发送到固件暂存？此步不会立即改变 PID。")) return; state.trialId = Math.floor(Date.now() / 1000); state.trialState = "PREPARING"; updateUi(); await send(C.prepare(state.trialId, 30000, state.candidate.candidate)); }
  async function apply() { if (state.trialState !== "PREPARED") return; if (!confirm("APPLY 会在控制边界切换 PID。确认小车周围安全并继续？")) return; state.trialState = "APPLYING"; updateUi(); await send(C.command("APPLY", state.trialId)); startHeartbeat(); }
  async function accept() { if (state.trialState !== "TRIAL" || !confirm("确认接受当前候选为新的 champion？")) return; await send(C.command("ACCEPT", state.trialId)); }
  async function rollback() { if (!["PREPARING","PREPARED","APPLYING","TRIAL"].includes(state.trialState)) return; await safeStop(); await send(C.command("ROLLBACK", state.trialId)); }
  function startHeartbeat() { stopHeartbeat(); state.heartbeat = setInterval(() => { if (["APPLYING","TRIAL"].includes(state.trialState)) send(C.command("HB", state.trialId)); }, 500); }
  function stopHeartbeat() { if (state.heartbeat) clearInterval(state.heartbeat); state.heartbeat = null; }

  async function safeStop() { releaseMotion(false); if (state.connected || state.mock) { await send(C.legacyMotion(0, 0, state.currentPid || C.DEFAULT_PID)); if (state.trialId) await send(C.command("STOP", state.trialId)); } }
  function bindMotionButton(button) {
    const [move, pivot] = button.dataset.motion.split(",").map(Number);
    const start = async (event) => { event.preventDefault(); if (!(state.connected || state.mock)) return; if (state.activeMotion && state.activeMotion !== button) await releaseMotion(false); state.activeMotion = button; button.setPointerCapture?.(event.pointerId); button.classList.add("pressed"); await send(C.legacyMotion(move, pivot, state.currentPid || C.DEFAULT_PID)); };
    const stop = (event) => { if (state.activeMotion === button) { event?.preventDefault(); releaseMotion(false); } };
    button.addEventListener("pointerdown", start); button.addEventListener("pointerup", stop); button.addEventListener("pointercancel", stop); button.addEventListener("lostpointercapture", stop);
  }
  async function releaseMotion(silent) { const b = state.activeMotion; state.activeMotion = null; if (b) b.classList.remove("pressed"); if (!silent && (state.connected || state.mock)) await send(C.legacyMotion(0, 0, state.currentPid || C.DEFAULT_PID)); }
  async function motorDiag(action) { if (state.balanceStarted || state.trainingLocked) return toast("诊断只允许平衡启动前且训练未锁定时使用"); await send(C.motorDiag(action)); }

  function startMock() {
    if (state.connected) return toast("请先断开真实设备"); stopMock(); state.mock = true; state.connected = false; state.balanceStarted = true; state.firmware = "RA1.5-MOCK"; connectionLabel("online", "演示模式"); $("deviceName").textContent = "Mock YahBoom_BL"; setPidUi(state.mockPid, true);
    emitMockTelemetry();
    state.mockTimer = setInterval(emitMockTelemetry, 200);
    log("SYS", "Mock mode started"); updateUi(); switchPage("status");
  }
  function emitMockTelemetry() {
    try {
      state.mockClock += 200;
      const a = Math.sin(state.mockClock / 820) * 2.4 + Math.sin(state.mockClock / 210) * .35;
      const g = Math.cos(state.mockClock / 820) * 13;
      mockEmit(C.frame(`T1,F,${state.mockClock},A${a.toFixed(2)},G${g.toFixed(1)},EL${Math.round(g)},ER${Math.round(g + 1)},ML${Math.round(-a * 120)},MR${Math.round(-a * 118)},S${state.firmwareState},L${state.trainingLocked ? 1 : 0},R1,C1${Math.max(0,Math.round(a*20))},C2${Math.max(0,Math.round(-a*20))},C3${Math.max(0,Math.round(a*19))},C4${Math.max(0,Math.round(-a*19))}`));
      if (state.mockClock % 1000 === 0) mockEmit(C.frame(`T1,S,${state.mockClock},V11.82,B0,V0,T0,AP${state.mockPid.AP.toFixed(2)},AD${state.mockPid.AD.toFixed(2)},VP${state.mockPid.VP.toFixed(2)},VI${state.mockPid.VI.toFixed(2)},TP${state.mockPid.TP.toFixed(2)},TD${state.mockPid.TD.toFixed(2)},D0,L${state.trainingLocked ? 1 : 0},R1,RA1.5-MOCK`));
    } catch (e) { log("ERR", `mock telemetry: ${e.message}`); }
  }
  function stopMock() { if (state.mockTimer) clearInterval(state.mockTimer); state.mockTimer = null; if (state.mock) log("SYS", "Mock mode stopped"); state.mock = false; connectionLabel("offline", "未连接"); updateUi(); }
  function mockEmit(raw) { for (const f of state.parser.feed(raw)) handleFrame(f); }
  function mockReceiveCommand(raw) {
    const body = raw.startsWith("$") && raw.includes(",C") ? raw.slice(1, raw.lastIndexOf(",C")) : "";
    if (body === "P1,GET") setTimeout(() => mockEmit(C.frame(`P1,PID,${C.pidCsv(state.mockPid)},${state.firmwareState},${state.trainingLocked ? 1 : 0},1`)), 40);
    else if (body.startsWith("P1,MSET,")) { const v = body.split(",").slice(2).map(Number); C.PID_KEYS.forEach((k,i) => state.mockPid[k] = v[i]); setTimeout(() => mockEmit(C.frame("P1,ACK,MSET,0,0,0")), 40); }
    else if (body === "P1,TRAIN,START") { state.trainingLocked = true; setTimeout(() => mockEmit(C.frame("P1,ACK,TRAIN_START,0,0,1")), 40); }
    else if (body === "P1,TRAIN,STOP") { state.trainingLocked = false; state.firmwareState = 0; setTimeout(() => mockEmit(C.frame("P1,ACK,TRAIN_STOP,0,0,0")), 40); }
    else if (body.startsWith("P1,PREP,")) { state.firmwareState = 1; const id = body.split(",")[2]; setTimeout(() => mockEmit(C.frame(`P1,ACK,PREP,${id},1,1`)), 40); }
    else if (body.startsWith("P1,APPLY,")) { state.firmwareState = 2; const id = body.split(",")[2]; if (state.candidate) state.mockPid = { ...state.candidate.candidate }; setTimeout(() => mockEmit(C.frame(`P1,ACK,APPLY,${id},1,1`)), 40); }
    else if (body.startsWith("P1,ACCEPT,")) { const id = body.split(",")[2]; state.firmwareState = 0; setTimeout(() => mockEmit(C.frame(`P1,ACK,ACCEPT,${id},0,1`)), 40); }
    else if (body.startsWith("P1,ROLLBACK,")) { const id = body.split(",")[2]; state.firmwareState = 0; setTimeout(() => mockEmit(C.frame(`P1,ACK,ROLLBACK,${id},0,1`)), 40); }
    return Promise.resolve();
  }

  function draw() { drawGauge(); drawChart(); }
  function prepCanvas(canvas, height) { const dpr = window.devicePixelRatio || 1, w = canvas.clientWidth || canvas.width, h = height || canvas.clientHeight || canvas.height; if (canvas.width !== Math.round(w*dpr) || canvas.height !== Math.round(h*dpr)) { canvas.width = Math.round(w*dpr); canvas.height = Math.round(h*dpr); } const ctx = canvas.getContext("2d"); ctx.setTransform(dpr,0,0,dpr,0,0); return {ctx,w,h}; }
  function drawGauge() { const {ctx,w,h} = prepCanvas($("angleGauge")); const a = state.fast ? state.fast.angle : 0, r = Math.min(w,h)/2-12, cx=w/2, cy=h/2; ctx.clearRect(0,0,w,h); ctx.lineWidth=10;ctx.lineCap="round";ctx.strokeStyle="#1d3a51";ctx.beginPath();ctx.arc(cx,cy,r,.75*Math.PI,2.25*Math.PI);ctx.stroke();ctx.strokeStyle=Math.abs(a)>10?"#fb4b5c":"#2dd4bf";ctx.beginPath();ctx.arc(cx,cy,r,.75*Math.PI,.75*Math.PI+Math.max(0,Math.min(1,(a+25)/50))*1.5*Math.PI);ctx.stroke();ctx.save();ctx.translate(cx,cy);ctx.rotate(a*Math.PI/180);ctx.strokeStyle="#eff7ff";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(0,8);ctx.lineTo(0,-r+18);ctx.stroke();ctx.restore();ctx.fillStyle="#8fa7ba";ctx.font="12px -apple-system";ctx.textAlign="center";ctx.fillText("±25° guard",cx,cy+7); }
  function drawChart() { const canvas=$("chart"),{ctx,w,h}=prepCanvas(canvas,210);ctx.clearRect(0,0,w,h);ctx.strokeStyle="#1b354b";ctx.lineWidth=1;for(let i=1;i<5;i++){const y=i*h/5;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}const pts=state.chart.filter(p=>Date.now()-p.at<=30000);const series=(key,scale,color)=>{if(pts.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();pts.forEach((p,i)=>{const x=w-(Date.now()-p.at)/30000*w,y=h/2-p[key]*scale;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()};series("angle",h/30,"#2dd4bf");series("gyro",h/300,"#fbbf24"); }

  function download(name, mime, content) { const a=document.createElement("a");a.href=URL.createObjectURL(new Blob([content],{type:mime}));a.download=name;document.body.appendChild(a);a.click();setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove()},1000); }
  function exportCsv() { const cols=["at","ms","angle","gyro","el","er","ml","mr","state","trainingLocked","balanceStarted","c1","c2","c3","c4"]; download(`routea-${Date.now()}.csv`,"text/csv;charset=utf-8","\ufeff"+cols.join(",")+"\n"+state.telemetry.map(x=>cols.map(k=>x[k]).join(",")).join("\n")); }
  function exportJson() { download(`routea-${Date.now()}.json`,"application/json",JSON.stringify({exportedAt:new Date().toISOString(),pid:state.currentPid,metrics:state.metrics,proposal:state.candidate,telemetry:state.telemetry,logs:state.logs},null,2)); }
  function switchPage(name) { document.querySelectorAll(".page").forEach((p)=>p.classList.toggle("active",p.dataset.page===name));document.querySelectorAll("nav button").forEach((b)=>b.classList.toggle("active",b.dataset.nav===name));window.scrollTo({top:0,behavior:"smooth"});setTimeout(draw,80); }
  function bind() {
    document.querySelectorAll("[data-nav]").forEach((b)=>b.addEventListener("click",()=>switchPage(b.dataset.nav)));
    document.querySelectorAll("[data-motion]").forEach(bindMotionButton);
    document.querySelectorAll("[data-diag]").forEach((b)=>b.addEventListener("click",()=>motorDiag(b.dataset.diag)));
    const actions={connect,disconnect:safeDisconnect,stop:safeStop,"get-pid":()=>send(C.getPid()),"manual-pid":manualPid,"train-start":trainStart,"train-stop":trainStop,"baseline-start":baselineStart,"baseline-finish":baselineFinish,harness,llm:requestLlm,"validate-json":validateJson,prepare,apply,accept,rollback,"save-settings":saveSettings,mock:startMock,"export-csv":exportCsv,"export-json":exportJson,"clear-log":()=>{$("log").textContent=""}};
    document.querySelectorAll("[data-action]").forEach((b)=>b.addEventListener("click",()=>actions[b.dataset.action]?.()));
    document.addEventListener("visibilitychange",()=>{if(document.hidden)safeStop().catch(()=>{});});window.addEventListener("pagehide",()=>{releaseMotion(false);});window.addEventListener("resize",draw);
  }
  function boot() {
    initPidUi();loadSettings();bind();updateUi();draw();setInterval(()=>{updateUi();draw()},1000);
    if (!window.isSecureContext) $("secureBanner").textContent="当前页面不是 HTTPS 安全上下文。Bluefy 的 Web Bluetooth 可能不可用，请按 README 用 GitHub Pages 打开。";
    else if (!navigator.bluetooth) $("secureBanner").textContent="当前浏览器未提供 Web Bluetooth。请确认页面是在 Bluefy 内打开，而不是 Safari、Chrome 或“文件”预览。";
    else $("secureBanner").textContent="";
    log("SYS",`页面就绪 secure=${window.isSecureContext} bluetooth=${!!navigator.bluetooth}`);
    if (new URLSearchParams(location.search).get("mock") === "1") setTimeout(startMock, 0);
  }
  boot();
})();
