(function () {
  "use strict";
  const C = window.RouteACore;
  const $ = (id) => document.getElementById(id);
  const state = {
    device: null, server: null, characteristic: null, connected: false, mock: false,
    parser: new C.StreamParser(384), sendChain: Promise.resolve(),
    currentPid: null, profile: "N", balanceStarted: false, trainingLocked: false,
    firmwareState: 0, firmware: "—", fast: null, slow: null,
    rxTimes: [], frameOk: 0, frameError: 0, logs: [], telemetry: [], chart: [],
    manualDirty: false, pendingManual: null, activeMotion: null, hardwareFault: false,
    heartbeat: null, mockTimer: null, mockClock: 0, mockState: 0,
    mockPid: C.presetPid("N"), mockProfile: "N", mockChampionPid: C.presetPid("N"),
    mockChampionProfile: "N", mockCandidatePid: null, mockCandidateProfile: "N",
    auto: {
      enabled: false, phase: "IDLE", cycle: 0, timer: null, collecting: false,
      fastSamples: [], slowSamples: [], baselineMetrics: null, latestMetrics: null,
      decision: null, candidatePid: null, candidateProfile: "N", trialId: 0,
      firstTrialMetrics: null, history: [], rollbackReason: "", requestBusy: false
    }
  };
  let toastTimer = null;
  const writer = new window.RouteASequence.Writer({send, onProgress: (text) => {
    $("sequenceProgress").textContent = text; updateUi();
  }});

  function nowText() { return new Date().toLocaleTimeString("zh-CN", { hour12: false }); }
  function delay(ms) { return new Promise((resolve) => setTimeout(resolve, ms)); }
  function settingNumber(id, fallback) {
    const value = Number($(id).value);
    return Number.isFinite(value) ? value : fallback;
  }
  function toast(message) {
    const element = $("toast");
    element.textContent = message;
    element.classList.add("show");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => element.classList.remove("show"), 2800);
  }
  function log(direction, text) {
    const item = { time: new Date().toISOString(), direction, text };
    state.logs.push(item);
    if (state.logs.length > 2000) state.logs.shift();
    $("log").textContent = state.logs.slice(-140).map((x) => `${x.time.slice(11, 23)} ${x.direction.padEnd(4)} ${x.text}`).join("\n");
    $("log").scrollTop = $("log").scrollHeight;
    $("logCount").textContent = String(state.logs.length);
  }
  function connectionLabel(kind, text) {
    const pill = $("connectionPill");
    pill.className = `pill ${kind}`;
    pill.querySelector("b").textContent = text;
  }
  function profileName(profile) { return profile === "L" ? "Load / 2档" : "Standard / 1档"; }
  function firmwareStateName(value) { return ["IDLE", "PREPARED", "TRIAL"][value] || `UNKNOWN ${value}`; }
  function pidText(pid) { return pid ? C.PID_KEYS.map((key) => `${key}${pid[key].toFixed(2)}`).join(" · ") : "—"; }

  function loadSettings() {
    let saved = {};
    try { saved = JSON.parse(localStorage.getItem("routea-remote-adaptive-settings") || "{}"); } catch (_) {}
    const defaults = {
      serviceUuid: "0000ffe0-0000-1000-8000-00805f9b34fb",
      charUuid: "0000ffe1-0000-1000-8000-00805f9b34fb",
      namePrefix: "YahBoom_BL", chunkSize: "20", chunkDelay: "18",
      llmBase: "https://api.deepseek.com", llmModel: "deepseek-chat",
      baselineMs: "4000", settleMs: "1500", evalMs: "6000", improvementPct: "3"
    };
    for (const [id, fallback] of Object.entries(defaults)) $(id).value = saved[id] ?? fallback;
  }
  function saveSettings() {
    const saved = {};
    for (const id of ["serviceUuid", "charUuid", "namePrefix", "chunkSize", "chunkDelay", "llmBase", "llmModel", "baselineMs", "settleMs", "evalMs", "improvementPct"]) saved[id] = $(id).value.trim();
    localStorage.setItem("routea-remote-adaptive-settings", JSON.stringify(saved));
    toast("设置已保存；API Key未持久化");
  }

  async function connect() {
    if (state.mock) stopMock();
    state.firmware = "—"; state.currentPid = null;
    if (!navigator.bluetooth) {
      toast("当前浏览器没有Web Bluetooth，请在Bluefy中打开");
      switchPage("settings");
      return;
    }
    try {
      const serviceUuid = $("serviceUuid").value.trim().toLowerCase();
      const charUuid = $("charUuid").value.trim().toLowerCase();
      const prefix = $("namePrefix").value.trim();
      connectionLabel("connecting", "选择设备");
      state.device = await navigator.bluetooth.requestDevice({
        filters: prefix ? [{ namePrefix: prefix }, { namePrefix: " Yah" }] : [{ services: [serviceUuid] }],
        optionalServices: [serviceUuid]
      });
      state.device.addEventListener("gattserverdisconnected", onDisconnected);
      $("deviceName").textContent = state.device.name || "BLE设备";
      connectionLabel("connecting", "发现服务");
      state.server = await state.device.gatt.connect();
      const service = await state.server.getPrimaryService(serviceUuid);
      state.characteristic = await service.getCharacteristic(charUuid);
      state.characteristic.addEventListener("characteristicvaluechanged", onNotification);
      await state.characteristic.startNotifications();
      state.connected = true;
      state.parser = new C.StreamParser(384);
      connectionLabel("online", "BLE已连接");
      log("SYS", `BLE UART ready: ${state.device.name || "unknown"}`);
      toast("双向链路已就绪，正在同步固件");
      updateUi();
      await send(C.getPid());
    } catch (error) {
      log("ERR", `${error.name || "Error"}: ${error.message}`);
      toast(`连接失败：${error.message}`);
      await cleanupConnection(false);
    }
  }
  async function cleanupConnection(disconnectGatt) {
    writer.cancel("连接已关闭");
    clearAutoTimer();
    stopHeartbeat();
    state.auto.enabled = false;
    state.auto.collecting = false;
    state.connected = false;
    state.pendingManual = null;
    state.fast = null;
    state.currentPid = null;
    state.firmware = "—";
    state.characteristic = null;
    state.server = null;
    if (disconnectGatt && state.device?.gatt?.connected) state.device.gatt.disconnect();
    connectionLabel("offline", "未连接");
    updateUi();
  }
  function onDisconnected() {
    log("SYS", "BLE disconnected; firmware heartbeat will roll back an active trial");
    cleanupConnection(false);
    toast("蓝牙已断开；活动试验将由固件心跳超时回滚");
  }
  async function safeDisconnect() {
    try {
      if (state.auto.enabled || state.trainingLocked) await stopAuto(true);
      await safeStop();
      await delay(180);
    } catch (_) {}
    if (state.mock) stopMock();
    else await cleanupConnection(true);
  }
  function onNotification(event) {
    const bytes = new Uint8Array(event.target.value.buffer, event.target.value.byteOffset, event.target.value.byteLength);
    for (const raw of state.parser.feed(bytes)) handleFrame(raw);
  }
  async function writeChunks(text) {
    if (state.mock) return mockReceiveCommand(text);
    if (!state.connected || !state.characteristic) throw new Error("BLE UART尚未就绪");
    const bytes = new TextEncoder().encode(text);
    const size = Math.max(1, Math.min(20, settingNumber("chunkSize", 20)));
    const pause = Math.max(0, Math.min(100, settingNumber("chunkDelay", 18)));
    for (let offset = 0; offset < bytes.length; offset += size) {
      const chunk = bytes.slice(offset, offset + size);
      const characteristic = state.characteristic;
      if (typeof characteristic.writeValue === "function") await characteristic.writeValue(chunk);
      else if (typeof characteristic.writeValueWithResponse === "function") await characteristic.writeValueWithResponse(chunk);
      else if (typeof characteristic.writeValueWithoutResponse === "function") await characteristic.writeValueWithoutResponse(chunk);
      else throw new Error("BLE特征不支持写入");
      if (pause && offset + size < bytes.length) await delay(pause);
    }
  }
  function send(text) {
    log("TX", text);
    const task = state.sendChain.then(() => writeChunks(text));
    state.sendChain = task.catch((error) => {
      log("ERR", `write: ${error.message}`);
      toast(`发送失败：${error.message}`);
    });
    return task;
  }

  function handleFrame(raw) {
    log("RX", raw);
    state.rxTimes.push(Date.now());
    state.rxTimes = state.rxTimes.filter((time) => Date.now() - time < 5000);
    $("lastRx").textContent = nowText();
    if ((raw.startsWith("$P3,") || raw.startsWith("$T3,")) && !C.verify(raw)) {
      state.frameError += 1;
      updateUi();
      return;
    }
    if(raw.startsWith("$P1,") || raw.startsWith("$T1,")) {
      state.frameError += 1; toast("检测到旧P1/T1协议，请核对固件与配套网页版本"); return;
    }
    state.frameOk += 1;
    writer.feed(raw);

    const status = C.parseStatus(raw);
    if (status) {
      if (!pidTelemetrySafe(status.pid)) {
        hardwareFailure("STM32读回PID越界或损坏"); return;
      }
      state.currentPid = status.pid;
      state.profile = status.profile;
      state.firmwareState = status.state;
      state.trainingLocked = status.trainingLocked;
      state.balanceStarted = status.balanceStarted;
      confirmManual(status);
      confirmTrial(status);
      syncManualFields();
      toast("STM32状态已同步");
    }
    const fast = C.parseFast(raw);
    if (fast) {
      state.fast = fast;
      state.profile = fast.profile;
      state.firmwareState = fast.state;
      state.trainingLocked = fast.trainingLocked;
      state.telemetry.push({ at: new Date().toISOString(), ...fast });
      if (state.telemetry.length > 15000) state.telemetry.shift();
      state.chart.push({ at: Date.now(), angle: fast.angle, gyro: fast.gyro });
      state.chart = state.chart.filter((point) => Date.now() - point.at <= 60000);
      if (state.auto.collecting) {
        state.auto.fastSamples.push(fast);
        $("windowSamples").textContent = `${state.auto.fastSamples.length}样本`;
      }
      if (state.auto.enabled && state.auto.phase === "APPLYING" && !writer.busy && fast.state === 2 && !state.auto.readbackRequested) {
        state.auto.readbackRequested = true;
        send(C.getPid());
      }
      if (state.auto.enabled && fast.state === 2 && Math.abs(fast.angle) >= 20) requestRollback("倾角达到网页安全阈值");
      if (state.auto.enabled && state.auto.phase === "ROLLING_BACK" && fast.state === 0) afterRollback();
    }
    const slow = C.parseSlow(raw);
    if (slow) {
      state.slow = slow;
      if (!pidTelemetrySafe(slow.pid)) {
        hardwareFailure("STM32遥测PID越界或损坏"); return;
      }
      state.currentPid = slow.pid;
      state.profile = slow.profile;
      state.balanceStarted = slow.balanceStarted;
      state.firmware = slow.firmware;
      if (state.auto.collecting) state.auto.slowSamples.push(slow);
      syncManualFields();
    }
    if (raw.includes("P3,READY,1")) state.balanceStarted = true;
    const ack = raw.match(/\$P3,ACK,([^,]+),(\d+),(\d+),(\d+),P([NL]),C/);
    if (ack) handleAck(ack[1], Number(ack[2]), Number(ack[3]), Number(ack[4]), ack[5]);
    const error = raw.match(/\$P3,ERR,([^,]+)/);
    if (error) handleFirmwareError(error[1]);
    updateUi();
    drawChart();
  }

  function handleAck(command, id, fwState, locked, profile) {
    state.firmwareState = fwState;
    state.trainingLocked = locked === 1;
    state.profile = profile;
    if (command === "PROBE") toast("通信测试已确认，PID和控制状态未修改");
    if (command === "TRAIN_START" && state.auto.enabled && state.auto.phase === "STARTING") {
      toast("训练锁已建立，开始自动基线");
      setTimeout(() => startWindow("BASELINE", settingNumber("baselineMs", 4000)), 400);
    } else if (command === "TRAIN_STOP") state.trainingLocked = false;
    else if (command === "PREP" && state.auto.enabled && state.auto.phase === "PREPARING") {
      if (id !== state.auto.trialId) return;
      state.auto.phase = "APPLYING";
      state.auto.readbackRequested = false;
      clearAutoTimer();
      state.auto.timer = setTimeout(() => requestRollback("实际PID读回确认超时"), 3500);
      send(C.command("APPLY", state.auto.trialId));
      startHeartbeat();
    } else if (command === "APPLY" && state.auto.enabled && state.auto.phase === "APPLYING") startHeartbeat();
    else if (command === "ACCEPT" && state.auto.enabled && state.auto.phase === "ACCEPTING") {
      stopHeartbeat();
      recordHistory("接受", state.auto.latestMetrics);
      state.currentPid = { ...state.auto.candidatePid };
      state.profile = state.auto.candidateProfile;
      state.auto.phase = "ACCEPTED";
      state.auto.baselineMetrics = state.auto.latestMetrics;
      send(C.getPid());
      scheduleNextBaseline(1200);
    } else if (command === "ROLLBACK") state.auto.phase = "ROLLING_BACK";
    else if (command === "ROLLED_BACK" && state.auto.enabled && state.auto.phase === "ROLLING_BACK") afterRollback();
    else if (command === "MSET_APPLIED") {
      if (state.pendingManual) state.pendingManual.appliedAck = true;
      send(C.getPid());
      toast("控制边界直接赋值完成，正在读回实际PID");
    }
    updateUi();
  }
  function pidTelemetrySafe(pid) {
    try { C.validatePid(pid); return true; } catch (_) { return false; }
  }
  function hardwareFailure(reason) {
    if (state.hardwareFault) return;
    writer.cancel(reason);
    state.hardwareFault = true;
    clearAutoTimer(); stopHeartbeat();
    state.auto.enabled = false; state.auto.collecting = false; state.auto.phase = "ERROR";
    state.pendingManual = null; state.currentPid = null;
    for(const key of C.PID_KEYS) $("live"+key).textContent="实际无效";
    $("pidFault").textContent=reason+"；已取消后续参数写入，请保存日志并核对配套固件。";
    $("pidFault").classList.remove("hidden");
    log("ERR", reason + "; training stopped, physical restart required");
    toast(reason + "；停止训练，请断电检查并重启");
    send(C.training("STOP")).catch(() => {});
    send(C.legacyMotion(0, 0)).catch(() => {});
    updateUi();
  }
  function handleFirmwareError(code) {
    if (code === "PID_DATA" || code === "PID_MEMORY" || code === "MEMORY_LATCH") { hardwareFailure(code); return; }
    writer.cancel(`固件拒绝：${code}`);
    state.pendingManual = null;
    toast(`固件拒绝：${code}`);
    log("ERR", `firmware rejected command: ${code}`);
    if (state.auto.enabled) {
      state.auto.enabled = false;
      state.auto.phase = "ERROR";
      clearAutoTimer();
      stopHeartbeat();
    }
  }

  function confirmManual(status) {
    const pending = state.pendingManual;
    if (!pending || !pending.appliedAck) return;
    if (status.profile === pending.profile && C.pidMatches(status.pid, pending.pid)) {
      state.pendingManual = null;
      state.manualDirty = false;
      syncManualFields(true);
      toast("人工PID已应用，STM32读回确认一致");
    } else log("ERR", "MSET_APPLIED received but PID readback differs; waiting for matching status");
  }
  function syncManualFields(force = false) {
    if (!state.currentPid) return;
    for (const key of C.PID_KEYS) {
      $("live" + key).textContent = `实际 ${state.currentPid[key].toFixed(2)}`;
      if (!state.manualDirty || force) $("edit" + key).value = state.currentPid[key].toFixed(2);
    }
    if (!state.manualDirty || force) $("manualProfile").value = state.profile;
  }
  function directFirmwareReady() { return state.mock || state.firmware.startsWith("RA3.0-SEQ1"); }
  function confirmTrial(status) {
    if (!writer.busy && state.auto.enabled && state.auto.phase === "APPLYING" && status.state === 2 &&
        status.profile === state.auto.candidateProfile && C.pidMatches(status.pid, state.auto.candidatePid)) beginSettle();
  }
  function readManualPid(mask = 7) {
    const pid = {};
    for (const key of C.PID_KEYS) pid[key] = $("edit" + key).value;
    return C.mergePid(state.currentPid, pid, mask);
  }
  async function applyManual(profile, pid, mask = 7, preset = false) {
    if (state.hardwareFault) return toast("PID数据异常，请停止试验并核对固件");
    if (!directFirmwareReady()) return toast("需要配套固件RA3.0-SEQ1");
    if (state.trainingLocked || state.auto.enabled) return toast("请先停止自动训练");
    if (!state.currentPid) return toast("请先同步实际PID");
    if (state.pendingManual || writer.busy) return toast("请等待当前逐项更新完成");
    try {
      if (!preset && profile !== state.profile) throw new Error("切换配置请使用预设按钮");
      const valid = preset ? C.presetPid(profile) : C.mergePid(state.currentPid, pid, mask);
      state.pendingManual = { profile, pid:valid, appliedAck:false };
      updateUi();
      const result=await writer.run({id:trialId(),mode:"M",profile,initialProfile:state.profile,initialPid:state.currentPid,targetPid:valid,mask:preset?7:mask});
      state.currentPid=result.pid;state.profile=result.profile;state.pendingManual=null;state.manualDirty=false;
      syncManualFields(true);toast("逐项写入及实际PID读回全部完成");
    } catch(error) { state.pendingManual=null;log("ERR",error.message);toast(error.message); }
    finally { updateUi(); }
  }
  function applyPreset(profile) { return applyManual(profile,C.presetPid(profile),7,true); }

  function clearAutoTimer() {
    if (state.auto.timer) clearTimeout(state.auto.timer);
    state.auto.timer = null;
  }
  function startHeartbeat() {
    stopHeartbeat();
    state.heartbeat = setInterval(() => {
      if (state.auto.enabled && ["APPLYING", "SETTLING", "TRIAL1", "TRIAL2"].includes(state.auto.phase)) send(C.command("HB", state.auto.trialId));
    }, 500);
  }
  function stopHeartbeat() {
    if (state.heartbeat) clearInterval(state.heartbeat);
    state.heartbeat = null;
  }
  function startWindow(kind, durationMs) {
    if (!state.auto.enabled) return;
    clearAutoTimer();
    state.auto.phase = kind;
    state.auto.collecting = true;
    state.auto.fastSamples = [];
    state.auto.slowSamples = [];
    $("windowSamples").textContent = "0样本";
    updateUi();
    state.auto.timer = setTimeout(() => finishWindow(kind), Math.max(1000, durationMs));
  }
  async function finishWindow(kind) {
    if (!state.auto.enabled || state.auto.phase !== kind) return;
    state.auto.collecting = false;
    try {
      const metrics = C.metrics(state.auto.fastSamples, state.auto.slowSamples);
      state.auto.latestMetrics = metrics;
      renderMetrics(metrics);
      if (kind === "BASELINE") {
        if (C.metricsDangerous(metrics)) {
          pauseAutoOnError("基线已危险：请先恢复平衡，禁止应用LLM候选");
          await safeStop();
          return;
        }
        state.auto.baselineMetrics = metrics;
        await decideNext(metrics);
      } else if (kind === "TRIAL1") {
        if (!trialImproved(metrics) || C.metricsDangerous(metrics)) await requestRollback("首个评估窗口未改善或触发风险");
        else {
          state.auto.firstTrialMetrics = metrics;
          startWindow("TRIAL2", settingNumber("evalMs", 6000));
        }
      } else if (kind === "TRIAL2") {
        if (!trialImproved(metrics) || C.metricsDangerous(metrics)) await requestRollback("确认窗口未保持改善");
        else {
          state.auto.phase = "ACCEPTING";
          await send(C.command("ACCEPT", state.auto.trialId));
        }
      }
    } catch (error) {
      log("ERR", `window: ${error.message}`);
      if (kind.startsWith("TRIAL")) await requestRollback("遥测窗口数据不足");
      else pauseAutoOnError(error.message);
    }
  }
  function trialImproved(metrics) {
    const base = state.auto.baselineMetrics;
    if (!base) return false;
    const required = Math.max(0, settingNumber("improvementPct", 3)) / 100;
    return metrics.score <= base.score * (1 - required);
  }
  function beginSettle() {
    if (!state.auto.enabled || state.auto.phase !== "APPLYING") return;
    clearAutoTimer();
    state.auto.phase = "SETTLING";
    updateUi();
    state.auto.timer = setTimeout(() => startWindow("TRIAL1", settingNumber("evalMs", 6000)), Math.max(500, settingNumber("settleMs", 1500)));
  }
  function scheduleNextBaseline(waitMs) {
    if (!state.auto.enabled) return;
    clearAutoTimer();
    state.auto.phase = "HOLD";
    updateUi();
    state.auto.timer = setTimeout(() => startWindow("BASELINE", settingNumber("baselineMs", 4000)), waitMs);
  }
  function trialId() { state.nextTransactionId = ((state.nextTransactionId ?? Date.now()) % 999999) + 1; return state.nextTransactionId; }

  async function decideNext(metrics) {
    if (!state.auto.enabled || state.auto.requestBusy) return;
    state.auto.phase = "LLM";
    state.auto.requestBusy = true;
    updateUi();
    try {
      const decision = await requestLlmDecision(metrics);
      if (!state.auto.enabled) return;
      state.auto.decision = decision;
      state.auto.candidatePid = { ...decision.candidate };
      state.auto.candidateProfile = decision.target_profile;
      renderDecision(decision);
      if (decision.action === "hold") {
        recordHistory("保持", metrics);
        scheduleNextBaseline(1800);
      } else if (decision.action === "stop") {
        recordHistory("LLM停止", metrics);
        await stopAuto(false);
      } else if (decision.action === "rollback") {
        recordHistory("建议回滚", metrics);
        scheduleNextBaseline(1800);
      } else {
        state.auto.cycle += 1;
        state.auto.trialId = trialId();
        state.auto.phase = "APPLYING";
        const result=await writer.run({id:state.auto.trialId,mode:"T",profile:decision.target_profile,
          initialProfile:state.profile,initialPid:state.currentPid,targetPid:decision.candidate});
        if(!state.auto.enabled)return;
        state.auto.candidatePid=result.pid;state.auto.candidateProfile=result.profile;
        startHeartbeat();beginSettle();
      }
    } catch (error) {
      log("ERR", `LLM: ${error.message}`);
      pauseAutoOnError(`LLM请求失败：${error.message}`);
    } finally {
      state.auto.requestBusy = false;
      updateUi();
    }
  }

  async function requestLlmDecision(metrics) {
    if (state.mock) return C.mockDecision(state.profile, state.currentPid, metrics);
    const key = $("llmKey").value.trim();
    if (!key) throw new Error("设置页缺少API Key");
    const system = [
      "PROMPT_VERSION: route-a-remote-adaptive-v3.",
      "You supervise PID adaptation for a two-wheel STM32 balance robot.",
      "The STM32 runs the 5 ms balance loop. You make one slow decision per telemetry window.",
      "There are exactly two profiles: N is official mode 1, L is official mode 4.",
      "L has fixed firmware scales balance 2.0, velocity 1.35, turn 1.0.",
      "Return exactly one JSON object with exactly these fields:",
      "schema_version, action, target_profile, candidate, reason, expected_effect, confidence.",
      "schema_version must be 3.",
      "action must be switch_preset, propose_pid, hold, rollback, or stop.",
      "target_profile must be N or L.",
      "candidate must contain exactly numeric AP,AD,VP,VI,TP,TD.",
      "For switch_preset, use the exact selected official preset.",
      "For propose_pid, target_profile MUST equal current_profile. Only switch_preset may change profile or fixed multipliers. There is no relative percentage step limit, but every value must stay inside the supplied absolute ranges. Only changed control loops will be written.",
      "Prefer AP/AD changes for stationary balance data. Change VP/VI only when speed evidence exists and TP/TD only when turning evidence exists.",
      "Do not react to a single impulse by switching profiles. Persistent load/effort/lean may justify L.",
      "Never output Markdown, motor commands, thresholds, or claims that a candidate was accepted."
    ].join(" ");
    const recent = state.auto.history.slice(-6).map((item) => ({ action: item.action, profile: item.profile, score: item.score, result: item.result }));
    const user = {
      schema_version: 3, current_profile: state.profile, current_pid: state.currentPid,
      official_presets: { N: C.presetPid("N"), L: C.presetPid("L") },
      fixed_profile_scales: { N: { balance: 1, velocity: 1, turn: 1 }, L: { balance: 2, velocity: 1.35, turn: 1 } },
      absolute_pc_protection_ranges: C.RANGES, metrics, recent_history: recent,
      context: {
        stationary_balance: Math.abs(state.fast?.el || 0) + Math.abs(state.fast?.er || 0) < 20,
        automatic_cycle: state.auto.cycle,
        note: "Lower normalized score is better. A deterministic safety layer evaluates and may roll back your proposal."
      }
    };
    const base = $("llmBase").value.trim().replace(/\/$/, "");
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 30000);
    try {
      const response = await fetch(`${base}/chat/completions`, {
        method: "POST", signal: controller.signal,
        headers: { Authorization: `Bearer ${key}`, "Content-Type": "application/json" },
        body: JSON.stringify({ model: $("llmModel").value.trim(), response_format: { type: "json_object" }, messages: [{ role: "system", content: system }, { role: "user", content: JSON.stringify(user) }] })
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}: ${(await response.text()).slice(0, 180)}`);
      const body = await response.json();
      const raw = body.choices?.[0]?.message?.content;
      if (!raw) throw new Error("响应中没有choices[0].message.content");
      return C.validateDecision(raw, state.profile, state.currentPid);
    } finally { clearTimeout(timeout); }
  }

  async function startAuto() {
    if (!(state.connected || state.mock)) return toast("请先连接小车");
    if (state.hardwareFault) return toast("固件数据故障锁定，请断电重启并刷新页面");
    if (!directFirmwareReady()) return toast("请等待并确认固件版本RA3.0-SEQ1");
    if (!state.balanceStarted) return toast("请先在小车端按键启动平衡");
    if (!state.currentPid) return toast("尚未同步实际PID");
    if (writer.busy || state.pendingManual || state.fast?.transitionStep) return toast("请等待人工参数应用完成");
    if (!state.mock && !$("llmKey").value.trim()) { switchPage("settings"); return toast("请先填写LLM API Key"); }
    if (state.auto.enabled) return;
    state.auto.enabled = true;
    state.auto.phase = "STARTING";
    state.auto.cycle = 0;
    state.auto.history = [];
    state.auto.decision = null;
    state.auto.baselineMetrics = null;
    state.auto.latestMetrics = null;
    state.auto.rollbackReason = "";
    updateUi();
    await send(C.training("START"));
  }
  async function stopAuto(silent) {
    writer.cancel("自动训练已停止");
    clearAutoTimer();
    state.auto.collecting = false;
    state.auto.enabled = false;
    stopHeartbeat();
    try {
      if (state.firmwareState !== 0 && state.auto.trialId) {
        await send(C.command("ROLLBACK", state.auto.trialId));
        await delay(180);
      }
      if (state.trainingLocked) await send(C.training("STOP"));
    } catch (_) {}
    state.auto.phase = "IDLE";
    if (!silent) toast("自动训练已停止；活动候选已请求回滚");
    updateUi();
  }
  function pauseAutoOnError(message) {
    clearAutoTimer();
    stopHeartbeat();
    state.auto.enabled = false;
    state.auto.collecting = false;
    state.auto.phase = "ERROR";
    toast(message);
    if (state.trainingLocked) send(C.training("STOP"));
    updateUi();
  }
  async function requestRollback(reason) {
    writer.cancel(reason);
    if (!state.auto.enabled || state.auto.phase === "ROLLING_BACK") return;
    clearAutoTimer();
    state.auto.collecting = false;
    state.auto.rollbackReason = reason;
    state.auto.phase = "ROLLING_BACK";
    recordHistory("回滚", state.auto.latestMetrics, reason);
    updateUi();
    await send(C.command("ROLLBACK", state.auto.trialId));
  }
  function afterRollback() {
    stopHeartbeat();
    state.auto.phase = "ROLLED_BACK";
    state.auto.trialId = 0;
    send(C.getPid());
    scheduleNextBaseline(1200);
  }

  function recordHistory(result, metrics, note = "") {
    const decision = state.auto.decision;
    state.auto.history.push({
      cycle: state.auto.cycle, at: new Date().toISOString(),
      action: decision?.action || "none", profile: decision?.target_profile || state.profile,
      pid: decision?.candidate || state.currentPid, score: metrics?.score ?? null, result, note
    });
    if (state.auto.history.length > 100) state.auto.history.shift();
    renderHistory();
  }
  function renderMetrics(metrics) {
    const values = metrics ? {
      metricScore: metrics.score.toFixed(2), metricRms: `${metrics.rmsAngle.toFixed(3)}°`,
      metricPeak: `${metrics.peakAngle.toFixed(2)}°`, metricGyro: metrics.rmsGyro.toFixed(2),
      metricOsc: metrics.meanOsc.toFixed(3), metricLean: `${metrics.meanLean.toFixed(3)}°`,
      metricSat: `${(metrics.saturationRate * 100).toFixed(1)}%`, metricEffort: metrics.motorEffort.toFixed(0)
    } : {};
    for (const id of ["metricScore", "metricRms", "metricPeak", "metricGyro", "metricOsc", "metricLean", "metricSat", "metricEffort"]) $(id).textContent = values[id] || "--";
  }
  function renderDecision(decision) {
    if (!decision) return;
    $("decisionAction").textContent = decision.action;
    $("decisionProfile").textContent = profileName(decision.target_profile);
    $("decisionPid").textContent = pidText(decision.candidate);
    $("decisionReason").textContent = decision.reason;
    $("decisionEffect").textContent = decision.expected_effect;
  }
  function renderHistory() {
    const rows = state.auto.history.slice().reverse();
    $("historyBody").innerHTML = rows.length ? rows.map((item) => `<tr><td>${item.cycle}</td><td>${escapeHtml(item.action)}</td><td>${item.profile}</td><td>${item.score == null ? "—" : item.score.toFixed(2)}</td><td>${escapeHtml(item.result)}</td></tr>`).join("") : '<tr><td colspan="5">尚无记录</td></tr>';
  }
  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[ch]));
  }

  async function safeStop() {
    if(writer.busy)writer.cancel("已按停止按钮，取消后续参数");
    releaseMotion(true);
    if (state.connected || state.mock) {
      await send(C.legacyMotion(0, 0, state.currentPid || C.PRESETS.N));
      if (state.auto.trialId && state.firmwareState !== 0) await send(C.command("STOP", state.auto.trialId));
    }
  }
  function bindMotionButton(button) {
    const [move, pivot] = button.dataset.motion.split(",").map(Number);
    const start = async (event) => {
      event.preventDefault();
      if (!(state.connected || state.mock)) return;
      state.activeMotion = button;
      button.classList.add("pressed");
      button.setPointerCapture?.(event.pointerId);
      await send(C.legacyMotion(move, pivot, state.currentPid || C.PRESETS.N));
    };
    const stop = (event) => {
      if (state.activeMotion === button) {
        event?.preventDefault();
        releaseMotion(false);
      }
    };
    button.addEventListener("pointerdown", start);
    button.addEventListener("pointerup", stop);
    button.addEventListener("pointercancel", stop);
    button.addEventListener("lostpointercapture", stop);
  }
  async function releaseMotion(silent) {
    const button = state.activeMotion;
    state.activeMotion = null;
    if (button) button.classList.remove("pressed");
    if (!silent && (state.connected || state.mock)) await send(C.legacyMotion(0, 0, state.currentPid || C.PRESETS.N));
  }

  function updateUi() {
    const connected = state.connected || state.mock;
    $("rxHz").textContent = `${(state.rxTimes.length / 5).toFixed(1)} Hz`;
    $("frameCounts").textContent = `${state.frameOk} / ${state.frameError}`;
    $("angleHero").textContent = state.fast ? `${state.fast.angle.toFixed(2)}°` : "--.-°";
    $("gyroHero").textContent = state.fast ? `角速度 ${state.fast.gyro.toFixed(1)} raw` : "角速度 --.- raw";
    $("profileHero").textContent = profileName(state.profile);
    $("autoHero").textContent = state.auto.enabled ? "运行中" : "未启动";
    $("battery").textContent = state.slow ? `${state.slow.battery.toFixed(2)} V` : "-- V";
    $("encoders").textContent = state.fast ? `${state.fast.el} / ${state.fast.er}` : "-- / --";
    $("motors").textContent = state.fast ? `${state.fast.ml} / ${state.fast.mr}` : "-- / --";
    $("oscValue").textContent = state.fast ? state.fast.osc.toFixed(3) : "--";
    $("leanValue").textContent = state.fast ? `${state.fast.lean.toFixed(3)}°` : "--°";
    $("rmsValue").textContent = state.auto.latestMetrics ? `${state.auto.latestMetrics.rmsAngle.toFixed(3)}°` : "--°";
    $("scoreValue").textContent = state.auto.latestMetrics ? state.auto.latestMetrics.score.toFixed(2) : "--";
    $("firmware").textContent = state.firmware;
    $("balanceState").textContent = state.balanceStarted ? "已启动" : "未启动";
    $("trainingLock").textContent = state.trainingLocked ? "已锁定" : "未锁定";
    $("firmwareState").textContent = firmwareStateName(state.firmwareState);
    $("dropped").textContent = state.slow ? String(state.slow.dropped) : "—";
    $("stackFree").textContent = state.slow?.stackFree != null ? `${state.slow.stackFree} B / 4096 B` : "—";
    $("controlGap").textContent = state.slow?.controlGapUs != null ? `${(state.slow.controlGapUs / 1000).toFixed(3)} ms` : "—";
    $("controlExec").textContent = state.slow?.controlExecUs != null ? `${(state.slow.controlExecUs / 1000).toFixed(3)} ms` : "—";
    $("activeProfileTag").textContent = `当前 ${state.profile}`;
    $("presetNormal").classList.toggle("active", state.profile === "N");
    $("presetLoad").classList.toggle("active", state.profile === "L");
    $("manualLockTag").textContent = state.trainingLocked ? "训练锁定" : state.pendingManual ? "等待读回确认" : !directFirmwareReady() ? "等待固件版本同步" : "可编辑";
    $("applyManual").textContent = state.pendingManual ? "等待STM32确认…" : "更新全部PID";
    const manualDisabled = writer.busy || state.hardwareFault || !directFirmwareReady() || !connected || !state.currentPid || state.trainingLocked || state.auto.enabled || !!state.pendingManual;
    for (const id of ["presetNormal", "presetLoad", "applyManual", "applyBalance", "applyVelocity", "applyTurn", ...C.PID_KEYS.map((key) => "edit" + key)]) $(id).disabled = manualDisabled;
    document.querySelectorAll("[data-motion]").forEach((button) => button.disabled = !connected);
    $("autoPhaseTag").textContent = state.auto.enabled ? "运行中" : state.auto.phase === "ERROR" ? "错误" : "停止";
    $("autoPhase").textContent = state.auto.phase;
    $("autoCycle").textContent = String(state.auto.cycle);
    $("autoNext").textContent = nextAutoText();
    $("manualProfile").disabled = true;
    $("autoStart").disabled = writer.busy || state.hardwareFault || !directFirmwareReady() || !connected || !state.balanceStarted || state.auto.enabled || !state.currentPid || !!state.pendingManual;
    $("autoPause").disabled = !state.auto.enabled && !state.trainingLocked;
  }
  function nextAutoText() {
    if(writer.busy)return writer.message;
    const map = {
      IDLE: "点击开始自动LLM调参", STARTING: "等待STM32训练锁",
      BASELINE: "采集当前基线", LLM: "等待LLM决策", PREPARING: "暂存候选",
      APPLYING: "下个控制边界赋值，等待读回实际PID", SETTLING: "排除切换瞬态", TRIAL1: "首个实车评估窗口",
      TRIAL2: "确认改善是否保持", ACCEPTING: "接受为新基准", ACCEPTED: "同步新基准",
      ROLLING_BACK: "恢复上一组确认参数", ROLLED_BACK: "同步回滚结果",
      HOLD: "保持当前参数后进入下一轮", ERROR: "检查日志与LLM设置"
    };
    return map[state.auto.phase] || state.auto.phase;
  }

  function drawChart() {
    const canvas = $("chart");
    const dpr = window.devicePixelRatio || 1;
    const width = canvas.clientWidth || 320, height = 220;
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.strokeStyle = "#1d3a51"; ctx.lineWidth = 1;
    for (let i = 1; i < 5; i += 1) {
      const y = i * height / 5;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }
    const points = state.chart.filter((point) => Date.now() - point.at <= 30000);
    function series(key, scale, color) {
      if (points.length < 2) return;
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
      points.forEach((point, index) => {
        const x = width - (Date.now() - point.at) / 30000 * width;
        const y = height / 2 - point[key] * scale;
        if (index) ctx.lineTo(x, y); else ctx.moveTo(x, y);
      });
      ctx.stroke();
    }
    series("angle", height / 30, "#2dd4bf");
    series("gyro", height / 300, "#fbbf24");
  }

  function switchPage(name) {
    document.querySelectorAll(".page").forEach((page) => page.classList.toggle("active", page.dataset.page === name));
    document.querySelectorAll("nav button").forEach((button) => button.classList.toggle("active", button.dataset.nav === name));
    window.scrollTo({ top: 0, behavior: "smooth" });
    setTimeout(drawChart, 60);
  }
  function download(name, mime, content) {
    const anchor = document.createElement("a");
    anchor.href = URL.createObjectURL(new Blob([content], { type: mime }));
    anchor.download = name;
    document.body.appendChild(anchor);
    anchor.click();
    setTimeout(() => { URL.revokeObjectURL(anchor.href); anchor.remove(); }, 1000);
  }
  function exportCsv() {
    const columns = ["at", "ms", "angle", "gyro", "osc", "lean", "el", "er", "ml", "mr", "state", "trainingLocked", "profile"];
    const rows = state.telemetry.map((item) => columns.map((key) => item[key]).join(","));
    download(`routea-telemetry-${Date.now()}.csv`, "text/csv;charset=utf-8", "\ufeff" + columns.join(",") + "\n" + rows.join("\n"));
  }
  function exportJson() {
    download(`routea-session-${Date.now()}.json`, "application/json", JSON.stringify({
      exportedAt: new Date().toISOString(), currentPid: state.currentPid, profile: state.profile,
      latestMetrics: state.auto.latestMetrics, decisions: state.auto.history,
      telemetry: state.telemetry, logs: state.logs
    }, null, 2));
  }

  function startMock() {
    if (state.connected) return toast("请先断开真实设备");
    stopMock();
    state.mock = true;
    state.hardwareFault = false;
    $("pidFault").classList.add("hidden");
    state.balanceStarted = true;
    state.trainingLocked = false;
    state.mockState = 0;
    state.mockClock = 0;
    state.mockProfile = "N";
    state.mockPid = C.presetPid("N");
    state.mockChampionPid = { ...state.mockPid };
    state.mockChampionProfile = "N";
    state.currentPid = { ...state.mockPid };
    state.profile = "N";
    state.firmware = "RA3.0-SEQ1-MOCK";
    $("deviceName").textContent = "Mock YahBoom_BL";
    connectionLabel("online", "模拟设备");
    state.mockTimer = setInterval(emitMockTelemetry, 200);
    emitMockTelemetry();
    syncManualFields(true);
    updateUi();
    switchPage("status");
    toast("模拟设备已启动；自动模式使用本地模拟决策");
  }
  function stopMock() {
    if (state.mockTimer) clearInterval(state.mockTimer);
    state.mockTimer = null;
    state.mock = false;
    if (!state.connected) connectionLabel("offline", "未连接");
  }
  function emitMockTelemetry() {
    if (!state.mock) return;
    state.mockClock += 200;
    const target = state.mockProfile === "L" ? { AP: 100, AD: 82 } : { AP: 94, AD: 58 };
    const error = Math.abs(state.mockPid.AP - target.AP) / 25 + Math.abs(state.mockPid.AD - target.AD) / 35;
    const amplitude = 0.35 + Math.min(2.4, error);
    const angle = Math.sin(state.mockClock / 210) * amplitude + Math.sin(state.mockClock / 73) * amplitude * 0.22;
    const gyro = Math.cos(state.mockClock / 210) * amplitude * 45;
    const osc = amplitude * 0.65;
    const lean = state.mockProfile === "L" ? 0.25 : 0.35;
    const ml = Math.round(-angle * 430), mr = Math.round(-angle * 420);
    mockEmit(C.frame(`T3,F,${state.mockClock},A${angle.toFixed(2)},G${gyro.toFixed(1)},O${osc.toFixed(3)},N${lean.toFixed(3)},EL${Math.round(gyro / 8)},ER${Math.round(gyro / 8 + 1)},ML${ml},MR${mr},S${state.mockState},L${state.trainingLocked ? 1 : 0},P${state.mockProfile}`));
    if (state.mockClock % 1000 === 0) {
      mockEmit(C.frame(`T3,S,${state.mockClock},V11.82,A2${(amplitude * amplitude / 2).toFixed(3)},G2${(amplitude * amplitude * 1012).toFixed(3)},PK${(amplitude * 1.22).toFixed(2)},X0.000,AP${state.mockPid.AP.toFixed(2)},AD${state.mockPid.AD.toFixed(2)},VP${state.mockPid.VP.toFixed(2)},VI${state.mockPid.VI.toFixed(2)},TP${state.mockPid.TP.toFixed(2)},TD${state.mockPid.TD.toFixed(2)},P${state.mockProfile},D0,B1,RA3.0-SEQ1-MOCK`));
    }
  }
  function mockEmit(raw) { handleFrame(raw); }
  function mockReceiveCommand(raw) {
    if (!raw.startsWith("$P3,") || !C.verify(raw)) return Promise.resolve();
    const body = raw.slice(1, raw.lastIndexOf(",C")), parts = body.split(","), command = parts[1];
    const ack = (name, id = state.auto.trialId) => setTimeout(() => mockEmit(C.frame(`P3,ACK,${name},${id},${state.mockState},${state.trainingLocked ? 1 : 0},P${state.mockProfile}`)), 30);
    if (command === "GET") setTimeout(() => mockEmit(C.frame(`P3,STATUS,${C.pidCsv(state.mockPid)},P${state.mockProfile},S${state.mockState},L${state.trainingLocked ? 1 : 0},B1`)), 30);
    else if (command === "PROBE") ack("PROBE", 0);
    else if (command === "TRAIN" && parts[2] === "START") { state.trainingLocked = true; ack("TRAIN_START", 0); }
    else if (command === "TRAIN" && parts[2] === "STOP") { state.trainingLocked = false; state.mockState = 0; ack("TRAIN_STOP", 0); }
    else if(command === "BSEQ") {
      state.mockSeqId=Number(parts[2]);state.mockSeqManual=parts[4]==="M";state.mockSeqProfile=parts[5];
      state.mockChampionPid={...state.mockPid};state.mockChampionProfile=state.mockProfile;
      state.mockState=1;ack("SEQ_BEGIN",state.mockSeqId);
    } else if(command === "ONE") {
      const id=Number(parts[2]),step=Number(parts[3]),key=parts[4],value=Number(parts[5]);
      setTimeout(()=>{
        state.mockPid={...state.mockPid,[key]:value};state.mockState=2;
        mockEmit(C.frame(`P3,SEQACK,ONE_APPLIED,${id},${step},${key},${value.toFixed(2)}`));
      },5);
    } else if(command === "SGET") {
      setTimeout(()=>mockEmit(C.frame(`P3,STATUS,${C.pidCsv(state.mockPid)},P${state.mockProfile},S${state.mockState},L${state.trainingLocked?1:0},B1,Q${parts[2]},I${parts[3]}`)),30);
    } else if(command === "FSEQ") {
      setTimeout(()=>{
        state.mockProfile=state.mockSeqProfile;state.mockState=state.mockSeqManual?0:2;
        if(state.mockSeqManual){state.mockChampionPid={...state.mockPid};state.mockChampionProfile=state.mockProfile;}
        ack("SEQ_DONE",Number(parts[2]));
      },5);
    } else if (command === "HB") ack("HB", Number(parts[2]));
    else if (command === "ACCEPT") {
      state.mockChampionPid = { ...state.mockPid }; state.mockChampionProfile = state.mockProfile;
      state.mockState = 0; ack("ACCEPT", Number(parts[2]));
    } else if (command === "ROLLBACK") {
      state.mockPid = { ...state.mockChampionPid }; state.mockProfile = state.mockChampionProfile;
      state.mockState = 0; ack("ROLLED_BACK", Number(parts[2])); setTimeout(emitMockTelemetry, 70);
    } else if (command === "STOP") ack("STOP", Number(parts[2]));
    return Promise.resolve();
  }

  function bind() {
    document.querySelectorAll("[data-nav]").forEach((button) => button.addEventListener("click", () => switchPage(button.dataset.nav)));
    document.querySelectorAll("[data-motion]").forEach(bindMotionButton);
    $("presetNormal").addEventListener("click", () => applyPreset("N"));
    $("presetLoad").addEventListener("click", () => applyPreset("L"));
    for (const key of C.PID_KEYS) $("edit" + key).addEventListener("input", () => { state.manualDirty = true; });
    $("manualProfile").addEventListener("change", () => { state.manualDirty = true; });
    const actions = {
      connect, disconnect: safeDisconnect, "get-pid": () => send(C.getPid()),
      "manual-apply": () => applyManual(state.profile, readManualPid()),
      "manual-balance": () => applyManual(state.profile, readManualPid(1), 1),
      "manual-velocity": () => applyManual(state.profile, readManualPid(2), 2),
      "manual-turn": () => applyManual(state.profile, readManualPid(4), 4),
      "auto-start": startAuto, "auto-pause": () => stopAuto(false),
      probe: () => send(C.frame("P3,PROBE")), stop: safeStop, "save-settings": saveSettings, mock: startMock,
      "export-csv": exportCsv, "export-json": exportJson,
      "clear-log": () => { $("log").textContent = ""; }
    };
    document.querySelectorAll("[data-action]").forEach((button) => button.addEventListener("click", () => { try { Promise.resolve(actions[button.dataset.action]?.()).catch((error) => toast(error.message)); } catch (error) { toast(error.message); } }));
    document.addEventListener("visibilitychange", () => {
      if (document.hidden && state.auto.enabled) {
        log("SYS", "Page hidden: pausing automatic training and requesting rollback");
        stopAuto(true);
      }
      if (document.hidden) releaseMotion(false);
    });
    window.addEventListener("pagehide", () => releaseMotion(false));
    window.addEventListener("resize", drawChart);
  }
  function boot() {
    loadSettings(); bind(); syncManualFields(); renderMetrics(null); renderHistory(); updateUi(); drawChart();
    setInterval(() => { updateUi(); drawChart(); }, 1000);
    if (!navigator.bluetooth) {
      $("secureBanner").classList.remove("hidden");
      $("secureBanner").textContent = "当前页面没有Web Bluetooth。请使用Bluefy打开；Safari、iOS Chrome和普通文件预览无法连接。";
    } else if (!window.isSecureContext) {
      $("secureBanner").classList.remove("hidden");
      $("secureBanner").textContent = "当前不是标准HTTPS安全上下文；只有Bluefy明确提供Web Bluetooth时才可继续。";
    }
    log("SYS", `页面就绪 secure=${window.isSecureContext} bluetooth=${!!navigator.bluetooth}`);
  }
  boot();
})();
