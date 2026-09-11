(function () {
  "use strict";

  const Protocol = window.BalanceBotProtocol;
  const {
    SERVICE_UUID,
    CHARACTERISTIC_UUID,
    COMMANDS,
    FrameAssembler,
    parseRobotFrame,
    buildPidUpdate,
    isOfficialCommand,
  } = Protocol;

  const $ = (id) => document.getElementById(id);
  const elements = {
    connectionPill: $("connection-pill"), connectionText: $("connection-text"), notice: $("compatibility-notice"),
    secureContext: $("secure-context"), connect: $("connect-btn"), reconnect: $("reconnect-btn"), disconnect: $("disconnect-btn"),
    deviceName: $("device-name"), gattState: $("gatt-state"), charProperties: $("char-properties"),
    rxRate: $("rx-rate"), rxCount: $("rx-count"), txCount: $("tx-count"), lastRx: $("last-rx"),
    runTest: $("run-test-btn"), reportOn: $("report-on-btn"), reportOff: $("report-off-btn"), testResult: $("test-result"),
    telemetryAge: $("telemetry-age"), chartMetric: $("chart-metric"), chart: $("wave-chart"),
    driveUnlock: $("drive-unlock"), emergencyStop: $("emergency-stop"),
    pidRead: $("pid-read-btn"), pidWrite: $("pid-write-btn"),
    rawCommand: $("raw-command"), rawSend: $("raw-send-btn"), log: $("protocol-log"), clearLog: $("clear-log-btn"), exportLog: $("export-log-btn"),
  };

  const checkIds = ["browser", "device", "gatt", "notify", "ack", "telemetry"];
  const sensorKeys = ["LV", "RV", "AC", "GY", "CSB", "VT"];
  const sensorElements = Object.fromEntries(sensorKeys.map((key) => [key, $(`sensor-${key.toLowerCase()}`)]));
  const pidElements = Object.fromEntries(["AP", "AD", "VP", "VI"].map((key) => [key, $(`pid-${key.toLowerCase()}`)]));

  const state = {
    device: null,
    server: null,
    characteristic: null,
    assembler: new FrameAssembler(),
    connected: false,
    notifications: false,
    rxFrames: 0,
    txFrames: 0,
    rxBytes: 0,
    rateWindow: [],
    lastRxAt: 0,
    latestTelemetry: null,
    history: [],
    lastHistoryAt: 0,
    logs: [],
    pendingAck: [],
    test: null,
    driveCommandActive: false,
    connecting: false,
    connectionStage: "idle",
    gattConnectedAt: 0,
  };

  function nowText() {
    return new Date().toLocaleTimeString("zh-CN", { hour12: false, fractionalSecondDigits: 3 });
  }

  function setConnectionStatus(kind, text) {
    elements.connectionPill.className = `connection-pill${kind ? ` ${kind}` : ""}`;
    elements.connectionText.textContent = text;
  }

  function setCheck(name, status) {
    const element = $(`check-${name}`);
    element.classList.remove("pass", "fail");
    if (status === true) element.classList.add("pass");
    if (status === false) element.classList.add("fail");
  }

  function showNotice(message) {
    elements.notice.textContent = message;
    elements.notice.classList.toggle("visible", Boolean(message));
  }

  function log(kind, message) {
    const record = { at: new Date().toISOString(), displayAt: nowText(), kind, message };
    state.logs.push(record);
    if (state.logs.length > 1000) state.logs.splice(0, state.logs.length - 1000);
    const line = document.createElement("span");
    line.className = kind;
    line.textContent = `[${record.displayAt}] ${kind.toUpperCase()}  ${message}\n`;
    elements.log.appendChild(line);
    while (elements.log.childNodes.length > 500) elements.log.removeChild(elements.log.firstChild);
    elements.log.scrollTop = elements.log.scrollHeight;
  }

  function updateAvailability() {
    const connected = state.connected && state.characteristic;
    elements.connect.disabled = !navigator.bluetooth || !window.isSecureContext || state.connected;
    elements.disconnect.disabled = !connected;
    elements.runTest.disabled = !connected || !state.notifications;
    elements.reportOn.disabled = !connected;
    elements.reportOff.disabled = !connected;
    elements.emergencyStop.disabled = !connected;
    elements.pidRead.disabled = !connected;
    elements.pidWrite.disabled = !connected;
    elements.rawSend.disabled = !connected;
    const driveEnabled = Boolean(connected && elements.driveUnlock.checked);
    document.querySelectorAll("[data-drive]").forEach((button) => {
      button.disabled = button.dataset.drive !== "STOP" ? !driveEnabled : !connected;
    });
  }

  async function refreshRememberedDevices() {
    if (!navigator.bluetooth || typeof navigator.bluetooth.getDevices !== "function") return;
    try {
      const devices = await navigator.bluetooth.getDevices();
      elements.reconnect.disabled = devices.length === 0 || state.connected;
      return devices;
    } catch (error) {
      log("err", `读取已授权设备失败：${error.message || error}`);
      return [];
    }
  }

  function resetConnectionState() {
    state.server = null;
    state.characteristic = null;
    state.connected = false;
    state.notifications = false;
    state.pendingAck.length = 0;
    state.driveCommandActive = false;
    elements.gattState.textContent = "未连接";
    elements.charProperties.textContent = "—";
    setConnectionStatus("", "未连接");
    setCheck("gatt", null);
    setCheck("notify", null);
    updateAvailability();
    refreshRememberedDevices();
  }

  function onDisconnected() {
    if (state.connecting) {
      const elapsed = state.gattConnectedAt ? Math.round(performance.now() - state.gattConnectedAt) : 0;
      state.server = null;
      state.characteristic = null;
      state.connected = false;
      state.notifications = false;
      elements.gattState.textContent = "服务发现前断开，准备重试";
      log("sys", `GATT 在“${state.connectionStage}”阶段断开；连接存活约 ${elapsed} ms。`);
      return;
    }
    log("sys", `设备已断开：${state.device?.name || "未知设备"}`);
    if (state.test?.active) finishTest(false, "验收期间连接断开。");
    resetConnectionState();
  }

  async function connectDevice(device) {
    state.device = device;
    state.assembler.reset();
    state.connecting = true;
    setConnectionStatus("connecting", "连接中");
    elements.deviceName.textContent = device.name || "未命名 BLE 设备";
    setCheck("device", true);
    device.removeEventListener("gattserverdisconnected", onDisconnected);
    device.addEventListener("gattserverdisconnected", onDisconnected);
    localStorage.setItem("balancebot-device-id", device.id);
    log("sys", `已授权设备：${device.name || "未命名"}`);

    let lastError;
    const retryDelays = [0, 1200, 2500];
    for (let index = 0; index < retryDelays.length; index += 1) {
      const attempt = index + 1;
      try {
        if (retryDelays[index]) await sleep(retryDelays[index]);
        elements.gattState.textContent = `连接与服务发现 ${attempt}/${retryDelays.length}`;
        log("sys", `GATT 连接尝试 ${attempt}/${retryDelays.length}`);

        state.connectionStage = "建立物理 GATT 连接";
        const server = device.gatt.connected ? device.gatt : await device.gatt.connect();
        state.server = server;
        state.gattConnectedAt = performance.now();
        log("sys", `gatt.connect() 已返回，connected=${server.connected}`);
        if (!server.connected || !device.gatt.connected) throw new Error("GATT connect 返回后设备仍处于断开状态");

        // 断线后旧的 GATT 属性会失效，因此每次重试都重新获取服务和特征。
        // JDY-23 建连后应立即发起服务访问，避免外设在空闲窗口主动断开。
        state.connectionStage = "读取 FFE0 服务";
        const service = await server.getPrimaryService(SERVICE_UUID);
        state.connectionStage = "读取 FFE1 特征";
        const characteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);
        state.characteristic = characteristic;
        state.connected = true;
        elements.gattState.textContent = "已连接";
        const propertyNames = [
          "read", "write", "writeWithoutResponse", "notify", "indicate",
          "authenticatedSignedWrites", "reliableWrite", "writableAuxiliaries",
        ];
        const properties = propertyNames.filter((name) => characteristic.properties[name]).join(", ");
        elements.charProperties.textContent = properties || "浏览器未报告";
        setCheck("gatt", true);

        state.connectionStage = "订阅 FFE1 通知";
        characteristic.removeEventListener("characteristicvaluechanged", onNotification);
        characteristic.addEventListener("characteristicvaluechanged", onNotification);
        await characteristic.startNotifications();
        state.notifications = true;
        state.connecting = false;
        state.connectionStage = "ready";
        setCheck("notify", true);
        setConnectionStatus("connected", "已连接");
        showNotice("");
        log("sys", `FFE1 通知已开启；属性：${properties || "未知"}`);
        updateAvailability();
        await refreshRememberedDevices();
        return;
      } catch (error) {
        lastError = error;
        state.server = null;
        state.characteristic = null;
        state.connected = false;
        state.notifications = false;
        log("err", `GATT 尝试 ${attempt} 失败：${error.message || error}`);
        if (device.gatt.connected) device.gatt.disconnect();
        if (!isRetryableGattError(error) || attempt === retryDelays.length) break;
      }
    }

    state.connecting = false;
    state.connectionStage = "failed";
    setCheck("gatt", false);
    resetConnectionState();
    const detail = lastError?.message || String(lastError || "未知错误");
    throw new Error(`${detail}。请确认选择的是 YahBoom_BL/JDY-23、手机 App 已关闭，然后给蓝牙模块重新上电再试。`);
  }

  function isRetryableGattError(error) {
    const text = `${error?.name || ""} ${error?.message || error}`.toLowerCase();
    return text.includes("disconnected") || text.includes("networkerror") || text.includes("gatt") || text.includes("unreachable");
  }

  async function requestAndConnect() {
    try {
      const device = await navigator.bluetooth.requestDevice({
        filters: [
          { namePrefix: " Ya" },
          { namePrefix: "YahBoom" },
          { namePrefix: "JDY" },
          { services: [SERVICE_UUID] },
        ],
        optionalServices: [SERVICE_UUID],
      });
      await connectDevice(device);
    } catch (error) {
      if (error.name !== "NotFoundError") {
        showNotice(`设备连接失败：${error.message || error}`);
        log("err", `设备选择/连接失败：${error.message || error}`);
      }
    }
  }

  async function reconnectRemembered() {
    const devices = await refreshRememberedDevices();
    if (!devices?.length) return;
    const rememberedId = localStorage.getItem("balancebot-device-id");
    const device = devices.find((item) => item.id === rememberedId) || devices[0];
    try {
      await connectDevice(device);
    } catch (error) {
      showNotice(`重连失败：${error.message || error}`);
    }
  }

  function sleep(milliseconds) {
    return new Promise((resolve) => setTimeout(resolve, milliseconds));
  }

  async function writeBytes(bytes) {
    const characteristic = state.characteristic;
    if (!state.connected || !characteristic) throw new Error("设备未连接");
    const properties = characteristic.properties;
    const chunks = [];
    for (let offset = 0; offset < bytes.length; offset += 20) chunks.push(bytes.slice(offset, offset + 20));

    for (const chunk of chunks) {
      if (properties.write && typeof characteristic.writeValueWithResponse === "function") {
        await characteristic.writeValueWithResponse(chunk);
      } else if (properties.writeWithoutResponse && typeof characteristic.writeValueWithoutResponse === "function") {
        await characteristic.writeValueWithoutResponse(chunk);
      } else if (typeof characteristic.writeValue === "function") {
        await characteristic.writeValue(chunk);
      } else {
        throw new Error("FFE1 不支持网页可用的写入方式");
      }
      if (chunks.length > 1) await sleep(15);
    }
  }

  async function sendFrame(frame, purpose = "command", expectAck = false) {
    if (!isOfficialCommand(frame)) throw new Error("协议帧必须以 $ 开始、以 # 结束，且不超过 79 字符");
    const bytes = new TextEncoder().encode(frame);
    const pending = expectAck ? { purpose, sentAt: performance.now() } : null;
    if (pending) state.pendingAck.push(pending);
    try {
      await writeBytes(bytes);
    } catch (error) {
      if (pending) {
        const index = state.pendingAck.indexOf(pending);
        if (index >= 0) state.pendingAck.splice(index, 1);
      }
      throw error;
    }
    state.txFrames += 1;
    elements.txCount.textContent = String(state.txFrames);
    log("tx", frame);
  }

  async function safeDisconnect() {
    const characteristic = state.characteristic;
    try {
      if (state.connected) {
        await sendFrame(COMMANDS.STOP, "disconnect-stop", false);
        await sendFrame(COMMANDS.AUTO_REPORT_OFF, "disconnect-report-off", true);
        await sleep(80);
      }
    } catch (error) {
      log("err", `断开前停止命令未确认：${error.message || error}`);
    }
    try {
      if (characteristic && state.notifications) await characteristic.stopNotifications();
    } catch (_) {}
    if (state.device?.gatt?.connected) state.device.gatt.disconnect();
    resetConnectionState();
  }

  function onNotification(event) {
    const value = event.target.value;
    const bytes = new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
    state.rxBytes += bytes.length;
    const frames = state.assembler.push(bytes);
    for (const frame of frames) handleFrame(frame);
  }

  function handleFrame(frame) {
    const at = performance.now();
    state.rxFrames += 1;
    state.lastRxAt = Date.now();
    state.rateWindow.push(at);
    while (state.rateWindow.length && at - state.rateWindow[0] > 5000) state.rateWindow.shift();
    elements.rxCount.textContent = String(state.rxFrames);
    elements.lastRx.textContent = nowText();
    log("rx", frame);

    const parsed = parseRobotFrame(frame);
    if (parsed.type === "ack") {
      const pending = state.pendingAck.shift();
      if (pending) {
        const rtt = Math.round(at - pending.sentAt);
        log("sys", `${pending.purpose} 收到通用 ACK，约 ${rtt} ms`);
      }
      if (state.test?.active && at >= state.test.startedAt) {
        state.test.ack = true;
        setCheck("ack", true);
        updateTestResult();
      }
    } else if (parsed.type === "telemetry") {
      state.latestTelemetry = { ...parsed.values, receivedAt: Date.now() };
      if (at - state.lastHistoryAt >= 100) {
        state.history.push({ ...parsed.values, at });
        if (state.history.length > 300) state.history.shift();
        state.lastHistoryAt = at;
      }
      if (state.test?.active && at >= state.test.startedAt) {
        state.test.telemetry = true;
        setCheck("telemetry", true);
        updateTestResult();
      }
    } else if (parsed.type === "pid") {
      for (const key of ["AP", "AD", "VP", "VI"]) {
        if (Number.isFinite(parsed.values[key])) pidElements[key].value = parsed.values[key].toFixed(2);
      }
    } else if (parsed.type === "error") {
      log("err", `机器人协议错误：${parsed.message}`);
    }
  }

  function updateTestResult() {
    if (!state.test?.active) return;
    if (state.test.ack && state.test.telemetry) {
      finishTest(true, "验收通过：PC 指令已到达 STM32，且 PC 已收到传感器回传。实时上报保持开启。");
    } else if (state.test.ack) {
      elements.testResult.textContent = "已收到 STM32 的 $OK#，正在等待传感器帧…";
    } else if (state.test.telemetry) {
      elements.testResult.textContent = "已收到传感器帧，正在等待本次指令的 $OK#…";
    }
  }

  function finishTest(passed, message) {
    if (!state.test) return;
    state.test.active = false;
    clearTimeout(state.test.timer);
    elements.testResult.textContent = message;
    elements.testResult.style.color = passed ? "var(--green)" : "var(--red)";
    log(passed ? "sys" : "err", message);
  }

  async function runLinkTest() {
    setCheck("ack", null);
    setCheck("telemetry", null);
    elements.testResult.style.color = "";
    elements.testResult.textContent = "正在发送停止和开启自动上报命令…";
    const test = { active: true, ack: false, telemetry: false, startedAt: performance.now(), timer: null };
    state.test = test;
    state.pendingAck.length = 0;
    test.timer = setTimeout(() => {
      if (!test.active) return;
      setCheck("ack", test.ack ? true : false);
      setCheck("telemetry", test.telemetry ? true : false);
      finishTest(false, test.ack
        ? "收到 ACK，但 8 秒内没有传感器帧。当前固件可能未调用 SendAutoUp。"
        : "8 秒内未同时收到 ACK 和遥测，请检查固件、设备名称及 FFE1 通知。"
      );
    }, 8000);
    try {
      await sendFrame(COMMANDS.STOP, "验收停止", false);
      await sleep(80);
      test.startedAt = performance.now();
      await sendFrame(COMMANDS.AUTO_REPORT_ON, "验收开启上报", true);
    } catch (error) {
      finishTest(false, `验收发送失败：${error.message || error}`);
    }
  }

  async function sendDrive(commandName) {
    const command = COMMANDS[commandName];
    if (!command) return;
    try {
      await sendFrame(command, `drive-${commandName}`, false);
      state.driveCommandActive = commandName !== "STOP";
    } catch (error) {
      log("err", `控制发送失败：${error.message || error}`);
    }
  }

  function setupDriveControls() {
    document.querySelectorAll("[data-drive]").forEach((button) => {
      button.addEventListener("pointerdown", (event) => {
        event.preventDefault();
        if (button.dataset.drive === "STOP" || elements.driveUnlock.checked) sendDrive(button.dataset.drive);
      });
      button.addEventListener("keydown", (event) => {
        if ((event.key === "Enter" || event.key === " ") && !event.repeat) sendDrive(button.dataset.drive);
      });
      button.addEventListener("keyup", (event) => {
        if ((event.key === "Enter" || event.key === " ") && button.dataset.drive !== "STOP") sendDrive("STOP");
      });
    });
    window.addEventListener("pointerup", () => {
      if (state.driveCommandActive) sendDrive("STOP");
    });
    window.addEventListener("pointercancel", () => {
      if (state.driveCommandActive) sendDrive("STOP");
    });
    window.addEventListener("blur", () => {
      if (state.driveCommandActive && state.connected) sendDrive("STOP");
    });
  }

  function formatValue(value) {
    return Number.isFinite(value) ? value.toFixed(2) : "—";
  }

  function renderTelemetry() {
    const telemetry = state.latestTelemetry;
    if (!telemetry) return;
    for (const key of sensorKeys) {
      const suffix = key === "VT" ? '<span class="unit">V</span>' : "";
      sensorElements[key].innerHTML = `${formatValue(telemetry[key])}${suffix}`;
    }
    const age = Date.now() - telemetry.receivedAt;
    elements.telemetryAge.textContent = age < 1500 ? `${age} ms 前` : `${(age / 1000).toFixed(1)} s 前`;
  }

  function drawChart() {
    const canvas = elements.chart;
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.floor(rect.width * ratio));
    const height = Math.max(1, Math.floor(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, width, height);
    ctx.save();
    ctx.scale(ratio, ratio);
    const w = rect.width;
    const h = rect.height;
    ctx.strokeStyle = "rgba(158,190,211,.12)";
    ctx.lineWidth = 1;
    for (let i = 1; i < 5; i += 1) {
      const y = (h * i) / 5;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }

    const metric = elements.chartMetric.value;
    const points = state.history.filter((item) => Number.isFinite(item[metric]));
    if (points.length < 2) {
      ctx.fillStyle = "#91a8b8";
      ctx.font = "13px Segoe UI";
      ctx.fillText("等待传感器数据…", 14, 24);
      ctx.restore();
      return;
    }
    const values = points.map((item) => item[metric]);
    let min = Math.min(...values);
    let max = Math.max(...values);
    if (min === max) { min -= 1; max += 1; }
    const padding = (max - min) * 0.12;
    min -= padding; max += padding;

    ctx.strokeStyle = "#31d7e8";
    ctx.lineWidth = 2;
    ctx.beginPath();
    points.forEach((point, index) => {
      const x = (index / (points.length - 1)) * w;
      const y = h - ((point[metric] - min) / (max - min)) * (h - 24) - 12;
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.fillStyle = "#91a8b8";
    ctx.font = "11px Segoe UI";
    ctx.fillText(`${metric}  ${values.at(-1).toFixed(2)}`, 12, 18);
    ctx.fillText(max.toFixed(2), w - 58, 18);
    ctx.fillText(min.toFixed(2), w - 58, h - 8);
    ctx.restore();
  }

  function updateStats() {
    const now = performance.now();
    while (state.rateWindow.length && now - state.rateWindow[0] > 5000) state.rateWindow.shift();
    const rate = state.rateWindow.length / 5;
    elements.rxRate.textContent = `${rate.toFixed(1)} 帧/秒`;
    renderTelemetry();
    drawChart();
  }

  async function initialize() {
    const supported = Boolean(navigator.bluetooth);
    setCheck("browser", supported);
    elements.secureContext.textContent = window.isSecureContext ? "安全上下文" : "非安全上下文";
    elements.secureContext.style.color = window.isSecureContext ? "var(--green)" : "var(--red)";
    if (!supported) {
      showNotice("当前浏览器没有 Web Bluetooth。请使用 Windows 版 Chrome 或 Edge，并通过 localhost 打开本页面。");
    } else if (!window.isSecureContext) {
      showNotice("Web Bluetooth 需要安全上下文。请运行本目录 server.py，并访问 http://127.0.0.1:8765/。");
    }
    elements.connect.disabled = !supported || !window.isSecureContext;
    await refreshRememberedDevices();
    updateAvailability();
    setupDriveControls();
    setInterval(updateStats, 100);
    log("sys", "页面已就绪；未改变机器人 UART5 9600 baud 配置。");
  }

  elements.connect.addEventListener("click", requestAndConnect);
  elements.reconnect.addEventListener("click", reconnectRemembered);
  elements.disconnect.addEventListener("click", safeDisconnect);
  elements.runTest.addEventListener("click", runLinkTest);
  elements.reportOn.addEventListener("click", () => sendFrame(COMMANDS.AUTO_REPORT_ON, "开启上报", true).catch((error) => log("err", error.message)));
  elements.reportOff.addEventListener("click", () => sendFrame(COMMANDS.AUTO_REPORT_OFF, "关闭上报", true).catch((error) => log("err", error.message)));
  elements.emergencyStop.addEventListener("click", () => sendDrive("STOP"));
  elements.driveUnlock.addEventListener("change", updateAvailability);
  elements.pidRead.addEventListener("click", () => sendFrame(COMMANDS.PID_READ_AND_RESET, "读取并恢复 PID", true).catch((error) => log("err", error.message)));
  elements.pidWrite.addEventListener("click", async () => {
    try {
      const frame = buildPidUpdate(Object.fromEntries(Object.entries(pidElements).map(([key, input]) => [key, input.value])));
      await sendFrame(frame, "写入 PID", true);
    } catch (error) {
      showNotice(error.message || String(error));
      log("err", error.message || String(error));
    }
  });
  elements.rawSend.addEventListener("click", async () => {
    const frame = elements.rawCommand.value.trim();
    try { await sendFrame(frame, "raw", false); }
    catch (error) { showNotice(error.message || String(error)); log("err", error.message || String(error)); }
  });
  elements.clearLog.addEventListener("click", () => { state.logs.length = 0; elements.log.textContent = ""; });
  elements.exportLog.addEventListener("click", () => {
    const content = state.logs.map((item) => `${item.at}\t${item.kind.toUpperCase()}\t${item.message}`).join("\n");
    const link = document.createElement("a");
    link.href = URL.createObjectURL(new Blob([content], { type: "text/plain;charset=utf-8" }));
    link.download = `balancebot-ble-${new Date().toISOString().replace(/[:.]/g, "-")}.log`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  });
  elements.chartMetric.addEventListener("change", drawChart);
  window.addEventListener("beforeunload", () => {
    if (state.device?.gatt?.connected) state.device.gatt.disconnect();
  });

  initialize();
})();
