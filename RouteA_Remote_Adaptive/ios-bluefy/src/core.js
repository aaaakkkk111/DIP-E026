(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.RouteACore = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const PID_KEYS = Object.freeze(["AP", "AD", "VP", "VI", "TP", "TD"]);
  const RANGES = Object.freeze({
    AP: Object.freeze([20, 288]), AD: Object.freeze([5, 200]),
    VP: Object.freeze([10, 72]), VI: Object.freeze([1, 100]),
    TP: Object.freeze([1, 100]), TD: Object.freeze([1, 100])
  });
  const PRESETS = Object.freeze({
    N: Object.freeze({ name: "Normal / 官方模式1", AP: 96, AD: 48, VP: 62, VI: 31, TP: 17, TD: 20, balanceScale: 1, velocityScale: 1, turnScale: 1 }),
    L: Object.freeze({ name: "Load / 官方模式4", AP: 96, AD: 75, VP: 70, VI: 35, TP: 14, TD: 20, balanceScale: 2, velocityScale: 1.35, turnScale: 1 })
  });
  const ACTIONS = Object.freeze(["switch_preset", "propose_pid", "hold", "rollback", "stop"]);

  function crc16(text) {
    const bytes = new TextEncoder().encode(text);
    let crc = 0xffff;
    for (const byte of bytes) {
      crc ^= byte << 8;
      for (let i = 0; i < 8; i += 1) crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
    return crc;
  }
  function frame(body) { return `$${body},C${crc16(body).toString(16).toUpperCase().padStart(4, "0")}#`; }
  function verify(raw) {
    if (typeof raw !== "string" || raw[0] !== "$" || !raw.endsWith("#")) return false;
    const pos = raw.lastIndexOf(",C");
    if (pos < 0 || raw.length - pos !== 7) return false;
    const expected = Number.parseInt(raw.slice(pos + 2, pos + 6), 16);
    return Number.isFinite(expected) && expected === crc16(raw.slice(1, pos));
  }
  class StreamParser {
    constructor(max = 384) { this.max = max; this.buffer = ""; }
    feed(input) {
      const text = typeof input === "string" ? input : new TextDecoder("ascii").decode(input);
      const out = [];
      for (const ch of text) {
        if (ch === "$") this.buffer = "$";
        else if (this.buffer) {
          this.buffer += ch;
          if (this.buffer.length > this.max) this.buffer = "";
          else if (ch === "#") { out.push(this.buffer); this.buffer = ""; }
        }
      }
      return out;
    }
  }
  function finiteNumber(value, name) {
    const n = Number(value);
    if (value === null || value === undefined || String(value).trim() === "" || !Number.isFinite(n)) throw new Error(`${name} 必须是有限数字`);
    return n;
  }
  function normalizePid(input) {
    const pid = {};
    for (const key of PID_KEYS) pid[key] = finiteNumber(input[key], key);
    return pid;
  }
  function validatePid(input) {
    const pid = normalizePid(input);
    for (const key of PID_KEYS) {
      const [min, max] = RANGES[key];
      if (pid[key] < min || pid[key] > max) throw new Error(`${key} 超出PC保护范围 ${min}..${max}`);
    }
    return pid;
  }
  function pidMatches(actual, expected) {
    if (!actual || !expected) return false;
    return PID_KEYS.every((key) => Number.isFinite(Number(actual[key])) &&
      Math.abs(Number(actual[key]) - Number(expected[key])) <= 0.011);
  }
  function presetPid(profile) {
    const preset = PRESETS[profile];
    if (!preset) throw new Error("配置必须是 N 或 L");
    return normalizePid(preset);
  }
  function pidCsv(input) {
    const pid = normalizePid(input);
    return PID_KEYS.map((key) => pid[key].toFixed(2)).join(",");
  }
  function profileCode(profile) {
    if (profile !== "N" && profile !== "L") throw new Error("配置必须是 N 或 L");
    return profile;
  }
  function getPid() { return frame("P3,GET"); }
  function training(action) { return frame(`P3,TRAIN,${action}`); }
  function command(action, id) { return frame(`P3,${action},${Math.trunc(id)}`); }
  function validateMask(mask) {
    if (!Number.isInteger(mask) || mask < 1 || mask > 7) throw new Error("控制环掩码必须为1..7");
    return mask;
  }
  function mergePid(current, proposed, mask = 7) {
    validateMask(mask);
    const merged = normalizePid(current);
    PID_KEYS.forEach((key, i) => { if (mask & (1 << Math.floor(i / 2))) merged[key] = finiteNumber(proposed[key], key); });
    return validatePid(merged);
  }
  function changedMask(current, proposed) {
    return PID_KEYS.reduce((mask, key, i) => mask | (Math.abs(current[key] - proposed[key]) > 0.011 ? 1 << Math.floor(i / 2) : 0), 0);
  }
  function prepare(id, ttl, profile, pid, mask = 7) {
    return frame(`P3,PREP,${Math.trunc(id)},${Math.trunc(ttl)},${profileCode(profile)},${pidCsv(validatePid(pid))},M${validateMask(mask)}`);
  }
  function manualSet(profile, pid, mask = 7) { return frame(`P3,MSET,${profileCode(profile)},${pidCsv(validatePid(pid))},M${validateMask(mask)}`); }
  function manualPreset(profile) { return frame(`P3,PRESET,${profileCode(profile)}`); }
  function preparePreset(id, ttl, profile) { return frame(`P3,PPRESET,${Math.trunc(id)},${Math.trunc(ttl)},${profileCode(profile)}`); }
  function motorDiag(action) { return frame(`P3,DIAG,${action}`); }
  function legacyMotion(move, pivot, pid = PRESETS.N) {
    // All PID-update flags are zero: fixed filler prevents corrupt telemetry from making STOP oversized.
    const p = presetPid("N");
    return `$${move},${pivot},0,0,0,0,0,AP${p.AP.toFixed(2)},AD${p.AD.toFixed(2)},VP${p.VP.toFixed(2)},VI${p.VI.toFixed(2)},TP${p.TP.toFixed(2)},TD${p.TD.toFixed(2)}#`;
  }
  function bodyParts(raw) {
    if (!verify(raw)) return null;
    return raw.slice(1, raw.lastIndexOf(",C")).split(",");
  }
  function prefixed(value, prefix) {
    if (typeof value !== "string" || !value.startsWith(prefix)) throw new Error(`缺少 ${prefix}`);
    return finiteNumber(value.slice(prefix.length), prefix);
  }
  function profileField(value) {
    if (value === "PN") return "N";
    if (value === "PL") return "L";
    throw new Error("配置字段无效");
  }
  function parseStatus(raw) {
    const x = bodyParts(raw);
    if (!x || x.length < 12 || x[0] !== "P3" || x[1] !== "STATUS") return null;
    try {
      return {
        pid: normalizePid({ AP: x[2], AD: x[3], VP: x[4], VI: x[5], TP: x[6], TD: x[7] }),
        profile: profileField(x[8]), state: prefixed(x[9], "S"),
        trainingLocked: prefixed(x[10], "L") === 1, balanceStarted: prefixed(x[11], "B") === 1
      };
    } catch (_) { return null; }
  }
  function parseFast(raw) {
    const x = bodyParts(raw);
    if (!x || x.length < 14 || x[0] !== "T3" || x[1] !== "F") return null;
    try {
      return {
        ms: finiteNumber(x[2], "ms"), angle: prefixed(x[3], "A"), gyro: prefixed(x[4], "G"),
        osc: prefixed(x[5], "O"), lean: prefixed(x[6], "N"), el: prefixed(x[7], "EL"),
        er: prefixed(x[8], "ER"), ml: prefixed(x[9], "ML"), mr: prefixed(x[10], "MR"),
        state: prefixed(x[11], "S"), trainingLocked: prefixed(x[12], "L") === 1,
        profile: profileField(x[13]), transitionStep: x[14] ? prefixed(x[14], "U") : 0
      };
    } catch (_) { return null; }
  }
  function parseSlow(raw) {
    const x = bodyParts(raw);
    if (!x || x.length < 18 || x[0] !== "T3" || x[1] !== "S") return null;
    try {
      return {
        ms: finiteNumber(x[2], "ms"), battery: prefixed(x[3], "V"),
        angleSqMean: prefixed(x[4], "A2"), gyroSqMean: prefixed(x[5], "G2"),
        peak: prefixed(x[6], "PK"), saturation: prefixed(x[7], "X"),
        pid: normalizePid({ AP: prefixed(x[8], "AP"), AD: prefixed(x[9], "AD"), VP: prefixed(x[10], "VP"), VI: prefixed(x[11], "VI"), TP: prefixed(x[12], "TP"), TD: prefixed(x[13], "TD") }),
        profile: profileField(x[14]), dropped: prefixed(x[15], "D"),
        balanceStarted: prefixed(x[16], "B") === 1, firmware: x[17],
        stackFree: x[18] ? prefixed(x[18], "SF") : null,
        controlGapUs: x[19] ? prefixed(x[19], "DT") : null,
        controlExecUs: x[20] ? prefixed(x[20], "CT") : null
      };
    } catch (_) { return null; }
  }
  function mean(values) { return values.reduce((sum, value) => sum + value, 0) / values.length; }
  function metrics(fastSamples, slowSamples = []) {
    if (!Array.isArray(fastSamples) || fastSamples.length < 3) throw new Error("有效快速遥测样本不足");
    const slow = Array.isArray(slowSamples) ? slowSamples : [];
    const rmsAngle = slow.length ? Math.sqrt(Math.max(0, mean(slow.map((x) => x.angleSqMean)))) : Math.sqrt(mean(fastSamples.map((x) => x.angle * x.angle)));
    const rmsGyro = slow.length ? Math.sqrt(Math.max(0, mean(slow.map((x) => x.gyroSqMean)))) : Math.sqrt(mean(fastSamples.map((x) => x.gyro * x.gyro)));
    const peakAngle = slow.length ? Math.max(...slow.map((x) => x.peak)) : Math.max(...fastSamples.map((x) => Math.abs(x.angle)));
    const saturationRate = slow.length ? mean(slow.map((x) => x.saturation)) : fastSamples.filter((x) => Math.abs(x.ml) >= 2595 || Math.abs(x.mr) >= 2595).length / fastSamples.length;
    const result = {
      rmsAngle, peakAngle, rmsGyro,
      meanOsc: mean(fastSamples.map((x) => x.osc)),
      meanLean: mean(fastSamples.map((x) => x.lean)),
      absLean: mean(fastSamples.map((x) => Math.abs(x.lean))),
      saturationRate,
      motorEffort: mean(fastSamples.map((x) => (Math.abs(x.ml) + Math.abs(x.mr)) / 2)),
      wheelActivity: mean(fastSamples.map((x) => (Math.abs(x.el) + Math.abs(x.er)) / 2)),
      battery: slow.length ? mean(slow.map((x) => x.battery)) : null,
      samples: fastSamples.length,
      durationMs: Math.max(0, fastSamples[fastSamples.length - 1].ms - fastSamples[0].ms)
    };
    result.score = scoreMetrics(result);
    return result;
  }
  function scoreMetrics(m) {
    return 30 * Math.min(m.rmsAngle / 3, 3) +
      15 * Math.min(m.peakAngle / 10, 3) +
      10 * Math.min(m.rmsGyro / 100, 3) +
      15 * Math.min(m.meanOsc / 2, 3) +
      15 * Math.min(m.absLean / 3, 3) +
      10 * Math.min(m.saturationRate, 1) +
      5 * Math.min(m.motorEffort / 2600, 1);
  }
  function metricsDangerous(m) {
    return m.peakAngle >= 20 || m.saturationRate >= 0.5 || (m.battery != null && m.battery < 9.6);
  }
  function validateDecision(raw, currentProfile, currentPid) {
    const value = typeof raw === "string" ? JSON.parse(raw) : raw;
    const keys = ["schema_version", "action", "target_profile", "candidate", "reason", "expected_effect", "confidence"];
    if (!value || Object.keys(value).sort().join() !== keys.slice().sort().join()) throw new Error("LLM JSON 字段不符合schema v3");
    if (value.schema_version !== 3 || !ACTIONS.includes(value.action)) throw new Error("LLM动作无效");
    if (!PRESETS[value.target_profile]) throw new Error("target_profile 必须是 N 或 L");
    if (typeof value.reason !== "string" || !value.reason.trim() || value.reason.length > 500) throw new Error("reason 无效");
    if (typeof value.expected_effect !== "string" || !value.expected_effect.trim() || value.expected_effect.length > 500) throw new Error("expected_effect 无效");
    value.confidence = finiteNumber(value.confidence, "confidence");
    if (value.confidence < 0 || value.confidence > 1) throw new Error("confidence 超出 0..1");
    if (value.action === "switch_preset") value.candidate = presetPid(value.target_profile);
    else if (value.action === "propose_pid") value.candidate = validatePid(value.candidate);
    else value.candidate = normalizePid(currentPid);
    if (value.action === "propose_pid" && value.target_profile !== currentProfile) throw new Error("普通调参不得切换配置，请使用switch_preset");
    return value;
  }
  function mockDecision(profile, pid, m) {
    let action = "propose_pid", target = profile, candidate = normalizePid(pid);
    let reason = "根据窗口指标向较低振荡方向优化。";
    let effect = "降低角度RMS与osc，同时保持PWM余量。";
    if (profile === "N" && (m.absLean > 1.4 || m.motorEffort > 1200)) {
      action = "switch_preset"; target = "L"; candidate = presetPid("L");
      reason = "持续倾斜或电机负担较高，选择官方负重配置。";
      effect = "提高负重条件下的平衡与速度环控制能力。";
    } else {
      const targetAd = profile === "L" ? 82 : 58;
      candidate.AD = Number((candidate.AD + (targetAd - candidate.AD) * 0.55).toFixed(2));
      candidate.AP = Number((candidate.AP + ((profile === "L" ? 100 : 94) - candidate.AP) * 0.35).toFixed(2));
    }
    return validateDecision({ schema_version: 3, action, target_profile: target, candidate, reason, expected_effect: effect, confidence: 0.78 }, profile, pid);
  }

  return {
    PID_KEYS, RANGES, PRESETS, ACTIONS, crc16, frame, verify, StreamParser,
    finiteNumber, normalizePid, validatePid, pidMatches, presetPid, pidCsv, getPid,
    training, command, prepare, manualSet, manualPreset, preparePreset, mergePid, changedMask, validateMask, motorDiag, legacyMotion,
    parseStatus, parseFast, parseSlow, metrics, scoreMetrics, metricsDangerous,
    validateDecision, mockDecision
  };
});

