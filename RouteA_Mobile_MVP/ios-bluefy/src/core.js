(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.RouteACore = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const DEFAULT_PID = Object.freeze({ AP: 96, AD: 48, VP: 62, VI: 31, TP: 17, TD: 20 });
  const PID_KEYS = Object.freeze(["AP", "AD", "VP", "VI", "TP", "TD"]);

  function crc16(text) {
    const bytes = new TextEncoder().encode(text);
    let crc = 0xffff;
    for (const byte of bytes) {
      crc ^= byte << 8;
      for (let i = 0; i < 8; i += 1) {
        crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
      }
    }
    return crc;
  }

  function frame(body) {
    return `$${body},C${crc16(body).toString(16).toUpperCase().padStart(4, "0")}#`;
  }

  function verify(raw) {
    if (typeof raw !== "string" || raw[0] !== "$" || !raw.endsWith("#")) return false;
    const p = raw.lastIndexOf(",C");
    if (p < 0 || raw.length - p !== 7) return false;
    const expected = Number.parseInt(raw.slice(p + 2, p + 6), 16);
    return Number.isFinite(expected) && expected === crc16(raw.slice(1, p));
  }

  class StreamParser {
    constructor(max = 256) { this.max = max; this.buffer = ""; }
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
    if (!Number.isFinite(n)) throw new Error(`${name} 必须是有限数字`);
    return n;
  }

  function normalizePid(input) {
    const p = {};
    for (const key of PID_KEYS) p[key] = finiteNumber(input[key], key);
    return p;
  }

  function pidCsv(pid) {
    const p = normalizePid(pid);
    return PID_KEYS.map((key) => p[key].toFixed(2)).join(",");
  }

  function command(name, id) { return frame(`P1,${name},${Math.trunc(id)}`); }
  function getPid() { return frame("P1,GET"); }
  function training(action) { return frame(`P1,TRAIN,${action}`); }
  function manualSet(pid) { return frame(`P1,MSET,${pidCsv(pid)}`); }
  function prepare(id, ttl, pid) { return frame(`P1,PREP,${Math.trunc(id)},${Math.trunc(ttl)},${pidCsv(pid)}`); }
  function motorDiag(action) { return frame(`P1,DIAG,${action}`); }

  function legacyMotion(move, pivot, pid = DEFAULT_PID) {
    const p = normalizePid(pid);
    return `$${move},${pivot},0,0,0,0,0,AP${p.AP.toFixed(2)},AD${p.AD.toFixed(2)},VP${p.VP.toFixed(2)},VI${p.VI.toFixed(2)},TP${p.TP.toFixed(2)},TD${p.TD.toFixed(2)}#`;
  }

  function bodyParts(raw) {
    if (!verify(raw)) return null;
    return raw.slice(1, raw.lastIndexOf(",C")).split(",");
  }

  function parsePidStatus(raw) {
    const x = bodyParts(raw);
    if (!x || x.length < 11 || x[0] !== "P1" || x[1] !== "PID") return null;
    try {
      return {
        pid: normalizePid({ AP: x[2], AD: x[3], VP: x[4], VI: x[5], TP: x[6], TD: x[7] }),
        state: Number.parseInt(x[8], 10), trainingLocked: x[9] === "1", balanceStarted: x[10] === "1"
      };
    } catch (_) { return null; }
  }

  function parseFast(raw) {
    const x = bodyParts(raw);
    if (!x || x.length < 10 || x[0] !== "T1" || x[1] !== "F") return null;
    const n = (i, prefix) => finiteNumber(x[i].slice(prefix.length), prefix);
    try {
      return {
        ms: n(2, ""), angle: n(3, "A"), gyro: n(4, "G"), el: n(5, "EL"), er: n(6, "ER"),
        ml: n(7, "ML"), mr: n(8, "MR"), state: n(9, "S"),
        trainingLocked: x[10]?.slice(1) === "1", balanceStarted: x[11]?.slice(1) === "1",
        c1: x[12] ? n(12, "C1") : 0, c2: x[13] ? n(13, "C2") : 0,
        c3: x[14] ? n(14, "C3") : 0, c4: x[15] ? n(15, "C4") : 0
      };
    } catch (_) { return null; }
  }

  function parseSlow(raw) {
    const x = bodyParts(raw);
    if (!x || x.length < 17 || x[0] !== "T1" || x[1] !== "S") return null;
    const n = (i, prefix) => finiteNumber(x[i].slice(prefix.length), prefix);
    try {
      return {
        ms: n(2, ""), battery: n(3, "V"), balancePwm: n(4, "B"), velocityPwm: n(5, "V"), turnPwm: n(6, "T"),
        pid: normalizePid({ AP: n(7, "AP"), AD: n(8, "AD"), VP: n(9, "VP"), VI: n(10, "VI"), TP: n(11, "TP"), TD: n(12, "TD") }),
        dropped: n(13, "D"), trainingLocked: x[14].slice(1) === "1", balanceStarted: x[15].slice(1) === "1", firmware: x[16]
      };
    } catch (_) { return null; }
  }

  function metrics(samples) {
    if (!Array.isArray(samples) || samples.length === 0) throw new Error("没有可用遥测样本");
    const sumAngle = samples.reduce((s, x) => s + x.angle * x.angle, 0);
    const sumGyro = samples.reduce((s, x) => s + x.gyro * x.gyro, 0);
    return {
      rmsAngle: Math.sqrt(sumAngle / samples.length),
      peakAngle: Math.max(...samples.map((x) => Math.abs(x.angle))),
      rmsGyro: Math.sqrt(sumGyro / samples.length),
      saturationRate: samples.filter((x) => Math.abs(x.ml) >= 2595 || Math.abs(x.mr) >= 2595).length / samples.length,
      samples: samples.length
    };
  }

  function harness(base, m) {
    const p = normalizePid(base);
    if (m.peakAngle > 8) p.AP *= 1.04;
    if (m.rmsGyro > 20) p.AD *= 1.04;
    return { schema_version: 1, decision: "propose", stage: "balance", candidate: p, expected_effect: "Simulation Harness balance-stage rule.", requested_test: "balance_recovery", confidence: 0.7 };
  }

  function validateProposal(raw, base, maxPct = 5) {
    const p = typeof raw === "string" ? JSON.parse(raw) : raw;
    const exact = ["schema_version", "decision", "stage", "candidate", "expected_effect", "requested_test", "confidence"];
    if (Object.keys(p).sort().join() !== exact.sort().join()) throw new Error("LLM JSON 字段不符合固定 schema");
    if (p.schema_version !== 1 || !["propose", "hold", "rollback"].includes(p.decision) || p.stage !== "balance") throw new Error("LLM 决策或阶段无效");
    if (!["balance_recovery", "quiet_balance"].includes(p.requested_test)) throw new Error("requested_test 无效");
    if (typeof p.expected_effect !== "string" || !p.expected_effect.trim() || p.expected_effect.length > 500) throw new Error("expected_effect 无效");
    p.confidence = finiteNumber(p.confidence, "confidence");
    if (p.confidence < 0 || p.confidence > 1) throw new Error("confidence 超出 0..1");
    const b = normalizePid(base), c = normalizePid(p.candidate);
    if (Object.keys(p.candidate).sort().join() !== PID_KEYS.slice().sort().join()) throw new Error("candidate 必须恰好包含六个 PID");
    for (const key of ["VP", "VI", "TP", "TD"]) if (Math.abs(c[key] - b[key]) >= 0.005) throw new Error(`${key} 在 balance 阶段不得变化`);
    if (p.decision === "propose") for (const key of ["AP", "AD"]) {
      const pct = Math.abs(c[key] - b[key]) * 100 / Math.abs(b[key]);
      if (!Number.isFinite(pct) || pct > Math.min(5, maxPct) + 1e-9) throw new Error(`${key} 变化 ${pct.toFixed(2)}% 超过限制`);
    }
    p.candidate = c;
    return p;
  }

  return { DEFAULT_PID, PID_KEYS, crc16, frame, verify, StreamParser, normalizePid, pidCsv, command, getPid, training, manualSet, prepare, motorDiag, legacyMotion, parsePidStatus, parseFast, parseSlow, metrics, harness, validateProposal };
});
