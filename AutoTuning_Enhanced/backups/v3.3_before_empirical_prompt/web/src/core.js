(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.RouteACore = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const DEFAULT_PID = Object.freeze({ AP: 96, AD: 48, VP: 62, VI: 31, TP: 17, TD: 20 });
  const PID_KEYS = Object.freeze(["AP", "AD", "VP", "VI", "TP", "TD"]);
  const RANGES = Object.freeze({ AP: [20,288], AD: [5,200], VP: [10,100], VI: [1,100], TP: [1,100], TD: [1,100] });

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
    if (typeof body !== "string" || /[$#\r\n]/.test(body)) throw new Error("报文正文无效");
    return `$${body},C${crc16(body).toString(16).toUpperCase().padStart(4, "0")}#`;
  }

  function verify(raw) {
    if (typeof raw !== "string" || raw[0] !== "$" || !raw.endsWith("#")) return false;
    const p = raw.lastIndexOf(",C");
    if (p < 0 || raw.length - p !== 7) return false;
    if (!/^[0-9a-f]{4}$/i.test(raw.slice(p + 2, p + 6))) return false;
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
    if (value === null || value === undefined || typeof value === "boolean" || (typeof value === "string" && !value.trim())) throw new Error(`${name} 不能为空`);
    const n = Number(value);
    if (!Number.isFinite(n)) throw new Error(`${name} 必须是有限数字`);
    return n;
  }

  function normalizePid(input) {
    if (!input || typeof input !== "object") throw new Error("PID 必须是对象");
    const p = {};
    for (const key of PID_KEYS) p[key] = finiteNumber(input[key], key);
    return p;
  }

  function pidCsv(pid) {
    const p = normalizePid(pid);
    return PID_KEYS.map((key) => p[key].toFixed(2)).join(",");
  }

  function boundedPid(input) {
    const p = normalizePid(input);
    for (const k of PID_KEYS) {
      p[k] = Math.round(p[k] * 100) / 100;
      if (p[k] < RANGES[k][0] || p[k] > RANGES[k][1]) throw new Error(`${k} 超出保护范围 ${RANGES[k].join("..")}`);
    }
    return p;
  }
  function samePid(a,b) { return !!a && !!b && PID_KEYS.every(k => Math.abs(a[k]-b[k]) < .005); }
  function parseAck(raw) {
    const x = bodyParts(raw);
    if (!x || x[0] !== "P1") return null;
    if (x[1] === "ERR") return { type: "ERR", code: x.slice(2).join(",") };
    if (x[1] !== "ACK" || x.length < 4 || !/^\d+$/.test(x[3])) return null;
    return { type: "ACK", name: x[2], id: Number(x[3]), fields: x.slice(4) };
  }
  function endpoint(base) {
    const url = new URL(base, typeof location !== "undefined" ? location.href : "http://localhost");
    if (url.protocol !== "https:" && !(url.protocol === "http:" && ["localhost","127.0.0.1","[::1]"].includes(url.hostname))) throw new Error("LLM 地址必须使用 HTTPS（本机 localhost 除外）");
    if (url.username || url.password || url.search || url.hash) throw new Error("接口地址不能包含账号、查询参数或片段");
    const path = url.pathname.replace(/\/+$/, "");
    url.pathname = path.endsWith("/chat/completions") ? path : path + "/chat/completions";
    return url.href;
  }

  function command(name, id) { return frame(`P1,${name},${Math.trunc(id)}`); }
  function getPid() { return frame("P1,GET"); }
  function training(action) { return frame(`P1,TRAIN,${action}`); }
  function manualSet(pid) { return frame(`P1,MSET,${pidCsv(pid)}`); }
  function prepare(id, ttl, pid) { return frame(`P1,PREP,${Math.trunc(id)},${Math.trunc(ttl)},${pidCsv(pid)}`); }
  function motorDiag(action) { return frame(`P1,DIAG,${action}`); }

  function legacyMotion(move, pivot) {
    if (!Number.isInteger(move) || move < 0 || move > 4 || !Number.isInteger(pivot) || pivot < 0 || pivot > 2) throw new Error("移动字段无效");
    // Official 21-byte movement-only frame: PID-operation and all update flags remain zero.
    return `$${move},${pivot},0,0,0,0,0,0,0,0#`;
  }

  function bodyParts(raw) {
    if (!verify(raw)) return null;
    return raw.slice(1, raw.lastIndexOf(",C")).split(",");
  }

  function parsePidStatus(raw) {
    const x = bodyParts(raw);
    if (!x || x.length !== 11 || x[0] !== "P1" || x[1] !== "PID" || !/^[012]$/.test(x[8]) || !/^[01]$/.test(x[9]) || !/^[01]$/.test(x[10])) return null;
    try {
      return {
        pid: normalizePid({ AP: x[2], AD: x[3], VP: x[4], VI: x[5], TP: x[6], TD: x[7] }),
        state: Number.parseInt(x[8], 10), trainingLocked: x[9] === "1", balanceStarted: x[10] === "1"
      };
    } catch (_) { return null; }
  }

  function parseFast(raw) {
    const x = bodyParts(raw);
    if (!x || x.length !== 16 || x[0] !== "T1" || x[1] !== "F" || !/^S[012]$/.test(x[9]) || !/^L[01]$/.test(x[10]) || !/^R[01]$/.test(x[11])) return null;
    const n = (i, prefix) => { if (!x[i].startsWith(prefix)) throw new Error("字段前缀错误"); return finiteNumber(x[i].slice(prefix.length), prefix); };
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
    if (!x || x.length !== 17 || x[0] !== "T1" || x[1] !== "S" || !/^L[01]$/.test(x[14]) || !/^R[01]$/.test(x[15])) return null;
    const n = (i, prefix) => { if (!x[i].startsWith(prefix)) throw new Error("字段前缀错误"); return finiteNumber(x[i].slice(prefix.length), prefix); };
    try {
      return {
        ms: n(2, ""), battery: n(3, "V"), balancePwm: n(4, "B"), velocityPwm: n(5, "V"), turnPwm: n(6, "T"),
        pid: normalizePid({ AP: n(7, "AP"), AD: n(8, "AD"), VP: n(9, "VP"), VI: n(10, "VI"), TP: n(11, "TP"), TD: n(12, "TD") }),
        dropped: n(13, "D"), trainingLocked: x[14].slice(1) === "1", balanceStarted: x[15].slice(1) === "1", firmware: x[16]
      };
    } catch (_) { return null; }
  }

  function parseFeatures(raw) {
    const x=bodyParts(raw);
    if(!x||x.length!==7||x[0]!=="T1"||x[1]!=="E"||!/^W[01]$/.test(x[6]))return null;
    try { const read=(i,p)=>{if(!x[i].startsWith(p))throw Error("prefix");return finiteNumber(x[i].slice(p.length),p);};
      const f={ms:read(2,""),osc:read(3,"O"),lean:read(4,"L"),leanBand:read(5,"B"),warm:x[6]==="W1"};
      if(f.osc<0||f.lean<0)return null;return f;
    }catch(e){return null;}
  }
  function metrics(samples) {
    if (!Array.isArray(samples) || samples.length === 0) throw new Error("没有可用遥测样本");
    const sumAngle = samples.reduce((s, x) => s + x.angle * x.angle, 0);
    const sumGyro = samples.reduce((s, x) => s + x.gyro * x.gyro, 0);
    const stamps = samples.map(x=>x.at).filter(Number.isFinite);
    const gaps = stamps.slice(1).map((t,i)=>t-stamps[i]);
    return {
      featureSamples: samples.filter(x=>x.featuresFresh&&x.warm).length,
      meanOsc: samples.reduce((v,x)=>v+(x.featuresFresh&&x.warm?x.osc:0),0)/Math.max(1,samples.filter(x=>x.featuresFresh&&x.warm).length),
      peakOsc: Math.max(0,...samples.filter(x=>x.featuresFresh&&x.warm).map(x=>x.osc)),
      meanLean: samples.reduce((v,x)=>v+(x.featuresFresh&&x.warm?x.lean:0),0)/Math.max(1,samples.filter(x=>x.featuresFresh&&x.warm).length),
      peakLean: Math.max(0,...samples.filter(x=>x.featuresFresh&&x.warm).map(x=>x.lean)),
      peakLeanBand: Math.max(0,...samples.filter(x=>x.featuresFresh&&x.warm).map(x=>Math.abs(x.leanBand))),
      rmsAngle: Math.sqrt(sumAngle / samples.length),
      peakAngle: Math.max(...samples.map((x) => Math.abs(x.angle))),
      rmsGyro: Math.sqrt(sumGyro / samples.length),
      saturationRate: samples.filter((x) => Math.abs(x.ml) >= 2595 || Math.abs(x.mr) >= 2595).length / samples.length,
      samples: samples.length,
      durationMs: stamps.length > 1 ? stamps[stamps.length-1]-stamps[0] : 0,
      maxGapMs: gaps.length ? Math.max(...gaps) : 0
    };
  }

  const ACCEPTANCE = Object.freeze({maxWeightedCost:1.10, weights:{rmsAngle:.30,rmsGyro:.15,meanOsc:.25,meanLean:.25,saturationRate:.05},floors:{rmsAngle:.25,rmsGyro:5,meanOsc:1,meanLean:.15,saturationRate:.05}});
  function evaluateCost(base,trial){
    const terms={};let cost=0;
    for(const key of Object.keys(ACCEPTANCE.weights)){
      const a=base?.[key],b=trial?.[key],floor=ACCEPTANCE.floors[key];
      if(!Number.isFinite(a)||!Number.isFinite(b)||a<0||b<0)throw Error('评价指标无效：'+key);
      terms[key]=(b+floor)/(a+floor);cost+=ACCEPTANCE.weights[key]*terms[key];
    }
    return {approved:cost<=ACCEPTANCE.maxWeightedCost,cost,limit:ACCEPTANCE.maxWeightedCost,terms,reason:`综合代价 ${cost.toFixed(3)} / 接受阈值 ${ACCEPTANCE.maxWeightedCost.toFixed(2)}；${cost<=1?'窗口指标改善或持平':cost<=ACCEPTANCE.maxWeightedCost?'接受探索容差内的取舍，不代表已改善':'超过探索容差，恢复上一组参数'}`};
  }
  function harness(base, m) {
    const p = normalizePid(base);
    if (m.peakAngle > 8) p.AP *= 1.04;
    if (m.rmsGyro > 20) p.AD *= 1.04;
    return { schema_version: 1, decision: samePid(p,base) ? "hold" : "propose", stage: "balance", candidate: p, expected_effect: "本地确定性规则建议；仅作流程测试，不代表参数已优化。", requested_test: "quiet_balance", confidence: 0.7 };
  }

  function validateProposal(raw, base) {
    const p = typeof raw === "string" ? JSON.parse(raw) : raw;
    if (!p || typeof p !== "object" || Array.isArray(p)) throw new Error("LLM JSON 必须是对象");
    p.candidate = p.candidate && { ...p.candidate };
    const exact = ["schema_version", "decision", "stage", "candidate", "expected_effect", "requested_test", "confidence"];
    if (Object.keys(p).sort().join() !== exact.sort().join()) throw new Error("LLM JSON 字段不符合固定 schema");
    if (p.schema_version !== 1 || !["propose", "hold", "rollback"].includes(p.decision) || p.stage !== "balance") throw new Error("LLM 决策或阶段无效");
    if (!["balance_recovery", "quiet_balance"].includes(p.requested_test)) throw new Error("requested_test 无效");
    if (typeof p.expected_effect !== "string" || !p.expected_effect.trim() || p.expected_effect.length > 500) throw new Error("expected_effect 无效");
    if (typeof p.confidence !== "number") throw new Error("confidence 必须是数字");
    if (p.confidence < 0 || p.confidence > 1) throw new Error("confidence 超出 0..1");
    const b = boundedPid(base);
    if (!p.candidate || PID_KEYS.some(k=>typeof p.candidate[k] !== "number")) throw new Error("candidate 六个 PID 必须是数字");
    const c = boundedPid(p.candidate);
    if (Object.keys(p.candidate).sort().join() !== PID_KEYS.slice().sort().join()) throw new Error("candidate 必须恰好包含六个 PID");
    
    if (p.decision !== "propose" && !samePid(c,b)) throw new Error("hold / rollback 必须返回当前 PID");
    p.candidate = c;
    return p;
  }

  return { DEFAULT_PID, PID_KEYS, RANGES, crc16, frame, verify, StreamParser, normalizePid, boundedPid, samePid, parseAck, endpoint, pidCsv, command, getPid, training, manualSet, prepare, motorDiag, legacyMotion, parsePidStatus, parseFast, parseSlow, parseFeatures, metrics, ACCEPTANCE, evaluateCost, harness, validateProposal };
});

