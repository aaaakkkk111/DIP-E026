const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const C = require("../src/core.js");

const frame = C.frame("P3,GET");
assert.equal(C.verify(frame), true);
assert.equal(C.verify(frame.replace(/C....#$/, "C0000#")), false);
const parser = new C.StreamParser();
assert.deepEqual(parser.feed("junk$OK#$OK#tail"), ["$OK#", "$OK#"]);
assert.deepEqual(parser.feed("$T3,F,"), []);
assert.deepEqual(parser.feed("123#"), ["$T3,F,123#"]);

assert.deepEqual(C.presetPid("N"), { AP:96, AD:48, VP:62, VI:31, TP:17, TD:20 });
assert.deepEqual(C.presetPid("L"), { AP:96, AD:75, VP:70, VI:35, TP:14, TD:20 });
assert.equal(C.PRESETS.L.balanceScale, 2);
assert.equal(C.PRESETS.L.velocityScale, 1.35);

const prep = C.prepare(999999, 30000, "L", { AP:288, AD:200, VP:72, VI:100, TP:100, TD:100 });
assert.match(prep, /^\$P3,PREP,999999,30000,L,/);
assert.ok(prep.length < 80, "PREP frame must fit firmware RX buffer, got " + prep.length);

const status = C.parseStatus(C.frame("P3,STATUS,96.00,48.00,62.00,31.00,17.00,20.00,PN,S0,L1,B1"));
assert.equal(status.profile, "N");
assert.equal(status.trainingLocked, true);
assert.equal(status.pid.AP, 96);

const fast = C.parseFast(C.frame("T3,F,200,A1.25,G-3.0,O0.520,N-0.170,EL4,ER5,ML6,MR7,S0,L1,PN"));
assert.equal(fast.angle, 1.25);
assert.equal(fast.lean, -0.17);
assert.equal(fast.profile, "N");

const slow = C.parseSlow(C.frame("T3,S,1000,V11.82,A20.640,G29.000,PK1.40,X0.020,AP96.00,AD48.00,VP62.00,VI31.00,TP17.00,TD20.00,PN,D0,B1,RA3.0"));
assert.equal(slow.pid.AD, 48);
assert.equal(slow.angleSqMean, 0.64);
assert.equal(slow.firmware, "RA3.0");

const metrics = C.metrics([
  fast,
  { ...fast, ms:400, angle:-1, gyro:3, osc:0.4, lean:-0.2, ml:100, mr:110 },
  { ...fast, ms:600, angle:0.5, gyro:1, osc:0.3, lean:-0.1, ml:50, mr:60 }
], [slow]);
assert.equal(metrics.samples, 3);
assert.ok(metrics.score > 0);
assert.ok(Math.abs(metrics.rmsAngle - 0.8) < 1e-9);

const largeChange = {
  schema_version:3, action:"propose_pid", target_profile:"N",
  candidate:{ AP:200, AD:120, VP:70, VI:50, TP:30, TD:40 },
  reason:"test unrestricted relative change", expected_effect:"test", confidence:0.7
};
assert.equal(C.validateDecision(largeChange, "N", C.presetPid("N")).candidate.AP, 200);
assert.throws(() => C.validateDecision({ ...largeChange, candidate:{ ...largeChange.candidate, AP:500 } }, "N", C.presetPid("N")));
const switched = C.validateDecision({ ...largeChange, action:"switch_preset", target_profile:"L", candidate:C.presetPid("N") }, "N", C.presetPid("N"));
assert.deepEqual(switched.candidate, C.presetPid("L"));

const app = fs.readFileSync(path.join(__dirname, "../src/app.js"), "utf8");
const template = fs.readFileSync(path.join(__dirname, "../template.html"), "utf8");
for (const match of app.matchAll(/\$\("([A-Za-z0-9_-]+)"\)/g)) {
  const id = match[1];
  assert.match(template, new RegExp("id=[\"']" + id + "[\"']"), "missing DOM id " + id);
}
assert.doesNotMatch(app + fs.readFileSync(path.join(__dirname, "../src/core.js"), "utf8"), /P2,|T2,|LEVELS|maxPct/);
assert.match(app, /switch_preset/);
assert.match(template, /开始自动LLM调参/);
console.log("Route A Remote Adaptive core tests: PASS");


assert.equal(C.parseFast(C.frame("T3,F,200,A1.25,G-3.0,O0.520,N-0.170,EL4,ER5,ML6,MR7,S1,L1,PN,U50")).transitionStep, 50);
assert.equal(fast.transitionStep, 0, "previous P3 firmware remains parseable");
assert.equal(C.pidMatches(C.presetPid("N"), C.presetPid("N")), true);
assert.equal(C.pidMatches({ ...C.presetPid("N"), AD:54 }, C.presetPid("N")), false);

const diagnostic = C.parseSlow(C.frame("T3,S,1000,V11.82,A20.640,G29.000,PK1.40,X0.020,AP96.00,AD48.00,VP62.00,VI31.00,TP17.00,TD20.00,PN,D0,B1,RA3.2-STACKFIX,SF3000,DT5100,CT650"));
assert.equal(diagnostic.stackFree, 3000);
assert.equal(diagnostic.controlGapUs, 5100);
assert.equal(diagnostic.controlExecUs, 650);
assert.equal(slow.stackFree, null);

const damaged={AP:-354141999257534600000,AD:1.27,VP:3.6,VI:91860664320,TP:0,TD:0};
const stop=C.legacyMotion(0,0,damaged);
assert.ok(stop.length<80);
assert.ok(stop.includes("AP96.00,AD48.00"));

// Direct-update extension: partial groups, distinct profile operations and RX bound.
assert.match(C.manualSet("N", C.presetPid("N"), 1), /,M1,C[0-9A-F]{4}#$/);
assert.match(C.prepare(999999, 30000, "N", C.presetPid("N"), 4), /,M4,C/);
assert.match(C.manualPreset("L"), /^\$P3,PRESET,L,C/);
assert.match(C.preparePreset(9, 30000, "L"), /^\$P3,PPRESET,9,30000,L,C/);
assert.deepEqual(C.mergePid(C.presetPid("N"), { AP:98, AD:52, VP:"", VI:0, TP:NaN, TD:Infinity }, 1),
  { AP:98, AD:52, VP:62, VI:31, TP:17, TD:20 });
assert.equal(C.changedMask(C.presetPid("N"), { ...C.presetPid("N"), AD:54 }), 1);
assert.equal(C.changedMask(C.presetPid("N"), { ...C.presetPid("N"), VI:35, TD:22 }), 6);
assert.equal(C.changedMask(C.presetPid("N"), C.presetPid("N")), 0);
for (const bad of [0,8,1.5,NaN]) assert.throws(()=>C.manualSet("N", C.presetPid("N"), bad));
for (const bad of ["", " ", null, undefined, NaN, Infinity]) assert.throws(()=>C.validatePid({...C.presetPid("N"),AP:bad}));
assert.throws(()=>C.validateDecision({...largeChange,target_profile:"L"},"N",C.presetPid("N")), /switch_preset/);
for (const mask of [1,2,3,4,5,6,7]) {
  const wire=C.prepare(999999,30000,"N",{AP:288,AD:200,VP:72,VI:100,TP:100,TD:100},mask);
  assert.ok(new TextEncoder().encode(wire).length<80);assert.ok(C.verify(wire));
}
console.log("RA3.3 direct protocol tests: PASS");
