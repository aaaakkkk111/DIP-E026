"use strict";

const assert = require("node:assert/strict");
const {
  COMMANDS,
  FrameAssembler,
  parseRobotFrame,
  buildPidUpdate,
  isOfficialCommand,
} = require("../protocol.js");

const assembler = new FrameAssembler();
assert.deepEqual(assembler.push("noise$O"), []);
assert.deepEqual(assembler.push("K#$LV1.25,RV-2.50,AC0.10,GY3.20,CSB18.00,VT11.70#$OK#"), [
  "$OK#",
  "$LV1.25,RV-2.50,AC0.10,GY3.20,CSB18.00,VT11.70#",
  "$OK#",
]);
assert.equal(assembler.droppedCharacters, 5);

const resync = new FrameAssembler();
assert.deepEqual(resync.push("$broken$OK#"), ["$OK#"]);
assert.equal(resync.droppedCharacters, 7);

const telemetry = parseRobotFrame("$LV1.25,RV-2.50,AC0.10,GY3.20,CSB18.00,VT11.70#");
assert.equal(telemetry.type, "telemetry");
assert.deepEqual(telemetry.values, { LV: 1.25, RV: -2.5, AC: 0.1, GY: 3.2, CSB: 18, VT: 11.7 });

const pid = parseRobotFrame("$AP23.54,AD85.45,VP10.78,VI0.26#");
assert.equal(pid.type, "pid");
assert.equal(pid.values.AP, 23.54);
assert.equal(pid.values.VI, 0.26);

assert.equal(parseRobotFrame("$OK#").type, "ack");
assert.equal(parseRobotFrame("$ReceivePackError#").type, "error");
assert.equal(isOfficialCommand(COMMANDS.AUTO_REPORT_ON), true);
assert.equal(isOfficialCommand("0,0#"), false);
assert.equal(buildPidUpdate({ AP: 23.54, AD: 8.5, VP: 10.78, VI: 0.26 }), "$0,0,0,0,1,1,AP23.54,AD8.50,VP10.78,VI0.26#");
assert.throws(() => buildPidUpdate({ AP: -1, AD: 1, VP: 1, VI: 1 }), RangeError);

console.log("protocol tests passed");
