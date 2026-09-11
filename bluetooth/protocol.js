(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) {
    module.exports = api;
  }
  root.BalanceBotProtocol = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  const SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
  const CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb";

  const COMMANDS = Object.freeze({
    STOP: "$0,0,0,0,0,0,0,0,0,0#",
    FORWARD: "$1,0,0,0,0,0,0,0,0,0#",
    BACKWARD: "$2,0,0,0,0,0,0,0,0,0#",
    LEFT: "$3,0,0,0,0,0,0,0,0,0,0#",
    RIGHT: "$4,0,0,0,0,0,0,0,0,0,0#",
    ROTATE_LEFT: "$0,1,0,0,0,0,0,0,0,0#",
    ROTATE_RIGHT: "$0,2,0,0,0,0,0,0,0,0#",
    PID_READ_AND_RESET: "$0,0,1,0,0,0,0,0,0,0#",
    AUTO_REPORT_ON: "$0,0,0,1,0,0,0,0,0,0#",
    AUTO_REPORT_OFF: "$0,0,0,2,0,0,0,0,0,0#",
  });

  function bytesToAscii(value) {
    if (typeof value === "string") return value;
    let output = "";
    const bytes = value instanceof Uint8Array ? value : new Uint8Array(value);
    for (const byte of bytes) output += String.fromCharCode(byte);
    return output;
  }

  class FrameAssembler {
    constructor(maxBufferLength = 4096) {
      this.buffer = "";
      this.maxBufferLength = maxBufferLength;
      this.droppedCharacters = 0;
    }

    reset() {
      this.buffer = "";
      this.droppedCharacters = 0;
    }

    push(chunk) {
      this.buffer += bytesToAscii(chunk);
      const frames = [];

      if (this.buffer.length > this.maxBufferLength) {
        const lastStart = this.buffer.lastIndexOf("$");
        const keepFrom = lastStart >= 0 ? lastStart : this.buffer.length;
        this.droppedCharacters += keepFrom;
        this.buffer = this.buffer.slice(keepFrom);
      }

      while (this.buffer.length) {
        const start = this.buffer.indexOf("$");
        if (start < 0) {
          this.droppedCharacters += this.buffer.length;
          this.buffer = "";
          break;
        }
        if (start > 0) {
          this.droppedCharacters += start;
          this.buffer = this.buffer.slice(start);
        }

        const end = this.buffer.indexOf("#", 1);
        const newerStart = this.buffer.indexOf("$", 1);
        if (newerStart >= 0 && (end < 0 || newerStart < end)) {
          this.droppedCharacters += newerStart;
          this.buffer = this.buffer.slice(newerStart);
          continue;
        }
        if (end < 0) break;
        frames.push(this.buffer.slice(0, end + 1));
        this.buffer = this.buffer.slice(end + 1);
      }

      return frames;
    }
  }

  function extractTaggedNumbers(frame) {
    const values = {};
    const expression = /([A-Z]{2,3})([-+]?(?:\d+(?:\.\d*)?|\.\d+))/g;
    let match;
    while ((match = expression.exec(frame)) !== null) {
      values[match[1]] = Number(match[2]);
    }
    return values;
  }

  function parseRobotFrame(frame) {
    if (typeof frame !== "string" || !frame.startsWith("$") || !frame.endsWith("#")) {
      return { type: "invalid", frame };
    }
    if (frame === "$OK#") return { type: "ack", ok: true, frame };
    if (frame === "$ReceivePackError#" || frame === "$GetPIDError#") {
      return { type: "error", message: frame.slice(1, -1), frame };
    }

    const values = extractTaggedNumbers(frame);
    if (frame.startsWith("$LV")) {
      return {
        type: "telemetry",
        frame,
        values: {
          LV: values.LV,
          RV: values.RV,
          AC: values.AC,
          GY: values.GY,
          CSB: values.CSB,
          VT: values.VT,
        },
      };
    }
    if (frame.startsWith("$AP")) {
      return {
        type: "pid",
        frame,
        values: {
          AP: values.AP,
          AD: values.AD,
          VP: values.VP,
          VI: values.VI,
          TP: values.TP,
          TD: values.TD,
        },
      };
    }
    return { type: "raw", frame, values };
  }

  function buildPidUpdate(values) {
    const limits = {
      AP: [0, 200],
      AD: [0, 100],
      VP: [0, 200],
      VI: [0, 100],
    };
    const normalized = {};
    for (const [key, [minimum, maximum]] of Object.entries(limits)) {
      const number = Number(values[key]);
      if (!Number.isFinite(number) || number < minimum || number > maximum) {
        throw new RangeError(`${key} 必须在 ${minimum} 到 ${maximum} 之间`);
      }
      normalized[key] = number.toFixed(2);
    }
    return `$0,0,0,0,1,1,AP${normalized.AP},AD${normalized.AD},VP${normalized.VP},VI${normalized.VI}#`;
  }

  function isOfficialCommand(value) {
    return typeof value === "string" && value.startsWith("$") && value.endsWith("#") && value.length <= 79;
  }

  return Object.freeze({
    SERVICE_UUID,
    CHARACTERISTIC_UUID,
    COMMANDS,
    FrameAssembler,
    parseRobotFrame,
    buildPidUpdate,
    isOfficialCommand,
  });
});
