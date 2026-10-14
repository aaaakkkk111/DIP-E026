"""Passive, READ-ONLY real-robot behaviour logger for the mistral/tzejun project.

Purpose: while you run a flashed model on the real Yahboom robot, this saves a
timestamped file of what the robot actually did, so the measured behaviour can be
fed back to improve the model (the finetune loop can read the summary's gap metrics).

SAFETY: this tool NEVER writes a single byte to the serial port. It only reads the
telemetry the firmware already streams. It therefore cannot select a controller,
arm the robot, or command any PWM. Arming stays physical-only (KEY1), exactly as the
firmware enforces. Stopping this logger does NOT stop the robot - use KEY1.

Protocol (matches the firmware, yb_protocol.h / ollama_supervisor.py):
  frame = AA 55 | version=1 | kind | seq:u16 | len:u16 | payload | crc16:u16
  CRC16/CCITT-FALSE over version..payload. Telemetry kind = 0x81,
  payload = '<BBHI15fII' = mode, stopped, fault, tick, [15 floats], infcyc, maxloop.

Outputs (into robot_logs/<timestamp>/):
  telemetry.csv  - one row per streamed sample (wall time + decoded fields)
  summary.json   - duration, rates, fault codes, behaviour stats, and a
                   'feedback_for_model' block the training loop can consume.

Usage:
  python -E robot_logger.py --list-ports
  python -E robot_logger.py --port COM5 --seconds 120 --label ollamaD_trial1
  python -E robot_logger.py --self-test        # decode a synthetic frame, no serial
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent

GAIN_KEYS = ("balance_kp", "balance_kd", "velocity_kp", "velocity_ki", "turn_kp", "turn_kd")
TELEM_FIELDS = ("mode", "stopped", "fault", "tick", "pitch_deg", "roll_deg",
                "pitch_rate_rad_s", "yaw_rate_rad_s", "speed_m_s", "x_m",
                "pwm_left", "pwm_right", "battery_v", *GAIN_KEYS,
                "inference_cycles", "max_loop_cycles")
TELEM_STRUCT = struct.Struct("<BBHI15fII")
TELEMETRY = 0x81
MODES = {0: "stock", 1: "llm-pid", 2: "tzejun"}
FAULTS = {1: "non-finite sensor/policy", 2: "tilt >0.70rad (~40deg)", 4: "loop >5ms",
          8: "unsupported config / startup fault", 32: "cadence outside 3-7ms",
          128: "physical KEY1 stop"}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def fault_names(code: int) -> list[str]:
    return [name for bit, name in FAULTS.items() if int(code) & bit]


class FrameDecoder:
    """Accumulates bytes and yields (kind, seq, payload) for CRC-valid frames."""

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)
        out = []
        while True:
            start = self.buf.find(b"\xaa\x55")
            if start < 0:
                if len(self.buf) > 4096:
                    del self.buf[:-1]
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 8:
                break
            version, kind, seq, size = struct.unpack_from("<BBHH", self.buf, 2)
            if version != 1 or size > 512:
                del self.buf[:2]
                continue
            if len(self.buf) < 8 + size + 2:
                break
            got = struct.unpack_from("<H", self.buf, 8 + size)[0]
            if crc16(bytes(self.buf[2:8 + size])) == got:
                out.append((kind, seq, bytes(self.buf[8:8 + size])))
                del self.buf[:8 + size + 2]
            else:
                del self.buf[:2]
        return out


def decode_telemetry(payload: bytes) -> dict | None:
    if len(payload) != TELEM_STRUCT.size:
        return None
    row = dict(zip(TELEM_FIELDS, TELEM_STRUCT.unpack(payload)))
    row["mode_name"] = MODES.get(row["mode"], str(row["mode"]))
    row["fault_names"] = "|".join(fault_names(row["fault"]))
    return row


def summarize(rows: list[dict], duration_s: float, label: str) -> dict:
    out = {"label": label, "requested_seconds": duration_s, "n_samples": len(rows),
           "read_only": True, "armed_by": "physical KEY1 only; logger never writes serial",
           "scope": "Streamed telemetry (~20 Hz snapshots), not the full 200 Hz loop"}
    if not rows:
        out["status"] = "no telemetry received; check port/cable and that the board streams"
        return out
    pitch = [abs(r["pitch_deg"]) for r in rows]
    roll = [abs(r["roll_deg"]) for r in rows]
    speed = [r["speed_m_s"] for r in rows]
    yaw = [r["yaw_rate_rad_s"] for r in rows]
    pwm = [max(abs(r["pwm_left"]), abs(r["pwm_right"])) for r in rows]
    batt = [r["battery_v"] for r in rows if r["battery_v"] > 0]
    faults = sorted({int(r["fault"]) for r in rows if r["fault"]})
    armed = [r for r in rows if not r["stopped"]]
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else None
    out.update(
        max_abs_pitch_deg=max(pitch), max_abs_roll_deg=max(roll),
        max_abs_pwm_counts=max(pwm), fell=bool(any(int(r["fault"]) & 2 for r in rows)),
        fault_codes=faults, fault_meanings=sorted({n for c in faults for n in fault_names(c)}),
        battery_v_range=[min(batt), max(batt)] if batt else None,
        armed_fraction=len(armed) / len(rows), stopped_fraction=1 - len(armed) / len(rows))
    # Behaviour stats over the ARMED portion, geared for feeding back to training.
    a = armed or rows
    out["feedback_for_model"] = {
        "pitch_rms_deg": rms([r["pitch_deg"] for r in a]),
        "yaw_rate_rms_rad_s": rms([r["yaw_rate_rad_s"] for r in a]),
        "speed_rms_m_s": rms([r["speed_m_s"] for r in a]),
        "mean_speed_m_s": sum(r["speed_m_s"] for r in a) / len(a),
        "net_displacement_m": a[-1]["x_m"] - a[0]["x_m"],
        "dominant_failure": ("fell_tilt" if any(int(r["fault"]) & 2 for r in rows)
                             else "cadence/timing" if any(int(r["fault"]) & (4 | 32) for r in rows)
                             else "none_observed"),
        "note": ("Use these as the real-world gap vs sim. High yaw_rate_rms or net "
                 "displacement under zero command => raise yaw/position/speed emphasis "
                 "and/or train with more sensing-latency randomisation."),
        "armed_samples": len(armed),
        "armed_coverage_ok": len(armed) >= 20,
    }
    return out


def record(port: str, seconds: float, out_dir: Path, label: str) -> int:
    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial is required: install with  python -E -m pip install pyserial", flush=True)
        return 2
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"READ-ONLY logger on {port} for {seconds:.0f}s. This NEVER arms the robot; "
          f"use KEY1 to start/stop. Ctrl-C to end early (robot keeps running).", flush=True)
    decoder = FrameDecoder()
    rows: list[dict] = []
    csv_path = out_dir / "telemetry.csv"
    deadline = time.monotonic() + seconds
    with serial.Serial(port, 115200, timeout=0.1) as ser, \
            open(csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["wall_s", *TELEM_FIELDS, "mode_name", "fault_names"])
        writer.writeheader()
        t0 = time.monotonic()
        try:
            while time.monotonic() < deadline:
                chunk = ser.read(512)
                if not chunk:
                    continue
                for kind, _seq, payload in decoder.feed(chunk):
                    if kind != TELEMETRY:
                        continue
                    row = decode_telemetry(payload)
                    if row is None:
                        continue
                    row = {"wall_s": round(time.monotonic() - t0, 4), **row}
                    rows.append(row)
                    writer.writerow(row)
                    stream.flush()
                    if row["fault"]:
                        print(f"  FAULT {row['fault']} ({row['fault_names']}) at t={row['wall_s']}s", flush=True)
        except KeyboardInterrupt:
            print("Stopped logging early (robot NOT disarmed; press KEY1).", flush=True)
    summary = summarize(rows, seconds, label)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("n_samples", "fault_meanings", "feedback_for_model")
                      if k in summary}, indent=2), flush=True)
    print(f"Saved {csv_path}  and  {out_dir / 'summary.json'}", flush=True)
    return 0


def self_test() -> int:
    """Build a synthetic telemetry frame, decode it, and summarise - no serial needed."""
    payload = TELEM_STRUCT.pack(2, 0, 2, 1234, 3.5, 0.2, 0.1, 0.4, 0.03, 0.12,
                                800, -820, 12.3, 9600, 48, 6200, 31, 1700, 20, 3300, 4800)
    body = struct.pack("<BBHH", 1, TELEMETRY, 0, len(payload)) + payload
    frame = b"\xaa\x55" + body + struct.pack("<H", crc16(body))
    dec = FrameDecoder()
    got = dec.feed(b"\x00\x01" + frame + b"\xaa")  # leading junk + trailing partial
    assert len(got) == 1 and got[0][0] == TELEMETRY, "frame decode failed"
    row = decode_telemetry(got[0][2])
    assert row and abs(row["pitch_deg"] - 3.5) < 1e-6 and row["mode_name"] == "tzejun"
    row["wall_s"] = 0.0
    summ = summarize([row], 1.0, "selftest")
    print("SELF-TEST PASS:", json.dumps({"decoded_pitch_deg": row["pitch_deg"],
          "fault_names": row["fault_names"] or "none",
          "feedback_keys": list(summ["feedback_for_model"])}, indent=2))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--label", default="trial")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.list_ports:
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                print(p.device, "-", p.description)
        except ImportError:
            print("pyserial not installed: python -E -m pip install pyserial")
        return 0
    if not args.port:
        ap.error("--port COMx is required (or --self-test / --list-ports)")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.out or (HERE / "robot_logs" / f"{args.label}_{stamp}")
    return record(args.port, args.seconds, out_dir, args.label)


if __name__ == "__main__":
    raise SystemExit(main())
