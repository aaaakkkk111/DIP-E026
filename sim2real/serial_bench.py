"""Non-actuating PC/STM32 transport acceptance test. Python 3.10+.

Only PING and INFO are sent. Requires the companion UART_BENCH firmware.
Self-test uses the standard library; real serial I/O requires pyserial.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import struct
import time

MAGIC = b"\xaa\x55"
HEADER = struct.Struct("<2sBBHHI")
MAX_PAYLOAD = 64
PING, INFO, PONG, INFO_REPLY = 1, 2, 0x81, 0x82


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value


def encode(kind: int, seq: int, payload: bytes = b"", tick_us: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 64 bytes")
    frame = HEADER.pack(MAGIC, 1, kind, seq & 0xFFFF, len(payload), tick_us & 0xFFFFFFFF) + payload
    return frame + struct.pack("<H", crc16(frame[2:]))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.header_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self.buffer[-1] == MAGIC[0] else 0
                self.discarded_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-1:] if keep else b""
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, seq, size, tick = HEADER.unpack_from(self.buffer)
            if version != 1 or size > MAX_PAYLOAD:
                self.header_errors += 1
                del self.buffer[0]
                continue
            end = HEADER.size + size + 2
            if len(self.buffer) < end:
                break
            frame = bytes(self.buffer[:end])
            if crc16(frame[2:-2]) != struct.unpack_from("<H", frame, end - 2)[0]:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            frames.append((kind, seq, tick, frame[HEADER.size:-2]))
            del self.buffer[:end]
        return frames


def self_test():
    assert crc16(b"123456789") == 0x29B1
    expected = [(PING, 65535, 0x12345678, bytes(range(64))), (INFO, 0, 0, b"")]
    stream = encode(PING, 65535, bytes(range(64)), 0x12345678) + encode(INFO, 0)
    for step in range(1, len(stream) + 1):
        parser, found = Parser(), []
        for start in range(0, len(stream), step):
            found.extend(parser.feed(stream[start:start + step]))
        assert found == expected
    bad = bytearray(encode(PING, 17, b"corrupt me"))
    bad[-1] ^= 0x80
    parser = Parser()
    recovered = parser.feed(b"noise\xaa" + bad + stream)
    assert recovered == expected and parser.crc_errors == 1
    parser = Parser()
    assert parser.feed(HEADER.pack(MAGIC, 1, 1, 0, 0xFFFF, 0) + stream) == expected
    parser = Parser()
    assert parser.feed(b"x" * 10000) == [] and len(parser.buffer) == 0
    assert parser.feed(encode(PING, 1, b"\xaa\x55" * 20))[0][3] == b"\xaa\x55" * 20
    print("PASS: CRC known vector, all chunk sizes, concatenation, corruption recovery, oversized length, noise, embedded magic.")


def get_info(port, parser: Parser, seq: int):
    port.write(encode(INFO, seq))
    deadline = time.perf_counter() + 1.0
    while time.perf_counter() < deadline:
        for kind, reply_seq, tick, payload in parser.feed(port.read(max(1, port.in_waiting))):
            if kind == INFO_REPLY and reply_seq == seq and len(payload) == 20:
                if payload[:4] != b"S2RB":
                    raise RuntimeError("Wrong firmware signature")
                version, overflow, uart_errors, crc_errors = struct.unpack("<IIII", payload[4:])
                return {"signature": "S2RB", "firmware_version": version, "rx_overflow": overflow,
                        "uart_errors": uart_errors, "crc_errors": crc_errors, "tick_us": tick}
    raise TimeoutError("No matching S2RB INFO reply: check firmware, COM port, power and boot state")


def run(args):
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as error:
        raise SystemExit("Install pyserial in your Python environment: python -m pip install pyserial==3.5") from error
    if args.list:
        ports = list(list_ports.comports())
        for port in ports:
            print(port.device, port.description, port.hwid)
        if not ports:
            print("No serial ports enumerated. Connect the Type-C data cable and check the CH340 driver.")
        return 0
    if not args.port:
        raise SystemExit("Specify --port COMx (use --list to enumerate).")
    if not 1 <= args.rate <= 100 or not 1 <= args.duration <= 86400:
        raise SystemExit("Require rate 1..100 Hz and duration 1..86400 seconds.")
    parser = Parser()
    rtts, pending = [], {}
    sent = lost = unexpected = 0
    port = serial.Serial(port=None, baudrate=args.baud, timeout=0.005, write_timeout=0.2,
                         bytesize=8, parity="N", stopbits=1, xonxoff=False, rtscts=False, dsrdtr=False)
    # The board connects these lines to its automatic boot/reset circuit.
    # Setting them before open reduces intentional toggling; driver glitches remain possible.
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        time.sleep(1.5)
        port.reset_input_buffer()
        initial = get_info(port, parser, 65534)
        print("Connected:", json.dumps(initial))
        start = time.perf_counter()
        next_send = start
        deadline = start + args.duration
        while time.perf_counter() < deadline or pending:
            now = time.perf_counter()
            if now < deadline and now >= next_send:
                payload = struct.pack("<Q", sent) + os.urandom(40)
                seq = sent & 0xFFFF
                wire = encode(PING, seq, payload)
                stamp = time.perf_counter()
                count = port.write(wire)
                if count != len(wire):
                    raise IOError("Partial serial write")
                pending[(seq, payload)] = stamp
                sent += 1
                next_send = now + 1.0 / args.rate
            for kind, seq, _, payload in parser.feed(port.read(max(1, port.in_waiting))):
                stamp = pending.pop((seq, payload), None) if kind == PONG else None
                if stamp is None:
                    unexpected += 1
                else:
                    rtts.append((time.perf_counter() - stamp) * 1000.0)
            now = time.perf_counter()
            for key, stamp in list(pending.items()):
                if now - stamp > 1.0:
                    lost += 1
                    del pending[key]
        final = get_info(port, parser, 65533)
    finally:
        port.close()
    ordered = sorted(rtts)
    p99 = ordered[max(0, math.ceil(0.99 * len(ordered)) - 1)] if ordered else None
    errors = lost + unexpected + parser.crc_errors + parser.header_errors + parser.discarded_bytes
    errors += final["rx_overflow"] + final["uart_errors"] + final["crc_errors"]
    passed = errors == 0 and sent == len(rtts) and p99 is not None and p99 <= args.max_p99_ms
    result = {"scope": "UART PING/INFO transport only; no motors, telemetry or control-loop verification",
              "port": args.port, "baud": args.baud, "requested_duration_s": args.duration,
              "requested_rate_hz": args.rate, "actual_rate_hz": sent / args.duration,
              "sent": sent, "matched": len(rtts), "lost": lost, "unexpected": unexpected,
              "host_crc_errors": parser.crc_errors, "host_header_errors": parser.header_errors,
              "host_discarded_bytes": parser.discarded_bytes,
              "rtt_p99_ms": p99, "rtt_max_ms": max(rtts) if rtts else None,
              "board_initial": initial, "board_final": final, "pass": passed}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--rate", type=float, default=50)
    parser.add_argument("--max-p99-ms", type=float, default=50)
    parser.add_argument("--output", default=str(Path(__file__).with_name("link_result.json")))
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
