"""Official-compatible ASCII protocol plus versioned deployable telemetry."""

from __future__ import annotations

import re
from dataclasses import dataclass
from math import isfinite
from typing import Iterable

from .firmware_profile import PID_KEYS, PIDValues


MAX_RX_BYTES = 80
BAUD = 9600
BYTES_PER_SECOND_8N1 = BAUD / 10.0
_NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
_PID_RE = re.compile(rf"^\$AP({_NUMBER}),AD({_NUMBER}),VP({_NUMBER}),VI({_NUMBER}),TP({_NUMBER}),TD({_NUMBER})#$")


class ProtocolError(ValueError):
    pass


class FrameStreamParser:
    def __init__(self, max_length: int = MAX_RX_BYTES) -> None:
        self.max_length = max_length
        self.buffer = bytearray()
        self.inside = False
        self.discarded_frames = 0
        self.discarded_bytes = 0

    def reset(self) -> None:
        self.buffer.clear(); self.inside = False

    def feed(self, chunk: bytes) -> list[bytes]:
        result: list[bytes] = []
        for byte in chunk:
            if byte == ord("$"):
                if self.inside and self.buffer: self.discarded_frames += 1
                self.buffer = bytearray((byte,)); self.inside = True; continue
            if not self.inside:
                self.discarded_bytes += 1; continue
            self.buffer.append(byte)
            if len(self.buffer) >= self.max_length:
                self.discarded_frames += 1; self.reset(); continue
            if byte == ord("#"):
                result.append(bytes(self.buffer)); self.reset()
        return result


def _format(value: float) -> str:
    if isinstance(value, bool) or not isfinite(float(value)):
        raise ProtocolError("PID values must be finite numbers")
    return f"{float(value):.2f}"


def ensure_downlink_frame(frame: str) -> bytes:
    try: encoded = frame.encode("ascii")
    except UnicodeEncodeError as exc: raise ProtocolError("frames must be ASCII") from exc
    if len(encoded) >= MAX_RX_BYTES: raise ProtocolError(f"downlink frame is {len(encoded)} bytes; firmware accepts at most 79")
    if len(encoded) < 21: raise ProtocolError("firmware rejects control frames shorter than 21 bytes")
    if not frame.startswith("$") or not frame.endswith("#"): raise ProtocolError("frame must use $...# framing")
    return encoded


def build_control_frame(movement: int = 0, pivot: int = 0, pid_operation: int = 0,
                        auto_report: int = 0, groups: Iterable[str] = (),
                        pid: PIDValues | None = None) -> bytes:
    if movement not in range(5) or pivot not in range(3): raise ProtocolError("movement must be 0..4 and pivot 0..2")
    if pid_operation not in range(3) or auto_report not in range(3): raise ProtocolError("PID/report operation must be 0..2")
    selected = set(groups)
    if selected - {"balance", "velocity", "turn"}: raise ProtocolError("unknown PID group")
    fields = [str(movement), str(pivot), str(pid_operation), str(auto_report),
              "1" if "balance" in selected else "0", "1" if "velocity" in selected else "0",
              "1" if "turn" in selected else "0"]
    if selected:
        if pid is None: raise ProtocolError("PID is required for an update")
        fields += [f"{key}{_format(getattr(pid, key))}" for key in PID_KEYS]
    else: fields += ["0", "0", "0"]
    return ensure_downlink_frame("$" + ",".join(fields) + "#")


def build_motion_frame(command: str) -> bytes:
    mapping = {"stop": (0, 0), "forward": (1, 0), "backward": (2, 0), "left": (3, 0),
               "right": (4, 0), "pivot_left": (0, 1), "pivot_right": (0, 2)}
    if command not in mapping: raise ProtocolError(f"unknown motion command: {command}")
    return build_control_frame(*mapping[command])


def build_pid_query_frame() -> bytes: return build_control_frame(pid_operation=1)
def build_restore_frame() -> bytes: return build_control_frame(pid_operation=2)
def build_pid_update_frame(pid: PIDValues, stage: str | None = None) -> bytes:
    return build_control_frame(groups=(stage,) if stage else ("balance", "velocity", "turn"), pid=pid)


def build_pid_report(pid: PIDValues) -> bytes:
    return ("$" + ",".join(f"{key}{_format(getattr(pid, key))}" for key in PID_KEYS) + "#").encode("ascii")


def parse_pid_report(frame: bytes | str) -> PIDValues | None:
    text = frame.decode("ascii") if isinstance(frame, bytes) else frame
    match = _PID_RE.fullmatch(text)
    return PIDValues(*map(float, match.groups())) if match else None


def xor_checksum(payload: str) -> int:
    value = 0
    for byte in payload.encode("ascii"): value ^= byte
    return value


def build_checked_frame(payload: str) -> bytes:
    frame = f"${payload}*{xor_checksum(payload):02X}#".encode("ascii")
    if len(frame) >= MAX_RX_BYTES: raise ProtocolError(f"telemetry frame too long: {len(frame)}")
    return frame


def verify_checked_frame(frame: bytes | str) -> str:
    text = frame.decode("ascii") if isinstance(frame, bytes) else frame
    match = re.fullmatch(r"\$(.+)\*([0-9A-Fa-f]{2})#", text)
    if not match: raise ProtocolError("invalid checked frame")
    payload, checksum = match.groups()
    if xor_checksum(payload) != int(checksum, 16): raise ProtocolError("checksum mismatch")
    return payload


@dataclass(frozen=True)
class ControlRequest:
    movement: int; pivot: int; pid_operation: int; auto_report: int
    groups: tuple[str, ...]; pid: PIDValues | None


def parse_control_request(frame: bytes) -> ControlRequest:
    if len(frame) >= MAX_RX_BYTES or len(frame) < 21: raise ProtocolError("ReceivePackError")
    try: text = frame.decode("ascii")
    except UnicodeDecodeError as exc: raise ProtocolError("ReceivePackError") from exc
    if not text.startswith("$") or not text.endswith("#"): raise ProtocolError("ReceivePackError")
    parts = text[1:-1].split(",")
    if len(parts) < 10: raise ProtocolError("ReceivePackError")
    try:
        movement, pivot, pid_op, auto = map(int, parts[:4]); flags = tuple(int(value) for value in parts[4:7])
    except ValueError as exc: raise ProtocolError("ReceivePackError") from exc
    groups = tuple(name for name, flag in zip(("balance", "velocity", "turn"), flags) if flag == 1)
    pid = None
    if groups:
        pid = parse_pid_report("$" + ",".join(parts[7:]) + "#")
        if pid is None: raise ProtocolError("ReceivePackError")
    return ControlRequest(movement, pivot, pid_op, auto, groups, pid)
