"""Deployable telemetry decoding and data-quality accounting."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from .protocol import ProtocolError, verify_checked_frame


@dataclass(frozen=True)
class FastTelemetry:
    tick_ms: int; pitch_deg: float; pitch_rate_deg_s: float; yaw_rate_deg_s: float
    encoder_left: int; encoder_right: int; balance_pwm: int; velocity_pwm: int
    turn_pwm: int; pwm_left: int; pwm_right: int; flags: int; command: int
    def as_dict(self) -> dict[str, Any]: return asdict(self)


@dataclass(frozen=True)
class SlowTelemetry:
    tick_ms: int; battery_v: float; rx_errors: int; tx_drops: int; control_overruns: int
    def as_dict(self) -> dict[str, Any]: return asdict(self)


def parse_extended_telemetry(frame: bytes) -> FastTelemetry | SlowTelemetry:
    payload = verify_checked_frame(frame); parts = payload.split(",")
    try:
        if parts[0] == "E1F" and len(parts) == 14:
            v = list(map(int, parts[1:])); return FastTelemetry(v[0], v[1]/100, v[2]/100, v[3]/100,
                v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12])
        if parts[0] == "E1S" and len(parts) == 6:
            v = list(map(int, parts[1:])); return SlowTelemetry(v[0], v[1]/1000, v[2], v[3], v[4])
    except (ValueError, IndexError) as exc: raise ProtocolError("invalid telemetry field type") from exc
    raise ProtocolError("unknown telemetry version/type")


class TelemetryDecoder:
    def __init__(self) -> None:
        from .protocol import FrameStreamParser
        self.parser = FrameStreamParser(); self.fast: list[FastTelemetry] = []
        self.slow: list[SlowTelemetry] = []; self.raw_frames: list[str] = []
        self.invalid_frames = 0; self.other_frames: list[bytes] = []

    def feed(self, chunk: bytes) -> None:
        for frame in self.parser.feed(chunk):
            self.raw_frames.append(frame.decode("ascii", errors="replace"))
            if frame.startswith(b"$E1"):
                try: item = parse_extended_telemetry(frame)
                except ProtocolError: self.invalid_frames += 1
                else: (self.fast if isinstance(item, FastTelemetry) else self.slow).append(item)
            else: self.other_frames.append(frame)

    def quality(self, expected_fast: int) -> dict[str, Any]:
        received = len(self.fast); missing = max(0, expected_fast-received)
        return {"expected_fast_frames": expected_fast, "received_fast_frames": received,
                "missing_fast_frames": missing, "loss_ratio": missing/max(1, expected_fast),
                "invalid_frames": self.invalid_frames, "parser_discarded_frames": self.parser.discarded_frames,
                "latest_tick_ms": self.fast[-1].tick_ms if self.fast else None}
