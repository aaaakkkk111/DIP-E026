from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Any
import math

@dataclass
class RobotState:
    pitch: float = 0.; pitch_rate: float = 0.; pitch_accel: float = 0.
    position: float = 0.; velocity: float = 0.
    left_speed: float = 0.; right_speed: float = 0.; battery: float = 7.4
    timestamp: float = 0.; disturbance: float = 0.

@dataclass
class MotorCommand:
    left: float = 0.; right: float = 0.; timestamp: float = 0.

@dataclass
class SafetyStatus:
    armed: bool = False; estop: bool = False; reason: str = "disarmed"; saturated: bool = False

@dataclass
class PolicyOutput:
    command: MotorCommand = field(default_factory=MotorCommand)
    terms: dict[str, float] = field(default_factory=dict)

def finite_state(s: RobotState) -> bool:
    return all(math.isfinite(v) for v in asdict(s).values())

def clamp(x: float, lo: float, hi: float) -> float: return max(lo, min(hi, x))
