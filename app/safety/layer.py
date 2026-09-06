from __future__ import annotations
from dataclasses import dataclass
import time
from app.types import RobotState, MotorCommand, SafetyStatus, finite_state, clamp

@dataclass
class SafetyConfig:
    max_pitch_rad: float=.52; max_wheel_speed: float=25.; max_command: float=1.; watchdog_s: float=.25

class SafetyLayer:
    def __init__(self, config: SafetyConfig | None=None): self.config=config or SafetyConfig(); self.status=SafetyStatus(); self._heartbeat=0.
    def arm(self):
        if not self.status.estop: self.status=SafetyStatus(True,False,"armed"); self.heartbeat()
    def disarm(self, reason="disarmed"): self.status=SafetyStatus(False,self.status.estop,reason)
    def estop(self): self.status=SafetyStatus(False,True,"E-STOP")
    def reset_estop(self): self.status=SafetyStatus(False,False,"disarmed")
    def heartbeat(self): self._heartbeat=time.monotonic()
    def filter(self, state: RobotState, raw: MotorCommand) -> MotorCommand:
        c=self.config
        reason=None
        if not self.status.armed: reason=self.status.reason
        elif self.status.estop: reason="E-STOP"
        elif not finite_state(state) or not all(__import__('math').isfinite(x) for x in (raw.left,raw.right)): reason="invalid data"
        elif time.monotonic()-self._heartbeat>c.watchdog_s: reason="watchdog timeout"
        elif abs(state.pitch)>c.max_pitch_rad: reason="max pitch"
        elif max(abs(state.left_speed),abs(state.right_speed))>c.max_wheel_speed: reason="max wheel speed"
        if reason:
            self.disarm(reason); return MotorCommand(timestamp=time.monotonic())
        left,right=clamp(raw.left,-c.max_command,c.max_command),clamp(raw.right,-c.max_command,c.max_command)
        self.status.saturated=(left!=raw.left or right!=raw.right); self.status.reason="armed"; return MotorCommand(left,right,time.monotonic())
