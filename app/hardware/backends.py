from __future__ import annotations
from abc import ABC, abstractmethod
from app.types import RobotState, MotorCommand
from app.physics.simulation import SimulationBackend

class RobotBackend(ABC):
    @abstractmethod
    def reset(self, seed: int | None = None) -> RobotState: ...
    @abstractmethod
    def observe(self) -> RobotState: ...
    @abstractmethod
    def apply(self, command: MotorCommand) -> None: ...
    @abstractmethod
    def step(self, dt: float, external_force: float = 0.) -> RobotState: ...

class ReplayBackend(RobotBackend):
    def __init__(self, states: list[RobotState]): self.states=states or [RobotState()]; self.i=0
    def reset(self, seed=None): self.i=0; return self.observe()
    def observe(self): return self.states[min(self.i,len(self.states)-1)]
    def apply(self, command): pass
    def step(self, dt, external_force=0.): self.i=min(self.i+1,len(self.states)-1); return self.observe()

class HardwareBackend(RobotBackend):
    """Protocol-neutral, disarmed adapter. Subclass only after vendor transport is verified."""
    def __init__(self): self._state=RobotState(); self.verified_protocol=False
    def reset(self, seed=None): return self._state
    def observe(self): return self._state
    def apply(self, command):
        if not self.verified_protocol: raise RuntimeError("Hardware protocol is unverified; commands blocked")
        self.send_verified(command)
    def send_verified(self, command): raise NotImplementedError("Configure verified Yahboom STM32 transport")
    def step(self, dt, external_force=0.): return self.observe()

class MockHardwareBackend(SimulationBackend, RobotBackend):
    """Interchangeable development stand-in for hardware."""
    pass
