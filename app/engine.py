from __future__ import annotations
import threading,time
from app.types import RobotState
from app.physics.simulation import SimulationBackend
from app.safety.layer import SafetyLayer
from app.control.disturbances import DisturbanceGenerator
from app.reward.spec import RewardSpec
from app.telemetry.store import TelemetryStore
class ControlEngine:
    """Dedicated fixed-period control thread; UI communicates through safe methods only."""
    def __init__(self,backend:SimulationBackend,policy,safety:SafetyLayer,telemetry:TelemetryStore,disturbance=None,reward=None,dt=.01, permit_arm=True):
        self.backend,self.policy,self.safety,self.telemetry=backend,policy,safety,telemetry;self.disturbance=disturbance or DisturbanceGenerator();self.reward=reward or RewardSpec();self.dt=dt;self.permit_arm=permit_arm;self.running=False;self.thread=None;self.latest=backend.observe();self._last=None
    def start(self):
        if self.running:return
        if not self.permit_arm: self.safety.disarm("hardware requires verified protocol and explicit arming"); return
        self.running=True;self.safety.arm();self.thread=threading.Thread(target=self._loop,daemon=True);self.thread.start()
    def pause(self):self.running=False;self.safety.disarm("paused")
    def reset(self):self.pause();self.backend.reset();self.policy.reset();self._last=None
    def reset_pose(self, position=0., pitch=0., velocity=0., pitch_rate=0.):
        self.pause()
        if not hasattr(self.backend,"set_pose"): raise RuntimeError("Custom pose reset is available for simulation/mock backends only")
        self.latest=self.backend.set_pose(position,pitch,velocity,pitch_rate);self.policy.reset();self._last=None
    def estop(self):self.safety.estop()
    def _loop(self):
        next_tick=time.monotonic()
        while self.running:
            s=self.backend.observe();out=self.policy.act(s,self.dt);self.safety.heartbeat();cmd=self.safety.filter(s,out.command);self.backend.apply(cmd);self.latest=self.backend.step(self.dt,self.disturbance.force(self.dt));r=self.reward.score(self.latest,cmd,self._last,self.safety.status.armed);self.telemetry.record(self.latest,cmd,self.safety.status,out.terms,r);self._last=cmd
            next_tick+=self.dt;time.sleep(max(0,next_tick-time.monotonic()))
