from __future__ import annotations
from abc import ABC,abstractmethod
from dataclasses import dataclass
from app.types import RobotState, MotorCommand, PolicyOutput, clamp

class Policy(ABC):
    @abstractmethod
    def act(self, state: RobotState, dt: float) -> PolicyOutput: ...
    def reset(self): pass

@dataclass
class PIDConfig: kp:float=22.; ki:float=1.2; kd:float=2.2; output_limit:float=1.; integral_limit:float=.5; desired_pitch:float=0.
class PIDPolicy(Policy):
    def __init__(self, config: PIDConfig|None=None): self.c=config or PIDConfig(); self.reset()
    def reset(self): self.integral=0.; self.previous=0.
    def act(self,s,dt):
        e=self.c.desired_pitch-s.pitch; p=self.c.kp*e; d=self.c.kd*(-s.pitch_rate)
        candidate=clamp(self.integral+e*dt,-self.c.integral_limit,self.c.integral_limit)
        raw=p+self.c.ki*candidate+d; u=clamp(raw,-self.c.output_limit,self.c.output_limit)
        if raw==u or e*u<0: self.integral=candidate
        return PolicyOutput(MotorCommand(u,u),{"P":p,"I":self.c.ki*self.integral,"D":d,"action":u})

class CascadedPIDPolicy(Policy):
    """Outer position loop generates a bounded pitch target for the inner balance loop."""
    def __init__(self, position_kp=.12, max_pitch=.12, inner:PIDConfig|None=None):
        self.position_kp=position_kp;self.max_pitch=max_pitch;self.inner=PIDPolicy(inner)
    def reset(self):self.inner.reset()
    def act(self,s,dt):
        target=clamp(-self.position_kp*s.position,-self.max_pitch,self.max_pitch); old=self.inner.c.desired_pitch;self.inner.c.desired_pitch=target;out=self.inner.act(s,dt);self.inner.c.desired_pitch=old;out.terms["desired_pitch"]=target;return out

class LQRPolicy(Policy):
    # Gain vector is intentionally tuneable, rather than a claim based on unknown plant matrices.
    def __init__(self, gains=(15.,2.5,1.0,.8)): self.gains=gains
    def act(self,s,dt):
        x=(s.pitch,s.pitch_rate,s.position,s.velocity); u=clamp(-sum(a*b for a,b in zip(self.gains,x)),-1.,1.)
        return PolicyOutput(MotorCommand(u,u),{"action":u,"lqr":u})

class PPOPolicy(Policy):
    def __init__(self, model_path: str):
        try:
            from stable_baselines3 import PPO
            self.model=PPO.load(model_path); self.error=None
        except Exception as exc: self.model=None; self.error=str(exc)
    def act(self,s,dt):
        if not self.model: return PolicyOutput(MotorCommand(),{"ppo_error":1.})
        action,_=self.model.predict([s.pitch,s.pitch_rate,s.position,s.velocity],deterministic=True); u=float(action[0] if hasattr(action,'__len__') else action)
        return PolicyOutput(MotorCommand(clamp(u,-1,1),clamp(u,-1,1)),{"action":u,"ppo_action":u})

def load_policy(kind: str, config: dict) -> Policy:
    if kind=="pid": return PIDPolicy(PIDConfig(**config.get("pid",{})))
    if kind=="cascaded_pid": return CascadedPIDPolicy(config.get("position_kp",.12),config.get("max_pitch",.12),PIDConfig(**config.get("pid",{})))
    if kind=="lqr": return LQRPolicy(tuple(config.get("lqr_gains",(15.,2.5,1.,.8))))
    if kind=="ppo": return PPOPolicy(config.get("model_path", ""))
    raise ValueError(f"Unknown controller {kind}")
