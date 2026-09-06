from dataclasses import dataclass
import random, math
@dataclass
class DisturbanceConfig:
    kind:str="manual_impulse"; magnitude:float=0.; duration_s:float=.1; direction:float=1.; probability:float=0.; frequency_hz:float=1.; seed:int=0
class DisturbanceGenerator:
    def __init__(self,c:DisturbanceConfig|None=None): self.c=c or DisturbanceConfig(); self.rng=random.Random(self.c.seed); self.manual_until=0.; self.t=0.
    def trigger(self, magnitude=None, duration=None, direction=None):
        if magnitude is not None:self.c.magnitude=magnitude
        if duration is not None:self.c.duration_s=duration
        if direction is not None:self.c.direction=direction
        self.manual_until=self.t+self.c.duration_s
    def force(self,dt):
        self.t+=dt; c=self.c; active=self.t<self.manual_until
        if c.kind in ("manual_impulse","step") and active:return c.magnitude*c.direction
        if c.kind=="sinusoidal":return c.magnitude*c.direction*math.sin(2*math.pi*c.frequency_hz*self.t)
        if c.kind=="random" and self.rng.random()<c.probability*dt:return c.magnitude*c.direction*(1 if self.rng.random()<.5 else -1)
        return 0.
