"""Optional CPU PPO training entry point; physical validation never invokes it."""
from __future__ import annotations
from dataclasses import asdict
from pathlib import Path
import hashlib, json
from app.physics.simulation import SimulationBackend,PhysicsConfig
from app.reward.spec import RewardSpec
from app.types import MotorCommand
import argparse

class SimulationCandidateEvaluator:
    """Ranks policies by balance/recovery metrics on held-out nominal and randomized seeds, never reward alone."""
    def __init__(self,policy_factory,seeds=(101,102,103),steps=1000):self.policy_factory=policy_factory;self.seeds=seeds;self.steps=steps
    def evaluate(self,reward:RewardSpec,randomized=False):
        scores=[]
        for seed in self.seeds:
            c=PhysicsConfig()
            if randomized:
                import random;r=random.Random(seed);c.body_mass_scale=r.uniform(.75,1.25);c.com_height_scale=r.uniform(.8,1.2);c.wheel_radius_scale=r.uniform(.9,1.1);c.motor_gain=r.uniform(.7,1.3);c.friction=r.uniform(.03,.18);c.left_right_mismatch=r.uniform(-.15,.15);c.sensor_noise=r.uniform(0,.01);c.sensor_bias=r.uniform(-.02,.02);c.sensor_latency_s=r.uniform(0,.04);c.actuator_latency_s=r.uniform(0,.04);c.battery_voltage=r.uniform(6.5,8.2)
            b=SimulationBackend(c,seed);p=self.policy_factory();angles=[];commands=[]
            for _ in range(self.steps):
                s=b.observe();out=p.act(s,.01);b.apply(out.command);s=b.step(.01);angles.append(s.pitch);commands.append(out.command.left)
                if abs(s.pitch)>.52:break
            import math;scores.append({"survival_time":len(angles)*.01,"rms_angle":math.sqrt(sum(x*x for x in angles)/len(angles)),"peak_angle":max(map(abs,angles)),"position_drift":abs(s.position),"control_effort":sum(map(abs,commands))/len(commands)})
        return {k:sum(x[k] for x in scores)/len(scores) for k in scores[0]}

def train_ppo(output_path,reward:RewardSpec,seed=0,timesteps=10000,resume_path=None,checkpoint_steps=0):
    """Train only in simulation; requires the optional `. [ppo]` dependencies."""
    try:
        import gymnasium as gym
        from gymnasium import spaces
        import numpy as np
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CheckpointCallback
    except ImportError as exc:raise RuntimeError("Install optional PPO dependencies: python -m pip install -e '.[ppo]'") from exc
    class Env(gym.Env):
        observation_space=spaces.Box(-np.inf,np.inf,(4,),np.float32);action_space=spaces.Box(-1,1,(1,),np.float32)
        def reset(self,seed=None,options=None):super().reset(seed=seed);self.b=SimulationBackend(seed=seed or 0);self.prev=MotorCommand();self.i=0;return self.obs(),{}
        def obs(self):s=self.b.observe();return np.array([s.pitch,s.pitch_rate,s.position,s.velocity],dtype=np.float32)
        def step(self,a):
            cmd=MotorCommand(float(a[0]),float(a[0]));self.b.apply(cmd);s=self.b.step(.01);self.i+=1;bad=abs(s.pitch)>.52;value=reward.score(s,cmd,self.prev,not bad);self.prev=cmd;return self.obs(),value,bad,self.i>=2000,{"physical_metrics_only":True}
    model=PPO.load(resume_path,env=Env()) if resume_path else PPO("MlpPolicy",Env(),seed=seed,verbose=0)
    callbacks=[]
    if checkpoint_steps:callbacks.append(CheckpointCallback(save_freq=checkpoint_steps,save_path=str(Path(output_path).parent/"checkpoints"),name_prefix=Path(output_path).stem))
    model.learn(total_timesteps=timesteps,callback=callbacks or None,reset_num_timesteps=not bool(resume_path));model.save(output_path);return output_path

def main():
    p=argparse.ArgumentParser(description="Train PPO only in the local simulation")
    p.add_argument("--output",default="models/ppo_baseline.zip");p.add_argument("--timesteps",type=int,default=10000);p.add_argument("--seed",type=int,default=7);p.add_argument("--reward",default="configs/default.json");p.add_argument("--resume",help="existing SB3 PPO .zip checkpoint");p.add_argument("--checkpoint-steps",type=int,default=10000)
    a=p.parse_args();raw=json.loads(Path(a.reward).read_text());spec=RewardSpec(**raw.get("reward",{})).validate();Path(a.output).parent.mkdir(parents=True,exist_ok=True);print(train_ppo(a.output,spec,a.seed,a.timesteps,a.resume,a.checkpoint_steps))
if __name__=="__main__":main()
