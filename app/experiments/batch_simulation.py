"""Checkpointable large-scale simulation evaluation. Never selects/acts on hardware."""
from __future__ import annotations
import argparse,json,math,random
from pathlib import Path
from app.control.policies import load_policy
from app.physics.simulation import SimulationBackend,PhysicsConfig
from app.types import MotorCommand

def one_trial(controller,model_path,seed,timesteps,randomized):
    rng=random.Random(seed);c=PhysicsConfig()
    if randomized:
        c.body_mass_scale=rng.uniform(.75,1.25);c.com_height_scale=rng.uniform(.8,1.2);c.wheel_radius_scale=rng.uniform(.9,1.1);c.motor_gain=rng.uniform(.7,1.3);c.friction=rng.uniform(.03,.18);c.left_right_mismatch=rng.uniform(-.15,.15);c.sensor_noise=rng.uniform(0,.01);c.sensor_bias=rng.uniform(-.02,.02);c.sensor_latency_s=rng.uniform(0,.04);c.actuator_latency_s=rng.uniform(0,.04);c.battery_voltage=rng.uniform(6.5,8.2)
    b=SimulationBackend(c,seed);p=load_policy(controller,{"model_path":model_path});angles=[];effort=[]
    for _ in range(timesteps):
        s=b.observe();o=p.act(s,.01);b.apply(o.command);s=b.step(.01);angles.append(s.pitch);effort.append(abs(o.command.left))
        if abs(s.pitch)>.52:break
    return {"seed":seed,"survival_time":len(angles)*.01,"rms_pitch":math.sqrt(sum(x*x for x in angles)/len(angles)),"peak_pitch":max(map(abs,angles)),"position_drift":abs(s.position),"motor_effort":sum(effort)/len(effort),"randomized":randomized}
def main():
    p=argparse.ArgumentParser();p.add_argument("--controller",default="pid",choices=["pid","cascaded_pid","lqr","ppo"]);p.add_argument("--model-path",default="");p.add_argument("--trials",type=int,default=10);p.add_argument("--iterations",type=int,default=1);p.add_argument("--timesteps",type=int,default=1000);p.add_argument("--seed",type=int,default=7);p.add_argument("--randomized",action="store_true");p.add_argument("--results",default="results/batch_simulation.json");p.add_argument("--allow-large",action="store_true");a=p.parse_args()
    if a.trials*a.iterations>1000 and not a.allow_large:raise SystemExit("Over 1000 runs needs --allow-large; this is simulation-only and may take time.")
    if a.controller=="ppo" and not a.model_path:raise SystemExit("--model-path is required for PPO")
    path=Path(a.results);path.parent.mkdir(parents=True,exist_ok=True);saved=json.loads(path.read_text()) if path.exists() else {"parameters":vars(a),"trials":[]};done=len(saved["trials"])
    for i in range(done,a.trials*a.iterations):
        saved["trials"].append(one_trial(a.controller,a.model_path,a.seed+i,a.timesteps,a.randomized));path.write_text(json.dumps(saved,indent=2))
        print(f"checkpoint {i+1}/{a.trials*a.iterations}: {path}")
if __name__=="__main__":main()
