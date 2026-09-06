from __future__ import annotations
from dataclasses import dataclass,asdict
from pathlib import Path
import json, math, random, time
from app.physics.simulation import SimulationBackend,PhysicsConfig
from app.safety.layer import SafetyLayer
from app.control.policies import Policy
from app.control.disturbances import DisturbanceGenerator
from app.reward.spec import RewardSpec
from app.telemetry.store import TelemetryStore
@dataclass
class ExperimentConfig:
    mode:str="development"; trials:int=1; iterations:int=1; timesteps:int=300; seed:int=7; randomized:bool=False
class ExperimentManager:
    def __init__(self, results="results"):self.results=Path(results);self.results.mkdir(exist_ok=True)
    def run(self, config:ExperimentConfig, make_policy, reward=RewardSpec(), checkpoint="checkpoint.json"):
        if config.trials>1000 or config.iterations>10:raise ValueError("limits are 1000 trials x 10 iterations")
        random.seed(config.seed); all_metrics=[]
        for trial in range(config.trials):
            pc=PhysicsConfig()
            if config.randomized: pc.motor_gain=random.uniform(.7,1.3);pc.friction=random.uniform(.03,.18);pc.sensor_noise=random.uniform(0,.01);pc.left_right_mismatch=random.uniform(-.15,.15)
            backend=SimulationBackend(pc,config.seed+trial);policy=make_policy();safety=SafetyLayer();safety.arm();store=TelemetryStore(self.results);dist=DisturbanceGenerator()
            previous=None; angles=[]; alive_steps=0
            for _ in range(config.timesteps):
                s=backend.observe();out=policy.act(s,.01);safety.heartbeat();cmd=safety.filter(s,out.command);backend.apply(cmd);s=backend.step(.01,dist.force(.01)); alive=safety.status.armed; r=reward.score(s,cmd,previous,alive);store.record(s,cmd,safety.status,out.terms,r);previous=cmd;angles.append(s.pitch);alive_steps+=int(alive)
                if not alive:break
            rms=math.sqrt(sum(x*x for x in angles)/max(1,len(angles)))
            # Stable, comparable evaluation metrics used by both advisory pipelines.
            metrics={"rms_angle":rms,"peak_angle":max(map(abs,angles),default=0),"settling_time":next((i*.01 for i,x in enumerate(angles) if all(abs(y)<.05 for y in angles[i:])),len(angles)*.01),"recovery_time":next((i*.01 for i,x in enumerate(angles) if abs(x)<.1),len(angles)*.01),"survival_time":alive_steps*.01,"angular_velocity":abs(s.pitch_rate),"angular_acceleration":abs(s.pitch_accel),"position_drift":abs(s.position),"control_effort":sum(abs(r["left_command"]) for r in store.rows)/max(1,len(store.rows)),"command_variation":sum(abs(store.rows[i]["left_command"]-store.rows[i-1]["left_command"]) for i in range(1,len(store.rows)))/max(1,len(store.rows)-1),"success":safety.status.armed,"failure_reason":safety.status.reason if not safety.status.armed else ""}
            all_metrics.append(metrics);store.flush(f"trial_{trial}")
            Path(self.results/checkpoint).write_text(json.dumps({"next_trial":trial+1,"config":asdict(config),"metrics":all_metrics}),encoding="utf8")
        return all_metrics
