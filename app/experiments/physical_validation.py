"""Frozen physical-validation protocol. It cannot actuate an unverified robot."""
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import argparse, hashlib, json, math, random, statistics, time
from app.safety.layer import SafetyConfig
from app.types import RobotState, MotorCommand

REQUIRED_METRICS=("survival_time","rms_pitch","peak_pitch","recovery_time","angular_velocity_rms","motor_effort","position_drift","success")

def canonical_hash(value): return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":")).encode()).hexdigest()
def file_hash(path):
    h=hashlib.sha256()
    with open(path,"rb") as f:
        for block in iter(lambda:f.read(65536),b""):h.update(block)
    return h.hexdigest()

@dataclass(frozen=True)
class ModelArtifact:
    model_path:str; model_sha256:str; observation_schema_version:str; action_scaling_version:str; reward_spec:dict; validated:bool=False
    @classmethod
    def load(cls,path):
        raw=json.loads(Path(path).read_text()); artifact=cls(**raw)
        if not artifact.validated:raise ValueError("model artifact is not explicitly validated")
        if not Path(artifact.model_path).is_file():raise ValueError("model artifact file is missing")
        if file_hash(artifact.model_path)!=artifact.model_sha256:raise ValueError("model hash mismatch")
        return artifact

@dataclass
class TrialResult:
    trial_id:str; controller_id:str; condition_id:str; order:int; seed:int; timestamp:str; battery_voltage:float; metrics:dict; metadata:dict

class PhysicalValidationRunner:
    """Schedules immutable trials, checkpointing after each result. No Ollama or reward search is reachable here."""
    def __init__(self,config_path,results_root="results"):
        self.config_path=Path(config_path);self.config=json.loads(self.config_path.read_text());self.root=Path(results_root)/self.config["experiment_id"];self.root.mkdir(parents=True,exist_ok=True)
        self.fingerprint=canonical_hash(self.config);self.checkpoint=self.root/"checkpoint.json";self.raw=self.root/"physical_trials.jsonl"
        if self.checkpoint.exists() and json.loads(self.checkpoint.read_text()).get("config_fingerprint")!=self.fingerprint:raise RuntimeError("configuration changed after trials began; start a new experiment id")
    def validate_preflight(self):
        c=self.config;SafetyConfig(**c["safety"])
        if c["mode"] not in ("development","final"):raise ValueError("mode must be development or final")
        if c["trials_per_controller"] < (5 if c["mode"]=="development" else 10):raise ValueError("insufficient trials for selected mode")
        hw=json.loads(Path(c["hardware_config"]).read_text())
        if hw.get("status")!="VERIFIED_FOR_TEST" : raise RuntimeError("hardware remains DISARMED: verified transport/configuration required")
        for item in c["controllers"]:
            if item["type"]=="ppo":
                a=ModelArtifact.load(item["artifact_manifest"])
                if a.observation_schema_version!=c["observation_schema_version"] or a.action_scaling_version!=c["action_scaling_version"]:raise ValueError("PPO observation/action version mismatch")
        return True
    def schedule(self):
        rows=[];n=self.config["trials_per_controller"]
        for condition in self.config["conditions"]:
            for controller in self.config["controllers"]:
                for trial in range(n):rows.append({"controller_id":controller["id"],"condition_id":condition["id"],"trial":trial})
        random.Random(self.config["seed"]).shuffle(rows)
        return rows
    def completed(self):
        if not self.raw.exists():return []
        return [json.loads(x) for x in self.raw.read_text().splitlines() if x]
    def next_trial(self):
        done={x["trial_id"] for x in self.completed()}
        for order,row in enumerate(self.schedule()):
            trial_id=f"{row['condition_id']}--{row['controller_id']}--{row['trial']:03d}"
            if trial_id not in done:return order,trial_id,row
        return None
    def record(self,trial_id,order,row,battery_voltage,metrics,raw_telemetry_path,operator=""):
        if not self.config["battery_voltage_min"]<=battery_voltage<=self.config["battery_voltage_max"]:raise ValueError("battery outside frozen test range")
        if set(metrics)!=set(REQUIRED_METRICS):raise ValueError("metrics must exactly match protocol")
        if not Path(raw_telemetry_path).is_file():raise ValueError("raw telemetry file is required")
        result=TrialResult(trial_id,row["controller_id"],row["condition_id"],order,self.config["seed"]+order,time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()),battery_voltage,metrics,{"config_fingerprint":self.fingerprint,"raw_telemetry_sha256":file_hash(raw_telemetry_path),"raw_telemetry_path":str(raw_telemetry_path),"operator":operator,"hardware_config":self.config["hardware_config"]})
        with self.raw.open("a",encoding="utf8") as f:f.write(json.dumps(asdict(result))+"\n")
        self.checkpoint.write_text(json.dumps({"config_fingerprint":self.fingerprint,"completed":len(self.completed()),"next":self.next_trial()}),encoding="utf8")
        return result

def telemetry_metrics(rows,dt=.01):
    if not rows:return {k:0 for k in REQUIRED_METRICS}
    pitch=[float(x["pitch"]) for x in rows];rate=[float(x.get("pitch_rate",0)) for x in rows];cmd=[abs(float(x.get("left_command",0))) for x in rows]
    recovery=next((i*dt for i,x in enumerate(pitch) if abs(x)<.05),len(rows)*dt);success=all(bool(x.get("armed",False)) for x in rows)
    return {"survival_time":len(rows)*dt,"rms_pitch":math.sqrt(sum(x*x for x in pitch)/len(pitch)),"peak_pitch":max(map(abs,pitch)),"recovery_time":recovery,"angular_velocity_rms":math.sqrt(sum(x*x for x in rate)/len(rate)),"motor_effort":sum(cmd)/len(cmd),"position_drift":abs(float(rows[-1].get("position",0))),"success":success}

def summary(records):
    groups={}
    for r in records:groups.setdefault((r["controller_id"],r["condition_id"]),[]).append(r["metrics"])
    out={}
    for key,rows in groups.items():
        out[" / ".join(key)]={}
        for metric in REQUIRED_METRICS:
            values=[float(x[metric]) for x in rows];mean=statistics.mean(values);sd=statistics.stdev(values) if len(values)>1 else 0
            out[" / ".join(key)][metric]={"n":len(values),"mean":mean,"median":statistics.median(values),"std":sd,"ci95_half_width":1.96*sd/math.sqrt(len(values)) if values else 0}
    return out

def svg_bar_chart(stats,metric,path):
    items=[(k,v[metric]["mean"]) for k,v in stats.items()];maximum=max((x[1] for x in items),default=1) or 1;w,h=760,60+55*len(items);parts=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}"><style>text{{font:12px sans-serif}}</style>']
    for i,(name,value) in enumerate(items):y=30+i*55;bar=500*value/maximum;parts+= [f'<text x="8" y="{y}">{name}</text>',f'<rect x="230" y="{y-14}" width="{bar}" height="22" fill="#2563eb"/>',f'<text x="{238+bar}" y="{y}">{value:.4g}</text>']
    Path(path).write_text("".join(parts)+"</svg>")
def svg_traces(records,path,field):
    lines=['<svg xmlns="http://www.w3.org/2000/svg" width="760" height="280"><rect width="100%" height="100%" fill="white"/><text x="8" y="16" font-family="sans-serif" font-size="12">'+field+' traces (one line per recorded trial)</text>']; colors=("#2563eb","#dc2626","#16a34a","#7c3aed","#ea580c")
    for n,r in enumerate(records):
        try:rows=[json.loads(x) for x in Path(r["metadata"]["raw_telemetry_path"]).read_text().splitlines() if x];vals=[float(x.get(field,0)) for x in rows]
        except (OSError,KeyError,ValueError):vals=[]
        if len(vals)>1:
            scale=max(max(map(abs,vals)),.001);pts=[]
            for i,v in enumerate(vals):pts.extend((30+i*700/(len(vals)-1),145-v*110/scale))
            lines.append('<polyline points="'+' '.join(f'{pts[i]:.1f},{pts[i+1]:.1f}' for i in range(0,len(pts),2))+'" fill="none" stroke="'+colors[n%len(colors)]+'"/>')
    Path(path).write_text("".join(lines)+"</svg>")
def generate_report(results_dir):
    root=Path(results_dir);records=[json.loads(x) for x in (root/"physical_trials.jsonl").read_text().splitlines() if x];stats=summary(records)
    for metric in ("survival_time","rms_pitch","recovery_time","motor_effort","success"):svg_bar_chart(stats,metric,root/f"{metric}.svg")
    svg_traces(records,root/"pitch_traces.svg","pitch");svg_traces(records,root/"recovery_traces.svg","pitch")
    lines=["# Physical validation report","","## Scope","This report separates recorded physical trials from simulation results. It makes no improvement claim unless the physical measurements below support it.","","## Trial-by-trial table","","| trial | controller | condition | survival | RMS pitch | recovery | success |","|---|---|---|---:|---:|---:|---:|"]
    lines += [f"| {r['trial_id']} | {r['controller_id']} | {r['condition_id']} | {r['metrics']['survival_time']:.3f} | {r['metrics']['rms_pitch']:.4f} | {r['metrics']['recovery_time']:.3f} | {r['metrics']['success']} |" for r in records]
    lines += ["","## Aggregate statistics","","```json",json.dumps(stats,indent=2),"```"]
    (root/"report.md").write_text("\n".join(lines),encoding="utf8");return root/"report.md"

def main():
    p=argparse.ArgumentParser();p.add_argument("--config",default="configs/physical_validation_development.json");p.add_argument("--results",default="results");p.add_argument("--plan",action="store_true");p.add_argument("--report",action="store_true");p.add_argument("--record",help="raw telemetry JSONL path for the next frozen trial");p.add_argument("--battery",type=float);p.add_argument("--operator",default="");args=p.parse_args();runner=PhysicalValidationRunner(args.config,args.results)
    if args.report:print(generate_report(runner.root));return
    if args.plan:
        print(json.dumps({"frozen_config_fingerprint":runner.fingerprint,"scheduled_trials":runner.schedule(),"next":runner.next_trial(),"hardware_status":"DISARMED until verified preflight"},indent=2));return
    if args.record:
        nxt=runner.next_trial()
        if not nxt:raise SystemExit("all trials are already recorded")
        order,trial_id,row=nxt; rows=[json.loads(x) for x in Path(args.record).read_text().splitlines() if x];metrics=telemetry_metrics(rows)
        print(json.dumps(asdict(runner.record(trial_id,order,row,args.battery,metrics,args.record,args.operator)),indent=2));return
    runner.validate_preflight()
if __name__=="__main__":main()
