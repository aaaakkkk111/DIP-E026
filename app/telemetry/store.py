from __future__ import annotations
import json, sqlite3, csv, threading
from pathlib import Path
from dataclasses import asdict
from app.types import RobotState,MotorCommand,SafetyStatus
class TelemetryStore:
    def __init__(self, root="results"):
        self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True); self.rows=[]; self.lock=threading.Lock()
        self.db=sqlite3.connect(self.root/"lab.sqlite",check_same_thread=False); self.db.execute("create table if not exists runs (id text primary key, config text, summary text)");self.db.commit()
    def record(self,state:RobotState,cmd:MotorCommand,safety:SafetyStatus,terms:dict,reward:float):
        row={**asdict(state),"left_command":cmd.left,"right_command":cmd.right,"armed":safety.armed,"safety_reason":safety.reason,"saturated":safety.saturated,"reward":reward,**terms}
        with self.lock:self.rows.append(row)
    def flush(self, run_id="live"):
        with self.lock: rows=self.rows[:]
        if not rows:return
        keys=sorted({k for row in rows for k in row}); path=self.root/f"{run_id}.jsonl"
        with path.open("w",encoding="utf8") as f:
            for row in rows:f.write(json.dumps(row)+"\n")
        with (self.root/f"{run_id}.csv").open("w",newline="",encoding="utf8") as f:
            w=csv.DictWriter(f,keys,extrasaction="ignore");w.writeheader();w.writerows(rows)
    def save_run(self,run_id,config,summary): self.db.execute("insert or replace into runs values(?,?,?)",(run_id,json.dumps(config),json.dumps(summary)));self.db.commit()
class LineAnalyzer:
    def __init__(self, store:TelemetryStore):self.store=store
    def series(self, field):
        with self.store.lock:return [r.get(field) for r in self.store.rows]
    def bounds(self,field):
        s=[x for x in self.series(field) if isinstance(x,(int,float))]
        return (min(s),max(s)) if s else (0.,0.)
    @staticmethod
    def load_jsonl(path):
        with Path(path).open(encoding="utf8") as f:return [json.loads(line) for line in f]
