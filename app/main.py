from __future__ import annotations
import json,argparse
from pathlib import Path
from app.physics.simulation import SimulationBackend,PhysicsConfig
from app.hardware.backends import HardwareBackend,MockHardwareBackend,ReplayBackend
from app.control.policies import load_policy
from app.safety.layer import SafetyLayer,SafetyConfig
from app.telemetry.store import TelemetryStore
from app.engine import ControlEngine
def make_engine(config):
    p={k:v for k,v in config.get("physics",{}).items() if v is not None}; kind=config.get("backend","simulation")
    if kind=="simulation": backend=SimulationBackend(PhysicsConfig(**p),config.get("seed",0))
    elif kind=="mock_hardware": backend=MockHardwareBackend(PhysicsConfig(**p),config.get("seed",0))
    elif kind=="replay": backend=ReplayBackend([])
    elif kind=="hardware": backend=HardwareBackend()
    else: raise ValueError(f"Unknown backend {kind}")
    cc=config["controller"];return ControlEngine(backend,load_policy(cc["type"],cc),SafetyLayer(SafetyConfig(**config.get("safety",{}))),TelemetryStore("results"),permit_arm=kind!="hardware")
def main():
    ap=argparse.ArgumentParser();ap.add_argument("--config",default="configs/default.json");ap.add_argument("--headless-smoke",action="store_true");a=ap.parse_args();config=json.loads(Path(a.config).read_text());engine=make_engine(config)
    if a.headless_smoke:
        engine.start();__import__('time').sleep(.15);engine.pause();engine.telemetry.flush("smoke");print("headless smoke passed",len(engine.telemetry.rows));return
    import tkinter as tk
    from app.ui.dashboard import Dashboard
    root=tk.Tk();Dashboard(root,engine,config["controller"]);root.protocol("WM_DELETE_WINDOW",lambda:(engine.pause(),engine.telemetry.flush(),root.destroy()));root.mainloop()
if __name__=="__main__":main()
