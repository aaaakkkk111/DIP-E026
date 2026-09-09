from __future__ import annotations
import tkinter as tk
from tkinter import ttk, filedialog
import math, threading
from app.control.policies import load_policy
from app.reward.spec import RewardSpec
from app.training.ollama import OllamaAdvisor
from app.experiments.physical_validation import PhysicalValidationRunner, generate_report

class Dashboard:
    """Render-only UI. Control timing lives in ControlEngine's worker thread."""
    def __init__(self,root,engine,controller_config):
        self.root,self.engine,self.cc=root,engine,controller_config;root.title("Balance Bot Lab");root.geometry("1180x780")
        top=ttk.Frame(root);top.pack(fill="x",padx=8,pady=5)
        for text,fn in (("START",engine.start),("PAUSE",engine.pause),("RESET",engine.reset),("E-STOP",engine.estop)):ttk.Button(top,text=text,command=fn).pack(side="left",padx=2)
        self.kind=tk.StringVar(value=controller_config.get("type","pid"));ttk.Combobox(top,textvariable=self.kind,values=["pid","cascaded_pid","lqr","ppo"],width=12,state="readonly").pack(side="left",padx=8);ttk.Button(top,text="Apply controller",command=self.change).pack(side="left");ttk.Button(top,text="Physical experiment plan",command=self.physical_plan).pack(side="right")
        frame=ttk.PanedWindow(root,orient="horizontal");frame.pack(fill="both",expand=True,padx=8)
        left=ttk.Frame(frame);right=ttk.Frame(frame);frame.add(left,weight=3);frame.add(right,weight=2)
        viewbox=ttk.LabelFrame(left,text="Manufacturer-informed 3D robot view — orbit controls");viewbox.pack(fill="both",expand=True)
        self.canvas=tk.Canvas(viewbox,width=650,height=360,bg="#f3f6fb");self.canvas.pack(fill="both",expand=True);controls=ttk.Frame(viewbox);controls.pack(fill="x")
        self.yaw=tk.DoubleVar(value=-25);self.view_pitch=tk.DoubleVar(value=18)
        ttk.Label(controls,text="yaw").pack(side="left");ttk.Scale(controls,from_=-180,to=180,variable=self.yaw,orient="horizontal",length=210).pack(side="left");ttk.Label(controls,text="view elevation").pack(side="left");ttk.Scale(controls,from_=-70,to=70,variable=self.view_pitch,orient="horizontal",length=190).pack(side="left")
        graph=ttk.LabelFrame(left,text="Live PID / command line analyzer");graph.pack(fill="x",pady=5);self.graph=tk.Canvas(graph,width=650,height=160,bg="white");self.graph.pack(fill="both")
        self.info=tk.StringVar();ttk.Label(left,textvariable=self.info,font=("Consolas",9),justify="left").pack(anchor="w")
        self._control_panel(right);self.refresh()
    def _control_panel(self,parent):
        reset=ttk.LabelFrame(parent,text="Custom simulation reset state");reset.pack(fill="x",pady=4);self.reset_vars={name:tk.DoubleVar(value=0.) for name in ("position","pitch","velocity","pitch_rate")}
        for name,v in self.reset_vars.items():ttk.Label(reset,text=name).pack(side="left");ttk.Entry(reset,textvariable=v,width=6).pack(side="left",padx=2)
        ttk.Button(reset,text="Reset pose",command=self.reset_pose).pack(side="left",padx=4)
        dist=ttk.LabelFrame(parent,text="Disturbances");dist.pack(fill="x",pady=4);self.mag=tk.DoubleVar(value=.5);self.duration=tk.DoubleVar(value=.12);self.direction=tk.DoubleVar(value=1)
        for label,var in (("magnitude",self.mag),("duration s",self.duration),("direction",self.direction)):ttk.Label(dist,text=label).pack(side="left");ttk.Entry(dist,textvariable=var,width=6).pack(side="left")
        ttk.Button(dist,text="Manual impulse",command=lambda:self.engine.disturbance.trigger(self.mag.get(),self.duration.get(),self.direction.get())).pack(side="left",padx=3);ttk.Button(dist,text="Random force",command=self.random_force).pack(side="left")
        ppo=ttk.LabelFrame(parent,text="PPO + local Ollama reward design (Pipeline B)");ppo.pack(fill="both",expand=True,pady=4);self.model_path=tk.StringVar(value=self.cc.get("model_path",""));ttk.Entry(ppo,textvariable=self.model_path,width=38).pack(fill="x",padx=4,pady=2);ttk.Button(ppo,text="Browse .zip PPO model",command=self.browse_model).pack(anchor="w",padx=4)
        ttk.Label(ppo,text="Prompt for local Ollama (structured weights only):").pack(anchor="w",padx=4);self.prompt=tk.Text(ppo,height=5,width=48);self.prompt.insert("1.0","Design a robust balance reward. Prioritize recovery after a disturbance and reduce motor jerk.");self.prompt.pack(padx=4,pady=2)
        self.ollama_model=tk.StringVar(value="llama3.2");self.ollama_url=tk.StringVar(value="http://localhost:11434/api/generate");ttk.Label(ppo,text="Ollama model").pack(anchor="w",padx=4);ttk.Entry(ppo,textvariable=self.ollama_model).pack(fill="x",padx=4);ttk.Label(ppo,text="Local API endpoint").pack(anchor="w",padx=4);ttk.Entry(ppo,textvariable=self.ollama_url).pack(fill="x",padx=4);ttk.Button(ppo,text="Ask Ollama and preview reward",command=self.ask_ollama).pack(pady=4);self.ollama_output=tk.Text(ppo,height=12,width=48,state="disabled");self.ollama_output.pack(fill="both",expand=True,padx=4,pady=3)
    def change(self):self.cc["type"]=self.kind.get();self.cc["model_path"]=self.model_path.get();self.engine.policy=load_policy(self.kind.get(),self.cc);self.engine.policy.reset()
    def browse_model(self):
        path=filedialog.askopenfilename(filetypes=[("Stable-Baselines PPO","*.zip"),("All files","*")])
        if path:self.model_path.set(path);self.kind.set("ppo");self.change()
    def reset_pose(self):
        try:self.engine.reset_pose(**{k:v.get() for k,v in self.reset_vars.items()})
        except Exception as exc:self.show_output("Reset failed: "+str(exc))
    def random_force(self):
        d=self.engine.disturbance;d.c.kind="random";d.c.magnitude=self.mag.get();d.c.probability=.5;d.c.direction=self.direction.get()
    def physical_plan(self):
        """Schedule/report viewer only; it has no physical command path."""
        win=tk.Toplevel(self.root);win.title("Physical validation — plan only (DISARMED)");win.geometry("800x600");runner=PhysicalValidationRunner("configs/physical_validation_development.json")
        ttk.Label(win,text="Frozen experiment schedule. This viewer cannot arm or command hardware.",foreground="#b91c1c").pack(anchor="w",padx=10,pady=8)
        box=tk.Text(win,wrap="none");box.pack(fill="both",expand=True,padx=10);next_trial=runner.next_trial();box.insert("end",f"Config fingerprint: {runner.fingerprint}\nCompleted: {len(runner.completed())}\nNext: {next_trial}\n\nORDER | CONDITION | CONTROLLER | REPETITION\n")
        for order,row in enumerate(runner.schedule()):box.insert("end",f"{order:02d} | {row['condition_id']} | {row['controller_id']} | {row['trial']}\n")
        box.configure(state="disabled")
        def report():
            try:self.show_output("Report written: "+str(generate_report(runner.root)))
            except FileNotFoundError:self.show_output("No physical trial records yet. The report remains unavailable until supervised raw telemetry has been recorded.")
        ttk.Button(win,text="Generate report from recorded trials",command=report).pack(pady=6)
    def show_output(self,text):self.ollama_output.configure(state="normal");self.ollama_output.delete("1.0","end");self.ollama_output.insert("1.0",text);self.ollama_output.configure(state="disabled")
    def ask_ollama(self):
        self.show_output("Contacting local Ollama…")
        prompt=self.prompt.get("1.0","end");model_name=self.ollama_model.get();model_path=self.model_path.get();url=self.ollama_url.get()
        def worker():
            base=RewardSpec();raw=OllamaAdvisor(model_name,url).propose(prompt+"\nAllowed numeric JSON keys: "+", ".join(base.__dataclass_fields__))
            if "error" in raw:
                self.root.after(0,lambda:self.show_output("Local Ollama is unavailable; reward unchanged.\n\n"+raw["error"]));return
            try:
                spec=RewardSpec(**{k:float(raw[k]) for k in base.__dataclass_fields__}).validate();self.engine.reward=spec;text="Validated reward applied to simulation preview. Ollama cannot command motors.\n\nOllama response / selected reward:\n"+str(raw)+"\n\nPPO model: "+(model_path or "none selected")+"\nThe 3D view and reward trace now show this candidate against the selected policy. PPO retraining is separate."
            except Exception as exc:text="Ollama response was rejected; current reward unchanged.\n"+str(raw)+"\nValidation: "+str(exc)
            self.root.after(0,lambda:self.show_output(text))
        threading.Thread(target=worker,daemon=True).start()
    def project(self,p):
        yaw=math.radians(self.yaw.get());el=math.radians(self.view_pitch.get());x,y,z=p;x,z=x*math.cos(yaw)-z*math.sin(yaw),x*math.sin(yaw)+z*math.cos(yaw);y,z=y*math.cos(el)-z*math.sin(el),y*math.sin(el)+z*math.cos(el);return 325+x*120,255-y*120+z*30
    def line3(self,a,b,color="#334155",width=2):self.canvas.create_line(*self.project(a),*self.project(b),fill=color,width=width)
    def draw_robot(self):
        c=self.canvas;c.delete("all");c.create_text(10,10,anchor="nw",text="Metal chassis • paired wheels • upright STM32/IMU stack • expansion deck",fill="#334155");s=self.engine.latest;pitch=s.pitch;body=lambda x,y,z:(x,y*math.cos(pitch)-z*math.sin(pitch),y*math.sin(pitch)+z*math.cos(pitch))
        for x in (-.48,.48):c.create_oval(*self.project((x,-.26,-.18)),*self.project((x+.12,.26,.18)),outline="#111",width=5)
        corners=[(-.42,-.18,-.12),(.42,-.18,-.12),(.42,-.18,.12),(-.42,-.18,.12)]
        for a,b in zip(corners,corners[1:]+corners[:1]):self.line3(a,b,"#64748b",5)
        upright=[body(x,y,z) for x,y,z in ((-.25,0,0),(.25,0,0),(.25,.78,0),(-.25,.78,0))]
        for a,b in zip(upright,upright[1:]+upright[:1]):self.line3(a,b,"#0f766e",7)
        self.line3(body(-.18,.5,-.14),body(.18,.5,-.14),"#dc2626",6);self.line3(body(-.18,.5,.14),body(.18,.5,.14),"#dc2626",6)
    def draw_graph(self):
        g=self.graph;g.delete("all");rows=self.engine.telemetry.rows[-240:];g.create_text(5,5,anchor="nw",text="P red, I green, D blue, reward violet")
        for field,color in (("P","#dc2626"),("I","#16a34a"),("D","#2563eb"),("reward","#7c3aed")):
            vals=[r.get(field,0.) for r in rows]
            if len(vals)>1:
                span=max(max(map(abs,vals)),.01);pts=[]
                for i,v in enumerate(vals):pts.extend((i*650/(len(vals)-1),85-v*70/span))
                g.create_line(*pts,fill=color)
    def refresh(self):
        self.draw_robot();self.draw_graph();s=self.engine.latest;last=self.engine.telemetry.rows[-1] if self.engine.telemetry.rows else {}
        self.info.set(f"pitch {s.pitch:+.3f} rad | rate {s.pitch_rate:+.3f} | accel {s.pitch_accel:+.3f}\nposition {s.position:+.3f} | velocity {s.velocity:+.3f} | wheels {s.left_speed:+.2f}/{s.right_speed:+.2f}\nbattery {s.battery:.2f} V | disturbance {s.disturbance:+.2f} | safety {self.engine.safety.status.reason}\nPID: P={last.get('P',0):+.3f} I={last.get('I',0):+.3f} D={last.get('D',0):+.3f} | action={last.get('action',0):+.3f} | reward={last.get('reward',0):+.3f}")
        self.root.after(80,self.refresh)
