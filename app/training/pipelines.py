from __future__ import annotations
from dataclasses import asdict
from app.control.policies import PIDPolicy,PIDConfig
from app.reward.spec import RewardSpec
class PIDTuningPipeline:
    def __init__(self,evaluator,advisor=None):self.evaluator=evaluator;self.advisor=advisor;self.history=[]
    def run(self,initial,iterations=2):
        candidate=initial
        for i in range(iterations):
            metrics=self.evaluator(PIDPolicy(PIDConfig(**candidate))); self.history.append({"iteration":i,"candidate":candidate,"metrics":metrics})
            if self.advisor:
                proposal=self.advisor.propose(f"Tune PID from metrics {metrics}. keys kp ki kd")
                try:candidate=self.advisor.pid_candidate(proposal)
                except ValueError:pass
        return sorted(self.history,key=lambda h:h["metrics"].get("rms_angle",999))
class RewardSearchPipeline:
    def __init__(self,evaluator,advisor=None):self.evaluator=evaluator;self.advisor=advisor;self.history=[]
    def run(self,initial:RewardSpec,iterations=2):
        candidate=initial.validate()
        for i in range(iterations):
            nominal=self.evaluator(candidate,False); randomized=self.evaluator(candidate,True); score=(nominal["survival_time"]+randomized["survival_time"])/2-nominal["rms_angle"]-randomized["rms_angle"]
            self.history.append({"iteration":i,"lineage":i-1,"spec":asdict(candidate),"nominal":nominal,"randomized":randomized,"rank_score":score})
            if self.advisor:
                p=self.advisor.propose(f"Improve structured reward {asdict(candidate)}. Return same numeric keys.")
                try:candidate=RewardSpec(**{k:float(p[k]) for k in asdict(candidate)}).validate()
                except (KeyError,ValueError,TypeError):pass
        return sorted(self.history,key=lambda x:x["rank_score"],reverse=True)
