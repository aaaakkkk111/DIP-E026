"""Deterministic proposal validation, safety gates, acceptance and rollback."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .firmware_profile import DEFAULT_PID, PID_KEYS, STAGE_KEYS, PIDValues
from .metrics import PerformanceMetrics, score_metrics
from .schemas import LLMProposal


PID_BOUNDS = {"AP":(20,200),"AD":(5,100),"VP":(10,120),"VI":(1,80),"TP":(1,80),"TD":(0,60)}


@dataclass
class HarnessConfig:
    max_relative_change: float = 0.12
    max_absolute_change: float = 5.0
    bootstrap_max_relative_change: float = 0.60
    bootstrap_max_absolute_change: float = 40.0
    bootstrap_min_change_allowance: float = 10.0
    maximum_iterations: int = 60
    api_budget: int = 60
    no_improvement_limit: int = 12
    minimum_score_improvement: float = 1.0
    maximum_telemetry_loss: float = 0.20
    maximum_staleness_s: float = 0.30
    maximum_saturation_ratio: float = 0.50
    automatic_unlock_manual_successes: int = 3


@dataclass(frozen=True)
class CandidateValidation:
    accepted: bool
    candidate: PIDValues | None
    original: PIDValues | None
    shrunk: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class Evaluation:
    decision: str
    baseline_score: float
    candidate_score: float
    reasons: tuple[str, ...]


class SafetyHarness:
    def __init__(self, config: HarnessConfig | None = None, state_path: Path | None = None) -> None:
        self.config = config or HarnessConfig(); self.state_path = state_path
        self.champion = DEFAULT_PID; self.manual_successes = 0; self.iterations = 0
        self.api_calls = 0; self.no_improvement = 0
        if state_path and state_path.exists(): self._load_state()

    def _load_state(self) -> None:
        data = json.loads(self.state_path.read_text(encoding="utf-8"))
        self.champion = PIDValues.from_mapping(data["champion"])
        self.manual_successes = int(data.get("manual_successes",0))

    def save_state(self) -> None:
        if not self.state_path: return
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.state_path.with_suffix(".tmp")
        temp.write_text(json.dumps({"champion":self.champion.as_dict(),"manual_successes":self.manual_successes},indent=2), encoding="utf-8")
        os.replace(temp,self.state_path)

    @property
    def automatic_unlocked(self) -> bool:
        return self.manual_successes >= self.config.automatic_unlock_manual_successes

    def validate_candidate(self, proposal: LLMProposal, stage: str, *, bootstrap: bool = False) -> CandidateValidation:
        reasons: list[str] = []
        if proposal.decision != "propose": return CandidateValidation(False,None,proposal.candidate,False,(f"LLM decision is {proposal.decision}",))
        if proposal.stage != stage: return CandidateValidation(False,None,proposal.candidate,False,("proposal stage mismatch",))
        allowed = set(STAGE_KEYS[stage]); champion = self.champion.as_dict(); raw = proposal.candidate.as_dict()
        for key in PID_KEYS:
            low,high = PID_BOUNDS[key]
            if not low <= raw[key] <= high: reasons.append(f"{key} outside [{low},{high}]")
            if key not in allowed and abs(raw[key]-champion[key]) > 0.005: reasons.append(f"{key} changed outside {stage} stage")
        if reasons: return CandidateValidation(False,None,proposal.candidate,False,tuple(reasons))
        processed = dict(raw); shrunk = False
        for key in allowed:
            delta = raw[key]-champion[key]
            if bootstrap:
                limit=min(self.config.bootstrap_max_absolute_change,
                    max(self.config.bootstrap_min_change_allowance,
                        abs(champion[key])*self.config.bootstrap_max_relative_change))
            else:
                limit=min(self.config.max_absolute_change,
                    max(0.5,abs(champion[key])*self.config.max_relative_change))
            if abs(delta) > limit:
                processed[key] = champion[key] + (limit if delta>0 else -limit); shrunk = True
                mode="bootstrap" if bootstrap else "normal"
                reasons.append(f"{key} change shrunk to deterministic {mode} limit {limit:.3g}")
        # The official ASCII protocol carries exactly two decimal places.
        # Quantize before transmission so readback and persisted champion agree.
        processed = {key:float(f"{value:.2f}") for key,value in processed.items()}
        return CandidateValidation(True,PIDValues.from_mapping(processed),proposal.candidate,shrunk,tuple(reasons))

    def data_is_complete(self, metrics: PerformanceMetrics) -> tuple[bool,tuple[str,...]]:
        reasons=[]
        if metrics.sample_count < 5: reasons.append("insufficient telemetry samples")
        if metrics.telemetry_loss_ratio > self.config.maximum_telemetry_loss: reasons.append("telemetry loss limit exceeded")
        if metrics.telemetry_staleness_s > self.config.maximum_staleness_s: reasons.append("telemetry timeout/staleness")
        return not reasons,tuple(reasons)

    def hard_safety(self, metrics: PerformanceMetrics) -> tuple[bool,tuple[str,...]]:
        reasons=[]; complete,data_reasons=self.data_is_complete(metrics); reasons.extend(data_reasons)
        if metrics.fall_count: reasons.append("fall detected")
        if metrics.protection_trigger_count: reasons.append("firmware protection triggered")
        if metrics.pwm_saturation_ratio > self.config.maximum_saturation_ratio: reasons.append("PWM saturation ratio exceeded")
        return complete and not reasons,tuple(reasons)

    def evaluate(self, baseline: PerformanceMetrics, candidate: PerformanceMetrics, stage: str) -> Evaluation:
        base_score=score_metrics(baseline,stage); candidate_score=score_metrics(candidate,stage)
        safe,reasons=self.hard_safety(candidate)
        if not safe: return Evaluation("rollback",base_score,candidate_score,reasons)
        improvement=candidate_score-base_score
        if improvement >= self.config.minimum_score_improvement:
            return Evaluation("accept",base_score,candidate_score,(f"score improved by {improvement:.3f}",))
        if improvement >= -3.0:
            return Evaluation("shrink_retry",base_score,candidate_score,(f"mild score change {improvement:.3f}",))
        return Evaluation("rollback",base_score,candidate_score,(f"score degraded by {abs(improvement):.3f}",))

    def set_champion(self, pid: PIDValues, *, manual_success: bool = False) -> None:
        self.champion=pid
        if manual_success: self.manual_successes += 1
        self.no_improvement=0; self.save_state()

    def reset_training_state(self, initial_pid: PIDValues) -> None:
        self.champion=initial_pid
        self.manual_successes=0
        self.iterations=0
        self.api_calls=0
        self.no_improvement=0
        self.save_state()

    def can_continue(self) -> tuple[bool,str]:
        if self.iterations >= self.config.maximum_iterations: return False,"maximum iterations reached"
        if self.api_calls >= self.config.api_budget: return False,"API budget reached"
        if self.no_improvement >= self.config.no_improvement_limit: return False,"no-improvement stop reached"
        return True,"ok"

    def snapshot(self) -> dict[str,Any]:
        return {"config":asdict(self.config),"champion":self.champion.as_dict(),"manual_successes":self.manual_successes,
                "iterations":self.iterations,"api_calls":self.api_calls,"no_improvement":self.no_improvement}
