"""Route A state machine. LLM proposes; deterministic code decides."""

from __future__ import annotations

import json
import random
import threading
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from .deepseek_client import LLMClient, LLMError, LLMResult, MockLLM
from .experiment_runner import ExperimentAborted, ExperimentRunner, ProtocolTimeout, TrialResult
from .firmware_profile import EnvironmentConfig, FIRMWARE_CONSTANTS, PIDValues
from .safety_harness import CandidateValidation, Evaluation, HarnessConfig, PID_BOUNDS, SafetyHarness
from .transport import MuJoCoTransport, Transport


class RunMode(str,Enum): MANUAL="manual-review"; AUTOMATIC="automatic"
class CycleState(str,Enum): IDLE="idle"; BASELINE="baseline"; WAITING_LLM="waiting-llm"; AWAITING_REVIEW="awaiting-review"; CANDIDATE="candidate-test"; PAUSED="paused"; COMPLETE="complete"; EMERGENCY="emergency-stop"; ERROR="error"


@dataclass
class PendingCycle:
    directory: Path; baseline: TrialResult; llm: LLMResult; validation: CandidateValidation
    bootstrap_required: bool = False


class RouteAOrchestrator:
    STAGE_COMMAND={"balance":"stop","velocity":"forward","turn":"right"}

    def __init__(self, transport: Transport | None = None, llm: LLMClient | None = None,
                 harness: SafetyHarness | None = None) -> None:
        self.transport=transport or MuJoCoTransport(); self.llm=llm or MockLLM()
        state_path=Path(__file__).resolve().parent/"state"/"champion.json"
        if harness is None:
            config_path=Path(__file__).resolve().parent/"config"/"default.json"
            raw=json.loads(config_path.read_text(encoding="utf-8")) if config_path.exists() else {}
            allowed=HarnessConfig.__dataclass_fields__.keys()
            harness=SafetyHarness(HarnessConfig(**{key:value for key,value in raw.items() if key in allowed}),state_path=state_path)
        self.harness=harness; self.runner=ExperimentRunner(self.transport)
        self.mode=RunMode.MANUAL; self.stage="balance"; self.state=CycleState.IDLE
        self.pending:PendingCycle|None=None; self.history:list[dict[str,Any]]=[]
        self.last_trial:TrialResult|None=None; self.last_evaluation:Evaluation|None=None
        self.pause_requested=False; self.abort_event=threading.Event(); self.lock=threading.RLock(); self.last_error=""

    def set_mode(self, mode: RunMode) -> None:
        mode=RunMode(mode)
        if mode is RunMode.AUTOMATIC and not self.harness.automatic_unlocked:
            raise RuntimeError(f"automatic mode requires {self.harness.config.automatic_unlock_manual_successes} accepted manual trials")
        self.mode=mode

    def begin_cycle(self, environment: EnvironmentConfig, duration_s: float = 4.0) -> PendingCycle:
        with self.lock:
            if self.abort_event.is_set(): raise RuntimeError("system is paused or emergency-stopped")
            ok,reason=self.harness.can_continue()
            if not ok: raise RuntimeError(reason)
            self.state=CycleState.BASELINE; self.harness.iterations += 1
            directory=self.runner.create_round_directory(self.harness.iterations,self.stage)
            try:
                self.runner.protocol.write_pid(self.harness.champion)
                baseline=self.runner.run_trial(name="baseline",stage=self.stage,command=self.STAGE_COMMAND[self.stage],
                    duration_s=duration_s,environment=environment,seed=environment.seed,output_dir=directory/"baseline",
                    abort_event=self.abort_event)
                self.last_trial=baseline; self.last_evaluation=None
                complete,reasons=self.harness.data_is_complete(baseline.metrics)
                if not complete: raise RuntimeError("baseline data rejected: "+"; ".join(reasons))
                summary=self.runner.build_llm_summary(baseline,stage=self.stage,history=self.history,
                    harness_limits=self.harness.snapshot()["config"])
                bootstrap_required=(self.stage == "balance" and (
                    baseline.metrics.fall_count > 0 or baseline.metrics.protection_trigger_count > 0
                    or baseline.metrics.pitch_peak_deg >= 40.0))
                summary["bootstrap_recovery"]={
                    "required":bootstrap_required,
                    "reason":"baseline is outside the recoverable balance envelope" if bootstrap_required else "normal tuning",
                    "acceptance_rule":"candidate must eliminate hard safety events; LLM never accepts its own proposal",
                    "allowed_stage_parameters":["AP","AD"],
                    "max_relative_change":self.harness.config.bootstrap_max_relative_change if bootstrap_required else self.harness.config.max_relative_change,
                    "max_absolute_change":self.harness.config.bootstrap_max_absolute_change if bootstrap_required else self.harness.config.max_absolute_change}
                (directory/"llm_input.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
                self.state=CycleState.WAITING_LLM; self.harness.api_calls += 1
                llm_result=self.llm.propose(summary)
                (directory/"llm_raw_response.json").write_text(llm_result.raw_json,encoding="utf-8")
                validation=self.harness.validate_candidate(llm_result.proposal,self.stage,bootstrap=bootstrap_required)
                (directory/"schema_and_harness.json").write_text(json.dumps({"schema_valid":True,
                    "accepted_for_review":validation.accepted,"processed_candidate":validation.candidate.as_dict() if validation.candidate else None,
                    "shrunk":validation.shrunk,"reasons":validation.reasons},indent=2),encoding="utf-8")
                self.pending=PendingCycle(directory,baseline,llm_result,validation,bootstrap_required)
                self.state=CycleState.AWAITING_REVIEW if self.mode is RunMode.MANUAL else CycleState.CANDIDATE
                return self.pending
            except Exception as exc:
                self.last_error=str(exc)
                if not isinstance(exc,ExperimentAborted): self.state=CycleState.ERROR
                try: self.runner.protocol.write_pid(self.harness.champion)
                except Exception: pass
                raise

    def approve(self, duration_s: float = 4.0) -> Evaluation:
        with self.lock:
            if not self.pending or not self.pending.validation.accepted or not self.pending.validation.candidate:
                raise RuntimeError("no Harness-approved candidate is waiting")
            candidate=self.pending.validation.candidate; self.state=CycleState.CANDIDATE
            try:
                actual,frames=self.runner.protocol.write_pid(candidate)
                candidate_trial=self.runner.run_trial(name="candidate",stage=self.stage,command=self.STAGE_COMMAND[self.stage],
                    duration_s=duration_s,environment=self.pending.baseline.environment,seed=self.pending.baseline.seed,
                    output_dir=self.pending.directory/"candidate",abort_event=self.abort_event)
                evaluation=self.harness.evaluate(self.pending.baseline.metrics,candidate_trial.metrics,self.stage)
                final_trial=candidate_trial; final_pid=actual
                if evaluation.decision=="shrink_retry":
                    shrunk=self.harness.champion.interpolate(candidate,0.5)
                    self.runner.protocol.write_pid(shrunk)
                    final_trial=self.runner.run_trial(name="shrink_retry",stage=self.stage,command=self.STAGE_COMMAND[self.stage],
                        duration_s=duration_s,environment=self.pending.baseline.environment,seed=self.pending.baseline.seed,
                        output_dir=self.pending.directory/"shrink_retry",abort_event=self.abort_event)
                    evaluation=self.harness.evaluate(self.pending.baseline.metrics,final_trial.metrics,self.stage)
                    final_pid=shrunk
                    if evaluation.decision=="shrink_retry":
                        evaluation=Evaluation("rollback",evaluation.baseline_score,evaluation.candidate_score,("shrunken candidate did not meet acceptance threshold",))
                if evaluation.decision=="accept":
                    self.harness.set_champion(final_pid,manual_success=self.mode is RunMode.MANUAL)
                else:
                    self.runner.protocol.write_pid(self.harness.champion); self.harness.no_improvement += 1
                record={"stage":self.stage,"champion":self.harness.champion.as_dict(),"candidate":candidate.as_dict(),
                        "baseline_score":evaluation.baseline_score,"candidate_score":evaluation.candidate_score,
                        "decision":evaluation.decision,"reasons":evaluation.reasons,"provider":self.pending.llm.provider,
                        "model":self.pending.llm.model,"usage":self.pending.llm.usage,"prompt_version":"route-a-pid-v4"}
                self.history.append(record); self.runner.save_round_context(self.pending.directory,**record,
                    configuration=self.pending.baseline.environment.as_dict(),random_seed=self.pending.baseline.seed,
                    actual_readback=final_pid.as_dict(),raw_protocol_frames=frames)
                self.last_trial=final_trial; self.last_evaluation=evaluation
                self.pending=None; self.state=CycleState.COMPLETE; return evaluation
            except Exception as exc:
                self.last_error=str(exc)
                if not isinstance(exc,ExperimentAborted): self.state=CycleState.ERROR
                try:self.runner.protocol.write_pid(self.harness.champion)
                except Exception:pass
                raise

    def reject(self, reason: str = "user rejected LLM proposal") -> None:
        with self.lock:
            if not self.pending: raise RuntimeError("no candidate is waiting")
            self.runner.protocol.write_pid(self.harness.champion)
            record={"stage":self.stage,"decision":"user-reject","reason":reason,"champion":self.harness.champion.as_dict()}
            self.history.append(record); self.runner.save_round_context(self.pending.directory,**record)
            self.pending=None; self.state=CycleState.COMPLETE

    def reset_training(self, environment: EnvironmentConfig, *, random_seed: int | None = None) -> PIDValues:
        with self.lock:
            rng=random.Random(random_seed) if random_seed is not None else random.SystemRandom()
            proposed=PIDValues.from_mapping({
                key:float(f"{rng.uniform(*PID_BOUNDS[key]):.2f}") for key in PID_BOUNDS})
            self.runner.protocol.motion("stop")
            actual,_=self.runner.protocol.write_pid(proposed)
            if isinstance(self.transport,MuJoCoTransport):
                self.transport.reset_scenario(environment,seed=environment.seed,
                    initial_pitch_deg=FIRMWARE_CONSTANTS["mid_angle_deg"]+0.2)
            self.harness.reset_training_state(actual)
            self.pending=None; self.history.clear(); self.last_trial=None; self.last_evaluation=None
            self.pause_requested=False; self.abort_event.clear(); self.last_error=""
            self.mode=RunMode.MANUAL; self.stage="balance"; self.state=CycleState.IDLE
            return actual

    def reset_environment(self) -> None:
        with self.lock:
            if not isinstance(self.transport,MuJoCoTransport):
                raise RuntimeError("environment reset is available only for MuJoCoTransport")
            environment=self.transport.environment
            self.transport.reset_scenario(seed=environment.seed,
                initial_pitch_deg=FIRMWARE_CONSTANTS["mid_angle_deg"]+0.2)

    def rollback(self) -> PIDValues:
        actual,_=self.runner.protocol.write_pid(self.harness.champion); self.pending=None; self.state=CycleState.IDLE; return actual

    def pause(self) -> None: self.pause_requested=True; self.abort_event.set(); self.state=CycleState.PAUSED
    def resume(self) -> None: self.pause_requested=False; self.abort_event.clear(); self.state=CycleState.IDLE
    def emergency_stop(self) -> None:
        self.abort_event.set()
        if hasattr(self.transport,"emergency_stop"): self.transport.emergency_stop()
        self.runner.protocol.motion("stop"); self.state=CycleState.EMERGENCY

    def run_automatic(self, environment: EnvironmentConfig, max_rounds: int = 3, duration_s: float = 4.0) -> list[Evaluation]:
        self.set_mode(RunMode.AUTOMATIC); results=[]
        for _ in range(max_rounds):
            if self.pause_requested: break
            pending=self.begin_cycle(environment,duration_s)
            if not pending.validation.accepted:
                self.reject("Harness rejected automatic proposal"); break
            results.append(self.approve(duration_s))
            if results[-1].decision != "accept": break
        return results

    def close(self) -> None: self.transport.close()
