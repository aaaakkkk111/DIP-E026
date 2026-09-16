"""Short repeatable trials, protocol client, records and plots."""

from __future__ import annotations

import csv
import json
import subprocess
import threading
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .firmware_profile import EnvironmentConfig, FIRMWARE_CONSTANTS, PIDValues
from .metrics import PerformanceMetrics, compute_metrics, score_metrics
from .plant import OracleSample
from .protocol import FrameStreamParser, build_motion_frame, build_pid_query_frame, build_pid_update_frame, parse_pid_report
from .telemetry import FastTelemetry, TelemetryDecoder
from .transport import MuJoCoTransport, Transport


ROOT = Path(__file__).resolve().parents[1]
RUNS_DIR = Path(__file__).resolve().parent / "runs"


class ProtocolTimeout(RuntimeError): pass
class ExperimentAborted(RuntimeError): pass


class ProtocolClient:
    """PC-side protocol client. It only sees Transport bytes."""
    def __init__(self, transport: Transport) -> None: self.transport=transport

    def _exchange(self, request: bytes, timeout_s: float = 1.0) -> list[bytes]:
        parser=FrameStreamParser(); frames=[]; self.transport.write(request); elapsed=0.0
        while elapsed < timeout_s:
            self.transport.advance(0.005); elapsed += 0.005
            frames.extend(parser.feed(self.transport.read()))
            if frames and not any(frame.startswith(b"$E1") for frame in frames[-1:]):
                # Keep draining because PID query deliberately answers twice.
                if elapsed >= 0.30: break
        return frames

    def query_pid(self) -> tuple[PIDValues,list[str]]:
        frames=self._exchange(build_pid_query_frame())
        reports=[value for frame in frames if (value:=parse_pid_report(frame)) is not None]
        if len(reports)<2: raise ProtocolTimeout(f"PID query expected duplicate replies, got {len(reports)}")
        if reports[0] != reports[1]: raise ProtocolTimeout("duplicate PID query replies disagree")
        return reports[-1],[frame.decode("ascii",errors="replace") for frame in frames]

    def write_pid(self, pid: PIDValues, stage: str | None = None) -> tuple[PIDValues,list[str]]:
        protocol_pid=PIDValues.from_mapping({key:float(f"{value:.2f}") for key,value in pid.as_dict().items()})
        frame=build_pid_update_frame(protocol_pid,stage); frames=self._exchange(frame)
        expected=1 if stage else 3; ack=sum(item==b"$OK#" for item in frames)
        if ack != expected: raise ProtocolTimeout(f"PID update expected {expected} ACK, got {ack}")
        actual,query_frames=self.query_pid()
        if actual != protocol_pid:
            raise ProtocolTimeout(f"PID readback mismatch: requested={protocol_pid.as_dict()} actual={actual.as_dict()}")
        return actual,[item.decode("ascii",errors="replace") for item in frames]+query_frames

    def motion(self, command: str) -> None: self.transport.write(build_motion_frame(command))


@dataclass
class TrialResult:
    name: str
    stage: str
    command: str
    seed: int
    pid: PIDValues
    environment: EnvironmentConfig
    telemetry: list[FastTelemetry]
    oracle: list[OracleSample]
    raw_frames: list[str]
    data_quality: dict[str,Any]
    metrics: PerformanceMetrics
    score: float
    directory: Path | None = None

    def summary(self) -> dict[str,Any]:
        return {"name":self.name,"stage":self.stage,"command":self.command,"seed":self.seed,
                "pid":self.pid.as_dict(),"environment":self.environment.as_dict(),
                "data_quality":self.data_quality,"metrics":self.metrics.as_dict(),"score":self.score}


class ExperimentRunner:
    def __init__(self, transport: Transport, runs_dir: Path = RUNS_DIR) -> None:
        self.transport=transport; self.protocol=ProtocolClient(transport); self.runs_dir=runs_dir

    def run_trial(self, *, name: str, stage: str, command: str, duration_s: float,
                  environment: EnvironmentConfig, seed: int, output_dir: Path | None = None,
                  apply_disturbance: bool | None = None,
                  abort_event: threading.Event | None = None) -> TrialResult:
        if duration_s < 1.0: raise ValueError("trial duration must be at least 1 s")
        if not isinstance(self.transport, MuJoCoTransport):
            raise RuntimeError("real SerialTransport trial scheduling requires a hardware capture adapter")
        current,_=self.protocol.query_pid()
        disturbance_enabled=(stage == "balance") if apply_disturbance is None else apply_disturbance
        initial_pitch=FIRMWARE_CONSTANTS["mid_angle_deg"]+0.2
        self.transport.reset_scenario(environment,seed=seed,initial_pitch_deg=initial_pitch)
        self.protocol.motion("stop")
        self.transport.advance(0.80); self.transport.read()
        decoder=TelemetryDecoder(); self.protocol.motion(command)
        oracle=[]; elapsed=0.0; pushed=False; turn_phase_switched=False
        while elapsed < duration_s:
            if abort_event is not None and abort_event.is_set():
                self.protocol.motion("stop")
                raise ExperimentAborted("trial interrupted by Pause or Emergency Stop")
            oracle.extend(self.transport.advance(0.01)); elapsed += 0.01
            decoder.feed(self.transport.read())
            if stage == "turn" and not turn_phase_switched and elapsed >= duration_s*0.5:
                self.protocol.motion("forward"); turn_phase_switched=True
            if disturbance_enabled and not pushed and elapsed >= min(1.0,duration_s*0.35):
                self.transport.apply_push("forward",environment.push_force_n); pushed=True
        self.protocol.motion("stop"); self.transport.advance(0.10); decoder.feed(self.transport.read())
        expected=max(1,int(duration_s*10)); quality=decoder.quality(expected)
        target_speed=0.18 if stage=="velocity" and command=="forward" else (-0.18 if stage=="velocity" and command=="backward" else 0.0)
        target_yaw=45.0 if stage=="turn" and command in ("right","pivot_right") else (-45.0 if stage=="turn" and command in ("left","pivot_left") else 0.0)
        metrics=compute_metrics(decoder.fast,quality,target_speed_m_s=target_speed,target_yaw_deg_s=target_yaw,
                                targets_from_command=stage == "turn")
        result=TrialResult(name,stage,command,seed,current,environment,list(decoder.fast),oracle,
                           decoder.raw_frames,quality,metrics,score_metrics(metrics,stage),output_dir)
        if output_dir: self.save_trial(result,output_dir)
        return result

    def create_round_directory(self, index: int, stage: str) -> Path:
        stamp=datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path=self.runs_dir/f"{stamp}_round_{index:03d}_{stage}"; path.mkdir(parents=True,exist_ok=False); return path

    @staticmethod
    def _write_csv(path: Path, rows: list[dict[str,Any]]) -> None:
        if not rows: path.write_text("",encoding="utf-8"); return
        with path.open("w",newline="",encoding="utf-8") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)

    def save_trial(self, result: TrialResult, directory: Path) -> None:
        directory.mkdir(parents=True,exist_ok=True)
        self._write_csv(directory/"telemetry.csv",[item.as_dict() for item in result.telemetry])
        self._write_csv(directory/"oracle.csv",[item.as_dict() for item in result.oracle])
        (directory/"summary.json").write_text(json.dumps(result.summary(),indent=2,ensure_ascii=False,allow_nan=False),encoding="utf-8")
        (directory/"raw_frames.log").write_text("\n".join(result.raw_frames),encoding="utf-8")
        self._plot(result,directory/"curves.png")

    @staticmethod
    def _plot(result: TrialResult, path: Path) -> None:
        try:
            import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        except ImportError: return
        fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True)
        t=[item.tick_ms/1000 for item in result.telemetry]
        axes[0].plot(t,[item.pitch_deg for item in result.telemetry],label="deployable pitch")
        axes[0].plot([item.time_s for item in result.oracle],[item.pitch_deg for item in result.oracle],alpha=.45,label="Oracle pitch")
        axes[0].set_ylabel("Pitch (deg)"); axes[0].legend()
        axes[1].plot(t,[item.encoder_left for item in result.telemetry],label="encoder L")
        axes[1].plot(t,[item.encoder_right for item in result.telemetry],label="encoder R"); axes[1].set_ylabel("counts / 100 ms"); axes[1].legend()
        axes[2].plot(t,[item.pwm_left for item in result.telemetry],label="PWM L")
        axes[2].plot(t,[item.pwm_right for item in result.telemetry],label="PWM R"); axes[2].set_ylabel("PWM"); axes[2].legend()
        axes[3].plot([item.time_s for item in result.oracle],
                     [item.longitudinal_position_m for item in result.oracle],label="Oracle X (plot only)")
        axes[3].set_ylabel("Position (m)"); axes[3].set_xlabel("Physical time (s)"); axes[3].legend()
        fig.suptitle("Telemetry (controller input) and Oracle (plot only)"); fig.tight_layout(); fig.savefig(path,dpi=180); plt.close(fig)

    @staticmethod
    def git_commit() -> str:
        try: return subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True,stderr=subprocess.DEVNULL).strip()
        except Exception: return "unavailable"

    def build_llm_summary(self, trial: TrialResult, *, stage: str, history: list[dict[str,Any]],
                          harness_limits: dict[str,Any]) -> dict[str,Any]:
        duration_s=((trial.telemetry[-1].tick_ms-trial.telemetry[0].tick_ms+100)/1000.0
                    if trial.telemetry else 0.0)
        scenario_name={"balance":"balance_recovery","velocity":"velocity_step","turn":"turn_step"}[stage]
        disturbance_applied=stage == "balance"
        trial_scenario={"name":scenario_name,"completed":True,
            "initial_pitch_deg":FIRMWARE_CONSTANTS["mid_angle_deg"]+0.2,"warmup_s":0.80,
            "motion_command":trial.command,"duration_s":duration_s,"random_seed":trial.seed,
            "scheduled_disturbance":{"applied":disturbance_applied,
                "direction":"forward" if disturbance_applied else None,
                "onset_s":min(1.0,duration_s*0.35),"force_n":trial.environment.push_force_n,
                "duration_s":trial.environment.push_duration_s},
            "candidate_test_plan":"Repeat the identical scenario, environment and random seed; deterministic Harness compares scores and owns acceptance or rollback."}
        if stage == "turn":
            trial_scenario["phases"]=[
                {"fraction":[0.0,0.5],"command":"right","purpose":"identify TP turning response; firmware TD is disabled in this command"},
                {"fraction":[0.5,1.0],"command":"forward","purpose":"identify TD straight-line yaw damping; firmware turn target is zero"}]
        return {"schema_version":1,"current_pid":trial.pid.as_dict(),"stage":stage,
                "environment":trial.environment.as_dict(),"trial_command":trial.command,"trial_scenario":trial_scenario,
                "data_quality":trial.data_quality,"metrics":trial.metrics.as_dict(),
                "score":trial.score,"recent_history":history[-5:],"harness_limits":harness_limits,
                "notes":{"controller_command_semantics":"firmware Movement/Turn_Target; not m/s control",
                          "velocity_metric_reference":"0.18 m/s is calibration_required and is scoring-only",
                          "oracle_used":False,
                          "evidence_note":"trial_scenario is a completed controlled test, not a requested future test",
                          "stage_isolation":"balance uses a scheduled push; velocity uses a clean forward step; turn uses right then forward phases without an added push"}}

    def save_round_context(self,directory:Path,**items:Any)->None:
        metadata={"time_utc":datetime.now(timezone.utc).isoformat(),"git_commit":self.git_commit(),**items}
        (directory/"round.json").write_text(json.dumps(metadata,indent=2,ensure_ascii=False,default=lambda x:x.as_dict() if hasattr(x,"as_dict") else str(x)),encoding="utf-8")
