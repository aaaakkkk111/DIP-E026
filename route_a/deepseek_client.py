"""DeepSeek JSON-output client with bounded retry and a deterministic mock."""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from .firmware_profile import PIDValues
from .schemas import LLMProposal, SchemaValidationError, parse_proposal_json


ROOT = Path(__file__).resolve().parents[1]
PROMPT_PATH = Path(__file__).resolve().parent / "prompts" / "system_v1.txt"


class LLMError(RuntimeError): pass


@dataclass(frozen=True)
class LLMResult:
    raw_json: str
    proposal: LLMProposal
    model: str
    usage: dict[str, Any]
    provider: str


class LLMClient(Protocol):
    def propose(self, summary: dict[str,Any]) -> LLMResult: ...


def load_env(path: Path | None = None) -> dict[str,str]:
    values = dict(os.environ); path = path or ROOT/"route_a.env"
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            line=line.strip()
            if line and not line.startswith("#") and "=" in line:
                key,value=line.split("=",1); values.setdefault(key.strip(),value.strip())
    return values


class DeepSeekClient:
    RETRYABLE = {429,500,503}; NON_RETRYABLE = {400,401,402,422}

    def __init__(self, *, env_path: Path | None = None, timeout_s: float = 30.0,
                 max_retries: int = 2) -> None:
        env=load_env(env_path); self.api_key=env.get("DEEPSEEK_API_KEY","")
        self.base_url=env.get("DEEPSEEK_BASE_URL","https://api.deepseek.com").rstrip("/")
        self.model=env.get("DEEPSEEK_MODEL","deepseek-flash")
        self.timeout_s=timeout_s; self.max_retries=max_retries
        if not self.api_key: raise LLMError("DEEPSEEK_API_KEY is missing; select MockLLM or manual candidate mode")

    def propose(self, summary: dict[str,Any]) -> LLMResult:
        retry_feedback=""
        for attempt in range(self.max_retries+1):
            user_content=json.dumps(summary,ensure_ascii=False,allow_nan=False)
            if retry_feedback:
                user_content += ("\n\nYour previous response was rejected by the deterministic parser: "
                    + retry_feedback + ". Return a corrected, non-empty JSON object with exactly every field shown "
                    "in the system-prompt example. Do not omit fields and do not add fields.")
            body=json.dumps({"model":self.model,"messages":[
                {"role":"system","content":PROMPT_PATH.read_text(encoding="utf-8")},
                {"role":"user","content":user_content}],
                "response_format":{"type":"json_object"},"thinking":{"type":"disabled"},
                "temperature":0.2,"max_tokens":1200,"stream":False}).encode("utf-8")
            request=urllib.request.Request(self.base_url+"/chat/completions",data=body,
                headers={"Authorization":"Bearer "+self.api_key,"Content-Type":"application/json"},method="POST")
            try:
                with urllib.request.urlopen(request,timeout=self.timeout_s) as response:
                    envelope=json.loads(response.read().decode("utf-8"))
                choice=envelope["choices"][0]; raw=choice["message"].get("content")
                if not isinstance(raw,str) or not raw.strip():
                    finish_reason=choice.get("finish_reason","unknown")
                    if attempt < self.max_retries:
                        retry_feedback=f"empty content (finish_reason={finish_reason})"
                        time.sleep(0.5*(2**attempt)); continue
                    raise LLMError(f"DeepSeek returned empty content after {attempt+1} attempts "
                                   f"(model={envelope.get('model',self.model)}, finish_reason={finish_reason})")
                try:
                    proposal=parse_proposal_json(raw)
                except SchemaValidationError as exc:
                    if attempt < self.max_retries:
                        retry_feedback=str(exc); time.sleep(0.5*(2**attempt)); continue
                    raise LLMError(f"DeepSeek JSON failed strict schema validation after {attempt+1} attempts: {exc}") from exc
                bootstrap=summary.get("bootstrap_recovery",{})
                if bootstrap.get("required") is True and proposal.decision != "propose":
                    if attempt < self.max_retries:
                        retry_feedback=("bootstrap recovery is required and telemetry is complete; "
                            "decision must be propose with a decisive balance-stage AP/AD rescue candidate")
                        time.sleep(0.5*(2**attempt)); continue
                    raise LLMError("DeepSeek refused to propose a bootstrap recovery candidate after bounded retries")
                return LLMResult(raw,proposal,self.model,envelope.get("usage",{}),"deepseek")
            except urllib.error.HTTPError as exc:
                status=exc.code
                guidance={400:"bad request or unavailable model; check DEEPSEEK_MODEL and JSON Output support",
                          401:"invalid/missing API key",402:"insufficient API balance",422:"request validation failed"}
                if status in self.NON_RETRYABLE: raise LLMError(f"DeepSeek HTTP {status}: {guidance[status]}; request was not retried") from exc
                if status not in self.RETRYABLE or attempt>=self.max_retries: raise LLMError(f"DeepSeek HTTP {status}") from exc
            except (urllib.error.URLError,TimeoutError) as exc:
                if attempt>=self.max_retries: raise LLMError(f"DeepSeek connection/timeout error: {exc}") from exc
            except (KeyError,IndexError,json.JSONDecodeError) as exc:
                raise LLMError(f"invalid DeepSeek response envelope: {exc}") from exc
            time.sleep(0.5*(2**attempt))
        raise LLMError("unreachable")


class MockLLM:
    """Deterministic offline adviser; it does not guarantee acceptance."""
    def propose(self, summary: dict[str,Any]) -> LLMResult:
        stage=summary["stage"]; current=dict(summary["current_pid"]); metrics=summary["metrics"]
        if stage=="balance":
            if metrics["pitch_peak_deg"]>8: current["AP"]*=1.04
            if metrics["pitch_rate_rms_deg_s"]>20: current["AD"]*=1.04
            test="balance_recovery"
        elif stage=="velocity":
            current["VP"]*=1.03; current["VI"]*=1.02; test="velocity_step"
        else:
            current["TP"]*=1.03; current["TD"]*=1.02; test="turn_step"
        payload={"schema_version":1,"decision":"propose","stage":stage,
                 "candidate":{k:round(v,4) for k,v in current.items()},
                 "expected_effect":"Small deterministic stage-local adjustment from summarized telemetry.",
                 "requested_test":test,"confidence":0.55}
        raw=json.dumps(payload,separators=(",",":")); return LLMResult(raw,parse_proposal_json(raw),"mock-v1",{},"mock")


class ManualCandidateLLM:
    def __init__(self, proposal_json: str) -> None: self.proposal_json=proposal_json
    def propose(self, summary: dict[str,Any]) -> LLMResult:
        return LLMResult(self.proposal_json,parse_proposal_json(self.proposal_json),"manual",{},"manual")
