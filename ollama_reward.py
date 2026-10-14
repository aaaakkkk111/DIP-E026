"""Offboard Ollama reward-weight designer for PPO fine-tuning of the tzejun actor.

This module ONLY proposes the eight bounded reward WEIGHTS consumed by
deps/ollama_tzejun_env.py. It never runs inside the control loop, never edits
network weights, and never changes the immutable fall penalty (-25). PPO (in
finetune.py) is what actually changes actor parameters; a reward-weight change
alone does not alter a deployed actor.

JSON reliability pipeline (addresses the historical sum!=1 rejection that made
every past candidate fall back to default weights):

  1. Ollama /api/generate with a structured-output `format` schema.
  2. Parse cascade: direct json -> fenced-block strip -> balanced-brace
     extraction -> trailing-comma repair.
  3. Strict validation: exact keys, finite numbers, each weight in [LO, HI].
  4. Sum check with an EXPLICIT tolerance (SUM_ABS_TOL).
  5. Bounded repair retry: the exact validation error is sent back to the model.
  6. Documented renormalization transform (w_i / sum) applied ONLY when a
     proposal is otherwise valid but the sum is off; logged as a transformation
     with the raw proposal and the normalized result recorded SEPARATELY. This
     is never reported as raw LLM output.
  7. If the proposal cannot be made key-complete/finite/in-range, fall back to
     the previous safe weights and mark the iteration FALLBACK-ONLY (which does
     not count as an LLM contribution).

Everything (prompt, prompt hash, raw response, each repaired response, parse
path, validation errors, normalization transform, latency, final status) is
returned so finetune.py can persist it immutably.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import time
import urllib.request
from typing import Mapping

# Must match deps/ollama_tzejun_env.py REWARD_KEYS and DEFAULT_REWARD_WEIGHTS.
KEYS = ("angle", "rate", "position", "speed", "yaw", "effort", "slew", "survival")
DEFAULT_WEIGHTS = {
    "angle": 0.22, "rate": 0.08, "position": 0.12, "speed": 0.18,
    "yaw": 0.20, "effort": 0.05, "slew": 0.05, "survival": 0.10,
}
# Per-term proposal bounds the LLM is asked to respect (design guideline).
LO, HI = 0.01, 0.60
# Documented acceptance tolerance on the weight sum.
SUM_ABS_TOL = 1e-6

TERM_DEFINITIONS = {
    "angle": "pitch error theta/10deg, squared  (keep upright)",
    "rate": "pitch angular velocity theta_dot/2rad/s, squared  (no shaking)",
    "position": "horizontal reference-tracking error/0.25m, squared  (no drift from the moving reference)",
    "speed": "forward speed error (v-command)/0.15m/s, squared  (hold still with zero command, track with command)",
    "yaw": "yaw-rate error (yawrate-command)/0.25rad/s, squared  (no idle rotation)",
    "effort": "mean (motor torque/0.4Nm)^2  (limit effort/saturation)",
    "slew": "mean (action change/0.2)^2  (no motor chatter)",
    "survival": "constant +1 each step while not fallen  (stay alive/recover)",
}

OLLAMA_GENERATE = "http://127.0.0.1:11434/api/generate"
OLLAMA_TAGS = "http://127.0.0.1:11434/api/tags"


def available_models(timeout: float = 10.0) -> list[str]:
    with urllib.request.urlopen(OLLAMA_TAGS, timeout=timeout) as reply:
        tags = json.load(reply)
    return [m["name"] for m in tags.get("models", [])]


def _schema() -> dict:
    props = {k: {"type": "number", "minimum": LO, "maximum": HI} for k in KEYS}
    return {
        "type": "object",
        "properties": {
            "weights": {"type": "object", "properties": props,
                        "required": list(KEYS), "additionalProperties": False},
            "reason": {"type": "string"},
        },
        "required": ["weights", "reason"],
    }


def _system_task() -> str:
    return (
        "You design the eight scalar REWARD WEIGHTS for PPO fine-tuning of an EXISTING, "
        "already-trained standalone two-wheel self-balancing robot policy. You are NOT "
        "tuning PID gains and NOT changing the network. Actor is 34->64tanh->64tanh->2.\n"
        "PRIMARY TASK: stand upright and stay stationary under ZERO command without shaking, "
        "translating or rotating; follow deliberate speed/yaw commands when given; return to a "
        "stationary hold after commands stop. Penalize shaking, drift, idle rotation, motor "
        "saturation and chatter. Every term except 'survival' is a negative normalized squared "
        "penalty; 'survival' is a positive per-step bonus. The fall penalty (-25) is immutable and "
        "is NOT one of these weights.\n"
        f"Reply with ONLY one JSON object: {{\"weights\":{{...}},\"reason\":\"...\"}}. "
        f"Use EXACTLY these keys: {list(KEYS)}. Each weight must be finite and within [{LO}, {HI}], "
        "and the eight weights must sum to 1.0.\n"
        "Term meaning (what each weight trades off):\n" + json.dumps(TERM_DEFINITIONS, indent=0)
    )


def build_prompt(previous: Mapping[str, float], metrics: Mapping, iteration: int,
                 repair_error: str | None = None) -> str:
    parts = [_system_task(),
             f"\nIteration: {iteration}",
             "Previous accepted weights: " + json.dumps(previous),
             "Measured validation metrics of the CURRENT best policy (lower pitch/speed/yaw/"
             "displacement RMS and higher survival are better): " + json.dumps(metrics)]
    if repair_error:
        parts.append(
            "YOUR PREVIOUS REPLY WAS REJECTED. Fix exactly this and resend ONLY the JSON object. "
            f"Validation error: {repair_error}. Remember: exact keys {list(KEYS)}, each weight in "
            f"[{LO},{HI}], and all eight must sum to exactly 1.0.")
    return "\n".join(parts)


def _call_ollama(model: str, prompt: str, iteration: int, timeout: float = 180.0) -> dict:
    payload = {
        "model": model, "stream": False, "prompt": prompt, "format": _schema(),
        "options": {"temperature": 0.2, "seed": 100 + iteration, "num_predict": 400},
    }
    request = urllib.request.Request(
        OLLAMA_GENERATE, data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as reply:
        result = json.load(reply)
    return result


def _parse_cascade(raw: str) -> tuple[dict, str]:
    """Return (parsed_object, method). Raises ValueError if nothing parses."""
    try:
        return json.loads(raw), "direct"
    except json.JSONDecodeError:
        pass
    cleaned = re.sub(r"```(?:json)?|```", "", raw).strip()
    try:
        return json.loads(cleaned), "fenced_strip"
    except json.JSONDecodeError:
        pass
    # Balanced-brace extraction: first '{' to its matching '}'.
    start = cleaned.find("{")
    if start != -1:
        depth = 0
        for i in range(start, len(cleaned)):
            if cleaned[i] == "{":
                depth += 1
            elif cleaned[i] == "}":
                depth -= 1
                if depth == 0:
                    candidate = cleaned[start:i + 1]
                    try:
                        return json.loads(candidate), "balanced_brace"
                    except json.JSONDecodeError:
                        no_trailing = re.sub(r",\s*([}\]])", r"\1", candidate)
                        return json.loads(no_trailing), "trailing_comma_repair"
    raise ValueError("no JSON object could be extracted from the response")


def _validate(weights: Mapping) -> tuple[dict, str | None]:
    """Return (coerced_weights, error_or_None) for key/finite/range checks only.

    The sum check is handled separately so a good-shape proposal can be
    renormalized rather than discarded.
    """
    if set(weights) != set(KEYS):
        return {}, f"keys must be exactly {list(KEYS)}, got {sorted(weights)}"
    out = {}
    for k in KEYS:
        v = weights[k]
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return {}, f"weight '{k}' must be a number, got {type(v).__name__}"
        v = float(v)
        if not math.isfinite(v):
            return {}, f"weight '{k}' is not finite"
        if not (LO - 1e-9 <= v <= HI + 1e-9):
            return {}, f"weight '{k}'={v} outside [{LO},{HI}]"
        out[k] = v
    if out["angle"] <= 0.0:
        return {}, "angle weight must be positive"
    return out, None


def propose_reward(model: str, previous: Mapping[str, float], metrics: Mapping,
                   iteration: int, max_repair: int = 2) -> dict:
    """Ask Ollama for reward weights; return a fully-logged decision record.

    Status is one of:
      - 'accepted'                          valid and summed to 1 within tolerance
      - 'accepted_after_documented_normalization'  valid shape, sum renormalized
      - 'fallback_keep_previous'            unrecoverable; previous weights reused
    """
    previous = dict(previous)
    record: dict = {
        "iteration": iteration, "model": model, "weights_before": previous,
        "sum_abs_tol": SUM_ABS_TOL, "per_term_bounds": [LO, HI],
        "attempts": [], "status": None, "weights_accepted": None,
        "normalization": None, "llm_contributed": None,
    }
    started = time.monotonic()
    repair_error: str | None = None
    last_valid: dict | None = None  # valid shape/range but wrong sum

    for attempt in range(1 + max_repair):
        prompt = build_prompt(previous, metrics, iteration, repair_error)
        entry = {"attempt": attempt,
                 "prompt_sha256": hashlib.sha256(prompt.encode()).hexdigest(),
                 "prompt": prompt, "repair_error_sent": repair_error}
        try:
            api = _call_ollama(model, prompt, iteration)
            raw = api.get("response", "")
            entry["raw_response"] = raw
            entry["ollama_eval_count"] = api.get("eval_count")
            entry["ollama_total_duration_ns"] = api.get("total_duration")
            parsed, method = _parse_cascade(raw)
            entry["parse_method"] = method
            entry["parsed"] = parsed
            weights = parsed.get("weights", parsed)  # tolerate a bare weights object
            coerced, err = _validate(weights)
            if err:
                entry["validation_error"] = err
                record["attempts"].append(entry)
                repair_error = err
                continue
            total = sum(coerced.values())
            entry["weight_sum"] = total
            entry["reason"] = parsed.get("reason")
            if abs(total - 1.0) <= SUM_ABS_TOL:
                entry["result"] = "accepted_exact_sum"
                record["attempts"].append(entry)
                record["status"] = "accepted"
                record["weights_accepted"] = coerced
                record["accepted_reason"] = parsed.get("reason")
                record["llm_contributed"] = True
                break
            # Good shape but wrong sum: remember it, ask the model to fix once more.
            last_valid = coerced
            entry["result"] = "valid_shape_wrong_sum"
            record["attempts"].append(entry)
            repair_error = (f"weights summed to {total:.6f}, not 1.0; rescale so they sum to 1.0 "
                            f"while keeping each in [{LO},{HI}]")
        except Exception as exc:  # network / parse / schema failure
            entry["exception"] = repr(exc)
            record["attempts"].append(entry)
            repair_error = f"response was not valid JSON for the schema ({exc!r})"

    if record["status"] is None and last_valid is not None:
        # Documented renormalization transform (NOT silent, NOT raw LLM output).
        raw_sum = sum(last_valid.values())
        normalized = {k: last_valid[k] / raw_sum for k in KEYS}
        # Guard numerical drift so the env's strict sum check passes.
        drift = 1.0 - sum(normalized.values())
        normalized["survival"] += drift
        record["status"] = "accepted_after_documented_normalization"
        record["weights_accepted"] = normalized
        record["llm_contributed"] = True
        record["normalization"] = {
            "method": "divide_by_sum_then_absorb_residual_into_survival",
            "raw_weights_from_llm": last_valid,
            "raw_sum": raw_sum,
            "normalized_weights": normalized,
            "residual_absorbed": drift,
            "note": ("LLM proposed these relative weights; the sum was rescaled to 1.0. "
                     "This is a logged transformation of the raw proposal, not raw LLM output. "
                     "Any normalized term leaving [%.2f,%.2f] is acceptable to the env "
                     "(which only requires [0,1], sum 1, angle>0)." % (LO, HI)),
            "normalized_terms_outside_design_bounds": {
                k: v for k, v in normalized.items() if not (LO <= v <= HI)},
        }

    if record["status"] is None:
        record["status"] = "fallback_keep_previous"
        record["weights_accepted"] = previous
        record["llm_contributed"] = False
        record["fallback_note"] = ("No parseable in-range proposal after repair retries; "
                                    "reused previous safe weights. This iteration does NOT "
                                    "demonstrate an LLM reward-design contribution.")

    record["llm_wall_s"] = time.monotonic() - started
    return record


if __name__ == "__main__":
    # Smoke test the client against the live model with the default metrics stub.
    import argparse
    ap = argparse.ArgumentParser(description="Probe the Ollama reward designer once.")
    ap.add_argument("--model", default="llama3.2:3b")
    ap.add_argument("--iteration", type=int, default=0)
    args = ap.parse_args()
    print("available:", available_models())
    stub_metrics = {"pitch_rms_deg": {"mean": 0.3}, "speed_rms_m_s": {"mean": 0.02},
                    "yaw_rms_rad_s": {"mean": 0.1}, "final_x_m": {"mean": 0.0},
                    "survived": {"mean": 1.0}, "selection_loss_lower_is_better": 0.8}
    rec = propose_reward(args.model, DEFAULT_WEIGHTS, stub_metrics, args.iteration)
    print(json.dumps({k: rec[k] for k in ("status", "weights_accepted", "llm_contributed",
                                           "normalization")}, indent=2, default=str))
    print("attempts:", len(rec["attempts"]), "wall_s:", round(rec["llm_wall_s"], 2))
