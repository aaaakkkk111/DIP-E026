"""Offboard mistral reward-CODE designer for PPO fine-tuning of the tzejun actor.

Unlike ollama_reward.py (which only tuned eight bounded WEIGHTS), this module lets
the LLM (mistral:latest, offboard) WRITE the reward function body as a short Python
snippet over a fixed set of physical variables. That snippet is NEVER executed as
free Python. It is:

  1. Parsed with `ast` and checked against a strict node/name ALLOWLIST
     (no imports, attributes, subscripts, loops, comprehensions, lambdas,
     function/class defs, dunder names, or calls to anything but whitelisted
     math helpers). Node count is capped.
  2. Compiled once and evaluated in a namespace with NO builtins, only the
     whitelisted helpers plus the per-step input variables.
  3. Self-tested on a battery of random + extreme inputs; every output must be
     finite and within a bounded range or the snippet is REJECTED.
  4. On any failure the previous safe reward code is retained and the iteration
     is marked fallback-only (does NOT count as an LLM contribution).

The immutable fall penalty (-25) is applied by the env OUTSIDE this code; the LLM
cannot see, change or remove it. PPO (in finetune_mistral.py) is what actually
changes actor weights; a reward change alone does not alter a deployed actor.

Everything (prompt, prompt hash, raw response, extracted code, validation errors,
self-test result, latency, final status) is returned so the caller can persist it.
"""
from __future__ import annotations

import ast
import hashlib
import json
import math
import re
import time
import urllib.request

OLLAMA_GENERATE = "http://127.0.0.1:11434/api/generate"
OLLAMA_TAGS = "http://127.0.0.1:11434/api/tags"

# Per-step physical variables the reward code may read. All finite floats.
INPUT_VARS = (
    "pitch", "pitch_rate", "roll", "yaw_rate",
    "v_forward", "target_v", "speed_err",
    "target_yaw", "yaw_err", "position_error",
    "torque_left", "torque_right", "effort",
    "action_left", "action_right", "da_left", "da_right", "slew",
)

# Whitelisted helpers available to the reward code (no builtins otherwise).
def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

SAFE_FUNCS = {
    "abs": abs, "min": min, "max": max, "clip": _clip,
    "sqrt": lambda x: math.sqrt(x) if x >= 0 else 0.0,
    "exp": lambda x: math.exp(_clip(x, -50.0, 50.0)),
    "tanh": math.tanh, "sin": math.sin, "cos": math.cos,
    "pow2": lambda x: x * x,
}

ALLOWED_FUNC_NAMES = frozenset(SAFE_FUNCS)
REWARD_CLIP = 10.0            # per-step shaped reward is clipped to +/- this
MAX_NODES = 400
MAX_ABS_EXPONENT = 6

# Node types permitted anywhere in the snippet.
_ALLOWED_NODES = (
    ast.Module, ast.Expr, ast.Assign, ast.AugAssign, ast.Name, ast.Load,
    ast.Store, ast.Constant, ast.BinOp, ast.UnaryOp, ast.BoolOp, ast.Compare,
    ast.IfExp, ast.Call,
    ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod, ast.Pow,
    ast.USub, ast.UAdd, ast.And, ast.Or, ast.Not,
    ast.Lt, ast.LtE, ast.Gt, ast.GtE, ast.Eq, ast.NotEq,
)

# A safe default reward in the same variable language. Seeds iteration 0 and is
# the fallback whenever a proposal cannot be validated. Mirrors the intent of the
# eight-term weighted default (keep upright, still, no drift/rotation/chatter).
DEFAULT_REWARD_CODE = (
    "reward = -( 1.6*pow2(pitch/0.1745)\n"
    "          + 0.6*pow2(pitch_rate/2.0)\n"
    "          + 0.9*pow2(position_error/0.25)\n"
    "          + 1.4*pow2(speed_err/0.15)\n"
    "          + 1.5*pow2(yaw_err/0.25)\n"
    "          + 0.4*pow2(effort/0.4)\n"
    "          + 0.4*pow2(slew/0.2) ) + 0.8"
)


def available_models(timeout: float = 10.0) -> list[str]:
    with urllib.request.urlopen(OLLAMA_TAGS, timeout=timeout) as reply:
        return [m["name"] for m in json.load(reply).get("models", [])]


def validate_code(code: str) -> tuple[bool, str]:
    """Static allowlist check. Returns (ok, error_message)."""
    if len(code) > 4000:
        return False, "code too long"
    try:
        tree = ast.parse(code, mode="exec")
    except SyntaxError as exc:
        return False, f"syntax error: {exc}"
    assigned: set[str] = set()
    n = 0
    for node in ast.walk(tree):
        n += 1
        if n > MAX_NODES:
            return False, "too many AST nodes"
        if not isinstance(node, _ALLOWED_NODES):
            return False, f"disallowed syntax: {type(node).__name__}"
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if not isinstance(tgt, ast.Name):
                    return False, "assignment target must be a plain name"
                assigned.add(tgt.id)
        if isinstance(node, ast.AugAssign):
            if not isinstance(node.target, ast.Name):
                return False, "augmented assignment target must be a plain name"
            assigned.add(node.target.id)
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name) or node.func.id not in ALLOWED_FUNC_NAMES:
                return False, "only whitelisted math helpers may be called"
            if node.keywords:
                return False, "keyword arguments are not allowed"
        if isinstance(node, ast.Constant) and not isinstance(node.value, (int, float)):
            return False, "only numeric constants are allowed"
        if isinstance(node, ast.Pow):
            pass  # exponent magnitude checked below
        if isinstance(node, ast.Name) and node.id.startswith("__"):
            return False, "dunder names are forbidden"
    # Exponent magnitude guard (prevents pow blow-ups).
    for node in ast.walk(tree):
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Pow):
            rhs = node.right
            if isinstance(rhs, ast.Constant) and isinstance(rhs.value, (int, float)):
                if abs(rhs.value) > MAX_ABS_EXPONENT:
                    return False, "exponent magnitude too large"
            elif isinstance(rhs, ast.UnaryOp) and isinstance(rhs.operand, ast.Constant):
                if abs(rhs.operand.value) > MAX_ABS_EXPONENT:
                    return False, "exponent magnitude too large"
            else:
                return False, "exponent must be a small numeric constant"
    # Every name read must be an input var, a helper, or a name assigned above it.
    known = set(INPUT_VARS) | ALLOWED_FUNC_NAMES | assigned | {"reward"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load):
            if node.id not in known:
                return False, f"unknown name: {node.id}"
    if "reward" not in assigned:
        return False, "code must assign a variable named 'reward'"
    return True, ""


def compile_reward(code: str):
    ok, err = validate_code(code)
    if not ok:
        raise ValueError(f"reward code rejected: {err}")
    return compile(ast.parse(code, mode="exec"), "<llm_reward>", "exec")


def _vars_from(info: dict, action, prev_action) -> dict:
    al, ar = float(action[0]), float(action[1])
    pl = al if prev_action is None else float(prev_action[0])
    pr = ar if prev_action is None else float(prev_action[1])
    tl, tr = float(info["torque_left_nm"]), float(info["torque_right_nm"])
    v = float(info["v_forward_m_s"])
    tv = float(info["target_v_m_s"])
    yr = float(info["yaw_rate_rad_s"])
    tyaw = float(info["target_yaw_rad_s"])
    return {
        "pitch": float(info["pitch"]), "pitch_rate": float(info["pitch_rate_rad_s"]),
        "roll": float(info["roll"]), "yaw_rate": yr,
        "v_forward": v, "target_v": tv, "speed_err": v - tv,
        "target_yaw": tyaw, "yaw_err": yr - tyaw,
        "position_error": float(info["position_error_m"]),
        "torque_left": tl, "torque_right": tr, "effort": 0.5 * (abs(tl) + abs(tr)),
        "action_left": al, "action_right": ar,
        "da_left": al - pl, "da_right": ar - pr, "slew": 0.5 * (abs(al - pl) + abs(ar - pr)),
    }


def make_reward_fn(code_obj):
    """Return callable(info, action, prev_action) -> finite reward in +/-REWARD_CLIP."""
    def reward_fn(info, action, prev_action):
        ns = {"__builtins__": {}}
        ns.update(SAFE_FUNCS)
        ns.update(_vars_from(info, action, prev_action))
        exec(code_obj, ns)  # noqa: S102 - sandboxed: allowlisted AST, no builtins
        r = ns.get("reward")
        if not isinstance(r, (int, float)) or not math.isfinite(float(r)):
            return -REWARD_CLIP
        return _clip(float(r), -REWARD_CLIP, REWARD_CLIP)
    return reward_fn


def self_test_reward(code_obj, trials: int = 400, seed: int = 0) -> tuple[bool, str]:
    """Every output over random + extreme inputs must be finite. Rejects blow-ups."""
    import random
    rng = random.Random(seed)
    fn = make_reward_fn(code_obj)
    extremes = [
        dict(pitch=0.0, pitch_rate=0.0, roll=0.0, yaw_rate=0.0, v=0.0, tv=0.0, tyaw=0.0, pe=0.0, tl=0.0, tr=0.0),
        dict(pitch=0.7, pitch_rate=20.0, roll=0.7, yaw_rate=15.0, v=1.0, tv=-0.3, tyaw=0.5, pe=2.0, tl=0.4, tr=-0.4),
        dict(pitch=-0.7, pitch_rate=-20.0, roll=-0.7, yaw_rate=-15.0, v=-1.0, tv=0.3, tyaw=-0.5, pe=2.0, tl=-0.4, tr=0.4),
    ]
    def mk(s):
        info = {"pitch": s["pitch"], "pitch_rate_rad_s": s["pitch_rate"], "roll": s["roll"],
                "yaw_rate_rad_s": s["yaw_rate"], "v_forward_m_s": s["v"], "target_v_m_s": s["tv"],
                "target_yaw_rad_s": s["tyaw"], "position_error_m": s["pe"],
                "torque_left_nm": s["tl"], "torque_right_nm": s["tr"]}
        return info
    for s in extremes:
        try:
            r = fn(mk(s), [1.0, -1.0], [-1.0, 1.0])
        except Exception as exc:  # noqa: BLE001 - any eval error -> reject
            return False, f"evaluation raised {type(exc).__name__}: {exc}"
        if not math.isfinite(r):
            return False, "non-finite reward on extreme input"
    for _ in range(trials):
        s = dict(pitch=rng.uniform(-0.8, 0.8), pitch_rate=rng.uniform(-25, 25),
                 roll=rng.uniform(-0.8, 0.8), yaw_rate=rng.uniform(-20, 20),
                 v=rng.uniform(-1.2, 1.2), tv=rng.uniform(-0.3, 0.3), tyaw=rng.uniform(-0.5, 0.5),
                 pe=rng.uniform(0, 2.5), tl=rng.uniform(-0.5, 0.5), tr=rng.uniform(-0.5, 0.5))
        a = [rng.uniform(-1, 1), rng.uniform(-1, 1)]
        pa = [rng.uniform(-1, 1), rng.uniform(-1, 1)]
        try:
            r = fn(mk(s), a, pa)
        except Exception as exc:  # noqa: BLE001
            return False, f"evaluation raised {type(exc).__name__}: {exc}"
        if not math.isfinite(r):
            return False, "non-finite reward on random input"
    return True, ""


_PROMPT = """You design the per-step REWARD for a reinforcement-learning controller of a
two-wheel self-balancing robot. Reward is MAXIMISED by PPO. Higher reward must mean
better behaviour: stay upright and still, do not drift or rotate when the command is
zero, follow movement commands, and avoid motor chatter and saturation.

Write a short Python snippet (no imports, no function defs, no loops) that assigns a
variable named `reward`. You MAY read only these per-step variables (all finite floats):
  pitch (rad, 0=upright), pitch_rate (rad/s), roll (rad), yaw_rate (rad/s),
  v_forward (m/s), target_v (m/s), speed_err (=v_forward-target_v),
  target_yaw (rad/s), yaw_err (=yaw_rate-target_yaw), position_error (m),
  torque_left, torque_right (Nm), effort (mean |torque|),
  action_left, action_right, da_left, da_right, slew (mean |action change|).
You MAY call only: abs min max clip(x,lo,hi) sqrt exp tanh sin cos pow2(x)=x*x.
Those variables and helpers are the ONLY names allowed in the code. The measured
behaviour shown below (names like pitch_rms_deg, yaw_rms_rad_s, stationkeeping_success,
survived, selection_loss) is a REPORT to guide you - those names are NOT available in
the code and using any of them makes the snippet invalid. Steer the per-step variables
to fix whatever the report shows is weak (e.g. if yaw_rms is high, weight yaw_err more).
Keep the per-step reward roughly within [-10, 10]; it will be clipped to that range.
Do NOT add a fall penalty - a fixed -25 is applied automatically on a fall; you cannot
change it. Penalise bad behaviour (negative terms) and optionally add a small positive
alive bonus. The ONLY identifiers allowed anywhere in the code are these per-step
variables -- pitch pitch_rate roll yaw_rate v_forward target_v speed_err target_yaw
yaw_err position_error torque_left torque_right effort action_left action_right
da_left da_right slew -- plus the helpers abs min max clip sqrt exp tanh sin cos pow2
and your own temporary variables. Any other name (especially a metric name like
pitch_rms_deg) makes it invalid. Two valid examples:
  reward = -(2.0*pow2(pitch/0.1745) + 1.5*pow2(yaw_err/0.25) + 1.2*pow2(speed_err/0.15) + 0.4*pow2(slew/0.2)) + 0.8
  tilt = pow2(pitch/0.1745); spin = pow2(yaw_err/0.25)
  reward = clip(1.0 - 1.8*tilt - 1.6*spin - 1.0*pow2(speed_err/0.15) - 0.5*pow2(slew/0.2), -10.0, 10.0)
Return ONLY a JSON object on one line: {"reward_code": "<python>", "rationale": "<one line>"}.
"""


def _generate(model: str, prompt: str, timeout: float = 180.0,
              temperature: float = 0.3) -> tuple[str, float]:
    body = json.dumps({"model": model, "prompt": prompt, "stream": False,
                       "options": {"temperature": temperature, "num_predict": 700,
                                   "top_p": 0.95}}).encode()
    t0 = time.monotonic()
    req = urllib.request.Request(OLLAMA_GENERATE, data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as reply:
        out = json.load(reply)
    return out.get("response", ""), time.monotonic() - t0


def _extract_code(raw: str) -> str | None:
    if not raw:
        return None
    cleaned = raw.strip()
    candidates = [cleaned, re.sub(r"^```(?:json|python)?|```$", "", cleaned, flags=re.I).strip()]
    candidates += re.findall(r"\{.*?\}", cleaned, flags=re.S)
    candidates += re.findall(r"\{.*\}", cleaned, flags=re.S)
    for text in candidates:
        try:
            obj = json.loads(re.sub(r",\s*([}\]])", r"\1", text))
        except Exception:  # noqa: BLE001
            continue
        code = obj.get("reward_code")
        if isinstance(code, list):
            code = "\n".join(str(x) for x in code)
        if isinstance(code, str) and "reward" in code:
            return code.replace("\\n", "\n").strip()
    # Fenced python block that assigns reward.
    m = re.search(r"```(?:python)?\s*(.*?reward\s*=.*?)```", cleaned, flags=re.S | re.I)
    if m:
        return m.group(1).strip()
    # Last resort: collect any assignment lines if one of them assigns reward.
    lines = [ln for ln in cleaned.splitlines() if re.match(r"\s*[A-Za-z_]\w*\s*=", ln)]
    if any(re.match(r"\s*reward\s*=", ln) for ln in lines):
        return "\n".join(lines).strip()
    return None


def design_reward(model: str, prev_code: str, metrics_summary: str, iteration: int,
                  max_attempts: int = 4) -> dict:
    """Ask mistral to WRITE reward code; validate + self-test; on failure feed the exact
    error back and retry (bounded repair). Fall back to prev_code only if every attempt
    fails. Returns full provenance plus 'code_accepted' (str) and 'reward_code_obj'
    (compiled code object, not JSON-serialisable)."""
    base_prompt = (_PROMPT
                   + f"\nITERATION {iteration}. Previous reward code to improve on:\n{prev_code}\n"
                   + "\nMeasured behaviour of the current best policy (steer the per-step "
                     "variables to fix the weak metrics):\n" + metrics_summary
                   + "\nReturn ONLY the JSON object.")
    record = {"iteration": iteration, "model": model, "attempts": [],
              "prompt_sha256": hashlib.sha256(base_prompt.encode()).hexdigest()}
    prompt, last_err, total_latency = base_prompt, "", 0.0
    for attempt in range(max_attempts):
        # Escalate temperature across retries so repairs actually diversify instead
        # of re-emitting the same rejected code.
        temp = 0.25 + 0.3 * attempt
        try:
            raw, latency = _generate(model, prompt, temperature=temp)
        except Exception as exc:  # noqa: BLE001 - server/network error
            last_err = f"generate_error: {exc}"
            record["attempts"].append({"attempt": attempt, "error": last_err})
            continue
        total_latency += latency
        code = _extract_code(raw)
        if not code:
            last_err = "no reward_code found in the response JSON"
        else:
            ok, verr = validate_code(code)
            if not ok:
                last_err = f"validation failed: {verr}"
            else:
                code_obj = compile_reward(code)
                safe, sreason = self_test_reward(code_obj)
                if not safe:
                    last_err = f"self-test failed: {sreason}"
                else:
                    record["attempts"].append({"attempt": attempt, "code": code, "error": ""})
                    record.update(status="accepted", llm_contributed=True, code_accepted=code,
                                  validation_error="", latency_s=total_latency,
                                  n_attempts=attempt + 1, raw_response=raw)
                    record["reward_code_obj"] = code_obj
                    return record
        record["attempts"].append({"attempt": attempt, "code": code, "error": last_err,
                                   "raw": raw[:400]})
        hint = ""
        m = re.search(r"unknown name:\s*(\w+)", last_err)
        if m:
            hint = (f" The identifier '{m.group(1)}' does NOT exist - delete every term that "
                    "uses it and re-weight the remaining allowed variables.")
        prompt = (base_prompt + f"\n\nYour previous attempt was REJECTED: {last_err}.{hint}\n"
                  "Write a DIFFERENT, simpler snippet. Use ONLY these names: pitch pitch_rate "
                  "roll yaw_rate v_forward target_v speed_err target_yaw yaw_err position_error "
                  "effort action_left action_right da_left da_right slew, and the helpers "
                  "abs min max clip sqrt exp tanh sin cos pow2. Return ONLY the JSON object.")
    record.update(status=f"fallback_after_{max_attempts}_attempts", llm_contributed=False,
                  code_accepted=prev_code, validation_error=last_err, latency_s=total_latency)
    record["reward_code_obj"] = compile_reward(prev_code)
    return record


def self_test() -> dict:
    """Prove the sandbox blocks the obvious escapes and accepts a sane reward."""
    good_ok, _ = validate_code(DEFAULT_REWARD_CODE)
    attacks = [
        "import os\nreward=0",
        "reward = open('x')",
        "reward = (1).__class__",
        "reward = eval('2')",
        "reward = effort.__dict__",
        "reward = [x for x in range(3)][0]",
        "reward = globals()",
        "for i in range(3):\n reward=i",
        "reward = pitch ** 99",
        "reward = unknown_var",
    ]
    blocked = []
    for a in attacks:
        ok, err = validate_code(a)
        blocked.append({"code": a.split(chr(10))[0], "blocked": not ok, "reason": err})
    code_obj = compile_reward(DEFAULT_REWARD_CODE)
    safe, reason = self_test_reward(code_obj)
    return {"default_valid": good_ok, "default_selftest_ok": safe, "selftest_reason": reason,
            "all_attacks_blocked": all(b["blocked"] for b in blocked), "attacks": blocked}


if __name__ == "__main__":
    print(json.dumps(self_test(), indent=2))
