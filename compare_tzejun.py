"""Five-arm, matched-condition comparison on the tzejun measured MuJoCo plant.

The four requested controls are PID, LLM-PID, fixed-reward PPO residual,
and LLM-reward PPO residual. The original tzejun PPO is a fifth reference,
not mislabeled as an LLM/PID controller. No hardware commands are sent.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import time
import urllib.request
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import gymnasium as gym
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
import torch

from tzejun_experiment import (
    ROOT, CONTROL_DT, MAX_STEPS, CheckpointPolicy, EpisodeConfig,
    TzejunPlant, aggregate, decode_state, episode_metrics, run_episode,
)

torch.set_num_threads(1)
OUT = ROOT / "experiment_comparison"
KEYS = ("angle", "vel", "pos", "ctrl", "survival")


@dataclass
class Gains:
    pitch: float = 1.0
    pitch_rate: float = 0.08
    speed: float = 0.7
    speed_i: float = 0.08
    yaw: float = 0.35
    yaw_rate: float = 0.0


class Cascade:
    def __init__(self, gains: Gains):
        self.gains = gains
        self.plant: TzejunPlant | None = None
        self.residual = np.zeros(2, dtype=np.float32)
        self.speed_i = 0.0
        self.last_action = np.zeros(2, dtype=np.float32)

    def reset(self):
        self.speed_i = 0.0
        self.residual[:] = 0.0
        self.last_action[:] = 0.0

    def predict(self, _obs):
        p = self.plant
        assert p is not None
        _roll, pitch, _yaw, lv, yaw_rate = decode_state(p.data)
        pitch_rate = float(p.data.qvel[4])
        speed_err = float(p.target_v - lv[0])
        self.speed_i = float(np.clip(self.speed_i + speed_err * CONTROL_DT, -0.4, 0.4))
        g = self.gains
        # Positive wheel torque reduces positive forward pitch on this model.
        balance = g.pitch * pitch + g.pitch_rate * pitch_rate - g.speed * speed_err - g.speed_i * self.speed_i
        turn = g.yaw * (p.target_yaw - yaw_rate) - g.yaw_rate * yaw_rate
        action = np.asarray([balance - turn, balance + turn], dtype=np.float32) + self.residual
        self.last_action = np.clip(action, -1.0, 1.0)
        return self.last_action


def policy_observation(plant: TzejunPlant) -> np.ndarray:
    roll, pitch, _yaw, lv, yaw_rate = decode_state(plant.data)
    q = plant.data.qvel
    return np.clip(np.asarray([
        pitch / 0.35, q[4] / 5.0, roll / 0.35, q[3] / 5.0,
        yaw_rate / 2.0, q[6] / 35.0, q[7] / 35.0,
        plant.target_v / 0.3, plant.target_yaw / 0.5,
    ], dtype=np.float32), -5.0, 5.0)


def fixed_reward(row: dict) -> float:
    theta = math.radians(row["pitch_deg"])
    theta_dot = row["pitch_rate_rad_s"]
    torque = 0.5 * (row["torque_left_nm"] ** 2 + row["torque_right_nm"] ** 2)
    return -theta * theta - 0.1 * theta_dot * theta_dot - 0.001 * torque


def weighted_reward(row: dict, weights: dict) -> float:
    theta = math.radians(row["pitch_deg"])
    terms = {
        "angle": -theta * theta,
        "vel": -row["pitch_rate_rad_s"] ** 2,
        "pos": -row["x_m"] ** 2,
        "ctrl": -0.5 * (row["torque_left_nm"] ** 2 + row["torque_right_nm"] ** 2),
        "survival": 1.0 if not row["terminated"] else -10.0,
    }
    return float(sum(weights[k] * terms[k] for k in KEYS))


class ResidualEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, gains: Gains, reward_kind: str, weights: dict | None = None):
        super().__init__()
        self.cascade = Cascade(gains)
        self.plant = TzejunPlant(self.cascade)
        self.cascade.plant = self.plant
        self.reward_kind = reward_kind
        self.weights = weights or {}
        self.observation_space = spaces.Box(-5, 5, shape=(9,), dtype=np.float32)
        self.action_space = spaces.Box(-1, 1, shape=(2,), dtype=np.float32)
        self.rng = np.random.default_rng(0)
        self.episode = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.episode += 1
        pitch = float(self.rng.choice([-1.0, 1.0]) * self.rng.uniform(0.05, 0.1745329252))
        target_v = float(self.rng.choice([0.0, 0.0, 0.0, -0.15, 0.15]))
        target_yaw = float(self.rng.choice([0.0, 0.0, -0.25, 0.25]))
        cfg = EpisodeConfig("train", self.episode, self.episode, 0.0, pitch, False, target_v, target_yaw, 0.0)
        self.cascade.reset()
        self.plant.reset(cfg)
        return policy_observation(self.plant), {}

    def step(self, action):
        self.cascade.residual = np.asarray(action, dtype=np.float32) * 0.3
        _, row = self.plant.step(np.empty(34, dtype=np.float32))
        terminated = bool(row["terminated"])
        truncated = self.plant.step_count >= MAX_STEPS
        reward = fixed_reward(row) if self.reward_kind == "fixed" else weighted_reward(row, self.weights)
        if terminated:
            reward -= 10.0
        return policy_observation(self.plant), reward, terminated, truncated, row


class EvalResidual(Cascade):
    def __init__(self, gains: Gains, model: PPO | None):
        super().__init__(gains)
        self.model = model

    def predict(self, obs):
        if self.model is not None:
            assert self.plant is not None
            a, _ = self.model.predict(policy_observation(self.plant), deterministic=True)
            self.residual = np.asarray(a, dtype=np.float32) * 0.3
        return super().predict(obs)


def configs(seeds: int):
    return [EpisodeConfig("fixed_pitch", i, i, 0.0,
                          (1 if i % 2 == 0 else -1) * math.radians(10),
                          False, 0.0, 0.0, 0.0) for i in range(seeds)]


def evaluate(policy, seeds=15):
    plant = TzejunPlant(policy)
    if isinstance(policy, Cascade):
        policy.plant = plant
    metrics = []
    traces = []
    for cfg in configs(seeds):
        if isinstance(policy, Cascade):
            policy.reset()
        rows, metric = run_episode(plant, cfg)
        metrics.append(metric)
        if cfg.episode < 2:
            traces.append(rows)
    return metrics, traces


def score(metrics):
    a = aggregate(metrics)
    s = a["settling_time_s"]["mean"]
    r = a["steady_rms_pitch_deg"]["mean"]
    return (s if s is not None else 10.0) + (r if r is not None else 90.0) / 10.0 + 10.0 * (1-a["survival_fraction"])


def calibrate_pid():
    # Calibration uses 2 held-out-from-report pilot episodes, never eval seeds 0..14.
    candidates = [
        Gains(kp, kd, kv, 0.08, 0.35, 0.0)
        for kp in (0.5, 1.0, 2.0, 4.0, 6.0)
        for kd in (0.02, 0.08, 0.16, 0.3)
        for kv in (0.3, 0.7)
    ]
    pilots = [EpisodeConfig("pilot", i, 200+i, 0.0, s * math.radians(10), False, 0.0, 0.0, 0.0)
              for i, s in enumerate((1, -1))]
    results = []
    for gains in candidates:
        p = EvalResidual(gains, None)
        plant = TzejunPlant(p); p.plant = plant
        vals = []
        for cfg in pilots:
            p.reset()
            _, m = run_episode(plant, cfg)
            vals.append(m)
        results.append((score(vals), gains))
    best = min(results, key=lambda x: x[0])
    return best[1], [{"score": float(s), "gains": asdict(g)} for s,g in results]


def ollama_call(prompt: str, model: str):
    req = urllib.request.Request("http://127.0.0.1:11434/api/generate",
        data=json.dumps({"model": model, "prompt": prompt, "stream": False,
                         "options": {"temperature": 0.0}}).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=180) as resp:
        payload = json.load(resp)
    return payload.get("response", "")


def parse_object(raw):
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", raw, re.S)
        if not match:
            raise ValueError("no JSON object in response")
        return json.loads(re.sub(r",\s*([}\]])", r"\1", match.group(0)))


def propose_gains(gains, model, pilot_metrics):
    prompt = ("You tune a simulation-only two-wheel PID. Output JSON with exactly "
              "pitch,pitch_rate,speed,speed_i,yaw,yaw_rate. Change each by at most 20%, "
              "nonnegative only. No prose. Current gains=" + json.dumps(asdict(gains))
              + " Pilot metrics=" + json.dumps(aggregate(pilot_metrics)))
    decision = {"prompt": prompt, "before": asdict(gains)}
    try:
        raw = ollama_call(prompt, model); decision["raw_response"] = raw
        proposal = parse_object(raw)
        if set(proposal) != set(asdict(gains)):
            raise ValueError("gain keys mismatch")
        accepted = {}
        for k, old in asdict(gains).items():
            value = float(proposal[k])
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"invalid {k}")
            accepted[k] = float(np.clip(value, old*0.8, old*1.2))
        decision["proposed"] = proposal
        decision["accepted"] = accepted
        return Gains(**accepted), decision
    except Exception as exc:
        decision["error"] = repr(exc)
        decision["accepted"] = asdict(gains)
        return gains, decision


def propose_weights(weights, model, metrics, iteration):
    prompt = ("You design reward weights for a PPO residual on a PID balancer. "
              "Reply ONLY JSON with angle,vel,pos,ctrl,survival numbers in [0,1] "
              "summing to 1. Example: {\"angle\":0.4,\"vel\":0.2,\"pos\":0.15,"
              "\"ctrl\":0.1,\"survival\":0.15}. Current=" + json.dumps(weights)
              + " Evaluation=" + json.dumps(aggregate(metrics)) + f" Iteration={iteration}")
    decision = {"iteration": iteration, "prompt": prompt, "before": weights}
    try:
        raw = ollama_call(prompt, model); decision["raw_response"] = raw
        proposed = parse_object(raw)
        decision["proposed"] = proposed
        if set(proposed) != set(KEYS):
            raise ValueError("reward keys mismatch")
        accepted = {k: float(proposed[k]) for k in KEYS}
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in accepted.values()):
            raise ValueError("reward weight outside [0,1]")
        if abs(sum(accepted.values()) - 1.0) > 0.001:
            # Ask the LLM for an explicit repair; do not silently renormalize.
            repair = ("Your prior JSON summed to " + str(sum(accepted.values()))
                      + ", not 1. Return corrected JSON with the same five keys, "
                      "each in [0,1], whose sum is exactly 1. No prose. Prior: " + raw)
            repaired_raw = ollama_call(repair, model)
            decision["repair_prompt"] = repair
            decision["repair_raw_response"] = repaired_raw
            proposed = parse_object(repaired_raw)
            decision["repair_proposed"] = proposed
            if set(proposed) != set(KEYS):
                raise ValueError("repaired reward keys mismatch")
            accepted = {k: float(proposed[k]) for k in KEYS}
            if any(not math.isfinite(v) or not 0 <= v <= 1 for v in accepted.values()):
                raise ValueError("repaired reward weight outside [0,1]")
            repaired_sum = sum(accepted.values())
            if abs(repaired_sum - 1.0) > 0.001:
                if repaired_sum <= 0:
                    raise ValueError("repaired reward weights have non-positive sum")
                # Preserve the LLM's relative weighting while enforcing the
                # simplex constraint. This is explicit in the decision log.
                accepted = {k: v / repaired_sum for k, v in accepted.items()}
                decision["normalization_applied"] = True
                decision["pre_normalization_sum"] = repaired_sum
        decision["accepted"] = accepted
        return accepted, decision
    except Exception as exc:
        decision["error"] = repr(exc)
        decision["accepted"] = weights
        return weights, decision


def train(name, gains, kind, weights, timesteps, seed):
    env = ResidualEnv(gains, kind, weights)
    model = PPO("MlpPolicy", env, policy_kwargs={"net_arch": dict(pi=[64,64], vf=[64,64])},
                n_steps=2048, batch_size=64, n_epochs=10, gamma=0.99,
                gae_lambda=0.95, clip_range=0.2, learning_rate=3e-4,
                seed=seed, verbose=0, device="cpu")
    start = time.perf_counter()
    model.learn(total_timesteps=timesteps)
    duration = time.perf_counter() - start
    path = OUT / (name + ".zip")
    model.save(path)
    env.close()
    return model, {"path": str(path.relative_to(ROOT)), "timesteps": timesteps,
                   "seed": seed, "wall_time_s": duration, "reward_kind": kind,
                   "weights": weights, "actor": "9-64-64-2", "critic": "9-64-64-1"}


def plot_comparison(raw, summary):
    fields = [("settling_time_s", "Settling time (s)"),
              ("recovery_overshoot_deg", "Overshoot (deg)"),
              ("steady_rms_pitch_deg", "Steady RMS pitch (deg)"),
              ("mean_abs_torque_nm", "Mean |torque| (N m)"),
              ("torque_variance", "Torque variance"),
              ("success_rate", "Success rate")]
    labels = list(summary)
    fig, axes = plt.subplots(2, 3, figsize=(17, 9), constrained_layout=True)
    for ax, (key, title) in zip(axes.flat, fields):
        vals = [summary[k][key]["mean"] if isinstance(summary[k][key], dict) else summary[k][key] for k in labels]
        errs = [summary[k][key]["ci95"] if isinstance(summary[k][key], dict) else 0 for k in labels]
        ax.bar(range(len(labels)), vals, yerr=errs, capsize=3)
        ax.set_xticks(range(len(labels)), labels, rotation=25, ha="right")
        ax.set_title(title)
    fig.savefig(OUT / "comparison.png", dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=200000)
    ap.add_argument("--iterations", type=int, default=5)
    ap.add_argument("--seeds", type=int, default=15)
    ap.add_argument("--ollama-model", default="llama3.2:3b")
    ap.add_argument("--reuse-fixed", action="store_true",
                    help="reuse an existing 200k fixed-reward checkpoint")
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    started = datetime.now(timezone.utc).isoformat()
    baseline, sweep = calibrate_pid()
    (OUT / "pid_calibration.json").write_text(json.dumps({"selected": asdict(baseline), "sweep": sweep}, indent=2))
    print("PID calibrated", asdict(baseline), flush=True)

    # Gain proposal is made on calibration episodes, never on evaluation seeds.
    pilot = EvalResidual(baseline, None); plant = TzejunPlant(pilot); pilot.plant = plant
    pilot_metrics = []
    for i, s in enumerate((1,-1)):
        pilot.reset()
        _, m = run_episode(plant, EpisodeConfig("pilot", i, 300+i, 0.0, s*math.radians(10), False, 0, 0, 0))
        pilot_metrics.append(m)
    tuned, gain_decision = propose_gains(baseline, args.ollama_model, pilot_metrics)
    (OUT / "llm_pid_decision.json").write_text(json.dumps(gain_decision, indent=2))
    print("LLM-PID decision", "fallback" if "error" in gain_decision else "accepted", flush=True)

    fixed_path = OUT / "fixed_reward_ppo.zip"
    fixed_meta_path = OUT / "fixed_reward_training.json"
    if args.reuse_fixed and fixed_path.exists() and fixed_meta_path.exists():
        fixed_training = json.loads(fixed_meta_path.read_text())
        if int(fixed_training.get("timesteps", -1)) != args.timesteps:
            raise RuntimeError("existing fixed-reward checkpoint has a different training budget")
        fixed_model = PPO.load(fixed_path, device="cpu")
        print("Fixed PPO reused", fixed_path, flush=True)
    else:
        fixed_model, fixed_training = train("fixed_reward_ppo", baseline, "fixed", None, args.timesteps, 0)
        fixed_meta_path.write_text(json.dumps(fixed_training, indent=2))
        print("Fixed PPO trained", fixed_training["wall_time_s"], flush=True)

    weights = {"angle":0.4,"vel":0.2,"pos":0.15,"ctrl":0.1,"survival":0.15}
    design_log = []; best = None
    for iteration in range(args.iterations):
        model, training = train(f"llm_reward_iter_{iteration:02d}", baseline, "weighted", weights,
                                args.timesteps, iteration)
        proxy = EvalResidual(baseline, model)
        pilot_vals = []
        p = TzejunPlant(proxy); proxy.plant = p
        for i, s in enumerate((1,-1)):
            proxy.reset()
            _, metric = run_episode(p, EpisodeConfig("design", i, 400+i, 0.0, s*math.radians(10), False, 0, 0, 0))
            pilot_vals.append(metric)
        design_score = score(pilot_vals)
        if best is None or design_score < best["score"]:
            best = {"iteration": iteration, "score": design_score,
                    "weights": dict(weights), "checkpoint": training["path"]}
        next_weights, decision = propose_weights(weights, args.ollama_model, pilot_vals, iteration)
        design_log.append({"iteration": iteration, "weights_trained": dict(weights),
                           "training": training, "pilot_metrics": pilot_vals,
                           "score": design_score, "decision": decision})
        (OUT / "reward_design_log.json").write_text(json.dumps(design_log, indent=2))
        print("LLM iteration", iteration, "score", design_score,
              "decision", "fallback" if "error" in decision else "accepted", flush=True)
        weights = next_weights
    assert best is not None
    best_model = PPO.load(ROOT / best["checkpoint"], device="cpu")

    policies = {
        "PID-only": EvalResidual(baseline, None),
        "LLM-PID": EvalResidual(tuned, None),
        "Fixed-Reward-PPO+PID": EvalResidual(baseline, fixed_model),
        "LLM-Reward-PPO+PID": EvalResidual(baseline, best_model),
        "tzejun-trained-PPO": CheckpointPolicy(),
    }
    raw = {}; summary = {}
    for name, policy in policies.items():
        vals, _ = evaluate(policy, args.seeds)
        raw[name] = vals; summary[name] = aggregate(vals)
        (OUT / "comparison_raw.json").write_text(json.dumps(raw, indent=2))
        (OUT / "comparison_summary.json").write_text(json.dumps(summary, indent=2))
        print(name, "settle", summary[name]["settling_time_s"],
              "RMS", summary[name]["steady_rms_pitch_deg"],
              "success", summary[name]["success_rate"], flush=True)
    plot_comparison(raw, summary)
    protocol = {"started_utc": started, "finished_utc": datetime.now(timezone.utc).isoformat(),
                "plant": "real_robot.xml", "physics_dt_s": 0.001, "control_dt_s": CONTROL_DT,
                "fixed_initial_pitch_deg": "+/-10 alternating", "pitch_rate_rad_s": 0,
                "wheel_position_rad": 0, "eval_seeds": list(range(args.seeds)),
                "timesteps_per_ppo_training": args.timesteps, "llm_iterations": args.iterations,
                "fixed_reward_checkpoint_reused": bool(args.reuse_fixed),
                "ollama_model": args.ollama_model, "pid_gains": asdict(baseline),
                "llm_pid_gains": asdict(tuned), "best_llm_reward": best,
                "scientific_note": "tzejun policy is standalone PPO, not an LLM/PID residual"}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2))
    print("RESULTS", OUT / "comparison_summary.json", flush=True)


if __name__ == "__main__":
    main()
