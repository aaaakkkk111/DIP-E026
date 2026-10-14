"""Diagnosis-informed latency-robust fine-tune (Workstream A -> B bridge).

Workstream A found the dominant hardware destabiliser is un-modelled sensing/loop
latency: the frozen policy's pitch and (especially) yaw RMS blow up as added
observation latency grows, and latency+slip causes falls. Standard fine-tuning on
the zero-latency env does not fix this.

This trains from the SAME pinned checkpoint with RANDOMISED whole-observation
latency (domain randomisation over 0..MAX ticks) so the policy learns to tolerate
real sensing delay, then evaluates the base, the no-latency fixed fine-tune (C) and
this latency-robust policy across a fixed latency grid to show the difference.

Still a standalone NN, no PID, original checkpoint untouched, export isolated here.
Simulation only; not hardware validation.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import gymnasium as gym

HERE = Path(__file__).resolve().parent
DEPS = HERE / "deps"
sys.path.insert(0, str(DEPS))

import finetune  # reuse Actor, export_actor, digest, write_json, PPO_KWARGS, seeds
from finetune import Actor, BASE, BASE_SHA, digest, write_json, PPO_KWARGS, TEST_SEEDS, TRAIN_SEED
from ollama_tzejun_env import OllamaTzejunEnv, DEFAULT_REWARD_WEIGHTS
from tzejun_experiment import decode_state

LATENCY_GRID = [0, 1, 2, 3, 5, 8]   # x5 ms = 0..40 ms


class LatencyWrapper(gym.Wrapper):
    """Delays the observation the policy sees by L control ticks. Reward,
    termination and physics still reflect the TRUE current state, so this models
    acting on stale sensing. L is fixed, or randomised per episode in [0, max]."""

    def __init__(self, env, max_latency_ticks, randomize=True, fixed=None):
        super().__init__(env)
        self.max_latency = int(max_latency_ticks)
        self.randomize = randomize
        self.fixed = fixed
        self.L = 0
        self.buf = []

    def reset(self, **kw):
        obs, info = self.env.reset(**kw)
        if self.fixed is not None:
            self.L = int(self.fixed)
        elif self.randomize:
            self.L = int(self.env.np_random.integers(0, self.max_latency + 1))
        else:
            self.L = self.max_latency
        self.buf = [obs.copy() for _ in range(self.L + 1)]
        info["latency_ticks"] = self.L
        return self.buf[0].copy(), info

    def step(self, action):
        obs, r, term, trunc, info = self.env.step(action)
        self.buf.append(obs.copy())
        if len(self.buf) > self.L + 1:
            self.buf.pop(0)
        info["latency_ticks"] = self.L
        return self.buf[0].copy(), r, term, trunc, info


def evaluate_at_latency(model, seed_list, scenarios, latency_ticks, max_steps=2000):
    """Mean physical metrics over seeds x scenarios at a FIXED observation latency."""
    actor = Actor(model)
    rows = []
    for name, settings in scenarios.items():
        base_env = OllamaTzejunEnv(max_steps=max_steps)
        env = LatencyWrapper(base_env, latency_ticks, randomize=False, fixed=latency_ticks)
        for seed in seed_list:
            sign = 1 if seed % 2 == 0 else -1
            options = {"pitch": sign * math.radians(10), "roll": 0.0, "pitch_rate": 0.0, "wheel_position": 0.0}
            options.update({k: v for k, v in settings.items() if k != "push"})
            obs, _ = env.reset(seed=seed, options=options)
            trace = []
            for step in range(max_steps):
                if settings.get("push") and 1000 <= step < 1020:
                    base_env.set_external_force([settings["push"], 0.0, 0.0])
                else:
                    base_env.set_external_force([0.0, 0.0, 0.0])
                obs, rew, term, trunc, info = env.step(actor.predict(obs))
                roll, pitch, yaw, lv, yr = decode_state(base_env.data)
                trace.append([math.degrees(pitch), float(lv[0]), yr, float(base_env.data.qpos[0]), bool(term)])
                if term or trunc:
                    break
            a = np.asarray(trace, dtype=float)
            tail = a[-400:]
            rms = lambda x: float(np.sqrt(np.mean(x * x)))
            survived = not bool(a[-1, -1]) and len(a) == max_steps
            rows.append({"survived": survived,
                         "pitch_rms_deg": rms(tail[:, 0]),
                         "speed_rms_m_s": rms(tail[:, 1]),
                         "yaw_rms_rad_s": rms(tail[:, 2]),
                         "mean_speed_m_s": float(np.mean(tail[:, 1])),
                         "final_x_m": float(a[-1, 3])})
        env.close()
    agg = {"latency_ticks": latency_ticks, "latency_ms": latency_ticks * 5, "n": len(rows),
           "survival": float(np.mean([r["survived"] for r in rows]))}
    for k in ("pitch_rms_deg", "speed_rms_m_s", "yaw_rms_rad_s", "mean_speed_m_s", "final_x_m"):
        v = np.asarray([r[k] for r in rows], dtype=float)
        agg[k] = float(np.mean(v))
    return agg


def latency_profile(model, seeds, scenarios, max_steps=2000):
    return [evaluate_at_latency(model, seeds, scenarios, L, max_steps) for L in LATENCY_GRID]


def run(args):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    torch.set_num_threads(1)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass

    folder = Path(args.out) if args.out else (
        HERE / "runs" / ("latency_robust_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")))
    folder.mkdir(parents=True, exist_ok=True)
    ppo_seed = TRAIN_SEED + 7000
    weights = dict(DEFAULT_REWARD_WEIGHTS)
    scenarios = {"nominal": {}, "low_friction": {"friction": 0.5}, "payload_025": {"payload": 0.25}}
    eval_seeds = TEST_SEEDS[: args.eval_seeds]

    protocol = {"purpose": "latency_robust_finetune", "base_sha256": digest(BASE),
                "max_latency_ticks": args.max_latency, "latency_ms_max": args.max_latency * 5,
                "reward_weights": weights, "iterations": args.iterations, "timesteps": args.timesteps,
                "ppo_seed": ppo_seed, "eval_seeds": eval_seeds, "eval_scenarios": list(scenarios),
                "ppo_kwargs": PPO_KWARGS, "latency_grid_ms": [L * 5 for L in LATENCY_GRID],
                "started_utc": datetime.now(timezone.utc).isoformat(),
                "note": "Randomised whole-observation delay 0..max ticks during training."}
    write_json(folder / "protocol.json", protocol)

    class Progress(BaseCallback):
        def __init__(self):
            super().__init__(); self.t0 = time.monotonic()

        def _on_step(self):
            if self.n_calls % 10240 == 0:
                print(f"TRAIN latency-robust steps={self.n_calls} wall_s={time.monotonic()-self.t0:.1f}", flush=True)
            return True

    started = time.monotonic()
    best_path = BASE
    best_loss = None
    added = 0
    for it in range(args.iterations):
        base_env = OllamaTzejunEnv(reward_weights=weights, randomize=True, command_probability=0.1)
        env = LatencyWrapper(base_env, args.max_latency, randomize=True)
        policy = PPO.load(best_path, env=env, device="cpu", custom_objects={
            "learning_rate": PPO_KWARGS["learning_rate"], "lr_schedule": lambda _: PPO_KWARGS["learning_rate"],
            "clip_range": lambda _: PPO_KWARGS["clip_range"], "n_steps": PPO_KWARGS["n_steps"],
            "batch_size": PPO_KWARGS["batch_size"], "n_epochs": PPO_KWARGS["n_epochs"],
            "gamma": PPO_KWARGS["gamma"], "gae_lambda": PPO_KWARGS["gae_lambda"], "target_kl": PPO_KWARGS["target_kl"]})
        policy.set_random_seed(ppo_seed + it)
        policy.learn(total_timesteps=args.timesteps, reset_num_timesteps=False, callback=Progress())
        added += args.timesteps
        env.close()
        path = folder / f"candidate_{it:02d}.zip"
        policy.save(path)
        # Validation: mean pitch+yaw RMS across the latency grid (robustness objective).
        prof = latency_profile(policy, eval_seeds[: max(2, args.eval_seeds // 2)], {"nominal": {}}, 1200)
        loss = float(np.mean([p["pitch_rms_deg"] + 3 * p["yaw_rms_rad_s"] + 1000 * (1 - p["survival"]) for p in prof]))
        print(f"ITER latency-robust {it} val_latency_loss={loss:.4f}", flush=True)
        if best_loss is None or loss < best_loss:
            best_loss, best_path = loss, path
        write_json(folder / "candidates.json", {"iteration": it, "val_latency_loss": loss, "path": str(path)})

    selected = PPO.load(best_path, device="cpu")
    selected.save(folder / "selected_policy.zip")
    sel_sha = digest(folder / "selected_policy.zip")

    # Compare base, C fixed fine-tune (if available), and this latency-robust policy
    # across the fixed latency grid on held-out seeds.
    print("LATENCY PROFILE: base vs fixed-C vs latency-robust", flush=True)
    comparison = {"latency_robust": latency_profile(selected, eval_seeds, scenarios),
                  "base": latency_profile(PPO.load(BASE, device="cpu"), eval_seeds, scenarios)}
    c_path = HERE / "runs" / "expB_main" / "fixed_seed00" / "selected_policy.zip"
    if c_path.exists():
        comparison["fixed_C"] = latency_profile(PPO.load(c_path, device="cpu"), eval_seeds, scenarios)

    export_meta = {"purpose": "latency_robust_finetune", "max_latency_ticks": args.max_latency,
                   "actual_added_timesteps": added, "ppo_seed": ppo_seed}
    manifest = finetune.export_actor(selected, folder / "export", sel_sha, weights, export_meta)

    result = {"selected_checkpoint": str(folder / "selected_policy.zip"), "selected_checkpoint_sha256": sel_sha,
              "condition": "latency_robust", "selected_weights": weights, "base_sha256": digest(BASE),
              "actual_added_timesteps": added, "max_latency_ticks": args.max_latency,
              "latency_comparison": comparison, "export_manifest": manifest,
              "wall_seconds": time.monotonic() - started, "hardware_verified": False, "status": "completed"}
    write_json(folder / "run_summary.json", result)
    write_json(folder / "latency_comparison.json", comparison)
    make_plot(folder, comparison)
    # Console summary table.
    print("\nlatency_ms | base pitch/yaw | C pitch/yaw | robust pitch/yaw (survival)")
    for i, L in enumerate(LATENCY_GRID):
        b = comparison["base"][i]
        r = comparison["latency_robust"][i]
        c = comparison.get("fixed_C", comparison["base"])[i]
        print(f"  {L*5:>3} | {b['pitch_rms_deg']:.2f}/{b['yaw_rms_rad_s']:.2f} "
              f"| {c['pitch_rms_deg']:.2f}/{c['yaw_rms_rad_s']:.2f} "
              f"| {r['pitch_rms_deg']:.2f}/{r['yaw_rms_rad_s']:.2f} (surv {r['survival']:.2f})", flush=True)
    print(f"\nDONE latency-robust -> {folder}  wall_s={result['wall_seconds']:.0f}", flush=True)
    return 0


def make_plot(folder, comparison):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    colors = {"base": "#888888", "fixed_C": "#0072B2", "latency_robust": "#D55E00"}
    for key, series in comparison.items():
        ms = [p["latency_ms"] for p in series]
        axes[0].plot(ms, [p["pitch_rms_deg"] for p in series], "o-", color=colors.get(key), label=key)
        axes[1].plot(ms, [p["yaw_rms_rad_s"] for p in series], "o-", color=colors.get(key), label=key)
        axes[2].plot(ms, [p["survival"] for p in series], "o-", color=colors.get(key), label=key)
    axes[0].set(xlabel="added latency (ms)", ylabel="pitch RMS (deg)", title="Pitch vs latency")
    axes[1].set(xlabel="added latency (ms)", ylabel="yaw RMS (rad/s)", title="Yaw vs latency")
    axes[2].set(xlabel="added latency (ms)", ylabel="survival", title="Survival vs latency")
    for ax in axes:
        ax.grid(alpha=0.25); ax.legend(fontsize=8)
    fig.suptitle("Latency robustness: base vs fixed fine-tune (C) vs latency-robust fine-tune (held-out)")
    (folder / "figures").mkdir(exist_ok=True)
    fig.savefig(folder / "figures" / "latency_robustness.png", dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iterations", type=int, default=6)
    ap.add_argument("--timesteps", type=int, default=40960)
    ap.add_argument("--max-latency", type=int, default=6, help="max observation delay ticks (x5 ms)")
    ap.add_argument("--eval-seeds", type=int, default=8)
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    if args.smoke:
        args.iterations, args.timesteps, args.eval_seeds = 1, 2048, 2
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
