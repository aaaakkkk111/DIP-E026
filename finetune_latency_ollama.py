"""Synthesis condition: Ollama-designed reward + latency domain randomisation.

Combines the two wins found separately:
  * the Ollama-designed reward (D) produced the best low/moderate-latency policy
    and improved yaw, but still fell at the extreme 40 ms;
  * latency domain randomisation guaranteed survival across the whole latency
    grid, but did not improve yaw.

Here the Ollama loop REFLECTS on latency-stressed validation metrics (pitch and
yaw RMS and survival measured across a latency grid) and proposes reward weights,
while PPO trains with randomised whole-observation latency. The goal is D's
quality with latency-robust survival, specifically attacking yaw-under-latency.

Standalone NN, no PID, pinned checkpoint untouched, export isolated here.
Simulation only; not hardware validation.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "deps"))

import ollama_reward as rd
import finetune
import finetune_latency as fl
from finetune import Actor, BASE, digest, write_json, PPO_KWARGS, TRAIN_SEED, TEST_SEEDS
from finetune_latency import LatencyWrapper, latency_profile, LATENCY_GRID
from ollama_tzejun_env import OllamaTzejunEnv, DEFAULT_REWARD_WEIGHTS

VAL_LATENCY_TICKS = [0, 3, 6]   # 0,15,30 ms for per-iteration validation


def stress_summary(profile):
    """Compact latency-stressed metrics for the LLM to reflect on."""
    return {
        "pitch_rms_deg_by_latency_ms": {str(p["latency_ms"]): round(p["pitch_rms_deg"], 3) for p in profile},
        "yaw_rms_rad_s_by_latency_ms": {str(p["latency_ms"]): round(p["yaw_rms_rad_s"], 3) for p in profile},
        "survival_by_latency_ms": {str(p["latency_ms"]): round(p["survival"], 2) for p in profile},
        "worst_yaw_rms_rad_s": round(max(p["yaw_rms_rad_s"] for p in profile), 3),
        "worst_pitch_rms_deg": round(max(p["pitch_rms_deg"] for p in profile), 3),
        "min_survival": round(min(p["survival"] for p in profile), 2),
        "guidance": ("yaw drift and pitch shaking grow with sensing latency; reward should "
                     "prioritise upright balance, zero idle yaw/heading drift and smooth, "
                     "non-chattering action so the policy tolerates delayed sensing"),
    }


def val_profile(model, seeds):
    prof = [fl.evaluate_at_latency(model, seeds, {"nominal": {}}, L, 1200) for L in VAL_LATENCY_TICKS]
    loss = float(np.mean([p["pitch_rms_deg"] + 3 * p["yaw_rms_rad_s"] + 1000 * (1 - p["survival"]) for p in prof]))
    return prof, loss


def run(args):
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    torch.set_num_threads(1)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
    models = rd.available_models()
    if args.ollama_model not in models:
        raise SystemExit(f"Ollama model {args.ollama_model} unavailable; have {models}")

    folder = Path(args.out) if args.out else (
        HERE / "runs" / ("latency_ollama_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")))
    folder.mkdir(parents=True, exist_ok=True)
    ppo_seed = TRAIN_SEED + 9000
    val_seeds = TEST_SEEDS[: args.eval_seeds // 2 or 2]
    test_seeds = TEST_SEEDS[: args.eval_seeds]

    class Progress(BaseCallback):
        def __init__(self):
            super().__init__(); self.t0 = time.monotonic()

        def _on_step(self):
            if self.n_calls % 10240 == 0:
                print(f"TRAIN syn steps={self.n_calls} wall_s={time.monotonic()-self.t0:.1f}", flush=True)
            return True

    started = time.monotonic()
    base = PPO.load(BASE, device="cpu")
    base_prof, base_loss = val_profile(base, val_seeds)
    stressed = stress_summary(base_prof)
    write_json(folder / "baseline_latency_validation.json", {"profile": base_prof, "loss": base_loss})

    protocol = {"purpose": "ollama_reward_plus_latency_randomization", "base_sha256": digest(BASE),
                "max_latency_ticks": args.max_latency, "val_latency_ticks": VAL_LATENCY_TICKS,
                "iterations": args.iterations, "timesteps": args.timesteps, "ppo_seed": ppo_seed,
                "val_seeds": val_seeds, "test_seeds": test_seeds, "ollama_model": args.ollama_model,
                "ppo_kwargs": PPO_KWARGS, "started_utc": datetime.now(timezone.utc).isoformat()}
    write_json(folder / "protocol.json", protocol)

    weights = dict(DEFAULT_REWARD_WEIGHTS)
    best_path, best_loss = BASE, base_loss
    history, added = [], 0
    for it in range(args.iterations):
        decision = rd.propose_reward(args.ollama_model, weights, stressed, it)
        weights = decision["weights_accepted"]
        base_env = OllamaTzejunEnv(reward_weights=weights, randomize=True, command_probability=0.1)
        env = LatencyWrapper(base_env, args.max_latency, randomize=True)
        policy = PPO.load(best_path, env=env, device="cpu", custom_objects={
            "learning_rate": PPO_KWARGS["learning_rate"], "lr_schedule": lambda _: PPO_KWARGS["learning_rate"],
            "clip_range": lambda _: PPO_KWARGS["clip_range"], "n_steps": PPO_KWARGS["n_steps"],
            "batch_size": PPO_KWARGS["batch_size"], "n_epochs": PPO_KWARGS["n_epochs"],
            "gamma": PPO_KWARGS["gamma"], "gae_lambda": PPO_KWARGS["gae_lambda"], "target_kl": PPO_KWARGS["target_kl"]})
        policy.set_random_seed(ppo_seed + it)
        before = Actor(policy).layers
        policy.learn(total_timesteps=args.timesteps, reset_num_timesteps=False, callback=Progress())
        added += args.timesteps
        env.close()
        path = folder / f"candidate_{it:02d}.zip"
        policy.save(path)
        delta = max(float(np.max(np.abs(before[j][0] - Actor(policy).layers[j][0]))) for j in range(3))
        prof, loss = val_profile(policy, val_seeds)
        stressed = stress_summary(prof)
        accepted = loss < best_loss
        decision.update(reward_weights_used=weights, actor_max_weight_change=delta,
                        latency_val_profile=prof, latency_val_loss=loss, accepted_as_best=accepted,
                        added_timesteps_so_far=added)
        history.append(decision)
        if accepted:
            best_path, best_loss = path, loss
        write_json(folder / "reward_design_log.json", history)
        print(f"ITER syn {it} status={decision.get('status')} delta={delta:.4g} "
              f"stressed_loss={loss:.4f} accepted={accepted} weights={ {k:round(v,2) for k,v in weights.items()} }",
              flush=True)

    selected = PPO.load(best_path, device="cpu")
    selected.save(folder / "selected_policy.zip")
    sel_sha = digest(folder / "selected_policy.zip")

    print("FINAL latency grid: base vs C vs D vs latency_robust vs synthesis", flush=True)
    scn = {"nominal": {}, "low_friction": {"friction": 0.5}, "payload_025": {"payload": 0.25}}
    comparison = {"synthesis": latency_profile(selected, test_seeds, scn),
                  "base": latency_profile(base, test_seeds, scn)}
    for name, rel in [("fixed_C", "expB_main/fixed_seed00/selected_policy.zip"),
                      ("ollama_D", "expB_main/ollama_seed00/selected_policy.zip"),
                      ("latency_robust", "latency_robust_main/selected_policy.zip")]:
        p = HERE / "runs" / rel
        if p.exists():
            comparison[name] = latency_profile(PPO.load(str(p), device="cpu"), test_seeds, scn)

    manifest = finetune.export_actor(selected, folder / "export", sel_sha, weights,
                                     {"purpose": "ollama_latency_synthesis", "actual_added_timesteps": added,
                                      "ppo_seed": ppo_seed, "max_latency_ticks": args.max_latency})
    result = {"condition": "ollama_latency_synthesis", "selected_checkpoint": str(folder / "selected_policy.zip"),
              "selected_checkpoint_sha256": sel_sha, "selected_weights": weights, "base_sha256": digest(BASE),
              "actual_added_timesteps": added, "max_latency_ticks": args.max_latency,
              "llm_contribution_iterations": sum(1 for d in history if d.get("llm_contributed")),
              "latency_comparison": comparison, "export_manifest": manifest,
              "wall_seconds": time.monotonic() - started, "hardware_verified": False, "status": "completed"}
    write_json(folder / "run_summary.json", result)
    write_json(folder / "latency_comparison.json", comparison)
    fl.make_plot(folder, {k: comparison[k] for k in ("base", "latency_robust", "synthesis") if k in comparison})

    print("\nlatency_ms | " + " | ".join(f"{k} pitch/yaw/surv" for k in comparison))
    for i, L in enumerate(LATENCY_GRID):
        cells = [f"{comparison[k][i]['pitch_rms_deg']:5.2f}/{comparison[k][i]['yaw_rms_rad_s']:4.2f}/{comparison[k][i]['survival']:.2f}" for k in comparison]
        print(f"  {L*5:>3}      | " + " | ".join(cells), flush=True)
    print(f"\nDONE synthesis -> {folder} wall_s={result['wall_seconds']:.0f}", flush=True)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--iterations", type=int, default=5)
    ap.add_argument("--timesteps", type=int, default=40960)
    ap.add_argument("--max-latency", type=int, default=6)
    ap.add_argument("--eval-seeds", type=int, default=8)
    ap.add_argument("--ollama-model", default="llama3.2:3b")
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    if args.smoke:
        args.iterations, args.timesteps, args.eval_seeds = 1, 2048, 2
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
