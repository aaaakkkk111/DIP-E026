"""PPO fine-tuning of the pinned tzejun checkpoint with LLM-WRITTEN reward CODE.

Two matched conditions, exactly as in the ollama project, so the contribution of
LLM-authored reward code is isolated:
  * condition 'fixed'   (C): the documented eight-term weighted reward, no LLM.
  * condition 'mistral' (D): each iteration mistral WRITES a reward-function body
                             (mistral_reward_code.design_reward); it is AST-allowlist
                             validated + self-tested, then used via CodeRewardTzejunEnv.

Shared across C and D: parent checkpoint, architecture, PPO hyper-params, per-iteration
timesteps, iteration count, training seed, disjoint validation/test seeds and scenarios.
Only the reward SOURCE differs. The immutable fall penalty (-25) is applied by the env
outside any LLM code. Candidate selection uses finetune.evaluate's reward-INDEPENDENT
physical loss, so a policy cannot win by inflating its own learned return. Export is
redirected into this project; nothing here runs on the robot.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

import finetune as base  # reuse evaluate/export_actor/Actor/digest/write_json/constants
import mistral_reward_code as rc

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "deps"))


def metrics_summary(m: dict) -> str:
    def g(k):
        v = m.get(k)
        return v["mean"] if isinstance(v, dict) else v
    keys = ("pitch_rms_deg", "pitch_rate_note", "yaw_rms_rad_s", "speed_rms_m_s",
            "mean_speed_m_s", "reversal_rate", "stationkeeping_success", "survived")
    out = {k: g(k) for k in keys if m.get(k) is not None}
    out["selection_loss"] = m.get("selection_loss_lower_is_better")
    return json.dumps(out)


def run(args) -> int:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    import gymnasium, mujoco
    from ollama_tzejun_env import OllamaTzejunEnv, DEFAULT_REWARD_WEIGHTS
    from codereward_env import CodeRewardTzejunEnv

    import torch
    torch.set_num_threads(max(1, args.threads))
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass

    tag = f"{args.condition}_seed{args.run_seed:02d}"
    folder = Path(args.out or (HERE / "runs" / (tag + "_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))))
    folder.mkdir(parents=True, exist_ok=True)
    ppo_seed = base.TRAIN_SEED + 1000 * args.run_seed

    val_seeds = base.VALIDATION_SEEDS[:2] if args.smoke else base.VALIDATION_SEEDS
    test_seeds = base.TEST_SEEDS[:2] if args.smoke else base.TEST_SEEDS
    val_scn = {k: base.SCENARIOS[k] for k in (("nominal",) if args.smoke else base.VALIDATION_SCENARIOS)}
    test_scn = {k: base.SCENARIOS[k] for k in (("nominal", "low_friction") if args.smoke else base.SCENARIOS)}
    val_steps = 300 if args.smoke else 2000
    test_steps = 300 if args.smoke else 2000

    started = time.monotonic()
    model0 = PPO.load(base.BASE, device="cpu")
    model0.set_random_seed(ppo_seed)
    inherited = int(model0.num_timesteps)
    base_metrics = base.evaluate(model0, folder / "validation_base", val_seeds, val_scn, val_steps)
    base.write_json(folder / "baseline_validation.json", base_metrics)

    best_path = base.BASE
    best_score = base_metrics["selection_loss_lower_is_better"]
    prev_code = rc.DEFAULT_REWARD_CODE
    history, candidates, added = [], [], 0

    class Progress(BaseCallback):
        def __init__(self, it): super().__init__(); self.it = it; self.t0 = time.monotonic()
        def _on_step(self):
            if self.n_calls % 10240 == 0:
                print(f"TRAIN {tag} iter={self.it} added={self.n_calls} "
                      f"wall_s={time.monotonic()-self.t0:.1f}", flush=True)
            return True

    for it in range(args.iterations):
        if args.condition == "mistral":
            decision = rc.design_reward(args.model, prev_code, metrics_summary(base_metrics), it)
            code_obj = decision.pop("reward_code_obj")
            reward_fn = rc.make_reward_fn(code_obj)
            env = CodeRewardTzejunEnv(reward_fn=reward_fn, randomize=True, command_probability=0.1)
            reward_descr = {"reward_code": decision["code_accepted"]}
        else:
            decision = {"iteration": it, "status": "fixed_defaults", "llm_contributed": False,
                        "code_accepted": None}
            env = OllamaTzejunEnv(reward_weights=DEFAULT_REWARD_WEIGHTS, randomize=True,
                                  command_probability=0.1)
            reward_descr = {"reward_weights": dict(DEFAULT_REWARD_WEIGHTS)}

        policy = PPO.load(best_path, env=env, device="cpu", custom_objects={
            "learning_rate": base.PPO_KWARGS["learning_rate"],
            "lr_schedule": lambda _: base.PPO_KWARGS["learning_rate"],
            "clip_range": lambda _: base.PPO_KWARGS["clip_range"],
            "n_steps": base.PPO_KWARGS["n_steps"], "batch_size": base.PPO_KWARGS["batch_size"],
            "n_epochs": base.PPO_KWARGS["n_epochs"], "gamma": base.PPO_KWARGS["gamma"],
            "gae_lambda": base.PPO_KWARGS["gae_lambda"], "target_kl": base.PPO_KWARGS["target_kl"]})
        policy.set_random_seed(ppo_seed + it)
        before = base.Actor(policy).layers
        policy.learn(total_timesteps=args.timesteps, reset_num_timesteps=False, callback=Progress(it))
        added += args.timesteps
        path = folder / f"candidate_{it:02d}.zip"
        policy.save(path); env.close()
        delta = max(float(np.max(np.abs(before[j][0] - base.Actor(policy).layers[j][0]))) for j in range(3))
        if delta == 0:
            raise RuntimeError("no actor tensor changed; not a genuine fine-tune")
        metrics = base.evaluate(policy, folder / f"validation_{it:02d}", val_seeds, val_scn, val_steps)
        accepted = metrics["selection_loss_lower_is_better"] < best_score
        decision.update(checkpoint=str(path), checkpoint_sha256=base.digest(path),
                        reward=reward_descr, validation_selection_loss=metrics["selection_loss_lower_is_better"],
                        actor_max_weight_change=delta, accepted_as_best=accepted,
                        added_timesteps_so_far=added, elapsed_s=time.monotonic() - started)
        history.append(decision)
        candidates.append({"loss": metrics["selection_loss_lower_is_better"], "path": str(path),
                           "reward": reward_descr, "iteration": it, "accepted_as_best": accepted})
        if accepted:
            best_path, best_score, base_metrics = path, metrics["selection_loss_lower_is_better"], metrics
            if args.condition == "mistral":
                prev_code = decision["code_accepted"]
        base.write_json(folder / "reward_design_log.json", history)
        base.write_json(folder / "candidates.json", candidates)
        print(f"ITER {tag} {it} status={decision.get('status')} delta={delta:.4g} "
              f"val_loss={metrics['selection_loss_lower_is_better']:.5g} accepted={accepted}", flush=True)

    best_cand = min(candidates, key=lambda c: c["loss"])
    selected = PPO.load(best_cand["path"], device="cpu")
    selected.save(folder / "selected_policy.zip")
    selected_sha = base.digest(folder / "selected_policy.zip")
    beats_parent = best_cand["loss"] < base.evaluate(
        PPO.load(base.BASE, device="cpu"), folder / "parent_recheck", val_seeds, val_scn, val_steps
    )["selection_loss_lower_is_better"]

    print(f"FINAL TEST {tag}: base vs selected on {len(test_seeds)} held-out seeds", flush=True)
    base_test = base.evaluate(model0, folder / "test_base", test_seeds, test_scn, test_steps)
    tuned_test = base.evaluate(selected, folder / "test_selected", test_seeds, test_scn, test_steps)

    run_meta = {"condition": args.condition, "run_seed_index": args.run_seed, "ppo_seed": ppo_seed,
                "inherited_num_timesteps": inherited, "actual_added_timesteps": added,
                "llm_model": args.model if args.condition == "mistral" else None,
                "reward_mechanism": "llm_written_code" if args.condition == "mistral" else "fixed_weighted_terms"}
    export = base.export_actor(selected, folder / "export", selected_sha, best_cand["reward"], run_meta)

    result = {"condition": args.condition, "run_seed_index": args.run_seed, "run": str(folder),
              "selected_checkpoint": str(folder / "selected_policy.zip"),
              "selected_checkpoint_sha256": selected_sha, "selected_reward": best_cand["reward"],
              "selected_validation_loss": best_cand["loss"], "selected_iteration": best_cand["iteration"],
              "beats_parent_on_validation": bool(beats_parent), "base_sha256": base.digest(base.BASE),
              "inherited_num_timesteps": inherited, "actual_added_timesteps": added,
              "n_iterations": args.iterations, "timesteps_per_iteration": args.timesteps,
              "llm_contribution_iterations": sum(1 for d in history if d.get("llm_contributed")),
              "test_base": base_test, "test_selected": tuned_test, "candidates": candidates,
              "export_manifest": export, "wall_seconds": time.monotonic() - started,
              "hardware_verified": False, "status": "completed",
              "finished_utc": datetime.now(timezone.utc).isoformat()}
    base.write_json(folder / "run_summary.json", result)
    print(json.dumps({k: result[k] for k in ("condition", "run_seed_index", "selected_validation_loss",
          "beats_parent_on_validation", "actual_added_timesteps", "llm_contribution_iterations",
          "wall_seconds")}, indent=2), flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--condition", choices=("fixed", "mistral"), default="mistral")
    ap.add_argument("--model", default="mistral:latest")
    ap.add_argument("--run-seed", type=int, default=0, dest="run_seed")
    ap.add_argument("--iterations", type=int, default=5)
    ap.add_argument("--timesteps", type=int, default=40960)
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--smoke", action="store_true")
    args = ap.parse_args()
    if args.smoke:
        args.iterations = min(args.iterations, 1)
        args.timesteps = min(args.timesteps, 2048)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
