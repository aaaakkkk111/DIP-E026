"""PPO fine-tuning of the pinned tzejun checkpoint (Workstream B).

Two matched conditions isolate the LLM contribution:
  * condition 'fixed'  (C): reward weights are FIXED to the documented defaults
                            for every iteration; no LLM is called.
  * condition 'ollama' (D): reward weights are proposed each iteration by the
                            local Ollama model via ollama_reward.propose_reward.

C and D share: parent checkpoint, actor/critic architecture, PPO hyper-params,
per-iteration timesteps, iteration count, training seed, and the disjoint
validation/test seed sets and scenarios. Only the reward-weight SOURCE differs.

Key corrections vs. the historical ollama_tzejun_train.py:
  * Export is redirected INTO this project (export/), never the old Keil tree.
  * Train / reward-selection-validation / final-test seeds are DISJOINT.
  * Actual ADDED PPO timesteps are recorded separately from the inherited
    checkpoint counter (which carries the original training's millions of steps).
  * Candidate selection uses a reward-INDEPENDENT physical loss, so a policy
    cannot win by inflating its own (differently-defined) return.
  * Rejected / regressed candidates are preserved and clearly marked; a policy
    that does not beat its parent on validation is not silently deployed.

Nothing here runs on the robot. A reward-weight change does not alter a
deployed actor; only PPO changes actor parameters, and only the C export +
a fresh Keil build + the gated hardware procedure can reach hardware.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEPS = HERE / "deps"
sys.path.insert(0, str(DEPS))

import ollama_reward as reward_designer  # noqa: E402  (local module next to this file)

BASE = DEPS / "models" / "best_real" / "best_model.zip"
XML = DEPS / "real_robot.xml"
BASE_SHA = "a850b7cd7b0fbb9bdec828439d4bec6effa589099acb2405acdc6fa4f879511d"

# Disjoint seed sets (correcting the historical val 0-5 / test 0-14 overlap).
TRAIN_SEED = 7
VALIDATION_SEEDS = [101, 102, 103, 104, 105, 106]
TEST_SEEDS = list(range(201, 216))  # 15 held-out states

VALIDATION_SCENARIOS = ("nominal", "low_friction", "payload_025")
# Full test suite (same scenario settings used by the env in deps).
SCENARIOS = {
    "nominal": {}, "low_friction": {"friction": 0.5}, "high_friction": {"friction": 1.2},
    "payload_025": {"payload": 0.25}, "payload_050": {"payload": 0.5},
    "push_forward": {"push": 0.5}, "push_backward": {"push": -0.5},
}

PPO_KWARGS = dict(learning_rate=3e-5, n_steps=2048, batch_size=64, n_epochs=10,
                  gamma=0.99, gae_lambda=0.95, clip_range=0.1, target_kl=0.01)


def digest(p) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write_json(p, value) -> None:
    p = Path(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(p.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(tmp, p)


class Actor:
    """Numpy mirror of the SB3 policy actor (34->64->64->2), used for fast eval
    and for the C export. Reads tensors straight from the loaded PPO policy."""

    def __init__(self, model):
        state = model.policy.state_dict()
        self.layers = []
        for name in ("mlp_extractor.policy_net.0", "mlp_extractor.policy_net.2", "action_net"):
            self.layers.append((state[name + ".weight"].detach().cpu().numpy().copy(),
                                state[name + ".bias"].detach().cpu().numpy().copy()))
        assert [w.shape for w, _ in self.layers] == [(64, 34), (64, 64), (2, 64)]

    def predict(self, obs):
        x = np.asarray(obs, dtype=np.float32)
        for i, (w, b) in enumerate(self.layers):
            x = w @ x + b
            x = np.tanh(x) if i < 2 else np.clip(x, -1.0, 1.0)
        return x.astype(np.float32)


def evaluate(model, out, seed_list, scenarios, max_steps=2000):
    """Deterministic frozen-policy evaluation over an explicit seed list.

    Returns a summary dict with a reward-INDEPENDENT selection loss. State is
    decoded from the MuJoCo plant, never inferred from the learned reward.
    """
    from ollama_tzejun_env import OllamaTzejunEnv
    from tzejun_experiment import decode_state

    actor = Actor(model)
    rows = []
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for name, settings in scenarios.items():
        env = OllamaTzejunEnv(max_steps=max_steps)
        for seed in seed_list:
            sign = 1 if seed % 2 == 0 else -1
            options = {"pitch": sign * math.radians(10), "roll": 0.0,
                       "pitch_rate": 0.0, "wheel_position": 0.0}
            options.update({k: v for k, v in settings.items() if k != "push"})
            obs, _ = env.reset(seed=seed, options=options)
            trace = []
            for step in range(max_steps):
                if settings.get("push") and 1000 <= step < 1020:
                    env.set_external_force([settings["push"], 0.0, 0.0])
                else:
                    env.set_external_force([0.0, 0.0, 0.0])
                action = actor.predict(obs)
                obs, rew, terminated, truncated, info = env.step(action)
                roll, pitch, yaw, lv, yr = decode_state(env.data)
                torque = env.data.actuator_force[:2].copy()
                trace.append([float(env.data.time), math.degrees(pitch), math.degrees(roll),
                              float(env.data.qvel[4]), float(lv[0]), yr, float(env.data.qpos[0]),
                              *torque, *action, float(rew), bool(terminated)])
                if terminated or truncated:
                    break
            a = np.asarray(trace, dtype=float)
            tail = a[-400:]
            outside = np.flatnonzero(np.abs(a[:, 1]) >= 2.0)
            idx = int(outside[-1] + 1) if len(outside) else 0
            settling = float(a[idx, 0]) if idx < len(a) - 100 else None
            rms = lambda x: float(np.sqrt(np.mean(x * x)))
            survived = not bool(a[-1, -1]) and len(a) == max_steps
            row = {"scenario": name, "seed": seed, "initial_pitch_deg": sign * 10.0,
                   "survived": survived, "settling_s": settling,
                   "pitch_rms_deg": rms(tail[:, 1]), "roll_rms_deg": rms(tail[:, 2]),
                   "speed_rms_m_s": rms(tail[:, 4]), "yaw_rms_rad_s": rms(tail[:, 5]),
                   "mean_speed_m_s": float(np.mean(tail[:, 4])),
                   "final_x_m": float(a[-1, 6]), "max_abs_x_m": float(np.max(np.abs(a[:, 6]))),
                   "mean_abs_torque_nm": float(np.mean(np.abs(a[:, 7:9]))),
                   "rms_torque_nm": rms(a[:, 7:9]), "torque_variance": float(np.var(a[:, 7:9])),
                   "saturation_fraction": float(np.mean(np.max(np.abs(a[:, 9:11]), axis=1) >= 0.99)),
                   "reversal_rate": float(np.mean(np.abs(np.diff(np.sign(a[:, 9]))) > 0)),
                   "overshoot_deg": float(max(0.0, np.max(-sign * a[:, 1]))),
                   "return": float(a[:, 11].sum()), "steps": len(a)}
            row["stationkeeping_success"] = bool(
                survived and settling is not None and settling < 1.5
                and row["pitch_rms_deg"] < 1.0 and row["speed_rms_m_s"] < 0.02
                and abs(row["mean_speed_m_s"]) < 0.005 and row["yaw_rms_rad_s"] < 0.05
                and abs(row["final_x_m"]) < 0.25)
            rows.append(row)
            stem = out / f"{name}_seed_{seed:03d}"
            np.savez_compressed(stem.with_suffix(".npz"), data=a, columns=np.asarray([
                "t", "pitch_deg", "roll_deg", "pitch_rate", "speed", "yaw_rate", "x",
                "torque_left", "torque_right", "action_left", "action_right", "reward", "fallen"]))
            if seed == seed_list[0]:
                np.savetxt(stem.with_suffix(".csv"), a, delimiter=",", comments="", header=(
                    "t,pitch_deg,roll_deg,pitch_rate,speed,yaw_rate,x,torque_left,torque_right,"
                    "action_left,action_right,reward,fallen"))
        env.close()
    write_json(out / "metrics.json", rows)
    # Reward-independent physical selection loss (lower is better). A fall costs
    # 1000, so an inflated learned return can never select a failing policy.
    score = float(np.mean([1000 * (not r["survived"]) + r["pitch_rms_deg"]
                           + 5 * r["speed_rms_m_s"] + 3 * r["yaw_rms_rad_s"]
                           + 2 * abs(r["final_x_m"]) + 0.2 * (r["settling_s"] or 10)
                           + 0.1 * r["mean_abs_torque_nm"] for r in rows]))
    summary = {"episodes": len(rows), "seeds": list(seed_list),
               "scenarios": list(scenarios), "selection_loss_lower_is_better": score}
    for key in ("pitch_rms_deg", "roll_rms_deg", "speed_rms_m_s", "yaw_rms_rad_s",
                "mean_speed_m_s", "final_x_m", "max_abs_x_m", "mean_abs_torque_nm",
                "rms_torque_nm", "torque_variance", "saturation_fraction", "reversal_rate",
                "overshoot_deg", "settling_s", "survived", "stationkeeping_success"):
        vals = np.asarray([r[key] for r in rows if r[key] is not None], dtype=float)
        summary[key] = {"mean": float(vals.mean()) if len(vals) else None,
                        "ci95": float(1.96 * vals.std(ddof=1) / math.sqrt(len(vals))) if len(vals) > 1 else 0.0,
                        "n": int(len(vals))}
    write_json(out / "summary.json", summary)
    return summary


def c_array(name, array) -> str:
    data = np.asarray(array, dtype=np.float32).ravel()
    parts = []
    for v in data:
        token = format(float(v), ".9g")
        if "." not in token and "e" not in token.lower():
            token += "."
        parts.append(token + "f")
    body = ",\n".join("    " + ", ".join(parts[i:i + 8]) for i in range(0, len(parts), 8))
    return f"static const float {name}[{len(data)}] = {{\n{body}\n}};\n"


def export_actor(model, out_dir, sha, reward_weights, run_meta) -> dict:
    """Write policy_weights.h + policy_testvectors.h + manifest INTO this project.
    Never touches keil_ollama_tzejun or any original firmware."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    actor = Actor(model)
    n_obs = int(actor.layers[0][0].shape[1])
    n_h1 = int(actor.layers[0][0].shape[0])
    n_h2 = int(actor.layers[1][0].shape[0])
    n_act = int(actor.layers[2][0].shape[0])
    text = ("#ifndef POLICY_WEIGHTS_H\n#define POLICY_WEIGHTS_H\n"
            f"#define POLICY_N_OBS   {n_obs}\n#define POLICY_N_H1    {n_h1}\n"
            f"#define POLICY_N_H2    {n_h2}\n#define POLICY_N_ACT   {n_act}\n"
            "#define POLICY_ACTION_LIMIT 1.000000f\n"
            f'#define POLICY_CHECKPOINT_SHA256 "{sha}"\n'
            "/* Ollama/fixed-reward fine-tuned tzejun; standalone NN; no PID residual; "
            "original pinned checkpoint retained unmodified. */\n")
    for i, (w, b) in enumerate(actor.layers):
        text += c_array(f"policy_w{i}", w) + c_array(f"policy_b{i}", b)
    text += "#endif\n"
    (out_dir / "policy_weights.h").write_text(text, encoding="utf-8")

    rng = np.random.default_rng(7)
    obs = rng.uniform(-1, 1, (32, 34)).astype(np.float32)
    expected, _ = model.predict(obs, deterministic=True)
    actual = np.stack([actor.predict(o) for o in obs])
    error = float(np.max(np.abs(expected - actual)))
    if error > 2e-5:
        raise RuntimeError(f"SB3/export parity failed: {error}")
    vectors = ("#ifndef POLICY_TESTVECTORS_H\n#define POLICY_TESTVECTORS_H\n"
               f"#define POLICY_N_TESTS {len(obs)}\n"
               + c_array("policy_test_obs", obs) + c_array("policy_test_expected", expected)
               + "#endif\n")
    (out_dir / "policy_testvectors.h").write_text(vectors, encoding="utf-8")
    manifest = {"purpose": "finetuned_tzejun_actor_export_isolated_to_this_project",
                "exported_actor_header_sha256": digest(out_dir / "policy_weights.h"),
                "selected_checkpoint_sha256": sha, "base_checkpoint_sha256": BASE_SHA,
                "actor": "34-64(tanh)-64(tanh)-2(linear clipped) standalone",
                "actor_parameters": int(sum(w.size + b.size for w, b in actor.layers)),
                "reward_weights": reward_weights, "sb3_export_max_abs_action_error": error,
                "written_into": str(out_dir), "touches_original_keil": False,
                "hardware_verified": False, "requires_fresh_keil_build": True, **run_meta}
    write_json(out_dir / "export_manifest.json", manifest)
    print(f"EXPORTED actor sha={sha[:16]} parity={error:.3g} -> {out_dir}", flush=True)
    return manifest


def run(args) -> int:
    import torch
    import stable_baselines3
    import mujoco
    import gymnasium
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from ollama_tzejun_env import OllamaTzejunEnv, DEFAULT_REWARD_WEIGHTS

    torch.set_num_threads(args.torch_threads)
    # set_num_interop_threads may be called only once per process (and only
    # before parallel work). When run() is invoked repeatedly in one process
    # (the orchestrator), later calls raise; that is harmless, so guard it.
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass

    if digest(BASE) != BASE_SHA:
        raise RuntimeError("base checkpoint is not the pinned tzejun checkpoint")

    if args.condition == "ollama":
        models = reward_designer.available_models()
        if args.ollama_model not in models:
            raise RuntimeError(f"Ollama model {args.ollama_model} not available; have {models}")

    # Seed/scenario/step budgets (shrunk for a fast end-to-end smoke test).
    val_seeds = VALIDATION_SEEDS[:2] if args.smoke else VALIDATION_SEEDS
    test_seeds = TEST_SEEDS[:2] if args.smoke else TEST_SEEDS
    val_scn = ("nominal",) if args.smoke else VALIDATION_SCENARIOS
    test_scn = {k: SCENARIOS[k] for k in (("nominal", "low_friction") if args.smoke else SCENARIOS)}
    val_steps = 400 if args.smoke else 1600
    test_steps = 500 if args.smoke else 2000

    tag = f"{args.condition}_seed{args.run_seed:02d}" + ("_smoke" if args.smoke else "")
    folder = Path(args.out) if args.out else (
        HERE / "runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") / tag)
    folder.mkdir(parents=True, exist_ok=False)

    ppo_seed = TRAIN_SEED + 1000 * args.run_seed  # distinct per fine-tuning seed
    protocol = {
        "condition": args.condition, "run_seed_index": args.run_seed, "ppo_seed": ppo_seed,
        "base_checkpoint": str(BASE), "base_sha256": digest(BASE), "xml_sha256": digest(XML),
        "iterations": args.iterations, "timesteps_per_iteration": args.timesteps,
        "planned_added_timesteps": args.iterations * args.timesteps,
        "smoke": bool(args.smoke),
        "train_seed": TRAIN_SEED, "validation_seeds": val_seeds, "test_seeds": test_seeds,
        "validation_scenarios": list(val_scn), "test_scenarios": list(test_scn),
        "seed_disjointness": {
            "train_vs_validation": sorted(set([TRAIN_SEED]) & set(val_seeds)) == [],
            "train_vs_test": sorted(set([TRAIN_SEED]) & set(test_seeds)) == [],
            "validation_vs_test": sorted(set(val_seeds) & set(test_seeds)) == []},
        "ppo_kwargs": PPO_KWARGS, "immutable_fall_penalty": -25,
        "ollama_model": args.ollama_model if args.condition == "ollama" else None,
        "reward_source": "fixed_documented_defaults" if args.condition == "fixed"
                         else "ollama_proposed_with_logged_normalization",
        "versions": {"torch": torch.__version__, "sb3": stable_baselines3.__version__,
                     "mujoco": mujoco.__version__, "gymnasium": gymnasium.__version__,
                     "numpy": np.__version__, "python": sys.version.split()[0]},
        "started_utc": datetime.now(timezone.utc).isoformat()}
    write_json(folder / "protocol.json", protocol)

    class Progress(BaseCallback):
        def __init__(self, it):
            super().__init__(); self.it = it; self.t0 = time.monotonic()

        def _on_step(self):
            if self.n_calls % 10240 == 0:
                print(f"TRAIN {tag} iter={self.it} added_steps={self.n_calls} "
                      f"wall_s={time.monotonic() - self.t0:.1f}", flush=True)
            return True

    started = time.monotonic()
    base = PPO.load(BASE, device="cpu")
    base.set_random_seed(ppo_seed)
    inherited_counter = int(base.num_timesteps)
    valid_sc = {k: SCENARIOS[k] for k in val_scn}
    print(f"BASE {digest(BASE)[:16]} inherited_num_timesteps={inherited_counter}", flush=True)

    base_metrics = evaluate(base, folder / "validation_base", val_seeds, valid_sc, val_steps)
    write_json(folder / "baseline_validation.json", base_metrics)
    best_path = BASE
    best_score = base_metrics["selection_loss_lower_is_better"]
    weights = dict(DEFAULT_REWARD_WEIGHTS)
    history = []
    candidates = []
    added_steps = 0

    for iteration in range(args.iterations):
        if args.condition == "ollama":
            decision = reward_designer.propose_reward(
                args.ollama_model, weights, base_metrics, iteration)
            weights = decision["weights_accepted"]
        else:
            decision = {"iteration": iteration, "status": "fixed_defaults",
                        "weights_accepted": dict(DEFAULT_REWARD_WEIGHTS),
                        "llm_contributed": False}
            weights = dict(DEFAULT_REWARD_WEIGHTS)

        env = OllamaTzejunEnv(reward_weights=weights, randomize=True, command_probability=0.1)
        policy = PPO.load(best_path, env=env, device="cpu", custom_objects={
            "learning_rate": PPO_KWARGS["learning_rate"],
            "lr_schedule": lambda _: PPO_KWARGS["learning_rate"],
            "clip_range": lambda _: PPO_KWARGS["clip_range"],
            "n_steps": PPO_KWARGS["n_steps"], "batch_size": PPO_KWARGS["batch_size"],
            "n_epochs": PPO_KWARGS["n_epochs"], "gamma": PPO_KWARGS["gamma"],
            "gae_lambda": PPO_KWARGS["gae_lambda"], "target_kl": PPO_KWARGS["target_kl"]})
        policy.set_random_seed(ppo_seed + iteration)
        before = Actor(policy).layers
        policy.learn(total_timesteps=args.timesteps, reset_num_timesteps=False,
                     callback=Progress(iteration))
        added_steps += args.timesteps
        path = folder / f"candidate_{iteration:02d}.zip"
        policy.save(path)
        env.close()
        delta = max(float(np.max(np.abs(before[j][0] - Actor(policy).layers[j][0])))
                    for j in range(3))
        if delta == 0:
            raise RuntimeError("no actor tensor changed; not a genuine fine-tune")
        metrics = evaluate(policy, folder / f"validation_{iteration:02d}", val_seeds, valid_sc, val_steps)
        accepted = metrics["selection_loss_lower_is_better"] < best_score
        decision.update(checkpoint=str(path), checkpoint_sha256=digest(path),
                        reward_weights_used=weights, validation=metrics,
                        actor_max_weight_change=delta, accepted_as_best=accepted,
                        fine_tune_parent=str(best_path), added_timesteps_so_far=added_steps,
                        elapsed_s=time.monotonic() - started)
        history.append(decision)
        candidates.append({"loss": metrics["selection_loss_lower_is_better"], "path": str(path),
                           "weights": dict(weights), "iteration": iteration,
                           "accepted_as_best": accepted})
        if accepted:
            best_path = path
            best_score = metrics["selection_loss_lower_is_better"]
            base_metrics = metrics  # feed the improved metrics back to the LLM next round
        write_json(folder / "reward_design_log.json", history)
        write_json(folder / "candidates.json", candidates)
        print(f"ITER {tag} {iteration} status={decision.get('status')} "
              f"delta={delta:.4g} val_loss={metrics['selection_loss_lower_is_better']:.5g} "
              f"accepted={accepted}", flush=True)

    # Select best candidate by validation loss; deliver it even if it regressed
    # vs. the parent, clearly marked. Never auto-prefer a worse-than-parent policy.
    best_cand = min(candidates, key=lambda c: c["loss"])
    beats_parent = best_cand["loss"] < base_metrics_initial_loss(folder)
    selected_model = PPO.load(best_cand["path"], device="cpu")
    selected_model.save(folder / "selected_policy.zip")
    selected_sha = digest(folder / "selected_policy.zip")

    # Final matched test on HELD-OUT seeds, base vs selected.
    print(f"FINAL TEST {tag}: base vs selected on {len(test_seeds)} held-out seeds", flush=True)
    base_test = evaluate(base, folder / "test_base", test_seeds, test_scn, test_steps)
    tuned_test = evaluate(selected_model, folder / "test_selected", test_seeds, test_scn, test_steps)

    run_meta = {"condition": args.condition, "run_seed_index": args.run_seed,
                "ppo_seed": ppo_seed, "inherited_num_timesteps": inherited_counter,
                "actual_added_timesteps": added_steps,
                "note_on_counter": ("PPO's internal num_timesteps includes the original "
                                    "training; actual_added_timesteps is the only NEW training.")}
    export_manifest = export_actor(selected_model, folder / "export", selected_sha,
                                   best_cand["weights"], run_meta)

    result = {"condition": args.condition, "run_seed_index": args.run_seed, "run": str(folder),
              "selected_checkpoint": str(folder / "selected_policy.zip"),
              "selected_checkpoint_sha256": selected_sha,
              "selected_weights": best_cand["weights"],
              "selected_validation_loss": best_cand["loss"],
              "selected_iteration": best_cand["iteration"],
              "beats_parent_on_validation": bool(beats_parent),
              "base_sha256": digest(BASE), "inherited_num_timesteps": inherited_counter,
              "actual_added_timesteps": added_steps,
              "n_iterations": args.iterations, "timesteps_per_iteration": args.timesteps,
              "llm_contribution_iterations": sum(1 for d in history if d.get("llm_contributed")),
              "test_base": base_test, "test_selected": tuned_test,
              "candidates": candidates, "export_manifest": export_manifest,
              "wall_seconds": time.monotonic() - started, "hardware_verified": False,
              "status": "completed", "finished_utc": datetime.now(timezone.utc).isoformat()}
    write_json(folder / "run_summary.json", result)
    protocol.update(finished_utc=result["finished_utc"], actual_added_timesteps=added_steps,
                    wall_seconds=result["wall_seconds"])
    write_json(folder / "protocol.json", protocol)
    print(json.dumps({k: result[k] for k in (
        "condition", "run_seed_index", "selected_validation_loss", "beats_parent_on_validation",
        "actual_added_timesteps", "llm_contribution_iterations", "wall_seconds")},
        indent=2), flush=True)
    return 0


def base_metrics_initial_loss(folder) -> float:
    return json.loads((Path(folder) / "baseline_validation.json").read_text())[
        "selection_loss_lower_is_better"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--condition", choices=("fixed", "ollama"), required=True)
    ap.add_argument("--run-seed", type=int, default=0, help="fine-tuning seed index (0,1,2,...)")
    ap.add_argument("--iterations", type=int, default=5)
    ap.add_argument("--timesteps", type=int, default=40960)
    ap.add_argument("--ollama-model", default="llama3.2:3b")
    ap.add_argument("--torch-threads", type=int, default=1)
    ap.add_argument("--smoke", action="store_true", help="fast end-to-end pipeline check")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    return run(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
