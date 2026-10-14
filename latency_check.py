"""Evaluate any set of policies across the fixed latency grid (held-out seeds).

Answers: does a given fine-tuned policy generalise to sensing latency, or does it
overfit to the zero-latency training env? Reuses finetune_latency.latency_profile.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "deps"))

import finetune_latency as fl
from finetune import BASE, TEST_SEEDS


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--scenario", default="nominal")
    ap.add_argument("--out", type=Path, default=HERE / "runs" / "latency_check.json")
    args = ap.parse_args()
    import torch
    from stable_baselines3 import PPO
    torch.set_num_threads(1)

    seeds = TEST_SEEDS[: args.seeds]
    scn = {args.scenario: ({} if args.scenario == "nominal" else {"friction": 0.5})}
    runs = HERE / "runs"
    policies = {"base": BASE}
    for name, rel in [("fixed_C", "expB_main/fixed_seed00/selected_policy.zip"),
                      ("ollama_D", "expB_main/ollama_seed00/selected_policy.zip"),
                      ("latency_robust", "latency_robust_main/selected_policy.zip")]:
        p = runs / rel
        if p.exists():
            policies[name] = p

    out = {"seeds": seeds, "scenario": args.scenario, "profiles": {}}
    print(f"latency_ms | " + " | ".join(f"{k} pitch/yaw/surv" for k in policies))
    grid = {k: fl.latency_profile(PPO.load(str(p), device="cpu"), seeds, scn) for k, p in policies.items()}
    out["profiles"] = grid
    for i, L in enumerate(fl.LATENCY_GRID):
        cells = []
        for k in policies:
            r = grid[k][i]
            cells.append(f"{r['pitch_rms_deg']:5.2f}/{r['yaw_rms_rad_s']:4.2f}/{r['survival']:.2f}")
        print(f"  {L*5:>3}      | " + " | ".join(cells), flush=True)
    args.out.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
