"""Workstream-B experiment driver: matched C (fixed reward) vs D (Ollama reward).

Runs each (condition, fine-tuning-seed) sequentially, single-core, into ONE
timestamped parent directory, then aggregates the held-out test metrics across
fine-tuning seeds as a SEPARATE source of variability from the paired
per-state comparison. Ordered so seed-0 of both conditions completes first,
giving an early look while later seeds keep running.

Nothing here runs on hardware. Each run self-contains its baseline, candidates,
reward-design log and C export.
"""
from __future__ import annotations

import argparse
import json
import math
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import finetune

HERE = Path(__file__).resolve().parent


def write_json(p, value):
    p = Path(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def paired_test_stats(parent: Path, conditions, seeds) -> dict:
    """Aggregate per-run held-out test summaries across fine-tuning seeds."""
    agg = {"metric_keys": [], "per_condition": {}, "base_reference": None}
    metric_keys = ("pitch_rms_deg", "speed_rms_m_s", "mean_speed_m_s", "yaw_rms_rad_s",
                   "final_x_m", "max_abs_x_m", "mean_abs_torque_nm", "reversal_rate",
                   "saturation_fraction", "survived", "stationkeeping_success")
    agg["metric_keys"] = list(metric_keys)
    base_vals = {k: [] for k in metric_keys}
    for cond in conditions:
        rows = {k: [] for k in metric_keys}
        losses = []
        beats = []
        for s in seeds:
            rs = parent / f"{cond}_seed{s:02d}" / "run_summary.json"
            if not rs.exists():
                continue
            summ = json.loads(rs.read_text())
            losses.append(summ["selected_validation_loss"])
            beats.append(summ["beats_parent_on_validation"])
            for k in metric_keys:
                sel = summ["test_selected"].get(k)
                if isinstance(sel, dict) and sel.get("mean") is not None:
                    rows[k].append(sel["mean"])
                bas = summ["test_base"].get(k)
                if isinstance(bas, dict) and bas.get("mean") is not None:
                    base_vals[k].append(bas["mean"])

        def summarize(vals):
            v = np.asarray([x for x in vals if x is not None], dtype=float)
            if not len(v):
                return {"mean": None, "std": None, "n": 0}
            return {"mean": float(v.mean()),
                    "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0,
                    "ci95": float(1.96 * v.std(ddof=1) / math.sqrt(len(v))) if len(v) > 1 else 0.0,
                    "n": int(len(v))}

        agg["per_condition"][cond] = {
            "n_seeds": len([1 for s in seeds if (parent / f"{cond}_seed{s:02d}" / "run_summary.json").exists()]),
            "selected_validation_loss": summarize(losses),
            "beats_parent_fraction": float(np.mean(beats)) if beats else None,
            "test_selected_across_seeds": {k: summarize(rows[k]) for k in metric_keys},
        }
    agg["base_reference"] = {k: (float(np.mean(base_vals[k])) if base_vals[k] else None)
                             for k in metric_keys}
    return agg


def make_plots(parent: Path, conditions, seeds):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig_dir = parent / "figures"
    fig_dir.mkdir(exist_ok=True)

    # Reward-weight evolution for the ollama runs (per seed).
    for cond in conditions:
        if cond != "ollama":
            continue
        fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
        plotted = False
        for s in seeds:
            log = parent / f"{cond}_seed{s:02d}" / "reward_design_log.json"
            if not log.exists():
                continue
            hist = json.loads(log.read_text())
            keys = ("angle", "rate", "position", "speed", "yaw", "effort", "slew", "survival")
            for k in keys:
                series = [d["reward_weights_used"][k] for d in hist]
                ax.plot(range(len(series)), series, "o-", alpha=0.7,
                        label=(k if s == seeds[0] else None))
            plotted = True
        if plotted:
            ax.set(xlabel="fine-tuning iteration", ylabel="reward weight",
                   title="Ollama-designed reward-weight evolution (all seeds overlaid)")
            ax.legend(ncol=4, fontsize=8)
            ax.grid(alpha=0.25)
            fig.savefig(fig_dir / "reward_weight_evolution.png", dpi=150)
        plt.close(fig)

    # C vs D held-out test comparison bars for key stationkeeping metrics.
    agg = json.loads((parent / "experiment_B_summary.json").read_text())
    metrics = ["pitch_rms_deg", "speed_rms_m_s", "yaw_rms_rad_s", "mean_abs_torque_nm",
               "reversal_rate", "stationkeeping_success"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    for ax, m in zip(axes.flat, metrics):
        labels, means, errs = [], [], []
        base_ref = agg["base_reference"].get(m)
        for cond in conditions:
            pc = agg["per_condition"].get(cond, {})
            st = pc.get("test_selected_across_seeds", {}).get(m, {})
            if st.get("mean") is not None:
                labels.append(cond)
                means.append(st["mean"])
                errs.append(st.get("ci95", 0.0))
        x = np.arange(len(labels))
        ax.bar(x, means, yerr=errs, capsize=4, color=["#0072B2", "#D55E00"][:len(labels)])
        if base_ref is not None:
            ax.axhline(base_ref, color="k", ls="--", lw=1, label="frozen base")
            ax.legend(fontsize=8)
        ax.set_xticks(x, labels)
        ax.set_title(m)
        ax.grid(alpha=0.25, axis="y")
    fig.suptitle("Held-out test: fixed-reward (C) vs Ollama-reward (D), mean±95%CI across seeds; dashed=frozen base")
    fig.savefig(fig_dir / "C_vs_D_test.png", dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--conditions", nargs="+", default=["fixed", "ollama"])
    ap.add_argument("--iterations", type=int, default=5)
    ap.add_argument("--timesteps", type=int, default=40960)
    ap.add_argument("--ollama-model", default="llama3.2:3b")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    parent = args.out or (HERE / "runs" / ("expB_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")))
    parent.mkdir(parents=True, exist_ok=True)
    # Interleave so seed-0 of every condition finishes before later seeds.
    order = [(s, c) for s in args.seeds for c in args.conditions]
    order.sort(key=lambda sc: (sc[0], args.conditions.index(sc[1])))
    plan = {"parent": str(parent), "order": [{"seed": s, "condition": c} for s, c in order],
            "iterations": args.iterations, "timesteps": args.timesteps,
            "ollama_model": args.ollama_model,
            "started_utc": datetime.now(timezone.utc).isoformat()}
    write_json(parent / "experiment_plan.json", plan)
    print(f"EXPERIMENT_B parent={parent} runs={len(order)}", flush=True)

    results = []
    t0 = time.monotonic()
    for i, (seed, cond) in enumerate(order):
        tag = f"{cond}_seed{seed:02d}"
        out = parent / tag
        run_args = SimpleNamespace(condition=cond, run_seed=seed, iterations=args.iterations,
                                   timesteps=args.timesteps, ollama_model=args.ollama_model,
                                   torch_threads=1, smoke=False, out=out)
        print(f"\n==== RUN {i+1}/{len(order)} {tag} ({time.monotonic()-t0:.0f}s elapsed) ====", flush=True)
        entry = {"tag": tag, "seed": seed, "condition": cond}
        if (out / "run_summary.json").exists():
            entry["status"] = "completed"
            entry["note"] = "already present; skipped (resume)"
            print(f"SKIP {tag}: run_summary.json already present", flush=True)
            results.append(entry)
            write_json(parent / "experiment_runs.json", results)
            continue
        try:
            finetune.run(run_args)
            entry["status"] = "completed"
        except Exception as exc:
            entry["status"] = "failed"
            entry["error"] = repr(exc)
            entry["traceback"] = traceback.format_exc()
            print(f"RUN FAILED {tag}: {exc!r}", flush=True)
        results.append(entry)
        write_json(parent / "experiment_runs.json", results)
        # Incremental aggregate so partial results are usable early.
        try:
            agg = paired_test_stats(parent, args.conditions, args.seeds)
            agg.update(completed_runs=sum(1 for r in results if r["status"] == "completed"),
                       total_runs=len(order), elapsed_s=time.monotonic() - t0)
            write_json(parent / "experiment_B_summary.json", agg)
            make_plots(parent, args.conditions, args.seeds)
        except Exception as exc:
            print(f"aggregate/plot skipped: {exc!r}", flush=True)

    plan.update(finished_utc=datetime.now(timezone.utc).isoformat(),
                wall_seconds=time.monotonic() - t0,
                completed=sum(1 for r in results if r["status"] == "completed"))
    write_json(parent / "experiment_plan.json", plan)
    print(f"\nEXPERIMENT_B DONE completed={plan['completed']}/{len(order)} "
          f"wall_s={plan['wall_seconds']:.0f} parent={parent}", flush=True)


if __name__ == "__main__":
    main()
