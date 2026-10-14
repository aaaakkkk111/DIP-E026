"""Build the Workstream-B report: matched fixed (C) vs Ollama (D) reward fine-tuning.

Reads runs/expB_main/experiment_B_summary.json plus each run's run_summary.json and
reward_design_log.json. Reports held-out C-vs-D tradeoffs with across-seed variability,
the LLM reward-design contribution (accept / documented-normalization / fallback counts),
reward-weight evolution, failed/regressed candidates, and limitations.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def fmt(s, d=3):
    if s is None:
        return "n/a"
    if isinstance(s, dict):
        m = s.get("mean")
        if m is None:
            return "n/a"
        ci = s.get("ci95")
        return f"{m:.{d}f}" + (f" ±{ci:.{d}f}" if ci else "")
    return f"{s:.{d}f}"


def llm_contribution(parent: Path):
    """Summarize the reward-design statuses across all ollama runs."""
    counts = {"accepted": 0, "accepted_after_documented_normalization": 0,
              "fallback_keep_previous": 0, "fixed_defaults": 0, "other": 0}
    examples = []
    for log in sorted(parent.glob("ollama_seed*/reward_design_log.json")):
        hist = json.loads(log.read_text())
        for d in hist:
            st = d.get("status", "other")
            counts[st] = counts.get(st, 0) + 1
            if d.get("normalization") and len(examples) < 3:
                nz = d["normalization"]
                examples.append({"run": log.parent.name, "iter": d["iteration"],
                                 "raw_sum": round(nz["raw_sum"], 3),
                                 "raw": {k: round(v, 3) for k, v in nz["raw_weights_from_llm"].items()},
                                 "normalized": {k: round(v, 3) for k, v in nz["normalized_weights"].items()}})
    return counts, examples


def write_report(parent: Path):
    summ = json.loads((parent / "experiment_B_summary.json").read_text())
    runs = json.loads((parent / "experiment_runs.json").read_text()) if (parent / "experiment_runs.json").exists() else []
    counts, examples = llm_contribution(parent)
    base = summ["base_reference"]
    pc = summ["per_condition"]
    keys = ["stationkeeping_success", "survived", "pitch_rms_deg", "speed_rms_m_s",
            "mean_speed_m_s", "yaw_rms_rad_s", "mean_abs_torque_nm", "reversal_rate"]

    L = ["# Workstream B — Ollama-designed reward fine-tuning vs fixed-reward control", "",
         "**Executed PPO fine-tuning from the pinned tzejun checkpoint.** Ollama (`llama3.2:3b`) "
         "designed the reward weights offboard; a matched fixed-reward condition isolates its "
         "contribution. A reward change alone does not alter a deployed actor — PPO changed the "
         "network. Simulation is not hardware validation.", "",
         f"Completed runs: {summ.get('completed_runs')}/{summ.get('total_runs')} "
         f"({', '.join(r['tag'] for r in runs if r.get('status')=='completed')}).", "",
         "## LLM reward-design contribution", "",
         "Per-iteration reward-proposal status across all Ollama runs:", "",
         "| status | count |", "|---|---|"]
    for k, v in counts.items():
        if v:
            L.append(f"| {k} | {v} |")
    L += ["", "`accepted` = valid and summed to 1 directly; "
          "`accepted_after_documented_normalization` = valid shape, sum rescaled (logged transform, "
          "not raw output); `fallback_keep_previous` = unrecoverable, previous weights reused "
          "(does NOT count as an LLM contribution).", ""]
    if examples:
        L += ["Example logged normalization transforms (raw LLM proposal → rescaled):", ""]
        for e in examples:
            L.append(f"- `{e['run']}` iter {e['iter']}: raw sum {e['raw_sum']} → normalized. "
                     f"raw={e['raw']} → norm={e['normalized']}")
        L.append("")

    L += ["## Held-out test: C (fixed) vs D (Ollama), mean ± 95% CI across fine-tuning seeds", "",
          "Lower is better except survival/stationkeeping. Dashed = frozen base reference.", "",
          "| metric | frozen base | C fixed | D ollama |", "|---|---|---|---|"]
    for m in keys:
        c = pc.get("fixed", {}).get("test_selected_across_seeds", {}).get(m)
        d = pc.get("ollama", {}).get("test_selected_across_seeds", {}).get(m)
        L.append(f"| {m} | {fmt(base.get(m))} | {fmt(c)} | {fmt(d)} |")
    L += ["",
          f"- Fixed (C): {pc.get('fixed',{}).get('n_seeds',0)} seeds, "
          f"beats-parent fraction {fmt(pc.get('fixed',{}).get('beats_parent_fraction'))}, "
          f"selection loss {fmt(pc.get('fixed',{}).get('selected_validation_loss'))}.",
          f"- Ollama (D): {pc.get('ollama',{}).get('n_seeds',0)} seeds, "
          f"beats-parent fraction {fmt(pc.get('ollama',{}).get('beats_parent_fraction'))}, "
          f"selection loss {fmt(pc.get('ollama',{}).get('selected_validation_loss'))}.", ""]

    # Per-run detail incl. selected weights and regressions.
    L += ["## Per-run detail", "", "| run | beats parent? | sel. iter | val loss | selected weights |",
          "|---|---|---|---|---|"]
    for rs in sorted(parent.glob("*_seed*/run_summary.json")):
        d = json.loads(rs.read_text())
        w = d.get("selected_weights", {})
        wtxt = " ".join(f"{k}={v:.2f}" for k, v in w.items()) if w else "(defaults)"
        L.append(f"| {rs.parent.name} | {d.get('beats_parent_on_validation')} | "
                 f"{d.get('selected_iteration')} | {fmt(d.get('selected_validation_loss'))} | {wtxt} |")

    L += ["", "## Interpretation", "",
          "- Report multi-metric tradeoffs, not pitch alone. A candidate that lowers pitch RMS while "
          "adding yaw drift, motor chatter or saturation is not better.",
          "- Returns under different reward definitions are NOT comparable; selection uses a "
          "reward-independent physical loss and the held-out test uses matched physical metrics.",
          "- `reversal_rate` near 1.0 indicates per-tick action sign flips (chatter) inherited from "
          "the frozen policy; watch whether D raises the `slew` weight to reduce it.", "",
          "## Figures", "", "![reward evolution](figures/reward_weight_evolution.png)", "",
          "![C vs D](figures/C_vs_D_test.png)", "",
          "## Limitations", "",
          "- Fine-tuning used the reference MuJoCo observation pipeline (no sensing-latency model); "
          "Workstream A shows latency is the dominant hardware destabiliser, so a latency-robust "
          "variant is the recommended follow-up before hardware.",
          "- Evaluation seeds are held out from training/validation, but a frozen policy evaluated "
          "over seeds is not the same as independent training-seed variability (reported separately).",
          "- No policy was flashed or tested on hardware in this project."]
    (parent / "REPORT.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("wrote", parent / "REPORT.md")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--parent", type=Path, default=HERE / "runs" / "expB_main")
    args = ap.parse_args()
    write_report(args.parent)


if __name__ == "__main__":
    main()
