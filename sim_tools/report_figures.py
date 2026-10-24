"""Report figures for runs 11-18: training progress, the calm-vs-push trade-off,
the diagnostics, payload, and simulator against car.

    python sim_tools/report_figures.py        -> report_figures/*.png

Training curves are read from session-logs/*_progress.csv. Every other number
is a measured result copied from the session log named beside it (simulator:
2 deg slack, 5/10/15 ms delay, 2-3 seeds; ranges are min-max over those).
Current draw is not plotted: the car has no current sensor.
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "report_figures")

# dataviz reference palette, light mode; categorical slots in fixed order.
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
TICK_S = 0.005                                    # one control tick, s

plt.rcParams.update({
    "font.family": ["Segoe UI", "DejaVu Sans"], "font.size": 10,
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False, "legend.labelcolor": INK2, "lines.linewidth": 2,
})


def save(fig, name):
    fig.tight_layout()
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print("wrote", os.path.relpath(path, ROOT))


def progress(run_file, column):
    """(steps in millions, seconds) for one progress.csv column."""
    xs, ys = [], []
    with open(os.path.join(ROOT, "session-logs", run_file), newline="") as f:
        for r in csv.DictReader(f):
            if r.get(column):
                xs.append(float(r["time/total_timesteps"]) / 1e6)
                ys.append(float(r[column]) * TICK_S)
    return xs, ys


def rolling(ys, n):
    return [sum(ys[max(0, i - n + 1):i + 1]) / len(ys[max(0, i - n + 1):i + 1]) for i in range(len(ys))]


def end_label(ax, x, y, text, color):
    ax.annotate(text, (x, y), xytext=(6, 0), textcoords="offset points", va="center",
                color=INK2, fontsize=9)
    ax.plot([x], [y], "o", color=color, ms=5)


RUNS = [("Run 12", "train_real_robot_RUN12_measured_motor_pushes_progress.csv"),
        ("Run 14", "train_real_robot_RUN14_scratch_pushes_wobble15_progress.csv"),
        ("Run 15", "train_real_robot_RUN15_push3N_penaltypause_progress.csv"),
        ("Run 16", "train_real_robot_RUN16_reversal05_slack6_progress.csv"),
        ("Run 17", "train_real_robot_RUN17_reversal03_slack6_progress.csv")]


def fig_training():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharex=True)
    panels = [("eval/mean_ep_length", 3, "Eval episode length (no pushes)"),
              ("rollout/ep_len_mean", 25, "Training episode length (with pushes)")]
    for ax, (col, n, title) in zip(axes, panels):
        for (name, f), color in zip(RUNS, C):
            x, y = progress(f, col)
            y = rolling(y, n)
            ax.plot(x, y, color=color, label=name)
        ax.set_title(title)
        ax.set_xlabel("training steps (millions)")
        ax.set_ylabel("survival per episode (s, max 10)")
        ax.set_ylim(0, 10.5)
        ax.set_xlim(0, 34)
    axes[0].legend(loc="lower right")
    fig.suptitle("Training progress: how long the car stays up", x=0.01, ha="left",
                 fontweight="bold", color=INK)
    save(fig, "fig1_training_progress.png")


# (name, wobble min, max at 2 deg, 3 N pushes survived of 18); sources: session logs
# 2026-10-08-run12..., 2026-10-14-run14-desktop, 2026-10-18-run15-desktop,
# 2026-10-21-run16-desktop; diag C is a 10M diagnostic (2026-10-21).
TRADEOFF = [("Stock PID", 6.6, 6.9, 17), ("Run 11", 11, 16, 0), ("Run 12", 26, 35, 14),
            ("Run 13", 24, 33, 18), ("Run 14", 17.4, 27.8, 5), ("Run 15", 19.5, 24.5, 18),
            ("Run 16", 14.6, 29.6, 0), ("Run 17", 14.5, 28.0, 0), ("Diag C (no payload)", 4.6, 10.2, 0)]
# Runs sharing a push score are spread sideways so their ranges don't overlap.
X_SHIFT = {"Run 11": -0.45, "Run 16": -0.15, "Run 17": 0.15, "Diag C (no payload)": 0.45,
           "Run 13": -0.2, "Run 15": 0.2}
LABEL_DX = {"Run 11": -40, "Run 16": -40, "Run 13": -48}


def fig_tradeoff():
    fig, ax = plt.subplots(figsize=(7.5, 5))
    ax.fill_between([15, 18.8], 0, 10, color=GRID, alpha=0.6, lw=0)
    ax.text(16.9, 1.2, "target\n(PID level)", ha="center", color=INK2, fontsize=9)
    for name, lo, hi, push in TRADEOFF:
        push += X_SHIFT.get(name, 0.0)
        color = C[1] if name == "Stock PID" else C[0]
        ax.plot([push, push], [lo, hi], color=color, lw=2, solid_capstyle="round")
        ax.plot([push], [(lo + hi) / 2], "o", color=color, ms=7, mec=SURFACE, mew=1.5)
        ax.annotate(name, (push, (lo + hi) / 2), xytext=(LABEL_DX.get(name, 8), 0),
                    textcoords="offset points", va="center", color=INK2, fontsize=9)
    ax.set_xlim(-2.5, 20)
    ax.set_xticks(range(0, 19, 3))
    ax.set_ylim(0, 38)
    ax.set_xlabel("3 N pushes survived (of 18)")
    ax.set_ylabel("standing wobble, sim 2° slack (deg/s)")
    ax.set_title("Calm against push-proof: no RL run reaches the PID's corner")
    save(fig, "fig2_calm_vs_push.png")


# (run, [(checkpoint M, wobble min, max)]); same session logs as above.
CHECKPOINTS = [("Run 14", [(11.75, 10, 13), (18.75, 18, 27), (30, 17.4, 27.8)]),
               ("Run 15", [(11, 9.8, 11.9), (17.75, 14.2, 18.3), (30, 19.5, 24.5)]),
               ("Run 16", [(10, 2.6, 3.1), (16.5, 4.3, 10.8), (30, 14.6, 29.6)])]


def fig_checkpoints():
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    ax.axvline(15, color=AXIS, lw=1, ls="--")
    ax.text(15.3, 31, "curriculum at full strength\n(pushes, commands, payload)", color=MUTED, fontsize=8.5)
    ax.axhline(6.8, color=C[1], lw=1, ls=":")
    ax.text(30.3, 7.3, "stock PID", color=INK2, fontsize=9, va="bottom")
    for (name, pts), color in zip(CHECKPOINTS, [C[0], C[2], C[3]]):
        x = [p[0] for p in pts]
        mid = [(p[1] + p[2]) / 2 for p in pts]
        ax.fill_between(x, [p[1] for p in pts], [p[2] for p in pts], color=color, alpha=0.15, lw=0)
        ax.plot(x, mid, "-o", color=color, ms=6, label=name)
    ax.set_xlim(8, 33)
    ax.set_ylim(0, 35)
    ax.set_xlabel("training steps (millions)")
    ax.set_ylabel("standing wobble, sim 2° slack (deg/s)")
    ax.set_title("Calm while the curriculum ramps up, shaky once it is at full strength")
    ax.legend(loc="upper left")
    save(fig, "fig3_calm_then_lost.png")


# 10M diagnostics (2026-10-21): (name, wobble 2 deg min, max, reversals/s min, max)
DIAG = [("D: control", 19.5, 30.1, 20.5, 28.6), ("A: pushes 50 %", 18.7, 27.0, 20.2, 24.4),
        ("B: commands 50 %", 14.4, 22.7, 18.9, 27.5), ("C: no payload", 4.6, 10.2, 5.0, 12.5)]


def fig_diagnostics():
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    ys = range(len(DIAG))[::-1]
    for ax, (i_lo, i_hi, title, ref) in zip(axes, [(1, 2, "Standing wobble, 2° slack (deg/s)", 6.8),
                                                   (3, 4, "Motor reversals (per s per wheel)", None)]):
        for y, row in zip(ys, DIAG):
            color = C[0] if row[0].startswith("C") else MUTED
            ax.barh(y, row[i_hi] - row[i_lo], left=row[i_lo], height=0.45, color=color)
            ax.text(row[i_hi] + 0.6, y, f"{row[i_lo]:g}–{row[i_hi]:g}", va="center", color=INK2, fontsize=9)
        if ref:
            ax.axvline(ref, color=C[1], lw=1, ls=":")
            ax.text(ref + 0.4, 0.5, "stock PID", color=INK2, fontsize=8.5, va="center")
        ax.set_title(title)
        ax.set_xlim(0, 36)
        ax.grid(axis="y", visible=False)
    axes[0].set_yticks(list(ys), [r[0] for r in DIAG])
    fig.suptitle("Diagnostics (10M steps each): only removing the payload calms the policy",
                 x=0.01, ha="left", fontweight="bold", color=INK)
    save(fig, "fig4_diagnostics.png")


KG = [0, 0.5, 1, 2, 4]
# Payload sweeps (2026-10-21): standing (wobble min, max) and driving +/-0.25 m/s.
STAND = {"Run 15 (trained 0–1 kg)": [(19.5, 24.5), (10.1, 11.8), (8.2, 10.0), (8.6, 10.2), (7.4, 8.3)],
         "Diag D (trained 0–1 kg)": [(21.4, 30.1), (11.0, 16.1), (8.1, 10.2), (6.3, 9.2), (6.4, 8.8)],
         "Diag C (no payload)": [(7.0, 10.2), (4.4, 8.6), (6.5, 7.7), (6.7, 7.8), (6.9, 8.2)]}
DRIVE_KG = [0, 1, 2, 4]
DRIVE_HELD = {"Run 15 (trained 0–1 kg)": [3, 3, 3, 3], "Diag C (no payload)": [3, 3, 1, 0]}


def fig_payload():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    ax = axes[0]
    for (name, rng), color in zip(STAND.items(), C):
        mid = [(a + b) / 2 for a, b in rng]
        ax.fill_between(KG, [a for a, _ in rng], [b for _, b in rng], color=color, alpha=0.15, lw=0)
        ax.plot(KG, mid, "-o", color=color, ms=6, label=name)
    ax.set_xlabel("payload (kg), 10.5 cm above the axle")
    ax.set_ylabel("standing wobble, sim 2° slack (deg/s)")
    ax.set_title("Standing: all hold 0–4 kg; extra mass calms them")
    ax.set_ylim(0, 32)
    ax.legend(loc="upper right")
    ax = axes[1]
    w = 0.38
    for k, ((name, held), color) in enumerate(zip(DRIVE_HELD.items(), [C[0], C[2]])):
        xs = [i + (k - 0.5) * w for i in range(len(DRIVE_KG))]
        ax.bar(xs, held, width=w - 0.04, color=color, label=name)
        for x, h in zip(xs, held):
            ax.text(x, h + 0.08, f"{h}/3", ha="center", color=INK2, fontsize=9)
    ax.set_xticks(range(len(DRIVE_KG)), [f"{k} kg" for k in DRIVE_KG])
    ax.set_ylim(0, 4.2)
    ax.set_yticks([0, 1, 2, 3])
    ax.set_ylabel("runs that stayed up (of 3)")
    ax.set_title("Driving ±0.25 m/s: no-payload training fails at 2 kg+")
    ax.grid(axis="x", visible=False)
    ax.legend(loc="upper center", ncol=2)
    save(fig, "fig5_payload.png")


# Simulator against the car (sessions 2026-10-08/14, ~2 deg; 2026-10-20, ~4 deg, 11.8 V).
VALID = [("2026-10-08/14 car (≈2° slack)", [("Stock PID", (6.6, 6.9), (7.6, 7.6)),
                                             ("Run 11", (13.5, 21), (12, 22)),
                                             ("Run 13", (23, 37), (26, 36))]),
         ("2026-10-20 car (≈4° slack, worse day)", [("Stock PID", (10.4, 10.4), (11.7, 11.7)),
                                                    ("Run 13", (43, 50), (51, 51)),
                                                    ("Run 15", (41, 42), (43, 49))])]


def fig_validation():
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharex=True)
    for ax, (title, rows) in zip(axes, VALID):
        for i, (name, sim, car) in enumerate(rows[::-1]):
            for dy, rng, color, lab in ((0.14, sim, C[0], "simulator"), (-0.14, car, C[1], "car")):
                lo, hi = rng
                ax.plot([lo, hi] if hi > lo else [lo - 0.3, hi + 0.3], [i + dy] * 2, color=color, lw=6,
                        solid_capstyle="round", label=lab if i == 0 else None)
            ax.text(max(sim[1], car[1]) + 1.2, i, name, va="center", color=INK2, fontsize=9)
        ax.set_yticks([])
        ax.set_title(title)
        ax.set_xlabel("standing wobble (deg/s)")
        ax.set_xlim(0, 65)
        ax.set_ylim(-0.6, len(rows) - 0.4)
    axes[0].legend(loc="lower right")
    fig.suptitle("The simulator predicts the car", x=0.01, ha="left", fontweight="bold", color=INK)
    save(fig, "fig6_sim_vs_car.png")


# Every run since the measured motor: (name, standing wobble sim 2 deg, car mid-run or None,
# empty 3 N x 0.1 s pushes survived of 18 in sim). From PROJECT_OVERVIEW.md and the session logs;
# car = the run's best-conditions day (run 15 only ran on the worse 2026-10-20 car).
ALL_RUNS = [("Stock PID", (6.6, 6.9), (7.6, 11.7), 17),
            ("Run 11", (11, 16), (12, 22), 0), ("Run 12", (26, 35), None, 14),
            ("Run 13", (24, 33), (26, 36), 18), ("Run 14", (17, 28), None, 5),
            ("Run 15", (20, 25), (43, 49), 18), ("Run 16", (15, 30), None, 0),
            ("Run 17", (15, 28), None, 0), ("Run 18", (14.6, 17.8), None, 0),
            ("Run 19", (26, 30), None, 0), ("Run 20", (18, 33), None, 0),
            ("Run 21", (6.6, 8.3), (2.8, 4.3), 0), ("Run 22", (4.0, 5.4), (4.0, 4.3), 0),
            ("Run 23", (3.7, 4.2), (3.7, 5.5), 0)]


def fig_all_runs():
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.6), sharey=True, gridspec_kw={"width_ratios": [2.2, 1]})
    rows = ALL_RUNS[::-1]                                      # first run at the top
    ax = axes[0]
    pid = ALL_RUNS[0][1]
    ax.axvspan(*pid, color=GRID, lw=0)
    ax.text(pid[1] + 0.6, len(rows) - 0.45, "stock PID, sim", color=MUTED, fontsize=8, va="center")
    for i, (_, sim, car, _) in enumerate(rows):
        for dy, rng, color, lab in ((0.16, sim, C[0], "simulator, 2° slack"), (-0.16, car, C[1], "car, mid-run")):
            if rng:
                ax.plot(rng, [i + dy] * 2, color=color, lw=6, solid_capstyle="round",
                        label=lab if i == len(rows) - 2 else None)
    i15 = [r[0] for r in rows].index("Run 15")
    ax.text(42.5, i15 - 0.5, "car only on the worse 2026-10-20 day", color=MUTED, fontsize=8, va="center")
    ax.set_yticks(range(len(rows)), [r[0] for r in rows])
    ax.set_xlabel("standing wobble, pitch-rate sd (deg/s)  ·  lower is calmer")
    ax.set_xlim(0, 52)
    ax.set_title("Standing still: runs 21–23 reach the stock PID's calm")
    ax.grid(axis="y", visible=False)
    ax.legend(loc="lower right")
    ax = axes[1]
    for i, r in enumerate(rows):
        ax.barh(i, r[3], height=0.55, color=C[0] if r[3] else GRID)
        ax.text(r[3] + 0.4, i, f"{r[3]}/18", va="center", color=INK2, fontsize=8)
    ax.set_xlim(0, 22)
    ax.set_xlabel("empty 3 N pushes survived, sim (of 18)")
    ax.set_title("…but no calm run survives a 3 N push")
    ax.grid(axis="y", visible=False)
    save(fig, "fig7_all_runs.png")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for f in (fig_training, fig_tradeoff, fig_checkpoints, fig_diagnostics, fig_payload, fig_validation,
              fig_all_runs):
        f()
