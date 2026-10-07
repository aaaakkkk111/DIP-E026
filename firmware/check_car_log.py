"""Check a mode-28 car-test log against what run 8 should do.

Reads the serial log that the team's rl_trim_helper.py writes
(rl_serial_log.txt) and, for every run the car reported, prints the checks
from Step 10 of README-STM32-DEPLOYMENT.md: did it stay up, is the output
frozen, are there impossible speed readings, did the control loop keep 200 Hz,
how often the PWM flips sign, and how fast it crept.

    python firmware/check_car_log.py path/to/rl_serial_log.txt
    python firmware/check_car_log.py path/to/rl_serial_log.txt --last 5 --plot

The helper appends to the same file every session, so either move the old log
aside before a test or use --last. Standard library only; --plot needs
matplotlib and saves one PNG per run next to the log.

Log lines (USART1, as printed by run8_dropin/rl_mode.c), each prefixed with the
PC time by the helper:
  D <ms> <state> <pitch deg> <v m/s> <L> <R> <zero deg>       10 Hz, state 2 = running
  R <run ms> <reason> <max |pitch| deg>                         a run ended
  E <k> <pitch*100> <pitch rate*100 deg/s> <v mm/s> <L> <R>    last 0.6 s at 200 Hz
  R end
  T <isr> <stock> <rl> <net> <max gap> <late> <ticks>           us, maxima over the run
"""
import argparse
import os
import re
from dataclasses import dataclass, field

TICK_MS = 5
REASON = {1: "tilted past 40 deg", 2: "motors cut by the stock code (KEY1, its own 40 deg check, or low battery)",
          3: "wheels spinning free (lifted)"}
# Reason 2 is also the stock firmware's own 40 deg cut-out, which can fire
# before rl_mode.c's. A run that reached this pitch fell; one that did not was
# stopped (KEY1) or lost battery.
FELL_DEG = 30.0

# Even spinning in the air at full PWM the wheels reach ~33 rad/s, 1.1 m/s at
# the rim (333 RPM at 12 V; motor_model.py agrees), and the pitch-rate term of
# v adds ~0.2 m/s at most, so 1.5 m/s is out of reach. One encoder count over
# the 4-tick window is ~8 mm/s, so a 200 mm/s change in one 5 ms tick is a
# corrupted reading, not motion (run 7's stack overflow gave tens of m/s).
V_IMPOSSIBLE_MM_S = 1500
V_JUMP_MM_S = 200
# The network's output is a float turned into integer PWM; with noisy sensors
# it does not repeat the same pair for 0.1 s unless something upstream froze
# (run 7: -1923/-1466 for seconds). Saturated values are excluded.
PWM_SAT = 2800
FROZEN_E_TICKS = 20
FROZEN_D_LINES = 5
# rl_mode.c counts a gap of more than 7.5 ms between control ticks as late.
LATE_GAP_US = 7500
ISR_BUDGET_US = 5000
# Run 8 in simulation flips the PWM sign on 0-27 % of ticks; run 7 ~100 %.
FLIP_WARN = 0.5
# Ignore the first 2 s of D lines when averaging creep: the policy is settling.
SETTLE_MS = 2000

TIME_RE = re.compile(r"^\d{1,2}:\d{2}:\d{2}$")


@dataclass
class Run:
    clock: str
    ms: int
    reason: int
    maxp: float
    d: list = field(default_factory=list)      # (ms, pitch deg, v m/s, L, R)
    e: list = field(default_factory=list)      # (pitch deg, rate deg/s, v mm/s, L, R)
    t: tuple = None
    notes: list = field(default_factory=list)  # cal / trim lines before this run


def parse(path):
    runs, d_buf, notes = [], [], []
    cur = last = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            p = raw.split()
            clock = ""
            if p and TIME_RE.match(p[0]):
                clock, p = p[0], p[1:]
            if not p:
                continue
            try:
                if p[0] == "D" and len(p) == 8:
                    if int(p[2]) == 2:
                        d_buf.append((int(p[1]), float(p[3]), float(p[4]), int(p[5]), int(p[6])))
                elif p[0] == "R" and len(p) == 4:
                    cur = Run(clock, int(p[1]), int(p[2]), float(p[3]), d=d_buf, notes=notes)
                    d_buf, notes = [], []
                elif p[0] == "E" and len(p) == 7 and cur is not None:
                    k, pit, q, v, l, r = (int(x) for x in p[1:])
                    cur.e.append((pit / 100.0, q / 100.0, v, l, r))
                elif p[0] == "R" and p[1:] == ["end"] and cur is not None:
                    runs.append(cur)
                    last, cur = cur, None
                elif p[0] == "T" and len(p) == 8 and last is not None and last.t is None:
                    last.t = tuple(int(x) for x in p[1:])
                elif p[0] in ("cal", "pitch") and len(p) > 1 and p[1].startswith(("done", "zero")):
                    notes.append(" ".join(p))
            except ValueError:
                continue                               # older firmware's formats, garbage
    return runs


def longest_same(pairs):
    best, n, prev = (0, None), 0, None
    for pr in pairs:
        if abs(pr[0]) >= PWM_SAT or abs(pr[1]) >= PWM_SAT:
            n, prev = 0, None
            continue
        n = n + 1 if pr == prev else 1
        prev = pr
        if n > best[0]:
            best = (n, pr)
    return best


def flip_rate(xs):
    pairs = [(a, b) for a, b in zip(xs, xs[1:]) if a and b]
    return sum((a > 0) != (b > 0) for a, b in pairs) / len(pairs) if pairs else 0.0


def stopped(run):
    return run.reason == 2 and run.maxp < FELL_DEG


def check(run):
    """-> list of (level, text); level is FAIL, WARN, ok or info."""
    out = []
    if stopped(run):
        out.append(("ok", "stayed up until the motors were switched off"))
    elif run.reason in (1, 2):
        out.append(("FAIL" if run.ms < 5000 else "WARN", "fell after %.2f s" % (run.ms / 1000.0)))
    else:
        out.append(("info", "ended by %s" % REASON.get(run.reason, "reason %d" % run.reason)))

    if len(run.e) >= 2:
        n, pr = longest_same([(r[3], r[4]) for r in run.e])
        if n >= FROZEN_E_TICKS:
            out.append(("FAIL", "frozen output: PWM stuck at %s for %d ticks in a row" % (pr, n)))
        else:
            out.append(("ok", "output changes (longest repeat %d ticks)" % n))
        v = [r[2] for r in run.e]
        bad = [x for x in v if abs(x) > V_IMPOSSIBLE_MM_S]
        jumps = sum(abs(b - a) > V_JUMP_MM_S for a, b in zip(v, v[1:]))
        if bad or jumps:
            out.append(("FAIL", "speed readings: %d impossible (|v| > %d mm/s, worst %+d), %d jumps > %d mm/s "
                        "in one tick" % (len(bad), V_IMPOSSIBLE_MM_S, max(bad, key=abs) if bad else 0,
                                         jumps, V_JUMP_MM_S)))
        else:
            out.append(("ok", "speed readings plausible (max |v| %d mm/s)" % max(abs(x) for x in v)))
        fl, fr = flip_rate([r[3] for r in run.e]), flip_rate([r[4] for r in run.e])
        out.append(("WARN" if max(fl, fr) > FLIP_WARN else "info",
                    "PWM sign flips: L %.0f %%, R %.0f %% of ticks (sim run 8: 0-27 %%)" % (100 * fl, 100 * fr)))
    else:
        out.append(("WARN", "no flight-recorder lines for this run"))

    if run.t:
        isr, _, _, net, gap, late, ticks = run.t
        lvl = "FAIL" if late > 0 or gap > LATE_GAP_US or isr > ISR_BUDGET_US else "ok"
        out.append((lvl, "timing: %d late ticks, longest gap %d us, longest interrupt %d us, network %d us"
                    % (late, gap, isr, net)))
    else:
        out.append(("WARN", "no T line (older firmware?)"))

    if len(run.d) >= 2:
        n, pr = longest_same([(r[3], r[4]) for r in run.d])
        if n >= FROZEN_D_LINES:
            out.append(("FAIL", "frozen output in the 10 Hz stream: %s for %.1f s" % (pr, n / 10.0)))
        t0 = run.d[0][0]
        steady = [r for r in run.d if r[0] - t0 >= SETTLE_MS] or run.d
        vm = sum(r[2] for r in steady) / len(steady)
        pm = sum(r[1] for r in steady) / len(steady)
        out.append(("info", "10 Hz stream after the first 2 s: mean speed %+.3f m/s, mean pitch %+.2f deg "
                    "(%d lines; includes any app commands)" % (vm, pm, len(steady))))
    return out


def plot(run, idx, folder):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if not run.e:
        return None
    t = [(k - len(run.e) + 1) * TICK_MS for k in range(len(run.e))]
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
    for a, col, lab in zip(ax[:3], range(3), ("pitch (deg)", "pitch rate (deg/s)", "v (mm/s)")):
        a.plot(t, [r[col] for r in run.e])
        a.set_ylabel(lab)
        a.grid(alpha=0.3)
    ax[3].plot(t, [r[3] for r in run.e], label="L")
    ax[3].plot(t, [r[4] for r in run.e], label="R")
    ax[3].set_ylabel("PWM")
    ax[3].legend()
    ax[3].grid(alpha=0.3)
    ax[3].set_xlabel("ms before the run ended")
    fig.suptitle("run %d (%s): %.2f s, %s" % (idx, run.clock, run.ms / 1000.0, REASON.get(run.reason, "?")))
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, "run_%02d.png" % idx)
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log", help="rl_serial_log.txt written by rl_trim_helper.py")
    ap.add_argument("--last", type=int, default=0, help="only the last N runs in the file")
    ap.add_argument("--plot", action="store_true", help="save a PNG of each run's flight recording")
    args = ap.parse_args()

    runs = parse(args.log)
    first = 1
    if args.last:
        first = max(1, len(runs) - args.last + 1)
        runs = runs[-args.last:]
    if not runs:
        print("no complete runs (R ... R end) in %s" % args.log)
        return
    folder = os.path.join(os.path.dirname(os.path.abspath(args.log)), "car_log_plots")
    totals = {"FAIL": 0, "WARN": 0}
    stood = 0
    for i, run in enumerate(runs, first):
        for n in run.notes:
            print("  > " + n)
        print("run %d  %s  %.2f s, ended: %s, max |pitch| %.1f deg"
              % (i, run.clock, run.ms / 1000.0, REASON.get(run.reason, "?"), run.maxp))
        res = check(run)
        for lvl, text in res:
            print("    %-4s %s" % (lvl, text))
        if any(l == "FAIL" for l, _ in res):
            totals["FAIL"] += 1
        elif any(l == "WARN" for l, _ in res):
            totals["WARN"] += 1
        stood += stopped(run)
        if args.plot:
            p = plot(run, i, folder)
            if p:
                print("    plot %s" % p)
    print("\n%d runs: %d stopped by you, %d with a FAIL, %d with only warnings"
          % (len(runs), stood, totals["FAIL"], totals["WARN"]))
    print("Frozen output, impossible speeds or late ticks are firmware problems: fix them before judging the policy.")


if __name__ == "__main__":
    main()
