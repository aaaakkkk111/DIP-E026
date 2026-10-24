# -*- coding: utf-8 -*-
"""High-CoM matrix, 2026-10-24: load CoM 0.105 (plate) / 0.15 / 0.20 / 0.25 / 0.30 m above the
axle x 0.5 / 1 / 2 / 3 kg, two scenes, factory baselines included.
  stand   load put on a standing car (+3 deg nudge), 15 s: falls / sway deg (last 3 s) / final gear /
          gear changes in the last 5 s
  course  benchmark course with that load: penalty / falls
    python scripts/rl/v52_hmatrix.py stand|course [kinds,comma-separated]
"""
import os, sys
from multiprocessing import Pool
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import v52_score as V
HS = (0.105, 0.15, 0.20, 0.25, 0.30)
KGS = (0.5, 1.0, 2.0, 3.0)
SEEDS = (0, 1, 2)

def main():
    what = sys.argv[1]
    kinds = sys.argv[2].split(",") if len(sys.argv) > 2 else list(V.KINDS)
    jobs = [(k, h, kg, sd) for k in kinds for h in HS for kg in KGS for sd in SEEDS]
    fn = V.job_high if what == "stand" else V.job_hcourse
    with Pool(16, initializer=V._init) as pool:
        res = pool.map(fn, jobs)
    if what == "stand":
        print("load put on a standing car; 15 s; 3 seeds; cell = falls / sway deg / final gear / changes last 5 s")
    else:
        print("benchmark course with the load; 3 seeds; cell = penalty / falls")
    for h in HS:
        print("== load CoM %.3f m above the axle" % h)
        for k in kinds:
            cells = []
            for kg in KGS:
                rs = [r for (kk, hh, g, sd), r in res if kk == k and hh == h and g == kg]
                if what == "stand":
                    ok = [r for r in rs if not r["fell"]]
                    if not ok:
                        cells.append("FELL 3/3"); continue
                    cells.append("%d/%.2f/%.1f/%.1f" % (len(rs) - len(ok), np.mean([r["sway"] for r in ok]),
                                                       np.mean([r["f_end"] for r in ok]), np.mean([r["changes"] for r in ok])))
                else:
                    cells.append("%.1f/%d" % (np.mean([r["pen"] for r in rs]), sum(not r["ok"] for r in rs)))
            print("  %-17s " % V.NAMES[k] + " ".join("%gkg %-19s" % (kg, c) for kg, c in zip(KGS, cells)))
    sys.stdout.flush()

if __name__ == "__main__":
    main()
