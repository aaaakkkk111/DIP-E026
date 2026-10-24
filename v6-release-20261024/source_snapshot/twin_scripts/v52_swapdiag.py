# -*- coding: utf-8 -*-
"""Per-pair load swap breakdown, v5.1 vs v5.2 (debug 10-24)."""
import os, sys
from multiprocessing import Pool
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import v52_score as V, load_matrix as L, chatter_twin as C
if __name__ == "__main__":
    pairs = [(x, y) for x in L.SWAP_SET for y in L.SWAP_SET if x != y]
    jobs = [(k, x, y, sd) for k in ("v5.1", "v5.2") for x, y in pairs for sd in (0, 1, 2)]
    with Pool(16, initializer=V._init) as pool:
        res = pool.map(L.job_swap, jobs)
    print("pair      | v5.1 shake sd   f_a->f_end fmax rp dn | v5.2 shake sd   f_a->f_end fmax rp dn")
    for x, y in pairs:
        cells = []
        for k in ("v5.1", "v5.2"):
            rs = [r for (kk, a, b, sd), r in res if kk == k and a == x and b == y and not r["fell"]]
            m = lambda key: np.mean([r[key] for r in rs])
            cells.append("%5.2f %.3f %.2f->%.2f %.2f %.1f %.1f" % (m("shake"), m("th_sd"), m("f_a"), m("f_end"), m("f_max"), m("reprobes"), m("downs")))
        print("%g->%g kg  | %s | %s" % (x, y, cells[0], cells[1]))
