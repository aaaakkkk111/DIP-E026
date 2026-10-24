# -*- coding: utf-8 -*-
"""Trace v5.2 internals on the benchmark course with a high load (debug 10-24).
   python scripts/rl/v52_trace_course.py DLL h kg seed"""
import ctypes, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import chatter_twin as C
import v52_score as V
FIELDS = "f fl boost lat rec osc oscf sw swold swhz floor ceil ceiln strug rearms downs swups thref evp".split()
def main():
    C.DLLS["v5.2"] = sys.argv[1]; h, kg, sd = float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4])
    V._init(h)
    lib = C.LIBS["v5.2"]; buf = (ctypes.c_float * 19)()
    orig = C.V5C.__call__; cnt = [0]
    def call(self, env, moving=None):
        a = orig(self, env, moving)
        lib.v4_dbg(buf)
        if cnt[0] % 20 == 0:
            print("%5.2f th %6.2f v %5.2f " % (cnt[0] / 200.0, float(env.core._angle_filt), float(env.v_ref))
                  + " ".join("%5.2f" % buf[i] for i in (0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 13, 14, 16)))
        cnt[0] += 1
        return a
    C.V5C.__call__ = call
    print("t           th        v   f    fl    bst   lat   rec   osc   sw    swold swhz  floor strug rearm swups")
    r = C.FA.run("v5.2", kg, 0.0, 0.0, sd, 0.0)
    print({k: r[k] for k in ("ok", "legs", "settle")})
main()
