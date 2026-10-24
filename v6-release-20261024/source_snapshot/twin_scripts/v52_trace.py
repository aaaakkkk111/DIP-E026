# -*- coding: utf-8 -*-
"""Trace v5.2 internals for one high-load placement (debug; 2026-10-24).
   python scripts/rl/v52_trace.py DLL h kg seed [opt]"""
import ctypes, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import chatter_twin as C
import v52_score as V
FIELDS = "f fl boost lat rec osc oscf sw swold swhz floor ceil ceiln strug rearms downs swups thref evp".split()

def main():
    dll, h, kg, sd = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4])
    os.environ["V52_OPT"] = sys.argv[5] if len(sys.argv) > 5 else "50"
    C.DLLS["v5.2"] = dll
    V._init(h)
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import FS
    lib = C.LIBS["v5.2"]
    env = C._env(sd); c = env.core; ctl = C.V5C(lib)
    buf = (ctypes.c_float * 19)()
    for _ in range(int(4 * FS)): env.step(ctl(env))
    env._apply_payload(kg); T.set_tilt(env, 3.0, 60.0)
    print("t    th    " + " ".join("%6s" % x for x in FIELDS))
    for n in range(int(15 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env))
        lib.v4_dbg(buf)
        if n % 20 == 0 or d:
            print("%5.2f %6.2f " % (n / FS, float(c._angle_filt)) + " ".join("%6.2f" % buf[i] for i in range(19)))
        if d:
            print("FELL"); break

if __name__ == "__main__":
    main()
