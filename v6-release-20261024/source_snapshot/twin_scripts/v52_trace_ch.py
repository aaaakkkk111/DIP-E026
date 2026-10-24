# -*- coding: utf-8 -*-
"""Trace v5.2 internals while the chatter downshift runs (empty car forced to f=1). debug 10-24
   python scripts/rl/v52_trace_ch.py DLL kg seed"""
import ctypes, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import chatter_twin as C
import v52_score as V
import load_matrix as L
FIELDS = "f fl boost lat rec osc oscf sw swold swhz floor ceil ceiln strug rearms downs swups thref evp".split()
def main():
    C.DLLS["v5.2"] = sys.argv[1]; kg, sd = float(sys.argv[2]), int(sys.argv[3])
    V._init()
    from balance_bot.rl.e2e_env import FS
    lib = C.LIBS["v5.2"]; env = C._env(sd, kg); c = env.core; ctl = C.V5C(lib)
    buf = (ctypes.c_float * 19)(); osc = L.Osc()
    for _ in range(int(4 * FS)): env.step(ctl(env))
    lib.v4_force(1.0)
    print("t   meter " + " ".join("%6s" % x for x in FIELDS))
    for n in range(int(4 * FS)):
        _o, _r, d, _t, _ = env.step(ctl(env)); lib.v4_dbg(buf); o = osc(float(c._gyro_lsb))
        if n % 10 == 0: print("%5.2f %5.1f " % (n / FS, o) + " ".join("%6.2f" % buf[i] for i in range(19)))
        if d: print("FELL"); break
main()
