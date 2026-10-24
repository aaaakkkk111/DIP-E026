# -*- coding: utf-8 -*-
"""Trace v5.2 internals across a load swap (debug 10-24).  python v52_trace_swap.py DLL ka kb seed"""
import ctypes, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import chatter_twin as C
import v52_score as V
def main():
    C.DLLS["v5.2"] = sys.argv[1]; ka, kb, sd = float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4])
    V._init()
    import tilt_envelope as T
    from balance_bot.rl.e2e_env import FS
    lib = C.LIBS["v5.2"]; buf = (ctypes.c_float * 19)()
    env = C._env(sd, ka); c = env.core; ctl = C.FA.Ctl("v5.2", ka)
    print("t      th     f    fl   bst  lat  rec   osc  oscf   sw   swold swhz floor ceil ceiln strug rearm downs swups")
    for n in range(int(16 * FS)):
        if n == int(6 * FS):
            env._apply_payload(kb); T.set_tilt(env, 3.0, 60.0); print("---- swap")
        _o, _r, d, _t, _ = env.step(ctl(env)); lib.v4_dbg(buf)
        if n % 20 == 0 or d:
            print("%5.2f %6.2f " % (n / FS, float(c._angle_filt)) + " ".join("%5.2f" % buf[i] for i in range(17)))
        if d: print("FELL"); break
main()
