# -*- coding: utf-8 -*-
"""Is the 10-03 code in v4rel.dll actually exercised in the twin's scoring tasks?
Counts command edges sent to the C core, upshifts, and upshifts that the grace
windows suppressed (edge_skips), per scoring case, seed 0."""
import ctypes
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.argv = [sys.argv[0]]
import score_release_v4 as S  # noqa: E402

S._init()
lib = ctypes.CDLL(os.path.join(HERE, "v4rel.dll"))
calls = {"edge": 0}
orig = None
for cn, kg, imp, st, sl, w in S.OBJ_CASES:
    import factory_vs_auto as FA
    Ctl = FA.Ctl

    class Probe(Ctl):
        def __init__(self, kind, kg_):
            super().__init__(kind, kg_)
            lib_ = self.sw.lib
            f0 = lib_.v4_cmd_edge

            def edge():
                calls["edge"] += 1
                f0()
            lib_.v4_cmd_edge = edge
            Probe.last = self

    FA.Ctl = Probe
    calls["edge"] = 0
    r = FA.run("v4rel", kg, imp, st, 0, sl)
    lib_ = Probe.last.sw.lib
    lib_.v4_rearms.restype = ctypes.c_long
    print("%-8s ok %s  edges %d  upshifts %d" % (cn, r["ok"], calls["edge"], lib_.v4_rearms()))
    FA.Ctl = Ctl
