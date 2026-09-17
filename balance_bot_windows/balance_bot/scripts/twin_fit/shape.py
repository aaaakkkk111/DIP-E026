# -*- coding: utf-8 -*-
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)
import replay as R, bench as B
real = R.parse("base_empty.txt")["real"]
tw = R.replay("base_empty.txt", dict(source=True), seed=100, settle=5.0)["twin"]
for lab, d in (("真车", real), ("孪生", tw)):
    ml = B._fill(np.asarray(d["ml"], float)); g = B._fill(np.asarray(d["gyro"], float)) / 16.4
    el = B._fill(np.asarray(d["el"], float))
    print("==", lab)
    s = np.sign(ml); segs = []; i = 0
    a0 = 2000
    x = s[a0:a0 + 400]
    j = 0
    while j < len(x):
        k = j
        while k < len(x) and x[k] == x[j]:
            k += 1
        segs.append("%s%d" % ("+" if x[j] > 0 else ("-" if x[j] < 0 else "0"), k - j)); j = k
    print("段:", " ".join(segs))
    print("PWM-1500 前100拍:", " ".join("%d" % (np.sign(v) * (abs(v) - 1500)) if v else "0" for v in ml[a0:a0 + 100]))
    print("gyro 前100拍:", " ".join("%.0f" % v for v in g[a0:a0 + 100]))
    print("enc 前100拍:", "".join("%d" % v if v >= 0 else chr(ord('a') - 1 - int(v)) for v in el[a0:a0 + 100]))
