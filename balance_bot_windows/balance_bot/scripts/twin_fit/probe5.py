# -*- coding: utf-8 -*-
"""找孪生慢速游走的来源：逐个关掉候选因素，同时记录姿态估计误差。"""
import os, sys
import numpy as np
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", ".."))
BASE = dict(stall=0.5717, damp=0.0105, irot=6.33e-4, dpos=1522.0, dneg=1529.48, delay=2.0253,
            gnoise=0.000842, k=11.19, c=0.0778, bl=0.5806, fric=0.0, ib=1.0, mnt=0.2804,
            gbias_dps=2.02, v2=True)
CASES = [
    ("现状（含零偏）", {}, None),
    ("去掉陀螺零偏", dict(gbias_dps=0.0), None),
    ("去掉陀螺噪声", dict(gnoise=0.0), None),
    ("去掉零偏+噪声", dict(gbias_dps=0.0, gnoise=0.0), None),
    ("去掉延迟", dict(delay=0.0), None),
    ("去掉间隙", dict(bl=0.0), None),
    ("死区=补偿 1500", dict(dpos=1500.0, dneg=1500.0), None),
    ("理想姿态（真值，无零偏）", {}, "ideal_imu"),
    ("理想姿态但保留零偏进 D 项", {}, "ideal_imu_bias"),
]


def init():
    sys.path.insert(0, REPO); sys.path.insert(0, HERE); os.chdir(REPO)


def one(args):
    label, over, mode = args
    import replay as R, bench as B
    import balance_bot.firmware.twin_baseline as TB
    from balance_bot.dynamics import ITH
    p = dict(BASE, **over)
    est_err = []
    orig = None
    if mode:
        orig = TB.STM32TwinMixin._read_imu

        def patched(self, meas, st, dt):
            out = orig(self, meas, st, dt)
            ang = float(np.rad2deg(float(st[ITH]))) + self.mount_offset_deg
            rate = float(st[TB.ITHD])
            b = self.gyro_bias_rad_s if mode == "ideal_imu_bias" else 0.0
            self._angle_filt = ang
            self._gyro_raw = (rate + b) * TB.GYRO_LSB_PER_RAD_S
            self._gyro_lsb = float(self._gyro_raw)
            return ang, self._gyro_raw
        TB.STM32TwinMixin._read_imu = patched
    ms = []
    try:
        for sd in (0, 1):
            recs = []

            def hook_factory(recs=recs):
                def h(tw, t):
                    if tw._angle_filt is not None:
                        recs.append(tw._angle_filt - tw.mount_offset_deg - np.degrees(float(tw.state[ITH])))
                return h
            import replay
            old_replay_hook = None
            r = R.replay("base_empty.txt", p, seed=sd, settle=5.0)
            if r["fell_at"] is not None:
                return label, None
            ms.append(B.metrics(r["twin"]))
    finally:
        if orig is not None:
            TB.STM32TwinMixin._read_imu = orig
    return label, B.mean_metrics(ms)


if __name__ == "__main__":
    init()
    import replay as R, bench as B
    real = B.metrics(R.parse("base_empty.txt")["real"])
    with Pool(len(CASES), initializer=init) as pool:
        res = pool.map(one, CASES)
    keys = ("disp_pp", "disp_std", "f_disp", "f_gyro", "rev", "enc_abs", "ang_std", "gyro_rms", "gyro_mean")
    print("%-28s" % "情形" + "".join("%9s" % k for k in keys) + "%8s" % "综合")
    print("%-28s" % "真车" + "".join("%9.2f" % real[k] for k in keys) + "%8.3f" % B.score(real)[0])
    for label, m in res:
        if m is None:
            print("%-28s 摔了" % label); continue
        print("%-28s" % label + "".join("%9.2f" % m[k] for k in keys) + "%8.3f" % B.score(m)[0])
