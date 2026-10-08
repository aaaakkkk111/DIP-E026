"""Does the car's int16 network (policy_q.c) balance like the float network
the simulator runs?

    python sim_tools/policy_q_sim.py [--slack 2] [--delay 1 2 3]

policy_q.c is copied into numpy: Q16.16 inputs, int16 weights with a float
scale per row, int64 accumulators, the fast rational tanh in Q12 -> Q15
integers (C integer division), float32 for the scale and bias. Weights come
from firmware/quantize_weights.py's quantise() for each checkpoint (nothing
is written). The copy is checked against firmware/policy_testvectors.h
(run 11, float network: expect agreement to ~1e-2, the fast-tanh error).
Closed loop: fitted motor A (motor_fit.py), standing still, as simlib.run().
"""
import argparse
import os
import re
import sys

import numpy as np

import motor_fit as F
import simlib as S
from policy_filter_sim import FITTED_A, CAR

sys.path.insert(0, os.path.join(S.ROOT, "firmware"))
import quantize_weights as QW  # noqa: E402

Q12_THREE = 3 * 4096
Q12_27 = 27 * 4096


def c_div(a, b):
    """C integer division (truncates toward zero), elementwise."""
    q = np.abs(a) // np.abs(b)
    return np.where((a < 0) != (b < 0), -q, q)


def tanh_q12_to_q15(x):
    x = x.astype(np.int64)
    x2 = (x * x) >> 12
    y = c_div(x * (Q12_27 + x2), (Q12_27 + 9 * x2) >> 3)
    y = np.clip(y, -32767, 32767)
    return np.where(x >= Q12_THREE, 32767, np.where(x <= -Q12_THREE, -32767, y))


def pre_to_q12(v):
    v = np.clip(v, -Q12_THREE, Q12_THREE)
    return np.trunc(v).astype(np.int64)


class QuantPolicy:
    def __init__(self, model_path):
        self.layers = [(q.astype(np.int64), k.astype(np.float32), c.astype(np.float32))
                       for q, k, c, _ in QW.quantise(S.repo_path(model_path))]

    def infer(self, obs):
        v = np.asarray(obs, dtype=np.float32) * np.float32(65536.0)
        x = np.trunc(np.clip(v, -2147418112.0, 2147418112.0)).astype(np.int64)
        for q, k, c in self.layers[:2]:
            acc = q @ x
            x = tanh_q12_to_q15(pre_to_q12(acc.astype(np.float32) * k + c))
        q, k, c = self.layers[2]
        a = (q @ x).astype(np.float32) * k + c
        return np.clip(a, -1.0, 1.0)

    def predict(self, obs, deterministic=True):
        return self.infer(obs), None


def check_vectors(qp):
    txt = open(os.path.join(S.ROOT, "firmware", "policy_testvectors.h")).read()
    nums = lambda name: np.array([float(v.rstrip("f")) for v in re.findall(
        r"[-+]?\d+\.\d+(?:e[-+]?\d+)?f?", re.search(name + r"\[\d+\] = \{(.*?)\};", txt, re.S).group(1))])
    obs, exp = nums("policy_test_obs"), nums("policy_test_expected")
    n = len(exp) // 2
    got = np.array([qp.infer(o) for o in obs.reshape(n, -1)]).ravel()
    return float(np.max(np.abs(got - exp)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slack", type=float, nargs="+", default=[2.0])
    ap.add_argument("--delay", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--seeds", type=int, default=2)
    args = ap.parse_args()

    q11 = QuantPolicy("models/best_real_RUN11/best_model.zip")
    print(f"int16 copy vs policy_testvectors.h (run 11, float network): worst action difference "
          f"{check_vectors(q11):.4f}")
    print("cells: pitch-rate sd deg/s, Hz, mean |PWM|; fitted motor A, standing")
    for name, path in (("run8", "models/best_real_RUN8/best_model.zip"),
                       ("run9", "models/best_real_RUN9/best_model.zip"),
                       ("run11", "models/best_real_RUN11/best_model.zip")):
        pols = (("float", S.load(path)), ("int16", q11 if name == "run11" else QuantPolicy(path)))
        print(f"\n{name}  (car {CAR[name]} deg/s)")
        for g in args.slack:
            for lat in args.delay:
                cells = []
                for pname, pol in pols:
                    ss = [F.run_with_motor(pol, FITTED_A, slack_deg=(g, g), latency_ticks=lat, seed=sd)
                          for sd in range(args.seeds)]
                    ok = [s for s in ss if s["fell"] is None]
                    mn = lambda k: float(np.mean([s[k] for s in ok]))
                    cells.append(f"{pname}: " + (f"{mn('rate_sd'):6.1f} {mn('freq'):5.1f} {mn('pwm'):5.0f}" if ok else "FELL"))
                print(f"  {g:3.0f} deg {5 * lat:3d} ms   " + "   ".join(cells), flush=True)


if __name__ == "__main__":
    main()
