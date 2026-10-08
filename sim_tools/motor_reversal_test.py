"""Motor reversal test on the car: two-level PWM steps on each wheel (1 s at
a first PWM, 1 s at a second, then 0.5 s at 0), encoder counts at 200 Hz.
The second level reverses the motor while the wheel is turning fast, or
slows it without reversing, which the one-level step test never did.

Needs the car_firmware_reversal build ('mt L <pwm> <pwm2>') and the COM
port to itself. Car upright on a box, wheels in the air, as for
motor_step_test.py:

    python sim_tools/motor_reversal_test.py [--port COM3]

Start it, switch the car on, select mode 28, calibrate, then press KEY1
with the car on the box. Each wheel and direction, from +2000 and from
+2800 (and mirrored): to 0 (coast), to -1500, -1700, -2000, -2800
(reversals) and to +1600 / +1800 (slowing without reversing): 48 steps.

Writes car_logs/<date>/motor_reversal.csv (step, side, pwm1, pwm2, k, enc_l,
enc_r; k = 5 ms tick, pwm1 for k < 200, pwm2 for 200 <= k < 400, 0 after;
enc at k counts the tick that ends at k) and motor_reversal_raw.txt.
"""
import argparse
import csv
import os
import sys
import time

from motor_step_test import ROOT, Car

N_TWO = 500


def run_two(car, side, p1, p2, patience):
    told = None
    end = time.time() + patience
    while time.time() < end:
        car.send(f"mt {side} {p1} {p2}")
        rows = []
        while True:
            s = car.line(6.0)
            if s is None:
                break
            if s.startswith("mt:"):
                if s != told:
                    print(f"    car: {s[4:]}  (waiting)")
                    told = s
                time.sleep(1.0)
                break
            f = s.split()
            if s.startswith("M end"):
                if s == "M end" and len(rows) == N_TWO:
                    return rows
                print(f"    step cut after {len(rows)} ticks ({s}); repeating it")
                time.sleep(1.0)
                break
            if len(f) == 6 and f[0] == "M" and f[1] == side and int(f[2]) == p1:
                rows.append((int(f[3]), int(f[4]), int(f[5])))
    raise SystemExit(f"gave up on mt {side} {p1} {p2} after {patience:.0f} s")


def steps(which="reversal"):
    out = []
    for side in "LR":
        for sgn in (1, -1):
            if which == "reversal":
                pairs = [(p1, p2) for p1, slow in ((2000, 1600), (2800, 1800))
                         for p2 in (0, -1500, -1700, -2000, -2800, slow)]
            else:                                       # "lowpwm": the region near the dead zone
                pairs = ([(p, p) for p in (1420, 1440, 1460, 1480, 1500, 1520)]          # from rest, 2 s
                         + [(1800, p) for p in (1420, 1440, 1460, 1480, 1500)]          # from motion
                         + [(1500, 1600), (1500, 1700), (1600, 1800)]                   # small steps up
                         + [(1700, 1500), (1700, 1600), (2000, 1500)]                   # small steps down
                         + [(1600, -1600), (1700, -1500), (1500, -1500)])               # low-PWM reversals
            out += [(side, sgn * p1, sgn * p2) for p1, p2 in pairs]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--set", choices=("reversal", "lowpwm"), default="reversal",
                    help="reversal: the 48 steps above; lowpwm: 80 steps near the dead zone "
                         "(thresholds from rest and from motion, small steps, low-PWM reversals)")
    ap.add_argument("--out", default=None, help="default car_logs/<date>/motor_<set>.csv")
    ap.add_argument("--wait", type=float, default=600)
    args = ap.parse_args()

    out = args.out or os.path.join("car_logs", time.strftime("%Y-%m-%d"), f"motor_{args.set}.csv")
    out = out if os.path.isabs(out) else os.path.join(ROOT, out)
    if os.path.exists(out):
        sys.exit(f"{out} exists; use --out")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    car = Car(args.port, os.path.splitext(out)[0] + "_raw.txt")
    car.raw.reconfigure(line_buffering=True)
    print(f"connected {args.port}. Switch the car on, select mode 28, let it calibrate.")

    end = time.time() + args.wait
    ack = None
    unknown = 0                                         # a '?' can be a command cut by the car starting up
    while (ack is None or ack == "?") and time.time() < end:
        car.send("mt on")
        ack = car.wait_for(("mt on", "?"), 1.0)
        unknown = unknown + 1 if ack == "?" else 0
        if unknown >= 3:
            sys.exit("the car answered '?' to 'mt on' three times: it is not running the car_firmware_reversal build")
    if ack is None:
        sys.exit("no answer to 'mt on'")
    print(f"car: {ack}")
    print("Put the car upright on the box, wheels in the air, and press KEY1.")
    run_two(car, "L", 0, 0, args.wait)                  # probe: returns once the motors are enabled
    for i in (3, 2, 1):
        print(f"  starting in {i}")
        time.sleep(1.0)

    todo = steps(args.set)
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["step", "side", "pwm1", "pwm2", "k", "enc_l", "enc_r"])
        for n, (side, p1, p2) in enumerate(todo, 1):
            rows = run_two(car, side, p1, p2, args.wait)
            for k, el, er in rows:
                w.writerow([n, side, p1, p2, k, el, er])
            fh.flush()
            e = [r[1] if side == "L" else r[2] for r in rows]
            print(f"  [{n:2d}/{len(todo)}] {side} {p1:+5d} -> {p2:+5d}: {sum(e[150:200]) / 50:+6.1f} -> "
                  f"{sum(e[350:400]) / 50:+6.1f} counts/tick", flush=True)
            time.sleep(1.0)

    car.send("mt off")
    print(f"car: {car.wait_for(('mt off',), 3.0)}")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
