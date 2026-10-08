"""Motor step test on the car: raw PWM steps on each wheel, encoder counts
recorded at 200 Hz, for fitting motor_model.py (see motor_fit.py).

Needs the car_firmware_motortest build (mode 28 with the 'mt' commands) and
the logger (rl_trim_helper.py) closed, since both use the same COM port.

    python sim_tools/motor_step_test.py [--port COM3]

Start this first, then switch the car on, select mode 28 and let it calibrate.
The script starts a test session as soon as the car answers ('mt on' blocks
the policy from arming). Then hold the car upright with both wheels in the
air and press KEY1. The steps start about 3 s later.

Per wheel (L, R) and sign, raw PWM 1300 ... 2800 (52 steps). Each step is
1.0 s at the PWM, then 0.5 s at 0, with the other wheel at 0. If the motors
are cut (tilt over 40 deg, low battery), the step is repeated once they are
back.

Writes car_logs/<date>/motor_steps.csv (step, side, pwm, k, enc_l, enc_r;
k = 5 ms tick, PWM on for k < 200; enc = counts in the tick that ends at k,
forward positive) and motor_steps_raw.txt (everything the car sent).
"""
import argparse
import csv
import os
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PWMS = [1300, 1350, 1400, 1450, 1500, 1550, 1600, 1700, 1800, 2000, 2200, 2400, 2800]
ON_TICKS = 200


class Car:
    def __init__(self, port, raw_path):
        self.sp = serial.Serial()
        self.sp.port, self.sp.baudrate, self.sp.timeout = port, 230400, 0.05
        self.sp.dtr = False      # FlyMCU auto-reset / BOOT0 lines: keep low
        self.sp.rts = False
        self.sp.open()
        self.raw = open(raw_path, "a", encoding="utf-8", buffering=1)
        self.buf = b""
        self.lines = []

    def send(self, s):
        self.sp.write((s + "\r\n").encode())
        self.raw.write(f"{time.strftime('%H:%M:%S')} > {s}\n")

    def line(self, timeout):
        """Next line from the car, or None after timeout seconds."""
        end = time.time() + timeout
        while not self.lines:
            if time.time() > end:
                return None
            self.buf += self.sp.read(4096)
            *done, self.buf = self.buf.split(b"\n")
            for b in done:
                s = b.decode("ascii", "replace").strip()
                if s:
                    self.raw.write(f"{time.strftime('%H:%M:%S')} {s}\n")
                    self.lines.append(s)
        return self.lines.pop(0)

    def wait_for(self, prefixes, timeout):
        end = time.time() + timeout
        while time.time() < end:
            s = self.line(end - time.time())
            if s and s.startswith(prefixes):
                return s
        return None


def run_step(car, side, pwm, patience):
    """One step, repeated until it completes. Returns the list of (k, enc_l, enc_r)."""
    told = None
    end = time.time() + patience
    while time.time() < end:
        car.send(f"mt {side} {pwm}")
        rows = []
        while True:
            s = car.line(4.0)
            if s is None:
                break                                   # no answer: send again
            if s.startswith("mt:"):
                if s != told:
                    print(f"    car: {s[4:]}  (waiting)")
                    told = s
                time.sleep(1.0)
                break
            f = s.split()
            if s.startswith("M end"):
                if s == "M end" and len(rows) == ON_TICKS + 100:
                    return rows
                print(f"    step cut after {len(rows)} ticks ({s}); repeating it")
                time.sleep(1.0)
                break
            if len(f) == 6 and f[0] == "M" and f[1] == side and int(f[2]) == pwm:
                rows.append((int(f[3]), int(f[4]), int(f[5])))
    raise SystemExit(f"gave up on mt {side} {pwm} after {patience:.0f} s")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--out", default=os.path.join("car_logs", time.strftime("%Y-%m-%d"), "motor_steps.csv"))
    ap.add_argument("--wait", type=float, default=600, help="seconds to wait for the car at each stage")
    ap.add_argument("--force", action="store_true", help="overwrite an existing CSV")
    args = ap.parse_args()

    out = args.out if os.path.isabs(args.out) else os.path.join(ROOT, args.out)
    if os.path.exists(out) and not args.force:
        sys.exit(f"{out} exists; use --force or --out")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    car = Car(args.port, os.path.splitext(out)[0] + "_raw.txt")
    print(f"connected {args.port}. Switch the car on, select mode 28, let it calibrate.")

    end = time.time() + args.wait
    ack = None
    while ack is None and time.time() < end:
        car.send("mt on")
        ack = car.wait_for(("mt on", "?"), 1.0)
        if ack == "?":
            sys.exit("the car answered '?' to 'mt on': it is not running the car_firmware_motortest build")
    if ack is None:
        sys.exit("no answer to 'mt on'")
    print(f"car: {ack}")
    print("Hold the car upright, wheels in the air, and press KEY1.")
    run_step(car, "L", 0, args.wait)                    # probe: returns once the motors are enabled
    for i in (3, 2, 1):
        print(f"  starting in {i}")
        time.sleep(1.0)

    steps = [(side, sign * p) for side in "LR" for sign in (1, -1) for p in PWMS]
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["step", "side", "pwm", "k", "enc_l", "enc_r"])
        for n, (side, pwm) in enumerate(steps, 1):
            rows = run_step(car, side, pwm, args.wait)
            for k, el, er in rows:
                w.writerow([n, side, pwm, k, el, er])
            fh.flush()
            e = [r[1] if side == "L" else r[2] for r in rows]
            print(f"  [{n:2d}/{len(steps)}] {side} {pwm:+5d}: mean of the last 0.5 s on "
                  f"{sum(e[100:ON_TICKS]) / 100:+6.1f} counts/tick")
            time.sleep(1.0)                             # let the wheel stop

    car.send("mt off")
    print(f"car: {car.wait_for(('mt off',), 3.0)}")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
