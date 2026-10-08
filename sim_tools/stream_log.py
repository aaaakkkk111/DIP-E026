"""Record the car's serial output (car_firmware_pidlog: the 200 Hz 'P' stream
in mode 1 and mode 28), with PC time, until stopped or --seconds pass.

    python sim_tools/stream_log.py --name pid_standing [--port COM3] [--seconds 300]

P line: P <k> <angle*100> <gyro> <L> <R> <encL> <encR> <accY> <accZ>
  k        tick counter (16 bit; a jump > 1 means the car's ring overflowed)
  angle    stock Angle_Balance, deg * 100 (Kalman filter, mode 1's PID input)
  gyro     stock Gyro_Balance, raw counts (-gyro X, 16.4 per deg/s)
  L, R     PWM that reached the motors (0 while they are cut)
  encL/R   encoder counts this tick, forward positive
  accY/Z   raw accelerometer Y and Z (16384 per g)

Writes car_logs/<date>/<name>.txt (every line, prefixed with PC time).
Needs the COM port to itself: close rl_trim_helper.py and FlyMCU first.

Battery: in mode 28 the car answers 'mt off' with its own battery reading
('mt off, battery 12.48 V'; outside a motor test the command changes
nothing). The script sends it once the car is talking and then every
--battery-every seconds. Mode 1 does not listen, so there is no reading.
"""
import argparse
import collections
import os
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--name", required=True, help="file name in car_logs/<date>/, without .txt")
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--battery-every", type=float, default=30.0, help="seconds; 0 = never ask")
    args = ap.parse_args()

    out = os.path.join(ROOT, "car_logs", time.strftime("%Y-%m-%d"), args.name + ".txt")
    if os.path.exists(out):
        sys.exit(f"{out} exists; choose another --name")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    sp = serial.Serial()
    sp.port, sp.baudrate, sp.timeout = args.port, 230400, 0.05
    sp.dtr = False      # FlyMCU auto-reset / BOOT0 lines: keep low
    sp.rts = False
    sp.open()
    print(f"recording {args.port} to {out}", flush=True)

    t0 = time.time()
    last_report = t0
    last_battery = None                             # not asked yet; ask once the car talks
    n_p = n_gap = 0
    prev_k = None
    recent = collections.deque(maxlen=200)          # last second of P samples
    buf = b""
    with open(out, "w", encoding="utf-8", buffering=1) as fh:
        try:
            while args.seconds is None or time.time() - t0 < args.seconds:
                buf += sp.read(8192)
                *lines, buf = buf.split(b"\n")
                now = time.time()
                stamp = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now * 1000) % 1000:03d}"
                if args.battery_every and lines and (last_battery is None or now - last_battery >= args.battery_every):
                    sp.write(b"mt off\r\n")
                    fh.write(f"{stamp} > mt off\n")
                    last_battery = now
                for b in lines:
                    s = b.decode("ascii", "replace").strip()
                    if not s:
                        continue
                    fh.write(f"{stamp} {s}\n")
                    f = s.split()
                    if f[0] == "P" and len(f) == 10:
                        try:
                            v = [int(x) for x in f[1:]]
                        except ValueError:
                            continue
                        n_p += 1
                        if prev_k is not None and (v[0] - prev_k) % 65536 != 1:
                            n_gap += 1
                        prev_k = v[0]
                        recent.append(v)
                    elif s.startswith("mt off, battery"):
                        print(f"  battery {f[3]} V (car's reading)", flush=True)
                    elif f[0] not in ("D", "E"):
                        print(f"  car: {s}", flush=True)
                if now - last_report >= 2.0:
                    last_report = now
                    if recent:
                        a = [r[1] / 100 for r in recent]
                        pwm = [(abs(r[3]) + abs(r[4])) / 2 for r in recent]
                        print(f"{now - t0:6.0f} s  {n_p} samples, {n_gap} gaps | last 1 s: pitch "
                              f"{min(a):+.1f}..{max(a):+.1f} deg, mean |PWM| {sum(pwm) / len(pwm):.0f}", flush=True)
                    else:
                        print(f"{now - t0:6.0f} s  no P lines yet", flush=True)
        except KeyboardInterrupt:
            pass
    print(f"stopped: {n_p} samples ({n_p / 200:.0f} s at 200 Hz), {n_gap} gaps; {out}")


if __name__ == "__main__":
    main()
