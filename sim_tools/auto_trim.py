"""Find the balance-point trim on the car, live, over the USB cable.

    python sim_tools/auto_trim.py --name autotrim_1 [--start 1.1] [--seconds 900]

Logs everything like stream_log.py (car_logs/<date>/<name>.txt). After each
`cal done` it sends `trim <start>`. While the car balances (state RUN), it
averages the forward speed from the D lines over 10 s windows with no push
(|pitch| < 8 deg) and nudges the trim against the creep: creeping back -> trim
up. The car's trim command is incremental. On the car (2026-10-21) the creep
was about 1 cm/s per degree (trim 0: -1.2 cm/s, trim 1.5: +0.4).
"""
import argparse
import datetime
import os
import time

import serial

WINDOW_S = 10.0
PUSH_DEG = 8.0
GAIN = 0.5            # deg per cm/s: half the measured slope, so one noisy window can't overshoot
STEP_MAX = 0.5        # deg per nudge
TRIM_MAX = 4.0        # deg total
DEADBAND_CM_S = 0.3   # below this the creep is within the speed estimate's noise


def nudge(v_cm_s, trim):
    """Trim change (deg) for a window's mean speed, keeping the total within TRIM_MAX."""
    if abs(v_cm_s) < DEADBAND_CM_S:
        return 0.0
    step = max(-STEP_MAX, min(STEP_MAX, -GAIN * v_cm_s))
    return max(-TRIM_MAX, min(TRIM_MAX, trim + step)) - trim


assert nudge(-1.2, 0.0) == 0.5 and abs(nudge(0.4, 1.5) + 0.2) < 1e-9 and nudge(0.1, 1.0) == 0.0
assert abs(nudge(-5.0, 3.8) - 0.2) < 1e-9     # capped at TRIM_MAX


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--start", type=float, default=1.1, help="trim sent after each cal, deg")
    ap.add_argument("--seconds", type=float, default=900)
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    day = datetime.date.today().isoformat()
    os.makedirs(os.path.join(root, "car_logs", day), exist_ok=True)
    path = os.path.join(root, "car_logs", day, args.name + ".txt")
    sp = serial.Serial()
    sp.port, sp.baudrate, sp.timeout = args.port, 230400, 0.05
    sp.dtr = False      # FlyMCU auto-reset / BOOT0 lines: keep low
    sp.rts = False
    sp.open()
    print(f"recording {args.port} to {path}", flush=True)

    trim, win, t_end, buf = 0.0, [], time.time() + args.seconds, b""
    with open(path, "a", encoding="utf-8") as fh:
        def send(cmd):
            sp.write((cmd + "\r\n").encode())
            fh.write(f"{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} > {cmd}\n")

        while time.time() < t_end:
            buf += sp.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                s = raw.decode(errors="replace").strip()
                if not s:
                    continue
                now = time.time()
                fh.write(f"{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} {s}\n")
                if s.startswith("cal done"):
                    trim, win = args.start, []
                    send(f"trim {args.start}")
                    print(f"cal done -> trim {trim:.2f} deg", flush=True)
                elif s.startswith("pitch zero now"):
                    print(f"  car: {s}", flush=True)
                elif s.startswith("D "):
                    f = s.split()
                    try:
                        state, pitch, v = int(f[2]), float(f[3]), float(f[4])
                    except (IndexError, ValueError):
                        continue
                    if state != 2 or abs(pitch) > PUSH_DEG:
                        if win and state == 2:
                            print("  push or fall: window restarted", flush=True)
                        win = []
                        continue
                    win.append((now, v))
                    if now - win[0][0] >= WINDOW_S:
                        v_cm = 100 * sum(x[1] for x in win) / len(win)
                        d = nudge(v_cm, trim)
                        print(f"creep {v_cm:+.2f} cm/s at trim {trim:.2f} -> "
                              + (f"trim {d:+.2f}" if d else "keep"), flush=True)
                        if d:
                            send(f"trim {d:.2f}")
                            trim += d
                        win = []
    sp.close()
    print(f"final trim {trim:.2f} deg", flush=True)


if __name__ == "__main__":
    main()
