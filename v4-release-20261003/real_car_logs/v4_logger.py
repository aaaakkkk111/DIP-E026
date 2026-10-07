# -*- coding: utf-8 -*-
"""Mode 27 (v4) serial recorder for v4_realtest2_diag_log.hex.

Records the 200 Hz tune_io stream to v4_log_<date>_<time>.csv and writes one line per
gear event to v4_events.txt:
  UPSHIFT  the tilt rule fired (lvl gains the +2000 recovery flag)
  BOOST    the slow-escape rule raised f while latched (no recovery flag)
  LATCH    probing ended, gear chosen
Columns: seq,gyro,ang_c,ml,mr,el,er,inj,osc_c,thref_c,lvl,cs,tv
  ang_c / thref_c = deg x100; lvl = f x100 + 1000 latched + 2000 recovery.
Commands: write a line into cmd.txt next to this script and it is sent.

    python v4_logger.py [COM3]
"""
import os
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
HERE = os.path.dirname(os.path.abspath(__file__))
CMD = os.path.join(HERE, "cmd.txt")
EVT = os.path.join(HERE, "v4_events.txt")


def open_port():
    while True:
        try:
            sp = serial.Serial()
            sp.port, sp.baudrate, sp.timeout = PORT, 230400, 0.05
            sp.dtr = False
            sp.rts = False
            sp.open()
            return sp
        except Exception:
            time.sleep(1.0)


def decode(lvl):
    rec = lvl >= 2000
    lvl -= 2000 if rec else 0
    latched = lvl >= 1000
    lvl -= 1000 if latched else 0
    return lvl / 100.0, latched, rec


def main():
    csv = open(os.path.join(HERE, time.strftime("v4_log_%Y%m%d_%H%M%S.csv")), "w", encoding="utf-8")
    ev = open(EVT, "a", encoding="utf-8")

    def say(s):
        ev.write(s + "\n")
        ev.flush()

    say("\n=== start %s ===" % time.strftime("%H:%M:%S"))
    sp = open_port()
    say("[connected %s]" % PORT)
    sp.write(b"s\r\n")             # stop any recording so KEY1 works; start with 'r' via cmd.txt
    buf = b""
    last_row = time.time()
    prev = None              # (f, latched, rec)
    f_latch = None
    hist = []                # last 1 s of rows for context
    while True:
        if os.path.exists(CMD):
            try:
                c = open(CMD, encoding="utf-8").read().strip()
                open(CMD, "w").close()
            except Exception:
                c = ""
            if c:
                try:
                    sp.write((c + "\r\n").encode())
                    say("> " + c)
                except Exception:
                    pass
        try:
            buf += sp.read(4096)
        except Exception:
            say("[serial lost %s]" % time.strftime("%H:%M:%S"))
            try:
                sp.close()
            except Exception:
                pass
            sp = open_port()
            say("[reconnected %s]" % time.strftime("%H:%M:%S"))
            buf = b""
            continue
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("ascii", "replace").strip()
            if not t:
                continue
            if t[0] == "#":
                csv.write(t + "\n")
                if not t.startswith("#start") and not t.startswith("#seq"):
                    say("< " + t)
                # Do NOT restart recording on a reboot: while the 200 Hz stream is
                # draining, the main loop polls the keys too rarely and the second
                # KEY1 (start balancing) is missed.  Start with 'r' in cmd.txt.
                continue
            p = t.split(",")
            if len(p) != 13:
                continue
            try:
                v = [int(x) for x in p]
            except ValueError:
                continue
            last_row = time.time()
            csv.write(time.strftime("%H:%M:%S") + "," + t + "\n")
            seq, gyro, ang, ml, mr, el, er, inj, osc, thref, lvl, cs, tv = v
            hist.append(v)
            if len(hist) > 200:
                hist.pop(0)
            cur = decode(lvl)
            if prev is not None:
                f, lat, rec = cur
                pf, plat, prec = prev
                stamp = time.strftime("%H:%M:%S")
                if rec and not prec:
                    mv = any(h[11] != 0 for h in hist[-200:])
                    say("%s UPSHIFT seq %d: f %.2f -> %.2f, angle %.2f, ref %.2f, err %+.2f deg, gyro %d, "
                        "cmd %d (moved in last 1 s: %s), osc %.1f"
                        % (stamp, seq, pf, f, ang / 100.0, thref / 100.0, (ang - thref) / 100.0, gyro, cs,
                           "yes" if mv else "no", osc / 100.0))
                if lat and not plat:
                    f_latch = f
                    say("%s LATCH seq %d: f %.2f, osc %.1f" % (stamp, seq, f, osc / 100.0))
                if lat and plat and not rec and f_latch is not None and f > f_latch + 0.05 and pf <= f_latch + 0.05:
                    say("%s BOOST seq %d: f %.2f (latched %.2f), angle %.2f, ref %.2f, cmd %d"
                        % (stamp, seq, f, f_latch, ang / 100.0, thref / 100.0, cs))
            prev = cur
        csv.flush()


if __name__ == "__main__":
    main()
