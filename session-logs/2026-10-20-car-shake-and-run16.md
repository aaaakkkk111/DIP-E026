# 2026-10-20: run 15 on the car, why the policies shake, run 16 set up

Laptop session. Logs: `car_logs/2026-10-20/` (`run15.txt`, `run13_recheck.txt`,
`pid_today.txt`). Battery 11.8-12.2 V (12.26 V on 2026-10-14).

## 1. Run 15 on the car

Flashed from `car_firmware_run15` (run-13 folder plus the run-15 headers;
test_policy PASS, worst error 1.9e-2). Forward-back wobble 43-49 deg/s mid-run.
It also wags left-right: yaw 30-34 deg/s, wheels opposite in 42-62 % of ticks,
against run 13's 13-19 deg/s and 9-19 % on 2026-10-14. The simulator gives
both runs the same yaw (7-12 deg/s), so this is not reproduced in sim.

**Not the calibration.** Run 15's cals had a low yaw-gyro bias (z 0.0060-0.0063
vs 0.0094-0.0100 before). Run 13 calibrated to the same low bias (0.0059-0.0069)
still twisted only about 20 deg/s, with 18-21 % opposite.

## 2. Today's car is worse, for every controller

| standing, mid-run | 2026-10-08/14 | today |
|---|---|---|
| stock PID (mode 1) | 7.6 deg/s | 11.7 |
| run 13 | 26-36 | 51 → 82 → 98 → 96 in about 6 minutes |

**The battery is not the cause, for the policies.** Simulated at 2-3 deg slack
and 10 ms, with the PWM scaled by V/12.3 (less torque, a relatively higher
dead zone):

| battery | PID | run 13 | run 15 |
|---|---|---|---|
| 12.3 V | 6.8-7.5 | 27-41 | 20-33 |
| 11.8 V | 10.1-10.2 | 23-40 | 16-27 |
| 11.0 V | 18 | 18-33 | 14-23 |

The PID's fixed gains lose out as the voltage drops. The policies' high
PWM barely notices.

## 3. The shake

From the P stream, which arrives in about 0.4 s bursts while running: almost all of
the policies' wobble on the car is **fast shake**, gyro sd after removing each
burst's trend.

| car | wobble | fast shake | PWM reversals |
|---|---|---|---|
| run 13, 10-14 | 27-39 | 26-37 | 11-15 /s |
| run 13, today | 51-98 | 48-96 | 5-14 /s |
| run 15, today | 44-49 | 43-47 | 15-17 /s |

The simulator agrees. At every slack, more than 95 % of the policies' wobble
is above 3 Hz, peaking at 7-10 Hz, with 15-30 reversals per second per wheel.
The mechanism: the policy reverses, the rotor crosses the gear slack and slams
into the far side, the jolt shows on the gyro, and the policy reverses again.
The PID changes its PWM by about 13 counts a tick and rarely reverses, so it
stays on one side of the gap.

Slack and delay sweep (sim, 12.3 V, deg/s):

| | PID | run 13 | run 15 |
|---|---|---|---|
| 2 deg, 10 ms | 6.8 | 27 | 20-24 |
| 4 deg, 10 ms | 51 (10.4 at 11.8 V) | 50-53 | 41-42 |
| 6 deg, 10 ms | 71 | 59-63 | 62-64 |
| 2 deg, 20 ms | | 44 | 34 |
| 4 deg, 20 ms | | 60 | 44 |

Today's car fits **about 4 deg** of slack at 11.8 V: PID 10.4 (car 11.7) and
run 13 43-50 (car's first run 51). Run 13's climb to 96 within minutes is not
reproduced even at 10 deg. Something got worse during the session: heat, the
battery sagging further, or the gearbox loosening. The logs can't tell which.

## 4. An output filter does not fix it

The firmware-style filter a_out = k·a_out + (1-k)·a was applied in sim without
retraining (the observation does not contain the action). It cuts reversals
from about 28/s to 6-10/s, but the wobble does not fall reliably:

| k | 0 | 0.5 | 0.7 | 0.8 | 0.9 |
|---|---|---|---|---|---|
| run 13, 2 / 4 deg | 27 / 54 | 33 / 51 | 28 / 52 | 25 / 40 | 21 / 55 |
| run 15, 2 / 4 deg | 22 / 43 | 22 / 39 | 23 / 37 | 19 / 51 | 30 / 80 (fails 3 of 6 pushes) |

With the lag, the loop runs slower and larger. Three seeds each, so changes
under about 10 deg/s are noise. Not flashed.

## 5. Run 16 (set up, for the desktop)

Run 15 plus:
- `REVERSAL_WEIGHT = 0.5`: charged per wheel per sign change of the command,
  against the last nonzero sign. Run 15's rate in sim (about 28/s per wheel) costs
  about 0.14/step; the PID's rate about 0.02. The old rate term barely sees a
  0.1 → -0.1 flip, which through the 1300-count compensation is a 2600-count
  swing.
- `GEAR_SLACK_DEG = (0, 6)` (was 0-4). Eval stays at 2 deg.
- From scratch, 30M steps (`RUN_NAME = "run16"`, `INIT_FROM = "none"`).

Check it with the slack sweep above. Worth flashing only if, at 2 and 4 deg:
- it stands calmer than run 15;
- it reverses less;
- it still survives 3 N pushes.

If it goes sluggish or stops surviving pushes, lower the weight to 0.2-0.3.
