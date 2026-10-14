# 2026-10-14 (laptop) — Run 13 on the car, simulator check, run 14 set up

Follows `2026-10-10-turning-calibration-mujoco.md`. Car logs:
`car_logs/2026-10-14/`.

## Summary

- **Run 13 on the car balances well but stands jerkier than run 11:** 26–36
  deg/s mid-run against run 11's 12–22. That is what the simulator predicted
  (23–37).
- **The simulator matches the car for three controllers** (the stock PID, run
  11, run 13), so it can be trusted to judge calm.
- **The stock PID is calm (7.6 deg/s) and recovers from hard hand pushes on
  the same car.** Run 14 aims for that: from scratch with pushes and the
  wobble penalty back at 1.5. Not trained yet.

## 1. Firmware: `car_firmware_run13`

`car_firmware_reversal` (stream, LOG_Poll fix) with the run-13 headers and
the two edits from the 2026-10-10 log in `APP/RL/rl_mode.c`:
- `CMD_W 1.00f`;
- `pitch_zero = roll_zero = 0.0f;` after `att_from_acc(ab, &roll_zero, &pitch_zero);`.

Keil: 0 errors, 4 stock warnings. Image end `0x0800FEE0`, **288 bytes** under
the 64 KB this board loads reliably, so the next addition may not fit.

## 2. On the car (`stream_log.py`)

`run13_jerk.txt`, standing, and `run13_push.txt`, pushes and app turns.
Runs that end in a reset were ended with the RESET button (asked), not a
brownout.

| | run 13 |
|---|---|
| runs | 39, 25+, 12, 58, 27, 12 s; no falls except pushed over |
| mid-run pitch-rate sd | 27–28 deg/s standing; 26–36 with turns and pushes |
| mean \|PWM\| | 1500–1550, never full |
| creep | +0.000 / +0.003 m/s over whole runs (pitch zero fixed at 0) |
| pushes | recovered from the 2 pushes given mid-run (gentle: at most ~8° tilt) |
| turning (wheel encoders) | up to ~1.0–1.15 rad/s |

`cal` with the fixed zero gave `pitch zero 0.00`, bias about −0.04 rad/s.

Caveat: during runs the stream delivered only ~13 % of ticks (gaps of 3–100
ticks), so the wobble size is a sample statistic and the frequency is not
usable. The encoder yaw cannot tell app turns from left-right wagging.

## 3. Simulator against the car

Same measure (pitch-rate sd while balancing), simulator at the measured 2°
slack and 5–15 ms (`simlib.run`, 2 seeds; PID from
`pid_sim.py --log car_logs/2026-10-08/pid_mode1.txt --sim`):

| | car | simulator |
|---|---|---|
| stock PID | 7.6 deg/s, \|PWM\| 1461 | 6.2–6.9, 1486–1502 |
| run 11 | 12–22 | 12.3–15.0, 1464–1483 |
| run 13 | 26–36, ~1530 | 23.0–36.5, 1585–1646 |

The PID pins down slack and delay: in simulation it shakes at 3° and 15 ms
(52–54 deg/s), yet on the car it is calm. So the car has about 2° and ≤ 10–15
ms. The 2026-10-08 estimate of 4–6° at 10–20 ms came from the old motor model
and the E-line push artifact.

On the car the PID recovered from 8 of 10 hard hand pushes, peaking at 22–30°
of tilt.

## 4. Run 14 (set up, not trained)

`train_real_robot.py`: `RUN_NAME = "run14"`, `INIT_FROM = "none"` (from
scratch, 30M), `PITCH_RATE_WEIGHT = 1.5`, pushes as in runs 12–13. A 20k-step
smoke run trains.

Why 1.5 and from scratch:
- run 13's 0.5 fine-tune moved push tolerance, not calm (2026-10-08 log §7);
- run 11's 1.5 bought calm but cost push tolerance only because it never saw
  pushes;
- a fine-tune from run 13 would keep run 13's way of balancing.

**Flash it only if, at 2° and 5–15 ms:** standing wobble ≤ ~10 deg/s and 3 N ×
0.1 s survived in ≥ 15 of 18 (PID: 6.6–6.9 deg/s, 17). Calm but not
push-proof → try 1.0; push-proof but not calm → stronger.

`sim_tools/compare_models.py` defaults: runs 11, 13
(`models/ppo_real_robot_RUN13_10M.zip`) and `models/best_real`.

## Also in this commit

- `sim_tools/stream_log.py`: commands typed in its terminal go to the car
  (`st`, `trim`, `cal`), and it reads what is waiting instead of blocking.
  This was an uncommitted local change from 2026-10-08, used for today's
  logs.
- `car_logs/2026-10-08/run11_baseline.txt`: an uncommitted car log from
  2026-10-08.
