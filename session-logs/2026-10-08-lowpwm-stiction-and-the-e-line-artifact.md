# 2026-10-08 — Low-PWM test, stiction, and why the car's wobble numbers were too high

Follows `2026-10-08-motor-reversal-test.md` §7.

## Summary

- **Low-PWM test** (80 two-level steps, `car_firmware_reversal`, car on the
  box). Near the dead zone the motor has **stiction with hysteresis**:
  - from rest it does not turn below about **1500**;
  - once turning, it keeps turning at **1420** (1.7–2.6 rad/s);
  - it follows small PWM changes 1.3–2× more slowly than the model.

  Fitted as a dead zone that depends on speed: **1485 at rest, 1400 above
  about 1 rad/s**, with Coulomb friction **0.011 N·m**. That cuts the fit
  error 2.6× against one fixed dead zone.
- **The stiction does not make the simulated policies wobble more.** Runs 9
  and 11 come out at 14–24 deg/s, and the stock PID still matches (6.5–6.7
  deg/s).
- **The car's "wobble" numbers for the policies were mostly the final
  push.** `simlib.car_stats()` uses the E lines: the flight recorder's last
  0.6 s before the run ends, cut where |pitch| first passes 8°. Every run
  ends with a push by hand. The 200 Hz stream of today's run 11 session
  shows, in pitch-rate sd:
  - **12–22 deg/s in the middle of the runs** (mean |PWM| 1500–1600), a slow
    sway peaking at 1–3 Hz;
  - 72–110 deg/s in the last 0.6 s;
  - **53–57 deg/s by the E lines.**

  **The simulator with the measured motor gives 14–25 deg/s and 1460–1560
  mean |PWM| for run 11. It matches the car's actual balancing.** The
  "2–3× under-prediction" chased all day was a measurement artifact.

## 1. The low-PWM test

`python sim_tools/motor_reversal_test.py --set lowpwm` →
`car_logs/2026-10-08/motor_lowpwm.csv` and `motor_lowpwm_raw.txt`.
- 80 steps, all complete.
- Battery (the car's own reading): 12.41 V.
- For each wheel and direction:
  - 2 s from rest at 1420, 1440, 1460, 1480, 1500 and 1520;
  - 1800, then down to 1420–1500;
  - small steps 1500 → 1600 and → 1700, 1600 → 1800, 1700 → 1500 and →
    1600, 2000 → 1500;
  - reversals ±1600, 1700 → −1500 and ±1500.

**Results** (`python sim_tools/motor_reversal_fit.py --lowpwm`; car against
the corrected motor of `2026-10-08-motor-reversal-test.md` §6):

| | car | model (dead zone 1395, Coulomb 0.019) |
|---|---|---|
| from rest at 1420–1480 | no motion; starts at 1500–1520 (one case at 1460) | creeps at 0.2–1.1 rad/s |
| turning, then 1420 | 1.7–2.6 rad/s | 0.2 |
| turning, then 1500 | 2.7–3.6 rad/s | 1.4–1.6 |
| small steps up, to half way | 80–115 ms | 65 ms |
| small steps down (1700 → 1600) | 145–200 ms | 85–95 ms |
| reversals ±1500–1600, to half way | 60–80 ms | 35–50 ms |

**Fit** (`sim_tools/stiction_check.py`):

    dead_zone(ω) = dz_kin + (dz_stat − dz_kin) · max(0, 1 − |ω|/w0)

- dz_stat **1485**, dz_kin **1400**, w0 **0.96 rad/s**, Coulomb **0.0112**;
- torque limit 0.75 and viscous 0.00054 kept;
- cost 7,211 against 18,613 for one dead zone of 1395 with Coulomb 0.0186.

It reproduces:
- no motion at 1480 from rest;
- 2.3 rad/s at 1520 (car 2.3);
- 1.0 rad/s after 1800 → 1420 (car 1.8, old model 0.2).

## 2. Stiction in closed loop (`stiction_check.py`)

| | car | simulator, measured motor with stiction |
|---|---|---|
| stock PID standing (2°, 5–15 ms; 3°, 5 ms) | 7.6 deg/s, 1461 | 6.5–6.7 deg/s, 1485–1489 |
| run 8 (2°, 5–15 ms; 3°, 10 ms) | 95.5 (E lines) | 51–107, 1931–2245 |
| run 9 | 68.2 (E lines) | 14–24, 1489–1559 |
| run 11 | 57.5 (E lines); **12–22 mid-run (stream)** | 14–19, 1462–1490 |

## 3. The E-line artifact

The run 11 session recorded with `stream_log.py`
(`car_logs/2026-10-08/run11_mode28_stream.txt`) has the stock gyro at about
50 Hz on average, irregularly sampled, through every run. Pitch-rate sd,
using Gyro_Balance minus its +40 offset, divided by 16.4:

| run | first 2 s | middle | last 1.5 s | last 0.6 s |
|---|---|---|---|---|
| 11.5 s | 3.0 | **15.3** | 71.4 | 92.4 |
| 20.5 s (switched off, no push) | 6.0 | **19.2** | 1.5 | 1.6 |
| 26.3 s | 27.1 | **13.1** | 64.9 | 71.6 |
| 26.3 s | 8.8 | **21.9** | 78.2 | 110.2 |
| 11.9 s | 24.4 | **11.9** | 60.7 | 83.9 |

The E lines of the same session give 52.7. Mid-run spectral peaks
(Lomb–Scargle) are 0.9–3.2 Hz.

**What this changes:**
- **Every car number from the E lines measures the push**, not the
  balancing:
  - runs 8 and 9 on 2026-10-07 (95.5 and 68.2 deg/s, "about 7 Hz");
  - run 11 at 15:55 today (57.5 deg/s);
  - the delay × slack fit (`fit_delay_slack.py`);
  - the criteria in `TRAINING_ON_DESKTOP.md` §3;
  - the reason for run 11's wobble penalty.

  Runs 8 and 9 were never streamed, so their real balancing wobble is
  unknown. Run 8 is wobbly in simulation (50–107 deg/s), so it may be
  genuine there.
- **Measure from the stream in the middle of a run**, away from arming and
  the push. Alternatively, stop pushing: switch the car off or lift it, and
  use only the stream.
- **For run 11 the simulator now matches the car's balancing.** The fit
  uses the measured motor, about 2° of slack and ≤ 15 ms of delay. The sway
  is slower on the car (1–3 Hz) than in simulation (7–10 Hz); the size is
  the same.

## 4. For the desktop

**Motor:** the table in `2026-10-08-motor-reversal-test.md` §6, plus the
speed-dependent dead zone above (1485 at rest, 1400 when turning,
w0 ≈ 1 rad/s, Coulomb 0.011). Implementing that needs a change to
`pwm_to_torque`. `stiction_check.torque()` is a reference version.

**Evaluating a policy on the car:** use `car_firmware_reversal` (stream on,
LOG_Poll fixed) with `sim_tools/stream_log.py`. Compare the mid-run
pitch-rate sd and mean |PWM| with `simlib.run()`, not `car_stats()`.

## Files

| file | what |
|---|---|
| `sim_tools/motor_reversal_test.py` | gained `--set lowpwm` |
| `sim_tools/motor_reversal_fit.py` | gained `--lowpwm` |
| `sim_tools/stiction_check.py` | stiction fit, PID and policies with it |
| `car_logs/2026-10-08/motor_lowpwm.csv`, `motor_lowpwm_raw.txt` | the test |
