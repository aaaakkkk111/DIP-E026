# 2026-10-08 — Motor reversal test, corrected motor model, runs 9 and 11 still too calm

Follows `2026-10-08-stock-pid-closed-loop.md` §6. The stock PID had
confirmed the simulated hardware, but it hardly ever reverses the motors,
while the policies do it every half wobble. So the two had to be checked
separately.

## Summary

- **The motor model was wrong in two ways that the one-level step test and
  the PID could not show** (48 two-level steps on the box):
  - **Reversing a fast wheel:** the real motor stops it about 2× faster
    (70–75 ms against 130–140 ms from full speed). The fit puts the torque
    limit at **0.75 N·m**; training uses 0.4.
  - **Losing speed:** the real wheel slows about 2× faster at speed, from
    viscous friction of **0.00054 N·m·s/rad**; the model has none.

  Adding both cuts the fit error 6×.
- **With the corrected motor:**
  - the stock PID still matches the car (6.3–6.8 against 7.6 deg/s);
  - **run 8 now matches the car** (98–106 against 95.5 deg/s at 2–3° and
    10–15 ms);
  - **runs 9 and 11 are unchanged and still 2–3× too calm** (19–31 against
    68 and 57.5 deg/s).
- **The one motor error still unmodelled is near the dead zone:**
  - the wheel does not turn at all at PWM 1450;
  - between 1500 and 1700 it reacts 1.5–3× more slowly than the model;
  - reversals that end at −1500 take 185–275 ms on the car against
    145–235 ms in the model.

  Runs 9 and 11 spend 41–44 % of their ticks below 1700, against 26 % for
  run 8. That makes this the prime suspect for the gap that remains.

## 1. First, the pushes in the PID log (inconclusive)

`sim_tools/pid_sim.py --pushes` simulates the stock PID under a horizontal
push sized to match each car push's swing-back peak rate.
- **Backward push:** the only one that could be matched agrees roughly.
  The car went to +22.0°, back to 0 in 640 ms, with the wheel at 40
  counts/tick at reversal. The simulator gave +28.3°, 585 ms and 57.
- **Forward pushes:** the car rolled forward at up to 45 counts/tick while
  leaning only 4.6°, then swung back to −22°. No simple simulated push
  reproduces that: pushes at the top tip the simulated car over, and at
  body height the wheels reach only 29–31 counts/tick at 33° of lean.

The hand's push is an unknown input, so a controlled test followed.

## 2. Firmware: `car_firmware_reversal`

`car_firmware_pidlog` plus three changes, all in `rl_mode.c`
(`firmware/reversal_step.patch`, against `car_firmware_pidlog`):
- **Two-level step:** `mt L|R <pwm1> <pwm2>` runs 200 ticks at pwm1, 200
  at pwm2 and 100 at 0, which is 500 `M` lines. One-level steps work as
  before.
- **Stream pause:** the 200 Hz stream is off during an `mt` session.
- **Bounded LOG_Poll:** at most 16 lines per call, so the main loop no
  longer stalls (the bug found in §6 of the PID log).

**Build:** 0 errors and the usual 4 warnings. The image ends at
0x0800FED0, 304 bytes under 64 KB.

## 3. The test (`sim_tools/motor_reversal_test.py`)

The car stood upright on a box with its wheels free, in mode 28, after
KEY1. The battery, read by the car, was 12.44 V before and 12.41 V after.

**Steps:** for each wheel and direction, from ±2000 and from ±2800:
- to 0 (coast);
- to −1500, −1700, −2000 and −2800 (reversals);
- to 1600 or 1800 (slowing down without reversing).

That is 48 steps, all complete with no cuts. Output:
`car_logs/2026-10-08/motor_reversal.csv` and `motor_reversal_raw.txt`.

The first attempt stopped at once: the script's `mt on` arrived while the
car was starting up, came out garbled, and the car answered `?`. The script
now gives up only after three `?` answers.

## 4. Car against the model (`sim_tools/motor_reversal_fit.py`)

Times are in ms after the PWM change. Ranges cover both wheels and
directions:

| after the change | car | fitted A (0.4 N·m) | + limit 0.75 and viscous |
|---|---|---|---|
| 2800 → −2800, to zero | 70–75 | 130–140 | 70 |
| 2000 → −2800, to zero | 35–40 | 55–60 | 35 |
| 2800 → −2000, to zero | 110 | 135–145 | 105 |
| 2800 → −1500, to zero | 270–275 | 220–235 | 205 |
| 2800 → 0, speed after 1 s (counts/tick) | 12–15 | 24–27 | 15 |
| 2800 → 1800, speed after 1 s | 16–20 | 24–27 | 15 |

**Median time to half way, car / model A:** reversals 0.93, coast 0.84,
slow-down 0.81. The fast reversals are where the model is clearly too
weak; weak reversals are the other way round.

**Fit** (`fit_extension`, both wheels, the rest of model A kept):
- torque limit **0.750 N·m** (motor_model `TAU_DRIVER_LIMIT` and the
  actuator ctrlrange are both 0.4);
- viscous friction **0.00054 N·m·s/rad**;
- cost 3,443 against 20,858 for model A.

The torque limit also explains why the car spins up a little faster than
model A from rest at 2800 (90–95 ms against 100–110 ms to 63 %).

## 5. With the corrected motor (`sim_tools/extended_motor_check.py`)

**Stock PID, standing:**

| | 2°, 5–15 ms | 3°, 5 ms | 3°, ≥ 10 ms |
|---|---|---|---|
| model A | 6.2–6.7 | 6.8 | 33–53 at 10 Hz |
| corrected motor | 6.3–6.8 | 7.0 | 33–53 at 10 Hz |

The car's 7.6 deg/s is still matched at about 2° of slack.

**Policies** (pitch-rate sd, deg/s, standing; 2–3°, 5–15 ms):

| | car | model A | corrected motor |
|---|---|---|---|
| run 8 | 95.5 | 49–96 | 51–106 (106 at 2° / 15 ms, 105 at 3° / 10 ms) |
| run 9 | 68.2 | 19–30 | 19–31 |
| run 11 | 57.5 | 19–26 | 19–25 |

## 6. For the desktop: the motor as now measured

| | value | where it comes from |
|---|---|---|
| dead zone | 1395 | step test, symmetric |
| tau_scale (with kv_scale 1) | 1.07 / 1.11 | step test, at about 12.5 V |
| Coulomb friction | 0.019 / 0.018 N·m | step test |
| armature | 1.45e-3 / 1.51e-3 kg·m² | step test |
| **torque limit** | **0.75 N·m** | reversal test; was 0.4 |
| **viscous friction** | **0.00054 N·m·s/rad** | reversal test; new term, −b·ω on the rotor |
| gear slack | about 2° (≤ 3) | stock PID and the hand measurement |
| delay | ≤ 15 ms | stock PID |
| pitch-zero error | up to ±3–4° | hand calibrations today; training covers ±2° |

The limit is applied twice: in `pwm_to_torque` (`TAU_DRIVER_LIMIT`) and in
`real_robot.xml`'s actuator ctrlrange. Both must change together.

## 7. What is still missing

The low-PWM region:
- static friction, so no motion at 1450;
- a slow response between 1500 and 1700;
- weak braking when a reversal ends at −1500.

That region is where runs 9 and 11 spend their time. The
`car_firmware_reversal` build already does two-level steps, so measuring it
needs no new firmware, only new step lists for `motor_reversal_test.py`.
Examples: 1700 → 1500, 1500 → 1600, 2000 → 1500, and the 1440–1500
threshold, from rest and from motion.

## Files

| file | what |
|---|---|
| `firmware/reversal_step.patch` | `car_firmware_reversal` against `car_firmware_pidlog` |
| `sim_tools/motor_reversal_test.py` | runs the 48 two-level steps |
| `sim_tools/motor_reversal_fit.py` | car against model, and the torque-limit and viscous fit |
| `sim_tools/extended_motor_check.py` | PID and policies with the corrected motor |
| `sim_tools/pid_sim.py` | gained `--pushes` |
| `car_logs/2026-10-08/motor_reversal.csv`, `motor_reversal_raw.txt` | the test |

The car now runs `car_firmware_reversal` (run 11 weights in mode 28, stream,
`mt` commands). To go back, flash `car_firmware_run11`'s hex.
