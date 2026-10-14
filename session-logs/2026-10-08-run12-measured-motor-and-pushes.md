# 2026-10-08 (desktop, evening) — Run 12: measured motor, no wobble penalty, pushes

Follows the laptop's `2026-10-08-motor-step-test.md`,
`-stock-pid-closed-loop.md`, `-motor-reversal-test.md` and
`-lowpwm-stiction-and-the-e-line-artifact.md`. Done on the desktop.

## Summary

- **The simulator now uses the motor measured on the car**, with stiction.
  The stock PID run through it matches the car standing still: 6.6–6.9
  against 7.6 deg/s. Run 11 matches the car's real mid-run balancing:
  13.5–21 against 12–22 deg/s.
- **The wobble penalty is off.** It had been added for a wobble that turned
  out to be the hand push at the end of each run (the E-line artifact). On
  the corrected motor it bought 1.3 deg/s of calm and cost a quarter of the
  push tolerance.
- **Training now includes pushes.** On the same motor and pushed the same
  way, the stock PID survives 3 N × 0.1 s, run 10 2 N and run 11 1 N.
- **Run 12** trained from scratch, 30M steps. Compared with runs 10 and 11 on
  the measured motor:
  - it **survives pushes far better**: 3 N in 14 of 18 trials, against 0 of
    18 for runs 10 and 11, and 17 of 18 for the stock PID;
  - it **tracks better**: 97–100 % against 88–93 %;
  - it **wobbles about twice as much standing**: 25–35 against 11–16 deg/s
    at 2° (§6).
- **Windows efficiency mode had throttled training to a quarter of its
  speed.** `train_real_robot.py` now opts out of it.

## 1. A first run 12, stopped

The first run 12 started on the motor of `-motor-reversal-test.md` §6:
- a single dead zone of 1395;
- Coulomb friction 0.019;
- the run-11 wobble penalty kept.

It was stopped at 1.25M steps (best eval 1069) when the low-PWM test arrived.
Its logs are in `logs/run12_aborted/` and `training_run12_aborted.log`.
`models/best_real/` was restored to run 11 from git.

## 2. Motor, slack, delay (`motor_model.py`, `real_robot.xml`, `train_real_robot.py`)

| | value | training range | was |
|---|---|---|---|
| dead zone, turning | 1400 | 1360–1440 | 1460 (1350–1600) |
| dead zone at rest (stiction) | 1485, blending to the turning value by 0.96 rad/s | 1450–1520 | — |
| tau_scale | 1.09 | 0.95–1.15 | 1 (0.85–1.05) |
| Coulomb | 0.011 N·m | 0.007–0.015 | 0.0025 (0.002–0.02) |
| viscous | 0.00054 N·m·s/rad, after the torque clip | 0.00035–0.00075 | — |
| armature | 1.5e-3 kg·m² | 1.1e-3 – 2.0e-3 | 2.25e-3 (9e-4 – 4.5e-3) |
| torque limit | 0.75 N·m, `TAU_DRIVER_LIMIT` and `ctrlrange` | — | 0.4 |
| gear slack | eval 2° | 0–4° | eval 5° (0–8°) |
| delay | eval 10 ms | 5–15 ms | eval 15 ms (5–30 ms) |
| pitch-zero error | — | ±4° | ±2° |

**How the new terms are built:**
- `MotorParams` gained `viscous` and `dead_zone_static`. Both default to
  None, so motors built the old way, as in the laptop's fit scripts that add
  their own terms, behave as before.
- `pwm_to_torque` equals `sim_tools/stiction_check.torque()` exactly: zero
  difference over 15,933 (PWM, ω) points.

**Note for the laptop:** `motor_fit.py` and `motor_reversal_fit.py` read
`TAU_DRIVER_LIMIT` at import, so if re-run they now use 0.75 N·m. Their
results were fitted at 0.4.

## 3. Checks against the car

**`sim_tools/extended_motor_check.py`** reproduces the laptop's log exactly.
It uses its own fitted motor, without stiction: stock PID 6.3–6.8 deg/s at
2°, runs 9 and 11 19–21.

**Training defaults, no overrides** (stock PID 30 s; policies 10 s, 2 seeds):

| | car | 2°, 5–15 ms | 3°, 5–10 ms | 3°, 15 ms |
|---|---|---|---|---|
| stock PID, deg/s / mean \|PWM\| | 7.6 / 1461 | 6.6–6.9 / 1486 | 7.4–7.7 / 1488 | 54 at 10 Hz |
| run 9 | E lines only (push) | 16.1–19.3 | 23.1 | 24.3 |
| run 11 | 12–22 mid-run (stream), 1500–1600 | 13.5–14.4 / 1469 | 20.2 / 1504 | 21.1 |

The stock PID's 10 Hz limit cycle at 3° and 15 ms, which the car does not
show, keeps the car's slack near 2°.

## 4. The wobble penalty, and pushes

Run 10 (no penalty) against run 11 (penalty 1.5 per (rad/s)²), on the
training defaults at 2° and 10 ms.

**Standing and tracking:**
- standing: 14.7 against 13.4 deg/s;
- tracking: forward 92 against 89 %, reverse 93 against 92 %, turning 100 %
  for both.

**Pushes:** a force 10 cm above the axle, applied as `pid_sim.sim_push` does
(the force at the COM plus its torque), for 0.1 s. Largest push survived in
both directions, and the peak pitch rate:

| | largest push | peak pitch rate |
|---|---|---|
| stock PID | 3 N | 241 deg/s (the car's PID recovered from 184–287 and fell at 407) |
| run 10 | 2 N | 149 deg/s |
| run 11 | 1 N | 73 deg/s |

The penalty taught the policy to avoid the fast pitch rates a recovery needs,
so it is off for run 12 (`PITCH_RATE_WEIGHT = 0`; the code is kept).

**Pushes in training** (the user chose this):
- **Size and timing:** 0–4 N, 0.05–0.15 s, 5–15 cm above the axle, every
  1.5–4 s.
- **Direction:** along the car's own heading, forward or back.
- **Phase-in:** the size ramps in with the velocity curriculum, through
  `CurriculumCallback` and `set_push_curriculum`.
- **Scope:** training only. Evaluation, and simlib's test cars, stay
  push-free.

Checked:
- about one push per 3 s;
- nothing carries over a reset;
- the eval env never pushes;
- the curriculum reaches the wrapper through Monitor and TimeLimit.

## 5. Run 12

From scratch, 30M steps, 19 envs: `python train_real_robot.py`, with
`RUN_NAME` run12 and `INIT_FROM` none.

**Windows efficiency mode.** Throughput started at about 6,500 steps/s and
fell to about 1,500 at 21:28, three minutes in. Every worker was about 86 %
idle and the CPU at 28 %: Windows 11 had put the background training
processes into efficiency mode (EcoQoS), on the i7-12700KF's efficiency
cores.
- Raising the priority gave 1,770 steps/s.
- Opting the 22 processes out of power throttling
  (`SetProcessInformation`, ProcessPowerThrottling) brought back
  6,485–7,782 at once.
- `train_real_robot.py` now does this at import, for the main process and
  every worker. Read back: ControlMask 0x5, StateMask 0.

About 40 minutes were lost.

**Eval** (no pushes, 2° and 10 ms, full command range):

| steps | 1.5M | 2.75M | 4.25M | 6.25M | 8.25M | 10.25M |
|---|---|---|---|---|---|---|
| reward | 956 | 478 | 1172 | 1895 | 2699 | 2764 |
| mean episode length (of 2000) | 1588 | 1094 | 1949 | 1271 | 1744 | 1824 |

The stand phase runs to 4.5M. The dip after it is the command and push
ramp.

**Training episodes** (with pushes, rolling mean):
- 3680 / 2000 ticks while standing;
- falling to 1714 / 1315 by 15M, as commands and pushes reach full strength;
- then recovering to 2865 / 1947 at the end.

PPO stayed healthy throughout: KL about 0.0045, explained variance about
0.89, action std 0.12 → 0.16.

**Eval, second half:**
- full-length episodes at every eval from 17.75M;
- best **3577 at 29.75M**, final 3570.

Run time 8,354 s: 2.3 h, including the 40 min throttled.

Backups:
- `models/best_real_RUN12/` holds the best checkpoint (29.75M);
- `models/ppo_real_robot_RUN12_30M.zip` holds the final one;
- training curve: `session-logs/train_real_robot_RUN12_measured_motor_pushes_progress.csv`.

## 6. Run 12 against runs 10 and 11 (training defaults, measured motor)

**Standing still** (`compare_models.py --slack 2 3 --delay 1 2 3`;
pitch-rate sd in deg/s):

| | run 10 | run 11 | run 12 best | run 12 final |
|---|---|---|---|---|
| 2°, 5–15 ms | 13.1–14.1 | 11.2–15.8 | 25.2–34.8 | 25.6–35.4 |
| 3°, 5–15 ms | 21.2–24.2 | 19.5–20.0 | 33.4–42.7 | 34.3–44.1 |

None ever reaches full PWM. Run 12 best on 8 training-like cars: all held,
4.0–34.2 deg/s, drift −0.018 to +0.058 m/s.

**Pushes:** 2° slack, 10 cm above the axle, 0.1 s. Survivals out of 18:
3 seeds × delays of 5, 10 and 15 ms × both directions.

| | 2 N | 3 N | 4 N | 5 N |
|---|---|---|---|---|
| stock PID | 18 | 17 | 0 | 0 |
| run 10 | 18 | 0 | 0 | 0 |
| run 11 | 2 | 0 | 0 | 0 |
| run 12 best | 18 | 12 | 0 | 0 |
| run 12 final | 18 | **14** | 0 | 0 |

**Tracking** (2°, 10 ms, steady state; forward / reverse / turn, then the
drift standing still):
- run 10: 93 / 91 / 100 %, drift +0.002 m/s;
- run 11: 88 / 93 / 100 %, drift −0.002 m/s;
- run 12 best: 100 / 98 / 100 %, drift +0.013 m/s;
- run 12 final: 99 / 97 / 100 %, drift +0.009 m/s.

**Reading:**
- Push training gave run 12 the stock PID's tolerance at 3 N, which runs 10
  and 11 never survive, and better tracking.
- The price is a harder-working balance: about twice the standing wobble of
  runs 10 and 11.
- The simulator matched run 11's real mid-run wobble (13.5–21 against 12–22
  deg/s). So on the car, expect run 12 at roughly 25–35 deg/s standing:
  visible, but far from run 8.
- The final checkpoint is the better candidate: more push tolerance, the
  same wobble and tracking.

## 7. Run 13 (2026-10-09, just after midnight): run 12 plus a mild wobble penalty

**Setup:**
- fine-tuned from run 12's final checkpoint
  (`models/ppo_real_robot_RUN12_30M.zip`);
- 10M steps at learning rate 1e-4;
- pushes at full strength from the start;
- `PITCH_RATE_WEIGHT = 0.5`, a third of run 11's.

At that weight, run 12's standing wobble (26.7 deg/s rms at 2°, 10 ms) costs
0.109 per step, against 0.020 at run 11's 11.5 deg/s.

**Training:**
- 2,435 s (41 min), with no errors;
- the built-in efficiency-mode opt-out held throughput at 6,485 steps/s
  well past the 3-minute mark.
- Training episodes, with pushes: 1,484 / 1,183 ticks at the start, 2,792 /
  1,992 at the end. The policy almost stopped falling to pushes.
- Eval: best 3498 at 0.5M; the final eval scored 3471.
- Backups: `models/best_real_RUN13/` (the 0.5M checkpoint) and
  `models/ppo_real_robot_RUN13_10M.zip` (final).

**Against run 12** (training defaults):

| | run 12 final | run 13 best (0.5M) | run 13 final |
|---|---|---|---|
| standing wobble, 2° (5–15 ms), deg/s | 25.6–35.4 | 27.8–29.4 | 23.8–32.8 |
| standing wobble, 3° (5–15 ms), deg/s | 34.3–44.1 | 35.1–36.2 | 34.6–47.9 |
| survives 3 N (of 18) | 14 | 15 | **18** |
| survives 2 N / 4 N (of 18) | 18 / 0 | 18 / 0 | 18 / 0 |
| tracking fwd / rev (9 runs) | 100 / 97 % | 94 / 98 % | 99 / **90** % |
| standing drift (15 runs) | +0.011 m/s (max 0.017) | +0.009 (0.013) | **+0.024** (0.036) |

The stock PID survives 3 N in 17 of 18 trials.

On 8 training-like cars, run 13 final held all 10 s, at 3.6–37.6 deg/s.

**Reading:**
- **The 0.5 penalty did not calm standing.** All three checkpoints sit at
  about 25–35 deg/s at 2°. The extra 10M steps of push training made run 13
  final the most push-tolerant policy so far, slightly ahead of the stock
  PID.
- **Its costs:**
  - a forward creep of about 2.4 cm/s standing. That is roughly what 1.5° of
    pitch-zero error causes, well inside the scatter of hand calibration, and
    `trim` corrects it;
  - 90 % reverse tracking.

**Chosen: run 13 final.** It is copied to `models/best_real/best_model.zip`
and exported:
- `export_stm32.py` → `policy_weights.h` and `policy_testvectors.h`;
- `quantize_weights.py` → `policy_weights_q.h` (worst int16 rounding 2.5e-5).

PC checks:
- `test_policy`: fast tanh 1.88e-2 against a tolerance of 5e-2, PASS; exact
  1.8e-7, PASS;
- `check_obs_builder.py`: PASS (worst 5.4e-7).

Run 12 final is the fallback if the creep matters on the car.

Docs updated: `README.md` (status), `TRAINING_ON_DESKTOP.md` (run 13, new
car criteria) and `sim_tools/compare_models.py` (defaults: runs 11, 12, 13).

## Open

- **Run 13 on the car,** with the 200 Hz stream (`stream_log.py`):
  - judge the wobble mid-run, not from the E lines;
  - try a few hand pushes;
  - expect about 25–35 deg/s standing (run 11 was 12–22) and a small forward
    creep.
- **If the standing wobble is too much on the car:** a calmer push-tolerant
  policy needs more than a fine-tune. Options are a stronger wobble penalty
  (about 1.0) together with pushes, or training from scratch with both. The
  0.5 fine-tune moved push tolerance, not calm.
