# 2026-10-08 — Run 8 and run 9 on the car, delay and gear slack, run 10

Follows `2026-10-07-car-test-data-collection.md`. Car logs:
`car_logs/2026-10-07/` (all times below are the logger's PC time that day).

## Summary

- Run 8 ran on the car for the first time and **balanced for up to 51 s**,
  but wobbled at about 7 Hz with the motors swinging close to full power.
- The simulator reproduces that wobble only with **~30 ms of sense-to-act
  delay** (training assumed 5–10 ms). Run 9 was trained over 5–40 ms. It is
  calm in simulation at every delay, and on the car it was **gentler but
  still wobbled** at 7.2 Hz.
- Delay alone cannot explain run 9's wobble. Fitting both runs' car behaviour
  in a simulator with **gearbox slack** puts it at ~4–6° at the wheel with
  10–20 ms of delay. Run 10 (`train_real_robot.py`, not trained yet) trains
  with slack 0–8° and delay 5–30 ms, continuing from run 9.

## 1. Mode-28 firmware rebuilt without the team's Keil project

The team's `rl_mode28_20261003.zip` has their mode-28 sources but not the
whole project, and six stock files they patched (their README §2.3) are not
in it. `firmware/assemble_mode28_project.py` rebuilds the project from the
stock Yahboom `stm32_Balance_Car_L` + the team's `src/` + those patches,
re-created from the README (every change marked `mode 28`):

| file | change |
|---|---|
| `myenum.h` | modes 21–28 (`Load_*`, `Load_Adapt`, `Adapt_V4`, `RL_Policy`) |
| `app_mode.c` | the wheel menu toggles between modes 1 and 28; `RL_Reset()` on selecting 28 |
| `oled_show.c` | `28.RL Policy` |
| `AllHeader.h` | includes `rl_mode.h` and a new `removed_modes21_27.h` (prototypes for the stubs) |
| `main.c` | mode 28: wait for `cal` showing `CAL hold nn%`, console polled while waiting for KEY1, main loop with Bluetooth, `TUNE_Poll()` and an OLED status line (`ARM p…` / `L… R…`) |
| `bsp.c` | USART1 at 230400 (the logger's rate); Bluetooth and `TUNE_Init()` in mode 28 |
| `usart.c` | received bytes go to `TUNE_RxByte()` (the receive interrupt is only enabled in mode 28) |

The stock project in the Yahboom download is the team's base: their startup
file differs only in the stack size, their project file only in the RL files
and the removed modes. With the repo's run-8 files and Step 1 of the
deployment README applied: 0 errors (4 stock warnings), image end
`0x0800FA78`, 1,416 bytes under the 64 KB this board loads reliably (team v6:
644 bytes). On the car the timing matched the team's v6: **0 late ticks**,
worst interrupt 4.4–4.5 ms, longest gap 5.2 ms, in every run.

## 2. Running the car: what went wrong first

- **Held starts.** Mode 28 arms itself after 1 s upright and still. Runs that
  started in the hand ran the wheels up with the body held and were cut by
  the free-spin check after ~0.65 s (17 of the first 20 runs, reason 3).
  Let go as soon as the OLED line changes from `ARM` to `L… R…`.
- **Calibration.** The `cal done` pitch zero is the raw sensor angle at the
  balance point, about **3.2–3.5° on this car** (sensor mounting and centre of
  mass), not 0. Calibrations held "upright by eye" landed near 0° (and once
  at −6.7°). Fastest fix: `trim` to about 3.2° (±5° per command, one command
  at a time). With the zero at 3.22° the 51 s run averaged +0.16° pitch and
  +0.006 m/s.
- **Auto-trim is on every time the logger starts** (`auto = True` in
  `rl_trim_helper.py`): write `auto off` to `cmd.txt` after each start.
- **The logger prints nothing to its terminal.** Watch
  `Get-Content rl_runs_summary.txt -Wait -Tail 5`.
- **Status lines alias.** During a run the main loop gets ~10 % of the CPU,
  so the `D` lines come every ~0.52 s, not 0.1 s. A 7 Hz wobble then looks
  like a 2–4 s sway. Judge the wobble from the 200 Hz `E` lines, and end runs
  by **pushing the car over**: catching, lifting or switching off leaves no
  balancing in the recording.

## 3. Run 8 on the car (15:39–15:41, three runs on a table)

| | run 8 on the car |
|---|---|
| stayed up | 51.2, 23.6, 17.1 s |
| wobble | 6.9 Hz, pitch-rate sd 95 deg/s, pitch sd 2.5° |
| wheel at \|PWM\| ≥ 2700 | 41 % of ticks |
| left-right (turn) PWM sd | 977 |

Mode 1 (stock PID) balances smoothly on the same table, so the car and the
surface are fine; the wobble is the policy against the real actuator.

## 4. Delay, and run 9

Run 8 in simulation, standing still: pitch-rate sd 1.2 deg/s at 5–15 ms of
delay; at 20–40 ms it oscillates (51.8 deg/s and **6.9 Hz at 30 ms**, the
same 7 % sign-flip rate as the car). A smaller motor dead zone gives a tiny
20–30 Hz buzz instead, which does not match. The firmware accounts for ~5 ms
and the MPU6050 DLPF ~3 ms (98 Hz: `mpu_set_sample_rate(200)` sets it to half
the rate), so ~20 ms was unaccounted for.

Run 9: `ACTION_LATENCY_TICKS = (1, 8)` (5–40 ms), best checkpoint picked at
30 ms, 30M steps from scratch, 3.86 h on a Ryzen 5 7530U (11 envs). Best eval
3,503.5 at 27.8M steps, full-length episodes. 39 % of the time went to
evaluations (every 110k steps, 20 episodes, ~25 s each). Training log:
`session-logs/train_real_robot_RUN9_delay_progress.csv`.

In simulation run 9 is calm at every delay from 5 to 40 ms (pitch-rate sd
~2 deg/s), tracks 91–94 % forward and 100 % turning at 30 ms, and all eight
training-like cars held 10 s. Float network test: worst error 1.40e-2 (PASS,
tolerance 5e-2). The team's int16 `policy_q.c` with the run-9 header, built
as a DLL, balanced the simulated car at 5, 30 and 40 ms and tracked commands,
within 0.02 of PyTorch.

On the car (20:18–20:19, three runs):

| | run 8 | run 9 |
|---|---|---|
| stayed up | 51, 24, 17 s | 18, 32, 21 s |
| wobble | 6.9 Hz | 7.2 Hz |
| pitch-rate sd | 95 deg/s | 68 deg/s (63–72 in the two clean runs) |
| wheel at \|PWM\| ≥ 2700 | 41 % | 6 % (0 % in the clean runs) |
| mean \|PWM\| | 2,170 | 1,886 |
| turn PWM sd | 977 | 512 |

Better, not fixed.

## 5. Gear slack

The JGB37-520's encoder is on the motor shaft, before the 1:30 gearbox. With
free play in the gearbox the motor crosses the gap before the wheel is
pushed, which acts like extra delay at every reversal (the car reverses ~14
times a second) and ends with a jolt. Estimate: at ~2° of slack and the
model's rotor inertia (2.25e-3 kg m² at the wheel) and ~0.3 N m, crossing
the gap takes ~20 ms.

Modelled in `real_robot.xml`: the rotor is its own hinge carrying the
reflected inertia, the motor drives the rotor (its reaction always lands on
the chassis), and a fixed tendon `rotor − wheel` limited to ±slack/2 couples
it to the wheel, solved by MuJoCo's constraint solver (a hand-written
explicit contact went unstable against the wheel's own 1.96e-5 inertia). The
encoders read the rotor. With slack 0 it reproduces the previous model
(run 8: 1.3 vs 1.2 deg/s at 5 ms; 46.9 vs 51.8 deg/s, 6.9 Hz at 30 ms).

`sim_tools/fit_delay_slack.py` simulates run 8 and run 9 over delay × slack
and matches both to the car (pitch-rate sd, Hz, % at full PWM, mean |PWM|):

| delay, slack | run 8 sim (car 95, 6.9 Hz, 41 %) | run 9 sim (car 68, 7.2 Hz, 6 %) |
|---|---|---|
| 30 ms, 0° | 47, 6.9 Hz, 1 % | 2 (calm) |
| 5 ms, 4° | 54, 8.4 Hz, 30 % | 21, 5.4 Hz, 0 % |
| **10 ms, 5°** | **65, 7.0 Hz, 27 %** | **27, 6.0 Hz, 0 %** |
| 20 ms, 6° | 74, 5.2 Hz, 40 % | 36, 5.0 Hz, 0 % |

Delay alone leaves run 9 calm at every delay, so slack is needed. Best fits:
slack 4–6° at the wheel with 10–20 ms. The fit pins the slack better than
the delay, and it under-predicts run 9's wobble by about half, so something
is still missing (more slack, uneven slack between the wheels, or the
driver's behaviour at reversal).

## 6. Run 10 (code ready, not trained)

- `real_robot.xml`: rotor joints `jr_l`/`jr_r` after the wheels, tendons
  `bl_l`/`bl_r` (default range ~0 = no slack), motors on the rotors.
- `train_real_robot.py`:
  - slack drawn per wheel per episode, `GEAR_SLACK_DEG = (0, 8)`; delay
    `ACTION_LATENCY_TICKS = (1, 6)` (5–30 ms);
  - eval at 5° and 15 ms;
  - the raw observation keeps its 17-value layout, with the encoder slots now
    the rotor's;
  - back-EMF from the rotor speed;
  - continues from `models/best_real_RUN9/best_model.zip`, 10M steps, LR 1e-4
    decaying, full command range from the start (payload ramps), evaluation
    every 250k steps;
  - `--init none` trains from scratch, `--steps`, `--run`, `--best-dir`,
    `--envs`.
- `enjoy_drive.py`: two direct `qpos`/`qvel` reads replaced.
- Checked: `firmware/check_obs_builder.py` PASS (worst 7.2e-7, encoders on the
  rotor); a 30k-step warm-start run trains and leaves `models/best_real/`
  alone; the repo tools reproduce the scratch results above.

How to train it on another PC and get it onto the car: `TRAINING_ON_DESKTOP.md`.

## Still open

- **Measure the slack by hand.** Car off, body held, tape mark on the tyre
  edge, turn the wheel each way until the gears engage. If the fit is right:
  2.5–3.5 mm at the edge of the 67 mm wheel (1 mm ≈ 1.7°).
- Mode 1 is not logged, so it cannot be used as a third fit point.
- The tilt filter's accelerometer correction is not simulated (training uses
  the true tilt plus noise); probably minor at 7 Hz, where the filter
  follows the gyro.

## Files

- `firmware/assemble_mode28_project.py` — new
- `real_robot.xml`, `train_real_robot.py`, `enjoy_drive.py` — run 10
- `sim_tools/` — new: `simlib.py`, `compare_models.py`, `fit_delay_slack.py`
- `models/best_real/` = run 9; `models/best_real_RUN8/`, `models/best_real_RUN9/`
- `firmware/policy_weights.h`, `policy_weights_q.h`, `policy_testvectors.h` = run 9
- `car_logs/2026-10-07/` — logger output for runs 8 and 9 on the car
- `session-logs/train_real_robot_RUN9_delay_progress.csv`
- `TRAINING_ON_DESKTOP.md` — new

## 7. Run 10 trained on the desktop (afternoon)

Followed `TRAINING_ON_DESKTOP.md` steps 2–3 on the desktop (20 threads, 19
envs). Before starting, run 8's final model was saved as
`models/ppo_real_robot_RUN8_30M.zip`, because training overwrites
`models/ppo_real_robot.zip`.

**Training.** 10M steps from run 9 took 2,436 s (41 min, about 4,100 steps/s
including evaluations), with no errors. Eval at 5° slack and 15 ms:
- 3350 at 250k steps, 3478 at 1.5M, and a plateau of 3426–3478 to 3.25M;
- then a rise to the **best, 3575 at 6.0M** (saved as `models/best_real/`);
- then a drift down to 3458 at 10M.

Every eval episode ran its full 2000 ticks. The best checkpoint is backed up
to `models/best_real_RUN10/`, and the final model to
`models/ppo_real_robot_RUN10_10M.zip`. Training curve:
`session-logs/train_real_robot_RUN10_slack_progress.csv`.

**`sim_tools/compare_models.py`: run 10 fails the doc's main criterion.**

Standing still, pitch-rate sd in deg/s and the wobble frequency:

| 5° slack | run 8 | run 9 | run 10 |
|---|---|---|---|
| 5 ms | 63.5, 7.7 Hz | 26.8, 5.2 Hz | 23.6, 7.5 Hz |
| 10 ms | 64.2, 7.0 Hz | 27.7, 6.1 Hz | **19.6**, 7.8 Hz |
| 15 ms | 63.3, 6.1 Hz | 31.0, 5.1 Hz | 29.9, 7.1 Hz |
| 20 ms | 67.3, 5.6 Hz | 34.3, 5.1 Hz | **24.5**, 6.1 Hz |
| 30 ms | 67.7, 4.4 Hz | 34.0, 4.9 Hz | 46.5, 3.4 Hz (worse) |

At 0° slack all three are calm, at 1–2 deg/s up to 15 ms, and runs 9 and 10
stay calm at 30 ms.

Against the three criteria:
- **Wobble:** the bar was "a few deg/s where run 8 and run 9 show tens".
  Run 10 still shows tens, so it **fails**. It is 10–30 % calmer than run 9
  at 5–20 ms, the fitted range, and worse at 30 ms.
- **Training-like cars:** all 8 held 10 s, at rate sd 7.5–33.5 deg/s, so it
  **passes**.
- **Tracking at 5°/15 ms:** forward 93 % / 91 %, turning 100 %, so it
  **passes**.

Not exported and not pushed: `firmware/policy_weights*.h` are still run 9.

**Why fine-tuning does not remove the wobble.** Probes on the 3.75M
checkpoint, with scripts in the session scratchpad:

- **A calmer controller exists.** A plain controller on pitch, pitch rate,
  speed and the position integral, acting through the same observation and
  motor path, was searched over 150 gain sets at 5° and 15 ms. 33 sets held
  10 s, and the calmest reached **8.2 deg/s** (gains 2, 0.4, 1.5, 0) and
  9.2 deg/s (gains 4, 0.2, 1.5, 2), as slow 2–3 Hz sways. So 5° of slack
  does not force a 25–30 deg/s wobble, though "a few deg/s" may be too strict
  at 5°.
- **The reward prefers the wobble.** Standing still at 5°/15 ms, per step:
  - run 10: **1.82**;
  - calm PD (4, 0.2, 1.5, 2): 1.645;
  - calm PD (2, 0.4, 1.5): 1.47;
  - run 9: 1.20.
- **Breakdown, run 10 vs PD (4, 0.2, 1.5, 2):**
  - speed −0.053 vs −0.154;
  - position −0.024 vs −0.081;
  - pitch −0.050 vs −0.110 (mean |pitch| 0.96° vs 2.09°);
  - action rate −0.002 vs ~0.

  The fast wobble keeps the *average* pitch, speed and position closer to
  zero than a slow sway does. No term penalises pitch rate, and the
  action-rate term barely sees the wobble.
- **Implication:** more fine-tuning on this reward will not remove the
  wobble. A run 11 needs a term that prices it, for example −w·(pitch
  rate)². By the numbers above, w of about 1–2 (rad/s)⁻² would make the calm
  controller score higher. Check the weight with the same probe before
  training, and make sure driving, which needs brief pitch rates to lean,
  still pays.
- **Caveat:** the slack fit under-predicts run 9's car wobble by about half
  (section 5), so these simulated numbers are a proxy for the car.

Desktop working tree after this section:
- `models/best_real/best_model.zip` = run 10, modified, not committed;
- `firmware/policy_weights*.h` = run 9 (unchanged).

## 8. Run 11: wobble penalty (desktop), pushed for the car

**Reward change** (`train_real_robot.py`):
`reward -= min(PITCH_RATE_WEIGHT · q², PITCH_RATE_PENALTY_MAX)`, where `q` is
the true chassis pitch rate (`data.qvel[4]`, body frame), `PITCH_RATE_WEIGHT`
is 1.5 per (rad/s)² and `PITCH_RATE_PENALTY_MAX` is 1.0 per step.

The weight was picked with a probe before training. Per step, after the first
2 s, at 5° slack and 15 ms:

| controller | rms pitch rate | w = 0 | w = 1 | w = 1.5 |
|---|---|---|---|---|
| plain PD (4, 0.2, 1.5, 2) | 9 deg/s | 1.64 | 1.61 | 1.60 |
| run 10, standing | 30 deg/s | 1.77 | 1.50 | 1.36 |
| run 10, driving at 0.25 m/s | 28 deg/s | 1.72 | 1.48 | 1.37 |

- Without slack, the policies are calm (1.7 deg/s) and pay under 0.002 per
  step.
- Refusing to drive would cost more than 1.7 per step, so driving still pays.
- **The cap:** without it, a 64 deg/s wobble would cost 1.9 per step against
  the +2 alive bonus, so falling could pay. A first cap of 0.75 clipped run
  10's own wobble peaks and cut its penalty from 0.42 to 0.30 per step. At
  1.0 the calm PD still wins by 0.16 per step: 1.60 against 1.44.
- **Across training-like cars,** run 10's wobble costs 0.19 per step on
  average (0.03–0.43): a steady signal.

**Training.** 10M steps from run 10 took 3,214 s (54 min; probes ran
alongside), with no errors.

On the run-11 reward, the same 20-episode eval scores:
- run 10: 3451 ± 336;
- run 11: best **3440 at 1.25M**, then a slide to 3200–3310, ending at 3236.

The training reward stayed flat at 2800–2950, with healthy PPO statistics
(KL about 0.004, explained variance about 0.9), while payload ramped in.

**`compare_models.py` at 5° slack, standing still** (pitch-rate sd, deg/s):

| delay | run 9 | run 10 | run 11 best (1.25M) | run 11 final (10M) |
|---|---|---|---|---|
| 5 ms | 26.8 | 23.6 | 20.0 | 22.0 |
| 10 ms | 27.7 | 19.6 | 15.4 | 17.0 |
| 15 ms | 31.0 | 29.9 | 24.1 | 19.3 |
| 20 ms | 34.3 | 24.5 | 22.9 | 23.5 |
| 30 ms | 34.0 | 46.5 | 25.4 | 24.8 |

Other checks:

| | run 10 | run 11 best | run 11 final |
|---|---|---|---|
| driving at 5° / 15 ms, rate sd | 28.5 | 15–18 | 20 |
| tracking, forward / turning | 91–93 % / 100 % | 88–92 % / 100 % | **75–78 %** / 100 % |
| 8 training-like cars | all held, 7.5–33.5 deg/s | all held, **5.5–16.8** | all held, 7.7–23.1 |
| no slack | 1.8 deg/s | 2.5–2.8 deg/s | 1.8–2.2 deg/s |

**Verdict:**
- **Run 11 best is the calmest policy so far:**
  - about 20 % less wobble standing at 5° than run 10;
  - 35–45 % less while driving;
  - the 30 ms case is halved;
  - about half the wobble on the training-like cars;
  - tracking still above 85 %.
- **It still misses "a few deg/s".** That bar now looks unrealistic: the
  best plain PD reaches about 8 deg/s here, and `TRAINING_ON_DESKTOP.md` §3
  is updated.
- **Fine-tuning ran out of room.** The best checkpoint came at 1.25M, after
  which the policy traded tracking for calmness.
- **Next, if run 11 still wobbles on the car:** run 12 from scratch with this
  reward (`--init none`, 30M steps, about 2.5 h on the desktop).

**Pushed for the laptop.**

The exported files are from run 11 best:
- `firmware/policy_weights.h` and `policy_testvectors.h`;
- `firmware/policy_weights_q.h` (worst int16 rounding 1.98e-5).

`test_policy` PASS:
- fast tanh: worst 1.14e-2 against a tolerance of 5e-2;
- exact tanh: 1.2e-7.

The int16 `policy_q.c` was not re-run in closed loop this time.

Models:
- `models/best_real/` = run 11 best;
- `models/best_real_RUN10/` and `models/best_real_RUN11/` added.

Also pushed:
- training curves `train_real_robot_RUN10_slack_progress.csv` and
  `train_real_robot_RUN11_wobble_progress.csv`;
- `sim_tools/compare_models.py`, which now defaults to runs 9, 10 and 11;
- `TRAINING_ON_DESKTOP.md`, updated for run 11 (the car folder is
  `car_firmware_run11`).

Kept on the desktop, ignored by git:
- `models/ppo_real_robot_RUN8_30M.zip`, `ppo_real_robot_RUN10_10M.zip`,
  `ppo_real_robot_RUN11_10M.zip`;
- `logs/run10/`, `logs/run11/`.
