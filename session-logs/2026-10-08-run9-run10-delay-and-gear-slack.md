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
