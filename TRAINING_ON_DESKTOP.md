# Train on the desktop, test on the laptop

Training needs CPU and nothing else; the car needs Keil, FlyMCU and the USB
cable. So the desktop trains and the laptop builds, flashes and tests. GitHub
(this repo, branch `tzejun-branch`) carries files between them:

| | desktop | laptop |
|---|---|---|
| does | train, check in simulation, export the weights | build the hex, flash, test, log |
| pushes | the checkpoint and the two weight headers | the car logs |

**Rule:** `git pull` before you start on either machine, commit and push when
you finish. Only the desktop changes training code and models; only the
laptop adds car logs. Then the two never conflict.

Background for the current run (run 13: the measured motor with stiction,
pushes in training, and a mild wobble penalty; run 12 before it):
`session-logs/2026-10-08-run12-measured-motor-and-pushes.md`. The motor
measurements behind it are in the laptop's 2026-10-08 logs (motor step test,
stock PID, reversal test, low-PWM stiction). Turning speed, calibration and
the two firmware edits run 13 needs:
`session-logs/2026-10-10-turning-calibration-mujoco.md`.

## 1. Desktop, once

1. Install **Git** (git-scm.com) and **Python 3.12** (python.org; tick *Add
   python.exe to PATH*).
2. In a terminal:
   ```
   git clone https://github.com/aaaakkkk111/DIP-E026.git
   cd DIP-E026
   git checkout tzejun-branch
   python -m venv venv
   venv\Scripts\activate
   pip install -r requirements.txt tqdm rich
   ```
3. Optional, to drive the desktop from the laptop: install VS Code on the
   desktop, then *Accounts → Turn on Remote Tunnel Access* and sign in with
   GitHub. On the laptop, in VS Code: *Remote Explorer → Tunnels* → the
   desktop. A terminal opened there runs on the desktop.

## 2. Desktop: train

```
cd DIP-E026
git pull
venv\Scripts\activate
python train_real_robot.py
```

`train_real_robot.py` is currently set up as **run 13** (already trained,
2026-10-09). It continues from `models/ppo_real_robot_RUN12_30M.zip` (run
12's final checkpoint) for 10M steps, with:
- the measured motor and stiction;
- gear slack 0–4° and delay 5–15 ms;
- pushes;
- the wobble penalty at 0.5.

Run it again only after changing `RUN_NAME` and `INIT_FROM` for the next
run.

On the desktop (20 threads, 19 envs), 10M steps take about 40 min and 30M
from scratch about 1.6 h. On the laptop's 12 threads, expect about 1.5–2×
that. Keep the desktop awake (*Settings → System → Power → Sleep: Never*
while plugged in). The script opts itself and its workers out of Windows
efficiency mode, which had cut run 12's speed to a quarter.

- The best checkpoint goes to `models/best_real/best_model.zip` (overwritten as
  it improves), the final one to `models/ppo_real_robot.zip`. The best is
  chosen by the push-free eval, so for a push-trained run also check the
  final one (§3).
- Progress: `logs/<run>/progress.csv`. The column `eval/mean_reward` is the
  score at 2° slack and 10 ms, with no pushes. Scores are only comparable
  within one reward and one simulator: runs 11–13 changed both.
- Options: `--steps N`, `--init none` (from scratch, 30M steps), `--run NAME`
  (log folder), `--envs N`.

## 3. Desktop: check it in simulation

```
python sim_tools/compare_models.py --slack 2 3 --delay 1 2 3
```

It compares runs 11, 12 and `models/best_real` (now run 13) on the simulated
car; `--model NAME=PATH` picks others. Use the car's measured 2–3° of slack and
5–15 ms of delay. The defaults (0 and 5°) are from before the motor was
measured.

**The old car criteria were wrong.** Wobble numbers taken from the E lines
(the flight recorder's last 0.6 s) measured the final push, not the
balancing: runs 8, 9 and 11 "95, 68, 57.5 deg/s". Run 11's real mid-run
wobble, from the 200 Hz stream, is 12–22 deg/s, and the simulator with the
measured motor reproduces it (13.5–21). So judge a car run from the stream,
mid-run (`sim_tools/stream_log.py`), and compare it with `simlib.run()`.

A new run is worth flashing if, on the 2° car:
- it is no wobblier standing still than the last one;
- all eight training-like cars hold 10 s;
- tracking stays above about 85 %;
- it survives at least as many pushes (the push check is in the run-12
  session log, §6).

Reference, simulated at 2° slack:

| | standing wobble | survives 3 N × 0.1 s (of 18) |
|---|---|---|
| stock PID | 6.6–6.9 deg/s | 17 |
| run 11 | 11–16 | 0 |
| run 12 | 26–35 | 14 |
| run 13 | 24–33 | 18 |

## 4. Desktop: export and push

For the next run (14 here; use its number):

```
python export_stm32.py
python firmware/quantize_weights.py
mkdir models\best_real_RUN14
copy models\best_real\best_model.zip models\best_real_RUN14\
git add -f models/best_real/best_model.zip models/best_real_RUN14/best_model.zip
git add firmware/policy_weights.h firmware/policy_weights_q.h firmware/policy_testvectors.h
git commit -m "Run 14: <what changed>"
git push
```

If you chose the final checkpoint over the best one (as for runs 12 and 13),
copy `models\ppo_real_robot.zip` to `models\best_real\best_model.zip` before
exporting.

`models/` is git-ignored, hence `git add -f`. If gcc is installed, the PC check
of the exported network is
`cd firmware` → `gcc -O2 -o test_policy test_policy.c policy.c -lm` →
`test_policy` (expect `PASS`).

## 5. Laptop: build, flash, test

1. `git pull` in the repo.
2. Make a project folder for the new weights (once per run), leaving the
   flashed ones intact. Copy from the newest build: `car_firmware_reversal`
   (it has the 200 Hz stream that `stream_log.py` records), or your run 12
   folder if you made one from it:
   ```
   robocopy C:\Users\USER\Desktop\car_firmware_reversal C:\Users\USER\Desktop\car_firmware_run13 /E /XD OBJ Objects Listings
   ```
3. Copy `firmware\policy_weights.h` and `firmware\policy_weights_q.h` into
   `car_firmware_run13\stm32_Balance_Car_L\APP\RL\`, replacing the old ones.
4. **Run 13 only, once:** two edits in that folder's `APP\RL\rl_mode.c`. A
   folder copied from `car_firmware_run13` already has them.
   ```c
   #define CMD_W           1.00f        /* was 0.50; runs 12-13 track up to 1.5 rad/s in sim */
   ```
   and, in the block that finishes `cal`, one line after
   `att_from_acc(ab, &roll_zero, &pitch_zero);`:
   ```c
   pitch_zero = roll_zero = 0.0f;   /* fixed zero: cal now only measures gyro bias, in any still pose */
   ```
5. Keil: *Project → Open Project* →
   `car_firmware_run13\stm32_Balance_Car_L\USER\stm32_Balance_Car.uvprojx`,
   then **F7**. Expect `0 Error(s)`. The hex is
   `...\OBJ\stm32_Balance_Car_L.hex`.
6. FlyMCU: that hex, COM3, *Auto Reload Before Program* ticked, *Program
   OptionBytes when ISP* unticked, *Reset@DTR Low, ISP@RTS High*, **Start
   ISP**, then close FlyMCU.
7. Logger: `python sim_tools/stream_log.py --name run13_1` (one name per
   session). It writes `car_logs\<date>\run13_1.txt` itself and needs COM3 to
   itself, so close FlyMCU and `rl_trim_helper.py`.
8. Car: switch on, select mode 28 and leave it **still in any position** for
   about 5 s (`cal` measures the gyro bias; with the step 4 edit the pitch
   zero is fixed at 0, so no balance point and no `trim`). KEY1, stand it up;
   it starts after 1 s upright.
9. Let it stand ~20 s with no input, turn both ways with the app, drive, and
   give a few gentle pushes. End each run by **pushing it over** past 40° (not
   catch, lift or switch off). Three runs.
10. Stop the logger (Ctrl+C), then commit and push `car_logs\<date>\`. Judge
    the wobble from the middle of a run, not the start or the final push. In
    simulation run 13 sways at 24–33 deg/s standing at 2° slack; run 11's car
    runs were 12–22.
11. If it creeps forward standing still, send `trim -1` (each degree is about
    1 cm/s of creep; `trim 1` if it creeps back). `trim` needs
    `rl_trim_helper.py` (`..\venv\Scripts\python.exe rl_trim_helper.py COM3`
    in `rl_mode28_20261003`), so stop `stream_log.py` first.

To go back to an earlier policy, flash the hex in `car_firmware_run12` (if
you kept it), `car_firmware_run11` or `car_firmware_run9`.

## Using Claude Code on the desktop

A new session does not know this history. Start it in the repo and ask it to
read this file and the latest file in `session-logs/` first.
