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

Background for the current run (run 10, gear slack):
`session-logs/2026-10-08-run9-run10-delay-and-gear-slack.md`.

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

That is run 10 as configured in `train_real_robot.py`: it continues from
`models/best_real_RUN9/best_model.zip` for 10M steps with gear slack 0–8° and
delay 5–30 ms. On the laptop's 12-thread CPU run 9 managed ~3,500 steps/s
when not evaluating, so expect about 1–1.5 h there; a desktop with more
threads is faster. Keep the desktop awake (*Settings → System → Power →
Sleep: Never* while plugged in).

- The best checkpoint goes to `models/best_real/best_model.zip` (overwritten as
  it improves), the final one to `models/ppo_real_robot.zip`.
- Progress: `logs/run10/progress.csv`; the column `eval/mean_reward` is the
  score at 5° slack and 15 ms.
- Options: `--steps N`, `--init none` (from scratch, 30M steps), `--run NAME`
  (log folder), `--envs N`.

## 3. Desktop: check it in simulation

```
python sim_tools/compare_models.py
```

It compares run 8, run 9 and the new `models/best_real` on the simulated car.
Run 10 is worth flashing if, in the **5° slack** rows, its pitch-rate sd is a
few deg/s where run 8 and run 9 show tens, all eight training-like cars hold
10 s, and tracking stays above ~85 %. (On the car, run 8 showed 95 deg/s and
run 9 68 deg/s.)

## 4. Desktop: export and push

```
python export_stm32.py
python firmware/quantize_weights.py
mkdir models\best_real_RUN10
copy models\best_real\best_model.zip models\best_real_RUN10\
git add -f models/best_real/best_model.zip models/best_real_RUN10/best_model.zip
git add firmware/policy_weights.h firmware/policy_weights_q.h firmware/policy_testvectors.h
git commit -m "Run 10: trained with gear slack"
git push
```

`models/` is git-ignored, hence `git add -f`. If gcc is installed, the PC check
of the exported network is
`cd firmware` → `gcc -O2 -o test_policy test_policy.c policy.c -lm` →
`test_policy` (expect `PASS`).

## 5. Laptop: build, flash, test

1. `git pull` in the repo.
2. Make a project folder for the new weights (once per run), leaving the
   flashed ones intact:
   ```
   robocopy C:\Users\USER\Desktop\car_firmware_run9 C:\Users\USER\Desktop\car_firmware_run10 /E /XD OBJ Objects Listings
   ```
3. Copy `firmware\policy_weights.h` and `firmware\policy_weights_q.h` into
   `car_firmware_run10\stm32_Balance_Car_L\APP\RL\`, replacing the old ones.
   Nothing else changes.
4. Keil: *Project → Open Project* →
   `car_firmware_run10\stm32_Balance_Car_L\USER\stm32_Balance_Car.uvprojx`,
   then **F7**. Expect `0 Error(s), 4 Warning(s)`. The hex is
   `...\OBJ\stm32_Balance_Car_L.hex`.
5. FlyMCU: that hex, COM3, *Auto Reload Before Program* ticked, *Program
   OptionBytes when ISP* unticked, *Reset@DTR Low, ISP@RTS High*, **Start
   ISP**, then close FlyMCU.
6. Logger, in `rl_mode28_20261003`:
   `..\venv\Scripts\python.exe rl_trim_helper.py COM3`. Then write `auto off`
   to `cmd.txt`, **every time it starts**. Watch it from a second terminal:
   `Get-Content rl_runs_summary.txt -Wait -Tail 5`.
7. Car: switch on, mode 28, KEY1, hold still at the balance point until `cal
   done`. If the pitch zero is not about 3.2°, `trim` it there (±5° per
   command, one at a time; e.g. `trim 3` from about 0).
8. KEY1, stand it up near `p 0.0`, let go when the OLED shows `L… R…`. Let it
   balance ~10 s, then **push it over** past 40° (not catch, lift or switch
   off, or the 200 Hz recording shows no balancing). Three runs.
9. Stop the logger (Ctrl+C), copy `rl_serial_log.txt` and
   `rl_runs_summary.txt` into `car_logs\<date>\`, then commit and push. Check
   the runs with
   `python firmware/check_car_log.py car_logs/<date>/rl_serial_log.txt`, and
   compare the wobble with run 8 and run 9 using
   `sim_tools/simlib.py`'s `car_stats()` (see `sim_tools/fit_delay_slack.py`).

To go back to an earlier policy, flash the hex in `car_firmware_run8` or
`car_firmware_run9`.

## Using Claude Code on the desktop

A new session does not know this history. Start it in the repo and ask it to
read this file and the latest file in `session-logs/` first.
