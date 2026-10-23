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

## Now: flash run 23 (2026-10-23)

`models/best_real` and the weight headers are **run 23 at 25.5M steps** (its best).
All its results are from the simulator. Full log: `session-logs/2026-10-23-run23-desktop.md`.

**Firmware:** `policy.h` now has `POLICY_POS_DECAY 1.0f` (no leak). That is the only
change from run 22: 0 B. Rebuild with `policy.h`, `policy.c` and the two weight headers.
Run `check_obs_builder` (rebuild the DLL first). It could not run on the desktop: Windows
Smart App Control blocked the new DLL.

**Simulator results:**

| | run 23, 25.5M | run 22, 24M (car now) | PID |
|---|---|---|---|
| Standing empty, 2° / 4° slack (deg/s) | 3.7–4.2 / **5.6–9.9** | 4.0–5.4 / 7.1–23.5 | 6.6–6.9 |
| Creep after 30 s (slopes, ±2° trim error, 0 and 2 kg) | **≤ 0.3 cm/s, stops** | 0.3–0.7, keeps going | — |
| Offset after 60 s | ≤ 17 cm | 19–41 cm | — |
| Empty 2 N pushes / 2 kg 4 N pushes | 13/18 / 18/18 | 17/18 / 18/18 | 18/18 |

**What to look for on the car:**
- The trim should matter much less now. Try the built-in 1.7° empty *and* with 2 kg: it
  should stop creeping within about 30 s in both cases.
- If it is pushed or carried far by hand while running, it drives back up to 0.5 m to its
  original spot (like the stock PID). A new drive command clears that memory.
- Empty pushes are slightly weaker than run 22's in simulation, so push gently.
- Log long stands (1–2 min) with the P stream off (`p`), so position is accurate.

## Before: run 23 spec, full position memory vs run 22 (2026-10-22)

**Goal:** remove the creep and the load-dependent trim (1.7° empty, about 2.5° with 2 kg) the
way the stock PID does. Its speed loop (`APP/PID/pid_control.c`, `Velocity_PI`) integrates the
encoders **without forgetting**, clamped at ±8000, and clears only when the motors cut out.
Any balance-point offset builds up in that integral until it is cancelled: an automatic trim.

**Run 23 = run 22 with:**
1. **A full position memory:** `POSITION_TAU_S = inf` (decay 1.0). Keep the ±0.5 m clip
   (`POS_ERR_CLIP_M`) and keep the reset on command change.
2. **Each episode starts with a random position integral, uniform over ±clip.** Episodes are
   10 s, but car runs last minutes. A full integral can reach values a 10 s episode never
   shows, and the policy would see inputs it was never trained on (as run 7's wheel angle
   did). The random start covers the whole range.
3. Keep the random pitch offset (±4°, `PITCH_OFFSET_RAD`). With a full memory, the policy
   can learn to cancel it, which is the self-trim.

**Compare against run 22 (24M), same seeds, in simulation:**

| check | run 23 must |
|---|---|
| creep, standing, ±2° pitch offset and ±2° slope, 0 and 2 kg | be smaller than run 22's (about 1 cm/s): near 0 after it settles |
| 60 s standing, not 10 s (the integral's long-run behaviour) | not grow a slow sway: tilt sd ≤ run 22's, position within about ±10 cm |
| wobble, 2° and 4° slack | be no worse than run 22 (4.0–5.4 / 7.1–23.5) |
| pushes: 2 N empty, 4 N with 2 kg; return to the spot | be no worse (17/18, 18/18) |
| starting with the integral at ±clip | recover without falling or running away |

The risk is integral windup: a bigger, slower sway, and an overshoot after pushes. If it
fails, try a long leak (τ 30–60 s) between the two.

**Hardware limit (keep the policy within it):**
- The firmware image must end at or below **0x08010000 (64 KB)**; past that the board loads
  garbage. Run 22's image has **40 B** spare.
- Run 23 must **not** change the network (34→64→64→2 int16) or add inputs or filters.
- Its firmware change is constants only, 0 B: `POLICY_POS_DECAY 1.0f` in `policy.h`. Then
  `g_pos_err = 1.0f * g_pos_err + …`, and the clip and reset rule stay as they are.
- Anything that adds code must remove the same, or say so for the laptop to trim.

## Now: run 22 is on the car (laptop, 2026-10-22)

It is the best on the car so far.
- **Empty, trim 1.7°:** 4.0–4.3 deg/s standing, as simulated, and it hovers around its spot
  (creep +0.1 to +0.5 cm/s).
- **With 2 kg:** 2.7 deg/s, calmer than simulated, with a best trim of about 2.5°.
- It survived every hand push, empty and loaded.

The trim is built into the firmware (1.7°, `rl_mode.c`, Keil folder `car_firmware_run22`). It
depends on the load.

Open, in order:
1. Empty 3 N pushes (0/18 in simulation).
2. Load-dependent trim.
3. Slope climbing with a load.

Details: `session-logs/2026-10-22-run22-on-car.md`. Flash space: **40 B** left.

## Before: flash run 22 (2026-10-22)

`models/best_real` and the weight headers are **run 22 at 24M steps**. All its
results are from the simulator. Full log: `session-logs/2026-10-22-run22-desktop.md`.

**Rebuild with the new `firmware/policy.c` *and* `policy.h`.** Run 22's position
memory is 8 s (`POLICY_POS_DECAY`), and it restarts when the forward command
changes (`g_last_cmd_f` in `policy.c`). Run 21's headers with run 22's weights,
or the reverse, don't match.

**Flash:** this adds a float compare and a store, and run 21 left 48 B free.
Check the build size. If it no longer fits, trim a console message as before.

**Simulator results (2° slack):**

| | run 22, 24M | run 21 (car now) | PID |
|---|---|---|---|
| Standing empty (deg/s) | 4.0–5.4 | 6.6–8.3 | 6.6–6.9 |
| Empty pushes, 2 N / 3 N | 17/18 / 0/18 | 1/18 / 0/18 | 18 / 17 |
| Creep on a ±2° floor (cm/s) | about 1 or less | 2.5–4.8 | — |
| After a 2 N push | back within about 5 cm | falls | — |
| 2 kg: standing, pushes at 4 N | 5.0–6.7 deg/s, 18/18 | 18/18 | — |
| Drives with 4 kg | yes | no | — |

At 4° slack (a worse-geared car) it can be shakier than run 21 (up to 23.5 against
13). Empty 3 N pushes still fail, so push gently.

**Steps:**
1. Build with the four files. Keep the trim at about 2.5° if it's not built in yet.
2. Test standing empty and with 2 kg, with gentle pushes, then creep over a minute.
3. Log the 200 Hz stream and push the logs.

## Before: train run 22 (2026-10-22)

**Run 21 on the car (laptop, 2026-10-21):**
- It **does not shake**: 2.8–4.3 deg/s standing, against the stock PID's 7.6–11.7 and run 15's
  44–49. It reverses the motors only 0.5–1 times a second.
- It creeps 1–2 cm/s, and the creep follows the **room** (a floor slope or the cable), not the
  car. The car's own trim is about 2.5°.
- After pushes it wanders and never returns to its spot.
- Push recovery when empty, and the 2 kg load, are not tested on the car yet.

Full write-up: `session-logs/2026-10-21-run21-on-car-trim-slope.md`.

**Run 22 = run 21 plus** (the session log, §4, has the details and the pass criteria):
1. Longer position memory while standing still: τ 5–10 s, reset on a new command. Change
   `POLICY_POS_DECAY` and `policy.c` to match.
2. Random floor slope of ±2° in training. The pitch input must stay relative to gravity, as the
   car's IMU is.
3. Pushes scaled to the car's mass, for empty-car push recovery.

**Firmware space:** the run 21 image has **48 B** left under the board's 64 KB limit. A
`policy.c` change that adds code must remove some too, or say so for the laptop to trim.

## Before: flash run 21 (2026-10-21)

`models/best_real` and the weight headers are **run 21 at 21M steps**. All its
results are from the simulator; this is its first time on the car.

**What changed:** the network now reads `osc` and `lean` (a teammate's Mode 26
`load_adapt.c` filters) in `obs[5]` and `obs[6]`, which used to be 0.
`firmware/policy.c` computes them inside, so `rl_mode.c` needs no edit. Rebuild
with the new **`policy.c`**, not just the weight headers. With an old `policy.c`
the network sees zeros there, and in simulation it then shakes like run 15
(up to 31 deg/s).

**Simulator results (2° slack):**

| | standing wobble (deg/s) | pushes survived |
|---|---|---|
| Empty | 6.6–8.3 (run 15: 18–25; PID 6.6–6.9) | **2 N: 1/18** (PID 18/18) |
| With 2 kg | 4.6–5.2 | 3 N and 4 N: 18/18 |

It drives at 0–2 kg at full speed. **Empty, it barely survives a push, so keep a
hand ready.** Details: `session-logs/2026-10-21-run21-desktop.md`.

**Steps:**
1. `git pull`, then rebuild the Keil project with `firmware/policy.c`,
   `policy.h`, `policy_weights.h` and `policy_weights_q.h`
   (`assemble_mode28_project.py` copies them).
2. Watch the flash use. The filters add a little code and 24 B of RAM, and
   `sqrtf` is already linked. If it no longer fits, say so and I'll trim.
3. Flash, then `cal` as usual.
4. Test standing empty, then with about 2 kg.
5. Log the 200 Hz stream mid-run, as for run 15, and push the logs. Gentle
   pushes only when empty.

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

`train_real_robot.py` is currently set up as **run 16** (not yet trained): run
15 plus a penalty per motor reversal (`REVERSAL_WEIGHT` 0.5) and gear slack
0–6°, from scratch, 30M steps. Why: on the car (2026-10-20) runs 13 and 15
shake at about 7 Hz through the gear slack, reversing 10–17 times a second;
the PID rarely reverses
(`session-logs/2026-10-20-car-shake-and-run16.md`). Check it with
`compare_models.py --slack 2 4` and the reversal counts in that log, §3.

Run 15 (trained 2026-10-18; `models/best_real` is its final checkpoint):
from scratch, 30M steps, with:
- the measured motor and stiction;
- gear slack 0–4° and delay 5–15 ms;
- pushes up to **3 N** (runs 12–14: 4 N, more than even the stock PID
  survives);
- the wobble penalty at 1.5, **paused for 1 s after each push**
  (`PENALTY_PAUSE_S`), so recovering hard isn't charged.

Why: on the car (2026-10-14) run 13 stood at 26–36 deg/s mid-run, as the
simulator predicted, while the stock PID stands at 7.6 deg/s on the same car
and recovers from hard hand pushes. Run 14 (penalty 1.5, 4 N pushes) was
calmer but lost push tolerance. Run 15 keeps run 13's push tolerance and is
about 20 % calmer, still short of the PID. Background:
`session-logs/2026-10-14-run13-on-car-and-run14.md`,
`2026-10-14-run14-desktop.md`, `2026-10-18-run15-desktop.md`.

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

It compares runs 11, 13 and `models/best_real` (the new run once trained;
until then it is run 13) on the simulated car; `--model NAME=PATH` picks others. Use the car's measured 2–3° of slack and
5–15 ms of delay. The defaults (0 and 5°) are from before the motor was
measured.

**The old car criteria were wrong.** Wobble numbers taken from the E lines
(the flight recorder's last 0.6 s) measured the final push, not the
balancing: runs 8, 9 and 11 "95, 68, 57.5 deg/s". Run 11's real mid-run
wobble, from the 200 Hz stream, is 12–22 deg/s, and the simulator with the
measured motor reproduces it (13.5–21). So judge a car run from the stream,
mid-run (`sim_tools/stream_log.py`), and compare it with `simlib.run()`.

**The calm target** (the stock PID's level; runs 14 and 15 missed it): standing wobble at most about
10 deg/s at 2° slack, 5–15 ms, and surviving 3 N × 0.1 s in at least 15 of
18. If it is calm but no longer push-proof, the penalty is too strong for the
pushes (try 1.0); if it is push-proof but not calm, too weak.

A new run is worth flashing if, on the 2° car:
- it is no wobblier standing still than the last one;
- all eight training-like cars hold 10 s;
- tracking stays above about 85 %;
- it survives at least as many pushes (the push check is in the run-12
  session log, §6).

Reference, simulated at 2° slack:

| | standing wobble | on the car (stream, mid-run) | survives 3 N × 0.1 s (of 18) |
|---|---|---|---|
| stock PID | 6.6–6.9 deg/s | 7.6 | 17 |
| run 11 | 11–16 | 12–22 | 0 |
| run 12 | 26–35 | | 14 |
| run 13 | 24–33 | 26–36 | 18 |
| run 14 final | 17–28 | | 5 |
| run 15 final | 20–25 | 43–49 (2026-10-20, a worse car: PID 11.7) | 18 |

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
   flashed ones intact. **For run 15:** copy `car_firmware_run13`, which
   already has the stream and the step 4 edits, so only the headers change
   (use `run15` for `run13` in steps 3, 5 and 7):
   ```
   robocopy C:\Users\USER\Desktop\car_firmware_run13 C:\Users\USER\Desktop\car_firmware_run15 /E /XD OBJ Objects Listings
   ```
   Run 13 itself was copied from `car_firmware_reversal` (the 200 Hz stream
   that `stream_log.py` records):
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
