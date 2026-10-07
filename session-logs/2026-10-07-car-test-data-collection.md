# 2026-10-07 — Car-test data collection guide, run 8 pushed

Follows `2026-10-06-hardware-mode28-diagnosis.md` (diagnosis of run 7 on the
car, then run 8).

## What was done

1. **Pushed run 8 to `tzejun-branch`.** One commit with the dead-zone motor
   model, the firmware-exact sensing in `train_real_robot.py`, the run-8
   checkpoint in `models/best_real/`, the firmware changes, the new tools
   (`quantize_weights.py`, `check_obs_builder.py`), the run-8 weight headers
   and the 2026-10-06 session log and training log. The three
   `training_*.log` files in the repo root stay untracked, as asked.
   - Removed `firmware/test_obs_builder.exe`. It was built on 2026-09-29 from
     the old exe version of `test_obs_builder.c`, which now builds a DLL, so
     the binary no longer matched its source. The DLL is not committed; Step 2
     of the deployment README builds it.
   - Fixed two stale run-7 numbers in the deployment README: the
     `test_policy` expected output (now 2.198e-02 against 5.0e-02) and the
     fast-tanh error quoted in Step 3.
2. **Wrote how to collect readings from the car test** as Step 10 of
   `firmware/README-STM32-DEPLOYMENT.md`, linked from the top-level README.
   It covers the serial setup, `rl_trim_helper.py`, every line the mode-28
   firmware prints (`D`, `R`, `E`, `R end`, `T`), the test sequence, and a
   pass table for run 8. Same steps, condensed, in the team's
   `rl_mode28_20261003/run8_dropin/README.md` (outside the repo).

## Found while reading the logging code

- **`rl_trim_helper.py` starts with auto-trim on** (`auto = True` in
  `main()`), although the team's README says it is off. After any run of
  1 s or more that ends in a fall, it sends a `trim` based on the mean speed
  over the last 0.6 s, using 0.035 m/s per degree from run 7's simulation.
  Both guides now say to write `auto off` into `cmd.txt` before the first
  run. The team's script itself is unchanged.
- **The flight recorder dumps only when a run ends.** A run that balances
  must be stopped with KEY1 (reason 2) to get its `E`/`T` lines. Picking the
  car up ends it by tilt or free-spin and records the pick-up instead.
- **What is never logged:** roll, yaw rate, raw encoder counts, commands and
  the 34 network inputs. Full-rate data covers only the last 0.6 s
  (`FR_N = 120`). Streaming at 200 Hz is not possible because the control
  interrupt uses 4.6 ms of each 5 ms tick; a larger post-run buffer is the
  way to get more.

## Later the same day: controls, next steps, log checker

3. **Added the car's controls to Step 10:**
   - KEY1, the auto-start, the app buttons and their command values, and the
     cut-out reason codes.
   - Read from `rl_mode.c` and the patched `app_control.c`.
   - Added a warning that the car restarts by itself when stood back up with
     the motors on.
4. **Added Step 11, "What to do next":**
   - A decision table from test result to cause, action and files.
   - The bench motor sweep procedure, with the model's free-spin speed at each
     PWM.
   - The retrain and re-flash steps. Run 8 took 9197 s for 30M steps.
   - A table of every Python file involved and what it needs.
5. **Wrote `firmware/check_car_log.py`.**
   - It parses the helper's log and prints, for each run: stayed up or fell,
     frozen output, impossible speeds and one-tick jumps, timing, PWM
     sign-flip rate, and the creep from the 10 Hz stream.
   - `--last N` limits it to the most recent runs; `--plot` saves a PNG of
     each run.
   - Standard library only; matplotlib is needed only for `--plot`.
   - **Checked against the team's run-7 log (11 runs):**
     - It flags the frozen −1923/−1466 output, the tens-of-m/s speed readings
       and the late ticks on the half-rate builds.
     - The plot of the last run shows the sequence: a −10 m/s spike, then the
       PWM flips to the frozen values, then the fall.

Corrections made while writing it:

- **Reason 2 is not always "stopped by you".** The stock `Turn_Off()` also
  cuts at 40° by its own angle estimate. Two run-7 runs ended with reason 2
  at 39–40°. The checker counts reason 2 as a fall when the largest pitch is
  30° or more, and the guide says so.
- **"Impossible speed" is 1.5 m/s, not 1 m/s.** In the model, a wheel spinning
  in the air reaches 32.9 rad/s at PWM 2800, which is 1.10 m/s at the rim.
  That agrees with the spec's 333 RPM at 12 V. The 9.35 rad/s figure quoted
  earlier is for the logged free-spin PWMs (about 1985/1705), not for full PWM.
- **The bench sweep cannot be done on the mode-28 build yet.** It needs a
  serial command that sets a raw PWM. The guide says this.

## Repo cleanup: files the current model does not use

I traced what each script imports and loads, then made three changes.

6. **Moved the run-8 dependencies out of `train_yahboom_3d.py`.**
   `train_real_robot.py` imported `VelocityCommandWrapper`,
   `CurriculumCallback` and `linear_schedule` from it.
   - These now live in `train_real_robot.py`, together with the constants they
     read: `EVAL_COMMANDS`, the turn-reward weights and `SHAPING_GAMMA`. Long
     comments about the earlier plant are shortened.
   - `PWMCommandWrapper` no longer overrides `_sample_command_timer`. The
     override existed only because the parent read the other module's
     `MAX_EPISODE_STEPS` (1000); in the same module it reads 2000.
   - **Checked for identical behaviour:**
     - A seeded 3000-step rollout of the training env and of the eval env,
       recorded before and after the change, matches bit-for-bit: all
       observations, rewards and episode ends.
     - The run-8 checkpoint loads without warnings.
     - `export_stm32.py` and `quantize_weights.py` regenerate byte-identical
       headers.
     - A 1024-step PPO run with the curriculum and eval callbacks works.
     - `check_obs_builder.py` passes (worst 1.2e-6).
     - `enjoy_drive.py` imports cleanly.
7. **Moved 34 unused files into `archive/` with `git mv`, so their history is
   kept:**
   - `train_yahboom_3d.py`, `their_robot.xml`, `my_robot.xml`, `enjoy_yahboom.py`;
   - `pid_baseline.py`, `pid_eval_benchmark.py`, `policy_eval_benchmark.py`;
   - `fix_stls.py` and `inverted_pendulum/` (26 files).

   `archive/README.md` says what each file was and why it is archived. The
   archived scripts still run from `archive/`: `train_yahboom_3d` builds its
   env on `their_robot.xml`.
8. **Replaced the `.vscode` configs**, which were copied from the pendulum
   project:
   - They launched `play.py` and `record_gif.py` and used a `.venv` folder;
     none of these exist here.
   - The new configs run `enjoy_drive.py` (with and without the slider panel),
     training, both exports and `check_car_log.py`, using `venv/`.

Docs:
- The top-level README now describes the PID scripts as running on the old
  plant. It previously said "the same plant", which was wrong.
- `requirements.txt` lists the optional packages: pyserial, matplotlib and
  trimesh.

**Found, not fixed:** `PWMCommandWrapper.reset()` draws the episode's car
randomisation (motor, latency, IMU errors) from `np_random` before the parent
`reset(seed=...)` seeds it. So the first episode after a seeded reset is not
reproducible: two runs of `check_obs_builder.py` give slightly different
worst-case numbers. Training is unaffected in any way that matters, but a
fully reproducible seeded run would need the draws moved after
`super().reset()`. That change would alter the random stream, so it is not
part of this refactor.

## Docs rewrite: one spoon-fed deployment path

9. **Rewrote `README.md` and `firmware/README-STM32-DEPLOYMENT.md`** to be
   shorter, around a single proven route:
   - the team's mode-28 Keil project (v6), plus the repo's four run-8 files,
     one edited line in `rl_mode.c`, and one Keil define
     (`POLICY_NO_FLOAT_INFER`);
   - then build, FlyMCU, logger, mode select and calibration, in-hand test,
     floor runs, log check, and what next: Steps 1–9.

   Everything technical moved to Reference sections A–H: log format, action to
   PWM and the 1300 dead band, hardware and pins, the 34 inputs, timing, stack
   and safety, PC checks, porting, and evidence. The exact strings come from
   the team's v6 sources:
   - the old `policy_build_obs_rp` call;
   - the Keil define box, `STM32F10X_HD,USE_STDPERIPH_DRIVER`;
   - the `APP\RL` folder and the `OBJ\stm32_Balance_Car_L.hex` output.

10. **Correction: KEY1 does not stop a run.** I read the stock firmware's
    start-up (`main.c`, `app_mode.c`, `bsp_key.c` in the team repo's copy,
    branch `codex/routea-mobile-v1.5`, `RouteA_Mobile_MVP/firmware`):
    - KEY1 is read only at power-up. The first press confirms the mode,
      chosen by turning a wheel by hand. The second press sets
      `Stop_Flag = 0` (`put down key start!`).
    - Nothing reads KEY1 after that. The pick-up/put-down detection runs only
      in mode 1, and `Turn_Off()` cuts on tilt above 40°, battery below 9.6 V,
      or `Stop_Flag`.

    So in mode 28 a run ends only by tipping past 40°, a lifted car, or low
    battery. Earlier today's Step 10 and the drop-in README said to stop a run
    with KEY1; both now say to tip the car past 40° by hand.

    `check_car_log.py` changed to match:
    - reason 2 means the stock 40° check or a low battery;
    - a run of 5 s or more that ends past 40° counts as "stayed up";
    - a reason-2 end at low tilt is flagged as a possible low battery.

    Caveat: the team's project is a different copy of the same Yahboom
    firmware. Its KEY handling was not in their source snapshot, so this rests
    on the stock code plus their README ("keeps the stock KEY mode
    selection").
11. **Dropped from the guide as not applicable:**
    - **STM32CubeMX steps.** The stock project uses the Standard Peripheral
      Library (`USE_STDPERIPH_DRIVER`) and has no `.ioc` file to open.
    - **The 200 Hz timer.** The stock MPU6050 interrupt already runs the loop
      at 200 Hz.
    - **The 2 KB stack advice.** It is now 4 KB, as measured necessary.
    - **The float-inference timing estimates.** They are replaced by the
      team's measured int16 timings.

### Moved here from the deployment guide

**Slope sweep, run 7 (superseded policy; kept for the method).**
`apply_slope()` in `enjoy_drive.py` tilts gravity, not the floor, while the
observation and reward stay world-referenced. That overstates difficulty: an
earlier policy drifted about 5× worse at a nominal 5° than under a true floor
tilt (`2026-09-23-slope-run3-invalid.md`). Read the angles as a stress
ordering, not a real-incline rating.

| slope | hold: steps / v_fwd | +0.15 cmd: steps / v_fwd | +0.30 cmd: steps / v_fwd |
|---|---|---|---|
| 0° | 2000 / +0.002 | 2000 / +0.147 (98%) | 2000 / +0.270 (90%) |
| 2° | 2000 / −0.105 | 2000 / +0.064 | 2000 / +0.187 |
| 4° | 2000 / −0.213 | 2000 / −0.061 | 2000 / +0.107 |
| 6° | 2000 / −0.358 | 2000 / −0.177 | 2000 / +0.037 |
| 8° | 838 / −0.565 (falls) | 2000 / −0.334 | 2000 / −0.135 |
| 10° | 291 / −0.545 (falls fast) | 973 / −0.520 (falls on 2/3 seeds) | 2000 / −0.300 |
| 12.5° | 115 / −0.518 (falls) | 145 / −0.549 (falls) | 732 / −0.512 (2/3 full, 1/3 falls at 110) |
| 15–20° | under 110 steps on every command | — | — |

What the table shows:
- Up to 6° the robot drifts downhill at about 0.06 m/s per degree while
  holding station. The leaky integrals are for tracking error, not for
  rejecting a constant force.
- 8° is the first angle where holding station fails while driving still
  works.
- Above 15° nothing survives half a second.
- Not tested: friction, and payload combined with slope.

**Motor sizing, measured on the superseded torque-output policy** (0.6 N·m
driver limit; never re-measured for the PWM policies). Commanded torque over 20
eval conditions:

| | per wheel | at the motor | share of the 0.6 N·m limit |
|---|---|---|---|
| mean | 0.053 N·m | 2.5 mN·m | 8.9 % |
| p99 | 0.148 N·m | 7.0 mN·m | 24.6 % |
| peak | 0.442 N·m | 21.0 mN·m | 73.6 % |

The torque was above half the limit 0.04 % of the time and never above 80 %,
so the motor has ample margin. Still to check: the AT8236 current rating
against the peak current (21 mN·m / Kt, about 1–2 A for Kt of 0.01–0.02 N·m/A).
`real_robot.xml`'s driver limit is now 0.4 N·m at the wheel; the motor stalls
at 0.5679 N·m.

**PID rate sweep, superseded 80 Hz plant.** Of 120 gain sets, the number
stable was 91 at 80 Hz, 24 at 40 Hz and 0 at 25 Hz. It is about a different
plant and controller, and does not transfer.

## Open items

- The on-car run-8 test itself, following Steps 5–8 of the new guide.
- Optional: a serial `stop` command in `rl_mode.c`, so that a balancing run
  can end without tipping and its flight recording shows the balancing.
- A raw-PWM serial command in `rl_mode.c` for the bench sweep.
- Optional: a longer flight recorder (about 5 s, with commands and roll) in
  `rl_mode.c`, after checking free RAM in the Keil map file.
- Still open from 2026-10-06: bench motor sweep, watchdog, slope and
  friction sweeps for run 8, gearbox backlash.
