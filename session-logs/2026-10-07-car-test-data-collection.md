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

## Open items

- The on-car run-8 test itself, following Steps 10–11.
- A raw-PWM serial command in `rl_mode.c` for the bench sweep.
- Optional: a longer flight recorder (about 5 s, with commands and roll) in
  `rl_mode.c`, after checking free RAM in the Keil map file.
- Still open from 2026-10-06: bench motor sweep, watchdog, slope and
  friction sweeps for run 8, gearbox backlash.
