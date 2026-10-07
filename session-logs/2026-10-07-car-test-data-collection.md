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

## Open items

- The on-car run-8 test itself, following Step 10.
- A script that parses `rl_serial_log.txt` and prints the run-8 pass table
  per run (run length, frozen output, speed spikes, late ticks, sign-flip
  rate), with plots of the `E` data.
- Optional: a longer flight recorder (about 5 s, with commands and roll) in
  `rl_mode.c`, after checking free RAM in the Keil map file.
- Still open from 2026-10-06: bench motor sweep, watchdog, slope and
  friction sweeps for run 8, gearbox backlash.
