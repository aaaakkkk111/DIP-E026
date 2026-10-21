# 2026-10-21/22: run 21 on the car, trim, floor slope (laptop)

Logs: `car_logs/2026-10-21/`:
- `run21_1`: no trim;
- `run21_trim3`: trim 1.5;
- `autotrim_1`: live auto-trim;
- `direction_test`: trim 2.5, turned 180°;
- `run21_trim`, `run21_trim2`: aborted attempts to send the trim, no runs.

Battery 12.0–12.2 V.

## 1. Build

Folder `C:\Users\USER\Desktop\car_firmware_run21`, copied from `car_firmware_run15`, with the
repo's `policy.c`, `policy.h`, `policy_weights.h` and `policy_weights_q.h` (run 21, 21M).

Checks:
- `test_policy`: PASS (worst error 1.96e-2).
- `check_obs_builder`: PASS. It first failed on obs[6] only because
  `firmware/test_obs_builder.dll` was still built from the old `policy.c`. **Rebuild it after
  every `policy.c` change:**
  ```
  gcc -O2 -shared -o test_obs_builder.dll test_obs_builder.c policy.c -lm
  ```

**Flash:**
- The `osc`/`lean` filters took the image to 65,576 B, **40 B past 64 KB**. This board loads
  garbage initial values past 0x08010000; see `removed_modes_stubs.c` in the Keil project.
- Fix: in `APP/RL/rl_mode.c` (Keil folder only, not in the repo), the motor-test console's four
  refusal messages are merged into one: `mt: no (mt on, WAIT, motors, idle)`.
- Image now 65,488 B, hex ending at 0x0800FFD0: **48 B spare**. Any firmware addition needs
  trimming first.

## 2. Run 21 on the car: the shake is gone

Mid-run, from the 200 Hz stream ("fast" = gyro sd after removing each 0.4 s burst's trend):

| | wobble | fast shake | motor reversals |
|---|---|---|---|
| **run 21**, 4 runs | **2.8–4.3 deg/s** | 1.7–3.1 | 0.5–1.1 /s |
| run 21, one run with touches | 12.2 | 9.0 | 3.6 /s |
| stock PID (2026-10-08 / 10-20) | 7.6 / 11.7 | | |
| run 15 (2026-10-20) | 44–49 | 43–47 | 15–17 /s |
| run 13 (2026-10-20) | 51–98 | 48–96 | 5–14 /s |

**Run 21 is the calmest controller this car has run, calmer than the stock PID.** It sits in
the motor dead zone (|PWM| < 1450) 50–80 % of the time and rarely reverses, so it does not bang
through the gear slack the way runs 13 and 15 do: the 7 Hz loop in
`2026-10-20-car-shake-and-run16.md`.

Push recovery when empty, the simulator's weak point (2 N: 1/18), was **not tested on the car
yet**. Neither was the 2 kg load.

## 3. Creep, trim and the floor

`cal` measures only the gyro bias; the pitch zero is fixed at 0. `trim X` adds X degrees to the
zero (incremental, any state, lost at restart or `cal`).

| setup | trim | creep |
|---|---|---|
| `run21_1` | 0 | −1.2 cm/s (back), 0.63 m in 53 s |
| `run21_trim3` | 1.5 | +0.3 to +0.5 cm/s (forward) |
| `autotrim_1`, live | 1.1 → 3.7 | −2 cm/s at 1.1–2.6°, about −0.5 at 3–4° |
| `direction_test`, front wall | 2.5 | **0.00 cm/s over 186 s** (4 cm), then backwards after pushes |
| `direction_test`, turned 180° | 2.5 | forwards, i.e. toward the **same wall** |

**The creep follows the room, not the car.** Turned around, it still drifts toward the same
wall. So there is a floor slope or cable pull (both are possible; neither was measured). That also explains why
two sessions gave 1.1° and about 4°. The car's own offset is between them: **about 2.5°**.

Within one setup the creep falls by about 1 cm/s per degree of trim, as the simulator says.

`sim_tools/auto_trim.py` (new) logs like `stream_log.py` and nudges the trim live:
- 10 s push-free windows, gain 0.5 deg per cm/s, at most 0.5° per step and 4° in total.
- It works, but the speed wanders ±2–4 cm/s within 10 s, so most steps chase noise. It is
  useful only on a level floor without cable pull.

**After pushes it wanders and does not come back.** In a pushed run the 4 s mean speed swings
between −8 and +6 cm/s while the tilt swings ±4°. Two reasons:
- It controls speed only in occasional nudges out of the dead zone.
- The position memory (leaky, τ = 2 s) forgets the spot within seconds.

On a slope, nothing then stops the downhill roll.

## 4. For run 22 (desktop)

Keep everything that made run 21 calm:
- the PWM-step penalty, gated near upright;
- the shake (osc) penalty;
- the `osc`/`lean` inputs;
- payload 0–2 kg.

Then:
1. **Longer position memory while standing still.** `POSITION_TAU_S` 2 → 5–10 s, with the
   integral reset (or the short τ kept) whenever the command changes, so it still never "catches
   up" after driving. The firmware must match: `POLICY_POS_DECAY` in `policy.h`, and the same reset
   rule in `policy.c`.
2. **Floor slope in training.** Random ±2° per episode, by tilting gravity as
   `enjoy_drive.apply_slope` does. Check first what the policy's pitch input means then. On the
   car the IMU pitch is relative to gravity (the accelerometer), so with tilted gravity the
   simulated pitch observation must be relative to gravity too, not to the world frame.
3. **Pushes scaled to the car's mass** (from the run 21 log), so the empty car learns push
   recovery. In simulation run 21 survives only 1/18 2 N pushes empty, against 18/18 for the PID.

Criteria (simulation):
- standing wobble no worse than run 21 (6.6–8.3 empty, 2° slack);
- reversals ≤ about 10 /s;
- 2 N empty pushes clearly better than 1/18, ideally the PID's 18/18;
- on a 1–2° slope: creep under about 1 cm/s, and back to the spot within about 10 s after a push.

## 5. Open

- The trim is not yet built into the firmware. Planned: 2.5°, by setting `pitch_zero` to
  2.5° after `att_from_acc()` in `rl_mode.c` instead of 0, then rebuilding (it should fit in 48 B).
- Floor slope vs cable pull: not separated. A marble or spirit level would tell.
- Run 15's left-right wagging (2026-10-20) is still unexplained; run 21 did not show it.
