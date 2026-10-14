# 2026-10-10 (desktop) — Turning speed, calibration, MuJoCo 3.2.7

Follows `2026-10-08-run12-measured-motor-and-pushes.md`. The laptop had run 12
on the car: it balanced and turned, but turned slowly, and it crept unless
`cal` was done with the pitch zero near 0°. No training this session; the
simulation checks below use runs 12 and 13 as trained.

## Summary

- **Turning is limited by the firmware, not the policy.** `CMD_W = 0.50f` in
  `rl_mode.c` is also the training maximum (`MAX_V_TURN = 0.5`). In
  simulation, runs 12 and 13 track up to 1.5 rad/s. Run 13's firmware uses
  `CMD_W = 1.00f`.
- **`cal` can't go: it measures the gyro bias.** But the pitch and roll zeros
  can be fixed at 0, so `cal` works in any still pose. The creep from a wrong
  zero is about 1 cm/s per degree, with no falls within ±4°.
- **MuJoCo is now 3.2.7.** Smart App Control blocked 3.2.3. 3.2.7 reproduces
  run 13's numbers exactly.

## 1. Turning

All in simulation: 2° slack, 2 seeds × 5/10/15 ms delay (6 runs per cell),
`simlib.run(w_cmd=...)`.

| command (rad/s) | 0.5 | 0.75 | 1.0 | 1.5 |
|---|---|---|---|---|
| run 12 tracking | 100 % | 100 % | 100 % | 100 % |
| run 13 tracking | 100 % | 100 % | 100 % | 100 % |
| full turn | 12.6 s | 8.4 s | 6.3 s | 4.2 s |
| held 10 s | 6/6 | 6/6 | 6/6 | 6/6 |

The pitch-rate wobble while turning is 2–4 deg/s for both. Turning is gentle
on balance; the policies generalise past their trained ±0.5 rad/s.

**Firmware:** `#define CMD_W 1.00f` (was 0.50). 1.5 also held, but 1.0 leaves
margin for floor grip, which the simulator doesn't model beyond its friction
range.

## 2. Calibration and creep

**Why mode 1 needs no `cal` but mode 28 does:**
- Mode 1's Kalman filter estimates the gyro bias as it runs, and its speed
  loop has a true integral, which absorbs a wrong angle zero.
- Mode 28's gyro bias is about 0.04 rad/s raw, 14× the ±0.003 the policies
  were trained with, so `cal` must measure it.
- Mode 28's position memory is a leaky integral (τ 2 s), so a wrong pitch
  zero doesn't get absorbed: it turns into a steady creep.

**More tolerance from training?** Runs 12 and 13 already trained with pitch
zero errors up to ±4° (`PITCH_OFFSET_RAD`), and they don't fall. The creep
comes from the leaky integral, not from intolerance.

**Creep against pitch-zero error**, standing, 2° slack, 10 ms, 2 seeds,
mean speed over 2–10 s, in cm/s (positive is forward):

| zero error | −4° | −2° | 0° | +2° | +4° |
|---|---|---|---|---|---|
| run 12 | +4.9 | +3.0 | +1.0 | −1.0 | −3.2 |
| run 13 | +6.8 | +4.4 | +2.6 | −0.2 | −2.9 |

That is about 1 cm/s per degree, with no falls. Run 13 has a built-in forward
creep of about 2.6 cm/s even at 0° error; `trim -1` to `trim -2` cancels it.
The laptop's run 12 experience (no creep with the zero near 0°) fits run 12's
+1.0 at 0°.

**Firmware:** in the block that finishes `cal`, after
`att_from_acc(ab, &roll_zero, &pitch_zero);`:

```c
pitch_zero = roll_zero = 0.0f;   /* fixed zero: cal now only measures gyro bias, in any still pose */
```

Fixing the roll zero too means `cal` with the car lying down doesn't store a
wrong roll. The routine becomes:
1. select mode 28 and leave the car still, in any position, for about 5 s;
2. press KEY1;
3. stand it up; it starts after 1 s upright.

`trim` still adds to the fixed zero.

Both edits are in `TRAINING_ON_DESKTOP.md` §5, step 4. Neither is compiled
yet.

## 3. MuJoCo blocked again, now 3.2.7

`import mujoco` failed with `WinError 4551 … An Application Control policy has
blocked this file`. This time it was 3.2.3's `_structs` .pyd; on 2026-09-23 it
was `_specs` (see `2026-09-23-smart-app-control-mujoco-block.md`).

- Retrying for 5 minutes didn't help, and WSL isn't installed.
- I downloaded the wheels for 3.2.0–3.3.7, 3.14 and 3.15 into a scratch
  folder and loaded every .pyd with `ctypes.WinDLL`. All of them loaded.
- 3.2.7 reproduces run 13 final exactly at 2° slack: 23.8 / 28.1 / 32.8 deg/s
  at 5 / 10 / 15 ms.
- The venv is now on 3.2.7 (`pip install mujoco==3.2.7`), and
  `requirements.txt` pins it. To undo: `pip install mujoco==3.2.3`.

No Windows security setting was changed. Don't turn Smart App Control off: it
can't be turned back on without reinstalling Windows.

## Open

- **Run 13 on the car** with both firmware edits:
  - standing creep (expect about +2.6 cm/s; `trim -1`);
  - turning at 1.0 rad/s (a full turn in about 6 s);
  - pushes;
  - standing wobble mid-run from the stream (simulation: 24–33 deg/s).
- Driving and turning together above 0.5 rad/s is untested; the app's
  diagonal buttons use `CMD_W` as well.
