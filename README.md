# DIP-E026 — self-balancing robot, RL policy + STM32 deployment

A two-wheeled self-balancing robot (Yahboom kit hardware) controlled by a PPO
policy trained in MuJoCo, with a hand-written C port for flashing that same
policy onto the robot's STM32F103 over UART.

**Current status**: the policy (run 7) is trained, converged, and its STM32
port is bit-verified against it on the PC side — but it has not yet run on
real hardware. See [firmware/README-STM32-DEPLOYMENT.md](firmware/README-STM32-DEPLOYMENT.md#readiness-check-before-you-start)'s
"Readiness check" before treating this as ready to just deploy.

## Quickstart

```bash
python -m venv venv
venv\Scripts\activate          # Windows; `source venv/bin/activate` on Linux/Mac
pip install -r requirements.txt
python enjoy_drive.py          # drive the trained policy in simulation
```

`enjoy_drive.py` opens a MuJoCo viewer plus a slider panel for the drive
commands and for stress-testing conditions the policy wasn't trained on
(friction, slope, payload beyond the trained range). Arrow keys drive too.
Use `--no-gui` for a terminal-only version if the slider panel's `tkinter`
dependency isn't available on your system.

Verified (2026-10-01): a clean clone + fresh venv + `pip install -r
requirements.txt` successfully runs `enjoy_drive.py`, `export_stm32.py`, and
the firmware's own PC-side verification build — not just assumed to work.

## What's actually in this repo

| path | what it is |
|---|---|
| `train_real_robot.py` | **the current training script.** Trains against `real_robot.xml`, the measured hardware's actual plant, at 200 Hz, with a PWM (not torque) action space. This is what produced `models/best_real/best_model.zip`. |
| `real_robot.xml` | MuJoCo model built from the team's measured parameter sheet (mass, wheel geometry, inertia) — the plant the current policy was trained on. |
| `motor_model.py` | Lumped DC-motor model (stall torque + back-EMF) that turns the policy's PWM output into wheel torque in simulation, matching the real firmware's deadband compensation exactly. |
| `enjoy_drive.py` | Interactive demo: drive the trained policy in the MuJoCo viewer, with live sliders for commands and for conditions (friction/slope/payload) to see where the policy's trained envelope ends. |
| `export_stm32.py` | Exports a trained checkpoint to `firmware/policy_weights.h`, a C header holding the network's weights. |
| `firmware/` | The STM32 port: `policy.c`/`policy.h` (network inference + observation assembly), generated weight headers, PC-side verification tests, and **[README-STM32-DEPLOYMENT.md](firmware/README-STM32-DEPLOYMENT.md)** — the full flashing guide. Start there for anything hardware-related. |
| `models/best_real/best_model.zip` | **the current trained policy** (run 7: 34 inputs, 200 Hz, PWM action, eval reward 3400 at 29.83M steps). What `enjoy_drive.py` defaults to and what `firmware/policy_weights.h` was generated from. |
| `session-logs/` | Dated write-ups of every training run and why each plant/reward change was made, plus the raw training logs. The detailed history behind every number and design choice in this README. |
| `pid_baseline.py`, `pid_eval_benchmark.py`, `policy_eval_benchmark.py` | A classical cascade-PID controller on the same plant, and benchmarks to compare it against the trained policy head-to-head — used early on to confirm the plant itself supports sustained driving before committing to RL. |
| `train_yahboom_3d.py`, `their_robot.xml`, `models/best_their/` (now untracked) | **Superseded.** An earlier plant model that turned out to be wrong for the actual hardware by 3.5x in COM height and 12x in inertia. Kept for history; `train_real_robot.py` imports some shared infrastructure from `train_yahboom_3d.py` (the curriculum callback, the velocity-command wrapper), but the plant and action space are both current. |
| `inverted_pendulum/` | A **separate, unrelated** earlier PPO experiment (single-cart inverted pendulum, not the balance car). Has its own README and requirements. |

## The short version of how we got here

1. A classical PID controller confirmed the plant could sustain driving while
   balancing — ruling out "the plant is uncontrollable" before blaming RL.
2. Early training used `their_robot.xml`, a plant model that turned out to be
   wrong for the real hardware by a wide margin (3.5x COM height, 12x inertia)
   — a different control problem, not a tolerance issue.
3. `real_robot.xml` was rebuilt from the team's measured parameters.
   Discovering the actuator is bang-bang (no gentle nudge — the smallest
   non-zero command is already 64% of the driver limit) drove most of the
   observation and reward design from there: history taps so the policy can
   dither, a rate penalty so it doesn't dither needlessly fast, and two leaky
   integrals (forward-velocity error, then heading error) to kill steady-state
   drift that a reward with no memory can't see.
4. Seven training runs later (see `session-logs/`), run 7 fixed the last
   measured defect (turn overshoot from an un-integrated heading error) and
   converged cleanly at 30M steps.
5. The STM32 firmware port is new as of this writing and PC-verified, not yet
   hardware-tested — see the firmware README's readiness section for exactly
   what that does and doesn't mean.

## Caveats, stated plainly

- Everything above "converged" and "bit-verified" is simulation or PC-side
  verification. **Nothing has run on real hardware yet.**
- The 200 Hz / 5 ms control-loop timing budget on the actual F103 (no FPU) is
  an estimate, not a measurement.
- Slope tolerance has been characterised in simulation (see the firmware
  README's "Known limitations") with a known caveat about the test method
  overstating difficulty; friction has not been characterised at all.
- `fix_stls.py` and `meshes/` relate to an abandoned mesh-based prototype from
  day one of the project and aren't part of the active pipeline.
