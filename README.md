# DIP-E026 — self-balancing robot, RL policy + STM32 deployment

A two-wheeled self-balancing robot (Yahboom kit hardware) controlled by a PPO
policy trained in MuJoCo, with a hand-written C port for flashing that same
policy onto the robot's STM32F103 over UART.

**Current status**: run 8, trained on a motor model and sensor pipeline
corrected from the car's own logs after run 7 failed on hardware. In
simulation it holds every stress case tested, and the actual C firmware code
balances the simulated robot in closed loop — but run 8 has not yet run on the
car. See [firmware/README-STM32-DEPLOYMENT.md](firmware/README-STM32-DEPLOYMENT.md#readiness-check-before-you-start)'s
"Readiness check" first.

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
| `motor_model.py` | Lumped gearmotor model that turns the policy's PWM output into wheel torque: the firmware's 1300-count compensation, then a true dead zone (~1460 counts, fitted to the car's free-spin log) below which the motor gives no torque. Randomised per training episode. |
| `enjoy_drive.py` | Interactive demo: drive the trained policy in the MuJoCo viewer, with live sliders for commands and for conditions (friction/slope/payload) to see where the policy's trained envelope ends. |
| `export_stm32.py` | Exports a trained checkpoint to `firmware/policy_weights.h`, a C header holding the network's weights. |
| `firmware/` | The STM32 port: `policy.c`/`policy.h` (network inference + observation assembly), generated weight headers, PC-side verification tests, and **[README-STM32-DEPLOYMENT.md](firmware/README-STM32-DEPLOYMENT.md)** — the full flashing guide. Start there for anything hardware-related. |
| `models/best_real/best_model.zip` | **the current trained policy** (run 8: 34 inputs, 200 Hz, PWM action, eval reward 3637 at 29.64M steps). What `enjoy_drive.py` defaults to and what `firmware/policy_weights.h` and `policy_weights_q.h` were generated from. Run 7 is kept in `models/best_real_RUN7_yaw/` (untracked). |
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
3. `real_robot.xml` was rebuilt from the team's measured parameters. History
   taps, an action-rate penalty and two leaky integrals (forward-velocity
   error, then heading error) followed, to kill steady-state drift that a
   reward with no memory can't see. Run 7 converged cleanly in simulation.
4. On the car, run 7 fell within 0.2–2 s. After two integration bugs on the
   firmware side (a half-rate control loop, a stack overflow), the car's own
   logs showed the real cause: the motor model treated the firmware's
   dead-band compensation as useful torque ("no gentle nudge"), when the real
   gearmotor gives ~nothing below ~1460 counts. Run 7 had learned a control law
   for an actuator that does not exist; with a realistic dead zone it falls in
   0.7–2 s in simulation too (`session-logs/2026-10-06-hardware-mode28-diagnosis.md`).
5. Run 8 corrects the motor model and trains on exactly what the firmware
   observes: encoder odometry, yaw pinned to 0, IMU calibration errors, a 1–2
   tick control delay, no wheel angles. Not yet tested on the car.

## Caveats, stated plainly

- Run 8's results are simulation and PC-side verification. **Run 8 has not run
  on real hardware yet.** Run 7 did, and failed — that failure is what run 8
  was built from.
- The motor dead zone is fitted to one logged free-spin run plus the parameter
  sheet, not a proper bench sweep.
- Slope and friction have not been characterised for run 8.
- `fix_stls.py` and `meshes/` relate to an abandoned mesh-based prototype from
  day one of the project and aren't part of the active pipeline.
