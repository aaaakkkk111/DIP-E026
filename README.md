# DIP-E026 — self-balancing robot: RL policy and STM32 deployment

A PPO policy trained in MuJoCo to balance and drive the Yahboom two-wheeled
car (STM32F103, MPU6050, AT8236 driver, JGB37-520 motors), plus the C code that
runs it on the car.

**Status (2026-10-08):** run 8 and run 9 have both run on the car. They
balance (17–51 s per run, ended by hand) but wobble at about 7 Hz. The causes
are sense-to-act delay and gearbox slack, both now modelled in the simulator;
run 10 trains with them. The checkpoint in `models/best_real/` is run 9. See
[session-logs/2026-10-08-run9-run10-delay-and-gear-slack.md](session-logs/2026-10-08-run9-run10-delay-and-gear-slack.md),
and [TRAINING_ON_DESKTOP.md](TRAINING_ON_DESKTOP.md) for training on one PC
and testing on another.

## Run it in simulation

```bash
python -m venv venv
venv\Scripts\activate          # Windows; `source venv/bin/activate` on Linux/Mac
pip install -r requirements.txt
python enjoy_drive.py          # drive the trained policy; add --no-gui without tkinter
```

A MuJoCo viewer opens with a slider panel. Drive with the sliders or the arrow
keys. Other sliders change friction, slope and payload, including beyond the
trained range.

## Put it on the car

Follow **[firmware/README-STM32-DEPLOYMENT.md](firmware/README-STM32-DEPLOYMENT.md)**.
In short:

1. Copy four files from `firmware/` into the team's mode-28 Keil project, edit
   one line, and add one define.
2. Build the hex in Keil.
3. Flash it with FlyMCU.
4. Start the serial logger.
5. Select mode 28 and calibrate.
6. Test in hand, with the wheels off the ground.
7. Do floor runs.
8. Check the log with `firmware/check_car_log.py`.
9. Decide what to do next, using the guide's table.

## What's in this repo

| path | what it is |
|---|---|
| `train_real_robot.py` | training script: `real_robot.xml` at 200 Hz, PWM actions, inputs computed exactly as the firmware computes them, randomised motor and sensor errors |
| `real_robot.xml` | MuJoCo model built from the team's measured parameter sheet |
| `motor_model.py` | turns a PWM command into wheel torque: the firmware's 1300-count compensation, then the motor's real dead zone (about 1460 counts) |
| `enjoy_drive.py` | interactive simulator demo |
| `export_stm32.py` | writes the trained network to `firmware/policy_weights.h` |
| `models/best_real/best_model.zip` | **the current policy**: run 8, 34 inputs, eval reward 3637 at 29.64 M steps |
| `firmware/` | the C port (`policy.c`/`policy.h`), float and int16 weight headers, PC checks, `check_car_log.py`, and the deployment guide |
| `session-logs/` | dated write-ups of every run and every change, with the reasons and the raw training logs |
| `.vscode/` | VS Code run configurations for the scripts above |
| [`archive/`](archive/README.md) | files run 8 does not use: the first robot models and training script, the PID baseline, an unrelated pendulum project |

Checked on a clean clone and a fresh venv (2026-10-01). After the 2026-10-07
cleanup, training, export and the firmware checks were re-run in the project
venv.

## How we got here

1. A classical PID showed that the early plant (`their_robot.xml`) could
   balance and drive, which ruled out "uncontrollable" before RL was blamed.
2. That plant turned out to be wrong for the real car by 3.5× in
   centre-of-mass height and 12× in inertia. `real_robot.xml` was rebuilt from
   measured parameters.
3. History inputs, an action-rate penalty and two leaky integrals (position,
   then heading) removed the steady drift. Run 7 converged in simulation.
4. On the car, run 7 fell within 0.2–2 s. The team fixed two firmware bugs, a
   half-rate loop and a stack overflow. The logs then showed the real cause:
   the motor model treated the firmware's dead-band compensation as useful
   torque, while the real motor gives almost nothing below about 1460 counts
   ([diagnosis](session-logs/2026-10-06-hardware-mode28-diagnosis.md)).
5. Run 8 fixes the motor model and trains on exactly what the firmware sees:
   encoder odometry, yaw pinned to 0, IMU calibration errors, a 1–2 tick delay,
   and no wheel angles.

## Caveats

- Run 8's results come from simulation and PC checks only. It has not run on
  the car.
- The motor dead zone is fitted to one logged free-spin run plus the parameter
  sheet, not to a bench sweep.
- Slope and friction have not been characterised for run 8.
- The mode-28 firmware has no watchdog yet.
