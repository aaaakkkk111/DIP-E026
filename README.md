# DIP-E026 — self-balancing robot: RL policy and STM32 deployment

A PPO policy trained in MuJoCo to balance and drive the Yahboom two-wheeled
car (STM32F103, MPU6050, AT8236 driver, JGB37-520 motors), plus the C code that
runs it on the car.

**Status (2026-10-21):** run 21 (21M) is exported for its first car test: in
simulation it stands at PID level empty and loaded (2 kg), but barely survives
a push when empty. See `TRAINING_ON_DESKTOP.md` ("Now: flash run 21") and
`PROJECT_OVERVIEW.md`.

**Status (2026-10-19):** runs 8, 9, 11 and 13 have balanced on the car, 11–58 s
per run, ended by hand. The simulator now uses the motor measured on the car,
with stiction, and about 2° of gear slack. The stock PID and run 11's real
balancing both match it: run 11 sways at 12–22 deg/s on the car and 13.5–21 in
simulation.

The checkpoint in `models/best_real/` is **run 15**, not yet tested on the
car. It was trained on that motor with pushes up to 3 N. In simulation it
survives a 3 N shove as often as the stock PID does, like run 13 (on the car
since 2026-10-14), and stands about 20 % calmer than run 13, though still
about 3× wobblier than the PID
([log](session-logs/2026-10-18-run15-desktop.md)). Its firmware needs two
`rl_mode.c` edits: turning at 1.0 rad/s instead of 0.5, and a fixed pitch
zero, so `cal` only needs the car held still. See
[session-logs/2026-10-08-run12-measured-motor-and-pushes.md](session-logs/2026-10-08-run12-measured-motor-and-pushes.md),
[session-logs/2026-10-10-turning-calibration-mujoco.md](session-logs/2026-10-10-turning-calibration-mujoco.md),
and [TRAINING_ON_DESKTOP.md](TRAINING_ON_DESKTOP.md) for training on one PC
and testing on another (§5 has the firmware edits).

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
| `real_robot.xml` | MuJoCo model built from the team's measured parameter sheet, with the motor rotors behind a gearbox with slack |
| `motor_model.py` | turns a PWM command into wheel torque, as measured on the car: the firmware's 1300-count compensation, a dead zone of 1485 at rest and 1400 when turning, 0.75 N·m limit, Coulomb and viscous friction |
| `enjoy_drive.py` | interactive simulator demo |
| `export_stm32.py` | writes the trained network to `firmware/policy_weights.h` |
| `models/best_real/best_model.zip` | **the current policy**: run 13, 34 inputs. Earlier runs are kept as `models/best_real_RUN*/` and `models/ppo_real_robot_RUN*` |
| `sim_tools/`, `car_logs/` | the motor tests, car-log analysis and simulator checks behind the measured model, and the car's logs |
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
   and no wheel angles. Runs 8, 9 and 11 balance on the car.
6. The motors were then measured on the car: steps, reversals and low-PWM
   stiction. The stock PID, run through the measured motor, matches the car.
   The policies' large "wobble" numbers turned out to be the final hand push in
   the flight recorder; their real balancing matches the simulator too.
7. Runs 12 and 13 train on the measured motor with pushes
   ([log](session-logs/2026-10-08-run12-measured-motor-and-pushes.md)).

## Caveats

- Run 13 has not run on the car yet.
- In simulation, runs 12 and 13 sway about twice as much standing still as run
  11: 24–35 against 11–16 deg/s at 2° of slack. That is the price of their
  push tolerance; the stock PID manages 7.
- Turning above 0.5 rad/s and the fixed pitch zero are checked in simulation
  only: 100 % tracking up to 1.5 rad/s, about 1 cm/s of creep per degree of
  zero error, no falls within ±4°.
- MuJoCo is pinned to 3.2.7: Windows Smart App Control blocks 3.2.3 on the
  desktop. It reproduces run 13's numbers exactly.
- Slope and friction have not been characterised.
- The mode-28 firmware has no watchdog yet.
