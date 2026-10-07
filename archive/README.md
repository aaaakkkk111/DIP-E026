# Archive: files the current model does not use

Nothing here is used by run 8, the current policy: not by training, the
simulator demo, the weight export or the firmware. Nothing outside this folder
imports from it. These files are kept because the session logs refer to them
and they record how the project reached its current design.

For the current pipeline, start at the [top-level README](../README.md).

| file | what it was | why it is here |
|---|---|---|
| `train_yahboom_3d.py` | the first training script: 80 Hz, torque actions, on `their_robot.xml` | superseded by `train_real_robot.py`. The three pieces run 8 still uses (the command wrapper with the base reward, the curriculum callback and the learning-rate schedule) were moved into `train_real_robot.py` unchanged |
| `their_robot.xml` | the teammate's first robot model | wrong for the real car by 3.5× in centre-of-mass height and 12× in inertia about the axle |
| `my_robot.xml` | the very first robot model | badly conditioned; replaced by `their_robot.xml` before any useful training |
| `enjoy_yahboom.py` | viewer for `train_yahboom_3d.py` policies | the model it loads (`models/ppo_yahboom_3d.zip`) is not in the repo |
| `pid_baseline.py` | classical cascade PID on `their_robot.xml` | showed that plant could balance and drive, before RL was blamed. Not re-run on `real_robot.xml` |
| `pid_eval_benchmark.py`, `policy_eval_benchmark.py` | PID and policy scored on the same 20 commands | both run on `their_robot.xml`; the policy benchmark's default model, `models/best_their/`, is not in the repo |
| `fix_stls.py` | repaired STL meshes for a mesh-based prototype | the prototype was abandoned on day one; `meshes/` is not in the repo |
| `inverted_pendulum/` | a separate PPO experiment: a single-cart inverted pendulum | unrelated to the balance car. Self-contained, with its own README, requirements and launch scripts |

## Running them

The scripts find each other and their XML files relative to this folder, so
run them from here with the main venv:

```
cd archive
..\venv\Scripts\python.exe pid_baseline.py
```

`inverted_pendulum/` has its own setup; see its README.
