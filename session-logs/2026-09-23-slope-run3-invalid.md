# Session log — 2026-09-23: run 3 (slope) invalid, and the correction

A 5M-step training run was spent on a slope curriculum that **did nothing**.
This records the bug, how it evaded verification, what the real slope
behaviour is, and what a correct fix looks like.

## The bug

`train_yahboom_3d.py` randomised ground slope by rotating the floor geom:

```python
model.geom_quat[floor_geom_id] = <rotation>
```

**This has no effect.** The floor geom is attached to the world body
(`geom_bodyid == 0`), and MuJoCo treats world-attached geoms as *static*: their
pose is baked at compile time and never recomputed from `geom_quat`. Proof:

```
model.geom_quat set to      : [0.9976 0. 0.0698 0.]     (8 degrees)
data.geom_xmat normal       : [0. 0. 1.]                 <- unchanged
after mj_resetData + forward: [0. 0. 1.]                 <- still unchanged
with a completely fresh MjData: [0. 0. 1.]               <- still unchanged
```

Control test, zero torque, which on any real slope must slide downhill:

| slope | mechanism | drift |
|---|---|---|
| 8° | floor geom rotated | **−0.010 m** (identical to level) |
| 8° | gravity tilted | −0.131 m (slides, as it should) |

## How it evaded verification

The pre-flight check (`scratchpad/slope_ground.py`) read the surface normal
back from **the scipy rotation object it had just constructed**, not from
MuJoCo's `data.geom_xmat`:

```python
normal = rot.apply([0.0, 0.0, 1.0])   # reports what was INTENDED
```

so it echoed the input and confirmed nothing. Contact count (4) and
penetration (<0.33 mm) were also consistent with a flat floor, so they raised
no flags either. **A verification that never reads back from the system under
test is not a verification.**

## Consequences

- Run 3 trained entirely on level ground, with an 18th observation input
  (the velocity-error integral) that carried no usable signal.
- Every "run 3 holds station on slopes" measurement reported during the run
  was taken on flat ground.
- Run 3 is **worse than run 2** and was discarded.

| capability | run 2 | run 3 |
|---|---|---|
| Turning, cmd +0.50 rad/s | **+0.394** | +0.093 |
| Turn on 8° | **+0.396** | +0.097 |
| Drive + turn together | **+0.690 / +0.504** | +0.296 / **+0.007** |
| Uphill drive, cmd +0.30 | +0.298 | +0.309 |
| Station-keeping drift | **+0.016** | +0.065 |

Run 3 lost the turn axis — the capability run 2 had just been retrained to
win — and gained nothing, because the slope it was meant to learn never
existed.

## The other half of the error: the enjoy_drive slope is unfaithful

`enjoy_drive.apply_slope` tilts **gravity**, which does work. But the
observation's orientation quaternion and the reward's pitch term both stay
referenced to world-z, while gravity moves away from it. The robot is then
told it is leaning when it is upright *relative to gravity*, and the reward
penalises exactly the posture the slope requires. A real IMU senses gravity,
so this is not what hardware on a hill experiences.

Effect: it overstates slope difficulty by roughly 5x.

| 5° slope, run 2 | drift over 8.75 s | v at cmd +0.30 |
|---|---|---|
| gravity tilted, world-referenced obs (what enjoy_drive does) | −3.109 m | −0.116 |
| gravity tilted, **gravity-referenced obs** (faithful) | −0.587 m | +0.220 |

## What the real slope behaviour actually is

Measured with gravity tilt *and* the attitude re-expressed in the
gravity-aligned frame, which does reproduce a real incline:

| slope | run 2 drift (8.75 s) | run 2 uphill v at cmd +0.30 |
|---|---|---|
| 0° | +0.018 | +0.296 |
| 3° | −0.334 | — |
| 5° | −0.587 | +0.220 |
| 8° | −0.964 | +0.166 |
| 10° | −1.078 | −0.391 |

So slope **is** a genuine limitation — roughly −0.067 m/s of downhill creep at
5° — but far milder than the −0.414 m/s originally reported, and the robot
still drives uphill successfully up to about 8°. The earlier claim that it
"travels backwards while commanded forward" at 5° was an artifact of the
unfaithful implementation.

## Correct fix (not applied)

The ground geom cannot be rotated at runtime. Options:

1. **Tilt gravity + express attitude in the gravity frame**, in both the
   observation (`_decode_state`) and the reward. Proven working in
   `scratchpad/true_slope.py`. Cheap, no model recompile.
2. Bake the slope into the XML and compile one model per angle. Correct but
   needs a model rebuild per episode.
3. Move the floor into a non-static body so its pose becomes dynamic.

Option 1 is recommended. Whether the velocity-error integral is still needed
alongside it is now an open question rather than a settled one: run 2 already
drives uphill to 8° without it, and the integral cost a visible amount of
early training speed.

## State after this session

- `models/best_their/` — run 2 restored as the active checkpoint
- `models/best_their_RUN3_noop_slope/` — run 3, kept for the record
- `train_yahboom_3d.py`, `enjoy_drive.py` — reverted to their committed
  (run 2) state; the slope changes were never committed
- Training logs preserved for all three runs

## Lesson

Two checks would have caught this before the run:

1. Read the property back **from the simulator**, not from the value that was
   just written.
2. Include a null-control: a zero-torque rollout on the supposed slope. A
   robot that does not slide downhill is not on a slope.
