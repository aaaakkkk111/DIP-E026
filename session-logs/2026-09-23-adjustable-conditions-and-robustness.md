# Session log — 2026-09-23: adjustable conditions in enjoy_drive.py

Added friction, slope and payload as live-adjustable parameters so the trained
policy can be stress-tested against conditions it never saw, then measured how
far it actually generalises.

## Interface

```bash
python enjoy_drive.py --friction 0.4 --slope 5 --payload 0.15
```

and, while the viewer is running, typed into the terminal:

```
f 0.4   friction      s 5   slope (deg)      p 0.15   payload (kg)
r       reset         ?     show settings    q        quit
```

**Why terminal commands rather than keyboard shortcuts:** MuJoCo's viewer binds
every letter key to a rendering toggle (W wireframe, A auto-connect, S shadows,
D static-body visibility …) and fires our `key_callback` *in addition* to its
own, so any letter shortcut would silently flip render state as a side effect —
the cause of an earlier "why does the screen go black when I press D" bug. Key
handling lives in the compiled `_simulate` extension, so which keys are free
can't be determined statically. A background stdin thread sidesteps the problem
entirely and allows exact numeric values.

## Implementation notes

| knob | mechanism | gotcha |
|---|---|---|
| friction | `model.geom_friction[:, 0]` on floor **and** both wheels | MuJoCo combines a contact pair's friction with an element-wise **max** — lowering only the floor leaves the wheels' 0.90 in charge and changes nothing |
| slope | tilt gravity: `g = (−9.81·sinθ, 0, −9.81·cosθ)` | Rotating the ground plane instead would spawn the robot partially inside it. Tilting gravity is the same physics in the slope's frame; verified \|g\| stays exactly 9.81. Trade-off: floor still *looks* level in the viewer |
| payload | `model.body_mass[payload_body]` | The wrapper's `reset()` re-randomises payload mass, so it must be re-applied after every reset |

Verified all three reach the simulation. Payload moves the composite centre of
mass as expected:

| payload | total mass | subtree COM z |
|---|---|---|
| 0.0 kg | 1.700 | 0.1559 |
| 0.5 kg | 2.200 | 0.1864 |
| 2.0 kg | 3.700 | 0.2284 |

## Robustness results (commanded +0.30 m/s, 600 steps)

### Friction — policy is essentially insensitive

| friction | v achieved | vs target |
|---|---|---|
| 0.90 (as trained) | +0.284 | 95% |
| 0.50 | +0.283 | 94% |
| 0.25 | +0.284 | 95% |
| 0.10 | +0.287 | 96% |

Never falls, tracking unchanged at 9× below the trained value. Physically
reasonable: steady rolling at 0.3 m/s needs very little traction — friction
would bite during hard acceleration, braking, or on a slope, not in cruise.

### Payload — handled far beyond the trained range

Steady-state velocity is **identical** (+0.284) from 0 to 1.5 kg, which is not
a measurement artifact: a wheeled inverted pendulum cruises at pitch ≈ 0
regardless of mass, so terminal velocity is mass-independent. The effect is
entirely in the transient:

| payload | rise time to 90% | peak pitch | steady torque |
|---|---|---|---|
| 0.0 kg | 85 steps | 0.0564 | 0.0362 |
| 0.2 kg (trained max) | 79 steps | 0.0574 | 0.0363 |
| 0.5 kg | 73 steps | 0.0587 | 0.0364 |
| 1.0 kg | 64 steps | 0.0612 | 0.0365 |
| 1.5 kg | 55 steps | 0.0635 | 0.0367 |

Monotonic and well-behaved at 1.5 kg — **7.5× the trained maximum**, and 88% of
the robot's own 1.7 kg mass. A higher payload raises the COM, which lengthens
the pendulum and lets a given lean produce more horizontal force, hence the
*faster* rise time.

### Slope — the clear failure mode

| slope | v achieved (commanded +0.30) |
|---|---|
| level | +0.284 (95%) |
| +5° uphill | **−0.117** (−39%) |
| +10° uphill | **−0.455** (−152%) |

At +5° the robot **travels backwards** while being commanded forward, and at
+10° it slides downhill faster than it was ever asked to move forward. It
never falls — balance holds throughout — but it cannot climb.

**Root cause:** the policy has no integral action and was trained exclusively
on level ground, so it has no mechanism to reject a *sustained* disturbance.
It leans uphill (measured mean pitch +0.1997 at +8°) but that lean only buys
a bounded force, and gravity wins. This mirrors the velocity-integral finding
from the earlier session: an integral term was tried in the observation and
regressed, but that was during the phase when balance itself was still being
learned. On a slope it is exactly what's missing.

**Fix if slope capability is wanted:** randomise slope during training the same
way payload is curriculum-ramped. The knob now exists (`apply_slope`), so it is
a small change to `train_yahboom_3d.py`'s reset. Untested.

## Summary

The policy generalises well on the two axes it was *partially* exposed to
(payload was curriculum-trained; friction, while fixed, doesn't matter much
dynamically) and fails on the one axis that introduces a sustained force it
was never trained against. Slope is the top sim-to-real gap for hardware
deployment — a real floor is rarely perfectly level.
