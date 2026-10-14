"""Lumped DC gearmotor model for the DIP-E026 balance car, as the stock
firmware drives it: AT8236, one input PWM'd and the other held low, so the
bridge coasts during the off-time (fast decay).

The policy outputs PWM rather than torque because the team has no Kt or
winding resistance. Two quantities WERE measured at the wheel:

    TAU_STALL = 0.5679 N.m   stall torque at full duty
    KV        = 0.01620      N.m/(rad/s), back-EMF damping
    TAU_STALL / KV = 35.06 rad/s against a stated no-load speed of 34.90

Runs 1-7 used tau = TAU_STALL * duty - KV * omega: linear through zero PWM,
so the firmware's 1300-count dead-band compensation was itself modelled as
0.257 N.m of useful torque ("no gentle nudge"). The car disagrees
(session-logs/2026-10-06-hardware-mode28-diagnosis.md):

  - held in the air at PWM 1985 / 1705 the wheels spun at a mean 9.5 rad/s;
    the linear model predicts 22.3, a dead zone at ~1460 counts predicts 9.5
  - at PWM ~1470-1540 from rest the encoders read ~0 for 55-65 ms; the linear
    model predicts ~0.23 m/s by then
  - the parameter sheet itself measured the dead band at 1480 fwd / 1455 rev

A dead zone near 50 % duty is also what fast-decay PWM predicts: with the
bridge coasting during the off-time the mean winding voltage is ~(2d - 1) V,
zero at d = 0.5 = 1440 counts. Below the dead zone the winding current is
discontinuous and the motor delivers ~no torque; above it, torque rises
linearly to the measured stall torque at full duty. Because the current
cannot reverse within a PWM period, there is also no regenerative braking:
a wheel spinning faster than the drive asks for just coasts.

Run 12: measured on the car (2026-10-08, battery ~12.5 V, wheels in the air;
session-logs/2026-10-08-motor-step-test.md and -motor-reversal-test.md):
  - 52 one-level steps: dead zone 1395 (both wheels and directions within
    ±3), tau_scale 1.07 / 1.11 with KV as above, Coulomb 0.018-0.019 N.m,
    rotor inertia 1.45-1.51e-3 (time constant 92-97 ms, not 140)
  - 48 two-level steps (reversals, slow-downs): the driver limit is 0.75 N.m,
    not 0.4, and there is viscous friction of 0.00054 N.m.s/rad on top
  - 80 low-PWM steps (-lowpwm-stiction-and-the-e-line-artifact.md): stiction
    with hysteresis. From rest the wheel does not turn below ~1485, but a
    turning wheel keeps turning down to ~1400, so the dead zone depends on
    speed: 1485 at rest, falling linearly to 1400 by 0.96 rad/s, with the
    Coulomb friction refitted to 0.011 N.m (sim_tools/stiction_check.torque()
    is the reference; pwm_to_torque below is the same function)
  - the stock PID run through this motor matches the car standing still
Only ratios are pinned by free-spin tests (tau : back-EMF : friction :
inertia); the absolute scale rests on KV and the torque limit.

The ranges below are centred on those measurements: battery sag and unit
spread for the torque, and roughly ±30 % on the friction terms.
"""
from dataclasses import dataclass

import numpy as np

# --- measured plant constants -------------------------------------------
TAU_STALL = 0.5679      # N.m at the wheel, stall, full duty
KV = 0.01620            # N.m/(rad/s), back-EMF; locked by the no-load spec
TAU_DRIVER_LIMIT = 0.75 # N.m - fitted to the reversal test (was 0.40, an
                        # estimate). Must match real_robot.xml's ctrlrange.

# --- firmware constants (app_motor.c / app_control.c) --------------------
PWM_PERIOD = 2880.0     # 25 kHz
PWM_LIMIT = 2800.0      # clamp in app_control.c
PWM_DEADBAND = 1300.0   # MOTOR_IGNORE_PULSE: the firmware's dead-band
                        # COMPENSATION, added after the controller. Not the
                        # motor's actual dead zone - see MOTOR_DEAD_ZONE.
PWM_USABLE = PWM_LIMIT - PWM_DEADBAND     # 1500 counts the action maps onto

# --- measured motor (2026-10-08), and the training ranges around it -------
# Dead zone, counts. Turning: MOTOR_DEAD_ZONE. At rest: MOTOR_DEAD_ZONE_STATIC,
# blending into the turning value linearly up to STICTION_W0 (wheel rad/s).
# Was a single 1460, fitted to one free-spin log.
MOTOR_DEAD_ZONE = 1400.0
MOTOR_DEAD_ZONE_RANGE = (1360.0, 1440.0)
MOTOR_DEAD_ZONE_STATIC = 1485.0  # car: no motion from rest at 1420-1480, starts at 1500-1520
MOTOR_DEAD_ZONE_STATIC_RANGE = (1450.0, 1520.0)   # always above the turning range
STICTION_W0 = 0.96               # rad/s at the wheel
TAU_SCALE = 1.09                 # 1.07 (L) / 1.11 (R) at ~12.5 V
TAU_SCALE_RANGE = (0.95, 1.15)   # 0.95 ~ a battery at 11.1 V
KV_SCALE_RANGE = (0.9, 1.1)
COULOMB = 0.011                  # N.m, refitted with the stiction (0.019 without it)
COULOMB_RANGE = (0.007, 0.015)
COULOMB_EPS = 0.5                # rad/s, width of the tanh through zero
VISCOUS = 0.00054                # N.m.s/rad, -b * omega; new in run 12
VISCOUS_RANGE = (0.00035, 0.00075)


@dataclass
class MotorParams:
    """Per-wheel motor parameters. dead_zone (turning) and dead_zone_static (at
    rest) are [wheel, direction] with direction 0 = forward PWM, 1 = reverse
    PWM. dead_zone_static and viscous default to None (no stiction, no viscous
    term), so motors built the old way, as in the sim_tools fit scripts that
    add their own terms, behave exactly as before."""
    dead_zone: np.ndarray
    tau_scale: np.ndarray
    kv_scale: np.ndarray
    coulomb: np.ndarray
    viscous: np.ndarray = None
    dead_zone_static: np.ndarray = None


NOMINAL_MOTOR = MotorParams(dead_zone=np.full((2, 2), MOTOR_DEAD_ZONE),
                            tau_scale=np.full(2, TAU_SCALE), kv_scale=np.ones(2),
                            coulomb=np.full(2, COULOMB), viscous=np.full(2, VISCOUS),
                            dead_zone_static=np.full((2, 2), MOTOR_DEAD_ZONE_STATIC))


def sample_motor(rng):
    """One random draw of every uncertain motor parameter, for one episode."""
    return MotorParams(dead_zone=rng.uniform(*MOTOR_DEAD_ZONE_RANGE, size=(2, 2)),
                       tau_scale=rng.uniform(*TAU_SCALE_RANGE, size=2),
                       kv_scale=rng.uniform(*KV_SCALE_RANGE, size=2),
                       coulomb=rng.uniform(*COULOMB_RANGE, size=2),
                       viscous=rng.uniform(*VISCOUS_RANGE, size=2),
                       dead_zone_static=rng.uniform(*MOTOR_DEAD_ZONE_STATIC_RANGE, size=(2, 2)))


def action_to_pwm(action):
    """Policy action in [-1, 1] -> PWM counts, with the firmware's dead-band
    compensation applied exactly as PWM_Ignore() / rl_mode.c action_to_pwm()
    do. Unchanged since run 3: the firmware side of the mapping is right; it
    was the motor's response to the result that was mis-modelled."""
    a = np.clip(action, -1.0, 1.0)
    pwm = a * PWM_USABLE
    pwm = np.where(np.abs(a) > 1e-9, pwm + np.sign(pwm) * PWM_DEADBAND, 0.0)
    return np.clip(pwm, -PWM_LIMIT, PWM_LIMIT)


def pwm_to_torque(pwm, omega, params=NOMINAL_MOTOR):
    """PWM counts + current wheel speed (rad/s) -> torque at each wheel (N.m)."""
    pwm = np.asarray(pwm, dtype=np.float64)
    omega = np.asarray(omega, dtype=np.float64)
    s = np.sign(pwm)
    dead = np.where(s >= 0, params.dead_zone[:, 0], params.dead_zone[:, 1])
    if params.dead_zone_static is not None:  # stiction: higher from rest
        dead_rest = np.where(s >= 0, params.dead_zone_static[:, 0], params.dead_zone_static[:, 1])
        dead = dead + (dead_rest - dead) * np.clip(1.0 - np.abs(omega) / STICTION_W0, 0.0, 1.0)
    eff = np.clip((np.abs(pwm) - dead) / (PWM_PERIOD - dead), 0.0, 1.0)
    drive = TAU_STALL * params.tau_scale * eff - KV * params.kv_scale * s * omega
    tau = s * np.maximum(drive, 0.0)
    tau = np.clip(tau, -TAU_DRIVER_LIMIT, TAU_DRIVER_LIMIT)
    tau = tau - params.coulomb * np.tanh(omega / COULOMB_EPS)
    if params.viscous is not None:          # after the clip, as in the reversal fit
        tau = tau - params.viscous * omega
    return tau


def action_to_torque(action, omega, params=NOMINAL_MOTOR):
    """Convenience: policy action straight through to delivered torque."""
    return pwm_to_torque(action_to_pwm(action), omega, params)
