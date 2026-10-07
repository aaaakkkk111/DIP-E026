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

The dead zone, motor strength (battery 11.1-12.6 V), back-EMF and friction
are all estimates, not bench measurements, so training randomises them per
wheel and per direction. Re-fit MOTOR_DEAD_ZONE once the motors have been
swept on a stand (see the session log's open items).
"""
from dataclasses import dataclass

import numpy as np

# --- measured plant constants -------------------------------------------
TAU_STALL = 0.5679      # N.m at the wheel, stall, full duty
KV = 0.01620            # N.m/(rad/s), back-EMF; locked by the no-load spec
TAU_DRIVER_LIMIT = 0.40 # N.m - the AT8236 current limit clamps here, below the
                        # motor's own 0.57 stall capability

# --- firmware constants (app_motor.c / app_control.c) --------------------
PWM_PERIOD = 2880.0     # 25 kHz
PWM_LIMIT = 2800.0      # clamp in app_control.c
PWM_DEADBAND = 1300.0   # MOTOR_IGNORE_PULSE: the firmware's dead-band
                        # COMPENSATION, added after the controller. Not the
                        # motor's actual dead zone - see MOTOR_DEAD_ZONE.
PWM_USABLE = PWM_LIMIT - PWM_DEADBAND     # 1500 counts the action maps onto

# --- the motor's actual dead zone (counts) ------------------------------
# Nominal fits the measured free-spin speed exactly; the range brackets the
# parameter sheet (1455/1480), the fast-decay prediction (1440) and the
# firmware's own compensation (1300).
MOTOR_DEAD_ZONE = 1460.0
MOTOR_DEAD_ZONE_RANGE = (1350.0, 1600.0)
TAU_SCALE_RANGE = (0.85, 1.05)   # battery sag and unit-to-unit spread
KV_SCALE_RANGE = (0.9, 1.1)
COULOMB = 0.0025                 # N.m, friction once the wheel is turning
COULOMB_RANGE = (0.002, 0.02)
COULOMB_EPS = 0.5                # rad/s, width of the tanh through zero


@dataclass
class MotorParams:
    """Per-wheel motor parameters. dead_zone is [wheel, direction] with
    direction 0 = forward PWM, 1 = reverse PWM."""
    dead_zone: np.ndarray
    tau_scale: np.ndarray
    kv_scale: np.ndarray
    coulomb: np.ndarray


NOMINAL_MOTOR = MotorParams(dead_zone=np.full((2, 2), MOTOR_DEAD_ZONE),
                            tau_scale=np.ones(2), kv_scale=np.ones(2),
                            coulomb=np.full(2, COULOMB))


def sample_motor(rng):
    """One random draw of every uncertain motor parameter, for one episode."""
    return MotorParams(dead_zone=rng.uniform(*MOTOR_DEAD_ZONE_RANGE, size=(2, 2)),
                       tau_scale=rng.uniform(*TAU_SCALE_RANGE, size=2),
                       kv_scale=rng.uniform(*KV_SCALE_RANGE, size=2),
                       coulomb=rng.uniform(*COULOMB_RANGE, size=2))


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
    eff = np.clip((np.abs(pwm) - dead) / (PWM_PERIOD - dead), 0.0, 1.0)
    drive = TAU_STALL * params.tau_scale * eff - KV * params.kv_scale * s * omega
    tau = s * np.maximum(drive, 0.0)
    tau = np.clip(tau, -TAU_DRIVER_LIMIT, TAU_DRIVER_LIMIT)
    return tau - params.coulomb * np.tanh(omega / COULOMB_EPS)


def action_to_torque(action, omega, params=NOMINAL_MOTOR):
    """Convenience: policy action straight through to delivered torque."""
    return pwm_to_torque(action_to_pwm(action), omega, params)
