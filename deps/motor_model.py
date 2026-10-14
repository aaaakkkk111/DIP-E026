"""Lumped DC motor model for the DIP-E026 balance car.

Lets the policy output a PWM command instead of a torque, which removes the
need for a motor torque constant or winding resistance -- neither of which the
team has. Both unknowns are already folded into two quantities that WERE
measured at the wheel:

    tau_wheel = tau_stall * duty  -  kv * omega_wheel

    tau_stall = Kt * (V/R) * G * eta     <- Kt, R, gear ratio, efficiency
    kv        = Kt^2 * G^2 / R           <- same unknowns, same cancellation

Self-consistency check against the official no-load spec:
    tau_stall / kv = 0.5679 / 0.01620 = 35.06 rad/s
    stated no-load speed              = 34.90 rad/s      -> agree to 0.5%

All constants are from the team's measured parameter sheet (PARAMS.md,
2026-09-23) and the stock firmware (app_motor.c, app_motor.h).
"""
import numpy as np

# --- measured plant constants -------------------------------------------
TAU_STALL = 0.5679      # Nm at the wheel, stall
KV = 0.01620            # Nm/(rad/s), back-EMF; locked by the no-load spec
TAU_DRIVER_LIMIT = 0.40 # Nm - the AT8236 current limit clamps here, below the
                        # motor's own 0.57 stall capability
COULOMB = 0.00251       # Nm, saturating (tanh) dissipation, Coulomb-like

# --- firmware constants (app_motor.c / app_control.c) --------------------
PWM_PERIOD = 2880.0     # 25 kHz
PWM_LIMIT = 2800.0      # clamp in app_control.c
PWM_DEADBAND = 1300.0   # MOTOR_IGNORE_PULSE in this build; the parameter sheet
                        # quotes 1480 fwd / 1455 rev for the unit it measured.
                        # Per-build - match whatever you actually flash.

# Usable command span after the firmware's deadband compensation. The policy's
# action maps onto this, NOT onto the full PWM range.
PWM_USABLE = PWM_LIMIT - PWM_DEADBAND     # 1500 counts

COULOMB_EPS = 0.5       # rad/s, width of the tanh transition through zero


def action_to_pwm(action):
    """Policy action in [-1, 1] -> PWM counts, with the firmware's deadband
    compensation applied exactly as PWM_Ignore() does.

    The compensation is why the action is defined pre-compensation: the
    firmware adds the offset downstream, so the policy never has to learn the
    deadband. It also means the smallest non-zero command jumps straight to
    PWM_DEADBAND - the parameter sheet's "no gentle nudge" - which the policy
    DOES have to learn to work around, and can only do so if training models it.
    """
    a = np.clip(action, -1.0, 1.0)
    pwm = a * PWM_USABLE
    pwm = np.where(np.abs(a) > 1e-9, pwm + np.sign(pwm) * PWM_DEADBAND, 0.0)
    return np.clip(pwm, -PWM_LIMIT, PWM_LIMIT)


def pwm_to_torque(pwm, omega):
    """PWM counts + current wheel speed -> torque actually delivered (Nm).

    Back-EMF is what makes this speed-dependent: at standstill torque is
    proportional to duty, but as the wheel spins up an increasing share of the
    supply opposes back-EMF, until at 35 rad/s the motor can produce nothing.
    """
    duty = pwm / PWM_PERIOD
    tau = TAU_STALL * duty - KV * omega
    tau = np.clip(tau, -TAU_DRIVER_LIMIT, TAU_DRIVER_LIMIT)
    return tau - COULOMB * np.tanh(omega / COULOMB_EPS)


def action_to_torque(action, omega):
    """Convenience: policy action straight through to delivered torque."""
    return pwm_to_torque(action_to_pwm(action), omega)
