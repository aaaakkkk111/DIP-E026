"""Train on the REAL measured robot, with the policy commanding PWM.

Differences from the earlier train_yahboom_3d.py (now in archive/), and why
each one is forced:

1. Plant is `real_robot.xml`, built from the team's measured parameter sheet.
   `their_robot.xml` is wrong for this hardware by 3.5x in COM height and 12x
   in inertia about the axle, which is far outside the range the level-ground
   policy was verified to tolerate.

2. Control runs at 200 Hz, matching the stock firmware's MPU6050-interrupt
   loop, rather than 80 Hz.

3. The action is a **PWM command in [-1, 1]**, not a torque. The team has no
   motor torque constant or winding resistance, so a torque-commanding policy
   could not be converted to motor drive on hardware. Commanding PWM removes
   both unknowns: the lumped model in motor_model.py needs only stall torque
   and back-EMF, both of which were measured at the wheel.

   This also matches the stock RL design in the parameter sheet, whose action
   is the motor command before deadband compensation.

4. The actuator's real nonlinearity is modelled. After the firmware's deadband
   compensation the smallest non-zero command still delivers 0.257 Nm - 64% of
   the driver limit. The previous policy commanded a MEAN of 0.053 Nm, five
   times smaller than anything this hardware can produce, which is the single
   clearest reason it cannot transfer. Training against the real actuator is
   the only way to get a policy that works within that constraint.
"""
import argparse
import collections
import os
import random
from pathlib import Path
from typing import Callable

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium.envs.mujoco.mujoco_env import MujocoEnv
from gymnasium.spaces import Box
from gymnasium.wrappers import TimeLimit
from scipy.spatial.transform import Rotation as R
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback, EvalCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

from motor_model import NOMINAL_MOTOR, action_to_pwm, action_to_torque, sample_motor


def _opt_out_of_power_throttling():
    """Keep Windows from running this process in efficiency mode (EcoQoS).

    Windows 11 throttles processes it judges to be in the background, which
    on a hybrid CPU (the desktop's i7-12700KF) confines them to the
    efficiency cores. Run 12 dropped from ~6,500 to ~1,500 steps/s three
    minutes in, with every worker ~86 % idle, and went straight back to
    6,500-7,800 when this was switched off for its processes. Called at
    import, so it covers the SubprocVecEnv workers too (they import this
    module). Process-scoped; does nothing on other systems or on failure."""
    if os.name != "nt":
        return
    try:
        import ctypes
        import ctypes.wintypes as w

        class _State(ctypes.Structure):
            _fields_ = [("Version", w.ULONG), ("ControlMask", w.ULONG), ("StateMask", w.ULONG)]

        k32 = ctypes.WinDLL("kernel32")
        k32.GetCurrentProcess.restype = w.HANDLE
        k32.SetProcessInformation.argtypes = [w.HANDLE, ctypes.c_int, ctypes.c_void_p, w.DWORD]
        # ProcessPowerThrottling (4): execution-speed (0x1) and timer-resolution
        # (0x4) throttling controlled by us, and off.
        st = _State(1, 0x1 | 0x4, 0)
        k32.SetProcessInformation(k32.GetCurrentProcess(), 4, ctypes.byref(st), ctypes.sizeof(st))
    except Exception:
        pass


_opt_out_of_power_throttling()

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
XML_FILE_PATH = os.path.join(CURRENT_DIR, "real_robot.xml")

# 1 ms timestep x 5 = 200 Hz, matching the firmware's control loop.
FRAME_SKIP = 5
# 2000 steps at 200 Hz = 10 s per episode, comparable in wall-clock experience
# to the 1000-step/12.5 s episodes used on the previous plant.
MAX_EPISODE_STEPS = 2000

RESET_TILT_NOISE = 0.05
RESET_VEL_NOISE = 0.10

# Payload ceiling, lowered from the sheet's rated 4.0 kg.
#
# 4.0 kg is four times the robot's own 0.942 kg, and with uniform(0, cap)
# sampling it dominated the second half of training: the resulting policy held
# 2000/2000 steps at 1-4 kg but drifted at 0.317 m/s when UNLOADED and
# commanded to hold station. Payload also makes the task easier, not harder -
# it raises the COM and slows the dynamics, which suits the coarse bang-bang
# actuator - so training mostly-loaded optimised for the easy case.
#
# 1.0 kg still spans 0-106% of the robot's mass, which is a realistic operating
# range, while leaving the unloaded case well represented.
# Run 18: 0, not 1.0. Diagnostics (2026-10-21, 10M each): only the run without
# payload stood calm (4.6-10.2 deg/s at 2 deg, 5-13 reversals/s, against
# 19.5-30.1 and 20-29 for the control). Training for a loaded car taught a
# stiff controller that shakes through the slack when empty. The no-payload
# policy still stood 10 s with 0-4 kg on board in sim, calmer with more load.
# Run 19: 2.0. Run 18 (no payload, 30M) shook anyway (14.6-17.8 deg/s) and could
# not drive with 1 kg; the car has to carry about 2 kg, so the load range is back
# and run 19 changes the calm terms instead.
MAX_PAYLOAD_KG = 2.0
MAX_V_FORWARD = 0.3
MAX_V_TURN = 0.5

# Fall threshold, raised from the 0.4 rad (23 deg) inherited from the previous
# plant to match the stock firmware's actual cut-out of 40 deg (PARAMS.md
# section C). Terminating at 23 deg with a -10 penalty taught the policy to
# treat as fatal a region the real robot recovers from routinely, and denied it
# any large-angle recovery experience at all.
FALL_ANGLE_LIMIT = 0.70

# Dilated history taps, in control ticks at 200 Hz, beyond the current frame.
# (0, 2, 5, 11, 23, 47) spans 235 ms - the same span the team's own RL design
# uses, and for the same reason.
#
# A single frame cannot distinguish "oscillating about equilibrium" from
# "drifting away", and with a bang-bang actuator the policy has to dither -
# alternate sign rapidly to synthesise a small average torque - which is a
# decision that depends on recent history, not the present instant. This is the
# same partial-observability that made sustained-disturbance rejection
# impossible earlier.
HISTORY_TAPS = (2, 5, 11, 23, 47)
HISTORY_LEN = max(HISTORY_TAPS) + 1
# Signals worth remembering: attitude, its rate, and forward speed.
N_HISTORY_SIGNALS = 3

# Run 20: load awareness. Run 19 stood calm with 2 kg (5-7 deg/s, 3 N 18/18)
# but shook empty (20-30) - one fixed policy cannot be soft empty and firm
# loaded without knowing which it is. osc and lean are Mode 26 load_adapt.c's
# filters, exactly (one-pole, alpha = dt/(RC+dt) at 200 Hz):
#   osc  = RMS of the pitch gyro in ~8-16 Hz, deg/s (shake: gains too high)
#   lean = RMS of the pitch angle in ~0.3-2 Hz, deg  (sway: too soft)
# They replace the two dropped wheel-angle slots (obs[5], obs[6]), so the
# network keeps 34 inputs and the firmware needs no new weights.
LA_A_8HZ, LA_A_16HZ, LA_A_03HZ, LA_A_2HZ, LA_A_E = 0.200849, 0.334511, 0.009337, 0.059120, 0.016393
OSC_OBS_SCALE = 1.0 / 25.0        # load_adapt's "too high" threshold -> 1.0
LEAN_OBS_SCALE = 1.0 / 2.0
LA_OBS_CLIP = 4.0

# Run 21: a reward charge on the SHAKE band only - osc of the TRUE pitch gyro.
# Runs 18-20 all shook empty once payload and full shoves arrived (15M): a
# 20 deg/s shake cost ~0.18/step against +2 alive and -10 per fall, so the
# policy bought margin with stiffness. Raising the broadband pitch-rate charge
# (run 14) lost pushes, because recovery is large slow motion. osc (~8-16 Hz)
# barely sees recovery (<3 Hz) but reads the 7-10 Hz limit cycle in full:
# osc ~10 at run 20's empty 20 deg/s shake costs 1.0/step; calm (~2) 0.04.
OSC_PENALTY_WEIGHT = 1.0
OSC_PENALTY_REF_DPS = 10.0
OSC_PENALTY_MAX = 1.0               # below the +2 alive reward, so falling never pays


def _alpha(f_hz, dt=0.005):
    return dt / (1.0 / (2.0 * np.pi * f_hz) + dt)


assert all(abs(a - b) < 5e-6 for a, b in ((LA_A_8HZ, _alpha(8)), (LA_A_16HZ, _alpha(16)),
                                          (LA_A_03HZ, _alpha(0.3)), (LA_A_2HZ, _alpha(2)),
                                          (LA_A_E, 0.005 / 0.305)))


def load_adapt_step(st, gyro_dps, angle_deg):
    """One 5 ms tick of load_adapt.c. st = [m, b, E, q, l, EL], updated in place."""
    st[0] += LA_A_8HZ * (gyro_dps - st[0])
    st[1] += LA_A_16HZ * ((gyro_dps - st[0]) - st[1])
    st[2] += LA_A_E * (st[1] * st[1] - st[2])
    st[3] += LA_A_03HZ * (angle_deg - st[3])
    st[4] += LA_A_2HZ * ((angle_deg - st[3]) - st[4])
    st[5] += LA_A_E * (st[4] * st[4] - st[5])
    return float(np.sqrt(st[2])), float(np.sqrt(st[5]))

# Deterministic payload per eval episode, paired with EVAL_COMMANDS.
#
# The parent wrapper samples payload randomly at reset, which is fine while the
# range is small but made the eval score a noisy mixture once the ceiling went
# to 4 kg - so "new best mean reward" reflected which payloads happened to be
# drawn rather than whether the policy improved. Same defect that was already
# fixed for the commands and for slope; it was missed here when the range grew.
# Gearbox-reflected rotor inertia at the wheel, J_rotor * G^2 (see real_robot.xml).
# It dominates the wheel dynamics (100x the wheel's own inertia).
#
# Run 12: measured by the step test on the car (2026-10-08), 1.45e-3 / 1.51e-3
# kg m^2, a 92-97 ms time constant. It was estimated at 2.25e-3 and randomised
# over 9e-4 .. 4.5e-3. The range now brackets the measurement, plus the
# fit's flat valley toward a lighter, weaker motor (about 1.0e-3).
ARMATURE_NOMINAL = 1.5e-3
ARMATURE_RANGE = (1.1e-3, 2.0e-3)

# Penalty on how fast the command changes, per step: -w * sum((a_t - a_{t-1})^2).
#
# The deadband forces the policy to dither - alternate sign to synthesise a mean
# torque below the 0.257 Nm floor - and that is legitimate. What is not is doing
# it at 100 Hz: the measured policy reversed both motors on ~50% of control
# ticks, and with the inertia fixed it went to ~95%. The mechanics filter that
# out, so the chassis no longer shakes, but the H-bridge still has to execute
# every reversal, and a reversal swings the winding voltage by 2x supply.
#
# Weight: a full reversal at the observed command amplitude (~0.28) costs about
# 0.13/step. Run 3 averaged ~0.92 reward per step, so this is ~14% of the
# achievable reward - enough to break ties toward smoothness, not enough to
# restructure the objective. 0.5 was tried first and charged 1.0 per full
# reversal, which is more than half the per-step budget; at that weight the
# policy would likely stop dithering altogether, and with a 0.257 Nm torque
# floor it NEEDS to dither to produce small torques. This is the knob to turn
# if it still buzzes (raise) or goes sluggish (lower).
ACTION_RATE_WEIGHT = 0.2

# Run 16: a charge per motor REVERSAL (sign change of a wheel's command), on
# top of the rate term. On the car (2026-10-20) runs 13 and 15 shake at ~7 Hz,
# reversing 10-17 times a second: each reversal crosses the gear slack, the
# rotor slams into the far side, the jolt shows on the gyro and the policy
# reverses again. The stock PID rarely reverses and stays on one side of the
# gap. The rate term cannot see this: a 0.1 -> -0.1 flip costs almost nothing,
# but through the 1300-count compensation it is a full 2600-count swing.
# 0.5 per wheel per reversal: run 15's ~28/s per wheel in sim costs ~0.14/step,
# the PID's 2-6/s ~0.02.
# Run 17: 0.3. At 0.5 (run 16) the policy stood calm at 10M (2.6-5 deg/s,
# 1-4 reversals/s) but lost it once full pushes and turns came in, ending at
# run 15's wobble and surviving no 3 N push (session-logs/2026-10-21-run16-desktop.md).
# Run 19: 0, replaced by PWM_STEP_WEIGHT below.
REVERSAL_WEIGHT = 0.0

# Run 19: charge the change in the PWM the motors actually get (after the
# 1300-count compensation, so a small flip costs what it does to the gears),
# scaled by upright_gate so it applies only near upright. Knocked over, the
# policy may move as hard as it needs. A full small-amplitude flip
# (2600 counts, ~0.86 squared) costs ~0.26, like run 17's reversal charge;
# the PID's ~13 counts/tick costs nothing.
PWM_STEP_WEIGHT = 0.3
UPRIGHT_GATE_RAD = 0.15           # ~8.6 deg: full charge at 0, none beyond


def upright_gate(pitch):
    return max(0.0, 1.0 - abs(pitch) / UPRIGHT_GATE_RAD)


assert upright_gate(0.0) == 1.0 and upright_gate(0.075) == 0.5 and upright_gate(-0.2) == 0.0

# --- station keeping -----------------------------------------------------
#
# Run 5 drifted 0.408 m per 10 s while commanded to hold station (run 3: 0.073 m),
# creeping at a steady ~0.039 m/s in a seed-dependent direction with the chassis
# level. Cause: the reward graded only SPEED, never PLACE, so "still" and
# "creeping at 4 cm/s" scored within 0.08 of each other per step, and velocity
# error integrates into unbounded position error. Run 3 looked better only
# because it could barely move at all (28% of commanded speed); better driving
# and worse standing were two faces of the same gain.
#
# A position penalty alone would NOT have worked: _get_conditioned_obs deletes
# world x and y (raw_obs[2:]), so the policy cannot see where it is, and
# penalising an error it cannot perceive just injects noise. Hence the paired
# change - the same quantity is added to BOTH the reward and the observation.
#
# The quantity is a LEAKY integral of forward velocity error:
#
#     e <- decay * e + (v_actual - v_target) * dt
#
# which is position error along the commanded trajectory. Leaky rather than pure
# for two reasons: it is bounded by construction (steady error E settles at
# E * tau instead of growing all episode), and it self-forgets within ~tau when
# the command changes, so the robot is never asked to "catch up" distance it lost
# under a previous command.
#
# tau = 2.0 s, so a 0.039 m/s creep settles at 0.078 m and costs 0.195 reward
# per step against run 5's ~1.74/step average - about 11%, deliberately the same
# order as the effort and action-rate terms rather than dominating them.
POSITION_TAU_S = 2.0
POSITION_WEIGHT = 2.5
# Scale for the observation only, to land the input in the same O(1) range as
# the rest of the vector. Does not affect the reward.
POSITION_OBS_SCALE = 10.0

# --- heading keeping -----------------------------------------------------
#
# Run 6 fixed position drift (0.408 -> 0.012 m per 10 s) but left the TURN axis
# with exactly the defect the forward axis had: the reward grades yaw RATE and
# never yaw ANGLE, so heading error integrates freely. Measured on run 6 at idle
# over 10 minutes, yaw wandered to -10.2 deg by 300 s and back to -6.1 deg by
# 600 s - bounded, but far more visible than the 9 cm of translation. The same
# omission is why run 6 overshoots turns (106% at +0.50, 113% combined) while
# forward, which got the integral, tracks 96-104%.
#
# Symmetric fix: leaky integral of yaw RATE error, which is heading error.
#
# Why leaky rather than absolute heading, and this matters for hardware: the
# MPU6050 has no magnetometer, so absolute yaw drifts without bound on the real
# robot. A policy that depended on absolute heading would work in sim and fail
# on hardware. A leaky integral only ever integrates the last few tau of gyro-z,
# which the MPU6050 measures directly and well.
#
# tau = 30 s, NOT the forward axis's 2 s. The two axes want different things:
#   - Forward used a short tau deliberately, so the robot is never asked to
#     chase distance lost under a previous command.
#   - Heading is the opposite: integral of (w - w_cmd) IS "actual heading minus
#     commanded heading", so driving it to zero means ending up pointing where
#     you were told to point. That is heading control, and it is what you want.
#     A 2 s tau cannot see a wander that develops over minutes.
#
# Clipped because the dynamic range otherwise spans two orders of magnitude: a
# slow idle wander produces ~0.018 rad while a 10% error on a commanded turn
# produces ~1.5 rad. Without the clip one linear weight cannot serve both, and
# the tracking case would dominate the whole reward. Clipped at 0.2 rad (11.5
# deg) the penalty tops out at 0.6/step, while the idle wander still costs
# ~0.054/step - small but no longer invisible.
YAW_TAU_S = 30.0
YAW_WEIGHT = 3.0
YAW_CLIP_RAD = 0.2
YAW_OBS_SCALE = 1.0 / YAW_CLIP_RAD   # maps the clipped range onto [-1, 1]

# --- run 11: wobble -------------------------------------------------------
#
# Runs 8-10 wobble at 5-8 Hz on the car (run 9: pitch-rate sd 68 deg/s) and,
# with gear slack, in simulation (run 10 at 5 deg / 15 ms: ~30 deg/s). Nothing
# in the reward priced it. A fast wobble keeps the AVERAGE pitch, speed and
# position nearer zero than a slow sway, so the reward preferred it: standing
# at 5 deg / 15 ms, run 10 scored 1.77 per step against 1.64 for a plain PD
# controller that holds the same car at 9 deg/s rms. And the action-rate term
# barely sees it (-0.002/step for run 10).
#
# Penalty on the true chassis pitch rate, squared so it falls mostly on the
# wobble rather than on the brief pitch rates of leaning into a manoeuvre.
# At w = 1.5 the calm PD wins (1.60 vs 1.36); without slack the policies are
# calm (1.7 deg/s) and pay < 0.002/step; driving still pays (run 10 driving
# 1.37/step vs > 1.7/step lost by not driving). Capped so that a car that
# cannot avoid wobbling never gains by falling: uncapped, 64 deg/s would cost
# 1.9/step against the +2 alive bonus; at the cap a heavily wobbling car still
# nets ~+0.7/step. The cap starts at 47 deg/s; at 0.75 (41 deg/s) it clipped
# run 10's wobble peaks and cut its penalty from 0.42 to 0.30/step.
#
# Run 12: OFF. The car's "wobble" that motivated it (57-95 deg/s) was the
# hand push at the end of each run, read from the E lines
# (session-logs/2026-10-08-lowpwm-stiction-and-the-e-line-artifact.md); run
# 11's real balancing wobble is 12-22 deg/s, which the simulator with the
# measured motor already reproduces. On that motor (2 deg, 10 ms), run 11
# against run 10 bought 1.3 deg/s of calm (13.4 vs 14.7) for a quarter of the
# push tolerance: it falls from a 4 N x 0.1 s push that run 10 survives,
# because recovering from a push needs exactly the fast pitch rates this term
# charges. Set the weight back above 0 to re-enable it.
#
# Run 13: back ON at a third of run 11's weight, now that training has
# pushes. Run 12 (pushes, no penalty) matched the stock PID's push tolerance
# but stood at 25-35 deg/s on the 2 deg car, twice runs 10-11. At 0.5 a
# 30 deg/s wobble costs ~0.14/step and a 15 deg/s one ~0.035, a pull toward
# calm. Run 11's penalty cost push tolerance only because nothing in its
# training ever pushed it; here a policy that stops recovering falls.
#
# Run 14: back to run 11's 1.5, trained FROM SCRATCH with pushes. On the car
# (2026-10-14) run 13 stood at 26-36 deg/s mid-run, as simulated (23-37), and
# the 0.5 fine-tune had not calmed it. The stock PID on the same car stands at
# 7.6 deg/s and recovers from hard hand pushes, so calm and push-tolerant is
# possible on this hardware; run 11 showed 1.5 buys calm, run 13 that pushes
# keep the push tolerance.
PITCH_RATE_WEIGHT = 1.5           # per (rad/s)^2; run 11 1.5, run 12 0, run 13 0.5
PITCH_RATE_PENALTY_MAX = 1.0      # per step

# --- run 12: pushes -----------------------------------------------------
#
# No run so far saw a disturbance beyond the small tilt at reset. On the
# measured motor (2 deg, 10 ms), pushed 10 cm above the axle for 0.1 s, the
# stock PID survives 3 N both ways, run 10 2 N and run 11 1 N
# (session-logs/2026-10-08-run12-*). Training episodes now get random shoves
# along the car's own forward axis, applied as pid_sim.sim_push applies them
# (a force at the chassis COM plus its torque about it). Training only:
# evaluation, and simlib's test cars, stay push-free so their scores compare.
# The size ramps in with the velocity curriculum (CurriculumCallback), so
# balance comes first.
# Run 15: 3 N, not 4. The stock PID survives 3 N x 0.1 s but never 4 N, and
# 10-23 % of run 14's pushes were above its limit; surviving those needs a
# twitchy controller, which run 14's wobble penalty then fought.
PUSH_MAX_N = 3.0                  # uniform 0..max, either direction
# Run 15: no wobble penalty during a push and for this long after it, so
# recovering hard is not charged; calm is graded only when left alone.
PENALTY_PAUSE_S = 1.0
PUSH_DURATION_S = (0.05, 0.15)
PUSH_HEIGHT_M = (0.05, 0.15)      # above the axle
PUSH_INTERVAL_S = (1.5, 4.0)      # from the end of one push to the next

# --- run 8: train on what the firmware actually observes -----------------
#
# Runs 1-7 fed the policy simulator ground truth: exact chassis velocity, the
# full quaternion including yaw, exact wheel angles and speeds, exact height.
# The firmware can supply none of that. It builds every input from an
# MPU6050 and 1320-count encoders (firmware/policy.c finish_obs, the team's
# odom.c), and on the car the network then met inputs it had never seen
# (session-logs/2026-10-06-hardware-mode28-diagnosis.md). Every input below
# is now computed the way the firmware computes it, from emulated sensors.
#
# Constants the firmware substitutes for things it cannot measure.
SENSOR_HEIGHT = 0.0334            # POLICY_NOMINAL_HEIGHT, chassis origin at the axle
# Encoders: 4x quadrature x 11 ppr x 1:30. Wheel speed is differenced over 4
# ticks exactly as odom.c does (1 count/tick is already 0.95 rad/s).
ENC_RAD_PER_COUNT = 2.0 * np.pi / 1320.0
ENC_VEL_WINDOW = 4
WHEEL_RADIUS = 0.0335
# Input sanitising, mirrored in the firmware. On the car one corrupted encoder
# sample (a stack overflow) produced a ~1e6 m/s v_forward; the unclamped
# station-keeping integral latched it and pinned the network in saturation
# for ~15 s. Physical values never come near these clamps.
V_FORWARD_CLIP = 2.0              # m/s
WHEEL_VEL_CLIP = 40.0             # rad/s, above the 35 rad/s no-load speed
POS_ERR_CLIP_M = 0.5              # m, the station-keeping integral
# Wheel angles (obs 5, 6) are no longer fed to the policy - always 0, here and
# in the firmware. Absolute wheel angle is physically meaningless, yet run 7
# learned to depend on it: the team's SIL found 6/10 long drives fell once the
# angle left the range a 10 s episode reaches, and the firmware had to leak
# it, which training never did. Kept as zeroed slots so the 34-input layout,
# the exporter and the fixed-point port do not change.
#
# Sense-to-act latency. The firmware computes the action from an IMU sample
# taken at the start of the 5 ms tick and writes the PWM 4.6 ms later (worst
# case measured on the car), so the action lands ~1 tick late; the MPU6050's
# own low-pass filter adds up to another. Run 7 assumed zero.
#
# Run 9: run 8 on the car (2026-10-07) balanced for 51 s but oscillated at
# 6.9 Hz, pitch +/-2.5 deg, PWM swinging close to +/-2800 every ~70 ms. In the
# simulator run 8 is calm at 5-15 ms and reproduces that oscillation only with
# 20-35 ms of delay (6.9 Hz and the same 7 % PWM sign-flip rate at 30 ms).
# The firmware accounts for ~5 ms and the MPU6050 DLPF (98 Hz under the DMP
# set-up) ~3 ms; the rest is presumably gearbox slack and motor response, not
# modelled. Run 9 trained over 5-40 ms and picked the best checkpoint at 30 ms.
#
# Run 10: gearbox slack. Run 9 is calm in simulation at every delay up to
# 40 ms, yet on the car it still wobbled at 7.2 Hz (tilt-rate sd 68 deg/s
# against ~2 in simulation): delay alone cannot produce that. Fitting run 8's
# AND run 9's car behaviour together over delay x slack
# (sim_tools/fit_delay_slack.py, session-logs/2026-10-08-*) puts the slack at
# ~4-6 deg at the wheel with 10-20 ms of delay. The fit under-predicts run 9's
# wobble by ~2x, so the slack range goes past it. Drawn per wheel, per
# training episode; the encoders read the motor side of the slack.
#
# Run 12: both pinned down by measurement (2026-10-08). The slack is ~2 deg
# (hand measurement 1-2 mm at the tyre edge, i.e. 2-3.5 deg). The stock PID,
# run through the measured motor, matches the car standing still only at
# ~2 deg and <= 15 ms; at 3-4 deg or 20 ms it falls into a 10 Hz limit cycle
# the car does not show. The 4-6 deg / 10-20 ms of the policy-only fit was
# compensating for the mis-modelled motor.
ACTION_LATENCY_TICKS = (1, 3)     # 5-15 ms, per training episode
EVAL_LATENCY_TICKS = 2            # 10 ms; fixed so evals compare
# Run 16: up to 6 (runs 12-15: 4). The 2026-10-20 car fits ~4 deg: the stock
# PID at 11.7 deg/s and run 13 at ~50 match the simulator at 4 deg, 11.8 V.
GEAR_SLACK_DEG = (0.0, 6.0)       # total free play, motor to wheel, per wheel
EVAL_GEAR_SLACK_DEG = 2.0         # the measured car value
# Joint layout of real_robot.xml (rotors after the wheels).
WHEEL_QVEL = slice(6, 8)
ROTOR_QPOS = slice(9, 11)
ROTOR_QVEL = slice(8, 10)
# IMU errors, per training episode. Pitch zero is set by hand at the balance
# point; gyro bias is what remains after 'cal' plus thermal drift. Run 12:
# hand calibrations on 2026-10-08 scattered over 1.87-5.06 deg, about ±2 deg
# around a balance point near +1-2 deg (the stock PID's), so the pitch-zero
# error range goes from ±2 to ±4 deg.
PITCH_OFFSET_RAD = np.radians(4.0)
ROLL_OFFSET_RAD = np.radians(2.0)
GYRO_BIAS_RADS = 0.003            # per axis, +/-
GYRO_NOISE_RADS = 0.01            # per tick, includes motor vibration
ATT_NOISE_RAD = 0.002             # complementary-filter output noise

EVAL_PAYLOADS = [
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.25, 0.25, 0.25, 0.25,
    0.5, 0.5, 0.5, 0.5,
    0.75, 0.75, 0.75,
    1.0, 1.0, 1.0, 1.0,
]


# --- shared task: commands, base reward, curriculum ------------------------
#
# Moved here from train_yahboom_3d.py (now in archive/) so this file stands on
# its own; the code is unchanged. Measurements quoted in the comments of this
# section were taken on that earlier plant (their_robot.xml, 80 Hz) unless they
# say otherwise. They explain why the reward has its shape; run 8 trains with
# exactly this reward plus the terms PWMCommandWrapper adds.

SHAPING_GAMMA = 0.99  # matches PPO's default discount factor

# Yaw-rate low-pass for the turn-error term. ~0.1s time constant at 80Hz.
#
# WHY: this plant can track a commanded yaw rate on AVERAGE but not moment to
# moment. Measured against a 0.50 rad/s command: achievable mean yaw is 0.485
# (97%), while the best achievable INSTANTANEOUS error is 0.277 - 55% of the
# command. Turning necessarily involves a yaw limit cycle here.
#
# Charging instantaneous error therefore prices in an oscillation the robot
# cannot avoid, and made turning literally not worth doing: a 5M-step run
# learned to ignore turn commands entirely. Under filtered scoring turning
# wins. A small instantaneous term is kept alongside it (TURN_INSTANT_WEIGHT)
# so the policy is not free to satisfy the average by oscillating wildly.
TURN_LPF = 0.9
TURN_INSTANT_WEIGHT = 0.2

# Filtered-turn-error weight, sized against the forward axis (2.0), which is the
# signal strength that demonstrably did get learned. More risks the policy
# trading away balance to chase yaw.
TURN_WEIGHT = 2.0

# Eval commands, as fractions of (max_v_forward, max_v_turn), one per eval
# episode and HELD for the whole episode. Fixed rather than sampled so the
# score stays comparable between evaluations; starts at (0,0) so standing is
# still measured. Paired with EVAL_PAYLOADS above.
EVAL_COMMANDS = [
    (0.0, 0.0),
    (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
    (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0),
    (0.5, 0.0), (-0.5, 0.0), (0.0, 0.5), (0.0, -0.5),
    (0.5, 0.5), (-0.5, 0.5), (0.5, -0.5), (-0.5, -0.5),
    (0.75, 0.25), (-0.75, -0.25), (0.25, 0.75),
]


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:
        return progress_remaining * initial_value
    return func


class VelocityCommandWrapper(gym.Wrapper):
    """Samples forward/turn commands, computes the base reward on simulator
    ground truth, and ends the episode on a fall. PWMCommandWrapper below
    builds on it."""

    def __init__(self, env: gym.Env, is_eval=False, max_payload_kg=0.2, max_v_forward=0.3,
                 max_v_turn=0.5, fall_angle_limit=0.4, settle_steps=10):
        super().__init__(env)
        self.is_eval = is_eval
        self.max_payload_kg = max_payload_kg
        self.max_v_forward = max_v_forward
        self.max_v_turn = max_v_turn
        self.fall_angle_limit = fall_angle_limit
        # Grace period (in control steps) after reset before the fall check kicks in,
        # so reset noise/transients can't instantly end the episode.
        self.settle_steps = settle_steps
        self.target_v_forward = 0.0
        self.target_v_turn = 0.0
        self.command_timer = 0
        self.steps_since_reset = 0
        self.eval_episode = 0
        self.turn_filt = 0.0
        self.prev_potential = 0.0
        self.payload_body_id = mujoco.mj_name2id(self.unwrapped.model, mujoco.mjtObj.mjOBJ_BODY, "payload_body")

        # Curriculum caps for training envs, ramped up from 0 by CurriculumCallback.
        # Eval always uses the full max_* values so the reported metric reflects the real task.
        self.curriculum_payload_kg = 0.0 if not is_eval else max_payload_kg
        self.curriculum_v_forward = 0.0 if not is_eval else max_v_forward
        self.curriculum_v_turn = 0.0 if not is_eval else max_v_turn

        original_obs_space = self.env.observation_space.shape[0]
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(original_obs_space,), dtype=np.float32)

        # Torque limit taken straight from the model's ctrlrange so it can never
        # silently disagree with the plant. PWMCommandWrapper replaces the
        # action space with PWM in [-1, 1]; tau_max still normalises the effort
        # term of the reward.
        self.tau_max = float(self.unwrapped.model.actuator_ctrlrange[:, 1].max())
        self.action_space = gym.spaces.Box(low=-self.tau_max, high=self.tau_max,
                                           shape=(2,), dtype=np.float32)

    def set_curriculum(self, payload_kg, v_forward, v_turn):
        self.curriculum_payload_kg = payload_kg
        self.curriculum_v_forward = v_forward
        self.curriculum_v_turn = v_turn

    def set_push_curriculum(self, frac):
        """Pushes are PWMCommandWrapper's; nothing to do here."""

    def _sample_command_timer(self):
        # Mostly short segments (reactive command changes), but sometimes a
        # long steady-state hold up to a full episode, so the policy practises
        # sustaining one command. Uses this module's MAX_EPISODE_STEPS (2000):
        # when this class lived in train_yahboom_3d.py it read that module's
        # 1000 and needed an override here.
        if random.random() < 0.3:
            return random.randint(400, MAX_EPISODE_STEPS)
        return random.randint(100, 300)

    def _get_conditioned_obs(self, raw_obs):
        # Replaced by PWMCommandWrapper's firmware-exact version.
        blindfolded_obs = raw_obs[2:].copy()
        _, _, local_vel, _ = self._decode_state(raw_obs)
        blindfolded_obs[7:10] = local_vel
        return np.concatenate([blindfolded_obs, [self.target_v_forward, self.target_v_turn]]).astype(np.float32)

    def _decode_state(self, raw_obs):
        qw, qx, qy, qz = raw_obs[3:7]
        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)
        QVEL_START_INDEX = 9
        local_vel = rot.inv().apply(np.array(raw_obs[QVEL_START_INDEX:QVEL_START_INDEX + 3]))
        actual_v_turn = raw_obs[QVEL_START_INDEX + 5]
        return roll, pitch, local_vel, actual_v_turn

    def _potential(self, pitch, roll, actual_v_forward, actual_v_lateral, filtered_v_turn):
        """Potential-based shaping (Ng et al. 1999): F(s,s')=gamma*Phi(s')-Phi(s)
        added to the reward is policy-invariant, so it can only make the right
        behavior easier to find, not change what the optimal policy is.

        Caveat since the turn axis switched to a filtered yaw rate: the filter
        is internal state that is not part of the observation, so Phi is
        strictly a function of an augmented state rather than the MDP state the
        policy sees, and the policy-invariance guarantee no longer holds
        rigorously. It is kept consistent with the main reward term
        deliberately - having the shaping grade instantaneous yaw while the
        reward grades filtered yaw would have the two terms pull against each
        other.

        Each velocity axis blends its own "hold still" term against its own
        "track the target" term, weighted by how much *that axis* is being
        commanded. A single shared blend weight let a large turn command
        dominate the blend and starve the forward axis of tracking signal -
        measured result was a policy that turned well but never translated.
        """
        w_forward = min(1.0, abs(self.target_v_forward) / self.max_v_forward) if self.max_v_forward > 0 else 0.0
        w_turn = min(1.0, abs(self.target_v_turn) / self.max_v_turn) if self.max_v_turn > 0 else 0.0

        forward_error = actual_v_forward - self.target_v_forward
        turn_error = filtered_v_turn - self.target_v_turn
        forward_weight = (1.0 - w_forward) * 0.5 + w_forward * 1.0
        turn_weight = (1.0 - w_turn) * 0.1 + w_turn * 1.0

        # Attitude is a shared property of the body rather than a per-axis
        # one, and holding it perfectly level fights the lean that
        # accelerating requires - so it stays scaled down while any command
        # is active.
        posture_weight = 1.0 - max(w_forward, w_turn)

        return (
            -posture_weight * (pitch ** 2 + roll ** 2)
            - forward_weight * forward_error ** 2
            - turn_weight * turn_error ** 2
            - 0.5 * actual_v_lateral ** 2  # never commanded, always unwanted
        )

    def reset(self, seed=None, options=None):
        raw_obs, info = self.env.reset(seed=seed, options=options)
        self.unwrapped.model.body_mass[self.payload_body_id] = random.uniform(0.0, self.curriculum_payload_kg)

        if self.is_eval:
            f_frac, t_frac = EVAL_COMMANDS[self.eval_episode % len(EVAL_COMMANDS)]
            self.eval_episode += 1
            self.target_v_forward = f_frac * self.max_v_forward
            self.target_v_turn = t_frac * self.max_v_turn
        else:
            self.target_v_forward = random.uniform(-self.curriculum_v_forward, self.curriculum_v_forward)
            self.target_v_turn = random.uniform(-self.curriculum_v_turn, self.curriculum_v_turn)
        self.command_timer = self._sample_command_timer()
        self.steps_since_reset = 0

        roll, pitch, local_vel, actual_v_turn = self._decode_state(raw_obs)
        # Seed the filter with the actual yaw rate rather than 0, so the first
        # steps of an episode are not graded against a fictitious history.
        self.turn_filt = float(actual_v_turn)
        self.prev_potential = self._potential(pitch, roll, local_vel[0], local_vel[1], self.turn_filt)
        return self._get_conditioned_obs(raw_obs), info

    def step(self, action):
        self.steps_since_reset += 1
        if not self.is_eval:
            self.command_timer -= 1
            if self.command_timer <= 0:
                self.target_v_forward = random.uniform(-self.curriculum_v_forward, self.curriculum_v_forward)
                self.target_v_turn = random.uniform(-self.curriculum_v_turn, self.curriculum_v_turn)
                self.command_timer = self._sample_command_timer()

        raw_obs, _, _, _, info = self.env.step(action)
        roll, pitch, local_vel, actual_v_turn = self._decode_state(raw_obs)
        actual_v_forward = local_vel[0]

        # Tracking weights sized so that the tilt needed just to accelerate does
        # not cost more than failing to track; at heavier pitch weights the
        # optimal policy was "ignore commands, minimise tilt". The effort term
        # is a fraction-of-available-torque cost, capped at 0.2 (10% of the
        # alive bonus) when both motors saturate.
        effort = np.sum(np.square(action / self.tau_max))
        total_reward = 2.0 - (abs(pitch) * 3.0) - (effort * 0.1)
        total_reward -= abs(actual_v_forward - self.target_v_forward) * 2.0

        # Turn error is graded on a low-passed yaw rate (see TURN_LPF), plus a
        # small instantaneous term so the average cannot be satisfied by
        # oscillating wildly.
        self.turn_filt = TURN_LPF * self.turn_filt + (1.0 - TURN_LPF) * actual_v_turn
        total_reward -= abs(self.turn_filt - self.target_v_turn) * TURN_WEIGHT
        total_reward -= abs(actual_v_turn - self.target_v_turn) * TURN_INSTANT_WEIGHT

        potential = self._potential(pitch, roll, actual_v_forward, local_vel[1], self.turn_filt)
        total_reward += SHAPING_GAMMA * potential - self.prev_potential
        self.prev_potential = potential

        terminated = False
        if self.steps_since_reset > self.settle_steps and (abs(pitch) > self.fall_angle_limit or abs(roll) > self.fall_angle_limit):
            terminated = True
            total_reward = -10.0

        return self._get_conditioned_obs(raw_obs), total_reward, terminated, False, info


class CurriculumCallback(BaseCallback):
    """Staged curriculum: stand, then drive forward/back and turn together,
    then carry payload - instead of ramping everything at once.

    Ramping everything together forces balance and velocity-tracking to
    compete for gradient signal before balance is solid, which produces a
    fragile policy. Forward and turn ramp together because staging them
    (forward first) taught an explicit yaw suppressor during the forward-only
    phase, which the turn phase then had to undo with exploration already
    decayed.
    """

    def __init__(self, total_timesteps, max_payload_kg, max_v_forward, max_v_turn,
                 stand_phase_end=0.15, velocity_phase_end=0.5,
                 cmd_cap=1.0, push_cap=1.0, payload_cap=1.0, verbose=0):
        super().__init__(verbose)
        # Diagnostic caps (default 1 = off): hold one part of the curriculum at
        # a fraction of full strength to see which part brings the shake back.
        self.cmd_cap, self.push_cap, self.payload_cap = cmd_cap, push_cap, payload_cap
        self.total_timesteps = total_timesteps
        self.max_payload_kg = max_payload_kg
        self.max_v_forward = max_v_forward
        self.max_v_turn = max_v_turn
        self.stand_phase_end = stand_phase_end
        self.velocity_phase_end = velocity_phase_end

    def _on_rollout_start(self) -> None:
        progress = self.num_timesteps / self.total_timesteps

        if progress < self.stand_phase_end:
            forward_frac, turn_frac, payload_frac = 0.0, 0.0, 0.0
        elif progress < self.velocity_phase_end:
            frac = (progress - self.stand_phase_end) / (self.velocity_phase_end - self.stand_phase_end)
            forward_frac, turn_frac = frac, frac
            payload_frac = 0.0
        else:
            forward_frac, turn_frac = 1.0, 1.0
            payload_frac = min(1.0, (progress - self.velocity_phase_end) / (1.0 - self.velocity_phase_end))

        self.training_env.env_method(
            "set_curriculum",
            self.max_payload_kg * min(payload_frac, self.payload_cap),
            self.max_v_forward * min(forward_frac, self.cmd_cap),
            self.max_v_turn * min(turn_frac, self.cmd_cap),
        )
        # Pushes ramp in with the commands: none while learning to stand.
        self.training_env.env_method("set_push_curriculum", min(forward_frac, self.push_cap))

    def _on_step(self) -> bool:
        return True


def set_gear_slack(model, slack_deg_l, slack_deg_r):
    """Total free play of each gearbox, in degrees at the wheel (0 = rigid)."""
    for i, deg in enumerate((slack_deg_l, slack_deg_r)):
        half = max(1e-6, 0.5 * np.radians(deg))
        model.tendon_range[i] = (-half, half)


class RealRobotEnv(MujocoEnv):
    """Raw observation, 17 values, the layout every index in this file uses:
    root qpos (7), encoder angles (2), root qvel (6), encoder speeds (2). The
    encoders sit on the motor shaft, so those four values are the ROTORS'
    (qpos[9:11], qvel[8:10]); the wheels behind the gear slack are not
    observed, as on the car."""

    def __init__(self):
        obs_space = Box(low=-np.inf, high=np.inf, shape=(17,), dtype=np.float64)
        super().__init__(model_path=XML_FILE_PATH, frame_skip=FRAME_SKIP,
                         observation_space=obs_space, default_camera_config={})

    def _get_obs(self):
        d = self.data
        return np.concatenate([d.qpos[:7], d.qpos[ROTOR_QPOS],
                               d.qvel[:6], d.qvel[ROTOR_QVEL]]).ravel()

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        return self._get_obs(), 0.0, False, False, {}

    def reset_model(self):
        qpos = self.init_qpos.copy()
        qvel = self.init_qvel.copy()
        roll, pitch = self.np_random.uniform(-RESET_TILT_NOISE, RESET_TILT_NOISE, size=2)
        q = R.from_euler("xyz", [roll, pitch, 0.0]).as_quat()
        qpos[3:7] = [q[3], q[0], q[1], q[2]]
        qvel[:6] += self.np_random.uniform(-RESET_VEL_NOISE, RESET_VEL_NOISE, size=6)
        self.set_state(qpos, qvel)
        return self._get_obs()


class PWMCommandWrapper(VelocityCommandWrapper):
    """Policy commands PWM; the motor model turns that into wheel torque.

    The observation is built the way the firmware builds it (policy.c
    finish_obs + odom.c), from emulated sensors. The reward, termination and
    curriculum are inherited and stay on simulator ground truth: the policy
    only ever sees what the car can measure, but is graded on what actually
    happened.
    """

    def __init__(self, env, **kwargs):
        super().__init__(env, **kwargs)
        # PWM command, pre-deadband-compensation, exactly as the firmware's
        # controller output is defined.
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,),
                                           dtype=np.float32)
        self._hist = None
        self._prev_action = None
        self._prev_pwm = None
        self._la_true = [0.0] * 6
        self._pos_err = None
        self._yaw_err = None
        self._motor = NOMINAL_MOTOR
        self._act_queue = collections.deque()
        self._enc_ring = None
        self._enc_filled = 0
        self._pitch_off = self._roll_off = 0.0
        self._gyro_bias = np.zeros(3)
        self._noisy = False
        # Pushes (PUSH_*): off until the curriculum sets a scale above 0.
        self._push_scale = 0.0
        self._push_left = 0           # ticks left in the current push
        self._push_next = 0           # ticks until the next push starts
        self._calm_pause = 0          # ticks left without the wobble penalty
        self._chassis_id = mujoco.mj_name2id(self.unwrapped.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        base = self.observation_space.shape[0]
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(base + len(HISTORY_TAPS) * N_HISTORY_SIGNALS + 2,),
            dtype=np.float32)

    def _sense(self, raw_obs):
        """What the firmware would measure this tick: fused roll/pitch, body
        gyro rates, encoder wheel speeds and odometry forward speed."""
        rng = self.np_random
        qw, qx, qy, qz = raw_obs[3:7]
        roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0))
        gyro = np.array(raw_obs[12:15], dtype=np.float64) + self._gyro_bias
        roll += self._roll_off
        pitch += self._pitch_off
        if self._noisy:
            roll += rng.normal(0.0, ATT_NOISE_RAD)
            pitch += rng.normal(0.0, ATT_NOISE_RAD)
            gyro += rng.normal(0.0, GYRO_NOISE_RADS, size=3)

        # Encoders count the motor shaft relative to the chassis (raw_obs[7:9]
        # is the rotor joint angle, see RealRobotEnv), which the simulator
        # zeroes on every reset as odom_reset() does. Through the gear slack
        # this is not quite the wheel.
        counts = np.floor(np.asarray(raw_obs[7:9]) / ENC_RAD_PER_COUNT)
        if self._enc_ring is None:
            self._enc_ring = collections.deque([np.zeros(2)], maxlen=ENC_VEL_WINDOW + 1)
            self._enc_filled = 0
        self._enc_ring.append(counts)
        self._enc_filled = min(self._enc_filled + 1, ENC_VEL_WINDOW)
        n = self._enc_filled
        dt = self.unwrapped.dt
        wheel_vel = (self._enc_ring[-1] - self._enc_ring[-1 - n]) * ENC_RAD_PER_COUNT / (n * dt)
        wheel_vel = np.clip(wheel_vel, -WHEEL_VEL_CLIP, WHEEL_VEL_CLIP)
        # odom.c: the encoders see the wheel relative to the chassis, so the
        # axle's rolling speed adds the pitch rate; the policy's v_forward is in
        # the pitched body frame, hence the cos.
        v_fwd = WHEEL_RADIUS * (0.5 * (wheel_vel[0] + wheel_vel[1]) + gyro[1]) * np.cos(pitch)
        v_fwd = float(np.clip(v_fwd, -V_FORWARD_CLIP, V_FORWARD_CLIP))
        return roll, pitch, gyro, wheel_vel, v_fwd

    def _get_conditioned_obs(self, raw_obs):
        """Mirror of firmware/policy.c finish_obs(), fed by _sense()."""
        dt = self.unwrapped.dt
        roll, pitch, gyro, wheel_vel, v_fwd = self._sense(raw_obs)
        _, _, local_vel, true_yaw_rate = self._decode_state(raw_obs)

        # Two copies of each leaky integral: the sensed one goes in the
        # observation (what the firmware computes), the true one into the reward
        # (whether the robot actually held station and heading). With gyro bias
        # the sensed heading integral drifts while the true heading does not -
        # the policy has to learn not to chase a bias.
        if self._pos_err is None:
            self._pos_err = self._pos_err_true = 0.0
            self._yaw_err = self._yaw_err_true = 0.0
        else:
            pd = float(np.exp(-dt / POSITION_TAU_S))
            yd = float(np.exp(-dt / YAW_TAU_S))
            self._pos_err = float(np.clip(
                pd * self._pos_err + (v_fwd - self.target_v_forward) * dt,
                -POS_ERR_CLIP_M, POS_ERR_CLIP_M))
            self._pos_err_true = float(np.clip(
                pd * self._pos_err_true + (local_vel[0] - self.target_v_forward) * dt,
                -POS_ERR_CLIP_M, POS_ERR_CLIP_M))
            self._yaw_err = float(np.clip(
                yd * self._yaw_err + (gyro[2] - self.target_v_turn) * dt,
                -YAW_CLIP_RAD, YAW_CLIP_RAD))
            self._yaw_err_true = float(np.clip(
                yd * self._yaw_err_true + (true_yaw_rate - self.target_v_turn) * dt,
                -YAW_CLIP_RAD, YAW_CLIP_RAD))

        if self._hist is None:
            self._la = [0.0] * 6      # load_adapt.c filter state, zeroed at arm
        osc, lean = load_adapt_step(self._la, np.degrees(gyro[1]), np.degrees(pitch))

        sample = np.array([pitch, gyro[1], v_fwd], dtype=np.float32)
        if self._hist is None:
            # First frame of an episode: no real history exists yet, so repeat
            # the current sample rather than feeding zeros, which would look
            # like a violent transient the policy must learn to ignore.
            self._hist = [sample.copy() for _ in range(HISTORY_LEN)]
        self._hist.insert(0, sample)
        del self._hist[HISTORY_LEN:]

        # Yaw pinned to zero, as policy_build_obs_rp() does: the MPU6050 has no
        # magnetometer, so the car cannot know its heading.
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        base = [SENSOR_HEIGHT, cp * cr, cp * sr, sp * cr, -sp * sr,
                min(osc * OSC_OBS_SCALE, LA_OBS_CLIP),    # run 20: in the dropped
                min(lean * LEAN_OBS_SCALE, LA_OBS_CLIP),  # wheel-angle slots
                v_fwd, 0.0, 0.0,                # lateral/vertical: not sensed
                gyro[0], gyro[1], gyro[2],
                wheel_vel[0], wheel_vel[1],
                self.target_v_forward, self.target_v_turn]
        return np.concatenate(
            [base] + [self._hist[t] for t in HISTORY_TAPS]
            + [[self._pos_err * POSITION_OBS_SCALE,
                self._yaw_err * YAW_OBS_SCALE]]).astype(np.float32)

    def set_push_curriculum(self, frac):
        self._push_scale = 0.0 if self.is_eval else float(frac)

    def _schedule_push(self):
        dt = self.unwrapped.dt
        self._push_next = int(self.np_random.uniform(*PUSH_INTERVAL_S) / dt)

    def _update_push(self):
        """Start, hold or end a push; the wrench stays in xfrc_applied while
        the push lasts (all five physics substeps of the tick)."""
        d = self.unwrapped.data
        if self._push_left > 0:
            self._push_left -= 1
            if self._push_left == 0:
                d.xfrc_applied[self._chassis_id] = 0.0
                self._schedule_push()
            return
        if self._push_scale <= 0.0:
            return
        self._push_next -= 1
        if self._push_next > 0:
            return
        rng, m = self.np_random, self.unwrapped.model
        force = rng.choice((-1.0, 1.0)) * rng.uniform(0.0, PUSH_MAX_N * self._push_scale)
        lever = rng.uniform(*PUSH_HEIGHT_M) - float(m.body_ipos[self._chassis_id][2])
        # Along the car's own heading, horizontal; torque = r x F with r up.
        fwd = d.xmat[self._chassis_id].reshape(3, 3)[:, 0].copy()
        fwd[2] = 0.0
        fwd /= max(np.linalg.norm(fwd), 1e-9)
        f = force * fwd
        d.xfrc_applied[self._chassis_id] = [f[0], f[1], 0.0, -lever * f[1], lever * f[0], 0.0]
        self._push_left = max(1, int(round(rng.uniform(*PUSH_DURATION_S) / self.unwrapped.dt)))
        self._calm_pause = self._push_left + int(PENALTY_PAUSE_S / self.unwrapped.dt)

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        if not self.is_eval:
            self._update_push()
        # The action computed this tick reaches the motors latency ticks later.
        self._act_queue.append(action.copy())
        applied = self._act_queue.popleft()
        # Rotor angular velocities drive the back-EMF term, so the torque a
        # given PWM delivers depends on how fast the motors are already going
        # (inside the gear slack the rotor and the wheel differ).
        omega = np.asarray(self.unwrapped.data.qvel[ROTOR_QVEL], dtype=np.float64)
        tau = action_to_torque(applied, omega, self._motor)
        obs, reward, terminated, truncated, info = super().step(tau.astype(np.float32))

        # Charge for reversing the motors, not just for the torque magnitude -
        # the inherited effort term costs the same whether the command is held
        # or flipped every tick, so nothing was discouraging chatter.
        if self._prev_action is not None:
            reward -= ACTION_RATE_WEIGHT * float(
                np.sum(np.square(action - self._prev_action)))
        self._prev_action = action.copy()
        # A reversal is judged against the last nonzero sign, so passing through 0 still counts.
        s = np.sign(action)
        reward -= REVERSAL_WEIGHT * float(np.sum((s != 0) & (s == -self._last_sign)))
        self._last_sign = np.where(s != 0, s, self._last_sign)
        qw, qx, qy, qz = self.unwrapped.data.qpos[3:7]
        true_pitch = float(np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)))
        gate = upright_gate(true_pitch)
        pwm = action_to_pwm(action)
        if self._prev_pwm is not None:
            reward -= gate * PWM_STEP_WEIGHT * float(np.sum(np.square((pwm - self._prev_pwm) / 2800.0)))
        self._prev_pwm = pwm

        # Station keeping and heading, graded on the TRUE integrals refreshed by
        # _get_conditioned_obs during the super().step() call above.
        reward -= POSITION_WEIGHT * abs(self._pos_err_true)
        reward -= YAW_WEIGHT * abs(self._yaw_err_true)

        # Wobble, on the true chassis pitch rate (qvel[4], body frame).
        q = float(self.unwrapped.data.qvel[4])
        if self._calm_pause > 0:
            self._calm_pause -= 1
        else:
            reward -= gate * min(PITCH_RATE_WEIGHT * q * q, PITCH_RATE_PENALTY_MAX)
        osc_true, _ = load_adapt_step(self._la_true, np.degrees(q), np.degrees(true_pitch))
        reward -= min(OSC_PENALTY_WEIGHT * (osc_true / OSC_PENALTY_REF_DPS) ** 2, OSC_PENALTY_MAX)
        return obs, reward, terminated, truncated, info

    def reset(self, **kwargs):
        self._hist = None          # refilled on the first observation
        self._prev_action = None   # no rate penalty on the first step
        self._prev_pwm = None
        self._la_true = [0.0] * 6  # run 21: osc of the true gyro, for the reward
        self._last_sign = np.zeros(2)   # no reversal charged on the first command
        self._pos_err = None       # re-zeroed by the first observation
        self._yaw_err = None
        self._enc_ring = None      # odom_reset()
        # Everything uncertain about the car is drawn once per training episode
        # and held fixed through it, the way it is fixed on any one car on any
        # one run. Eval uses the nominal car (latency EVAL_LATENCY_TICKS, gear
        # slack EVAL_GEAR_SLACK_DEG, no IMU error) so its score stays
        # comparable between evaluations.
        rng = self.np_random
        if self.is_eval:
            self._motor = NOMINAL_MOTOR
            latency = EVAL_LATENCY_TICKS
            self._pitch_off = self._roll_off = 0.0
            self._gyro_bias = np.zeros(3)
            self._noisy = False
        else:
            self._motor = sample_motor(rng)
            latency = int(rng.integers(ACTION_LATENCY_TICKS[0], ACTION_LATENCY_TICKS[1] + 1))
            self._pitch_off = rng.uniform(-PITCH_OFFSET_RAD, PITCH_OFFSET_RAD)
            self._roll_off = rng.uniform(-ROLL_OFFSET_RAD, ROLL_OFFSET_RAD)
            self._gyro_bias = rng.uniform(-GYRO_BIAS_RADS, GYRO_BIAS_RADS, size=3)
            self._noisy = True
        # Motors are off until the policy is armed, so the in-flight actions
        # at the start of an episode are zero.
        self._act_queue = collections.deque([np.zeros(2)] * latency)
        obs, info = super().reset(**kwargs)
        # No push carried over from the last episode; the first comes 1.5-4 s in.
        self.unwrapped.data.xfrc_applied[self._chassis_id] = 0.0
        self._push_left = 0
        self._calm_pause = 0
        self._schedule_push()

        # Armature (on the rotor joints) and gear slack (tendon ranges) are
        # MODEL fields, so they persist across resets and have to be re-set
        # every episode, exactly like payload.
        m = self.unwrapped.model
        if self.is_eval:
            m.dof_armature[ROTOR_QVEL] = ARMATURE_NOMINAL
            set_gear_slack(m, EVAL_GEAR_SLACK_DEG, EVAL_GEAR_SLACK_DEG)
        else:
            m.dof_armature[ROTOR_QVEL] = self.np_random.uniform(*ARMATURE_RANGE)
            set_gear_slack(m, *self.np_random.uniform(*GEAR_SLACK_DEG, size=2))
        if self.is_eval:
            # Override the parent's random draw with the fixed schedule, so the
            # eval score is comparable between evaluations. eval_episode was
            # already incremented by the parent, hence the -1.
            idx = (self.eval_episode - 1) % len(EVAL_PAYLOADS)
            self.unwrapped.model.body_mass[self.payload_body_id] = EVAL_PAYLOADS[idx]
        return obs, info


def make_env(**kwargs):
    def _init():
        env = PWMCommandWrapper(RealRobotEnv(), max_payload_kg=MAX_PAYLOAD_KG,
                                max_v_forward=MAX_V_FORWARD,
                                max_v_turn=MAX_V_TURN,
                                fall_angle_limit=FALL_ANGLE_LIMIT, **kwargs)
        return TimeLimit(env, max_episode_steps=MAX_EPISODE_STEPS)
    return _init


# Run 12 trains FROM SCRATCH (30M steps) on the measured motor (with
# stiction), slack and delay, with run 11's wobble penalty switched off
# (PITCH_RATE_WEIGHT). The plant changed too much to
# fine-tune (torque limit 0.4 -> 0.75 N m, friction ~7x, a viscous term), and
# run 11's fine-tune from run 10 could not leave run 10's way of balancing.
# Runs 10 and 11 continued from runs 9 and 10; pass --init PATH to continue
# from a checkpoint instead (10M steps by default).
#
# Run 13 continues from run 12's FINAL checkpoint (better push tolerance than
# its best-eval one, same wobble and tracking) for 10M steps, with pushes at
# full strength from the start and the wobble penalty at 0.5. Pass --init none
# to train from scratch (30M) as run 12 did.
#
# Run 14 trains from scratch (30M steps) with pushes and the wobble penalty at
# 1.5: a fine-tune from run 13 would keep run 13's way of balancing.
#
# Run 15 is run 14 with pushes capped at 3 N and the wobble penalty paused
# for 1 s after each push (session-logs/2026-10-14-run14-desktop.md §3-4).
#
# Run 16 is run 15 plus a charge per motor reversal (REVERSAL_WEIGHT) and
# gear slack up to 6 deg, from scratch: the 7 Hz shake through the slack is
# how runs 13-15 balance, and a fine-tune would keep it
# (session-logs/2026-10-20-car-shake-and-run16.md).
#
# Run 17 is run 16 with REVERSAL_WEIGHT 0.5 -> 0.3 (the laptop's fallback for
# a run that stops surviving pushes).
#
# Run 18 is run 17 with no payload in training (MAX_PAYLOAD_KG = 0).
# Run 19: payload 0-2 kg; reversal charge replaced by a PWM-change charge, and it
# and the wobble penalty apply only near upright (upright_gate). Checkpoints every 1M.
# Run 20: run 19 plus osc and lean (load_adapt.c) in obs[5], obs[6].
# Run 21: run 20 plus a reward charge on osc of the true gyro (OSC_PENALTY_*).
RUN_NAME = "run21"
INIT_FROM = "none"
FINETUNE_STEPS = 10_000_000
FINETUNE_LR = 1e-4
# Run 9 evaluated every 10,000 steps per env (110k total) with 20 episodes of
# up to 10 s each, one at a time: 39 % of its 3.9 h went to evaluation.
EVAL_EVERY_STEPS = 250_000


def main():
    ap = argparse.ArgumentParser(description="Train the real-robot PWM policy.")
    ap.add_argument("--init", default=INIT_FROM,
                    help="checkpoint to continue from, or 'none' to train from scratch "
                         f"(default: {INIT_FROM})")
    ap.add_argument("--steps", type=int, default=None,
                    help=f"total steps (default: {FINETUNE_STEPS:,} when continuing, 30,000,000 from scratch)")
    ap.add_argument("--run", default=RUN_NAME, help="log folder under logs/")
    ap.add_argument("--best-dir", default="models/best_real",
                    help="where the best checkpoint is saved")
    ap.add_argument("--envs", type=int, default=max(1, (os.cpu_count() or 2) - 1),
                    help="parallel environments (default: CPU threads - 1)")
    for cap in ("cmd", "push", "payload"):
        ap.add_argument(f"--{cap}-cap", type=float, default=1.0,
                        help=f"diagnostic: hold the {cap} curriculum at this fraction of full (default 1)")
    args = ap.parse_args()
    from_scratch = args.init.lower() == "none"

    best_dir = Path(args.best_dir)
    best_dir.mkdir(parents=True, exist_ok=True)
    # From scratch: 30M, up from 12M. Runs 2 and 3 were BOTH still climbing
    # when their 12M budget ran out - +202 and +93 reward per 1M steps
    # respectively over their final third, with the best eval inside the last
    # 3% of the run. Neither plateaued, oscillated or collapsed, and log_std
    # ended at 0.07-0.09 from an 0.25 start, so exploration was healthy too.
    # The runs were not failing to converge; they were being cut off mid-climb.
    total_timesteps = args.steps or (30_000_000 if from_scratch else FINETUNE_STEPS)
    num_cpu = args.envs

    def monitored(**kw):
        f = make_env(**kw)
        return lambda: Monitor(f())

    train_env = SubprocVecEnv([monitored(is_eval=False) for _ in range(num_cpu)])
    eval_env = monitored(is_eval=True)()

    if from_scratch:
        # log_std_init: the action space is now +/-1.0, so SB3's default std of
        # 1.0 is 100% of the range and would clip constantly. Same
        # 25%-of-range rule that fixed the first run on the previous plant.
        model = PPO("MlpPolicy", env=train_env,
                    policy_kwargs=dict(net_arch=[64, 64], log_std_init=float(np.log(0.25))),
                    learning_rate=linear_schedule(3e-4), ent_coef=0.01,
                    # 19 envs x n_steps 2048 = 38,912 samples per rollout. At
                    # batch_size 128 and SB3's default n_epochs=10 that was 3,040
                    # gradient updates per rollout - a lot of passes over one
                    # rollout, slow on CPU and prone to over-fitting each batch.
                    # 512 is conventional at this rollout size.
                    n_steps=2048, batch_size=512, device="cpu", verbose=1)
        curriculum = {}
    else:
        # Same hyperparameters as the checkpoint, a gentler learning rate, and
        # the full command range from the start (the policy already has it);
        # payload still ramps in over the run.
        lr = linear_schedule(FINETUNE_LR)
        model = PPO.load(args.init, env=train_env, device="cpu",
                         custom_objects={"learning_rate": lr, "lr_schedule": lr})
        curriculum = dict(stand_phase_end=0.0, velocity_phase_end=0.0)

    # Until now the log held ONLY eval rewards - no explained_variance, approx_kl,
    # clip_fraction or entropy_loss - so every diagnosis of "why is this slow"
    # was guesswork. The CSV is the cheap one to read back: one row per rollout,
    # greppable without parsing the stdout tables.
    model.set_logger(configure(f"logs/{args.run}", ["stdout", "csv"]))

    callback = CallbackList([
        CurriculumCallback(total_timesteps=total_timesteps,
                           max_payload_kg=MAX_PAYLOAD_KG,
                           max_v_forward=MAX_V_FORWARD, max_v_turn=MAX_V_TURN,
                           cmd_cap=args.cmd_cap, push_cap=args.push_cap,
                           payload_cap=args.payload_cap, **curriculum),
        EvalCallback(eval_env, best_model_save_path=str(best_dir),
                     eval_freq=max(1, EVAL_EVERY_STEPS // num_cpu), n_eval_episodes=20),
        # Every 1M steps: runs 15-16 were calm at 10-11M and shook by 30M.
        CheckpointCallback(save_freq=max(1, 1_000_000 // num_cpu),
                           save_path=f"models/ckpt_{args.run}"),
    ])

    print(f"\nTraining on real_robot.xml at 200Hz, PWM actions "
          f"({total_timesteps:,} steps, {num_cpu} envs, "
          f"{'from scratch' if from_scratch else 'from ' + args.init})...")
    try:
        model.learn(total_timesteps=total_timesteps, callback=callback,
                    progress_bar=True)
        model.save("models/ppo_real_robot")
    finally:
        train_env.close()
        eval_env.close()


if __name__ == "__main__":
    main()
