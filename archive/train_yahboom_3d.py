import argparse
import os
import random
from pathlib import Path
from typing import Callable

import gymnasium as gym
import numpy as np
import mujoco
from gymnasium.envs.mujoco.mujoco_env import MujocoEnv
from gymnasium.spaces import Box
from gymnasium.wrappers import TimeLimit
from scipy.spatial.transform import Rotation as R
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

MAX_EPISODE_STEPS = 1000  # ~12.5s of sim time (frame_skip=5, timestep=0.0025s)
SHAPING_GAMMA = 0.99  # matches PPO's default discount factor

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

# Switched from my_robot.xml to the teammate's model (DIP-E026), whose
# parameters are calibrated against the real firmware. my_robot.xml is a badly
# conditioned plant: a classical cascade PID could not stabilize it with ANY
# gains (0 of ~500 combinations over a 64x kp range, both signs, three torque
# limits), it topples within 0.6s from spawn with the contact count flickering
# 2->4->10, its inertia about the wheel axle is 10x lower than the real robot's
# (0.00284 vs 0.0296), and torque mostly spins the chassis rather than driving
# the wheels (ctrl=+0.02 gives v=0.008 here vs v=0.62 on their model). The same
# PID holds a commanded cruise on their model with error 0.000 at every speed
# tested and 56 continuous wheel revolutions over 60s, so driving-while-
# balancing is demonstrably achievable there. See pid_baseline.py.
XML_FILE_PATH = os.path.join(CURRENT_DIR, "their_robot.xml")

# Control rate = 1/(FRAME_SKIP * 0.0025) = 80Hz, episodes ~12.5s.
#
# This is NOT a free parameter on this plant. Sweeping 120 random gain sets
# through a cascade PID at each rate (scratchpad/rate_sweep.py) shows how many
# can balance AND hold a 0.3 m/s cruise:
#
#     400Hz 93/120 | 200Hz 95/120 | 100Hz 97/120 | 80Hz 91/120
#      50Hz 64/120 |  40Hz 24/120 |  25Hz  0/120 |  20Hz  0/120
#
# The old 40Hz rate (frame_skip=5 at my_robot.xml's 0.005 timestep) sits right
# on the cliff: only 20% of controllers can stabilize the plant there, and the
# 200Hz-tuned gains fell within 0.5s when run at 40Hz inside this harness.
# Asking PPO to find a controller in a basin that narrow is a needless
# handicap. 80Hz is in the flat region, is comfortably achievable on the STM32
# for a 64x64 MLP, and keeps the same frame_skip arithmetic as before.
FRAME_SKIP = 5

# Reset perturbation. my_robot.xml was numerically unstable at spawn and began
# toppling on its own, which accidentally supplied the disturbance the policy
# had to reject. Their model is well conditioned and sits at its equilibrium
# indefinitely: measured here, a zero-torque episode survives all 1000 steps
# and banks a return of 1634, so without this noise the optimal policy is
# literally "output nothing" and balance is never learned. Magnitudes are well
# inside the 0.4 rad fall limit but large enough that an uncontrolled robot
# topples, mirroring the pitch-rate kick used to make pid_baseline.py's
# balance test non-vacuous.
RESET_TILT_NOISE = 0.05   # rad, roll and pitch
RESET_VEL_NOISE = 0.10    # m/s and rad/s on the chassis freejoint

# Eval commands, as fractions of (max_v_forward, max_v_turn), one per eval
# episode and HELD for the whole episode.
#
# Eval previously forced both commands to zero, so "best model" was selected
# purely on how long the robot could stand still - the one thing the old policy
# already did perfectly (1000+/-0) while being unable to drive at all. That
# metric could not distinguish a good driver from a statue, which is precisely
# the failure being chased. The list is fixed rather than sampled so the number
# stays comparable between evaluations, starts at (0,0) so standing is still
# measured, and holds each command for a full episode so sustained driving and
# continuous turning are what get scored.
# Yaw-rate low-pass for the turn-error term. ~0.1s time constant at 80Hz.
#
# WHY: this plant can track a commanded yaw rate on AVERAGE but not moment to
# moment. Measured against a 0.50 rad/s command: achievable mean yaw is 0.485
# (97%), while the best achievable INSTANTANEOUS error is 0.277 - 55% of the
# command. Turning necessarily involves a yaw limit cycle here.
#
# Charging instantaneous error therefore prices in an oscillation the robot
# cannot avoid, and made turning literally not worth doing. Measured per-step
# turn cost over the 20 eval commands, under each definition:
#
#                            survives   instantaneous   filtered
#     policy that never turns   1000        0.237         0.236
#     PID kp_turn=0.5           1000        0.288         0.134
#     PID kp_turn=2.0            965        0.469         0.095
#
# Under instantaneous scoring, NOT turning is the cheapest option and turning
# harder is monotonically worse - so the 5M-step run learned to ignore turn
# commands entirely (turn output +0.032 against a +/-0.50 command). That was
# correct reward maximization, not a training failure: a tuned classical PID
# could not beat "don't turn" either. Under filtered scoring the ranking
# inverts and turning wins by ~0.14/step while still surviving 965/1000 steps.
#
# A small instantaneous term is kept alongside it (TURN_INSTANT_WEIGHT) so the
# policy is not free to satisfy the average by oscillating wildly. With it,
# kp_turn=0.5 (0.192) and kp_turn=2.0 (0.189) score almost equally, i.e. the
# smoother controller is no longer punished, but neither is wild spinning
# rewarded.
TURN_LPF = 0.9
TURN_INSTANT_WEIGHT = 0.2

# Filtered-turn-error weight, raised from 1.0 to match the forward axis.
#
# The 1.0/2.0 asymmetry against forward was arbitrary, and raising it used to
# be pointless: turning and not turning cost the SAME per step (0.266 vs
# 0.256), so any weight scaled both sides equally and the incentive stayed
# flat. Filtering changes that - turning now costs measurably less than not
# turning (0.095 vs 0.236) - so the weight finally has something to multiply.
#
# Sized against the forward axis, which is the signal strength that demonstrably
# did get learned. Per 1000-step episode, the reward available for tracking:
#
#     forward tracking (weight 2.0)        ~323
#     turn at weight 1.0                    ~94   (3.5x weaker - likely too weak)
#     turn at weight 2.0                   ~235   (comparable)
#
# 2.0 rather than higher because that is the value already proven safe on the
# forward axis; more risks the policy trading away balance to chase yaw.
TURN_WEIGHT = 2.0

EVAL_COMMANDS = [
    (0.0, 0.0),
    (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
    (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0),
    (0.5, 0.0), (-0.5, 0.0), (0.0, 0.5), (0.0, -0.5),
    (0.5, 0.5), (-0.5, 0.5), (0.5, -0.5), (-0.5, -0.5),
    (0.75, 0.25), (-0.75, -0.25), (0.25, 0.75),
]

# LEARNING RATE SCHEDULE
def linear_schedule(initial_value: float) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:
        return progress_remaining * initial_value
    return func

class YahboomEnv(MujocoEnv):
    def __init__(self):
        temp_model = mujoco.MjModel.from_xml_path(XML_FILE_PATH)
        obs_space = Box(low=-np.inf, high=np.inf, shape=(temp_model.nq + temp_model.nv,), dtype=np.float64)
        super().__init__(model_path=XML_FILE_PATH, frame_skip=FRAME_SKIP, observation_space=obs_space, default_camera_config={})

    def _get_obs(self): return np.concatenate([self.data.qpos, self.data.qvel]).ravel()
    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        return self._get_obs(), 0.0, False, False, {}
    def reset_model(self):
        qpos = self.init_qpos.copy()
        qvel = self.init_qvel.copy()
        roll, pitch = self.np_random.uniform(-RESET_TILT_NOISE, RESET_TILT_NOISE, size=2)
        # Yaw is left at 0. Every other signal the policy sees is already
        # body-frame, so randomizing yaw would only add a large, irrelevant
        # input variation to fit around.
        quat = R.from_euler('xyz', [roll, pitch, 0.0]).as_quat()  # scipy: x,y,z,w
        qpos[3:7] = [quat[3], quat[0], quat[1], quat[2]]
        qvel[:6] += self.np_random.uniform(-RESET_VEL_NOISE, RESET_VEL_NOISE, size=6)
        self.set_state(qpos, qvel)
        return self._get_obs()

class VelocityCommandWrapper(gym.Wrapper):
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

        # A velocity-error integral was appended here (mirroring the team's
        # STM32 PI velocity loop) and measurably REGRESSED training: at equal
        # progress, eval episode length was ~5x worse than without it (116 vs
        # 598 at 2.85M steps) and flat where the baseline was climbing. It was
        # not a feature-scaling problem - measured p95 of the integral was
        # 0.077, one of the SMALLEST inputs. The likely reason is that episodes
        # terminate long before the integral can accumulate anything useful
        # (max observed 0.108 against a +/-2.0 clamp), so during the phase when
        # balance is still being learned it contributes noise rather than
        # signal. Reverted.
        original_obs_space = self.env.observation_space.shape[0]
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(original_obs_space,), dtype=np.float32)

        # Raw torque in Nm, taken straight from the model's ctrlrange so it can
        # never silently disagree with the plant. On their model that is the
        # real motor's stall torque (+/-0.6 Nm) rather than my_robot.xml's
        # made-up +/-4.0, so the policy cannot learn to rely on torque the
        # hardware does not have.
        #
        # Normalizing this to +/-1.0 with a 0.3 Nm scale was tried and
        # measurably REGRESSED: action std rose 0.0155 -> 0.0575 as intended,
        # but eval episode length fell 1000+/-0 -> 704+/-352 and the policy
        # began overshooting targets 3x (turn peaks of -1.5 rad/s against a 0.5
        # target). Relative noise is what matters and it barely moved (~15% of
        # mean output either way), while the extra sustained noise degraded the
        # controller. Reverted.
        self.tau_max = float(self.unwrapped.model.actuator_ctrlrange[:, 1].max())
        self.action_space = gym.spaces.Box(low=-self.tau_max, high=self.tau_max,
                                           shape=(2,), dtype=np.float32)

    def set_curriculum(self, payload_kg, v_forward, v_turn):
        self.curriculum_payload_kg = payload_kg
        self.curriculum_v_forward = v_forward
        self.curriculum_v_turn = v_turn

    def _sample_command_timer(self):
        # Mostly short segments (reactive command changes), but sometimes a
        # long steady-state hold up to a full episode - without this, the
        # policy rarely if ever practices sustaining one direction (e.g. a
        # continuous circular turn) longer than ~7.5s, since commands
        # otherwise always change every 100-300 steps.
        if random.random() < 0.3:
            return random.randint(400, MAX_EPISODE_STEPS)
        return random.randint(100, 300)

    def _get_conditioned_obs(self, raw_obs):
        # MuJoCo reports a freejoint's linear velocity in the WORLD frame, but
        # the commands and the reward are both body-frame ("forward" means the
        # robot's own forward). Feeding world frame meant the same physical
        # motion produced completely different observations depending on yaw
        # ([0.3,0,0] at yaw 0 vs [0,0.3,0] at yaw 90) while the reward scored
        # both identically - so the policy had to learn a quaternion transform
        # just to perceive its own forward speed, and never did. Yaw rate needs
        # no such transform, which is why turning trained and driving did not.
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
        other, which is a worse problem than a softened theoretical guarantee.

        Each velocity axis blends its own "hold still" term against its own
        "track the target" term, weighted by how much *that axis* is being
        commanded. A single shared blend weight (the previous design) let a
        large turn command dominate the blend and starve the forward axis of
        tracking signal, since training samples the two commands
        independently and simultaneously - measured result was a policy that
        turned well but never translated at all.

        At w=0 an axis reduces exactly to the old "don't drift" term, and at
        w=1 to the old "close the tracking gap" term, so per-axis behavior is
        unchanged; only the cross-axis contamination is gone.
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

        # Pitch weight lowered from 5.0 and tracking weights raised from
        # 1.0/0.5: at the old weights, a mere 0.1 rad tilt (needed just to
        # accelerate) cost more reward than fully failing to track velocity
        # ever could, so the optimal policy was "ignore commands, minimize
        # tilt" - confirmed empirically (see diagnose_drive.py), the trained
        # policy made a brief attempt at commanded velocity then always
        # decayed back to standing still. These weights put tracking error
        # on comparable footing with tilt instead of letting tilt dominate.
        # Effort penalty is normalized by the torque limit so it means the same
        # thing on any plant: a fraction-of-available-torque cost, capped at 0.2
        # (10% of the alive bonus) when both motors saturate. Penalizing raw Nm
        # at a fixed 0.1 coefficient was calibrated for my_robot.xml's +/-4.0
        # range, where the learned policy used ~1% of it and the term was
        # effectively dead; carried over unchanged to a +/-0.6 plant it would
        # either vanish entirely or, if rescaled to match the old magnitude,
        # cost up to 3.2 - more than the alive bonus - and punish the very
        # torque the robot needs to stay upright.
        effort = np.sum(np.square(action / self.tau_max))
        total_reward = 2.0 - (abs(pitch) * 3.0) - (effort * 0.1)
        total_reward -= abs(actual_v_forward - self.target_v_forward) * 2.0

        # Turn error is graded on a low-passed yaw rate (see TURN_LPF): this
        # plant can hold the commanded yaw on average but not instantaneously,
        # and charging the instantaneous error made turning worth less than not
        # turning at all. A small instantaneous term remains so the average
        # cannot be satisfied by oscillating wildly.
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

def make_wrapped_env(**kwargs):
    def _init():
        env = VelocityCommandWrapper(YahboomEnv(), **kwargs)
        return TimeLimit(env, max_episode_steps=MAX_EPISODE_STEPS)
    return _init


class CurriculumCallback(BaseCallback):
    """Staged curriculum: stand, then drive forward/back, then turn, then
    carry payload - instead of ramping everything at once.

    Ramping everything together forces balance and velocity-tracking to
    compete for gradient signal before balance is solid, which produces a
    fragile policy that only occasionally balances well (the 1000-step eval
    that regressed to ~800 a couple evals later). Staging the difficulty lets
    each sub-skill build on a working version of the previous one.
    """

    def __init__(self, total_timesteps, max_payload_kg, max_v_forward, max_v_turn,
                 stand_phase_end=0.15, velocity_phase_end=0.5, verbose=0):
        super().__init__(verbose)
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
            # Forward AND turn ramp together.
            #
            # They used to be staged - forward alone until 0.35, then turn -
            # on the reasoning that translation needs a hard-to-discover
            # coordinated lean while spinning the wheels oppositely is easy, so
            # forward deserved an uncontested phase. That was measured on
            # my_robot.xml and does NOT transfer to this plant: here forward
            # trains readily while turning never appeared at all.
            #
            # Worse, the staging is actively harmful. For the 1.75M steps
            # before turn commands existed, the turn target was always 0 and
            # the reward penalized any yaw, so the policy learned an explicit
            # yaw SUPPRESSOR - confirmed by probing the trained policy, which
            # cancels a differential-torque bias below ~1.4 sigma and falls
            # within 35 steps above it. The turn phase then had to undo that
            # with exploration already decayed to 6% of the action range.
            # Ramping both together means no suppression prior consolidates.
            frac = (progress - self.stand_phase_end) / (self.velocity_phase_end - self.stand_phase_end)
            forward_frac, turn_frac = frac, frac
            payload_frac = 0.0
        else:
            forward_frac, turn_frac = 1.0, 1.0
            payload_frac = min(1.0, (progress - self.velocity_phase_end) / (1.0 - self.velocity_phase_end))

        self.training_env.env_method(
            "set_curriculum",
            self.max_payload_kg * payload_frac,
            self.max_v_forward * forward_frac,
            self.max_v_turn * turn_frac,
        )

    def _on_step(self) -> bool:
        return True

def main() -> None:
    # Separate output paths from the my_robot.xml runs: those checkpoints share
    # the same 17-dim observation shape so they would LOAD against this plant,
    # but they were trained on a +/-4.0 action space and different dynamics, so
    # silently overwriting or mixing them up would be easy and misleading.
    model_path = Path("models/ppo_yahboom_drive_their")
    model_path.parent.mkdir(parents=True, exist_ok=True)
    best_model_dir = Path("models/best_their")
    best_model_dir.mkdir(parents=True, exist_ok=True)
    
    total_timesteps = 5_000_000
    num_cpu = max(1, (os.cpu_count() or 2) - 1)

    def _make_monitored(**kwargs):
        env_fn = make_wrapped_env(**kwargs)
        def _init(): return Monitor(env_fn())
        return _init

    train_env = SubprocVecEnv([_make_monitored(is_eval=False) for _ in range(num_cpu)])
    eval_env = _make_monitored(is_eval=True)()

    # 64x64: plenty of capacity for a 17-obs/2-action balance task, and small
    # enough to run as a plain float32 forward pass on an STM32 (~22KB of
    # weights vs. ~277KB for 256x256, which wouldn't fit in 256KB flash at all).
    #
    # ent_coef: SB3 defaults this to 0.0, and the previous run's policy
    # collapsed to an action std of 0.01 on a +/-4.0 action space - effectively
    # deterministic, with no exploration left. It had learned to read the turn
    # command and to ignore the forward command entirely, and could not explore
    # its way out. Driving forward needs a counter-intuitive "lean back first"
    # maneuver sustained over many steps, which near-zero action noise will
    # never stumble onto.
    # log_std_init: SB3 defaults this to 0.0, i.e. an initial action std of
    # 1.0. That was 25% of my_robot.xml's +/-4.0 action range - reasonable
    # exploration. On this plant's physical +/-0.6 Nm limit the same default is
    # 167% of the range, so roughly a third of sampled actions clip to
    # saturation and the effective policy is bang-bang rather than Gaussian.
    # Measured on the first attempt at this run: after 380k steps the std was
    # still 0.67 (111% of range) and eval episodes lasted only 72 steps, since
    # ent_coef actively rewards keeping std large and there is far more entropy
    # available outside the clip bounds than inside them. Balancing needs fine
    # torque modulation, so the noise is set to the same 25%-of-range it had on
    # the old plant. Same lesson as the reverted action-scaling experiment,
    # running the other way: what matters is noise RELATIVE to the action range.
    tau_max = float(mujoco.MjModel.from_xml_path(XML_FILE_PATH).actuator_ctrlrange[:, 1].max())
    log_std_init = float(np.log(0.25 * tau_max))

    model = PPO("MlpPolicy", env=train_env,
                policy_kwargs=dict(net_arch=[64, 64], log_std_init=log_std_init),
                learning_rate=linear_schedule(3e-4), ent_coef=0.01,
                n_steps=2048, batch_size=128, device="cpu")

    curriculum_callback = CurriculumCallback(
        total_timesteps=total_timesteps, max_payload_kg=0.2, max_v_forward=0.3, max_v_turn=0.5,
    )
    eval_callback = EvalCallback(eval_env, best_model_save_path=str(best_model_dir),
                                 eval_freq=10000, n_eval_episodes=20)
    callback = CallbackList([curriculum_callback, eval_callback])

    print(f"\nStarting PPO training ({total_timesteps:,} Steps)...")
    try:
        model.learn(total_timesteps=total_timesteps, callback=callback, progress_bar=True)
        model.save(model_path)
    finally:
        train_env.close()
        eval_env.close()

if __name__ == "__main__":
    main()
