"""The learning environment.

``BalanceCore`` is pure NumPy so it can be exercised (and trained on) with no
third-party dependencies at all.  ``BalanceBotEnv`` is a thin Gymnasium
wrapper around it for stable-baselines3.

What the agent controls
-----------------------
The action is a 9-vector in ``[-1, 1]`` that scales the nine cascade-PID gains
multiplicatively around their hand-tuned nominal values (``action = 0`` *is*
the nominal PID).  The agent therefore never commands torque directly:
it *retunes the controller* 25 times a second while a conventional PID does
the actual stabilising at 200 Hz.  That is the "learn Kp/Kd/Ki" formulation,
and it keeps the whole thing interpretable -- you can watch the gains move on
the UI when you shove the robot.

Command source
--------------
``v_ref`` / ``yaw_rate_ref`` come from a scripted generator during training
and from the joystick / keyboard during interactive play.  The policy sees
both the reference and the tracking error, so it learns gains appropriate to
"stand still", "cruise" and "turn hard" separately.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .arena import Arena
from .controller import CascadePID, ControllerLimits
from .disturb import DisturbanceModel, sample_training_disturbance
from .dynamics import (BalanceBotDynamics, make_state, wrap_pi,
                       IX, IY, IPSI, IPSID, IV, ITH, ITHD)
from .params import (ArenaParams, DisturbanceConfig, GainSpace, RewardWeights,
                     RobotParams, SimParams)

OBS_DIM_BASE = 16          # everything except the rays and the gains
ACT_DIM = 9

MODE_PID = "pid"           # fixed nominal gains, action ignored (baseline)
MODE_PPO_GAINS = "ppo"     # PPO schedules the nine gains
MODES = (MODE_PID, MODE_PPO_GAINS)


def activity_gate(obs) -> float:
    """How much authority the learned policy should have right now, in [0, 1].

    A gain-scheduling policy sits in a feedback loop with the plant *through
    its own observation* -- it sees the gains it just set.  Left ungated that
    loop wallows: a slow ~0.2 Hz, ~3 deg pitch oscillation that the fixed PID
    does not have.  Freezing the policy's own mean gains removes it entirely,
    which proves the gains it picks are fine and only the dithering is at
    fault.

    Filtering cannot fix it (the wallow is *slower* than the shove response we
    want to keep) and a deadband makes it worse (a deadband inside a feedback
    loop is a limit-cycle generator -- measured, not guessed).  What does work
    is cutting the loop gain smoothly near equilibrium: scale the action by
    how far the robot actually is from its setpoint.

    This is not a patch over a bad policy.  The nominal gains are the LQR
    solution for the nominal linearised plant, so when the robot is quiet and
    on-model there is provably nothing to gain by deviating -- and because the
    action space is anchored so that ``action = 0`` *is* the nominal PID,
    scaling toward zero lands exactly on that controller.  Away from
    equilibrium the gate opens fully and the policy has its full authority.
    """
    o = np.asarray(obs, dtype=float)
    e_pitch = o[0] * 0.60 - o[8] * 0.25      # pitch - pitch_ref
    pitch_rate = o[1] * 6.0
    e_v = o[4] * 1.5
    e_yaw = o[7] * 4.0
    sat = o[14]
    activity = max(abs(e_pitch) / 0.05,
                   abs(pitch_rate) / 1.5,
                   abs(e_v) / 0.30,
                   abs(e_yaw) / 1.0,
                   sat)
    return float(np.clip(activity, 0.0, 1.0))


@dataclass
class CommandScript:
    """Piecewise-constant reference generator used during training."""
    v_max: float = 0.9
    yaw_max: float = 2.5
    hold_min: float = 1.0
    hold_max: float = 3.5
    p_zero: float = 0.35        # probability of a "stand still" segment

    def sample(self, rng):
        if rng.random() < self.p_zero:
            v = 0.0
        else:
            v = rng.uniform(-self.v_max, self.v_max)
        w = 0.0 if rng.random() < self.p_zero else rng.uniform(-self.yaw_max,
                                                              self.yaw_max)
        hold = rng.uniform(self.hold_min, self.hold_max)
        return float(v), float(w), float(hold)


class BalanceCore:
    """Simulation + controller + reward, with no RL-library dependency."""

    def __init__(self,
                 robot: RobotParams | None = None,
                 sim: SimParams | None = None,
                 gains: GainSpace | None = None,
                 arena: ArenaParams | None = None,
                 rewards: RewardWeights | None = None,
                 disturbance: DisturbanceConfig | None = None,
                 mode: str = MODE_PPO_GAINS,
                 randomize: bool = True,
                 obstacles: bool = True,
                 sample_difficulty: bool = True,
                 seed: int | None = None):
        self.rng = np.random.default_rng(seed)
        self.robot_nominal = robot or RobotParams()
        self.sim = sim or SimParams()
        self.gs = gains or GainSpace()
        self.rw = rewards or RewardWeights()
        self.arena = Arena(arena or ArenaParams(), self.rng)
        self.mode = mode
        self.randomize = randomize
        self.use_obstacles = obstacles
        self.difficulty = 1.0        # curriculum ceiling
        self.ep_difficulty = 1.0     # what this episode actually drew
        # Training samples difficulty per episode (see reset); EVALUATION must
        # not, or "difficulty 1.0" silently means "uniform over [0.25, 1.0]"
        # and every number you compare gets easier at once.
        self.sample_difficulty = sample_difficulty

        self.robot = self.robot_nominal
        # 载重：0 = 空车。实车额定最大 4 kg，默认压在顶板上（轮轴上方 105 mm，
        # 就是 CAD 里雷达固定板那一层）。
        # Payload in kg; the car is rated for 4 kg, sitting on the top plate.
        self.payload_mass = 0.0
        self.payload_height = 0.105
        self.dyn = BalanceBotDynamics(self.robot)
        self.pid = CascadePID(self.gs, ControllerLimits(), self.robot.tau_max)
        self.dist = DisturbanceModel(disturbance or DisturbanceConfig(), self.rng)
        self._external_cmd = None       # (v_ref, yaw_ref) when teleoperated
        self._fixed_disturbance = disturbance is not None

        self.state = make_state()
        self.reset()

    # ------------------------------------------------------------------
    @property
    def obs_dim(self) -> int:
        return OBS_DIM_BASE + self.arena.p.n_rays + ACT_DIM

    # ------------------------------------------------------------------
    def reset(self, seed: int | None = None, hard: bool = True):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
            self.arena.rng = self.rng
            self.dist.rng = self.rng

        # --- episode difficulty -------------------------------------------
        # Sample uniformly in [0, difficulty] rather than using the ramped
        # value directly.  A monotonic ramp means that past the halfway point
        # the agent only ever sees the hardest conditions and quietly gets
        # worse at the easy ones -- which is exactly where a user first tries
        # it.  Keeping the easy end in the distribution costs nothing.
        if not self.randomize:
            self.ep_difficulty = 0.0
        elif not self.sample_difficulty:
            self.ep_difficulty = self.difficulty
        else:
            lo = 0.40 * self.difficulty          # keep some floor
            self.ep_difficulty = float(self.rng.uniform(
                min(lo, self.difficulty), self.difficulty)) \
                if self.difficulty > 0 else 0.0
            # one episode in eight is the pristine nominal robot
            if self.rng.random() < 1.0 / 8.0:
                self.ep_difficulty = 0.0

        # --- physical parameters (domain randomisation) -------------------
        if self.randomize:
            self.robot = self.robot_nominal.randomized(self.rng,
                                                       self.ep_difficulty)
        else:
            self.robot = self.robot_nominal
        # 载重在随机化**之后**挂上：它是外加的负载，不是这台车自身的不确定性，
        # 不该被 randomized() 的百分比抖动再乘一遍。
        # The payload is attached after randomisation: it is an external load,
        # not uncertainty about the car itself.
        if self.payload_mass > 0.0:
            self.robot = self.robot.with_payload(self.payload_mass,
                                                 self.payload_height)
        self.dyn = BalanceBotDynamics(self.robot)
        self.pid.tau_max = self.robot.tau_max

        # --- disturbances --------------------------------------------------
        if self.randomize and not self._fixed_disturbance:
            self.dist.set_config(sample_training_disturbance(self.rng,
                                                             self.ep_difficulty))
        self.dist.reset()

        # --- arena ---------------------------------------------------------
        if hard:
            start_xy = (self.rng.uniform(-1.0, 1.0), self.rng.uniform(-1.0, 1.0)) \
                if self.randomize else (0.0, 0.0)
            if self.use_obstacles:
                self.arena.randomize(robot_xy=start_xy)
            else:
                self.arena.set_obstacles(np.zeros((0, 3)))
        else:
            start_xy = (0.0, 0.0)

        # --- robot state ---------------------------------------------------
        theta0 = self.rng.uniform(-0.10, 0.10) if self.randomize else 0.03
        psi0 = self.rng.uniform(-np.pi, np.pi) if self.randomize else 0.0
        thd0 = self.rng.uniform(-0.3, 0.3) if self.randomize else 0.0
        self._plant_reset(start_xy, theta0, psi0, thd0)

        # --- controller ----------------------------------------------------
        start_gains = self.gs.nominal.copy()
        if self.randomize:
            # start from a randomly de-tuned controller so the policy has to
            # actually *do* something rather than inherit a good setpoint
            jitter = self.rng.uniform(-0.6, 0.6, size=ACT_DIM)
            start_gains = self.gs.action_to_gains(np.clip(jitter, -1.0, 1.0))
        self.pid.reset(start_gains)

        # --- bookkeeping ---------------------------------------------------
        self.script = CommandScript()
        (self._script_v, self._script_w,
         self._cmd_left) = self.script.sample(self.rng)
        self.v_ref, self.yaw_ref = self._script_v, self._script_w
        self.t = 0.0
        self.step_count = 0
        self.last_action = self.gs.gains_to_action(start_gains)
        self.last_ctrl = None
        self.last_meas = None
        self.fell = False
        self.collided = False
        self.info = {}
        return self._observe()

    # ------------------------------------------------------------------
    def set_command(self, v_ref: float | None, yaw_ref: float | None):
        """Teleop hook: overrides the scripted reference until cleared."""
        if v_ref is None and yaw_ref is None:
            self._external_cmd = None
        else:
            self._external_cmd = (float(v_ref or 0.0), float(yaw_ref or 0.0))

    def set_payload(self, mass: float, height: float | None = None):
        """设置载重（kg）和它的高度（米，轮轴以上）。下次 reset 生效。
        Set the payload; takes effect on the next reset."""
        self.payload_mass = max(0.0, float(mass))
        if height is not None:
            self.payload_height = float(height)

    def set_disturbance(self, cfg: DisturbanceConfig):
        self.dist.set_config(cfg)
        self._fixed_disturbance = True

    def fire_impulse(self, force=None, direction=None, duration=None):
        self.dist.fire_impulse(force, direction, duration)

    def set_gains(self, gains):
        self.pid.reset(np.asarray(gains, dtype=float))

    # ------------------------------------------------------------------
    # Plant hooks.  Subclasses (MuJoCo, ROS 2 / Gazebo) override *only* these
    # two methods; the controller, observation, reward and episode logic are
    # then shared verbatim, which is what makes a policy trained on the fast
    # analytic model runnable in the other two backends.
    # ------------------------------------------------------------------
    def _plant_reset(self, start_xy, theta0, psi0, thd0):
        self.state = make_state(x=start_xy[0], y=start_xy[1], psi=psi0,
                                theta=theta0, th_dot=thd0)

    def _plant_step(self, tau_l, tau_r, wrench, slip):
        """Advance the plant by exactly one control period."""
        sim = self.sim
        for _ in range(sim.phys_per_ctrl):
            self.state = self.dyn.step(self.state, tau_l, tau_r,
                                       sim.dt_sub, wrench, slip)
            self._on_substep(sim.dt_sub)

    def _on_substep(self, dt):
        """Hook fired after every integration substep.

        Sensors that filter faster than the control loop live here -- the
        MPU6050's on-chip DLPF, for one.  Sampling them once per control tick
        instead would alias chassis chatter into the attitude estimate.
        Default: nothing to do.
        """

    # ------------------------------------------------------------------
    def _advance_command(self, dt):
        if self._external_cmd is not None:
            self.v_ref, self.yaw_ref = self._external_cmd
            return
        self._cmd_left -= dt
        if self._cmd_left <= 0.0:
            (self._script_v, self._script_w,
             self._cmd_left) = self.script.sample(self.rng)
        self.v_ref, self.yaw_ref = self._avoid(self._script_v, self._script_w)

    def _avoid(self, v_ref, yaw_ref):
        """Reactive obstacle avoidance layer on top of the scripted command.

        During training nobody is driving, so without this the robot would
        walk into walls no matter how well it balanced -- and a termination
        the policy cannot influence is pure gradient noise.  This stands in
        for the human at the joystick: slow down when something is ahead,
        steer towards the freer side.  It is bypassed entirely when the UI is
        driving (``set_command``).
        """
        if not self.use_obstacles:
            return v_ref, yaw_ref
        # ray casting at the full 200 Hz control rate is pure waste -- a human
        # operator does not re-plan every 5 ms either
        self._avoid_ticks = getattr(self, "_avoid_ticks", 0) + 1
        rays = getattr(self, "_avoid_rays", None)
        if rays is None or self._avoid_ticks % 5 == 1:
            st = self.state
            rays = self.arena.raycast(st[IX], st[IY], st[IPSI])
            self._avoid_rays = rays
        n = len(rays)
        # ray 0 is straight ahead; take the forward cone
        k = max(1, n // 8)
        front = np.concatenate([rays[:k + 1], rays[-k:]]) if k else rays[:1]
        d = float(front.min())
        brake = self.arena.p.ray_max
        if d > 1.2 or v_ref <= 0.0:
            return v_ref, yaw_ref
        scale = float(np.clip((d - 0.45) / 0.75, 0.0, 1.0))
        # sectors, skipping ray 0 (dead ahead) and ray n/2 (dead astern)
        left = rays[1:n // 2].mean() if n > 3 else brake
        right = rays[n // 2 + 1:].mean() if n > 3 else brake
        turn = 1.6 * (1.0 - scale) * (1.0 if left > right else -1.0)
        return v_ref * scale, float(np.clip(yaw_ref + turn, -3.0, 3.0))

    # ------------------------------------------------------------------
    def step(self, action: np.ndarray):
        """Advance one agent tick (default 40 ms).  Returns (obs, r, done, info)."""
        sim = self.sim
        rw = self.rw
        p = self.robot

        if self.mode == MODE_PPO_GAINS:
            action = activity_gate(self._observe()) * np.clip(
                np.asarray(action, dtype=float), -1.0, 1.0)
            self.pid.set_gain_command(self.gs.action_to_gains(action))
        else:
            action = self.gs.nominal_action
            self.pid.set_gain_command(self.gs.nominal)

        acc_theta = acc_ev = acc_ey = acc_tau = acc_thd = 0.0
        n_ticks = sim.ctrl_per_agent
        obstacle_pen = 0.0

        for _ in range(n_ticks):
            self._advance_command(sim.dt_ctrl)
            self.pid.slew_gains(sim.dt_ctrl)
            self.dist.maybe_random_impulse(sim.dt_ctrl)

            st = self.state
            meas = self.dist.measure(st[ITH], st[ITHD], st[IV], st[IPSID],
                                     sim.dt_ctrl)
            ctrl = self.pid.step(meas.theta, meas.theta_dot, meas.v,
                                 meas.yaw_rate, self.v_ref, self.yaw_ref,
                                 sim.dt_ctrl)
            tau_l, tau_r = self.dist.corrupt_torque(ctrl.tau_l, ctrl.tau_r,
                                                    p.tau_max)
            wrench = self.dist.wrench(st[IPSI], sim.dt_ctrl, p.l_com)
            slip = self.dist.cfg.ground_slip

            self._plant_step(tau_l, tau_r, wrench, slip)

            self.t += sim.dt_ctrl
            self.last_ctrl = ctrl
            self.last_meas = meas

            st = self.state
            acc_theta += st[ITH] ** 2
            acc_thd += st[ITHD] ** 2
            acc_ev += abs(self.v_ref - st[IV])
            acc_ey += abs(self.yaw_ref - st[IPSID])
            acc_tau += 0.5 * ((tau_l / p.tau_max) ** 2 + (tau_r / p.tau_max) ** 2)
            obstacle_pen = max(obstacle_pen,
                               self.arena.obstacle_penalty(st[IX], st[IY],
                                                           p.collision_radius))

            # 两条摔倒判据：固件自己的角度阈值，以及车身触地。
            # 只看角度是不够的——车身用真实外形之后（前后 151.6 mm，而轮子
            # 半径只有 33.5 mm），车会先托底被地面撑住，永远到不了 40 度，
            # 于是明明趴在地上却被算成「存活」。
            # Two fall criteria: the firmware's own angle cut-out, and the
            # chassis touching the ground.  With a real-sized body the car
            # bottoms out before it can ever reach 40 degrees.
            if abs(st[ITH]) > sim.pitch_fail or p.bottoms_out(float(st[ITH])):
                self.fell = True
                break
            if self.use_obstacles and self.arena.in_collision(
                    st[IX], st[IY], p.collision_radius):
                self.collided = True
                break

        k = max(1, n_ticks)
        mean_theta2 = acc_theta / k

        reward = (
            rw.alive
            + rw.upright * (1.0 - mean_theta2 / sim.pitch_fail ** 2)
            - rw.vel_track * (acc_ev / k)
            - rw.yaw_track * (acc_ey / k)
            - rw.torque * (acc_tau / k)
            - rw.pitch_rate * (acc_thd / k)
            - rw.gain_rate * float(np.mean((action - self.last_action) ** 2))
            # action == 0 is the nominal PID, so ||action||^2 is literally
            # "how far did I move the gains".  Without this the policy pays
            # nothing for fidgeting, and in clean conditions -- where nominal
            # is already near-optimal -- fidgeting is all it can do.
            - rw.gain_dev * float(np.mean(action ** 2))
            - rw.obstacle * obstacle_pen
        )

        terminated = False
        if self.fell:
            reward -= rw.fall_penalty
            terminated = True
        elif self.collided:
            reward -= rw.collision_penalty
            terminated = True

        self.last_action = action
        self.step_count += 1
        truncated = self.step_count >= sim.max_agent_steps

        self.info = {
            "t": self.t,
            "fell": self.fell,
            "collided": self.collided,
            "pitch": float(self.state[ITH]),
            "v": float(self.state[IV]),
            "yaw_rate": float(self.state[IPSID]),
            "v_ref": self.v_ref,
            "yaw_ref": self.yaw_ref,
            "gains": self.pid.gains.copy(),
            "obstacle_pen": obstacle_pen,
        }
        return self._observe(), float(reward), terminated, truncated, self.info

    # ------------------------------------------------------------------
    def _observe(self) -> np.ndarray:
        st = self.state
        p = self.robot
        sim = self.sim
        pid = self.pid
        meas = self.last_meas
        ctrl = self.last_ctrl

        theta = meas.theta if meas else st[ITH]
        theta_dot = meas.theta_dot if meas else st[ITHD]
        v = meas.v if meas else st[IV]
        yaw_rate = meas.yaw_rate if meas else st[IPSID]

        kp_p, ki_p, kd_p, kp_v, ki_v, kd_v, kp_y, ki_y, kd_y = pid.gains
        tau_l = ctrl.tau_l if ctrl else 0.0
        tau_r = ctrl.tau_r if ctrl else 0.0
        pitch_ref = ctrl.pitch_ref if ctrl else 0.0
        sat = 1.0 if (ctrl and ctrl.saturated) else 0.0

        clearance = self.arena.clearance(st[IX], st[IY]) - p.collision_radius

        base = np.array([
            theta / sim.pitch_fail,
            theta_dot / 6.0,
            v / 1.5,
            self.v_ref / 1.5,
            (self.v_ref - v) / 1.5,
            yaw_rate / 4.0,
            self.yaw_ref / 4.0,
            (self.yaw_ref - yaw_rate) / 4.0,
            pitch_ref / 0.25,
            np.tanh(ki_p * pid.i_pitch / 0.7),
            np.tanh(ki_v * pid.i_vel / 0.2),
            np.tanh(ki_y * pid.i_yaw / 0.3),
            tau_l / p.tau_max,
            tau_r / p.tau_max,
            sat,
            np.tanh(clearance / 1.0),
        ], dtype=np.float64)

        rays = self.arena.normalized_rays(st[IX], st[IY], st[IPSI])
        gains_n = self.gs.normalize(pid.gains)
        obs = np.concatenate([base, rays, gains_n])
        return np.clip(np.nan_to_num(obs), -10.0, 10.0).astype(np.float32)

    # ------------------------------------------------------------------
    def render_state(self):
        """Everything the UI / 3D viewers need, in one dict."""
        st = self.state
        return {
            "x": float(st[IX]), "y": float(st[IY]),
            "psi": float(st[IPSI]), "psi_dot": float(st[IPSID]),
            "theta": float(st[ITH]), "theta_dot": float(st[ITHD]),
            "v": float(st[IV]),
            "t": self.t,
            "gains": self.pid.gains.copy(),
            "tau": (self.last_ctrl.tau_l if self.last_ctrl else 0.0,
                    self.last_ctrl.tau_r if self.last_ctrl else 0.0),
            "v_ref": self.v_ref, "yaw_ref": self.yaw_ref,
            "fell": self.fell, "collided": self.collided,
            "obstacles": self.arena.obstacles,
        }


# ----------------------------------------------------------------------
# Gymnasium wrapper (only imported when gymnasium is installed)
# ----------------------------------------------------------------------
def make_gym_env(**kwargs):
    """Factory returning a Gymnasium-compatible env."""
    import gymnasium as gym
    from gymnasium import spaces

    class BalanceBotEnv(gym.Env):
        metadata = {"render_modes": []}

        def __init__(self, **kw):
            super().__init__()
            self.core = BalanceCore(**kw)
            self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
            self.observation_space = spaces.Box(
                -10.0, 10.0, (self.core.obs_dim,), np.float32)

        def reset(self, *, seed=None, options=None):
            super().reset(seed=seed)
            obs = self.core.reset(seed=seed)
            return obs, {}

        def step(self, action):
            obs, r, term, trunc, info = self.core.step(action)
            info = {k: v for k, v in info.items() if k != "gains"}
            return obs, r, term, trunc, info

        def set_difficulty(self, d):
            self.core.difficulty = float(np.clip(d, 0.0, 1.0))

    return BalanceBotEnv(**kwargs)
