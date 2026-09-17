"""balance-bot: two-wheeled inverted-pendulum robot simulation.

A cascade PID keeps the robot upright at 200 Hz; a PPO agent rewrites its
gains at 25 Hz.  Two interchangeable physics backends (analytic / MuJoCo)
share one controller, one observation vector and one policy file.

The ``firmware`` subpackage builds on all of this to reproduce a specific
real product -- the Yahboom STM32 balance car -- bit for bit; see TWIN_BASELINE.md.

Quick start::

    from balance_bot.env import BalanceCore, MODE_PID
    core = BalanceCore(mode=MODE_PID, seed=0)
    obs = core.reset()
    obs, reward, terminated, truncated, info = core.step(action)
"""

__version__ = "0.1.0"

from .params import (ArenaParams, DisturbanceConfig, GainSpace,   # noqa: F401
                     RewardWeights, RobotParams, SimParams, GAIN_NAMES)

__all__ = [
    "ArenaParams", "DisturbanceConfig", "GainSpace", "RewardWeights",
    "RobotParams", "SimParams", "GAIN_NAMES", "__version__",
]
