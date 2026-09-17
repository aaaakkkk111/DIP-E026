"""Interactive tuning window for the STM32 twin.

    python scripts\\tune_gui.py                          MuJoCo 3D + panel, LQR
    python scripts\\tune_gui.py --firmware stm32_pid     the PID build instead
    python scripts\\tune_gui.py --backend analytic       no MuJoCo, 2D views only
    python scripts\\tune_gui.py --params tuned_lqr.json  start from a saved set

Drag a gain slider, shove the car with the KICK button, watch what happens.
Every slider is a multiplier on the shipped firmware value (0.25x .. 4x), so
the centre position is the firmware and the axis is the same one the CEM
tuner searches.

What you see here is ONE episode with whatever disturbances you dialled in.
It is for building intuition, not for deciding anything -- when a setting
looks good, save it and score it on 40 held-out episodes:

    python scripts\\bench_twin_baseline.py --params tuned_by_hand.json --episodes 40
"""
from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from balance_bot.firmware.twin_baseline import (STM32Twin, make_mujoco_twin,     # noqa: E402
                                       MODE_STM32_LQR, MODE_STM32_PID)
from balance_bot.params import ArenaParams, DisturbanceConfig           # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--firmware", choices=(MODE_STM32_LQR, MODE_STM32_PID),
                    default=MODE_STM32_LQR)
    ap.add_argument("--backend", choices=("mujoco", "analytic"),
                    default="mujoco")
    ap.add_argument("--params", default="",
                    help="start from a saved parameter JSON")
    ap.add_argument("--arena", type=float, default=2.0,
                    help="arena half-size in metres (this car is small)")
    ap.add_argument("--obstacles", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=600.0,
                    help="long by default: you want it to keep running")
    args = ap.parse_args()

    gains = None
    if args.params:
        with open(args.params, encoding="utf-8") as f:
            spec = json.load(f)
        if spec.get("firmware") not in (None, args.firmware):
            print(f"注意：{args.params} 是 {spec['firmware']} 的参数，"
                  f"而 --firmware 是 {args.firmware}，已忽略。")
        else:
            gains = spec.get("gains")

    kw = dict(firmware=args.firmware, gains=gains,
              randomize=False, obstacles=args.obstacles > 0,
              disturbance=DisturbanceConfig(),
              arena=ArenaParams(half_x=args.arena, half_y=args.arena,
                                n_obstacles=args.obstacles),
              episode_seconds=args.episode_seconds)

    viewer = None
    if args.backend == "mujoco":
        core = make_mujoco_twin(**kw)
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(core.model, core.data,
                                                  show_left_ui=False,
                                                  show_right_ui=False)
            # this car is 8 cm tall; the default 3 m camera makes it a dot
            viewer.cam.distance = 0.9
            viewer.cam.elevation = -18
        except Exception as e:
            print(f"[warn] MuJoCo 窗口打不开（{e}），退回内置 2D 视图")
            viewer = None
    else:
        core = STM32Twin(**kw)

    core.set_command(0.0, 0.0)

    from balance_bot.ui.twin_baseline_panel import run_twin_baseline_panel
    sys.exit(run_twin_baseline_panel(core, viewer,
                            title=f"STM32 孪生调参  [{args.firmware} / "
                                  f"{args.backend}]"))


if __name__ == "__main__":
    main()
