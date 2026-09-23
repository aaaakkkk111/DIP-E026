"""Interactive driving demo with adjustable real-world conditions.

Drive with the arrow keys in the viewer window. Floor friction, ground slope
and payload mass are set on the command line and can be changed live by typing
commands into the terminal while the viewer keeps running:

    f 0.4      floor friction -> 0.4
    s 5        slope -> 5 degrees uphill (negative = downhill)
    p 0.15     payload -> 0.15 kg
    r          reset the robot where it stands
    ?          reprint the current conditions
    q          quit

Live adjustment goes through the terminal rather than the keyboard because
MuJoCo's viewer already binds every letter key to a rendering toggle (W
wireframe, A auto-connect, S shadows, D static-body visibility, ...) and fires
our callback *in addition* to its own, so a letter shortcut would silently
change the render state too. The arrow keys are safe: their built-in bindings
(playback speed / frame step) are no-ops here because physics is stepped
manually.
"""
import argparse
import math
import os
import threading
import time

import glfw
import mujoco
import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO

from train_yahboom_3d import YahboomEnv, VelocityCommandWrapper

# Matches the command ranges sampled during training (train_yahboom_3d.py).
# Commanding anything larger asks the policy to track targets it never saw.
MAX_V_FORWARD = 0.3
MAX_V_TURN = 0.5

# What the policy actually saw while training, used to flag settings that ask
# it to generalise beyond its experience. Friction was fixed at the XML value
# and the ground was always level, so ANY change to those two is extrapolation;
# payload was curriculum-ramped across 0-0.2 kg and is the only one of the
# three with genuine in-distribution range.
TRAINED_FRICTION = 0.90
TRAINED_SLOPE_DEG = 0.0
TRAINED_PAYLOAD_KG = (0.0, 0.2)

GRAVITY = 9.81
FRICTION_GEOMS = ("floor", "gw_l", "gw_r")


def apply_friction(model, mu):
    """Set sliding friction on the floor *and* both wheels.

    MuJoCo combines a contact pair's friction with an element-wise max, so
    lowering only the floor would leave the wheels' 0.90 in charge and change
    nothing. Both sides have to come down together.
    """
    for name in FRICTION_GEOMS:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid >= 0:
            model.geom_friction[gid, 0] = mu


def apply_slope(model, degrees):
    """Emulate an incline by tilting gravity instead of the floor.

    Rotating the ground plane would spawn the robot partially inside it and
    need a settling transient. Tilting gravity is the same physics expressed in
    the slope's own frame - the magnitude of g is unchanged, and the robot
    feels exactly the downhill pull and normal load it would on a real incline.
    The trade-off is visual: the floor still *looks* level in the viewer.

    Positive = uphill along +x, which is the robot's forward axis at zero yaw.
    Because the tilt lives in the world frame, turning changes how the robot
    meets the slope (a climb becomes a side-bank at 90 degrees of yaw), which
    is what a real fixed slope does.
    """
    th = math.radians(degrees)
    model.opt.gravity[:] = [-GRAVITY * math.sin(th), 0.0, -GRAVITY * math.cos(th)]


def apply_payload(model, body_id, kg):
    """Set payload mass. Re-applied after every reset, which randomises it."""
    model.body_mass[body_id] = kg


def describe(state):
    mu, slope, payload = state["friction"], state["slope"], state["payload"]
    lo, hi = TRAINED_PAYLOAD_KG
    flag_mu = "" if abs(mu - TRAINED_FRICTION) < 1e-9 else "  <- not trained on"
    flag_sl = "" if abs(slope - TRAINED_SLOPE_DEG) < 1e-9 else "  <- not trained on"
    flag_pl = "" if lo <= payload <= hi else "  <- outside training range"
    return (
        "\n  friction {:5.2f}     (trained: {:.2f}){}"
        "\n  slope    {:5.1f} deg (trained: level){}"
        "\n  payload  {:5.2f} kg  (trained: {:.1f}-{:.1f}){}\n"
    ).format(mu, TRAINED_FRICTION, flag_mu, slope, flag_sl, payload, lo, hi, flag_pl)


def handle_command(line, state, model, payload_id):
    """Parse one terminal command. Returns False if the user asked to quit."""
    parts = line.strip().split()
    if not parts:
        return True
    cmd = parts[0].lower()
    arg = parts[1] if len(parts) > 1 else None

    if cmd == "q":
        state["quit"] = True
        return False
    if cmd == "r":
        state["reset"] = True
        print("  reset requested")
        return True
    if cmd == "?":
        print(describe(state))
        return True

    if arg is None:
        print("  '{}' needs a value, e.g. '{} 0.5'".format(cmd, cmd))
        return True
    try:
        value = float(arg)
    except ValueError:
        print("  not a number: {}".format(arg))
        return True

    if cmd == "f":
        state["friction"] = float(np.clip(value, 0.0, 2.0))
        apply_friction(model, state["friction"])
        print("  friction -> {:.2f}".format(state["friction"]))
    elif cmd == "s":
        state["slope"] = float(np.clip(value, -30.0, 30.0))
        apply_slope(model, state["slope"])
        print("  slope -> {:.1f} deg".format(state["slope"]))
    elif cmd == "p":
        state["payload"] = float(np.clip(value, 0.0, 2.0))
        apply_payload(model, payload_id, state["payload"])
        print("  payload -> {:.2f} kg".format(state["payload"]))
    else:
        print("  unknown command {}  (f / s / p / r / ? / q)".format(cmd))
    return True


def console_loop(state, model, payload_id):
    """Read adjustment commands from the terminal on a background thread."""
    while not state["quit"]:
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            state["quit"] = True
            return
        handle_command(line, state, model, payload_id)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--friction", type=float, default=TRAINED_FRICTION,
                        help="floor/wheel sliding friction (default 0.90, as trained)")
    parser.add_argument("--slope", type=float, default=0.0,
                        help="ground slope in degrees, positive = uphill along +x (default 0)")
    parser.add_argument("--payload", type=float, default=0.0,
                        help="payload mass in kg (default 0.0; trained across 0.0-0.2)")
    parser.add_argument("--model", default="models/best_their/best_model.zip",
                        help="policy checkpoint to drive with")
    args = parser.parse_args()

    # models/best/ held the my_robot.xml policies. They have the same 17-dim
    # observation shape so they would load without error against their_robot.xml
    # and just saturate its +/-0.6 Nm motors with +/-4.0 Nm commands, which looks
    # like a bad policy rather than a mismatched one. Refuse instead of guessing.
    if not os.path.exists(args.model):
        raise SystemExit(
            "{} not found.\n"
            "Training was retargeted to their_robot.xml (see train_yahboom_3d.py);\n"
            "run train_yahboom_3d.py to produce a checkpoint for that plant.".format(args.model))
    model_ppo = PPO.load(args.model)

    # max_payload_kg=0.0 so the wrapper's reset zeroes the payload; the value
    # requested here is re-applied on top of every reset instead.
    env = VelocityCommandWrapper(YahboomEnv(), is_eval=True, max_payload_kg=0.0)
    obs, _ = env.reset()

    mj_model = env.unwrapped.model
    payload_id = env.payload_body_id

    trained_hi = float(np.max(model_ppo.action_space.high))
    if not np.isclose(trained_hi, env.tau_max, atol=1e-3):
        raise SystemExit(
            "Action-space mismatch: checkpoint limit +/-{:.2f} Nm, plant limit "
            "+/-{:.2f} Nm.\n{} belongs to a different robot.".format(
                trained_hi, env.tau_max, args.model))

    state = {"friction": float(np.clip(args.friction, 0.0, 2.0)),
             "slope": float(np.clip(args.slope, -30.0, 30.0)),
             "payload": float(np.clip(args.payload, 0.0, 2.0)),
             "quit": False,
             "reset": False}
    apply_friction(mj_model, state["friction"])
    apply_slope(mj_model, state["slope"])
    apply_payload(mj_model, payload_id, state["payload"])

    # MuJoCo's viewer runs its own built-in keyboard shortcuts alongside any
    # custom key_callback rather than replacing them, and every letter key is
    # already bound to a rendering/visualization toggle - hence the display
    # glitches when WASD was used here. Arrow keys avoid that collision.
    def key_callback(keycode):
        if keycode == glfw.KEY_UP:
            env.target_v_forward = 0.0 if env.target_v_forward > 0 else MAX_V_FORWARD
        elif keycode == glfw.KEY_DOWN:
            env.target_v_forward = 0.0 if env.target_v_forward < 0 else -MAX_V_FORWARD
        elif keycode == glfw.KEY_LEFT:
            env.target_v_turn = 0.0 if env.target_v_turn > 0 else MAX_V_TURN
        elif keycode == glfw.KEY_RIGHT:
            env.target_v_turn = 0.0 if env.target_v_turn < 0 else -MAX_V_TURN

    print("\n--- DRIVING CONTROLS (click the viewer window first) ---")
    print("Up/Down toggle forward/backward, Left/Right toggle turn")
    print("\n--- LIVE CONDITIONS (type here in the terminal, then Enter) ---")
    print("f <mu>   friction     s <deg>  slope       p <kg>  payload")
    print("r        reset        ?        show        q       quit")
    print(describe(state))

    threading.Thread(target=console_loop, args=(state, mj_model, payload_id),
                     daemon=True).start()

    with mujoco.viewer.launch_passive(mj_model, env.unwrapped.data,
                                      key_callback=key_callback) as viewer:
        while viewer.is_running() and not state["quit"]:
            action, _ = model_ppo.predict(obs, deterministic=True)
            obs, _, done, _, _ = env.step(action)
            viewer.sync()

            if done or state["reset"]:
                # A stumble past the fall threshold triggers an internal reset,
                # which would otherwise silently wipe whatever direction is
                # currently held (reset() zeroes targets in is_eval mode) and
                # re-randomise the payload. Preserve both across it.
                held_forward, held_turn = env.target_v_forward, env.target_v_turn
                obs, _ = env.reset()
                env.target_v_forward, env.target_v_turn = held_forward, held_turn
                apply_payload(mj_model, payload_id, state["payload"])
                state["reset"] = False
            time.sleep(0.005)

    state["quit"] = True


if __name__ == "__main__":
    main()
