"""Interactive driving demo with adjustable real-world conditions.

Opens two windows: the MuJoCo viewer showing the robot, and a control panel
with sliders for the drive commands and for the physical conditions (floor
friction, ground slope, payload mass). Everything is adjustable while the
simulation runs.

    python enjoy_drive.py                       # sliders (default)
    python enjoy_drive.py --slope 5             # sliders, starting at 5 deg
    python enjoy_drive.py --no-gui              # arrow keys + terminal commands

Driving is done from the panel's sliders rather than the keyboard so you never
have to move focus between the two windows. The arrow keys still work in the
viewer window (Up/Down forward/back, Left/Right turn) and stay in sync with the
sliders. Letter keys are deliberately unused: MuJoCo's viewer binds every one
of them to a rendering toggle (W wireframe, A auto-connect, S shadows, D
static-body visibility, ...) and fires our callback *in addition* to its own,
so a letter shortcut would silently change the render state too.
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


class ControlPanel:
    """Tkinter slider panel driving both the commands and the conditions.

    Built with tk.Scale rather than ttk.Scale because it shows its own value
    and supports `resolution`, which saves a pile of label-syncing code.
    """

    BG = "#1e1e2e"
    FG = "#cdd6f4"
    ACCENT = "#89b4fa"
    WARN = "#f9e2af"

    def __init__(self, state, model, payload_id):
        import tkinter as tk
        self.tk = tk
        self.state = state
        self.model = model
        self.payload_id = payload_id
        self.alive = True

        self.root = tk.Tk()
        self.root.title("Robot conditions")
        self.root.configure(bg=self.BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._section("DRIVE COMMAND")
        self.s_fwd = self._slider("Forward  (m/s)", -MAX_V_FORWARD, MAX_V_FORWARD,
                                  0.01, state["cmd_forward"], self._on_forward)
        self.s_turn = self._slider("Turn  (rad/s)", -MAX_V_TURN, MAX_V_TURN,
                                   0.01, state["cmd_turn"], self._on_turn)

        self._section("CONDITIONS")
        self.s_fric = self._slider("Friction        (trained 0.90)", 0.0, 1.5,
                                   0.05, state["friction"], self._on_friction)
        self.s_slope = self._slider("Slope  deg      (trained 0)", -15.0, 15.0,
                                    0.5, state["slope"], self._on_slope)
        self.s_load = self._slider("Payload  kg     (trained 0-0.2)", 0.0, 1.5,
                                   0.05, state["payload"], self._on_payload)

        row = tk.Frame(self.root, bg=self.BG)
        row.pack(fill="x", padx=10, pady=(4, 8))
        for text, cmd in (("Reset robot", self._on_reset),
                          ("Stop", self._on_stop),
                          ("Back to trained", self._on_defaults)):
            tk.Button(row, text=text, command=cmd, bg="#313244", fg=self.FG,
                      activebackground=self.ACCENT, relief="flat",
                      padx=8, pady=3).pack(side="left", padx=3)

        self._section("TELEMETRY")
        self.telemetry = tk.Label(self.root, text="", bg=self.BG, fg=self.FG,
                                  font=("Consolas", 10), justify="left", anchor="w")
        self.telemetry.pack(fill="x", padx=12, pady=(0, 10))

    def _section(self, title):
        self.tk.Label(self.root, text=title, bg=self.BG, fg=self.ACCENT,
                      font=("Segoe UI", 9, "bold"), anchor="w").pack(
                          fill="x", padx=10, pady=(10, 0))

    def _slider(self, label, lo, hi, res, init, cb):
        s = self.tk.Scale(self.root, from_=lo, to=hi, resolution=res,
                          orient="horizontal", label=label, command=cb,
                          length=330, bg=self.BG, fg=self.FG,
                          troughcolor="#313244", highlightthickness=0,
                          font=("Segoe UI", 8))
        s.set(init)
        s.pack(fill="x", padx=10)
        return s

    # --- slider callbacks; tk passes the value as a string ---------------
    def _on_forward(self, v):
        self.state["cmd_forward"] = float(v)

    def _on_turn(self, v):
        self.state["cmd_turn"] = float(v)

    def _on_friction(self, v):
        self.state["friction"] = float(v)
        apply_friction(self.model, self.state["friction"])

    def _on_slope(self, v):
        self.state["slope"] = float(v)
        apply_slope(self.model, self.state["slope"])

    def _on_payload(self, v):
        self.state["payload"] = float(v)
        apply_payload(self.model, self.payload_id, self.state["payload"])

    def _on_reset(self):
        self.state["reset"] = True

    def _on_stop(self):
        self.s_fwd.set(0.0)
        self.s_turn.set(0.0)

    def _on_defaults(self):
        self.s_fric.set(TRAINED_FRICTION)
        self.s_slope.set(TRAINED_SLOPE_DEG)
        self.s_load.set(0.0)

    def _on_close(self):
        self.alive = False
        self.state["quit"] = True
        try:
            self.root.destroy()
        except Exception:
            pass

    def sync_commands(self):
        """Push slider values onto the env (called every physics tick)."""
        return self.state["cmd_forward"], self.state["cmd_turn"]

    def set_commands(self, forward, turn):
        """Reflect an arrow-key change back onto the sliders."""
        self.s_fwd.set(forward)
        self.s_turn.set(turn)

    def update_telemetry(self, v_fwd, v_turn, pitch, steps, fell):
        lo, hi = TRAINED_PAYLOAD_KG
        warn = []
        if abs(self.state["friction"] - TRAINED_FRICTION) > 1e-9:
            warn.append("friction")
        if abs(self.state["slope"] - TRAINED_SLOPE_DEG) > 1e-9:
            warn.append("slope")
        if not (lo <= self.state["payload"] <= hi):
            warn.append("payload")
        note = ("outside training: " + ", ".join(warn)) if warn else "all within training"
        self.telemetry.configure(
            text=("forward  {:+.3f} / {:+.3f} m/s\n"
                  "yaw      {:+.3f} / {:+.3f} rad/s\n"
                  "pitch    {:+.3f} rad\n"
                  "steps    {:d}{}\n"
                  "{}").format(v_fwd, self.state["cmd_forward"],
                               v_turn, self.state["cmd_turn"],
                               pitch, steps, "   (FELL)" if fell else "", note),
            fg=self.WARN if warn else self.FG)

    def pump(self):
        """Service the GUI event queue. False once the window is gone."""
        if not self.alive:
            return False
        try:
            self.root.update()
        except Exception:
            self.alive = False
            return False
        return True


def handle_command(line, state, model, payload_id):
    """Parse one terminal command (--no-gui mode). False means quit."""
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
    parser.add_argument("--no-gui", action="store_true",
                        help="skip the slider panel; use arrow keys + terminal commands")
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
    policy = PPO.load(args.model)

    # max_payload_kg=0.0 so the wrapper's reset zeroes the payload; the value
    # requested here is re-applied on top of every reset instead.
    env = VelocityCommandWrapper(YahboomEnv(), is_eval=True, max_payload_kg=0.0)
    obs, _ = env.reset()

    mj_model = env.unwrapped.model
    payload_id = env.payload_body_id

    trained_hi = float(np.max(policy.action_space.high))
    if not np.isclose(trained_hi, env.tau_max, atol=1e-3):
        raise SystemExit(
            "Action-space mismatch: checkpoint limit +/-{:.2f} Nm, plant limit "
            "+/-{:.2f} Nm.\n{} belongs to a different robot.".format(
                trained_hi, env.tau_max, args.model))

    state = {"friction": float(np.clip(args.friction, 0.0, 2.0)),
             "slope": float(np.clip(args.slope, -30.0, 30.0)),
             "payload": float(np.clip(args.payload, 0.0, 2.0)),
             "cmd_forward": 0.0,
             "cmd_turn": 0.0,
             "cmd_dirty": False,
             "quit": False,
             "reset": False}
    apply_friction(mj_model, state["friction"])
    apply_slope(mj_model, state["slope"])
    apply_payload(mj_model, payload_id, state["payload"])

    panel = None
    if not args.no_gui:
        try:
            panel = ControlPanel(state, mj_model, payload_id)
        except Exception as exc:  # tkinter missing or no display
            print("Could not open the slider panel ({}); "
                  "falling back to terminal commands.".format(exc))

    # Arrow keys remain available in the viewer window. MuJoCo's viewer runs its
    # built-in shortcuts alongside any custom key_callback rather than replacing
    # them, and every letter key is already bound to a rendering toggle - hence
    # the display glitches when WASD was used here. Arrow keys avoid that.
    def key_callback(keycode):
        # Runs on MuJoCo's RENDER thread, not the main thread. It must not
        # touch tkinter: every Tk call has to come from the thread that created
        # the root window, and calling Scale.set() from here raises
        # "main thread is not in main loop", which kills the render loop and
        # freezes the viewer. So only plain state is mutated, and the main loop
        # pushes the new values onto the sliders when it sees cmd_dirty.
        try:
            if keycode == glfw.KEY_UP:
                state["cmd_forward"] = 0.0 if state["cmd_forward"] > 0 else MAX_V_FORWARD
            elif keycode == glfw.KEY_DOWN:
                state["cmd_forward"] = 0.0 if state["cmd_forward"] < 0 else -MAX_V_FORWARD
            elif keycode == glfw.KEY_LEFT:
                state["cmd_turn"] = 0.0 if state["cmd_turn"] > 0 else MAX_V_TURN
            elif keycode == glfw.KEY_RIGHT:
                state["cmd_turn"] = 0.0 if state["cmd_turn"] < 0 else -MAX_V_TURN
            else:
                return
            state["cmd_dirty"] = True
        except Exception:
            # A raise here would take the render thread down with it.
            pass

    if panel is None:
        print("\n--- DRIVING CONTROLS (click the viewer window first) ---")
        print("Up/Down toggle forward/backward, Left/Right toggle turn")
        print("\n--- LIVE CONDITIONS (type here in the terminal, then Enter) ---")
        print("f <mu>   friction     s <deg>  slope       p <kg>  payload")
        print("r        reset        ?        show        q       quit")
        print(describe(state))
        threading.Thread(target=console_loop, args=(state, mj_model, payload_id),
                         daemon=True).start()

    steps = 0
    fell = False
    # One control step is FRAME_SKIP * timestep of sim time; pace the loop to
    # that so the demo runs at roughly real time instead of as fast as the CPU
    # allows, which is far too quick to watch or steer.
    period = env.unwrapped.frame_skip * mj_model.opt.timestep
    with mujoco.viewer.launch_passive(mj_model, env.unwrapped.data,
                                      key_callback=key_callback) as viewer:
        while viewer.is_running() and not state["quit"]:
            tick = time.perf_counter()
            env.target_v_forward = state["cmd_forward"]
            env.target_v_turn = state["cmd_turn"]

            action, _ = policy.predict(obs, deterministic=True)
            obs, _, done, _, _ = env.step(action)
            steps += 1
            viewer.sync()

            if panel is not None:
                # Arrow keys are handled on the render thread and can only set
                # a flag; moving the sliders has to happen here, on the thread
                # that owns the Tk window.
                if state["cmd_dirty"]:
                    state["cmd_dirty"] = False
                    panel.set_commands(state["cmd_forward"], state["cmd_turn"])

                d = env.unwrapped.data
                _, pitch, local_vel, v_turn = env._decode_state(
                    np.concatenate([d.qpos, d.qvel]))
                if steps % 5 == 0:  # ~16Hz refresh, keeps the sim smooth
                    panel.update_telemetry(local_vel[0], v_turn, pitch, steps, fell)
                if not panel.pump():
                    break

            if done or state["reset"]:
                # A stumble past the fall threshold triggers an internal reset,
                # which would otherwise re-randomise the payload. Commands live
                # in `state` now, so they survive on their own.
                fell = done and not state["reset"]
                obs, _ = env.reset()
                apply_payload(mj_model, payload_id, state["payload"])
                state["reset"] = False
                steps = 0

            lag = period - (time.perf_counter() - tick)
            if lag > 0:
                time.sleep(lag)

    state["quit"] = True
    if panel is not None and panel.alive:
        try:
            panel.root.destroy()
        except Exception:
            pass


if __name__ == "__main__":
    main()
