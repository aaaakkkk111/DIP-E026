import os
import time

import glfw
import mujoco.viewer
from stable_baselines3 import PPO

from train_yahboom_3d import YahboomEnv, VelocityCommandWrapper

# Matches the command ranges sampled during training (train_yahboom_3d.py).
# Commanding anything larger asks the policy to track targets it never saw.
MAX_V_FORWARD = 0.3
MAX_V_TURN = 0.5


def main():
    # models/best/ holds the my_robot.xml policies. They have the same 17-dim
    # observation shape so they would load without error against their_robot.xml
    # and just saturate its +/-0.6 Nm motors with +/-4.0 Nm commands, which looks
    # like a bad policy rather than a mismatched one. Refuse instead of guessing.
    model_path = "models/best_their/best_model.zip"
    if not os.path.exists(model_path):
        raise SystemExit(
            f"{model_path} not found.\n"
            "Training was retargeted to their_robot.xml (see train_yahboom_3d.py);\n"
            "run train_yahboom_3d.py to produce a checkpoint for that plant.\n"
            "The older models/best/ checkpoints belong to my_robot.xml and are not\n"
            "valid here - its action space was +/-4.0 Nm against this model's +/-0.6."
        )
    model = PPO.load(model_path)

    # Empty payload for testing (try 0.2 to see it balance a heavy load!)
    env = VelocityCommandWrapper(YahboomEnv(), is_eval=True, max_payload_kg=0.0)
    obs, _ = env.reset()

    # MuJoCo's viewer runs its own built-in keyboard shortcuts alongside any
    # custom key_callback rather than replacing them, and every letter key is
    # already bound to a rendering/visualization toggle (W=wireframe, A=auto
    # connect, S=shadow, D=static body visibility, etc.) - hence the display
    # glitches when WASD was used here. Arrow keys avoid that collision: their
    # built-in bindings (playback speed / frame-step while paused) are no-ops
    # in this script since physics is stepped manually via env.step().
    def key_callback(keycode):
        if keycode == glfw.KEY_UP:
            env.target_v_forward = 0.0 if env.target_v_forward > 0 else MAX_V_FORWARD
        elif keycode == glfw.KEY_DOWN:
            env.target_v_forward = 0.0 if env.target_v_forward < 0 else -MAX_V_FORWARD
        elif keycode == glfw.KEY_LEFT:
            env.target_v_turn = 0.0 if env.target_v_turn > 0 else MAX_V_TURN
        elif keycode == glfw.KEY_RIGHT:
            env.target_v_turn = 0.0 if env.target_v_turn < 0 else -MAX_V_TURN

    print("\n--- DRIVING CONTROLS ---")
    print("Click the viewer window, then: Up/Down toggle forward/backward, Left/Right toggle turn")

    with mujoco.viewer.launch_passive(env.unwrapped.model, env.unwrapped.data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            action, _ = model.predict(obs, deterministic=True)
            obs, _, done, _, _ = env.step(action)
            viewer.sync()

            if done:
                # A stumble past the fall threshold triggers an internal
                # reset, which would otherwise silently wipe whatever
                # direction is currently held (reset() zeroes targets in
                # is_eval mode). Preserve the held command across it.
                held_forward, held_turn = env.target_v_forward, env.target_v_turn
                obs, _ = env.reset()
                env.target_v_forward, env.target_v_turn = held_forward, held_turn
            time.sleep(0.005)


if __name__ == "__main__":
    main()
