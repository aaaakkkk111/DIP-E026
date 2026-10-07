"""Export the trained PPO policy to a C header for STM32 deployment.

Emits `policy_weights.h`, a single self-contained header holding every weight
as `static const float` arrays so they land in flash rather than RAM.

Only the ACTOR is exported. PPO also trains a value network, but that is used
solely to compute advantages during learning and plays no part in choosing an
action, so it is dead weight on the microcontroller.

The exported network is N_OBS -> 64 -> 64 -> N_ACT with tanh activations,
both dynamic (read from the checkpoint, not assumed):

    h1 = tanh(W0 . obs + b0)
    h2 = tanh(W1 . h1  + b1)
    a  = clamp(W2 . h2 + b2, -POLICY_ACTION_LIMIT, +POLICY_ACTION_LIMIT)

The action limit is generic on purpose: for the current PWM-output policy
(train_real_robot.py, real_robot.xml) it is 1.0, a duty fraction; for the
superseded torque-output policies it was 0.4-0.6 N.m. `policy.c` treats
whichever number comes out as "the trained action-space bound", not as N.m.

Usage:
    python export_stm32.py                       # from models/best_real
    python export_stm32.py --model path/to.zip --out firmware/policy_weights.h
"""
import argparse
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from train_real_robot import HISTORY_TAPS, N_HISTORY_SIGNALS

# The base 17 inputs have been unchanged since the very first VelocityCommandWrapper
# run (train_yahboom_3d.py) and are shared by every checkpoint in this project's
# history, torque or PWM, 80 Hz or 200 Hz - so unlike the history/integral block
# below, these are safe to hand-list rather than import.
BASE_OBS_NAMES = [
    "z_height          constant 0.0334 (not sensed; training feeds the same)",
    "quat_w            from fused roll/pitch, yaw pinned to 0",
    "quat_x            from fused roll/pitch, yaw pinned to 0",
    "quat_y            from fused roll/pitch, yaw pinned to 0",
    "quat_z            from fused roll/pitch, yaw pinned to 0",
    "wheel_angle_L     always 0 since run 8 (dropped)",
    "wheel_angle_R     always 0 since run 8 (dropped)",
    "v_forward         r*(mean wheel rate + pitch rate)*cos(pitch), clip +/-2 m/s",
    "v_lateral         (NOT sensed - use 0)",
    "v_vertical        (NOT sensed - use 0)",
    "gyro_roll         rad/s",
    "gyro_pitch        rad/s",
    "gyro_yaw          rad/s",
    "wheel_vel_L       rad/s, encoder counts over 4 ticks, clip +/-40",
    "wheel_vel_R       rad/s, encoder counts over 4 ticks, clip +/-40",
    "cmd_forward       commanded body-frame velocity, m/s",
    "cmd_turn          commanded yaw rate, rad/s",
]

# 200 Hz control (real_robot.xml: 1 ms timestep x FRAME_SKIP=5), so one tick is
# 5 ms. Only used to label tap ages in the generated header comment - HISTORY_TAPS
# itself is imported, so the ages below can never drift from what the policy
# was actually trained against the way the old hand-copied comment did.
MS_PER_TICK = 5


def obs_names(n_obs):
    """Build the observation-name list for whichever checkpoint was loaded.

    Every checkpoint in this project shares the same 17-input base. Dimension
    alone then tells you the rest: 32 adds the history taps (run 3+), 33 adds
    the station-keeping integral (run 6+), 34 adds the heading-keeping integral
    (run 7+). A bare 17-input checkpoint predates all of that.
    """
    names = list(BASE_OBS_NAMES)
    if n_obs >= len(BASE_OBS_NAMES) + N_HISTORY_SIGNALS * len(HISTORY_TAPS):
        for tap in HISTORY_TAPS:
            age_ms = tap * MS_PER_TICK
            names += [
                f"hist_pitch        t-{age_ms}ms ({tap} ticks back)",
                f"hist_pitch_rate   t-{age_ms}ms ({tap} ticks back)",
                f"hist_v_forward    t-{age_ms}ms ({tap} ticks back)",
            ]
    if n_obs >= len(names) + 1:
        names.append("pos_err_scaled    leaky integral of (v_forward - cmd_forward), clip +/-0.5 m")
    if n_obs >= len(names) + 1:
        names.append("yaw_err_scaled    leaky integral of (yaw_rate - cmd_turn), clipped")
    return names


def c_array(name, arr):
    """Emit a flat C array. PyTorch stores Linear weights as (out, in), and
    they are written row-major so element (o, i) sits at [o * n_in + i]."""
    flat = arr.astype(np.float32).ravel()
    body = ",\n    ".join(
        ", ".join(f"{v:+.8e}f" for v in flat[i:i + 6]) for i in range(0, len(flat), 6))
    return f"static const float {name}[{flat.size}] = {{\n    {body}\n}};\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="models/best_real/best_model.zip")
    ap.add_argument("--out", default="firmware/policy_weights.h")
    args = ap.parse_args()

    model = PPO.load(args.model, device="cpu")
    sd = model.policy.state_dict()

    w0 = sd["mlp_extractor.policy_net.0.weight"].numpy()
    b0 = sd["mlp_extractor.policy_net.0.bias"].numpy()
    w1 = sd["mlp_extractor.policy_net.2.weight"].numpy()
    b1 = sd["mlp_extractor.policy_net.2.bias"].numpy()
    w2 = sd["action_net.weight"].numpy()
    b2 = sd["action_net.bias"].numpy()

    n_obs = w0.shape[1]
    n_h1, n_h2 = w0.shape[0], w1.shape[0]
    n_act = w2.shape[0]
    action_limit = float(np.max(model.action_space.high))
    total = sum(a.size for a in (w0, b0, w1, b1, w2, b2))

    act = model.policy.activation_fn.__name__
    if act != "Tanh":
        raise SystemExit(f"Exporter assumes tanh activations, model uses {act}. "
                         "Update the C inference to match before deploying.")

    names = obs_names(n_obs)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as f:
        f.write("/* Auto-generated by export_stm32.py - do not edit by hand.\n"
                f" * Source checkpoint: {args.model}\n"
                f" * Network: {n_obs} -> {n_h1} -> {n_h2} -> {n_act}, tanh activations\n"
                f" * Parameters: {total}  ({total * 4} bytes as float32)\n"
                " *\n"
                " * Observation vector layout (index: meaning):\n")
        for i, nm in enumerate(names[:n_obs]):
            f.write(f" *   {i:2d}: {nm}\n")
        f.write(" */\n#ifndef POLICY_WEIGHTS_H\n#define POLICY_WEIGHTS_H\n\n")
        f.write(f"#define POLICY_N_OBS   {n_obs}\n")
        f.write(f"#define POLICY_N_H1    {n_h1}\n")
        f.write(f"#define POLICY_N_H2    {n_h2}\n")
        f.write(f"#define POLICY_N_ACT   {n_act}\n")
        f.write(f"#define POLICY_ACTION_LIMIT {action_limit:.6f}f\n\n")
        for nm, a in (("policy_w0", w0), ("policy_b0", b0),
                      ("policy_w1", w1), ("policy_b1", b1),
                      ("policy_w2", w2), ("policy_b2", b2)):
            f.write(c_array(nm, a) + "\n")
        f.write("#endif /* POLICY_WEIGHTS_H */\n")

    print(f"wrote {out}")
    print(f"  network      : {n_obs} -> {n_h1} -> {n_h2} -> {n_act}, tanh")
    print(f"  parameters   : {total}")
    print(f"  flash        : {total * 4 / 1024:.1f} KB (float32, const -> .rodata)")
    print(f"  RAM          : {(n_h1 + n_h2) * 4} bytes of activations")
    print(f"  action limit : +/-{action_limit}"
          + ("  (PWM duty fraction, pre-deadband)" if np.isclose(action_limit, 1.0)
             else "  (N.m per wheel - this is a torque-output checkpoint)"))
    if n_obs != 34:
        print(f"  NOTE: {n_obs} inputs, not 34 - policy.c as currently written builds\n"
              "        the 34-input layout (base 17 + history taps + both leaky\n"
              "        integrals) and will not match this checkpoint's expectations.")

    # Reference vectors so the C port can be checked against Python exactly.
    rng = np.random.default_rng(0)
    obs = rng.uniform(-1.0, 1.0, size=(4, n_obs)).astype(np.float32)
    ref = Path(out.parent, "policy_testvectors.h")
    with ref.open("w") as f:
        f.write("/* Auto-generated reference vectors. Feed each observation to\n"
                " * policy_infer() and compare against the expected action; this is\n"
                " * what proves the C port matches the trained network. */\n"
                "#ifndef POLICY_TESTVECTORS_H\n#define POLICY_TESTVECTORS_H\n\n")
        f.write(f"#define POLICY_N_TESTS {obs.shape[0]}\n\n")
        acts = []
        for o in obs:
            a, _ = model.predict(o, deterministic=True)
            acts.append(np.asarray(a, dtype=np.float32))
        f.write(c_array("policy_test_obs", obs))
        f.write("\n")
        f.write(c_array("policy_test_expected", np.array(acts, dtype=np.float32)))
        f.write("\n#endif /* POLICY_TESTVECTORS_H */\n")
    print(f"wrote {ref}  ({obs.shape[0]} reference vectors)")
    print("\nNote: these test vectors exercise policy_infer() (the raw network)\n"
          "only. They do NOT exercise policy_build_obs()'s history buffer or\n"
          "integral state, since those depend on a sequence of calls, not one\n"
          "observation in isolation.")


if __name__ == "__main__":
    main()
