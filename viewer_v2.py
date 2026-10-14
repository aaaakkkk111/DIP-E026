"""Interactive viewer for a fine-tuned tzejun actor vs the frozen base.

Reuses the EXACT training/evaluation environment (deps/ollama_tzejun_env.py) so
what you see is what was trained and scored. Custom GLFW window owns WASD. The
overlay always shows which policy is active, its checkpoint SHA256 and the
reward weights that produced it, so the wrong model cannot be viewed by mistake.

Controls:
  W/S drive   A/D turn   Space stop   arrows push   [ / ] payload   ; / ' friction
  , / . push strength   R reset (applies payload/friction)   P pause
  1 fine-tuned policy   2 frozen base   Esc/Q quit

--test-frames N runs N autonomous frames from a +10 deg tilt, screenshots, exits.
Simulation only; never hardware.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
DEPS = HERE / "deps"
sys.path.insert(0, str(DEPS))

from ollama_tzejun_env import OllamaTzejunEnv, MAX_V_FORWARD, MAX_V_TURN  # noqa: E402
from finetune import Actor, BASE  # noqa: E402

MAX_V, MAX_YAW = 0.15, 0.25


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def find_latest_selected():
    runs = HERE / "runs"
    best = None
    for rs in runs.rglob("run_summary.json"):
        d = json.loads(rs.read_text())
        if d.get("condition") == "ollama" and Path(d["selected_checkpoint"]).exists():
            t = rs.stat().st_mtime
            if best is None or t > best[0]:
                best = (t, d)
    if best is None:
        for rs in runs.rglob("run_summary.json"):
            d = json.loads(rs.read_text())
            if Path(d["selected_checkpoint"]).exists():
                best = (rs.stat().st_mtime, d)
    return best[1] if best else None


class FineViewer:
    def __init__(self, args):
        import glfw
        from stable_baselines3 import PPO
        import torch
        torch.set_num_threads(1)
        self.glfw = glfw
        self.args = args
        run = json.loads(Path(args.run).read_text()) if args.run else find_latest_selected()
        if run is None:
            raise SystemExit("no completed fine-tune run found; train first")
        self.run_info = run
        self.tuned_actor = Actor(PPO.load(run["selected_checkpoint"], device="cpu"))
        self.base_actor = Actor(PPO.load(BASE, device="cpu"))
        self.tuned_sha = run["selected_checkpoint_sha256"]
        self.base_sha = digest(BASE)
        self.weights = run.get("selected_weights", {})
        self.condition = run.get("condition", "?")
        self.mode = "tuned"
        self.mu, self.payload, self.push_strength = 0.9, 0.0, 1.0
        self.force = np.zeros(3); self.force_end = 0.0; self.last_push = "none"
        self.held = set(); self.paused = self.fallen = False; self.follow = True
        self.rows = []
        self.out = args.out or (HERE / "interactive_runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
        self.out.mkdir(parents=True, exist_ok=True)
        self.env = OllamaTzejunEnv(max_steps=10_000_000)
        self.reset()
        if not glfw.init():
            raise RuntimeError("GLFW init failed: no display/context")
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE if args.test_frames else glfw.TRUE)
        self.window = glfw.create_window(1280, 820, "ollama_tzejun_reward_v2 | fine-tuned tzejun", None, None)
        if not self.window:
            glfw.terminate(); raise RuntimeError("could not create OpenGL window")
        glfw.make_context_current(self.window); glfw.swap_interval(0)
        self.cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(self.cam)
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 0.7, 135, -22
        self.opt = mujoco.MjvOption()
        self.scene = mujoco.MjvScene(self.env.model, maxgeom=2000)
        self.ctx = mujoco.MjrContext(self.env.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        glfw.set_key_callback(self.window, self.on_key)
        glfw.set_window_focus_callback(self.window, lambda w, f: self.held.clear() if not f else None)
        print(f"VIEWER_CREATED mode={self.mode} tuned_sha={self.tuned_sha[:16]} out={self.out}", flush=True)

    @property
    def actor(self):
        return self.tuned_actor if self.mode == "tuned" else self.base_actor

    def reset(self):
        self.held.clear(); self.force[:] = 0; self.force_end = 0.0; self.fallen = False; self.last_push = "none"
        opts = {"pitch": math.radians(10.0), "roll": 0.0, "pitch_rate": 0.0,
                "wheel_position": 0.0, "payload": self.payload, "friction": self.mu}
        self.obs, _ = self.env.reset(seed=0, options=opts)
        self.step_count = 0

    def on_key(self, window, key, scancode, action, mods):
        g = self.glfw
        if action == g.RELEASE:
            self.held.discard(key); return
        if key in (g.KEY_W, g.KEY_A, g.KEY_S, g.KEY_D):
            if action in (g.PRESS, g.REPEAT): self.held.add(key)
            return
        if action != g.PRESS:
            return
        if key in (g.KEY_ESCAPE, g.KEY_Q): g.set_window_should_close(window, True)
        elif key == g.KEY_SPACE: self.held.clear()
        elif key == g.KEY_R: self.reset()
        elif key == g.KEY_P: self.paused = not self.paused; self.held.clear()
        elif key == g.KEY_C: self.follow = not self.follow
        elif key == g.KEY_1: self.mode = "tuned"; self.reset()
        elif key == g.KEY_2: self.mode = "base"; self.reset()
        elif key in (g.KEY_LEFT_BRACKET, g.KEY_RIGHT_BRACKET):
            self.payload = float(np.clip(self.payload + (0.25 if key == g.KEY_RIGHT_BRACKET else -0.25), 0, 1.0)); self.reset()
        elif key in (g.KEY_SEMICOLON, g.KEY_APOSTROPHE):
            self.mu = float(np.clip(round(self.mu + (0.1 if key == g.KEY_APOSTROPHE else -0.1), 2), 0.3, 1.2)); self.reset()
        elif key in (g.KEY_COMMA, g.KEY_PERIOD):
            self.push_strength = float(np.clip(self.push_strength + (0.25 if key == g.KEY_PERIOD else -0.25), 0.25, 3.0))
        elif key in (g.KEY_UP, g.KEY_DOWN, g.KEY_LEFT, g.KEY_RIGHT):
            d = {g.KEY_UP: ("+X fwd", (1, 0, 0)), g.KEY_DOWN: ("-X back", (-1, 0, 0)),
                 g.KEY_LEFT: ("+Y left", (0, 1, 0)), g.KEY_RIGHT: ("-Y right", (0, -1, 0))}[key]
            self.last_push = f"{d[0]} {self.push_strength:.2f}N"
            self.force = np.asarray(d[1], dtype=float) * self.push_strength
            self.force_end = (self.step_count + 40) * 0.005

    def tick(self):
        g = self.glfw
        if self.paused or self.fallen:
            return
        v = MAX_V * ((g.KEY_W in self.held) - (g.KEY_S in self.held))
        yaw = MAX_YAW * ((g.KEY_A in self.held) - (g.KEY_D in self.held))
        self.env.set_command(v, yaw)
        t = self.step_count * 0.005
        self.env.set_external_force(self.force if t < self.force_end - 1e-9 else np.zeros(3))
        action = self.actor.predict(self.obs)
        self.obs, reward, terminated, truncated, info = self.env.step(action)
        self.step_count += 1
        self.rows.append([t, info["pitch_deg"], info["roll_deg"], info["v_forward_m_s"],
                          info["yaw_rate_rad_s"], info["x_m"], info["radius_m"]])
        if terminated:
            self.fallen = True

    def overlay(self, w, h):
        r = self.rows[-1] if self.rows else None
        sha = self.tuned_sha if self.mode == "tuned" else self.base_sha
        name = (f"FINE-TUNED tzejun ({self.condition} reward)" if self.mode == "tuned"
                else "FROZEN base tzejun")
        wtxt = " ".join(f"{k}={v:.2f}" for k, v in self.weights.items()) if self.mode == "tuned" else "(original training reward)"
        lines = [f"MODEL: {name}  [1=tuned 2=base]",
                 f"SHA256: {sha[:32]}...",
                 f"reward weights: {wtxt}",
                 f"payload={self.payload:.2f}kg  friction={self.mu:.2f}  push={self.last_push}",
                 ("FALLEN - press R to reset" if self.fallen else
                  (f"pitch={r[1]:+.1f}deg v={r[3]:+.3f}m/s yaw={r[4]:+.3f}rad/s x={r[5]:+.3f}m" if r else "ready")),
                 "SIMULATION ONLY - not hardware. W/S A/D drive, Space stop, arrows push, R reset, Esc quit"]
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                           mujoco.MjrRect(0, 0, w, h), "\n".join(lines), "", self.ctx)

    def render(self):
        if self.follow:
            self.cam.lookat[:] = self.env.data.subtree_com[self.env.chassis_id]
        w, h = self.glfw.get_framebuffer_size(self.window)
        mujoco.mjv_updateScene(self.env.model, self.env.data, self.opt, None, self.cam,
                               mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(mujoco.MjrRect(0, 0, w, h), self.scene, self.ctx)
        self.overlay(w, h)
        return w, h

    def screenshot(self, path):
        w, h = self.glfw.get_framebuffer_size(self.window)
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb, None, mujoco.MjrRect(0, 0, w, h), self.ctx)
        import matplotlib.image as mpimg
        mpimg.imsave(path, np.flipud(rgb))

    def run(self):
        g = self.glfw
        if self.args.test_frames:
            for _ in range(self.args.test_frames):
                self.tick(); self.render(); g.poll_events()
            shot = self.out / "viewer_startup.png"
            self.screenshot(shot); self._finish()
            print(f"VIEWER_TEST frames={self.args.test_frames} screenshot={shot} fell={self.fallen}", flush=True)
            return 0
        print("Viewer running. Esc/Q to quit.", flush=True)
        while not g.window_should_close(self.window):
            start = time.perf_counter()
            self.tick(); self.render()
            g.swap_buffers(self.window); g.poll_events()
            delay = 0.005 - (time.perf_counter() - start)
            if delay > 0: time.sleep(delay)
        self._finish()
        return 0

    def _finish(self):
        if self.rows:
            with (self.out / "session.csv").open("w", newline="", encoding="utf-8") as f:
                wr = csv.writer(f)
                wr.writerow(["t_s", "pitch_deg", "roll_deg", "v_forward_m_s", "yaw_rate_rad_s", "x_m", "radius_m"])
                wr.writerows(self.rows)
        try:
            self.glfw.terminate()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", type=str, default=None, help="path to a run_summary.json")
    ap.add_argument("--test-frames", type=int, default=0)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    return FineViewer(args).run()


if __name__ == "__main__":
    raise SystemExit(main())
