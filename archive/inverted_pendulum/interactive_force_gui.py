"""
Interactive Force Testing GUI for Inverted Pendulum.
Features:
1. True Physics Disturbance Injection via MuJoCo qfrc_applied (Cart Force & Pole Torque).
2. Elimination of Action Mismatch (policy control output is isolated and uncorrupted).
3. Tunable Impulse Magnitude (up to 60 N) and Duration (up to 50 steps) for high-impact visual strikes.
4. Trained PPO AI Control vs Passive / Manual Physics modes.
5. Expanded Failure Reset Angle (45°) and Track Boundary ([-1.5m, +1.5m]).
6. Live Telemetry Dashboard & Matplotlib Graphing with dynamic camera tracking.
"""

import os
import math
import sys
import time
import tkinter as tk
from tkinter import ttk
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Import wrapper from training file to ensure environment alignment
sys.path.insert(0, os.path.dirname(__file__))
try:
    from reinforced_inverted_pendulum_ppo_trainer import WideCatchDisturbanceWrapper
except ImportError:
    from train_ppo import WideCatchDisturbanceWrapper


class InteractiveForceGUI:
    def __init__(self):
        self.max_actuator_force = 20.0
        self.track_limit = 1.5

        # Build env using identical wrapper settings in evaluation mode
        raw_env = gym.make("InvertedPendulum-v5", render_mode="human")
        self.env = WideCatchDisturbanceWrapper(
            raw_env,
            max_angle_deg=30.0,
            max_omega=8.0,
            disturbance_prob=0.0,   # Automatic training noise disabled; user-driven injection
            max_push=4.0,
            track_limit=self.track_limit,
            max_fail_angle_deg=45.0,
            is_eval=True,
        )

        # Disturbance State Buffers (Cart Slide Force & Pole Hinge Torque)
        self.manual_cart_force = 0.0
        self.cart_impulse_remaining = 0
        self.cart_impulse_val = 0.0

        # Tunable impulse strike parameters
        self.cart_impulse_mag = 30.0       # Default: 30 N (exceeds 20 N motor to force visible drift)
        self.cart_impulse_duration = 15    # Default: 15 steps (~0.30 seconds)

        self.manual_pole_torque = 0.0
        self.pole_impulse_remaining = 0
        self.pole_impulse_val = 0.0

        self.running = True
        self.mode = "ppo"  # "ppo" or "manual"

        # Model Loading
        self.model = None
        model_paths = [
            "models/ppo_inverted_pendulum.zip",
            "models/ppo_inverted_pendulum",
            "models/best/best_model.zip",
            "ppo_inverted_pendulum.zip",
        ]
        for path in model_paths:
            if os.path.exists(path) or os.path.exists(path + ".zip"):
                try:
                    self.model = PPO.load(path, env=None, device="cpu")
                    print(f"Successfully loaded PPO model from: {path}")
                    break
                except Exception as e:
                    print(f"Notice: Could not load model from {path}: {e}")

        # Telemetry & Plotting Buffers (200 steps ~ 4-8 seconds)
        self.max_plot_len = 200
        self.plot_x = list(range(self.max_plot_len))
        self.plot_actual = [0.0] * self.max_plot_len
        self.plot_target = [0.0] * self.max_plot_len
        self.event_markers = []
        self.plot_step_counter = 0

        # UI Initialization
        self.root = tk.Tk()
        self.root.title("Inverted Pendulum: Interactive Force & Physics Tester")
        self.root.geometry("560x1020")
        self.root.configure(bg="#1e1e2e")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._apply_styles()
        self._build_ui()

        # Initial Environment Reset
        self.reset_env()

    def _apply_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", background="#1e1e2e", foreground="#cdd6f4", font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"), foreground="#89b4fa")
        style.configure("Title.TLabel", font=("Segoe UI", 14, "bold"), foreground="#f5c2e7")
        style.configure("Metric.TLabel", font=("Segoe UI", 10, "bold"), foreground="#a6e3a1")
        style.configure("TButton", font=("Segoe UI", 9, "bold"), padding=4)
        style.configure("TScale", background="#1e1e2e")

    def _build_ui(self):
        title_frame = tk.Frame(self.root, bg="#1e1e2e")
        title_frame.pack(fill="x", padx=15, pady=(8, 4))
        ttk.Label(title_frame, text="⚡ Physics Disturbance & Recovery Tester", style="Title.TLabel").pack(anchor="w")
        ttk.Label(title_frame, text="Exerting forces directly on cart (DOF 0) and pendulum (DOF 1)", style="TLabel").pack(anchor="w")

        # ── Section 1: Cart Force Disturbances ──
        cart_frame = tk.LabelFrame(
            self.root, text=" 🛒 1. Cart Track Disturbances (Slide Force: N) ", bg="#1e1e2e", fg="#f38ba8",
            font=("Segoe UI", 10, "bold"), padx=10, pady=6
        )
        cart_frame.pack(fill="x", padx=15, pady=4)

        self.cart_force_label = ttk.Label(cart_frame, text="Continuous Cart Force: +0.00 N", style="Metric.TLabel")
        self.cart_force_label.pack(anchor="w")

        self.cart_slider = ttk.Scale(
            cart_frame, from_=-20.0, to=20.0, orient="horizontal",
            command=self._on_cart_slider_changed
        )
        self.cart_slider.set(0.0)
        self.cart_slider.pack(fill="x", pady=2)

        # Impulse Strike Magnitude & Duration Controls
        ttk.Label(cart_frame, text="Calibrated Impulse Strike Settings:", style="TLabel").pack(anchor="w", pady=(6, 2))

        self.impulse_mag_label = ttk.Label(cart_frame, text=f"Strike Magnitude: {self.cart_impulse_mag:.1f} N", style="Metric.TLabel")
        self.impulse_mag_label.pack(anchor="w")
        self.mag_slider = ttk.Scale(
            cart_frame, from_=5.0, to=60.0, orient="horizontal",
            command=self._on_mag_slider_changed
        )
        self.mag_slider.set(self.cart_impulse_mag)
        self.mag_slider.pack(fill="x", pady=2)

        self.impulse_dur_label = ttk.Label(
            cart_frame,
            text=f"Strike Duration: {self.cart_impulse_duration} steps (~{self.cart_impulse_duration * 0.02:.2f}s)",
            style="Metric.TLabel"
        )
        self.impulse_dur_label.pack(anchor="w")
        self.dur_slider = ttk.Scale(
            cart_frame, from_=5, to=50, orient="horizontal",
            command=self._on_dur_slider_changed
        )
        self.dur_slider.set(self.cart_impulse_duration)
        self.dur_slider.pack(fill="x", pady=2)

        # Strike Action Buttons
        cart_strike_row = tk.Frame(cart_frame, bg="#1e1e2e")
        cart_strike_row.pack(fill="x", pady=4)

        tk.Button(
            cart_strike_row, text="💥 STRIKE LEFT", bg="#e64553", fg="#ffffff", font=("Segoe UI", 9, "bold"),
            relief="flat", pady=4, command=lambda: self.apply_cart_impulse(-self.cart_impulse_mag, steps=self.cart_impulse_duration)
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            cart_strike_row, text="Zero Continuous", bg="#313244", fg="#cdd6f4", font=("Segoe UI", 9),
            relief="flat", pady=4, command=self.zero_cart_force
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            cart_strike_row, text="STRIKE RIGHT 💥", bg="#40a02b", fg="#ffffff", font=("Segoe UI", 9, "bold"),
            relief="flat", pady=4, command=lambda: self.apply_cart_impulse(+self.cart_impulse_mag, steps=self.cart_impulse_duration)
        ).pack(side="left", expand=True, fill="x", padx=2)

        # ── Section 2: Pendulum Direct Torque Disturbances ──
        pole_frame = tk.LabelFrame(
            self.root, text=" 📍 2. Direct Pendulum Torque (Hinge Torque: N·m) ", bg="#1e1e2e", fg="#fab387",
            font=("Segoe UI", 10, "bold"), padx=10, pady=6
        )
        pole_frame.pack(fill="x", padx=15, pady=4)

        self.pole_torque_label = ttk.Label(pole_frame, text="Continuous Pole Torque: +0.00 N·m", style="Metric.TLabel")
        self.pole_torque_label.pack(anchor="w")

        self.pole_slider = ttk.Scale(
            pole_frame, from_=-4.0, to=4.0, orient="horizontal",
            command=self._on_pole_slider_changed
        )
        self.pole_slider.set(0.0)
        self.pole_slider.pack(fill="x", pady=2)

        pole_btn_row = tk.Frame(pole_frame, bg="#1e1e2e")
        pole_btn_row.pack(fill="x", pady=2)
        tk.Button(
            pole_btn_row, text="↺ CCW Strike (-2.5 N·m)", bg="#585b70", fg="#fab387", font=("Segoe UI", 9, "bold"),
            relief="flat", command=lambda: self.apply_pole_impulse(-2.5, steps=5)
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            pole_btn_row, text="Zero Pole", bg="#313244", fg="#cdd6f4", font=("Segoe UI", 9),
            relief="flat", command=self.zero_pole_torque
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            pole_btn_row, text="CW Strike (+2.5 N·m) ↻", bg="#585b70", fg="#fab387", font=("Segoe UI", 9, "bold"),
            relief="flat", command=lambda: self.apply_pole_impulse(+2.5, steps=5)
        ).pack(side="left", expand=True, fill="x", padx=2)

        # ── Section 3: Controller Mode ──
        ctrl_frame = tk.LabelFrame(
            self.root, text=" ⚙️ 3. Controller Mode ", bg="#1e1e2e", fg="#a6e3a1",
            font=("Segoe UI", 10, "bold"), padx=10, pady=6
        )
        ctrl_frame.pack(fill="x", padx=15, pady=4)

        mode_btn_frame = tk.Frame(ctrl_frame, bg="#1e1e2e")
        mode_btn_frame.pack(fill="x", pady=2)

        self.mode_btn = tk.Button(
            mode_btn_frame, text="🧠 Mode: Trained PPO AI Control", bg="#a6e3a1", fg="#11111b",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=4, command=self.toggle_mode
        )
        if self.model is None:
            self.mode_btn.config(text="⚠️ PPO Model Not Found (Passive Physics Mode)", state="disabled", bg="#313244", fg="#cdd6f4")
            self.mode = "manual"
        self.mode_btn.pack(side="left", expand=True, fill="x", padx=2)

        tk.Button(
            mode_btn_frame, text="🔄 Reset Simulation", bg="#89b4fa", fg="#11111b",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=4, command=self.reset_env
        ).pack(side="left", expand=True, fill="x", padx=2)

        # ── Section 4: Live Telemetry ──
        tele_frame = tk.LabelFrame(
            self.root, text=" 📊 Live Telemetry ", bg="#1e1e2e", fg="#cba6f7",
            font=("Segoe UI", 10, "bold"), padx=10, pady=6
        )
        tele_frame.pack(fill="x", padx=15, pady=4)

        self.tele_angle = ttk.Label(tele_frame, text="Pole Angle: 0.00° | Omega: 0.00 rad/s")
        self.tele_angle.pack(anchor="w")

        self.tele_forces = ttk.Label(tele_frame, text="Actuator (AI): 0.00 N | Dist Cart: 0.00 N | Dist Pole: 0.00 N·m")
        self.tele_forces.pack(anchor="w")

        self.tele_cart = ttk.Label(tele_frame, text="Cart Position: 0.00 m | Velocity: 0.00 m/s")
        self.tele_cart.pack(anchor="w")

        self.tele_status = tk.Label(
            tele_frame, text="🟢 BALANCING UPRIGHT", bg="#1e1e2e", fg="#a6e3a1",
            font=("Segoe UI", 10, "bold")
        )
        self.tele_status.pack(anchor="w", pady=(3, 0))

        # ── Section 5: Live Angle Graph ──
        plot_frame = tk.LabelFrame(
            self.root, text=" 📈 Dynamic Angle Response ", bg="#1e1e2e", fg="#f9e2af",
            font=("Segoe UI", 10, "bold"), padx=5, pady=5
        )
        plot_frame.pack(fill="both", expand=True, padx=15, pady=(4, 10))

        self.fig = Figure(figsize=(5, 2.0), dpi=100, facecolor="#1e1e2e")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#1e1e2e")
        self.ax.tick_params(colors="#cdd6f4")
        self.ax.spines['bottom'].set_color('#cdd6f4')
        self.ax.spines['top'].set_color('#1e1e2e')
        self.ax.spines['right'].set_color('#1e1e2e')
        self.ax.spines['left'].set_color('#cdd6f4')

        self.line_actual, = self.ax.plot(self.plot_x, self.plot_actual, color="#a6e3a1", label="Angle (°)")
        self.line_target, = self.ax.plot(self.plot_x, self.plot_target, color="#f38ba8", linestyle="--", label="Target (0°)")

        self.ax.set_ylim(-45, 45)
        self.ax.set_xlim(0, self.max_plot_len)
        self.ax.set_xticks([])
        self.ax.legend(loc="upper left", fontsize=8, facecolor="#313244", edgecolor="#313244", labelcolor="#cdd6f4")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ── Callbacks & Controls ──
    def _add_plot_marker(self, color):
        self.event_markers.append((self.plot_step_counter, color))

    def _on_cart_slider_changed(self, val):
        try:
            self.manual_cart_force = float(val)
            self.cart_force_label.config(text=f"Continuous Cart Force: {self.manual_cart_force:+.2f} N")
        except Exception:
            pass

    def _on_mag_slider_changed(self, val):
        self.cart_impulse_mag = float(val)
        self.impulse_mag_label.config(text=f"Strike Magnitude: {self.cart_impulse_mag:.1f} N")

    def _on_dur_slider_changed(self, val):
        self.cart_impulse_duration = int(float(val))
        self.impulse_dur_label.config(
            text=f"Strike Duration: {self.cart_impulse_duration} steps (~{self.cart_impulse_duration * 0.02:.2f}s)"
        )

    def _on_pole_slider_changed(self, val):
        try:
            self.manual_pole_torque = float(val)
            self.pole_torque_label.config(text=f"Continuous Pole Torque: {self.manual_pole_torque:+.2f} N·m")
        except Exception:
            pass

    def zero_cart_force(self):
        self.manual_cart_force = 0.0
        try:
            self.cart_slider.set(0.0)
        except Exception:
            pass
        self.cart_force_label.config(text="Continuous Cart Force: +0.00 N")

    def zero_pole_torque(self):
        self.manual_pole_torque = 0.0
        try:
            self.pole_slider.set(0.0)
        except Exception:
            pass
        self.pole_torque_label.config(text="Continuous Pole Torque: +0.00 N·m")

    def apply_cart_impulse(self, force: float, steps: int = 15):
        self.cart_impulse_val = force
        self.cart_impulse_remaining = steps
        self._add_plot_marker("#f38ba8")

    def apply_pole_impulse(self, torque: float, steps: int = 5):
        self.pole_impulse_val = torque
        self.pole_impulse_remaining = steps
        self._add_plot_marker("#fab387")

    def toggle_mode(self):
        if self.model is None:
            return
        if self.mode == "ppo":
            self.mode = "manual"
            self.mode_btn.config(text="🕹️ Mode: Passive Physics Mode", bg="#f38ba8", fg="#11111b")
        else:
            self.mode = "ppo"
            self.mode_btn.config(text="🧠 Mode: Trained PPO AI Control", bg="#a6e3a1", fg="#11111b")

    def reset_env(self):
        self.obs, _ = self.env.reset()
        unwrapped = self.env.unwrapped

        # Clear external physics forces completely
        unwrapped.data.qfrc_applied[:] = 0.0
        self.cart_impulse_remaining = 0
        self.pole_impulse_remaining = 0
        self.zero_cart_force()
        self.zero_pole_torque()

        if hasattr(self, 'plot_step_counter'):
            self._add_plot_marker("#89b4fa")

    def on_close(self):
        self.running = False
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        step_counter = 0

        while self.running:
            # 1. Pump GUI events
            try:
                self.root.update()
            except tk.TclError:
                break

            obs = self.obs
            cart_pos = float(obs[0])
            pole_angle = float(obs[1])
            cart_vel = float(obs[2])
            pole_omega = float(obs[3])

            # 2. Query Agent for Clean Action (isolated from disturbance)
            if self.mode == "ppo" and self.model is not None:
                action, _ = self.model.predict(obs, deterministic=True)
                ai_action = action
                ai_force = float(action[0])
            else:
                ai_action = np.array([0.0], dtype=np.float32)
                ai_force = 0.0

            # 3. Calculate External Physical Disturbances
            effective_cart_force = self.manual_cart_force
            if self.cart_impulse_remaining > 0:
                effective_cart_force += self.cart_impulse_val
                self.cart_impulse_remaining -= 1

            effective_pole_torque = self.manual_pole_torque
            if self.pole_impulse_remaining > 0:
                effective_pole_torque += self.pole_impulse_val
                self.pole_impulse_remaining -= 1

            # 4. Inject forces directly into MuJoCo generalized force array
            unwrapped = self.env.unwrapped
            unwrapped.data.qfrc_applied[0] = effective_cart_force   # Cart slide axis (N)
            unwrapped.data.qfrc_applied[1] = effective_pole_torque  # Pole hinge rotational axis (N*m)

            # 5. Step Environment using uncorrupted action
            self.obs, _, terminated, truncated, _ = self.env.step(ai_action)

            # 6. Clear applied forces immediately post-integration
            unwrapped.data.qfrc_applied[:] = 0.0

            # Render 3D frame
            self.env.unwrapped.render()

            # Dynamic Camera Center on Cart
            try:
                viewer = self.env.unwrapped.mujoco_renderer.viewer
                if viewer is not None:
                    viewer.cam.lookat[0] = cart_pos
                    viewer.cam.lookat[1] = 0.0
                    viewer.cam.lookat[2] = 0.3
                    viewer.cam.distance = 2.6
            except Exception:
                pass

            # Check boundary conditions
            angle_deg = math.degrees(pole_angle)
            err_deg = abs(angle_deg)

            if terminated or truncated:
                status_text = "🔴 FALLEN — Auto-Resetting..."
                status_color = "#f38ba8"
                self.reset_env()
            elif abs(effective_cart_force) > 0.1 or abs(effective_pole_torque) > 0.05:
                status_text = f"🟡 DISTURBED (Cart: {effective_cart_force:+.1f}N, Pole: {effective_pole_torque:+.2f}Nm)"
                status_color = "#fab387"
            elif err_deg < 1.0:
                status_text = "🟢 BALANCED UPRIGHT (Error < 1.0°)"
                status_color = "#a6e3a1"
            else:
                status_text = f"🔵 COUNTER-STEERING (Angle: {angle_deg:+5.1f}°)..."
                status_color = "#89b4fa"

            # Telemetry and graph updates throttled for UI responsiveness
            if step_counter % 2 == 0:
                try:
                    self.tele_angle.config(text=f"Pole Angle: {angle_deg:+6.2f}° | Omega: {pole_omega:+5.2f} rad/s")
                    self.tele_forces.config(
                        text=f"Actuator (AI): {ai_force:+5.2f} N | Dist Cart: {effective_cart_force:+5.2f} N | Dist Pole: {effective_pole_torque:+5.2f} N·m"
                    )
                    self.tele_cart.config(
                        text=f"Cart Position: {cart_pos:+6.2f} m | Velocity: {cart_vel:+5.2f} m/s"
                    )
                    self.tele_status.config(text=status_text, fg=status_color)
                except tk.TclError:
                    break

                self.plot_actual.pop(0)
                self.plot_actual.append(angle_deg)
                self.plot_target.pop(0)
                self.plot_target.append(0.0)
                self.plot_step_counter += 1

                if step_counter % 8 == 0:
                    self.line_actual.set_ydata(self.plot_actual)
                    self.line_target.set_ydata(self.plot_target)

                    while len(self.ax.lines) > 2:
                        self.ax.lines[-1].remove()

                    active_markers = []
                    for absolute_idx, color in self.event_markers:
                        relative_x = self.max_plot_len - (self.plot_step_counter - absolute_idx)
                        if relative_x > 0:
                            self.ax.axvline(x=relative_x, color=color, linestyle=":", linewidth=1.5, alpha=0.8)
                            active_markers.append((absolute_idx, color))
                    self.event_markers = active_markers

                    try:
                        self.canvas.draw_idle()
                    except Exception:
                        pass

            step_counter += 1
            time.sleep(0.01)

        self.env.close()


if __name__ == "__main__":
    app = InteractiveForceGUI()
    app.run()