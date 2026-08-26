"""
Interactive Force Testing GUI for Inverted Pendulum.
Features:
1. Real-Time Force Disturbance Injection (continuous sliders, momentary nudges, calibrated impulses).
2. Trained PPO AI Control vs Direct Manual Drive modes.
3. Expanded Failure Reset Angle (45°) to prevent abrupt resets during strong pushes.
4. Calibrated Impulse Disturbances (3 steps / 0.06s) matching PPO training conditions.
5. Live Telemetry & Matplotlib Graphing alongside 3D MuJoCo rendering.
"""

import os
import math
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


class InteractiveForceGUI:
    def __init__(self):
        # 1. Initialize Gymnasium Environment
        self.max_actuator_force = 20.0
        self.env = gym.make("InvertedPendulum-v5", render_mode="human")
        self.env.unwrapped.model.jnt_limited[0] = False
        self.env.unwrapped.model.actuator_ctrlrange[0] = [
            -self.max_actuator_force,
            self.max_actuator_force,
        ]

        # Simulation State
        self.manual_force = 0.0
        self.impulse_steps_remaining = 0
        self.impulse_force_value = 0.0
        self.running = True
        self.mode = "ppo"  # "ppo" or "manual"

        # PPO Model Loading
        self.model = None
        model_paths = [
            "models/ppo_inverted_pendulum.zip",
            "models/ppo_inverted_pendulum",
            "ppo_pid_pendulum_model.zip",
            "ppo_pid_pendulum_model",
        ]
        for path in model_paths:
            if os.path.exists(path) or os.path.exists(path + ".zip"):
                try:
                    self.model = PPO.load(path, env=None, device="cpu")
                    print(f"Successfully loaded PPO model from: {path}")
                    break
                except Exception as e:
                    print(f"Notice: Could not load model from {path}: {e}")

        # Plotting Data Buffers (store last 200 UI steps ~ 8 seconds)
        self.max_plot_len = 200
        self.plot_x = list(range(self.max_plot_len))
        self.plot_actual = [0.0] * self.max_plot_len
        self.plot_target = [0.0] * self.max_plot_len
        self.event_markers = []
        self.plot_step_counter = 0

        # 2. Build Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Inverted Pendulum: Interactive Force & Recovery Tester")
        self.root.geometry("520x950")
        self.root.configure(bg="#1e1e2e")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._apply_styles()
        self._build_ui()

        # 3. Reset Environment
        self.reset_env()

    def _apply_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", background="#1e1e2e", foreground="#cdd6f4", font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI", 13, "bold"), foreground="#89b4fa")
        style.configure("Title.TLabel", font=("Segoe UI", 15, "bold"), foreground="#f5c2e7")
        style.configure("Metric.TLabel", font=("Segoe UI", 11, "bold"), foreground="#a6e3a1")
        style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=5)
        style.configure("TScale", background="#1e1e2e")

    def _build_ui(self):
        # Header
        title_frame = tk.Frame(self.root, bg="#1e1e2e")
        title_frame.pack(fill="x", padx=15, pady=(10, 5))
        ttk.Label(title_frame, text="⚡ Interactive Force Recovery Tester", style="Title.TLabel").pack(anchor="w")
        ttk.Label(title_frame, text="Test PPO Counter-Steering Against Live External Disturbances", style="TLabel").pack(anchor="w")

        # -------------------------------------------------------------
        # Section 1: Manual Force Input & Disturbances
        # -------------------------------------------------------------
        force_frame = tk.LabelFrame(
            self.root, text=" 🕹️ 1. External Force & Disturbance Injection ", bg="#1e1e2e", fg="#f38ba8",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8
        )
        force_frame.pack(fill="x", padx=15, pady=5)

        # Continuous Force Slider
        self.force_label = ttk.Label(force_frame, text=f"Continuous Constant Force: {self.manual_force:+.2f} N", style="Metric.TLabel")
        self.force_label.pack(anchor="w")

        self.force_slider = ttk.Scale(
            force_frame, from_=-10.0, to=10.0, orient="horizontal",
            command=self._on_force_slider_changed
        )
        self.force_slider.set(0.0)
        self.force_slider.pack(fill="x", pady=4)

        # Momentary Push Buttons (Short 0.1s Nudges)
        ttk.Label(force_frame, text="Momentary Push Nudges (0.1s):", style="TLabel").pack(anchor="w", pady=(4, 2))
        push_btn_frame1 = tk.Frame(force_frame, bg="#1e1e2e")
        push_btn_frame1.pack(fill="x", pady=2)
        tk.Button(
            push_btn_frame1, text="← Nudge Left (-2N)", bg="#45475a", fg="#f38ba8", font=("Segoe UI", 9, "bold"),
            relief="flat", pady=3, command=lambda: self.apply_impulse(-2.0, steps=5)
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            push_btn_frame1, text="Zero Continuous Force", bg="#313244", fg="#cdd6f4", font=("Segoe UI", 9),
            relief="flat", pady=3, command=self.zero_manual_force
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            push_btn_frame1, text="Nudge Right (+2N) →", bg="#45475a", fg="#a6e3a1", font=("Segoe UI", 9, "bold"),
            relief="flat", pady=3, command=lambda: self.apply_impulse(+2.0, steps=5)
        ).pack(side="left", expand=True, fill="x", padx=2)

        # Calibrated Short-Pulse Impulses (3 steps = 0.06s to match training)
        ttk.Label(force_frame, text="Calibrated Training Impulses (0.06s):", style="TLabel").pack(anchor="w", pady=(4, 2))
        push_btn_frame2 = tk.Frame(force_frame, bg="#1e1e2e")
        push_btn_frame2.pack(fill="x", pady=2)
        tk.Button(
            push_btn_frame2, text="⚡ Shock Impulse (-3N)", bg="#585b70", fg="#fab387",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=3, command=lambda: self.apply_impulse(-3.0, steps=3)
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            push_btn_frame2, text="⚡ Shock Impulse (+3N)", bg="#585b70", fg="#fab387",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=3, command=lambda: self.apply_impulse(3.0, steps=3)
        ).pack(side="left", expand=True, fill="x", padx=2)

        # -------------------------------------------------------------
        # Section 2: Controller Mode
        # -------------------------------------------------------------
        ctrl_frame = tk.LabelFrame(
            self.root, text=" ⚙️ 2. Controller Mode ", bg="#1e1e2e", fg="#a6e3a1",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8
        )
        ctrl_frame.pack(fill="x", padx=15, pady=5)

        mode_btn_frame = tk.Frame(ctrl_frame, bg="#1e1e2e")
        mode_btn_frame.pack(fill="x", pady=2)

        self.mode_btn = tk.Button(
            mode_btn_frame, text="🧠 Mode: Trained PPO AI Control", bg="#a6e3a1", fg="#11111b",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=4, command=self.toggle_mode
        )
        if self.model is None:
            self.mode_btn.config(text="⚠️ PPO Model Not Found (Direct Manual Mode Only)", state="disabled", bg="#313244", fg="#cdd6f4")
            self.mode = "manual"
        self.mode_btn.pack(side="left", expand=True, fill="x", padx=2)

        tk.Button(
            mode_btn_frame, text="🔄 Reset Simulation", bg="#fab387", fg="#11111b",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=4, command=self.reset_env
        ).pack(side="left", expand=True, fill="x", padx=2)

        # -------------------------------------------------------------
        # Section 3: Live Telemetry Dashboard
        # -------------------------------------------------------------
        tele_frame = tk.LabelFrame(
            self.root, text=" 📊 Live Telemetry & Status ", bg="#1e1e2e", fg="#cba6f7",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8
        )
        tele_frame.pack(fill="both", expand=True, padx=15, pady=(5, 10))

        self.tele_angle = ttk.Label(tele_frame, text="Pole Angle: 0.00°")
        self.tele_angle.pack(anchor="w")

        self.tele_forces = ttk.Label(tele_frame, text="AI Force: 0.00 N | Disturbance: 0.00 N | Total: 0.00 N")
        self.tele_forces.pack(anchor="w")

        self.tele_cart = ttk.Label(tele_frame, text="Cart Position: 0.00 m | Velocity: 0.00 m/s")
        self.tele_cart.pack(anchor="w")

        self.tele_status = tk.Label(
            tele_frame, text="🟢 BALANCING UPRIGHT", bg="#1e1e2e", fg="#a6e3a1",
            font=("Segoe UI", 10, "bold")
        )
        self.tele_status.pack(anchor="w", pady=(4, 0))

        # -------------------------------------------------------------
        # Section 4: Live Angle Graph
        # -------------------------------------------------------------
        plot_frame = tk.LabelFrame(
            self.root, text=" 📈 Live Angle Response ", bg="#1e1e2e", fg="#f9e2af",
            font=("Segoe UI", 10, "bold"), padx=5, pady=5
        )
        plot_frame.pack(fill="both", expand=True, padx=15, pady=(5, 15))

        self.fig = Figure(figsize=(5, 2.3), dpi=100, facecolor="#1e1e2e")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#1e1e2e")
        self.ax.tick_params(colors="#cdd6f4")
        self.ax.spines['bottom'].set_color('#cdd6f4')
        self.ax.spines['top'].set_color('#1e1e2e')
        self.ax.spines['right'].set_color('#1e1e2e')
        self.ax.spines['left'].set_color('#cdd6f4')

        self.line_actual, = self.ax.plot(self.plot_x, self.plot_actual, color="#a6e3a1", label="Actual Angle (°)")
        self.line_target, = self.ax.plot(self.plot_x, self.plot_target, color="#f38ba8", linestyle="--", label="Target Angle (0°)")

        # Expanded Y-limits to match 45° threshold
        self.ax.set_ylim(-45, 45)
        self.ax.set_xlim(0, self.max_plot_len)
        self.ax.set_xticks([])
        self.ax.legend(loc="upper left", fontsize=8, facecolor="#313244", edgecolor="#313244", labelcolor="#cdd6f4")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # --- UI Callbacks ---
    def _add_plot_marker(self, color):
        self.event_markers.append((self.plot_step_counter, color))

    def _on_force_slider_changed(self, val):
        try:
            self.manual_force = float(val)
            self.force_label.config(text=f"Continuous Constant Force: {self.manual_force:+.2f} N")
        except Exception:
            pass

    def zero_manual_force(self):
        self.manual_force = 0.0
        try:
            self.force_slider.set(0.0)
        except Exception:
            pass
        self.force_label.config(text="Continuous Constant Force: +0.00 N")

    def apply_impulse(self, force, steps=3):
        """Short-pulse impulse disturbance matching PPO training conditions."""
        self.impulse_force_value = force
        self.impulse_steps_remaining = steps
        self._add_plot_marker("#f38ba8")

    def toggle_mode(self):
        if self.model is None:
            return
        if self.mode == "ppo":
            self.mode = "manual"
            self.mode_btn.config(
                text="🕹️ Mode: Direct Manual Cart Control", bg="#f38ba8", fg="#11111b"
            )
        else:
            self.mode = "ppo"
            self.mode_btn.config(
                text="🧠 Mode: Trained PPO AI Control", bg="#a6e3a1", fg="#11111b"
            )

    def reset_env(self):
        self.env.reset()
        self.zero_manual_force()
        self.impulse_steps_remaining = 0
        if hasattr(self, 'plot_step_counter'):
            self._add_plot_marker("#89b4fa")

    def on_close(self):
        self.running = False
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        unwrapped = self.env.unwrapped
        step_counter = 0

        while self.running:
            # 1. Update Tkinter UI event pump
            try:
                self.root.update()
            except tk.TclError:
                break

            # 2. Get current observations
            obs = unwrapped._get_obs()
            cart_pos = obs[0]
            pole_angle = obs[1]
            cart_vel = obs[2]
            pole_ang_vel = obs[3]

            # 3. Calculate Effective External Disturbance Force
            effective_disturbance = self.manual_force
            if self.impulse_steps_remaining > 0:
                effective_disturbance += self.impulse_force_value
                self.impulse_steps_remaining -= 1

            # 4. Compute AI Action vs Manual Drive
            if self.mode == "ppo" and self.model is not None:
                action, _ = self.model.predict(obs, deterministic=True)
                ai_force = float(action[0])
                total_force = ai_force + effective_disturbance
            else:
                ai_force = 0.0
                total_force = effective_disturbance

            clipped_force = np.clip([total_force], -self.max_actuator_force, self.max_actuator_force)

            # 5. Step physics simulation
            unwrapped.do_simulation(clipped_force, unwrapped.frame_skip)
            unwrapped.render()

            # 6. Smooth Camera Tracking
            try:
                viewer = unwrapped.mujoco_renderer.viewer
                if viewer is not None:
                    viewer.cam.lookat[0] = float(cart_pos)
                    viewer.cam.lookat[1] = 0.0
                    viewer.cam.lookat[2] = 0.3
                    viewer.cam.distance = 2.6
            except Exception:
                pass

            # 7. Health Check: Expanded 45° angle threshold
            angle_deg = math.degrees(pole_angle)
            err_deg = abs(angle_deg)

            if err_deg > 45.0:
                status_text = "🔴 FALLEN (Angle > 45°) - Auto-Resetting..."
                status_color = "#f38ba8"
                self.reset_env()
            elif abs(effective_disturbance) > 0.1:
                status_text = f"🟡 DISTURBED (Injected {effective_disturbance:+.1f} N)"
                status_color = "#fab387"
            elif err_deg < 1.0:
                status_text = f"🟢 BALANCED UPRIGHT (Error < 1.0°)"
                status_color = "#a6e3a1"
            else:
                status_text = f"🔵 COUNTER-STEERING (Angle: {angle_deg:+5.1f}°)..."
                status_color = "#89b4fa"

            # 8. Update Live Telemetry & Graph
            if step_counter % 2 == 0:
                self.tele_angle.config(
                    text=f"Pole Angle: {angle_deg:+6.2f}°"
                )
                self.tele_forces.config(
                    text=f"AI Force: {ai_force:+5.2f} N | Disturbance: {effective_disturbance:+5.2f} N | Total: {clipped_force[0]:+5.2f} N"
                )
                self.tele_cart.config(
                    text=f"Cart Position: {cart_pos:+6.2f} m | Velocity: {cart_vel:+5.2f} m/s"
                )
                self.tele_status.config(text=status_text, fg=status_color)

                # Update plot data buffers
                self.plot_actual.pop(0)
                self.plot_actual.append(angle_deg)
                self.plot_target.pop(0)
                self.plot_target.append(0.0)
                self.plot_step_counter += 1

                # Redraw graph every 8 steps to save CPU
                if step_counter % 8 == 0:
                    self.line_actual.set_ydata(self.plot_actual)
                    self.line_target.set_ydata(self.plot_target)

                    # Remove old marker lines
                    while len(self.ax.lines) > 2:
                        self.ax.lines[-1].remove()

                    # Add active impulse markers
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
