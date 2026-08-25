"""
Interactive Control Application for Inverted Pendulum.
Features:
1. Dynamic Equilibrium Angle Balancing (e.g. 0°, 15°, 30°, -30°, 45°).
2. Live Manual Force Injection (slider, impulse buttons, left/right pushes).
3. Real-time PID parameter tuning.
4. Auto-Balancing vs Direct Manual Cart Drive modes.
5. Live Telemetry & Status Dashboard alongside 3D MuJoCo rendering.
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


class InteractivePendulumApp:
    def __init__(self):
        # 1. Initialize Gymnasium Environment
        self.max_force = 20.0
        self.env = gym.make("InvertedPendulum-v4", render_mode="human")
        self.env.unwrapped.model.jnt_limited[0] = False
        self.env.unwrapped.model.actuator_ctrlrange[0] = [-self.max_force, self.max_force]

        # Simulation State
        self.target_angle_deg = 30.0
        self.target_angle_rad = math.radians(self.target_angle_deg)
        self.manual_force = 0.0
        self.impulse_steps_remaining = 0
        self.impulse_force_value = 0.0
        self.running = True
        self.mode = "auto"  # "auto" or "manual_only"
        
        # PPO AI Model Integration
        self.ai_tuning_enabled = False
        self.model = None
        if os.path.exists("ppo_pid_pendulum_model.zip"):
            try:
                self.model = PPO.load("ppo_pid_pendulum_model", env=None)
            except Exception as e:
                print(f"Warning: Could not load PPO model: {e}")

        # PID gains (Tuned for ultra-fast setpoint tracking & disturbance rejection)
        self.kp = 10.0
        self.ki = 14.0
        self.kd = 2.2
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.04
        
        # Plotting Data Buffers (store last 200 UI steps ~ 8 seconds)
        self.max_plot_len = 200
        self.plot_x = list(range(self.max_plot_len))
        self.plot_actual = [0.0] * self.max_plot_len
        self.plot_target = [self.target_angle_deg] * self.max_plot_len
        self.event_markers = [] 
        self.plot_step_counter = 0

        # 2. Build Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Inverted Pendulum: Interactive Controller")
        self.root.geometry("480x950")
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
        ttk.Label(title_frame, text="⚡ Inverted Pendulum Controller", style="Title.TLabel").pack(anchor="w")
        ttk.Label(title_frame, text="EE3180 DIP - Manual Force & Custom Equilibrium", style="TLabel").pack(anchor="w")

        # -------------------------------------------------------------
        # Section 1: Target Equilibrium Angle
        # -------------------------------------------------------------
        angle_frame = tk.LabelFrame(
            self.root, text=" 🎯 1. Equilibrium Angle Setpoint ", bg="#1e1e2e", fg="#89b4fa",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8
        )
        angle_frame.pack(fill="x", padx=15, pady=5)

        self.angle_label = ttk.Label(angle_frame, text=f"Target Angle: {self.target_angle_deg:.1f}°", style="Metric.TLabel")
        self.angle_label.pack(anchor="w")

        self.angle_slider = ttk.Scale(
            angle_frame, from_=-60.0, to=60.0, orient="horizontal",
            command=self._on_angle_slider_changed
        )
        self.angle_slider.set(self.target_angle_deg)
        self.angle_slider.pack(fill="x", pady=4)

        # Preset Angle Buttons
        preset_btn_frame = tk.Frame(angle_frame, bg="#1e1e2e")
        preset_btn_frame.pack(fill="x", pady=2)
        presets = [("0° Upright", 0.0), ("15° Tilt", 15.0), ("30° Tilt", 30.0), ("-30° Tilt", -30.0), ("45° Tilt", 45.0)]
        for label, val in presets:
            btn = tk.Button(
                preset_btn_frame, text=label, bg="#313244", fg="#cdd6f4", activebackground="#45475a",
                activeforeground="#ffffff", font=("Segoe UI", 9), relief="flat", padx=4, pady=2,
                command=lambda v=val: self.set_target_angle(v)
            )
            btn.pack(side="left", expand=True, fill="x", padx=2)

        # -------------------------------------------------------------
        # Section 2: Manual Force Input & Disturbances
        # -------------------------------------------------------------
        force_frame = tk.LabelFrame(
            self.root, text=" 🕹️ 2. Manual Force & Disturbance Input ", bg="#1e1e2e", fg="#f38ba8",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8
        )
        force_frame.pack(fill="x", padx=15, pady=5)

        self.force_label = ttk.Label(force_frame, text=f"Manual Force: {self.manual_force:+.2f} N", style="Metric.TLabel")
        self.force_label.pack(anchor="w")

        self.force_slider = ttk.Scale(
            force_frame, from_=-10.0, to=10.0, orient="horizontal",
            command=self._on_force_slider_changed
        )
        self.force_slider.set(0.0)
        self.force_slider.pack(fill="x", pady=4)

        # Manual Push Buttons
        push_btn_frame1 = tk.Frame(force_frame, bg="#1e1e2e")
        push_btn_frame1.pack(fill="x", pady=2)
        tk.Button(
            push_btn_frame1, text="← Push Left (-2N)", bg="#45475a", fg="#f38ba8", font=("Segoe UI", 9, "bold"),
            relief="flat", pady=3, command=lambda: self.add_manual_force(-2.0)
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            push_btn_frame1, text="Zero Force (0N)", bg="#313244", fg="#cdd6f4", font=("Segoe UI", 9),
            relief="flat", pady=3, command=self.zero_manual_force
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            push_btn_frame1, text="Push Right (+2N) →", bg="#45475a", fg="#a6e3a1", font=("Segoe UI", 9, "bold"),
            relief="flat", pady=3, command=lambda: self.add_manual_force(+2.0)
        ).pack(side="left", expand=True, fill="x", padx=2)

        push_btn_frame2 = tk.Frame(force_frame, bg="#1e1e2e")
        push_btn_frame2.pack(fill="x", pady=2)
        tk.Button(
            push_btn_frame2, text="⚡ Impulse Disturbance (+3N, 0.5s)", bg="#585b70", fg="#fab387",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=3, command=lambda: self.apply_impulse(3.0, 12)
        ).pack(side="left", expand=True, fill="x", padx=2)
        tk.Button(
            push_btn_frame2, text="⚡ Impulse (-3N, 0.5s)", bg="#585b70", fg="#fab387",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=3, command=lambda: self.apply_impulse(-3.0, 12)
        ).pack(side="left", expand=True, fill="x", padx=2)

        # -------------------------------------------------------------
        # Section 3: Controller Mode & Tuning
        # -------------------------------------------------------------
        ctrl_frame = tk.LabelFrame(
            self.root, text=" ⚙️ 3. Control Mode & Gains ", bg="#1e1e2e", fg="#a6e3a1",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8
        )
        ctrl_frame.pack(fill="x", padx=15, pady=5)

        mode_btn_frame = tk.Frame(ctrl_frame, bg="#1e1e2e")
        mode_btn_frame.pack(fill="x", pady=2)
        self.mode_btn = tk.Button(
            mode_btn_frame, text="Mode: Auto-Balance at Target Angle (PID)", bg="#a6e3a1", fg="#11111b",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=3, command=self.toggle_mode
        )
        self.mode_btn.pack(side="left", expand=True, fill="x", padx=2)

        tk.Button(
            mode_btn_frame, text="🔄 Reset Pendulum", bg="#fab387", fg="#11111b",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=3, command=self.reset_env
        ).pack(side="left", expand=True, fill="x", padx=2)
        
        # AI Auto-Tune Button
        ai_btn_frame = tk.Frame(ctrl_frame, bg="#1e1e2e")
        ai_btn_frame.pack(fill="x", pady=2)
        self.ai_btn = tk.Button(
            ai_btn_frame, text="🧠 AI Auto-Tune (PPO): OFF", bg="#313244", fg="#cdd6f4",
            font=("Segoe UI", 9, "bold"), relief="flat", pady=3, command=self.toggle_ai_tuning
        )
        if self.model is None:
            self.ai_btn.config(state="disabled", text="🧠 AI Auto-Tune (Model Not Found)")
        self.ai_btn.pack(side="left", expand=True, fill="x", padx=2)

        # PID Sliders
        gains_frame = tk.Frame(ctrl_frame, bg="#1e1e2e")
        gains_frame.pack(fill="x", pady=4)

        # Kp
        kp_row = tk.Frame(gains_frame, bg="#1e1e2e")
        kp_row.pack(fill="x")
        self.kp_label = ttk.Label(kp_row, text=f"Kp: {self.kp:.1f}", width=10)
        self.kp_label.pack(side="left")
        self.kp_scale = ttk.Scale(kp_row, from_=0.0, to=50.0, orient="horizontal", command=self._on_kp_changed)
        self.kp_scale.set(self.kp)
        self.kp_scale.pack(side="right", expand=True, fill="x")

        # Ki
        ki_row = tk.Frame(gains_frame, bg="#1e1e2e")
        ki_row.pack(fill="x")
        self.ki_label = ttk.Label(ki_row, text=f"Ki: {self.ki:.1f}", width=10)
        self.ki_label.pack(side="left")
        self.ki_scale = ttk.Scale(ki_row, from_=0.0, to=15.0, orient="horizontal", command=self._on_ki_changed)
        self.ki_scale.set(self.ki)
        self.ki_scale.pack(side="right", expand=True, fill="x")

        # Kd
        kd_row = tk.Frame(gains_frame, bg="#1e1e2e")
        kd_row.pack(fill="x")
        self.kd_label = ttk.Label(kd_row, text=f"Kd: {self.kd:.1f}", width=10)
        self.kd_label.pack(side="left")
        self.kd_scale = ttk.Scale(kd_row, from_=0.0, to=10.0, orient="horizontal", command=self._on_kd_changed)
        self.kd_scale.set(self.kd)
        self.kd_scale.pack(side="right", expand=True, fill="x")

        # -------------------------------------------------------------
        # Section 4: Live Telemetry Dashboard
        # -------------------------------------------------------------
        tele_frame = tk.LabelFrame(
            self.root, text=" 📊 Live Telemetry & Status ", bg="#1e1e2e", fg="#cba6f7",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8
        )
        tele_frame.pack(fill="both", expand=True, padx=15, pady=(5, 10))

        self.tele_angle = ttk.Label(tele_frame, text="Current Angle: 0.00° (Error: 0.00°)")
        self.tele_angle.pack(anchor="w")

        self.tele_forces = ttk.Label(tele_frame, text="Control Force: 0.00 N | Manual: 0.00 N | Total: 0.00 N")
        self.tele_forces.pack(anchor="w")

        self.tele_cart = ttk.Label(tele_frame, text="Cart Position: 0.00 m | Velocity: 0.00 m/s")
        self.tele_cart.pack(anchor="w")

        self.tele_status = tk.Label(
            tele_frame, text="🟢 BALANCING AT SETPOINT", bg="#1e1e2e", fg="#a6e3a1",
            font=("Segoe UI", 10, "bold")
        )
        self.tele_status.pack(anchor="w", pady=(4, 0))

        # -------------------------------------------------------------
        # Section 5: Live Angle Graph
        # -------------------------------------------------------------
        plot_frame = tk.LabelFrame(
            self.root, text=" 📈 Live Angle Response ", bg="#1e1e2e", fg="#f9e2af",
            font=("Segoe UI", 10, "bold"), padx=5, pady=5
        )
        plot_frame.pack(fill="both", expand=True, padx=15, pady=(5, 15))
        
        self.fig = Figure(figsize=(5, 2.5), dpi=100, facecolor="#1e1e2e")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#1e1e2e")
        self.ax.tick_params(colors="#cdd6f4")
        self.ax.spines['bottom'].set_color('#cdd6f4')
        self.ax.spines['top'].set_color('#1e1e2e')
        self.ax.spines['right'].set_color('#1e1e2e')
        self.ax.spines['left'].set_color('#cdd6f4')
        
        self.line_actual, = self.ax.plot(self.plot_x, self.plot_actual, color="#a6e3a1", label="Actual Angle")
        self.line_target, = self.ax.plot(self.plot_x, self.plot_target, color="#f38ba8", linestyle="--", label="Target Angle")
        
        self.ax.set_ylim(-60, 60)
        self.ax.set_xlim(0, self.max_plot_len)
        self.ax.set_xticks([]) 
        self.ax.legend(loc="upper left", fontsize=8, facecolor="#313244", edgecolor="#313244", labelcolor="#cdd6f4")
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # --- UI Callbacks ---
    def _add_plot_marker(self, color):
        self.event_markers.append((self.plot_step_counter, color))
    def _on_angle_slider_changed(self, val):
        try:
            self.set_target_angle(float(val), update_slider=False)
        except Exception:
            pass

    def _on_force_slider_changed(self, val):
        try:
            self.manual_force = float(val)
            self.force_label.config(text=f"Manual Force: {self.manual_force:+.2f} N")
        except Exception:
            pass

    def _on_kp_changed(self, val):
        try:
            self.kp = float(val)
            self.kp_label.config(text=f"Kp: {self.kp:.1f}")
        except Exception:
            pass

    def _on_ki_changed(self, val):
        try:
            self.ki = float(val)
            self.ki_label.config(text=f"Ki: {self.ki:.1f}")
        except Exception:
            pass

    def _on_kd_changed(self, val):
        try:
            self.kd = float(val)
            self.kd_label.config(text=f"Kd: {self.kd:.1f}")
        except Exception:
            pass

    def set_target_angle(self, angle_deg, update_slider=True):
        self.target_angle_deg = float(angle_deg)
        self.target_angle_rad = math.radians(self.target_angle_deg)
        self.angle_label.config(text=f"Target Angle: {self.target_angle_deg:.1f}°")
        if update_slider:
            try:
                self.angle_slider.set(self.target_angle_deg)
            except Exception:
                pass

    def add_manual_force(self, delta):
        self.manual_force = float(np.clip(self.manual_force + delta, -10.0, 10.0))
        try:
            self.force_slider.set(self.manual_force)
        except Exception:
            pass
        self.force_label.config(text=f"Manual Force: {self.manual_force:+.2f} N")
        self._add_plot_marker("#fab387")

    def zero_manual_force(self):
        self.manual_force = 0.0
        try:
            self.force_slider.set(0.0)
        except Exception:
            pass
        self.force_label.config(text="Manual Force: +0.00 N")

    def apply_impulse(self, force, steps=12):
        self.impulse_force_value = force
        self.impulse_steps_remaining = steps
        self._add_plot_marker("#f38ba8")

    def toggle_mode(self):
        if self.mode == "auto":
            self.mode = "manual_only"
            self.mode_btn.config(
                text="Mode: Direct Manual Cart Control", bg="#f38ba8", fg="#11111b"
            )
        else:
            self.mode = "auto"
            self.mode_btn.config(
                text="Mode: Auto-Balance at Target Angle (PID)", bg="#a6e3a1", fg="#11111b"
            )

    def toggle_ai_tuning(self):
        if self.model is None:
            return
        self.ai_tuning_enabled = not self.ai_tuning_enabled
        if self.ai_tuning_enabled:
            self.ai_btn.config(text="🧠 AI Auto-Tune (PPO): ON", bg="#cba6f7", fg="#11111b")
            self.kp_scale.state(["disabled"])
            self.ki_scale.state(["disabled"])
            self.kd_scale.state(["disabled"])
        else:
            self.ai_btn.config(text="🧠 AI Auto-Tune (PPO): OFF", bg="#313244", fg="#cdd6f4")
            self.kp_scale.state(["!disabled"])
            self.ki_scale.state(["!disabled"])
            self.kd_scale.state(["!disabled"])

    def reset_env(self):
        self.env.reset()
        self.integral = 0.0
        self.prev_error = 0.0
        self.zero_manual_force()
        # Set state to target angle
        qpos = np.array([0.0, self.target_angle_rad])
        qvel = np.array([0.0, 0.0])
        self.env.unwrapped.set_state(qpos, qvel)
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

            # 3. Compute Control Force
            effective_manual_force = self.manual_force
            if self.impulse_steps_remaining > 0:
                effective_manual_force += self.impulse_force_value
                self.impulse_steps_remaining -= 1

            if self.mode == "auto":
                # 3.1 Optional: Use PPO to dynamically tune PID gains
                if self.ai_tuning_enabled and self.model is not None:
                    action, _ = self.model.predict(obs, deterministic=True)
                    cos_t = max(float(math.cos(self.target_angle_rad)), 0.2)
                    base_kp = 8.0 / cos_t
                    base_kd = 1.8 / cos_t
                    base_ki = 12.0 / cos_t

                    self.kp = float((action[0] + 1.0) * base_kp * 0.6 + 1.0)
                    self.ki = float((action[1] + 1.0) * base_ki * 0.6 + 0.5)
                    self.kd = float((action[2] + 1.0) * base_kd * 0.6 + 0.2)
                    
                    if step_counter % 2 == 0:
                        try:
                            self.kp_scale.set(self.kp)
                            self.ki_scale.set(self.ki)
                            self.kd_scale.set(self.kd)
                        except Exception:
                            pass

                # Error relative to setpoint angle
                error = pole_angle - self.target_angle_rad
                self.integral += error * self.dt
                self.integral = float(np.clip(self.integral, -6.0, 6.0))
                derivative = pole_ang_vel

                cos_t = max(math.cos(self.target_angle_rad), 0.2)
                kp_eff = self.kp / cos_t
                kd_eff = self.kd / cos_t
                ki_eff = self.ki / cos_t

                # Feedforward equilibrium force
                if abs(self.target_angle_rad) > 1e-4:
                    feedforward = 0.8807 * (
                        math.tan(self.target_angle_rad) / math.tan(math.radians(30.0))
                    )
                else:
                    feedforward = 0.0

                pid_force = feedforward + (kp_eff * error) + (kd_eff * derivative) + (ki_eff * self.integral)
                
                # Soft return to origin mechanism when upright
                if abs(self.target_angle_rad) < 1e-4:
                    k_cart = 0.8
                    k_vel = 1.2
                    # MUST BE POSITIVE: Pushing cart right (positive) tilts pendulum left (negative angle), 
                    # which makes the strong angle PID pull the cart left (negative) toward origin.
                    cart_correction = (k_cart * cart_pos + k_vel * cart_vel)
                    pid_force += cart_correction

                total_force = pid_force + effective_manual_force
            else:
                pid_force = 0.0
                total_force = effective_manual_force

            clipped_force = np.clip([total_force], -self.max_force, self.max_force)

            # 4. Step physics
            unwrapped.do_simulation(clipped_force, unwrapped.frame_skip)
            unwrapped.render()

            # 5. Smooth Camera Tracking: Follow cart horizontally so it never runs out of view
            try:
                viewer = unwrapped.mujoco_renderer.viewer
                if viewer is not None:
                    viewer.cam.lookat[0] = float(cart_pos)
                    viewer.cam.lookat[1] = 0.0
                    viewer.cam.lookat[2] = 0.3  # Center on pole height
                    viewer.cam.distance = 2.6
            except Exception:
                pass

            # 6. Check health & recovery
            angle_deg = math.degrees(pole_angle)
            err_deg = abs(angle_deg - self.target_angle_deg)

            if err_deg > 30.0:
                status_text = "🔴 FALLEN - Auto-Resetting..."
                status_color = "#f38ba8"
                self.reset_env()
            elif abs(effective_manual_force) > 0.1:
                status_text = f"🟡 DISTURBED (Injected {effective_manual_force:+.1f} N)"
                status_color = "#fab387"
            elif err_deg < 1.0:
                status_text = f"🟢 BALANCED at {self.target_angle_deg:.1f}° (Error < 1°)"
                status_color = "#a6e3a1"
            else:
                status_text = f"🔵 RECOVERING towards {self.target_angle_deg:.1f}°..."
                status_color = "#89b4fa"

            # 6. Update Live Telemetry every 2 steps
            if step_counter % 2 == 0:
                self.tele_angle.config(
                    text=f"Pole Angle: {angle_deg:+6.2f}°  (Target: {self.target_angle_deg:+5.1f}°, Error: {err_deg:4.2f}°)"
                )
                self.tele_forces.config(
                    text=f"PID Force: {pid_force:+5.2f} N | Manual: {effective_manual_force:+5.2f} N | Total: {clipped_force[0]:+5.2f} N"
                )
                self.tele_cart.config(
                    text=f"Cart Position: {cart_pos:+6.2f} m | Velocity: {cart_vel:+5.2f} m/s"
                )
                self.tele_status.config(text=status_text, fg=status_color)
                
                # Update plot data buffers
                self.plot_actual.pop(0)
                self.plot_actual.append(angle_deg)
                self.plot_target.pop(0)
                self.plot_target.append(self.target_angle_deg)
                self.plot_step_counter += 1
                
                # Redraw graph (every 4 UI steps) to save CPU
                if step_counter % 8 == 0:
                    self.line_actual.set_ydata(self.plot_actual)
                    self.line_target.set_ydata(self.plot_target)
                    
                    # Remove old marker lines
                    while len(self.ax.lines) > 2:
                        self.ax.lines[-1].remove()
                        
                    # Add active markers
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
            time.sleep(0.01)  # Frame limiter for smooth rendering

        self.env.close()


if __name__ == "__main__":
    app = InteractivePendulumApp()
    app.run()
