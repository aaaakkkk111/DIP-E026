"""The control panel window.

Everything the task asked for lives here: drive the robot, dial in noise and
actuator faults, shove it, watch the PPO agent retune Kp/Ki/Kd live, and flip
between the fixed-PID baseline and the learned gain scheduler to compare.

The window owns the simulation loop; the 3D viewer (MuJoCo) or the built-in
2D views just render whatever state the loop produced.
"""
from __future__ import annotations

import os
import numpy as np

from .qtcompat import (QtCore, QtGui, QtWidgets, Qt, ALIGN_CENTER,
                       ALIGN_RIGHT, ORIENT_H, FOCUS_STRONG, FOCUS_NONE,
                       EV_KEYPRESS, EV_KEYRELEASE, exec_app)
from .widgets import ArenaView, SideView, Scope, GainBars, Joystick
from ..env import MODE_PID, MODE_PPO_GAINS
from ..params import GAIN_NAMES, DisturbanceConfig, GainSpace
from ..policy_io import GainPolicy

DARK_QSS = """
QWidget { background:#1a1c20; color:#c3c9d4; font-size:11px; }
QGroupBox { border:1px solid #2f343e; border-radius:5px; margin-top:9px;
            padding-top:8px; font-weight:bold; }
QGroupBox::title { subcontrol-origin:margin; left:8px; color:#8fa2be; }
QPushButton { background:#2a2f38; border:1px solid #3a4150; border-radius:4px;
              padding:5px 10px; }
QPushButton:hover { background:#343b47; }
QPushButton:checked { background:#2f5c86; border-color:#4a9adc; }
QPushButton#kick { background:#7a2f2c; border-color:#b04a44; font-weight:bold; }
QPushButton#kick:hover { background:#963a35; }
QSlider::groove:horizontal { height:4px; background:#2f343e; border-radius:2px; }
QSlider::handle:horizontal { background:#4a9adc; width:11px; margin:-5px 0;
                             border-radius:5px; }
QComboBox, QSpinBox, QDoubleSpinBox { background:#242830; border:1px solid #3a4150;
                                      border-radius:4px; padding:3px; }
QLabel#value { color:#8fa2be; }
QLabel#status { font-family:monospace; font-size:11px; }
"""


class Slider(QtWidgets.QWidget):
    """Labelled float slider with a live value readout."""

    def __init__(self, label, lo, hi, init, fmt="{:.3f}", steps=1000, parent=None):
        super().__init__(parent)
        self.lo, self.hi, self.fmt = lo, hi, fmt
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        self.name = QtWidgets.QLabel(label)
        self.name.setMinimumWidth(104)
        self.sl = QtWidgets.QSlider(ORIENT_H)
        # Sliders keep the arrow keys to themselves once clicked, which is
        # exactly how you end up unable to drive the robot after touching a
        # noise slider.  Mouse still works, keyboard goes to the window.
        self.sl.setFocusPolicy(FOCUS_NONE)
        self.sl.setRange(0, steps)
        self.sl.setValue(int((init - lo) / max(hi - lo, 1e-9) * steps))
        self.val = QtWidgets.QLabel(fmt.format(init))
        self.val.setObjectName("value")
        self.val.setMinimumWidth(52)
        self.val.setAlignment(ALIGN_RIGHT)
        lay.addWidget(self.name)
        lay.addWidget(self.sl, 1)
        lay.addWidget(self.val)
        self.sl.valueChanged.connect(self._changed)

    def _changed(self, _):
        self.val.setText(self.fmt.format(self.value()))

    def value(self):
        s = self.sl.maximum()
        return self.lo + (self.hi - self.lo) * self.sl.value() / s

    def set_value(self, v):
        s = self.sl.maximum()
        self.sl.setValue(int(np.clip((v - self.lo) / max(self.hi - self.lo, 1e-9), 0, 1) * s))


class ControlPanel(QtWidgets.QMainWindow):
    def __init__(self, core, policy=None, viewer=None, title="balance-bot"):
        super().__init__()
        self.core = core
        self.policy = policy
        self.viewer = viewer
        self.gs = core.gs
        self.keys = set()
        self._key_seen = False
        self.paused = False
        self.speed = 1.0
        self._accum = 0.0
        self._manual_gains = None

        self.setWindowTitle(title)
        self.setStyleSheet(DARK_QSS)
        self.setFocusPolicy(FOCUS_STRONG)
        self._build()

        # Grab keys application-wide.  Qt delivers a key press to the focused
        # child widget first, so without this a click on any control silently
        # steals WASD from the window.  (If the separate MuJoCo window has the
        # OS focus, nothing Qt can do -- click the panel to bring it back.)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(core.sim.dt_agent * 1000))
        self.elapsed = QtCore.QElapsedTimer()
        self.elapsed.start()

    # ==================================================================
    def _build(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ---------------- left: views + scopes ------------------------
        left = QtWidgets.QVBoxLayout()
        self.arena_view = ArenaView()
        self.side_view = SideView()
        left.addWidget(self.arena_view, 3)
        left.addWidget(self.side_view, 2)

        self.sc_pitch = Scope("pitch (deg)", ["pitch", "pitch ref"], (-8, 8))
        self.sc_vel = Scope("speed (m/s) / yaw rate (rad/s)",
                            ["v", "v ref", "yaw", "yaw ref"], (-1, 1))
        self.sc_tau = Scope("wheel torque (N·m)", ["left", "right"], (-0.6, 0.6))
        for s in (self.sc_pitch, self.sc_vel, self.sc_tau):
            left.addWidget(s, 1)
        root.addLayout(left, 3)

        # ---------------- right: controls -----------------------------
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(6)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        holder = QtWidgets.QWidget()
        holder.setLayout(right)
        scroll.setWidget(holder)
        scroll.setMinimumWidth(400)
        root.addWidget(scroll, 2)

        right.addWidget(self._grp_mode())
        right.addWidget(self._grp_drive())
        right.addWidget(self._grp_gains())
        right.addWidget(self._grp_noise())
        right.addWidget(self._grp_impulse())
        right.addStretch(1)

        self.status = QtWidgets.QLabel("")
        self.status.setObjectName("status")
        right.addWidget(self.status)

    # ------------------------------------------------------------------
    def _grp_mode(self):
        g = QtWidgets.QGroupBox("controller")
        lay = QtWidgets.QVBoxLayout(g)

        row = QtWidgets.QHBoxLayout()
        self.btn_pid = QtWidgets.QPushButton("fixed PID")
        self.btn_ppo = QtWidgets.QPushButton("PPO gain scheduling")
        for b in (self.btn_pid, self.btn_ppo):
            b.setCheckable(True)
        self.btn_pid.setChecked(self.core.mode == MODE_PID)
        self.btn_ppo.setChecked(self.core.mode == MODE_PPO_GAINS)
        self.btn_pid.clicked.connect(lambda: self._set_mode(MODE_PID))
        self.btn_ppo.clicked.connect(lambda: self._set_mode(MODE_PPO_GAINS))
        row.addWidget(self.btn_pid)
        row.addWidget(self.btn_ppo)
        lay.addLayout(row)

        row2 = QtWidgets.QHBoxLayout()
        self.lbl_policy = QtWidgets.QLabel("policy: " +
                                           ("loaded" if self.policy else "none"))
        b = QtWidgets.QPushButton("load .npz…")
        b.clicked.connect(self._load_policy)
        row2.addWidget(self.lbl_policy, 1)
        row2.addWidget(b)
        lay.addLayout(row2)

        row3 = QtWidgets.QHBoxLayout()
        for label, fn in (("reset", self._reset),
                          ("new obstacles", self._new_arena),
                          ("pause", self._toggle_pause)):
            btn = QtWidgets.QPushButton(label)
            btn.clicked.connect(fn)
            if label == "pause":
                btn.setCheckable(True)
                self.btn_pause = btn
            row3.addWidget(btn)
        lay.addLayout(row3)

        self.sl_speed = Slider("sim speed ×", 0.1, 4.0, 1.0, "{:.2f}")
        lay.addWidget(self.sl_speed)
        return g

    # ------------------------------------------------------------------
    def _grp_drive(self):
        g = QtWidgets.QGroupBox("drive  (click the pad, or use W A S D)")
        lay = QtWidgets.QHBoxLayout(g)
        self.joy = Joystick()
        self.joy.moved.connect(self._joy_moved)
        lay.addWidget(self.joy)

        col = QtWidgets.QVBoxLayout()
        self.sl_vmax = Slider("max speed", 0.2, 1.5, 0.8, "{:.2f} m/s")
        self.sl_wmax = Slider("max turn", 0.5, 4.0, 2.0, "{:.2f} r/s")
        col.addWidget(self.sl_vmax)
        col.addWidget(self.sl_wmax)
        self.lbl_cmd = QtWidgets.QLabel("v* 0.00   ψ̇* 0.00")
        self.lbl_cmd.setObjectName("value")
        col.addWidget(self.lbl_cmd)
        col.addStretch(1)
        lay.addLayout(col, 1)
        self._joy_cmd = (0.0, 0.0)
        return g

    # ------------------------------------------------------------------
    def _grp_gains(self):
        g = QtWidgets.QGroupBox("PID gains  (white tick = hand-tuned nominal)")
        lay = QtWidgets.QVBoxLayout(g)
        self.bars = GainBars(GAIN_NAMES, self.gs.low, self.gs.high)
        lay.addWidget(self.bars)
        row = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("force nominal")
        b1.clicked.connect(lambda: self.core.set_gains(self.gs.nominal))
        b2 = QtWidgets.QPushButton("detune ×0.5 pitch Kp")
        b2.clicked.connect(self._detune)
        row.addWidget(b1)
        row.addWidget(b2)
        lay.addLayout(row)
        self.sc_gains = Scope("gains (normalised)", list(GAIN_NAMES), (-1, 1))
        self.sc_gains.autoscale = False
        lay.addWidget(self.sc_gains)
        return g

    # ------------------------------------------------------------------
    def _grp_noise(self):
        g = QtWidgets.QGroupBox("noise, sensing & actuation")
        lay = QtWidgets.QVBoxLayout(g)
        self.noise_sliders = {}
        spec = [
            ("noise_pitch", "IMU pitch σ", 0.0, 0.05, 0.0, "{:.4f} rad"),
            ("noise_pitch_rate", "gyro σ", 0.0, 0.60, 0.0, "{:.3f} r/s"),
            ("noise_vel", "odom σ", 0.0, 0.30, 0.0, "{:.3f} m/s"),
            ("noise_yaw_rate", "yaw gyro σ", 0.0, 0.40, 0.0, "{:.3f} r/s"),
            ("imu_bias_walk", "gyro drift", 0.0, 0.02, 0.0, "{:.4f}"),
            ("torque_noise", "torque noise", 0.0, 0.35, 0.0, "{:.3f}"),
            ("torque_scale_err", "motor gain err", 0.0, 0.40, 0.0, "{:.3f}"),
            ("ground_slip", "ground slip", 0.0, 0.70, 0.0, "{:.3f}"),
            ("wind_force", "wind force", 0.0, 4.0, 0.0, "{:.2f} N"),
            ("wind_dir", "wind dir", -np.pi, np.pi, 0.0, "{:.2f} rad"),
        ]
        for key, label, lo, hi, init, fmt in spec:
            s = Slider(label, lo, hi, init, fmt)
            self.noise_sliders[key] = s
            lay.addWidget(s)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("sensor latency"))
        self.sp_lat = QtWidgets.QSpinBox()
        self.sp_lat.setRange(0, 20)
        self.sp_lat.setSuffix(" × 5 ms")
        row.addWidget(self.sp_lat)
        row.addStretch(1)
        lay.addLayout(row)

        # One-click operating points.  At "clean" the hand-tuned PID is already
        # optimal and gain scheduling has nothing to win -- comparing the two
        # controllers there tells you nothing.  The learned policy only starts
        # to pay off from "rough" onwards, so make that one click away.
        row2 = QtWidgets.QHBoxLayout()
        row2.addWidget(QtWidgets.QLabel("preset"))
        for name, fn in (("clean", self._preset_clean),
                         ("rough", self._preset_rough),
                         ("brutal", self._preset_brutal)):
            b = QtWidgets.QPushButton(name)
            b.clicked.connect(fn)
            row2.addWidget(b)
        lay.addLayout(row2)
        return g

    # ------------------------------------------------------------------
    def _apply_preset(self, values, latency=0, kick_hz=0.0, kick_max=0.0):
        for k, s in self.noise_sliders.items():
            s.set_value(values.get(k, 0.0))
        self.sp_lat.setValue(latency)
        self.sl_rand_hz.set_value(kick_hz)
        self.sl_rand_max.set_value(kick_max)

    def _preset_clean(self):
        self._apply_preset({})

    def _preset_rough(self):
        self._apply_preset(dict(noise_pitch=0.006, noise_pitch_rate=0.06,
                                noise_vel=0.03, noise_yaw_rate=0.03,
                                imu_bias_walk=0.002, torque_noise=0.03,
                                torque_scale_err=0.08, ground_slip=0.10,
                                wind_force=0.6),
                           latency=2, kick_hz=0.4, kick_max=12.0)

    def _preset_brutal(self):
        self._apply_preset(dict(noise_pitch=0.012, noise_pitch_rate=0.10,
                                noise_vel=0.05, noise_yaw_rate=0.06,
                                imu_bias_walk=0.004, torque_noise=0.06,
                                torque_scale_err=0.15, ground_slip=0.25,
                                wind_force=1.2),
                           latency=4, kick_hz=0.8, kick_max=20.0)

    # ------------------------------------------------------------------
    def _grp_impulse(self):
        g = QtWidgets.QGroupBox("disturbance / impact")
        lay = QtWidgets.QVBoxLayout(g)
        self.sl_kick = Slider("impulse force", 0.0, 40.0, 12.0, "{:.1f} N")
        self.sl_kick_dir = Slider("direction", -np.pi, np.pi, np.pi, "{:.2f} rad")
        self.sl_kick_dur = Slider("duration", 0.01, 0.30, 0.06, "{:.3f} s")
        for s in (self.sl_kick, self.sl_kick_dir, self.sl_kick_dur):
            lay.addWidget(s)

        row = QtWidgets.QHBoxLayout()
        self.btn_kick = QtWidgets.QPushButton("KICK  (space)")
        self.btn_kick.setObjectName("kick")
        # NB: connect through a lambda.  `clicked` emits a bool, and Qt would
        # happily bind it to `_kick(direction=...)`, silently overriding the
        # direction slider with False -> 0 rad.
        self.btn_kick.clicked.connect(lambda: self._kick())
        row.addWidget(self.btn_kick, 2)
        for lbl, d in (("← push back", np.pi), ("push fwd →", 0.0),
                       ("side ↑", np.pi / 2)):
            b = QtWidgets.QPushButton(lbl)
            b.clicked.connect(lambda _=False, dd=d: self._kick(direction=dd))
            row.addWidget(b, 1)
        lay.addLayout(row)

        self.sl_rand_hz = Slider("random kicks", 0.0, 3.0, 0.0, "{:.2f} Hz")
        self.sl_rand_max = Slider("random max", 0.0, 30.0, 10.0, "{:.1f} N")
        lay.addWidget(self.sl_rand_hz)
        lay.addWidget(self.sl_rand_max)
        return g

    # ==================================================================
    # actions
    # ==================================================================
    def _set_mode(self, mode):
        self.core.mode = mode
        self.btn_pid.setChecked(mode == MODE_PID)
        self.btn_ppo.setChecked(mode == MODE_PPO_GAINS)

    def _load_policy(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "load policy", os.getcwd(), "policy (*.npz)")
        if fn:
            try:
                self.policy = GainPolicy.load(fn)
                self.lbl_policy.setText("policy: " + os.path.basename(fn))
                self._set_mode(MODE_PPO_GAINS)
            except Exception as e:                       # pragma: no cover
                QtWidgets.QMessageBox.warning(self, "load failed", str(e))

    def _reset(self):
        self.core.reset()
        self.arena_view.clear_trail()
        for s in (self.sc_pitch, self.sc_vel, self.sc_tau, self.sc_gains):
            s.clear()

    def _new_arena(self):
        self.core.arena.randomize(robot_xy=(self.core.state[0], self.core.state[1]))
        self._reset()

    def _toggle_pause(self):
        self.paused = self.btn_pause.isChecked()

    def _detune(self):
        g = self.core.pid.gains.copy()
        g[0] *= 0.5
        self.core.set_gains(g)

    def _kick(self, direction=None):
        self.core.fire_impulse(force=self.sl_kick.value(),
                               direction=(self.sl_kick_dir.value()
                                          if direction is None else direction),
                               duration=self.sl_kick_dur.value())

    def _joy_moved(self, fwd, yaw):
        self._joy_cmd = (fwd, yaw)

    # ------------------------------------------------------------------
    def eventFilter(self, obj, ev):
        """Route every key press in the application to this window."""
        try:
            et = ev.type()
        except Exception:
            return False
        if et not in (EV_KEYPRESS, EV_KEYRELEASE):
            return False
        # ...except while the user is typing into something
        fw = QtWidgets.QApplication.focusWidget()
        if isinstance(fw, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                           QtWidgets.QComboBox, QtWidgets.QTextEdit)):
            return False
        if et == EV_KEYPRESS:
            self.keyPressEvent(ev)
        else:
            self.keyReleaseEvent(ev)
        return False        # never consume: let widgets still see it

    @staticmethod
    def _keycode(name):
        """Key constant as a plain int, on every binding.

        PyQt5 exposes ``Qt.Key_W``; PyQt6 and PySide6 moved it to
        ``Qt.Key.Key_W``.  Everything is normalised to ``int`` because
        ``QKeyEvent.key()`` returns an int and comparing an int against a
        scoped enum member is not reliably true across bindings.
        """
        k = getattr(Qt, name, None)
        if k is None:
            k = getattr(getattr(Qt, "Key", Qt), name, None)
        return None if k is None else int(k)

    def keyPressEvent(self, ev):
        code = int(ev.key())
        self.keys.add(code)
        self._key_seen = True
        if code == self._keycode("Key_Space"):
            self._kick()
        elif ev.text().lower() == "r":
            self._reset()
        elif ev.text().lower() == "p":
            self.btn_pause.toggle()
            self._toggle_pause()

    def keyReleaseEvent(self, ev):
        self.keys.discard(int(ev.key()))

    def _key_cmd(self):
        def down(*names):
            return any(self._keycode(n) in self.keys for n in names)
        fwd = (1.0 if down("Key_W", "Key_Up") else 0.0) - \
              (1.0 if down("Key_S", "Key_Down") else 0.0)
        yaw = (1.0 if down("Key_A", "Key_Left") else 0.0) - \
              (1.0 if down("Key_D", "Key_Right") else 0.0)
        return fwd, yaw

    # ==================================================================
    def _collect_disturbance(self):
        cfg = DisturbanceConfig(
            **{k: s.value() for k, s in self.noise_sliders.items()})
        cfg.latency_steps = int(self.sp_lat.value())
        cfg.impulse_force = self.sl_kick.value()
        cfg.impulse_dir = self.sl_kick_dir.value()
        cfg.impulse_duration = self.sl_kick_dur.value()
        cfg.random_impulse_hz = self.sl_rand_hz.value()
        cfg.random_impulse_max = self.sl_rand_max.value()
        return cfg

    # ------------------------------------------------------------------
    def _tick(self):
        core = self.core
        if self.paused:
            return

        core.set_disturbance(self._collect_disturbance())

        kf, ky = self._key_cmd()
        jf, jy = self._joy_cmd
        fwd = kf if abs(kf) > 1e-6 else jf
        yaw = ky if abs(ky) > 1e-6 else jy
        if abs(kf) > 1e-6 or abs(ky) > 1e-6:
            self.joy.set_external(fwd, yaw)
        v_ref = fwd * self.sl_vmax.value()
        w_ref = yaw * self.sl_wmax.value()
        core.set_command(v_ref, w_ref)

        n_sub = max(1, int(round(self.sl_speed.value())))
        for _ in range(n_sub):
            if core.mode == MODE_PPO_GAINS and self.policy is not None:
                action = self.policy.act(core._observe(), deterministic=True)
            else:
                action = np.zeros(9)
            _, _, term, trunc, info = core.step(action)
            if term or trunc:
                core.reset(hard=False)
                self.arena_view.clear_trail()
                break

        self._render()

    # ------------------------------------------------------------------
    def _render(self):
        core = self.core
        st = core.render_state()
        p = core.robot
        rays = core.arena.raycast(st["x"], st["y"], st["psi"])

        self.arena_view.update_state(
            (core.arena.p.half_x, core.arena.p.half_y),
            core.arena.obstacles, (st["x"], st["y"], st["psi"]),
            rays=rays, ok=not (st["fell"] or st["collided"]),
            radius=p.collision_radius, ray_max=core.arena.p.ray_max)

        self.side_view.update_state(
            st["theta"], st["v"], st["tau"],
            kick=(self.sl_kick.value() / 40.0
                  if core.dist.impulse_active else 0.0),
            dt=core.sim.dt_agent, tau_max=p.tau_max,
            pitch_fail=core.sim.pitch_fail, l_com=p.l_com,
            r_wheel=p.r_wheel, body_h=p.body_height,
            # CAD 侧面轮廓，只有这台 STM32 车有；通用机器人没有 3D 模型，
            # 传空元组让侧视图退回单个方块。
            profile=getattr(p, "side_profile", ()))

        ctrl = core.last_ctrl
        self.sc_pitch.push([np.degrees(st["theta"]),
                            np.degrees(ctrl.pitch_ref if ctrl else 0.0)])
        self.sc_vel.push([st["v"], st["v_ref"], st["psi_dot"], st["yaw_ref"]])
        self.sc_tau.push(list(st["tau"]))
        gn = self.gs.normalize(st["gains"])
        self.sc_gains.push(gn)
        self.bars.set_gains(st["gains"], self.gs.nominal)

        kb = "keyboard OK" if self._key_seen else "keyboard: click this window"
        self.lbl_cmd.setText(
            f"v* {st['v_ref']:+.2f} m/s   ψ̇* {st['yaw_ref']:+.2f} rad/s     [{kb}]")
        mode = "PPO" if core.mode == MODE_PPO_GAINS and self.policy else "PID"
        self.status.setText(
            f"[{mode}]  t={st['t']:6.1f}s  pitch={np.degrees(st['theta']):+6.2f}°  "
            f"v={st['v']:+.2f}  clear={core.arena.clearance(st['x'], st['y']):.2f} m"
            + ("   FALLEN" if st["fell"] else "")
            + ("   COLLISION" if st["collided"] else ""))

        if self.viewer is not None:
            try:
                self.viewer.sync()
            except Exception:
                self.viewer = None


def run_panel(core, policy=None, viewer=None, title="balance-bot"):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = ControlPanel(core, policy, viewer, title)
    win.resize(1320, 900)
    win.show()
    return exec_app(app)
