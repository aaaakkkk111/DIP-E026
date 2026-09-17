"""Interactive tuning panel for the STM32 twin.

The command-line tuner (`scripts/tune_twin_baseline.py`) searches gains without you
watching.  This is the other half: the car in MuJoCo, the firmware's own gains
on live sliders, and the disturbance knobs already in the base panel.  Drag a
gain, shove the car, see what breaks.

Each slider is a **multiplier on the shipped firmware value**, 0.25x to 4x on a
log scale -- the same space the CEM tuner searches, so what you find by hand
and what the optimiser finds are directly comparable.  The centre of every
slider is the firmware, and "重置为固件 / reset to firmware" puts them all back there.

What you tune here is not a benchmark.  You are looking at one episode with
the disturbances you happen to have dialled in; two runs with different noise
will disagree.  When a setting looks good, press 保存 JSON and score it
properly:

    python scripts\\bench_twin_baseline.py --params tuned_by_hand.json --episodes 40
"""
from __future__ import annotations

import json
import os

import numpy as np

from .panel import ControlPanel, Slider, DARK_QSS
from .qtcompat import QtCore, QtWidgets, exec_app
from .widgets import Scope
from ..firmware.controllers import LQR_GAINS, PID_GAINS
from ..firmware.twin_baseline import MODE_STM32_LQR, MODE_STM32_PID

# which gains get a slider, in display order, with a human label
LQR_SLIDERS = [
    ("K1", "K1  位移 pos"),
    ("K2", "K2  速度 vel"),
    ("K3", "K3  俯仰角 pitch"),
    ("K4", "K4  俯仰角速度 pitch rate"),
    ("K5", "K5  偏航角 yaw"),
    ("K6", "K6  偏航率 yaw rate"),
]
PID_SLIDERS = [
    ("balance_kp", "直立 upright Kp"),
    ("balance_kd", "直立 upright Kd"),
    ("velocity_kp", "速度 vel Kp"),
    ("velocity_ki", "速度 vel Ki"),
    ("turn_kp", "转向 turn Kp"),
    ("turn_kd", "转向 turn Kd"),
]

SPAN = 4.0          # slider range: nominal / SPAN .. nominal * SPAN


class TwinPanel(ControlPanel):
    """The base panel, with the PPO gain bars replaced by firmware gains."""

    def __init__(self, core, viewer=None, title="stm32-twin"):
        self.firmware = core.firmware_name
        if self.firmware == MODE_STM32_LQR:
            self.spec, self.nominal = LQR_SLIDERS, dict(LQR_GAINS)
        else:
            self.spec, self.nominal = PID_SLIDERS, dict(PID_GAINS)
        super().__init__(core, policy=None, viewer=viewer, title=title)

    # ==================================================================
    def _grp_gains(self):
        g = QtWidgets.QGroupBox(
            f"固件增益 Firmware gains —— {self.firmware}"
            f"（滑条 = 相对固件值的倍数 / slider = multiple of the stock value）")
        lay = QtWidgets.QVBoxLayout(g)

        self.gain_sliders = {}
        for key, label in self.spec:
            n = self.nominal[key]
            s = Slider(label, -1.0, 1.0, 0.0, "{:+.2f}")
            s.name.setToolTip(f"固件值 stock {n:.4f}")
            self.gain_sliders[key] = s
            lay.addWidget(s)

        self.lbl_gains = QtWidgets.QLabel("")
        self.lbl_gains.setObjectName("status")
        lay.addWidget(self.lbl_gains)

        row = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("重置为固件 / reset to firmware")
        b1.clicked.connect(self._reset_gains)
        b2 = QtWidgets.QPushButton("保存 JSON… / save")
        b2.clicked.connect(self._save_gains)
        b3 = QtWidgets.QPushButton("载入 JSON… / load")
        b3.clicked.connect(self._load_gains)
        for b in (b1, b2, b3):
            row.addWidget(b)
        lay.addLayout(row)

        self.sc_gains = Scope("增益（相对固件的倍数，对数） / gains, log multiple of stock",
                              [k for k, _ in self.spec], (-1, 1))
        self.sc_gains.autoscale = False
        lay.addWidget(self.sc_gains)
        return g

    # ------------------------------------------------------------------
    def _current_gains(self):
        """Slider positions -> real gain values."""
        out = {}
        for key, _ in self.spec:
            x = self.gain_sliders[key].value()          # -1 .. +1
            out[key] = float(self.nominal[key] * SPAN ** x)
        return out

    def _reset_gains(self):
        for key, _ in self.spec:
            self.gain_sliders[key].set_value(0.0)

    def _save_gains(self):
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存参数 / save parameters", os.path.join(os.getcwd(), "tuned_by_hand.json"),
            "JSON (*.json)")
        if not fn:
            return
        gains = self._current_gains()
        if self.firmware == MODE_STM32_PID:
            gains["mid_angle_deg"] = PID_GAINS["mid_angle_deg"]
        with open(fn, "w", encoding="utf-8") as f:
            json.dump({"name": "手调 hand-tuned " + os.path.basename(fn),
                       "firmware": self.firmware,
                       "gains": gains}, f, indent=2, ensure_ascii=False)
        QtWidgets.QMessageBox.information(
            self, "已保存 / saved",
            f"写到 {fn}\n\n用这条正式打分（40 个没见过的种子）：\n\n"
            f"python scripts\\bench_twin_baseline.py --params \"{fn}\" --episodes 40")

    def _load_gains(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "载入参数 / load parameters", os.getcwd(), "JSON (*.json)")
        if not fn:
            return
        try:
            with open(fn, encoding="utf-8") as f:
                spec = json.load(f)
            if spec.get("firmware") != self.firmware:
                raise ValueError(
                    f"这个文件是 {spec.get('firmware')} 的，当前面板是 "
                f"/ that file is for {spec.get('firmware')}, this panel is "
                    f"{self.firmware}")
            for key, _ in self.spec:
                if key in spec.get("gains", {}):
                    ratio = spec["gains"][key] / self.nominal[key]
                    self.gain_sliders[key].set_value(
                        float(np.clip(np.log(abs(ratio)) / np.log(SPAN),
                                      -1.0, 1.0)))
        except Exception as e:                              # pragma: no cover
            QtWidgets.QMessageBox.warning(self, "载入失败 / load failed", str(e))

    # ==================================================================
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
        core.set_command(fwd * self.sl_vmax.value(),
                         yaw * self.sl_wmax.value())

        # push the sliders into the firmware controller every tick, so a drag
        # takes effect immediately rather than at the next reset
        gains = self._current_gains()
        if self.firmware == MODE_STM32_LQR:
            core.fw.K = np.array([gains[k] for k, _ in self.spec], dtype=float)
        else:
            core.fw.g.update(gains)

        n_sub = max(1, int(round(self.sl_speed.value())))
        for _ in range(n_sub):
            _, _, term, trunc, info = core.step()
            if term or trunc:
                core.reset(hard=False)
                self.arena_view.clear_trail()
                break

        self._render_twin(gains)

    # ------------------------------------------------------------------
    def _render_twin(self, gains):
        core = self.core
        p = core.robot
        st = core.state
        x, y, psi = float(st[0]), float(st[1]), float(st[2])
        theta, v = float(st[6]), float(st[5])
        info = core.info

        self.arena_view.update_state(
            (core.arena.p.half_x, core.arena.p.half_y),
            core.arena.obstacles, (x, y, psi),
            rays=core.arena.raycast(x, y, psi),
            ok=not (core.fell or core.collided),
            radius=p.collision_radius, ray_max=core.arena.p.ray_max)

        ccr = info.get("ccr", (0, 0))
        # a PWM compare value is not a torque, but the side view's bar is just
        # "how hard is it pushing" -- scale it to the timer period
        period = core.motor.cal.pwm_period
        self.side_view.update_state(
            theta, v, (ccr[0] / period * p.tau_max, ccr[1] / period * p.tau_max),
            kick=(self.sl_kick.value() / 40.0
                  if core.dist.impulse_active else 0.0),
            dt=core.sim.dt_agent, tau_max=p.tau_max,
            pitch_fail=core.sim.pitch_fail, l_com=p.l_com,
            r_wheel=p.r_wheel, body_h=p.body_height,
            # CAD 侧面轮廓，只有这台 STM32 车有；通用机器人没有 3D 模型，
            # 传空元组让侧视图退回单个方块。
            profile=getattr(p, "side_profile", ()))

        self.sc_pitch.push([np.degrees(theta), 0.0])
        self.sc_vel.push([v, info.get("v_ref", 0.0),
                          float(st[3]), info.get("yaw_ref", 0.0)])
        self.sc_tau.push([ccr[0], ccr[1]])
        self.sc_gains.push([self.gain_sliders[k].value() for k, _ in self.spec])

        self.lbl_gains.setText("   ".join(
            f"{k}={gains[k]:.3f}({gains[k] / self.nominal[k]:.2f}x)"
            for k, _ in self.spec))

        kb = "keyboard OK" if self._key_seen else "keyboard: 点一下本窗口 / click this window first"
        self.lbl_cmd.setText(
            f"v* {info.get('v_ref', 0):+.2f} m/s   "
            f"ψ̇* {info.get('yaw_ref', 0):+.2f} rad/s     [{kb}]")
        self.status.setText(
            f"[{self.firmware}]  t={info.get('t', 0):6.1f}s  "
            f"俯仰={np.degrees(theta):+6.2f}°  v={v:+.2f} m/s  "
            f"CCR={ccr[0]:+5d}/{ccr[1]:+5d}  "
            + ("电机已切断 motors cut (>40°)" if info.get("motors_off") else "")
            + ("   摔倒 FELL" if core.fell else ""))

        if self.viewer is not None:
            try:
                self.viewer.sync()
            except Exception:
                self.viewer = None

    # ------------------------------------------------------------------
    # the base class's PPO-specific controls have no meaning here
    def _grp_mode(self):
        g = QtWidgets.QGroupBox("仿真 / Simulation")
        lay = QtWidgets.QVBoxLayout(g)
        row = QtWidgets.QHBoxLayout()
        for label, fn in (("重置 / reset", self._reset),
                          ("新障碍 / new obstacles", self._new_arena),
                          ("暂停", self._toggle_pause)):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(fn)
            if label == "暂停":
                b.setCheckable(True)
                self.btn_pause = b
            row.addWidget(b)
        lay.addLayout(row)
        self.sl_speed = Slider("仿真倍速 speed ×", 0.1, 4.0, 1.0, "{:.2f}")
        lay.addWidget(self.sl_speed)
        # attributes the base class's other methods expect to exist
        self.btn_pid = QtWidgets.QPushButton()
        self.btn_ppo = QtWidgets.QPushButton()
        self.lbl_policy = QtWidgets.QLabel()
        return g

    def _set_mode(self, mode):
        pass

    def _detune(self):
        pass


def run_twin_baseline_panel(core, viewer=None, title="stm32-twin"):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = TwinPanel(core, viewer, title)
    win.resize(1360, 920)
    win.show()
    return exec_app(app)
