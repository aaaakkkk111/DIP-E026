"""Interactive console for the STM32 balance-car twin, with the MuJoCo view
rendered *inside* the window.

    python -m balance_bot.ui.sim_gui
    python scripts/sim_gui.py --policy policy_stm32_mujoco.npz

What is on screen
-----------------
* the MuJoCo scene, drawn into the panel itself (drag to orbit, wheel to zoom)
  so the picture and the controls are one window and the keyboard never has to
  be handed between two of them;
* a controller selector covering both shipped firmwares, every PID gain set the
  source tree ships, the CEM-tuned gains and a trained PPO policy;
* the attitude filter selector -- ``KF.c`` as the board runs it, or the old
  one-pole placeholder, so the difference is visible rather than argued about;
* sensor noise, floor friction, side wind and battery voltage;
* a kick with a magnitude and a direction;
* WASD / arrows / on-screen stick, mapped onto the *firmware's own* command
  values (0.5 m/s forward, 0.3 m/s back, 2 rad/s turn -- see
  ``FIRMWARE_COMMANDS``), not onto invented ones;
* survival scoring: steps since the last fall, best, every past run, and an
  automatic stand-up when it goes over.

Floor friction is a real change to the contact model (``geom_friction`` on the
floor and both wheels), not a torque haircut.  The wheels genuinely spin up and
the car genuinely slides, which is the point of having MuJoCo underneath.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from collections import deque

import numpy as np

from .qtcompat import (ALIGN_CENTER, ALIGN_RIGHT, ALIGN_VCENTER,  # noqa: F401
                       ASPECT_KEEP, EV_KEYPRESS, EV_KEYRELEASE, FMT_RGB888,
                       FOCUS_STRONG, KEY, ORIENT_H, QtCore, QtGui,
                       QtWidgets, Qt, SCROLLBAR_OFF, SIZE_EXPANDING,
                       TRANSFORM_SMOOTH, exec_app)
from .widgets import Joystick, Scope
from ..dynamics import IPSI, IPSID, ITH, ITHD, IV, IX, IY
from ..firmware.controllers import (FIRMWARE_COMMANDS, LQR_GAINS,
                                    PID_DEFAULT_SET, PID_GAIN_SETS, pid_gains)
from ..firmware.twin_baseline import (MODE_STM32_LQR, MODE_STM32_PID, STM32Twin,
                             make_mujoco_twin, CAR_STOP, CAR_RUN,
                             CAR_BACK, CAR_LEFT, CAR_RIGHT,
                             CAR_TLEFT, CAR_TRIGHT,
                             ROLLING_RESIST_M, ROTOR_FRIC_STATIC,
                             ROTOR_FRIC_KINETIC)
from ..firmware.motor import MotorCalibration
from ..firmware.robot import STM32_CAR
from ..params import ArenaParams, DisturbanceConfig
from ..stm32_policy import STM32InferencePolicy

FC_CMD = FIRMWARE_COMMANDS
V_FWD = FC_CMD["lqr_forward_ms"]        # 0.5 m/s
V_BACK = FC_CMD["lqr_reverse_ms"]       # -0.3 m/s
YAW_CMD = FC_CMD["lqr_turn_rad_s"]      # 2.0 rad/s

# How fast the *command* may change, per second.  These are not the car's
# physical limits -- the firmware still decides what it can actually do --
# they are what makes the stick feel like a throttle instead of a switch.
V_ACCEL = 0.9          # m/s per second while a key is held
V_BRAKE = 1.8          # m/s per second back toward zero once released
YAW_ACCEL = 7.0        # rad/s per second
YAW_BRAKE = 12.0

# 能跑多快 / How fast it can actually go
# --------------------------------------------------------------------------
# 固件自己发的是 0.5 m/s（FIRMWARE_COMMANDS 里的 lqr_forward_ms）。2026-09-10
# 换成官方 333 rpm 的电机直线之后重测（MuJoCo 斜坡、5 种子 x 12 s）：PID 卡在
# 0.80 m/s（90.3% 占空比上限，先补死区后钳位），LQR 到 1.15 m/s。取 0.65 按较弱
# 的 PID 那一支定，和 stm32_env.V_SUSTAINABLE 是同一个数。
#
# 【2026-09-17】原来这里是两个上限（勾不勾「修正编码器」各一个，0.30 / 0.10），
# 那两个数是**旧电机模型**下用 UI 自己的斜坡量的（那时空载轮速只有 187.7 rpm），
# 早已作废；编码器开关也已删除，所以合并成一个。没有重跑 UI 那条斜坡。
# One cap since the encoder switch was removed; the old 0.30/0.10 pair was
# measured under the superseded motor model.
V_MAX = 0.65                 # m/s，按住 W 的前进指令上限 / forward command cap
V_BACK_FRAC = 0.6            # 固件的 -0.3 / 0.5 之比 / the firmware's ratio

# 满舵时的转弯半径。差速车的稳态半径就是 v / omega，所以固件那个 2.0 rad/s 的
# 转向指令配 0.30 m/s 只有 0.15 m 半径——比轮距 0.161 m 还小，那是原地打转顺便
# 挪一点，不是开车。按固定半径给舵，转起来才是一条弧。停住时退回满转向，这样
# 原地掉头仍然做得到。
# Cornering radius at full lock.  A differential drive's steady-state radius is
# just v / omega, so the firmware's 2.0 rad/s against 0.30 m/s is a 0.15 m
# radius -- tighter than the 0.161 m track.  Steering to a fixed radius gives
# an arc instead; at a standstill it falls back to full yaw so the car can
# still spin on the spot.
TURN_RADIUS = 0.45           # m

# 转向不削权：同一次扫描里 yaw = 0.0 / 1.0 / 2.0 三列结果完全一致，也就是说在车
# 站得住的任何速度下，满转向权限都是免费的。之前那个 STEER_AT_SPEED = 0.15 的
# 削权是基于错误测量加上的（当时把「速度太高摔的」记成了「转弯摔的」），它才是
# 车拐不了弯的直接原因。
# Steering is free: across that same sweep the yaw = 0.0 / 1.0 / 2.0 columns are
# identical, so at any speed the car can hold, full turn authority costs
# nothing.  The old STEER_AT_SPEED taper blamed the turn for a fall the
# throttle caused, and it was what stopped the car cornering.

# 粗糙度滑块的两端。最低端是**干燥光滑地板**（抛光瓷砖、木地板），不是冰面
# ——这台车实际就在这种地面上跑，把冰当下限会让整个滑块的大半个行程落在
# 根本不会遇到的工况上。橡胶轮在干燥抛光地面上大约 μ=0.6，在橡胶垫、地毯、
# 水泥这类粗糙面上到 1.2。
#
# 出厂 mjcf.py 里地面写的是 μ=1.0，那是**基线**，对应滑块 67——改范围时要
# 保证这个点还在，否则一动滑块就把所有已有基线的地面条件换掉了。
#
# The low end is a dry smooth floor (polished tile/wood), not ice: this car
# runs on exactly that surface, and anchoring the slider at ice would spend
# most of its travel on conditions it never meets.  mjcf.py ships μ=1.0, so
# that value must stay reachable (slider 67) or every existing baseline moves.
# 【2026-09-17】轮子是**带花纹的橡胶胎**，不是硬塑料轮：干燥室内地面上花纹
# 橡胶的 μ 本来就在 0.9-1.1，掉到 0.7 要积灰或受潮，所以下限取 0.70；上限
# 取 1.50（橡胶垫、地毯）。默认停在 0.90——这台车最常跑的干燥瓷砖/木地板，
# 和 mjcf.py 的地面是同一个数。滑块直接就是 μ x 100，不用换算，也就不会因为
# 取整把基线挪掉（旧版基线在第 67 格，是算出来的）。
# The tyres are treaded rubber: 0.9-1.1 on dry indoor floors, 0.7 only when
# dusty or damp.  The slider is mu*100 and defaults to the shipped floor.
MU_SMOOTH = 0.70      # 积灰/受潮的光滑地面 / dusty or damp smooth floor
MU_GRIP = 1.50        # 橡胶垫、地毯 / rubber mat, carpet
MU_BASELINE = 0.90    # mjcf.py 的地面，干燥瓷砖/木地板 / the shipped floor
SLIDER_BASELINE = int(round(100 * MU_BASELINE))

# 滚动阻力：MuJoCo 的 friction[2] 是长度量纲的力臂，除以轮半径才是常说的滚动
# 阻力系数 c_rr。花纹橡胶胎在硬地面上 c_rr ~ 0.02-0.04（花纹块反复压缩，比光面
# 胎高），地毯上能到 0.06。模式 1 静止基准拟合出来的 0.000802 m 对应 c_rr=0.024,
# 正好落在硬地面那一段——滑块以 0.0001 m 为一格，基线是第 8 格。
# Rolling resistance: friction[2] is a moment arm; divide by the wheel radius
# for the usual coefficient.  Treaded rubber runs c_rr ~ 0.02-0.04 on hard
# floors, up to 0.06 on carpet; the fitted 0.000802 m is c_rr = 0.024.
ROLL_STEP_M = 1e-4
ROLL_SLIDER_MAX = 20          # 0.0020 m = c_rr 0.06（地毯）
ROLL_BASELINE = int(round(ROLLING_RESIST_M / ROLL_STEP_M))

# 传动摩擦倍率：减速箱库仑摩擦 + 转子静/动摩擦一起缩放，100% = 模式 1 静止
# 基准拟合出来的这台车。真车之间的差异（润滑、磨合、装配预紧）大致就在
# 0.5-2 倍，上限留到 3 倍看「干涩的减速箱」。
# Drivetrain friction multiplier; 100 % is this car as fitted.
GEAR_FRICTION_BASE = MotorCalibration().gear_friction

DARK = """
QWidget { background-color:#16181d; color:#e2e8f0;
          font-family:'Segoe UI','Microsoft YaHei',sans-serif; font-size:12px; }
QGroupBox { border:1px solid #2d3748; border-radius:8px; margin-top:13px;
            padding-top:13px; font-weight:bold; color:#90cdf4; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; }
QPushButton { background-color:#2b3240; border:1px solid #3e4859;
              border-radius:6px; padding:6px 10px; font-weight:600;
              color:#f7fafc; }
QPushButton:hover { background-color:#3b4559; border-color:#63b3ed; }
QPushButton:pressed { background-color:#1a202c; }
QPushButton#kick { background:qlineargradient(x1:0,y1:0,x2:1,y2:0,
                   stop:0 #e53e3e, stop:1 #dd6b20);
                   border:1px solid #fc8181; color:white; font-weight:bold; }
QPushButton#kick:hover { background:qlineargradient(x1:0,y1:0,x2:1,y2:0,
                         stop:0 #f56565, stop:1 #ed8936); }
QPushButton#reset { background-color:#2b6cb0; border-color:#4299e1; }
QPushButton#reset:hover { background-color:#3182ce; }
QComboBox, QSpinBox, QDoubleSpinBox { background-color:#1f242e;
    border:1px solid #3e4859; border-radius:5px; padding:4px; color:#f7fafc; }
QComboBox QAbstractItemView { background-color:#1f242e;
    selection-background-color:#2b6cb0; }
QSlider::groove:horizontal { height:6px; background:#2d3748; border-radius:3px; }
QSlider::handle:horizontal { background:#4299e1; border:1px solid #63b3ed;
    width:14px; margin:-5px 0; border-radius:7px; }
QSlider::handle:horizontal:hover { background:#63b3ed; }
QCheckBox { spacing:7px; }
QCheckBox::indicator { width:15px; height:15px; border-radius:3px;
    border:1px solid #4a5568; background-color:#1a202c; }
QCheckBox::indicator:checked { background-color:#3182ce; border-color:#63b3ed; }
QLabel#cap { color:#a0aec0; font-size:11px; }
QLabel#big { font-size:21px; font-weight:bold; color:#68d391;
             font-family:'Consolas',monospace; }
QLabel#big2 { font-size:21px; font-weight:bold; color:#ecc94b;
              font-family:'Consolas',monospace; }
QLabel#big3 { font-size:21px; font-weight:bold; color:#fc8181;
              font-family:'Consolas',monospace; }
QLabel#banner { background-color:#232731; border-radius:6px; padding:6px 10px;
                font-family:'Consolas',monospace; color:#cbd5e0; }
QListWidget { background-color:#1a1d24; border:1px solid #2d3748;
              border-radius:6px; font-family:'Consolas',monospace; }
QScrollArea { border:none; }
"""


# ==========================================================================
class MujocoView(QtWidgets.QWidget):
    """The scene, rendered offscreen and blitted into this widget.

    ``mujoco.viewer.launch_passive`` would be less code, but it opens a second
    top-level window that owns its own keyboard focus -- which makes driving
    with WASD a game of clicking back and forth.  Rendering here keeps one
    window, one focus, and lets the camera follow the car without fighting the
    viewer's own controls.
    """

    def __init__(self, buf_w=1280, buf_h=760, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 300)
        self.setFocusPolicy(FOCUS_STRONG)
        self.setSizePolicy(SIZE_EXPANDING, SIZE_EXPANDING)
        self._buf = (buf_w, buf_h)
        self._img = None
        self._mj = None
        self.renderer = None
        self.cam = None
        self.scene_opt = None
        self.follow = True
        self.error = ""
        self._drag = None
        self._model = None
        # Rebuilding the offscreen buffer costs ~50 ms, so a resize is
        # debounced rather than acted on per mouse move.
        self._resize_timer = QtCore.QTimer(self)
        self._resize_timer.setSingleShot(True)
        self._resize_timer.timeout.connect(self._refit)

    # ------------------------------------------------------------------
    def attach(self, model):
        """(Re)build the renderer for a model.  Safe to call repeatedly."""
        try:
            import mujoco
        except Exception as e:                       # pragma: no cover
            self.error = f"MuJoCo 不可用 / MuJoCo unavailable: {e}"
            return False
        self._mj = mujoco
        self._model = model
        try:
            if self.renderer is not None:
                self.renderer.close()
            self.renderer = mujoco.Renderer(model, height=self._buf[1],
                                            width=self._buf[0])
        except Exception as e:                       # pragma: no cover
            self.error = f"离屏渲染器打不开 / cannot open offscreen renderer: {e}"
            self.renderer = None
            return False
        if self.cam is None:
            self.cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(self.cam)
            self.cam.distance = 0.62
            self.cam.elevation = -19.0
            self.cam.azimuth = 118.0
            self.cam.lookat[:] = [0.0, 0.0, 0.055]
        if self.scene_opt is None:
            self.scene_opt = mujoco.MjvOption()
            mujoco.mjv_defaultOption(self.scene_opt)
        self.error = ""
        return True

    # ------------------------------------------------------------------
    def draw(self, data, lookat=None):
        if self.renderer is None:
            return
        if self.follow and lookat is not None:
            # ease toward the car so a kick does not whip the camera
            tgt = np.asarray(lookat, dtype=float)
            self.cam.lookat[:] = 0.75 * np.asarray(self.cam.lookat) + 0.25 * tgt
        try:
            self.renderer.update_scene(data, self.cam, self.scene_opt)
            px = self.renderer.render()
        except Exception as e:                       # pragma: no cover
            self.error = f"渲染失败 / render failed: {e}"
            self.renderer = None
            self.update()
            return
        px = np.ascontiguousarray(px)
        h, w, _ = px.shape
        self._img = QtGui.QImage(px.data, w, h, 3 * w,
                                 FMT_RGB888).copy()
        self.update()

    # ------------------------------------------------------------------
    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor("#0f1116"))
        if self._img is None:
            p.setPen(QtGui.QColor("#a0aec0"))
            msg = self.error or "正在启动 MuJoCo 渲染… / starting MuJoCo renderer…"
            p.drawText(self.rect(), ALIGN_CENTER, msg)
            p.end()
            return
        scaled = self._img.scaled(self.size(), ASPECT_KEEP,
                                  TRANSFORM_SMOOTH)
        x = (self.width() - scaled.width()) // 2
        y = (self.height() - scaled.height()) // 2
        p.drawImage(x, y, scaled)
        p.end()

    # ---- camera controls ---------------------------------------------
    def mousePressEvent(self, ev):
        self.setFocus()
        self._drag = ev.position() if hasattr(ev, "position") else ev.pos()

    def mouseMoveEvent(self, ev):
        if self._drag is None or self.cam is None:
            return
        pos = ev.position() if hasattr(ev, "position") else ev.pos()
        dx = pos.x() - self._drag.x()
        dy = pos.y() - self._drag.y()
        self._drag = pos
        self.cam.azimuth = (self.cam.azimuth - dx * 0.4) % 360.0
        self.cam.elevation = float(np.clip(self.cam.elevation - dy * 0.3,
                                           -89.0, 20.0))

    def mouseReleaseEvent(self, _):
        self._drag = None

    def wheelEvent(self, ev):
        if self.cam is None:
            return
        d = ev.angleDelta().y() / 120.0
        self.cam.distance = float(np.clip(self.cam.distance * (0.88 ** d),
                                          0.25, 8.0))

    # ------------------------------------------------------------------
    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._resize_timer.start(250)

    def _refit(self):
        """Match the offscreen buffer to the widget, so nothing is letterboxed."""
        if self._model is None or self.height() < 80:
            return
        h = 760
        w = int(np.clip(round(h * self.width() / self.height()), 640, 1900))
        if abs(w - self._buf[0]) < 40:
            return
        self._buf = (w, h)
        self.attach(self._model)

    def closeRenderer(self):
        if self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
            self.renderer = None


# ==========================================================================
def _slider_row(label, lo, hi, val, fmt, on_change):
    """A labelled slider that reports its own value.  Returns (row, slider, lbl)."""
    row = QtWidgets.QHBoxLayout()
    cap = QtWidgets.QLabel(label)
    cap.setMinimumWidth(96)
    sl = QtWidgets.QSlider(ORIENT_H)
    sl.setRange(lo, hi)
    sl.setValue(val)
    out = QtWidgets.QLabel(fmt(val))
    out.setMinimumWidth(58)
    out.setAlignment(ALIGN_RIGHT | ALIGN_VCENTER)

    def _fire(v):
        out.setText(fmt(v))
        on_change()

    sl.valueChanged.connect(_fire)
    row.addWidget(cap)
    row.addWidget(sl, 1)
    row.addWidget(out)
    return row, sl, out


# ==========================================================================
class SimWindow(QtWidgets.QMainWindow):

    TICK_MS = 40                      # 25 Hz, one agent step per frame
    RECOVER_TICKS = 12                # ~0.5 s of "it fell" before standing up

    def __init__(self, policy_path="", backend="mujoco"):
        super().__init__()
        self.setWindowTitle("STM32 平衡小车 · MuJoCo 数字孪生控制台  —  STM32 Balance Car · MuJoCo Digital-Twin Console")
        self.setStyleSheet(DARK)
        self.resize(1360, 840)

        self.backend = backend
        self.run_steps = 0
        self.best_steps = 0
        self.total_falls = 0
        self.history = deque(maxlen=200)
        self.run_index = 0
        self.auto_reset = True
        self.reset_in_place = True
        self.fallen = False
        self._recover_left = 0
        self.paused = False

        self.cmd_v = 0.0
        self.cmd_yaw = 0.0
        self.target_v = 0.0
        self.target_yaw = 0.0
        self.stick_owned = False
        self.keys = set()

        self.policies = self._find_policies(policy_path)
        self.rl_policy = None
        self.tuned_gains = self._load_tuned()
        self.goto_gains = self._load_goto()

        self.core = self._build_core()
        self.mode = "pid"
        self.pid_set = PID_DEFAULT_SET
        self.last_action = np.zeros(
            self.rl_policy.gain_space.dim if self.rl_policy else 6,
            dtype=np.float64)

        self._build_ui()
        self._refresh_speed_cap()      # 填一次 W/S 的速度标签 / seed the label
        self._apply_controller()
        self._apply_environment()

        if self.backend == "mujoco":
            self.view.attach(self.core.model)

        QtWidgets.QApplication.instance().installEventFilter(self)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(self.TICK_MS)

    # ------------------------------------------------------------------
    # setup
    # ------------------------------------------------------------------
    # The project directory: two levels up from this file (balance_bot/ui/).
    # Resources live there, not next to whichever script was launched.
    PROJ_DIR = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))

    @classmethod
    def _search_dirs(cls):
        """cwd first (an explicit launch wins), then the project directory."""
        seen, out = set(), []
        for d in (os.getcwd(), cls.PROJ_DIR):
            r = os.path.realpath(d)
            if r not in seen:
                seen.add(r)
                out.append(d)
        return out

    @classmethod
    def _find_policies(cls, preferred=""):
        found = []
        cands = [preferred] if preferred else []
        for d in cls._search_dirs():
            cands += sorted(glob.glob(os.path.join(d, "policy_*.npz")))
        for cand in cands:
            if not cand or not os.path.exists(cand) or cand in found:
                continue
            try:
                STM32InferencePolicy(cand)      # cheap validity check
            except Exception:
                continue
            found.append(cand)
        return found

    @classmethod
    def _load_tuned(cls):
        for d in cls._search_dirs():
            for name in ("tuned_lqr.json", "tuned_pid.json"):
                path = os.path.join(d, name)
                if not os.path.exists(path):
                    continue
                try:
                    with open(path, encoding="utf-8") as f:
                        spec = json.load(f)
                    g = spec.get("gains")
                    if g:
                        return {"path": path, "gains": g,
                                "firmware": spec.get("firmware",
                                                     MODE_STM32_LQR)}
                except Exception:
                    pass
        return None

    @classmethod
    def _load_goto(cls):
        """找 PPO 定点行驶训练出来的**静态** PID 增益集。

        和上面的 CEM 产物是两回事：这个是 scripts/train_stm32_goto.py 的输出，
        一组固定的数（不是每拍改写增益的策略），而且是在 0~4 kg 全载重范围上
        搜的，所以 GUI 里不需要跟着载重滑块换模型。
        Static gains from the goto trainer -- one fixed set, valid across the
        whole 0-4 kg payload range, unlike the per-tick PPO policies.
        """
        out = []
        for d in cls._search_dirs():
            try:
                names = sorted(os.listdir(d))
            except OSError:
                continue
            for name in names:
                if not (name.startswith("policy_stm32_goto")
                        and name.endswith(".json")):
                    continue
                path = os.path.join(d, name)
                try:
                    with open(path, encoding="utf-8") as f:
                        spec = json.load(f)
                    g = spec.get("gains")
                    if not g:
                        continue
                    out.append({"path": path, "gains": g,
                                "name": spec.get("name", name),
                                "prov": spec.get("_provenance", {})})
                except Exception:
                    pass
        return out

    def _build_core(self):
        kw = dict(firmware=MODE_STM32_PID, gains=pid_gains(PID_DEFAULT_SET),
                  randomize=False, obstacles=False,
                  disturbance=DisturbanceConfig(),
                  arena=ArenaParams(half_x=3.0, half_y=3.0, n_obstacles=0),
                  episode_seconds=1e6)      # the UI owns the episode, not the clock
        core = (make_mujoco_twin(**kw) if self.backend == "mujoco"
                else STM32Twin(**kw))
        core.reset(seed=0)
        core.set_command(0.0, 0.0)
        return core

    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # ---------------- left: the picture ----------------------------
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(10)

        self.view = MujocoView()
        if self.backend != "mujoco":
            self.view.error = "解析后端没有 3D 画面（用 --backend mujoco） / no 3D view on the analytic backend (use --backend mujoco)"
        left.addWidget(self.view, 1)

        self.banner = QtWidgets.QLabel("准备就绪。点画面后用 WASD 驾驶，空格踹一脚，R 复位。 / Ready. Click the view, then WASD to drive, Space to shove, R to reset.")
        self.banner.setWordWrap(True)
        self.banner.setObjectName("banner")
        left.addWidget(self.banner)

        self.scope = Scope("", ["俯仰角 θ (°) / pitch", "左电机占空比 ÷10 (%) / left motor duty ÷10"],
                           (-15, 15))
        self.scope.setMinimumHeight(140)
        left.addWidget(self.scope)

        root.addLayout(left, 5)

        # ---------------- right: the controls --------------------------
        panel = QtWidgets.QWidget()
        col = QtWidgets.QVBoxLayout(panel)
        col.setSpacing(10)
        col.setContentsMargins(0, 0, 6, 0)

        col.addWidget(self._grp_score())
        col.addWidget(self._grp_controller())
        col.addWidget(self._grp_env())
        col.addWidget(self._grp_kick())
        col.addWidget(self._grp_drive())
        col.addStretch(1)

        # Bilingual text is roughly twice as long, so anything that sizes
        # itself to its content has to be told not to.  Labels wrap; combos
        # elide (their popup and tooltip still show the whole string).
        for lb in panel.findChildren(QtWidgets.QLabel):
            if len(lb.text()) > 24:
                lb.setWordWrap(True)

        for cb in panel.findChildren(QtWidgets.QComboBox):
            cb.setSizeAdjustPolicy(
                QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
                if hasattr(QtWidgets.QComboBox, "SizeAdjustPolicy")
                else QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
            cb.setMinimumContentsLength(12)
            cb.setToolTip(cb.currentText())
            cb.currentTextChanged.connect(cb.setToolTip)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        scroll.setHorizontalScrollBarPolicy(SCROLLBAR_OFF)
        scroll.setFixedWidth(505)
        root.addWidget(scroll, 0)

    # ---- groups -------------------------------------------------------
    def _grp_score(self):
        g = QtWidgets.QGroupBox("存活步数 / Survival")
        lay = QtWidgets.QGridLayout(g)
        cap = QtWidgets.QLabel("25 Hz，1 步 = 40 ms  /  25 Hz, one step = 40 ms")
        cap.setWordWrap(True)
        cap.setStyleSheet("color:#8a93a6;")
        lay.addWidget(cap, 0, 0, 1, 6)

        def cell(title, obj):
            t = QtWidgets.QLabel(title)
            t.setObjectName("cap")
            v = QtWidgets.QLabel("0")
            v.setObjectName(obj)
            return t, v

        t1, self.lbl_cur = cell("本次存活 / current", "big")
        t2, self.lbl_best = cell("历史最佳 / best", "big2")
        t3, self.lbl_falls = cell("摔倒次数 / falls", "big3")
        for i, (t, v) in enumerate(((t1, self.lbl_cur), (t2, self.lbl_best),
                                    (t3, self.lbl_falls))):
            lay.addWidget(t, 0, i)
            lay.addWidget(v, 1, i)

        self.lbl_stats = QtWidgets.QLabel("尚无完整回合 / no completed run yet")
        self.lbl_stats.setObjectName("cap")
        self.lbl_stats.setWordWrap(True)
        lay.addWidget(self.lbl_stats, 2, 0, 1, 3)

        self.list_runs = QtWidgets.QListWidget()
        self.list_runs.setMaximumHeight(96)
        lay.addWidget(self.list_runs, 3, 0, 1, 3)

        row = QtWidgets.QHBoxLayout()
        self.chk_auto = QtWidgets.QCheckBox("自动回正 / auto-right")
        self.chk_auto.setChecked(True)
        self.chk_auto.stateChanged.connect(
            lambda s: setattr(self, "auto_reset", bool(s)))
        self.chk_place = QtWidgets.QCheckBox("原地扶起 / in place")
        self.chk_place.setChecked(True)
        self.chk_auto.setToolTip("摔倒后自动扶起并开始新回合\nautomatically stand the car up after a fall and start a new run")
        self.chk_place.setToolTip("勾选：在摔倒的位置站起来；不勾：回到场地原点\nchecked: stand up where it fell; unchecked: return to the arena origin")
        self.chk_place.stateChanged.connect(
            lambda s: setattr(self, "reset_in_place", bool(s)))
        btn_clear = QtWidgets.QPushButton("清空 / clear")
        btn_clear.clicked.connect(self._clear_scores)
        btn_reset = QtWidgets.QPushButton("复位 Reset [R]")
        btn_reset.setObjectName("reset")
        btn_reset.clicked.connect(lambda: self.reset_car(manual=True))
        row.addWidget(self.chk_auto)
        row.addWidget(self.chk_place)
        row.addStretch(1)
        lay.addLayout(row, 4, 0, 1, 3)

        row2 = QtWidgets.QHBoxLayout()
        row2.addStretch(1)
        row2.addWidget(btn_clear)
        row2.addWidget(btn_reset)
        lay.addLayout(row2, 5, 0, 1, 3)
        return g

    def _grp_controller(self):
        g = QtWidgets.QGroupBox("控制模型 / Controller")
        lay = QtWidgets.QVBoxLayout(g)

        self.cb_ctrl = QtWidgets.QComboBox()
        # 【原厂】只剩真车刷的 0.Large program。原来那项 6.LQR 全状态反馈不在
        # 这份固件里，2026-09-16 按用户要求去掉。
        # [STOCK] is only what the real car runs: 0.Large program.  The 6.LQR
        # entry is not part of that firmware and was removed on 2026-09-16.
        self.cb_ctrl.addItem(
            "【原厂】0.Large program 串级 PID  /  [STOCK] 0.Large program cascade PID",
            "pid")
        # 理想模型：同一车体 + 同一套固件 PID，把所有非理想因素关掉。用来单独看
        # PID 和刚体力学本身的行为（和 scripts/ideal_pid.py 的线性模型逐拍吻合，
        # 1.72° 起步最大差 0.03°）。
        # Ideal plant: same body and firmware PID, every non-ideal effect off.
        self.cb_ctrl.addItem(
            "【理想】理想模型 串级 PID · 无死区/延迟/限幅/噪声/弹性  /  "
            "[IDEAL] ideal plant, cascade PID",
            "ideal")
        self.cb_ctrl.addItem(
            "【自动】负载自动切档  串级 PID · 空车/重载两套自动切  /  "
            "[AUTO] automatic load switching, cascade PID",
            "autoload")
        if self.tuned_gains:
            self.cb_ctrl.addItem(
                f"【调参】CEM 搜索 LQR  /  [TUNED] {os.path.basename(self.tuned_gains['path'])}",
                "tuned")
        for i, gg in enumerate(self.goto_gains):
            base = os.path.splitext(os.path.basename(gg["path"]))[0]
            self.cb_ctrl.addItem(
                f"【定点】{base}  静态 PID · 0~4kg 通用  /  "
                f"[GOTO] {base}, static PID, one set for 0-4 kg",
                f"goto:{i}")
        for path in self.policies:
            try:
                lab = STM32InferencePolicy(path).label()
            except Exception:
                lab = os.path.basename(path)
            self.cb_ctrl.addItem(f"【训练】{lab}  /  [TRAINED] {lab}",
                                 f"rl:{path}")
        if self.policies:
            # 默认落在【原厂】串级 PID。
            #
            # 这里**曾经**默认选编号最大的训练模型，理由是原厂 LQR 开起来像
            # 卡住（当时限速只有 0.10 m/s）。那个理由没错，但「编号最大」
            # 是个坏规则：新 != 好。#9 就是反例——它无扰动站定 1.4 mm、PWM
            # 换向 0.0 次/秒（原厂 9.2 mm / 16.7 次），却因为把死区补偿砍到
            # 780 而丢了低速权限，0.30 m/s 指令只跑到 0.115（比值 0.38），
            # 侧风下漂 0.43 m。它一训完就成了默认项，等于把一台开不动的车
            # 塞给用户。
            #
            # 现在默认给原厂 PID：它不是最好的（16.6 次/秒的抖动、33 度航向
            # 漂移都还在），但它是**每一项都能用**的那个，也是所有训练要超越
            # 的基准。训练模型仍在列表里，一点即达。
            #
            # This used to default to the highest-numbered policy.  "Newest"
            # is a bad rule: #9 holds position to 1.4 mm with zero PWM
            # reversals yet tracks a 0.30 m/s command at 0.38 and drifts
            # 0.43 m in wind, because it cut the dead-band compensation to
            # 780 and lost low-speed authority.  Shipping it as the default
            # handed the user a car that will not drive.  Stock PID is not the
            # best on any single metric but it is the one that works on all of
            # them, and it is the baseline every model is trying to beat.
            self.cb_ctrl.setCurrentIndex(0)
        else:
            self.cb_ctrl.addItem(
                "【训练】还没有模型 / [TRAINED] none yet", "none")
            item = self.cb_ctrl.model().item(self.cb_ctrl.count() - 1)
            if item is not None:
                item.setEnabled(False)
            self.cb_ctrl.setItemData(
                self.cb_ctrl.count() - 1,
                "工程目录下没有 policy_*.npz。跑一次就有了：\n"
                "  python scripts/train_stm32_rl.py\n"
                "no policy_*.npz in the project directory -- "
                "run scripts/train_stm32_rl.py to make one",
                Qt.ItemDataRole.ToolTipRole if hasattr(Qt, "ItemDataRole")
                else Qt.ToolTipRole)
        self.cb_ctrl.currentIndexChanged.connect(self._apply_controller)
        lay.addWidget(self.cb_ctrl)

        self.row_pid = QtWidgets.QWidget()
        rl = QtWidgets.QHBoxLayout(self.row_pid)
        rl.setContentsMargins(0, 0, 0, 0)
        cap = QtWidgets.QLabel("模式 / mode")
        cap.setObjectName("cap")
        self.cb_pid = QtWidgets.QComboBox()
        # 顺序和编号同真车开机选模式时 OLED 上显示的一样
        for name, gains in PID_GAIN_SETS.items():
            self.cb_pid.addItem(f"{gains['label']}  ({name})", name)
        self.cb_pid.setCurrentIndex(
            list(PID_GAIN_SETS).index(PID_DEFAULT_SET))
        self.cb_pid.currentIndexChanged.connect(self._apply_controller)
        rl.addWidget(cap)
        rl.addWidget(self.cb_pid, 1)
        lay.addWidget(self.row_pid)

        # 姿态滤波三选一，对应固件 app_control.c 的 Get_Angle(way)：
        #   1 = DMP、2 = 卡尔曼、3 = 互补滤波
        # main.c 里 GET_Angle_Way = 2，所以默认停在卡尔曼——那是板子出厂真正
        # 在跑的，也是所有已发布基线的前提，换档等于换了一台车。
        # The firmware's Get_Angle(way): 1 DMP, 2 Kalman, 3 complementary.
        # main.c ships way = 2, so Kalman stays the default; every published
        # baseline assumes it.
        imu_row = QtWidgets.QHBoxLayout()
        cap2 = QtWidgets.QLabel("姿态滤波 / attitude filter")
        cap2.setObjectName("cap")
        self.cb_imu = QtWidgets.QComboBox()
        self.cb_imu.addItem("卡尔曼 KF.c（出厂 way=2） / Kalman", "kalman")
        self.cb_imu.addItem("互补滤波 filter.c（way=3） / complementary",
                            "complementary")
        self.cb_imu.addItem("DMP 片上融合（way=1，近似） / DMP (approximate)",
                            "dmp")
        self.cb_imu.setToolTip(
            "对应 app_control.c 的 Get_Angle(way)，main.c 出厂选 2。\n"
            "卡尔曼：KF.c 逐状态复现，Q=1e-10 R=1e-4，收敛后时间常数 3.02 秒，"
            "几乎是纯陀螺积分器，对加速度计被平移加速度污染不敏感。\n"
            "互补：filter.c 一行公式，K1=0.02 dt=5 ms，时间常数 0.245 秒，"
            "快十二倍但更信加速度计——刹车时读到的「重力方向」是歪的。\n"
            "DMP：片上四元数融合 + 陀螺零偏自校准。芯片里是二进制 blob，"
            "源码树里没有，所以这一档是按已知行为建的**近似**，不像另外两档"
            "是逐行复现的；拿它做出来的结论要打折扣。")
        self.cb_imu.currentIndexChanged.connect(self._apply_controller)
        imu_row.addWidget(cap2)
        imu_row.addWidget(self.cb_imu, 1)
        lay.addLayout(imu_row)

        self.lbl_ctrl = QtWidgets.QLabel("")
        self.lbl_ctrl.setObjectName("cap")
        self.lbl_ctrl.setWordWrap(True)
        lay.addWidget(self.lbl_ctrl)
        return g

    def _grp_env(self):
        g = QtWidgets.QGroupBox("环境与传感器 / Environment")
        lay = QtWidgets.QVBoxLayout(g)
        add = self._apply_environment

        r, self.sl_noise, _ = _slider_row(
            "传感器噪声 / sensor noise", 0, 100, 0, lambda v: f"{v}%", add)
        lay.addLayout(r)
        # 粗糙度：往右 = 更粗糙 = 摩擦更大。默认 100（干燥地板）——原来那个
        # 滑块叫「滑度」、默认 0，方向是反的；改名的同时默认值也得跟着翻，
        # 否则一改就把所有基线的地面条件换掉了。
        # Roughness: right = rougher = more grip.  Default 100 keeps the old
        # default surface, which was "slip = 0".
        r, self.sl_slip, _ = _slider_row(
            "地面粗糙度 / floor roughness",
            int(round(100 * MU_SMOOTH)), int(round(100 * MU_GRIP)),
            SLIDER_BASELINE, lambda v: f"μ={v / 100:.2f}", add)
        lay.addLayout(r)
        # 滚动阻力和传动摩擦：这两项是模式 1 静止基准拟合出来的真车值，默认位置
        # 就是那台车，往右是更涩的地面/减速箱。
        # Both default to the values fitted against the real car.
        r, self.sl_roll, _ = _slider_row(
            "滚动阻力 / rolling resist", 0, ROLL_SLIDER_MAX, ROLL_BASELINE,
            lambda v: f"c={v * ROLL_STEP_M / STM32_CAR.r_wheel:.3f}", add)
        lay.addLayout(r)
        r, self.sl_dfric, _ = _slider_row(
            "传动摩擦 / drivetrain fric", 0, 300, 100, lambda v: f"{v}%", add)
        lay.addLayout(r)

        # 载重 0~4 kg（官方额定 4 kg）。空车 942 g，满载总质量接近 5 倍，而且
        # 压在顶板上会把质心从 38 mm 抬到 92 mm——电机力矩一点没变。
        # 之前这一行写成 lay.addLayout(r)，把粗糙度那行加了两次，滑块建出来
        # 却从没进过布局，所以 GUI 里根本看不到。
        # This used to add `r` (the roughness row) twice, so the payload slider
        # was created but never placed.
        r, self.sl_load, _ = _slider_row(
            "载重 / payload", 0, 40, 0,
            lambda v: f"{v / 10:.1f} kg", add)
        lay.addLayout(r)
        r, self.sl_wind, _ = _slider_row(
            "持续侧风 / steady wind", 0, 30, 0, lambda v: f"{v / 10:.1f} N", add)
        lay.addLayout(r)
        r, self.sl_wdir, _ = _slider_row(
            "风向 / wind dir", 0, 359, 0, lambda v: f"{v}°", add)
        lay.addLayout(r)
        r, self.sl_torque, _ = _slider_row(
            "电机扭矩噪声 / torque noise", 0, 100, 0, lambda v: f"{v * 0.15:.0f}%", add)
        lay.addLayout(r)
        r, self.sl_batt, _ = _slider_row(
            "电池电压 / battery", 90, 126, 120, lambda v: f"{v / 10:.1f} V", add)
        lay.addLayout(r)

        note = QtWidgets.QLabel(
            "地面粗糙度改的是接触模型的摩擦系数（地面与两个轮子）。轮子是花纹"
            "橡胶胎，默认 μ=0.90 是干燥瓷砖/木地板，往右到橡胶垫、地毯 μ=1.5，"
            "往左 μ=0.7 是积灰或受潮的光滑地面——最低端不是冰面，这台车不在冰上跑。"
            "滚动阻力和传动摩擦的默认位置就是真车实测拟合出来的值：滚动阻力显示的是"
            "系数 c_rr（花纹橡胶胎硬地面 0.02-0.04、地毯到 0.06），传动摩擦 100% "
            "是这台车的减速箱和转子摩擦，往右模拟更涩的传动。"
            "载重压在顶板上，会同时加大质量、抬高质心、加大惯量。"
            "  |  Floor roughness edits the contact "
            "friction of the floor and both wheels, not a torque discount: "
            "the wheels really do spin up and slide. "
            "电压低于 9.6 V 固件会自己断电机（Turn_Off）。  |  Below 9.6 V the "
            "firmware cuts the motors itself (Turn_Off).")
        note.setObjectName("cap")
        note.setWordWrap(True)
        lay.addWidget(note)
        return g

    def _grp_kick(self):
        g = QtWidgets.QGroupBox("冲击输入 / Impulse")
        lay = QtWidgets.QVBoxLayout(g)
        r, self.sl_kick, _ = _slider_row(
            "力度 / force", 5, 250, 60, lambda v: f"{v / 10:.1f} N", lambda: None)
        lay.addLayout(r)
        r, self.sl_kickms, _ = _slider_row(
            "作用时间 / duration", 10, 200, 50, lambda v: f"{v} ms", lambda: None)
        lay.addLayout(r)

        grid = QtWidgets.QGridLayout()
        for text, ang, pos in (("前 Fwd ↑", 0.0, (0, 1)),
                               ("左 Left ←", np.pi / 2, (1, 0)),
                               ("随机 Rnd", None, (1, 1)),
                               ("右 Right →", -np.pi / 2, (1, 2)),
                               ("后 Back ↓", np.pi, (2, 1))):
            b = QtWidgets.QPushButton(text)
            if ang is None:
                b.setObjectName("kick")
                b.setText("踹一脚 Shove\n[空格 Space]")
            b.clicked.connect(lambda _=False, a=ang: self.kick(a))
            grid.addWidget(b, *pos)
        lay.addLayout(grid)

        note = QtWidgets.QLabel(
            f"参考：这辆车轮上净推力只有约 6.5 N（车重 9.8 N），"
            f"超过大概 10 N 的横推在算术上就已经救不回来了。  |  For scale: net "
            "traction is about 6.5 N (the car weighs 9.8 N), so a side shove "
            "past roughly 10 N is already arithmetically unrecoverable.")
        note.setObjectName("cap")
        note.setWordWrap(True)
        lay.addWidget(note)
        return g

    def _grp_drive(self):
        g = QtWidgets.QGroupBox("驾驶 / Drive")
        lay = QtWidgets.QHBoxLayout(g)
        self.stick = Joystick()
        self.stick.moved.connect(self._on_stick)
        lay.addWidget(self.stick)

        side = QtWidgets.QVBoxLayout()
        self.lbl_keys = QtWidgets.QLabel()
        self.lbl_keys.setObjectName("cap")
        self.lbl_keys.setToolTip(
            f"固件发的是 {V_FWD:.2f} m/s，但孪生在那个速度上直着走都会摔。"
            f"实测（5 种子 x 12 秒）：原样 0.10 m/s 跑满、0.15 全摔；"
            f"修正编码器后 0.35 跑满、0.40 全摔。所以这里压到悬崖之下。"
            f"\n转向不随速度削权——同一次扫描里满转向不影响存活。"
            f"\nThe firmware commands {V_FWD:.2f} m/s but the twin falls at it "
            f"going straight; the cap is set just under the measured cliff. "
            f"Turning is not tapered -- it costs nothing.")
        side.addWidget(self.lbl_keys)
        for t in (f"<b>A / D</b> 左右转 turn ±{YAW_CMD:.1f} rad/s",
                  "<b>空格 Space</b> 随机方向踹一脚 / random shove",
                  "<b>R</b> 立刻扶正 / stand up now",
                  "<b>P</b> 暂停 / 继续 · pause / resume"):
            lb = QtWidgets.QLabel(t)
            lb.setObjectName("cap")
            side.addWidget(lb)
        self.lbl_cmd = QtWidgets.QLabel("指令 cmd 0.00 m/s · 0.00 rad/s")
        self.lbl_cmd.setStyleSheet("color:#63b3ed;font-weight:bold;")
        side.addWidget(self.lbl_cmd)
        side.addStretch(1)
        lay.addLayout(side, 1)
        return g

    # ------------------------------------------------------------------
    # controller / environment
    # ------------------------------------------------------------------
    def _apply_controller(self):
        key = self.cb_ctrl.currentData() or "pid"
        # 下拉框选的那一档；默认 kalman（main.c 的 GET_Angle_Way=2）。
        imu = self.cb_imu.currentData() or "kalman"
        self.rl_policy = None
        gains = None
        # 换控制器会重建 core（car_state 回到 None），窗口这边必须跟着忘掉，
        # 否则 _set_car_state 会因为「状态没变」短路，新 core 永远收不到指令。
        # The core is rebuilt on every controller change; forget the cached
        # state or _set_car_state short-circuits and the new core never hears.
        self._car_state = None

        if key == "lqr":
            fw = MODE_STM32_LQR
            desc = ("原厂 app_control.c 的 K1~K6，含两个照抄的固件 bug："
                    "偏航估计小 10⁶ 倍、LQR 环把速度读低 15%。  |  K1..K6 straight from "
                    "app_control.c, including the two reproduced firmware "
                    "bugs: the yaw-rate estimate is 10^6 too small, and the "
                    "LQR loop under-reads speed by 15%.")
        elif key == "pid":
            fw = MODE_STM32_PID
            self.pid_set = self.cb_pid.currentData()
            gains = pid_gains(self.pid_set)
            m = PID_GAIN_SETS[self.pid_set]
            src = m["source"]
            extra = (f"  转向环在真车上是 {m['turn_loop']}（靠传感器），孪生没有那个"
                     f"传感器，只跑遥控通路。" if m["turn_loop"] != "Turn_PD" else "")
            desc = (f"{m['label']}：Kp {gains['balance_kp']:.0f} / Kd {gains['balance_kd']:.2f} / "
                    f"Vkp {gains['velocity_kp']:.1f} / Vki {gains['velocity_ki']:.3f} / "
                    f"Tkp {gains['turn_kp']:.0f}，中值 {gains['mid_angle_deg']:+.0f}°，"
                    f"前倾保护 {gains['angle_max_deg']:.0f}°，遥控速度 "
                    f"{gains['car_target_velocity']:.0f}/{gains['car_turn_amplitude']:.0f}，"
                    f"死区补偿 {self.core.fw.deadband_comp if hasattr(self.core, 'fw') else ''}。"
                    f"{extra}\n"
                    f"直立 PD + 速度 PI + 转向 PD，参数取自 {src}。"
                    f"Balance_Kd 作用在未换算的 16 位陀螺寄存器上。  |  Upright PD + "
                    f"velocity PI + steering PD, gains from {src}. Balance_Kd "
                    "acts on the raw 16-bit gyro register, not a scaled rate.")
        elif key == "ideal":
            fw = MODE_STM32_PID
            self.pid_set = self.cb_pid.currentData()
            gains = pid_gains(self.pid_set)
            m = PID_GAIN_SETS[self.pid_set]
            desc = (f"【理想模型】{m['label']}：Kp {gains['balance_kp']:.0f} / Kd {gains['balance_kd']:.2f} / "
                    f"Vkp {gains['velocity_kp']:.1f} / Vki {gains['velocity_ki']:.3f}。\n"
                    "同一个车体、同一套固件 PID 公式，关掉：死区失配、PWM 延迟、PWM 限幅、"
                    "驱动器电流限、反电动势与阻尼、传动弹性与间隙、卡尔曼 / DLPF / 量化 / "
                    "噪声 / 安装零偏（角度和角速度取真值）。力矩 = 补偿以上的 PWM × "
                    "0.568/1380 N·m。姿态滤波下拉框在这一档不起作用。  |  Same body and "
                    "firmware PID with every non-ideal effect switched off.")
        elif key == "autoload":
            from ..firmware.load_sched import NORMAL, HEAVY
            fw = MODE_STM32_PID
            gains = NORMAL.gains()      # 真正的增益由检测器每拍写入
            desc = ("开机用 HEAVY，检测到空车后自动切 NORMAL。判决只在静止时"
                    "更新（行驶时驾驶动作会污染抖动统计量）。\n"
                    f"NORMAL kp {NORMAL.balance_kp:.0f}/kd {NORMAL.balance_kd:.2f}"
                    f"/vkp {NORMAL.velocity_kp:.0f}   "
                    f"HEAVY kp {HEAVY.balance_kp:.0f}/kd {HEAVY.balance_kd:.2f}"
                    f"/vkp {HEAVY.velocity_kp:.0f}\n"
                    "拖动载重滑块，看右下角档位跟着变。空车下开机会先抖一两秒"
                    "——那是 HEAVY 用在空车上的症状，也正是检测信号。  |  "
                    "Boots in HEAVY, switches to NORMAL once it detects an "
                    "empty car; the verdict only updates while stationary.")
        elif key == "tuned":
            fw = self.tuned_gains["firmware"]
            gains = dict(self.tuned_gains["gains"])
            desc = (f"交叉熵法在难度 0.75/1.0 上搜出来的增益，"
                    f"评测种子与调参种子不相交。  |  Cross-entropy-method gains searched at "
                    "difficulty 0.75/1.0; the evaluation seeds are disjoint "
                    "from the tuning seeds.")
        elif key.startswith("goto:"):
            gg = self.goto_gains[int(key[5:])]
            fw = MODE_STM32_PID
            # 只把 6 个固件增益喂给底座；nav_* 是导航外环的，GUI 没有定点游戏，
            # 只在说明里显示。 The nav_* gains belong to the outer loop, which
            # the GUI does not run; only the six firmware gains go to the core.
            gains = {k: v for k, v in gg["gains"].items()
                     if not k.startswith("nav_")}
            gains.setdefault("mid_angle_deg", 1.0)
            pv = gg["prov"]
            nav = " ".join(f"{k}={gg['gains'][k]:.3g}"
                           for k in ("nav_kv", "nav_kw") if k in gg["gains"])
            desc = (f"[{os.path.basename(gg['path'])}]  "
                    f"{pv.get('steps', '?')} 局 · {pv.get('backend', '?')} 后端 · "
                    f"{pv.get('when', '未记录')}\n"
                    f"PPO 在「开到随机目标点并停稳」上搜出来的**一组静态增益**，"
                    f"训练时载重在 0/1/2/3/4 kg 之间轮换，选参按最差载重打分，"
                    f"所以载重滑块怎么拖都用这一套。导航外环增益（GUI 不用）：{nav}。"
                    f"  |  One static gain set from the goto game, searched "
                    f"across the whole 0-4 kg payload ladder and selected on "
                    f"worst-case payload, so the payload slider needs no "
                    f"model switch. Outer-loop gains (unused here): {nav}.")
        elif key.startswith("rl:"):
            path = key[3:]
            try:
                self.rl_policy = STM32InferencePolicy(path)
            except Exception as e:
                self.lbl_ctrl.setText(f"策略加载失败 / failed to load policy: {e}")
                return
            fw = self.rl_policy.firmware
            pol = self.rl_policy
            desc = (f"[{os.path.basename(path)}]  "
                    f"{pol.steps:,} 步 · {pol.backend or '?'} 后端 · "
                    f"{pol.n_envs or '?'} 并行环境 · 训练于 "
                    f"{pol.trained_utc or '未记录'}\n"
                    f"PPO 策略每 40 ms 改写一次底层增益（动作 = 相对固件值的"
                    f"对数倍数，0 就是原厂）。底座固件：{fw}。  |  The PPO policy rewrites "
                    f"the firmware gains every 40 ms (action = log multiplier "
                    f"on the stock value, so 0 is the firmware verbatim). "
                    f"Base firmware: {fw}.")
            # A policy is only meaningful on the twin it was trained against,
            # so compare rather than assume: the npz records its own filter.
            trained_on = self.rl_policy.imu_filter
            if trained_on != imu:
                desc += (f"  ⚠ 这个策略是在「{trained_on}」姿态模型上训的，"
                         f"现在跑的是「{imu}」。观测分布不一样，成绩不作数——"
                         f"要么把上面的姿态滤波切回 {trained_on}，要么重训： / trained on "
                         f"{trained_on}, running on {imu}: the score does not "
                         f"count. Switch back or retrain with "
                         f"python scripts/train_stm32_rl.py --imu {imu}")
        else:
            fw = MODE_STM32_LQR
            desc = ""

        self.mode = key
        self.row_pid.setVisible(key in ("pid", "ideal"))
        self.cb_imu.setEnabled(key != "ideal")
        self.core.firmware_name = fw
        # 站定版策略要求孪生打开站定外环（出厂固件没有的那一环）。必须走参数，
        # 因为 _init_firmware 会用默认值覆盖直接赋的属性。
        # A station-hold policy needs the outer loop the firmware lacks -- pass
        # it as an argument, since _init_firmware overwrites the attribute.
        self.core._init_firmware(firmware=fw, gains=gains, imu_filter=imu,
                                 ideal=(key == "ideal"),
                                 hold_station=bool(
                                     self.rl_policy is not None
                                     and self.rl_policy.hold_station))
        # 自动切档：_init_firmware 会新建固件对象，所以必须在它之后打开，
        # 否则检测器写的增益会被覆盖掉。
        # Must come after _init_firmware, which rebuilds the firmware object.
        if hasattr(self.core, "enable_load_detect"):
            self.core.enable_load_detect(key == "autoload")
        self.lbl_ctrl.setText(desc)
        self.last_action = np.zeros(
            self.rl_policy.gain_space.dim if self.rl_policy else 6,
            dtype=np.float64)
        self.reset_car(manual=True)
        self.view.setFocus()

    def _apply_environment(self):
        n = self.sl_noise.value() / 100.0
        wind = self.sl_wind.value() / 10.0
        wdir = np.deg2rad(self.sl_wdir.value())
        tq = self.sl_torque.value() / 100.0
        self.core.battery_v = self.sl_batt.value() / 10.0

        self.core.dist.set_config(DisturbanceConfig(
            noise_pitch=0.015 * n,
            noise_pitch_rate=0.20 * n,
            noise_vel=0.05 * n,
            noise_yaw_rate=0.10 * n,
            imu_bias_walk=0.004 * n,
            torque_noise=0.06 * tq,
            torque_scale_err=0.0,
            wind_force=wind,
            wind_dir=wdir,
            ground_slip=0.0,        # handled by real contact friction below
        ))
        self._apply_friction()
        self._apply_payload()

    def _apply_payload(self):
        """载重滑块立刻生效：重算质量/质心/惯量，两个后端都更新。

        set_payload() 本身只在下次 reset 生效，但滑块要的是所见即所得，所以
        这里直接把 robot 参数换掉、解析动力学重建；MuJoCo 后端则改 body 的
        质量、质心位置和惯量张量——重建整个模型会丢掉当前状态。
        Applied live: rebuilding the whole MuJoCo model would drop the state,
        so the chassis body's mass, COM and inertia are edited in place.
        """
        kg = self.sl_load.value() / 10.0
        core = self.core
        core.set_payload(kg)
        base = core.robot_nominal
        core.robot = base.with_payload(kg, core.payload_height) if kg > 0 else base
        p = core.robot
        if hasattr(core, "dyn"):
            from balance_bot.dynamics import BalanceBotDynamics
            core.dyn = BalanceBotDynamics(p)
        if hasattr(core, "pid"):
            core.pid.tau_max = p.tau_max
        mj, model = self._mj(), getattr(self.core, "model", None)
        if mj is not None and model is not None:
            bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "chassis")
            if bid >= 0:
                model.body_mass[bid] = p.m_body
                model.body_ipos[bid] = (0.0, 0.0, p.l_com)
                hw = 0.5 * p.track
                i_diam = 0.25 * p.m_wheel * p.r_wheel ** 2
                izz = max(p.I_yaw - 2.0 * (p.m_wheel * hw ** 2 + i_diam), 1e-4)
                model.body_inertia[bid] = (p.I_body, p.I_body, izz)

    def _apply_friction(self):
        """粗糙度写成接触摩擦系数，不是把力矩打折。
        Roughness as a contact property, not a torque discount."""
        mu = self.sl_slip.value() / 100.0      # 滑块就是 μ x 100
        self._apply_mech_friction()
        # 解析后端没有接触模型，只能用「牵引力损失比例」近似。以出厂的
        # μ=1.0 为基准：比它粗糙就是 0（不打滑），比它光滑按比例折算。
        # The analytic backend has no contacts, so scale traction loss against
        # the shipped μ=1.0 rather than against the slider's own end points.
        frac = float(max(0.0, 1.0 - mu / MU_BASELINE))
        model = getattr(self.core, "model", None)
        if model is None or self._mj() is None:
            # analytic backend has no contact model; fall back to the
            # traction-loss term the plant does understand
            self.core.dist.cfg.ground_slip = float(frac)
            return
        mj = self._mj()
        for name in ("floor", "gw_l", "gw_r"):
            gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
            if gid >= 0:
                model.geom_friction[gid, 0] = mu

    def _apply_mech_friction(self):
        """滚动阻力和传动摩擦。前者是接触模型的滚动摩擦（需要 condim 6），
        后者同时缩放减速箱库仑摩擦和转子静/动摩擦。
        Rolling resistance (needs condim 6) and the drivetrain friction
        multiplier, which scales the gearbox and rotor friction together."""
        core = self.core
        rr = self.sl_roll.value() * ROLL_STEP_M
        kf = self.sl_dfric.value() / 100.0
        # 写在实例上，不改模块全局：重建模型时 _apply_model_mode 会读回来。
        core.rolling_resist_m = rr
        core.rotor_fric_static = ROTOR_FRIC_STATIC * kf
        core.rotor_fric_kinetic = ROTOR_FRIC_KINETIC * kf
        cal = getattr(getattr(core, "motor", None), "cal", None)
        if cal is not None and hasattr(cal, "gear_friction"):
            cal.gear_friction = GEAR_FRICTION_BASE * kf
        model = getattr(core, "model", None)
        mj = self._mj()
        if model is None or mj is None:
            return
        ideal = bool(getattr(core, "_ideal_on", lambda: False)())
        fl = 0.0 if ideal else GEAR_FRICTION_BASE * kf
        model.dof_frictionloss[6] = fl
        model.dof_frictionloss[7] = fl
        rr_on = 0.0 if ideal else rr
        for name in ("gw_l", "gw_r", "floor"):
            gid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, name)
            if gid < 0:
                continue
            if rr_on > 0.0:
                model.geom_condim[gid] = 6
                model.geom_friction[gid, 2] = rr_on
            else:
                model.geom_condim[gid] = 3
                model.geom_friction[gid, 2] = 0.0001

    def _mj(self):
        return getattr(self.core, "_mj", None)

    # ------------------------------------------------------------------
    # episode control
    # ------------------------------------------------------------------
    def _clear_scores(self):
        self.history.clear()
        self.run_index = 0
        self.best_steps = 0
        self.total_falls = 0
        self.list_runs.clear()
        self._update_score()

    def reset_car(self, manual=False):
        keep = None
        if self.reset_in_place and not manual:
            st = self.core.state
            keep = (float(st[IX]), float(st[IY]), float(st[IPSI]))
        self.core.reset()
        if keep is not None:
            self._place(*keep)
        self.core.set_command(0.0, 0.0)
        self.cmd_v = self.cmd_yaw = 0.0
        self.target_v = self.target_yaw = 0.0
        self.stick_owned = False
        self.stick.set_external(0.0, 0.0)
        self.run_steps = 0
        self.fallen = False
        self._recover_left = 0
        self.last_action = np.zeros(
            self.rl_policy.gain_space.dim if self.rl_policy else 6,
            dtype=np.float64)
        self._apply_friction()
        self.scope.clear()
        self._update_score()

    def _place(self, x, y, psi):
        """Stand the car back up where it fell, facing where it faced."""
        mj = self._mj()
        data = getattr(self.core, "data", None)
        if mj is None or data is None:
            return
        from ..backends.mujoco_backend import quat_to_rpy
        _, pitch, _ = quat_to_rpy(data.qpos[3:7])
        cy, sy = np.cos(0.5 * psi), np.sin(0.5 * psi)
        cp, sp = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
        q = np.array([cy * cp, -sy * sp, cy * sp, sy * cp])
        data.qpos[0:2] = [x, y]
        data.qpos[3:7] = q / np.linalg.norm(q)
        mj.mj_forward(self.core.model, data)
        self.core._sync_state()
        if getattr(self.core, "imu_filter", "") == "kalman":
            self.core.imu.seed(float(self.core.state[ITH]))

    def kick(self, angle=None):
        f = self.sl_kick.value() / 10.0
        d = (float(np.random.uniform(-np.pi, np.pi)) if angle is None
             else float(angle))
        self.core.dist.fire_impulse(force=f, direction=d,
                                    duration=self.sl_kickms.value() / 1000.0)
        self.banner.setText(
            f"【冲击 KICK】{f:.1f} N，方向 {np.degrees(d):+.0f}°（0° = 车头方向），"
            f"持续 {self.sl_kickms.value()} ms  |  impulse {f:.1f} N at "
            f"{np.degrees(d):+.0f}deg for {self.sl_kickms.value()} ms")

    # ------------------------------------------------------------------
    # input
    # ------------------------------------------------------------------
    def eventFilter(self, obj, ev):
        t = ev.type()
        if t == EV_KEYPRESS and not ev.isAutoRepeat():
            k = ev.key()
            self.keys.add(k)
            if k == KEY.Key_Space:
                self.kick(None)
            elif k == KEY.Key_R:
                self.reset_car(manual=True)
            elif k == KEY.Key_P:
                self.paused = not self.paused
                self.banner.setText("已暂停（P 继续） / paused (P to resume)" if self.paused
                                    else "继续运行 / running")
            self._keys_to_command()
        elif t == EV_KEYRELEASE and not ev.isAutoRepeat():
            self.keys.discard(ev.key())
            self._keys_to_command()
        return super().eventFilter(obj, ev)

    def v_max(self) -> float:
        """能持续跑住的前进速度。
        The forward speed the car can actually sustain."""
        return V_MAX

    def yaw_limit(self, v: float) -> float:
        """当前速度下允许的最大偏航角速度：停住能原地转，开起来按半径画弧。
        Max yaw rate at this speed: pivot when stopped, fixed-radius arc when
        driving."""
        frac = min(1.0, abs(v) / max(self.v_max(), 1e-6))
        return min(YAW_CMD, YAW_CMD * (1.0 - frac) + abs(v) / TURN_RADIUS)

    def _keys_to_command(self):
        """Keys pick a target; _drive_step walks the command toward it.

        Releasing every key targets zero, which is the case the old early
        return skipped -- that is why the car used to keep going.

        W+D 这类组合直接相加，转向不再随速度削权，所以按住 W 再按 D 就是一条
        平滑的弧线——和开车一样。 / W+D simply add; the turn is no longer
        tapered by speed, so holding W and D traces a smooth arc.
        """
        K = KEY
        if self._factory_ctl():
            # 原厂蓝牙通路：单一离散状态，没有斜坡，幅值固定。
            # app_bluetooth.c 一次只解出一个 g_newcarstate，所以同时按下要
            # 定优先级：前/后优先于左/右——这也对应固件的实际限制，
            # **enLEFT/enRIGHT 根本不设 Movement**，原厂转向就是零前进下转的。
            # One state at a time, as the firmware parses it; forward/back wins.
            if self.keys & {K.Key_Q}:
                st = CAR_TLEFT
            elif self.keys & {K.Key_E}:
                st = CAR_TRIGHT
            elif self.keys & {K.Key_W, K.Key_Up}:
                st = CAR_RUN
            elif self.keys & {K.Key_S, K.Key_Down}:
                st = CAR_BACK
            elif self.keys & {K.Key_A, K.Key_Left}:
                st = CAR_LEFT
            elif self.keys & {K.Key_D, K.Key_Right}:
                st = CAR_RIGHT
            else:
                st = CAR_STOP
            self._set_car_state(st)
            return
        v = yaw = 0.0
        vmax = self.v_max()
        if self.keys & {K.Key_W, K.Key_Up}:
            v += vmax
        if self.keys & {K.Key_S, K.Key_Down}:
            v -= vmax * V_BACK_FRAC
        # Firmware sign: enLEFT is Target_gyro_z = -4, enRIGHT is +4.
        if self.keys & {K.Key_A, K.Key_Left}:
            yaw -= YAW_CMD
        if self.keys & {K.Key_D, K.Key_Right}:
            yaw += YAW_CMD
        self.target_v = v
        self.target_yaw = float(np.clip(yaw, -self.yaw_limit(v), self.yaw_limit(v)))
        self.stick_owned = False

    @staticmethod
    def _approach(cur, target, accel, brake, dt):
        """Move cur toward target, braking faster than accelerating."""
        rate = brake if (abs(target) < abs(cur) or cur * target < 0.0) else accel
        step = rate * dt
        if abs(target - cur) <= step:
            return target
        return cur + step * (1.0 if target > cur else -1.0)

    def _drive_step(self, dt):
        """One frame of throttle/steering ramp."""
        if self._factory_ctl():
            return          # 原厂没有斜坡，指令是阶跃的 / the factory steps
        if self.stick_owned:
            return                      # the joystick writes the command itself
        v = self._approach(self.cmd_v, self.target_v, V_ACCEL, V_BRAKE, dt)
        yaw = self._approach(self.cmd_yaw, self.target_yaw,
                             YAW_ACCEL, YAW_BRAKE, dt)
        if v != self.cmd_v or yaw != self.cmd_yaw:
            self._set_command(v, yaw)
            self.stick.set_external(v / max(self.v_max(), 1e-6), yaw / YAW_CMD)

    def _on_stick(self, fwd, yaw):
        # 拖动本来就是连续手势，所以它直接写指令；斜坡是给按键用的。
        # Dragging is already a continuous gesture, so it sets the command
        # outright; the ramp exists to give the keys one.
        if self._factory_ctl():
            # 摇杆推多深不影响幅值——原厂只有固定的 25/30。按最大分量定状态。
            # Depth does not scale anything; the factory has fixed magnitudes.
            f, y = float(fwd), float(yaw)
            if max(abs(f), abs(y)) < 0.15:
                self._set_car_state(CAR_STOP)
            elif abs(f) >= abs(y):
                self._set_car_state(CAR_RUN if f > 0 else CAR_BACK)
            else:
                self._set_car_state(CAR_RIGHT if y > 0 else CAR_LEFT)
            return
        self.stick_owned = abs(fwd) > 1e-3 or abs(yaw) > 1e-3
        f = float(fwd)
        v = f * self.v_max() * (1.0 if f >= 0.0 else V_BACK_FRAC)
        self.target_v = v
        self.target_yaw = float(yaw) * self.yaw_limit(v)
        self._set_command(self.target_v, self.target_yaw)

    def _refresh_speed_cap(self):
        """填一次 W/S 的速度标签，并重算当前按键的目标值。
        Refresh the W/S label and re-evaluate whatever is held down."""
        vmax = self.v_max()
        self.lbl_keys.setText(
            f"<b>W / S</b> 前进 fwd +{vmax:.2f} / 后退 back "
            f"-{vmax * V_BACK_FRAC:.2f} m/s")
        if not self.stick_owned:
            self._keys_to_command()

    def _factory_ctl(self) -> bool:
        """当前底座是不是 PID 工程——只有它才有蓝牙那套离散状态。
        True when the base is the PID build, which is what ships the
        Bluetooth state machine.  The LQR project uses a different path
        (Target_x_speed / Target_gyro_z), so it keeps the continuous command.
        """
        return getattr(self.core, "firmware_name", None) == MODE_STM32_PID

    def _set_car_state(self, state):
        if getattr(self, "_car_state", None) == state:
            return
        self._car_state = state
        self.core.set_car_state(state)
        self.cmd_v = self.cmd_yaw = 0.0
        names = {CAR_STOP: "停止 stop", CAR_RUN: "前进 enRUN",
                 CAR_BACK: "后退 enBACK", CAR_LEFT: "左转 enLEFT",
                 CAR_RIGHT: "右转 enRIGHT", CAR_TLEFT: "原地左转 enTLEFT",
                 CAR_TRIGHT: "原地右转 enTRIGHT"}
        from ..firmware.twin_baseline import CAR_STATE_CMD
        mv, tt, kd = CAR_STATE_CMD[state]
        self.lbl_cmd.setText(
            f"原厂蓝牙状态 {names[state]}  ·  Movement {mv:+.0f} · "
            f"Turn_Target {tt:+.0f} · Turn_Kd {'开' if kd else '关'}")

    def _set_command(self, v, yaw):
        self.cmd_v, self.cmd_yaw = float(v), float(yaw)
        self.core.set_command(self.cmd_v, self.cmd_yaw)
        self.lbl_cmd.setText(
            f"指令 cmd {self.cmd_v:+.2f} m/s · {self.cmd_yaw:+.2f} rad/s")

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def _tick(self):
        if self.paused:
            self._draw()
            return

        if self._recover_left > 0:
            self._recover_left -= 1
            if self._recover_left == 0:
                self.reset_car()
            self._draw()
            return

        # Throttle/steering ramp runs before the plant, so the command the
        # firmware sees this frame is the one the keys are asking for now.
        self._drive_step(self.TICK_MS / 1000.0)

        if self.rl_policy is not None:
            self._apply_policy_gains()

        _, _, term, trunc, info = self.core.step()

        if info["fell"]:
            if not self.fallen:
                self.fallen = True
                self.total_falls += 1
                self._record_run(self.run_steps)
                if self.auto_reset:
                    self._recover_left = self.RECOVER_TICKS
                    self.banner.setText(
                        f"【倾覆 FELL】本次存活 {self.run_steps} 步 "
                        f"({self.run_steps * 0.04:.1f} s) — 正在自动扶起… / auto-righting…")
                else:
                    self.banner.setText(
                        f"【倾覆 FELL】本次存活 {self.run_steps} 步。按 R 复位"
                        f"（或勾选“倒下自动回正”）。  |  fell after {self.run_steps} steps; "
                f"press R to reset, or tick auto-right.")
        else:
            self.run_steps += 1
            if self.run_steps > self.best_steps:
                self.best_steps = self.run_steps
            self._status(info)

        self._update_score()
        # duty is divided by ten so a 135 %-duty spike cannot autoscale the
        # pitch trace flat -- the LQR build really does reach 3900/2880
        self.scope.push([np.degrees(info["pitch"]),
                         info["ccr"][0] / 2880.0 * 10.0])
        self._draw()

    def _apply_policy_gains(self):
        st = self.core.state
        # 观测的搭法和增益写回都在策略对象里，UI / bench / 训练共用一份。
        # The observation and the write-back live in the policy object, shared
        # by the UI, the bench and the training env.
        self.last_action = self.rl_policy.drive(
            self.core, self.last_action, self.cmd_v, self.cmd_yaw)

    def _record_run(self, steps):
        self.history.append(int(steps))
        self.run_index += 1
        self.list_runs.insertItem(
            0, f"#{self.run_index:<4d} {steps:5d} 步 steps  "
               f"{steps * 0.04:6.2f} s   [{self.cb_ctrl.currentText()[:18]}]")
        if self.list_runs.count() > 200:
            self.list_runs.takeItem(self.list_runs.count() - 1)

    def _status(self, info):
        ccr_l, ccr_r = info["ccr"]
        off = " · 电机已断电 motors cut (Turn_Off)" if info["motors_off"] else ""
        self.banner.setText(
            f"θ {np.degrees(info['pitch']):+5.1f}°  "
            f"v {info['v']:+5.2f} m/s  "
            f"ψ̇ {info['yaw_rate']:+5.2f} rad/s  |  "
            f"PWM L {ccr_l:+5d} R {ccr_r:+5d} "
            f"({abs(ccr_l) / 2880.0 * 100:4.0f}% 占空比 duty){off}"
            + self._load_mode_text())

    def _load_mode_text(self):
        """自动切档打开时，把当前档位和检测统计量显示出来。"""
        det = getattr(self.core, "load_detect", None)
        if det is None:
            return ""
        m = getattr(self.core, "load_mode", None)
        return (f"  |  档位 {('HEAVY' if det.heavy else 'NORMAL'):<6s} "
                f"kd {getattr(self.core.fw, 'g', {}).get('balance_kd', 0):.2f}  "
                f"抖动 {det.rms_deg_s:5.1f}°/s")

    def _update_score(self):
        self.lbl_cur.setText(f"{self.run_steps}")
        self.lbl_best.setText(f"{self.best_steps}")
        self.lbl_falls.setText(f"{self.total_falls}")
        if self.history:
            h = np.asarray(self.history, dtype=float)
            self.lbl_stats.setText(
                f"{len(h)} 个回合 runs：均值 mean {h.mean():.1f} 步 "
                f"({h.mean() * 0.04:.1f} s) · 最差 worst {int(h.min())} 步 · "
                f"标准差 sd {h.std():.1f}\n"
                f"存活步数是长尾分布，比均值更值得看的是最差回合。  |  Survival is "
              "long-tailed; the worst run tells you more than the mean.")
        else:
            self.lbl_stats.setText(
                f"本次已站 {self.run_steps * 0.04:.1f} s，尚无完整回合 / "
              f"standing {self.run_steps * 0.04:.1f} s, no completed run yet")

    def _draw(self):
        data = getattr(self.core, "data", None)
        if data is None:
            return
        st = self.core.state
        self.view.draw(data, lookat=(st[IX], st[IY], 0.055))

    # ------------------------------------------------------------------
    def closeEvent(self, ev):
        self.timer.stop()
        self.view.closeRenderer()
        ev.accept()


# ==========================================================================
def run_ui(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("policy", nargs="?", default="",
                    help="a policy_*.npz to preselect")
    ap.add_argument("--policy", dest="policy_opt", default="")
    ap.add_argument("--backend", choices=("mujoco", "analytic"),
                    default="mujoco")
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv[:1])
    win = SimWindow(policy_path=args.policy or args.policy_opt,
                    backend=args.backend)
    win.show()
    win.view.setFocus()
    return exec_app(app)


if __name__ == "__main__":
    sys.exit(run_ui())
