"""Custom QPainter widgets: arena view, side elevation, scope, gain bars.

Deliberately dependency-free (no pyqtgraph) -- one less thing to install, and
at 25 Hz with a few hundred points QPainter is more than fast enough.
"""
from __future__ import annotations

from collections import deque
import numpy as np

from .qtcompat import QtCore, QtGui, QtWidgets, ANTIALIAS, SOLID, DASH, NOPEN

C_BG = QtGui.QColor(24, 26, 32)
C_GRID = QtGui.QColor(45, 49, 58)
C_TEXT = QtGui.QColor(190, 196, 208)
C_ROBOT = QtGui.QColor(74, 154, 220)
C_ROBOT_BAD = QtGui.QColor(226, 96, 80)
C_OBS = QtGui.QColor(196, 108, 74)
C_RAY = QtGui.QColor(90, 190, 140, 110)
C_TRAIL = QtGui.QColor(74, 154, 220, 90)
SERIES_COLORS = [
    QtGui.QColor(74, 154, 220), QtGui.QColor(232, 170, 70),
    QtGui.QColor(120, 200, 130), QtGui.QColor(226, 96, 80),
    QtGui.QColor(170, 140, 230), QtGui.QColor(90, 200, 210),
    QtGui.QColor(235, 130, 180), QtGui.QColor(150, 160, 175),
    QtGui.QColor(210, 210, 120),
]


class ArenaView(QtWidgets.QWidget):
    """Top-down view: arena, obstacles, robot, lidar rays, trail."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 340)
        self.half = (4.0, 4.0)
        self.obstacles = np.zeros((0, 3))
        self.pose = (0.0, 0.0, 0.0)
        self.rays = None
        self.ray_max = 2.5
        self.radius = 0.11
        self.ok = True
        self.trail = deque(maxlen=900)

    def update_state(self, half, obstacles, pose, rays=None, ok=True,
                     radius=0.11, ray_max=2.5):
        if half != self.half or len(obstacles) != len(self.obstacles):
            self.trail.clear()
        self.half, self.obstacles, self.pose = half, obstacles, pose
        self.rays, self.ok, self.radius, self.ray_max = rays, ok, radius, ray_max
        self.trail.append((pose[0], pose[1]))
        self.update()

    def clear_trail(self):
        self.trail.clear()

    # ------------------------------------------------------------------
    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(ANTIALIAS)
        p.fillRect(self.rect(), C_BG)
        w, h = self.width(), self.height()
        hx, hy = self.half
        s = 0.92 * min(w / (2 * hx), h / (2 * hy))
        cx, cy = w / 2.0, h / 2.0

        def T(x, y):
            return QtCore.QPointF(cx + x * s, cy - y * s)

        # grid
        p.setPen(QtGui.QPen(C_GRID, 1, SOLID))
        g = 1.0
        n = int(hx / g)
        for i in range(-n, n + 1):
            p.drawLine(T(i * g, -hy), T(i * g, hy))
        n = int(hy / g)
        for i in range(-n, n + 1):
            p.drawLine(T(-hx, i * g), T(hx, i * g))

        # walls
        p.setPen(QtGui.QPen(QtGui.QColor(80, 88, 102), 2))
        p.drawRect(QtCore.QRectF(T(-hx, hy), T(hx, -hy)))

        # trail
        if len(self.trail) > 2:
            p.setPen(QtGui.QPen(C_TRAIL, 1.6))
            path = QtGui.QPainterPath(T(*self.trail[0]))
            for pt in list(self.trail)[1:]:
                path.lineTo(T(*pt))
            p.drawPath(path)

        # obstacles
        p.setPen(NOPEN)
        p.setBrush(C_OBS)
        for ox, oy, orr in self.obstacles:
            p.drawEllipse(T(ox, oy), orr * s, orr * s)

        # rays
        x, y, psi = self.pose
        if self.rays is not None and len(self.rays):
            p.setPen(QtGui.QPen(C_RAY, 1))
            k = len(self.rays)
            for i, d in enumerate(self.rays):
                a = psi + 2 * np.pi * i / k        # ray 0 = straight ahead
                p.drawLine(T(x, y), T(x + d * np.cos(a), y + d * np.sin(a)))

        # robot
        col = C_ROBOT if self.ok else C_ROBOT_BAD
        p.setBrush(QtGui.QBrush(col))
        p.setPen(QtGui.QPen(col.lighter(140), 1.5))
        r = self.radius * s
        p.drawEllipse(T(x, y), r, r)
        p.setPen(QtGui.QPen(QtGui.QColor(255, 215, 120), 2.5))
        p.drawLine(T(x, y), T(x + 1.7 * self.radius * np.cos(psi),
                              y + 1.7 * self.radius * np.sin(psi)))

        p.setPen(C_TEXT)
        p.drawText(8, 16, f"top-down   x={x:+.2f}  y={y:+.2f}  yaw={np.degrees(psi):+.0f}°")
        p.end()


class SideView(QtWidgets.QWidget):
    """Side elevation -- the inverted-pendulum view."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 190)
        self.theta = 0.0
        self.v = 0.0
        self.tau = (0.0, 0.0)
        self.tau_max = 0.6
        self.pitch_fail = 0.6
        self.l_com = 0.12
        self.r_wheel = 0.05
        self.body_h = 0.24
        # 侧面轮廓：((名字, 前后 y0, y1, 高 z0, z1), ...)，原点在轮轴，单位米。
        # 给了就按真实分层画，没给就退回一个方块（通用机器人没有 CAD）。
        # Per-part rectangles from the CAD; falls back to one box when absent.
        self.profile = ()
        self.kick = 0.0
        self.wheel_angle = 0.0

    def update_state(self, theta, v, tau, kick=0.0, dt=0.04, **geom):
        self.theta, self.v, self.tau, self.kick = theta, v, tau, kick
        for k, val in geom.items():
            setattr(self, k, val)
        self.wheel_angle += v / max(self.r_wheel, 1e-6) * dt
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(ANTIALIAS)
        p.fillRect(self.rect(), C_BG)
        w, h = self.width(), self.height()
        # 缩放按「轮子底到车顶」的真实总高算，否则分层轮廓会画出画布。
        top = max((z1 for _n, _a, _b, _z0, z1 in self.profile), default=0.0)
        s = 0.62 * h / (max(self.body_h, top + self.r_wheel) + self.r_wheel)
        cx, ground = w / 2.0, h - 30.0
        axle = QtCore.QPointF(cx, ground - self.r_wheel * s)

        p.setPen(QtGui.QPen(QtGui.QColor(70, 76, 90), 2))
        p.drawLine(QtCore.QPointF(0, ground), QtCore.QPointF(w, ground))

        # body
        bad = abs(self.theta) > 0.6 * self.pitch_fail
        col = C_ROBOT_BAD if bad else C_ROBOT
        p.save()
        p.translate(axle)
        p.rotate(np.degrees(self.theta))
        p.setBrush(QtGui.QBrush(col))
        p.setPen(QtGui.QPen(col.lighter(150), 1.5))
        if self.profile:
            # 真实分层：底盘、电池仓、铜柱、雷达板各画一块。屏幕坐标 y 向下，
            # 所以 CAD 的 z 要取负。
            # Screen y points down, so the CAD z flips sign.
            for _nm, y0, y1, z0, z1 in self.profile:
                p.drawRoundedRect(
                    QtCore.QRectF(y0 * s, -z1 * s,
                                  (y1 - y0) * s, (z1 - z0) * s), 2, 2)
        else:
            bw = 0.09 * s
            bh = self.body_h * s
            p.drawRoundedRect(QtCore.QRectF(-bw / 2, -bh, bw, bh), 4, 4)
        p.setBrush(QtGui.QColor(255, 215, 120))
        p.setPen(NOPEN)
        p.drawEllipse(QtCore.QPointF(0, -self.l_com * s), 3.5, 3.5)
        p.restore()

        # 轮子画在车身**之后**：真车的轮子在车体外侧，侧视图里挡在前面。
        # 车身前后 151.6 mm 而轮子直径只有 67 mm，先画轮子就会被完全盖住。
        # The wheels sit outboard of the body, so they occlude it in a side
        # view; drawn first they would vanish behind a 151.6 mm long chassis.
        rw = self.r_wheel * s
        p.setBrush(QtGui.QColor(40, 44, 52))
        p.setPen(QtGui.QPen(QtGui.QColor(120, 128, 145), 2))
        p.drawEllipse(axle, rw, rw)
        p.drawLine(axle, QtCore.QPointF(axle.x() + rw * np.cos(self.wheel_angle),
                                        axle.y() + rw * np.sin(self.wheel_angle)))

        # fail cone
        p.setPen(QtGui.QPen(QtGui.QColor(226, 96, 80, 110), 1, DASH))
        for sgn in (-1, 1):
            a = sgn * self.pitch_fail
            p.drawLine(axle, QtCore.QPointF(
                axle.x() + np.sin(a) * self.body_h * s,
                axle.y() - np.cos(a) * self.body_h * s))

        # torque bar
        t = 0.5 * (self.tau[0] + self.tau[1]) / max(self.tau_max, 1e-6)
        p.setPen(NOPEN)
        p.setBrush(QtGui.QColor(232, 170, 70))
        p.drawRect(QtCore.QRectF(cx, ground + 8, np.clip(t, -1, 1) * 70, 8))
        p.setPen(QtGui.QPen(QtGui.QColor(90, 96, 110), 1))
        p.drawRect(QtCore.QRectF(cx - 70, ground + 8, 140, 8))

        # impulse arrow
        if abs(self.kick) > 1e-6:
            p.setPen(QtGui.QPen(QtGui.QColor(240, 90, 90), 3))
            L = np.clip(self.kick, -1, 1) * 55
            y0 = axle.y() - self.l_com * s
            p.drawLine(QtCore.QPointF(cx - L, y0), QtCore.QPointF(cx, y0))

        p.setPen(C_TEXT)
        p.drawText(8, 16, f"pitch {np.degrees(self.theta):+6.2f}°   v {self.v:+.2f} m/s"
                          f"   τ {self.tau[0]:+.2f}/{self.tau[1]:+.2f} N·m")
        p.end()


class Scope(QtWidgets.QWidget):
    """Scrolling multi-trace plot."""

    def __init__(self, title, labels, ylim=(-1, 1), maxlen=500, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(120)
        self.title = title
        self.labels = labels
        self.ylim = ylim
        self.data = [deque(maxlen=maxlen) for _ in labels]
        self.autoscale = True

    def push(self, values):
        for d, v in zip(self.data, values):
            d.append(float(v))
        self.update()

    def clear(self):
        for d in self.data:
            d.clear()
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(ANTIALIAS)
        p.fillRect(self.rect(), C_BG)
        w, h = self.width(), self.height()
        pad_l, pad_t, pad_b = 46, 16, 4
        lo, hi = self.ylim
        if self.autoscale:
            vals = [v for d in self.data for v in d]
            if vals:
                m = max(abs(min(vals)), abs(max(vals)), 1e-3) * 1.15
                lo, hi = min(lo, -m), max(hi, m)
        span = max(hi - lo, 1e-9)

        def Y(v):
            return pad_t + (hi - v) / span * (h - pad_t - pad_b)

        p.setPen(QtGui.QPen(C_GRID, 1))
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            yy = pad_t + frac * (h - pad_t - pad_b)
            p.drawLine(pad_l, int(yy), w, int(yy))
        p.setPen(QtGui.QPen(QtGui.QColor(70, 76, 90), 1, DASH))
        if lo < 0 < hi:
            p.drawLine(pad_l, int(Y(0)), w, int(Y(0)))

        n = max((len(d) for d in self.data), default=0)
        if n > 1:
            dx = (w - pad_l) / (n - 1)
            for i, d in enumerate(self.data):
                if len(d) < 2:
                    continue
                p.setPen(QtGui.QPen(SERIES_COLORS[i % len(SERIES_COLORS)], 1.6))
                path = QtGui.QPainterPath(QtCore.QPointF(pad_l, Y(d[0])))
                for j, v in enumerate(list(d)[1:], start=1):
                    path.lineTo(pad_l + j * dx, Y(v))
                p.drawPath(path)

        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        p.setPen(C_TEXT)
        p.drawText(4, 12, self.title)
        p.drawText(4, int(Y(hi)) + 10, f"{hi:+.2f}")
        p.drawText(4, int(Y(lo)) - 2, f"{lo:+.2f}")
        x = pad_l + 6
        for i, lab in enumerate(self.labels):
            p.setPen(SERIES_COLORS[i % len(SERIES_COLORS)])
            p.drawText(x, 12, lab)
            x += p.fontMetrics().horizontalAdvance(lab) + 14
        p.end()


class GainBars(QtWidgets.QWidget):
    """Nine bars showing where each gain sits inside its allowed range."""

    def __init__(self, names, low, high, parent=None):
        super().__init__(parent)
        self.names = list(names)
        self.low = np.asarray(low, float)
        self.high = np.asarray(high, float)
        self.gains = self.low.copy()
        self.nominal = None
        self.setMinimumHeight(24 * len(self.names) + 10)

    def set_gains(self, gains, nominal=None):
        self.gains = np.asarray(gains, float)
        if nominal is not None:
            self.nominal = np.asarray(nominal, float)
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(ANTIALIAS)
        p.fillRect(self.rect(), C_BG)
        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        w = self.width()
        x0, x1 = 78, w - 62
        for i, name in enumerate(self.names):
            y = 8 + i * 24
            lo, hi = self.low[i], self.high[i]
            frac = float(np.clip((self.gains[i] - lo) / max(hi - lo, 1e-9), 0, 1))
            p.setPen(C_TEXT)
            p.drawText(4, y + 12, name)
            p.setPen(NOPEN)
            p.setBrush(QtGui.QColor(40, 44, 52))
            p.drawRoundedRect(QtCore.QRectF(x0, y + 2, x1 - x0, 12), 3, 3)
            c = SERIES_COLORS[i % len(SERIES_COLORS)]
            p.setBrush(c)
            p.drawRoundedRect(QtCore.QRectF(x0, y + 2, (x1 - x0) * frac, 12), 3, 3)
            if self.nominal is not None:
                nf = float(np.clip((self.nominal[i] - lo) / max(hi - lo, 1e-9), 0, 1))
                p.setPen(QtGui.QPen(QtGui.QColor(235, 235, 235, 170), 1.5))
                nx = x0 + (x1 - x0) * nf
                p.drawLine(QtCore.QPointF(nx, y), QtCore.QPointF(nx, y + 16))
                p.setPen(NOPEN)
            p.setPen(C_TEXT)
            p.drawText(x1 + 6, y + 12, f"{self.gains[i]:.3f}")
        p.end()


class Joystick(QtWidgets.QWidget):
    """Click-and-drag pad: up/down = speed, left/right = turn rate."""

    moved = QtCore.pyqtSignal(float, float) if hasattr(QtCore, "pyqtSignal") \
        else QtCore.Signal(float, float)

    DEAD = 0.12          # fraction of the radius that reads as centred

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(150, 150)
        self.pos_xy = [0.0, 0.0]
        self.sticky = False
        self._held = False

    def _emit(self, ev):
        w, h = self.width(), self.height()
        px = ev.position().x() if hasattr(ev, "position") else ev.x()
        py = ev.position().y() if hasattr(ev, "position") else ev.y()
        x = px / w * 2 - 1
        y = 1 - py / h * 2

        # Round, not square: a corner drag should not ask for 1.41x on both
        # axes.  Anything past the rim is pulled back onto it, keeping the
        # direction the user pointed.
        r = float(np.hypot(x, y))
        if r > 1.0:
            x, y = x / r, y / r
            r = 1.0

        # Dead zone, rescaled so the usable range still reaches full travel.
        if r < self.DEAD:
            x = y = 0.0
        elif r > 0.0:
            k = (r - self.DEAD) / (1.0 - self.DEAD) / r
            x, y = x * k, y * k

        self.pos_xy = [float(x), float(y)]
        # Sign matches the keys and the firmware: left is negative yaw.
        self.moved.emit(self.pos_xy[1], self.pos_xy[0])   # forward, yaw
        self.update()

    def mousePressEvent(self, ev):
        self._held = True
        self._emit(ev)

    def mouseMoveEvent(self, ev):
        if self._held:
            self._emit(ev)

    def mouseReleaseEvent(self, ev):
        self._held = False
        if not self.sticky:
            self.pos_xy = [0.0, 0.0]
            self.moved.emit(0.0, 0.0)
            self.update()

    def set_external(self, fwd, yaw):
        self.pos_xy = [float(np.clip(yaw, -1, 1)), float(np.clip(fwd, -1, 1))]
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(ANTIALIAS)
        p.fillRect(self.rect(), C_BG)
        w, h = self.width(), self.height()
        p.setPen(QtGui.QPen(C_GRID, 1))
        p.drawEllipse(QtCore.QRectF(4, 4, w - 8, h - 8))
        p.drawLine(w // 2, 4, w // 2, h - 4)
        p.drawLine(4, h // 2, w - 4, h // 2)
        d = self.DEAD * (w - 8) / 2
        p.drawEllipse(QtCore.QPointF(w / 2, h / 2), d, d)
        x = w / 2 + self.pos_xy[0] * (w / 2 - 12)
        y = h / 2 - self.pos_xy[1] * (h / 2 - 12)
        p.setPen(NOPEN)
        p.setBrush(C_ROBOT)
        p.drawEllipse(QtCore.QPointF(x, y), 9, 9)
        p.setPen(C_TEXT)
        f = p.font()
        f.setPointSize(7)
        p.setFont(f)
        p.drawText(6, 14, "拖动 drag · 上下=速度 · 左右=转向")
        p.end()
