"""MJCF (MuJoCo) model generation.

The model is generated from the same :class:`RobotParams` / :class:`Arena`
objects that drive the analytic plant, so the two stay consistent: same
masses, same inertias, same wheel radius, same torque limits, same obstacle
layout.  Explicit ``<inertial>`` elements are used rather than letting MuJoCo
infer inertia from geometry -- otherwise the yaw inertia in particular drifts
away from the analytic model and the yaw gains stop transferring.
"""
# 积分器和接触刚度：integrator="RK4" solref="0.004 1" 曾经让这台车在 MuJoCo 里
# **一边跳一边横移**——原厂 PID 静止 60 秒漂 1.6 m，而同工况 analytic 后端只漂
# 11 mm，用户实测真车是守得住的。
#
# 诊断链（scratchpad 的三轮扫描）：
#   轮子净转角只有 60~70 mm 而车身滑了 1.16 m -> 不是控制环放任不管
#   改滚动/扭转摩擦，三档数字完全相同          -> 不是「滑」
#   某些时刻 ncon = 0，离地占比 13.6%          -> 是「跳」
#   只把 RK4 换掉，漂移 801 mm -> 2 mm         -> 根因是积分器
#
# MuJoCo 文档明确不建议 RK4 配接触：RK4 在中间点上对不连续的接触力求值，会往
# 系统里注入能量。换 Euler 后再把接触时间常数从 4 ms 放到 20 ms（步长
# dt_sub = 1/800 = 1.25 ms 的 16 倍，远高于建议的 2 倍下限）：
#
#   RK4  + 0.004   位置跨度 1215.9mm  漂移 801.3mm  俯仰 1.92°  离地 13.6%
#   Euler+ 0.020   位置跨度    6.3mm  漂移   1.9mm  俯仰 0.34°  离地  0.0%
#   analytic 对照  位置跨度   11.3mm  漂移   0.0mm  俯仰 0.44°
#
# 两个后端这才对得上，也才对得上真车。改这两个值之前先重跑那三轮扫描。
#
# RK4 with contacts injects energy (MuJoCo's docs advise against it); the car
# hopped 13.6 % of the time and ratcheted 1.6 m across the floor.  Euler plus a
# 20 ms contact time constant brings MuJoCo in line with both the analytic
# backend and the real car.
from __future__ import annotations

from .arena import Arena
from .params import RobotParams


def build_mjcf(robot: RobotParams, arena: Arena | None = None,
               timestep: float = 0.0025) -> str:
    p = robot
    hw = 0.5 * p.track                      # half track
    wheel_th = 0.014                        # wheel thickness
    # 轮子刚体绕自转轴的惯量：**只有轮盘**。折算转子惯量挂在关节 armature 上。
    #
    # 【2026-09-16 结构修正】原来是 I_spin = I_wheel（轮盘 + 转子），把转子当成
    # 轮子本身的一部分。物理上转子装在车身上、由减速箱驱动，它转多少取决于
    # 「轮子相对车身」的转角——车身前后摆而轮子不动时，转子也在转。MuJoCo 里
    # 表示这种惯量的正是关节 armature。平衡车恰好就是车身在摆的工况，放错位置
    # 影响极大（同一组参数：饱和 25.6% -> 0%，主频 15.3 Hz -> 1.8 Hz）。
    # Rotor inertia belongs on the joint (armature), not on the wheel body:
    # the rotor turns with wheel-relative-to-chassis, which is exactly what a
    # balancing chassis exercises.
    I_spin = 0.5 * p.m_wheel * p.r_wheel ** 2
    I_diam = 0.25 * p.m_wheel * p.r_wheel ** 2

    # body inertia expressed at its own COM; izz is padded so that the total
    # yaw inertia (body + the two offset wheels) matches RobotParams.I_yaw
    izz_body = max(p.I_yaw - 2.0 * (p.m_wheel * hw ** 2 + I_diam), 1e-4)
    ixx_body = p.I_body

    # 零件外观：CAD 解出来的盒子，只画不碰（contype/conaffinity=0，mass=0）。
    # 碰撞仍然由下面那个单独的 body 方块负责——外观细分不该改变物理。
    # 没有 part_boxes 时（通用机器人没有 3D 模型）这里是空串，视图退回单方块。
    #
    # Appearance only: contype/conaffinity 0 and mass 0, so subdividing the
    # look never changes the physics.  Collision stays with the single body
    # box below.  Empty for robots with no CAD.
    _PART_RGBA = {
        "chassis_plate": "0.30 0.34 0.40 1",
        "battery_box": "0.22 0.48 0.78 1",
        "battery_lid": "0.28 0.56 0.86 1",
        "radar_plate": "0.30 0.34 0.40 1",
        "acrylic_guard": "0.80 0.85 0.92 0.55",
        "ultrasonic": "0.85 0.80 0.35 1",
    }
    parts = ""
    for nm, x0, x1, y0, y1, z0, z1 in getattr(p, "part_boxes", ()):
        cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
        sx, sy, sz = (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2
        base = nm.rstrip("_1234567890")
        rgba = _PART_RGBA.get(nm, _PART_RGBA.get(base, "0.55 0.58 0.64 1"))
        parts += ('\n      '
                  f'<geom name="part_{nm}" type="box" '
                  f'pos="{cx:.4f} {cy:.4f} {cz:.4f}" '
                  f'size="{sx:.4f} {sy:.4f} {sz:.4f}" '
                  f'rgba="{rgba}" mass="0" contype="0" conaffinity="0"/>')

    obstacles = ""
    if arena is not None and len(arena.obstacles):
        for i, (ox, oy, orr) in enumerate(arena.obstacles):
            obstacles += (
                f'    <geom name="obs{i}" type="cylinder" '
                f'pos="{ox:.4f} {oy:.4f} 0.25" size="{orr:.4f} 0.25" '
                f'rgba="0.75 0.35 0.25 1" friction="1 0.005 0.0001"/>\n')

    walls = ""
    if arena is not None:
        hx, hy = arena.p.half_x, arena.p.half_y
        t = 0.05
        for name, (px, py, sx, sy) in {
            "wall_px": (hx, 0.0, t, hy + t),
            "wall_nx": (-hx, 0.0, t, hy + t),
            "wall_py": (0.0, hy, hx + t, t),
            "wall_ny": (0.0, -hy, hx + t, t),
        }.items():
            walls += (f'    <geom name="{name}" type="box" '
                      f'pos="{px:.3f} {py:.3f} 0.15" '
                      f'size="{sx:.3f} {sy:.3f} 0.15" '
                      f'rgba="0.35 0.38 0.45 0.55"/>\n')

    # 有细分零件时，那个碰撞方块就不该再挡着看了，调成近乎透明的线框感。
    # With parts drawn, the collision box should stop hiding them.
    body_rgba = "0.25 0.55 0.85 0.12" if parts else "0.25 0.55 0.85 1"

    return f"""<mujoco model="balance_bot">
  <compiler angle="radian" autolimits="true" balanceinertia="true"/>
  <option timestep="{timestep}" integrator="Euler" gravity="0 0 -9.81"
          cone="elliptic" impratio="10"/>

  <default>
    <geom friction="0.90 0.005 0.0001" solref="0.020 1" solimp="0.95 0.99 0.001"/>
    <joint damping="0"/>
  </default>

  <!-- The offscreen framebuffer defaults to 640x480, which silently caps
       mujoco.Renderer.  The in-window 3D view asks for more than that. -->
  <visual>
    <global offwidth="1920" offheight="1080"/>
    <quality shadowsize="2048" offsamples="4"/>
    <map znear="0.01" zfar="30"/>
  </visual>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient" width="256" height="256"
             rgb1="0.29 0.33 0.42" rgb2="0.08 0.09 0.12"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.22 0.24 0.28" rgb2="0.28 0.30 0.35"/>
    <material name="grid" texture="grid" texrepeat="12 12" reflectance="0.05"/>
  </asset>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <light pos="3 -3 3" dir="-0.5 0.5 -0.6" diffuse="0.35 0.35 0.4"/>
    <!-- 地面 μ=0.90：轮子是**花纹橡胶胎**，干燥瓷砖/木地板上实测区间 0.9-1.1，
         这是这台车最常跑的地面。以前写 1.0 是随手取的整数，从没对过真车；模式 1
         静止基准对 μ 几乎不敏感（0.7-1.5 综合误差 0.005-0.015），所以改这个数
         不会动已拟合的基线。GUI 的粗糙度滑块默认就停在这里。
         mu=0.90: treaded rubber on a dry tile or wood floor, the surface this
         car actually runs on.  The standstill benchmark barely identifies mu,
         so this does not move the fitted baseline. -->
    <geom name="floor" type="plane" size="0 0 0.05" material="grid"
          friction="0.90 0.005 0.0001"/>
{walls}{obstacles}
    <body name="chassis" pos="0 0 {p.r_wheel:.4f}">
      <freejoint name="root"/>
      <inertial pos="0 0 {p.l_com:.4f}" mass="{p.m_body:.4f}"
                diaginertia="{ixx_body:.6f} {ixx_body:.6f} {izz_body:.6f}"/>
      <geom name="body" type="box"
            pos="0 0 {p.box_center:.4f}"
            size="{0.5 * p.body_depth:.4f} {0.5 * p.body_width:.4f} {0.5 * p.body_height:.4f}"
            rgba="{body_rgba}" mass="0" contype="1" conaffinity="1"/>{parts}
      <geom name="nose" type="capsule" fromto="0 0 {p.l_com:.4f} {0.5 * p.body_depth + 0.05:.4f} 0 {p.l_com:.4f}"
            size="0.008" rgba="0.95 0.75 0.2 1" mass="0" contype="0" conaffinity="0"/>
      <site name="imu" pos="0 0 {p.l_com:.4f}" size="0.006"/>

      <body name="wheel_l" pos="0 {hw:.4f} 0">
        <joint name="jw_l" type="hinge" axis="0 1 0" armature="{p.I_rotor:.6g}"/>
        <inertial pos="0 0 0" mass="{p.m_wheel:.4f}"
                  diaginertia="{I_diam:.6f} {I_spin:.6f} {I_diam:.6f}"/>
        <geom name="gw_l" type="cylinder" size="{p.r_wheel:.4f} {wheel_th:.4f}"
              quat="0.7071068 0.7071068 0 0" rgba="0.15 0.15 0.18 1" mass="0"/>
      </body>

      <body name="wheel_r" pos="0 {-hw:.4f} 0">
        <joint name="jw_r" type="hinge" axis="0 1 0" armature="{p.I_rotor:.6g}"/>
        <inertial pos="0 0 0" mass="{p.m_wheel:.4f}"
                  diaginertia="{I_diam:.6f} {I_spin:.6f} {I_diam:.6f}"/>
        <geom name="gw_r" type="cylinder" size="{p.r_wheel:.4f} {wheel_th:.4f}"
              quat="0.7071068 0.7071068 0 0" rgba="0.15 0.15 0.18 1" mass="0"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="m_l" joint="jw_l" gear="1"
           ctrlrange="{-p.tau_max:.4f} {p.tau_max:.4f}"/>
    <motor name="m_r" joint="jw_r" gear="1"
           ctrlrange="{-p.tau_max:.4f} {p.tau_max:.4f}"/>
  </actuator>

  <sensor>
    <framequat name="s_quat" objtype="site" objname="imu"/>
    <gyro name="s_gyro" site="imu"/>
    <accelerometer name="s_acc" site="imu"/>
    <framelinvel name="s_vel" objtype="body" objname="chassis"/>
    <jointvel name="s_wl" joint="jw_l"/>
    <jointvel name="s_wr" joint="jw_r"/>
  </sensor>
</mujoco>
"""
