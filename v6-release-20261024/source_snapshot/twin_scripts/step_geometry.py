# -*- coding: utf-8 -*-
"""Car geometry straight from the STEP model (2026-10-21: the STEP file is the authority
for shape and collision -- user's instruction).

Writes balance_bot/firmware/stm32_step_geom.py (plain numbers, no CAD libraries needed
at run time).  Needs a STEP reader only here:

    python -m venv stepenv && stepenv/Scripts/pip install cascadio trimesh numpy scipy
    stepenv/Scripts/python scripts/step_geometry.py

STEP frame: X left-right, Y fore-aft, Z up, millimetres.  The car's FRONT is -Y (the
ultrasonic module sits at Y -36..-17).  Output frame = the twin's chassis frame:
origin on the wheel axle (mid-way between the wheels), x forward, y left, z up, metres.

What STEP does NOT have: materials/masses and the battery cells (the battery holder is
empty).  Mass, centre of mass and inertia therefore are NOT taken from here.
"""
import collections
import io
import os
import re
import sys

import numpy as np
import trimesh
import cascadio

STEP = r"C:\Users\jiang li\Downloads\STM322\10.attch\3Dmodel\STM32_Balance_V2.STEP"
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "balance_bot", "firmware", "stm32_step_geom.py")
GLB = os.path.join(os.environ.get("TEMP", "."), "stm32_balance_v2.glb")
WHEEL_PART = "平衡车轮"
ENGLISH = {
    "减速电机L": "gear_motor", "平衡车轮": "wheel", "YB-EST01-V2.0平衡车": "main_board",
    "雷达固定板": "lidar_plate", "平衡车V2底盘": "chassis", "电池仓": "battery_holder",
    "蝙蝠车超声波": "ultrasonic", "电池仓盖": "battery_lid", "联轴器": "coupler",
    "超声波亚克力挡板": "ultrasonic_guard", "铜柱M3-32+6": "standoff",
}


def part_names(path):
    """NAUO id -> product name (names are GBK in this file)."""
    raw = open(path, "rb").read().decode("gbk", errors="replace").replace("\r", "").replace("\n", " ")
    ent = {m.group(1): m.group(2) for m in re.finditer(r"#(\d+)\s*=\s*(.*?)\)\s*;", raw)}
    prod = {k: re.match(r"PRODUCT\s*\(\s*'([^']*)'", v).group(1)
            for k, v in ent.items() if re.match(r"PRODUCT\s*\(", v)}
    pdf = {}
    for k, v in ent.items():
        m = re.match(r"PRODUCT_DEFINITION_FORMATION\w*\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,\s*#(\d+)", v)
        if m:
            pdf[k] = m.group(1)
    pd = {}
    for k, v in ent.items():
        m = re.match(r"PRODUCT_DEFINITION\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,\s*#(\d+)", v)
        if m and m.group(1) in pdf:
            pd[k] = prod.get(pdf[m.group(1)], "?")
    nauo = {}
    for k, v in ent.items():
        m = re.match(r"NEXT_ASSEMBLY_USAGE_OCCURRENCE\s*\(\s*'([^']*)'.*?#(\d+)\s*,\s*#(\d+)", v)
        if m:
            nauo[m.group(1)] = pd.get(m.group(3), "?")
    return nauo


def main():
    cascadio.step_to_glb(STEP, GLB, 0.1, 0.5)
    sc = trimesh.load(GLB)
    nauo = part_names(STEP)
    parts = []
    for node in sc.graph.nodes_geometry:
        T, g = sc.graph[node]
        m = sc.geometry[g].copy()
        m.apply_transform(T)
        parts.append((nauo.get(node.split("_")[0], node), m.vertices * 1000.0))   # mm

    wheels = [v for n, v in parts if n == WHEEL_PART]
    wv = np.vstack(wheels)
    z_axle = 0.5 * (wv[:, 2].min() + wv[:, 2].max())
    y_axle = 0.5 * (wv[:, 1].min() + wv[:, 1].max())
    r_wheel = 0.5 * (wv[:, 2].max() - wv[:, 2].min())
    # the two wheels: split the wheel vertices by X
    xs = np.sort(wv[:, 0])
    gap = np.argmax(np.diff(xs))
    w1, w2 = wv[wv[:, 0] <= xs[gap]], wv[wv[:, 0] > xs[gap]]
    c1 = 0.5 * (w1[:, 0].min() + w1[:, 0].max())
    c2 = 0.5 * (w2[:, 0].min() + w2[:, 0].max())
    x_mid = 0.5 * (c1 + c2)
    track = abs(c2 - c1)
    wheel_width = 0.5 * ((w1[:, 0].max() - w1[:, 0].min()) + (w2[:, 0].max() - w2[:, 0].min()))
    ground_z = wv[:, 2].min()

    def to_twin(v):          # mm, STEP frame -> m, twin frame (x fwd = -Y, y left = +X, z up)
        return np.c_[-(v[:, 1] - y_axle), v[:, 0] - x_mid, v[:, 2] - z_axle] / 1000.0

    body = [(n, to_twin(v)) for n, v in parts if n != WHEEL_PART]
    bv = np.vstack([v for _, v in body])
    hull = trimesh.convex.convex_hull(bv)
    hv = np.round(hull.vertices, 5)

    # ground contact: pitch the body about the axle (positive = nose down, i.e. the top
    # moves forward), find where its lowest point reaches the ground (-r_wheel)
    def contact(sign):
        for a in np.arange(0.0, 90.001, 0.05):
            t = np.radians(sign * a)
            z = hv[:, 2] * np.cos(t) - hv[:, 0] * np.sin(t)
            if z.min() <= -r_wheel / 1000.0:
                i = int(np.argmin(z))
                return a, hv[i]
        return None, None
    a_f, p_f = contact(+1)
    a_b, p_b = contact(-1)
    who = {}
    for lab, p in (("fwd", p_f), ("back", p_b)):
        if p is not None:
            best = min(body, key=lambda nv: np.min(np.linalg.norm(nv[1] - p, axis=1)))
            who[lab] = best[0]

    boxes = collections.OrderedDict()
    for n, v in body:
        key = ENGLISH.get(n, "screw" if "screw" in n.lower() else n)
        lo, hi = v.min(0), v.max(0)
        if key in boxes:
            lo = np.minimum(lo, boxes[key][0]); hi = np.maximum(hi, boxes[key][1])
        boxes[key] = (lo, hi)

    with io.open(OUT, "w", encoding="utf-8") as f:
        f.write('# -*- coding: utf-8 -*-\n')
        f.write('r"""GENERATED by scripts/step_geometry.py from\n   %s\n' % STEP)
        f.write('Do not edit by hand -- re-run the script.  Frame: origin on the wheel axle (mid-way\n'
                'between the wheels), x forward, y left, z up, metres.  Geometry only: STEP has no\n'
                'materials and no battery cells, so mass / CoM / inertia are not in here."""\n\n')
        f.write("R_WHEEL = %.5f\nTRACK = %.5f\nWHEEL_WIDTH = %.5f\n" % (r_wheel / 1000, track / 1000, wheel_width / 1000))
        f.write("AXLE_HEIGHT = %.5f   # axle above the ground\n" % ((z_axle - ground_z) / 1000))
        lo, hi = bv.min(0), bv.max(0)
        f.write("BODY_MIN = (%.5f, %.5f, %.5f)\nBODY_MAX = (%.5f, %.5f, %.5f)\n" % (*lo, *hi))
        f.write("# pitch at which the body first touches flat ground, degrees (nose down / tail down)\n")
        f.write("CONTACT_DEG_FWD = %.2f   # first: %s\nCONTACT_DEG_BACK = %.2f   # first: %s\n" % (
            a_f, ENGLISH.get(who.get("fwd"), who.get("fwd")), a_b, ENGLISH.get(who.get("back"), who.get("back"))))
        f.write("# convex hull of everything but the wheels (exact for contact with flat ground)\n")
        f.write("BODY_HULL = (\n")
        for p in hv:
            f.write("    (%.5f, %.5f, %.5f),\n" % tuple(p))
        f.write(")\n# per-part bounding boxes (name, x0, x1, y0, y1, z0, z1) -- for drawing only\n")
        f.write("PART_BOXES = (\n")
        for k, (lo, hi) in boxes.items():
            f.write("    (%r, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f),\n" % (k, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
        f.write(")\n")
    print("wrote", OUT)
    print("r_wheel %.2f mm, track %.1f mm, wheel width %.1f mm, axle %.1f mm above ground" % (
        r_wheel, track, wheel_width, z_axle - ground_z))
    blo, bhi = bv.min(0) * 1e3, bv.max(0) * 1e3
    print("body x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f mm" % (blo[0], bhi[0], blo[1], bhi[1], blo[2], bhi[2]))
    print("contact: nose down %.2f deg (%s), tail down %.2f deg (%s); hull %d vertices" % (
        a_f, ENGLISH.get(who.get("fwd")), a_b, ENGLISH.get(who.get("back")), len(hv)))


if __name__ == "__main__":
    main()
