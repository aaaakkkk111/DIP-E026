"""Arena: bounding walls, cylindrical obstacles, and a lidar-style ray sensor.

Kept deliberately simple and fully vectorised so that ray casting costs
almost nothing inside the PPO training loop.
"""
from __future__ import annotations

import numpy as np

from .params import ArenaParams


class Arena:
    """Rectangular arena with circular obstacles."""

    def __init__(self, params: ArenaParams | None = None,
                 rng: np.random.Generator | None = None):
        self.p = params or ArenaParams()
        self.rng = rng or np.random.default_rng()
        self.obstacles = np.zeros((0, 3))          # (x, y, radius)
        # ray 0 points straight ahead, then counter-clockwise
        self._ray_angles = np.linspace(0.0, 2.0 * np.pi, self.p.n_rays,
                                       endpoint=False)

    # ------------------------------------------------------------------
    def randomize(self, robot_xy=(0.0, 0.0), clearance: float = 0.8,
                  n: int | None = None):
        """Scatter obstacles, keeping a clear disc around the robot start."""
        p = self.p
        n = p.n_obstacles if n is None else n
        obs = []
        guard = 0
        while len(obs) < n and guard < 400:
            guard += 1
            x = self.rng.uniform(-p.half_x + 0.5, p.half_x - 0.5)
            y = self.rng.uniform(-p.half_y + 0.5, p.half_y - 0.5)
            r = self.rng.uniform(p.obstacle_r_min, p.obstacle_r_max)
            if np.hypot(x - robot_xy[0], y - robot_xy[1]) < clearance + r:
                continue
            if any(np.hypot(x - o[0], y - o[1]) < r + o[2] + 0.25 for o in obs):
                continue
            obs.append((x, y, r))
        self.obstacles = np.array(obs).reshape(-1, 3)
        return self.obstacles

    def set_obstacles(self, obstacles):
        self.obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 3)

    # ------------------------------------------------------------------
    def clearance(self, x: float, y: float) -> float:
        """Distance from (x, y) to the nearest obstacle surface or wall."""
        p = self.p
        d_wall = min(p.half_x - abs(x), p.half_y - abs(y))
        if len(self.obstacles) == 0:
            return float(d_wall)
        d = np.hypot(self.obstacles[:, 0] - x,
                     self.obstacles[:, 1] - y) - self.obstacles[:, 2]
        return float(min(d_wall, d.min()))

    def in_collision(self, x: float, y: float, radius: float) -> bool:
        return self.clearance(x, y) < radius

    # ------------------------------------------------------------------
    def raycast(self, x: float, y: float, heading: float) -> np.ndarray:
        """Distances along ``n_rays`` beams fixed in the *body* frame.

        Returns an array of length ``n_rays`` clipped to ``ray_max``.
        Ray 0 points straight ahead; angles increase counter-clockwise.
        """
        p = self.p
        angles = self._ray_angles + heading
        dirs = np.stack([np.cos(angles), np.sin(angles)], axis=1)   # (R, 2)
        origin = np.array([x, y])
        dist = np.full(p.n_rays, p.ray_max)

        # --- walls (axis aligned box) ------------------------------------
        for axis, half in ((0, p.half_x), (1, p.half_y)):
            d = dirs[:, axis]
            with np.errstate(divide="ignore", invalid="ignore"):
                t_pos = (half - origin[axis]) / d
                t_neg = (-half - origin[axis]) / d
            for t in (t_pos, t_neg):
                ok = np.isfinite(t) & (t > 0)
                dist = np.where(ok, np.minimum(dist, t), dist)

        # --- circular obstacles ------------------------------------------
        if len(self.obstacles):
            oc = self.obstacles[:, :2] - origin                # (N, 2)
            rr = self.obstacles[:, 2]                          # (N,)
            proj = dirs @ oc.T                                 # (R, N)
            oc2 = np.sum(oc ** 2, axis=1)[None, :]             # (1, N)
            disc = proj ** 2 - (oc2 - rr[None, :] ** 2)
            hit = disc > 0
            sq = np.sqrt(np.where(hit, disc, 0.0))
            t = proj - sq
            valid = hit & (t > 0)
            t = np.where(valid, t, np.inf)
            dist = np.minimum(dist, t.min(axis=1))

        return np.clip(dist, 0.0, p.ray_max)

    def normalized_rays(self, x, y, heading) -> np.ndarray:
        """Ray distances mapped to [0, 1] (1 = nothing in range)."""
        return self.raycast(x, y, heading) / self.p.ray_max

    # ------------------------------------------------------------------
    def obstacle_penalty(self, x: float, y: float, radius: float) -> float:
        """Smooth 0..1 penalty that grows as the robot closes on something."""
        c = self.clearance(x, y) - radius
        safe = self.p.safe_dist
        if c >= safe:
            return 0.0
        return float(np.clip((safe - c) / max(safe, 1e-6), 0.0, 1.0)) ** 2

    # ------------------------------------------------------------------
    def to_dict(self):
        return {
            "half_x": self.p.half_x,
            "half_y": self.p.half_y,
            "obstacles": self.obstacles.tolist(),
        }
