"""Pinhole camera <-> ground plane, for the front camera of enimia.urdf (no ROS).

Vehicle frame = base_footprint (x forward, y left, z up, origin on the road below the middle
of the wheelbase). The camera sits at (x_off, 0, height) and looks forward, pitched `pitch`
radians DOWN. Pixel (u, v): u to the right, v down.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class CameraModel:
    fx: float = 335.65
    fy: float = 335.65
    cx: float = 400.0
    cy: float = 225.0
    height: float = 0.732        # m above the road
    pitch: float = 0.1745        # rad, down (10 deg)
    x_off: float = 0.18          # m, camera x in base_footprint

    def rays(self, u, v):
        """Unit-less ray directions in the vehicle frame, shape (n, 3)."""
        u, v = np.atleast_1d(u).astype(float), np.atleast_1d(v).astype(float)
        # optical frame (x right, y down, z forward) -> camera_link (x forward, y left, z up)
        xc, yc, zc = np.ones_like(u), -(u - self.cx) / self.fx, -(v - self.cy) / self.fy
        c, s = np.cos(self.pitch), np.sin(self.pitch)
        return np.stack([c * xc + s * zc, yc, -s * xc + c * zc], axis=1)

    def pixel_to_ground(self, u, v, z=0.0):
        """Intersect the pixel rays with the plane z = `z` -> (n, 2) x, y (nan above the horizon)."""
        d = self.rays(u, v)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z - self.height) / d[:, 2]
        t = np.where(t > 0, t, np.nan)
        return np.stack([self.x_off + t * d[:, 0], t * d[:, 1]], axis=1)

    def ground_to_pixel(self, x, y, z=0.0):
        """Project vehicle-frame points to pixels -> (n, 2) u, v (nan if behind the camera)."""
        x, y, z = (np.atleast_1d(a).astype(float) for a in (x, y, z))
        px, py, pz = x - self.x_off, y, z - self.height
        c, s = np.cos(self.pitch), np.sin(self.pitch)
        xc, zc = c * px - s * pz, s * px + c * pz       # inverse pitch rotation
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.cx - self.fx * py / xc
            v = self.cy - self.fy * zc / xc
        bad = xc <= 0
        return np.stack([np.where(bad, np.nan, u), np.where(bad, np.nan, v)], axis=1)

    def point_from_depth(self, u, v, depth):
        """3D point in the vehicle frame of pixel (u, v) at `depth` m along the optical axis."""
        d = self.rays(u, v)[0]
        c, s = np.cos(self.pitch), np.sin(self.pitch)
        # ray scale so that the optical-axis component (= forward in camera_link) equals depth
        axis = np.array([c, 0.0, -s])
        t = depth / float(d @ axis)
        return np.array([self.x_off, 0.0, self.height]) + t * d
