"""Lane estimation from lane-line points on the ground plane (no ROS).

All points are in the vehicle frame (x forward, y left). A lane line is fitted as y = f(x).
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class LaneLine:
    cls: str                  # 'lane_line'
    coeffs: np.ndarray        # polynomial coefficients of y(x), highest degree first
    x_min: float
    x_max: float
    n: int

    def y(self, x):
        return float(np.polyval(self.coeffs, x))

    def y_extrap(self, x):
        """y(x); outside the fitted range the line is extended linearly (safer than the polynomial)."""
        if x < self.x_min:
            return self.y(self.x_min) + self.slope(self.x_min) * (x - self.x_min)
        if x > self.x_max:
            return self.y(self.x_max) + self.slope(self.x_max) * (x - self.x_max)
        return self.y(x)

    def slope(self, x):
        return float(np.polyval(np.polyder(self.coeffs), x))


def fit_line(cls, pts, min_points=15, min_span=1.5):
    """Fit y(x) to (n, 2) ground points; None if too few points or too short."""
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) < min_points:
        return None
    x, y = pts[:, 0], pts[:, 1]
    span = x.max() - x.min()
    if span < min_span:
        return None
    deg = 2 if span > 4.0 else 1
    # one robust pass: drop points far from the first fit
    c = np.polyfit(x, y, deg)
    keep = np.abs(np.polyval(c, x) - y) < 0.25
    if keep.sum() >= min_points:
        c = np.polyfit(x[keep], y[keep], deg)
    return LaneLine(cls, c, float(x.min()), float(x.max()), int(len(pts)))


def estimate_lane(lines, lane_width=3.5, x_eval=0.0, width_tol=0.7, road_width=None):
    """Ego lane from the two boundaries closest to the vehicle.

    Lines are evaluated at x_eval (0 = the vehicle position, extrapolated linearly from the nearest
    visible point), so a heading error does not bias the lateral offset.

    road_width: set it when the road has only its two EDGE lines (no centre line). The ego lane is then
    always the RIGHT lane (keep right): its centre is lane_width/2 from the right edge, whichever edge(s) are seen.

    Returns dict(valid, offset, heading, width, n_boundaries):
      offset  = how far the vehicle is LEFT of the centre of its lane (m)  [lidar detector ego_lane_offset]
      heading = lane direction minus vehicle heading (rad, + = lane turns left)
    """
    bad = {"valid": False, "offset": float("nan"), "heading": float("nan"),
           "width": float("nan"), "n_boundaries": 0}
    cands = [(ln.y_extrap(x_eval), ln) for ln in lines]
    left = min(((y, ln) for y, ln in cands if y > 0), key=lambda t: t[0], default=None)
    right = max(((y, ln) for y, ln in cands if y <= 0), key=lambda t: t[0], default=None)
    xs = lambda ln: min(max(x_eval, ln.x_min), ln.x_max)      # slope taken inside the fitted range

    if road_width is not None:
        return _keep_right(left, right, xs, lane_width, road_width, width_tol, bad)

    if left and right:
        width = left[0] - right[0]
        if abs(width - lane_width) <= width_tol:
            center = 0.5 * (left[0] + right[0])
            slope = 0.5 * (left[1].slope(xs(left[1])) + right[1].slope(xs(right[1])))
            return {"valid": True, "offset": -center, "heading": float(np.arctan(slope)),
                    "width": width, "n_boundaries": 2}
    # one boundary only (or inconsistent pair): use the nearest one and the nominal lane width
    near = min([t for t in (left, right) if t], key=lambda t: abs(t[0]), default=None)
    if near is None or abs(near[0]) > 1.3 * lane_width:
        return bad
    y0, ln = near
    center = y0 - lane_width / 2 if y0 > 0 else y0 + lane_width / 2
    return {"valid": True, "offset": -center, "heading": float(np.arctan(ln.slope(xs(ln)))),
            "width": float("nan"), "n_boundaries": 1}


def stop_line_distance(pts, front_x, max_lateral=3.6):
    """Distance from the front bumper to the nearest stop-line point (inf if none)."""
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[np.abs(pts[:, 1]) <= max_lateral]
    if len(pts) == 0:
        return float("inf")
    return float(max(pts[:, 0].min() - front_x, 0.0))


def _keep_right(left, right, xs, lane_width, road_width, width_tol, bad):
    """Ego lane = right lane of a road delimited by two edge lines `road_width` apart."""
    if left and right and abs(left[0] - right[0] - road_width) <= width_tol:
        slope = 0.5 * (left[1].slope(xs(left[1])) + right[1].slope(xs(right[1])))
        return {"valid": True, "offset": -(right[0] + lane_width / 2), "heading": float(np.arctan(slope)),
                "width": left[0] - right[0], "n_boundaries": 2}
    # one edge only: it is the right one if it is on the right of the car or too close to be the left one
    near = right if right else left
    if near is None:
        return bad
    y0, ln = near
    is_right_edge = bool(right) and (not left or abs(right[0]) < abs(left[0]) or abs(left[0] - right[0] - road_width) > width_tol)
    y_right = y0 if is_right_edge else y0 - road_width
    if abs(y_right + lane_width / 2) > 1.3 * lane_width:
        return bad
    return {"valid": True, "offset": -(y_right + lane_width / 2), "heading": float(np.arctan(ln.slope(xs(ln)))),
            "width": float("nan"), "n_boundaries": 1}
