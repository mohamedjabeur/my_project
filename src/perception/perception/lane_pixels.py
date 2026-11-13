"""Classical (no AI) road-paint detection: RGB image -> ground points per paint class (no ROS, unit-tested).

White paint = bright + unsaturated pixels below the horizon. Connected blobs are projected on the
ground plane; a blob much wider than long is a stop line, the others are grouped by collinearity
into painted lines (edge lines, centre dashes), which lane_geometry then fits.
The car's own body is dark, so it never passes the brightness test.
"""
import cv2
import numpy as np


def paint_mask(rgb, v_min, v_thresh=150, max_chroma=45, exclude=None):
    """Binary mask (h, w) of bright, unsaturated pixels on rows >= v_min, minus the `exclude` polygons (pixels)."""
    mx, mn = rgb.max(axis=2), rgb.min(axis=2)
    m = ((mx >= v_thresh) & ((mx - mn) <= max_chroma)).astype(np.uint8)
    m[:max(int(v_min), 0)] = 0
    for poly in exclude or []:
        cv2.fillPoly(m, [np.asarray(poly, np.int32)], 0)
    k = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)


def is_stop_line(g, stop_min_width=1.5):
    """Wide across the car and short along it: a stop line, not a lane line."""
    dx = float(g[:, 0].max() - g[:, 0].min())
    dy = float(g[:, 1].max() - g[:, 1].min())
    return dy >= stop_min_width and dy >= 2.0 * dx


def is_thin(g, max_std=0.1):
    """Paint is a thin strip along a (slightly curved) line on the ground.

    A raised object (parked box, tree) projected on the ground is a fat blob: its scatter around the
    best-fit line y(x) is much larger than the 0.15 m paint width.
    """
    if len(g) < 3:
        return False
    span = np.ptp(g[:, 0])
    if span < 0.3:
        # all points at (almost) the same distance: a line fit is ill-conditioned (numpy RankWarning), use the median
        resid = g[:, 1] - np.median(g[:, 1])
    else:
        resid = g[:, 1] - np.polyval(np.polyfit(g[:, 0], g[:, 1], 2 if span > 4.0 else 1), g[:, 0])
    return float(resid.std()) <= max_std


def points_to_vanishing_point(us, vs, cx, v_horizon, tol_px=160.0, min_elongation=2.0):
    """Painted lines are parallel on the ground, so in the image their axis meets the horizon near the centre.

    A vertical edge of a parked box / tree does not (it meets the horizon at its own column), which is how
    it is told apart from paint of the same brightness. Blobs too round or too short to have an axis pass.
    """
    if len(us) < 6:
        return True
    ev, vec = np.linalg.eigh(np.cov(np.stack([us, vs]).astype(float)))
    if ev[1] < min_elongation ** 2 * max(ev[0], 1e-6):
        return True
    du, dv = vec[:, 1]
    if abs(dv) < 1e-6:
        return False
    return abs(us.mean() + (v_horizon - vs.mean()) * du / dv - cx) <= tol_px


def group_collinear(blobs, tol=0.25):
    """Merge blobs lying on the same line (dashes of one marking, or pieces of a cut edge line).

    Greedy, nearest-to-the-car first: a blob joins a group when its median distance to the group's
    straight fit is below `tol` m. Returns a list of (n, 2) point arrays, one per painted line.
    """
    groups = []
    for g in sorted(blobs, key=lambda b: abs(np.median(b[:, 1]))):
        for grp in groups:
            pts = np.concatenate(grp)
            # a tiny group has no direction yet: compare lateral position only
            c = np.polyfit(pts[:, 0], pts[:, 1], 1) if np.ptp(pts[:, 0]) >= 0.5 else [0.0, np.median(pts[:, 1])]
            if np.median(np.abs(np.polyval(c, g[:, 0]) - g[:, 1])) < tol:
                grp.append(g)
                break
        else:
            groups.append([g])
    return [np.concatenate(grp) for grp in groups]


def detect_paint(rgb, cam, x_min=1.0, x_max=15.0, stride=2, min_area=8, v_thresh=150, max_chroma=45,
                 exclude=None):
    """Returns (lines, stops, mask): lines = [(n, 2) ground points per painted line], stops = [(n, 2) per stop line]."""
    horizon = cam.cy - cam.fy * np.tan(cam.pitch)
    mask = paint_mask(rgb, horizon + 3, v_thresh, max_chroma, exclude)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs, stops = [], []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_area:
            continue
        vs, us = np.nonzero(labels[::stride, ::stride] == i)
        us, vs = us * stride, vs * stride
        g = cam.pixel_to_ground(us, vs)
        g = g[np.isfinite(g).all(axis=1) & (g[:, 0] >= x_min) & (g[:, 0] <= x_max)]
        if len(g) < 3:
            continue
        if is_stop_line(g):
            stops.append(g)
        elif is_thin(g) and points_to_vanishing_point(us, vs, cam.cx, horizon):
            blobs.append(g)
    return group_collinear(blobs), stops, mask
