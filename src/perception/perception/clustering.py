"""Pure functions for lidar obstacle detection (no ROS): segmentation and lane logic."""
from dataclasses import dataclass

import numpy as np


@dataclass
class Cluster:
    points: np.ndarray          # (n, 3) x, y, z in the vehicle frame
    center: tuple               # (x, y) mean of the points
    size: tuple                 # (dx, dy) extent of the points
    closest_x: float            # smallest x of the cluster
    height: float               # highest z hit
    lane: str = "off_road"


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan arrays -> (indices, xyz in the lidar frame) of the valid returns."""
    ranges = np.asarray(ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    idx = np.nonzero(valid)[0]
    ang = angle_min + idx * angle_increment
    r = ranges[idx]
    xyz = np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)], axis=1)
    return idx, xyz


def transform_points(xyz, rotation, translation):
    """Apply a rigid transform (3x3 rotation, 3-vector translation)."""
    return xyz @ np.asarray(rotation).T + np.asarray(translation)


def filter_points(idx, xyz, z_min, z_max, x_range, y_max, self_box):
    """Keep obstacle returns: above the ground, inside the ROI, outside the vehicle body.

    self_box = (x_min, x_max, y_half_width) of the area occupied by the vehicle.
    """
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    keep = (z >= z_min) & (z <= z_max)
    keep &= (x >= x_range[0]) & (x <= x_range[1]) & (np.abs(y) <= y_max)
    keep &= ~((x >= self_box[0]) & (x <= self_box[1]) & (np.abs(y) < self_box[2]))
    return idx[keep], xyz[keep]


def segment(idx, xyz, angle_increment, eps_base, eps_scale, min_points, single_point_range=np.inf, max_idx_gap=1):
    """Breakpoint segmentation of an angularly ordered scan.

    A new cluster starts when two consecutive points are not neighbouring beams
    or are farther apart than eps_base + eps_scale * range * angle_increment.
    max_idx_gap: largest beam-index jump still counted as neighbours (1 for a dense RPLIDAR scan, ~3 for the
    sparse scan made from the Mid-360 point cloud, whose empty beams would otherwise cut an obstacle in pieces).
    A cluster needs min_points points, except beyond single_point_range (m) where one
    point is enough: a thin cone far away may be hit by a single beam.
    """
    if len(idx) == 0:
        return []
    clusters, start = [], 0
    for i in range(1, len(idx) + 1):
        if i == len(idx):
            brk = True
        else:
            gap = np.hypot(*(xyz[i, :2] - xyz[i - 1, :2]))
            r = np.hypot(*xyz[i - 1, :2])
            brk = (idx[i] - idx[i - 1] > max_idx_gap) or gap > eps_base + eps_scale * r * angle_increment
        if brk:
            pts = xyz[start:i]
            far = np.hypot(pts[:, 0], pts[:, 1]).min() >= single_point_range
            if len(pts) >= min_points or (far and len(pts) >= 1):
                clusters.append(_make_cluster(pts))
            start = i
    return clusters


def _make_cluster(pts):
    ext = pts.max(axis=0) - pts.min(axis=0)
    return Cluster(points=pts, center=(float(pts[:, 0].mean()), float(pts[:, 1].mean())),
                   size=(float(ext[0]), float(ext[1])), closest_x=float(pts[:, 0].min()),
                   height=float(pts[:, 2].max()))


def lane_centers(lane_width, ego_lane_offset, adjacent_side):
    """Lateral centres (y, left positive) of the ego lane and of the adjacent lane.

    ego_lane_offset: how far the vehicle is to the left of the centre of its lane.
    adjacent_side:  'left' or 'right' (side of the lane we would change to).
    """
    ego = -ego_lane_offset
    side = 1.0 if adjacent_side == "left" else -1.0
    return ego, ego + side * lane_width


def label_lane(y, ego_center, adj_center, lane_width):
    half = lane_width / 2.0
    if abs(y - ego_center) <= half:
        return "ego_lane"
    if abs(y - adj_center) <= half:
        return "adjacent_lane"
    return "off_road"


def clear_distance(clusters, center_y, half_width, front_x):
    """Distance from the front bumper to the first obstacle in a corridor (inf if free)."""
    best = np.inf
    for c in clusters:
        p = c.points
        m = (np.abs(p[:, 1] - center_y) < half_width) & (p[:, 0] > 0.0)
        if m.any():
            best = min(best, max(float(p[m, 0].min()) - front_x, 0.0))
    return best
