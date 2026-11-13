#!/usr/bin/env python3
"""Simulation stand-in for pointcloud_to_laserscan (not installed here): PointCloud2 -> LaserScan in a LEVEL frame.

Same parameters as config/pointcloud_to_scan.yaml (the file the real Mid-360 chain uses): points are moved to
`target_frame` (lidar_level_link), only those between min_height and max_height are kept (the ground is cut), and the
nearest one per angle bin gives the range. The real car keeps using pointcloud_to_laserscan.
"""
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cloud_to_ranges(xyz, min_h, max_h, a_min, a_inc, n_bins, r_min, r_max):
    """xyz (N,3) already in the level frame -> nearest range per angle bin (inf where empty)."""
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    keep = (xyz[:, 2] >= min_h) & (xyz[:, 2] <= max_h) & (r >= r_min) & (r <= r_max)
    idx = np.floor((np.arctan2(xyz[keep, 1], xyz[keep, 0]) - a_min) / a_inc).astype(int)
    ok = (idx >= 0) & (idx < n_bins)
    out = np.full(n_bins, np.inf)
    np.minimum.at(out, idx[ok], r[keep][ok])
    return out


class CloudToScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_scan")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.target = p("target_frame", "lidar_level_link")
        self.min_h, self.max_h = p("min_height", -0.6), p("max_height", 0.3)
        self.a_min, self.a_max, self.a_inc = p("angle_min", -3.14159), p("angle_max", 3.14159), p("angle_increment", 0.0175)
        self.r_min, self.r_max = p("range_min", 0.2), p("range_max", 40.0)
        self.scan_time = p("scan_time", 0.1)
        self.n = int(round((self.a_max - self.a_min) / self.a_inc))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "cloud_in", self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        try:
            t = self.tf.lookup_transform(self.target, msg.header.frame_id, Time(), Duration(seconds=0.2)).transform
        except Exception as e:  # TF not ready yet
            self.get_logger().warn(f"no TF {msg.header.frame_id} -> {self.target}: {e}", throttle_duration_sec=5.0)
            return
        pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
        xyz = xyz @ quat_to_matrix(t.rotation).T + np.array([t.translation.x, t.translation.y, t.translation.z])
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = msg.header.stamp, self.target
        scan.angle_min, scan.angle_max, scan.angle_increment = self.a_min, self.a_max, self.a_inc
        scan.scan_time, scan.range_min, scan.range_max = self.scan_time, self.r_min, self.r_max
        scan.ranges = cloud_to_ranges(xyz, self.min_h, self.max_h, self.a_min, self.a_inc, self.n,
                                      self.r_min, self.r_max).astype(np.float32).tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    rclpy.spin(CloudToScan())


if __name__ == "__main__":
    main()
