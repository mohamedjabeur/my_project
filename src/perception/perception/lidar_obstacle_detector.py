#!/usr/bin/env python3
"""Lidar obstacle detector: /scan -> obstacles with lane label + free distance per lane."""
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from perception import clustering as cl

LANE_COLORS = {"ego_lane": (1.0, 0.1, 0.1), "adjacent_lane": (1.0, 0.8, 0.0), "off_road": (0.5, 0.5, 0.5)}


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_detector")
        d = self.declare_parameter
        self.scan_topic = d("scan_topic", "/scan").value
        self.base_frame = d("base_frame", "base_footprint").value
        self.z_min, self.z_max = d("z_min", 0.12).value, d("z_max", 2.0).value
        self.x_range = (d("x_min", -1.0).value, d("x_max", 15.0).value)
        self.y_max = d("y_max", 8.0).value
        self.self_box = (d("self_x_min", -1.3).value, d("self_x_max", 1.5).value,
                         d("self_half_width", 0.5).value)
        self.eps_base, self.eps_scale = d("eps_base", 0.25).value, d("eps_scale", 2.0).value
        self.min_points = d("min_points", 2).value
        self.single_point_range = d("single_point_range", 5.0).value
        self.max_beam_gap = d("max_beam_gap", 1).value
        self.lane_width = d("lane_width", 3.5).value
        self.adjacent_side = d("adjacent_side", "left").value
        self.ego_offset = d("ego_lane_offset", 0.0).value
        self.corridor_half = d("vehicle_half_width", 0.385).value + d("corridor_margin", 0.30).value
        self.front_x = d("front_x", 1.445).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.det_pub = self.create_publisher(Detection3DArray, "/perception/lidar/obstacles", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/lidar/markers", 10)
        self.ego_pub = self.create_publisher(Float32, "/perception/lidar/ego_clear_distance", 10)
        self.adj_pub = self.create_publisher(Float32, "/perception/lidar/adjacent_clear_distance", 10)
        # own callback group so the /clock flood of use_sim_time cannot starve the scan callback (see line_detector.py)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data,
                                 callback_group=MutuallyExclusiveCallbackGroup())

    def on_scan(self, scan):
        try:
            # latest transform: lidar_link -> base_frame is a fixed joint (robot_state_publisher)
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, scan.header.frame_id, Time(), Duration(seconds=0.2))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no TF {scan.header.frame_id} -> {self.base_frame}: {e}",
                                   throttle_duration_sec=5.0)
            return
        rot = quat_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        trans = np.array([t.x, t.y, t.z])

        idx, xyz = cl.scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                     scan.range_min, scan.range_max)
        xyz = cl.transform_points(xyz, rot, trans)
        idx, xyz = cl.filter_points(idx, xyz, self.z_min, self.z_max, self.x_range,
                                    self.y_max, self.self_box)
        clusters = cl.segment(idx, xyz, scan.angle_increment, self.eps_base,
                              self.eps_scale, self.min_points, self.single_point_range, self.max_beam_gap)

        ego_c, adj_c = cl.lane_centers(self.lane_width, self.ego_offset, self.adjacent_side)
        for c in clusters:
            c.lane = cl.label_lane(c.center[1], ego_c, adj_c, self.lane_width)

        self.publish(scan.header.stamp, clusters,
                     cl.clear_distance(clusters, 0.0, self.corridor_half, self.front_x),
                     cl.clear_distance(clusters, adj_c, self.corridor_half, self.front_x))

    def publish(self, stamp, clusters, ego_clear, adj_clear):
        dets = Detection3DArray()
        dets.header.stamp, dets.header.frame_id = stamp, self.base_frame
        markers = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        for i, c in enumerate(clusters):
            det = Detection3D()
            det.header = dets.header
            det.id = str(i)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = c.lane, 1.0
            hyp.pose.pose.position.x, hyp.pose.pose.position.y = c.center
            det.results.append(hyp)
            det.bbox.center.position.x, det.bbox.center.position.y = c.center
            det.bbox.center.position.z = c.height / 2.0
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = max(c.size[0], 0.2), max(c.size[1], 0.2), c.height
            dets.detections.append(det)

            m = Marker()
            m.header = dets.header
            m.ns, m.id, m.type, m.action = "obstacles", i, Marker.CYLINDER, Marker.ADD
            m.pose.position = Point(x=c.center[0], y=c.center[1], z=c.height / 2.0)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = max(c.size[0], 0.3), max(c.size[1], 0.3), c.height
            m.color.r, m.color.g, m.color.b = LANE_COLORS[c.lane]
            m.color.a = 0.8
            m.lifetime = Duration(seconds=0.3).to_msg()
            markers.markers.append(m)
        self.det_pub.publish(dets)
        self.marker_pub.publish(markers)
        self.ego_pub.publish(Float32(data=float(ego_clear)))
        self.adj_pub.publish(Float32(data=float(adj_clear)))


def main():
    rclpy.init()
    node = LidarObstacleDetector()
    # multi-threaded on purpose: with use_sim_time the /clock topic (400-900 Hz in Gazebo) floods a single-threaded
    # executor and starves the camera/lidar callbacks (node alive, 70 % CPU, nothing published)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
