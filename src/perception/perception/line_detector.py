#!/usr/bin/env python3
"""Lane-line detector (classical vision, no AI): white-paint mask -> ground plane -> lane offset / heading / stop line."""
import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from perception import lane_geometry as lg
from perception.ground_projection import CameraModel
from perception.lane_pixels import detect_paint

MARKER_RGB = {"lane_line": (0.2, 1.0, 0.2), "stop_line": (0.2, 0.6, 1.0)}


class LineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")
        d = self.declare_parameter
        self.cam = CameraModel(d("fx", 335.65).value, d("fy", 335.65).value, d("cx", 400.0).value,
                               d("cy", 225.0).value, d("camera_height", 0.732).value,
                               d("camera_pitch", 0.1745).value, d("camera_x", 0.18).value)
        self.x_min, self.x_max = d("x_min", 1.0).value, d("x_max", 15.0).value
        self.lane_width = d("lane_width", 3.5).value
        self.road_width = d("road_width", 0.0).value or None      # > 0: edge lines only, keep right
        self.front_x = d("front_x", 1.445).value
        self.debug = d("publish_debug_image", True).value
        self.pixel_stride = d("pixel_stride", 2).value
        self.v_thresh, self.max_chroma = d("v_thresh", 150).value, d("max_chroma", 45).value
        self.min_area = d("min_area", 8).value
        # the car's own bonnet rim is bright in the image and touches the lane lines: mask it out
        # an empty list in a yaml file (OAK-D: bonnet not visible) arrives as None: no mask
        flat = d("car_mask", [0]).value or []
        self.exclude = [np.array(flat, np.int32).reshape(-1, 2)] if len(flat) >= 6 else []

        self.offset_pub = self.create_publisher(Float32, "/perception/camera/ego_lane_offset", 10)
        self.heading_pub = self.create_publisher(Float32, "/perception/camera/heading_error", 10)
        self.valid_pub = self.create_publisher(Bool, "/perception/camera/lane_valid", 10)
        self.stop_pub = self.create_publisher(Float32, "/perception/camera/stop_line_distance", 10)
        self.lines_pub = self.create_publisher(MarkerArray, "/perception/camera/lane_lines", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/camera/lines_debug_image", 1)
        # own callback group: the /clock subscription of use_sim_time lives in the default group, and a busy default
        # group (800 msg/s) blocked the image callback for good (node at 100 % CPU, no output, seen 3 times in sim)
        self.create_subscription(Image, d("image_topic", "/camera/image_raw").value, self.on_image,
                                 qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())

    def on_image(self, msg):
        rgb = self.to_rgb(msg)
        if rgb is None:
            return
        paint, stops, mask = detect_paint(rgb, self.cam, self.x_min, self.x_max, self.pixel_stride,
                                          self.min_area, self.v_thresh, self.max_chroma, self.exclude)
        lines = [ln for ln in (lg.fit_line("lane_line", p) for p in paint) if ln]
        est = lg.estimate_lane(lines, self.lane_width, road_width=self.road_width)
        stop = lg.stop_line_distance(np.concatenate(stops), self.front_x) if stops else float("inf")

        self.offset_pub.publish(Float32(data=float(est["offset"])))
        self.heading_pub.publish(Float32(data=float(est["heading"])))
        self.valid_pub.publish(Bool(data=bool(est["valid"])))
        self.stop_pub.publish(Float32(data=float(stop)))
        self.publish_markers(msg.header, lines)
        if self.debug and self.debug_pub.get_subscription_count() > 0:   # nobody watching: skip the drawing
            self.publish_debug(msg.header, rgb, mask, est, stop)

    def to_rgb(self, msg):
        """Image msg -> HxWx3 RGB array. Sim/V4L2 send rgb8, the OAK-D (depthai) sends bgr8; rows may be padded."""
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"unsupported image encoding {msg.encoding} (need rgb8 or bgr8)",
                                   throttle_duration_sec=5.0)
            return None
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(img[:, :, ::-1]) if msg.encoding == "bgr8" else img

    def publish_markers(self, header, lines):
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        for i, ln in enumerate(lines):
            m = Marker()
            m.header.stamp, m.header.frame_id = header.stamp, "base_footprint"
            m.ns, m.id, m.type, m.action = "lane_lines", i, Marker.LINE_STRIP, Marker.ADD
            m.scale.x = 0.08
            m.color.r, m.color.g, m.color.b = MARKER_RGB[ln.cls]
            m.color.a = 1.0
            for x in np.linspace(ln.x_min, ln.x_max, 25):
                m.points.append(Point(x=float(x), y=ln.y(x), z=0.03))
            arr.markers.append(m)
        self.lines_pub.publish(arr)

    def publish_debug(self, header, rgb, mask, est, stop):
        img = rgb.copy()
        img[mask > 0] = (255, 60, 60)
        txt = (f"offset {est['offset']:+.2f} m  heading {np.degrees(est['heading']):+.1f} deg"
               if est["valid"] else "lane: no estimate")
        cv2.putText(img, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if np.isfinite(stop):
            cv2.putText(img, f"stop line {stop:.1f} m", (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        out = Image()
        out.header, out.height, out.width, out.encoding = header, img.shape[0], img.shape[1], "rgb8"
        # numpy array, not bytes: the message setter checks bytes one by one in Python (seconds per 960x540 frame!)
        out.step, out.data = img.shape[1] * 3, np.ascontiguousarray(img).reshape(-1)
        self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = LineDetector()
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
