#!/usr/bin/env python3
"""V4L2 camera driver: publishes /camera/image_raw (rgb8) and /camera/camera_info."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")
        p = self.declare_parameter
        self.device = p("device", "/dev/video0").value
        self.width = p("width", 800).value
        self.height = p("height", 450).value
        self.fps = p("fps", 15.0).value
        self.frame_id = p("frame_id", "camera_optical_link").value
        self.calib = (p("calib_width", 800).value, p("calib_height", 450).value)
        self.fx, self.fy = p("fx", 335.65).value, p("fy", 335.65).value
        self.cx, self.cy = p("cx", 400.0).value, p("cy", 225.0).value
        self.dist_model = p("distortion_model", "plumb_bob").value
        self.dist = list(p("distortion_coefficients", [0.0] * 5).value)

        self.image_pub = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)

        self.cap = None
        self.create_timer(1.0 / self.fps, self.tick)

    def open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap = cap
        self.get_logger().info(
            f"opened {self.device}: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        return True

    def camera_info(self, stamp, w, h):
        sx, sy = w / self.calib[0], h / self.calib[1]
        fx, fy, cx, cy = self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy
        msg = CameraInfo()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
        msg.width, msg.height = w, h
        msg.distortion_model = self.dist_model
        msg.d = self.dist
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def fit(self, frame):
        """Centre-crop to the requested aspect ratio, then resize to width x height."""
        h, w = frame.shape[:2]
        if w * self.height > h * self.width:          # too wide: crop columns
            nw = h * self.width // self.height
            frame = frame[:, (w - nw) // 2:(w - nw) // 2 + nw]
        elif w * self.height < h * self.width:        # too tall: crop rows
            nh = w * self.height // self.width
            frame = frame[(h - nh) // 2:(h - nh) // 2 + nh]
        if frame.shape[1] != self.width:
            frame = cv2.resize(frame, (self.width, self.height))
        return frame

    def tick(self):
        if self.cap is None and not self.open():
            self.get_logger().warn(f"cannot open {self.device}, retrying", throttle_duration_sec=5.0)
            return
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("frame grab failed, reopening", throttle_duration_sec=5.0)
            self.cap.release()
            self.cap = None
            return
        frame = self.fit(frame)
        h, w = frame.shape[:2]
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp, img.header.frame_id = stamp, self.frame_id
        img.height, img.width, img.encoding = h, w, "rgb8"
        img.step = w * 3
        img.data = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).tobytes()
        self.image_pub.publish(img)
        self.info_pub.publish(self.camera_info(stamp, w, h))

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
