"""Real car: /drive_cmd -> ESP32 over serial (BLDC speed + steering servo); encoder ticks -> /wheel_feedback.

Protocol: see vehicle/ackermann.py. The ESP32 firmware was removed from the repo (backup ~/ros2_backup_2026-11-08.tar.gz).
Publishes sensor_msgs/JointState on /wheel_feedback:
  rear_wheel_joint  position [rad] = ticks * 2*pi / ticks_per_rev,  velocity [rad/s] (from successive ticks)
  steer             position [rad] = steering angle reported by the ESP32 (bicycle-model angle)
"""
import math
import threading
import time

import rclpy
import serial
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from vehicle.ackermann import format_command, parse_status


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__("esp32_bridge")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.port = p("port", "/dev/ttyUSB0")
        self.baud = p("baud", 115200)
        self.ticks_per_rev = p("ticks_per_rev", 90)     # encoder/hall ticks per WHEEL revolution (hall x pole pairs x gear)
        self.cmd_timeout = p("cmd_timeout", 0.5)        # s without /drive_cmd -> send zero speed
        self.ser = None
        self.cmd = (0.0, 0.0)
        self.last_cmd = time.time()
        self.prev = None                                # (t, ticks)
        self.pub = self.create_publisher(JointState, "/wheel_feedback", 10)
        self.create_subscription(AckermannDriveStamped, "/drive_cmd", self.on_cmd, 10)
        self.create_timer(0.02, self.send)              # 50 Hz command stream = ESP32 watchdog heartbeat
        threading.Thread(target=self.read_loop, daemon=True).start()

    def on_cmd(self, msg):
        self.cmd = (msg.drive.speed, msg.drive.steering_angle)
        self.last_cmd = time.time()

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info("ESP32 connected on %s @ %d" % (self.port, self.baud))
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn("cannot open %s: %s (retrying)" % (self.port, e), throttle_duration_sec=5.0)

    def send(self):
        if self.ser is None:
            return
        speed, steer = self.cmd if time.time() - self.last_cmd < self.cmd_timeout else (0.0, self.cmd[1])
        try:
            self.ser.write(format_command(speed, steer).encode())
        except serial.SerialException:
            self.ser = None

    def read_loop(self):
        while rclpy.ok():
            if self.ser is None:
                self.open()
                time.sleep(1.0)
                continue
            try:
                line = self.ser.readline().decode(errors="ignore")
            except serial.SerialException:
                self.ser = None
                continue
            st = parse_status(line)
            if st:
                self.publish(*st)

    def publish(self, ms, ticks, steer):
        t = time.time()
        angle = ticks * 2.0 * math.pi / self.ticks_per_rev
        vel = 0.0
        if self.prev is not None and t > self.prev[0]:
            vel = (ticks - self.prev[1]) * 2.0 * math.pi / self.ticks_per_rev / (t - self.prev[0])
        self.prev = (t, ticks)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["rear_wheel_joint", "steer"]
        msg.position = [angle, steer]
        msg.velocity = [vel, 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
