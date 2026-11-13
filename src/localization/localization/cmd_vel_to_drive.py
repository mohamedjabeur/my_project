"""Twist (/cmd_vel) -> Gazebo drive commands of enimia.urdf (rear wheel velocity + Ackermann steering)."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


def ackermann(v, w, wheelbase, track, max_steer):
    """(left, right) front steering angles [rad] for forward speed v and yaw rate w (bicycle model)."""
    if abs(v) < 1e-3 or abs(w) < 1e-6:
        return 0.0, 0.0
    delta = max(-max_steer, min(max_steer, math.atan(wheelbase * w / v)))
    if abs(delta) < 1e-6:
        return 0.0, 0.0
    radius = wheelbase / math.tan(delta)
    left = math.atan(wheelbase / (radius - track / 2.0))
    right = math.atan(wheelbase / (radius + track / 2.0))
    return left, right


class CmdVelToDrive(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_drive")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.wheelbase = p("wheelbase", 1.704)
        self.track = p("track", 0.567)
        self.wheel_radius = p("rear_wheel_radius", 0.18)
        self.max_steer = p("max_steer", 0.21)
        self.max_speed = p("max_speed", 5.6)
        self.timeout = p("cmd_timeout", 0.0)   # s; 0 = off (teleop_twist_keyboard only publishes on key presses)
        self.rear = self.create_publisher(Float64, "/enimia/rear_wheel_cmd", 10)
        self.left = self.create_publisher(Float64, "/enimia/steer_left_cmd", 10)
        self.right = self.create_publisher(Float64, "/enimia/steer_right_cmd", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.last = self.get_clock().now()
        self.create_timer(0.1, self.watchdog)

    def send(self, v, left, right):
        self.rear.publish(Float64(data=v / self.wheel_radius))
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))

    def on_cmd(self, msg):
        self.last = self.get_clock().now()
        v = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        left, right = ackermann(v, msg.angular.z, self.wheelbase, self.track, self.max_steer)
        self.send(v, left, right)

    def watchdog(self):
        if self.timeout > 0 and (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout:
            self.send(0.0, 0.0, 0.0)


def main():
    rclpy.init()
    node = CmdVelToDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
