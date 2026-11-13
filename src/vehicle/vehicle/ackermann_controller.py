"""/cmd_vel (Twist) -> /drive_cmd (AckermannDriveStamped) with acceleration and steering-rate limits.

Same node for simulation and hardware. `cmd_timeout` > 0 stops the car when /cmd_vel goes silent
(use it on the real car; keep 0 with teleop_twist_keyboard, which only publishes on key presses).
"""
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node

from vehicle.ackermann import MAX_STEER, WHEELBASE, rate_limit, steering_from_twist


class AckermannController(Node):
    def __init__(self):
        super().__init__("ackermann_controller")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.wheelbase = p("wheelbase", WHEELBASE)
        self.max_steer = p("max_steer", MAX_STEER)
        self.max_speed = p("max_speed", 5.6)           # m/s (20 km/h)
        self.max_accel = p("max_accel", 1.5)           # m/s^2
        self.max_decel = p("max_decel", 5.0)           # m/s^2
        self.max_steer_rate = p("max_steer_rate", 1.0)  # rad/s
        self.timeout = p("cmd_timeout", 0.0)           # s, 0 = off
        self.rate = 50.0
        self.target_v = self.target_delta = 0.0
        self.v = self.delta = 0.0
        self.last_cmd = self.get_clock().now()
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive_cmd", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / self.rate, self.step)

    def on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        self.target_v = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.target_delta = steering_from_twist(self.target_v, msg.angular.z, self.wheelbase, self.max_steer)

    def step(self):
        if self.timeout > 0 and (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.timeout:
            self.target_v, self.target_delta = 0.0, 0.0
        dt = 1.0 / self.rate
        braking = abs(self.target_v) < abs(self.v)
        self.v = rate_limit(self.target_v, self.v, (self.max_decel if braking else self.max_accel) * dt)
        self.delta = rate_limit(self.target_delta, self.delta, self.max_steer_rate * dt)
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.drive.speed = float(self.v)
        msg.drive.steering_angle = float(self.delta)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AckermannController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
