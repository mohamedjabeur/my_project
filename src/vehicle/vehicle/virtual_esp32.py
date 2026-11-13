"""Simulation twin of the ESP32 + BLDC: same /drive_cmd in, same /wheel_feedback out.

  /drive_cmd -> BLDC speed loop model (first-order lag + torque-limited acceleration) -> Gazebo wheel commands
  Gazebo ground-truth speed (/enimia/odom) -> VIRTUAL ENCODER: quantized, noisy ticks -> /wheel_feedback
So the stack above this node (Nav2, odometry, ...) cannot tell simulation from the real car.
"""
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from vehicle.ackermann import REAR_WHEEL_RADIUS, rate_limit, wheel_steering


class VirtualEsp32(Node):
    def __init__(self):
        super().__init__("virtual_esp32")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.r = p("rear_wheel_radius", REAR_WHEEL_RADIUS)
        self.ticks_per_rev = p("ticks_per_rev", 90)
        self.motor_tau = p("motor_tau", 0.25)            # s, speed-loop time constant of the BLDC
        self.motor_max_accel = p("motor_max_accel", 2.0)  # m/s^2
        self.servo_tau = p("servo_tau", 0.08)            # s, steering servo lag
        self.tick_noise = p("tick_noise", 0.0)           # ticks, gaussian jitter on the counter
        self.cmd_timeout = p("cmd_timeout", 0.5)         # s: like the firmware, stop when /drive_cmd goes silent
        self.rate = 50.0
        self.cmd_v = self.cmd_delta = 0.0
        self.motor_v = 0.0
        self.servo = 0.0
        self.true_v = 0.0
        self.ticks_f = 0.0
        self.last_cmd = None
        self.rng = np.random.default_rng()
        self.rear = self.create_publisher(Float64, "/enimia/rear_wheel_cmd", 10)
        self.left = self.create_publisher(Float64, "/enimia/steer_left_cmd", 10)
        self.right = self.create_publisher(Float64, "/enimia/steer_right_cmd", 10)
        self.fb = self.create_publisher(JointState, "/wheel_feedback", 10)
        self.create_subscription(AckermannDriveStamped, "/drive_cmd", self.on_cmd, 10)
        self.create_subscription(Odometry, "/enimia/odom", self.on_truth, 10)
        self.create_timer(1.0 / self.rate, self.step)

    def on_cmd(self, msg):
        self.cmd_v, self.cmd_delta = msg.drive.speed, msg.drive.steering_angle
        self.last_cmd = self.get_clock().now()

    def on_truth(self, msg):
        t = msg.twist.twist.linear                        # frame-independent magnitude, forward-only car
        self.true_v = math.hypot(t.x, t.y) * (1.0 if self.motor_v >= -1e-3 else -1.0)

    def step(self):
        dt = 1.0 / self.rate
        target_v = self.cmd_v
        if self.last_cmd is None or (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            target_v = 0.0                                  # firmware watchdog
        # BLDC speed loop: first-order response, torque-limited
        wanted = self.motor_v + (target_v - self.motor_v) * (1.0 - math.exp(-dt / self.motor_tau))
        self.motor_v = rate_limit(wanted, self.motor_v, self.motor_max_accel * dt)
        self.servo += (self.cmd_delta - self.servo) * (1.0 - math.exp(-dt / self.servo_tau))

        self.rear.publish(Float64(data=self.motor_v / self.r))
        left, right = wheel_steering(self.servo)
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))

        # virtual encoder: integrates the REAL wheel speed (ground truth), then quantizes
        self.ticks_f += self.true_v / self.r * dt * self.ticks_per_rev / (2.0 * math.pi)
        ticks = int(round(self.ticks_f + (self.rng.normal(0.0, self.tick_noise) if self.tick_noise else 0.0)))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["rear_wheel_joint", "steer"]
        msg.position = [ticks * 2.0 * math.pi / self.ticks_per_rev, self.servo]
        msg.velocity = [self.true_v / self.r, 0.0]
        self.fb.publish(msg)


def main():
    rclpy.init()
    node = VirtualEsp32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
