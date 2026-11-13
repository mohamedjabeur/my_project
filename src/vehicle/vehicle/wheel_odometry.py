"""/wheel_feedback (encoder + steering) -> /odom and TF odom -> base_footprint (bicycle-model dead reckoning).

Same node in simulation (virtual encoder) and on the real car. Distance comes from the rear wheel angle
(position delta, so no velocity noise), yaw from the steering angle: dyaw = ds * tan(delta) / L.
Add an IMU/GPS fusion (robot_localization) later to cancel the drift.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from vehicle.ackermann import REAR_WHEEL_RADIUS, WHEELBASE


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.r = p("rear_wheel_radius", REAR_WHEEL_RADIUS)
        self.wheelbase = p("wheelbase", WHEELBASE)
        self.odom_frame = p("odom_frame", "odom")
        self.base_frame = p("base_frame", "base_footprint")
        # base_footprint is the middle of the wheelbase, the rear axle is 0.852 m behind it (enimia.urdf):
        # we integrate the rear axle and report the pose of base_footprint
        self.rear_to_base = p("rear_axle_to_base", 0.852)
        self.x = p("x0", 0.0)
        self.y = p("y0", 0.0)
        self.yaw = p("yaw0", 0.0)
        self.rear_x = self.x - self.rear_to_base * math.cos(self.yaw)
        self.rear_y = self.y - self.rear_to_base * math.sin(self.yaw)
        self.last_angle = None
        self.last_stamp = None
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(JointState, "/wheel_feedback", self.on_feedback, 10)

    def on_feedback(self, msg):
        try:
            angle = msg.position[msg.name.index("rear_wheel_joint")]
            delta = msg.position[msg.name.index("steer")]
        except ValueError:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_angle is None:
            self.last_angle, self.last_stamp = angle, stamp
            return
        ds = (angle - self.last_angle) * self.r
        dt = stamp - self.last_stamp
        self.last_angle, self.last_stamp = angle, stamp
        if dt <= 0:
            return
        dyaw = ds * math.tan(delta) / self.wheelbase
        mid = self.yaw + dyaw / 2.0
        self.rear_x += ds * math.cos(mid)
        self.rear_y += ds * math.sin(mid)
        self.yaw += dyaw
        self.x = self.rear_x + self.rear_to_base * math.cos(self.yaw)
        self.y = self.rear_y + self.rear_to_base * math.sin(self.yaw)
        self.publish(msg.header.stamp, ds / dt, dyaw / dt)

    def publish(self, stamp, v, w):
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.odom_frame, self.base_frame
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)
        o = Odometry()
        o.header, o.child_frame_id = t.header, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        self.pub.publish(o)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
