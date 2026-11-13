"""Ground-truth Gazebo odometry (/enimia/odom) -> /odom + TF odom -> base_footprint."""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__("odom_tf")
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Odometry, "/enimia/odom", self.on_odom, 10)

    def on_odom(self, msg):
        msg.header.frame_id, msg.child_frame_id = self.odom_frame, self.base_frame
        self.pub.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
