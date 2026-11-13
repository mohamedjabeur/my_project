"""RViz '2D Goal Pose' (/nav_goal) -> NavigateToPose, adapted to an Ackermann car.

The car cannot turn on the spot (min turning radius ~8 m), so a click is turned into a goal it can reach:
  - the heading is the direction car -> click (the drag direction is ignored);
  - a goal closer than `min_distance` is pushed out along the same bearing, with a warning;
  - a goal behind the car (more than `max_angle_deg` off the car heading) is rejected.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalRelay(Node):
    def __init__(self):
        super().__init__("goal_relay")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.frame = p("goal_frame", "odom")
        self.base = p("base_frame", "base_footprint")
        self.min_distance = p("min_distance", 15.0)
        self.max_angle = math.radians(p("max_angle_deg", 100.0))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(PoseStamped, "/nav_goal", self.on_goal, 10)
        # the goal really sent to Nav2 (a click can be moved out to min_distance or rejected): shown in RViz
        self.sent_pub = self.create_publisher(PoseStamped, "/nav_goal_sent", 1)
        self.get_logger().info("click a goal with 2D Goal Pose in RViz (topic /nav_goal)")

    def on_goal(self, msg):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.base, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error("no TF %s -> %s: %s" % (self.frame, self.base, e))
            return
        cx, cy = t.transform.translation.x, t.transform.translation.y
        cyaw = yaw_of(t.transform.rotation)
        dx, dy = msg.pose.position.x - cx, msg.pose.position.y - cy
        dist, bearing = math.hypot(dx, dy), math.atan2(dy, dx)
        off = math.atan2(math.sin(bearing - cyaw), math.cos(bearing - cyaw))
        if abs(off) > self.max_angle:
            self.get_logger().warn("goal is behind the car (%.0f deg off): choose a point in front" % math.degrees(off))
            return
        if dist < self.min_distance:
            self.get_logger().warn("goal %.1f m away is too close for the turning radius: moved to %.0f m"
                                   % (dist, self.min_distance))
            dist = self.min_distance
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = cx + dist * math.cos(bearing)
        goal.pose.pose.position.y = cy + dist * math.sin(bearing)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(bearing / 2), math.cos(bearing / 2)
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_to_pose not available")
            return
        self.client.send_goal_async(goal)
        self.sent_pub.publish(goal.pose)
        self.get_logger().info("goal sent: (%.1f, %.1f)" % (goal.pose.pose.position.x, goal.pose.pose.position.y))


def main():
    rclpy.init()
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
