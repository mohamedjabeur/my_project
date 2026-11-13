"""Send a navigation goal in the odom frame:  ros2 run navigation send_goal X Y [YAW_DEG]"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    x, y = float(sys.argv[1]), float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = Node("send_goal", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("navigate_to_pose not available: is the navigation running?")
        return

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "odom"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

    def feedback(msg):
        fb = msg.feedback
        node.get_logger().info("distance remaining: %.1f m" % fb.distance_remaining, throttle_duration_sec=2.0)

    handle = client.send_goal_async(goal, feedback_callback=feedback)
    rclpy.spin_until_future_complete(node, handle)
    handle = handle.result()
    if not handle.accepted:
        node.get_logger().error("goal rejected (no path? outside the 80 m costmap window?)")
        return
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    status = result.result().status
    node.get_logger().info({4: "GOAL REACHED", 5: "canceled", 6: "FAILED"}.get(status, "status %d" % status))
    node.destroy_node()
    rclpy.shutdown()
