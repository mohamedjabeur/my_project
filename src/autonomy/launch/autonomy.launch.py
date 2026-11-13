"""Kalman fusion + lane-keeping command. Run with the sim/car, teleop.launch.py and the perception launch files.

direct:=true  (default) publishes straight to /cmd_vel -> do NOT run Nav2 / keyboard teleop at the same time.
direct:=false leaves the command on /cmd_vel_lane (to be muxed with something else).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("autonomy"), "config", "autonomy.yaml"])
    use_sim_time = LaunchConfiguration("use_sim_time")

    def node(**kw):
        return Node(package="autonomy", executable="fusion_node", output="screen",
                    parameters=[params, {"use_sim_time": use_sim_time}], **kw)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("direct", default_value="true"),
        node(condition=IfCondition(LaunchConfiguration("direct")), remappings=[("/cmd_vel_lane", "/cmd_vel")]),
        node(condition=UnlessCondition(LaunchConfiguration("direct"))),
    ])
