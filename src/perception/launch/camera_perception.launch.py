"""Lane / line detector. Real car with the OAK-D Lite:
  ros2 launch perception camera_perception.launch.py config:=line_detector_oak.yaml use_sim_time:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare("perception"), "config", LaunchConfiguration("config")])
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="line_detector.yaml", description="line_detector.yaml (sim) | line_detector_oak.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="false on the real car (no /clock)"),
        Node(package="perception", executable="line_detector", name="line_detector", output="screen",
             parameters=[cfg, {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
