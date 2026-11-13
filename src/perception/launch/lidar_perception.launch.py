"""Lidar obstacle detector. Real car with the Livox Mid-360, add the overrides after the base file:
  ros2 launch perception lidar_perception.launch.py override:=lidar_obstacle_detector_mid360.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = lambda name: PathJoinSubstitution([FindPackageShare("perception"), "config", name])
    return LaunchDescription([
        # the override file is loaded second, so its values win (default: the base file again = no change)
        DeclareLaunchArgument("override", default_value="lidar_obstacle_detector.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="false on the real car (no /clock)"),
        Node(
            package="perception",
            executable="lidar_obstacle_detector",
            name="lidar_obstacle_detector",
            output="screen",
            parameters=[share("lidar_obstacle_detector.yaml"), share(LaunchConfiguration("override")),
                        {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
        ),
    ])
