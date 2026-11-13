"""Click a point on the road in RViz ("2D Goal Pose"): the car drives the lane to it, avoids obstacles, stops there,
Stands still until the first click, then drives the lane (never stops on the way, not even at stop lines) until the
clicked point is reached, then holds there.

    ros2 launch autonomy drive.launch.py [v_cruise:=5.56]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("autonomy"), "launch", "lap.launch.py"])),
        launch_arguments={"require_goal": "true", "stop_at_lines": "false", "open_rviz": "true",
                          "v_cruise": LaunchConfiguration("v_cruise")}.items())
    return LaunchDescription([DeclareLaunchArgument("v_cruise", default_value="5.56"), lap])
