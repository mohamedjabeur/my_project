from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="perception",
            executable="camera_fov_marker",
            name="camera_fov_marker",
            output="screen",
            remappings=[("fov_marker", "/camera/fov_marker")],
        ),
    ])
