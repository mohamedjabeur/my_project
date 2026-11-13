from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("vehicle_description")
    urdf_file = PathJoinSubstitution([pkg, "urdf", "enimia.urdf"])
    rviz_file = PathJoinSubstitution([pkg, "rviz", "enimia.rviz"])

    robot_description = ParameterValue(
        Command(["cat ", urdf_file]),
        value_type=str,
    )

    # sim callers pass use_sim_time:=true; the real car has no /clock. A robot_state_publisher on the wall clock next to
    # sim-time nodes gives "Detected jump back in time" in RViz / slam_toolbox / tf2.
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("fov", default_value="true",
                              description="camera field-of-view markers (needs perception)"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="joint_state_publisher_gui (sliders)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}, sim],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[sim],
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="perception",
            executable="camera_fov_marker",
            remappings=[("fov_marker", "/camera/fov_marker")],
            parameters=[sim],
            condition=IfCondition(LaunchConfiguration("fov")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_file],
            parameters=[sim],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
