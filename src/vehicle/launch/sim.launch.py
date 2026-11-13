"""Simulation drive chain (Gazebo must be running with the car spawned as 'enimia'):

  /cmd_vel -> ackermann_controller -> /drive_cmd -> virtual_esp32 (BLDC model + virtual encoder)
           -> Gazebo wheels;  /wheel_feedback -> wheel_odometry -> /odom + TF odom->base_footprint
Also: robot_state_publisher, lidar bridge (/scan), /clock.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    here = lambda pkg, name: PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(pkg), "launch", name]))
    cfg = PathJoinSubstitution([FindPackageShare("vehicle"), "config", "vehicle.yaml"])
    sim_time = {"use_sim_time": True}

    description = IncludeLaunchDescription(
        here("vehicle_description", "description.launch.py"),
        launch_arguments={"rviz": "false", "gui": "false", "fov": "false", "use_sim_time": "true"}.items())
    lidar = IncludeLaunchDescription(here("vehicle_drivers", "lidar.launch.py"),
                                     launch_arguments={"use_sim": "true"}.items())
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="drive_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/enimia/rear_wheel_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/enimia/steer_left_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/enimia/steer_right_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/enimia/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",   # ground truth: only feeds the virtual encoder
        ])

    return LaunchDescription([
        DeclareLaunchArgument("cmd_timeout", default_value="0.0",
                              description="s without /cmd_vel before stopping (0 = off, needed for keyboard teleop)"),
        description, lidar, bridge,
        Node(package="vehicle", executable="ackermann_controller", output="screen",
             parameters=[cfg, sim_time, {"cmd_timeout": LaunchConfiguration("cmd_timeout")}]),
        Node(package="vehicle", executable="virtual_esp32", output="screen", parameters=[cfg, sim_time]),
        Node(package="vehicle", executable="wheel_odometry", output="screen", parameters=[cfg, sim_time]),
    ])
