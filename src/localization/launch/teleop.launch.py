"""Sim bridge + drive nodes so the car can be driven with /cmd_vel.

Then, in another terminal:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
(Gazebo with the car spawned as 'enimia' must already be running.)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg = lambda name: FindPackageShare(name)

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg("vehicle_description"), "launch", "description.launch.py"])),
        launch_arguments={"rviz": "false", "gui": "false", "fov": "false", "use_sim_time": use_sim_time}.items(),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg("vehicle_drivers"), "launch", "lidar.launch.py"])),
        launch_arguments={"use_sim": "true"}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drive_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/enimia/rear_wheel_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/enimia/steer_left_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/enimia/steer_right_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/enimia/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
    )
    drive = Node(package="localization", executable="cmd_vel_to_drive", output="screen",
                 parameters=[{"use_sim_time": use_sim_time}])
    odom = Node(package="localization", executable="odom_tf", output="screen",
                parameters=[{"use_sim_time": use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        description, lidar, bridge, drive, odom,
    ])
