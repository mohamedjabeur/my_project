"""One command: Gazebo + car + drive nodes + Nav2 + RViz. Give a goal with the RViz '2D Goal Pose' tool
or:  ros2 run navigation send_goal X Y [YAW_DEG]   (goals are in the odom frame = world frame in simulation)"""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    here = lambda pkg, name: PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(pkg), "launch", name]))

    sim = IncludeLaunchDescription(here("localization", "sim.launch.py"))
    drive = IncludeLaunchDescription(here("localization", "teleop.launch.py"))   # bridge, /cmd_vel -> wheels, odom, lidar
    nav = IncludeLaunchDescription(here("navigation", "navigation.launch.py"),
                                   launch_arguments={"use_sim_time": "true"}.items())
    rviz = Node(
        condition=IfCondition(LaunchConfiguration("open_rviz")),
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("navigation"), "rviz", "navigation.rviz"])],
        parameters=[{"use_sim_time": True}],
    )

    # draw the Gazebo world in RViz (world frame = odom frame in simulation)
    world_dir = Path(get_package_prefix("localization")).parents[1] / "src" / "world" / "track"
    world_sdf = str(world_dir / "shell_eco_track.sdf")
    markers = Node(condition=IfCondition(LaunchConfiguration("open_rviz")), package="navigation",
                   executable="world_markers", output="screen",
                   parameters=[{"use_sim_time": True, "sdf": world_sdf}])
    # wheels/steering have no joint states in the sim: publish zeros so RViz can place every link of the car
    joints = Node(condition=IfCondition(LaunchConfiguration("open_rviz")), package="joint_state_publisher",
                  executable="joint_state_publisher", output="screen", parameters=[{"use_sim_time": True}])

    # front camera: Gazebo -> ROS image, plus its field of view on the ground in RViz (teleop.launch turns fov off)
    camera = IncludeLaunchDescription(here("vehicle_drivers", "camera.launch.py"),
                                      launch_arguments={"use_sim": "true"}.items())
    fov = Node(package="perception", executable="camera_fov_marker", output="screen",
               remappings=[("fov_marker", "/camera/fov_marker")],
               parameters=[{"use_sim_time": True, "width": 960, "height": 540, "fx": 698.0, "fy": 698.0, "cx": 480.0, "cy": 270.0}])

    relay = Node(package="navigation", executable="goal_relay", output="screen",
                 parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("open_rviz", default_value="true"),
        sim, drive,
        # wait for Gazebo, the car and TF before starting Nav2
        TimerAction(period=12.0, actions=[nav, markers, joints, rviz, relay, camera, fov]),
    ])
