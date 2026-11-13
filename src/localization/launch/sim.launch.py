"""Opens Gazebo (track + GUI with the camera panel) and spawns the ENIMIA car."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # the world is not an installed package: it lives in <workspace>/src/world
    world_dir = Path(get_package_prefix("localization")).parents[1] / "src" / "world"
    vehicle_share = get_package_share_directory("vehicle_description")

    urdf = PathJoinSubstitution([FindPackageShare("vehicle_description"), "urdf", "enimia.urdf"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=str(world_dir / "track" / "shell_eco_track.sdf"),
                              description="world SDF to load"),
        DeclareLaunchArgument("gui_config", default_value=str(world_dir / "gui.config")),
        # start of shell_eco_track
        DeclareLaunchArgument("x", default_value="0"),
        DeclareLaunchArgument("y", default_value="0"),
        DeclareLaunchArgument("yaw", default_value="0"),

        # lets Gazebo resolve package://vehicle_description/meshes/...
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.pathsep.join([str(Path(vehicle_share).parent), str(world_dir),
                             os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")])),

        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", LaunchConfiguration("world"),
                 "--gui-config", LaunchConfiguration("gui_config")],
            output="screen",
        ),
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_sim", "create", "-file", urdf, "-name", "enimia",
                     "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"), "-z", "0.3",
                     "-Y", LaunchConfiguration("yaw")],
                output="screen",
            ),
        ]),
    ])
