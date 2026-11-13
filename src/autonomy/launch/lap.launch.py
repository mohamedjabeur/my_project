"""One command for a lap in simulation: Gazebo + car + drive chain + lidar/camera perception + autonomy.

    ros2 launch autonomy lap.launch.py [v_cruise:=5.56] [x:=0 y:=0 yaw:=0]

    ros2 launch autonomy drive.launch.py      # + RViz waypoint click (stands still until you click)

Do NOT run Nav2 or keyboard teleop at the same time (the autonomy node publishes /cmd_vel).
A second simulation at the same time is refused (two Gazebo = two /clock = "Detected jump back in time" in RViz / tf2);
set ENIMIA_ALLOW_SECOND_SIM=1 to override.
"""
import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def refuse_second_simulation():
    """Stop with a clear message if a Gazebo server of an earlier launch is still alive."""
    if os.environ.get("ENIMIA_ALLOW_SECOND_SIM"):
        return
    out = subprocess.run(["pgrep", "-f", "ign gazebo server"], capture_output=True, text=True).stdout.split()
    if out:
        raise SystemExit("A simulation is already running (ign gazebo server PID %s). Press Ctrl+C in its terminal first: two "
                         "simulations give two /clock publishers and 'Detected jump back in time'. "
                         "(override: ENIMIA_ALLOW_SECOND_SIM=1)" % ", ".join(out))


def generate_launch_description():
    refuse_second_simulation()
    here = lambda pkg, name: PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(pkg), "launch", name]))
    pose = {k: LaunchConfiguration(k) for k in ("x", "y", "yaw")}

    sim = IncludeLaunchDescription(here("localization", "sim.launch.py"), launch_arguments=pose.items())
    drive = IncludeLaunchDescription(here("localization", "teleop.launch.py"))
    camera = IncludeLaunchDescription(here("vehicle_drivers", "camera.launch.py"),
                                      launch_arguments={"use_sim": "true"}.items())
    # the sim lidar emulates the Mid-360 (3-D cloud -> /scan), so use its detector overrides (25 m range, beam gaps)
    lidar_perc = IncludeLaunchDescription(here("perception", "lidar_perception.launch.py"),
                                          launch_arguments={"override": "lidar_obstacle_detector_mid360.yaml"}.items())
    # the sim camera emulates the OAK-D Lite (960x540, HFOV 69 deg): same detector settings as the real car
    camera_perc = IncludeLaunchDescription(here("perception", "camera_perception.launch.py"),
                                           launch_arguments={"config": "line_detector_oak.yaml"}.items())
    params = PathJoinSubstitution([FindPackageShare("autonomy"), "config", "autonomy.yaml"])
    fusion = Node(package="autonomy", executable="fusion_node", output="screen",
                  parameters=[params, {"use_sim_time": True,
                              "v_cruise": ParameterValue(LaunchConfiguration("v_cruise"), value_type=float),
                              "require_goal": ParameterValue(LaunchConfiguration("require_goal"), value_type=bool),
                              "stop_at_lines": ParameterValue(LaunchConfiguration("stop_at_lines"), value_type=bool)}],
                  remappings=[("/cmd_vel_lane", "/cmd_vel")])

    # --- optional: RViz (waypoint click, car, camera)
    use_rviz = IfCondition(LaunchConfiguration("open_rviz"))
    world_sdf = str(Path(get_package_prefix("localization")).parents[1] / "src" / "world" / "track" / "shell_eco_track.sdf")
    markers = Node(condition=use_rviz, package="navigation", executable="world_markers", output="screen",
                   parameters=[{"use_sim_time": True, "sdf": world_sdf}])
    joints = Node(condition=use_rviz, package="joint_state_publisher", executable="joint_state_publisher",
                  output="screen", parameters=[{"use_sim_time": True}])
    fov = Node(condition=use_rviz, package="perception", executable="camera_fov_marker", output="screen",
               remappings=[("fov_marker", "/camera/fov_marker")],
               parameters=[{"use_sim_time": True, "width": 960, "height": 540, "fx": 698.0, "fy": 698.0, "cx": 480.0, "cy": 270.0}])
    speed = Node(condition=use_rviz, package="autonomy", executable="speed_display", output="screen",
                 parameters=[{"use_sim_time": True}])
    rviz = Node(condition=use_rviz, package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", PathJoinSubstitution([FindPackageShare("autonomy"), "rviz", "drive.rviz"])],
                parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("v_cruise", default_value="5.56"),
        DeclareLaunchArgument("require_goal", default_value="false", description="true: stand still until a waypoint is clicked"),
        DeclareLaunchArgument("stop_at_lines", default_value="true", description="false: never stop at stop lines"),
        DeclareLaunchArgument("open_rviz", default_value="false"),
        DeclareLaunchArgument("x", default_value="-0.552"),   # right lane centre at the start line
        DeclareLaunchArgument("y", default_value="-1.661"),
        DeclareLaunchArgument("yaw", default_value="-0.321"),   # track heading at s=0
        sim, drive,
        # wait for Gazebo, the car and TF before starting the sensors and the controller
        TimerAction(period=12.0, actions=[camera, lidar_perc, camera_perc, fusion, markers, joints, fov, speed, rviz]),
    ])
