"""All real-car sensors in one command (run ON the Jetson): Mid-360 lidar + BNO055 IMU + OAK-D Lite camera + URDF TF.

  ros2 launch vehicle_drivers jetson_sensors.launch.py [i2c_bus:=7]

Outputs /scan, /livox/lidar, /imu/data, /camera/image_raw, /camera/camera_info and the TF tree (robot_state_publisher).
Not tested on hardware. Perception / autonomy / drive chain are started separately (see the Jetson PDF guide).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    inc = lambda pkg, f, args=None: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), "launch", f])),
        launch_arguments=(args or {}).items())
    return LaunchDescription([
        DeclareLaunchArgument("i2c_bus", default_value="7"),
        # TF (lidar_link, lidar_level_link, imu_link, camera_link ...) from the single-source URDF; real clock
        inc("vehicle_description", "description.launch.py", {"use_sim_time": "false", "rviz": "false", "gui": "false", "fov": "false"}),  # headless on the car
        inc("vehicle_drivers", "lidar.launch.py", {"use_sim": "false", "lidar_model": "mid360"}),
        inc("vehicle_drivers", "imu.launch.py", {"use_sim": "false", "i2c_bus": LaunchConfiguration("i2c_bus")}),
        inc("vehicle_drivers", "camera.launch.py", {"use_sim": "false", "camera_model": "oakd_lite"}),
    ])
