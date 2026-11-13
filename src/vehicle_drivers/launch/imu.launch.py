"""BNO055 IMU (I2C) -> /imu/data (sensor_msgs/Imu, frame imu_link). Simulation: Gazebo IMU bridged to /imu/data.

  ros2 launch vehicle_drivers imu.launch.py [use_sim:=true]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    config = PathJoinSubstitution([FindPackageShare("vehicle_drivers"), "config", "bno055.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),
        # Jetson Orin 40-pin header (pins 3/5) = /dev/i2c-7 (Raspberry Pi = 1). Check: i2cdetect -l ; i2cdetect -y -r 7
        DeclareLaunchArgument("i2c_bus", default_value="7", description="I2C bus number of the BNO055"),
        Node(
            package="bno055", executable="bno055", name="bno055", output="screen",
            parameters=[config, {"i2c_bus": ParameterValue(LaunchConfiguration("i2c_bus"), value_type=int)}],
            remappings=[("bno055/imu", "/imu/data"), ("bno055/mag", "/imu/mag"),
                        ("bno055/calib_status", "/imu/calib_status")],
            condition=UnlessCondition(use_sim),
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="imu_bridge", output="screen",
            arguments=["/imu_link/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
            remappings=[("/imu_link/imu", "/imu/data")],
            condition=IfCondition(use_sim),
        ),
    ])
