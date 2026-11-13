"""Lidar -> /scan (LaserScan, the only lidar input of perception / Nav2 / autonomy).

  use_sim:=true                      Gazebo 3-D lidar (Mid-360 emulation) -> /livox/lidar -> cloud_to_scan.py -> /scan
  lidar_model:=mid360 (default)      Livox Mid-360 (3D PointCloud2 on /livox/lidar) -> pointcloud_to_laserscan -> /scan
  lidar_model:=rplidar               RPLIDAR A1M8 (serial) -> /scan
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def real_model(name):
    """Condition: real hardware (use_sim false) AND lidar_model == name."""
    return IfCondition(PythonExpression(
        ["'", LaunchConfiguration("use_sim"), "' == 'false' and '",
         LaunchConfiguration("lidar_model"), "' == '", name, "'"]))


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    share = lambda *p: PathJoinSubstitution([FindPackageShare("vehicle_drivers"), *p])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="true: bridge the Gazebo lidar instead of the real one"),
        DeclareLaunchArgument("lidar_model", default_value="mid360", description="mid360 | rplidar"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0", description="RPLIDAR only"),
        DeclareLaunchArgument("livox_config", default_value=share("config", "livox_mid360.json"),
                              description="Mid-360 network config (host + lidar IP)"),

        # --- Livox Mid-360 (Ethernet): PointCloud2 on /livox/lidar (xfer_format 0 = standard PointCloud2)
        Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_lidar",
            output="screen",
            parameters=[{
                "xfer_format": 0, "multi_topic": 0, "data_src": 0, "publish_freq": 10.0,
                "output_data_type": 0, "frame_id": "lidar_link", "lvx_file_path": "/tmp/livox.lvx",
                "user_config_path": LaunchConfiguration("livox_config"), "cmdline_input_bd_code": "livox0000000001",
            }],
            condition=real_model("mid360"),
        ),
        # 3D cloud -> 2D scan in a LEVEL frame at lidar height (lidar_level_link, see URDF)
        Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_scan", output="screen",
            parameters=[share("config", "pointcloud_to_scan.yaml")],
            remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
            condition=real_model("mid360"),
        ),

        # --- RPLIDAR A1M8 (serial)
        Node(
            package="rplidar_ros", executable="rplidar_node", name="rplidar_node", output="screen",
            parameters=[share("config", "rplidar_a1.yaml"), {"serial_port": LaunchConfiguration("serial_port")}],
            remappings=[("scan", "/scan")],
            condition=real_model("rplidar"),
        ),

        # --- simulation: Gazebo 3-D lidar (Mid-360 emulation) -> /livox/lidar (PointCloud2) -> pointcloud_to_laserscan
        # -> /scan: the same chain as the real Mid-360 above, so sim and real behave alike
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="lidar_bridge", output="screen",
            arguments=["/lidar_link/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked"],
            remappings=[("/lidar_link/scan/points", "/livox/lidar")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(use_sim),
        ),
        # (pointcloud_to_laserscan is not installed with the sim: cloud_to_scan.py does the same with the same yaml)
        Node(
            package="vehicle_drivers", executable="cloud_to_scan.py",
            name="pointcloud_to_scan", output="screen",
            parameters=[share("config", "pointcloud_to_scan.yaml"), {"use_sim_time": True}],
            remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
            condition=IfCondition(use_sim),
        ),
    ])
