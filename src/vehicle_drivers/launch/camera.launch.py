"""Front camera -> /camera/image_raw + /camera/camera_info (what perception expects).

  use_sim:=true                    Gazebo camera bridged to ROS
  camera_model:=oakd_lite          Luxonis OAK-D Lite (USB3, depthai_ros_driver), default for the real car
  camera_model:=v4l2               generic V4L2 webcam (camera_driver.py), device:=/dev/video0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def real_model(name):
    """Condition: real hardware (use_sim false) AND camera_model == name."""
    return IfCondition(PythonExpression(
        ["'", LaunchConfiguration("use_sim"), "' == 'false' and '",
         LaunchConfiguration("camera_model"), "' == '", name, "'"]))


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    share = lambda pkg, *p: PathJoinSubstitution([FindPackageShare(pkg), *p])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="true: bridge the Gazebo camera instead of the real one"),
        DeclareLaunchArgument("camera_model", default_value="oakd_lite", description="oakd_lite | v4l2"),
        DeclareLaunchArgument("device", default_value="/dev/video0", description="v4l2 only"),

        # --- OAK-D Lite: publishes /oak/rgb/image_raw + /oak/rgb/camera_info, relayed to /camera/*
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("depthai_ros_driver", "launch", "camera.launch.py")),
            launch_arguments={"name": "oak", "camera_model": "OAK-D-LITE",
                              "params_file": share("vehicle_drivers", "config", "oak_d_lite.yaml"),
                              "parent_frame": "camera_link", "publish_tf": "false"}.items(),
            condition=real_model("oakd_lite")),
        Node(package="topic_tools", executable="relay", name="oak_image_relay",
             arguments=["/oak/rgb/image_raw", "/camera/image_raw"], condition=real_model("oakd_lite")),
        Node(package="topic_tools", executable="relay", name="oak_info_relay",
             arguments=["/oak/rgb/camera_info", "/camera/camera_info"], condition=real_model("oakd_lite")),

        # --- generic V4L2 camera
        Node(
            package="vehicle_drivers", executable="camera_driver.py", name="camera_driver", namespace="camera",
            output="screen",
            parameters=[share("vehicle_drivers", "config", "camera.yaml"), {"device": LaunchConfiguration("device")}],
            condition=real_model("v4l2"),
        ),

        # --- simulation: Gazebo camera -> ROS
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="camera_bridge", output="screen",
            arguments=["/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
                       "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo"],
            condition=IfCondition(use_sim),
        ),
    ])
