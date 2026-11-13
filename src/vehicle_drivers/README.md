# vehicle_drivers: real-car sensors

| Sensor | Launch | Output | ROS driver (apt / source) |
|---|---|---|---|
| Livox Mid-360 | `lidar.launch.py` (`lidar_model:=mid360`) | `/livox/lidar` (PointCloud2) → `/scan` | `livox_ros_driver2` (source, needs Livox-SDK2), `pointcloud_to_laserscan` |
| BNO055 (I2C) | `imu.launch.py` | `/imu/data`, `/imu/mag`, `/imu/calib_status` | `bno055` (`ros-humble-bno055` or source) |
| OAK-D Lite | `camera.launch.py` (`camera_model:=oakd_lite`) | `/oak/rgb/*` → `/camera/image_raw`, `/camera/camera_info` | `ros-humble-depthai-ros` |

(The all-in-one real-car launch files `vehicle/hardware.launch.py`, `autonomy/real.launch.py` and the ESP32 Arduino firmware were removed on 2026-11-08 to keep the project simulation-only; copy in ~/ros2_backup_2026-11-08.tar.gz.)
Old sensors still work: `lidar_model:=rplidar`, `camera_model:=v4l2`. `use_sim:=true` bridges Gazebo as before.

## To do on the real car (nothing here was tested with hardware)
- **Mid-360**: wired Ethernet, host NIC static IP `192.168.1.50/24`; lidar IP is `192.168.1.1xx` (xx = last 2 digits of the serial).
  Edit `config/livox_mid360.json` (`ip`). Mount it level; `pointcloud_to_scan.yaml` keeps points 0.18 m to 1.08 m above the road.
- **BNO055**: enable I2C, `i2cdetect -y 1` (0x28/0x29). Calibrate (move it until `/imu/calib_status` is 3/3/3/3). The mag is disturbed by the BLDC motor: check the yaw.
  The IMU is not read by `fusion_node` yet (it uses `/odom`).
- **OAK-D Lite**: USB3 cable, udev rule from the depthai docs. HFOV is ~69° (sim camera: 100°) so `perception/config/line_detector.yaml`
  (`fx fy cx cy`, `camera_height`, `camera_pitch`, `car_mask`) MUST be redone from the real `/camera/camera_info` and a bonnet image.
  `i_width/i_height` in `oak_d_lite.yaml` must match `cx, cy` (960x540 → `cx=480, cy=270`).
- Real mount positions replace the sim ones in `enimia.urdf` (`lidar_joint`, `lidar_level_joint`, `imu_joint`, `camera_joint`).
- Verify the `depthai_ros_driver` launch arguments (`name`, `params_file`, `parent_frame`) against the installed version.
