# localization: Gazebo spawn + simulated drive path

Simulation only (mapping, map saving and AMCL were removed on 2026-11-08: driving uses the camera, the lidar and odometry,
never a map; copy in ~/ros2_backup_2026-11-08.tar.gz).

- `launch/sim.launch.py`: Ignition Gazebo with the Shell Eco track, spawns the car (`x`, `y`, `yaw`, `world`, `gui_config` arguments).
- `launch/teleop.launch.py`: robot_state_publisher, lidar, `/clock` bridge, `cmd_vel_to_drive` (`/cmd_vel` -> wheel commands),
  `odom_tf` (ground-truth `/odom` + `odom -> base_footprint` TF). Don't run it together with `vehicle/sim.launch.py`.

Used by `ros2 launch autonomy drive.launch.py` (one command: Gazebo + perception + autonomy + RViz).
