"""Fusion + lane-keeping node: sensors -> Kalman filter -> /cmd_vel_lane (Twist).

Reads  /odom (forward speed: wheel odometry on the car, ground truth in the direct sim path) or /wheel_feedback
       (encoder, `speed_source: encoder`) + steering from /wheel_feedback, /perception/camera/{ego_lane_offset,heading_error,lane_valid,
       stop_line_distance}, /perception/lidar/ego_clear_distance.
Writes /autonomy/state (JointState-free: Float32MultiArray [offset, heading, speed, steer, target lane (0 right, 1 left), lateral ref]) and /cmd_vel_lane.
Twist convention for ackermann_controller: angular.z = v * tan(steer) / wheelbase.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray
from vision_msgs.msg import Detection3DArray

from autonomy.avoidance import LaneChanger, Obstacle
from autonomy.goal_gate import GoalGate
from autonomy.kalman import LaneEKF
from autonomy.lane_controller import LaneController


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.L = p("wheelbase", 1.704)
        self.r_wheel = p("rear_wheel_radius", 0.18)
        self.ekf = LaneEKF(self.L, (p("q_offset", 0.02), p("q_heading", 0.01), p("q_speed", 0.5)), p("gate", 9.0))
        self.r_off, self.r_head, self.r_speed = p("r_offset", 0.04), p("r_heading", 0.01), p("r_speed", 0.01)
        self.ctrl = LaneController(p("v_cruise", 2.0), p("v_curve", 0.8), p("k_stanley", 1.0), p("k_soft", 0.5),
                                   p("decel", 1.0), p("obstacle_margin", 3.0), p("stop_margin", 0.5),
                                   p("stop_hold", 3.0), p("stop_ignore", 6.0), p("stop_trigger", 1.5), p("v_avoid", 3.5))
        self.changer = LaneChanger(p("lane_shift", 3.4), p("lane_change_slope", 0.3), p("look_ahead", 14.0),
                                   p("slow_dist", 15.0))
        self.accel = p("accel", 1.5)               # m/s^2 limit on speeding up (braking is never delayed)
        self.v_prev = 0.0
        self.gate = GoalGate(self.ctrl.decel, p("goal_stop_margin", 0.3), p("goal_reach_dist", 3.0),
                             latency=p("goal_latency", 0.5), near_dist=p("goal_near_dist", 40.0))
        self.require_goal = p("require_goal", False)   # True: stand still until a waypoint is given
        self.stop_at_lines = p("stop_at_lines", True)   # False: only the waypoint stops the car
        self.pose = None                           # (x, y, yaw) in odom, from /odom
        self.lidar_timeout = p("lidar_timeout", 1.0)   # s without a lidar result: creep, we are blind
        self.obstacles, self.obs_t = [], -1e9
        self.cam_delay = p("camera_delay", 0.45)   # s from the scene to the detector output (render + 0.22 s processing)
        self.timeout = p("sensor_timeout", 0.5)
        self.speed_source = p("speed_source", "odom")      # "odom" works in both sim drive paths and on the car
        self.steer = 0.0
        self.have_feedback = False                 # no /wheel_feedback (direct sim path): use our own command as steering
        self.last_t = None
        self.last_angle = None
        self.last_enc_t = None                      # wall of the last encoder sample (node clock)
        self.cam = {"offset": math.nan, "heading": math.nan, "valid": False, "t": -1e9}
        self.stop_line = self.obstacle = math.inf

        self.create_subscription(JointState, "/wheel_feedback", self.on_feedback, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Float32, "/perception/camera/ego_lane_offset", lambda m: self.cam.update(offset=m.data), 10)
        self.create_subscription(Float32, "/perception/camera/heading_error", lambda m: self.cam.update(heading=m.data), 10)
        self.create_subscription(Bool, "/perception/camera/lane_valid", self.on_lane_valid, 10)
        self.create_subscription(Float32, "/perception/camera/stop_line_distance", lambda m: setattr(self, "stop_line", m.data), 10)
        self.create_subscription(Detection3DArray, "/perception/lidar/obstacles", self.on_obstacles, 10)
        # waypoint: RViz "2D Goal Pose" (topic /nav_goal) -> drive the lane to it and stop; /nav_goal_sent = accepted goal
        self.create_subscription(PoseStamped, p("goal_topic", "/nav_goal"), self.on_goal, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/nav_goal_sent", 1)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_lane", 10)
        self.state_pub = self.create_publisher(Float32MultiArray, "/autonomy/state", 10)
        self.control_rate = p("control_rate", 20.0)
        self.create_timer(1.0 / self.control_rate, self.on_timer)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def advance(self):
        """Predict the filter up to the current time."""
        t = self.now()
        if self.last_t is not None:
            self.ekf.predict(t - self.last_t, self.steer)
        self.last_t = t
        return t

    def on_feedback(self, msg):
        try:
            angle = msg.position[msg.name.index("rear_wheel_joint")]
            self.steer = msg.position[msg.name.index("steer")]
            self.have_feedback = True
        except ValueError:
            return
        t = self.advance()
        if self.speed_source == "encoder":
            if self.last_angle is not None and self.last_enc_t is not None and t > self.last_enc_t:
                self.ekf.update(2, (angle - self.last_angle) * self.r_wheel / (t - self.last_enc_t), self.r_speed)
            self.last_enc_t = t
        self.last_angle = angle

    def on_goal(self, msg):
        if self.pose is None:
            self.get_logger().warn("no /odom yet: goal ignored")
            return
        if msg.header.frame_id not in ("odom", ""):
            self.get_logger().warn(f"goal frame '{msg.header.frame_id}' != odom: set the RViz fixed frame to odom")
            return
        if self.gate.set_goal(msg.pose.position.x, msg.pose.position.y, *self.pose):
            self.goal_pub.publish(msg)
            self.get_logger().info("goal accepted: (%.1f, %.1f), %.1f m away (straight line): driving the lane, then stopping"
                                   % (msg.pose.position.x, msg.pose.position.y, self.gate.remaining(*self.pose[:2])))
        else:
            self.get_logger().warn("goal rejected: closer than 40 m it must be in front of the car (no reverse) and more than 1 m away")

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        if self.speed_source != "odom":
            return
        t = self.advance()
        self.last_enc_t = t
        self.ekf.update(2, msg.twist.twist.linear.x, self.r_speed)

    def on_lane_valid(self, msg):
        # line_detector publishes offset, heading, then valid: this one closes the camera frame
        t = self.advance()
        self.cam["valid"], self.cam["t"] = bool(msg.data), t
        if msg.data:
            # the picture is `cam_delay` old: move the measurement forward with the motion since then, otherwise the
            # controller keeps steering on a stale position and overshoots (seen in sim at 3 m/s: +-2 m weaving)
            v, psi, d = self.ekf.speed, self.ekf.heading, self.cam_delay
            self.ekf.update(0, self.cam["offset"] + v * math.sin(psi) * d, self.r_off)
            self.ekf.update(1, -self.cam["heading"] + v * math.tan(self.steer) / self.L * d, self.r_head)  # psi = car heading - lane direction

    def on_obstacles(self, msg):
        # nearest face, lateral centre and half width of every lidar cluster (base_footprint frame)
        self.obstacles = [Obstacle(d.bbox.center.position.x - d.bbox.size.x / 2.0, d.bbox.center.position.y,
                                   d.bbox.size.y / 2.0) for d in msg.detections]
        self.obs_t = self.now()

    def on_timer(self):
        t = self.advance()
        enc_ok = self.last_enc_t is not None and t - self.last_enc_t < self.timeout
        lane_ok = self.cam["valid"] and t - self.cam["t"] < self.timeout
        kappa = math.tan(self.steer) / self.L                       # lane curvature ~ what the car is following
        plan = self.changer.update(1.0 / self.control_rate, self.ekf.speed, self.ekf.offset, self.ekf.heading,
                                   kappa, self.obstacles)
        v_cmd, steer = self.ctrl.command(t, self.ekf.offset, self.ekf.heading, self.ekf.speed,
                                         lane_ok, plan["clear"], self.stop_line if self.stop_at_lines else math.inf,
                                         plan["ref"], plan["heading_ref"], plan["slow"])
        if t - self.obs_t > self.lidar_timeout:
            v_cmd = min(v_cmd, self.ctrl.v_curve)                   # no lidar data: creep
        if self.require_goal and self.gate.goal is None:
            v_cmd = 0.0
        if self.pose is not None:
            was = self.gate.reached
            v_cmd = min(v_cmd, self.gate.speed_cap(*self.pose, self.ekf.speed))
            if self.gate.reached and not was:
                self.get_logger().info("goal reached: holding at (%.1f, %.1f)" % self.pose[:2])
        v_cmd = min(v_cmd, max(self.v_prev, self.ekf.speed) + self.accel / self.control_rate)   # ramp up smoothly
        if not enc_ok:
            v_cmd = 0.0                                              # blind on speed: do not drive
        if not self.have_feedback:
            self.steer = steer
        self.v_prev = v_cmd
        cmd = Twist()
        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(v_cmd * math.tan(steer) / self.L)
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(Float32MultiArray(data=[self.ekf.offset, self.ekf.heading, self.ekf.speed, steer, float(plan["target"]), plan["ref"]]))


def main():
    rclpy.init()
    rclpy.spin(FusionNode())


if __name__ == "__main__":
    main()
