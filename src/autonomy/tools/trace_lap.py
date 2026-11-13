#!/usr/bin/env python3
"""Print one line per ~1 s of sim time: ground-truth position on the track next to what the car thinks.

    python3 src/autonomy/tools/trace_lap.py [seconds=60]

columns: sim time | s along track | d (true lateral from centreline, left +) | v | steer |
         Kalman offset / lane target / lane ref | camera offset, valid | lidar obstacles ahead (count, nearest x)
"""
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Float32MultiArray
from vision_msgs.msg import Detection3DArray

sys.path.insert(0, str(Path(__file__).parent))
from eval_lap import Eval  # noqa: E402


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    n = rclpy.create_node("trace_lap", parameter_overrides=[Parameter("use_sim_time", value=True)])
    ev = Eval()
    st = {"nv": 0, "state": [0] * 6, "cam": float("nan"), "valid": False, "obs": (0, 0.0), "last": -9.0}
    n.create_subscription(Float32MultiArray, "/autonomy/state", lambda m: st.update(state=list(m.data)), 10)
    n.create_subscription(Float32, "/perception/camera/ego_lane_offset", lambda m: st.update(cam=m.data), 10)
    n.create_subscription(Bool, "/perception/camera/lane_valid", lambda m: st.update(valid=m.data, nv=st["nv"] + 1), 10)

    def on_obs(m):
        xs = [d.bbox.center.position.x - d.bbox.size.x / 2 for d in m.detections if -1 < d.bbox.center.position.x < 15]
        st["obs"] = (len(xs), min(xs) if xs else float("inf"))
    n.create_subscription(Detection3DArray, "/perception/lidar/obstacles", on_obs, 10)

    def on_odom(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ev.on_odom(m, t)
        if t - st["last"] >= 1.0:
            st["last"] = t
            _, s, d, v, _w = ev.rows[-1]
            o, h, sv, steer, tgt, ref = st["state"][:6]
            print(f"t={t:6.1f} s={s:6.1f} d={d:6.2f} v={v:4.2f} steer={steer:5.2f} | kf_off={o:5.2f} tgt={tgt:.0f} ref={ref:4.2f} "
                  f"| cam={st['cam']:5.2f} {'ok ' if st['valid'] else 'BAD'}#{st['nv']} | obs n={st['obs'][0]} x={st['obs'][1]:5.1f}", flush=True)
    n.create_subscription(Odometry, "/odom", on_odom, 10)
    t0 = time.time()
    while time.time() - t0 < secs and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
    ev.report()


if __name__ == "__main__":
    main()
