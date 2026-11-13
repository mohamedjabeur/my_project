#!/usr/bin/env python3
"""End-to-end check of line_detector in Gazebo: place the car, compare the estimate to the ground truth.

Needs: the world running, the car spawned (name 'enimia'), ros_gz_bridge for /camera/image_raw,
and line_detector running. Truth comes from track_info.json (centreline) and the pose we set:
  offset  = lateral position of the car minus the centre of the lane it is in (m, + = left of the lane centre)
  heading = track direction minus car yaw (rad, + = the lane turns left relative to the car)

Usage: python3 eval_lane_offset.py --track ~/ros2/src/world/track/track_info.json --n 60
"""
import argparse
import json
import math
import random
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

WORLD = "shell_eco_track"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Probe(Node):
    def __init__(self):
        super().__init__("lane_probe")
        self.v = {"offset": None, "heading": None, "valid": None, "stop": None}
        self.create_subscription(Float32, "/perception/camera/ego_lane_offset", lambda m: self.v.__setitem__("offset", m.data), 10)
        self.create_subscription(Float32, "/perception/camera/heading_error", lambda m: self.v.__setitem__("heading", m.data), 10)
        self.create_subscription(Bool, "/perception/camera/lane_valid", lambda m: self.v.__setitem__("valid", m.data), 10)
        self.create_subscription(Float32, "/perception/camera/stop_line_distance", lambda m: self.v.__setitem__("stop", m.data), 10)

    def spin(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)


def place(x, y, yaw):
    req = (f'name: "enimia" position {{x: {x}, y: {y}, z: 0.12}} '
           f'orientation {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    subprocess.run(["ign", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "ignition.msgs.Pose",
                    "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", req], capture_output=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", required=True)
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--settle", type=float, default=3.0)
    a = ap.parse_args()
    t = json.load(open(a.track))
    rng = random.Random(a.seed)
    rclpy.init()
    node = Probe()
    rows = []
    for _ in range(a.n):
        s = rng.uniform(0, t["lap"])
        i = int(s / t["ds"]) % len(t["centerline"])
        lane = -t["lane_c"]                                         # no centre line: always the right lane
        lat = lane + rng.uniform(-0.8, 0.8)                         # lateral position of the car (left +)
        yaw_err = math.radians(rng.uniform(-10, 10))
        cx, cy = t["centerline"][i]
        ty = t["yaw"][i]
        x, y = cx - math.sin(ty) * lat, cy + math.cos(ty) * lat
        place(x, y, ty + yaw_err)
        node.spin(a.settle)
        node.v.update(offset=None, heading=None, valid=None)
        node.spin(1.5)
        truth_off = lat + t["lane_c"]
        truth_head = wrap(-yaw_err)
        ok = node.v["valid"] and node.v["offset"] is not None and not math.isnan(node.v["offset"])
        rows.append({"s": round(s), "lat": round(lat, 2), "valid": bool(ok),
                     "off_true": truth_off, "off_est": node.v["offset"] if ok else None,
                     "head_true": truth_head, "head_est": node.v["heading"] if ok else None})
        print(rows[-1], flush=True)
    good = [r for r in rows if r["valid"]]
    eo = np.array([abs(r["off_est"] - r["off_true"]) for r in good])
    eh = np.array([abs(math.degrees(wrap(r["head_est"] - r["head_true"]))) for r in good])
    print(json.dumps({"poses": len(rows), "valid_rate": round(len(good) / len(rows), 3),
                      "offset_abs_err_m": {"mean": round(float(eo.mean()), 3), "p90": round(float(np.percentile(eo, 90)), 3)} if len(good) else None,
                      "heading_abs_err_deg": {"mean": round(float(eh.mean()), 2), "p90": round(float(np.percentile(eh, 90)), 2)} if len(good) else None}))


if __name__ == "__main__":
    main()
