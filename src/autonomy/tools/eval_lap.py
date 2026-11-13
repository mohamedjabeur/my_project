#!/usr/bin/env python3
"""Score a lap in Gazebo with GROUND TRUTH (/odom of the direct sim path), independent of the car's own sensors.

    python3 src/autonomy/tools/eval_lap.py [seconds=600] [stop_at_s=0]

Reports: progress along the track, time in right / left lane, white-line crossings (car body over an edge line),
collisions and minimum clearance to every obstacle (oriented rectangles), stops (v < 0.1 m/s), speed statistics.
Ctrl+C prints the report early. Track data: src/world/track/track_info.json (centreline, yaw, obstacles).
"""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter

TRACK = Path(__file__).resolve().parents[2] / "world" / "track" / "track_info.json"
CAR_BACK, CAR_FRONT, CAR_HALF = -1.3, 1.445, 0.385          # base_footprint frame (enimia.urdf)
EDGE_D = 3.375                                               # lateral position of the white edge lines
SPEC = {"cone": (0.3, 0.3), "box": (1.0, 1.0), "bigbox": (2.0, 1.0), "barrel": (0.6, 0.6),
        "person": (0.4, 0.4), "wall": (4.0, 0.4)}


def rect_dist(px, py, cx, cy, yaw, half_a, half_l):
    """Distance from a point to an oriented rectangle (0 inside)."""
    dx, dy = px - cx, py - cy
    a = dx * math.cos(yaw) + dy * math.sin(yaw)
    l = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return math.hypot(max(abs(a) - half_a, 0.0), max(abs(l) - half_l, 0.0))


class Eval:
    def __init__(self):
        d = json.load(open(TRACK))
        self.c, self.yaw, self.n = d["centerline"], d["yaw"], len(d["centerline"])
        self.obs = []
        for o in d["obstacles"]:
            a, l = SPEC[o["kind"].partition(":")[0]]
            self.obs.append((o["s"], o["x"], o["y"], self.yaw[int(o["s"]) % self.n], a / 2, l / 2, o["kind"]))
        self.i = 0
        self.rows = []
        self.min_clear = {o[0]: 9e9 for o in self.obs}
        self.collided = set()
        self.cross = 0
        self.crossing = False
        self.lap_s = 0.0
        self.prev_i = 0

    def locate(self, x, y):
        best, bi = 9e18, self.i
        for k in range(-6, 16):            # small window: the track passes close to itself, never jump to another part
            j = (self.i + k) % self.n
            dd = (self.c[j][0] - x) ** 2 + (self.c[j][1] - y) ** 2
            if dd < best:
                best, bi = dd, j
        if bi < self.prev_i - self.n // 2:
            self.lap_s += self.n                       # wrapped around the start
        self.prev_i = bi
        self.i = bi
        yw = self.yaw[bi]
        d = -(x - self.c[bi][0]) * math.sin(yw) + (y - self.c[bi][1]) * math.cos(yw)   # left of centreline +
        return bi + self.lap_s, d

    def on_odom(self, m, t):
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        q = m.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        s, d = self.locate(x, y)
        v = m.twist.twist.linear.x
        # corners/sides of the car body -> over the edge line?
        corners = [(CAR_FRONT, CAR_HALF), (CAR_FRONT, -CAR_HALF), (CAR_BACK, CAR_HALF), (CAR_BACK, -CAR_HALF)]
        yw, c0 = self.yaw[self.i], self.c[self.i]
        worst = 0.0
        for cx, cy in corners:
            wx = x + cx * math.cos(th) - cy * math.sin(th)
            wy = y + cx * math.sin(th) + cy * math.cos(th)
            worst = max(worst, abs(-(wx - c0[0]) * math.sin(yw) + (wy - c0[1]) * math.cos(yw)))
        over = worst > EDGE_D
        if over and not self.crossing:
            self.cross += 1
        self.crossing = over
        # obstacles within 12 m: sample the car outline
        for so, ox, oy, oyaw, ha, hl, kind in self.obs:
            if (ox - x) ** 2 + (oy - y) ** 2 > 14.0 ** 2:
                continue
            dmin = 9e9
            for k in range(0, 21):
                for (cx, cy) in ((CAR_BACK + (CAR_FRONT - CAR_BACK) * k / 20, CAR_HALF),
                                 (CAR_BACK + (CAR_FRONT - CAR_BACK) * k / 20, -CAR_HALF),
                                 (CAR_FRONT, CAR_HALF * (k - 10) / 10), (CAR_BACK, CAR_HALF * (k - 10) / 10)):
                    wx = x + cx * math.cos(th) - cy * math.sin(th)
                    wy = y + cx * math.sin(th) + cy * math.cos(th)
                    dmin = min(dmin, rect_dist(wx, wy, ox, oy, oyaw, ha, hl))
            if dmin < self.min_clear[so]:
                self.min_clear[so] = dmin
            if dmin <= 0.02:
                self.collided.add((so, kind))
        self.rows.append((t, s, d, v, worst))

    def report(self):
        r = self.rows
        if not r:
            print("no data")
            return
        dur = r[-1][0] - r[0][0]
        right = sum(1 for x in r if x[2] < 0) / len(r)
        stops, cur = [], None
        for t, s, d, v, w in r:
            if v < 0.1:
                cur = cur or (t, s)
            elif cur:
                if t - cur[0] > 1.0:
                    stops.append((round(cur[1]), round(t - cur[0], 1)))
                cur = None
        if cur and r[-1][0] - cur[0] > 1.0:
            stops.append((round(cur[1]), round(r[-1][0] - cur[0], 1)))
        seen = {k: v for k, v in self.min_clear.items() if v < 9e8}
        worst5 = sorted(seen.items(), key=lambda kv: kv[1])[:5]
        print(f"duration {dur:.0f} s  progress s={r[0][1]:.0f} -> {r[-1][1]:.0f} m  ({r[-1][1] - r[0][1]:.0f} m)")
        print(f"speed mean {sum(x[3] for x in r) / len(r):.2f}  max {max(x[3] for x in r):.2f} m/s")
        print(f"lateral d from centreline: min {min(x[2] for x in r):.2f}  max {max(x[2] for x in r):.2f} m "
              f"(right lane centre -1.75, left +1.75, edge line +-{EDGE_D})")
        print(f"time in right half {100 * right:.0f}%  left half {100 * (1 - right):.0f}%")
        print(f"WHITE-LINE CROSSINGS (body over an edge line): {self.cross}   max body extent {max(x[4] for x in r):.2f} m")
        print(f"COLLISIONS: {len(self.collided)} {sorted(self.collided)}")
        print(f"obstacles passed: {len(seen)}; closest clearances (s, m): "
              f"{[(s, round(c, 2)) for s, c in worst5]}")
        print(f"stops > 1 s (s, seconds): {stops}")


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
    stop_s = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    node = rclpy.create_node("eval_lap", parameter_overrides=[Parameter("use_sim_time", value=True)])
    ev = Eval()
    t0 = time.time()
    node.create_subscription(Odometry, "/odom", lambda m: ev.on_odom(m, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 10)
    try:
        while time.time() - t0 < secs and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if stop_s and ev.rows and ev.rows[-1][1] >= stop_s:
                break
    except KeyboardInterrupt:
        pass
    ev.report()


if __name__ == "__main__":
    main()
