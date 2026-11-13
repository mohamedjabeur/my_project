"""Lane-keeping command from the fused state (no ROS, unit-tested).

Steering = Stanley law (+ = left, same as the Ackermann steering angle), speed = min of several limits:
cruise speed, steering-dependent speed, and a braking profile towards the nearest stop point
(lidar obstacle in our lane, stop sign, stop line). A stop sign / line is obeyed once: the car stops,
waits `stop_hold` s, then ignores signs for `stop_ignore` s so it can leave the line.
"""
import math

MAX_STEER = 0.21  # rad (enimia.urdf)


def stanley_steer(offset, heading, v, k=1.0, k_soft=0.5, max_steer=MAX_STEER, lane_ref=0.0, heading_ref=0.0):
    """offset: + = left of lane centre, heading: + = car points left of the lane. Returns steer [rad].

    lane_ref / heading_ref: wanted lateral position and heading (lane change in progress), 0 = stay on the centre."""
    delta = -(heading - heading_ref) - math.atan2(k * (offset - lane_ref), v + k_soft)
    return max(-max_steer, min(max_steer, delta))


def brake_speed(distance, margin, decel, v_max):
    """Highest speed from which the car can still stop `margin` m before an obstacle `distance` m ahead."""
    if not math.isfinite(distance):
        return v_max
    return min(v_max, math.sqrt(2.0 * decel * max(distance - margin, 0.0)))


class LaneController:
    def __init__(self, v_cruise=2.0, v_curve=0.8, k=1.0, k_soft=0.5, decel=1.0, obstacle_margin=3.0,
                 stop_margin=0.5, stop_hold=3.0, stop_ignore=6.0, stop_trigger=1.5, v_avoid=3.5):
        self.v_cruise, self.v_curve, self.v_avoid = v_cruise, v_curve, v_avoid
        self.k, self.k_soft, self.decel = k, k_soft, decel
        self.obstacle_margin, self.stop_margin = obstacle_margin, stop_margin
        self.stop_hold, self.stop_ignore, self.stop_trigger = stop_hold, stop_ignore, stop_trigger
        self.hold_until = -1.0
        self.ignore_until = -1.0

    def command(self, now, offset, heading, v, lane_valid, obstacle_dist, stop_dist,
                lane_ref=0.0, heading_ref=0.0, slow=False):
        """Returns (speed, steer). Distances are m ahead (inf / nan = nothing seen).

        obstacle_dist = clear distance ALONG THE PLANNED PATH (the lane change already steers around obstacles);
        slow = an obstacle is on the road ahead or a lane change is under way: drive at v_avoid, speed up after."""
        steer = stanley_steer(offset, heading, v, self.k, self.k_soft, lane_ref=lane_ref, heading_ref=heading_ref)
        # slow down in corners / when far from the centre line
        cornering = min(1.0, abs(steer) / MAX_STEER)
        v_cmd = self.v_cruise + (self.v_curve - self.v_cruise) * cornering
        if slow:
            v_cmd = min(v_cmd, self.v_avoid)
        if not lane_valid:
            v_cmd = min(v_cmd, self.v_curve)                       # camera lost the lane: creep on the filter
        v_cmd = min(v_cmd, brake_speed(_clean(obstacle_dist), self.obstacle_margin, self.decel, v_cmd))

        stop_dist = _clean(stop_dist)
        if now < self.hold_until:
            v_cmd = 0.0
        elif now >= self.ignore_until and stop_dist < math.inf:
            v_cmd = min(v_cmd, brake_speed(stop_dist, self.stop_margin, self.decel, v_cmd))
            if stop_dist < self.stop_trigger and v < 0.15:         # stopped at the line: wait, then go
                self.hold_until = now + self.stop_hold
                self.ignore_until = self.hold_until + self.stop_ignore
                v_cmd = 0.0
        return v_cmd, steer


def _clean(d):
    return d if d is not None and math.isfinite(d) else math.inf
