"""Lane-change planner around obstacles (no ROS, unit-tested).

Everything is expressed in the *lane frame* of the camera: lateral position 0 = centre of the RIGHT lane,
+ = left (the left lane centre is `shift` m further left). The car's own position is the Kalman offset `e`.

Each cycle `LaneChanger.update` decides which lane to drive in:
  * the planned path leaves the car's position towards the target lane at a fixed slope (m lateral per m forward);
  * an obstacle blocks that path when it is closer to the path than half the car + margin + its own half width;
  * target lane blocked and the other lane free over `look` m  -> change lane;
  * (optional, prefer_right) in the left lane, right lane free again -> go back right;
  * both blocked -> stay, and the controller brakes in front of the obstacle (never drives into it).
It also tells the controller to slow down while an obstacle is on the road ahead or a change is under way,
and gives the lateral reference (slew-limited, so the car moves smoothly) with its heading feed-forward.
"""
import math
from dataclasses import dataclass


@dataclass
class Obstacle:
    x: float        # m ahead of base_footprint of its nearest face
    y: float        # m, lateral centre in the base frame (+ left)
    half_w: float   # m, half of its lateral size


def lateral_in_lane(obs, e, psi, kappa):
    """Lateral position of the obstacle relative to the right-lane centre line (+ = left)."""
    x = max(obs.x, 0.0)
    y_lane = -e - psi * x + 0.5 * kappa * x * x       # centre line of the right lane seen from the car
    return obs.y - y_lane


class LaneChanger:
    def __init__(self, shift=3.4, slope=0.3, look=14.0, slow_dist=15.0, veh_half=0.385, margin=0.45,
                 front_x=1.445, dwell=5.0, rear_len=1.3, prefer_right=False):
        self.shift, self.slope, self.look, self.slow_dist = shift, slope, look, slow_dist
        self.clear_w = veh_half + margin
        self.front_x = front_x
        self.dwell = dwell
        self.prefer_right = prefer_right  # False: stay in the lane until an obstacle forces a change (fewer manoeuvres,
        #                                   and the lidar only sees ~8 m, so a needless return can meet an unseen obstacle)
        self.rear_len = rear_len          # base_footprint -> rear bumper: the tail still has to clear the obstacle
        self.target = 0                  # 0 = right lane, 1 = left lane
        self.ref = 0.0                   # slew-limited lateral reference (lane frame)
        self.odo = 0.0
        self.last_switch = -1e9

    def lane_offset(self, lane):
        return self.shift * lane

    def _clear(self, lane, e, obs_lane, ahead_only=False):
        """Distance (from the bumper) to the first obstacle on the path from the car to `lane` (inf if free).

        ahead_only: ignore obstacles whose front face is already behind the bumper. They still matter for the
        lane decision (tail clearance) but braking cannot help any more, and stopping beside an obstacle with
        a standing car makes any evasive move impossible (deadlock seen in sim)."""
        goal = self.lane_offset(lane)
        best = math.inf
        for ob, o in obs_lane:
            if ahead_only and ob.x < self.front_x - 0.05:
                continue
            # the car is as long as the obstacle stays beside it: check the path where the bumper reaches the
            # obstacle AND where the tail leaves it (matters when the path comes back towards the obstacle)
            # base_footprint has travelled x - front_x when the bumper reaches the obstacle, and x + rear_len + depth
            # when the tail has left it (depth ~ 0.5 m, unknown)
            for s in (max(ob.x - self.front_x, 0.0), max(ob.x + self.rear_len + 0.5, 0.0)):
                path = e + max(-self.slope * s, min(self.slope * s, goal - e))
                if abs(o - path) < self.clear_w + ob.half_w:
                    best = min(best, max(ob.x - self.front_x, 0.0))
                    break
        return best

    def update(self, dt, v, e, psi, kappa, obstacles):
        """Returns dict(target, ref, heading_ref, clear, slow)."""
        self.odo += max(v, 0.0) * dt
        obs_lane = [(ob, lateral_in_lane(ob, e, psi, kappa)) for ob in obstacles]

        if self._clear(self.target, e, obs_lane) < self.look:
            alt = 1 - self.target
            if self._clear(alt, e, obs_lane) >= self.look:
                self.target, self.last_switch = alt, self.odo
        elif self.prefer_right and self.target == 1 and self.odo - self.last_switch >= self.dwell \
                and self._clear(0, e, obs_lane) >= self.look:
            self.target, self.last_switch = 0, self.odo

        goal = self.lane_offset(self.target)
        step = self.slope * max(v, 0.0) * dt
        before = self.ref
        self.ref += max(-step, min(step, goal - self.ref))
        moving = abs(self.ref - before) > 1e-6
        heading_ref = math.atan(self.slope) * (1.0 if goal > self.ref else -1.0) if moving else 0.0

        on_road = any(-1.8 < o < self.shift + 1.8 and -1.0 < ob.x < self.slow_dist for ob, o in obs_lane)
        slow = on_road or abs(goal - self.ref) > 0.05 or abs(goal - e) > 0.8
        return {"target": self.target, "ref": self.ref, "heading_ref": heading_ref,
                "clear": self._clear(self.target, e, obs_lane, ahead_only=True), "slow": slow}
