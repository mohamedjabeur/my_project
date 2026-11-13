"""Waypoint gate: drive the lane up to a clicked point, then stop there (no ROS, unit-tested).

The lane controller keeps the car on the road and around obstacles; this module only limits the SPEED so that
the car comes to rest at the goal. Goal and car pose are in the same frame (odom). The remaining distance is the
straight line to the goal: on the last metres, where braking happens, it is close to the distance along the lane.
Far from the goal (`near_dist`) the straight line says nothing about the lane (the track is a loop: a goal 300 m ahead
along the lane can lie "behind" the heading), so there the gate neither rejects nor stops the car: it only drives on.
"""
import math


def brake_cap(distance, margin, decel):
    """Highest speed from which the car can still stop `margin` m before the goal."""
    return math.sqrt(2.0 * decel * max(distance - margin, 0.0))


class GoalGate:
    def __init__(self, decel=3.0, stop_margin=0.3, reach_dist=3.0, reach_speed=0.2, min_goal_dist=1.0, latency=0.5,
                 near_dist=40.0, pass_margin=2.0):
        self.decel, self.stop_margin = decel, stop_margin
        self.reach_dist, self.reach_speed, self.min_goal_dist = reach_dist, reach_speed, min_goal_dist
        self.latency = latency    # s between command and wheels (measured ~0.3 s + motor lag): brake that much earlier
        self.near_dist = near_dist  # m: inside it the straight line to the goal is reliable (braking, "behind" tests)
        self.pass_margin = pass_margin  # m: distance grew this much after the closest approach -> drove past the goal
        self.goal = None          # (x, y) or None
        self.reached = False
        self.best = math.inf      # closest distance seen inside near_dist

    def set_goal(self, x, y, car_x, car_y, car_yaw):
        """Accept a goal in front of the car (the car cannot reverse). Returns True if accepted.

        A goal farther than `near_dist` is accepted whatever its direction: the car follows the lane (and the loop)
        to it. Only a near goal behind the car is refused."""
        dx, dy = x - car_x, y - car_y
        dist = math.hypot(dx, dy)
        behind = dx * math.cos(car_yaw) + dy * math.sin(car_yaw) <= 0.0
        if dist < self.min_goal_dist or (behind and dist < self.near_dist):
            return False
        self.goal, self.reached, self.best = (x, y), False, math.inf
        return True

    def clear(self):
        self.goal, self.reached, self.best = None, False, math.inf

    def remaining(self, car_x, car_y):
        """Distance to the goal (inf without goal)."""
        if self.goal is None:
            return math.inf
        return math.hypot(self.goal[0] - car_x, self.goal[1] - car_y)

    def speed_cap(self, car_x, car_y, car_yaw, v):
        """Speed limit [m/s] for this control step (inf = no goal, drive normally). Latches `reached`."""
        if self.goal is None:
            return math.inf
        if self.reached:
            return 0.0
        dx, dy = self.goal[0] - car_x, self.goal[1] - car_y
        dist = math.hypot(dx, dy)
        if dist >= self.near_dist:
            return math.inf                      # far: the lane decides the way, nothing to brake for yet
        passed = dx * math.cos(car_yaw) + dy * math.sin(car_yaw) < 0.0
        self.best = min(self.best, dist)
        if passed or dist > self.best + self.pass_margin or (dist < self.reach_dist and v < self.reach_speed):
            self.reached = True                  # stopped near the goal, or drove past it (also an off-road goal): hold here
            return 0.0
        return brake_cap(dist - v * self.latency, self.stop_margin, self.decel)
