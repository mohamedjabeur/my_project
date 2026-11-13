"""Ackermann geometry and the ESP32 serial protocol (no ROS, unit-tested)."""
import math

WHEELBASE = 1.704          # m   (enimia.urdf)
TRACK = 0.567              # m   front track
REAR_WHEEL_RADIUS = 0.18   # m
MAX_STEER = 0.21           # rad (front_*_steer_joint limit)


def steering_from_twist(v, w, wheelbase=WHEELBASE, max_steer=MAX_STEER):
    """Bicycle-model steering angle [rad] for forward speed v [m/s] and yaw rate w [rad/s]."""
    if abs(v) < 1e-3 or abs(w) < 1e-6:
        return 0.0
    return max(-max_steer, min(max_steer, math.atan(wheelbase * w / v)))


def wheel_steering(delta, wheelbase=WHEELBASE, track=TRACK):
    """(left, right) front wheel angles for a bicycle-model steering angle `delta`."""
    if abs(delta) < 1e-6:
        return 0.0, 0.0
    radius = wheelbase / math.tan(delta)
    return math.atan(wheelbase / (radius - track / 2.0)), math.atan(wheelbase / (radius + track / 2.0))


def rate_limit(target, current, max_step):
    return current + max(-max_step, min(max_step, target - current))


# ---- serial protocol, one ASCII line each way ('\n' terminated) ----
#  PC -> ESP32 :  C <speed_mps> <steer_rad>          drive command (sent at 50 Hz, ESP32 stops after 500 ms of silence)
#                 E                                   emergency stop
#  ESP32 -> PC :  S <ms> <rear_ticks> <steer_rad>     status at 50 Hz: time, hall/encoder ticks (signed, cumulative), steering
def format_command(speed, steer):
    return "C %.3f %.4f\n" % (speed, steer)


def parse_status(line):
    """'S <ms> <ticks> <steer>' -> (ms, ticks, steer) or None if the line is not a valid status."""
    parts = line.strip().split()
    if len(parts) != 4 or parts[0] != "S":
        return None
    try:
        return int(parts[1]), int(parts[2]), float(parts[3])
    except ValueError:
        return None
