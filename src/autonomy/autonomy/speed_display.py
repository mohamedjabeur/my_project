"""Speed display for RViz: big text above the car, "12.3 km/h", coloured by state. /odom -> /speed_marker."""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker

KMH = 3.6


def speed_text(v, v_slow_kmh=11.0, v_stop_kmh=0.5):
    """(text, (r, g, b)) for a forward speed v [m/s]: red = stopped, orange = slowed (<= ~10 km/h band), green = cruising."""
    kmh = abs(v) * KMH
    if kmh < v_stop_kmh:
        return "0.0 km/h  STOP", (1.0, 0.2, 0.2)
    if kmh <= v_slow_kmh:
        return "%.1f km/h  SLOW" % kmh, (1.0, 0.65, 0.0)
    return "%.1f km/h" % kmh, (0.2, 0.9, 0.3)


class SpeedDisplay(Node):
    def __init__(self):
        super().__init__("speed_display")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.frame, self.height, self.size = p("frame_id", "base_footprint"), p("height", 4.0), p("text_size", 5.0)
        self.pub = self.create_publisher(Marker, "/speed_marker", 1)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

    def on_odom(self, msg):
        text, (r, g, b) = speed_text(msg.twist.twist.linear.x)
        m = Marker()
        m.header.frame_id, m.header.stamp = self.frame, msg.header.stamp
        m.ns, m.id, m.type, m.action = "speed", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.z, m.pose.orientation.w = self.height, 1.0
        m.scale.z = self.size
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.text = text
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(SpeedDisplay())


if __name__ == "__main__":
    main()
