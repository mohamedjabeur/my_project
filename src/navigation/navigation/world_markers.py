"""Draw the Gazebo world (SDF visuals: meshes, boxes, spheres) in RViz as a latched MarkerArray on /world_markers.

World frame = odom frame in simulation, so the markers line up with the car, the scan and the costmaps.
  ros2 run navigation world_markers --ros-args -p sdf:=/path/to/world.sdf
"""
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def floats(el, path, n):
    """Numbers of the text of el/path (missing element -> zeros). Note: a leaf Element is falsy, so no `or`."""
    node = el.find(path)
    v = [float(t) for t in (node.text if node is not None and node.text else "").split()] + [0.0] * n
    return v[:n]


def quat(yaw_roll_pitch):
    r, p, y = yaw_roll_pitch
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class WorldMarkers(Node):
    def __init__(self):
        super().__init__("world_markers")
        sdf = self.declare_parameter("sdf", "").value
        self.frame = self.declare_parameter("frame", "odom").value
        self.root = os.path.dirname(os.path.dirname(os.path.abspath(sdf)))       # model://<name>/... lives next to it
        self.pub = self.create_publisher(MarkerArray, "/world_markers",
                                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.msg = self.build(sdf)
        self.pub.publish(self.msg)
        self.create_timer(5.0, lambda: self.pub.publish(self.msg))               # for RViz restarts
        self.get_logger().info("%d world markers from %s" % (len(self.msg.markers), sdf))

    def build(self, sdf):
        out = MarkerArray()
        for model in ET.parse(sdf).getroot().iter("model"):
            mx, my, mz, mr, mp, myaw = floats(model, "pose", 6)
            for vis in model.iter("visual"):
                geom = vis.find("geometry")
                if geom is None:
                    continue
                m = Marker()
                m.header.frame_id, m.ns, m.id = self.frame, vis.get("name"), len(out.markers)
                m.action = Marker.ADD
                x, y, z, r, p, yaw = floats(vis, "pose", 6)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x + mx, y + my, z + mz
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat((r, p, yaw + myaw))
                col = floats(vis, "material/diffuse", 4)
                m.color.r, m.color.g, m.color.b, m.color.a = col[0], col[1], col[2], 1.0
                if geom.find("mesh") is not None:
                    uri = geom.find("mesh/uri").text.replace("model://", "")
                    # the generated sdf already holds absolute file:// URIs; model:// ones are relative to root
                    m.type = Marker.MESH_RESOURCE
                    m.mesh_resource = uri if uri.startswith("file://") else "file://" + os.path.join(self.root, uri)
                    m.mesh_use_embedded_materials = False
                    m.scale.x = m.scale.y = m.scale.z = 1.0
                elif geom.find("box") is not None:
                    m.type = Marker.CUBE
                    m.scale.x, m.scale.y, m.scale.z = floats(geom, "box/size", 3)
                elif geom.find("sphere") is not None:
                    m.type = Marker.SPHERE
                    m.scale.x = m.scale.y = m.scale.z = 2.0 * float(geom.find("sphere/radius").text)
                else:
                    continue
                out.markers.append(m)
        return out


def main():
    rclpy.init()
    node = WorldMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
