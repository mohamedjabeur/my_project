"""Publishes the front camera field of view (frustum + ground footprint) as RViz markers."""
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from perception.ground_projection import CameraModel


def frustum_corners(cam, width, height, depth):
    """4 far-plane corners (TL, TR, BR, BL) in the vehicle frame, `depth` m along the optical axis."""
    px = [(0, 0), (width, 0), (width, height), (0, height)]
    return [cam.point_from_depth(u, v, depth) for u, v in px]


def ground_footprint(cam, width, height, max_range, n=20):
    """Visible ground polygon (x, y) of the image, clipped at `max_range` m. Skips sky pixels."""
    us = np.linspace(0, width, n)
    v_top = max(cam.cy - cam.fy * np.tan(cam.pitch), 0.0) + 1.0   # just below the horizon
    ring = [(u, height) for u in us] + [(width, v) for v in np.linspace(height, v_top, n)] \
        + [(u, v_top) for u in us[::-1]] + [(0, v) for v in np.linspace(v_top, height, n)]
    pts = cam.pixel_to_ground(*np.array(ring).T)
    pts = pts[~np.isnan(pts).any(axis=1)]
    r = np.hypot(pts[:, 0] - cam.x_off, pts[:, 1])
    scale = np.minimum(1.0, max_range / np.maximum(r, 1e-6))
    pts[:, 0] = cam.x_off + (pts[:, 0] - cam.x_off) * scale
    pts[:, 1] *= scale
    return pts


def _pt(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


class CameraFovMarker(Node):
    def __init__(self):
        super().__init__("camera_fov_marker")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.frame_id = p("frame_id", "base_footprint")
        self.width, self.height = p("width", 800), p("height", 450)
        self.range = p("range", 12.0)
        self.cam = CameraModel(fx=p("fx", 335.65), fy=p("fy", 335.65), cx=p("cx", 400.0),
                               cy=p("cy", 225.0), height=p("mount_height", 0.732),
                               pitch=p("pitch", 0.1745), x_off=p("x_off", 0.18))
        self.pub = self.create_publisher(MarkerArray, "fov_marker", 1)
        self.create_timer(0.5, self.publish)

    def _marker(self, mid, kind, r, g, b, a, width=0.0):
        m = Marker()
        # stamp 0 = "latest transform": with a slow sim clock now() is ahead of the newest TF and RViz said
        # "No transform to fixed frame [odom]" for every marker
        m.header.frame_id = self.frame_id
        m.ns, m.id, m.type, m.action = "camera_fov", mid, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = width if width else 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        return m

    def publish(self):
        cam = self.cam
        apex = np.array([cam.x_off, 0.0, cam.height])
        far = frustum_corners(cam, self.width, self.height, self.range)

        edges = self._marker(0, Marker.LINE_LIST, 1.0, 0.8, 0.0, 1.0, 0.02)
        for c in far:
            edges.points += [_pt(apex), _pt(c)]
        for i in range(4):
            edges.points += [_pt(far[i]), _pt(far[(i + 1) % 4])]

        faces = self._marker(1, Marker.TRIANGLE_LIST, 1.0, 0.8, 0.0, 0.15)
        for i in range(4):
            faces.points += [_pt(apex), _pt(far[i]), _pt(far[(i + 1) % 4])]

        ground = self._marker(2, Marker.LINE_STRIP, 0.0, 1.0, 0.4, 1.0, 0.04)
        pts = ground_footprint(cam, self.width, self.height, self.range)
        if len(pts):
            ground.points = [_pt((x, y, 0.01)) for x, y in pts] + [_pt((pts[0][0], pts[0][1], 0.01))]

        self.pub.publish(MarkerArray(markers=[edges, faces, ground]))


def main():
    rclpy.init()
    node = CameraFovMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
