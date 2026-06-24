#!/usr/bin/env python3
"""Publish solid RViz markers for the static lab scene.

Gazebo renders SDF world visuals, but RViz does not subscribe to Gazebo world
visuals directly. These markers provide a complete visual reference in RViz.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class StaticLabSceneMarkers(Node):
    def __init__(self) -> None:
        super().__init__("static_lab_scene_markers")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(MarkerArray, "/lab_scene/markers", qos)
        self._timer = self.create_timer(1.0, self._publish)
        self._publish()

    def _base_marker(self, marker_id: int, name: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lab_scene"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=0).to_msg()
        marker.text = name
        return marker

    def _box(self, marker_id, name, xyz, size, rgba) -> Marker:
        marker = self._base_marker(marker_id, name, Marker.CUBE)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _cylinder(self, marker_id, name, xyz, radius, height, rgba) -> Marker:
        marker = self._base_marker(marker_id, name, Marker.CYLINDER)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = height
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _publish(self) -> None:
        markers = MarkerArray()
        add = markers.markers.append

        grey = (0.42, 0.46, 0.48, 1.0)
        leg = (0.34, 0.37, 0.39, 1.0)
        station = (0.46, 0.42, 0.36, 0.75)

        add(self._box(1, "lab_workbench_top", (0.75, 0.0, 0.78), (1.0, 0.7, 0.06), grey))
        for idx, xyz in enumerate(
            [(1.18, 0.28, 0.39), (1.18, -0.28, 0.39), (0.32, 0.28, 0.39), (0.32, -0.28, 0.39)],
            start=2,
        ):
            add(self._box(idx, f"lab_workbench_leg_{idx}", xyz, (0.05, 0.05, 0.78), leg))

        add(self._box(10, "sample_block", (0.58, 0.18, 0.84), (0.07, 0.07, 0.07), (0.0, 0.45, 0.85, 1.0)))
        add(self._cylinder(11, "target_pad", (0.58, -0.18, 0.815), 0.08, 0.01, (0.05, 0.7, 0.25, 1.0)))
        add(self._cylinder(12, "sample_vial_red", (0.62, 0.30, 0.865), 0.018, 0.10, (0.85, 0.12, 0.12, 1.0)))
        add(self._box(13, "sample_tray", (0.45, -0.30, 0.8225), (0.22, 0.16, 0.025), (0.88, 0.88, 0.90, 1.0)))
        add(self._box(14, "tool_caddy", (1.05, 0.25, 0.85), (0.18, 0.12, 0.08), (0.80, 0.55, 0.10, 1.0)))

        add(self._box(20, "station_b_top", (2.2, 0.0, 0.78), (0.8, 0.6, 0.06), station))
        for idx, xyz in enumerate(
            [(2.53, 0.23, 0.39), (2.53, -0.23, 0.39), (1.87, 0.23, 0.39), (1.87, -0.23, 0.39)],
            start=21,
        ):
            add(self._box(idx, f"station_b_leg_{idx}", xyz, (0.05, 0.05, 0.78), station))

        add(self._cylinder(30, "moving_obstacle", (0.58, 0.7, 1.05), 0.06, 0.5, (0.95, 0.75, 0.20, 0.45)))
        self._pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = StaticLabSceneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
