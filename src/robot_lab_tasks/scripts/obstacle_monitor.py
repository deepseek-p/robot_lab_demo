#!/usr/bin/env python3
"""Dynamic obstacle monitor: Gazebo pose -> MoveIt planning scene.

Subscribes to the obstacle's dedicated pose topic (published by the
gz-sim PosePublisher plugin and bridged to PoseStamped), and applies its
pose to the MoveIt planning scene as a collision cylinder whenever it
moves. MoveIt then plans around the obstacle's current position, and
in-flight trajectories are aborted by move_group's scene validation when
the obstacle invades the path — the pick executor retries with a fresh
plan, which together forms the dynamic-avoidance behavior of this demo.

The pose-update -> scene-publish latency is measured per update and
published on ``/task/obstacle_latency`` (milliseconds); optionally appended
to a CSV for the competition evidence package (planning-response <= 200 ms).
"""

import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64

OBSTACLE = "moving_obstacle"
CYLINDER_HEIGHT = 0.5
CYLINDER_RADIUS = 0.06


class ObstacleMonitor(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_monitor")
        self.declare_parameter("pose_topic", "/model/moving_obstacle/pose")
        self.declare_parameter("min_move", 0.005)
        self.declare_parameter("latency_csv", "")

        self._min_move = float(self.get_parameter("min_move").value)
        self._last_pose: Pose | None = None
        self._csv_path = str(self.get_parameter("latency_csv").value)
        self._csv_rows: list = []
        self._update_count = 0

        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self._latency_pub = self.create_publisher(Float64, "/task/obstacle_latency", 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            10,
        )
        self.get_logger().info("obstacle_monitor up; tracking '%s'" % OBSTACLE)

    def _on_pose(self, msg: PoseStamped) -> None:
        received = time.monotonic()
        if not self._moved(msg.pose):
            return
        self._apply(msg.pose)
        latency_ms = (time.monotonic() - received) * 1000.0
        self._latency_pub.publish(Float64(data=latency_ms))
        self._record(latency_ms)
        self._update_count += 1
        if self._update_count == 1:
            self.get_logger().info("first obstacle scene update applied")

    def _moved(self, pose: Pose) -> bool:
        if self._last_pose is None:
            return True
        dx = pose.position.x - self._last_pose.position.x
        dy = pose.position.y - self._last_pose.position.y
        dz = pose.position.z - self._last_pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) > self._min_move

    def _apply(self, pose: Pose) -> None:
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = OBSTACLE
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [CYLINDER_HEIGHT, CYLINDER_RADIUS]
        obj.primitives = [cylinder]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._scene_pub.publish(scene)
        self._last_pose = pose

    def _record(self, latency_ms: float) -> None:
        if not self._csv_path:
            return
        self._csv_rows.append(
            {"stamp": time.time(), "latency_ms": round(latency_ms, 3)}
        )
        if len(self._csv_rows) % 20 == 0:
            self._flush()

    def _flush(self) -> None:
        if not self._csv_rows:
            return
        path = Path(self._csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=["stamp", "latency_ms"])
            writer.writeheader()
            writer.writerows(self._csv_rows)


def main() -> None:
    rclpy.init()
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._flush()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
