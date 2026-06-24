#!/usr/bin/env python3
"""Publish lab scene objects as MoveIt planning scene collision objects.

Static support surfaces come from the same dimensions as the RViz LabScene
markers. Manipulable objects are updated from /perception/detections when
perception is running.
"""

import json
import math
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


@dataclass(frozen=True)
class SceneObjectSpec:
    name: str
    role: str
    source: str
    shape: str
    xyz: tuple[float, float, float]
    dimensions: tuple[float, ...]
    color_hint: str = ""

    @property
    def collision_id(self) -> str:
        return f"{self.role}__{self.name}"


STATIC_OBJECTS = [
    SceneObjectSpec(
        "lab_workbench_top",
        "support_surface",
        "lab_scene",
        "box",
        (0.75, 0.0, 0.78),
        (1.0, 0.7, 0.06),
        "grey",
    ),
    SceneObjectSpec(
        "lab_workbench_leg_front_left",
        "environment_obstacle",
        "lab_scene",
        "box",
        (1.18, 0.28, 0.39),
        (0.05, 0.05, 0.78),
        "grey",
    ),
    SceneObjectSpec(
        "lab_workbench_leg_front_right",
        "environment_obstacle",
        "lab_scene",
        "box",
        (1.18, -0.28, 0.39),
        (0.05, 0.05, 0.78),
        "grey",
    ),
    SceneObjectSpec(
        "lab_workbench_leg_rear_left",
        "environment_obstacle",
        "lab_scene",
        "box",
        (0.32, 0.28, 0.39),
        (0.05, 0.05, 0.78),
        "grey",
    ),
    SceneObjectSpec(
        "lab_workbench_leg_rear_right",
        "environment_obstacle",
        "lab_scene",
        "box",
        (0.32, -0.28, 0.39),
        (0.05, 0.05, 0.78),
        "grey",
    ),
    SceneObjectSpec(
        "target_pad",
        "support_surface",
        "lab_scene",
        "cylinder",
        (0.58, -0.18, 0.815),
        (0.01, 0.08),
        "green",
    ),
    SceneObjectSpec(
        "sample_tray",
        "support_surface",
        "lab_scene",
        "box",
        (0.45, -0.30, 0.8225),
        (0.22, 0.16, 0.025),
        "white",
    ),
    SceneObjectSpec(
        "tool_caddy",
        "environment_obstacle",
        "lab_scene",
        "box",
        (1.05, 0.25, 0.85),
        (0.18, 0.12, 0.08),
        "yellow",
    ),
    SceneObjectSpec(
        "station_b_top",
        "support_surface",
        "lab_scene",
        "box",
        (2.2, 0.0, 0.78),
        (0.8, 0.6, 0.06),
        "brown",
    ),
    SceneObjectSpec(
        "station_b_leg_front_left",
        "environment_obstacle",
        "lab_scene",
        "box",
        (2.53, 0.23, 0.39),
        (0.05, 0.05, 0.78),
        "brown",
    ),
    SceneObjectSpec(
        "station_b_leg_front_right",
        "environment_obstacle",
        "lab_scene",
        "box",
        (2.53, -0.23, 0.39),
        (0.05, 0.05, 0.78),
        "brown",
    ),
    SceneObjectSpec(
        "station_b_leg_rear_left",
        "environment_obstacle",
        "lab_scene",
        "box",
        (1.87, 0.23, 0.39),
        (0.05, 0.05, 0.78),
        "brown",
    ),
    SceneObjectSpec(
        "station_b_leg_rear_right",
        "environment_obstacle",
        "lab_scene",
        "box",
        (1.87, -0.23, 0.39),
        (0.05, 0.05, 0.78),
        "brown",
    ),
]

KNOWN_DETECTED_SHAPES = {
    "sample_block": ("box", (0.07, 0.07, 0.07)),
    "sample_vial_red": ("cylinder", (0.10, 0.018)),
}


class PlanningSceneObjectPublisher(Node):
    def __init__(self) -> None:
        super().__init__("planning_scene_object_publisher")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("publish_static_scene", True)
        self.declare_parameter("use_known_detected_shapes", True)
        self.declare_parameter("detection_timeout", 2.0)
        self.declare_parameter("publish_rate", 2.0)

        self._publish_static = bool(self.get_parameter("publish_static_scene").value)
        self._use_known_shapes = bool(
            self.get_parameter("use_known_detected_shapes").value
        )
        self._detection_timeout = float(self.get_parameter("detection_timeout").value)
        self._detections: dict[str, dict[str, Any]] = {}
        self._last_publish = 0.0

        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self._roles_pub = self.create_publisher(
            String, "/planning_scene/object_roles", 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            10,
        )

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish_scene)
        self.get_logger().info(
            "planning_scene_object_publisher up; static=%s detections=%s"
            % (self._publish_static, self.get_parameter("detections_topic").value)
        )

    def _on_detections(self, msg: String) -> None:
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError as ex:
            self.get_logger().warn(f"could not parse detections JSON: {ex}")
            return
        if not isinstance(detections, list):
            self.get_logger().warn("detections message is not a JSON list")
            return

        now = time.monotonic()
        for det in detections:
            if not isinstance(det, dict):
                continue
            name = str(det.get("name", ""))
            position = det.get("position")
            if not name or not isinstance(position, list) or len(position) != 3:
                continue
            self._detections[name] = {"stamp": now, "data": det}

    def _make_pose(
        self,
        xyz: tuple[float, float, float] | list[float],
        yaw: float = 0.0,
    ) -> Pose:
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _primitive(self, shape: str, dimensions: tuple[float, ...]) -> SolidPrimitive:
        primitive = SolidPrimitive()
        if shape == "box":
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(dimensions)
        elif shape == "cylinder":
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = list(dimensions)
        else:
            raise ValueError(f"unsupported shape: {shape}")
        return primitive

    def _collision_object(
        self,
        object_id: str,
        shape: str,
        dimensions: tuple[float, ...],
        pose: Pose,
    ) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = object_id
        obj.primitives = [self._primitive(shape, dimensions)]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _static_collision_objects(self) -> tuple[list[CollisionObject], list[dict]]:
        objects: list[CollisionObject] = []
        roles: list[dict] = []
        if not self._publish_static:
            return objects, roles

        for spec in STATIC_OBJECTS:
            objects.append(
                self._collision_object(
                    spec.collision_id,
                    spec.shape,
                    spec.dimensions,
                    self._make_pose(spec.xyz),
                )
            )
            roles.append(
                {
                    "id": spec.collision_id,
                    "name": spec.name,
                    "role": spec.role,
                    "source": spec.source,
                    "shape": spec.shape,
                    "dimensions": list(spec.dimensions),
                    "support_surface": spec.role == "support_surface",
                    "detected": False,
                    "color_hint": spec.color_hint,
                }
            )
        return objects, roles

    def _detected_collision_objects(self) -> tuple[list[CollisionObject], list[dict]]:
        objects: list[CollisionObject] = []
        roles: list[dict] = []
        now = time.monotonic()
        stale_names = [
            name
            for name, wrapped in self._detections.items()
            if now - float(wrapped["stamp"]) > self._detection_timeout
        ]
        for name in stale_names:
            del self._detections[name]

        for name, wrapped in self._detections.items():
            det = wrapped["data"]
            position = det["position"]
            yaw = float(det.get("yaw", 0.0))
            object_id = f"detected_object__{name}"

            if self._use_known_shapes and name in KNOWN_DETECTED_SHAPES:
                shape, dimensions = KNOWN_DETECTED_SHAPES[name]
            else:
                shape = "box"
                size = det.get("size", [0.04, 0.04, 0.04])
                dimensions = tuple(max(float(value), 0.02) for value in size[:3])

            objects.append(
                self._collision_object(
                    object_id,
                    shape,
                    tuple(dimensions),
                    self._make_pose(position, yaw),
                )
            )
            roles.append(
                {
                    "id": object_id,
                    "name": name,
                    "role": "detected_object",
                    "source": "perception",
                    "shape": shape,
                    "dimensions": list(dimensions),
                    "support_surface": False,
                    "detected": True,
                    "points": int(det.get("points", 0)),
                    "last_seen_age_s": round(now - float(wrapped["stamp"]), 3),
                }
            )
        return objects, roles

    def _publish_scene(self) -> None:
        static_objects, static_roles = self._static_collision_objects()
        detected_objects, detected_roles = self._detected_collision_objects()
        collision_objects = static_objects + detected_objects
        roles = static_roles + detected_roles
        if not collision_objects:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        self._scene_pub.publish(scene)

        payload = {
            "frame_id": "world",
            "stamp": time.time(),
            "objects": roles,
            "support_surfaces": [
                item["id"] for item in roles if item["role"] == "support_surface"
            ],
            "detected_objects": [
                item["id"] for item in roles if item["role"] == "detected_object"
            ],
        }
        self._roles_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

        if self._last_publish == 0.0:
            self.get_logger().info(
                "published %d planning scene objects (%d support surfaces, %d detected)"
                % (
                    len(collision_objects),
                    len(payload["support_surfaces"]),
                    len(payload["detected_objects"]),
                )
            )
        self._last_publish = time.monotonic()


def main() -> None:
    rclpy.init()
    node = PlanningSceneObjectPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
