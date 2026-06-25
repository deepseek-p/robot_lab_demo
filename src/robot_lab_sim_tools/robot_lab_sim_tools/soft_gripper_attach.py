#!/usr/bin/env python3
import math
from dataclasses import dataclass

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from robot_lab_sim_tools.gazebo_state import GazeboClassicStateClient


DEFAULT_OBJECTS = ["sample_block", "sample_vial_red"]
DEFAULT_TOPIC_CONTRACTS = [
    "/gripper/attach/sample_block",
    "/gripper/detach/sample_block",
    "/gripper/attach/sample_vial_red",
    "/gripper/detach/sample_vial_red",
]


@dataclass
class Attachment:
    offset_tcp: tuple[float, float, float]
    relative_orientation: Quaternion


def quat_tuple(q: Quaternion) -> tuple[float, float, float, float]:
    return (q.x, q.y, q.z, q.w)


def quat_msg(values: tuple[float, float, float, float]) -> Quaternion:
    q = Quaternion()
    q.x, q.y, q.z, q.w = values
    return q


def quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply_raw(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return quat_normalize(quat_multiply_raw(a, b))


def rotate_vector(
    q: tuple[float, float, float, float],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    vx, vy, vz = v
    rotated = quat_multiply_raw(
        quat_multiply_raw(quat_normalize(q), (vx, vy, vz, 0.0)),
        quat_conjugate(quat_normalize(q)),
    )
    return (rotated[0], rotated[1], rotated[2])


class SoftGripperAttach(Node):
    def __init__(self) -> None:
        super().__init__("soft_gripper_attach")
        self.declare_parameter("objects", DEFAULT_OBJECTS)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("tcp_frame", "gripper_tcp")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("update_rate", 30.0)

        self._objects = list(self.get_parameter("objects").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._tcp_frame = str(self.get_parameter("tcp_frame").value)
        self._model_poses: dict[str, Pose] = {}
        self._attached: dict[str, Attachment] = {}
        self._pending = {}
        self._owned_subscriptions = []

        # GazeboClassicStateClient sends gazebo_msgs/srv/SetEntityState requests.
        self._state = GazeboClassicStateClient(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._owned_subscriptions.append(
            self.create_subscription(
                ModelStates,
                str(self.get_parameter("model_states_topic").value),
                self._on_model_states,
                10,
            )
        )

        for name in self._objects:
            self._owned_subscriptions.append(
                self.create_subscription(
                    Empty,
                    f"/gripper/attach/{name}",
                    lambda _msg, object_name=name: self._attach(object_name),
                    10,
                )
            )
            self._owned_subscriptions.append(
                self.create_subscription(
                    Empty,
                    f"/gripper/detach/{name}",
                    lambda _msg, object_name=name: self._detach(object_name),
                    10,
                )
            )

        rate = float(self.get_parameter("update_rate").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            "soft_gripper_attach up; objects=%s tcp_frame=%s"
            % (",".join(self._objects), self._tcp_frame)
        )

    def _on_model_states(self, msg: ModelStates) -> None:
        for index, name in enumerate(msg.name):
            if name in self._objects:
                self._model_poses[name] = msg.pose[index]

    def _tcp_pose(self) -> Pose | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame,
                self._tcp_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().debug(f"TCP transform unavailable: {ex}")
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _attach(self, object_name: str) -> None:
        tcp = self._tcp_pose()
        obj = self._model_poses.get(object_name)
        if tcp is None or obj is None:
            self.get_logger().warn(
                "cannot attach %s; missing TCP transform or model pose" % object_name
            )
            return

        tcp_q = quat_tuple(tcp.orientation)
        obj_q = quat_tuple(obj.orientation)
        offset_world = (
            obj.position.x - tcp.position.x,
            obj.position.y - tcp.position.y,
            obj.position.z - tcp.position.z,
        )
        offset_tcp = rotate_vector(quat_conjugate(tcp_q), offset_world)
        relative_q = quat_msg(quat_multiply(quat_conjugate(tcp_q), obj_q))
        self._attached[object_name] = Attachment(offset_tcp, relative_q)
        self.get_logger().info("attached %s to %s" % (object_name, self._tcp_frame))

    def _detach(self, object_name: str) -> None:
        if object_name in self._attached:
            del self._attached[object_name]
            self.get_logger().info("detached %s" % object_name)

    def _tick(self) -> None:
        if not self._attached:
            return

        tcp = self._tcp_pose()
        if tcp is None:
            return

        tcp_q = quat_tuple(tcp.orientation)

        for object_name, attachment in list(self._attached.items()):
            pending = self._pending.get(object_name)
            if pending is not None and not pending.done():
                continue

            offset_world = rotate_vector(tcp_q, attachment.offset_tcp)
            pose = Pose()
            pose.position.x = tcp.position.x + offset_world[0]
            pose.position.y = tcp.position.y + offset_world[1]
            pose.position.z = tcp.position.z + offset_world[2]
            pose.orientation = quat_msg(
                quat_multiply(tcp_q, quat_tuple(attachment.relative_orientation))
            )

            self._pending[object_name] = self._state.set_pose_async(
                object_name,
                pose,
                reference_frame=self._world_frame,
            )


def main() -> None:
    rclpy.init()
    node = SoftGripperAttach()
    try:
        if not node._state.wait(timeout_sec=10.0):
            node.get_logger().error(
                "Gazebo Classic state services unavailable; "
                "expected /gazebo/set_entity_state and /gazebo/get_entity_state"
            )
            raise SystemExit(1)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as ex:
        if rclpy.ok():
            node.get_logger().error(f"soft_gripper_attach stopped unexpectedly: {ex}")
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
