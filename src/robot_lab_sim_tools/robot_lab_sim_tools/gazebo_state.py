import math
from typing import Optional

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose
from rclpy.node import Node


def make_pose(x: float, y: float, z: float, yaw: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


class GazeboClassicStateClient:
    def __init__(
        self,
        node: Node,
        set_service: str = "/gazebo/set_entity_state",
        get_service: str = "/gazebo/get_entity_state",
    ) -> None:
        self._node = node
        self._set_client = node.create_client(SetEntityState, set_service)
        self._get_client = node.create_client(GetEntityState, get_service)

    def wait(self, timeout_sec: float = 10.0) -> bool:
        set_ready = self._set_client.wait_for_service(timeout_sec=timeout_sec)
        get_ready = self._get_client.wait_for_service(timeout_sec=timeout_sec)
        return bool(set_ready and get_ready)

    def build_state(
        self,
        name: str,
        pose: Pose,
        reference_frame: str = "world",
    ) -> EntityState:
        state = EntityState()
        state.name = name
        state.pose = pose
        state.reference_frame = reference_frame
        return state

    def set_pose_async(
        self,
        name: str,
        pose: Pose,
        reference_frame: str = "world",
    ):
        request = SetEntityState.Request()
        request.state = self.build_state(name, pose, reference_frame)
        return self._set_client.call_async(request)

    def set_pose(
        self,
        name: str,
        pose: Pose,
        reference_frame: str = "world",
        timeout_sec: float = 5.0,
    ) -> bool:
        future = self.set_pose_async(name, pose, reference_frame)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            return False
        response = future.result()
        return bool(response and response.success)

    def get_pose(
        self,
        name: str,
        reference_frame: str = "world",
        timeout_sec: float = 5.0,
    ) -> Optional[Pose]:
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = reference_frame
        future = self._get_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        response = future.result()
        if not response or not response.success:
            return None
        return response.state.pose
