"""Color + point-cloud based object detection and 6D pose estimation.

Subscribes to the overhead RGB-D camera's organized point cloud, transforms
it into the world frame, segments the manipulable lab objects by color,
and publishes a 6D pose estimate (centroid + principal-axis yaw) per object.

Frame handling: the Gazebo rgbd sensor publishes points in one of two axis
conventions (sensor-link x-forward, or optical z-forward) depending on the
stack version. Instead of hardcoding, the node self-calibrates on the first
cloud: it tries both conventions and keeps the one that renders the bench
surface as a horizontal plane at the expected height.

A pluggable detector backend interface is provided so the color segmenter
can be swapped for a deep-learning detector (see detector_backends.py).
"""

import json
import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from robot_lab_perception.detector_backends import make_backend


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Intrinsic XYZ (roll-pitch-yaw) rotation matrix, SDF convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# Rotation from camera OPTICAL axes (x right, y down, z forward) to
# camera LINK axes (x forward, y left, z up).
OPTICAL_TO_LINK = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


class ObjectPoseEstimator(Node):
    """Detect colored lab objects in the bench camera cloud."""

    def __init__(self) -> None:
        super().__init__("object_pose_estimator")

        self.declare_parameter("cloud_topic", "/bench_camera/points")
        self.declare_parameter("camera_xyz", [0.62, 0.0, 2.0])
        self.declare_parameter("camera_rpy", [0.0, 1.5708, 0.0])
        self.declare_parameter("bench_top_z", 0.81)
        self.declare_parameter("workspace_x", [0.25, 1.25])
        self.declare_parameter("workspace_y", [-0.45, 0.45])
        self.declare_parameter("workspace_z_max", 1.4)
        self.declare_parameter("point_stride", 4)
        self.declare_parameter("min_points", 40)
        self.declare_parameter("detector_backend", "color")
        self.declare_parameter("onnx_model_path", "")
        self.declare_parameter("publish_rate", 3.0)

        xyz = self.get_parameter("camera_xyz").value
        rpy = self.get_parameter("camera_rpy").value
        self._cam_pos = np.array(xyz, dtype=float)
        self._cam_rot = rotation_matrix(*rpy)
        self._bench_z = float(self.get_parameter("bench_top_z").value)
        self._ws_x = list(self.get_parameter("workspace_x").value)
        self._ws_y = list(self.get_parameter("workspace_y").value)
        self._ws_z_max = float(self.get_parameter("workspace_z_max").value)
        self._stride = int(self.get_parameter("point_stride").value)
        self._min_points = int(self.get_parameter("min_points").value)

        self._backend = make_backend(
            str(self.get_parameter("detector_backend").value),
            onnx_model_path=str(self.get_parameter("onnx_model_path").value),
            logger=self.get_logger(),
        )

        # Resolved on the first cloud: True -> points are in optical axes.
        self._optical: bool | None = None
        self._latest_cloud: PointCloud2 | None = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            qos,
        )

        self._pose_pubs = {
            name: self.create_publisher(PoseStamped, f"/perception/{name}/pose", 10)
            for name in self._backend.target_names()
        }
        self._summary_pub = self.create_publisher(String, "/perception/detections", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/perception/markers", 10)

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(rate, 0.1), self._process_latest)
        self.get_logger().info(
            "object_pose_estimator up; backend=%s topic=%s"
            % (self._backend.name, self.get_parameter("cloud_topic").value)
        )

    # ------------------------------------------------------------------
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _cloud_to_array(self, msg: PointCloud2) -> np.ndarray | None:
        """Return an (N, 4) array of x, y, z, rgb-float points."""
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z", "rgb"), skip_nans=True
            )
        except Exception as ex:  # noqa: BLE001 - diagnostic path
            self.get_logger().warn(f"cloud parse failed: {ex}")
            return None
        if pts.size == 0:
            return None
        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 4)
        if self._stride > 1:
            arr = arr[:: self._stride]
        return arr

    def _resolve_convention(self, cam_points: np.ndarray) -> None:
        """Pick the axis convention that makes the bench a horizontal plane."""
        candidates = {True: OPTICAL_TO_LINK, False: np.eye(3)}
        best, best_spread = None, float("inf")
        for is_optical, correction in candidates.items():
            world = (self._cam_rot @ correction @ cam_points.T).T + self._cam_pos
            z = world[:, 2]
            plane = z[(z > self._bench_z - 0.2) & (z < self._bench_z + 0.2)]
            if plane.size < 100:
                continue
            spread = float(np.std(plane))
            if spread < best_spread:
                best, best_spread = is_optical, spread
        self._optical = bool(best) if best is not None else True
        self.get_logger().info(
            f"cloud axis convention resolved: optical={self._optical} "
            f"(bench plane z-spread {best_spread:.4f} m)"
        )

    def _process_latest(self) -> None:
        msg = self._latest_cloud
        if msg is None:
            return
        cam_arr = self._cloud_to_array(msg)
        if cam_arr is None:
            return
        cam_points = cam_arr[:, :3].astype(np.float64)
        if self._optical is None:
            self._resolve_convention(cam_points)
        correction = OPTICAL_TO_LINK if self._optical else np.eye(3)
        world = (self._cam_rot @ correction @ cam_points.T).T + self._cam_pos

        rgb_float = cam_arr[:, 3]
        rgb_uint = rgb_float.view(np.uint32) if rgb_float.dtype == np.float32 else (
            np.frombuffer(
                struct.pack(f"{rgb_float.size}f", *rgb_float), dtype=np.uint32
            )
        )
        r = ((rgb_uint >> 16) & 0xFF).astype(np.int16)
        g = ((rgb_uint >> 8) & 0xFF).astype(np.int16)
        b = (rgb_uint & 0xFF).astype(np.int16)

        in_ws = (
            (world[:, 0] > self._ws_x[0])
            & (world[:, 0] < self._ws_x[1])
            & (world[:, 1] > self._ws_y[0])
            & (world[:, 1] < self._ws_y[1])
            & (world[:, 2] > self._bench_z + 0.005)
            & (world[:, 2] < self._ws_z_max)
        )

        detections = self._backend.detect(
            world[in_ws], r[in_ws], g[in_ws], b[in_ws], self._min_points
        )

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        summary = []
        for det in detections:
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = now
            pose.pose.position.x = det.position[0]
            pose.pose.position.y = det.position[1]
            pose.pose.position.z = det.position[2]
            pose.pose.orientation.z = math.sin(det.yaw / 2.0)
            pose.pose.orientation.w = math.cos(det.yaw / 2.0)
            if det.name in self._pose_pubs:
                self._pose_pubs[det.name].publish(pose)

            marker = Marker()
            marker.header = pose.header
            marker.ns = "perception"
            marker.id = hash(det.name) % 10000
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x, marker.scale.y, marker.scale.z = (
                max(det.size[0], 0.02),
                max(det.size[1], 0.02),
                max(det.size[2], 0.02),
            )
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                0.1,
                0.9,
                0.2,
                0.55,
            )
            marker.lifetime.sec = 1
            markers.markers.append(marker)

            summary.append(
                {
                    "name": det.name,
                    "position": [round(v, 4) for v in det.position],
                    "yaw": round(det.yaw, 4),
                    "size": [round(v, 4) for v in det.size],
                    "points": det.num_points,
                }
            )

        self._marker_pub.publish(markers)
        self._summary_pub.publish(String(data=json.dumps(summary)))


def main() -> None:
    rclpy.init()
    node = ObjectPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
