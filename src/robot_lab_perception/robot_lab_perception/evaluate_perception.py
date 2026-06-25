"""Perception accuracy evaluation against simulation ground truth.

Runs N trials: teleports the sample block to a random reachable bench
position via the Gazebo set_pose service, waits for the perception
pipeline to settle, then compares the estimated pose against the commanded
ground truth. Produces a CSV and a summary (detection rate, mean/max
position error) suitable for the competition evidence package.

Usage (with the demo bringup and object_pose_estimator running):
    ros2 run robot_lab_perception evaluate_perception
    ros2 run robot_lab_perception evaluate_perception --ros-args -p trials:=30
"""

import csv
import math
import random
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_lab_sim_tools.gazebo_state import GazeboClassicStateClient, make_pose

BLOCK = "sample_block"
BLOCK_Z = 0.845
# Keep the block inside the camera view and the arm workspace.
X_RANGE = (0.45, 0.75)
Y_RANGE = (-0.25, 0.25)


class PerceptionEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("perception_evaluator")
        self.declare_parameter("trials", 20)
        self.declare_parameter("settle_time", 2.0)
        self.declare_parameter("output_csv", "results/perception_eval.csv")
        self._latest: PoseStamped | None = None
        self._gazebo = GazeboClassicStateClient(self)
        self.create_subscription(
            PoseStamped, f"/perception/{BLOCK}/pose", self._on_pose, 10
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest = msg

    def run(self) -> int:
        trials = int(self.get_parameter("trials").value)
        settle = float(self.get_parameter("settle_time").value)
        out_path = Path(self.get_parameter("output_csv").value)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        if not self._gazebo.wait(timeout_sec=10.0):
            self.get_logger().error(
                "Gazebo Classic state services are unavailable; "
                "expected /gazebo/set_entity_state and /gazebo/get_entity_state"
            )
            return 1

        # Wait for the estimator's publisher to be discovered before trial 0,
        # otherwise the first trial races DDS discovery and reads nothing.
        topic = f"/perception/{BLOCK}/pose"
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and self.count_publishers(topic) == 0:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.count_publishers(topic) == 0:
            self.get_logger().error(f"no publisher on {topic}; is the estimator up?")
            return 1

        random.seed(20260610)
        rows = []
        detected = 0
        for i in range(trials):
            gx = random.uniform(*X_RANGE)
            gy = random.uniform(*Y_RANGE)
            if not self._gazebo.set_pose(BLOCK, make_pose(gx, gy, BLOCK_Z)):
                self.get_logger().warn(f"trial {i}: set_pose failed, skipping")
                continue

            # Let physics settle and the estimator publish fresh poses.
            deadline = time.monotonic() + settle
            self._latest = None
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

            row = {"trial": i, "gt_x": gx, "gt_y": gy}
            if self._latest is None:
                row.update(found=0, err_xy_mm="", err_x_mm="", err_y_mm="")
                self.get_logger().warn(f"trial {i}: no detection")
            else:
                px = self._latest.pose.position.x
                py = self._latest.pose.position.y
                ex, ey = px - gx, py - gy
                err = math.hypot(ex, ey)
                detected += 1
                row.update(
                    found=1,
                    err_xy_mm=round(err * 1000.0, 2),
                    err_x_mm=round(ex * 1000.0, 2),
                    err_y_mm=round(ey * 1000.0, 2),
                )
                self.get_logger().info(
                    f"trial {i}: gt=({gx:.3f},{gy:.3f}) "
                    f"est=({px:.3f},{py:.3f}) err={err * 1000:.1f} mm"
                )
            rows.append(row)

        with out_path.open("w", newline="") as fh:
            writer = csv.DictWriter(
                fh,
                fieldnames=[
                    "trial", "gt_x", "gt_y", "found",
                    "err_xy_mm", "err_x_mm", "err_y_mm",
                ],
            )
            writer.writeheader()
            writer.writerows(rows)

        errors = [r["err_xy_mm"] for r in rows if r["found"]]
        n = len(rows)
        rate = 100.0 * detected / n if n else 0.0
        mean_err = sum(errors) / len(errors) if errors else float("nan")
        max_err = max(errors) if errors else float("nan")
        self.get_logger().info(
            f"=== perception evaluation: {detected}/{n} detected ({rate:.1f}%), "
            f"mean err {mean_err:.1f} mm, max err {max_err:.1f} mm -> {out_path}"
        )
        return 0 if detected == n and n > 0 else 1


def main() -> None:
    rclpy.init()
    node = PerceptionEvaluator()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
