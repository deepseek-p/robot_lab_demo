#!/usr/bin/env python3
"""Pick-and-place success-rate evaluation with optional dynamic obstacle.

Runs N full perception-driven pick/place cycles through the task pipeline
(/task/command -> pick_place_node), randomizing the block position each
trial via the Gazebo set_pose service. Optionally sweeps the moving
obstacle across the transfer corridor during execution to exercise dynamic
avoidance. Writes results/pick_place_eval.csv with per-trial success and
cycle time — evidence for the 任务分解与规划成功率 metric.

Usage (demo bringup + perception + pick_place_node running):
    ros2 run robot_lab_tasks evaluate_pick_place.py
    ros2 run robot_lab_tasks evaluate_pick_place.py --ros-args -p trials:=10 -p sweep_obstacle:=true
"""

import csv
import json
import random
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WORLD = "robot_lab_minimal"
BLOCK = "sample_block"
BLOCK_Z = 0.845
X_RANGE = (0.48, 0.70)
Y_RANGE = (0.05, 0.28)  # keep clear of the place targets on the -y side
OBSTACLE_PARK = (0.58, 0.7, 1.05)


def gz_set_pose(name: str, x: float, y: float, z: float) -> bool:
    req = f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'
    result = subprocess.run(
        [
            "gz", "service", "-s", f"/world/{WORLD}/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", req,
        ],
        capture_output=True, text=True, timeout=10,
    )
    return "true" in result.stdout


class ObstacleSweeper(threading.Thread):
    """Intermittent human-like interference: park clear of the bench, then
    periodically sweep once across the transfer corridor and park again.
    Continuous sweeping would invalidate every multi-second trajectory; an
    intermittent pass exercises the abort-replan-retry loop while letting
    motions complete between passes."""

    PARK_S = 18.0
    SWEEP_S = 6.0

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.stop_flag = threading.Event()

    def run(self) -> None:
        while not self.stop_flag.is_set():
            # Parked phase.
            gz_set_pose("moving_obstacle", *OBSTACLE_PARK)
            if self.stop_flag.wait(self.PARK_S):
                break
            # One pass: 0.7 -> -0.3 -> 0.7 over SWEEP_S seconds.
            t0 = time.monotonic()
            while not self.stop_flag.is_set():
                phase = (time.monotonic() - t0) / self.SWEEP_S
                if phase >= 1.0:
                    break
                # Triangle: 0.7 -> -0.3 -> 0.7 over one sweep period.
                y = 0.7 - (1.0 - abs(2.0 * phase - 1.0)) * 1.0
                gz_set_pose("moving_obstacle", 0.58, y, 1.05)
                time.sleep(0.2)
        gz_set_pose("moving_obstacle", *OBSTACLE_PARK)


class PickPlaceEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_evaluator")
        self.declare_parameter("trials", 10)
        self.declare_parameter("timeout", 180.0)
        self.declare_parameter("sweep_obstacle", False)
        self.declare_parameter("output_csv", "results/pick_place_eval.csv")
        self._status = None
        self.create_subscription(String, "/task/status", self._on_status, 10)
        self._cmd_pub = self.create_publisher(String, "/task/command", 10)

    def _on_status(self, msg: String) -> None:
        try:
            self._status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _await_state(self, states, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._status and self._status.get("state") in states:
                return self._status
        return None

    def run(self) -> int:
        trials = int(self.get_parameter("trials").value)
        timeout = float(self.get_parameter("timeout").value)
        sweep = bool(self.get_parameter("sweep_obstacle").value)
        out_path = Path(self.get_parameter("output_csv").value)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Wait for the pick_place_node to be discovered.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and self.count_subscribers("/task/command") == 0:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.count_subscribers("/task/command") == 0:
            self.get_logger().error("pick_place_node not running")
            return 1

        sweeper = None
        if sweep:
            sweeper = ObstacleSweeper()
            sweeper.start()
            self.get_logger().info("dynamic obstacle sweep ENABLED")

        random.seed(20260611)
        rows = []
        successes = 0
        for i in range(trials):
            gx = random.uniform(*X_RANGE)
            gy = random.uniform(*Y_RANGE)
            if not gz_set_pose(BLOCK, gx, gy, BLOCK_Z):
                self.get_logger().warn(f"trial {i}: set_pose failed")
                continue
            time.sleep(2.5)  # physics + perception settle

            self._status = None
            self._cmd_pub.publish(
                String(
                    data=json.dumps(
                        {"action": "pick_place", "object": BLOCK, "target": "target_pad"}
                    )
                )
            )
            t_start = time.monotonic()
            result = self._await_state(("done", "error"), timeout)
            elapsed = time.monotonic() - t_start

            ok = bool(result and result.get("state") == "done")
            successes += int(ok)
            rows.append(
                {
                    "trial": i,
                    "gt_x": round(gx, 4),
                    "gt_y": round(gy, 4),
                    "success": int(ok),
                    "cycle_s": round(elapsed, 1),
                    "obstacle": int(sweep),
                }
            )
            self.get_logger().info(
                f"trial {i}: {'OK' if ok else 'FAIL'} in {elapsed:.1f}s"
            )

        if sweeper:
            sweeper.stop_flag.set()
            sweeper.join(timeout=5)

        with out_path.open("w", newline="") as fh:
            writer = csv.DictWriter(
                fh,
                fieldnames=["trial", "gt_x", "gt_y", "success", "cycle_s", "obstacle"],
            )
            writer.writeheader()
            writer.writerows(rows)

        n = len(rows)
        rate = 100.0 * successes / n if n else 0.0
        self.get_logger().info(
            f"=== pick/place evaluation: {successes}/{n} succeeded ({rate:.1f}%) "
            f"-> {out_path}"
        )
        return 0 if successes == n and n > 0 else 1


def main() -> None:
    rclpy.init()
    node = PickPlaceEvaluator()
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
