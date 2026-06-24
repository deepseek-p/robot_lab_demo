#!/usr/bin/env python3
"""Task orchestrator: text instructions -> atomic pick/place actions.

Listens for natural-language instructions (Chinese or English) on
``/task/instruction``, decomposes them into a queue of atomic actions, and
dispatches them one at a time to the pick_place_node via ``/task/command``,
waiting for completion on ``/task/status`` between actions. Supports
multi-step instructions ("先把方块放到垫子, 再把试管瓶放到托盘").

Task planning is pluggable: the default RuleBasedPlanner maps object/target
keywords; LlmPlanner is the integration point for a large-language-model
planner with the same parse() contract (instruction -> list of actions), so
upgrading "关键词匹配" to "大模型任务拆解" does not touch the orchestration.
"""

import json
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

OBJECT_KEYWORDS = {
    "sample_block": ["block", "cube", "方块", "样品块", "蓝色"],
    "sample_vial_red": ["vial", "tube", "bottle", "试管", "瓶", "红色"],
}

TARGET_KEYWORDS = {
    "target_pad": ["pad", "垫", "目标垫", "绿色"],
    "sample_tray": ["tray", "托盘"],
    "bench_center": ["center", "中间", "中央"],
}

# Clause separators for multi-step instructions.
SEPARATORS = [";", "；", ",", "，", "然后", "再", "之后", " then ", " and then "]


class RuleBasedPlanner:
    """Keyword-matching task decomposition (default backend)."""

    name = "rule"

    def parse(self, instruction: str) -> list:
        clauses = [instruction.lower()]
        for sep in SEPARATORS:
            next_clauses = []
            for clause in clauses:
                next_clauses.extend(part for part in clause.split(sep) if part.strip())
            clauses = next_clauses

        actions = []
        for clause in clauses:
            obj = self._match(clause, OBJECT_KEYWORDS)
            target = self._match(clause, TARGET_KEYWORDS)
            if "home" in clause or "归位" in clause or "复位" in clause:
                actions.append({"action": "home"})
            elif obj and target:
                actions.append(
                    {"action": "pick_place", "object": obj, "target": target}
                )
        return actions

    @staticmethod
    def _match(clause: str, table: dict) -> str:
        for name, keywords in table.items():
            if any(kw in clause for kw in keywords):
                return name
        return ""


class LlmPlanner:
    """LLM task-decomposition integration point.

    Contract: parse(instruction) -> list of atomic action dicts, identical
    to RuleBasedPlanner. A production implementation prompts a large model
    with the scene inventory and the action schema, then validates the
    returned JSON against the known objects/targets before execution.
    Requires network/API access, so it is not enabled by default.
    """

    name = "llm"

    def __init__(self):
        raise NotImplementedError(
            "Wire your LLM endpoint here; the rule backend is the default."
        )


def make_planner(kind: str):
    return LlmPlanner() if kind == "llm" else RuleBasedPlanner()


class TaskOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("task_orchestrator")
        self.declare_parameter("planner_backend", "rule")
        self._planner = make_planner(
            str(self.get_parameter("planner_backend").value)
        )

        self._queue: queue.Queue = queue.Queue()
        self._busy = False

        self.create_subscription(String, "/task/instruction", self._on_instruction, 10)
        self.create_subscription(String, "/task/status", self._on_status, 10)
        self._command_pub = self.create_publisher(String, "/task/command", 10)
        self._plan_pub = self.create_publisher(String, "/task/plan", 10)
        self.create_timer(0.5, self._dispatch)

        self.get_logger().info(
            f"task_orchestrator up (planner={self._planner.name}); "
            'publish text to /task/instruction, e.g. "把样品块放到目标垫"'
        )

    def _on_instruction(self, msg: String) -> None:
        actions = self._planner.parse(msg.data)
        if not actions:
            self.get_logger().warn(
                f'could not decompose instruction: "{msg.data}" '
                f"(objects: {list(OBJECT_KEYWORDS)}, targets: {list(TARGET_KEYWORDS)})"
            )
            return
        plan = {"instruction": msg.data, "actions": actions}
        self._plan_pub.publish(String(data=json.dumps(plan, ensure_ascii=False)))
        self.get_logger().info(
            f'instruction "{msg.data}" -> {len(actions)} action(s): '
            + json.dumps(actions, ensure_ascii=False)
        )
        for action in actions:
            self._queue.put(action)

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if status.get("state") in ("done", "error"):
            if self._busy:
                self.get_logger().info(f"action finished: {status}")
            self._busy = False

    def _dispatch(self) -> None:
        if self._busy or self._queue.empty():
            return
        action = self._queue.get()
        self._busy = True
        self._command_pub.publish(String(data=json.dumps(action)))
        self.get_logger().info(f"dispatched: {action}")


def main() -> None:
    rclpy.init()
    node = TaskOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
