#!/usr/bin/env python3
"""
Fault detector probe hook.

Runs a focused integration probe against a live Confluence stack.
It checks:
1. Fault detector can receive/parse data and run inference (via diagnostics probe).
2. Forced fault command path works.
3. Forced clear command path works.
4. Optional injector status visibility.

Usage:
  ros2 run confluence fault_detector_probe
  ros2 run confluence fault_detector_probe --fault-index 3 --timeout 20
  ros2 run confluence fault_detector_probe --probe-only
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


FAULT_PARAM_BY_INDEX = {
    1: "PWM_MAIN_FUNC4",
    2: "PWM_MAIN_FUNC1",
    3: "PWM_MAIN_FUNC2",
    4: "PWM_MAIN_FUNC3",
}


class FaultDetectorProbeNode(Node):
    def __init__(self, args):
        super().__init__("fault_detector_probe_node")
        self.args = args
        self.outputs: list[dict[str, Any]] = []
        self.statuses: list[dict[str, Any]] = []
        self.detector_statuses: list[dict[str, Any]] = []
        self.diags: list[dict[str, Any]] = []

        self.command_pub = self.create_publisher(String, args.command_topic, 10)
        self.output_sub = self.create_subscription(
            String, args.output_topic, self._output_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, args.inject_status_topic, self._status_callback, 10
        )
        self.detector_status_sub = self.create_subscription(
            String, args.status_topic, self._detector_status_callback, 10
        )
        self.diag_sub = self.create_subscription(
            String, args.diagnostics_topic, self._diag_callback, 10
        )

    def _output_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.outputs.append(payload)

    def _status_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.statuses.append(payload)

    def _diag_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.diags.append(payload)

    def _detector_status_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.detector_statuses.append(payload)

    def send_command(self, payload: dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload)
        self.command_pub.publish(msg)
        self.get_logger().info(f"Command published: {payload}")

    def wait_for(self, predicate, timeout: float, label: str):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                self.get_logger().info(f"PASS: {label}")
                return True
        self.get_logger().error(f"TIMEOUT: {label}")
        return False


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Confluence fault detector probe hook")
    parser.add_argument("--fault-index", type=int, default=1, choices=[1, 2, 3, 4])
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--command-topic", default="fault_detector/command")
    parser.add_argument("--output-topic", default="fault_detector/output")
    parser.add_argument("--status-topic", default="fault_detector/status")
    parser.add_argument("--diagnostics-topic", default="fault_detector/diagnostics")
    parser.add_argument("--inject-status-topic", default="px4_injector/status")
    parser.add_argument(
        "--no-inject-status-check",
        action="store_true",
        help="Skip checking px4_injector/status acknowledgements.",
    )
    return parser.parse_args(argv)


def _run_probe(node: FaultDetectorProbeNode, timeout: float):
    probe_id = int(time.time() * 1000)
    node.send_command({"action": "run_probe", "probe_id": probe_id})
    ok = node.wait_for(
        lambda: any(
            d.get("event") == "probe_result" and d.get("probe_id") == probe_id
            for d in node.diags
        ),
        timeout=timeout,
        label="probe_result observed",
    )
    if not ok:
        return False
    result = next(
        d for d in reversed(node.diags)
        if d.get("event") == "probe_result" and d.get("probe_id") == probe_id
    )
    passed = bool(result.get("passed"))
    node.get_logger().info(f"Probe payload: {result}")
    return passed


def run_test(node: FaultDetectorProbeNode):
    timeout = node.args.timeout
    fault_index = int(node.args.fault_index)
    expected_param = FAULT_PARAM_BY_INDEX[fault_index]

    # Baseline clear, no restore.
    node.send_command({"action": "clear_fault", "restore": False})
    node.wait_for(
        lambda: any(
            o.get("source") == "forced"
            and not bool(o.get("fault"))
            for o in node.outputs
        ),
        timeout=min(5.0, timeout),
        label="initial clear acknowledged",
    )
    node.outputs.clear()
    node.statuses.clear()
    node.detector_statuses.clear()
    node.diags.clear()

    if not _run_probe(node, timeout):
        node.get_logger().error("Probe checks failed (data/inference path).")
        return 1
    if node.args.probe_only:
        node.get_logger().info("Probe-only run completed.")
        return 0

    # Forced fault path.
    node.send_command({"action": "inject_fault", "fault_index": fault_index})
    ok_fault_output = node.wait_for(
        lambda: any(
            o.get("source") == "forced"
            and bool(o.get("fault"))
            and expected_param in (o.get("parameters") or {})
            for o in node.outputs
        ),
        timeout=timeout,
        label="forced fault output observed",
    )
    if not ok_fault_output:
        return 2

    ok_fault_status = node.wait_for(
        lambda: any(
            s.get("source") == "forced"
            and bool(s.get("fault"))
            for s in node.detector_statuses
        ),
        timeout=timeout,
        label="forced fault status observed",
    )
    if not ok_fault_status:
        return 3

    if not node.args.no_inject_status_check:
        ok_fault_inject_status = node.wait_for(
            lambda: any(
                s.get("action") == "set_params"
                and (
                    bool(s.get("queued"))
                    or (expected_param in (s.get("results") or {}))
                )
                for s in node.statuses
            ),
            timeout=timeout,
            label="fault inject status observed",
        )
        if not ok_fault_inject_status:
            return 4

    # Clear path with restore.
    node.outputs.clear()
    node.statuses.clear()
    node.detector_statuses.clear()
    node.send_command({"action": "clear_fault", "restore": True})
    ok_clear_output = node.wait_for(
        lambda: any(
            o.get("source") == "forced"
            and not bool(o.get("fault"))
            for o in node.outputs
        ),
        timeout=timeout,
        label="forced nominal output after clear observed",
    )
    if not ok_clear_output:
        return 5

    ok_clear_status = node.wait_for(
        lambda: any(
            s.get("source") == "forced"
            and not bool(s.get("fault"))
            for s in node.detector_statuses
        ),
        timeout=timeout,
        label="forced clear status observed",
    )
    if not ok_clear_status:
        return 6

    if not node.args.no_inject_status_check:
        ok_clear_inject_status = node.wait_for(
            lambda: any(
                s.get("action") == "set_params"
                and s.get("source") == "fault_detector_clear"
                and (
                    bool(s.get("queued"))
                    or ("PWM_MAIN_FUNC1" in (s.get("results") or {}))
                )
                for s in node.statuses
            ),
            timeout=timeout,
            label="clear restore status observed",
        )
        if not ok_clear_inject_status:
            return 7

    node.get_logger().info("All probe checks passed.")
    return 0


def main(args=None):
    parsed = parse_args(args)
    rclpy.init(args=None)
    node = FaultDetectorProbeNode(parsed)
    try:
        rc = run_test(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
