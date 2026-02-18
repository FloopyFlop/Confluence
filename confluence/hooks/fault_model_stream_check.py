#!/usr/bin/env python3
"""
Model stream check hook for fault detector.

This hook validates that model input/output is active on a live stack by
inspecting `fault_detector/diagnostics` and `fault_detector/output`.
"""

from __future__ import annotations

import argparse
import json
import statistics
import time
from collections import Counter
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModelStreamCheckNode(Node):
    def __init__(self, args):
        super().__init__("fault_model_stream_check")
        self.args = args
        self.diagnostics: list[dict[str, Any]] = []
        self.outputs: list[dict[str, Any]] = []

        self.diag_sub = self.create_subscription(
            String, args.diagnostics_topic, self._diag_callback, 20
        )
        self.out_sub = self.create_subscription(
            String, args.output_topic, self._output_callback, 20
        )
        self.command_pub = self.create_publisher(String, args.command_topic, 10)

    def _diag_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.diagnostics.append(payload)

    def _output_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.outputs.append(payload)

    def send_command(self, payload: dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload)
        self.command_pub.publish(msg)
        self.get_logger().info(f"Command published: {payload}")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Fault detector model stream check")
    parser.add_argument("--duration", type=float, default=8.0, help="Collection window in seconds")
    parser.add_argument("--expected-feature-len", type=int, default=81)
    parser.add_argument("--inject-fault-index", type=int, choices=[1, 2, 3, 4], default=None)
    parser.add_argument("--expect-fault-label", default=None)
    parser.add_argument("--clear-at-end", action="store_true")
    parser.add_argument("--command-topic", default="fault_detector/command")
    parser.add_argument("--output-topic", default="fault_detector/output")
    parser.add_argument("--diagnostics-topic", default="fault_detector/diagnostics")
    return parser.parse_args(argv)


def _spin_for(node: Node, duration: float):
    end = time.time() + max(0.1, duration)
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def run_check(node: ModelStreamCheckNode) -> int:
    args = node.args
    node.get_logger().info(
        "Collecting model stream data "
        f"for {args.duration:.1f}s from {args.output_topic} / {args.diagnostics_topic}"
    )
    if args.inject_fault_index is not None:
        node.send_command({"action": "inject_fault", "fault_index": int(args.inject_fault_index)})
        _spin_for(node, 0.5)

    diag_start = len(node.diagnostics)
    out_start = len(node.outputs)
    _spin_for(node, args.duration)
    diag_window = node.diagnostics[diag_start:]
    out_window = node.outputs[out_start:]

    inference_events = [d for d in diag_window if d.get("event") == "inference"]
    skipped_events = [d for d in diag_window if d.get("event") == "inference_skipped"]
    model_outputs = [o for o in out_window if o.get("source") == "model"]

    if not inference_events:
        reasons = Counter(str(d.get("reason", "unknown")) for d in skipped_events)
        node.get_logger().error(
            "No model inference events observed. "
            f"skip_reasons={dict(reasons)} (arm + throttle-up gate may be closed)"
        )
        return 2

    feature_lens = sorted(
        {int(d.get("feature_len")) for d in inference_events if d.get("feature_len") is not None}
    )
    if args.expected_feature_len > 0 and feature_lens and feature_lens != [args.expected_feature_len]:
        node.get_logger().error(
            f"Unexpected feature length(s): {feature_lens}, expected {args.expected_feature_len}"
        )
        return 3

    confidences = [float(o.get("confidence")) for o in model_outputs if o.get("confidence") is not None]
    labels = [str(o.get("fault_label", "unknown")) for o in model_outputs]
    preds = [int(o.get("stable_pred", -1)) for o in model_outputs if o.get("stable_pred") is not None]
    label_counts = Counter(labels)
    pred_counts = Counter(preds)

    node.get_logger().info(
        "Model stream summary: "
        f"inference_events={len(inference_events)} "
        f"model_outputs={len(model_outputs)} "
        f"feature_lens={feature_lens} "
        f"stable_pred_hist={dict(pred_counts)} "
        f"fault_label_hist={dict(label_counts)}"
    )
    if confidences:
        node.get_logger().info(
            "Confidence stats: "
            f"min={min(confidences):.4f} "
            f"mean={statistics.mean(confidences):.4f} "
            f"max={max(confidences):.4f}"
        )

    if args.expect_fault_label:
        expected = str(args.expect_fault_label).strip().lower()
        if not any(str(label).lower() == expected for label in labels):
            node.get_logger().error(
                f"Expected fault label not observed: {args.expect_fault_label}. "
                f"Observed labels: {dict(label_counts)}"
            )
            return 4

    if args.clear_at_end:
        node.send_command({"action": "clear_fault", "restore": True})
        _spin_for(node, 0.5)

    node.get_logger().info("Model stream check passed.")
    return 0


def main(args=None):
    parsed = parse_args(args)
    rclpy.init(args=None)
    node = ModelStreamCheckNode(parsed)
    try:
        rc = run_check(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

