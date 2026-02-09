#!/usr/bin/env python3
"""
One-shot ROS2 hook to verify parameter injection via px4_injector topics.

Example:
  ros2 run confluence verify_injection --param PWM_MAIN_FUNC4 --value 101 --timeout 5
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VerifyInjectionNode(Node):
    def __init__(self, param_name, param_value, timeout_sec):
        super().__init__('verify_injection_hook')
        self.param_name = param_name
        self.param_value = param_value
        self.timeout_sec = timeout_sec
        self.deadline = time.time() + timeout_sec
        self.done = False
        self.success = False
        self.last_status = None

        self.pub = self.create_publisher(String, 'px4_injector/command', 10)
        self.sub = self.create_subscription(String, 'px4_injector/status', self._status_cb, 10)

        # Publish once after startup so discovery has a chance to settle.
        self.timer = self.create_timer(0.5, self._send_once)

    def _send_once(self):
        self.timer.cancel()
        payload = {
            'action': 'set_params',
            'params': {self.param_name: self.param_value},
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)
        self.get_logger().info(f'Sent command: {payload}')

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.last_status = data

        results = data.get('results', {})
        details = data.get('details', {})
        if self.param_name not in results:
            return

        ok = bool(results.get(self.param_name, False))
        detail = details.get(self.param_name, {}) if isinstance(details, dict) else {}
        source = data.get('source', 'unknown')

        if ok:
            self.get_logger().info(f'Verified by {source}: {self.param_name}={detail.get("actual")}')
            self.success = True
        else:
            self.get_logger().error(f'Injection failed via {source}: {detail}')
            self.success = False
        self.done = True

    def timed_out(self):
        return time.time() > self.deadline


def _parse_value(raw: str):
    try:
        if '.' in raw:
            return float(raw)
        return int(raw)
    except Exception:
        return raw


def main(args=None):
    parser = argparse.ArgumentParser(description='Verify one parameter injection end-to-end')
    parser.add_argument('--param', required=True, help='PX4 parameter name')
    parser.add_argument('--value', required=True, help='Target value')
    parser.add_argument('--timeout', type=float, default=5.0, help='Timeout seconds')
    parsed, ros_args = parser.parse_known_args(args)

    value = _parse_value(parsed.value)

    rclpy.init(args=ros_args)
    node = VerifyInjectionNode(parsed.param, value, parsed.timeout)
    try:
        while rclpy.ok() and not node.done and not node.timed_out():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if not node.done:
            node.get_logger().error('Timed out waiting for verification status')
            if node.last_status is not None:
                node.get_logger().error(f'Last status: {node.last_status}')
        exit_code = 0 if node.success else 1
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
