#!/usr/bin/env python3
"""
ROS2 Inject node.

Single responsibility:
- Subscribe to `px4_injector/direct_command`
- Apply MAVLink PARAM_SET using the same logic as `monolithic_fault`
- Publish result on `px4_injector/direct_status`
"""

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from confluence.utils.param_injection import (
    ParamInjectorClient,
    env_or_default,
    load_env_file,
)


class InjectNode(Node):
    def __init__(self, cli_args=None):
        super().__init__("px4_config_injector")

        self.declare_parameter("env_file", ".drone-env")
        self.declare_parameter("connection", "")
        self.declare_parameter("baud", 0)
        self.declare_parameter("mavsdk_address", "")
        self.declare_parameter("timeout", 0.0)
        self.declare_parameter("skip_mavsdk", True)
        self.declare_parameter("command_topic", "px4_injector/direct_command")
        self.declare_parameter("status_topic", "px4_injector/direct_status")

        self._apply_cli_overrides(cli_args)
        self._client = self._build_client()

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._command_callback,
            10,
        )
        self.get_logger().info("Inject node ready. Listening for commands.")

    def _apply_cli_overrides(self, cli_args):
        if cli_args is None:
            return
        updates = []
        if cli_args.env_file:
            updates.append(
                Parameter("env_file", Parameter.Type.STRING, cli_args.env_file)
            )
        if cli_args.connection:
            updates.append(
                Parameter("connection", Parameter.Type.STRING, cli_args.connection)
            )
        if cli_args.baud is not None:
            updates.append(Parameter("baud", Parameter.Type.INTEGER, int(cli_args.baud)))
        if cli_args.mavsdk_address:
            updates.append(
                Parameter(
                    "mavsdk_address",
                    Parameter.Type.STRING,
                    cli_args.mavsdk_address,
                )
            )
        if cli_args.timeout is not None:
            updates.append(Parameter("timeout", Parameter.Type.DOUBLE, float(cli_args.timeout)))
        if cli_args.skip_mavsdk:
            updates.append(Parameter("skip_mavsdk", Parameter.Type.BOOL, True))
        if updates:
            self.set_parameters(updates)

    def _build_client(self) -> ParamInjectorClient:
        env_file = str(self.get_parameter("env_file").value)
        env_values = load_env_file(env_file)

        connection_raw = str(self.get_parameter("connection").value).strip()
        baud_raw = int(self.get_parameter("baud").value)
        mavsdk_raw = str(self.get_parameter("mavsdk_address").value).strip()
        timeout_raw = float(self.get_parameter("timeout").value)
        skip_mavsdk = bool(self.get_parameter("skip_mavsdk").value)

        connection = env_or_default(
            connection_raw if connection_raw else None,
            env_values,
            "CONFLUENCE_MAVLINK_CONNECTION",
            "/dev/ttyTHS3",
            str,
        )
        baud = env_or_default(
            baud_raw if baud_raw > 0 else None,
            env_values,
            "CONFLUENCE_MAVLINK_BAUD",
            115200,
            int,
        )
        mavsdk_address = env_or_default(
            mavsdk_raw if mavsdk_raw else None,
            env_values,
            "CONFLUENCE_MAVSDK_ADDRESS",
            f"serial://{connection}:{baud}",
            str,
        )
        timeout = env_or_default(
            timeout_raw if timeout_raw > 0 else None,
            env_values,
            "CONFLUENCE_PARAM_TIMEOUT",
            5.0,
            float,
        )

        self.get_logger().info(
            f"Inject config connection={connection} baud={baud} mavsdk={mavsdk_address}"
        )
        return ParamInjectorClient(
            connection=connection,
            baud=baud,
            mavsdk_address=mavsdk_address,
            timeout_sec=timeout,
            enable_mavsdk_warmup=not skip_mavsdk,
        )

    def _command_callback(self, msg: String):
        try:
            command = json.loads(msg.data)
        except Exception:
            self.get_logger().error("Command must be JSON")
            return

        if command.get("action", "set_params") != "set_params":
            self.get_logger().warning(f"Unsupported action: {command.get('action')}")
            return

        params = command.get("params")
        if not isinstance(params, dict):
            single = command.get("param")
            if single is None:
                self.get_logger().warning("No params supplied")
                return
            params = {single: command.get("value")}

        results: dict[str, bool] = {}
        details: dict[str, dict[str, object]] = {}
        try:
            for param_name, param_value in params.items():
                ok, actual, reason = self._client.set_and_verify(param_name, param_value)
                results[param_name] = bool(ok)
                detail: dict[str, object] = {"ok": bool(ok), "actual": actual}
                if reason is not None:
                    detail["reason"] = reason
                details[param_name] = detail
        finally:
            # Release serial ownership after each command to avoid long-lived
            # contention with firehose when both target the same device.
            try:
                self._client.close()
            except Exception:
                pass

        status_payload = {
            "action": "set_params",
            "results": results,
            "details": details,
            "source": "inject",
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload)
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        try:
            self._client.close()
        except Exception:
            pass
        super().destroy_node()


def _parse_cli_args(argv):
    parser = argparse.ArgumentParser(add_help=False, description="Inject connection helpers")
    parser.add_argument("--env-file", dest="env_file", help="Path to .drone-env file")
    parser.add_argument("--connection", dest="connection", help="MAVLink connection string")
    parser.add_argument("--baud", dest="baud", type=int, help="Serial baud")
    parser.add_argument("--mavsdk-address", dest="mavsdk_address", help="MAVSDK warmup address")
    parser.add_argument("--timeout", dest="timeout", type=float, help="Timeout in seconds")
    parser.add_argument("--skip-mavsdk", dest="skip_mavsdk", action="store_true", help="Skip MAVSDK warmup")
    return parser.parse_known_args(argv)


def main(args=None):
    cli_args, ros_args = _parse_cli_args(args if args is not None else None)
    rclpy.init(args=ros_args)
    node = InjectNode(cli_args=cli_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
