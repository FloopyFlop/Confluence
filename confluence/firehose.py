#!/usr/bin/env python3
"""
ROS2 Firehose node.
Connects to PX4 via pymavlink and publishes raw MAVLink messages as JSON.
"""

from __future__ import annotations

import argparse
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, UInt8MultiArray

from confluence.utils.param_injection import (
    decode_param_value,
    encode_param_value,
    normalize_param_id,
)

try:
    from pymavlink import mavutil

    MAVLINK_AVAILABLE = True
except Exception:
    MAVLINK_AVAILABLE = False


def _json_safe(value):
    if isinstance(value, (bytes, bytearray, memoryview)):
        return list(value)
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(v) for v in value]
    return value


class MavFirehoseNode(Node):
    def __init__(self, cli_overrides=None):
        super().__init__("mav_firehose_node")

        self.declare_parameter("use_sitl", False)
        self.declare_parameter("connection_string", "")
        self.declare_parameter("serial_port", "/dev/ttyTHS3")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("sitl_address", "udp:127.0.0.1:14550")
        self.declare_parameter("data_stream_rate", 100)
        self.declare_parameter("publish_raw_bytes", False)
        self.declare_parameter("topic_all_messages", "mav/all_messages")
        self.declare_parameter("topic_raw_bytes", "mav/raw/bytes")
        self.declare_parameter("inject_command_topic", "px4_injector/command")
        self.declare_parameter("inject_status_topic", "px4_injector/status")

        self._apply_cli_overrides(cli_overrides)

        if not MAVLINK_AVAILABLE:
            self.get_logger().error(
                "pymavlink is not available. Install it before running firehose."
            )

        use_sitl = bool(self.get_parameter("use_sitl").value)
        connection_override = str(self.get_parameter("connection_string").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.serial_baud = int(self.get_parameter("serial_baud").value)

        if connection_override:
            self.connection_string = connection_override
        elif use_sitl:
            self.connection_string = str(self.get_parameter("sitl_address").value)
        else:
            self.connection_string = self.serial_port

        self.connection_string = self._normalize_connection_string(self.connection_string)
        self.stream_rate = int(self.get_parameter("data_stream_rate").value)
        self.publish_raw_bytes = bool(self.get_parameter("publish_raw_bytes").value)

        self.topic_all_messages = str(self.get_parameter("topic_all_messages").value)
        self.topic_raw_bytes = str(self.get_parameter("topic_raw_bytes").value)
        self.inject_command_topic = str(self.get_parameter("inject_command_topic").value)
        self.inject_status_topic = str(self.get_parameter("inject_status_topic").value)

        self.all_messages_pub = self.create_publisher(String, self.topic_all_messages, 100)
        self.raw_bytes_pub = None
        if self.publish_raw_bytes:
            self.raw_bytes_pub = self.create_publisher(UInt8MultiArray, self.topic_raw_bytes, 100)
        self.inject_status_pub = self.create_publisher(String, self.inject_status_topic, 10)
        self.inject_command_sub = self.create_subscription(
            String,
            self.inject_command_topic,
            self._inject_command_callback,
            10,
        )

        self.mav_conn = None
        self.mav_lock = threading.Lock()
        self._stop_event = threading.Event()

        if MAVLINK_AVAILABLE:
            self.mav_thread = threading.Thread(target=self._mavlink_loop, daemon=True)
            self.mav_thread.start()

    def _apply_cli_overrides(self, overrides):
        if overrides is None:
            return
        updates = []
        if getattr(overrides, "sitl", False):
            updates.append(Parameter("use_sitl", Parameter.Type.BOOL, True))
        if getattr(overrides, "connection_string", None):
            updates.append(
                Parameter(
                    "connection_string", Parameter.Type.STRING, overrides.connection_string
                )
            )
        if getattr(overrides, "serial_port", None):
            updates.append(
                Parameter("serial_port", Parameter.Type.STRING, overrides.serial_port)
            )
        if getattr(overrides, "serial_baud", None):
            updates.append(
                Parameter("serial_baud", Parameter.Type.INTEGER, overrides.serial_baud)
            )
        if getattr(overrides, "data_stream_rate", None):
            updates.append(
                Parameter(
                    "data_stream_rate", Parameter.Type.INTEGER, overrides.data_stream_rate
                )
            )
        if getattr(overrides, "disable_raw_bytes", False):
            updates.append(Parameter("publish_raw_bytes", Parameter.Type.BOOL, False))
        if updates:
            self.set_parameters(updates)

    def _normalize_connection_string(self, conn: str) -> str:
        if not conn:
            return conn
        raw = conn
        if raw.startswith("serial://"):
            raw = raw[len("serial://") :]
        elif raw.startswith("serial:"):
            raw = raw[len("serial:") :]

        if ":" in raw:
            path, maybe_baud = raw.rsplit(":", 1)
            if maybe_baud.isdigit():
                try:
                    self.serial_baud = int(maybe_baud)
                except Exception:
                    pass
                raw = path

        if raw.startswith("/dev/"):
            self.serial_port = raw
            return raw

        return conn

    def _request_streams(self):
        if not self.mav_conn:
            return
        try:
            self.mav_conn.mav.request_data_stream_send(
                self.mav_conn.target_system,
                self.mav_conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                self.stream_rate,
                1,
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to request data streams: {exc}")

    def _set_param_and_verify(self, param_name, value):
        if self.mav_conn is None:
            return False, None, "not_connected"

        try:
            if isinstance(value, float):
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            else:
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32

            encoded = encode_param_value(value, param_type)

            with self.mav_lock:
                self.mav_conn.mav.param_set_send(
                    self.mav_conn.target_system,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    param_name.encode("utf-8"),
                    encoded,
                    param_type,
                )
                time.sleep(0.1)
                self.mav_conn.mav.param_request_read_send(
                    self.mav_conn.target_system,
                    self.mav_conn.target_component,
                    param_name.encode("utf-8"),
                    -1,
                )
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    remaining = max(0.0, deadline - time.time())
                    msg = self.mav_conn.recv_match(
                        type="PARAM_VALUE",
                        blocking=True,
                        timeout=remaining,
                    )
                    if msg is None:
                        return False, None, "no_response"
                    if normalize_param_id(msg.param_id) != param_name:
                        continue
                    actual = decode_param_value(msg)
                    if isinstance(value, float):
                        ok = abs(float(actual) - float(value)) < 1e-4
                    else:
                        ok = int(actual) == int(value)
                    if ok:
                        return True, actual, None
                    return False, actual, "mismatch"
                return False, None, "timeout"
        except Exception as exc:
            return False, None, str(exc)

    def _inject_command_callback(self, msg):
        try:
            command = json.loads(msg.data)
        except Exception:
            self.get_logger().warning("Inject command must be JSON")
            return

        if command.get("action", "set_params") != "set_params":
            self.get_logger().warning(f"Unsupported inject action: {command.get('action')}")
            return

        params = command.get("params")
        if not isinstance(params, dict):
            single = command.get("param")
            if single is None:
                self.get_logger().warning("Inject command missing params")
                return
            params = {single: command.get("value")}

        results = {}
        details = {}
        for param_name, param_value in params.items():
            ok, actual, reason = self._set_param_and_verify(param_name, param_value)
            results[param_name] = bool(ok)
            detail = {"ok": bool(ok), "actual": actual}
            if reason is not None:
                detail["reason"] = reason
            details[param_name] = detail

        status_msg = String()
        status_msg.data = json.dumps(
            {
                "action": "set_params",
                "results": results,
                "details": details,
                "source": "firehose",
            }
        )
        self.inject_status_pub.publish(status_msg)

    def _mavlink_loop(self):
        try:
            conn_kwargs = {}
            if not bool(self.get_parameter("use_sitl").value) and self.serial_baud:
                conn_kwargs["baud"] = self.serial_baud
            self.get_logger().info(f"Connecting MAVLink: {self.connection_string}")
            self.mav_conn = mavutil.mavlink_connection(self.connection_string, **conn_kwargs)
            self.get_logger().info("Waiting for MAVLink heartbeat...")
            self.mav_conn.wait_heartbeat(timeout=10)
            self.get_logger().info("MAVLink connected.")
            self._request_streams()
        except Exception as exc:
            self.get_logger().error(f"Failed to connect MAVLink: {exc}")
            return

        while rclpy.ok() and not self._stop_event.is_set():
            try:
                with self.mav_lock:
                    msg = self.mav_conn.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                msg_type = msg.get_type()
                if msg_type == "BAD_DATA":
                    continue

                msg_dict = msg.to_dict()
                msg_dict["_msg_type"] = msg_type
                msg_dict["_timestamp"] = time.time()

                payload = String()
                payload.data = json.dumps(_json_safe(msg_dict))
                self.all_messages_pub.publish(payload)

                if self.raw_bytes_pub is not None and hasattr(msg, "get_msgbuf"):
                    raw = msg.get_msgbuf()
                    raw_msg = UInt8MultiArray()
                    raw_msg.data = list(raw)
                    self.raw_bytes_pub.publish(raw_msg)
            except Exception as exc:
                self.get_logger().warning(f"Error reading MAVLink message: {exc}")

    def destroy_node(self):
        self._stop_event.set()
        if self.mav_conn is not None:
            try:
                self.mav_conn.close()
            except Exception:
                pass
        super().destroy_node()


def _parse_cli_args(argv):
    parser = argparse.ArgumentParser(add_help=False, description="Firehose connection helpers")
    parser.add_argument("--sitl", action="store_true", help="Force SITL connection.")
    parser.add_argument("--connection", dest="connection_string", help="Override MAVLink connection string.")
    parser.add_argument("--serial-port", dest="serial_port", help="Serial port path for hardware connection.")
    parser.add_argument("--serial-baud", dest="serial_baud", type=int, help="Serial baud rate for hardware connection.")
    parser.add_argument("--stream-rate", dest="data_stream_rate", type=int, help="Data stream request rate (Hz).")
    parser.add_argument("--no-raw-bytes", dest="disable_raw_bytes", action="store_true", help="Disable raw byte publishing.")
    return parser.parse_known_args(argv)


def main(args=None):
    overrides, remaining = _parse_cli_args(args if args is not None else None)
    rclpy.init(args=remaining)
    node = MavFirehoseNode(cli_overrides=overrides)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
