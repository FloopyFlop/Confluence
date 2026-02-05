#!/usr/bin/env python3
"""
ROS2 Inject node.
Listens for parameter update commands and applies them via PX4 C API or MAVLink.
"""

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except Exception:
    MAVLINK_AVAILABLE = False

try:
    from confluence.px4 import PX4ParamAPI, PX4ParamType
    PX4_C_API_AVAILABLE = True
except Exception as exc:
    PX4_C_API_AVAILABLE = False
    PX4_API_ERROR = str(exc)


def _encode_param_value(value, param_type):
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return float(int(value))
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return float(int(value))
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return float(int(value))
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return float(int(value))
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return float(int(value))
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return float(int(value))
    raise ValueError(f'Unsupported param type {param_type}')


class PX4ConfigInjector(Node):
    def __init__(self, cli_args=None):
        super().__init__('px4_config_injector')

        self.declare_parameter('px4_build_path', '')
        self.declare_parameter('use_sitl', True)
        self.declare_parameter('sitl_connection', 'udp:127.0.0.1:14550')
        self.declare_parameter('serial_port', '/dev/ttyTHS3')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('mavlink_connection', '')
        self.declare_parameter('command_topic', 'px4_injector/command')
        self.declare_parameter('status_topic', 'px4_injector/status')

        self._apply_cli_overrides(cli_args)

        self.px4_build_path = self.get_parameter('px4_build_path').get_parameter_value().string_value
        self.use_sitl = bool(self.get_parameter('use_sitl').value)
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = int(self.get_parameter('serial_baud').value)
        self.command_topic = self.get_parameter('command_topic').value
        self.status_topic = self.get_parameter('status_topic').value

        if self.px4_build_path:
            path = Path(self.px4_build_path)
            if not path.exists():
                self.get_logger().warning(f'PX4 build path does not exist: {path}')

        self.px4_api = None
        if PX4_C_API_AVAILABLE:
            try:
                self.px4_api = PX4ParamAPI()
                self.get_logger().info('PX4 C API loaded successfully.')
            except Exception as exc:
                self.get_logger().warning(f'PX4 C API not available: {exc}')
                self.px4_api = None
        else:
            if 'PX4_API_ERROR' in globals():
                self.get_logger().warning(f'PX4 C API not available: {PX4_API_ERROR}')
            else:
                self.get_logger().warning('PX4 C API not available.')

        self.mavlink_conn_string = self._resolve_mavlink_connection()
        self.mav = None
        if self.px4_api is None and MAVLINK_AVAILABLE:
            try:
                self.get_logger().info(f'Connecting MAVLink at {self.mavlink_conn_string}')
                conn_kwargs = {}
                if not self.use_sitl and self.serial_baud:
                    conn_kwargs['baud'] = self.serial_baud
                self.mav = mavutil.mavlink_connection(self.mavlink_conn_string, **conn_kwargs)
                self.get_logger().info('Waiting for MAVLink heartbeat...')
                self.mav.wait_heartbeat(timeout=10)
                self.get_logger().info('MAVLink connected.')
            except Exception as exc:
                self.get_logger().warning(f'MAVLink connection failed: {exc}')
                self.mav = None
        elif self.px4_api is None:
            self.get_logger().warning('pymavlink not available; cannot apply params.')

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info('Inject node ready. Listening for commands.')

    def _apply_cli_overrides(self, cli_args):
        if cli_args is None:
            return
        updates = []
        if getattr(cli_args, 'px4_build_path', None):
            updates.append(Parameter('px4_build_path', Parameter.Type.STRING, cli_args.px4_build_path))
        if getattr(cli_args, 'sitl', None) is True:
            updates.append(Parameter('use_sitl', Parameter.Type.BOOL, True))
        if getattr(cli_args, 'sitl_connection', None):
            updates.append(Parameter('sitl_connection', Parameter.Type.STRING, cli_args.sitl_connection))
        if getattr(cli_args, 'serial_port', None):
            updates.append(Parameter('serial_port', Parameter.Type.STRING, cli_args.serial_port))
        if getattr(cli_args, 'serial_baud', None):
            updates.append(Parameter('serial_baud', Parameter.Type.INTEGER, cli_args.serial_baud))
        if getattr(cli_args, 'connection', None):
            updates.append(Parameter('mavlink_connection', Parameter.Type.STRING, cli_args.connection))
        if updates:
            self.set_parameters(updates)

    def _resolve_mavlink_connection(self):
        explicit = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        sitl_conn = self.get_parameter('sitl_connection').get_parameter_value().string_value
        if explicit:
            return explicit
        if self.use_sitl:
            return sitl_conn
        return self.serial_port

    def command_callback(self, msg: String):
        self.get_logger().info(f'Received command: {msg.data}')
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Command must be JSON.')
            return

        action = command.get('action', 'set_params')
        if action != 'set_params':
            self.get_logger().warning(f'Unknown action: {action}')
            return

        params = command.get('params')
        if not isinstance(params, dict):
            single = command.get('param')
            value = command.get('value')
            if single is None:
                self.get_logger().error('No params provided in command.')
                return
            params = {single: value}

        results = {}
        for name, value in params.items():
            ok = self._set_param(name, value)
            results[name] = ok

        status = {
            'action': 'set_params',
            'results': results,
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

    def _set_param(self, name, value):
        if self.px4_api is not None:
            try:
                if isinstance(value, bool):
                    value = int(value)
                ok = self.px4_api.set_param(name, value)
                if not ok:
                    self.get_logger().warning(f'PX4 C API failed to set {name}')
                return ok
            except Exception as exc:
                self.get_logger().warning(f'PX4 C API error for {name}: {exc}')

        if self.mav is None:
            self.get_logger().warning(f'No MAVLink connection for {name}')
            return False

        try:
            if isinstance(value, bool):
                value = int(value)
            if isinstance(value, int):
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
            else:
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            encoded = _encode_param_value(value, param_type)
            self.mav.mav.param_set_send(
                self.mav.target_system,
                self.mav.target_component,
                name.encode('utf-8'),
                encoded,
                param_type,
            )
            return True
        except Exception as exc:
            self.get_logger().warning(f'Failed MAVLink param set for {name}: {exc}')
            return False


def _parse_cli_args(argv):
    parser = argparse.ArgumentParser(add_help=False, description='Inject helpers')
    parser.add_argument('--px4-build-path', dest='px4_build_path', help='PX4 build directory')
    parser.add_argument('--sitl', action='store_true', help='Force SITL connection.')
    parser.add_argument('--sitl-connection', dest='sitl_connection', help='SITL connection string.')
    parser.add_argument('--serial-port', dest='serial_port', help='Serial port path for hardware connection.')
    parser.add_argument('--serial-baud', dest='serial_baud', type=int, help='Serial baud rate for hardware connection.')
    parser.add_argument('--connection', dest='connection', help='Override MAVLink connection string.')
    return parser.parse_known_args(argv)


def main(args=None):
    overrides, remaining = _parse_cli_args(args if args is not None else None)
    rclpy.init(args=remaining)
    node = PX4ConfigInjector(cli_args=overrides)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
