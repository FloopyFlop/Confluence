"""
Confluence fault hook.

Single hook entrypoint for:
1. Local param write+verify against PX4 (former monolithic fault runner).
2. Remote forced-fault command to a running orchestrator console (former induce_fault).

Examples:
  ros2 run confluence fault_hook
  ros2 run confluence fault_hook --param PWM_MAIN_FUNC4 --value 0
  ros2 run confluence fault_hook --mode remote --host 10.48.128.81 --port 9000 --motor 1
"""

from __future__ import annotations

import argparse
import json
import socket

from confluence.utils.param_injection import ParamInjectorClient, env_or_default, load_env_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Confluence fault hook")
    parser.add_argument(
        "--mode",
        choices=["local", "remote"],
        default="local",
        help="local: write+verify PX4 param, remote: send forced fault command to orchestrator",
    )

    # Shared defaults
    parser.add_argument(
        "--env-file",
        default=".drone-env",
        help="Path to environment defaults file (default: .drone-env)",
    )

    # Local mode args
    parser.add_argument("--connection", default=None, help="pymavlink connection string or serial device")
    parser.add_argument("--baud", type=int, default=None, help="Serial baud for pymavlink serial connections")
    parser.add_argument("--mavsdk-address", default=None, help="MAVSDK system address for warmup link")
    parser.add_argument("--skip-mavsdk", action="store_true", help="Skip MAVSDK warmup step")
    parser.add_argument("--param", default=None, help="PX4 parameter to set/read back")
    parser.add_argument("--value", type=int, default=None, help="INT32 value to set")
    parser.add_argument("--timeout", type=float, default=None, help="Heartbeat/read timeout seconds")

    # Remote mode args
    parser.add_argument("--host", default=None, help="Orchestrator host for remote mode")
    parser.add_argument("--port", type=int, default=None, help="Orchestrator console port for remote mode")
    parser.add_argument("--motor", type=int, default=1, help="Motor fault index (1-4) for remote mode")
    return parser.parse_args()


def _run_local(args: argparse.Namespace, env_values: dict[str, str]) -> int:
    connection = env_or_default(
        args.connection,
        env_values,
        "CONFLUENCE_MAVLINK_CONNECTION",
        "/dev/ttyTHS3",
        str,
    )
    baud = env_or_default(
        args.baud,
        env_values,
        "CONFLUENCE_MAVLINK_BAUD",
        115200,
        int,
    )
    mavsdk_address = env_or_default(
        args.mavsdk_address,
        env_values,
        "CONFLUENCE_MAVSDK_ADDRESS",
        "serial:///dev/ttyTHS3:115200",
        str,
    )
    timeout_sec = env_or_default(
        args.timeout,
        env_values,
        "CONFLUENCE_PARAM_TIMEOUT",
        5.0,
        float,
    )
    param = env_or_default(
        args.param,
        env_values,
        "CONFLUENCE_FAULT_PARAM",
        "PWM_MAIN_FUNC3",
        str,
    )
    value = env_or_default(
        args.value,
        env_values,
        "CONFLUENCE_FAULT_VALUE",
        0,
        int,
    )

    client = ParamInjectorClient(
        connection=connection,
        baud=baud,
        mavsdk_address=mavsdk_address,
        timeout_sec=timeout_sec,
        enable_mavsdk_warmup=not args.skip_mavsdk,
    )
    ok, actual, reason = client.set_and_verify(param, value)
    client.close()

    if not ok:
        print(
            f"Injection failed: param={param} expected={value} actual={actual} reason={reason}"
        )
        return 1
    print(f"Parameter write verified: {param}={actual}")
    return 0


def _run_remote(args: argparse.Namespace, env_values: dict[str, str]) -> int:
    host = env_or_default(args.host, env_values, "CONFLUENCE_CONSOLE_HOST", None, str)
    port = env_or_default(args.port, env_values, "CONFLUENCE_CONSOLE_PORT", None, int)
    if not host or not port:
        print("Remote mode requires --host and --port (or env defaults).")
        return 2
    if args.motor < 1 or args.motor > 4:
        print("motor must be between 1 and 4")
        return 2

    payload = {"cmd": "fault", "fault_index": args.motor}
    data = (json.dumps(payload) + "\n").encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((host, int(port)))
        sock.sendall(data)
    finally:
        try:
            sock.close()
        except Exception:
            pass

    print(f"Sent remote fault command: motor={args.motor} host={host} port={port}")
    return 0


def main() -> int:
    args = parse_args()
    env_values = load_env_file(args.env_file)
    if args.mode == "remote":
        return _run_remote(args, env_values)
    return _run_local(args, env_values)


if __name__ == "__main__":
    raise SystemExit(main())
