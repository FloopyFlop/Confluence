#!/usr/bin/env python3
"""
One-shot parameter injection hook.

Runs the exact same injection path as `monolithic_fault`, but exposed as a
single hook command.

Examples:
  ros2 run confluence inject_param
  ros2 run confluence inject_param --param PWM_MAIN_FUNC4 --value 101
"""

from __future__ import annotations

import argparse

from confluence.param_injection_core import ParamInjectorClient, env_or_default, load_env_file


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="One-shot MAVLink parameter injection hook")
    parser.add_argument("--env-file", default=".drone-env", help="Path to .drone-env defaults file")
    parser.add_argument("--connection", default=None, help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=None, help="Serial baud")
    parser.add_argument("--mavsdk-address", default=None, help="MAVSDK warmup address")
    parser.add_argument("--skip-mavsdk", action="store_true", help="Skip MAVSDK warmup")
    parser.add_argument("--timeout", type=float, default=None, help="Timeout in seconds")
    parser.add_argument("--param", default=None, help="Parameter name")
    parser.add_argument("--value", type=int, default=None, help="INT32 parameter value")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    env_values = load_env_file(args.env_file)

    connection = env_or_default(
        args.connection,
        env_values,
        "CONFLUENCE_MAVLINK_CONNECTION",
        "/dev/ttyTHS3",
        str,
    )
    baud = env_or_default(args.baud, env_values, "CONFLUENCE_MAVLINK_BAUD", 115200, int)
    mavsdk_address = env_or_default(
        args.mavsdk_address,
        env_values,
        "CONFLUENCE_MAVSDK_ADDRESS",
        f"serial://{connection}:{baud}",
        str,
    )
    timeout = env_or_default(args.timeout, env_values, "CONFLUENCE_PARAM_TIMEOUT", 5.0, float)
    param = env_or_default(args.param, env_values, "CONFLUENCE_FAULT_PARAM", "PWM_MAIN_FUNC3", str)
    value = env_or_default(args.value, env_values, "CONFLUENCE_FAULT_VALUE", 0, int)

    client = ParamInjectorClient(
        connection=connection,
        baud=baud,
        mavsdk_address=mavsdk_address,
        timeout_sec=timeout,
        enable_mavsdk_warmup=not args.skip_mavsdk,
    )
    ok, actual, reason = client.set_and_verify(param, value)
    client.close()

    if not ok:
        print(
            f"FAILED param={param} expected={value} actual={actual} reason={reason}"
        )
        return 1
    print(f"OK param={param} value={actual}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
