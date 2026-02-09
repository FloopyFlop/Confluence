"""
Monolithic PX4 fault induction runner (replicates waterfall_local/test.py behavior).

This is intentionally independent from the existing Confluence microservices.
It can be launched via ROS2 as a single process:

    ros2 run confluence monolithic_fault

Default behavior mirrors waterfall_local/test.py:
1. Open an initial MAVSDK link (best effort).
2. Open a pymavlink link.
3. Set PWM_MAIN_FUNC3 to 0 (INT32 encoding in MAVLink float field).
4. Read PWM_MAIN_FUNC3 back and print the confirmed value.
"""

from __future__ import annotations

import argparse

from confluence.utils.param_injection import ParamInjectorClient, env_or_default, load_env_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monolithic PX4 fault induction runner.")
    parser.add_argument(
        "--env-file",
        default=".drone-env",
        help="Path to environment defaults file (default: .drone-env)",
    )
    parser.add_argument(
        "--connection",
        default=None,
        help="pymavlink connection string or serial device",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=None,
        help="Serial baud for pymavlink serial connections",
    )
    parser.add_argument(
        "--mavsdk-address",
        default=None,
        help="MAVSDK system address for warmup link",
    )
    parser.add_argument(
        "--skip-mavsdk",
        action="store_true",
        help="Skip the MAVSDK warmup step.",
    )
    parser.add_argument(
        "--param",
        default=None,
        help="PX4 parameter to set/read back",
    )
    parser.add_argument(
        "--value",
        type=int,
        default=None,
        help="INT32 value to set",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Heartbeat/read timeout seconds",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    env_values = load_env_file(args.env_file)

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


if __name__ == "__main__":
    raise SystemExit(main())
