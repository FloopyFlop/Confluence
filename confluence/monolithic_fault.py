"""
Monolithic PX4 fault induction runner (replicates waterfall_local/test.py behavior).

This is intentionally independent from the existing Confluence microservices.
It can be launched via ROS2 as a single process:

    ros2 run confluence monolithic_fault

Default behavior mirrors waterfall_local/test.py:
1. Open an initial MAVSDK link on serial:///dev/ttyACM0:2000000 (best effort).
2. Open a pymavlink link on /dev/ttyACM0 at 2000000 baud.
3. Set PWM_MAIN_FUNC3 to 0 (INT32 encoding in MAVLink float field).
4. Read PWM_MAIN_FUNC3 back and print the confirmed value.
"""

from __future__ import annotations

import argparse
import struct
import time
from typing import Any

from pymavlink import mavutil


def encode_param_value(value: int | float, param_type: int) -> float:
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("f", struct.pack("i", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack("f", struct.pack("I", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack("f", struct.pack("h", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack("f", struct.pack("H", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack("f", struct.pack("b", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack("f", struct.pack("B", int(value)))[0]
    raise ValueError(f"Unsupported param type {param_type}")


def decode_param_value(msg: Any) -> int | float:
    t = msg.param_type
    v = msg.param_value

    if t == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return v
    if t == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("i", struct.pack("f", v))[0]
    if t == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack("I", struct.pack("f", v))[0]
    if t == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack("h", struct.pack("f", v))[0]
    if t == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack("H", struct.pack("f", v))[0]
    if t == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack("b", struct.pack("f", v))[0]
    if t == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack("B", struct.pack("f", v))[0]
    raise ValueError(f"Unsupported param type {t}")


def normalize_param_id(raw_param_id: Any) -> str:
    if isinstance(raw_param_id, bytes):
        return raw_param_id.decode("utf-8", errors="ignore").rstrip("\x00")
    return str(raw_param_id).rstrip("\x00")


def run_mavsdk_warmup(address: str) -> None:
    try:
        import asyncio
        from mavsdk import System  # type: ignore
    except Exception as exc:
        print(f"MAVSDK warmup skipped (mavsdk unavailable): {exc}")
        return

    async def _run() -> None:
        drone = System()
        print("Opening initial link over MAVSDK... [FIREHOSE]")
        await drone.connect(system_address=address)
        async for state in drone.core.connection_state():
            if state.is_connected:
                break
        print("Link established... [FIREHOSE]")

    try:
        asyncio.run(_run())
    except Exception as exc:
        print(f"MAVSDK warmup failed, continuing with pymavlink: {exc}")


def set_param(
    master: mavutil.mavfile,
    param_name: str,
    value: int,
    timeout_sec: float,
) -> None:
    hb = master.wait_heartbeat(timeout=timeout_sec)
    if hb is None:
        raise RuntimeError("No heartbeat received before PARAM_SET")
    print("Connected to system:", hb.get_srcSystem(), "component:", hb.get_srcComponent())

    param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    encoded_value = encode_param_value(value, param_type)

    master.mav.param_set_send(
        master.target_system,
        mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        param_name.encode("utf-8"),
        encoded_value,
        param_type,
    )
    # Keep behavior aligned with waterfall_local/test.py: set first, verify via explicit read.
    time.sleep(0.1)


def get_param(master: mavutil.mavfile, param_name: str, timeout_sec: float) -> int | float | None:
    master.wait_heartbeat(timeout=timeout_sec)
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode("utf-8"),
        -1,
    )

    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        remaining = max(0.0, deadline - time.time())
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=remaining)
        if msg is None:
            break

        if normalize_param_id(msg.param_id) != param_name:
            continue

        confirmed = decode_param_value(msg)
        print("Confirmed value:", confirmed)
        return confirmed

    print("No response")
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monolithic PX4 fault induction runner.")
    parser.add_argument(
        "--connection",
        default="/dev/ttyACM0",
        help="pymavlink connection string or serial device (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=2000000,
        help="Serial baud for pymavlink serial connections (default: 2000000)",
    )
    parser.add_argument(
        "--mavsdk-address",
        default="serial:///dev/ttyACM0:2000000",
        help="MAVSDK system address for initial warmup link.",
    )
    parser.add_argument(
        "--skip-mavsdk",
        action="store_true",
        help="Skip the MAVSDK warmup step.",
    )
    parser.add_argument(
        "--param",
        default="PWM_MAIN_FUNC3",
        help="PX4 parameter to set/read back (default: PWM_MAIN_FUNC3)",
    )
    parser.add_argument(
        "--value",
        type=int,
        default=0,
        help="INT32 value to set (default: 0)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Heartbeat/read timeout seconds (default: 5.0)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.skip_mavsdk:
        run_mavsdk_warmup(args.mavsdk_address)

    master = mavutil.mavlink_connection(args.connection, baud=args.baud)
    set_param(master, args.param, args.value, args.timeout)
    confirmed = get_param(master, args.param, args.timeout)

    if confirmed is None:
        return 1

    if int(confirmed) != int(args.value):
        print(
            f"Verification mismatch: wrote {args.value}, read back {confirmed}"
        )
        return 2

    print("Parameter write verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
