"""
Shared MAVLink parameter injection utilities.

This module centralizes the exact PARAM_SET + PARAM_REQUEST_READ flow used by
`fault_hook` so both one-shot hooks and ROS services use the same logic.
"""

from __future__ import annotations

import struct
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil


DEFAULT_ENV_FILE = ".drone-env"


def load_env_file(path: str | None = None) -> dict[str, str]:
    env_path = Path(path or DEFAULT_ENV_FILE)
    if not env_path.exists():
        return {}

    values: dict[str, str] = {}
    for raw in env_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            values[key] = value
    return values


def env_or_default(
    explicit: Any,
    env_values: dict[str, str],
    key: str,
    default: Any,
    cast=None,
):
    if explicit is not None:
        return explicit
    if key in env_values:
        raw = env_values[key]
        if cast is None:
            return raw
        try:
            return cast(raw)
        except Exception:
            return default
    return default


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


def _is_float_value(value: Any) -> bool:
    return isinstance(value, float)


class ParamInjectorClient:
    def __init__(
        self,
        connection: str,
        baud: int,
        mavsdk_address: str,
        timeout_sec: float = 5.0,
        enable_mavsdk_warmup: bool = True,
    ) -> None:
        self.connection = connection
        self.baud = int(baud)
        self.mavsdk_address = mavsdk_address
        self.timeout_sec = float(timeout_sec)
        self.enable_mavsdk_warmup = bool(enable_mavsdk_warmup)
        self.master: mavutil.mavfile | None = None

    def _run_mavsdk_warmup(self) -> None:
        if not self.enable_mavsdk_warmup:
            return
        try:
            import asyncio
            from mavsdk import System  # type: ignore
        except Exception:
            return

        async def _run() -> None:
            drone = System()
            await drone.connect(system_address=self.mavsdk_address)
            async for state in drone.core.connection_state():
                if state.is_connected:
                    break

        try:
            asyncio.run(_run())
        except Exception:
            pass

    def connect(self) -> None:
        if self.master is not None:
            return
        self._run_mavsdk_warmup()
        self.master = mavutil.mavlink_connection(self.connection, baud=self.baud)
        heartbeat = self.master.wait_heartbeat(timeout=self.timeout_sec)
        if heartbeat is None:
            raise RuntimeError("No heartbeat received from FC")

    def close(self) -> None:
        if self.master is None:
            return
        try:
            self.master.close()
        except Exception:
            pass
        self.master = None

    def set_param(self, param_name: str, value: int | float) -> None:
        if self.master is None:
            raise RuntimeError("Not connected")
        if _is_float_value(value):
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        else:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        encoded_value = encode_param_value(value, param_type)
        self.master.mav.param_set_send(
            self.master.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            param_name.encode("utf-8"),
            encoded_value,
            param_type,
        )
        time.sleep(0.1)

    def get_param(self, param_name: str) -> int | float | None:
        if self.master is None:
            raise RuntimeError("Not connected")
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode("utf-8"),
            -1,
        )
        deadline = time.time() + self.timeout_sec
        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=remaining)
            if msg is None:
                return None
            if normalize_param_id(msg.param_id) != param_name:
                continue
            return decode_param_value(msg)
        return None

    def set_and_verify(self, param_name: str, value: int | float) -> tuple[bool, int | float | None, str | None]:
        try:
            self.connect()
            self.set_param(param_name, value)
            actual = self.get_param(param_name)
            if actual is None:
                return False, None, "no_response"
            if _is_float_value(value):
                ok = abs(float(actual) - float(value)) < 1e-4
            else:
                ok = int(actual) == int(value)
            if ok:
                return True, actual, None
            return False, actual, "mismatch"
        except Exception as exc:
            return False, None, str(exc)
