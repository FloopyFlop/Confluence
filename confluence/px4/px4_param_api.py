"""
PX4 Parameter API - Python wrapper for direct C API access

This module provides a Python interface to PX4's parameter system via ctypes,
bypassing the MAVLink protocol entirely for lowest-level access.
"""

import ctypes
import os
import sys
from pathlib import Path
from typing import Optional, Union


class PX4ParamType:
    """PX4 parameter types"""
    INT32 = 0
    FLOAT = 1


class PX4ParamAPI:
    """
    Direct Python interface to PX4's C parameter API

    This class loads the px4_param_bridge shared library and provides
    Python methods to get/set parameters at the lowest level.
    """

    def __init__(self, library_path: Optional[str] = None):
        """
        Initialize the PX4 Parameter API

        Args:
            library_path: Path to libpx4_param_bridge.so/.dylib
                         If None, will search common locations
        """
        self.lib = None
        self._load_library(library_path)
        self._setup_function_signatures()

    def _load_library(self, library_path: Optional[str]):
        """Load the px4_param_bridge shared library"""

        # Try to find library if not specified
        if library_path is None:
            library_path = self._find_library()

        if library_path is None:
            raise RuntimeError(
                "Could not find libpx4_param_bridge library. "
                "Please build it first with build_bridge.sh or set "
                "PX4_PARAM_BRIDGE_LIB environment variable"
            )

        library_path = Path(library_path)
        if not library_path.exists():
            raise FileNotFoundError(f"Library not found: {library_path}")

        print(f"Loading PX4 parameter bridge library: {library_path}")

        try:
            self.lib = ctypes.CDLL(str(library_path))
            print("Successfully loaded PX4 parameter bridge library")
        except Exception as e:
            raise RuntimeError(f"Failed to load library {library_path}: {e}")

    def _find_library(self) -> Optional[str]:
        """Try to find the library in common locations"""

        # Check environment variable first
        if 'PX4_PARAM_BRIDGE_LIB' in os.environ:
            return os.environ['PX4_PARAM_BRIDGE_LIB']

        # Common locations to search
        search_paths = [
            Path(__file__).parent / 'build' / 'libpx4_param_bridge.so',
            Path(__file__).parent / 'build' / 'libpx4_param_bridge.dylib',
            Path.cwd() / 'libpx4_param_bridge.so',
            Path.cwd() / 'libpx4_param_bridge.dylib',
            Path.home() / 'libpx4_param_bridge.so',
            Path.home() / 'libpx4_param_bridge.dylib',
        ]

        for path in search_paths:
            if path.exists():
                return str(path)

        return None

    def _setup_function_signatures(self):
        """Setup ctypes function signatures for C library functions"""

        if self.lib is None:
            raise RuntimeError("Library not loaded")

        # px4_param_get_float(const char *name, float *value) -> int
        self.lib.px4_param_get_float.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)]
        self.lib.px4_param_get_float.restype = ctypes.c_int

        # px4_param_set_float(const char *name, float value) -> int
        self.lib.px4_param_set_float.argtypes = [ctypes.c_char_p, ctypes.c_float]
        self.lib.px4_param_set_float.restype = ctypes.c_int

        # px4_param_get_int32(const char *name, int32_t *value) -> int
        self.lib.px4_param_get_int32.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int32)]
        self.lib.px4_param_get_int32.restype = ctypes.c_int

        # px4_param_set_int32(const char *name, int32_t value) -> int
        self.lib.px4_param_set_int32.argtypes = [ctypes.c_char_p, ctypes.c_int32]
        self.lib.px4_param_set_int32.restype = ctypes.c_int

        # px4_param_get_type(const char *name) -> int
        self.lib.px4_param_get_type.argtypes = [ctypes.c_char_p]
        self.lib.px4_param_get_type.restype = ctypes.c_int

        print("Function signatures configured")

    def get_param_type(self, name: str) -> Optional[int]:
        """
        Get parameter type

        Args:
            name: Parameter name

        Returns:
            PX4ParamType.INT32 or PX4ParamType.FLOAT, or None on error
        """
        param_type = self.lib.px4_param_get_type(name.encode('utf-8'))
        if param_type < 0:
            return None
        return param_type

    def get_param(self, name: str) -> Optional[Union[float, int]]:
        """
        Get parameter value (auto-detects type)

        Args:
            name: Parameter name

        Returns:
            Parameter value (float or int), or None on error
        """
        param_type = self.get_param_type(name)
        if param_type is None:
            return None

        if param_type == PX4ParamType.FLOAT:
            return self.get_param_float(name)
        elif param_type == PX4ParamType.INT32:
            return self.get_param_int32(name)
        else:
            print(f"Unknown parameter type: {param_type}")
            return None

    def get_param_float(self, name: str) -> Optional[float]:
        """
        Get float parameter value

        Args:
            name: Parameter name

        Returns:
            Parameter value or None on error
        """
        value = ctypes.c_float()
        ret = self.lib.px4_param_get_float(name.encode('utf-8'), ctypes.byref(value))
        if ret != 0:
            return None
        return value.value

    def get_param_int32(self, name: str) -> Optional[int]:
        """
        Get int32 parameter value

        Args:
            name: Parameter name

        Returns:
            Parameter value or None on error
        """
        value = ctypes.c_int32()
        ret = self.lib.px4_param_get_int32(name.encode('utf-8'), ctypes.byref(value))
        if ret != 0:
            return None
        return value.value

    def set_param(self, name: str, value: Union[float, int]) -> bool:
        """
        Set parameter value (auto-detects type)

        Args:
            name: Parameter name
            value: New value

        Returns:
            True on success, False on error
        """
        param_type = self.get_param_type(name)
        if param_type is None:
            print(f"Cannot determine type for parameter: {name}")
            return False

        if param_type == PX4ParamType.FLOAT:
            return self.set_param_float(name, float(value))
        elif param_type == PX4ParamType.INT32:
            return self.set_param_int32(name, int(value))
        else:
            print(f"Unknown parameter type: {param_type}")
            return False

    def set_param_float(self, name: str, value: float) -> bool:
        """
        Set float parameter value

        Args:
            name: Parameter name
            value: New value

        Returns:
            True on success, False on error
        """
        ret = self.lib.px4_param_set_float(name.encode('utf-8'), ctypes.c_float(value))
        return ret == 0

    def set_param_int32(self, name: str, value: int) -> bool:
        """
        Set int32 parameter value

        Args:
            name: Parameter name
            value: New value

        Returns:
            True on success, False on error
        """
        ret = self.lib.px4_param_set_int32(name.encode('utf-8'), ctypes.c_int32(value))
        return ret == 0


# Example usage
if __name__ == '__main__':
    print("PX4 Parameter API Test")
    print("=" * 60)

    try:
        api = PX4ParamAPI()

        # Test reading a parameter
        test_param = "MC_ROLL_P"
        print(f"\nReading parameter: {test_param}")

        param_type = api.get_param_type(test_param)
        if param_type is not None:
            print(f"  Type: {param_type} ({'FLOAT' if param_type == PX4ParamType.FLOAT else 'INT32'})")

            value = api.get_param(test_param)
            if value is not None:
                print(f"  Current value: {value}")

                # Test setting
                new_value = value + 1.0
                print(f"\nSetting {test_param} = {new_value}")
                success = api.set_param(test_param, new_value)
                if success:
                    print("  Successfully set parameter")

                    # Read back
                    read_value = api.get_param(test_param)
                    print(f"  Read back value: {read_value}")
                else:
                    print("  Failed to set parameter")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
