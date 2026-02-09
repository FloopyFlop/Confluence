# Confluence

ROS2 package for MAVLink ingest, batching, fault detection, and parameter injection.

## Quick Start
```bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select confluence
source install/setup.bash
ros2 run confluence orchestrator --all
```

## Manual Fault Demo (Recommended)
On laptop:
```bash
python3 /Users/abm/XVOL/Cornell/MAGPIE/Confluence/Console/console.py --host <DRONE_IP> --port 9000
```

In console:
```text
help
list
fault 1
clear
```

Important:
- `fault <n>` publishes a forced fault event and injects fault params by default.
- `clear` clears forced-fault state in the detector.
- `clear` does not automatically restore PWM params.

Restore command template:
```text
inject PWM_MAIN_FUNC1=101 PWM_MAIN_FUNC2=102 PWM_MAIN_FUNC3=103 PWM_MAIN_FUNC4=104
```

## Standard Commands
Start stack (from `.drone-env` defaults):
```bash
ros2 run confluence orchestrator --all
```

Override any defaults:
```bash
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200" \
  --console-port 9001
```

One-shot monolithic injection:
```bash
ros2 run confluence monolithic_fault
```

One-shot param hook:
```bash
ros2 run confluence inject_param --param PWM_MAIN_FUNC4 --value 101
```

Console fault hook:
```bash
ros2 run confluence induce_fault --host <DRONE_IP> --port 9000 --motor 1
```

## Service Nodes
- `firehose`: publishes MAVLink JSON on `mav/all_messages`
- `uniform_pump`: publishes batched data on `mav/uniform_batch`
- `fault_detector`: publishes faults on `fault_detector/output`
- `inject`: applies `px4_injector/command` and publishes `px4_injector/status`
- `orchestrator`: launches services and provides remote console bridge

## `.drone-env`
Path:
- `/Users/abm/XVOL/Cornell/MAGPIE/Confluence/.drone-env`

Used by:
- `monolithic_fault`
- `inject`
- `inject_param`
- `orchestrator`

Current keys:
- `CONFLUENCE_MAVLINK_CONNECTION`
- `CONFLUENCE_MAVLINK_BAUD`
- `CONFLUENCE_MAVSDK_ADDRESS`
- `CONFLUENCE_PARAM_TIMEOUT`
- `CONFLUENCE_FAULT_PARAM`
- `CONFLUENCE_FAULT_VALUE`
- `CONFLUENCE_FIREHOSE_ARGS`
- `CONFLUENCE_UNIFORM_PUMP_ARGS`
- `CONFLUENCE_INJECT_ARGS`
- `CONFLUENCE_FAULT_DETECTOR_ARGS`
- `CONFLUENCE_CONSOLE_HOST`
- `CONFLUENCE_CONSOLE_PORT`

Sync file to drone if needed:
```bash
scp /Users/abm/XVOL/Cornell/MAGPIE/Confluence/.drone-env magpie@<DRONE_IP>:~/abm-sync/src/confluence/.drone-env
```

## Auto Injection Mode
Default behavior is manual-safe: model output does not auto-write params.

Enable model-driven auto injection:
```bash
ros2 run confluence fault_detector --ros-args -p publish_inject_command:=true
```
