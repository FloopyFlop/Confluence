# Confluence

ROS2 package for Waterfall microservices plus a clean, single injection path based on `waterfall_local/test.py`.

## What changed
- Parameter injection now uses one code path everywhere (`monolithic_fault` logic).
- Legacy injection bridge in `firehose` was removed.
- Legacy `verify_injection` hook was removed.
- `.drone-env` is now the default runtime config for serial/baud/MAVSDK settings.

## Nodes
- `firehose`: MAVLink ingest and JSON publish on `mav/all_messages`
- `uniform_pump`: batch/condense firehose data (default `condensation_mode=multi_step`, `batch_interval=0.2s`)
- `fault_detector`: model inference + forced fault publish + production auto-fix publish
- `inject`: applies `px4_injector/command` writes using the same logic as `monolithic_fault`
- `orchestrator`: starts/stops services and serves the remote console

## Build
```bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select confluence
source install/setup.bash
```

## Drone defaults (`.drone-env`)
File: `/Users/abm/XVOL/Cornell/MAGPIE/Confluence/.drone-env`

Current defaults:
- `CONFLUENCE_MAVLINK_CONNECTION=/dev/ttyTHS3`
- `CONFLUENCE_MAVLINK_BAUD=115200`
- `CONFLUENCE_MAVSDK_ADDRESS=serial:///dev/ttyTHS3:115200`
- `CONFLUENCE_PARAM_TIMEOUT=5.0`
- `CONFLUENCE_FAULT_PARAM=PWM_MAIN_FUNC3`
- `CONFLUENCE_FAULT_VALUE=0`
- `CONFLUENCE_FIREHOSE_ARGS="--serial-port /dev/ttyTHS3 --serial-baud 115200"`
- `CONFLUENCE_UNIFORM_PUMP_ARGS=""`
- `CONFLUENCE_INJECT_ARGS=""`
- `CONFLUENCE_FAULT_DETECTOR_ARGS=""`
- `CONFLUENCE_CONSOLE_HOST=0.0.0.0`
- `CONFLUENCE_CONSOLE_PORT=9000`

`monolithic_fault`, `inject`, `inject_param`, and `orchestrator` read this file automatically.

If you sync manually to the drone, also copy this file:
```bash
scp /Users/abm/XVOL/Cornell/MAGPIE/Confluence/.drone-env magpie@<DRONE_IP>:~/abm-sync/src/confluence/.drone-env
```

## One-shot fault induction (single process)
Minimal:
```bash
ros2 run confluence monolithic_fault
```

Equivalent explicit command (the one you validated):
```bash
ros2 run confluence monolithic_fault --connection /dev/ttyTHS3 --baud 115200 --mavsdk-address serial:///dev/ttyTHS3:115200
```

Hook form:
```bash
ros2 run confluence inject_param
ros2 run confluence inject_param --param PWM_MAIN_FUNC4 --value 101
```

Console fault hook form:
```bash
ros2 run confluence induce_fault --host <DRONE_IP> --port 9000 --motor 1
```

## Run microservices
Separate terminals:
```bash
ros2 run confluence firehose --serial-port /dev/ttyTHS3 --serial-baud 115200
ros2 run confluence uniform_pump
ros2 run confluence fault_detector
ros2 run confluence inject
```

Single terminal:
```bash
ros2 run confluence orchestrator --all
```

Override env defaults from CLI when needed:
```bash
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200 --stream-rate 200" \
  --console-port 9001
```

Important:
- If you do **not** run `inject`, forced faults are only software events (`fault_detector/output`) and do not write PX4 params.
- Production default fault profile is `disable_motor`:
  - fault 1 -> `PWM_MAIN_FUNC1=0`
  - fault 2 -> `PWM_MAIN_FUNC2=0`
  - fault 3 -> `PWM_MAIN_FUNC3=0`
  - fault 4 -> `PWM_MAIN_FUNC4=0`
- For demo-only forced-fault latch behavior, run:
  - `ros2 run confluence fault_detector --ros-args -p pause_inference_on_forced_fault:=true`

## Remote console
Run from laptop:
```bash
python3 /Users/abm/XVOL/Cornell/MAGPIE/Confluence/Console/console.py --host <DRONE_IP> --port 9000
```

In console:
```text
help
list
fault 1
clear
inject PWM_MAIN_FUNC3=0
quit
```

Advanced commands still available:
- `watch <topic> <type>`
- `unwatch <topic>`
- `pub <topic> <type> <json>`
- `send {json}`
