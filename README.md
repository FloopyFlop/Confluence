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
watch px4_injector/status std_msgs/msg/String
fault 1
clear
# optional: clear without restore
send {"cmd":"clear_fault","restore":false}
```

Important:
- `fault <n>` publishes a forced fault event and injects fault params by default.
- `clear` clears forced-fault state in the detector.
- `clear` restores default motor params (`PWM_MAIN_FUNC1..4`) by default.
- Parameter writes from console/fault detector are executed by `firehose` on `px4_injector/command` so only one service owns the MAVLink serial link.

Manual restore command template:
```text
inject PWM_MAIN_FUNC1=101 PWM_MAIN_FUNC2=102 PWM_MAIN_FUNC3=103 PWM_MAIN_FUNC4=104
```

Verification:
- Watch `px4_injector/status` and confirm each param has `"ok": true`.
- If you see `"ok": false` with `"reason": "no_response"` or `"reason": "not_connected"`, the FC write did not apply.

QGC actuator test mode:
- If QGroundControl "Actuator sliders are enabled", motor test commands can override normal motor behavior.
- Disable from Confluence console:
```text
inject COM_MOT_TEST_EN=0
# or shorthand
motortest off
```
- Re-enable actuator test mode:
```text
inject COM_MOT_TEST_EN=1
# or shorthand
motortest on
```

## Standard Commands
Start stack (from `.drone-env` defaults):
```bash
ros2 run confluence orchestrator --all
```

Note:
- `orchestrator` reads `.drone-env` from your current working directory.
- If your env file is elsewhere, pass it explicitly: `--env-file /path/to/.drone-env`
- Console bridge starts by default on `0.0.0.0:9000` (or `CONFLUENCE_CONSOLE_HOST/PORT`).
- Disable console bridge with `--console-port 0`.

Override any defaults:
```bash
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200" \
  --console-port 9001
```

One-shot local fault hook (param write + readback verify):
```bash
ros2 run confluence fault_hook
```

Remote fault hook (sends `fault <motor>` to orchestrator console):
```bash
ros2 run confluence fault_hook --mode remote --host <DRONE_IP> --port 9000 --motor 1
```

## Service Nodes
- `firehose`: publishes MAVLink JSON on `mav/all_messages`, applies `px4_injector/command`, publishes `px4_injector/status`
- `uniform_pump`: publishes batched data on `mav/uniform_batch`
- `fault_detector`: publishes faults on `fault_detector/output`
- `inject`: standalone direct injector on `px4_injector/direct_command` and `px4_injector/direct_status`
- `orchestrator`: launches services and provides remote console bridge

## `.drone-env`
Path:
- `/Users/abm/XVOL/Cornell/MAGPIE/Confluence/.drone-env`

Used by:
- `fault_hook`
- `inject`
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
