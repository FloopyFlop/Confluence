# Confluence

ROS2 package for drone runtime with four services and one launcher:

- `firehose`: MAVLink ingest + parameter write executor (`px4_injector/command` -> `px4_injector/status`)
- `uniform_pump`: message batching/condensation
- `fault_detector`: model inference + forced fault/clear control
- `inject`: direct standalone param injector (`px4_injector/direct_command`)
- `orchestrator`: starts services and hosts TCP console bridge

## Build

```bash
cd /path/to/ros2_ws
colcon build --symlink-install --packages-select confluence
source install/setup.bash
```

## Runtime Defaults (`.drone-env`)

`orchestrator`, `inject`, and `fault_hook` read `.drone-env`.

Default lookup for relative `.drone-env`:

- current working directory
- `<workspace>/src/confluence/.drone-env`

Important keys:

- `CONFLUENCE_FIREHOSE_ARGS`
- `CONFLUENCE_UNIFORM_PUMP_ARGS`
- `CONFLUENCE_INJECT_ARGS`
- `CONFLUENCE_FAULT_DETECTOR_ARGS`
- `CONFLUENCE_CONSOLE_HOST`
- `CONFLUENCE_CONSOLE_PORT`
- `CONFLUENCE_CONSOLE_STREAM_LOGS`
- `CONFLUENCE_MAVLINK_CONNECTION`
- `CONFLUENCE_MAVLINK_BAUD`
- `CONFLUENCE_MAVSDK_ADDRESS`
- `CONFLUENCE_PARAM_TIMEOUT`

Current defaults in `/Users/abm/XVOL/Cornell/MAGPIE/Confluence/.drone-env` already include:

- firehose serial args
- detector args in waterfall-compatible mode:
- `require_flight_gate:=true` (armed + throttle-up required for inference)
- `hold_model_fault_until_landing:=true` (latch first model fault during flight)
- `auto_apply_model_fix_after_landing:=true` (apply fix only after landing/disarm gate closes)
- `model_confidence_threshold:=0.0`, `model_vote_window:=1`, `model_vote_required:=1` (raw argmax behavior)
- console host/port (`0.0.0.0:9000`)
- console service-log stream disabled by default (`CONFLUENCE_CONSOLE_STREAM_LOGS=false`)

## Start Stack

```bash
ros2 run confluence orchestrator --all
```

Console bridge starts by default from `.drone-env`. Disable it only if needed:

```bash
ros2 run confluence orchestrator --all --console-port 0
```

Enable full service log streaming into remote console (off by default):

```bash
ros2 run confluence orchestrator --all --console-stream-logs
```

Override runtime args when needed:

```bash
ros2 run confluence orchestrator --all \
  --fault-detector-args "--ros-args -p publish_inject_command:=false -p model_confidence_threshold:=0.80 -p model_vote_window:=9 -p model_vote_required:=5"
```

## Remote Console

From laptop:

```bash
uv run Console/console.py --host <DRONE_IP> --port 9000
```

If running locally on same machine as orchestrator:

```bash
uv run Console/console.py
```

Console commands:

- `help`
- `list`
- `probe`
- `fault <1-4>`
- `clear`
- `clear no-restore`
- `clear now`
- `inject PARAM=VALUE ...`
- `motortest on|off`
- `watch <topic> <type>`
- `unwatch <topic>`
- `pub <topic> <type> <json>`
- `send {json}`

## Manual Fault/Fix Flow

1. Start stack: `ros2 run confluence orchestrator --all`
2. Connect console: `uv run Console/console.py --host <DRONE_IP> --port 9000`
3. Watch status:

```text
watch fault_detector/output std_msgs/msg/String
watch fault_detector/status std_msgs/msg/String
watch fault_detector/diagnostics std_msgs/msg/String
watch px4_injector/status std_msgs/msg/String
```

4. Inject fault: `fault 1`
5. Clear + restore: `clear`

Notes:

- `clear` restores defaults (`PWM_MAIN_FUNC1..4`) using detector config.
- Restore is deferred while armed by default (`defer_until_disarmed=true`).
- `clear now` forces immediate restore request (bench/debug only).
- Model inference only runs when flight gate is open (armed + throttle-up).
- Flight gate is driven from raw `mav/all_messages` (`HEARTBEAT` + `RC_CHANNELS`) with batch fallback only if raw gate updates stall.
- Model faults are latched in-flight and fixed after landing (waterfall behavior).
- Detector does not command landing; pilot/autopilot must land or disarm, then fix is applied.
- Detector auto-aligns model timesteps to available batch depth (so `multi_step_count=3` feeds true 3-step inputs).
- Model fix mapping matches waterfall_lite:
- motor 1 -> `PWM_MAIN_FUNC4=101`
- motor 2 -> `PWM_MAIN_FUNC1=102`
- motor 3 -> `PWM_MAIN_FUNC2=103`
- motor 4 -> `PWM_MAIN_FUNC3=104`

## Fault Detector Probe Hook

Run with orchestrator/services already running.

```bash
ros2 run confluence fault_detector_probe
```

Probe checks:

- detector has received batches
- detector has run inference (requires flight gate open, so drone must be armed + throttle-up)
- expected feature vector length reached
- forced fault publish path works
- forced clear publish path works
- injector status observed (unless disabled)
- detector status topic observed

Useful options:

```bash
ros2 run confluence fault_detector_probe --probe-only
ros2 run confluence fault_detector_probe --fault-index 3 --timeout 25
ros2 run confluence fault_detector_probe --no-inject-status-check
```

## Fault Hook

Unified hook entrypoint:

```bash
ros2 run confluence fault_hook
```

Remote forced fault command:

```bash
ros2 run confluence fault_hook --mode remote --host <DRONE_IP> --port 9000 --motor 1
```

Legacy alias (kept for compatibility):

```bash
ros2 run confluence monolithic_fault --connection /dev/ttyTHS3 --baud 115200 --mavsdk-address serial:///dev/ttyTHS3:115200
```

## QGroundControl Note

If motor behavior is overridden by actuator test mode, disable it:

```text
motortest off
```

Equivalent param command:

```text
inject COM_MOT_TEST_EN=0
```

## Troubleshooting

- `ConnectionRefusedError` from console:
  - orchestrator not running, wrong host, or wrong port.
  - verify orchestrator prints `Console server listening on ...`.
- Repeated `device disconnected or multiple access on port?` from firehose:
  - another process is contending for the same MAVLink serial endpoint.
  - close competing serial consumers or move one side to UDP.
- Detector publishes faults but no physical change:
  - check `px4_injector/status` details for each param (`ok`, `reason`, `actual`).
  - if `queued=true`, command is deferred until disarm.
- Detector never reports model faults:
  - check startup line shows `Model stabilization: confidence>=0.0 votes=1/1`.
  - if not, `.drone-env` was not loaded; start from workspace root or pass `--env-file /path/to/.drone-env`.
  - verify gate opens in flight (`Flight gate state: open=True ...`) after arm + throttle-up.
- Console says `fault_detector is not running`:
  - detector process crashed or exited; restart stack and inspect `fault_detector` logs first.
