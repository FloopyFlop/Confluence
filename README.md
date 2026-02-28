# Confluence

ROS2 fault-injection + detection stack for PX4.

Full technical reference: `/Users/abm/XVOL/Cornell/MAGPIE/Confluence/knowledgebase.md`

## Repository Layout

- `Device/`: ROS2 package (`package.xml`, `setup.py`, `confluence/*` runtime nodes)
- `Console/`: TCP remote CLI client (`console.py`)
- `UI/`: read-only demo dashboard (live status visualization)
- `knowledgebase.md`: full payload/contract/runtime reference

## Build

From this repo root:

```bash
colcon build --symlink-install --paths Device --packages-select confluence
source install/setup.bash
```

## Run

Start full stack (recommended with explicit env file now that `.drone-env` lives under `Device/`):

```bash
ros2 run confluence orchestrator --all --env-file Device/.drone-env
```

Optional:
- Disable console bridge: `ros2 run confluence orchestrator --all --console-port 0 --env-file Device/.drone-env`
- Enable service-log streaming to remote console: `ros2 run confluence orchestrator --all --console-stream-logs --env-file Device/.drone-env`

## Remote Console

From laptop:

```bash
uv run Console/console.py --host <DRONE_IP> --port 9000
```

Rich-formatted console (recommended):

```bash
uv run --with rich Console/console.py --host <DRONE_IP> --port 9000
```

Local (same machine as orchestrator):

```bash
uv run Console/console.py
```

Supported commands:
- `help`
- `list`
- `probe`
- `fault <1-4>`
- `clear`
- `clear no-restore`
- `clear now`
- `clear now no-restore`
- `inject PARAM=VALUE ...`
- `watch <topic> <type>`
- `unwatch <topic>`
- `pub <topic> <type> <json>`
- `quit`

## Demo UI (Read-only)

Build frontend assets (npm):

```bash
cd UI
npm install
npm run build
cd ..
```

Launch the dashboard bridge + web UI:

```bash
uv run UI/dashboard_server.py --orchestrator-host <DRONE_IP> --orchestrator-port 9000 --ui-host 127.0.0.1 --ui-port 8765
```

Open:

```text
http://127.0.0.1:8765
```

UI behavior:
- 3-column display (`Status`, `Details`, `Telemetry & Log`)
- status machine: `Operational` -> `Fault Detected` -> `Fault Fixing`
- `Fault Fixing` stage is intentionally stretched to 5 seconds for demos
- data source is live orchestrator stream only (no synthetic fallback)
- frontend is npm-built (`Vite`) and served locally by `UI/dashboard_server.py`
- theme toggle in header supports light/dark mode

## Standard Flight Test Flow

1. Start stack.
2. Connect console.
3. Watch outputs:

```text
watch fault_detector/output std_msgs/msg/String
watch fault_detector/status std_msgs/msg/String
watch fault_detector/diagnostics std_msgs/msg/String
watch px4_injector/status std_msgs/msg/String
```

4. Inject fault: `fault 1`
5. Clear fault: `clear`

Behavior:
- Inference runs only when flight gate is open (armed + throttle-up).
- Model fault is latched in flight and fixed after landing / gate close.
- `clear` restores motor params by default and defers restore if armed.

Model fix map:
- motor 1 -> `PWM_MAIN_FUNC4=101`
- motor 2 -> `PWM_MAIN_FUNC1=102`
- motor 3 -> `PWM_MAIN_FUNC2=103`
- motor 4 -> `PWM_MAIN_FUNC3=104`

Forced fault map:
- `fault 1` -> `PWM_MAIN_FUNC4=0`
- `fault 2` -> `PWM_MAIN_FUNC1=0`
- `fault 3` -> `PWM_MAIN_FUNC2=0`
- `fault 4` -> `PWM_MAIN_FUNC3=0`

## Runtime Defaults (`Device/.drone-env`)

Loaded by `orchestrator`, `inject`, and `fault_hook`.

Main keys:
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

## Hooks

Fault-detector integration probe:

```bash
ros2 run confluence fault_detector_probe
```

One-shot local/remote fault hook:

```bash
ros2 run confluence fault_hook
ros2 run confluence fault_hook --mode remote --host <DRONE_IP> --port 9000 --motor 1
```

Backward-compatible alias:

```bash
ros2 run confluence monolithic_fault --connection /dev/ttyTHS3 --baud 115200 --mavsdk-address serial:///dev/ttyTHS3:115200
```

## Troubleshooting

- Console `ConnectionRefusedError`: orchestrator not running, wrong host, or wrong port.
- Repeated firehose serial-read warnings: another process is using the same MAVLink serial device.
- Fault published but no hardware change: inspect `px4_injector/status` (`ok`, `actual`, `reason`, `queued`).
- No model detections: verify startup shows `confidence>=0.0 votes=1/1` and confirm gate opens in flight.
- UI shows disconnected: confirm orchestrator TCP bridge is enabled on expected host/port.
