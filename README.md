# Confluence

ROS2 microservices for the Waterfall stack, rebuilt to use ROS2 pub/sub while preserving the waterfall_local logic flow.

## Nodes (microservices)
- `firehose`: Connects to PX4 via MAVLink and publishes raw MAVLink messages as JSON.
- `uniform_pump`: Batches firehose data into uniform time windows for AI/ML ingestion.
- `fault_detector`: Consumes uniform batches, runs a Torch model, publishes fault results.
- `inject`: Applies parameter updates via PX4 C API (if available) or MAVLink.
- `orchestrator`: Spawns and manages the four services from a single terminal.

## Topics
- Firehose publishes:
  - `mav/all_messages` (`std_msgs/String`, JSON payload with `_msg_type` and `_timestamp`)
- Uniform Pump subscribes:
  - `mav/all_messages`
- Uniform Pump publishes:
  - `mav/uniform_batch` (`std_msgs/String`, JSON payload with `data` + `_batch_metadata`)
- Fault Detector subscribes:
  - `mav/uniform_batch`
  - `fault_detector/command` (optional; for remote fault injection)
- Fault Detector publishes:
  - `fault_detector/output` (`std_msgs/String`, JSON fault payload)
  - Optionally `px4_injector/command` when `publish_inject_command:=true`
- Inject subscribes:
  - `px4_injector/command`
- Inject publishes:
  - `px4_injector/status`

## Build
```bash
cd /path/to/your/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --packages-select confluence
source install/setup.bash
```

## Run (separate terminals)
Hardware (recommended, on-drone):
```bash
# Firehose (hardware serial)
ros2 run confluence firehose --serial-port /dev/ttyTHS3 --serial-baud 115200

# Uniform Pump
ros2 run confluence uniform_pump --ros-args -p condensation_mode:=multi_step -p multi_step_count:=3

# Fault Detector
ros2 run confluence fault_detector --ros-args -p model_path:=/path/to/model.ckpt

# Inject (hardware serial)
ros2 run confluence inject --serial-port /dev/ttyTHS3 --serial-baud 115200 --ros-args -p use_sitl:=false
```
Note: Firehose and Inject cannot share the same serial device at the same time. If you need both
on hardware, use a MAVLink router or a UDP connection for Inject (e.g. via mavlink-router or PX4
telemetry over UDP) so only Firehose owns `/dev/ttyTHS3`.

SITL (optional, retained for debugging):
```bash
ros2 run confluence firehose --sitl
ros2 run confluence uniform_pump
ros2 run confluence fault_detector --ros-args -p model_path:=/path/to/model.ckpt
ros2 run confluence inject --ros-args -p use_sitl:=true
```

## Run (single terminal with orchestrator)
```bash
# Start all services (hardware serial)
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200" \
  --uniform-pump-args "--ros-args -p condensation_mode:=multi_step -p multi_step_count:=3"

# Start all services with inject over UDP (requires MAVLink routing to this UDP port)
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200 --ros-args -p enable_command_bridge:=false" \
  --uniform-pump-args "--ros-args -p condensation_mode:=multi_step -p multi_step_count:=3" \
  --inject-args "--connection udp:127.0.0.1:14560 --ros-args -p use_sitl:=false"

# Start a subset
ros2 run confluence orchestrator --services firehose uniform_pump fault_detector

# Pass args to services
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200 --stream-rate 200" \
  --uniform-pump-args "--ros-args -p condensation_mode:=multi_step -p multi_step_count:=3" \
  --fault-detector-args "--ros-args -p model_path:=/path/to/model.ckpt"

# Combined output log
ros2 run confluence orchestrator --all --log-file /tmp/confluence_run.txt

# With console enabled (see next section for remote console access)
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200 --ros-args -p enable_command_bridge:=false" \
  --uniform-pump-args "--ros-args -p condensation_mode:=multi_step -p multi_step_count:=3" \
  --inject-args "--connection udp:127.0.0.1:14560 --ros-args -p use_sitl:=false" \
  --console-host 0.0.0.0 --console-port 9000

# Run console
uv run Console/console.py --host 10.48.134.181 --port 9000
```
If you do not have MAVLink routed to UDP for `inject`, run the subset command above and omit `inject`.

Orchestrator interactive commands:
- `list`
- `firehose`, `uniform_pump`, `inject`, `fault_detector` (attach)
- `attach <name>`
- `help`
- `quit`
Note: `watch`, `fault`, `clear`, `inject` are Console commands, not orchestrator shell commands.

While attached: press `Ctrl-]` to detach, `Ctrl-C` to stop all services.

## Remote Console (off-drone)
Run the orchestrator on the drone with the console port enabled:
```bash
ros2 run confluence orchestrator --all \
  --firehose-args "--serial-port /dev/ttyTHS3 --serial-baud 115200 --ros-args -p enable_command_bridge:=false" \
  --uniform-pump-args "--ros-args -p condensation_mode:=multi_step -p multi_step_count:=3" \
  --inject-args "--connection udp:127.0.0.1:14560 --ros-args -p use_sitl:=false" \
  --console-host 0.0.0.0 --console-port 9000
```

Then on your laptop (same network), run:
```bash
python3 Console/console.py --host <DRONE_IP> --port 9000
```

Console commands:
- `list`
- `fault <1-4>` (force a fault output for demo)
- `clear` (clear forced fault and resume model inference)
- `inject PARAM=VALUE ...` (send parameter updates to `inject`)
- `watch <topic> <type>` (stream a ROS2 topic, e.g. `mav/uniform_batch std_msgs/msg/String`)
- `unwatch <topic>`
- `pub <topic> <type> <json>` (publish to a ROS2 topic)
- `send {json}` (send raw JSON to orchestrator console)
- `quit`
To verify inject writes from Console:
- `watch px4_injector/status std_msgs/msg/String` and check `results` for `true/false`.
- Check `details` for each param (`actual`, `reason`) to confirm readback verification.

One-shot verification hook (on the ROS machine):
```bash
ros2 run confluence verify_injection --param PWM_MAIN_FUNC4 --value 101 --timeout 5
```
Exit code `0` means verified readback, `1` means failed/timeout.

Example console session:
```bash
list
fault 2
inject PWM_MAIN_FUNC4=101
watch mav/uniform_batch std_msgs/msg/String
unwatch mav/uniform_batch
pub fault_detector/command std_msgs/msg/String {"data":"{\"action\":\"inject_fault\",\"fault_index\":1}"}
clear
quit
```
Behavior note:
- `fault <n>` now latches a forced fault and pauses model inference until `clear`.
- Output messages include `source: "forced"` or `source: "model"` to distinguish origin.
- Forced faults (`fault <n>`) publish to `fault_detector/output` and also send parameter updates to `inject` by default when `inject` is running.
- Model-driven injection is gated by `publish_inject_command` (default `false`).

## Hooks (one-off commands)
Hooks are small scripts for quick, single-purpose commands. They talk to the orchestrator console.

Induce a motor fault:
```bash
python3 confluence/hooks/induce_fault.py --host <DRONE_IP> --port 9000 --motor 1
```

This is equivalent to sending `fault 1` from the Console. It requires the orchestrator
to be running with `--console-port` enabled.

### Console Topic Streaming/Publishing
To stream or publish a topic from the Console, you must provide the ROS2 message type
in either `pkg/msg/Type` or `pkg.msg.Type` format (for example `std_msgs/msg/String`).
For `pub`, the JSON payload is mapped onto ROS message fields (nested dicts are supported).

## Inject Command Format
Publish JSON on `px4_injector/command`:
```bash
ros2 topic pub /px4_injector/command std_msgs/String "{data: '{\"action\": \"set_params\", \"params\": {\"PWM_MAIN_FUNC4\": 101}}'}"
```

## Fault Detector Output Format
`fault_detector/output` publishes JSON like:
```json
{
  "fault": true,
  "fault_label": "motor 2 malfunction",
  "parameters": {"PWM_MAIN_FUNC1": 102},
  "timestamp": 1710000000.0,
  "source": "model"
}
```
Important:
- A forced fault event is a software signal; it does not physically stop a motor by itself.
- Physical effect depends on `inject` being connected and on whether the target PX4 parameter applies immediately for your airframe/output mapping.
- In the original `waterfall_local` flow, parameter reconfiguration happened after flight; on many PX4 setups, `PWM_MAIN_FUNC*` updates are only fully realized after disarm/re-arm (or reboot).
- For dry-run forced faults without writing params, use:
  `send {"cmd":"fault","fault_index":1,"apply_inject":false}`
- `px4_injector/command` can be handled by `firehose` (direct serial path) or `inject` service (separate MAVLink link). Do not leave both enabled unless you want duplicate writes.

## Torch on Jetson
The default `pip install torch` wheel is not compatible with Jetson. We need to use the NVIDIA Jetson PyTorch build.

## Notes
- `uniform_pump` maps MAVLink types to simplified labels (e.g., `ATTITUDE` -> `attitude`, `SERVO_OUTPUT_RAW` -> `servo_output`) so the fault detector can work with the waterfall_local feature layout.
- `orchestrator` already exists and can launch/attach to the four services from a single terminal (see “Run (single terminal with orchestrator)” above).
