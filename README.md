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
colcon build --symlink-install --packages-select confluence
source install/setup.bash
```

## Run (separate terminals)
```bash
# Firehose (SITL)
ros2 run confluence firehose --sitl

# Uniform Pump
ros2 run confluence uniform_pump --ros-args -p condensation_mode:=multi_step -p multi_step_count:=3

# Fault Detector
ros2 run confluence fault_detector --ros-args -p model_path:=/path/to/model.ckpt

# Inject
ros2 run confluence inject --ros-args -p use_sitl:=true
```

## Run (single terminal with orchestrator)
```bash
# Start all services
ros2 run confluence orchestrator --all

# Start a subset
ros2 run confluence orchestrator --services firehose uniform_pump fault_detector

# Pass args to services
ros2 run confluence orchestrator --all \
  --firehose-args "--sitl --stream-rate 200" \
  --uniform-pump-args "--ros-args -p condensation_mode:=multi_step -p multi_step_count:=3" \
  --fault-detector-args "--ros-args -p model_path:=/path/to/model.ckpt"

# Combined output log
ros2 run confluence orchestrator --all --log-file /tmp/confluence_run.txt
```

Orchestrator interactive commands:
- `list`
- `firehose`, `uniform_pump`, `inject`, `fault_detector` (attach)
- `attach <name>`
- `help`
- `quit`

While attached: press `Ctrl-]` to detach, `Ctrl-C` to stop all services.

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
  "timestamp": 1710000000.0
}
```

## Torch on Jetson
The default `pip install torch` wheel is not compatible with Jetson. We need to use the NVIDIA Jetson PyTorch build.

## Notes
- `uniform_pump` maps MAVLink types to simplified labels (e.g., `ATTITUDE` -> `attitude`, `SERVO_OUTPUT_RAW` -> `servo_output`) so the fault detector can work with the waterfall_local feature layout.
- `orchestrator` already exists and can launch/attach to the four services from a single terminal (see “Run (single terminal with orchestrator)” above).
