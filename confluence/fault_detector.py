#!/usr/bin/env python3
"""
ROS2 Fault Detector node.
Consumes uniform pump batches, runs a Torch model, and publishes fault results.
"""

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import torch
    TORCH_AVAILABLE = True
except Exception:
    TORCH_AVAILABLE = False

try:
    from confluence.models import NN_classifier
    MODEL_AVAILABLE = True
except Exception:
    MODEL_AVAILABLE = False


class FaultDetectorNode(Node):
    def __init__(self):
        super().__init__('fault_detector_node')

        self.declare_parameter('model_path', 'models/classifier_3ts.ckpt')
        self.declare_parameter('input_topic', 'mav/uniform_batch')
        self.declare_parameter('output_topic', 'fault_detector/output')
        self.declare_parameter('publish_inject_command', False)
        self.declare_parameter('inject_topic', 'px4_injector/command')
        self.declare_parameter('command_topic', 'fault_detector/command')

        self.model_path = str(self.get_parameter('model_path').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.inject_topic = self.get_parameter('inject_topic').value
        self.publish_inject_command = bool(self.get_parameter('publish_inject_command').value)
        self.command_topic = self.get_parameter('command_topic').value

        self._configure_model_spec()
        self.model = self._load_model()

        self.subscription = self.create_subscription(
            String,
            self.input_topic,
            self._batch_callback,
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._command_callback,
            10,
        )
        self.result_pub = self.create_publisher(String, self.output_topic, 10)
        self.inject_pub = None
        if self.publish_inject_command:
            self.inject_pub = self.create_publisher(String, self.inject_topic, 10)

        self.get_logger().info('Fault Detector started')

    def _configure_model_spec(self):
        if '3ts' in self.model_path:
            self.labels = ['attitude', 'attitude_quat', 'rc_channels', 'servo_output', 'timestamps']
            self.idxs = [2, 3, 4]
            self.input_layer = 81
        else:
            self.labels = ['attitude', 'attitude_quat', 'rc_channels', 'servo_output']
            self.idxs = [4]
            self.input_layer = 26

    def _resolve_model_path(self):
        model_path = Path(self.model_path)
        if model_path.is_absolute():
            return model_path
        # Try relative to package directory
        base_dir = Path(__file__).resolve().parent
        candidate = base_dir / model_path
        return candidate

    def _load_model(self):
        if not TORCH_AVAILABLE:
            self.get_logger().warning('Torch is not available. Fault detector will not run inference.')
            return None
        if not MODEL_AVAILABLE:
            self.get_logger().warning('Model class not available. Fault detector will not run inference.')
            return None

        model_path = self._resolve_model_path()
        if not model_path.exists():
            self.get_logger().warning(f'Model file not found: {model_path}')
            return None

        try:
            ckpt = torch.load(str(model_path), map_location='cpu')
            state_dict = ckpt.get('state_dict', ckpt)
            clean_state_dict = {k.replace('model.', ''): v for k, v in state_dict.items()}
            model = NN_classifier(input_layer=self.input_layer)
            model.load_state_dict(clean_state_dict)
            model.eval()
            self.get_logger().info(f'Loaded model: {model_path}')
            return model
        except Exception as exc:
            self.get_logger().warning(f'Failed to load model: {exc}')
            return None

    def _batch_callback(self, msg: String):
        if self.model is None:
            return

        try:
            batch = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning(f'Invalid batch JSON: {exc}')
            return

        data = batch.get('data', {})
        if not data:
            return

        try:
            processed = self.compile_feature_vector(data, self.labels, self.idxs)
        except Exception as exc:
            self.get_logger().warning(f'Failed to build feature vector: {exc}')
            return

        if processed.size == 0:
            return

        try:
            with torch.no_grad():
                output = self.model(torch.tensor(processed, dtype=torch.float32))
                if output.ndim > 1:
                    output = output.squeeze(0)
                pred = int(torch.argmax(output).item())
        except Exception as exc:
            self.get_logger().warning(f'Model inference failed: {exc}')
            return

        label, param_fix, fault = self.model_interpreter(pred)
        result = {
            'fault': fault,
            'fault_label': label or 'nominal',
            'parameters': param_fix or {},
            'timestamp': time.time(),
        }
        out_msg = String()
        out_msg.data = json.dumps(result)
        self.result_pub.publish(out_msg)

        if fault and self.inject_pub is not None:
            inject_cmd = {
                'action': 'set_params',
                'params': param_fix,
            }
            inject_msg = String()
            inject_msg.data = json.dumps(inject_cmd)
            self.inject_pub.publish(inject_msg)

    def _command_callback(self, msg: String):
        try:
            command = json.loads(msg.data)
        except Exception:
            self.get_logger().warning('Fault command must be JSON')
            return

        action = command.get('action', '')
        if action in ('inject_fault', 'fault'):
            fault_index = command.get('fault_index')
            fault_label = command.get('fault_label')
            params = command.get('params')
            if fault_index is not None:
                try:
                    fault_index = int(fault_index)
                except Exception:
                    self.get_logger().warning('fault_index must be an integer')
                    return
                if fault_index <= 0:
                    self._publish_fault(False, 'nominal', {})
                    return
                fault_label, params, fault = self.model_interpreter(fault_index)
                if not fault:
                    self.get_logger().warning('Invalid fault_index')
                    return
            elif fault_label and isinstance(params, dict):
                fault = True
            else:
                self.get_logger().warning('Fault command missing fault_index or fault_label/params')
                return
            self._publish_fault(True, fault_label, params)
            return

        if action in ('clear_fault', 'clear'):
            self._publish_fault(False, 'nominal', {})
            return

        self.get_logger().warning(f'Unknown fault command: {action}')

    def _publish_fault(self, fault, label, params):
        result = {
            'fault': bool(fault),
            'fault_label': label or 'nominal',
            'parameters': params or {},
            'timestamp': time.time(),
            'forced': True,
        }
        out_msg = String()
        out_msg.data = json.dumps(result)
        self.result_pub.publish(out_msg)

        if fault and self.inject_pub is not None and params:
            inject_cmd = {
                'action': 'set_params',
                'params': params,
            }
            inject_msg = String()
            inject_msg.data = json.dumps(inject_cmd)
            self.inject_pub.publish(inject_msg)

    def model_interpreter(self, output):
        if output <= 0:
            return None, None, False

        output_labels = [
            'motor 1 malfunction',
            'motor 2 malfunction',
            'motor 3 malfunction',
            'motor 4 malfunction',
        ]

        parameter_fix = [
            {'PWM_MAIN_FUNC4': 101},
            {'PWM_MAIN_FUNC1': 102},
            {'PWM_MAIN_FUNC2': 103},
            {'PWM_MAIN_FUNC3': 104},
        ]

        idx = int(output - 1)
        if idx < 0 or idx >= len(output_labels):
            return None, None, False
        return output_labels[idx], parameter_fix[idx], True

    def flatten_entry(self, entry, norm=None):
        values = []

        if isinstance(entry, dict):
            for v in entry.values():
                if isinstance(v, dict):
                    values.extend(self.flatten_entry(v))
                elif isinstance(v, (list, tuple)):
                    values.extend(v)
                else:
                    values.append(v)
        elif isinstance(entry, (list, tuple)):
            values.extend(entry)
        elif entry is None:
            values.append(0.0)
        else:
            values.append(entry)

        if norm is None:
            return values
        lo, hi, count = norm
        normalized = []
        for v in values[:count]:
            normalized.append((v - lo) / (hi - lo))
        return normalized

    def _select_entry(self, data, index):
        if data is None:
            return {}
        if isinstance(data, list):
            if not data:
                return {}
            if index < len(data):
                entry = data[index]
            else:
                entry = data[-1]
            if isinstance(entry, list) and entry:
                if isinstance(entry[-1], dict):
                    return entry[-1]
            return entry
        return data

    def compile_feature_vector(self, data, data_labels, idxs):
        feature_vector = []

        timestamps = data.get('timestamps')
        start_timestamp = None
        if isinstance(timestamps, list) and timestamps:
            try:
                start_timestamp = timestamps[idxs[0]]
            except Exception:
                start_timestamp = timestamps[0]

        for i in idxs:
            temp_vector = []
            for label in data_labels:
                if 'time' in label:
                    entry = data.get(label, [])
                    flattened = self.flatten_entry(entry)
                    if start_timestamp is not None:
                        flattened = [t - start_timestamp for t in flattened]
                    temp_vector.extend(flattened)
                else:
                    entry = self._select_entry(data.get(label), i)
                    if 'quat' in label:
                        if isinstance(entry, dict) and 'pose' in entry:
                            entry = entry['pose'].get('orientation', entry)
                    if 'rc' in label:
                        temp_vector.extend(self.flatten_entry(entry, norm=(1000, 2400, 8)))
                    elif 'servo' in label:
                        temp_vector.extend(self.flatten_entry(entry, norm=(0, 1000, 8)))
                    else:
                        temp_vector.extend(self.flatten_entry(entry))
            feature_vector.extend(temp_vector)

        return np.array(feature_vector, dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = FaultDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
