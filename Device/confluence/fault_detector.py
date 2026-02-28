#!/usr/bin/env python3
"""
ROS2 Fault Detector node.
Consumes uniform pump batches, runs a Torch model, and publishes fault results.
"""

import json
import time
from collections import deque
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
        self.declare_parameter('status_topic', 'fault_detector/status')
        self.declare_parameter('diagnostics_topic', 'fault_detector/diagnostics')
        self.declare_parameter('enable_diagnostics', True)
        self.declare_parameter('diagnostics_every_n', 1)
        self.declare_parameter('gate_from_raw_topic', True)
        self.declare_parameter('gate_topic', 'mav/all_messages')
        self.declare_parameter('require_flight_gate', True)
        self.declare_parameter('flight_gate_arm_required', True)
        self.declare_parameter('flight_gate_throttle_required', True)
        self.declare_parameter('throttle_channel_index', 3)
        self.declare_parameter('throttle_up_threshold', 1100.0)
        self.declare_parameter('hold_model_fault_until_landing', True)
        self.declare_parameter('auto_apply_model_fix_after_landing', True)
        # Safe default: only manual fault commands inject; model output is publish-only.
        self.declare_parameter('publish_inject_command', False)
        self.declare_parameter('defer_model_inject_until_disarmed', True)
        # Waterfall-compatible defaults: raw argmax without confidence/vote damping.
        self.declare_parameter('model_confidence_threshold', 0.0)
        self.declare_parameter('model_vote_window', 1)
        self.declare_parameter('model_vote_required', 1)
        self.declare_parameter('inject_topic', 'px4_injector/command')
        self.declare_parameter('command_topic', 'fault_detector/command')
        # Demo mode can set this true; production keeps inference active.
        self.declare_parameter('pause_inference_on_forced_fault', False)
        # Production default: clear should restore standard motor mappings.
        self.declare_parameter('clear_restores_params', True)
        self.declare_parameter('defer_restore_until_disarmed', True)
        self.declare_parameter(
            'clear_restore_params_json',
            '{"PWM_MAIN_FUNC4": 101, "PWM_MAIN_FUNC1": 102, "PWM_MAIN_FUNC2": 103, "PWM_MAIN_FUNC3": 104}',
        )

        self.model_path = str(self.get_parameter('model_path').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self.enable_diagnostics = bool(self.get_parameter('enable_diagnostics').value)
        self.diagnostics_every_n = max(1, int(self.get_parameter('diagnostics_every_n').value))
        self.gate_from_raw_topic = bool(self.get_parameter('gate_from_raw_topic').value)
        self.gate_topic = str(self.get_parameter('gate_topic').value)
        self.require_flight_gate = bool(self.get_parameter('require_flight_gate').value)
        self.flight_gate_arm_required = bool(self.get_parameter('flight_gate_arm_required').value)
        self.flight_gate_throttle_required = bool(
            self.get_parameter('flight_gate_throttle_required').value
        )
        self.throttle_channel_index = max(
            1, int(self.get_parameter('throttle_channel_index').value)
        )
        self.throttle_up_threshold = float(self.get_parameter('throttle_up_threshold').value)
        self.hold_model_fault_until_landing = bool(
            self.get_parameter('hold_model_fault_until_landing').value
        )
        self.auto_apply_model_fix_after_landing = bool(
            self.get_parameter('auto_apply_model_fix_after_landing').value
        )
        self.inject_topic = self.get_parameter('inject_topic').value
        self.publish_inject_command = bool(self.get_parameter('publish_inject_command').value)
        self.defer_model_inject_until_disarmed = bool(
            self.get_parameter('defer_model_inject_until_disarmed').value
        )
        self.model_confidence_threshold = float(
            self.get_parameter('model_confidence_threshold').value
        )
        self.model_vote_window = max(1, int(self.get_parameter('model_vote_window').value))
        self.model_vote_required = max(1, int(self.get_parameter('model_vote_required').value))
        if self.model_vote_required > self.model_vote_window:
            self.model_vote_required = self.model_vote_window
        self.command_topic = self.get_parameter('command_topic').value
        self.pause_inference_on_forced_fault = bool(
            self.get_parameter('pause_inference_on_forced_fault').value
        )
        self.clear_restores_params = bool(
            self.get_parameter('clear_restores_params').value
        )
        self.defer_restore_until_disarmed = bool(
            self.get_parameter('defer_restore_until_disarmed').value
        )
        self.clear_restore_params = self._parse_restore_params(
            str(self.get_parameter('clear_restore_params_json').value)
        )
        self._last_vector_warn = 0.0
        self._forced_fault_active = False
        self._last_model_fault_state = None
        self._model_pred_history = deque(maxlen=max(1, self.model_vote_window))
        self._batches_received = 0
        self._inferences_run = 0
        self._last_data_keys = []
        self._last_step_indices = []
        self._last_feature_len = 0
        self._last_pred_raw = None
        self._last_pred_stable = None
        self._last_confidence = None
        self._last_fault_label = "nominal"
        self._last_inference_ts = None
        self._armed = False
        self._throttle = 0.0
        self._throttle_up = False
        self._flight_gate_open = not self.require_flight_gate
        self._pending_model_fix = None
        self._model_fault_latched = False
        self._last_gate_update_ts = 0.0

        self._configure_model_spec()
        self.model = self._load_model()

        self.subscription = self.create_subscription(
            String,
            self.input_topic,
            self._batch_callback,
            10,
        )
        self.gate_sub = None
        if self.gate_from_raw_topic and self.gate_topic:
            self.gate_sub = self.create_subscription(
                String,
                self.gate_topic,
                self._gate_callback,
                50,
            )
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._command_callback,
            10,
        )
        self.result_pub = self.create_publisher(String, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.diagnostics_pub = self.create_publisher(String, self.diagnostics_topic, 10)
        # Always create inject publisher so forced fault commands can optionally apply params.
        self.inject_pub = self.create_publisher(String, self.inject_topic, 10)

        self.get_logger().info('Fault Detector started')
        self.get_logger().info(
            "Clear restore enabled="
            f"{self.clear_restores_params} defer_until_disarmed={self.defer_restore_until_disarmed} "
            f"params={self.clear_restore_params}"
        )
        self.get_logger().info(
            "Model stabilization: "
            f"confidence>={self.model_confidence_threshold} "
            f"votes={self.model_vote_required}/{self.model_vote_window} "
            f"defer_model_inject_until_disarmed={self.defer_model_inject_until_disarmed}"
        )
        self.get_logger().info(
            f"Diagnostics: enabled={self.enable_diagnostics} topic={self.diagnostics_topic} "
            f"every_n={self.diagnostics_every_n}"
        )
        self.get_logger().info(
            f"Gate source: raw_topic={self.gate_from_raw_topic} topic={self.gate_topic}"
        )
        self.get_logger().info(
            "Flight gate: "
            f"required={self.require_flight_gate} "
            f"arm_required={self.flight_gate_arm_required} "
            f"throttle_required={self.flight_gate_throttle_required} "
            f"threshold={self.throttle_up_threshold} "
            f"hold_fault_until_landing={self.hold_model_fault_until_landing} "
            f"auto_apply_fix_after_landing={self.auto_apply_model_fix_after_landing}"
        )

    def _parse_restore_params(self, raw):
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict):
                return parsed
        except Exception:
            pass
        self.get_logger().warning('Invalid clear_restore_params_json; using empty restore map')
        return {}

    def _publish_diagnostic(self, event, **fields):
        if not self.enable_diagnostics:
            return
        payload = {
            'event': event,
            'timestamp': time.time(),
            'source': 'fault_detector',
            'batches_received': self._batches_received,
            'inferences_run': self._inferences_run,
        }
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload)
        self.diagnostics_pub.publish(msg)

    def _publish_status(
        self,
        *,
        source,
        fault,
        fault_label,
        confidence=None,
        raw_pred=None,
        stable_pred=None,
        extra=None,
    ):
        payload = {
            'source': source,
            'fault': bool(fault),
            'fault_label': fault_label or 'nominal',
            'timestamp': time.time(),
        }
        if confidence is not None:
            payload['confidence'] = float(confidence)
        if raw_pred is not None:
            payload['raw_pred'] = int(raw_pred)
        if stable_pred is not None:
            payload['stable_pred'] = int(stable_pred)
        if isinstance(extra, dict):
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _diagnostic_snapshot(self):
        return {
            'model_loaded': self.model is not None,
            'forced_fault_active': self._forced_fault_active,
            'last_data_keys': list(self._last_data_keys),
            'last_step_indices': list(self._last_step_indices),
            'last_feature_len': int(self._last_feature_len),
            'last_pred_raw': self._last_pred_raw,
            'last_pred_stable': self._last_pred_stable,
            'last_confidence': self._last_confidence,
            'last_fault_label': self._last_fault_label,
            'last_inference_ts': self._last_inference_ts,
            'armed': bool(self._armed),
            'throttle': float(self._throttle),
            'throttle_up': bool(self._throttle_up),
            'flight_gate_open': bool(self._flight_gate_open),
            'model_fault_latched': bool(self._model_fault_latched),
            'pending_model_fix': self._pending_model_fix,
        }

    def _latest_entry(self, entry):
        if isinstance(entry, list):
            for item in reversed(entry):
                if item is None:
                    continue
                if isinstance(item, list):
                    nested = self._latest_entry(item)
                    if nested is not None:
                        return nested
                    continue
                return item
            return None
        return entry

    def _extract_armed(self, data):
        heartbeat = self._latest_entry(data.get('heartbeat'))
        if not isinstance(heartbeat, dict):
            return None
        base_mode = heartbeat.get('base_mode')
        if base_mode is None:
            return None
        mode = int(self._float_or_zero(base_mode))
        # MAV_MODE_FLAG_SAFETY_ARMED
        return bool(mode & 0x80)

    def _extract_throttle(self, data):
        rc = self._latest_entry(data.get('rc_channels'))
        if rc is None:
            return None
        idx = self.throttle_channel_index - 1
        if isinstance(rc, dict):
            key = f'chan{self.throttle_channel_index}_raw'
            if key in rc:
                return self._float_or_zero(rc.get(key))
            chans = rc.get('channels')
            if isinstance(chans, (list, tuple)) and idx < len(chans):
                return self._float_or_zero(chans[idx])
        if isinstance(rc, (list, tuple)) and idx < len(rc):
            return self._float_or_zero(rc[idx])
        return None

    def _set_flight_gate_state(self, *, armed=None, throttle=None, gate_source='batch'):
        prev_gate = self._flight_gate_open
        prev_armed = self._armed
        prev_throttle_up = self._throttle_up

        if armed is not None:
            self._armed = bool(armed)
        if throttle is not None:
            self._throttle = float(throttle)
        self._throttle_up = bool(self._throttle > self.throttle_up_threshold)

        gate = True
        if self.require_flight_gate:
            if self.flight_gate_arm_required:
                gate = gate and bool(self._armed)
            if self.flight_gate_throttle_required:
                gate = gate and bool(self._throttle_up)
        self._flight_gate_open = bool(gate)

        if (
            prev_gate != self._flight_gate_open
            or prev_armed != self._armed
            or prev_throttle_up != self._throttle_up
        ):
            self.get_logger().info(
                "Flight gate state: "
                f"open={self._flight_gate_open} armed={self._armed} "
                f"throttle={self._throttle:.1f} throttle_up={self._throttle_up} "
                f"source={gate_source}"
            )
            self._publish_diagnostic(
                'flight_gate_state',
                open=bool(self._flight_gate_open),
                threshold=float(self.throttle_up_threshold),
                gate_source=gate_source,
                **self._diagnostic_snapshot(),
            )
            self._publish_status(
                source='flight_gate',
                fault=False,
                fault_label='flying' if self._flight_gate_open else 'not_flying',
                extra={
                    'gate_open': bool(self._flight_gate_open),
                    'armed': bool(self._armed),
                    'throttle': float(self._throttle),
                    'throttle_up': bool(self._throttle_up),
                    'gate_source': gate_source,
                },
            )

    def _update_flight_gate_state(self, data, gate_source='batch'):
        armed = self._extract_armed(data)
        throttle = self._extract_throttle(data)
        self._set_flight_gate_state(armed=armed, throttle=throttle, gate_source=gate_source)

    def _gate_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        msg_type = str(payload.get('_msg_type') or payload.get('msg_type') or '').upper()
        if not msg_type:
            return

        armed = None
        throttle = None
        if msg_type == 'HEARTBEAT':
            armed = self._extract_armed({'heartbeat': [payload]})
        elif msg_type == 'RC_CHANNELS':
            throttle = self._extract_throttle({'rc_channels': [payload]})
        else:
            return

        self._last_gate_update_ts = time.time()
        self._set_flight_gate_state(
            armed=armed,
            throttle=throttle,
            gate_source=f'raw:{msg_type.lower()}',
        )

    def _queue_model_fix(self, label, params):
        self._model_fault_latched = True
        self._pending_model_fix = {
            'label': label or 'unknown',
            'params': params or {},
            'detected_at': time.time(),
        }
        self.get_logger().warning(
            f"Model fault latched: label={label} params={params} "
            "fix will apply when flight gate closes"
        )
        self._publish_diagnostic(
            'model_fault_latched',
            label=label,
            params=params,
            **self._diagnostic_snapshot(),
        )

    def _maybe_apply_pending_model_fix(self):
        if not self.auto_apply_model_fix_after_landing:
            return
        pending = self._pending_model_fix
        if not isinstance(pending, dict):
            return
        params = pending.get('params')
        if not isinstance(params, dict) or not params:
            return
        if self.require_flight_gate and self._flight_gate_open:
            return
        if self.inject_pub is None:
            return

        inject_msg = String()
        inject_msg.data = json.dumps(
            {
                'action': 'set_params',
                'params': params,
                'source': 'fault_detector_model_landing_fix',
                'defer_until_disarmed': False,
            }
        )
        self.inject_pub.publish(inject_msg)
        self.get_logger().warning(
            f"Applied model fix after landing: label={pending.get('label')} params={params}"
        )
        self._publish_diagnostic(
            'model_fix_applied_after_landing',
            label=pending.get('label'),
            params=params,
            **self._diagnostic_snapshot(),
        )
        self._publish_status(
            source='model_fix',
            fault=False,
            fault_label='nominal',
            extra={
                'fix_applied': True,
                'label': pending.get('label'),
                'params': params,
            },
        )
        self._pending_model_fix = None
        self._model_fault_latched = False
        self._model_pred_history.clear()

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
        self._batches_received += 1
        if self.model is None:
            if self._batches_received % self.diagnostics_every_n == 0:
                self._publish_diagnostic(
                    'batch_skipped',
                    reason='model_not_loaded',
                    **self._diagnostic_snapshot(),
                )
            return
        if self._forced_fault_active and self.pause_inference_on_forced_fault:
            return

        try:
            batch = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning(f'Invalid batch JSON: {exc}')
            if self._batches_received % self.diagnostics_every_n == 0:
                self._publish_diagnostic(
                    'error',
                    stage='batch_json_parse',
                    error=str(exc),
                    **self._diagnostic_snapshot(),
                )
            return

        data = batch.get('data', {})
        self._last_data_keys = sorted(list(data.keys())) if isinstance(data, dict) else []
        if not data:
            if self._batches_received % self.diagnostics_every_n == 0:
                self._publish_diagnostic(
                    'batch_skipped',
                    reason='empty_data',
                    **self._diagnostic_snapshot(),
                )
            return

        if self.gate_from_raw_topic:
            now = time.time()
            # Fallback to batched gate extraction only if raw gate updates stall.
            if self._last_gate_update_ts <= 0.0 or (now - self._last_gate_update_ts) > 2.0:
                self._update_flight_gate_state(data, gate_source='batch_fallback')
        else:
            self._update_flight_gate_state(data, gate_source='batch')
        self._maybe_apply_pending_model_fix()
        if self.require_flight_gate and not self._flight_gate_open:
            if self._batches_received % self.diagnostics_every_n == 0:
                self._publish_diagnostic(
                    'inference_skipped',
                    reason='flight_gate_closed',
                    **self._diagnostic_snapshot(),
                )
            return
        if self._model_fault_latched and self.hold_model_fault_until_landing:
            if self._batches_received % self.diagnostics_every_n == 0:
                self._publish_diagnostic(
                    'inference_skipped',
                    reason='model_fault_latched',
                    **self._diagnostic_snapshot(),
                )
            return

        try:
            processed = self.compile_feature_vector(data, self.labels, self.idxs)
            self._last_feature_len = int(processed.size)
        except Exception as exc:
            self.get_logger().warning(f'Failed to build feature vector: {exc}')
            if self._batches_received % self.diagnostics_every_n == 0:
                self._publish_diagnostic(
                    'error',
                    stage='feature_build',
                    error=str(exc),
                    **self._diagnostic_snapshot(),
                )
            return

        if processed.size == 0:
            return

        try:
            with torch.no_grad():
                logits = self.model(torch.tensor(processed, dtype=torch.float32))
                if logits.ndim > 1:
                    logits = logits.squeeze(0)
                probs = torch.softmax(logits, dim=0)
                pred = int(torch.argmax(probs).item())
                confidence = float(probs[pred].item())
        except Exception as exc:
            self.get_logger().warning(f'Model inference failed: {exc}')
            if self._batches_received % self.diagnostics_every_n == 0:
                self._publish_diagnostic(
                    'error',
                    stage='model_inference',
                    error=str(exc),
                    **self._diagnostic_snapshot(),
                )
            return

        self._inferences_run += 1
        self._last_pred_raw = pred
        self._last_confidence = confidence
        stable_pred = self._stabilize_model_prediction(pred, confidence)
        self._last_pred_stable = stable_pred
        self._last_inference_ts = time.time()
        label, param_fix, fault = self.model_interpreter(stable_pred)
        self._last_fault_label = label or 'nominal'
        result = {
            'fault': fault,
            'fault_label': label or 'nominal',
            'parameters': param_fix or {},
            'timestamp': time.time(),
            'source': 'model',
            'raw_pred': pred,
            'stable_pred': stable_pred,
            'confidence': confidence,
        }
        out_msg = String()
        out_msg.data = json.dumps(result)
        self.result_pub.publish(out_msg)
        self._publish_status(
            source='model',
            fault=bool(fault),
            fault_label=label or 'nominal',
            confidence=confidence,
            raw_pred=pred,
            stable_pred=stable_pred,
        )
        if self._inferences_run % self.diagnostics_every_n == 0:
            self._publish_diagnostic(
                'inference',
                fault=bool(fault),
                fault_label=label or 'nominal',
                confidence=confidence,
                raw_pred=pred,
                stable_pred=stable_pred,
                feature_len=int(processed.size),
                data_keys=self._last_data_keys,
                step_indices=list(self._last_step_indices),
            )
        self._log_model_fault_state(fault, label or 'nominal', param_fix or {})

        if not fault:
            return

        if self.hold_model_fault_until_landing:
            self._queue_model_fix(label, param_fix)
            return

        # Optional immediate model-driven injection for non-latched mode.
        if self.publish_inject_command and self.inject_pub is not None:
            inject_cmd = {
                'action': 'set_params',
                'params': param_fix,
                'source': 'fault_detector_model',
                'defer_until_disarmed': bool(self.defer_model_inject_until_disarmed),
            }
            inject_msg = String()
            inject_msg.data = json.dumps(inject_cmd)
            self.inject_pub.publish(inject_msg)
            self.get_logger().warning(f'Model-driven inject command published: {inject_cmd}')

    def _stabilize_model_prediction(self, pred, confidence):
        # Guard against low-confidence flicker in raw model output.
        if confidence < self.model_confidence_threshold:
            normalized_pred = 0
        else:
            normalized_pred = int(pred)

        self._model_pred_history.append(normalized_pred)
        if normalized_pred <= 0:
            return 0

        votes = sum(1 for p in self._model_pred_history if p == normalized_pred)
        if votes >= self.model_vote_required:
            return normalized_pred
        return 0

    def _log_model_fault_state(self, fault, label, params):
        state = (bool(fault), str(label))
        if state == self._last_model_fault_state:
            return
        self._last_model_fault_state = state
        if fault:
            self.get_logger().warning(
                f'Model fault detected: label={label} params={params}'
            )
        else:
            self.get_logger().info('Model output nominal')

    def _command_callback(self, msg: String):
        try:
            command = json.loads(msg.data)
        except Exception:
            self.get_logger().warning('Fault command must be JSON')
            return

        action = command.get('action', '')
        if action in ('run_probe', 'probe', 'diagnostics_probe'):
            probe_id = command.get('probe_id')
            checks = {
                'model_loaded': self.model is not None,
                'batch_received': self._batches_received > 0,
                'inference_run': self._inferences_run > 0,
                'feature_len_valid': self._last_feature_len == self.input_layer,
            }
            passed = all(checks.values())
            snapshot = self._diagnostic_snapshot()
            self._publish_diagnostic(
                'probe_result',
                probe_id=probe_id,
                passed=passed,
                checks=checks,
                snapshot=snapshot,
            )
            self.get_logger().info(f'Probe result: passed={passed} checks={checks}')
            return

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
                fault_label, params, fault = self.forced_fault_interpreter(fault_index)
                if not fault:
                    self.get_logger().warning('Invalid fault_index')
                    return
            elif fault_label and isinstance(params, dict):
                fault = True
            else:
                self.get_logger().warning('Fault command missing fault_index or fault_label/params')
                return
            self._forced_fault_active = True
            apply_inject = bool(command.get('apply_inject', True))
            self.get_logger().warning(
                f'Forced fault injected: label={fault_label} params={params} apply_inject={apply_inject}'
            )
            self._publish_fault(True, fault_label, params, apply_inject=apply_inject)
            return

        if action in ('clear_fault', 'clear'):
            self._forced_fault_active = False
            self._pending_model_fix = None
            self._model_fault_latched = False
            self._model_pred_history.clear()
            restore = bool(command.get('restore', self.clear_restores_params))
            defer_until_disarmed = bool(
                command.get('defer_until_disarmed', self.defer_restore_until_disarmed)
            )
            restore_params = command.get('restore_params')
            if not isinstance(restore_params, dict):
                restore_params = self.clear_restore_params if restore else {}
            self.get_logger().info(
                "Forced fault cleared. "
                f"restore={restore} "
                f"defer_until_disarmed={defer_until_disarmed} "
                f"restore_params={restore_params}"
            )
            self._publish_fault(False, 'nominal', {}, apply_inject=False)
            if restore and restore_params and self.inject_pub is not None:
                inject_msg = String()
                inject_msg.data = json.dumps({
                    'action': 'set_params',
                    'params': restore_params,
                    'source': 'fault_detector_clear',
                    'defer_until_disarmed': defer_until_disarmed,
                })
                self.inject_pub.publish(inject_msg)
                self.get_logger().info(
                    'Published restore params after clear_fault '
                    f'(defer_until_disarmed={defer_until_disarmed})'
                )
                self._publish_diagnostic(
                    'clear_restore_published',
                    defer_until_disarmed=defer_until_disarmed,
                    restore_params=restore_params,
                    **self._diagnostic_snapshot(),
                )
            return

        self.get_logger().warning(f'Unknown fault command: {action}')

    def _publish_fault(self, fault, label, params, apply_inject=True):
        result = {
            'fault': bool(fault),
            'fault_label': label or 'nominal',
            'parameters': params or {},
            'timestamp': time.time(),
            'forced': True,
            'source': 'forced',
            'apply_inject': bool(apply_inject),
        }
        out_msg = String()
        out_msg.data = json.dumps(result)
        self.result_pub.publish(out_msg)
        self._publish_status(
            source='forced',
            fault=bool(fault),
            fault_label=label or 'nominal',
        )

        if fault and apply_inject and self.inject_pub is not None and params:
            inject_cmd = {
                'action': 'set_params',
                'params': params,
                'source': 'fault_detector_forced',
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

    def forced_fault_interpreter(self, output):
        if output <= 0:
            return None, None, False

        output_labels = [
            'motor 1 malfunction',
            'motor 2 malfunction',
            'motor 3 malfunction',
            'motor 4 malfunction',
        ]
        # Align forced-fault motor indices with model fix mapping:
        # motor 1 <-> PWM_MAIN_FUNC4, motor 2 <-> PWM_MAIN_FUNC1,
        # motor 3 <-> PWM_MAIN_FUNC2, motor 4 <-> PWM_MAIN_FUNC3.
        parameter_fault = [
            {'PWM_MAIN_FUNC4': 0},
            {'PWM_MAIN_FUNC1': 0},
            {'PWM_MAIN_FUNC2': 0},
            {'PWM_MAIN_FUNC3': 0},
        ]

        idx = int(output - 1)
        if idx < 0 or idx >= len(output_labels):
            return None, None, False
        return output_labels[idx], parameter_fault[idx], True

    def flatten_entry(self, entry, norm=None):
        values = []

        if isinstance(entry, dict):
            for v in entry.values():
                if isinstance(v, dict):
                    values.extend(self.flatten_entry(v))
                elif isinstance(v, (list, tuple)):
                    values.extend(self.flatten_entry(v))
                else:
                    num = self._coerce_float(v)
                    if num is not None:
                        values.append(num)
        elif isinstance(entry, (list, tuple)):
            for v in entry:
                num = self._coerce_float(v)
                if num is not None:
                    values.append(num)
        elif entry is None:
            values.append(0.0)
        else:
            num = self._coerce_float(entry)
            if num is not None:
                values.append(num)

        if norm is None:
            return values
        lo, hi, count = norm
        normalized = []
        for v in values[:count]:
            try:
                normalized.append((float(v) - lo) / (hi - lo))
            except Exception:
                normalized.append(0.0)
        return normalized

    def _coerce_float(self, value):
        if isinstance(value, bool):
            return float(int(value))
        if isinstance(value, (int, float)):
            return float(value)
        try:
            return float(value)
        except Exception:
            return None

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

    def _float_or_zero(self, value):
        num = self._coerce_float(value)
        if num is None:
            return 0.0
        return num

    def _normalize_list(self, values, low, high, count):
        out = []
        span = high - low
        if span == 0:
            span = 1.0
        for idx in range(count):
            if idx < len(values):
                v = self._float_or_zero(values[idx])
                out.append((v - low) / span)
            else:
                out.append(0.0)
        return out

    def _extract_attitude(self, entry):
        if not isinstance(entry, dict):
            return [0.0] * 6
        return [
            self._float_or_zero(entry.get('roll')),
            self._float_or_zero(entry.get('pitch')),
            self._float_or_zero(entry.get('yaw')),
            self._float_or_zero(entry.get('rollspeed')),
            self._float_or_zero(entry.get('pitchspeed')),
            self._float_or_zero(entry.get('yawspeed')),
        ]

    def _extract_quat(self, entry):
        # waterfall_local shape
        if isinstance(entry, dict) and 'pose' in entry:
            orient = entry.get('pose', {}).get('orientation', {})
            return [
                self._float_or_zero(orient.get('w')),
                self._float_or_zero(orient.get('x')),
                self._float_or_zero(orient.get('y')),
                self._float_or_zero(orient.get('z')),
            ]
        # MAVLink raw ATTITUDE_QUATERNION shape
        if isinstance(entry, dict):
            return [
                self._float_or_zero(entry.get('q1')),
                self._float_or_zero(entry.get('q2')),
                self._float_or_zero(entry.get('q3')),
                self._float_or_zero(entry.get('q4')),
            ]
        return [0.0] * 4

    def _extract_channels(self, entry, key_prefix):
        if isinstance(entry, dict):
            # waterfall_local shape
            if 'channels' in entry and isinstance(entry['channels'], (list, tuple)):
                raw = [self._float_or_zero(v) for v in entry['channels']]
                return raw[:8]
            # MAVLink raw shape (chan1_raw.. / servo1_raw..)
            vals = []
            for i in range(1, 9):
                vals.append(self._float_or_zero(entry.get(f'{key_prefix}{i}_raw')))
            return vals
        if isinstance(entry, (list, tuple)):
            return [self._float_or_zero(v) for v in entry[:8]]
        return [0.0] * 8

    def compile_feature_vector(self, data, data_labels, idxs):
        feature_vector = []
        resolved_idxs = self._resolve_step_indices(data, idxs)
        self._last_step_indices = list(resolved_idxs)

        timestamps = data.get('timestamps')
        start_timestamp = None
        if isinstance(timestamps, list) and timestamps:
            try:
                start_timestamp = timestamps[resolved_idxs[0]]
            except Exception:
                start_timestamp = timestamps[0]

        for i in resolved_idxs:
            temp_vector = []
            # Fixed schema to match training pipeline:
            # attitude(6) + quat(4) + rc(8) + servo(8) + timestamp(1 optional)
            attitude = self._select_entry(data.get('attitude'), i)
            temp_vector.extend(self._extract_attitude(attitude))

            quat = self._select_entry(data.get('attitude_quat'), i)
            temp_vector.extend(self._extract_quat(quat))

            rc = self._select_entry(data.get('rc_channels'), i)
            temp_vector.extend(self._normalize_list(self._extract_channels(rc, 'chan'), 1000.0, 2400.0, 8))

            servo = self._select_entry(data.get('servo_output'), i)
            temp_vector.extend(self._normalize_list(self._extract_channels(servo, 'servo'), 0.0, 1000.0, 8))

            if 'timestamps' in data_labels:
                ts_val = 0.0
                if isinstance(timestamps, list) and timestamps:
                    if i < len(timestamps):
                        ts_val = self._float_or_zero(timestamps[i])
                    else:
                        ts_val = self._float_or_zero(timestamps[-1])
                start_num = self._coerce_float(start_timestamp)
                if start_num is not None:
                    ts_val = ts_val - start_num
                temp_vector.append(ts_val)
            feature_vector.extend(temp_vector)

        # Ensure vector length matches model input
        if len(feature_vector) != self.input_layer:
            now = time.time()
            if now - self._last_vector_warn > 5.0:
                self.get_logger().warning(
                    f'Feature vector length {len(feature_vector)} does not match expected {self.input_layer}. '
                    'Padding/truncating to fit.'
                )
                self._last_vector_warn = now
            if len(feature_vector) < self.input_layer:
                feature_vector.extend([0.0] * (self.input_layer - len(feature_vector)))
            else:
                feature_vector = feature_vector[: self.input_layer]

        return np.array(feature_vector, dtype=float)

    def _resolve_step_indices(self, data, preferred_indices):
        desired_count = max(1, len(preferred_indices))
        available_count = self._infer_available_step_count(data)

        if available_count <= 1:
            return [0] * desired_count
        if available_count >= desired_count:
            start = available_count - desired_count
            return list(range(start, available_count))

        idxs = list(range(available_count))
        idxs.extend([available_count - 1] * (desired_count - available_count))
        return idxs

    def _infer_available_step_count(self, data):
        if not isinstance(data, dict):
            return 1
        keys = ('attitude', 'attitude_quat', 'rc_channels', 'servo_output', 'timestamps')
        sizes = []
        for key in keys:
            entry = data.get(key)
            if isinstance(entry, list) and entry:
                sizes.append(len(entry))
        if not sizes:
            return 1
        return max(sizes)


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
