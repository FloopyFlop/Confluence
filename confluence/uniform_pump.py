#!/usr/bin/env python3
"""
ROS2 Uniform Pump node.
Subscribes to firehose JSON and publishes time-batched data for AI/ML.
"""

import json
import statistics
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MESSAGE_TYPE_ALIASES = {
    'ATTITUDE': 'attitude',
    'ATTITUDE_QUATERNION': 'attitude_quat',
    'RC_CHANNELS': 'rc_channels',
    'SERVO_OUTPUT_RAW': 'servo_output',
    'RAW_IMU': 'imu',
    'HIGHRES_IMU': 'imu',
    'LOCAL_POSITION_NED': 'local_pos',
    'GLOBAL_POSITION_INT': 'global_pos',
    'VFR_HUD': 'velocity',
    'ODOMETRY': 'odometry',
    'SCALED_IMU': 'imu',
    'SCALED_PRESSURE': 'pressure',
    'SYS_STATUS': 'sys_status',
    'HEARTBEAT': 'heartbeat',
}


class UniformPumpNode(Node):
    def __init__(self):
        super().__init__('uniform_pump_node')

        self.declare_parameter('batch_interval', 1.0)
        self.declare_parameter('condensation_mode', 'raw')
        self.declare_parameter('multi_step_count', 3)
        self.declare_parameter('bleeding_domain_duration', 15.0)
        self.declare_parameter('missing_data_strategy', 'bleeding_average')
        self.declare_parameter('input_topic', 'mav/all_messages')
        self.declare_parameter('output_topic', 'mav/uniform_batch')

        self.batch_interval = float(self.get_parameter('batch_interval').value)
        self.condensation_mode = str(self.get_parameter('condensation_mode').value)
        self.multi_step_count = int(self.get_parameter('multi_step_count').value)
        self.bleeding_domain_duration = float(self.get_parameter('bleeding_domain_duration').value)
        self.missing_data_strategy = str(self.get_parameter('missing_data_strategy').value)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.total_batches_published = 0
        self.total_messages_received = 0
        self.start_time = time.time()
        self.last_batch_time = time.time()

        self.current_batch_data = defaultdict(list)
        self.bleeding_domain = defaultdict(lambda: deque(maxlen=1000))

        self.msg_type_counts = defaultdict(int)
        self.last_batch_msg_types = set()

        self.subscriber = self.create_subscription(
            String,
            self.input_topic,
            self._mav_message_callback,
            100,
        )
        self.batch_pub = self.create_publisher(String, self.output_topic, 10)

        self.batch_timer = self.create_timer(self.batch_interval, self._publish_batch)

        self.get_logger().info('Uniform Pump started')
        self.get_logger().info(f'Batch interval: {self.batch_interval}s')
        self.get_logger().info(f'Condensation mode: {self.condensation_mode}')

    def _map_msg_type(self, msg_type: str) -> str:
        if not msg_type:
            return 'unknown'
        if msg_type in MESSAGE_TYPE_ALIASES:
            return MESSAGE_TYPE_ALIASES[msg_type]
        return msg_type.lower()

    def _mav_message_callback(self, msg: String):
        try:
            msg_dict = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning(f'Failed to parse MAV message JSON: {exc}')
            return

        msg_type = msg_dict.get('_msg_type') or msg_dict.get('msg_type') or 'UNKNOWN'
        key = self._map_msg_type(msg_type)
        msg_dict['_msg_type'] = msg_type
        if '_timestamp' not in msg_dict:
            msg_dict['_timestamp'] = time.time()

        self.current_batch_data[key].append(msg_dict)
        self.msg_type_counts[key] += 1
        self.total_messages_received += 1

        self.bleeding_domain[key].append((msg_dict['_timestamp'], msg_dict))
        self._clean_bleeding_domain(key)

    def _clean_bleeding_domain(self, msg_type):
        current_time = time.time()
        cutoff_time = current_time - self.bleeding_domain_duration
        while self.bleeding_domain[msg_type] and self.bleeding_domain[msg_type][0][0] < cutoff_time:
            self.bleeding_domain[msg_type].popleft()

    def _publish_batch(self):
        try:
            batch_data = dict(self.current_batch_data)
            batch_msg_types = set(batch_data.keys())
            self.current_batch_data = defaultdict(list)

            if self.condensation_mode == 'raw':
                packaged_data = self._package_raw(batch_data)
            elif self.condensation_mode == 'multi_step':
                packaged_data = self._package_multi_step(batch_data)
                packaged_data['data']['timestamps'] = self._derive_step_timestamps(batch_data)
            elif self.condensation_mode == 'single_step':
                packaged_data = self._package_single_step(batch_data)
            else:
                self.get_logger().warning(f'Unknown condensation mode: {self.condensation_mode}')
                return

            packaged_data['_batch_metadata'] = {
                'batch_number': self.total_batches_published,
                'batch_start_time': self.last_batch_time,
                'batch_end_time': time.time(),
                'batch_interval': self.batch_interval,
                'condensation_mode': self.condensation_mode,
                'message_types_count': len(batch_msg_types),
                'total_messages_in_batch': sum(len(msgs) for msgs in batch_data.values()),
            }

            payload = String()
            payload.data = json.dumps(packaged_data)
            self.batch_pub.publish(payload)

            self.total_batches_published += 1
            self.last_batch_time = time.time()
            self.last_batch_msg_types = batch_msg_types
        except Exception as exc:
            self.get_logger().warning(f'Error publishing batch: {exc}')

    def _package_raw(self, batch_data):
        packaged = {'data': {}}
        for msg_type, messages in batch_data.items():
            packaged['data'][msg_type] = messages
        packaged['data'] = self._fill_missing_data(packaged['data'], batch_data.keys())
        return packaged

    def _package_multi_step(self, batch_data):
        packaged = {'data': {}}
        for msg_type, messages in batch_data.items():
            if not messages:
                continue
            steps = [[] for _ in range(self.multi_step_count)]
            timestamps = [msg.get('_timestamp', 0) for msg in messages]
            min_time = min(timestamps)
            max_time = max(timestamps)
            time_range = max_time - min_time if max_time > min_time else 1.0
            step_duration = time_range / float(self.multi_step_count)

            for msg in messages:
                msg_time = msg.get('_timestamp', min_time)
                step_index = min(int((msg_time - min_time) / step_duration), self.multi_step_count - 1)
                steps[step_index].append(msg)

            averaged_steps = []
            for step_messages in steps:
                if step_messages:
                    averaged_steps.append(self._average_messages(step_messages))
                else:
                    averaged_steps.append(None)
            packaged['data'][msg_type] = averaged_steps

        packaged['data'] = self._fill_missing_data(packaged['data'], batch_data.keys())
        return packaged

    def _package_single_step(self, batch_data):
        packaged = {'data': {}}
        for msg_type, messages in batch_data.items():
            if messages:
                packaged['data'][msg_type] = self._average_messages(messages)
        packaged['data'] = self._fill_missing_data(packaged['data'], batch_data.keys())
        return packaged

    def _average_messages(self, messages):
        if not messages:
            return None
        field_values = defaultdict(list)
        for msg in messages:
            for field, value in msg.items():
                if field.startswith('_'):
                    continue
                field_values[field].append(value)

        averaged = {}
        for field, values in field_values.items():
            if self._is_numeric_list(values):
                averaged[field] = statistics.mean(values)
            else:
                averaged[field] = values[-1]
        return averaged

    def _is_numeric_list(self, values):
        try:
            for value in values:
                if value is None:
                    return False
                if isinstance(value, (list, dict)):
                    return False
                float(value)
            return True
        except (ValueError, TypeError):
            return False

    def _fill_missing_data(self, packaged_data, present_msg_types):
        all_msg_types = set(self.bleeding_domain.keys())
        missing_msg_types = all_msg_types - set(present_msg_types)

        for msg_type in missing_msg_types:
            bleeding_data = list(self.bleeding_domain[msg_type])
            if not bleeding_data:
                continue

            if self.missing_data_strategy == 'bleeding_average':
                messages = [msg_dict for _, msg_dict in bleeding_data]
                packaged_data[msg_type] = self._average_messages(messages)
            elif self.missing_data_strategy == 'bleeding_latest':
                _, latest_msg = bleeding_data[-1]
                packaged_data[msg_type] = latest_msg
            elif self.missing_data_strategy == 'null':
                packaged_data[msg_type] = None
            elif self.missing_data_strategy == 'zero':
                _, sample_msg = bleeding_data[-1]
                packaged_data[msg_type] = self._zero_fill_message(sample_msg)

        return packaged_data

    def _zero_fill_message(self, sample_msg):
        zeroed = {}
        for field, value in sample_msg.items():
            if field.startswith('_'):
                continue
            if isinstance(value, (int, float)):
                zeroed[field] = 0
            elif isinstance(value, list):
                zeroed[field] = [0] * len(value)
            else:
                zeroed[field] = value
        return zeroed

    def _derive_step_timestamps(self, batch_data):
        anchors = ['attitude', 'imu', 'attitude_quat']
        anchor_msgs = None
        for key in anchors:
            if key in batch_data and batch_data[key]:
                anchor_msgs = batch_data[key]
                break

        if not anchor_msgs:
            return [time.time()] * self.multi_step_count

        timestamps = [msg.get('_timestamp', time.time()) for msg in anchor_msgs]
        timestamps = sorted(timestamps)
        if len(timestamps) >= self.multi_step_count:
            return timestamps[-self.multi_step_count:]
        if timestamps:
            last = timestamps[-1]
        else:
            last = time.time()
        padding = [last] * (self.multi_step_count - len(timestamps))
        return timestamps + padding


def main(args=None):
    rclpy.init(args=args)
    node = UniformPumpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
