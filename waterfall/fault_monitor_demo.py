#!/usr/bin/env python3
"""
Fault monitor demo (no CLI flags).

Edit the configuration blocks below to match your hardware, model, and
parameter edits. The main() function shows the high-level flow:
  1) start services
  2) monitor telemetry while throttle is up
  3) classify and detect fault
  4) wait for disarm
  5) inject config edit
"""

import importlib
import importlib.util
import json
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


ARMED_MASK_DEFAULT = 128  # MAV_MODE_FLAG_SAFETY_ARMED


@dataclass
class ServiceConfig:
    start_services: bool = True
    services: List[str] = field(default_factory=lambda: ['firehose', 'inject'])
    use_sitl: bool = False
    connection: str = 'serial:/dev/ttyTHS3:115200'
    sitl_connection: str = 'udp:127.0.0.1:14550'
    serial_port: str = '/dev/ttyTHS3'
    serial_baud: int = 115200
    px4_build_path: str = '/path/to/PX4-Autopilot/build/px4_sitl_default'
    firehose_args: str = ''
    inject_args: str = ''


@dataclass
class TelemetryConfig:
    topic: str = 'mav/all_messages'
    include_types: List[str] = field(default_factory=list)
    throttle_msg_type: str = 'RC_CHANNELS'
    throttle_field: str = 'chan3_raw'
    throttle_threshold: float = 1200.0
    throttle_hysteresis: float = 50.0
    armed_mask: int = ARMED_MASK_DEFAULT


@dataclass
class ModelConfig:
    callable: Optional[Callable[[Dict[str, Any]], Any]] = None
    module: str = ''
    path: str = ''
    function: str = 'predict'
    include_types: List[str] = field(default_factory=list)
    classify_rate_hz: float = 5.0
    fault_label: str = 'fault'
    ok_label: str = 'ok'
    fault_score_threshold: float = 0.5


@dataclass
class InjectConfig:
    file: str = 'ROMFS/px4fmu_common/init.d/rc.autostart'
    modifications: Dict[str, float] = field(default_factory=lambda: {
        'MC_ROLL_P': -7.0,
        'MC_PITCH_P': -7.0,
    })


@dataclass
class TimeoutConfig:
    disarm_timeout_sec: float = 0.0  # 0 means wait forever


@dataclass
class DemoConfig:
    services: ServiceConfig = field(default_factory=ServiceConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    model: ModelConfig = field(default_factory=ModelConfig)
    inject: InjectConfig = field(default_factory=InjectConfig)
    timeouts: TimeoutConfig = field(default_factory=TimeoutConfig)


def example_model(snapshot: Dict[str, Any]) -> str:
    """Replace this with your model; return 'fault' or 'ok'."""
    return 'ok'


CONFIG = DemoConfig(
    services=ServiceConfig(),
    telemetry=TelemetryConfig(),
    model=ModelConfig(callable=example_model),
    inject=InjectConfig(),
    timeouts=TimeoutConfig(),
)


class FaultMonitorNode(Node):
    def __init__(self, config: DemoConfig, model: Callable[[Dict[str, Any]], Any]):
        super().__init__('waterfall_fault_monitor')
        self.config = config
        self.model = model

        self.latest_by_type: Dict[str, Dict[str, Any]] = {}
        self.throttle_active = False
        self.last_throttle: Optional[float] = None
        self.armed: Optional[bool] = None
        self.state = 'monitoring'
        self.fault_detected = False
        self.inject_sent = False
        self.last_classify = 0.0

        self.telemetry_include = self._parse_type_set(config.telemetry.include_types)
        self.model_include = self._parse_type_set(config.model.include_types)

        self.subscription = self.create_subscription(
            String,
            config.telemetry.topic,
            self._telemetry_callback,
            200,
        )
        self.inject_pub = self.create_publisher(String, 'px4_injector/command', 10)
        self.status_pub = self.create_publisher(String, 'waterfall/fault_monitor/status', 10)

    @staticmethod
    def _parse_type_set(items: List[str]) -> Optional[Set[str]]:
        if not items:
            return None
        cleaned = [item.strip() for item in items if item.strip()]
        return set(cleaned) if cleaned else None

    def _telemetry_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        msg_type = payload.get('_msg_type', 'UNKNOWN')

        if msg_type == 'HEARTBEAT':
            base_mode = payload.get('base_mode')
            if base_mode is not None:
                self.armed = bool(int(base_mode) & int(self.config.telemetry.armed_mask))
            return

        if msg_type == self.config.telemetry.throttle_msg_type:
            value = self._extract_field(payload, self.config.telemetry.throttle_field)
            if value is not None:
                self.last_throttle = float(value)
                self._update_throttle_state()

        if self.state != 'monitoring' or not self.throttle_active:
            return

        if self.telemetry_include and msg_type not in self.telemetry_include:
            return

        self.latest_by_type[msg_type] = payload

    def _update_throttle_state(self):
        if self.last_throttle is None:
            return
        threshold = self.config.telemetry.throttle_threshold
        hysteresis = self.config.telemetry.throttle_hysteresis
        if self.throttle_active:
            if self.last_throttle < (threshold - hysteresis):
                self.throttle_active = False
                self._publish_status('throttle_down')
        else:
            if self.last_throttle >= threshold:
                self.throttle_active = True
                self._publish_status('throttle_up')

    def should_classify(self, now: float) -> bool:
        if self.state != 'monitoring' or not self.throttle_active:
            return False
        interval = 1.0 / max(self.config.model.classify_rate_hz, 0.1)
        return (now - self.last_classify) >= interval

    def classify_once(self):
        snapshot = self._build_snapshot()
        try:
            result = self.model(snapshot)
        except Exception as exc:
            self.get_logger().error(f'Model inference failed: {exc}')
            return

        self.last_classify = time.time()
        fault = self._interpret_model_result(result)
        if fault:
            self.fault_detected = True
            self.state = 'waiting_for_disarm'
            self._publish_status('fault_detected', {'result': result})
            self.get_logger().warn('Fault detected; waiting for disarm before Inject.')

    def _build_snapshot(self) -> Dict[str, Any]:
        messages = self.latest_by_type
        if self.model_include is not None:
            messages = {k: v for k, v in messages.items() if k in self.model_include}
        return {
            'timestamp': time.time(),
            'throttle': self.last_throttle,
            'armed': self.armed,
            'messages': messages,
        }

    def _interpret_model_result(self, result: Any) -> bool:
        if isinstance(result, bool):
            return result
        if isinstance(result, (int, float)):
            return float(result) >= float(self.config.model.fault_score_threshold)
        if isinstance(result, str):
            label = result.strip().lower()
            if label == self.config.model.fault_label.lower():
                return True
            if label == self.config.model.ok_label.lower():
                return False
            return False
        if isinstance(result, dict):
            if 'fault' in result:
                return bool(result['fault'])
            if 'label' in result:
                return str(result['label']).strip().lower() == self.config.model.fault_label.lower()
            if 'score' in result:
                return float(result['score']) >= float(self.config.model.fault_score_threshold)
        return False

    @staticmethod
    def _extract_field(payload: Dict[str, Any], path: str) -> Optional[Any]:
        value: Any = payload
        for part in path.split('.'):
            if not isinstance(value, dict):
                return None
            value = value.get(part)
        return value

    def send_inject(self):
        if self.inject_sent:
            return
        command = {
            'action': 'edit',
            'file': self.config.inject.file,
            'modifications': self.config.inject.modifications,
        }
        msg = String()
        msg.data = json.dumps(command)
        self.inject_pub.publish(msg)
        self.inject_sent = True
        self._publish_status('inject_sent', {'command': command})
        self.get_logger().info('Inject command sent.')

    def _publish_status(self, state: str, extra: Optional[dict] = None):
        payload = {'state': state, 'timestamp': time.time()}
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def load_model(config: ModelConfig) -> Callable[[Dict[str, Any]], Any]:
    if config.callable is not None:
        return config.callable

    module = None
    if config.path:
        spec = importlib.util.spec_from_file_location('waterfall_user_model', config.path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f'Unable to load model from {config.path}')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    elif config.module:
        module = importlib.import_module(config.module)
    else:
        raise RuntimeError('Model callable is not configured.')

    func = getattr(module, config.function, None)
    if func is None or not callable(func):
        raise RuntimeError(f"Model callable '{config.function}' not found.")
    return func


def start_services(config: ServiceConfig) -> Dict[str, subprocess.Popen]:
    processes: Dict[str, subprocess.Popen] = {}
    for svc in config.services:
        cmd = build_service_cmd(svc, config)
        if not cmd:
            continue
        try:
            proc = subprocess.Popen(cmd)
            processes[svc] = proc
            print(f'[fault_demo] started {svc}: {" ".join(cmd)}')
        except Exception as exc:
            print(f'[fault_demo] failed to start {svc}: {exc}', file=sys.stderr)
    return processes


def stop_services(processes: Dict[str, subprocess.Popen]):
    for svc, proc in list(processes.items()):
        if proc.poll() is None:
            try:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        print(f'[fault_demo] stopped {svc} (rc={proc.poll()})')
        processes.pop(svc, None)


def build_service_cmd(svc: str, config: ServiceConfig) -> Optional[List[str]]:
    if svc == 'firehose':
        conn = config.connection or config.serial_port
        cmd = [
            'ros2', 'run', 'waterfall', 'firehose_service',
            '--connection', conn,
            '--serial-baud', str(config.serial_baud),
        ]
        if config.use_sitl:
            cmd.append('--sitl')
        if config.firehose_args:
            cmd.extend(shlex.split(config.firehose_args))
        return cmd

    if svc == 'inject':
        cmd = [
            'ros2', 'run', 'waterfall', 'inject_service',
            '--px4-build-path', config.px4_build_path,
            '--serial-baud', str(config.serial_baud),
            '--sitl-connection', config.sitl_connection,
            '--serial-port', config.serial_port,
        ]
        if config.connection:
            cmd.extend(['--connection', config.connection])
        if config.use_sitl:
            cmd.append('--sitl')
        if config.inject_args:
            cmd.extend(shlex.split(config.inject_args))
        return cmd

    return None


def monitor_until_fault(executor: SingleThreadedExecutor, node: FaultMonitorNode):
    print('[fault_demo] monitoring telemetry while throttle is up')
    while rclpy.ok() and not node.fault_detected:
        executor.spin_once(timeout_sec=0.1)
        now = time.time()
        if node.should_classify(now):
            node.classify_once()


def wait_for_disarm(executor: SingleThreadedExecutor, node: FaultMonitorNode, timeout_sec: float):
    print('[fault_demo] waiting for disarm')
    start = time.time()
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        if node.armed is False:
            return True
        if timeout_sec > 0 and (time.time() - start) >= timeout_sec:
            return False
    return False


def main():
    config = CONFIG
    model = load_model(config.model)

    rclpy.init(args=None)
    node = FaultMonitorNode(config, model)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    processes: Dict[str, subprocess.Popen] = {}
    try:
        if config.services.start_services:
            processes = start_services(config.services)

        # High-level flow
        monitor_until_fault(executor, node)

        if node.fault_detected:
            if node.armed is False:
                node.send_inject()
            else:
                ok = wait_for_disarm(executor, node, config.timeouts.disarm_timeout_sec)
                if ok:
                    node.send_inject()
                else:
                    print('[fault_demo] disarm timeout reached; no Inject sent')
        else:
            print('[fault_demo] exiting without fault detection')
    finally:
        if processes:
            stop_services(processes)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
