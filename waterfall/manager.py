import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SERVICE_CHOICES = ['firehose', 'uniform_pump', 'orchestra', 'inject']


@dataclass
class LaunchConfig:
    services: List[str]
    sitl: bool
    connection: Optional[str]
    sitl_connection: str
    serial_port: str
    serial_baud: int
    firehose_rate: int
    publish_raw: bool
    batch_interval: float
    condensation_mode: str
    multi_step_count: int
    bleeding_domain: float
    missing_strategy: str
    inject_px4_path: str
    firehose_args: List[str]
    pump_args: List[str]
    inject_args: List[str]
    orchestra_args: List[str]
    orchestra_sitl: str
    orchestra_hardware: str
    orchestra_connection: Optional[str]


class WaterfallManager(Node):
    """ROS2 node that spawns the four Waterfall services as child processes."""

    def __init__(self, config: LaunchConfig):
        super().__init__('waterfall_manager')
        self.config = config
        self.processes: Dict[str, subprocess.Popen] = {}

        self.status_pub = self.create_publisher(String, 'waterfall/status', 10)
        self.create_timer(1.0, self._poll_processes)

        self._start_requested_services()

    def destroy_node(self):
        self._stop_processes()
        super().destroy_node()

    def _start_requested_services(self):
        for service in self.config.services:
            cmd_env = self._build_command(service)
            if cmd_env is None:
                continue
            cmd, env = cmd_env
            try:
                proc = subprocess.Popen(cmd, env=env)
                self.processes[service] = proc
                self.get_logger().info(f'Started {service}: {" ".join(cmd)}')
                self._publish_status(service, 'started', {'pid': proc.pid})
            except Exception as exc:
                self.get_logger().error(f'Failed to start {service}: {exc}')
                self._publish_status(service, 'failed', {'error': str(exc)})

    def _build_command(self, service: str) -> Optional[Tuple[List[str], dict]]:
        env = os.environ.copy()

        if service == 'firehose':
            conn = self._resolve_mavlink_connection()
            cmd = [
                'ros2', 'run', 'waterfall', 'firehose_service',
                '--ros-args',
                '-p', f'use_sitl:={str(self.config.sitl).lower()}',
                '-p', f'connection_string:={conn}',
                '-p', f'serial_port:={self.config.serial_port}',
                '-p', f'serial_baud:={self.config.serial_baud}',
                '-p', f'data_stream_rate:={self.config.firehose_rate}',
                '-p', f'publish_raw_bytes:={"true" if self.config.publish_raw else "false"}',
            ]
            cmd.extend(self.config.firehose_args)
            return cmd, env

        if service == 'uniform_pump':
            cmd = [
                'ros2', 'run', 'waterfall', 'uniform_pump_service',
                '--ros-args',
                '-p', f'batch_interval:={self.config.batch_interval}',
                '-p', f'condensation_mode:={self.config.condensation_mode}',
                '-p', f'multi_step_count:={self.config.multi_step_count}',
                '-p', f'bleeding_domain_duration:={self.config.bleeding_domain}',
                '-p', f'missing_data_strategy:={self.config.missing_strategy}',
            ]
            cmd.extend(self.config.pump_args)
            return cmd, env

        if service == 'inject':
            if not self.config.inject_px4_path:
                self.get_logger().warning('Inject requested but --inject-px4-path was not provided; skipping.')
                return None

            conn = self._resolve_mavlink_connection()
            cmd = [
                'ros2', 'run', 'waterfall', 'inject_service',
                '--px4-build-path', self.config.inject_px4_path,
            ]
            if self.config.sitl:
                cmd.append('--sitl')
            cmd.extend([
                '--sitl-connection', self.config.sitl_connection,
                '--serial-port', self.config.serial_port,
                '--serial-baud', str(self.config.serial_baud),
            ])
            if self.config.connection:
                cmd.extend(['--connection', self.config.connection])

            cmd.extend(self.config.inject_args)

            # Keep ROS parameters aligned so the node can be introspected
            cmd.extend([
                '--ros-args',
                '-p', f'use_sitl:={str(self.config.sitl).lower()}',
                '-p', f'sitl_connection:={self.config.sitl_connection}',
                '-p', f'serial_port:={self.config.serial_port}',
                '-p', f'serial_baud:={self.config.serial_baud}',
            ])
            if self.config.connection:
                cmd.extend(['-p', f'mavlink_connection:={self.config.connection}'])

            return cmd, env

        if service == 'orchestra':
            connection = self._resolve_orchestra_connection()
            cmd = [
                'ros2', 'run', 'waterfall', 'orchestra_service',
                '--system-address', connection,
            ]
            if self.config.sitl:
                cmd.append('--sitl')
            cmd.extend(self.config.orchestra_args)
            return cmd, env

        self.get_logger().warning(f'Unknown service requested: {service}')
        return None

    def _resolve_mavlink_connection(self) -> str:
        if self.config.connection:
            return self.config.connection
        if self.config.sitl:
            return self.config.sitl_connection
        return self.config.serial_port

    def _resolve_orchestra_connection(self) -> str:
        if self.config.orchestra_connection:
            return self.config.orchestra_connection
        if self.config.sitl:
            return self.config.orchestra_sitl
        return self.config.orchestra_hardware

    def _poll_processes(self):
        for service, proc in list(self.processes.items()):
            code = proc.poll()
            if code is not None:
                self.get_logger().warning(f'{service} exited with code {code}')
                self._publish_status(service, 'exited', {'returncode': code})
                self.processes.pop(service, None)

    def _stop_processes(self):
        for service, proc in list(self.processes.items()):
            if proc.poll() is None:
                try:
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
            self._publish_status(service, 'stopped', {'returncode': proc.poll()})
            self.processes.pop(service, None)

    def _publish_status(self, service: str, status: str, extra: Optional[dict] = None):
        payload = {'service': service, 'status': status, 'timestamp': time.time()}
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def _parse_args(argv: List[str]) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(description='Waterfall ROS2 service manager')
    parser.add_argument('--services', nargs='+', choices=SERVICE_CHOICES,
                        help='Subset of services to start (default: firehose + uniform_pump)')
    parser.add_argument('--all', action='store_true', help='Start all services')
    parser.add_argument('--sitl', action='store_true', help='Use SITL connections for all services')
    parser.add_argument('--connection', help='Shared MAVLink connection string for Firehose/Inject (udp/tcp/serial)')
    parser.add_argument('--sitl-connection', default='udp:127.0.0.1:14550', help='Default SITL MAVLink endpoint')
    parser.add_argument('--serial-port', default='/dev/ttyTHS3', help='Serial port for hardware mode')
    parser.add_argument('--serial-baud', default=115200, type=int, help='Baud rate for hardware serial mode')

    parser.add_argument('--firehose-rate', default=100, type=int, help='Data stream request rate for firehose')
    parser.add_argument('--no-raw', dest='publish_raw', action='store_false', default=True,
                        help='Disable raw byte publishing from firehose')
    parser.add_argument('--batch-interval', type=float, default=1.0, help='Uniform pump batch interval')
    parser.add_argument('--condensation-mode', default='raw', choices=['raw', 'multi_step', 'single_step'],
                        help='Uniform pump condensation mode')
    parser.add_argument('--multi-step-count', type=int, default=3, help='Steps for multi_step mode')
    parser.add_argument('--bleeding-domain', type=float, default=15.0, help='Bleeding domain duration (s)')
    parser.add_argument('--missing-strategy', default='bleeding_average',
                        choices=['bleeding_average', 'bleeding_latest', 'null', 'zero'],
                        help='Missing data strategy for uniform pump')

    parser.add_argument('--inject-px4-path', dest='inject_px4_path', default='',
                        help='PX4 build directory required by the Inject node')

    parser.add_argument('--firehose-args', default='', help='Extra args forwarded to firehose_node')
    parser.add_argument('--pump-args', default='', help='Extra args forwarded to uniform_pump_node')
    parser.add_argument('--inject-args', default='', help='Extra args forwarded to injection_test')
    parser.add_argument('--orchestra-args', default='', help='Extra args forwarded to drone_api test_demo')

    parser.add_argument('--orchestra-connection', default=None,
                        help='Explicit MAVSDK connection string for Orchestra')
    parser.add_argument('--orchestra-sitl', default='udpin://0.0.0.0:14540',
                        help='SITL MAVSDK endpoint for Orchestra')
    parser.add_argument('--orchestra-hardware', default='serial:///dev/ttyACM0:57600',
                        help='Hardware MAVSDK endpoint for Orchestra when not using SITL')

    return parser.parse_known_args(argv)


def _build_config(args: argparse.Namespace) -> LaunchConfig:
    if args.all:
        services = SERVICE_CHOICES
    elif args.services:
        services = args.services
    else:
        services = ['firehose', 'uniform_pump']

    return LaunchConfig(
        services=services,
        sitl=bool(args.sitl),
        connection=args.connection,
        sitl_connection=args.sitl_connection,
        serial_port=args.serial_port,
        serial_baud=args.serial_baud,
        firehose_rate=args.firehose_rate,
        publish_raw=bool(args.publish_raw),
        batch_interval=args.batch_interval,
        condensation_mode=args.condensation_mode,
        multi_step_count=args.multi_step_count,
        bleeding_domain=args.bleeding_domain,
        missing_strategy=args.missing_strategy,
        inject_px4_path=args.inject_px4_path,
        firehose_args=shlex.split(args.firehose_args) if args.firehose_args else [],
        pump_args=shlex.split(args.pump_args) if args.pump_args else [],
        inject_args=shlex.split(args.inject_args) if args.inject_args else [],
        orchestra_args=shlex.split(args.orchestra_args) if args.orchestra_args else [],
        orchestra_sitl=args.orchestra_sitl,
        orchestra_hardware=args.orchestra_hardware,
        orchestra_connection=args.orchestra_connection,
    )


def main(argv=None):
    cli_args, ros_args = _parse_args(sys.argv[1:] if argv is None else argv)
    config = _build_config(cli_args)

    rclpy.init(args=ros_args)
    node = WaterfallManager(config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
