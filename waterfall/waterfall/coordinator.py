#!/usr/bin/env python3
"""
Coordinator demo: launches Firehose + UniformPump (+ Inject) in-process,
flies a simple circle (optional), monitors Firehose data, and triggers an Inject
command over ROS when a threshold is crossed.
"""

import argparse
import json
import shlex
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from waterfall.drone_api.test_demo import build_circle, _run_waypoints_with_timeout
from waterfall.drone_api.magpie import DroneAPI, Linear


class WaterfallCoordinator(Node):
    """
    Minimal orchestrator that can:
    - start Firehose, UniformPump, and Inject as child processes
    - optionally fly a circle using DroneAPI (MAVSDK)
    - monitor Firehose messages and trigger an Inject command via ROS2 topic
    """

    def __init__(self, args: argparse.Namespace):
        super().__init__('waterfall_coordinator')
        self.args = args
        self.processes: Dict[str, subprocess.Popen] = {}
        self.triggered = False
        self._lock = threading.Lock()

        # Subscriber to Firehose JSON stream
        self.subscription = self.create_subscription(
            String,
            'mav/all_messages',
            self._firehose_callback,
            100,
        )

        # Publisher to Inject command topic
        self.inject_pub = self.create_publisher(String, 'px4_injector/command', 10)

        # Start services
        self._start_services()

        # Optionally kick off circle mission
        if self.args.fly_circle:
            thread = threading.Thread(target=self._fly_circle, daemon=True)
            thread.start()

    # ---------- child process management ----------

    def destroy_node(self):
        self._stop_services()
        super().destroy_node()

    def _start_services(self):
        for svc in self.args.services:
            cmd = self._build_service_cmd(svc)
            if not cmd:
                continue
            try:
                proc = subprocess.Popen(cmd)
                self.processes[svc] = proc
                self.get_logger().info(f"Started {svc}: {' '.join(cmd)}")
            except Exception as exc:
                self.get_logger().error(f"Failed to start {svc}: {exc}")

    def _stop_services(self):
        for svc, proc in list(self.processes.items()):
            if proc.poll() is None:
                try:
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
            self.get_logger().info(f"Stopped {svc} (rc={proc.poll()})")
            self.processes.pop(svc, None)

    def _build_service_cmd(self, svc: str) -> Optional[List[str]]:
        base_env = []
        if svc == 'firehose':
            conn = self.args.connection or self.args.serial_port
            cmd = [
                'ros2', 'run', 'waterfall', 'firehose_service',
                '--connection', conn,
                '--serial-baud', str(self.args.serial_baud),
            ]
            if self.args.sitl:
                cmd.append('--sitl')
            if self.args.firehose_args:
                cmd.extend(shlex.split(self.args.firehose_args))
            return cmd

        if svc == 'uniform_pump':
            cmd = [
                'ros2', 'run', 'waterfall', 'uniform_pump_service',
                '--batch-interval', str(self.args.batch_interval),
                '--condensation-mode', self.args.condensation_mode,
                '--multi-step-count', str(self.args.multi_step_count),
                '--bleeding-domain', str(self.args.bleeding_domain),
                '--missing-strategy', self.args.missing_strategy,
            ]
            if self.args.pump_args:
                cmd.extend(shlex.split(self.args.pump_args))
            return cmd

        if svc == 'inject':
            cmd = [
                'ros2', 'run', 'waterfall', 'inject_service',
                '--px4-build-path', self.args.px4_build_path,
                '--serial-baud', str(self.args.serial_baud),
                '--sitl-connection', self.args.sitl_connection,
                '--serial-port', self.args.serial_port,
            ]
            if self.args.connection:
                cmd.extend(['--connection', self.args.connection])
            if self.args.sitl:
                cmd.append('--sitl')
            if self.args.inject_args:
                cmd.extend(shlex.split(self.args.inject_args))
            return cmd

        return None

    # ---------- Firehose monitoring ----------

    def _firehose_callback(self, msg: String):
        if self.triggered:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        msg_type = payload.get('_msg_type')
        if self.args.trigger_msg_type and msg_type != self.args.trigger_msg_type:
            return

        if self.args.trigger_field not in payload:
            return

        try:
            value = float(payload[self.args.trigger_field])
        except Exception:
            return

        if value >= self.args.trigger_threshold:
            with self._lock:
                if self.triggered:
                    return
                self.triggered = True
            self.get_logger().info(
                f"Trigger crossed: {self.args.trigger_field}={value:.3f} >= {self.args.trigger_threshold}"
            )
            self._publish_inject_command()

    def _publish_inject_command(self):
        command = {
            "action": "edit",
            "file": self.args.inject_file,
            "modifications": self.args.inject_modifications,
        }
        msg = String()
        msg.data = json.dumps(command)
        self.inject_pub.publish(msg)
        self.get_logger().info(f"Sent inject command: {command}")

    # ---------- Simple circle mission ----------

    def _fly_circle(self):
        """
        Run a one-off circle using DroneAPI (MAVSDK).
        This runs in a background thread so ROS spin can continue.
        """
        try:
            loop = rclpy.get_default_context()
        except Exception:
            pass

        async def run():
            drone = DroneAPI(
                system_address=self.args.system_address,
                default_interpolation=Linear,
                control_rate_hz=20.0,
                max_speed_m_s=1.5,
                use_velocity_command=True,
            )
            try:
                await drone.begin_mission(initial_altitude=self.args.circle_altitude, yaw=0.0)
                waypoints = build_circle(radius=self.args.circle_radius, alt=self.args.circle_altitude, samples=36)
                await _run_waypoints_with_timeout(
                    drone,
                    waypoints,
                    Linear,
                    segment_timeout=30.0,
                    threshold=0.25,
                )
                await drone.end_mission()
            finally:
                await drone.shutdown()

        import asyncio
        asyncio.run(run())


def _parse_modifications(pairs: List[str]) -> Dict[str, float]:
    mods: Dict[str, float] = {}
    for pair in pairs:
        if '=' not in pair:
            continue
        key, val = pair.split('=', 1)
        try:
            mods[key] = float(val)
        except ValueError:
            continue
    return mods


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="Waterfall coordinator demo")
    parser.add_argument('--services', nargs='+', default=['firehose', 'uniform_pump', 'inject'],
                        help='Services to start (firehose, uniform_pump, inject)')
    parser.add_argument('--sitl', action='store_true', help='Use SITL for Firehose/Inject')
    parser.add_argument('--connection', help='MAVLink connection (udp/tcp/serial:/dev/ttyXXX:baud)')
    parser.add_argument('--sitl-connection', default='udp:127.0.0.1:14550', help='SITL MAVLink endpoint')
    parser.add_argument('--serial-port', default='/dev/ttyTHS3', help='Serial port for hardware')
    parser.add_argument('--serial-baud', default=115200, type=int, help='Baud rate for hardware serial')
    parser.add_argument('--px4-build-path', default='/path/to/PX4-Autopilot/build/px4_sitl_default',
                        help='PX4 build path for Inject service')

    parser.add_argument('--firehose-args', default='', help='Extra args for Firehose')
    parser.add_argument('--pump-args', default='', help='Extra args for UniformPump')
    parser.add_argument('--inject-args', default='', help='Extra args for Inject')

    parser.add_argument('--batch-interval', type=float, default=1.0, help='UniformPump batch interval')
    parser.add_argument('--condensation-mode', default='raw', choices=['raw', 'multi_step', 'single_step'],
                        help='UniformPump condensation mode')
    parser.add_argument('--multi-step-count', type=int, default=3, help='UniformPump steps for multi_step')
    parser.add_argument('--bleeding-domain', type=float, default=15.0, help='UniformPump bleeding domain seconds')
    parser.add_argument('--missing-strategy', default='bleeding_average',
                        choices=['bleeding_average', 'bleeding_latest', 'null', 'zero'],
                        help='UniformPump missing data strategy')

    parser.add_argument('--trigger-msg-type', default='SCALED_IMU', help='Firehose msg type to watch')
    parser.add_argument('--trigger-field', default='xacc', help='Field to threshold on within the message')
    parser.add_argument('--trigger-threshold', type=float, default=5.0, help='Threshold value to trigger Inject')

    parser.add_argument('--inject-file', default='ROMFS/px4fmu_common/init.d/rc.autostart',
                        help='PX4 config file (relative to px4_build_path) for Inject edit action')
    parser.add_argument('--inject-set', nargs='*', default=['MC_ROLL_P=-7.0', 'MC_PITCH_P=-7.0'],
                        help='Param modifications to request via Inject (KEY=VALUE pairs)')

    parser.add_argument('--fly-circle', action='store_true', help='Run a circle mission using DroneAPI')
    parser.add_argument('--system-address', default='udp://:14540',
                        help='MAVSDK system address for circle mission')
    parser.add_argument('--circle-radius', type=float, default=2.5, help='Circle radius (m)')
    parser.add_argument('--circle-altitude', type=float, default=3.0, help='Circle altitude (m)')

    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    args.inject_modifications = _parse_modifications(args.inject_set)

    rclpy.init(args=None)
    node = WaterfallCoordinator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
