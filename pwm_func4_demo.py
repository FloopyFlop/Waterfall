#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

try:
    from waterfall.px4_param_api import PX4ParamAPI, PX4ParamType
    PX4_C_API_AVAILABLE = True
except Exception as e:
    print(f"Warning: PX4 C API not available: {e}")
    PX4_C_API_AVAILABLE = False


class PWMFunc4Demo(Node):
    def __init__(self, cli_args=None):
        super().__init__('pwm_func4_demo')

        # Declare and get parameters
        self.declare_parameter('px4_build_path', '')
        self.declare_parameter('use_sitl', True)
        self.declare_parameter('sitl_connection', 'udp:127.0.0.1:14550')
        self.declare_parameter('serial_port', '/dev/ttyTHS3')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('mavlink_connection', '')
        self.declare_parameter('target_value', 1)  # Default PWM function value

        self._apply_cli_overrides(cli_args)

        self.px4_build_path = self.get_parameter('px4_build_path').get_parameter_value().string_value
        self.use_sitl = bool(self.get_parameter('use_sitl').get_parameter_value().bool_value)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baud = int(self.get_parameter('serial_baud').get_parameter_value().integer_value)
        self.target_value = int(self.get_parameter('target_value').get_parameter_value().integer_value)

        if not self.px4_build_path:
            self.get_logger().error('PX4 build path not provided! Use --ros-args -p px4_build_path:=/path/to/px4')
            sys.exit(1)

        self.px4_build_path = Path(self.px4_build_path)

        if not self.px4_build_path.exists():
            self.get_logger().error(f'PX4 build path does not exist: {self.px4_build_path}')
            sys.exit(1)

        self.get_logger().info(f'PWM FUNC4 Demo initialized with build path: {self.px4_build_path}')
        self.get_logger().info(f'Target PWM_MAIN_FUNC4 value: {self.target_value}')

        # Try to initialize PX4 C API (lowest level access)
        self.px4_api = None
        if PX4_C_API_AVAILABLE:
            try:
                self.get_logger().info('Attempting to load PX4 C API for direct parameter access...')
                self.px4_api = PX4ParamAPI()
                self.get_logger().info('PX4 C API loaded successfully! Using direct parameter access.')
            except Exception as e:
                self.get_logger().warning(f'Failed to load PX4 C API: {e}. Falling back to MAVLink.')
                self.px4_api = None
        else:
            self.get_logger().warning('PX4 C API not available. Will use MAVLink fallback.')

        # MAVLink connection as fallback
        self.mavlink_conn_string = self._resolve_mavlink_connection()
        self.mav = None

        # Only try MAVLink if C API is not available
        if self.px4_api is None and MAVLINK_AVAILABLE:
            try:
                self.get_logger().info(f'Connecting to MAVLink at {self.mavlink_conn_string}')
                conn_kwargs = {}
                if not self.use_sitl and self.serial_baud:
                    conn_kwargs['baud'] = self.serial_baud
                self.mav = mavutil.mavlink_connection(self.mavlink_conn_string, **conn_kwargs)
                self.get_logger().info('Waiting for MAVLink heartbeat...')
                self.mav.wait_heartbeat(timeout=5)
                self.get_logger().info(f'MAVLink connected! System {self.mav.target_system} Component {self.mav.target_component}')
            except Exception as e:
                self.get_logger().warning(f'MAVLink connection failed: {e}.')
                self.mav = None
        elif self.px4_api is None:
            self.get_logger().warning('pymavlink not available. Install with: pip install pymavlink')

        self.get_logger().info('Node ready. Starting PWM_MAIN_FUNC4 demo in 2 seconds...')
        self.demo_timer = self.create_timer(2.0, self.run_pwm_func4_demo)

    def _apply_cli_overrides(self, cli_args):
        """Allow simple --sitl/--connection flag usage without long ROS args."""
        if cli_args is None:
            return

        updates = []
        if getattr(cli_args, 'px4_build_path', None):
            updates.append(Parameter('px4_build_path', Parameter.Type.STRING, cli_args.px4_build_path))
        if getattr(cli_args, 'sitl', None) is True:
            updates.append(Parameter('use_sitl', Parameter.Type.BOOL, True))
        if getattr(cli_args, 'sitl_connection', None):
            updates.append(Parameter('sitl_connection', Parameter.Type.STRING, cli_args.sitl_connection))
        if getattr(cli_args, 'serial_port', None):
            updates.append(Parameter('serial_port', Parameter.Type.STRING, cli_args.serial_port))
        if getattr(cli_args, 'serial_baud', None):
            updates.append(Parameter('serial_baud', Parameter.Type.INTEGER, cli_args.serial_baud))
        if getattr(cli_args, 'connection', None):
            updates.append(Parameter('mavlink_connection', Parameter.Type.STRING, cli_args.connection))
        if getattr(cli_args, 'target_value', None):
            updates.append(Parameter('target_value', Parameter.Type.INTEGER, cli_args.target_value))

        if updates:
            self.set_parameters(updates)

    def _resolve_mavlink_connection(self):
        explicit = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        sitl_conn = self.get_parameter('sitl_connection').get_parameter_value().string_value

        if explicit:
            return explicit

        if self.use_sitl:
            return sitl_conn

        return self.serial_port

    def run_pwm_func4_demo(self):
        """Run the focused PWM_MAIN_FUNC4 parameter demo"""
        self.demo_timer.cancel()  # Run only once

        self.get_logger().info('='*60)
        self.get_logger().info('STARTING PWM_MAIN_FUNC4 DEMO')
        self.get_logger().info(f'Using: {"PX4 C API (Direct)" if self.px4_api else "MAVLink Protocol"}')
        self.get_logger().info('='*60)

        # First read current value
        self.get_logger().info('')
        self.get_logger().info('--- Reading Current PWM_MAIN_FUNC4 Value ---')
        self.read_parameter('PWM_MAIN_FUNC4')
        
        # Set new value
        self.get_logger().info('')
        self.get_logger().info(f'--- Setting PWM_MAIN_FUNC4 to {self.target_value} ---')
        self.set_parameter_int32('PWM_MAIN_FUNC4', self.target_value)
        
        # Verify the change
        self.get_logger().info('')
        self.get_logger().info('--- Verifying PWM_MAIN_FUNC4 New Value ---')
        self.read_parameter('PWM_MAIN_FUNC4')

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('PWM_MAIN_FUNC4 DEMO COMPLETED')
        self.get_logger().info('='*60)

    def read_parameter(self, param_name):
        """Read and log current parameter value"""
        if self.px4_api:
            # Use direct C API
            self.get_logger().info('Reading parameter via PX4 C API...')
            try:
                self.get_logger().info(f'Reading parameter: {param_name}')

                # Get type
                param_type = self.px4_api.get_param_type(param_name)
                if param_type is None:
                    self.get_logger().warning(f'  Parameter not found: {param_name}')
                    return

                type_str = 'FLOAT' if param_type == PX4ParamType.FLOAT else 'INT32'
                self.get_logger().info(f'  Type: {type_str}')

                # Get value
                value = self.px4_api.get_param(param_name)
                if value is not None:
                    self.get_logger().info(f'  CURRENT VALUE: {param_name} = {value}')
                else:
                    self.get_logger().warning(f'  Failed to read value')

            except Exception as e:
                self.get_logger().error(f'Failed to read parameter {param_name}: {e}')

        elif self.mav:
            # Fallback to MAVLink
            self.get_logger().info('Reading parameter via MAVLink...')
            try:
                self.get_logger().info(f'Reading parameter: {param_name}')

                # Request parameter
                self.mav.mav.param_request_read_send(
                    self.mav.target_system,
                    self.mav.target_component,
                    param_name.encode('utf-8'),
                    -1
                )

                # Wait for response
                start_time = time.time()
                timeout = 2.0
                found = False

                while (time.time() - start_time) < timeout:
                    msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                    if msg:
                        # Handle both bytes and str for param_id
                        param_id = msg.param_id
                        if isinstance(param_id, bytes):
                            param_id = param_id.decode('utf-8').rstrip('\x00')
                        elif isinstance(param_id, str):
                            param_id = param_id.rstrip('\x00')

                        # Check if this is the parameter we requested
                        if param_id == param_name:
                            param_value = msg.param_value
                            self.get_logger().info(f'  CURRENT VALUE: {param_id} = {param_value}')
                            self.get_logger().info(f'  Type: {msg.param_type}, Index: {msg.param_index}/{msg.param_count}')
                            found = True
                            break

                if not found:
                    self.get_logger().warning(f'  No response for parameter {param_name}')

            except Exception as e:
                self.get_logger().error(f'Failed to read parameter {param_name}: {e}')
        else:
            self.get_logger().error('No parameter access method available (neither C API nor MAVLink)')

    def set_parameter_int32(self, param_name, value):
        """Set INT32 parameter"""
        if self.px4_api:
            # Use direct C API
            self.get_logger().info('Setting parameter via PX4 C API (Direct)...')
            try:
                self.get_logger().info(f'Setting parameter: {param_name} = {value}')
                self.get_logger().info(f'  C API call: px4_api.set_param("{param_name}", {value})')

                # Set parameter
                success = self.px4_api.set_param(param_name, value)

                if success:
                    self.get_logger().info(f'  SUCCESS: C API confirmed parameter set')

                    # Read back to verify
                    read_value = self.px4_api.get_param(param_name)
                    if read_value is not None:
                        self.get_logger().info(f'  VERIFIED: Read back value = {read_value}')

                        if abs(float(read_value) - float(value)) > 0.001:
                            self.get_logger().warning(f'  WARNING: Set value {value} differs from read {read_value}')
                    else:
                        self.get_logger().warning(f'  Could not read back parameter')
                else:
                    self.get_logger().error(f'  FAILED: C API returned error')

            except Exception as e:
                self.get_logger().error(f'Failed to set parameter {param_name}: {e}')

        elif self.mav:
            # Fallback to MAVLink
            self.get_logger().info('Setting INT32 parameter via MAVLink...')
            try:
                param_value = int(value)

                self.get_logger().info(f'Setting INT32 parameter: {param_name} = {param_value}')
                self.get_logger().info(f'  MAVLink call: mav.mav.param_set_send(...)')

                # Send parameter set command with INT32 type
                self.mav.mav.param_set_send(
                    self.mav.target_system,
                    self.mav.target_component,
                    param_name.encode('utf-8'),
                    float(param_value),  # MAVLink sends as float but with INT32 type flag
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )

                # Wait for ACK
                self.get_logger().info(f'  Waiting for acknowledgment...')
                start_time = time.time()
                timeout = 2.0
                found = False

                while (time.time() - start_time) < timeout:
                    msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                    if msg:
                        # Handle both bytes and str for param_id
                        param_id = msg.param_id
                        if isinstance(param_id, bytes):
                            param_id = param_id.decode('utf-8').rstrip('\x00')
                        elif isinstance(param_id, str):
                            param_id = param_id.rstrip('\x00')

                        # Check if this is the parameter we set
                        if param_id == param_name:
                            new_value = int(msg.param_value)
                            self.get_logger().info(f'  SUCCESS: {param_id} confirmed at {new_value}')

                            if new_value != param_value:
                                self.get_logger().warning(f'  WARNING: Set value {param_value} differs from confirmed {new_value}')
                            found = True
                            break

                if not found:
                    self.get_logger().warning(f'  No acknowledgment received for {param_name}')

            except Exception as e:
                self.get_logger().error(f'Failed to set INT32 parameter {param_name}: {e}')
        else:
            self.get_logger().error('No parameter access method available (neither C API nor MAVLink)')


def _parse_cli_args(argv):
    parser = argparse.ArgumentParser(
        add_help=False,
        description='PWM_MAIN_FUNC4 demo script - minimal version for parameter injection.'
    )
    parser.add_argument('--px4-build-path', dest='px4_build_path', help='PX4 build directory containing firmware artifacts.')
    parser.add_argument('--sitl', action='store_true', help='Use SITL UDP connection instead of hardware serial.')
    parser.add_argument('--connection', help='Explicit MAVLink connection string (overrides sitl/serial resolution).')
    parser.add_argument('--sitl-connection', dest='sitl_connection', help='SITL MAVLink connection string override.')
    parser.add_argument('--serial-port', dest='serial_port', help='Serial port for hardware connection.')
    parser.add_argument('--serial-baud', dest='serial_baud', type=int, help='Serial baud rate for hardware connection.')
    parser.add_argument('--target-value', dest='target_value', type=int, default=1, help='Target value for PWM_MAIN_FUNC4 parameter (default: 1)')
    return parser.parse_known_args(argv)


def main(args=None):
    cli_args, ros_args = _parse_cli_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=ros_args)

    try:
        node = PWMFunc4Demo(cli_args=cli_args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()