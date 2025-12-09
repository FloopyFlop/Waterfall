#!/usr/bin/env python3
"""
MAV FIRE HOSE - Maximum frequency raw MAVLink data streaming to ROS2
Connects to PX4 via pymavlink and publishes ALL raw messages at maximum rate
WITH LIVE TERMINAL DASHBOARD
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt8MultiArray, Header
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry

from pymavlink import mavutil
import json
import time
import threading
from collections import defaultdict, deque
from datetime import datetime
from queue import Queue

# Rich library for beautiful terminal UI
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.console import Console
from rich.text import Text
from rich.align import Align


class MavFirehoseNode(Node):
    """
    ROS2 Node that creates a DATA FIREHOSE from MAVLink to ROS2
    Publishes as much raw data as possible with minimal processing
    WITH LIVE TERMINAL DASHBOARD
    """

    def __init__(self):
        super().__init__('mav_firehose_node')

        # Parameters
        # USE_SITL flag: False = on-device mode (default), True = SITL simulation mode
        self.declare_parameter('use_sitl', False)
        self.declare_parameter('connection_string', '')  # Will be set based on use_sitl
        self.declare_parameter('serial_port', '/dev/ttyTHS3')  # Default on-device serial port
        self.declare_parameter('serial_baud', 115200)  # Default baud rate
        self.declare_parameter('sitl_address', 'udp:127.0.0.1:14550')  # SITL default
        self.declare_parameter('data_stream_rate', 100)  # Request 100Hz for everything
        self.declare_parameter('publish_raw_bytes', True)  # Publish raw MAVLink bytes

        use_sitl = self.get_parameter('use_sitl').value

        # Determine connection string based on mode
        connection_override = self.get_parameter('connection_string').value
        if connection_override:
            # If connection_string is explicitly provided, use it
            self.connection_string = connection_override
        elif use_sitl:
            # SITL mode: use UDP connection
            self.connection_string = self.get_parameter('sitl_address').value
        else:
            # On-device mode: use serial connection (DEFAULT)
            # pymavlink serial format: just the device path, baud is set separately
            self.serial_port = self.get_parameter('serial_port').value
            self.serial_baud = self.get_parameter('serial_baud').value
            self.connection_string = self.serial_port

        self.stream_rate = self.get_parameter('data_stream_rate').value
        self.publish_raw = self.get_parameter('publish_raw_bytes').value

        # Statistics
        self.msg_count = defaultdict(int)
        self.total_bytes = 0
        self.start_time = time.time()

        # Latest data storage for dashboard
        self.latest_data = {
            'heartbeat': {},
            'attitude': {},
            'position': {},
            'velocity': {},
            'imu': {},
            'gps': {},
            'battery': {},
            'rc_channels': [],
            'servo_output': [],
            'sys_status': {},
            'last_messages': deque(maxlen=20),  # Last 20 message types
        }

        # Store ALL raw message data for comprehensive display
        self.all_raw_messages = {}  # Dict of msg_type -> latest msg_dict
        self.data_lock = threading.Lock()

        # Logs for dashboard - store as (timestamp, message, style) tuples
        self.log_messages = deque(maxlen=30)
        self.log_lock = threading.Lock()

        # Connection error tracking
        self.connection_error = None

        # Pagination for data display
        self.current_page = 0
        self.items_per_page = 10  # Items per page
        self.page_lock = threading.Lock()
        self.key_queue = Queue()  # Queue for key presses

        # Track active message types (those received in last 2 seconds)
        self.last_msg_time = {}  # msg_type -> last receive time

        # Create publishers for different data streams
        self._create_publishers()

        # MAVLink connection
        self.mav_conn = None
        self.connection_alive = False

        # Add log
        self._add_log('Starting MAV FIRE HOSE', 'INFO', 'bold green')
        self._add_log(f'Connection: {self.connection_string}', 'INFO')
        self._add_log(f'Stream rate: {self.stream_rate} Hz', 'INFO')

        # Start MAVLink connection in separate thread
        self.mav_thread = threading.Thread(target=self._mavlink_loop, daemon=True)
        self.mav_thread.start()

        # Statistics timer
        self.create_timer(0.1, self._update_statistics)  # 10Hz update

        # Dashboard will be started by main()
        self.dashboard_active = False

    def process_keys(self):
        """Process any queued key presses"""
        while not self.key_queue.empty():
            key = self.key_queue.get()
            if key == 'next':
                with self.page_lock, self.data_lock:
                    total_messages = len(self.all_raw_messages)
                    if total_messages > 0:
                        max_page = max(0, (total_messages - 1) // self.items_per_page)
                        self.current_page = min(self.current_page + 1, max_page)
            elif key == 'prev':
                with self.page_lock:
                    self.current_page = max(0, self.current_page - 1)
            elif key == 'more':
                with self.page_lock:
                    self.items_per_page = min(50, self.items_per_page + 1)
            elif key == 'less':
                with self.page_lock:
                    self.items_per_page = max(1, self.items_per_page - 1)

    def _add_log(self, message, level='INFO', style=None):
        """Add a log message to the dashboard"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        # Determine style based on level if not provided
        if style is None:
            if level == 'ERROR':
                style = 'red'
            elif level == 'WARN':
                style = 'yellow'
            elif level == 'SUCCESS':
                style = 'green'
            else:
                style = 'white'

        with self.log_lock:
            self.log_messages.append((timestamp, message, style))

    def _create_publishers(self):
        """Create all ROS2 publishers for maximum data throughput"""

        # Raw MAVLink message publisher (JSON serialized)
        self.raw_msg_pub = self.create_publisher(String, 'mav/raw/json', 100)

        # Raw bytes publisher (for AI models that want pure bytes)
        if self.publish_raw:
            self.raw_bytes_pub = self.create_publisher(UInt8MultiArray, 'mav/raw/bytes', 100)

        # Attitude and orientation
        self.imu_pub = self.create_publisher(Imu, 'mav/imu', 100)
        self.attitude_pub = self.create_publisher(Vector3Stamped, 'mav/attitude', 100)
        self.attitude_quat_pub = self.create_publisher(PoseStamped, 'mav/attitude_quaternion', 100)

        # Position and velocity
        self.local_pos_pub = self.create_publisher(PoseStamped, 'mav/local_position', 100)
        self.global_pos_pub = self.create_publisher(NavSatFix, 'mav/global_position', 100)
        self.velocity_pub = self.create_publisher(TwistStamped, 'mav/velocity', 100)
        self.odometry_pub = self.create_publisher(Odometry, 'mav/odometry', 100)

        # Sensor data
        self.magnetometer_pub = self.create_publisher(MagneticField, 'mav/magnetometer', 100)
        self.pressure_pub = self.create_publisher(FluidPressure, 'mav/pressure', 100)
        self.battery_pub = self.create_publisher(BatteryState, 'mav/battery', 100)

        # RC and actuators
        self.rc_channels_pub = self.create_publisher(Float64MultiArray, 'mav/rc_channels', 100)
        self.servo_output_pub = self.create_publisher(Float64MultiArray, 'mav/servo_output', 100)

        # System status
        self.heartbeat_pub = self.create_publisher(String, 'mav/heartbeat', 100)
        self.sys_status_pub = self.create_publisher(String, 'mav/sys_status', 100)

        # All other messages categorized by type
        self.all_messages_pub = self.create_publisher(String, 'mav/all_messages', 100)

        # Statistics
        self.stats_pub = self.create_publisher(String, 'mav/statistics', 10)

    def _mavlink_loop(self):
        """Main MAVLink reception loop - runs in separate thread for maximum throughput"""

        try:
            # Connect to MAVLink
            self._add_log('Connecting to MAVLink...', 'INFO')

            # For serial connections, pass baud rate parameter
            if hasattr(self, 'serial_baud'):
                self.mav_conn = mavutil.mavlink_connection(
                    self.connection_string,
                    baud=self.serial_baud,
                    timeout=30
                )
            else:
                # UDP/TCP connections don't need baud rate
                self.mav_conn = mavutil.mavlink_connection(self.connection_string, timeout=30)

            # Wait for first heartbeat
            self._add_log('Waiting for heartbeat...', 'INFO')
            heartbeat = self.mav_conn.wait_heartbeat(timeout=30)

            if heartbeat is None:
                self._add_log('No heartbeat received! Is PX4 SITL running?', 'ERROR')
                self.connection_error = 'No heartbeat - check PX4 SITL'
                return

            self.connection_alive = True
            self.connection_error = None  # Clear any previous errors
            self._add_log(f'CONNECTED! System {self.mav_conn.target_system} Component {self.mav_conn.target_component}', 'SUCCESS', 'bold green')

            # Request ALL data streams at maximum rate
            self._request_all_data_streams()

            # Main reception loop - NO SLEEP, maximum throughput!
            while rclpy.ok() and self.connection_alive:
                # Receive message (non-blocking with timeout)
                msg = self.mav_conn.recv_match(blocking=True, timeout=1.0)

                if msg is not None:
                    # Process and publish the message
                    self._process_mavlink_message(msg)
                else:
                    # Check if we're still getting data
                    pass

        except Exception as e:
            self._add_log(f'MAVLink connection error: {str(e)}', 'ERROR')
            self.connection_error = str(e)
            self.connection_alive = False

    def _request_all_data_streams(self):
        """Request ALL available data streams at maximum rate"""

        self._add_log(f'Requesting all data streams at {self.stream_rate} Hz', 'INFO')

        # Request all MAVLink data streams
        stream_ids = [
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        ]

        for stream_id in stream_ids:
            self.mav_conn.mav.request_data_stream_send(
                self.mav_conn.target_system,
                self.mav_conn.target_component,
                stream_id,
                self.stream_rate,
                1  # Start streaming
            )

        self._add_log('Data streams activated!', 'SUCCESS', 'bold green')

    def _convert_to_json_serializable(self, obj):
        """Convert non-JSON-serializable objects to serializable formats"""
        if isinstance(obj, bytearray):
            return list(obj)
        elif isinstance(obj, bytes):
            return list(obj)
        elif isinstance(obj, dict):
            return {k: self._convert_to_json_serializable(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [self._convert_to_json_serializable(item) for item in obj]
        else:
            return obj

    def _process_mavlink_message(self, msg):
        """Process and publish a MAVLink message with minimal overhead"""

        msg_type = msg.get_type()
        self.msg_count[msg_type] += 1
        self.last_msg_time[msg_type] = time.time()  # Track last receive time

        # Store for dashboard
        with self.data_lock:
            self.latest_data['last_messages'].append(msg_type)

        # Publish raw bytes if enabled
        if self.publish_raw:
            raw_bytes = UInt8MultiArray()
            raw_bytes.data = list(msg.get_msgbuf())
            self.raw_bytes_pub.publish(raw_bytes)
            self.total_bytes += len(raw_bytes.data)

        # Publish as JSON (for debugging and flexible consumption)
        try:
            msg_dict = msg.to_dict()
            msg_dict['_timestamp'] = time.time()
            msg_dict['_msg_type'] = msg_type

            # Convert any bytearray/bytes to lists for JSON serialization
            msg_dict = self._convert_to_json_serializable(msg_dict)

            # Store raw message for dashboard display
            with self.data_lock:
                self.all_raw_messages[msg_type] = msg_dict

            json_msg = String()
            json_msg.data = json.dumps(msg_dict)
            self.all_messages_pub.publish(json_msg)

            # Also publish raw JSON version
            self.raw_msg_pub.publish(json_msg)
        except Exception as e:
            # If JSON serialization fails, log it but don't crash
            if self.msg_count[msg_type] == 1:  # Only log first occurrence per message type
                self._add_log(f'JSON serialization warning for {msg_type}: {str(e)}', 'WARN')

        # Parse and publish to specific topics based on message type
        self._publish_typed_message(msg, msg_type)

    def _publish_typed_message(self, msg, msg_type):
        """Publish to specialized ROS2 topics based on MAVLink message type"""

        now = self.get_clock().now().to_msg()

        # HEARTBEAT
        if msg_type == 'HEARTBEAT':
            with self.data_lock:
                self.latest_data['heartbeat'] = {
                    'type': msg.type,
                    'autopilot': msg.autopilot,
                    'base_mode': msg.base_mode,
                    'custom_mode': msg.custom_mode,
                    'system_status': msg.system_status,
                }

            heartbeat_msg = String()
            heartbeat_msg.data = json.dumps(self.latest_data['heartbeat'])
            self.heartbeat_pub.publish(heartbeat_msg)

        # SYS_STATUS
        elif msg_type == 'SYS_STATUS':
            with self.data_lock:
                self.latest_data['sys_status'] = {
                    'voltage': msg.voltage_battery / 1000.0,
                    'current': msg.current_battery / 100.0,
                    'battery_remaining': msg.battery_remaining,
                    'load': msg.load / 10.0,
                }

            status_msg = String()
            status_msg.data = json.dumps(msg.to_dict())
            self.sys_status_pub.publish(status_msg)

        # ATTITUDE
        elif msg_type == 'ATTITUDE':
            with self.data_lock:
                self.latest_data['attitude'] = {
                    'roll': msg.roll,
                    'pitch': msg.pitch,
                    'yaw': msg.yaw,
                    'rollspeed': msg.rollspeed,
                    'pitchspeed': msg.pitchspeed,
                    'yawspeed': msg.yawspeed,
                }

            # Publish as Vector3
            attitude = Vector3Stamped()
            attitude.header.stamp = now
            attitude.header.frame_id = 'base_link'
            attitude.vector.x = msg.roll
            attitude.vector.y = msg.pitch
            attitude.vector.z = msg.yaw
            self.attitude_pub.publish(attitude)

        # ATTITUDE_QUATERNION
        elif msg_type == 'ATTITUDE_QUATERNION':
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'base_link'
            pose.pose.orientation.w = msg.q1
            pose.pose.orientation.x = msg.q2
            pose.pose.orientation.y = msg.q3
            pose.pose.orientation.z = msg.q4
            self.attitude_quat_pub.publish(pose)

        # IMU (SCALED_IMU, RAW_IMU, HIGHRES_IMU)
        elif msg_type in ['SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3', 'RAW_IMU', 'HIGHRES_IMU']:
            if hasattr(msg, 'xacc'):
                with self.data_lock:
                    self.latest_data['imu'] = {
                        'accel_x': msg.xacc / 1000.0,
                        'accel_y': msg.yacc / 1000.0,
                        'accel_z': msg.zacc / 1000.0,
                        'gyro_x': msg.xgyro / 1000.0,
                        'gyro_y': msg.ygyro / 1000.0,
                        'gyro_z': msg.zgyro / 1000.0,
                    }

                imu = Imu()
                imu.header.stamp = now
                imu.header.frame_id = 'base_link'
                imu.linear_acceleration.x = msg.xacc / 1000.0
                imu.linear_acceleration.y = msg.yacc / 1000.0
                imu.linear_acceleration.z = msg.zacc / 1000.0
                imu.angular_velocity.x = msg.xgyro / 1000.0
                imu.angular_velocity.y = msg.ygyro / 1000.0
                imu.angular_velocity.z = msg.zgyro / 1000.0
                self.imu_pub.publish(imu)

            # Also publish magnetometer if available
            if hasattr(msg, 'xmag'):
                mag = MagneticField()
                mag.header.stamp = now
                mag.header.frame_id = 'base_link'
                mag.magnetic_field.x = msg.xmag / 1000.0
                mag.magnetic_field.y = msg.ymag / 1000.0
                mag.magnetic_field.z = msg.zmag / 1000.0
                self.magnetometer_pub.publish(mag)

        # LOCAL_POSITION_NED
        elif msg_type == 'LOCAL_POSITION_NED':
            with self.data_lock:
                self.latest_data['position'] = {
                    'x': msg.x,
                    'y': msg.y,
                    'z': msg.z,
                }
                self.latest_data['velocity'] = {
                    'vx': msg.vx,
                    'vy': msg.vy,
                    'vz': msg.vz,
                }

            # Publish position
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x = msg.x
            pose.pose.position.y = msg.y
            pose.pose.position.z = msg.z
            self.local_pos_pub.publish(pose)

            # Publish velocity
            twist = TwistStamped()
            twist.header.stamp = now
            twist.header.frame_id = 'base_link'
            twist.twist.linear.x = msg.vx
            twist.twist.linear.y = msg.vy
            twist.twist.linear.z = msg.vz
            self.velocity_pub.publish(twist)

            # Publish as odometry (combined)
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'map'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = msg.x
            odom.pose.pose.position.y = msg.y
            odom.pose.pose.position.z = msg.z
            odom.twist.twist.linear.x = msg.vx
            odom.twist.twist.linear.y = msg.vy
            odom.twist.twist.linear.z = msg.vz
            self.odometry_pub.publish(odom)

        # GLOBAL_POSITION_INT
        elif msg_type == 'GLOBAL_POSITION_INT':
            with self.data_lock:
                self.latest_data['gps'] = {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.alt / 1000.0,
                    'relative_alt': msg.relative_alt / 1000.0,
                }

            gps = NavSatFix()
            gps.header.stamp = now
            gps.header.frame_id = 'gps'
            gps.latitude = msg.lat / 1e7
            gps.longitude = msg.lon / 1e7
            gps.altitude = msg.alt / 1000.0
            self.global_pos_pub.publish(gps)

        # SCALED_PRESSURE
        elif msg_type == 'SCALED_PRESSURE':
            pressure = FluidPressure()
            pressure.header.stamp = now
            pressure.header.frame_id = 'base_link'
            pressure.fluid_pressure = msg.press_abs * 100.0  # hPa to Pa
            self.pressure_pub.publish(pressure)

        # BATTERY_STATUS
        elif msg_type == 'BATTERY_STATUS':
            with self.data_lock:
                self.latest_data['battery'] = {
                    'voltage': msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0.0,
                    'current': msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0,
                    'remaining': msg.battery_remaining if msg.battery_remaining != -1 else 0,
                }

            battery = BatteryState()
            battery.header.stamp = now
            battery.voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0.0
            battery.current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
            battery.percentage = msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else 0.0
            self.battery_pub.publish(battery)

        # RC_CHANNELS
        elif msg_type == 'RC_CHANNELS' or msg_type == 'RC_CHANNELS_RAW':
            rc_data = [
                msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw
            ]
            if hasattr(msg, 'chan9_raw'):
                rc_data.extend([
                    msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw,
                    msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw
                ])

            with self.data_lock:
                self.latest_data['rc_channels'] = rc_data

            rc = Float64MultiArray()
            rc.data = rc_data
            self.rc_channels_pub.publish(rc)

        # SERVO_OUTPUT_RAW
        elif msg_type == 'SERVO_OUTPUT_RAW':
            servo_data = [
                msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw
            ]
            if hasattr(msg, 'servo9_raw'):
                servo_data.extend([
                    msg.servo9_raw, msg.servo10_raw, msg.servo11_raw, msg.servo12_raw,
                    msg.servo13_raw, msg.servo14_raw, msg.servo15_raw, msg.servo16_raw
                ])

            with self.data_lock:
                self.latest_data['servo_output'] = servo_data

            servo = Float64MultiArray()
            servo.data = servo_data
            self.servo_output_pub.publish(servo)

    def _update_statistics(self):
        """Update statistics (called at 10Hz)"""
        pass  # Stats are updated in the dashboard

    def generate_dashboard(self):
        """Generate the live dashboard layout with active channels and logs"""

        # Process any key presses
        self.process_keys()

        layout = Layout()
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main"),
            Layout(name="footer", size=4),
        )

        # Split main: active channels | data | logs
        layout["main"].split_row(
            Layout(name="active", ratio=1),
            Layout(name="data", ratio=2),
            Layout(name="logs", ratio=1),
        )

        # Header
        with self.page_lock, self.data_lock:
            total_msg_types = len(self.all_raw_messages)
            max_page = max(0, (total_msg_types - 1) // self.items_per_page) if total_msg_types > 0 else 0
            header_text = Text(f"MAV FIRE HOSE | Page {self.current_page + 1}/{max_page + 1} | Items/page: {self.items_per_page}",
                              style="bold magenta", justify="center")
        layout["header"].update(Panel(header_text, style="bold white"))

        # Active channels panel
        active_channels = self._generate_active_channels()
        layout["active"].update(Panel(active_channels, title="[bold green]Active Channels", border_style="green"))

        # Data panel - all message data paginated
        data_table = self._generate_paginated_data()
        layout["data"].update(Panel(data_table, title="[bold cyan]Live MAVLink Data", border_style="cyan"))

        # Logs panel
        log_text = self._generate_logs()
        layout["logs"].update(Panel(log_text, title="[bold yellow]System Logs", border_style="yellow"))

        # Footer - Statistics
        footer_content = self._generate_footer()
        layout["footer"].update(Panel(footer_content, style="bold green"))

        return layout

    def _generate_active_channels(self):
        """Generate list of active channels (received in last 2 seconds)"""
        current_time = time.time()
        text = Text()

        # Connection status
        if self.connection_alive:
            text.append("● CONNECTED\n\n", style="bold green")
        elif self.connection_error:
            text.append(f"● ERROR\n", style="bold red")
            text.append(f"{self.connection_error}\n\n", style="red")
        else:
            text.append("○ CONNECTING...\n\n", style="bold yellow")

        # Active channels (received in last 2 seconds)
        active_count = 0
        sorted_msgs = sorted(self.msg_count.items(), key=lambda x: x[1], reverse=True)

        for msg_type, count in sorted_msgs:
            last_time = self.last_msg_time.get(msg_type, 0)
            if current_time - last_time < 2.0:  # Active in last 2 seconds
                active_count += 1
                elapsed = time.time() - self.start_time
                rate = count / elapsed if elapsed > 0 else 0

                # Color code by frequency
                if rate > 50:
                    style = "bold green"
                elif rate > 10:
                    style = "green"
                elif rate > 1:
                    style = "yellow"
                else:
                    style = "white"

                text.append(f"{msg_type}\n", style=style)
                text.append(f"  {rate:.1f} Hz\n", style="dim")

        if active_count == 0:
            text.append("No active channels\n", style="dim")
        else:
            text.append(f"\n[{active_count} active]", style="dim")

        return text

    def _generate_paginated_data(self):
        """Generate paginated data table"""
        with self.data_lock, self.page_lock:
            # Sort messages alphabetically
            sorted_messages = sorted(self.all_raw_messages.items())

            # Calculate pagination
            total_messages = len(sorted_messages)
            start_idx = self.current_page * self.items_per_page
            end_idx = min(start_idx + self.items_per_page, total_messages)
            page_messages = sorted_messages[start_idx:end_idx]

            # Create table
            table = Table(show_header=False, box=None)
            table.add_column("Field", style="bold cyan")
            table.add_column("Value", style="white")

            for msg_type, msg_data in page_messages:
                # Filter internal fields
                filtered_data = {k: v for k, v in msg_data.items() if not k.startswith('_')}
                if not filtered_data:
                    continue

                # Message type header
                count = self.msg_count.get(msg_type, 0)
                table.add_row(f"[bold yellow]{msg_type}[/bold yellow]", f"[dim](count: {count})[/dim]")

                # Message fields
                for field_name, field_value in sorted(filtered_data.items()):
                    formatted_value = self._format_value(field_value)
                    table.add_row(f"  {field_name}", formatted_value)

                table.add_row("", "")  # Spacer

            return table

    def _format_value(self, value):
        """Format a value for display"""
        if isinstance(value, float):
            return f"{value:.4f}"
        elif isinstance(value, list):
            if len(value) > 8:
                return f"[{', '.join(map(str, value[:8]))}...]"
            return f"[{', '.join(map(str, value))}]"
        else:
            return str(value)

    def _generate_message_list(self):
        """Generate list of all message types with rates"""

        with self.data_lock:
            text = Text()

            # Connection status
            if self.connection_alive:
                text.append("● CONNECTED\n\n", style="bold green")
            elif self.connection_error:
                text.append(f"● ERROR\n{self.connection_error}\n\n", style="bold red")
            else:
                text.append("○ CONNECTING...\n\n", style="bold yellow")

            # Sort messages by count (most frequent first)
            sorted_msgs = sorted(self.msg_count.items(), key=lambda x: x[1], reverse=True)

            for msg_type, count in sorted_msgs:
                # Calculate rate
                elapsed = time.time() - self.start_time
                rate = count / elapsed if elapsed > 0 else 0

                # Highlight high-frequency messages
                if rate > 50:
                    style = "bold green"
                elif rate > 10:
                    style = "green"
                elif rate > 1:
                    style = "yellow"
                else:
                    style = "white"

                text.append(f"{msg_type}\n", style=style)
                text.append(f"  {count} ({rate:.1f} Hz)\n", style="dim")

            return text

    def _generate_data_columns(self):
        """Generate two columns of data with pagination"""

        with self.data_lock, self.page_lock:
            # Sort messages alphabetically
            sorted_messages = sorted(self.all_raw_messages.items())

            # Calculate pagination
            total_messages = len(sorted_messages)
            max_page = max(0, (total_messages - 1) // self.items_per_page)
            start_idx = self.current_page * self.items_per_page
            end_idx = min(start_idx + self.items_per_page, total_messages)

            # Get messages for current page
            page_messages = sorted_messages[start_idx:end_idx]

            # Split into two columns
            mid_point = len(page_messages) // 2
            col1_messages = page_messages[:mid_point]
            col2_messages = page_messages[mid_point:]

            # Generate column 1
            table1 = Table(show_header=False, box=None, padding=(0, 1))
            table1.add_column("Field", style="bold cyan", width=25)
            table1.add_column("Value", style="white", overflow="fold")

            for msg_type, msg_data in col1_messages:
                filtered_data = {k: v for k, v in msg_data.items() if not k.startswith('_')}
                if not filtered_data:
                    continue

                count = self.msg_count.get(msg_type, 0)
                table1.add_row(f"[bold yellow]{msg_type}[/bold yellow]", f"[dim]({count})[/dim]")

                for field_name, field_value in sorted(filtered_data.items()):
                    formatted_value = self._format_value(field_value)
                    table1.add_row(f"  {field_name}", formatted_value)

                table1.add_row("", "")

            # Generate column 2
            table2 = Table(show_header=False, box=None, padding=(0, 1))
            table2.add_column("Field", style="bold cyan", width=25)
            table2.add_column("Value", style="white", overflow="fold")

            for msg_type, msg_data in col2_messages:
                filtered_data = {k: v for k, v in msg_data.items() if not k.startswith('_')}
                if not filtered_data:
                    continue

                count = self.msg_count.get(msg_type, 0)
                table2.add_row(f"[bold yellow]{msg_type}[/bold yellow]", f"[dim]({count})[/dim]")

                for field_name, field_value in sorted(filtered_data.items()):
                    formatted_value = self._format_value(field_value)
                    table2.add_row(f"  {field_name}", formatted_value)

                table2.add_row("", "")

            return table1, table2

    def _generate_logs(self):
        """Generate the logs panel with proper Rich rendering"""

        with self.log_lock:
            log_lines = list(self.log_messages)

        text = Text()
        for timestamp, message, style in log_lines:
            text.append(f"[{timestamp}] ", style="dim")
            text.append(message + "\n", style=style)

        return text

    def _generate_footer(self):
        """Generate the footer statistics with more detailed info"""

        elapsed = time.time() - self.start_time
        total_messages = sum(self.msg_count.values())
        msg_rate = total_messages / elapsed if elapsed > 0 else 0
        byte_rate = self.total_bytes / elapsed if elapsed > 0 else 0

        # Count active channels (received in last 2 seconds)
        current_time = time.time()
        active_count = sum(1 for msg_type, last_time in self.last_msg_time.items()
                          if current_time - last_time < 2.0)

        # Pagination info
        with self.data_lock, self.page_lock:
            total_msg_types = len(self.all_raw_messages)

        # Message type counts (top 8)
        top_messages = sorted(self.msg_count.items(), key=lambda x: x[1], reverse=True)[:8]
        msg_summary = " | ".join([f"{name}: {count}" for name, count in top_messages])

        footer_text = Text()
        footer_text.append(f"Total Messages: {total_messages:,} ", style="bold green")
        footer_text.append(f"({msg_rate:.1f} Hz) ", style="green")
        footer_text.append(f"| Data: {self.total_bytes/1024:.1f} KB ", style="bold cyan")
        footer_text.append(f"({byte_rate/1024:.1f} KB/s) ", style="cyan")
        footer_text.append(f"| Active: {active_count}/{total_msg_types} types ", style="bold yellow")
        footer_text.append(f"| Uptime: {elapsed:.0f}s\n", style="bold magenta")
        if msg_summary:
            footer_text.append(f"Top: {msg_summary}", style="dim white")

        return Align.center(footer_text)


def key_listener(node, should_exit):
    """Listen for arrow keys and +/- using getch"""
    import sys
    import tty
    import termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)  # Use cbreak instead of raw

        while not should_exit.is_set():
            import select
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)

                if ch == '\x1b':  # Arrow key escape sequence
                    sys.stdin.read(1)  # Skip '['
                    arrow = sys.stdin.read(1)
                    if arrow == 'C':  # Right
                        node.key_queue.put('next')
                    elif arrow == 'D':  # Left
                        node.key_queue.put('prev')
                elif ch in ['+', '=']:
                    node.key_queue.put('more')
                elif ch in ['-', '_']:
                    node.key_queue.put('less')
                elif ch in ['q', 'Q', '\x03']:  # q or Ctrl+C
                    should_exit.set()
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)

    node = MavFirehoseNode()

    console = Console()

    # Start ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # Start key listener in a separate thread
    should_exit = threading.Event()
    key_thread = threading.Thread(target=key_listener, args=(node, should_exit), daemon=True)
    key_thread.start()

    node.dashboard_active = True
    node._add_log('Dashboard started! Keys: ← → (pages)  + - (items)  q (quit)', 'SUCCESS', 'bold green')

    try:
        # Run the live dashboard
        with Live(node.generate_dashboard(), refresh_per_second=10, console=console, screen=True) as live:
            while rclpy.ok() and not should_exit.is_set():
                live.update(node.generate_dashboard())
                time.sleep(0.1)  # 10Hz update rate
    except KeyboardInterrupt:
        pass
    finally:
        should_exit.set()
        node.dashboard_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
