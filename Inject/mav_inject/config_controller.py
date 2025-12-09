#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import termios
import tty
import time
import select
import threading
from threading import Lock, Event
from collections import deque

# Rich library for beautiful terminal UI
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.console import Console
from rich.text import Text
from rich.align import Align


class ConfigController(Node):
    def __init__(self):
        super().__init__('config_controller')

        self.log_lock = Lock()
        self.status_lock = Lock()

        # ROS2 publisher for sending commands to injector
        self.command_pub = self.create_publisher(String, 'px4_injector/command', 10)

        # ROS2 subscriber for status updates
        self.status_sub = self.create_subscription(
            String,
            'px4_injector/status',
            self.status_callback,
            10
        )

        # State tracking
        self.connection_alive = False
        self.last_status_time = None
        self.injector_status = {}
        self.selected_index = 0
        self.running = True
        self.start_time = time.time()
        self.commands_sent = 0
        self.last_command_time = None

        # Logs (keep last 15)
        self.log_messages = deque(maxlen=15)

        # Define available action categories
        # Using REAL PX4 parameters that actually exist
        self.action_categories = {
            'Flight Tuning': [
                ('PID: Increase Roll Gains', 'More aggressive roll response',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_ROLL_P': '8.0', 'MC_ROLLRATE_P': '0.2', 'MC_ROLLRATE_I': '0.3'}}),
                ('PID: Decrease Roll Gains', 'Smoother roll response',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_ROLL_P': '6.0', 'MC_ROLLRATE_P': '0.15', 'MC_ROLLRATE_I': '0.2'}}),
                ('PID: Increase Pitch Gains', 'More aggressive pitch response',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_PITCH_P': '8.0', 'MC_PITCHRATE_P': '0.2', 'MC_PITCHRATE_I': '0.3'}}),
                ('PID: Decrease Pitch Gains', 'Smoother pitch response',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_PITCH_P': '6.0', 'MC_PITCHRATE_P': '0.15', 'MC_PITCHRATE_I': '0.2'}}),
                ('Increase Yaw Gains', 'Faster yaw response',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_YAW_P': '3.0', 'MC_YAWRATE_P': '0.25'}}),
                ('Decrease Yaw Gains', 'Gentler yaw response',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_YAW_P': '2.0', 'MC_YAWRATE_P': '0.15'}}),
            ],
            'Position Control': [
                ('Aggressive Position Hold', 'Tighter position control',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MPC_XY_P': '1.5', 'MPC_Z_P': '1.5', 'MPC_XY_VEL_P_ACC': '3.0'}}),
                ('Smooth Position Hold', 'Gentler position control',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MPC_XY_P': '0.95', 'MPC_Z_P': '1.0', 'MPC_XY_VEL_P_ACC': '1.8'}}),
                ('Increase Max Velocity', 'Allow faster flight',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MPC_XY_VEL_MAX': '15.0', 'MPC_Z_VEL_MAX_UP': '5.0'}}),
                ('Decrease Max Velocity', 'Slower, safer flight',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MPC_XY_VEL_MAX': '8.0', 'MPC_Z_VEL_MAX_UP': '3.0'}}),
            ],
            'Test Degradation': [
                ('Extreme Oscillation', 'Very high P gain, causes visible wobble',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_ROLL_P': '15.0', 'MC_PITCH_P': '15.0', 'MC_ROLLRATE_I': '0.05', 'MC_PITCHRATE_I': '0.05'}}),
                ('Sluggish Response', 'Very low gains, drone will drift badly',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_ROLLRATE_P': '0.05', 'MC_PITCHRATE_P': '0.05', 'MC_ROLLRATE_I': '0.01', 'MC_PITCHRATE_I': '0.01'}}),
                ('Asymmetric Roll/Pitch', 'Causes constant drift in one direction',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_ROLL_P': '10.0', 'MC_PITCH_P': '4.0', 'MC_ROLLRATE_P': '0.25', 'MC_PITCHRATE_P': '0.1'}}),
                ('Weak Yaw Control', 'Drone will spin slowly, hard to control heading',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_YAW_P': '0.5', 'MC_YAWRATE_P': '0.05', 'MC_YAWRATE_I': '0.01'}}),
                ('Position Hold Chaos', 'Terrible position control, constantly moving',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MPC_XY_P': '0.3', 'MPC_Z_P': '0.4', 'MPC_XY_VEL_P_ACC': '0.8'}}),
                ('High D Gain Jitter', 'Derivative too high, causes high frequency jitter',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'MC_ROLLRATE_D': '0.015', 'MC_PITCHRATE_D': '0.015', 'MC_ROLLRATE_P': '0.18', 'MC_PITCHRATE_P': '0.18'}}),
            ],
            'System': [
                ('Emergency: Kill Motors', 'EMERGENCY: Disable all motors',
                 {'action': 'edit', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x',
                  'modifications': {'PWM_MAIN_DIS': '900'}}),
                ('Reset All Tuning', 'Restore default parameters',
                 {'action': 'restore', 'file': 'ROMFS/px4fmu_common/init.d/airframes/4001_quad_x'}),
                ('Backup Configs', 'Create backup of all config files',
                 {'action': 'backup'}),
                ('Restore Backup', 'Restore from last backup',
                 {'action': 'restore'}),
            ]
        }

        # Flatten actions for selection
        self.actions = []
        for category, actions in self.action_categories.items():
            for name, desc, cmd in actions:
                self.actions.append({
                    'category': category,
                    'name': name,
                    'description': desc,
                    'command': cmd
                })

        self.add_log('System started', 'green')
        self.get_logger().info('Config Controller initialized')

    def add_log(self, message, style="white"):
        """Add a log message with timestamp"""
        with self.log_lock:
            timestamp = time.strftime("%H:%M:%S")
            self.log_messages.append((timestamp, message, style))

    def status_callback(self, msg):
        """Handle status updates from the injector node"""
        with self.status_lock:
            self.connection_alive = True
            self.last_status_time = time.time()

            try:
                status_data = json.loads(msg.data)
                self.injector_status = status_data
                status_text = status_data.get('status', 'unknown')

                if status_text in ['config_edited', 'backup_created', 'config_restored']:
                    self.add_log(f"Status: {status_text}", "green")

            except json.JSONDecodeError:
                self.injector_status = {'raw': msg.data[:100]}

    def send_command(self, command):
        """Send command to the injector node"""
        msg = String()
        msg.data = json.dumps(command)
        self.command_pub.publish(msg)

        self.commands_sent += 1
        self.last_command_time = time.time()

        action_name = command.get('action', 'unknown')
        self.add_log(f"Sent: {action_name}", "cyan")
        self.get_logger().info(f'Sent command: {action_name}')

    def generate_dashboard(self):
        """Generate the live dashboard layout"""
        layout = Layout()

        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main"),
            Layout(name="footer", size=4),
        )

        layout["main"].split_row(
            Layout(name="status", ratio=1),
            Layout(name="actions", ratio=2),
            Layout(name="logs", ratio=1),
        )

        # Header
        uptime = time.time() - self.start_time
        header_text = Text(f"MAV_INJECT Config Controller | Uptime: {uptime:.0f}s | Commands: {self.commands_sent}",
                          style="bold magenta", justify="center")
        layout["header"].update(Panel(header_text, style="bold white"))

        # Status panel
        status_content = self._generate_status_panel()
        layout["status"].update(Panel(status_content, title="[bold green]Connection Status", border_style="green"))

        # Actions panel
        actions_content = self._generate_actions_panel()
        layout["actions"].update(Panel(actions_content, title="[bold cyan]Available Actions", border_style="cyan"))

        # Logs panel
        logs_content = self._generate_logs()
        layout["logs"].update(Panel(logs_content, title="[bold yellow]Activity Log", border_style="yellow"))

        # Footer
        footer_content = self._generate_footer()
        layout["footer"].update(Panel(footer_content, style="bold green"))

        return layout

    def _generate_status_panel(self):
        """Generate connection status content"""
        text = Text()

        with self.status_lock:
            # Connection indicator
            if self.connection_alive and self.last_status_time:
                time_since = time.time() - self.last_status_time
                if time_since < 3.0:
                    text.append("● CONNECTED\n\n", style="bold green")
                else:
                    text.append("○ STALE\n\n", style="bold yellow")
            else:
                text.append("○ WAITING...\n\n", style="bold yellow")

            # Injector info
            if self.injector_status:
                for key, value in self.injector_status.items():
                    if key == 'status':
                        text.append(f"State: {value}\n", style="green")
                    elif key == 'px4_path':
                        short_path = value.split('/')[-1] if '/' in str(value) else str(value)
                        text.append(f"PX4: {short_path}\n", style="white")
                    elif key != 'timestamp':
                        text.append(f"{key}: {value}\n", style="dim")

            # Last command
            if self.last_command_time:
                elapsed = time.time() - self.last_command_time
                text.append(f"\nLast cmd: {elapsed:.1f}s ago", style="dim")

        return text

    def _generate_actions_panel(self):
        """Generate actions list content"""
        table = Table(show_header=False, box=None)
        table.add_column("", style="white")
        table.add_column("Action", style="white")
        table.add_column("Description", style="dim")

        current_category = None
        for idx, action in enumerate(self.actions):
            # Category header
            if action['category'] != current_category:
                current_category = action['category']
                if idx > 0:
                    table.add_row("", "", "")
                table.add_row("", f"[bold yellow]{current_category}[/bold yellow]", "")

            # Action row
            marker = "►" if idx == self.selected_index else " "
            name = f"[bold cyan]{action['name']}[/bold cyan]" if idx == self.selected_index else action['name']
            table.add_row(marker, name, action['description'])

        return table

    def _generate_logs(self):
        """Generate logs content"""
        with self.log_lock:
            log_lines = list(self.log_messages)

        text = Text()
        for timestamp, message, style in log_lines:
            text.append(f"[{timestamp}] ", style="dim")
            text.append(message + "\n", style=style)

        if not log_lines:
            text.append("No activity yet", style="dim")

        return text

    def _generate_footer(self):
        """Generate footer content"""
        elapsed = time.time() - self.start_time

        footer_text = Text()
        footer_text.append(f"Commands: {self.commands_sent} ", style="bold green")

        with self.status_lock:
            if self.last_status_time:
                time_since = time.time() - self.last_status_time
                footer_text.append(f"| Last Status: {time_since:.1f}s ago ", style="bold cyan")

        footer_text.append(f"| Uptime: {elapsed:.0f}s\n", style="bold magenta")
        footer_text.append("Controls: ", style="bold white")
        footer_text.append("↑/↓ Navigate  ", style="green")
        footer_text.append("Enter Execute  ", style="cyan")
        footer_text.append("Q Quit", style="red")

        return Align.center(footer_text)


def key_listener(node, should_exit):
    """Listen for keyboard input"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        while not should_exit.is_set():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)

                if ch == '\x1b':  # Escape sequence
                    sys.stdin.read(1)  # Skip '['
                    arrow = sys.stdin.read(1)
                    if arrow == 'A':  # Up
                        node.selected_index = (node.selected_index - 1) % len(node.actions)
                    elif arrow == 'B':  # Down
                        node.selected_index = (node.selected_index + 1) % len(node.actions)
                elif ch in ['\r', '\n']:  # Enter
                    selected_action = node.actions[node.selected_index]
                    node.send_command(selected_action['command'])
                elif ch in ['q', 'Q', '\x03']:  # q or Ctrl+C
                    should_exit.set()
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)

    node = ConfigController()
    console = Console()

    # Start ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # Start key listener in a separate thread
    should_exit = Event()
    key_thread = threading.Thread(target=key_listener, args=(node, should_exit), daemon=True)
    key_thread.start()

    node.add_log('Dashboard started! Keys: ↑/↓ navigate, Enter execute, Q quit', 'green')

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
