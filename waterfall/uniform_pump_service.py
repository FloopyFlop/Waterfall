#!/usr/bin/env python3
"""
MAV UNIFORM PUMP - Controlled, time-batched MAVLink data packaging for AI/ML
Subscribes to MAV FIRE HOSE data and packages it into uniform time intervals
WITH LIVE TERMINAL DASHBOARD
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
from collections import defaultdict, deque
from datetime import datetime
from queue import Queue
import statistics

# Rich library for beautiful terminal UI
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.console import Console
from rich.text import Text
from rich.align import Align


class UniformPumpNode(Node):
    """
    ROS2 Node that packages MAV FIRE HOSE data into uniform time batches
    Designed for AI/ML consumption with configurable aggregation modes
    WITH LIVE TERMINAL DASHBOARD
    """

    def __init__(self):
        super().__init__('uniform_pump_node')

        # Parameters
        self.declare_parameter('batch_interval', 1.0)  # dT in seconds (default 1 second)
        self.declare_parameter('condensation_mode', 'raw')  # 'raw', 'multi_step', 'single_step'
        self.declare_parameter('multi_step_count', 3)  # Number of steps for multi_step mode
        self.declare_parameter('bleeding_domain_duration', 15.0)  # Bleeding domain in seconds (default 15s)
        self.declare_parameter('missing_data_strategy', 'bleeding_average')  # 'bleeding_average', 'bleeding_latest', 'null', 'zero'

        self.batch_interval = self.get_parameter('batch_interval').value
        self.condensation_mode = self.get_parameter('condensation_mode').value
        self.multi_step_count = self.get_parameter('multi_step_count').value
        self.bleeding_domain_duration = self.get_parameter('bleeding_domain_duration').value
        self.missing_data_strategy = self.get_parameter('missing_data_strategy').value

        # Statistics
        self.total_batches_published = 0
        self.total_messages_received = 0
        self.start_time = time.time()
        self.last_batch_time = time.time()

        # Current batch data collection (stores all messages in current dT)
        self.current_batch_data = defaultdict(list)  # msg_type -> [list of message dicts]
        self.batch_lock = threading.Lock()

        # Bleeding domain data (stores historical data for filling gaps)
        # Structure: msg_type -> deque of (timestamp, message_dict) tuples
        self.bleeding_domain = defaultdict(lambda: deque(maxlen=1000))  # Limit to prevent memory issues
        self.bleeding_lock = threading.Lock()

        # Logs for dashboard - store as (timestamp, message, style) tuples
        self.log_messages = deque(maxlen=30)
        self.log_lock = threading.Lock()

        # Pagination for data display
        self.current_page = 0
        self.items_per_page = 10
        self.page_lock = threading.Lock()
        self.key_queue = Queue()

        # Track active message types and their rates
        self.msg_type_counts = defaultdict(int)
        self.last_batch_msg_types = set()
        self.batch_history = deque(maxlen=100)  # Store recent batch metadata

        # Enhanced statistics for visualization
        self.current_batch_stats = {
            'msg_types_in_batch': 0,
            'total_messages': 0,
            'filled_from_bleeding': 0,
            'missing_types': 0,
        }
        self.last_batch_data_snapshot = {}  # Store last batch for visualization
        self.stats_lock = threading.Lock()

        # Create subscriber to MAV FIRE HOSE all_messages topic
        self.mav_subscriber = self.create_subscription(
            String,
            'mav/all_messages',
            self._mav_message_callback,
            100  # Queue size
        )

        # Create publisher for batched data
        self.batch_pub = self.create_publisher(String, 'mav/uniform_batch', 10)

        # Add log
        self._add_log('Starting MAV UNIFORM PUMP', 'INFO', 'bold green')
        self._add_log(f'Batch interval (dT): {self.batch_interval}s', 'INFO')
        self._add_log(f'Condensation mode: {self.condensation_mode}', 'INFO')
        if self.condensation_mode == 'multi_step':
            self._add_log(f'Multi-step count: {self.multi_step_count}', 'INFO')
        self._add_log(f'Bleeding domain: {self.bleeding_domain_duration}s', 'INFO')
        self._add_log(f'Missing data strategy: {self.missing_data_strategy}', 'INFO')

        # Timer for batch publishing
        self.batch_timer = self.create_timer(self.batch_interval, self._publish_batch)

        # Dashboard will be started by main()
        self.dashboard_active = False

    def process_keys(self):
        """Process any queued key presses"""
        while not self.key_queue.empty():
            key = self.key_queue.get()
            if key == 'next':
                with self.page_lock:
                    if self.batch_history:
                        max_page = max(0, len(self.batch_history) - 1)
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

    def _mav_message_callback(self, msg):
        """Callback for receiving MAVLink messages from firehose"""
        try:
            msg_dict = json.loads(msg.data)
            msg_type = msg_dict.get('_msg_type', 'UNKNOWN')
            timestamp = msg_dict.get('_timestamp', time.time())

            # Store in current batch
            with self.batch_lock:
                self.current_batch_data[msg_type].append(msg_dict)
                self.msg_type_counts[msg_type] += 1
                self.total_messages_received += 1

            # Store in bleeding domain with timestamp
            with self.bleeding_lock:
                self.bleeding_domain[msg_type].append((timestamp, msg_dict))
                # Clean old data from bleeding domain
                self._clean_bleeding_domain(msg_type)

        except Exception as e:
            self._add_log(f'Error processing message: {str(e)}', 'ERROR')

    def _clean_bleeding_domain(self, msg_type):
        """Remove data older than bleeding_domain_duration from bleeding domain"""
        current_time = time.time()
        cutoff_time = current_time - self.bleeding_domain_duration

        # Remove old entries
        while (self.bleeding_domain[msg_type] and
               self.bleeding_domain[msg_type][0][0] < cutoff_time):
            self.bleeding_domain[msg_type].popleft()

    def _publish_batch(self):
        """Publish a batch of data according to condensation mode"""
        try:
            with self.batch_lock:
                # Get snapshot of current batch and reset
                batch_data = dict(self.current_batch_data)
                batch_msg_types = set(batch_data.keys())
                self.current_batch_data = defaultdict(list)

            # Process batch according to condensation mode
            if self.condensation_mode == 'raw':
                packaged_data = self._package_raw(batch_data)
            elif self.condensation_mode == 'multi_step':
                packaged_data = self._package_multi_step(batch_data)
            elif self.condensation_mode == 'single_step':
                packaged_data = self._package_single_step(batch_data)
            else:
                self._add_log(f'Unknown condensation mode: {self.condensation_mode}', 'ERROR')
                return

            # Add metadata
            packaged_data['_batch_metadata'] = {
                'batch_number': self.total_batches_published,
                'batch_start_time': self.last_batch_time,
                'batch_end_time': time.time(),
                'batch_interval': self.batch_interval,
                'condensation_mode': self.condensation_mode,
                'message_types_count': len(batch_msg_types),
                'total_messages_in_batch': sum(len(msgs) for msgs in batch_data.values()),
            }

            # Publish the batch
            batch_msg = String()
            batch_msg.data = json.dumps(packaged_data)
            self.batch_pub.publish(batch_msg)

            # Update statistics
            self.total_batches_published += 1
            self.last_batch_time = time.time()
            self.last_batch_msg_types = batch_msg_types

            # Calculate enhanced statistics
            with self.bleeding_lock:
                all_bleeding_types = set(self.bleeding_domain.keys())
            filled_types = all_bleeding_types - batch_msg_types

            # Store batch metadata for dashboard
            batch_metadata = {
                'batch_number': self.total_batches_published - 1,
                'timestamp': time.time(),
                'msg_types': len(batch_msg_types),
                'total_messages': packaged_data['_batch_metadata']['total_messages_in_batch'],
                'mode': self.condensation_mode,
                'filled_from_bleeding': len(filled_types),
            }
            self.batch_history.append(batch_metadata)

            # Update current batch stats for visualization
            with self.stats_lock:
                self.current_batch_stats = {
                    'msg_types_in_batch': len(batch_msg_types),
                    'total_messages': packaged_data['_batch_metadata']['total_messages_in_batch'],
                    'filled_from_bleeding': len(filled_types),
                    'missing_types': 0,  # Could be expanded to track truly missing types
                }
                # Store a sample of the last batch for visualization
                self.last_batch_data_snapshot = {
                    msg_type: len(msgs) for msg_type, msgs in list(batch_data.items())[:10]
                }

            # Log success
            if self.total_batches_published % 10 == 0:
                self._add_log(f'Published batch #{self.total_batches_published - 1} ({len(batch_msg_types)} msg types, {len(filled_types)} filled)',
                             'SUCCESS', 'green')

        except Exception as e:
            self._add_log(f'Error publishing batch: {str(e)}', 'ERROR')

    def _package_raw(self, batch_data):
        """Package data in raw mode - all messages with timestamps"""
        packaged = {'data': {}}

        for msg_type, messages in batch_data.items():
            packaged['data'][msg_type] = messages

        # Fill missing message types from bleeding domain
        packaged['data'] = self._fill_missing_data(packaged['data'], batch_data.keys())

        return packaged

    def _package_multi_step(self, batch_data):
        """Package data in multi-step mode - divide batch into N discrete steps"""
        packaged = {'data': {}}

        for msg_type, messages in batch_data.items():
            if not messages:
                continue

            # Divide messages into N steps based on timestamp
            steps = [[] for _ in range(self.multi_step_count)]

            if len(messages) > 0:
                # Get time range
                timestamps = [msg.get('_timestamp', 0) for msg in messages]
                min_time = min(timestamps)
                max_time = max(timestamps)
                time_range = max_time - min_time if max_time > min_time else 1.0
                step_duration = time_range / self.multi_step_count

                # Assign messages to steps
                for msg in messages:
                    msg_time = msg.get('_timestamp', min_time)
                    step_index = min(int((msg_time - min_time) / step_duration), self.multi_step_count - 1)
                    steps[step_index].append(msg)

            # Average each step
            averaged_steps = []
            for step_messages in steps:
                if step_messages:
                    averaged_steps.append(self._average_messages(step_messages))
                else:
                    averaged_steps.append(None)  # Will be filled by bleeding domain

            packaged['data'][msg_type] = averaged_steps

        # Fill missing message types from bleeding domain
        packaged['data'] = self._fill_missing_data(packaged['data'], batch_data.keys())

        return packaged

    def _package_single_step(self, batch_data):
        """Package data in single-step mode - average all data in batch to single values"""
        packaged = {'data': {}}

        for msg_type, messages in batch_data.items():
            if messages:
                packaged['data'][msg_type] = self._average_messages(messages)

        # Fill missing message types from bleeding domain
        packaged['data'] = self._fill_missing_data(packaged['data'], batch_data.keys())

        return packaged

    def _average_messages(self, messages):
        """Average a list of messages, handling numeric and non-numeric fields"""
        if not messages:
            return None

        # Collect all field values
        field_values = defaultdict(list)
        for msg in messages:
            for field, value in msg.items():
                if not field.startswith('_'):  # Skip metadata fields
                    field_values[field].append(value)

        # Average numeric fields, take most common for non-numeric
        averaged = {}
        for field, values in field_values.items():
            if self._is_numeric_list(values):
                # Average numeric values
                averaged[field] = statistics.mean(values)
            else:
                # For non-numeric, take the most recent value
                averaged[field] = values[-1]

        return averaged

    def _is_numeric_list(self, values):
        """Check if a list contains only numeric values"""
        try:
            for v in values:
                if v is None:
                    return False
                if isinstance(v, (list, dict)):
                    return False
                float(v)  # Try to convert to float
            return True
        except (ValueError, TypeError):
            return False

    def _fill_missing_data(self, packaged_data, present_msg_types):
        """Fill missing message types using bleeding domain data"""
        with self.bleeding_lock:
            # Get all message types from bleeding domain
            all_msg_types = set(self.bleeding_domain.keys())
            missing_msg_types = all_msg_types - set(present_msg_types)

            for msg_type in missing_msg_types:
                bleeding_data = list(self.bleeding_domain[msg_type])

                if not bleeding_data:
                    continue

                # Apply missing data strategy
                if self.missing_data_strategy == 'bleeding_average':
                    # Average all bleeding domain data for this type
                    messages = [msg_dict for _, msg_dict in bleeding_data]
                    packaged_data[msg_type] = self._average_messages(messages)

                elif self.missing_data_strategy == 'bleeding_latest':
                    # Use most recent bleeding domain data
                    _, latest_msg = bleeding_data[-1]
                    packaged_data[msg_type] = latest_msg

                elif self.missing_data_strategy == 'null':
                    # Mark as null
                    packaged_data[msg_type] = None

                elif self.missing_data_strategy == 'zero':
                    # Create zero-filled message
                    if bleeding_data:
                        _, sample_msg = bleeding_data[-1]
                        packaged_data[msg_type] = self._zero_fill_message(sample_msg)

        return packaged_data

    def _zero_fill_message(self, sample_msg):
        """Create a zero-filled version of a message"""
        zeroed = {}
        for field, value in sample_msg.items():
            if field.startswith('_'):
                continue
            if isinstance(value, (int, float)):
                zeroed[field] = 0
            elif isinstance(value, list):
                zeroed[field] = [0] * len(value)
            else:
                zeroed[field] = value  # Keep non-numeric as-is
        return zeroed

    def generate_dashboard(self):
        """Generate the live dashboard layout with enhanced visualizations"""

        # Process any key presses
        self.process_keys()

        layout = Layout()
        layout.split_column(
            Layout(name="header", size=5),
            Layout(name="main"),
            Layout(name="footer", size=6),
        )

        # Split main into three rows
        layout["main"].split_column(
            Layout(name="top_row", size=12),
            Layout(name="middle_row"),
            Layout(name="bottom_row", size=10),
        )

        # Top row: config | condensation viz | bleeding domain
        layout["top_row"].split_row(
            Layout(name="config", ratio=1),
            Layout(name="condensation_viz", ratio=2),
            Layout(name="bleeding_viz", ratio=1),
        )

        # Middle row: batch construction | batch history
        layout["middle_row"].split_row(
            Layout(name="construction", ratio=1),
            Layout(name="batches", ratio=1),
        )

        # Bottom row: logs (full width)

        # Header with mode visualization
        header_content = self._generate_header()
        layout["header"].update(Panel(header_content, style="bold white"))

        # Config panel
        config_panel = self._generate_config_panel()
        layout["config"].update(Panel(config_panel, title="[bold green]⚙ Configuration", border_style="green"))

        # Condensation mode visualization
        condensation_viz = self._generate_condensation_visualization()
        layout["condensation_viz"].update(Panel(condensation_viz, title=f"[bold cyan]🔄 {self.condensation_mode.upper()} Mode Pipeline", border_style="cyan"))

        # Bleeding domain visualization
        bleeding_viz = self._generate_bleeding_domain_viz()
        layout["bleeding_viz"].update(Panel(bleeding_viz, title="[bold yellow]📦 Bleeding Domain", border_style="yellow"))

        # Batch construction visualization
        construction_viz = self._generate_batch_construction_viz()
        layout["construction"].update(Panel(construction_viz, title="[bold magenta]🔨 Current Batch Construction", border_style="magenta"))

        # Batch history panel
        batch_table = self._generate_batch_history()
        layout["batches"].update(Panel(batch_table, title="[bold blue]📊 Batch History", border_style="blue"))

        # Logs panel
        log_text = self._generate_logs()
        layout["bottom_row"].update(Panel(log_text, title="[bold white]📝 System Logs", border_style="white"))

        # Footer - Statistics
        footer_content = self._generate_footer()
        layout["footer"].update(Panel(footer_content, style="bold green"))

        return layout

    def _generate_header(self):
        """Generate enhanced header with status indicators"""
        text = Text()

        # Title
        text.append("═" * 80 + "\n", style="bold magenta")
        text.append("  MAV UNIFORM PUMP - AI/ML Time-Batched Data Pipeline\n", style="bold white on magenta")
        text.append("═" * 80 + "\n", style="bold magenta")

        # Status line
        elapsed = time.time() - self.start_time
        status_line = f"  Status: "
        text.append(status_line, style="white")
        text.append("● ACTIVE ", style="bold green")
        text.append(f"| Uptime: {elapsed:.0f}s | ", style="dim white")
        text.append(f"Mode: {self.condensation_mode.upper()} ", style="bold cyan")
        text.append(f"| dT: {self.batch_interval}s ", style="bold yellow")
        if self.condensation_mode == 'multi_step':
            text.append(f"| Steps: {self.multi_step_count}", style="bold yellow")

        return Align.center(text)

    def _generate_config_panel(self):
        """Generate configuration panel showing current settings"""
        text = Text()

        # Mode
        text.append("MODE\n", style="bold green")
        mode_display = {
            'raw': '📋 Raw (All msgs)',
            'multi_step': f'📊 Multi-Step ({self.multi_step_count})',
            'single_step': '📍 Single-Step (Avg)'
        }
        text.append(f"{mode_display.get(self.condensation_mode, self.condensation_mode)}\n\n", style="cyan")

        # Timing
        text.append("TIMING\n", style="bold green")
        text.append(f"dT: {self.batch_interval}s\n", style="cyan")
        elapsed = time.time() - self.last_batch_time
        progress = min(elapsed / self.batch_interval, 1.0)
        bar_length = 15
        filled = int(progress * bar_length)
        bar = "█" * filled + "░" * (bar_length - filled)
        text.append(f"{bar} {progress*100:.0f}%\n\n", style="yellow")

        # Bleeding domain
        text.append("BLEEDING\n", style="bold green")
        text.append(f"{self.bleeding_domain_duration}s window\n", style="cyan")
        with self.bleeding_lock:
            bleeding_types = len(self.bleeding_domain)
            bleeding_msgs = sum(len(msgs) for msgs in self.bleeding_domain.values())
        text.append(f"{bleeding_types} types\n", style="yellow")
        text.append(f"{bleeding_msgs} msgs\n\n", style="yellow")

        # Strategy
        text.append("STRATEGY\n", style="bold green")
        strategy_display = {
            'bleeding_average': '📊 Avg',
            'bleeding_latest': '🔄 Latest',
            'null': '∅ Null',
            'zero': '0 Zero'
        }
        text.append(f"{strategy_display.get(self.missing_data_strategy, self.missing_data_strategy)}\n", style="cyan")

        return text

    def _generate_condensation_visualization(self):
        """Generate visual representation of the condensation pipeline"""
        text = Text()

        if self.condensation_mode == 'raw':
            # Raw mode visualization
            text.append("┌─────────────────────────────────────────┐\n", style="cyan")
            text.append("│  FIREHOSE  →  [Collect All]  →  BATCH   │\n", style="bold cyan")
            text.append("└─────────────────────────────────────────┘\n", style="cyan")
            text.append("\nAll messages preserved with timestamps\n", style="dim white")
            text.append("No aggregation or averaging\n\n", style="dim white")

            with self.stats_lock:
                total_msgs = self.current_batch_stats.get('total_messages', 0)
            text.append(f"Last Batch: {total_msgs} messages\n", style="green")

        elif self.condensation_mode == 'multi_step':
            # Multi-step mode visualization
            text.append("┌──────────────────────────────────────────────┐\n", style="cyan")
            text.append("│  FIREHOSE  →  [Divide into N Steps] →        │\n", style="bold cyan")
            text.append("│               [Average Each Step]   →  BATCH |\n", style="bold cyan")
            text.append("└──────────────────────────────────────────────┘\n", style="cyan")

            # Show step divisions
            step_duration = self.batch_interval / self.multi_step_count
            text.append(f"\nStep Duration: {step_duration:.3f}s\n", style="dim white")
            for i in range(self.multi_step_count):
                start = i * step_duration
                end = (i + 1) * step_duration
                bar = "█" * 10
                text.append(f"  Step {i}: [{start:.2f}s - {end:.2f}s] ", style="yellow")
                text.append(f"{bar}\n", style="green")

        elif self.condensation_mode == 'single_step':
            # Single-step mode visualization
            text.append("┌─────────────────────────────────────────┐\n", style="cyan")
            text.append("│  FIREHOSE  →  [Average All]  →  BATCH   │\n", style="bold cyan")
            text.append("└─────────────────────────────────────────┘\n", style="cyan")
            text.append("\nAll messages averaged into single value\n", style="dim white")
            text.append("Maximum data condensation\n\n", style="dim white")

            with self.stats_lock:
                total_msgs = self.current_batch_stats.get('total_messages', 0)
                msg_types = self.current_batch_stats.get('msg_types_in_batch', 0)
            text.append(f"Last: {total_msgs} msgs → {msg_types} types\n", style="green")

        return text

    def _generate_bleeding_domain_viz(self):
        """Generate visualization of bleeding domain status"""
        text = Text()

        with self.bleeding_lock:
            bleeding_types = len(self.bleeding_domain)
            bleeding_total = sum(len(msgs) for msgs in self.bleeding_domain.values())

            # Get top message types by count
            type_counts = [(msg_type, len(msgs)) for msg_type, msgs in self.bleeding_domain.items()]
            type_counts.sort(key=lambda x: x[1], reverse=True)
            top_types = type_counts[:8]

        # Header
        text.append(f"Types: {bleeding_types}\n", style="bold yellow")
        text.append(f"Messages: {bleeding_total}\n\n", style="bold yellow")

        # Show capacity
        capacity_pct = (bleeding_total / (bleeding_types * 1000 * 100)) if bleeding_types > 0 else 0
        text.append("Capacity:\n", style="bold green")
        bar_length = 20
        filled = int(capacity_pct * bar_length)
        bar = "█" * filled + "░" * (bar_length - filled)
        text.append(f"{bar}\n", style="cyan")
        text.append(f"{capacity_pct*100:.1f}%\n\n", style="dim white")

        # Top message types
        text.append("Top Types:\n", style="bold green")
        for msg_type, count in top_types[:5]:
            # Truncate long names
            display_name = msg_type[:12] + "..." if len(msg_type) > 15 else msg_type
            text.append(f"{display_name:15s} ", style="white")
            # Mini bar
            max_count = top_types[0][1] if top_types else 1
            bar_len = int((count / max_count) * 10)
            bar = "▰" * bar_len + "▱" * (10 - bar_len)
            text.append(f"{bar} {count}\n", style="cyan")

        return text

    def _generate_batch_construction_viz(self):
        """Generate visualization of current batch being constructed"""
        text = Text()

        with self.batch_lock:
            current_types = len(self.current_batch_data)
            current_total = sum(len(msgs) for msgs in self.current_batch_data.values())

            # Get snapshot of current data
            type_counts = [(msg_type, len(msgs)) for msg_type, msgs in self.current_batch_data.items()]
            type_counts.sort(key=lambda x: x[1], reverse=True)

        # Time to next batch
        elapsed = time.time() - self.last_batch_time
        remaining = max(0, self.batch_interval - elapsed)

        # Header
        text.append("Next Batch In:\n", style="bold magenta")
        text.append(f"{remaining:.2f}s\n\n", style="bold yellow")

        # Progress bar
        progress = min(elapsed / self.batch_interval, 1.0)
        bar_length = 30
        filled = int(progress * bar_length)
        bar = "█" * filled + "░" * (bar_length - filled)
        text.append(f"{bar}\n\n", style="green")

        # Current stats
        text.append("Collecting:\n", style="bold magenta")
        text.append(f"{current_types} msg types\n", style="cyan")
        text.append(f"{current_total} total msgs\n\n", style="cyan")

        # Show top message types being collected
        text.append("Active Streams:\n", style="bold green")
        for msg_type, count in type_counts[:8]:
            display_name = msg_type[:12] + "..." if len(msg_type) > 15 else msg_type
            # Rate indicator
            rate_indicator = "●" if count > 5 else "○"
            text.append(f"{rate_indicator} ", style="green" if count > 5 else "yellow")
            text.append(f"{display_name:15s} {count:3d}\n", style="white")

        # Stats from last batch
        with self.stats_lock:
            filled = self.current_batch_stats.get('filled_from_bleeding', 0)
        if filled > 0:
            text.append(f"\n💾 {filled} types filled from bleeding domain\n", style="dim yellow")

        return text

    def _generate_batch_history(self):
        """Generate table showing recent batch history with enhanced info"""
        table = Table(show_header=True, box=None, padding=(0, 1))
        table.add_column("#", style="bold cyan", width=6)
        table.add_column("Time", style="white", width=10)
        table.add_column("Types", style="yellow", width=7)
        table.add_column("Msgs", style="green", width=6)
        table.add_column("Filled", style="magenta", width=7)
        table.add_column("Mode", style="dim white", width=10)

        # Show recent batches (most recent first)
        recent_batches = list(self.batch_history)[-self.items_per_page:]
        recent_batches.reverse()

        for batch in recent_batches:
            timestamp = datetime.fromtimestamp(batch['timestamp']).strftime('%H:%M:%S')
            filled = batch.get('filled_from_bleeding', 0)
            filled_str = f"+{filled}" if filled > 0 else "-"

            # Visual indicator for activity level
            msg_count = batch['total_messages']
            if msg_count > 100:
                indicator = "●"
                style = "bold green"
            elif msg_count > 50:
                indicator = "◐"
                style = "green"
            else:
                indicator = "○"
                style = "dim green"

            table.add_row(
                f"{indicator} {batch['batch_number']}",
                timestamp,
                str(batch['msg_types']),
                str(msg_count),
                filled_str,
                batch['mode'][:8]
            )

        if not recent_batches:
            table.add_row("No batches yet", "", "", "", "", "")

        return table

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
        """Generate enhanced footer with comprehensive statistics and graphs"""
        elapsed = time.time() - self.start_time
        batch_rate = self.total_batches_published / elapsed if elapsed > 0 else 0
        msg_rate = self.total_messages_received / elapsed if elapsed > 0 else 0

        # Count bleeding domain size
        with self.bleeding_lock:
            bleeding_types = len(self.bleeding_domain)
            bleeding_total_msgs = sum(len(msgs) for msgs in self.bleeding_domain.values())

        # Current batch progress
        with self.batch_lock:
            current_collecting = sum(len(msgs) for msgs in self.current_batch_data.values())

        # Get stats
        with self.stats_lock:
            last_batch_types = self.current_batch_stats.get('msg_types_in_batch', 0)
            last_batch_total = self.current_batch_stats.get('total_messages', 0)
            filled_count = self.current_batch_stats.get('filled_from_bleeding', 0)

        # Create comprehensive footer
        table = Table(show_header=False, box=None, padding=(0, 2), expand=True)
        table.add_column("Metric", style="bold white", width=25)
        table.add_column("Value", style="cyan")
        table.add_column("Metric", style="bold white", width=25)
        table.add_column("Value", style="cyan")

        # Row 1
        table.add_row(
            "📦 Total Batches",
            f"[bold green]{self.total_batches_published}[/bold green] ({batch_rate:.2f} Hz)",
            "📨 Messages Received",
            f"[bold cyan]{self.total_messages_received:,}[/bold cyan] ({msg_rate:.1f} Hz)"
        )

        # Row 2
        table.add_row(
            "⏱  Uptime",
            f"[bold magenta]{elapsed:.0f}s[/bold magenta]",
            "🔄 Current Collecting",
            f"[bold yellow]{current_collecting}[/bold yellow] msgs"
        )

        # Row 3
        bleeding_pct = (bleeding_total_msgs / (bleeding_types * 1000)) * 100 if bleeding_types > 0 else 0
        table.add_row(
            "💾 Bleeding Domain",
            f"[bold yellow]{bleeding_types}[/bold yellow] types, [yellow]{bleeding_total_msgs}[/yellow] msgs ({bleeding_pct:.1f}%)",
            "📊 Last Batch",
            f"[bold green]{last_batch_types}[/bold green] types, [green]{last_batch_total}[/green] msgs" +
            (f" [dim](+{filled_count} filled)[/dim]" if filled_count > 0 else "")
        )

        # Add mini sparkline for recent batches
        if len(self.batch_history) > 5:
            recent = list(self.batch_history)[-20:]
            msg_counts = [b['total_messages'] for b in recent]
            max_msgs = max(msg_counts) if msg_counts else 1
            sparkline = ""
            spark_chars = ["▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"]
            for count in msg_counts:
                idx = min(int((count / max_msgs) * len(spark_chars)), len(spark_chars) - 1)
                sparkline += spark_chars[idx]

            table.add_row(
                "📈 Batch Trend (last 20)",
                f"[green]{sparkline}[/green]",
                "🎯 Mode",
                f"[bold cyan]{self.condensation_mode.upper()}[/bold cyan]"
            )

        return table


def key_listener(node, should_exit):
    """Listen for arrow keys and +/- using getch"""
    import sys
    import tty
    import termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

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
    parser = argparse.ArgumentParser(add_help=False, description="Uniform pump helpers")
    parser.add_argument('--headless', action='store_true', help='Disable the dashboard UI (ROS node only).')
    cli_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)

    node = UniformPumpNode()

    try:
        headless = bool(getattr(cli_args, 'headless', False))
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            headless = True

        if headless:
            rclpy.spin(node)
            return

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

        # Run the live dashboard
        with Live(node.generate_dashboard(), refresh_per_second=10, console=console, screen=True) as live:
            while rclpy.ok() and not should_exit.is_set():
                live.update(node.generate_dashboard())
                time.sleep(0.1)  # 10Hz update rate
    except KeyboardInterrupt:
        pass
    finally:
        if 'should_exit' in locals():
            should_exit.set()
        node.dashboard_active = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
