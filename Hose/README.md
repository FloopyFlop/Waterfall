# MAV DATA HOSE

**High-frequency raw MAVLink data streaming and controlled batching for AI/ML consumption**

This ROS2 package provides two complementary data streams:
1. **MAV FIRE HOSE**: Maximum frequency raw MAVLink data streaming (100+ Hz)
2. **MAV UNIFORM PUMP**: Controlled time-batched data packaging for AI classification

Perfect for feeding raw sensor streams to AI models with flexible data aggregation strategies.

## Fast Setup
Run:  ```colcon build --packages-select mav_data_hose && source install/setup.bash && ros2 run mav_data_hose firehose_node```

Run, in a separate terminal instance:  ```colcon build --packages-select mav_data_hose && source install/setup.bash && ros2 run mav_data_hose uniform_pump_node```

## Features

### MAV FIRE HOSE
- **Maximum throughput**: Requests all data streams at 100Hz+ with minimal processing overhead
- **Raw data publishing**: Publishes both raw bytes and JSON for flexible consumption
- **Comprehensive coverage**: Streams ALL available MAVLink messages
- **Structured topics**: Also publishes to standard ROS2 sensor topics (IMU, GPS, etc.)
- **Live statistics**: Real-time throughput monitoring
- **Thread-optimized**: Separate thread for MAVLink reception to maximize data rate

### MAV UNIFORM PUMP
- **Time-batched aggregation**: Packages data into uniform time intervals (configurable dT)
- **Multiple condensation modes**: Raw, multi-step, or single-step averaging
- **Bleeding domain**: Maintains historical data to fill missing fields
- **Configurable gap-filling**: Multiple strategies for handling missing data
- **AI/ML optimized**: Designed for feeding classification models at regular intervals
- **Live dashboard**: Real-time batch monitoring with Rich UI

## Published Topics

### MAV FIRE HOSE Topics

#### Raw Data (for AI models)
- `/mav/raw/bytes` (UInt8MultiArray) - Pure raw MAVLink bytes
- `/mav/raw/json` (String) - JSON-serialized MAVLink messages
- `/mav/all_messages` (String) - All messages with timestamps

#### Sensor Data
- `/mav/imu` (Imu) - IMU data (accel, gyro)
- `/mav/magnetometer` (MagneticField) - Magnetometer
- `/mav/pressure` (FluidPressure) - Barometric pressure
- `/mav/battery` (BatteryState) - Battery status

#### Position & Navigation
- `/mav/global_position` (NavSatFix) - GPS position
- `/mav/local_position` (PoseStamped) - Local position (NED)
- `/mav/odometry` (Odometry) - Combined position/velocity
- `/mav/velocity` (TwistStamped) - Velocity

#### Attitude & Orientation
- `/mav/attitude` (Vector3Stamped) - Roll/pitch/yaw
- `/mav/attitude_quaternion` (PoseStamped) - Quaternion orientation

#### Control & Actuators
- `/mav/rc_channels` (Float64MultiArray) - RC input channels
- `/mav/servo_output` (Float64MultiArray) - Servo/motor outputs

#### System Status
- `/mav/heartbeat` (String) - System heartbeat
- `/mav/sys_status` (String) - System status
- `/mav/statistics` (String) - Throughput statistics

### MAV UNIFORM PUMP Topics

- `/mav/uniform_batch` (String) - Time-batched data packages with metadata

## Setup Instructions (Run on your VM)

### 1. Prerequisites
```bash
# Make sure you have ROS2 installed (Humble, Iron, or Jazzy)
```

### 2. Create ROS2 Workspace and Clone
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy/clone this package to src/
# If you have this as a git repo:
# git clone <repo-url> mav_data_hose
# Or just copy the mav_data_hose directory here
```

### 3. Build the Package
```bash
cd ~/ros2_ws

# Build - this will automatically install pymavlink dependency
colcon build --packages-select mav_data_hose

source install/setup.bash
```

### 4. Run PX4 SITL (in separate terminal)
```bash
# Start PX4 Gazebo simulation
cd ~/PX4-Autopilot
make px4_sitl gazebo

# This will start MAVLink on udp:127.0.0.1:14550 by default
```

### 5. Launch the Fire Hose
```bash
# Terminal 1: Source workspace
cd ~/ros2_ws
source install/setup.bash

SPECIFICALLY, RUN THIS FOR AN ALL INCLUSIVE COMMAND:
colcon build --packages-select mav_data_hose && source install/setup.bash && ros2 run mav_data_hose firehose_node

# Run the node (default connects to udp:127.0.0.1:14550)
ros2 run mav_data_hose firehose_node

# OR use the launch file for more control:
ros2 launch mav_data_hose firehose.launch.py

# OR customize connection:
ros2 launch mav_data_hose firehose.launch.py connection_string:=udp:127.0.0.1:14550 data_stream_rate:=200
```

## Connection Strings

The node supports various MAVLink connection types:

```bash
# UDP (default for PX4 SITL)
ros2 run mav_data_hose firehose_node --ros-args -p connection_string:=udp:127.0.0.1:14550

# TCP
ros2 run mav_data_hose firehose_node --ros-args -p connection_string:=tcp:127.0.0.1:5760

# Serial (real drone)
ros2 run mav_data_hose firehose_node --ros-args -p connection_string:=/dev/ttyACM0

# Serial with baud rate
ros2 run mav_data_hose firehose_node --ros-args -p connection_string:=/dev/ttyUSB0:921600
```

## Monitoring the Fire Hose

### View Statistics
```bash
ros2 topic echo /mav/statistics
```

### List All Topics
```bash
ros2 topic list
```

### Monitor Message Rates
```bash
# See frequency of all messages
ros2 topic hz /mav/all_messages

# Monitor raw bytes throughput
ros2 topic hz /mav/raw/bytes

# Monitor IMU rate
ros2 topic hz /mav/imu
```

### View Raw Data
```bash
# See all messages (warning: LOTS of output!)
ros2 topic echo /mav/all_messages

# See raw bytes
ros2 topic echo /mav/raw/bytes

# See specific sensor
ros2 topic echo /mav/imu
```

## Consuming the Data (AI Model Example)

### Python Example - Subscribe to Raw Bytes
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class AIModelNode(Node):
    def __init__(self):
        super().__init__('ai_model_consumer')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/mav/raw/bytes',
            self.data_callback,
            100)  # Large queue for high frequency

    def data_callback(self, msg):
        # msg.data is a list of bytes - feed directly to your AI model
        raw_bytes = bytes(msg.data)
        # Process with your AI model...
        self.process_with_ai(raw_bytes)

    def process_with_ai(self, data):
        # Your AI model logic here
        pass

def main():
    rclpy.init()
    node = AIModelNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Python Example - Subscribe to JSON
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class JSONConsumerNode(Node):
    def __init__(self):
        super().__init__('json_consumer')
        self.subscription = self.create_subscription(
            String,
            '/mav/all_messages',
            self.json_callback,
            100)

    def json_callback(self, msg):
        data = json.loads(msg.data)
        msg_type = data['_msg_type']
        timestamp = data['_timestamp']
        # Process as needed...
```

## Performance Tuning

### Maximum Data Rate
```bash
# Request 200 Hz (may not be achievable depending on vehicle)
ros2 launch mav_data_hose firehose.launch.py data_stream_rate:=200
```

### Disable Raw Bytes (if you only need JSON)
```bash
ros2 launch mav_data_hose firehose.launch.py publish_raw_bytes:=false
```

### QoS Tuning
The node uses queue depth of 100 for high-frequency topics. Adjust in the code if needed.

## Troubleshooting

### No messages received
1. Check PX4 SITL is running: `ps aux | grep px4`
2. Check MAVLink port: `netstat -an | grep 14550`
3. Test connection: `python3 -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('udp:127.0.0.1:14550'); m.wait_heartbeat(); print('OK')"`

### Low message rate
1. Increase `data_stream_rate` parameter
2. Check PX4 parameters (some may limit rates)
3. Monitor with statistics topic

### Permission denied on serial port
```bash
sudo chmod 666 /dev/ttyACM0
# Or add user to dialout group
sudo usermod -a -G dialout $USER
```

## Architecture

```
┌─────────────┐         ┌──────────────────┐         ┌─────────────┐
│  PX4 SITL   │ MAVLink │  MAV FIRE HOSE   │  ROS2   │  AI Model   │
│  (Gazebo)   ├────────>│   (pymavlink)    ├────────>│  Consumer   │
│             │ UDP     │   100+ Hz        │ Topics  │             │
└─────────────┘ :14550  └──────────────────┘         └─────────────┘
```

---

# MAV UNIFORM PUMP

## Overview

The Uniform Pump subscribes to the MAV FIRE HOSE data and packages it into controlled, time-batched data structures optimized for AI/ML classification models. Unlike the firehose which streams at maximum frequency, the Uniform Pump delivers data at regular intervals (dT) with configurable aggregation strategies.

## Key Concepts

### Batch Interval (dT)
The time period over which data is collected before packaging and publishing. Default is 1 second, but can be configured to any value (e.g., 0.5s, 2s, etc.).

### Condensation Modes

1. **Raw Mode** (`raw`)
   - Packages all messages received during dT with timestamps
   - No averaging or condensation
   - Best for models that need full temporal resolution

2. **Multi-Step Mode** (`multi_step`)
   - Divides the batch interval into N discrete time steps
   - Averages messages within each step
   - Configurable number of steps (default: 3)
   - Example: 1s batch with 3 steps = data at [0-0.33s, 0.33-0.66s, 0.66-1s]
   - Best for temporal classification models

3. **Single-Step Mode** (`single_step`)
   - Averages all messages in the batch interval into a single data point
   - Maximum condensation
   - Best for models that only need steady-state values

### Bleeding Domain

A configurable time window (default: 15 seconds) of historical data maintained to fill gaps when certain message types are not present in the current batch.

**Purpose:** Ensure every batch contains all expected message types, even if some weren't received in the current dT.

### Missing Data Strategies

When a message type is missing from the current batch, the Uniform Pump can:

1. **`bleeding_average`** (default): Average all instances from the bleeding domain
2. **`bleeding_latest`**: Use the most recent value from bleeding domain
3. **`null`**: Mark as null/None
4. **`zero`**: Fill with zeros (for numeric fields)

## Running the Uniform Pump

### Quick Start (alongside Fire Hose)

```bash
# Terminal 1: Run Fire Hose
cd ~/ros2_ws
source install/setup.bash
ros2 run mav_data_hose firehose_node

# Terminal 2: Run Uniform Pump
cd ~/ros2_ws
source install/setup.bash
ros2 run mav_data_hose uniform_pump_node
```

### Using Launch File

```bash
# Default (1s batches, raw mode, 15s bleeding domain)
ros2 launch mav_data_hose uniform_pump.launch.py

# Custom configuration
ros2 launch mav_data_hose uniform_pump.launch.py \
  batch_interval:=2.0 \
  condensation_mode:=multi_step \
  multi_step_count:=5 \
  bleeding_domain_duration:=30.0 \
  missing_data_strategy:=bleeding_latest
```

### Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `batch_interval` | float | 1.0 | Batch interval (dT) in seconds |
| `condensation_mode` | string | 'raw' | Mode: `raw`, `multi_step`, or `single_step` |
| `multi_step_count` | int | 3 | Number of steps for multi_step mode |
| `bleeding_domain_duration` | float | 15.0 | Historical data window in seconds |
| `missing_data_strategy` | string | 'bleeding_average' | Strategy for missing data |

### Examples

**AI Classification at 1Hz with averaged data:**
```bash
ros2 launch mav_data_hose uniform_pump.launch.py \
  batch_interval:=1.0 \
  condensation_mode:=single_step
```

**Temporal model with 5 time steps per 2 seconds:**
```bash
ros2 launch mav_data_hose uniform_pump.launch.py \
  batch_interval:=2.0 \
  condensation_mode:=multi_step \
  multi_step_count:=5
```

**Raw data batches every 0.5 seconds:**
```bash
ros2 launch mav_data_hose uniform_pump.launch.py \
  batch_interval:=0.5 \
  condensation_mode:=raw
```

## Consuming Uniform Pump Data

### Python Example - AI Classification Model

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

class AIClassifierNode(Node):
    def __init__(self):
        super().__init__('ai_classifier')

        # Subscribe to uniform batches
        self.subscription = self.create_subscription(
            String,
            '/mav/uniform_batch',
            self.batch_callback,
            10)

    def batch_callback(self, msg):
        # Parse batch
        batch = json.loads(msg.data)

        # Extract metadata
        metadata = batch['_batch_metadata']
        batch_num = metadata['batch_number']
        msg_types = metadata['message_types_count']

        # Extract data
        data = batch['data']

        # Example: Extract IMU data
        if 'SCALED_IMU' in data:
            imu_data = data['SCALED_IMU']

            # Single-step mode: direct values
            if isinstance(imu_data, dict):
                accel_x = imu_data.get('xacc', 0)
                self.get_logger().info(f'Batch {batch_num}: accel_x = {accel_x}')

            # Multi-step mode: list of averaged steps
            elif isinstance(imu_data, list):
                for i, step_data in enumerate(imu_data):
                    if step_data:
                        accel_x = step_data.get('xacc', 0)
                        self.get_logger().info(f'Batch {batch_num}, Step {i}: accel_x = {accel_x}')

            # Raw mode: list of all messages
            else:
                # Process list of timestamped messages
                pass

        # Run your AI model
        prediction = self.run_classification_model(data)
        self.get_logger().info(f'Classification: {prediction}')

    def run_classification_model(self, data):
        # Your AI model here
        # Example: Extract features, run inference, return prediction
        return "NORMAL"  # Placeholder

def main():
    rclpy.init()
    node = AIClassifierNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Batch Data Structure

**Single-Step Mode:**
```json
{
  "data": {
    "ATTITUDE": {
      "roll": 0.123,
      "pitch": -0.045,
      "yaw": 1.234,
      "rollspeed": 0.01,
      "pitchspeed": 0.02,
      "yawspeed": 0.03
    },
    "SCALED_IMU": {
      "xacc": 0.98,
      "yacc": 0.05,
      "zacc": 9.81,
      "xgyro": 0.001,
      "ygyro": 0.002,
      "zgyro": 0.003
    }
  },
  "_batch_metadata": {
    "batch_number": 42,
    "batch_start_time": 1234567890.123,
    "batch_end_time": 1234567891.123,
    "batch_interval": 1.0,
    "condensation_mode": "single_step",
    "message_types_count": 15,
    "total_messages_in_batch": 342
  }
}
```

**Multi-Step Mode (3 steps):**
```json
{
  "data": {
    "ATTITUDE": [
      {"roll": 0.120, "pitch": -0.040, "yaw": 1.230},  // Step 0: avg of 0-0.33s
      {"roll": 0.123, "pitch": -0.045, "yaw": 1.234},  // Step 1: avg of 0.33-0.66s
      {"roll": 0.125, "pitch": -0.048, "yaw": 1.237}   // Step 2: avg of 0.66-1s
    ]
  },
  "_batch_metadata": { ... }
}
```

**Raw Mode:**
```json
{
  "data": {
    "ATTITUDE": [
      {"roll": 0.120, "pitch": -0.040, "_timestamp": 1234567890.123},
      {"roll": 0.121, "pitch": -0.041, "_timestamp": 1234567890.223},
      {"roll": 0.122, "pitch": -0.042, "_timestamp": 1234567890.323},
      // ... all messages in batch with timestamps
    ]
  },
  "_batch_metadata": { ... }
}
```

## Monitoring

### View Batches
```bash
ros2 topic echo /mav/uniform_batch
```

### Monitor Batch Rate
```bash
ros2 topic hz /mav/uniform_batch
```

### Dashboard Controls

The Uniform Pump includes a live Rich dashboard with:
- **Configuration panel**: Current settings
- **Batch history**: Recent batches with metadata
- **System logs**: Status messages
- **Statistics**: Batch rate, message counts, bleeding domain size

**Keyboard controls:**
- `←/→` Arrow keys: Navigate batch history
- `+/-`: Adjust items per page
- `q`: Quit

## Architecture

```
┌─────────────────┐         ┌───────────────────┐         ┌─────────────────┐
│  MAV FIRE HOSE  │ ROS2    │  UNIFORM PUMP     │  ROS2   │  AI Classifier  │
│  (100+ Hz)      ├────────>│  - Batch every dT │────────>│  - Run model    │
│  /mav/all_msgs  │ Topics  │  - Aggregate      │ 1-10 Hz │  - Classify     │
│                 │         │  - Fill gaps      │         │  - Publish pred │
└─────────────────┘         └───────────────────┘         └─────────────────┘
                                     ↓
                            Bleeding Domain
                            (15s history)
```

## Use Cases

1. **Anomaly Detection**: Single-step mode at 1Hz for detecting abnormal flight states
2. **Temporal Classification**: Multi-step mode for recognizing flight maneuvers over time
3. **Data Logging**: Raw mode for collecting full-resolution data in batches
4. **Reduced Bandwidth**: Single-step or multi-step for sending condensed data over network

## License

Apache-2.0

## Future Enhancements

- [ ] Add message filtering/selection
- [ ] Add data recording (rosbag2)
- [ ] Add compression options
- [ ] Add multiple vehicle support
- [ ] Add custom message definitions for raw MAVLink
- [x] Controlled batching for AI/ML (Uniform Pump)
- [ ] Add batch prediction feedback loop
- [ ] Support for custom aggregation functions
