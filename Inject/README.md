# MAV_INJECT

ROS2 package for dynamically sabotaging and editing PX4 drone parameters during flight operations via MAVLink.

## Overview

MAV_INJECT enables real-time modification of PX4 drone flight parameters as part of an autonomous diagnostics and testing system. The system can:

- Connect to PX4 via MAVLink protocol (udp:127.0.0.1:14550)
- Read all current parameter values from a running PX4 instance
- Set parameters with automatic type detection (FLOAT/INT32)
- Disable safety checks to allow extreme parameter modifications
- Sabotage flight control with inverted axes, extreme gains, and disabled stabilization
- Export complete parameter lists as JSON for analysis
- Run automated test sequences that progressively degrade drone performance

## Quick Start with RunEnv.sh

The easiest way to get started is using the automated deployment script:

```bash
# Normal startup (sabotage tests)
./RunEnv.sh

# Startup + export all parameters to JSON
./RunEnv.sh --list-params
```

This will:
1. Sync files from shared folder to VM
2. Build the PX4 Parameter Bridge C library
3. Launch PX4 SITL with Gazebo simulation
4. Launch MAVProxy
5. (Optional) Export all parameters to timestamped JSON file
6. Launch mav_inject with automated sabotage tests
7. Launch drone movement test script

## Project Structure

```
MavInject/
├── mav_inject/
│   ├── __init__.py
│   ├── injection_test.py      # Main ROS2 node with automated sabotage tests
│   ├── config_controller.py   # Config file controller (legacy)
│   ├── param_lister.py        # Parameter export utility
│   ├── px4_param_api.py       # Python ctypes wrapper for C API
│   ├── px4_param_bridge.c     # C bridge library (direct PX4 access)
│   └── CMakeLists.txt         # CMake build config for C bridge
├── build_bridge.sh            # Script to build C bridge library
├── RunEnv.sh                  # Automated environment setup
├── package.xml                # ROS2 package manifest
├── setup.py                   # Python package setup
└── README.md
```

## Components

### 1. injection_test.py - Main Sabotage Node

The primary ROS2 node that connects to PX4 and runs automated sabotage tests.

**Features:**
- Dual-mode operation: Direct C API (if available) or MAVLink fallback
- Automatic MAVLink heartbeat detection
- Asynchronous message handling with proper request/response matching
- 7 automated sabotage tests that progressively destroy flight stability

**Automated Test Sequence:**

1. **Disable Preflight Checks** - Disables circuit breakers and arming checks
2. **Invert Roll Control** - Makes roll respond backwards (MC_ROLL_P = -7.0)
3. **Invert Pitch Control** - Makes pitch respond backwards (MC_PITCH_P = -7.0)
4. **Extreme Oscillation** - Massive I gains + zero D (MC_ROLLRATE_I = 5.0)
5. **Disable Altitude Control** - Zero altitude stabilization (MPC_Z_P = 0.0)
6. **Inverted Yaw + Fast Spin** - Backwards yaw + 400°/s max rate
7. **Verify All Parameters** - Read back all sabotaged values

**Expected Results:**
- Drone will flip immediately when trying to stabilize
- Violent oscillations and shaking
- Uncontrollable spinning
- Altitude drop/loss of height control
- Failed movement tests

### 2. param_lister.py - Parameter Export Utility

Fetches all available parameters from PX4 via MAVLink and exports as JSON.

**Usage:**

```bash
# Via RunEnv.sh (automatic)
./RunEnv.sh --list-params

# Standalone after building
ros2 run mav_inject param_lister --output params.json

# Direct Python
python3 mav_inject/param_lister.py --output params.json

# Custom connection and timeout
ros2 run mav_inject param_lister \
  --connection udp:127.0.0.1:14550 \
  --timeout 60 \
  --output my_params.json
```

**Command Line Options:**
- `--connection`, `-c`: MAVLink connection string (default: udp:127.0.0.1:14550)
- `--timeout`, `-t`: Timeout in seconds (default: 30)
- `--output`, `-o`: Output JSON file path (default: stdout)

**Output Format:**

```json
{
  "timestamp": 1700000000.0,
  "system_id": 1,
  "component_id": 0,
  "total_parameters": 1018,
  "parameters": {
    "MC_ROLL_P": {
      "value": 7.0,
      "type": "REAL32",
      "type_id": 9,
      "index": 549
    },
    "CBRK_USB_CHK": {
      "value": 197848,
      "type": "INT32",
      "type_id": 6,
      "index": 123
    }
  }
}
```

**Parameter Types:**
- `UINT8`, `INT8`: 8-bit unsigned/signed integers
- `UINT16`, `INT16`: 16-bit unsigned/signed integers
- `UINT32`, `INT32`: 32-bit unsigned/signed integers (circuit breakers)
- `UINT64`, `INT64`: 64-bit unsigned/signed integers
- `REAL32`: 32-bit floating point (most flight parameters)
- `REAL64`: 64-bit floating point

**JSON Queries with jq:**

```bash
# Find all circuit breakers
ros2 run mav_inject param_lister | jq '.parameters | to_entries[] | select(.key | startswith("CBRK_"))'

# Find roll-related parameters
ros2 run mav_inject param_lister | jq '.parameters | to_entries[] | select(.key | contains("ROLL"))'

# Count parameters
ros2 run mav_inject param_lister | jq '.total_parameters'
```

### 3. px4_param_bridge.c & px4_param_api.py - Direct C API (Experimental)

Attempts to provide direct C-level access to PX4's parameter functions, bypassing MAVLink.

**Status:** Currently non-functional because PX4's `param_set()`/`param_get()` functions are not exported as shared library symbols. Falls back to MAVLink automatically.

**Build:**
```bash
./build_bridge.sh /path/to/PX4-Autopilot
```

## Installation

### Prerequisites

- ROS2 Kilted
- PX4-Autopilot (SITL)
- MAVProxy
- Python 3 with pymavlink

### Build Steps

```bash
# 1. Clone to ROS2 workspace
cd ~/ros2_ws/src
git clone <repo> MavInject

# 2. Build C bridge (optional, will fall back to MAVLink if fails)
cd MavInject
./build_bridge.sh ~/Documents/PX4-Autopilot

# 3. Build ROS2 package
cd ~/ros2_ws
colcon build --packages-select mav_inject
source install/setup.bash
```

## Manual Usage (Without RunEnv.sh)

### Start PX4 SITL

```bash
cd ~/Documents/PX4-Autopilot
make px4_sitl gz_x500
```

### Start MAVProxy

```bash
cd ~/Documents/MAVProxy
mavproxy.py --master=udp:127.0.0.1:14550
```

### Run MAV_INJECT

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run mav_inject injection_test --ros-args -p px4_build_path:=/path/to/PX4-Autopilot
```

## ROS2 Topics

### Published Topics

- `/px4_injector/status` (std_msgs/String): Status updates and health checks

### Subscribed Topics

- `/px4_injector/command` (std_msgs/String): JSON commands for config injection

## Key Parameters Modified

### Safety & Arming

- `CBRK_SUPPLY_CHK`: 894281 (INT32) - Disable power supply check
- `CBRK_USB_CHK`: 197848 (INT32) - Allow arming with USB
- `CBRK_IO_SAFETY`: 22027 (INT32) - Disable IO safety switch
- `CBRK_FLIGHTTERM`: 121212 (INT32) - Disable flight termination
- `COM_ARM_IMU_ACC`: 1000.0 (FLOAT) - Disable accel check
- `COM_ARM_IMU_GYR`: 1000.0 (FLOAT) - Disable gyro check
- `COM_DISARM_PRFLT`: -1.0 (FLOAT) - Disable auto-disarm

### Attitude Control (Sabotaged)

- `MC_ROLL_P`: -7.0 (INVERTED! Normal: 7.0)
- `MC_PITCH_P`: -7.0 (INVERTED! Normal: 7.0)
- `MC_ROLLRATE_P`: -0.15 (INVERTED! Normal: 0.15)
- `MC_PITCHRATE_P`: -0.15 (INVERTED! Normal: 0.15)
- `MC_YAW_P`: -2.8 (INVERTED! Normal: 2.8)
- `MC_YAWRATE_P`: -0.2 (INVERTED! Normal: 0.2)

### Rate Control (Sabotaged)

- `MC_ROLLRATE_I`: 5.0 (25x normal! Causes oscillation)
- `MC_PITCHRATE_I`: 5.0 (25x normal! Causes oscillation)
- `MC_ROLLRATE_D`: 0.0 (No damping! Normal: 0.003)
- `MC_PITCHRATE_D`: 0.0 (No damping! Normal: 0.003)
- `MC_YAWRATE_MAX`: 400.0 (2x normal! Normal: 200.0)

### Position Control (Sabotaged)

- `MPC_XY_P`: 0.1 (10x smaller! Normal: 0.95)
- `MPC_Z_P`: 0.0 (DISABLED! Normal: 1.0)
- `MPC_XY_VEL_P_ACC`: 0.5 (2x smaller! Normal: 1.8)
- `MPC_Z_VEL_P_ACC`: 0.0 (DISABLED! Normal: 4.0)
- `MPC_THR_HOVER`: 0.3 (Too low! Normal: 0.5)

## Architecture

### Access Methods (Current)

1. **MAVLink Protocol** (Primary): Uses `PARAM_SET` and `PARAM_VALUE` messages
   - Handles both REAL32 (float) and INT32 parameter types
   - Asynchronous message handling with proper request/response matching
   - Robust against out-of-order MAVLink responses

2. **Direct C API** (Fallback): Attempts to load `libpx4_param_bridge.so`
   - Currently non-functional (symbols not available)
   - Automatically falls back to MAVLink if loading fails

### MAVLink Parameter Types

The system correctly handles different parameter types:

```python
# FLOAT parameters (most common)
mav.mav.param_set_send(
    system, component,
    param_name.encode('utf-8'),
    float_value,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

# INT32 parameters (circuit breakers)
mav.mav.param_set_send(
    system, component,
    param_name.encode('utf-8'),
    float(int_value),  # Sent as float with INT32 flag
    mavutil.mavlink.MAV_PARAM_TYPE_INT32
)
```

## Troubleshooting

### "No acknowledgment received for parameter"

This is normal for some parameters that don't exist or are read-only. The system continues with other parameters.

### "Preflight Fail" / Auto-disarm

Run the automated tests - Test 1 disables all preflight checks. Or manually set:
```bash
ros2 topic pub /px4_injector/command std_msgs/String \
  "data: '{\"action\": \"disable_checks\"}'"
```

### Gazebo window disappeared

Restart PX4 SITL terminal. The Gazebo GUI sometimes crashes in SITL.

### Parameters not taking effect

Check that:
1. MAVLink is connected (check logs for "MAVLink connected!")
2. Parameters are being confirmed (look for "SUCCESS:" messages)
3. PX4 hasn't rejected the values (check PX4 terminal for errors)

## Development Notes

### Adding New Tests

Edit `injection_test.py` and add to `run_automated_tests()`:

```python
# Test X: Description
self.get_logger().info('--- Test X: Description ---')
self.set_parameters({
    'PARAM_NAME': value,
})
time.sleep(0.5)
```

### Testing Individual Parameters

```bash
# Read a parameter
ros2 run mav_inject param_lister | jq '.parameters.MC_ROLL_P'

# Or use MAVProxy
param show MC_ROLL_P
param set MC_ROLL_P 7.0
```

## Known Issues

1. C API bridge doesn't work (PX4 symbols not exported) - using MAVLink instead
2. Some circuit breaker parameters don't exist on all PX4 versions
3. Extreme parameter values may cause PX4 to crash or reject changes
4. Inverted controls are VERY dangerous - drone will crash immediately

## Safety Notes

⚠️ **WARNING**: This system is designed to SABOTAGE drone flight performance!

- Only use in SITL simulation
- Never use on real hardware
- Inverted control parameters will cause immediate crashes
- Extreme I gains will cause violent oscillations
- Zero altitude control will cause the drone to drop

## License

MIT

## Authors

MAV_INJECT Team - Cornell MAGPIE Project
