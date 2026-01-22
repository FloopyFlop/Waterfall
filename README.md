# Waterfall Manager

Single ROS2 wrapper that launches the four project services (Firehose, UniformPump, Orchestra, Inject) from one node while keeping each original UI/dashboard intact.

All service code is vendored inside this package (`waterfall/firehose_service.py`, `waterfall/uniform_pump_service.py`, `waterfall/inject_service.py`, `waterfall/drone_api/*` with `orchestra_service`). The legacy `Hose/`, `Inject/`, and `Orchestra/` trees are no longer needed for runtime once this package is built.

## Build
```bash
# 0. Sync source code to build machine (if needed)
HOST> rsync -avz --delete --ignore-times ./waterfall magpie@192.168.55.1:~/abm-sync/src/

# 1a. Create workspace and clone repo
mkdir -p ~/abm-sync/src
cd ~/abm-sync/

# 1b. Create a virtual environment using 'uv', linking system ROS packages
uv venv --system-site-packages .venv
source .venv/bin/activate
uv pip install pymavlink rich pyserial

# 2. Build waterfall package
PYTHON=$(which python)
$PYTHON -m colcon build --symlink-install --packages-select waterfall
source install/setup.bash
```

## Run examples
- Start Firehose + UniformPump in SITL:
  ```bash
  ros2 run waterfall waterfall_node --sitl --services firehose uniform_pump
  ```
- Start all services with a hardware serial connection:
  ```bash
  ros2 run waterfall waterfall_node --all --connection serial:/dev/ttyTHS3:115200
  ```
- Firehose + UniformPump on a real drone (serial hardware):
  ```bash
  ros2 run waterfall waterfall_node --services firehose uniform_pump \
    --connection serial:/dev/ttyTHS3:115200 --serial-baud 115200
  ```
- Include Inject (needs PX4 build path):
  ```bash
  ros2 run waterfall waterfall_node --services firehose uniform_pump inject \
    --inject-px4-path /path/to/PX4-Autopilot/build/px4_sitl_default
  ```
- Override Orchestra connection:
  ```bash
  ros2 run waterfall waterfall_node --services orchestra --orchestra-connection udp://:14540
  ```
- Coordinator demo (launch Firehose + Pump + Inject, watch Firehose data, trigger Inject when a threshold is crossed):
  ```bash
  ros2 run waterfall waterfall_coordinator \
    --services firehose uniform_pump inject \
    --connection serial:/dev/ttyTHS3:115200 --serial-baud 115200 \
    --px4-build-path /path/to/PX4-Autopilot/build/px4_sitl_default \
    --trigger-msg-type SCALED_IMU --trigger-field xacc --trigger-threshold 5.0 \
    --inject-set MC_ROLL_P=-7.0 MC_PITCH_P=-7.0 \
    --inject-file ROMFS/px4fmu_common/init.d/rc.autostart
  ```
  Add `--fly-circle` to also run a simple circle mission via the vendored DroneAPI (MAVSDK).
- Fault monitor demo (matches the “throttle up, classify, wait for disarm, then Inject” flow):
  ```bash
  ros2 run waterfall waterfall_fault_demo
  ```
  Configuration is declarative inside `waterfall/fault_monitor_demo.py` (edit the `CONFIG` block and/or `example_model`).

## Key flags
- `--sitl` switches all MAVLink/MAVSDK clients to the configured SITL endpoints (`--sitl-connection` for Firehose/Inject, `--orchestra-sitl` for Orchestra).
- `--connection` overrides Firehose/Inject connection (udp/tcp/serial). Hardware defaults use `--serial-port` and `--serial-baud`.
- Serial helpers: you can pass `serial:/dev/ttyTHS3:115200` or `/dev/ttyUSB0:57600` and Firehose will normalize to the device path while applying the baud.
- UniformPump tuning: `--batch-interval`, `--condensation-mode`, `--multi-step-count`, `--bleeding-domain`, `--missing-strategy`.
- Extra args to pass through to underlying nodes: `--firehose-args`, `--pump-args`, `--inject-args`, `--orchestra-args`.

The node publishes basic lifecycle status on `waterfall/status` for monitoring.

## PWM_MAIN_FUNC4 Demo Script

A focused demo script for modifying the `PWM_MAIN_FUNC4` parameter specifically:

```bash
# Basic demo with default value (1)
python3 pwm_func4_demo.py --px4-build-path /path/to/PX4-Autopilot/build/px4_sitl_default

# Set specific value (e.g., 3 for servo output)  
python3 pwm_func4_demo.py --px4-build-path /path/to/px4 --target-value 3

# Use SITL connection
python3 pwm_func4_demo.py --px4-build-path /path/to/px4 --sitl --target-value 2

# Hardware connection
python3 pwm_func4_demo.py --px4-build-path /path/to/px4 \
  --serial-port /dev/ttyTHS3 --serial-baud 115200 --target-value 4
```

This script:
- Focuses only on the `PWM_MAIN_FUNC4` parameter (PWM Main 4 output function)
- Reads current value, sets new value, and verifies the change
- Supports both PX4 C API and MAVLink connections
- Minimal implementation without the complex automated tests from the full inject service

**Note:** Modifying PWM function parameters can affect vehicle behavior. Ensure you understand the parameter's impact before making changes.
