#!/usr/bin/env bash
# MAV_INJECT Environment Setup Script
#
# Usage:
#   ./RunEnv.sh                 # Normal startup
#   ./RunEnv.sh --list-params   # Startup + dump all PX4 parameters to JSON file
#
set -e

# Check for optional --list-params flag
LIST_PARAMS=false
if [[ "$1" == "--list-params" ]]; then
    LIST_PARAMS=true
    echo "Will list all parameters after starting PX4..."
fi

# 1. Replace contents of mav_inject with files from /media/sf_VM-Shared
SRC_DIR="/media/sf_VM-Shared"
DEST_DIR="/home/j10pr/ros2_ws/src/mav_inject"

# Mirror SRC_DIR into DEST_DIR, deleting anything in DEST_DIR that is not in SRC_DIR
rsync -av --delete "${SRC_DIR}/" "${DEST_DIR}/"

# 2. Build the PX4 Parameter Bridge C library
echo "Building PX4 Parameter Bridge C library..."
cd "${DEST_DIR}"
./build_bridge.sh /home/j10pr/Documents/PX4-Autopilot

# 3. Start PX4 SITL in a new terminal
echo "Starting PX4 SITL..."
gnome-terminal -- bash -c 'cd ~/Documents/PX4-Autopilot && make px4_sitl gz_x500; exec bash'

# 4. Start MAVProxy in another new terminal
echo "Starting MAVProxy..."
gnome-terminal -- bash -c 'cd ~/Documents/MAVProxy/ && mavproxy.py --master=udp:127.0.0.1:14550; exec bash'

# Wait for user to confirm PX4 and MAVProxy are ready
echo ""
echo "Waiting for PX4 SITL and MAVProxy to finish starting up..."
echo "Press ENTER when both terminals show they are ready"
read -r

# Optional: List all parameters if --list-params flag was provided
if [ "$LIST_PARAMS" = true ]; then
    echo ""
    echo "Listing all PX4 parameters..."
    cd /home/j10pr/ros2_ws
    colcon build --packages-select mav_inject
    source install/setup.bash

    OUTPUT_FILE="/home/j10pr/px4_parameters_$(date +%Y%m%d_%H%M%S).json"
    echo "Fetching parameters and saving to: $OUTPUT_FILE"
    ros2 run mav_inject param_lister --connection udp:127.0.0.1:14550 --output "$OUTPUT_FILE"

    echo ""
    echo "Parameters saved! You can view them at: $OUTPUT_FILE"
    echo "Press ENTER to continue to mav_inject..."
    read -r
fi

# 5. Build and run mav_inject in a third terminal
echo "Building and running mav_inject..."
gnome-terminal -- bash -c '
cd /home/j10pr/ros2_ws && \
colcon build --packages-select mav_inject && \
source install/setup.bash && \
export PX4_PARAM_BRIDGE_LIB=/home/j10pr/ros2_ws/src/mav_inject/mav_inject/build/libpx4_param_bridge.so && \
ros2 run mav_inject injection_test --ros-args -p px4_build_path:=/home/j10pr/Documents/PX4-Autopilot;
exec bash'

# Wait for user to confirm mav_inject is running
echo ""
echo "Waiting for mav_inject to finish starting up..."
echo "Press ENTER when the mav_inject terminal shows it is ready"
read -r

# 6. Build and run the ROS2 Interpolation Movement Test
echo "Building and running ROS2 Interpolation Movement Test..."
gnome-terminal -- bash -c '
cd /home/j10pr/ros2_ws && \
colcon build --packages-select drone_api && \
source install/setup.bash && \
ros2 run drone_api test_demo'

echo "All terminals launched!"
