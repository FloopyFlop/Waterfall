#!/usr/bin/env bash
# Waterfall Environment Setup Script (SITL demo)
#
# This script syncs the current repo into ~/waterfall (or $WATERFALL_WS),
# builds the Waterfall package, launches PX4 SITL + MAVProxy, and runs the
# fault monitor demo.
#
# Usage:
#   ./RunEnv.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WORKSPACE_ROOT="${WATERFALL_WS:-$HOME/ros2_ws/waterfall}"
SRC_DIR="${SCRIPT_DIR}"
DEST_DIR="${WORKSPACE_ROOT}/src/waterfall"

PX4_DIR="${PX4_DIR:-$HOME/Documents/PX4-Autopilot}"
MAVPROXY_DIR="${MAVPROXY_DIR:-$HOME/Documents/MAVProxy}"
BUILD_PARAM_BRIDGE="${BUILD_PARAM_BRIDGE:-1}"
BRIDGE_LIB=""

echo "Syncing Waterfall source..."
mkdir -p "${DEST_DIR}"
rsync -av --delete \
  --exclude ".git" \
  --exclude "build" \
  --exclude "install" \
  --exclude "log" \
  --exclude "__pycache__" \
  --exclude ".venv" \
  "${SRC_DIR}/" "${DEST_DIR}/"

if [[ "${BUILD_PARAM_BRIDGE}" == "1" ]]; then
  echo "Building PX4 parameter bridge..."
  (cd "${DEST_DIR}" && ./build_bridge.sh "${PX4_DIR}")
  EXT="so"
  if [[ "$(uname -s)" == "Darwin" ]]; then
    EXT="dylib"
  fi
  BRIDGE_LIB="${DEST_DIR}/waterfall/build/libpx4_param_bridge.${EXT}"
fi

echo "Building Waterfall in ${WORKSPACE_ROOT}..."
cd "${WORKSPACE_ROOT}"
colcon build --symlink-install --packages-select waterfall
# colcon-generated setup may reference unset vars under 'set -u'
set +u
source install/setup.bash
set -u

echo "Starting PX4 SITL..."
gnome-terminal -- bash -c "cd \"${PX4_DIR}\" && make px4_sitl gz_x500; exec bash"

echo "Starting MAVProxy..."
gnome-terminal -- bash -c "cd \"${MAVPROXY_DIR}\" && mavproxy.py --master=udp:127.0.0.1:14550; exec bash"

echo ""
echo "Waiting for PX4 SITL and MAVProxy to finish starting up..."
echo "Press ENTER when both terminals show they are ready"
read -r

echo "Launching Waterfall motor reverse finder..."
PX4_BUILD_PATH="${PX4_BUILD_PATH:-${PX4_DIR}/build/px4_sitl_default}"
ENV_EXPORTS="export PX4_BUILD_PATH=\"${PX4_BUILD_PATH}\""
if [[ -n "${BRIDGE_LIB}" && -f "${BRIDGE_LIB}" ]]; then
  ENV_EXPORTS="${ENV_EXPORTS} && export PX4_PARAM_BRIDGE_LIB=\"${BRIDGE_LIB}\""
fi
gnome-terminal -- bash -c "cd \"${WORKSPACE_ROOT}\" && source install/setup.bash && ${ENV_EXPORTS} && ros2 run waterfall waterfall_motor_finder; exec bash"

echo "All terminals launched!"
