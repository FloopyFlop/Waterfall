#!/bin/bash
#
# Build script for PX4 Parameter Bridge C library
#
# Usage:
#   ./build_bridge.sh /path/to/PX4-Autopilot
#

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 /path/to/PX4-Autopilot"
    echo ""
    echo "Example:"
    echo "  ./build_bridge.sh /home/j10pr/Documents/PX4-Autopilot"
    exit 1
fi

PX4_DIR=$(realpath "$1")

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4 directory not found: $PX4_DIR"
    exit 1
fi

echo "Building PX4 Parameter Bridge"
echo "PX4 Directory: $PX4_DIR"
echo ""

# Create build directory
BUILD_DIR="$(dirname "$0")/mav_inject/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Run CMake
echo "Running CMake..."
cmake -DPX4_DIR="$PX4_DIR" ..

# Build
echo "Building..."
make

echo ""
echo "Build complete!"
echo "Library location: $BUILD_DIR/libpx4_param_bridge.so (or .dylib on macOS)"
echo ""
echo "To use this library, set:"
echo "  export PX4_PARAM_BRIDGE_LIB=$BUILD_DIR/libpx4_param_bridge.so"
