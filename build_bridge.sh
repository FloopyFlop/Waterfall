#!/usr/bin/env bash
# Build script for the PX4 parameter bridge library used by Waterfall.
#
# Usage:
#   ./build_bridge.sh /path/to/PX4-Autopilot
#
set -euo pipefail

if [[ -z "${1:-}" ]]; then
    echo "Usage: $0 /path/to/PX4-Autopilot"
    echo ""
    echo "Example:"
    echo "  $0 \$HOME/Documents/PX4-Autopilot"
    exit 1
fi

if command -v realpath >/dev/null 2>&1; then
    PX4_DIR="$(realpath "$1")"
else
    PX4_DIR="$(cd "$1" && pwd)"
fi
if [[ ! -d "$PX4_DIR" ]]; then
    echo "Error: PX4 directory not found: $PX4_DIR"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/waterfall/build"
SRC_FILE="${SCRIPT_DIR}/waterfall/px4_param_bridge.c"

EXT="so"
if [[ "$(uname -s)" == "Darwin" ]]; then
    EXT="dylib"
fi

OUT_LIB="${BUILD_DIR}/libpx4_param_bridge.${EXT}"

mkdir -p "${BUILD_DIR}"

echo "Building PX4 parameter bridge..."
echo "PX4 Directory: ${PX4_DIR}"
echo "Output: ${OUT_LIB}"

if [[ "${EXT}" == "dylib" ]]; then
    cc -dynamiclib -o "${OUT_LIB}" "${SRC_FILE}"
else
    cc -shared -fPIC -o "${OUT_LIB}" "${SRC_FILE}" -ldl
fi

echo ""
echo "Build complete!"
echo "Set this before running Inject:"
echo "  export PX4_PARAM_BRIDGE_LIB=${OUT_LIB}"
echo ""
echo "Note: the bridge only works when PX4 exports param_* symbols."
