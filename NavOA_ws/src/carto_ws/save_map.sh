#!/bin/bash
# Save Cartographer map in both .pbstream and .pgm formats
# Usage: ./save_map.sh [map_name]
# Example: ./save_map.sh meeting_room

MAP_NAME=${1:-"map"}
SCRIPT_DIR="$( cd "$(dirname "$0")" && pwd )"
WS_ROOT="$( cd "${SCRIPT_DIR}/../.." && pwd )"

# Source workspace so cartographer_ros_msgs types are available
if [ -f "${WS_ROOT}/devel_isolated/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/devel_isolated/setup.bash"
fi

MAP_DIR="${SCRIPT_DIR}/../map/${MAP_NAME}"
mkdir -p "$MAP_DIR"

MAP_PATH="${MAP_DIR}/${MAP_NAME}"

echo "[1/3] Finishing trajectory (triggers final optimization)..."
rosservice call /finish_trajectory 0

echo "[2/3] Saving .pbstream to ${MAP_PATH}.pbstream ..."
rosservice call /write_state \
  "{filename: '${MAP_PATH}.pbstream', include_unfinished_submaps: true}"

echo "[3/3] Saving .pgm + .yaml to ${MAP_PATH} ..."
rosrun map_server map_saver -f "${MAP_PATH}"

echo ""
echo "Done! Files saved:"
echo "  ${MAP_PATH}.pbstream  (for Cartographer localization)"
echo "  ${MAP_PATH}.pgm       (for Nav2 / map_server)"
echo "  ${MAP_PATH}.yaml      (for Nav2 / map_server)"
