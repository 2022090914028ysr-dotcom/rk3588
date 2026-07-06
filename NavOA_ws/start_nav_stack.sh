#!/bin/bash
# Start the full navigation stack: LiDAR + cartographer localization + NavOA.
# Sequential launch ensures TF topics are established before each dependent node.
#
# Usage:
#   ./start_nav_stack.sh                                # everything on (camera + ultrasonic)
#   ./start_nav_stack.sh --no-camera                    # disable camera
#   ./start_nav_stack.sh --no-ultrasonic                # disable ultrasonic
#   ./start_nav_stack.sh --no-camera --no-ultrasonic    # LiDAR + carto + NavOA only
#
# All flags can also be set via env vars:
#   USE_CAMERA=false USE_ULTRASONIC=false ./start_nav_stack.sh

set -e

# --- Defaults (match lidar_full.launch defaults) ---
USE_CAMERA=true
USE_ULTRASONIC=true

# --- Parse CLI args ---
for arg in "$@"; do
  case "$arg" in
    --no-camera)     USE_CAMERA=false     ;;
    --no-ultrasonic) USE_ULTRASONIC=false ;;
    *) echo "Unknown argument: $arg"; exit 1 ;;
  esac
done
# Env var override (lower priority than CLI)
[ "${USE_CAMERA_ENV:-}" != "" ]     && USE_CAMERA=$USE_CAMERA_ENV
[ "${USE_ULTRASONIC_ENV:-}" != "" ] && USE_ULTRASONIC=$USE_ULTRASONIC_ENV

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Sourcing workspace ==="
source /opt/ros/noetic/setup.bash
source "${SCRIPT_DIR}/devel_isolated/setup.bash"

echo "=== Activating conda env: control ==="
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate control

LIDAR_ARGS="use_camera:=${USE_CAMERA} use_ultrasonic:=${USE_ULTRASONIC}"

echo "=== [1/3] Launching LiDAR (camera=$USE_CAMERA, ultrasonic=$USE_ULTRASONIC) ==="
roslaunch lidar_ros lidar_full.launch $LIDAR_ARGS &
LIDAR_PID=$!
sleep 3


# echo "=== [2/3] Launching cartographer localization ==="
# roslaunch cartographer_ros livox_2d.launch &
# CARTO_PID=$!
# sleep 3

# echo "=== [2/3] Launching cartographer localization ==="
# roslaunch cartographer_ros livox_2d_localization.launch &
# CARTO_PID=$!
# sleep 3

echo "=== [3/3] Launching NavOA (nav-only) ==="
roslaunch NavOA_ros navoa_nav_only.launch &
NAVOA_PID=$!

echo ""
echo "=== Full stack started ==="
echo "  LiDAR:       PID $LIDAR_PID       (camera=$USE_CAMERA, ultrasonic=$USE_ULTRASONIC)"
echo "  Cartographer: PID $CARTO_PID"
echo "  NavOA:       PID $NAVOA_PID"
echo ""
echo "Kill all: kill $LIDAR_PID $CARTO_PID $NAVOA_PID"

# Wait for any process to exit
wait
