#!/bin/bash
# Start the full navigation stack: LiDAR + cartographer localization + NavOA.
# Sequential launch ensures TF topics are established before each dependent node.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Sourcing workspace ==="
source /opt/ros/noetic/setup.bash
source "${SCRIPT_DIR}/devel_isolated/setup.bash"

echo "=== Activating conda env: control ==="
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate control

# ---- 1. LiDAR driver ----
echo "=== [1/3] Launching LiDAR ==="
roslaunch marl_mapping exploration.launch &
LIDAR_PID=$!
sleep 3

# # ---- 2. Cartographer localization ----
# echo "=== [2/3] Launching cartographer localization ==="
# roslaunch cartographer_ros livox_2d_localization.launch &
# CARTO_PID=$!
# sleep 4

# ---- 3. NavOA nav stack ----
echo "=== [3/3] Launching NavOA (nav-only) ==="
roslaunch NavOA_ros navoa_nav_only.launch &
NAVOA_PID=$!

echo ""
echo "=== Full stack started ==="
echo "  LiDAR PID:       $LIDAR_PID"
echo "  Cartographer PID: $CARTO_PID"
echo "  NavOA PID:        $NAVOA_PID"
echo ""
echo "Kill all: kill $LIDAR_PID $CARTO_PID $NAVOA_PID"

# Wait for any process to exit
wait
