#!/bin/bash
set -euo pipefail

NAV_BASE="/data/app/nav"
CONDA_BASE="${NAV_BASE}/conda"
if [ -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]; then
  source "${CONDA_BASE}/etc/profile.d/conda.sh"
  conda activate "${CONDA_BASE}" || true
fi

export PATH="${CONDA_BASE}/bin:${PATH}"
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

set +u
source /opt/ros/noetic/setup.bash
source "${NAV_BASE}/setup.bash"
set -u

# Ensure ROS shared libraries are on the linker path (libtf.so, libceres.so, etc.)
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"

echo "Starting Robit Navigation Stack..."
roslaunch --wait rosbridge_server rosbridge_websocket.launch port:=9090 &
roslaunch --wait lidar_ros lidar_full.launch use_camera:=false use_ultrasonic:=true &
roslaunch --wait cartographer_ros livox_2d_localization.launch &
roslaunch --wait NavOA_ros navoa_nav_only.launch &
rosrun NavOA_ros status_state.py &
wait || true
