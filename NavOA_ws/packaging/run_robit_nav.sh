#!/bin/bash
set -euo pipefail

CONDA_BASE="/home/ubuntu/miniconda3"
if [ -f "${CONDA_BASE}/etc/profile.d/conda.sh" ]; then
  source "${CONDA_BASE}/etc/profile.d/conda.sh"
  conda activate control || true
fi

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

set +u
source /opt/ros/noetic/setup.bash
source /opt/robit/nav/setup.bash
set -u

echo "Starting Robit Navigation Stack..."
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 &
roslaunch cartographer_ros livox_2d_localization.launch &
roslaunch NavOA_ros navoa_nav_only.launch &
wait || true
