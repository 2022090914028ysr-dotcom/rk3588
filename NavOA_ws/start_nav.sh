#!/bin/bash
cd ~/Robit_SlamNav/NavOA_ws
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh   # conda 初始化
conda activate control
source devel_isolated/setup.bash
roslaunch NavOA_ros navoa_nav_only.launch