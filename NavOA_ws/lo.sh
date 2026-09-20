#!/bin/bash
cd ~/Robit_SlamNav/NavOA_ws
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh   # conda 初始化
conda activate control
source devel_isolated/setup.bash
roslaunch cartographer_ros livox_2d_localization.launch