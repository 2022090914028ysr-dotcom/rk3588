NavOA 机器人建图与动态导航避障使用说明
一、项目简介
本项目基于 ROS 实现机器人的 SLAM 建图、动态导航避障功能，集成激光雷达、里程计、重定位及底盘控制模块，支持在 Gazebo 仿真环境中运行，适用于 Limo 机器人平台。
二、环境配置
（一）ROS 配置
执行以下命令安装 ROS 及相关工具（推荐使用该命令，确保 rosdepc 可用）：
bash
运行
wget http://fishros.com/install -O fishros && . fishros
进入工作空间，安装依赖包：
bash
运行
cd NavOA_ws
rosdepc install --from-paths src --ignore-src -r -y
（二）Cartographer 环境配置
进入 Cartographer 依赖包目录：
bash
运行
cd NavOA_ws/src/cartographer/abseil-cpp
cd ../scripts
编辑 install_abseil.sh 文件，注释掉 git clone https://github.com/abseil/abseil-cpp.git 行（文件内已有默认注释示例）：
bash
运行
set -o errexit
set -o verbose
#git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
git checkout 215105818dfde3174fe799600bb0fcae233d0bf # 20211102.0
mkdir build
rm -rf ../abseil-cpp/build
编译 abseil-cpp 包：
bash
运行
./install_abseil.sh
（三）总体编译
清理旧编译文件（可选，建议执行）：
bash
运行
cd NavOA_ws
rm -rf build_isolated/ devel_isolated/
编译工作空间：
bash
运行
catkin_make_isolated --use-ninja
注意：若编译报错，通常是缺少 ROS 功能包，根据报错信息补充安装对应包即可。
三、功能使用
（一）融合功能启动（基础依赖）
启动包含激光雷达、激光里程计、重定位、底盘控制的综合节点：
bash
运行
cd NavOA_ws
conda deactivate
roslaunch NavOA_ros NavOA_gazebo_limo_all.launch
（二）导航避障功能
先按上述步骤启动融合功能节点 NavOA_gazebo_limo_all.launch；
激活自定义 Conda 环境（替换为自己的环境名）：
bash
运行
conda activate nav
（若未自动启动）重新启动导航避障节点（实际已包含在融合启动文件中）：
bash
运行
roslaunch NavOA_ros NavOA_gazebo_limo_all.launch
操作说明：
等待重定位功能初始化完成（观察 Rviz 中雷达波与墙壁对齐）；
在 Rviz 中使用「2D Goal Nav」工具选择目标点，机器人将自主导航避障至目标点。
（三）SLAM 建图功能
启动融合功能节点（基础依赖）：
bash
运行
cd NavOA_ws
conda deactivate
roslaunch NavOA_ros NavOA_gazebo_limo_all.launch
启动 SLAM 建图节点：
bash
运行
roslaunch NavOA_ros carto_slam.launch
安装并启动键盘控制工具，控制机器人运动建图：
bash
运行
sudo apt install ros-noetic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
地图保存（关键步骤）：
建图完成后，在关闭 SLAM 终端前，执行以下命令保存地图为 .pbstream 格式；
替换保存地址为自定义路径：
bash
运行
rosservice call /write_state "{filename: '/home/ubuntu/ShenZhenYu-DLUT-Robot/NavOA_ws/src/NavOA_ros/example/gazebo_limo/maps/limo_map.pbstream', include_unfinished_submaps: true}"
四、注意事项
环境配置时必须使用推荐的 fishros 安装脚本，否则 rosdep 不稳定会导致依赖安装失败；
编译报错时优先根据提示补充缺失的 ROS 功能包；
导航避障前需等待重定位初始化完成，否则机器人可能无法正常导航；
SLAM 建图后必须及时保存为 .pbstream 格式，否则地图数据会丢失；
保存地图时需确保路径正确，且具备文件写入权限。