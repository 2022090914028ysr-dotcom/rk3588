# Robit Navigation 导航自启包

导航栈开机自启包，支持 cartographer 2D 定位 + NavOA 导航控制 + rosbridge websocket。
自带完整 conda 运行时环境（Python 3.10, PyTorch, NavOA 依赖），无需联网下载。

## 目录说明

- `/data/app/nav/run_robit_nav.sh`  启动脚本（systemd 服务入口）
- `/data/app/nav/setup.bash`        ROS 环境初始化脚本
- `/data/app/nav/conda/`            Conda 运行时环境（安装时自动解压）
- `/etc/systemd/system/robit-nav.service`  systemd 服务单元

## 系统依赖

目标机器人必须先安装 ROS Noetic 及以下依赖包：

```bash
# 1. 安装 ROS Noetic（如已安装可跳过）
sudo apt install ros-noetic-ros-core

# 2. 安装 rosbridge（本包不包含，必须额外安装）
sudo apt install ros-noetic-rosbridge-server

# 3. 安装系统库依赖（部分自动被 ROS 引入，显式确保）
sudo apt install \
  libboost-iostreams1.71.0 \
  libcairo2 \
  libgoogle-glog0v5 \
  libgflags2.2 \
  liblua5.3-0 \
  libprotobuf17 \
  libpcl-1.10 \
  bzip2

# 4. 初始化 rosdep（首次使用 ROS 时）
sudo rosdep init && rosdep update
```

> **注意**：本包已内置 `cartographer_ros`、`NavOA_ros`、`rf2o_laser_odometry` 等 ROS 包，
> **无需**通过 apt 额外安装。只有 `rosbridge_server` 需系统级安装。

## 安装方法

```bash
# 安装（安装后自动启用开机自启，首次安装需约 1 分钟解压 conda 环境）
sudo dpkg -i robit_nav_1.0.0_arm64.deb

# 或从源码构建
cd /home/ubuntu/Nav_Release/Robit_SlamNav/NavOA_ws
./build_release.sh

sudo dpkg -i robit_nav_1.0.0_arm64.deb
```

## 使用方法

```bash
# 查看服务状态
systemctl status robit-nav.service --no-pager -l

# 启动/停止/重启
sudo systemctl start robit-nav.service
sudo systemctl stop robit-nav.service
sudo systemctl restart robit-nav.service

# 开机自启开关
sudo systemctl enable robit-nav.service
sudo systemctl disable robit-nav.service
```

## 启动内容

`run_robit_nav.sh` 启动顺序：

1. 激活 conda 环境（已从打包的 conda.tar.gz 解压至 `/data/app/nav/conda/`）
2. `source /opt/ros/noetic/setup.bash`
3. `source /data/app/nav/setup.bash`
4. 启动 `rosbridge_server rosbridge_websocket.launch port:=9090`
5. 启动 `cartographer_ros livox_2d_localization.launch`
6. 启动 `NavOA_ros navoa_nav_only.launch`
7. 状态 `rosrun NavOA_ros status_state.py`

## 日志查看

```bash
# 最近 100 行日志
journalctl -u robit-nav.service -n 100 --no-pager

# 实时跟踪日志
journalctl -u robit-nav.service -f

# ROS 日志目录
ls -al /root/.ros/log
```

## 常见问题

- **ROS_DISTRO: unbound variable**  
  原因：`set -u` 下直接 source ROS 环境  
  处理：启动脚本中已自动 `set +u` 后再 source

- **Resource not found: carto_ws**  
  原因：map 文件路径问题  
  处理：launch 文件使用 `$(find NavOA_ros)/../carto_ws/map/...` 绝对路径

- **服务启动后立即退出**  
  原因：roslaunch 子节点异常退出导致 `wait` 返回非零  
  处理：脚本使用 `wait || true` 保证父进程存活


- **安装时间长 / 包体较大（507MB）**  
  原因：包内集成了完整的 conda 环境（PyTorch, cvxpy 等），安装时需解压约 2.2GB 到 `/data/app/nav/conda/`  
  处理：首次安装耐心等待，`postinst` 完成后会自动清理压缩包释放空间
