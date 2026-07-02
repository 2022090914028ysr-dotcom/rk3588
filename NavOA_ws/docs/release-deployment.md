# Robit Navigation 导航自启包

导航栈开机自启包，支持 cartographer 2D 定位 + NavOA 导航控制 + rosbridge websocket。
自带完整 conda 运行时环境（Python 3.10, PyTorch, NavOA 依赖），无需联网下载。

## 目录说明

- `/data/app/nav/run_robit_nav.sh`  启动脚本（systemd 服务入口）
- `/data/app/nav/setup.bash`        ROS 环境初始化脚本
- `/data/app/nav/conda/`            Conda 运行时环境（安装时自动解压）
- `/etc/systemd/system/robit-nav.service`  systemd 服务单元

## 系统依赖

目标机器人必须安装 ROS Noetic 全平台及以下依赖。本包 `Depends` 已声明全部依赖，
`sudo dpkg -i` 时 apt 会自动提示缺失项，运行 `sudo apt --fix-broken install` 一键补齐。

### ROS 包依赖

| 包名 | apt 包 | 用途 |
|---|---|---|
| ros-core | `ros-noetic-ros-core` | ROS 主框架（roscpp, rospy, std_msgs 等） |
| rosbridge-server | `ros-noetic-rosbridge-server` | WebSocket 通信桥（9090 端口） |
| move-base | `ros-noetic-move-base` | 全局 / 局部路径规划执行器 |
| map-server | `ros-noetic-map-server` | 地图发布（navoa_nav_only.launch 使用） |
| tf | `ros-noetic-tf` | 坐标变换（static_transform_publisher, Python `import tf`） |
| tf2 | `ros-noetic-tf2` | TF2 核心库 |
| tf2-ros | `ros-noetic-tf2-ros` | TF2 ROS 绑定（Python `import tf2_ros`） |
| tf2-geometry-msgs | `ros-noetic-tf2-geometry-msgs` | TF2 几何消息转换 |
| actionlib-msgs | `ros-noetic-actionlib-msgs` | 动作状态消息（status_state.py 使用） |
| visualization-msgs | `ros-noetic-visualization-msgs` | Marker/MarkerArray 可视化消息 |
| dynamic-reconfigure | `ros-noetic-dynamic-reconfigure` | 动态参数配置（Python 客户端） |
| amcl | `ros-noetic-amcl` | 自适应蒙特卡洛定位（可选，slam_toolbox 方案用） |
| slam-toolbox | `ros-noetic-slam-toolbox` | 在线 / 离线 SLAM（可选，备选建图方案） |
| global-planner | `ros-noetic-global-planner` | 全局路径规划器（navoa_nav_only.launch 默认使用） |
| nav-core | `ros-noetic-nav-core` | move_base 核心接口（自动依赖） |
| costmap-2d | `ros-noetic-costmap-2d` | 全局 / 局部代价地图（自动依赖） |
| base-local-planner | `ros-noetic-base-local-planner` | 局部路径规划器（自动依赖） |

### 系统库依赖

| 库文件 | apt 包 | 被谁使用 |
|---|---|---|
| `libceres.so.1` | `libceres-dev` | cartographer_node 非线性优化 |
| `libglog.so.0` | `libgoogle-glog0v5` | cartographer/glog 日志 |
| `libgflags.so.2.2` | `libgflags2.2` | cartographer 命令行标志 |
| `liblua5.3.so.0` | `liblua5.3-0` | cartographer Lua 配置解析 |
| `libcairo.so.2` | `libcairo2` | 栅格地图渲染 |
| — | `bzip2` | 首次安装解压 conda 环境 |

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


- **旧版本更新：卸载之前安装的 robit-nav**  
  如果之前安装过旧版本（`/opt/robit/nav/`），先卸载再装新版：
  ```bash
  sudo systemctl stop robit-nav.service
  sudo systemctl disable robit-nav.service
  sudo dpkg -r robit-nav
  sudo rm -rf /opt/robit/nav   # 清理残留
  sudo dpkg -i robit_nav_1.0.0_arm64.deb
  ```

- **`error while loading shared libraries: libceres.so.1 / libglog.so.0`**  
  原因：系统库未安装。`.deb` 的 `Depends` 已声明这些依赖，但 `dpkg -i` **不会自动安装**它们。  
  处理：运行 `sudo apt --fix-broken install` 一键补齐所有缺失的系统依赖，然后重启服务：
  ```bash
  sudo apt --fix-broken install
  sudo systemctl restart robit-nav.service
  ```
  > 首次在新机器人部署时推荐先执行 `sudo apt update && sudo apt --fix-broken install`，
  > 或在安装 `.deb` 之前手动安装：
  > `libceres-dev libgoogle-glog0v5 libgflags2.2 liblua5.3-0 libcairo2 bzip2`

- **`No module named 'torch'` / conda Python 包找不到**  
  原因：`NavOA_node.py` 运行时使用了系统 Python 而非 conda 环境的 Python。  
  处理：检查 conda 环境是否正确解压：
  ```bash
  ls /data/app/nav/conda/lib/python3.10/site-packages/torch/
  /data/app/nav/conda/bin/python3 -c "import torch; print(torch.__version__)"
  ```
  如果 conda 环境损坏，手动清理后重新安装：
  ```bash
  sudo rm -rf /data/app/nav/conda
  sudo dpkg --configure -a
  ```

- **`Failed to create the global_planner/GlobalPlanner planner`**  
  原因：`navoa_nav_only.launch` 默认使用 `global_planner/GlobalPlanner`，但 `ros-noetic-global-planner` 未安装。  
  处理：安装缺失的 ROS 包：
  ```bash
  sudo apt install ros-noetic-global-planner
  sudo systemctl restart robit-nav.service
  ```

- **安装时间长 / 包体较大（507MB）**  
  原因：包内集成了完整的 conda 环境（PyTorch, cvxpy 等），安装时需解压约 2.2GB 到 `/data/app/nav/conda/`  
  处理：首次安装耐心等待，`postinst` 完成后会自动清理压缩包释放空间
