# marl_mapping - 自主建图模块（单机器人，真机）

基于 NeuralCoMapping (CVPR 2022) 多机器人主动建图算法，适配为单机器人真机自主探索建图模块。

## 目录结构

```
marl_mapping/
├── CMakeLists.txt          # CMake 构建文件
├── package.xml             # ROS 包描述
├── setup.py                # Python 包安装
├── launch/                 # 启动文件
│   ├── exploration.launch  # 自主探索启动
│   └── nav_exploration.launch  # 导航+探索联合启动
├── scripts/                # ROS 节点脚本
│   ├── exploration_node.py # 主探索节点
│   └── robot_interface.py  # 机器人接口测试
├── src/marl_mapping/       # Python 包
│   ├── arguments.py        # 参数配置
│   ├── env/                # 环境接口
│   │   ├── robot_interface.py   # 单机器人 ROS 接口
│   │   └── exploration_env.py   # 真机探索环境
│   └── utils/              # 工具函数
│       ├── map_builder.py       # 地图构建
│       ├── fmm_planner.py       # FMM 路径规划
│       └── frontier.py          # 前沿检测与目标选择
├── models/                 # RL 模型存放
└── config/                 # 配置文件
```

## 功能

- **前沿检测**：基于占据栅格地图检测探索前沿
- **目标选择**：支持 nearest / largest / combined 三种策略
- **路径规划**：基于 FMM (Fast Marching Method) 的路径规划
- **自主导航**：简单的比例控制器驱动机器人
- **地图保存**：支持 ROS map_server 兼容格式
- **可视化**：Rviz 中显示前沿点和目标点

## 依赖

- ROS (rospy, std_msgs, geometry_msgs, nav_msgs, sensor_msgs, tf, cv_bridge)
- numpy, opencv-python, scipy
- scikit-fmm (可选，用于 FMM 路径规划)
- scikit-learn (可选，用于前沿聚类)

## 使用

### 1. 单独启动自主探索

```bash
roslaunch marl_mapping exploration.launch robot_name:=car1
```

### 2. 与 NavOA 导航联合启动

```bash
roslaunch marl_mapping nav_exploration.launch robot_name:=car1 enable_exploration:=true
```

### 3. 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| robot_name | car1 | 机器人 ROS 命名空间 |
| map_size_cm | 2500 | 地图尺寸 (cm) |
| map_resolution | 5 | 地图分辨率 (cm/pixel) |
| max_steps | 500 | 最大探索步数 |
| linear_speed | 0.20 | 线速度 (m/s) |
| angular_speed | 0.60 | 角速度 (rad/s) |
| exploration_method | nearest | 目标选择策略 (nearest/largest/combined) |
| min_frontier_size | 5 | 最小前沿簇大小 |

## 与原 Shenzhen 项目的关系

本模块从 `深圳项目-殷雪峰真机代码/marl_NeuralCoMapping-v4_RL2-Real_World_shenzhen` 中提取并适配：

- **保留**：前沿检测、FMM 路径规划、Voronoi 划分算法
- **简化**：多机器人 → 单机器人
- **移除**：Gibson/iGibson 仿真环境、多场景支持、RL 训练代码
- **新增**：ROS 标准启动文件、NavOA 集成接口、地图保存
