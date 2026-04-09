# Python Leg Odometry

基于MIT卡尔曼滤波方法的双足机器人腿式里程计Python实现。

## 简介

本项目是对C++源代码 [CJW_KalmanFilter.cpp](../Wheel-legged-robot-body-estimation-main/src/CJW_KalmanFilter.cpp) 的Python移植版本，实现了轮腿机器人车身位置与速度的卡尔曼滤波估计。

## 功能特性

- **纯腿式双足机器人里程计**: 适用于双足/四足等腿式机器人
- **扩展卡尔曼滤波(EKF)**: 支持两种滤波方法:
  - `EKF_IL`: MIT IMU + 腿式里程计融合
  - `EKF_EIL`: MIT + 外部位置观测融合
- **多传感器融合**: 融合IMU加速度、腿式里程计、外部定位等数据
- **支撑腿检测**: 自动识别支撑腿与摆动腿
- **滑动检测**: 支撑腿滑动阈值检测

## 安装依赖

```bash
pip install numpy
```

## 快速开始

```python
import numpy as np
from leg_odom import create_default_odometry

# 创建里程计实例 (双足机器人, 5ms采样周期)
odom = create_default_odometry(num_legs=2, dt=0.005)

# 初始化状态
body_p = np.array([0.0, 0.0, 0.5])  # 机身位置 [x, y, z]
body_v = np.array([0.0, 0.0, 0.0])  # 机身速度 [vx, vy, vz]
foot_p = np.array([                 # 足部位置 (3 x num_legs)
    [0.2, -0.2],
    [0.0,  0.0],
    [0.0,  0.0]
])

odom.initialize_state(body_p, body_v, foot_p)

# 每周期更新数据
imu_acc = np.array([0.0, 0.0, -9.8])           # IMU加速度
foot_delta = np.array([                        # 足部增量位置
    [0.01, -0.01],
    [0.0,   0.0],
    [0.0,   0.0]
])
foot_vel = np.array([                          # 足部速度
    [0.1, -0.1],
    [0.0,  0.0],
    [0.0,  0.0]
])
contact = np.array([1.0, 1.0])                 # 接触标志 (1=支撑, 0=摆动)

odom.update(imu_acc, foot_delta, foot_vel, contact)

# 获取估计结果
body_pos, body_vel, foot_pos = odom.get_estimate()
```

## 类说明

### CJW_Leg_Body_KF

卡尔曼滤波核心类。

**构造函数参数:**
- `branchn`: 机器人的腿数量 (默认: 2)

**主要方法:**

| 方法 | 说明 |
|------|------|
| `set_type(kf_type)` | 设置滤波类型 (EKF_IL 或 EKF_EIL) |
| `initial(dt)` | 初始化滤波器 |
| `set_para(params)` | 设置噪声参数 |
| `set_body_leg_state()` | 设置初始状态 |
| `set_imu_acc()` | 设置IMU加速度 |
| `set_leg_obs()` | 设置腿部观测数据 |
| `set_ext_obs()` | 设置外部观测数据 |
| `estimate_once()` | 执行一次滤波 |
| `get_body_leg_state()` | 获取估计结果 |

### LegOdometry

封装好的腿式里程计类，提供更友好的接口。

**构造函数参数:**
- `num_legs`: 腿的数量 (默认: 2)
- `dt`: 采样周期，单位秒 (默认: 0.005)
- `filter_type`: 滤波类型 (默认: EKF_IL)

## 噪声参数说明

通过 `set_noise_parameters()` 设置8个噪声参数：

| 参数 | 含义 | 典型值 |
|------|------|--------|
| kpb | 预测位置噪声 | 0.1 |
| kvb | 预测速度噪声 | 0.1 |
| kpf | 预测足部位置噪声 | 0.1 |
| kpbf | 观测腿部位置噪声 | 0.1 |
| kpfv | 观测腿部速度噪声 | 0.1 |
| kfz | 观测腿部高度噪声 | 0.1 |
| kpex | 观测外部位置噪声 | 0.1 |
| kvex | 观测外部速度噪声 | 0.1 |

**调参建议:**
- 增加 kpb/kvb: 状态预测更依赖IMU
- 增加 kpbf/kpfv: 观测数据更可靠
- 使用外部定位时，降低 kpex/kvex

## 数据格式

### 输入数据

**IMU加速度** (`imu_acc`): shape (3,)
```python
np.array([ax, ay, az])  # 单位: m/s²
```

**足部增量位置** (`foot_delta`): shape (3, num_legs)
```python
np.array([
    [leg1_x, leg2_x],  # X方向增量
    [leg1_y, leg2_y],  # Y方向增量
    [leg1_z, leg2_z]   # Z方向增量
])
```

**足部速度** (`foot_vel`): shape (3, num_legs)
```python
np.array([
    [leg1_vx, leg2_vx],
    [leg1_vy, leg2_vy],
    [leg1_vz, leg2_vz]
])
```

**接触标志** (`contact`): shape (num_legs,)
```python
np.array([1.0, 1.0])  # 1.0=支撑腿, 0.0=摆动腿
```

### 输出数据

```python
body_pos, body_vel, foot_pos = odom.get_estimate()

# body_pos: 机身位置 [x, y, z]
# body_vel: 机身速度 [vx, vy, vz]
# foot_pos: 足部位置 shape (3, num_legs)
```

## 使用示例

查看 [example.py](example.py) 获取更多使用示例：

```bash
python example.py
```

### 示例1: 基础用法
```python
from leg_odom import create_default_odometry
odom = create_default_odometry()
# 初始化并运行滤波
```

### 示例2: 外部观测融合
```python
from leg_odom import LegOdometry, EKF_EIL
odom = LegOdometry(filter_type=EKF_EIL)
odom.update(imu_acc, foot_delta, foot_vel, contact, ext_pos, ext_vel)
```

### 示例3: 摆动腿检测
```python
# 支撑相: contact = [1.0, 1.0]
# 摆动相: contact = [1.0, 0.0]
odom.update(imu_acc, foot_delta, foot_vel, contact)
```

### 示例4: 自定义噪声参数
```python
odom.set_noise_parameters(kpb=0.5, kvb=0.8, kpf=0.3, ...)
```

### 示例5: 重置滤波器
```python
odom.reset(new_body_p, new_body_v, new_foot_p)
```

## 项目结构

```
python_leg_odom/
├── leg_odom.py    # 核心实现 (CJW_Leg_Body_KF 和 LegOdometry 类)
├── example.py     # 使用示例
└── README.md      # 本文档
```

## 参考

原始C++实现: [CJW_KalmanFilter.cpp](../Wheel-legged-robot-body-estimation-main/src/CJW_KalmanFilter.cpp)