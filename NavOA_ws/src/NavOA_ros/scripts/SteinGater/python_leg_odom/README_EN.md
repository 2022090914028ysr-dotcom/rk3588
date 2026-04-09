# Python Leg Odometry

A Python implementation of MIT Kalman Filter method for bipedal robot leg odometry.

## Overview

This project is a Python port of the C++ source code [CJW_KalmanFilter.cpp](../Wheel-legged-robot-body-estimation-main/src/CJW_KalmanFilter.cpp), implementing Kalman Filter estimation for wheel-legged robot body position and velocity.

## Features

- **Pure Leg Bipedal Robot Odometry**: Suitable for bipedal/quadruped legged robots
- **Extended Kalman Filter (EKF)**: Supports two filtering methods:
  - `EKF_IL`: MIT IMU + leg odometry fusion
  - `EKF_EIL`: MIT + external position observation fusion
- **Multi-Sensor Fusion**: Fuses IMU acceleration, leg odometry, external positioning, etc.
- **Stance Leg Detection**: Automatically identifies stance legs and swing legs
- **Slip Detection**: Stance leg slip threshold detection

## Installation

```bash
pip install numpy
```

## Quick Start

```python
import numpy as np
from leg_odom import create_default_odometry

# Create odometry instance (bipedal robot, 5ms sampling period)
odom = create_default_odometry(num_legs=2, dt=0.005)

# Initialize state
body_p = np.array([0.0, 0.0, 0.5])  # Body position [x, y, z]
body_v = np.array([0.0, 0.0, 0.0])  # Body velocity [vx, vy, vz]
foot_p = np.array([                 # Foot positions (3 x num_legs)
    [0.2, -0.2],
    [0.0,  0.0],
    [0.0,  0.0]
])

odom.initialize_state(body_p, body_v, foot_p)

# Update data every cycle
imu_acc = np.array([0.0, 0.0, -9.8])           # IMU acceleration
foot_delta = np.array([                        # Foot delta positions
    [0.01, -0.01],
    [0.0,   0.0],
    [0.0,   0.0]
])
foot_vel = np.array([                          # Foot velocities
    [0.1, -0.1],
    [0.0,  0.0],
    [0.0,  0.0]
])
contact = np.array([1.0, 1.0])                 # Contact flags (1=stance, 0=swing)

odom.update(imu_acc, foot_delta, foot_vel, contact)

# Get estimation results
body_pos, body_vel, foot_pos = odom.get_estimate()
```

## Class Documentation

### CJW_Leg_Body_KF

Core Kalman filter class.

**Constructor Parameters:**
- `branchn`: Number of robot legs (default: 2)

**Main Methods:**

| Method | Description |
|--------|-------------|
| `set_type(kf_type)` | Set filter type (EKF_IL or EKF_EIL) |
| `initial(dt)` | Initialize filter |
| `set_para(params)` | Set noise parameters |
| `set_body_leg_state()` | Set initial state |
| `set_imu_acc()` | Set IMU acceleration |
| `set_leg_obs()` | Set leg observation data |
| `set_ext_obs()` | Set external observation data |
| `estimate_once()` | Execute one filter iteration |
| `get_body_leg_state()` | Get estimation results |

### LegOdometry

Wrapped leg odometry class with friendly interface.

**Constructor Parameters:**
- `num_legs`: Number of legs (default: 2)
- `dt`: Sampling period in seconds (default: 0.005)
- `filter_type`: Filter type (default: EKF_IL)

## Noise Parameters

Set 8 noise parameters via `set_noise_parameters()`:

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| kpb | Prediction position noise | 0.1 |
| kvb | Prediction velocity noise | 0.1 |
| kpf | Prediction foot position noise | 0.1 |
| kpbf | Observation leg position noise | 0.1 |
| kpfv | Observation leg velocity noise | 0.1 |
| kfz | Observation leg height noise | 0.1 |
| kpex | Observation external position noise | 0.1 |
| kvex | Observation external velocity noise | 0.1 |

**Tuning Tips:**
- Increase kpb/kvb: State prediction more reliant on IMU
- Increase kpbf/kpfv: Observation data more reliable
- When using external positioning, decrease kpex/kvex

## Data Format

### Input Data

**IMU Acceleration** (`imu_acc`): shape (3,)
```python
np.array([ax, ay, az])  # Unit: m/s²
```

**Foot Delta Positions** (`foot_delta`): shape (3, num_legs)
```python
np.array([
    [leg1_x, leg2_x],  # X delta
    [leg1_y, leg2_y],  # Y delta
    [leg1_z, leg2_z]   # Z delta
])
```

**Foot Velocities** (`foot_vel`): shape (3, num_legs)
```python
np.array([
    [leg1_vx, leg2_vx],
    [leg1_vy, leg2_vy],
    [leg1_vz, leg2_vz]
])
```

**Contact Flags** (`contact`): shape (num_legs,)
```python
np.array([1.0, 1.0])  # 1.0=stance leg, 0.0=swing leg
```

### Output Data

```python
body_pos, body_vel, foot_pos = odom.get_estimate()

# body_pos: Body position [x, y, z]
# body_vel: Body velocity [vx, vy, vz]
# foot_pos: Foot positions shape (3, num_legs)
```

## Usage Examples

See [example.py](example.py) for more usage examples:

```bash
python example.py
```

### Example 1: Basic Usage
```python
from leg_odom import create_default_odometry
odom = create_default_odometry()
# Initialize and run filter
```

### Example 2: External Observation Fusion
```python
from leg_odom import LegOdometry, EKF_EIL
odom = LegOdometry(filter_type=EKF_EIL)
odom.update(imu_acc, foot_delta, foot_vel, contact, ext_pos, ext_vel)
```

### Example 3: Swing Leg Detection
```python
# Stance phase: contact = [1.0, 1.0]
# Swing phase: contact = [1.0, 0.0]
odom.update(imu_acc, foot_delta, foot_vel, contact)
```

### Example 4: Custom Noise Parameters
```python
odom.set_noise_parameters(kpb=0.5, kvb=0.8, kpf=0.3, ...)
```

### Example 5: Reset Filter
```python
odom.reset(new_body_p, new_body_v, new_foot_p)
```

## Project Structure

```
python_leg_odom/
├── leg_odom.py    # Core implementation (CJW_Leg_Body_KF and LegOdometry classes)
├── example.py     # Usage examples
└── README.md      # This file
```

## Reference

Original C++ Implementation: [CJW_KalmanFilter.cpp](../Wheel-legged-robot-body-estimation-main/src/CJW_KalmanFilter.cpp)