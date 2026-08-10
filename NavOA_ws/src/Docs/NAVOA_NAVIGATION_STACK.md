# NavOA Navigation Stack

> **Date:** 2026-07-24  
> **Package:** `NavOA_ros`  
> **Location:** `/home/ubuntu/Robit_SlamNav/NavOA_ws/src/NavOA_ros/`

---

## 1. Overview

NavOA (Neural Network Optimized Avoidance) is a complete navigation stack for differential-drive robots. It replaces the standard ROS `base_local_planner` with a **deep-learning-driven local planner** using Proximal Alternating Minimization (PAN) networks, while still leveraging `move_base` for global path planning and costmap management.

**Architecture: NavOA as a move_base plugin alternative**

```
move_base (global planner only)
     │
     │ GlobalPlanner/plan (Path)
     │
     ├── path_segment_publisher → /split_goal (segmented waypoints)
     │
     └── waypoint_manager → /NavOA_goal (extracted waypoints)
                │
                ▼
         NavOA_core.py (50 Hz controller)
                │
                ▼
         vel_filter.py (safety layer)
                │
                ▼
         /cmd_vel (serial bridge → motors)
```

**Key design: NavOA runs separately from move_base.**
- `move_base` handles only **global planning** (NavFN) and costmap management
- NavOA_core.py handles **local planning** (DUNE/PAN neural networks) and **control**
- `vel_filter.py` provides a **safety filter** between NavOA and the motor driver

---

## 2. Package Structure

```
NavOA_ros/
├── cfg/
│   └── RefSpeed.cfg              ← Dynamic reconfigure for reference speed
├── launch/
│   ├── navoa_nav_only.launch     ← Main NavOA bringup (used by sensors.launch)
│   ├── local_avoidance_only.launch ← Pure obstacle avoidance (no map)
│   ├── data_logger.launch        ← Data logging
│   ├── localization_recovery.launch ← Localization failure detection
│   ├── slam_toolbox_navoa.launch ← SLAM Toolbox + NavOA
│   ├── hector_map_amcl.launch    ← Hector + AMCL + NavOA
│   ├── teleop_bridge.launch      ← Keyboard teleop
│   ├── velocity_subscriber.launch← Direct motor control
│   └── voice_nav.launch          ← Voice navigation
├── param/
│   ├── costmap_common_params.yaml   ← Robot footprint, inflation
│   ├── global_costmap_params.yaml   ← Static map layer
│   ├── local_costmap_params.yaml    ← Rolling window, obstacle layer
│   ├── base_local_planner_params.yaml ← TrajectoryPlannerROS (fallback)
│   ├── amcl_params.yaml             ← AMCL for SLAM Toolbox mode
│   └── localization_recovery_params.yaml ← Recovery thresholds
├── NavOA/
│   └── NavOA/
│       ├── NavOA.py              ← Main NavOA class (torch.nn.Module)
│       ├── robot/robot.py        ← Differential drive kinematic model
│       ├── blocks/
│       │   ├── pan.py            ← Proximal Alternating Minimization
│       │   ├── dune.py           ← Deep Unrolling Network for avoidance
│       │   ├── nrmp.py           ← Neural Risk Metric Predictor
│       │   ├── initial_path.py   ← Initial path generation
│       │   └── obs_point_net.py  ← Obstacle point processing
│       ├── configuration/        ← Device/tensor utilities
│       └── util/                 ← Transform, timing, file helpers
├── scripts/
│   ├── NavOA_node.py             ← ROS entry point → imports NavOA_core
│   ├── src/NavOA_core.py         ← Main control loop (50 Hz)
│   ├── vel_filter.py             ← Safety velocity filter
│   ├── vel_filter_debug_monitor.py
│   ├── serial_cmd_vel_imu_bridge.py ← Bidirectional serial bridge
│   ├── imu_controller.py         ← WIT IMU low-level driver
│   ├── imu_mux.py                ← IMU multiplexer
│   ├── imu_publisher.py          ← IMU ROS publisher
│   ├── localization_recovery.py  ← Localization failure detection
│   ├── map_odom_freezer.py       ← Freeze map→odom TF
│   ├── path_segment_publisher.py ← Segment global path into waypoints
│   ├── waypoint_manager.py       ← Extract waypoints from global plan
│   ├── navoa_waypoint_sequencer.py ← Sequential waypoint publisher
│   ├── goal_publisher_node.py    ← Goal publishing utility
│   ├── send_forward_goal.py      ← Goal at configurable distance
│   ├── voice_goal_router.py      ← Voice navigation (English/Chinese)
│   ├── external_cmd.py           ← Keyboard external control
│   ├── launch_control_cmd.py     ← Scripted movement commands
│   ├── map_control.py            ← Cartographer mapping API
│   ├── maplocation.py            ← GUI map + click-to-navigate
│   ├── explorer.py               ← Simple frontier explorer
│   ├── data_logger.py            ← CSV data logging
│   ├── error_collector.py        ← Error logging
│   └── velocity_subscriber.py    ← Direct cmd_vel → motors
├── maps/                         ← Additional map YAML files
└── example/gazebo_limo/          ← Gazebo simulation example
    ├── config/
    │   └── NavOA_planner_limo.yaml  ← NavOA planner configuration
    ├── launch/
    │   └── NavOA_gazebo_limo.launch ← Gazebo bringup
    ├── maps/                     ← Gazebo maps
    ├── pretrain_limo/            ← Pretrained DUNE model weights
    ├── rviz/                     ← RViz configs
    └── world/                    ← Gazebo worlds
```

---

## 3. Core Navigation Algorithm

### 3.1 NavOA Neural Network Architecture

```
NavOA (torch.nn.Module) — the main class
│
├── Obstacle Point Network (obs_point_net.py)
│   Processes raw laser scan into obstacle features
│
├── Initial Path Generator (initial_path.py)
│   Generates reference path (line or circle) to goal
│   Config: interval=0.09m, curve_style='line'
│
├── DUNE (Deep Unrolling Network for obstacle avoidance)
│   Iterative optimization-based path unrolling
│   Max iterations: 100
│
├── NRMP (Neural Risk Metric Predictor)
│   Predicts collision risk for candidate trajectories
│   Max candidates: 10
│
└── PAN (Proximal Alternating Minimization)
    Alternately optimizes state and control trajectories
    Iterations: 2
    Uses: DUNE output → NRMP scoring → best trajectory
```

### 3.2 NavOA Planner Configuration

```yaml
# From NavOA_planner_limo.yaml
model:
  mpc:
    T: 5           # Prediction horizon (time steps)
    dt: 0.3        # Time step (seconds)
  reference:
    ref_speed: 1.5   # Reference speed (m/s) — overridden at runtime
  robot:
    type: differential
    max_speed: [0.25, 0.5]    # [linear max, angular max]
    max_acce: [0.5, 1.0]      # [linear accel max, angular accel max]
  initial_path:
    interval: 0.09
    curve_style: line
  pan:
    iter_num: 2
    dune_max: 100
    nrmp_max: 10
  adjust:
    q_s: 0.35      # State cost weight
    p_u: 2.5       # Control cost weight
    eta: 15        # Collision cost weight
    d_max: 0.13    # Max safety distance
    d_min: 0.01    # Min safety distance
```

### 3.3 Control Loop (NavOA_core.py)

Run at **50 Hz**, the control loop does:

```
1. Look up robot pose from TF (map→base_link)
2. Get current goal from /NavOA_goal or /NavOA_waypoints
3. If no active goal, publish zero velocity
4. Get laser scan from /scan
5. Generate smooth initial path toward goal
6. Convert scan points to obstacle representation
7. Run NavOA.forward(state, points) → [v, w] action
8. If external control enabled, blend external_cmd_vel
9. Publish [v, w] to /NavOA_cmd_vel
10. Publish visualization markers (path, goal, predicted trajectory)
11. Check goal tolerance (position + orientation)
12. Handle goal timeout (return to previous goal)
13. Handle button-based stand/walk toggling
```

**Goal Tolerance:**
- Position: 0.15m (tight) / 0.5m (loose for mid-points)
- Orientation: 0.05 rad
- Return to previous goal timeout: configurable

---

## 4. Velocity Safety Filter (vel_filter.py)

### 4.1 Purpose

`vel_filter.py` is the **critical safety layer** between the NavOA planner and the physical motors. It subscribes to `/cmd_vel_tem` (NavOA output) and publishes to `/cmd_vel` (motor input). It provides multiple independent safety checks.

### 4.2 Filter Pipeline

```
Input: /cmd_vel_tem (vx, vy, wz)
          │
          ▼
┌──────────────────────────────────────────────────────────┐
│ 1. Forward Obstacle Detection (LIDAR)                    │
│    - Front center angle: 1.57 rad (90°)                  │
│    - Stop distance: 0.30-0.40m                           │
│    - Slow distance: 0.60m                                │
│    - Hysteresis release distance                         │
│    - Divides scan into front / left / right sectors      │
│    - Latches stop until obstacle clears hysteresis       │
├──────────────────────────────────────────────────────────┤
│ 2. Ultrasonic Virtual Bumper                             │
│    - Front blocked → reverse + latched side evasion      │
│    - Left blocked → strafe right (linear.y proportional) │
│    - Right blocked → strafe left                        │
│    - Both sides + front clear → narrow passage mode      │
│    - Reverse side detection with extended stop (0.35m)   │
│    - Hold lateral velocity after reverse evasion ends    │
├──────────────────────────────────────────────────────────┤
│ 3. Camscan Bias (Glass/Transparent Obstacles)            │
│    - When /camscan detects obstacle but /scan doesn't    │
│    - Adds rotation-following lateral bias                │
│    - Ultrasonic-influenced lateral bias                  │
├──────────────────────────────────────────────────────────┤
│ 4. Free-Space Reactive Steering                         │
│    - When forward blocked, steer toward larger clearance │
│    - Configurable gain, deadzone                         │
│    - Persistent turn search when blocked                 │
├──────────────────────────────────────────────────────────┤
│ 5. Auto-Reverse on Collision                             │
│    - Detects front collision (sudden stop)               │
│    - Reverses for configurable duration                  │
├──────────────────────────────────────────────────────────┤
│ 6. Deadbands & Speed Clamps                              │
│    - max_forward_speed (0.0 = disabled)                  │
│    - max_angular_speed (0.0 = disabled)                  │
│    - wz_scale (angular speed multiplier)                 │
│    - deadband_linear_x, deadband_angular_z               │
└──────────────────────────────────────────────────────────┘
          │
          ▼
Output: /cmd_vel (vx, vy, wz) → serial bridge → motors
```

### 4.3 Debug Topics

| Topic | Content | Frequency |
|-------|---------|-----------|
| `/vel_filter/debug` | Human-readable string with motion mode, trends, source labels | ~10 Hz |
| `/vel_filter/status` | Structured status string | ~10 Hz |

---

## 5. Serial Bridge (serial_cmd_vel_imu_bridge.py)

### 5.1 Design

Bidirectional bridge between the ROS computer and the motor controller over serial (`/dev/ttyS1`, 460800 baud).

```
┌───────────────────┐         Serial (ttyS1)         ┌─────────────────────┐
│   ROS Computer    │ ◄─────── 460800 baud ────────► │  Motor Controller   │
│                   │                                  │                     │
│  Subscribes from: │   "CMD_VEL,0.25,0.00,0.00\r\n"  │ Receives cmd_vel    │
│  /cmd_vel         │ ◄────────────────────────────── │ Drives motors       │
│                   │                                  │                     │
│  Publishes to:    │   "IMU,ax,ay,az,gx,gy,gz,..."   │ Reads IMU           │
│  /imu_serial      │ ──────────────────────────────► │ Sends CSV lines     │
│  /nav_serial/event│                                  │                     │
└───────────────────┘                                  └─────────────────────┘
```

### 5.2 Dual-Thread Architecture

**Writer Thread (30 Hz):**
- Subscribes to `/cmd_vel` and `/app_controller/button`
- Formats: `CMD_VEL,vx,vy,w\r\n` or `BUTTON,id,type\r\n`
- Throttled to configurable rate (default 30 Hz)
- TX queue with thread-safe push

**Reader Thread:**
- Reads lines from serial port
- Parses `IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw` → `sensor_msgs/Imu`
- Parses JSONL protocol events → `/nav_serial/event`
- Parses `BUTTON,id,type` → `/app_controller/button`
- Euler-to-quaternion conversion for orientation

### 5.3 Timestamp Management

- Can use device-provided timestamp (with configurable offset)
- Min dt filter rejects duplicates
- Max future distance filter rejects out-of-order timestamps

---

## 6. IMU Handling

Three-layer IMU architecture:

### 6.1 IMU Controller (Hardware Driver)

**File:** `scripts/imu_controller.py`

```
WIT IMU (ttyS6, 230400 baud)
  │
  │ 33-byte frame blocks:
  │   0x55 0x51 + accel data (11 bytes)
  │   0x55 0x52 + gyro data  (11 bytes)
  │   0x55 0x53 + angle data (11 bytes)
  │
  ▼
IMUController class:
  - Background reader thread (~500 Hz)
  - Frame extraction with auto-resync
  - Accelerometer: ±16g (int16)
  - Gyroscope: ±2000°/s (int16)
  - Euler angles: ±180° (int16)
  - Calibration: roll/pitch bias removal
  - Angle safety checking
  - Simulation mode fallback
  - Frequency tracking
```

### 6.2 IMU Publisher (ROS Node)

**File:** `scripts/imu_publisher.py`

- Wraps `IMUController`
- Publishes `sensor_msgs/Imu` on `/imu`
- Configurable publish rate
- Converts Euler angles → quaternion

### 6.3 IMU Multiplexer

**File:** `scripts/imu_mux.py`

```
/imu_serial (primary, from serial bridge) ──┐
                                            ├──► /imu_mux
/imu (fallback, from local IMU) ────────────┘

- Prefers primary as long as messages arrive within 0.2s
- Falls back to /imu if primary times out
- Publishes combined stream to /imu_mux
```

---

## 7. Waypoint Management

### 7.1 path_segment_publisher.py

Subscribes to `/move_base/GlobalPlanner/plan`:
- Samples path at configurable intervals (default 1.5m)
- Publishes segmented sub-goals to `/split_goal`
- Loose tolerance (0.5m) for mid-points
- Tight tolerance (0.15m) for final goal
- Resets on new RViz goal

### 7.2 waypoint_manager.py

Extracts waypoints from global plan:
- Angle-based sampling (>25° turn → new waypoint)
- Fixed-interval sampling (>0.6m spacing)
- Publishes to `/NavOA_goal`
- TF-based arrival monitoring

### 7.3 navoa_waypoint_sequencer.py

Alternative waypoint extraction:
- Turn angle threshold: >25°
- Minimum spacing: >0.6m
- Sequential publishing to `/move_base_simple/goal`
- Radius-based arrival detection

---

## 8. Configuration Files

### 8.1 Costmap Parameters

| File | Key Settings |
|------|-------------|
| `costmap_common_params.yaml` | Footprint: [[0.20,0.20],[0.20,-0.20],[-0.20,-0.20],[-0.20,0.20]] |
| `global_costmap_params.yaml` | frame=map, static layer + inflation, map_topic=/map |
| `local_costmap_params.yaml` | rolling window 6×6m, obstacle layer from /scan, inflation_radius=0.0 |
| `base_local_planner_params.yaml` | TrajectoryPlannerROS, max_vel_x=0.8, xy_tol=0.30, yaw_tol=0.05 |

### 8.2 Dynamic Reconfigure

**File:** `cfg/RefSpeed.cfg` → `/NavOA_control` server

| Parameter | Type | Range | Default |
|-----------|------|-------|---------|
| `ref_speed` | Double | 0.0 — 5.0 | 0.1 |

### 8.3 Localization Recovery Parameters

| Parameter | Default | Threshold For |
|-----------|---------|---------------|
| `position_jump_threshold` | 1.0 m | Odometry pose jump detection |
| `rotation_jump_threshold` | 1.0 rad | Odometry rotation jump detection |
| `accel_spike_threshold` | 15 m/s² | IMU acceleration spike |
| `accel_delta_threshold` | 7 m/s² | IMU acceleration delta |
| `speed_inconsistency_threshold` | 1.5 m/s | Odometry speed inconsistency |
| `scan_timeout` | 2.0 s | Laser scan timeout |
| `recovery_hold` | 20.0 s | Hold time after recovery |
| `cooldown` | 30.0 s | Cooldown between recovery attempts |

---

## 9. Dependencies

**Build & Runtime (ROS):**
- roscpp, rospy, std_msgs, geometry_msgs, nav_msgs, sensor_msgs
- tf, tf2_ros, tf2_geometry_msgs
- cartographer_ros_msgs
- dynamic_reconfigure

**Python Runtime:**
- `pyserial` — serial bridge, IMU driver
- `numpy` — numerical processing
- `torch` — NavOA neural network (DUNE/PAN)
- `PyYAML` — configuration files
- `Pillow` — maplocation GUI

**NavOA Algorithm Dependencies:**
- PyTorch (torch.nn.Module base class)
- DUNE / PAN network implementations (custom)
- NRMP risk metric predictor

---

## 10. Supported Launch Configurations

| Launch File | Mode | Description |
|-------------|------|-------------|
| `navoa_nav_only.launch` | NavOA + move_base | Full navigation with external SLAM |
| `local_avoidance_only.launch` | Obstacle avoidance only | No map, no global planner |
| `slam_toolbox_navoa.launch` | SLAM Toolbox mapping | Full SLAM + NavOA |
| `slam_toolbox_navoa_localization.launch` | SLAM Toolbox loc | Localization mode |
| `hector_map_amcl.launch` | Hector + AMCL | Alternative SLAM |
| `velocity_subscriber.launch` | Direct motor control | No planner, pure cmd_vel → motors |
| `teleop_bridge.launch` | Teleoperation | Keyboard control via serial bridge |
| `voice_nav.launch` | Voice navigation | Named places in English/Chinese |
