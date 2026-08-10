# Data Flow — TF Tree, ROS Topics & Serial Protocols

> **Date:** 2026-07-24  
> **Prerequisite:** [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)

---

## 1. TF Transform Tree

The system uses a standard ROS TF tree with three key frames:

```
map
  │
  │  (published by Cartographer when provide_odom_frame=false)
  │  OR (published by map_server / rf2o in some configs)
  │
  odom
  │
  │  (published by rf2o_laser_odometry → /odom_rf2o)
  │  OR (published by EKF fusion → /odom)
  │
  base_link
  │
  │  (static TF, in sensors.launch)
  │  x=0.19, y=0, z=0.20, yaw=-1.57
  │
  laser_frame
```

### Key Observations

| Transform | Publisher | Notes |
|-----------|-----------|-------|
| `map → odom` | Cartographer or map_server | Config-dependent: Cartographer `provide_odom_frame=false` means rf2o provides this |
| `odom → base_link` | rf2o_laser_odometry (or EKF) | Published at 20 Hz via `/odom_rf2o` |
| `base_link → laser_frame` | Static TF | Fixed offset: (0.19, 0, 0.20) with yaw -90° |

### Current TF Issue (Known)

From the [Navigation Drift Executive Summary](../Navigation_Drift_Executive_Summary.md):

**Current (broken) flow:**
```
[LiDAR] → RF2O → EKF → /odom
                 \ → base_link
                        |
                    Cartographer
                        |
                    map → base_link   ← WRONG!
                        |
                    move_base / map_server
```

**Desired flow:**
```
[LiDAR] → RF2O → EKF → /odom → Cartographer
                 \ → base_link        |
                                             ▼
                                         map → odom   ← CORRECT
                                             |
                                     move_base / map_server
```

Cartographer is publishing `map → base_link` directly instead of `map → odom`. This bypasses the EKF odometry frame, causing global map alignment drift.

---

## 2. ROS Topic Map

### 2.1 Sensor → Processing (Inputs)

```
Source                    Topic                      Type              Consumers
─────────────────────────────────────────────────────────────────────────────────────
lidar_node                /scan                      LaserScan         rf2o, Cartographer, NavOA, vel_filter
ascamera_node            /ascamera/rgb/image_raw     Image             visualization
ascamera_node            /ascamera/depth/image_raw   Image             depth_to_laserscan
rf2o_laser_odometry      /odom_rf2o                  Odometry          Cartographer, NavOA, MARL
imu_controller           /imu                        Imu               imu_mux (fallback)
serial_cmd_vel_imu_bridge /imu_serial                Imu               imu_mux (primary)
imu_mux                  /imu_mux                    Imu               cartographer_guard, localization_recovery
multi_ultrasonic_scan    /ultrasonic/front            Float32           vel_filter
multi_ultrasonic_scan    /ultrasonic/left             Float32           vel_filter
multi_ultrasonic_scan    /ultrasonic/right            Float32           vel_filter
a21_ultrasonic_scan      /Ultrasound_scan             LaserScan         (optional, synthesized scan)
```

### 2.2 Navigation (Planning → Action)

```
Source                    Topic                      Type              Consumers
─────────────────────────────────────────────────────────────────────────────────────
move_base (NavFN)        /move_base/GlobalPlanner/plan  Path           path_segment_publisher, waypoint_manager
path_segment_publisher   /split_goal                  PoseStamped       NavOA_core
waypoint_manager         /NavOA_goal                  PoseStamped       NavOA_core
NavOA_core               /NavOA_cmd_vel               Twist             vel_filter
vel_filter               /cmd_vel                     Twist             serial_cmd_vel_imu_bridge, velocity_subscriber
serial_cmd_vel_imu_bridge (serial)                     CSV serial       Motor controller (other PC)
external_cmd             /external_cmd_vel             Twist             NavOA_core (override)
external_cmd             /external_control_enable      Bool              NavOA_core
voice_goal_router        /move_base_simple/goal        PoseStamped       move_base
```

### 2.3 SLAM & Mapping

```
Source                    Topic                      Type              Consumers
─────────────────────────────────────────────────────────────────────────────────────
cartographer_node        /matching_score              Float32           runner_node, status_state, guard scripts
cartographer_node        /submap_list                 SubmapList        visualization
cartographer_occupancy_grid /map                      OccupancyGrid     move_base (global_costmap), status_state, MARL
cartographer_occupancy_grid /carto_submap_viz          OccupancyGrid     visualization (in localization mode, remapped from /map)
map_server (static)      /map                        OccupancyGrid     move_base (global_costmap) in localization mode
```

### 2.4 Status & Control

```
Source                    Topic                      Type              Consumers
─────────────────────────────────────────────────────────────────────────────────────
Web App (via rosbridge)  /runner/command              String            runner_node
runner_node              /runner/status               String (JSON)     Web App (via rosbridge)
runner_node              /runner/mode                 String            Web App (via rosbridge)
status_state             /localization_status         String            runner_node
status_state             /Mapping_status              String            runner_node
status_state             /Nav_status                  String            runner_node
status_state             /monitor_map_boundary        String            runner_node
vel_filter               /vel_filter/debug            String            vel_filter_debug_monitor
localization_recovery    /localization_status         String            runner_node
localization_recovery    /relocalization_trigger      Trigger (srv)     external call
```

### 2.5 Cartographer Services

```
Service                    Type                      Called By
─────────────────────────────────────────────────────────────────────
/start_trajectory          StartTrajectory           runner_node, guard scripts
/finish_trajectory         FinishTrajectory          runner_node, guard scripts
/write_state               WriteState                runner_node, mode_manager
/get_trajectory_states     GetTrajectoryStates       runner_node, status_state
```

---

## 3. Navigation Command Chain (End-to-End)

This is the complete data flow from goal to motor command:

```
External
  │
  ├─ Web: POST /api/mode/... → rosbridge → /runner/command → runner_node
  │
  ├─ RViz: /move_base_simple/goal
  │
  ├─ Voice: voice_goal_router → /move_base_simple/goal
  │
  ├─ MARL: exploration_node → /move_base_simple/goal
  │
  └─ Scripted: send_forward_goal → /move_base_simple/goal
                                         │
                                         ▼
  ┌───────────────────────────────────────────────────┐
  │ move_base                                         │
  │  - GlobalPlanner (NavFN) → /GlobalPlanner/plan    │
  │  - LocalPlanner (TrajectoryPlannerROS)             │
  │  - global_costmap (/map + static layer)            │
  │  - local_costmap (/scan + obstacle layer)          │
  │  - publishes /cmd_vel to /cmd_vel (via remapping)  │
  └───────────────────────────────────────────────────┘
                      │
                      ▼
  ┌───────────────────────────────────────────────────┐
  │ NavOA (AI Local Planner, ALTERNATIVE to base_local)│
  │                                                    │
  │  path_segment_publisher → /split_goal               │
  │  waypoint_manager → /NavOA_goal                      │
  │        │                                             │
  │        ▼                                             │
  │  NavOA_core.py (50 Hz control loop)                  │
  │    - Gets robot pose from TF                         │
  │    - Gets laser scan from /scan                      │
  │    - Generates initial path (line/circle)            │
  │    - Runs DUNE/PAN neural network                    │
  │    - Publishes [v, w] to /NavOA_cmd_vel               │
  │    - Publishes visualization markers                 │
  └───────────────────────────────────────────────────┘
                      │
                      ▼
  ┌───────────────────────────────────────────────────┐
  │ vel_filter.py (Safety Filter, 30-50 Hz)            │
  │                                                    │
  │  Input: /cmd_vel_tem (or configured input topic)    │
  │  Output: /cmd_vel                                   │
  │                                                    │
  │  Filter stages (in order):                         │
  │  1. Lidar forward safety (stop/slow zones)         │
  │  2. Ultrasonic virtual bumper (front/left/right)   │
  │  3. Camscan bias (glass detection)                 │
  │  4. Free-space reactive steering                   │
  │  5. Auto-reverse on collision                      │
  │  6. Deadbands & speed clamps                       │
  │  7. Reverse side detection + lateral evasion latch │
  └───────────────────────────────────────────────────┘
                      │
                      ▼
  ┌───────────────────────────────────────────────────┐
  │ serial_cmd_vel_imu_bridge.py (Bidirectional)        │
  │                                                    │
  │  Writer Thread (30 Hz):                             │
  │    "/cmd_vel" → CSV: "CMD_VEL,vx,vy,w\r\n"         │
  │    over /dev/ttyS1 at 460800 baud                  │
  │                                                    │
  │  Reader Thread:                                     │
  │    CSV: "IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw"     │
  │    → /imu_serial as sensor_msgs/Imu                │
  │    JSONL: Nav protocol events → /nav_serial/event  │
  │    BUTTON: "BUTTON,id,type" → /app_controller/btn  │
  └───────────────────────────────────────────────────┘
                      │
                      ▼
              Motor Controller (other PC)
              - Receives CMD_VEL over serial
              - Drives motors via motor drivers
              - Reads IMU and sends CSV data back
```

---

## 4. Serial Communication Protocols

### 4.1 Serial Port Map

| Port | Baud Rate | Connected To | Protocol |
|------|-----------|-------------|----------|
| `/dev/ttyS1` | 460800 | Motor controller PC | CSV (CMD_VEL + IMU) |
| `/dev/ttyS4` | 230400 | NVistar LiDAR | LiDAR binary protocol (0x0208) |
| `/dev/ttyS6` | 230400 | WIT-series IMU | WIT 33-byte frame block |
| `/dev/ttyS?` | Variable | DYP-A21 ultrasonic | 4-byte frame (FF Data_H Data_L SUM) |

### 4.2 Serial Bridge Protocol (ttyS1)

**Direction: ROS → Motor Controller**

```
CMD_VEL, 0.25, 0.00, 0.00\r\n
         ↑      ↑      ↑
         vx     vy     wz (rad/s)
```

**Direction: Motor Controller → ROS**

```
IMU, 0.12, -0.01, 9.81, 0.02, -0.01, 0.05, 1.23, -0.45, 178.5\r\n
↑    ↑       ↑     ↑     ↑     ↑     ↑     ↑      ↑      ↑
tag  ax(m/s²) ay   az   gx(°/s) gy   gz   roll(°) pitch yaw
```

Nav JSONL Protocol (event-driven):
```json
{"protocol":"xrobot-nav-jsonl-v1","msg_type":"event","event_type":"button_pressed","data":{"button_id":1}}
```

### 4.3 WIT IMU Protocol (ttyS6)

33-byte block containing three 11-byte frames:

| Byte Offset | Header | Content | Purpose |
|-------------|--------|---------|---------|
| 00-10 | `0x55 0x51` | Accel X/Y/Z (int16 × 3) + Temp (int16) + SUM | Accelerometer data |
| 11-21 | `0x55 0x52` | Gyro X/Y/Z (int16 × 3) + Temp (int16) + SUM | Gyroscope data |
| 22-32 | `0x55 0x53` | Roll/Pitch/Yaw (int16 × 3) + Version (int16) + SUM | Euler angles |

Scale factors: Accel = raw/32768 × 16g, Gyro = raw/32768 × 2000°/s, Angle = raw/32768 × 180°

### 4.4 DYP-A21 Ultrasonic Protocol

4-byte frame: `0xFF | Data_H(6-bit) | Data_L(8-bit) | (H+L) & 0xFF`

Special values: 0xFFFE = Interference, 0xFFFD = No object

### 4.5 NVistar LiDAR Protocol

Using protocol 0x0208 (PROTOCOL_MODEL_NORMAL_NO_QUALITY):
- 360 points per revolution at fixed resolution
- Each point: distance (uint16, mm) + intensity (uint8)
- Range: 0.1 — 15m

---

## 5. Key Topic Frequencies

| Topic | Frequency | Publisher |
|-------|-----------|-----------|
| `/scan` | 10-15 Hz | lidar_node |
| `/odom_rf2o` | 20 Hz | rf2o_laser_odometry |
| `/odom` (EKF) | 30 Hz | robot_localization EKF |
| `/imu` | 500 Hz (raw), configurable (pub) | IMU controller |
| `/imu_serial` | 30-50 Hz | serial_cmd_vel_imu_bridge |
| `/imu_mux` | 30-50 Hz | imu_mux |
| `/matching_score` | ~10 Hz | cartographer_node |
| `/map` (occupancy grid) | 0.5-1 Hz | cartographer_occupancy_grid_node |
| `/NavOA_cmd_vel` | 50 Hz | NavOA_core.py |
| `/cmd_vel` (filtered) | 30-50 Hz | vel_filter.py |
| `/cmd_vel` (to serial) | 30 Hz (throttled) | serial_cmd_vel_imu_bridge |
| `/runner/status` | On change or request | runner_node |
| `/vel_filter/debug` | 10 Hz | vel_filter.py |

---

## 6. Coordinate Frames & Conventions

| Frame | Convention |
|-------|-----------|
| `map` | World-fixed, origin at start of first mapping trajectory. X=forward, Y=left, Z=up |
| `odom` | World-fixed, drifting reference for local consistency. Same orientation as `map` |
| `base_link` | Robot base, projected to ground plane. X=robot forward, Y=robot left, Z=up |
| `laser_frame` | LiDAR origin. Rotated -90° yaw from base_link (LiDAR scans horizontal plane) |

**NavOA coordinate conventions (in robot frame):**
- +X: forward
- +Z: angular velocity positive = counterclockwise (standard ROS convention)
- Velocity command: `[linear.x, angular.z]` for differential drive
