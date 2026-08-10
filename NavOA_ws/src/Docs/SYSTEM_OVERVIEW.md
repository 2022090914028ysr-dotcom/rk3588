# SlamNav System Overview

> **Version:** 1.0  
> **Date:** 2026-07-24  
> **Workspace:** `/home/ubuntu/Robit_SlamNav/NavOA_ws/`

---

## Table of Contents

1. [What is SlamNav?](#what-is-slamnav)
2. [System Architecture at a Glance](#system-architecture-at-a-glance)
3. [Key Capabilities](#key-capabilities)
4. [Package Map](#package-map)
5. [Hardware Platform](#hardware-platform)
6. [Operational Modes](#operational-modes)
7. [Quick-Start Guide](#quick-start-guide)
8. [Related Documents](#related-documents)

---

## What is SlamNav?

SlamNav is a ROS 1 (Melodic/Noetic) based autonomous navigation system for a differential-drive robotic platform. It integrates:

- **2D LiDAR SLAM** (Google Cartographer) for mapping and localization
- **AI-driven local planning** (NavOA — Neural Network Optimized Avoidance) for collision-free navigation
- **Multi-sensor fusion** (LiDAR, depth camera, IMU, ultrasonic array) for robust perception
- **Frontier-based autonomous exploration** (MARL — Multi-Agent Reinforcement Learning adapted to single-robot)
- **Web-based remote control** via FastAPI + rosbridge WebSocket

The system can operate in four primary modes — **Mapping**, **Localization**, **Sensors-only**, and **Auto Mapping** — with seamless transitions between them.

---

## System Architecture at a Glance

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                           WEB LAYER (FastAPI)                                │
│                                                                              │
│  POST /api/mode/mapping     POST /api/mode/stop-mapping                     │
│  POST /api/mode/auto_mapping   GET  /api/status                             │
│                       │                                                     │
│                 roslibpy (WebSocket)                                         │
└───────────────────────┬──────────────────────────────────────────────────────┘
                        │
                  rosbridge_server
                   (port 9090)
                        │
                /runner/command   (publish)
                /runner/status    (subscribe)
                /runner/mode      (subscribe)
                        │
┌───────────────────────┼──────────────────────────────────────────────────────┐
│               ROS LAYER (runner_node.py — orchestrator)                      │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────────┐│
│  │  SENSOR LAYER (sensors.launch — ALWAYS ON, never killed)                ││
│  │                                                                          ││
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ ││
│  │  │  LiDAR       │  │  Depth Cam   │  │  Ultrasonic  │  │  IMU (WIT)   │ ││
│  │  │  (NVistar)   │  │  (HP60C)     │  │  (DYP-A21)   │  │  (Serial)    │ ││
│  │  │  → /scan     │  │  → /ascamera │  │  → /ultra*   │  │  → /imu_mux  │ ││
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ ││
│  │         │                 │                  │                 │          ││
│  │         ▼                 ▼                  ▼                 ▼          ││
│  │  ┌──────────────────────────────────────────────────────────────────────┐ ││
│  │  │  rf2o_laser_odometry → /odom_rf2o  (range-flow laser odometry)     │ ││
│  │  │  serial_cmd_vel_imu_bridge  → /imu_serial + motor control           │ ││
│  │  │  imu_mux                  → /imu_mux   (primary:serial, fallback:imu)│ ││
│  │  └──────────────────────────────────────────────────────────────────────┘ ││
│  │                                                                          ││
│  │  ┌──────────────────────────────────────────────────────────────────────┐ ││
│  │  │  NavOA Navigation Stack (move_base + NavOA planner)                  │ ││
│  │  │    /move_base/GlobalPlanner/plan → path_segment_publisher            │ ││
│  │  │    navoa_waypoint_sequencer → waypoint_manager → NavOA_core.py       │ ││
│  │  │    NavOA_core.py → /NavOA_cmd_vel → vel_filter.py → /cmd_vel        │ ││
│  │  └──────────────────────────────────────────────────────────────────────┘ ││
│  └──────────────────────────────────────────────────────────────────────────┘│
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────────┐│
│  │  SLAM LAYER (cartographer_node — ALWAYS ON)                              ││
│  │                                                                          ││
│  │  inputs: /scan, /odom_rf2o                                               ││
│  │  services: /start_trajectory, /finish_trajectory, /write_state           ││
│  │  output: /map (via occupancy_grid_node)                                  ││
│  │                                                                          ││
│  │  Lua configs: livox_2d.lua (mapping), livox_2d_localization.lua (loc)   ││
│  │  Guard scripts: cartographer_guard_imu_fast_reloc.py (auto-recovery)    ││
│  └──────────────────────────────────────────────────────────────────────────┘│
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────────┐│
│  │  STATUS LAYER (status_state.py)                                          ││
│  │                                                                          ││
│  │  /localization_status  ← /matching_score                                 ││
│  │  /Mapping_status       ← /get_trajectory_states                          ││
│  │  /Nav_status           ← /move_base/status + cmd_vel                     ││
│  │  /monitor_map_boundary ← /map + TF                                       ││
│  └──────────────────────────────────────────────────────────────────────────┘│
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────────┐│
│  │  ON-DEMAND: MARL Autonomous Exploration                                  ││
│  │                                                                          ││
│  │  Started as subprocess by runner_node.py when mode=auto_mapping          ││
│  │  Killed on any mode switch                                               ││
│  │  Uses frontier detection → publishes goals to /move_base_simple/goal     ││
│  └──────────────────────────────────────────────────────────────────────────┘│
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Key Capabilities

| Capability | Description | Package |
|-----------|-------------|---------|
| **2D SLAM Mapping** | Build occupancy-grid maps using Google Cartographer | `carto_ws` |
| **Localization** | Localize against pre-built maps with pure-localization trajectories | `carto_ws` |
| **AI Motion Planning** | Neural-network-based local planning (DUNE/PAN) for smooth obstacle avoidance | `NavOA_ros` |
| **Safety Filter** | Multi-layer velocity filter: lidar stop zones, ultrasonic virtual bumper, camscan glass detection | `NavOA_ros` |
| **Autonomous Exploration** | Frontier-based exploration using OpenCV connected-components for unknown region detection | `marl_mapping` |
| **Web API** | FastAPI-based remote control via rosbridge WebSocket | External |
| **IMU Fall Detection** | Automatic recovery if robot tips over (roll > 20° / pitch > 30°) | `carto_ws` |
| **Localization Recovery** | Score-based guard that reinitializes Cartographer trajectory when matching degrades | `carto_ws` |
| **Map Boundary Monitor** | Detects when robot approaches or leaves the known map area | `runner` |
| **Voice Navigation** | Natural-language goal routing with English/Chinese support | `NavOA_ros` |

---

## Package Map

```
NavOA_ws/
├── src/
│   ├── runner/              ← Central orchestrator (full.launch, runner_node.py, status_state.py)
│   ├── NavOA_ros/           ← Navigation stack (NavOA planner, vel_filter, serial_bridge, IMU)
│   ├── carto_ws/            ← Google Cartographer SLAM (nested catkin workspace)
│   │   ├── src/cartographer/        ← Core Cartographer SLAM library (C++)
│   │   └── src/cartographer_ros/    ← ROS integration (launch files, Lua configs, guard scripts)
│   ├── lidar_ros/           ← NVistar VP100/T10 2D LiDAR driver (C++)
│   ├── rf2o_laser_odometry/ ← Range-flow laser odometry (C++, MAPIR lab)
│   ├── marl_mapping/        ← Frontier-based autonomous exploration (Python)
│   ├── ascam_ws/            ← Camera driver workspace (nested)
│   │   ├── ascamera/               ← HP60C depth camera driver (C++)
│   │   ├── ascam_visual/           ← Visual processing (OpenCV, AR tags, QR)
│   │   ├── depth_to_scancamera/    ← Depth-image to laser-scan conversion
│   │   ├── yahboomcar_msgs/        ← Custom message definitions
│   │   └── yahboomcar_mediapipe/   ← MediaPipe gesture/pose recognition
│   ├── map/                 ← ROS occupancy grid maps (.pgm, .yaml, .pbstream)
│   └── abseil-cpp/          ← Abseil C++ dependency (pre-built, for Cartographer)
└── Navigation_Drift_Executive_Summary.md  ← Known-issue document
```

---

## Hardware Platform

| Component | Specification | ROS Interface |
|-----------|--------------|---------------|
| **Main Computer** | Unknown ARM/x86 SBC (likely RK3588 or similar) | Runs ROS 1 master |
| **LiDAR** | NVistar VP100/T10 | `/dev/ttyS4`, 230400 baud, → `/scan` |
| **Depth Camera** | Ascamera HP60C | USB, → `/ascamera/...`, 640×480 |
| **IMU** | WIT-series (WT901?) | `/dev/ttyS6`, 230400 baud, → `/imu` |
| **Ultrasonic** | DYP-A21 x3 (front/left/right) | UART, → `/ultrasonic/*` |
| **Motor Controller** | Separate MCU board | `/dev/ttyS1`, 460800 baud, CSV protocol |
| **LiDAR Mount** | Forward-facing, rotated 90° | TF: `base_link → laser_frame` at (0.19, 0, 0.2, yaw=-1.57) |

**TF Tree:**
```
map → odom → base_link → laser_frame
                  ↓
            (rf2o publishes odom→base_link)
```

---

## Operational Modes

### Mode Comparison

| Mode | Cartographer Trajectory | NavOA | MARL | Use Case |
|------|------------------------|-------|------|----------|
| `MAPPING` | Active (mapping) | ON | OFF | Build a new map |
| `LOCALIZATION` | Active (pure localization) | ON | OFF | Navigate using pre-built map |
| `SENSORS_ONLY` | Idle (no active trajectory) | ON | OFF | Hardware testing, teleop |
| `AUTO_MAPPING` | Active (mapping) | ON | ON | Autonomous map building |
| `SHUTTING_DOWN` | Finished (idle) | KILLED | KILLED | Graceful NavOA shutdown |

### Mode Transition Diagram

```
                    ┌─────────────┐
                    │ SENSORS_ONLY│
                    └──────┬──────┘
                           │ mode:mapping
                           ▼
                    ┌─────────────┐
              ┌─────┤   MAPPING   ├─────┐
              │     └──────┬──────┘     │
              │            │            │
              │   stop_mapping    mode:auto_mapping
              │            │            │
              ▼            ▼            ▼
       ┌───────────┐ ┌──────────────┐ ┌──────────────┐
       │LOCALIZATION│ │AUTO_MAPPING │ │  (auto map)  │
       └───────────┘ └──────────────┘ └──────────────┘
              │            │
              │    stop_mapping
              ▼            ▼
       ┌───────────┐
       │SENSORS_ONLY│
       └───────────┘
```

---

## Quick-Start Guide

### Prerequisites

- ROS 1 (Melodic or Noetic) installed
- Workspace built: `catkin_make_isolated` or `catkin build`
- Source workspace: `source devel_isolated/setup.bash` or `devel/setup.bash`

### Launch Commands

```bash
# Convenience launcher (auto-sources workspace):
./src/runner/script/run.sh mapping
./src/runner/script/run.sh localization detached
./src/runner/script/run.sh auto_mapping detached

# Or via roslaunch directly:
roslaunch runner full.launch mode:=manager

# Manager mode starts in SENSORS_ONLY state → then send commands
```

### Web API Commands

```bash
# Start mapping
curl -X POST http://robot_ip:8000/api/mode/mapping

# Stop mapping → enter localization
curl -X POST http://robot_ip:8000/api/mode/stop-mapping

# Start autonomous exploration
curl -X POST http://robot_ip:8000/api/mode/auto-mapping

# Get system status
curl http://robot_ip:8000/api/status
```

---

## Related Documents

| Document | Description |
|----------|-------------|
| [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) | Detailed layered architecture and component interfaces |
| [DATA_FLOW.md](DATA_FLOW.md) | TF tree, ROS topics, serial protocols, and inter-module communication |
| [NAVOA_NAVIGATION_STACK.md](NAVOA_NAVIGATION_STACK.md) | NavOA_ros package: planner, vel_filter, serial bridge, IMU |
| [CARTOGRAPHER_INTEGRATION.md](CARTOGRAPHER_INTEGRATION.md) | Cartographer SLAM: Lua configs, guard scripts, trajectory management |
| [SENSOR_SUBSYSTEM.md](SENSOR_SUBSYSTEM.md) | LiDAR, camera, IMU, ultrasonic — drivers and calibration |
| [RUNNER_ORCHESTRATOR.md](RUNNER_ORCHESTRATOR.md) | runner package: full.launch, mode switching, status publishing |
| [AUTONOMOUS_EXPLORATION.md](AUTONOMOUS_EXPLORATION.md) | MARL frontier exploration module |
| [LIMITATIONS_AND_KNOWN_ISSUES.md](LIMITATIONS_AND_KNOWN_ISSUES.md) | Current limitations and known issues for NavOA_ros and Cartographer |
