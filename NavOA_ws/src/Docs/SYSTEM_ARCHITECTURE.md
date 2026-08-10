# System Architecture — Detailed Reference

> **Date:** 2026-07-24  
> **Package:** `runner` (orchestrator)  
> **Prerequisite:** [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)

---

## 1. Three-Layer Architecture

The system is organized into three logical layers:

### Layer 1: Web Application (FastAPI)

The web layer is entirely decoupled from ROS. It communicates exclusively via `roslibpy` over WebSocket to `rosbridge_server` (port 9090).

```
FastAPI Server                     Robot (ROS Master)
┌─────────────────┐               ┌──────────────────────┐
│  POST /api/...  │──WebSocket──▶│  rosbridge_server    │
│  GET  /api/...  │◀──JSON──────│  (port 9090)          │
│                 │               │  /runner/command (pub)│
│  roslibpy       │               │  /runner/status  (sub)│
└─────────────────┘               └──────────────────────┘
```

**API Endpoints:**

| Endpoint | Command Published | Effect |
|----------|-----------------|--------|
| `POST /api/mode/mapping` | `mode:mapping` | Start mapping trajectory |
| `POST /api/mode/stop-mapping` | `stop_mapping` | Save map → localization |
| `POST /api/mode/auto-mapping` | `mode:auto_mapping` | Mapping + MARL exploration |
| `POST /api/mode/sensors` | `mode:sensors` | Finish trajectory (idle) |
| `POST /api/mode/shutdown` | `shutdown` | Kill NavOA, finish traj |
| `GET /api/status` | (none) | Return cached JSON status |
| `POST /api/save-map` | `save_map [name]` | Save .pbstream to disk |
| `POST /api/list-maps` | `list_maps` | List saved maps |

### Layer 2: ROS Orchestration (runner_node.py)

The central brain. A single ROS node (`runner_node.py` or its alias `mode_manager.py`) that:

1. **Subscribes** to `/runner/command` (std_msgs/String)
2. **Translates** commands into Cartographer service calls
3. **Manages** the MARL exploration subprocess (the only subprocess)
4. **Publishes** JSON status on `/runner/status`

### Layer 3: Always-On Hardware & Navigation

Started via `roslaunch runner full.launch mode:=manager`. These processes run at the top-level roslaunch tree and are **never killed or restarted** as subprocesses:

- Sensors (LiDAR, camera, ultrasonic, IMU)
- rf2o laser odometry
- Serial bridge (cmd_vel ↔ IMU ↔ motor controller)
- NavOA navigation stack (move_base + NavOA controller + vel_filter)
- cartographer_node (SLAM engine — trajectory services only)
- Occupancy grid node
- rosbridge_server

The **only** process started/stopped on demand is the MARL exploration subprocess.

---

## 2. Component Responsibilities

### 2.1 runner_node.py (792 lines)

**File:** `src/runner/script/runner_node.py`

A ROS node class `RunnerNode` that acts as the system lifecycle manager.

**Publishers:**
- `/runner/status` (std_msgs/String, latched) — Full JSON status
- `/runner/mode` (std_msgs/String, latched) — Simple mode string

**Subscribers:**
- `/runner/command` (std_msgs/String) — Command input
- `/localization_status` (from status_state.py)
- `/Mapping_status` (from status_state.py)
- `/Nav_status` (from status_state.py)
- `/monitor_map_boundary` (from status_state.py)
- `/matching_score` (Float32, from Cartographer)

**Cartographer Service Clients:**
| Service | Purpose |
|---------|---------|
| `/start_trajectory` | Start new trajectory with config + initial pose |
| `/finish_trajectory` | Finish active trajectory (freezes its submaps) |
| `/write_state` | Save current pose graph to .pbstream file |
| `/get_trajectory_states` | Get state of all trajectories (0=ACTIVE,1=FINISHED,2=FROZEN,3=DELETED) |

**Key Internal Methods:**
| Method | Purpose |
|--------|---------|
| `_get_carto_service()` | Generic service proxy getter with timeout |
| `_is_carto_alive()` | Probes `/get_trajectory_states` |
| `_get_active_trajectory_id()` | Finds the latest ACTIVE trajectory (state=0) |
| `_get_reference_trajectory_id()` | Finds first FINISHED trajectory (reference frame) |
| `_call_finish()` | Calls `/finish_trajectory` |
| `_call_start()` | Calls `/start_trajectory` with config + pose + reference |
| `_call_write_state()` | Calls `/write_state` to save .pbstream |
| `_launch_exploration()` | Spawns MARL subprocess via Popen with `os.setsid` |
| `_kill_exploration()` | SIGTERM → 3s wait → SIGKILL to process group |

### 2.2 status_state.py (618 lines)

**File:** `src/runner/script/status_state.py`

Derives human-readable status from raw ROS topics. Publishes four status topics:

| Status Topic | Source Data | Values |
|-------------|-------------|--------|
| `/localization_status` | `/matching_score` + timeout (5s) | LOCALIZED HIGH (≥0.7) / LOW (≥0.3) / LOST (<0.3) |
| `/Mapping_status` | `/get_trajectory_states` service + `/app/map_status` | MAPPING ON / LOCALIZATION ON / INITIALIZING / LOST |
| `/Nav_status` | `/move_base/status` + `/NavOA_cmd_vel` timeout (2s) | IDLE / NAVIGATING / ARRIVED / FAILED / CANCELLED |
| `/monitor_map_boundary` | `/map` OccupancyGrid + TF map→base_link | INSIDE_MAP / NEAR_BOUNDARY / OUT_OF_MAP / (None during settle) |

The map boundary monitor uses a `MapInfo` class that computes:
- **rect**: Full map bounding box
- **tight**: Bounding box of known (non-unknown) cells
- Warn at 0.5m from edge with 0.1m hysteresis, 3s settle time after map switches

---

## 3. The Invariant Table

| Component | Started by | Can be killed? | Notes |
|-----------|-----------|----------------|-------|
| `lidar_node` | roslaunch (top-level) | ❌ NEVER | Part of sensors.launch |
| `rf2o_laser_odometry` | roslaunch (top-level) | ❌ NEVER | Part of sensors.launch |
| `ascamera_node` | roslaunch (top-level) | ❌ NEVER | Part of sensors.launch (conditional) |
| `serial_cmd_vel_imu_bridge` | roslaunch (top-level) | ❌ NEVER | Part of sensors.launch |
| `imu_mux` | roslaunch (top-level) | ❌ NEVER | Part of sensors.launch |
| `multi_ultrasonic_scan` | roslaunch (top-level) | ❌ NEVER | Part of sensors.launch |
| NavOA (move_base + controllers) | roslaunch (top-level) | ✅ On `shutdown` only | Via pkill -f |
| `cartographer_node` | roslaunch (top-level) | ❌ NEVER | Trajectory-level management only |
| `cartographer_occupancy_grid_node` | roslaunch (top-level) | ❌ NEVER | |
| `runner_status_state` | roslaunch (top-level) | ✅ Restarts if crashes | respawn="true" |
| `runner_manager` | roslaunch (top-level) | ✅ Restarts if crashes | respawn="true" |
| MARL exploration | **subprocess** of runner_node | ✅ Killed on mode switch | Managed via Popen |
| `rosbridge_server` | roslaunch (top-level) | ❌ NEVER | Port 9090 |

---

## 4. Mode Switching — Detailed Sequence

### 4.1 Sensors-only → Mapping

```
Web: "mode:mapping"
                │
runner_node:
  1. Check cartographer is alive (probe /get_trajectory_states)
  2. Get active trajectory ID (if any)
  3. Get reference trajectory — first FINISHED trajectory (ID 0)
  4. Cache robot pose from TF (map→base_link)
  5. /finish_trajectory  ← stops current trajectory (if active)
  6. /start_trajectory   ← starts mapping trajectory
     config: livox_2d.lua
     initial_pose: cached pose
     relative_to: ref_id 0
  7. Kill MARL (if running)
  8. Set mode → MAPPING
```

### 4.2 Mapping → Localization

```
Web: "stop_mapping" or "mode:localization"
                │
runner_node:
  1. Check cartographer is alive
  2. Cache robot pose from TF (map→base_link)
  3. /write_state  ← save map to map_<timestamp>.pbstream
  4. Find reference trajectory (ID 0 — FINISHED submaps)
  5. /finish_trajectory  ← stops mapping trajectory
  6. /start_trajectory   ← starts localization trajectory
     config: livox_2d_localization.lua
     initial_pose: cached pose
     relative_to: ref_id 0
     (pure_localization_trimmer keeps only 3 submaps)
  7. Kill MARL (if running)
  8. Set mode → LOCALIZATION
```

### 4.3 Mapping → Auto Mapping

```
Web: "mode:auto_mapping"
                │
runner_node:
  1. Ensure mapping trajectory is active (start one if needed)
  2. Launch MARL exploration subprocess:
     $ roslaunch marl_mapping exploration.launch
  3. Set mode → AUTO_MAPPING
```

### 4.4 Shutdown

```
Web: "shutdown"
                │
runner_node:
  1. Finish active cartographer trajectory (idle state)
  2. pkill -f move_base, map_server, amcl  ← kill NavOA
  3. Kill MARL (if running)
  4. Set mode → SHUTTING_DOWN
  5. rospy.signal_shutdown()
```

---

## 5. Cartographer Pose Graph Mechanics

The key insight enabling seamless mode switching is Cartographer's **pose graph architecture**.

```
Before switch:  [Trajectory 0: MAPPING]
                   ├── submap 0
                   ├── submap 1
                   └── submap 2 (building)

After /finish_trajectory(0):
                [Trajectory 0: FINISHED]   ← submaps frozen
                   ├── submap 0 ──────┐
                   ├── submap 1       │ still in pose graph
                   └── submap 2 ──────┘

After /start_trajectory(localization):
                [Trajectory 0: FINISHED]   ← reference frame
                [Trajectory 1: ACTIVE]     ← pure_localization
                   └── submap 3 (trimmed)

Trajectory 1 matches scans against Trajectory 0's finished submaps.
```

**Why this matters:** When you finish trajectory 0, its submaps stay in the pose graph as FINISHED state. When you start trajectory 1, it creates constraints against trajectory 0's submaps — same mechanism as loading a `.pbstream` file, but **in-memory with no disk I/O**. This makes mode switches instantaneous.

---

## 6. JSON Status Format

Published on `/runner/status` as `std_msgs/String` containing JSON:

```json
{
  "mode": "MAPPING",
  "timestamp": 1783758959.582,
  "trajectory": {
    "active_id": 2,
    "all_states": [1, 3, 0]
  },
  "loc_status": "LOCALIZED HIGH",
  "loc_score": 0.85,
  "map_status": "MAPPING ON",
  "nav_status": "IDLE",
  "oob_status": "INSIDE_MAP",
  "maps": ["map_123.pbstream", "Office.pbstream"],
  "cartographer_alive": true,
  "sensors_always_on": true,
  "error": ""
}
```

**Trajectory state values:** `0=ACTIVE`, `1=FINISHED`, `2=FROZEN`, `3=DELETED`

---

## 7. Launch File Hierarchy

```
full.launch (mode:=manager)
│
├── sensors.launch
│   ├── ascamera (HP60C depth camera)
│   ├── lidar_node (NVistar VP100 LiDAR)
│   ├── static TF (base_link → laser_frame)
│   ├── rf2o_laser_odometry
│   ├── multi_ultrasonic_scan
│   ├── serial_cmd_vel_imu_bridge
│   ├── imu_mux
│   ├── NavOA navigation stack (navoa_nav_only.launch)
│   │   ├── move_base (global + local planners)
│   │   ├── vel_filter.py
│   │   ├── path_segment_publisher.py
│   │   └── status_state types
│   └── rosbridge_server (port 9090)
│
├── cartographer_node (livox_2d.lua config)
├── cartographer_occupancy_grid_node (0.05m resolution)
├── runner_status_state (status_state.py)
└── runner_manager (mode_manager.py / runner_node.py)
```

---

## 8. File Reference

| File | Purpose |
|------|---------|
| `launch/full.launch` | Top-level launcher with `mode` argument |
| `launch/sensors.launch` | Always-on sensor + NavOA stack |
| `launch/mapping.launch` | Cartographer mapping (used standalone) |
| `launch/localization.launch` | Cartographer localization + map_server |
| `launch/auto_mapping.launch` | Mapping + MARL exploration |
| `launch/camera_imu_lidar.launch` | Minimal sensor test launcher |
| `script/runner_node.py` | System lifecycle manager |
| `script/mode_manager.py` | Alias importing RunnerNode |
| `script/status_state.py` | Enriched status publisher |
| `script/run.sh` | Shell convenience launcher |
