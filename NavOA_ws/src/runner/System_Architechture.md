# System Architecture — SlamNav Runner

## Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       WEB LAYER (FastAPI)                               │
│                                                                         │
│  POST /api/mode/mapping        ───┐                                     │
│  POST /api/mode/stop-mapping      │  roslibpy                            │
│  GET  /api/status               ──┤  (WebSocket)                        │
│                                   │                                     │
└───────────────────────────────────┼─────────────────────────────────────┘
                                    │
                          rosbridge_server
                         (port 9090, always running)
                                    │
                            /runner/command   (publish)
                            /runner/status    (subscribe)
                            /runner/mode      (subscribe)
                                    │
┌───────────────────────────────────┼─────────────────────────────────────┐
│                    ROS LAYER (robot, processes)                         │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  roslaunch runner full.launch mode:=manager                      │   │
│  │                                                                  │   │
│  │  ┌────────────────────────────────────────────┐                  │   │
│  │  │  ALWAYS-ON (managed by roslaunch — NEVER   │                  │   │
│  │  │  killed)                                   │                  │   │
│  │  │                                            │                  │   │
│  │  │  sensors.launch:                           │                  │   │
│  │  │    ├─ lidar_node        (/scan)            │                  │   │
│  │  │    ├─ rf2o_laser_odom   (/odom_rf2o)      │                  │   │
│  │  │    ├─ camera            (/ascamera_hp60c)  │                  │   │
│  │  │    ├─ multi_ultrasonic                     │                  │   │
│  │  │    ├─ serial_bridge     (cmd_vel + IMU)    │                  │   │
│  │  │    ├─ imu_mux           (/imu_mux)         │                  │   │
│  │  │    └─ NavOA navigation  (move_base +       │                  │   │
│  │  │                           velocity filter)  │                  │   │
│  │  │                                            │                  │   │
│  │  │  cartographer_node:                        │                  │   │
│  │  │    ├─ /start_trajectory  (service)         │                  │   │
│  │  │    ├─ /finish_trajectory (service)         │                  │   │
│  │  │    ├─ /write_state       (service)         │                  │   │
│  │  │    ├─ /get_trajectory_states (service)     │                  │   │
│  │  │    └─ /matching_score    (topic)           │                  │   │
│  │  │                                            │                  │   │
│  │  │  occupancy_grid_node: /carto_submap_viz    │                  │   │
│  │  └────────────────────────────────────────────┘                  │   │
│  │                                                                  │   │
│  │  ┌────────────────────────────────────────────┐                  │   │
│  │  │  ROS NODES (managed by roslaunch,          │                  │   │
│  │  │  restart if crash)                         │                  │   │
│  │  │                                            │                  │   │
│  │  │  runner_status_state:                      │                  │   │
│  │  │    ├─ /localization_status                 │                  │   │
│  │  │    ├─ /Mapping_status                      │                  │   │
│  │  │    ├─ /Nav_status                          │                  │   │
│  │  │    └─ /monitor_map_boundary                │                  │   │
│  │  │                                            │                  │   │
│  │  │  runner_manager (runner_node.py):           │                  │   │
│  │  │    ├─ subscribes /runner/command           │                  │   │
│  │  │    ├─ publishes /runner/status (JSON)      │                  │   │
│  │  │    └─ publishes /runner/mode (string)      │                  │   │
│  │  └────────────────────────────────────────────┘                  │   │
│  │                                                                  │   │
│  │  ┌────────────────────────────────────────────┐                  │   │
│  │  │  ON-DEMAND SUBPROCESS (started/killed       │                  │   │
│  │  │  by runner_node.py)                        │                  │   │
│  │  │                                            │                  │   │
│  │  │  MARL exploration_node: /autonomous_explore│                  │   │
│  │  │    (only active in mode:auto_mapping)      │                  │   │
│  │  └────────────────────────────────────────────┘                  │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## The Three Layers

### Layer 1: Web App (FastAPI)

Your web UI sends simple string commands and reads JSON back. All ROS complexity is hidden.

```python
from roslibpy import Ros, Topic

client = Ros("robot_ip", 9090)
cmd_pub = Topic(client, "/runner/command", "std_msgs/String")

@router.post("/api/mode/mapping")
async def mapping():
    cmd_pub.publish({"data": "mode:mapping"})
    return {"ok": True}

@router.post("/api/mode/stop-mapping")
async def stop_mapping():
    cmd_pub.publish({"data": "stop_mapping"})
    return {"ok": True}

@router.get("/api/status")
async def status():
    return latest_status  # cached from /runner/status subscription
```

**Commands FastAPI can send:**

| Command | Action |
|---------|--------|
| `mode:mapping` | Start mapping trajectory |
| `mode:localization` / `stop_mapping` | Save map, finish mapping, start localization |
| `mode:auto_mapping` | Mapping + MARL exploration |
| `mode:sensors` | Finish active trajectory (idle) |
| `save_map [name]` | Save current map to .pbstream |
| `list_maps` | List saved maps in JSON status |
| `status` | Publish fresh JSON status |
| `shutdown` | Finish current trajectory, kill NavOA + MARL, keep sensors + cartographer idle |

### Layer 2: ROS Nodes (the robot brain)

**runner_node.py** — subscribes to `/runner/command`, translates each command into Cartographer service calls, publishes JSON status.

**status_state.py** — derives human-readable status from raw ROS topics:

| ROS Topic | Status Field | Values |
|-----------|-------------|--------|
| `/matching_score` | `loc_status` | LOCALIZED HIGH / LOW / LOST |
| trajectory states | `map_status` | MAPPING ON / LOCALIZATION ON / INITIALIZING / LOST |
| `/move_base/status` | `nav_status` | IDLE / NAVIGATING / ARRIVED / FAILED / CANCELLED |
| `/map` + TF | `oob_status` | INSIDE_MAP / NEAR_BOUNDARY / OUT_OF_MAP |

### Layer 3: Always-On Processes (hardware)

Started with `roslaunch runner full.launch mode:=manager`, **never killed**:

- LiDAR, camera, ultrasonic, IMU, rf2o odometry, serial bridge
- NavOA navigation stack (move_base + controllers)
- cartographer_node (SLAM engine)

Only **MARL exploration** is started/stopped on demand (for `mode:auto_mapping`).

---

## Mode Switching — How It Works

### Mode: MAPPING

```
Web: "mode:mapping"
                │
runner_node:    │
  1. Check cartographer is alive
  2. Get active trajectory ID
  3. Get reference trajectory (ID 0, FINISHED)
  4. Cache robot pose from TF (map→base_footprint)
  5. /finish_trajectory     ← stops current trajectory
  6. /start_trajectory      ← starts mapping (no trimmer)
     config: livox_2d.lua
     initial_pose: cached
     relative_to: ref_id 0
  7. Kill MARL (if running)
  8. Set mode → MAPPING
```

**Result:** New trajectory ACTIVE, building map. Sensors + NavOA stay on.

### Mode: LOCALIZATION (stop_mapping)

```
Web: "stop_mapping"
                │
runner_node:    │
  1. Check cartographer is alive
  2. Cache robot pose from TF
  3. /write_state             ← saves map to .pbstream
     filename: map_<timestamp>.pbstream
  4. Find reference trajectory (ID 0)
  5. /finish_trajectory      ← stops mapping
  6. /start_trajectory       ← starts localization
     config: livox_2d_localization.lua
     initial_pose: cached
     relative_to: ref_id 0
     (pure_localization_trimmer keeps only 3 submaps)
  7. Kill MARL (if running)
  8. Set mode → LOCALIZATION
```

**Result:** Old trajectory FINISHED (submaps preserved in pose graph). New trajectory ACTIVE (localizing against old map). Sensors + NavOA stay on.

### Mode: SENSORS (idle)

```
Web: "mode:sensors"
                │
runner_node:    │
  1. /finish_trajectory (any active)
  2. Kill MARL (if running)
  3. Set mode → SENSORS_ONLY
```

**Result:** No active trajectory. cartographer_node stays alive (idle). Sensors + NavOA stay on.

### Mode: SHUTDOWN

```
Web: "shutdown"
                │
runner_node:    │
  1. Finish active cartographer trajectory (idle)
  2. pkill move_base, map_server, amcl (NavOA)
  3. Kill MARL (if running)
  4. Set mode → SHUTTING_DOWN
  5. rospy.signal_shutdown()
```

**Result:** Cartographer idle (no active trajectory). NavOA processes killed.
MARL killed. Sensors + cartographer_node process stay alive (they exit when
roslaunch gets Ctrl+C).

### Mode: AUTO_MAPPING

```
Web: "mode:auto_mapping"
                │
runner_node:    │
  1. Ensure mapping trajectory active
  2. Launch MARL exploration subprocess
  3. Set mode → AUTO_MAPPING
```

---

## Why It's Seamless

Cartographer's **pose graph** keeps all trajectory data in memory. When you finish trajectory 0, its submaps stay in the pose graph as `FINISHED` state. When you start trajectory 1, it creates constraints against trajectory 0's submaps — same mechanism as loading a `.pbstream` file, but in-memory with no disk I/O.

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

---

## The Invariant

| Component | Started by | Can be killed? |
|-----------|-----------|----------------|
| lidar_node | roslaunch (top-level) | ❌ NEVER |
| rf2o_laser_odometry | roslaunch (top-level) | ❌ NEVER |
| camera | roslaunch (top-level) | ❌ NEVER |
| serial bridge | roslaunch (top-level) | ❌ NEVER |
| imu_mux | roslaunch (top-level) | ❌ NEVER |
| NavOA (move_base + controllers) | roslaunch (top-level) | ✅ on `shutdown` only |
| cartographer_node | roslaunch (top-level) | ❌ NEVER |
| runner_status_state | roslaunch (top-level) | ✅ restarts if crashes |
| runner_manager | roslaunch (top-level) | ✅ restarts if crashes |
| MARL exploration | **subprocess** of runner_node | ✅ killed on mode switch |

Mode switches change only which **Cartographer trajectory** is active. Nothing is killed or restarted. The robot keeps driving and sensing throughout.

---

## JSON Status Format

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

**Trajectory states:** `0=ACTIVE`, `1=FINISHED`, `2=FROZEN`, `3=DELETED`

---

## Files

| File | Purpose |
|------|---------|
| `launch/full.launch` | Top-level launcher with mode selection + manager mode |
| `launch/sensors.launch` | Always-on: lidar, camera, ultrasonic, rf2o, serial bridge, IMU mux, NavOA |
| `launch/mapping.launch` | Sensors + Cartographer SLAM |
| `launch/localization.launch` | Sensors + Cartographer localization + map_server |
| `launch/auto_mapping.launch` | Sensors + Cartographer SLAM + NavOA + MARL exploration |
| `script/runner_node.py` | System lifecycle manager: topic command handler + trajectory ops + JSON status |
| `script/mode_manager.py` | Alias for runner_node.py |
| `script/status_state.py` | Enriched status publisher from real ROS topics |
| `script/run.sh` | Shell convenience launcher |

## Dependencies

- lidar_ros (LiDAR driver)
- NavOA_ros (navigation stack, serial bridge, controllers)
- marl_mapping (autonomous exploration)
- cartographer_ros (SLAM and localization)
- rf2o_laser_odometry (laser odometry)
- rosbridge_server (FastAPI integration via roslibpy)
