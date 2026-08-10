# Runner Orchestrator — System Lifecycle Management

> **Date:** 2026-07-24  
> **Package:** `runner`  
> **Location:** `/home/ubuntu/Robit_SlamNav/NavOA_ws/src/runner/`

---

## 1. Purpose

The `runner` package is the central orchestrator for the entire SlamNav system. It:

1. Provides the top-level launch file (`full.launch`) that brings up all subsystems
2. Implements mode switching via Cartographer trajectory services (`runner_node.py`)
3. Publishes enriched system status derived from multiple ROS topics (`status_state.py`)
4. Manages the MARL exploration subprocess (the only dynamically managed process)
5. Exposes a topic-based command interface for external applications (FastAPI/web)

---

## 2. Launch Architecture

### 2.1 full.launch — Top-Level Entry Point

**Usage:** `roslaunch runner full.launch mode:=manager`

**Arguments:**

| Argument | Default | Values |
|----------|---------|--------|
| `mode` | `manager` | manager, mapping, localization, auto_mapping, sensors |
| `use_camera` | `false` | true, false |
| `use_ultrasonic` | `true` | true, false |
| `launch_rviz` | `false` | true, false |

**Mode Behavior:**

| Mode | Includes | Cartographer State | MARL |
|------|----------|-------------------|------|
| `manager` | sensors.launch + cartographer_node + status_state + runner_node | Trajectories managed by runner_node | Subprocess (on demand) |
| `mapping` | mapping.launch (cartographer only) | Mapping trajectory | Off |
| `localization` | sensors.launch + localization.launch | Localization trajectory + map_server | Off |
| `auto_mapping` | sensors.launch + cartographer + MARL | Mapping trajectory | Launched directly |
| `sensors` | sensors.launch | No cartographer | Off |

**The `mode:=manager` mode is the primary production mode** — it starts everything and lets `runner_node.py` handle mode transitions via topic commands.

### 2.2 sensors.launch — Always-On Sensor Stack

**Included by:** `full.launch mode:=manager`, `localization.launch`, `auto_mapping.launch`

**Components (in order):**

```
1. ascamera (HP60C depth camera)         [conditional on use_camera]
2. lidar_node (/dev/ttyS4 → /scan)
3. static TF base_link → laser_frame
4. rf2o_laser_odometry (/odom_rf2o)
5. multi_ultrasonic                      [conditional on use_ultrasonic]
6. serial_cmd_vel_imu_bridge (/dev/ttyS1 → /imu_serial)
7. imu_mux (/imu_serial + /imu → /imu_mux)
8. NavOA navigation stack (navoa_nav_only.launch)
   ├── move_base (costmaps + GlobalPlanner)
   ├── vel_filter.py
   ├── path_segment_publisher.py
   └── NavOA status nodes
9. rosbridge_server (port 9090)
```

**Design invariant:** All components in sensors.launch run at the roslaunch top level and are **never killed**. They persist across all mode switches.

### 2.3 Launch File Dependencies

```
full.launch (mode:=manager)
  ├── sensors.launch          ← Always-on stack
  ├── cartographer_node       ← SLAM engine
  ├── cartographer_occupancy_grid_node
  ├── runner_status_state     ← status_state.py
  └── runner_manager          ← mode_manager.py (runner_node.py)

full.launch (mode:=mapping)
  └── mapping.launch
        ├── cartographer_node
        └── cartographer_occupancy_grid_node

full.launch (mode:=localization)
  └── localization.launch
        ├── sensors.launch
        ├── map_server (static .yaml map)
        ├── cartographer_node (with -load_state_filename .pbstream)
        └── cartographer_occupancy_grid_node (remaps /map → /carto_submap_viz)

full.launch (mode:=auto_mapping)
  └── auto_mapping.launch
        ├── sensors.launch
        ├── cartographer_node (mapping mode)
        ├── cartographer_occupancy_grid_node
        └── MARL exploration_node (roslaunch marl_mapping exploration.launch)
```

---

## 3. runner_node.py — System Lifecycle Manager

### 3.1 Overview

**File:** `script/runner_node.py` (792 lines)

A ROS node that acts as the central command-and-control point. It subscribes to `/runner/command` (std_msgs/String) and translates commands into Cartographer service calls and subprocess management.

### 3.2 Command Set

| Command String | Action | Cartographer Ops | MARL |
|---------------|--------|-----------------|------|
| `mode:mapping` | Switch to mapping trajectory | finish_traj → start_traj(mapping) | Kill |
| `start_mapping` | Same as mode:mapping | Same | Kill |
| `mode:localization` | Save map → localization | write_state → finish_traj → start_traj(loc) | Kill |
| `stop_mapping` | Same as mode:localization | Same | Kill |
| `mode:auto_mapping` | Mapping + MARL exploration | Ensure mapping active → launch subprocess | Launch |
| `mode:sensors` | Finish active trajectory, idle | finish_traj | Kill |
| `save_map [name]` | Save .pbstream to disk | write_state(name) | No change |
| `list_maps` | List saved .pbstream files | (scan directory) | No change |
| `status` | Publish current JSON status | (read state) | No change |
| `shutdown` | Finish trajectory, kill NavOA | finish_traj | Kill |

### 3.3 Internal Architecture

```
RunnerNode
│
├── Publishers:
│   ├── /runner/status (std_msgs/String, latched)
│   └── /runner/mode (std_msgs/String, latched)
│
├── Subscribers:
│   ├── /runner/command (std_msgs/String) — callback: on_command()
│   ├── /localization_status (String) — cached
│   ├── /Mapping_status (String) — cached
│   ├── /Nav_status (String) — cached
│   ├── /monitor_map_boundary (String) — cached
│   └── /matching_score (Float32) — cached for status
│
├── Cartographer Service Clients:
│   ├── /start_trajectory         (cartographer_ros_msgs/StartTrajectory)
│   ├── /finish_trajectory        (cartographer_ros_msgs/FinishTrajectory)
│   ├── /write_state              (cartographer_ros_msgs/WriteState)
│   └── /get_trajectory_states    (cartographer_ros_msgs/GetTrajectoryStates)
│
├── State:
│   ├── mode: str                  ← Current mode (MAPPING, LOCALIZATION, etc.)
│   ├── cartographer_alive: bool   ← Is Cartographer responding?
│   ├── loc_status: str            ← From status_state.py
│   ├── map_status: str            ← From status_state.py
│   ├── nav_status: str            ← From status_state.py
│   ├── oob_status: str            ← From status_state.py
│   ├── loc_score: float           ← From /matching_score
│   └── explore_process: Popen     ← MARL subprocess handle
│
└── Mode Switch Methods:
    ├── _switch_to_mapping()
    ├── _switch_to_localization()
    ├── _switch_to_sensors()
    ├── _launch_exploration()
    ├── _kill_exploration()
    └── _shutdown()
```

### 3.4 Trajectory Service Helpers

```python
def _get_carto_service(self, service_name, timeout=3.0):
    """Get service proxy with timeout and result check."""

def _is_carto_alive(self):
    """Probe /get_trajectory_states to verify Cartographer responsiveness."""

def _get_active_trajectory_id(self):
    """Find trajectory with state=0 (ACTIVE). Returns None if none."""

def _get_reference_trajectory_id(self):
    """Find first FINISHED trajectory for use as reference frame."""

def _call_finish(self, traj_id):
    """Call /finish_trajectory and log result."""

def _call_start(self, config, pose, ref_id):
    """Call /start_trajectory with Lua config, initial pose, reference traj."""

def _call_write_state(self, filename):
    """Call /write_state to save .pbstream."""
```

### 3.5 Subprocess Management (MARL)

The **only** subprocess managed by runner_node:

```python
def _launch_exploration(self):
    """Spawn MARL exploration as subprocess."""
    cmd = "roslaunch marl_mapping exploration.launch"
    self._explore_process = subprocess.Popen(
        cmd.split(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid  # <- creates process group for clean kill
    )

def _kill_exploration(self):
    """Kill MARL with escalation."""
    if self._explore_process is None:
        return
    pgid = os.getpgid(self._explore_process.pid)
    os.killpg(pgid, signal.SIGTERM)
    try:
        self._explore_process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(pgid, signal.SIGKILL)  # Force kill
    self._explore_process = None
```

### 3.6 Shutdown Sequence

On `shutdown` command:
1. Finish active Cartographer trajectory (idle)
2. Kill NavOA via `pkill -f move_base`, `pkill -f map_server`, `pkill -f amcl`
3. Kill MARL subprocess (if running)
4. Set mode → SHUTTING_DOWN
5. `rospy.signal_shutdown()`

**Note:** Only NavOA processes are killed. Sensors, cartographer_node, and rosbridge continue running (they exit when the roslaunch process gets Ctrl+C).

---

## 4. status_state.py — Enriched Status Publisher

### 4.1 Overview

**File:** `script/status_state.py` (618 lines)

Derives human-readable status from raw ROS topics. Publishes four separate status topics and also provides data consumed by `runner_node.py` for the JSON status.

### 4.2 Status Topics

#### /localization_status

| Condition | Value |
|-----------|-------|
| Matching score ≥ 0.7 | `"LOCALIZED HIGH"` |
| 0.3 ≤ score < 0.7 | `"LOCALIZED LOW"` |
| Score < 0.3 | `"LOST"` |
| No score for > 5 seconds | `"LOST"` |

#### /Mapping_status

| Condition | Value |
|-----------|-------|
| Any trajectory with state=ACTIVE | `"MAPPING ON"` |
| All trajectories FINISHED/FROZEN + matching score available | `"LOCALIZATION ON"` |
| Sensor data arriving but no trajectory | `"INITIALIZING"` |
| No data from any source | `"LOST"` |

#### /Nav_status

| Condition | Value |
|-----------|-------|
| `/move_base/status` shows ACTIVE | `"NAVIGATING"` |
| `/move_base/status` shows SUCCEEDED | `"ARRIVED"` |
| `/move_base/status` shows ABORTED | `"FAILED"` |
| `/move_base/status` shows PREEMPTED | `"CANCELLED"` |
| No cmd_vel on `/NavOA_cmd_vel` for > 2s | `"IDLE"` |
| Recent cmd_vel on `/NavOA_cmd_vel` | `"NAVIGATING"` |

#### /monitor_map_boundary

Uses a `MapInfo` class that wraps `nav_msgs/OccupancyGrid`:

```
MapInfo:
  - rect:    Full map bounding box
  - tight:   Bounding box of known (non-unknown) cells
  - cell_value(x, y): Occupancy at cell

Logic:
  1. Check if robot position is outside map bounding box
  2. Check distance to nearest map edge (warn at 0.5m, 0.1m hysteresis)
  3. Check if robot is on an occupied cell (>90 = OUT_OF_MAP)
  3s settle time after map switches to suppress false alerts
```

### 4.3 Configurable Parameters

All via ROS private params, env vars, or JSON config file:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `~score_ok` | 0.7 | Localization high threshold |
| `~score_warn` | 0.3 | Localization low/lost boundary |
| `~score_timeout` | 5.0s | Timeout for matching score loss |
| `~warn_distance_m` | 0.5 | Map boundary warning distance |
| `~exit_hysteresis_m` | 0.1 | Hysteresis for boundary exit |
| `~treat_unknown_as_oob` | true | Treat unknown cells as out-of-bounds |
| `~use_tight_bbox` | true | Use known-cell bounding box |
| `~map_settle_time_s` | 3.0 | Settle time after map change |
| `~oob_wall_threshold` | 90 | Occupancy threshold for OOB detection |
| `~nav_idle_timeout` | 2.0s | Timeout to declare navigation idle |

---

## 5. run.sh — Convenience Launcher

**File:** `script/run.sh`

```bash
./run.sh <mode> [detached]
```

**Modes:** `mapping`, `localization`, `auto_mapping`, `sensors`

**Features:**
- Auto-sources workspace by searching for `devel_isolated/setup.bash` → `devel/setup.bash`
- `detached` argument launches in a named tmux session (`slamnav_<mode>`)
- Validates mode name
- Prints usage with no arguments

---

## 6. External Interface Summary

### 6.1 For FastAPI / Web App Developers

**Connecting:**
```python
from roslibpy import Ros, Topic

client = Ros("robot_ip", 9090)
cmd_pub = Topic(client, "/runner/command", "std_msgs/String")
status_sub = Topic(client, "/runner/status", "std_msgs/String")
```

**Sending commands:**
```python
cmd_pub.publish({"data": "mode:mapping"})
cmd_pub.publish({"data": "mode:auto_mapping"})
cmd_pub.publish({"data": "stop_mapping"})  # save map + switch to localization
cmd_pub.publish({"data": "mode:sensors"})
cmd_pub.publish({"data": "shutdown"})
cmd_pub.publish({"data": "save_map MyMap"})
cmd_pub.publish({"data": "list_maps"})
cmd_pub.publish({"data": "status"})
```

**Reading status:**
```python
status_sub.subscribe(lambda msg: print(msg["data"]))
# Returns JSON string (see SYSTEM_OVERVIEW.md for schema)
```

### 6.2 For ROS Developers

**Mode switching (from within ROS):**
```bash
rostopic pub /runner/command std_msgs/String "mode:mapping"
rostopic pub /runner/command std_msgs/String "stop_mapping"
```

**Reading system status:**
```bash
rostopic echo /runner/status
rostopic echo /localization_status
rostopic echo /Mapping_status
rostopic echo /Nav_status
rostopic echo /monitor_map_boundary
```

**Direct Cartographer services:**
```bash
rosservice call /get_trajectory_states
rosservice call /write_state "{filename: '/path/map.pbstream'}"
```
