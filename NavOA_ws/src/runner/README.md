# runner — SlamNav System Launcher

The central orchestrator package for the SlamNav robot workspace. It launches
and coordinates all other packages into coherent system modes, and provides
a topic-based command interface for external applications (FastAPI, mobile app, etc.)
via `rosbridge_server`.

## Quick Start

```bash
# Source the workspace
source /opt/ros/noetic/setup.bash
source ~/Robit_SlamNav/NavOA_ws/devel_isolated/setup.bash
conda activate control
# Mapping mode (build a new map)
roslaunch runner mapping.launch

# Localization + Navigation mode (use pre-built map)
roslaunch runner localization.launch

# Autonomous mapping mode (SLAM + navigation + MARL exploration)
roslaunch runner auto_mapping.launch

# Sensors only (debug / calibration)
roslaunch runner sensors.launch
```

Or use `full.launch` with a `mode` argument:

```bash
roslaunch runner full.launch mode:=manager 
roslaunch runner full.launch mode:=mapping
roslaunch runner full.launch mode:=localization
roslaunch runner full.launch mode:=auto_mapping
roslaunch runner full.launch mode:=sensors
```

Or use the shell convenience script:

```bash
./src/runner/script/run.sh mapping
```

## System Modes

| Mode | Sensors | Cartographer | NavOA | MARL | Use Case |
|------|---------|--------------|-------|------|----------|
| `sensors` | ✓ | — | — | — | Debug sensor stack |
| `mapping` | ✓ | SLAM | — | — | Build a new map |
| `localization` | ✓ | Localization | Navigation | — | Navigate with existing map |
| `auto_mapping` | ✓ | SLAM | Navigation | Exploration | Autonomous exploration |
| `manager` | ✓ | dynamic | dynamic | dynamic | Runtime mode switching via `/runner/command` |

## Launch File Architecture

```
runner/
├── launch/
│   ├── sensors.launch       # Base: lidar + camera + ultrasonic + serial + rf2o + imu
│   ├── mapping.launch       # Sensors + Cartographer SLAM + NavOA nav stack
│   ├── localization.launch  # Sensors + Cartographer localization + NavOA nav stack
│   ├── auto_mapping.launch   # Sensors + Cartographer SLAM + NavOA + MARL exploration
│   └── full.launch          # Top-level: mode argument → any of the above
├── script/
│   ├── runner_node.py       # **ROS node: command handler + trajectory mgmt + JSON status**
│   ├── mode_manager.py      # Alias for runner_node.py (used by full.launch)
│   ├── status_state.py      # Enriched status publisher (/localization_status, /Nav_status, etc.)
│   └── run.sh               # Shell convenience launcher
├── param/                   # (reserved for future YAML parameter files)
├── CMakeLists.txt
└── package.xml
```

## Programmatic Control (Manager Mode)

```bash
roslaunch runner full.launch mode:=manager
```

This starts sensors + `runner_status_state` + the runner manager node.

### Command Protocol

Publish to `/runner/command` (`std_msgs/String`):

| Command | Action | Requires Cartographer? |
|---------|--------|----------------------|
| `mode:mapping` or `start_mapping` | Switch to mapping mode via trajectory services | No (launches if needed) |
| `mode:localization` or `stop_mapping` | Save map + finish mapping + start localization + launch NavOA | Yes |
| `mode:auto_mapping` | Mapping + NavOA + MARL exploration | No (launches if needed) |
| `mode:sensors` | Kill all cartographer / NavOA / MARL processes | No |
| `status` | Publish fresh JSON status on `/runner/status` | No |
| `save_map [name]` | Save current map; name is optional (auto-timestamped) | Yes |
| `list_maps` | List saved `.pbstream` files in JSON status | No |
| `shutdown` | Gracefully stop all managed processes | No |

### JSON Status (published on `/runner/status`)

```json
{
  "mode": "MAPPING | LOCALIZATION | SENSORS_ONLY | AUTO_MAPPING | SWITCHING | ERROR",
  "timestamp": 1783758959.582,
  "trajectory": {
    "active_id": 0,
    "all_states": [1, 0]
  },
  "loc_status": "LOCALIZED HIGH | LOCALIZED LOW | LOST | UNKNOWN",
  "loc_score": 0.85,
  "map_status": "MAPPING ON | LOCALIZATION ON | INITIALIZING | LOST | UNKNOWN",
  "nav_status": "IDLE | NAVIGATING | ARRIVED | FAILED | CANCELLED | UNKNOWN",
  "oob_status": "INSIDE_MAP | NEAR_BOUNDARY | OUT_OF_MAP | UNKNOWN",
  "maps": ["map_123.pbstream", "Office_map.pbstream"],
  "cartographer_alive": true,
  "error": ""
}
```

**Field reference:**

| Field | Source | Description |
|-------|--------|-------------|
| `mode` | `runner_manager` | Current system mode string |
| `trajectory.active_id` | `/get_trajectory_states` | ID of the active Cartographer trajectory, or `null` |
| `trajectory.all_states` | `/get_trajectory_states` | State values: 0=ACTIVE, 1=FINISHED, 2=FROZEN, 3=DELETED |
| `loc_status` | `runner_status_state` | Derives from `/matching_score` + timeout detection |
| `loc_score` | `/matching_score` | Raw Cartographer scan matching quality (0.0–1.0) |
| `map_status` | `runner_status_state` | Derives from trajectory states + `/app/map_status` |
| `nav_status` | `runner_status_state` | Derives from `/move_base/status` + cmd_vel activity |
| `oob_status` | `runner_status_state` | Derives from `/map` OccupancyGrid + TF (boundary detection) |
| `maps` | Filesystem | List of `.pbstream` files in `map_save_dir` |
| `cartographer_alive` | Service probe | Whether `/get_trajectory_states` responds |

### Simple Mode String (published on `/runner/mode`)

For backward compatibility or simple use cases, a plain string is also published:

```
MAPPING | LOCALIZATION | SENSORS_ONLY | AUTO_MAPPING | SWITCHING | ERROR
```

## Shell Aliases for Testing

Add these to `~/.bash_aliases` or `~/.bashrc` for quick CLI testing:

```bash
# Source the workspace
alias srs='source /opt/ros/noetic/setup.bash && source ~/Robit_SlamNav/NavOA_ws/devel_isolated/setup.bash'

# ---- runner_manager commands ----
alias r.status='rostopic pub /runner/command std_msgs/String "data: '\''status'\''" -1; sleep 0.5; rostopic echo /runner/status -n1 | head -10'
alias r.mode='rostopic pub /runner/command std_msgs/String "data: '\''status'\''" -1; sleep 0.5; rostopic echo /runner/mode -n1'

alias r.map='rostopic pub /runner/command std_msgs/String "data: '\''mode:mapping'\''" -1'
alias r.loc='rostopic pub /runner/command std_msgs/String "data: '\''stop_mapping'\''" -1'
alias r.sensors='rostopic pub /runner/command std_msgs/String "data: '\''mode:sensors'\''" -1'
alias r.explore='rostopic pub /runner/command std_msgs/String "data: '\''mode:auto_mapping'\''" -1'

alias r.save='rostopic pub /runner/command std_msgs/String "data: '\''save_map'\''" -1'
alias r.lsmap='rostopic pub /runner/command std_msgs/String "data: '\''list_maps'\''" -1'
alias r.shutdown='rostopic pub /runner/command std_msgs/String "data: '\''shutdown'\''" -1'

# ---- Direct Cartographer service tests ----
alias c.traj='rosservice call /get_trajectory_states'
alias c.score='rostopic echo /matching_score -n1'
alias c.finish='rosservice call /finish_trajectory "{trajectory_id: 0}"'

# ---- Launch shortcuts ----
alias l.manager='roslaunch runner full.launch mode:=manager'
alias l.map='roslaunch runner mapping.launch'
alias l.loc='roslaunch runner localization.launch'
alias l.sensors='roslaunch runner sensors.launch'

# ---- Status topics ----
alias s.score='rostopic echo /matching_score -n1 --noarr'
alias s.loc='rostopic echo /localization_status -n1'
alias s.map='rostopic echo /Mapping_status -n1'
alias s.nav='rostopic echo /Nav_status -n1'
alias s.oob='rostopic echo /monitor_map_boundary -n1'
alias s.json='rostopic echo /runner/status -n1 | python3 -c "import sys,json; data=sys.stdin.read(); [print(json.dumps(json.loads(l.split(chr(34))[1]),indent=2)) for l in data.split(chr(10)) if l.startswith(chr(34))]" 2>/dev/null || rostopic echo /runner/status -n1'

# ---- TMUX detached launcher ----
alias run.map='roslaunch runner mapping.launch'
alias run.loc='roslaunch runner localization.launch'
```

## Runtime Mode Switching (How It Works)

The `runner_node.py` uses **Cartographer trajectory services** for seamless mode transitions
instead of killing and restarting the node:

### stop_mapping (Map → Localize)

```
1. TF lookup map→base_link   → save current pose
2. /write_state                    → save .pbstream to disk
3. /get_trajectory_states          → find a stable reference trajectory
4. /finish_trajectory              → stop the mapping trajectory
5. /start_trajectory               → start localization with livox_2d_localization.lua
                                     (pure_localization_trimmer keeps only 3 recent submaps)
6. roslaunch NavOA nav stack       → navigation ready
```

### start_mapping (Localize → Map)

```
1. /finish_trajectory              → stop the current trajectory
2. /start_trajectory               → start mapping with livox_2d.lua
                                     (no trimmer — all submaps preserved)
3. Kill NavOA nav stack            → navigation not needed during mapping
```

**Key detail**: The `_get_reference_trajectory_id()` method always picks a state=FINISHED
trajectory (typically trajectory 0, the first mapping run) as the `relative_to` frame.
This avoids the bug where a `PureLocalizationTrimmer` deletes a finished trajectory,
making it invalid as a reference.

## Rosbridge / FastAPI Integration

Since `rosbridge_server` and `roslibpy` are pre-installed, connecting from FastAPI
is straightforward:

```python
from roslibpy import Ros, Topic

client = Ros(robot_ip, port=9090)
client.run()

cmd_pub = Topic(client, '/runner/command', 'std_msgs/String')

@router.post('/api/mode/mapping')
async def start_mapping():
    cmd_pub.publish({'data': 'mode:mapping'})
    return {"status": "ok"}

@router.post('/api/mode/stop-mapping')
async def stop_mapping():
    cmd_pub.publish({'data': 'stop_mapping'})
    return {"status": "ok"}

@router.get('/api/status')
async def get_status():
    # Subscribe to /runner/status and return the latest JSON
    ...
```

**Start rosbridge**: `roslaunch rosbridge_server rosbridge_websocket.launch`

## Code Reference

### Key Files

| File | Lines | Purpose |
|------|-------|---------|
| `runner_node.py` | ~788 | Main lifecycle manager: command handler, trajectory ops, status publishing, process mgmt |
| `status_state.py` | ~618 | Enriched status derivation from real ROS topics (matching_score, move_base, /map) |
| `mode_manager.py` | ~35 | Thin alias that imports and runs `RunnerNode` |
| `full.launch` | ~60 | Top-level launcher; `mode:=manager` starts sensors + status_state + runner_manager |

### Trajectory State Constants (from TrajectoryStates.msg)

```python
ACTIVE  = 0   # Trajectory accepting data, building submaps
FINISHED = 1  # Trajectory stopped, submaps preserved in pose graph
FROZEN  = 2   # Loaded from .pbstream, not optimized further
DELETED = 3   # Removed by PureLocalizationTrimmer after finish
```

### Localization Status Derivation (status_state.py)

| /matching_score | Duration | Result |
|----------------|----------|--------|
| ≥ 0.7 | — | `LOCALIZED HIGH` |
| ≥ 0.3, < 0.7 | — | `LOCALIZED LOW` |
| < 0.3 | — | `LOST` |
| any | > 5s since last score | `LOST` |

### Navigation Status Derivation (status_state.py)

| Source | Condition | Result |
|--------|-----------|--------|
| move_base GoalStatus | `ACTIVE` | `NAVIGATING` |
| move_base GoalStatus | `SUCCEEDED` | `ARRIVED` |
| move_base GoalStatus | `ABORTED` | `FAILED` |
| move_base GoalStatus | `PREEMPTED` | `CANCELLED` |
| cmd_vel | > 2s since last non-zero | `IDLE` |
| cmd_vel | ≤ 2s since last non-zero | `NAVIGATING` |

### Map Boundary Status (status_state.py)

Checks robot position against `/map` OccupancyGrid bounding box + cell values:

| Condition | Result |
|-----------|--------|
| Robot outside mapped area | `OUT_OF_MAP` |
| < 0.5m from edge | `NEAR_BOUNDARY` |
| Inside known area | `INSIDE_MAP` |

## Notes / Troubleshooting

### Cartographer APIs Unavailable
The `runner_node.py` handles missing `cartographer_ros_msgs` gracefully — the
`_HAS_CARTO` flag degrades trajectory operations to fallbacks (kill/relaunch node).
Status subscriptions (`/matching_score`, `/localization_status`) continue to work.

### Multiple Trajectories and PureLocalizationTrimmer
When a localization trajectory is finished, its `PureLocalizationTrimmer` deletes
all its submaps and sets state → DELETED. Always use a stable FINISHED trajectory
(trajectory 0, which has no trimmer) as the `relative_to` reference when starting
a new trajectory via `/start_trajectory`. The `_get_reference_trajectory_id()`
method handles this automatically.

### TF availability for mode switches
`stop_mapping` requires a valid `map → base_link` TF transform. If the
lidar is blocked or rf2o hasn't initialized, the switch will abort with
`MODE_ERROR`. Wait for `rostopic echo /matching_score` to show a value before
triggering mode switches.

### Logging
All runner nodes use `[Runner]` prefix for log messages. View logs:
```bash
# Real-time filtering
rosrun runner mode_manager.py 2>&1 | grep "\[Runner\]"

# Or via rqt_console
rqt_console
```

## Dependencies

- lidar_ros (LiDAR driver + full sensor stack)
- NavOA_ros (navigation stack, serial bridge, controllers)
- marl_mapping (autonomous exploration)
- cartographer_ros (SLAM and localization)
- rf2o_laser_odometry (laser odometry)
