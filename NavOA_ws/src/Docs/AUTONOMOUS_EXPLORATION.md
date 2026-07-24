# Autonomous Exploration — MARL Mapping

> **Date:** 2026-07-24  
> **Package:** `marl_mapping`  
> **Location:** `/home/ubuntu/Robit_SlamNav/NavOA_ws/src/marl_mapping/`

---

## 1. Overview

MARL Mapping provides **frontier-based autonomous exploration** for unknown environments. Despite its name (Multi-Agent Reinforcement Learning), the current deployment uses a **single-robot, deterministic frontier exploration** approach derived from the NeuralCoMapping project (CVPR 2022).

The exploration node detects **frontiers** (boundary cells between known-free and unknown space) in the occupancy grid, ranks them by configurable criteria, and publishes the best frontier as a navigation goal to `move_base`.

**Key Design:** MARL does NOT directly publish `cmd_vel`. It publishes `PoseStamped` goals to `/move_base_simple/goal`, leveraging the full NavOA navigation stack for path planning, obstacle avoidance, and safety.

---

## 2. Package Structure

```
marl_mapping/
├── launch/
│   ├── exploration.launch        ← Standalone exploration
│   └── nav_exploration.launch    ← Navigation + exploration
├── config/
│   └── exploration.yaml          ← Exploration parameters
├── scripts/
│   ├── exploration_node.py       ← Main ROS node (1133 lines)
│   ├── robot_interface.py        ← Standalone test interface
│   └── add_region_point.py       ← CLI region point helper
└── src/marl_mapping/
    ├── __init__.py
    ├── utils/
    │   ├── frontier.py           ← Frontier detection (OpenCV)
    │   ├── fmm_planner.py        ← Fast Marching Method / Dijkstra
    │   └── map_builder.py        ← OccupancyGrid processing
    └── env/
        ├── robot_interface.py    ← SingleRobot class (ROS interface)
        └── exploration_env.py    ← RL environment wrapper
```

---

## 3. Exploration Node (exploration_node.py)

### 3.1 Architecture

```
exploration_node.py
│
├── Subscribers:
│   ├── /map (OccupancyGrid)                 ← Current map
│   ├── /odom_rf2o (Odometry)                ← Robot pose
│   ├── /move_base/GlobalPlanner/plan (Path) ← Current global plan
│   ├── /move_base/global_costmap/costmap (OccupancyGrid) ← Costmap (optional)
│   └── /clicked_point (PointStamped)        ← Region of interest from RViz
│
├── Publishers:
│   └── /move_base_simple/goal (PoseStamped) ← Exploration goal
│
├── Timer Callback:
│   └── exploration_loop() → runs at configurable rate
│
└── State:
    ├── current_goal: PoseStamped
    ├── visited_frontiers: list of (x, y, timestamp)
    ├── region_points: list of (x, y) from user clicks
    └── idle_time: time since last frontier found
```

### 3.2 Frontier Detection Algorithm

**File:** `src/marl_mapping/utils/frontier.py`

```
Input: OccupancyGrid (ROS)

1. Convert grid to OpenCV image:
   - Free space: 0
   - Occupied: 255
   - Unknown: 128

2. Find frontier cells:
   - Unknown cell (128) with at least one free neighbor (0)

3. Cluster frontiers using OpenCV connectedComponentsWithStats:
   - Each cluster returns: (cx, cy, size, white_border)
   - white_border = number of free cells on frontier boundary

4. Filter clusters by min_size (default: 5 cells)

5. Rank frontiers by selected method

6. Select best frontier → publish as goal
```

### 3.3 Frontier Ranking Methods

| Method | Selection Criteria | Use Case |
|--------|-------------------|----------|
| `nearest` | Closest to robot position | General exploration |
| `largest` | Largest frontier (most cells) | Maximize area coverage |
| `whitest` | Largest free border | Discover new areas quickly |
| `combined` | Weighted combination of nearest + largest + whitest | Balanced exploration |

### 3.4 Goal Processing

Before publishing a frontier as a goal:

1. **Snap to free cell**: Uses `_snap_to_free_cell()` with BFS search to ensure the goal is in traversable space (within 3 cell radius)

2. **Coordinate transform**: Converts frontier center from map frame to world coordinates

3. **Pose orientation**: Sets goal orientation to face the frontier center from the robot's position

4. **Timeout handling**: If no frontiers found for 20+ seconds, triggers loop closure revisit

### 3.5 Loop Closure

When the robot has been idle for 20+ seconds (no frontiers found in current map):
- Selects up to 5 previously visited frontier locations
- Publishes them as goals to trigger Cartographer loop closure
- This helps Cartographer optimize the pose graph and potentially reveal new frontiers after optimization

### 3.6 Planner Monitoring

The node monitors `/move_base/GlobalPlanner/plan`:
- If the GlobalPlanner publishes an empty path for 3 consecutive cycles → switch to next best frontier
- This prevents the robot from getting stuck on unreachable frontiers

### 3.7 Region Filtering

Users can define regions of interest via RViz `/clicked_point`:
- **Convex hull mode**: Points define a convex polygon; only frontiers inside are explored
- **Closed polygon mode**: Same as convex hull but with explicit boundary
- **Add/clear regions**: CLI tool `add_region_point.py` or RViz point publishing

---

## 4. Configuration

### 4.1 exploration.yaml

```yaml
# General
map_width: 25            # meters
map_height: 25           # meters
map_resolution: 0.05     # meters/pixel
exploration_method: "nearest"  # nearest, largest, whitest, combined

# Frontier Detection
frontier_min_size: 5     # minimum cells per frontier cluster

# Goal Management
goal_tolerance: 1.5      # meters, distance to consider goal reached
goal_timeout: 30.0       # seconds without progress → switch goal

# Loop Closure
idle_timeout: 20.0       # seconds idle → trigger loop closure
loop_closure_points: 5   # number of past points to revisit

# Planner Monitor
empty_plan_retries: 3    # consecutive empty plans → switch goal

# Exploration Loop
exploration_rate: 0.2    # Hz (5 seconds between cycles)
```

### 4.2 Launch File Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `exploration_method` | `nearest` | Frontier ranking method |
| `enable_exploration` | `true` | Enable/disable exploration |
| `enable_loop_closure` | `true` | Enable loop closure after idle |
| `goal_topic` | `(empty)` | Override goal topic |

---

## 5. Integration with the System

### 5.1 In runner_node.py (subprocess)

```
runner_node:
  │
  ├── mode:auto_mapping
  │     └── _launch_exploration()
  │           └── subprocess.Popen("roslaunch marl_mapping exploration.launch")
  │                 └── exploration_node.py
  │                       └── publishes goals to /move_base_simple/goal
  │                             └── move_base plans → NavOA navigates
  │
  └── mode:mapping / mode:localization / mode:sensors / shutdown
        └── _kill_exploration()
              └── SIGTERM → 3s → SIGKILL
```

### 5.2 Data Dependencies

| Data | Topic | Used For |
|------|-------|----------|
| Occupancy grid | `/map` | Frontier detection |
| Robot pose | `/odom_rf2o` | Robot position, nearest-frontier calculation |
| Global plan | `/move_base/GlobalPlanner/plan` | Planner monitoring |
| Clicked point | `/clicked_point` | Region of interest definition |

### 5.3 Output

| Topic | Type | Description |
|-------|------|-------------|
| `/move_base_simple/goal` | PoseStamped | Next frontier navigation goal |

---

## 6. RL Environment (ExplorationEnv)

**File:** `src/marl_mapping/env/exploration_env.py`

While the current deployment uses deterministic frontier exploration, the package also includes a `RealWorldExplorationEnv` class that wraps the robot as a reinforcement learning environment:

```python
class RealWorldExplorationEnv:
    def reset(self):
        """Reset exploration state."""
        pass

    def step(self, action):
        """Execute action → return (observation, reward, done, info)."""
        pass

    def get_current_state(self):
        """Return current map + pose observation."""
        pass

    def get_global_reward(self):
        """Compute exploration reward (coverage gain)."""
        pass
```

This environment is compatible with MARL (Multi-Agent RL) training pipelines but is **not used in the current deployment**.

---

## 7. Exploration Launch Modes

### 7.1 standalone (via auto_mapping.launch)

```
roslaunch runner auto_mapping.launch
```
- Starts sensors + Cartographer mapping + MARL exploration
- Default mode for autonomous mapping missions

### 7.2 standalone (direct)

```
roslaunch marl_mapping exploration.launch
```
- Exploration only (assumes sensors + Cartographer already running)

### 7.3 nav_exploration

```
roslaunch marl_mapping nav_exploration.launch
```
- Includes NavOA navigation stack + exploration
- Alternative when not using full.launch mode=manager

---

## 8. Performance Notes

- **Frontier detection rate:** Every 5 seconds (0.2 Hz) while moving
- **Computational cost:** Low (OpenCV connected components on ~500×500 pixel map)
- **Goal switching:** Happens when robot reaches goal tolerance (1.5m) or goal times out (30s)
- **Loop closure triggering:** After 20s idle, revisits up to 5 past frontier locations
- **Region filtering:** User-defined polygons via RViz click points
