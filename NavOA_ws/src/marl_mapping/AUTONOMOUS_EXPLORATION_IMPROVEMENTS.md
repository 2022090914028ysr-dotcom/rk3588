# Autonomous Exploration Functionality Check and Improvement Notes

This document summarizes the inspection results, completed fixes, progress display feature, parameter configuration, and verification status of the `marl_mapping` autonomous exploration node.

## Files Involved

- `launch/exploration.launch`
- `scripts/exploration_node.py`
- `package.xml`
- `CMakeLists.txt`

## Improvement Overview

The following improvements have been completed:

1. Actively cancel move_base navigation goal when stopping exploration.
2. Fix loop closure revisit goals being prematurely aborted by frontier checks.
3. Re-project historical grid states when map origin, resolution, or size changes.
4. Align move_base global costmap to the expanded internal map.
5. Adjust default goal arrival tolerance from 1.5 m to 0.3 m.
6. Add exploration progress bar, status display, elapsed time, and ETA estimation.

---

## 1. Cancel Navigation Goal When Stopping Exploration

### Original Problem

When the node stopped exploration, it only cleared its own `current_goal_world` without notifying move_base. After reaching the maximum steps, coverage threshold, or loop closure completion, the robot could still continue executing the previous navigation goal.

### Improvement

- Added `/move_base/cancel` publisher.
- Uses an empty `actionlib_msgs/GoalID` to cancel move_base's current goal.
- Actively cancels navigation when `stop_exploration()` stops.
- Checks again via shutdown callback when the ROS node shuts down, preventing the robot from continuing to drive after node exit.
- Added configurable parameter `cancel_topic`.

Default configuration:

```xml
<arg name="cancel_topic" default="/move_base/cancel" />
```

Since `GoalID` messages were added, `package.xml` and `CMakeLists.txt` now include the `actionlib_msgs` dependency.

## 2. Fix Loop Closure Revisit Being Prematurely Aborted

### Original Problem

During normal frontier exploration, the node periodically checks whether frontiers still exist near the goal; if the frontier disappears, it reselects a goal.

Loop closure goals are located in already-explored regions where frontiers typically do not exist, so revisit goals would also be misjudged as invalid by the same check and aborted within the default 5-second check period.

### Improvement

Frontier survival checks are skipped when `_loop_closure_phase` is true. Loop closure goals now continue executing until:

- The goal is reached;
- Navigation timeout;
- Global planner confirms the path is unplannable;
- The exploration process is stopped.

## 3. State Re-projection After Map Metadata Changes

### Original Problem

Cartographer may modify the OccupancyGrid's:

- `origin`;
- `resolution`;
- `width` and `height`.

during map expansion.

Historical goals were previously saved in grid coordinates. If the map origin changes, old grid coordinates no longer correspond to their original world positions, potentially causing:

- Incorrect visited goal filter positions;
- Incorrect failed goal filter positions;
- Frontiers near the current goal being misjudged as disappeared;
- Regional exploration bias toward incorrect positions;
- Loop closure revisiting incorrect positions.

### Improvement

When map metadata changes, the node processes cached state as follows:

```text
Old grid coordinates -> Old map world coordinates -> New map grid coordinates
```

Re-projection targets include:

- `visited_goals`;
- `_failed_frontiers`;
- `_last_goal_grid`;
- `_loop_closure_history`;
- `_loop_closure_visited_now`.

Loop closure history simultaneously preserves world coordinates, which serve as the basis for recalculating grid positions.

## 4. Global Costmap Alignment with Expanded Map

### Original Problem

`exploration.launch` default setting:

```xml
<arg name="map_padding_cells" default="400" />
```

The exploration node adds 400 unknown cells on each side of the original `/map`, but move_base global costmap still uses the original `/map` dimensions. The old logic only used the costmap when costmap and internal map dimensions were identical, so under the default configuration it almost always fell back to manual obstacle inflation.

### Improvement

The node now saves the full metadata of the global costmap:

- origin;
- resolution;
- frame;
- width and height;
- origin yaw.

It then embeds or resamples the costmap into the exploration node's internal map grid based on world coordinate relationships:

- Fast slice copy when resolution is identical and offset is an integer grid multiple;
- Axis-aligned nearest-neighbor resampling when resolution or origin has fractional offsets;
- Regions in the internal map padding without costmap data keep the unknown value `-1`;
- Costmap free mask is intersected with SLAM-explored regions and non-obstacle regions; unknown areas are not mistakenly treated as traversable;
- Safely degrades to manual obstacle inflation when costmap frame is inconsistent or has non-zero rotation, with a warning logged.

Loop closure candidate points now require costmap values strictly equal to `0`; unknown values `-1` are no longer treated as reachable areas.

## 5. Adjust Goal Arrival Tolerance

### Original Problem

The launch default `goal_tolerance` was 1.5 m. The node only judges arrival based on the planar distance between the robot and the goal, so the robot could mark a goal as visited while still being 1.5 m from the frontier, causing missed exploration.

### Improvement

The code default and launch default have been unified and adjusted to:

```xml
<arg name="goal_tolerance" default="0.3" />
```

This parameter can still be overridden based on robot size, localization error, and navigation control accuracy.

---

## Exploration Progress Bar and ETA

The node now periodically outputs an exploration status bar to the terminal and publishes the same content to a ROS topic.

Example:

```text
[ExploreProgress] [############------------------] 42.1% | coverage=40.0/95.0% | state=NAVIGATING | elapsed=00:08:15 | ETA=00:11:22 | step=18/500
```

Field descriptions:

| Field | Meaning |
| --- | --- |
| Progress bar and percentage | Normalized completion progress relative to target coverage |
| `coverage` | Current map coverage / target coverage |
| `state` | Current exploration state |
| `elapsed` | Time elapsed since this exploration run started |
| `ETA` | Estimated remaining time based on recent progress changes |
| `step` | Goals sent / maximum goals |

### State Values

| State | Meaning |
| --- | --- |
| `INITIALIZING` | Node initializing |
| `WAITING_MAP` | Waiting for SLAM map |
| `STARTING` | Waiting for navigation stack initialization |
| `WAITING_REGION` | Waiting for user to close the exploration region polygon |
| `SELECTING_GOAL` | Detecting frontiers and selecting goal |
| `NAVIGATING` | Navigating to frontier goal |
| `UPDATING_MAP` | Waiting for map update after reaching goal |
| `WAITING_FRONTIERS` | No frontiers available, waiting for new map data |
| `STUCK_RETRY` | No reachable goal, in retry phase |
| `LOOP_CLOSURE` | Executing loop closure revisit |
| `COMPLETED` | Target coverage reached |
| `STOPPED` | Stopped due to other termination conditions |

### ETA Algorithm

ETA uses recent exploration progress samples for least-squares slope estimation, rather than simply comparing two adjacent map frames, thereby reducing errors caused by map update jitter.

ETA displays as `--:--:--` when:

- Insufficient valid samples;
- Sampling time is less than 10 seconds;
- No valid exploration progress recently;
- Dynamic map expansion causes coverage decrease;
- Estimation result exceeds reasonable range.

When the map reaches the target coverage, ETA displays as `00:00:00`.

### Progress Parameters

| Parameter | Default | Description |
| --- | ---: | --- |
| `progress_enabled` | `true` | Whether to enable the progress status bar |
| `progress_topic` | `/exploration/progress` | Progress text publish topic |
| `progress_update_interval` | `5.0` | Update interval in seconds |
| `progress_bar_width` | `30` | ASCII progress bar character width |
| `progress_eta_window` | `120.0` | ETA sampling window in seconds |
| `exploration_target_ratio` | `0.95` | Exploration completion target coverage |

View progress topic:

```bash
rostopic echo /exploration/progress
```

Custom launch example:

```bash
roslaunch marl_mapping exploration.launch \
  progress_update_interval:=2.0 \
  progress_bar_width:=40 \
  exploration_target_ratio:=0.95
```

## Completed Verification

The following static and targeted tests have been performed:

- `exploration_node.py` Python AST parsing;
- `exploration.launch` XML parsing;
- `package.xml` XML parsing;
- Launch progress parameters and default value checks;
- Costmap embedding test with padding of 400;
- Historical grid state re-projection test after map origin change;
- ETA linear growth estimation test;
- Time formatting test;
- State derivation tests for `NAVIGATING`, `LOOP_CLOSURE`, etc.;
- New `actionlib_msgs` dependency check.

The current check environment does not have `catkin_make` or a full ROS runtime, so ROS builds and on-robot navigation integration testing have not yet been performed.

## ROS Environment Integration Testing Suggestions

On a ROS robot or in a ROS workspace, check the following in sequence:

```bash
# Rebuild
catkin build
source devel/setup.bash
roslaunch marl_mapping exploration.launch
```

Verify key topics:

```bash
rostopic hz /map
rostopic hz /move_base/global_costmap/costmap
rostopic echo /exploration/progress
rostopic info /move_base/cancel
```

Recommended key verification items:

1. After clicking and closing the exploration region, state transitions from `WAITING_REGION` to `SELECTING_GOAL` or `NAVIGATING`.
2. With padding of 400, logs show aligned costmap usage rather than continuously degrading to safe free map.
3. After Cartographer map origin changes, the robot does not suddenly navigate toward incorrectly positioned historical goals.
4. After entering the loop closure phase, revisit goals are not automatically aborted within the frontier update cycle.
5. When stop conditions are met or the node shuts down, move_base's current goal is cancelled.
6. After coverage grows steadily for a period, ETA transitions from `--:--:--` to a valid time.

## Prerequisites and Limitations

- `exploration.launch` only starts the autonomous exploration node; it does not start Cartographer, move_base, or NavOA. The map, TF, and navigation stack must already be available before running.
- ETA is based on current OccupancyGrid coverage. Dynamic SLAM map expansion changes the statistical denominator, so ETA is a trend estimate, not a strict completion time.
- When the global costmap uses a non-`map` frame or has a rotated origin, the node chooses safe degradation and does not perform potentially incorrect coordinate indexing.
