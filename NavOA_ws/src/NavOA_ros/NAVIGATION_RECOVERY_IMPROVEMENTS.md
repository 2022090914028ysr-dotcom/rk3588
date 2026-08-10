# NavOA Universal Navigation Recovery Improvements

This document describes the improvements made to the `navoa_nav_only.launch` navigation pipeline to address the following issues:

1. When the robot starts in a residual obstacle inflation zone, GlobalPlanner cannot generate a path.
2. After a path has been generated, if the robot is blocked for a long time by NavOA, ultrasonic, or other obstacle avoidance logic, it cannot automatically switch paths.

These recovery mechanisms are located in the NavOA universal navigation pipeline and apply to RViz manual navigation, autonomous exploration, and other tasks sent to `/move_base_simple/goal`.

## Root Cause

The original `navoa_nav_only.launch` used the following parameters:

```xml
<param name="oscillation_timeout" value="0.0" />
<param name="oscillation_distance" value="0.0" />
<param name="max_planning_retries" value="-1" />
<param name="recovery_behavior_enabled" value="false" />
```

This disables move_base's oscillation detection and recovery behaviors, and allows the planner to retry indefinitely.

Additionally, the global costmap originally only had:

```text
static_layer -> inflation_layer
```

New obstacles detected by LiDAR only enter the local costmap and NavOA obstacle avoidance logic, not the global costmap. Therefore, even if GlobalPlanner periodically re-plans, it may repeatedly generate paths passing through the same blocked area.

When `path_segment_publisher.py` receives an empty global path, it simply ignores the message, and the previous `/split_goal` continues to be retained in NavOA.

## Improved Recovery Flow

```text
New navigation goal
    |
    v
GlobalPlanner plans path
    |
    +-- Consecutive empty paths --> NavOA holds position
    |                               Clear move_base costmap
    |                               Resend original goal and re-plan
    |
    +-- Valid path --> NavOA executes segmented goals
                       |
                       +-- Continuous progress --> Normal execution
                       |
                       +-- 25 seconds without valid progress
                              Preserve real-time obstacle data
                              NavOA holds position
                              Resend original goal
                              GlobalPlanner selects new path
```

After the maximum number of consecutive recoveries is reached, the system cancels the current move_base task and publishes the current position as a hold goal to NavOA, preventing indefinite stalling or continuing with an outdated path.

## move_base Parameters

`navoa_nav_only.launch` adds or adjusts the following parameters:

| Parameter | Default | Description |
| --- | ---: | --- |
| `oscillation_timeout` | `15.0` | Triggers move_base recovery after 15 seconds without leaving the oscillation zone |
| `oscillation_distance` | `0.15` | Displacement required to determine exit from the oscillation zone |
| `max_planning_retries` | `3` | Limits consecutive planning failures to avoid infinite retries |
| `recovery_behavior_enabled` | `true` | Enables move_base costmap recovery and re-planning |
| `clearing_rotation_allowed` | `false` | Disables move_base rotation recovery; motion remains controlled by NavOA |

## Path-Level Recovery Parameters

| Launch Parameter | Node Parameter | Default | Description |
| --- | --- | ---: | --- |
| `path_progress_timeout` | `progress_timeout` | `25.0` | Maximum time without valid progress toward the final goal |
| `path_progress_distance` | `progress_distance` | `0.20` | Distance change considered as valid progress |
| `path_replan_cooldown` | `replan_cooldown` | `5.0` | Minimum interval between two proactive re-plans |
| `path_max_replan_attempts` | `max_replan_attempts` | `3` | Maximum proactive re-plan attempts per single task |
| `empty_plan_threshold` | `empty_plan_threshold` | `3` | Consecutive empty paths reaching this count triggers costmap clearing and retry |
| `clear_costmaps_service` | `clear_costmaps_service` | `/move_base/clear_costmaps` | move_base clearing service |

## Global Costmap

The global costmap plugin order has been adjusted to:

```text
static_layer -> obstacle_layer -> inflation_layer
```

With the addition of the obstacle layer:

- `/scan` can mark real-time obstacles in the global costmap;
- Laser rays can clear dynamic obstacles that have disappeared;
- Dynamic obstacles within the robot's footprint are cleared;
- GlobalPlanner can choose alternative paths based on actual blockages;
- Real-time obstacles are not cleared when no navigation progress is made, preventing re-planning from selecting the same path again.

The global costmap update frequency has been increased from 1 Hz to 5 Hz, and the publish frequency has been adjusted to 2 Hz.

The global inflation radius has been adjusted from 0.35 m to 0.25 m, consistent with the local costmap, reducing the probability that residual obstacle inflation zones completely surround the robot's starting position.

## NavOA Stale Goal Stopping

The path segmentation node now publishes the robot's current position as a hold goal to `/split_goal` in the following cases:

- When a new navigation goal is received and a new path is pending;
- When GlobalPlanner returns an empty path;
- When the move_base goal is cancelled;
- When long-term lack of navigation progress is detected;
- When recovery attempts are exhausted.

This prevents NavOA from continuing to execute old segmented goals when move_base has already stopped or is re-planning.

## Recovery Status

Recovery status is published to:

```text
/navoa/recovery_status
```

View status:

```bash
rostopic echo /navoa/recovery_status
```

Common content:

```text
replan:empty-global-plan:1/3
replan:no-navigation-progress:2/3
failed:no-navigation-progress
cancelled
```

## Parameter Tuning Examples

In narrow environments where detours take longer, you can increase the no-progress timeout appropriately:

```bash
roslaunch NavOA_ros navoa_nav_only.launch \
  path_progress_timeout:=35.0 \
  path_progress_distance:=0.15 \
  path_max_replan_attempts:=4
```

To disable path-level no-progress monitoring:

```bash
roslaunch NavOA_ros navoa_nav_only.launch path_progress_timeout:=0.0
```

## Verification Items

Completed:

- `navoa_nav_only.launch` XML parsing;
- `NavOA_gazebo_limo.launch` XML parsing;
- `path_segment_publisher.py` Python AST parsing;
- Global costmap YAML parsing and plugin order verification;
- Three consecutive empty paths triggering costmap clear and re-plan test;
- 25-second no-goal-progress triggering preserve-obstacle re-plan test;
- `actionlib_msgs` dependency check.

## ROS Integration Testing Suggestions

```bash
# Rebuild
catkin build
source devel/setup.bash
roslaunch NavOA_ros navoa_nav_only.launch
```

Verify the following interfaces:

```bash
rostopic hz /scan
rostopic hz /move_base/global_costmap/costmap
rostopic echo /navoa/recovery_status
rosservice info /move_base/clear_costmaps
```

It is recommended to display both the global costmap and GlobalPlanner path in RViz simultaneously, to verify that the global path detours around obstacles when dynamic obstacles appear, and that the corresponding costmap area is cleared by ray tracing after obstacles are removed.
