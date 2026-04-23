# Navigation Drift and Object Pose Disorder Report

## 1. Executive Summary

Your current system is using Cartographer for localization on a saved `.pbstream` map, while EKF is fusing RF2O laser odometry and IMU data to publish `/odom`. The main issue is not a pure TF tree break, but rather a mismatch between: 1) Cartographer's published pose frame and 2) the planning map used by `move_base`.

This causes the robot to appear aware of its own location, while the positions of objects and obstacles on the map look disorganized and the system takes a long time to restore the correct pose after startup or relocalization.

## 2. Current Architecture

### 2.1 Active components

- `lidar_ros/launch/lidar.launch`
  - Publishes `/scan`
  - Provides static transform `base_footprint -> laser_frame`
- `rf2o_laser_odometry/launch/rf2o_laser_odometry.launch`
  - Consumes `/scan`
  - Publishes `/odom_rf2o`
  - Publishes a local `odom -> base_footprint` transform
- `rf2o_laser_odometry/launch/ekf_fusion.launch`
  - Runs `robot_localization/ekf_localization_node`
  - Loads `rf2o_laser_odometry/ekf_params.yaml`
  - Remaps filtered odometry to `/odom`
- `NavOA_ros/example/gazebo_limo/launch/carto_localization.launch`
  - Runs `cartographer_node`
  - Loads saved map state from `officeMap.pbstream`
  - Starts `cartographer_occupancy_grid_node`
- `NavOA_ros/example/gazebo_limo/launch/global_planner.launch`
  - Starts `map_server` loading `officeMap.yaml`
  - Starts `move_base`

### 2.2 Current map sources

- Localization map: `officeMap.pbstream` loaded by Cartographer
- Planner map: `officeMap.yaml` loaded by `map_server`

These are two separate map sources unless they are generated from the exact same saved map and same origin.

## 3. How Cartographer Processes the Data

### 3.1 Input data used by Cartographer

Cartographer receives:
- `/scan` from the laser
- `/imu_mux` from the IMU fallback manager
- `/odom` from EKF fused odometry

In your config:
- `map_frame = "map"`
- `tracking_frame = "base_footprint"`
- `odom_frame = "odom"`
- `provide_odom_frame = false`
- `use_odometry = true`

This means Cartographer should:
- trust external odometry from `/odom`
- localize the robot on the loaded map
- use scan matching and the existing map to compute pose updates

### 3.2 Expected transformation flow

Ideal flow:

```
/scan -> Cartographer frontend -> pose estimate in base_footprint
/odom -> EKF -> odom -> base_footprint
Cartographer publishes map -> odom
```

Then the full pose chain is:

```
map -> odom -> base_footprint -> laser_frame
```

### 3.3 What Cartographer actually does in the current launch

The launch file `carto_localization.launch` passes a node-level parameter:

- `published_frame = base_footprint`

That means the Cartographer node is likely publishing `map -> base_footprint` directly.

This is a mismatch with the config file `limo_localization_fast.lua`, which sets `published_frame = "odom"`. The launch override is therefore the most important current issue.

## 4. Missing Point in Your Data Flow

### 4.1 Frame anchoring mismatch

The missing point is this:

- EKF provides `/odom` and `odom -> base_footprint`
- Cartographer should publish `map -> odom`
- But the launch file overrides Cartographer to publish `map -> base_footprint`

The result is that Cartographer is not anchored through the EKF odometry frame, so:

- object poses on the map can appear shuffled
- the planner may use a different map origin or frame reference
- the robot's base pose can be restored slowly because the relocalization does not use the odometry anchor correctly

### 4.2 Separate map streams

Your system currently has two map sources:

- `officeMap.pbstream` for Cartographer localization
- `officeMap.yaml` for `map_server` and `move_base`

If these are not exactly matched, then:

- the robot's localized pose in `map` can differ from the planner's obstacle map
- map objects look disorganized because the planner and localization disagree on the map origin
- recovery appears slow because the planner waits for a consistent map pose while Cartographer is still converging

### 4.3 Slow relocalization

Slow recovery often comes from:

- no good initial pose in the saved map frame
- a large map with only local scan matching enabled
- constraints or global localization not being triggered quickly enough
- Cartographer effectively trying to relocalize using only map->base_footprint updates instead of map->odom

## 5. Why the Map Objects Look Disorganized

### 5.1 Two main causes

1. **Frame publication mismatch**
   - If Cartographer is publishing `map -> base_footprint`, it bypasses the EKF odometry anchor.
   - This makes the base pose visually correct in some cases, but the internal map reference drifts relative to the planner map.

2. **Dual map source mismatch**
   - `move_base` uses a static occupancy map from `officeMap.yaml`
   - Cartographer localizes on a live pbstream map
   - If origins or map definitions differ, object positions on the planner map are not the same as the localized Cartographer pose

### 5.2 Why recovery still feels slow

- Cartographer can localize, but only after it has enough scan overlap with the loaded map.
- If the initial pose is not close, the robot spends time searching.
- If `published_frame` is wrong, the relocalization search is effectively happening in the wrong coordinate chain.

## 6. Recommended Fix Strategy

### 6.1 Primary fix: align Cartographer with EKF odom

In `src/NavOA_ros/example/gazebo_limo/launch/carto_localization.launch`:
- remove or change the node param `published_frame = base_footprint`
- set `published_frame = odom`
- keep `odom_frame = odom`
- keep `map_frame = map`

In your Cartographer config files:
- keep `provide_odom_frame = false`
- keep `published_frame = "odom"`
- keep `use_odometry = true`

This will make Cartographer produce `map -> odom`, which is the proper link to the EKF-published `odom -> base_footprint`.

### 6.2 Secondary fix: unify map sources

Option A: Use the same `.pbstream` map as the authoritative source and generate the YAML from it.
- Save the `officeMap.pbstream` from the same run used for localization
- Generate `officeMap.yaml` from that pbstream with `map_saver` or `cartographer_pbstream_to_ros_map`
- Load that YAML in `map_server` and `move_base`

Option B: Use Cartographer's live occupancy grid topic for planning if you need dynamic updates.
- Remap `map` in `move_base` to Cartographer's occupancy grid topic
- Ensure the planner and localization share the same `map` frame and origin

### 6.3 Add explicit initial pose injection

To recover quickly:
- publish an initial pose close to the actual start location
- use RViz `/initialpose` or a scripted known-start publisher
- if the robot starts at a known location, use that as the first pose guess

This helps Cartographer align the loaded map immediately instead of searching for the correct match.

### 6.4 Validate the whole chain

Check these in the running system:
- `rosrun tf tf_echo map odom`
- `rosrun tf tf_echo odom base_footprint`
- `rostopic echo -n 1 /odom/header`
- `rostopic hz /scan`
- `rosrun tf view_frames`
- `rosrun rviz rviz` and compare `map` frame object positions

### 6.5 Tune relocalization and map matching

If the problem persists after fixing the frame flow:
- lower `POSE_GRAPH.global_localization_min_score` slightly
- ensure `use_online_correlative_scan_matching = true`
- keep `submap_publish_period_sec` and `pose_publish_period_sec` short enough for timely updates
- verify that `load_state_filename` is the correct `.pbstream` file for the environment

## 7. Practical Step-by-Step Plan

1. Inspect `carto_localization.launch` and remove the `published_frame = base_footprint` override.
2. Confirm `limo_localization_fast.lua` and `limo_2d_reloc.lua` both publish `odom` and do not provide the odom frame.
3. Confirm the EKF node is publishing `/odom` and `odom -> base_footprint`.
4. Verify the static `map_server` YAML map is derived from the same `.pbstream` used for localization.
5. Start the system and verify `map -> odom -> base_footprint` continuously.
6. Publish a known `initialpose` after startup to speed recovery.
7. Monitor `move_base` and RViz object positions for alignment.
8. If needed, switch `move_base` to use Cartographer's live map instead of a separate static YAML map.

## 8. Diagram of Current and Desired Flow

### Current problematic flow

```
[LiDAR] --> /scan --> [RF2O] --> /odom_rf2o --> [EKF] --> /odom
                                   \                         
                                    \--> odom->base_footprint  
                                            |                 
                                            v                 
                                       [Cartographer]         
                                            |                 
                                   map->base_footprint       
                                            |                 
                                 [move_base/map_server]
                                            ^                 
                                   officeMap.yaml (static)
```

### Desired corrected flow

```
[LiDAR] --> /scan --> [RF2O] --> /odom_rf2o --> [EKF] --> /odom --> [Cartographer]
                                            |                       |
                                            |                       v
                                            +-> base_footprint   map->odom
                                                                    |
                                                         [move_base / map_server]
                                                            same map origin
```

## 9. Management Summary

- The robot is not failing because TF is absent.
- The root problem is a frame and map alignment mismatch between Cartographer localization and planning.
- Fixing `published_frame` and unifying the map source should make object poses on the map consistent and restore pose faster.
- The highest-value change is to make Cartographer publish `map -> odom` and use the same `officeMap` map for both localization and planning.

If you want, I can also produce a short slide-style summary or a one-page executive version of this report.