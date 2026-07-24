# Cartographer Integration — SLAM & Localization

> **Date:** 2026-07-24  
> **Package:** `carto_ws` (nested catkin workspace)  
> **Location:** `/home/ubuntu/Robit_SlamNav/NavOA_ws/src/carto_ws/`

---

## 1. Overview

Google Cartographer provides the SLAM (Simultaneous Localization and Mapping) backbone for the SlamNav system. It is a 2D LiDAR-based SLAM that builds and optimizes a pose graph of submaps. The system uses Cartographer in two distinct modes:

1. **Mapping Mode** — Builds new maps from scratch using `livox_2d.lua` configuration
2. **Localization Mode** — Localizes against pre-built `.pbstream` maps using `livox_2d_localization.lua` with pure-localization trimmer

**Key Design Decision:** cartographer_node runs as a top-level roslaunch process and is **never killed**. Mode switching is done entirely through Cartographer's trajectory services (`start_trajectory`, `finish_trajectory`), keeping the process alive and its pose graph in memory.

---

## 2. Cartographer Lua Configuration Files

### 2.1 Lua Configuration Hierarchy

```
cartographer (upstream defaults)
  └── map_builder.lua
        ├── pose_graph.lua
        └── trajectory_builder_2d.lua
              └── (sensor-specific overrides)

cartographer_ros (custom configs)
  ├── livox_2d.lua                 ← Base mapping config
  │   ├── livox_mapping.lua        ← Alternative mapping (no optimization throttle)
  │   └── livox_2d_localization.lua ← Base localization config
  │       ├── livox_2d_localization_fast_reloc.lua  ← Fast relocalization
  │       └── livox_2d_localization_old.lua         ← Conservative localization
  ├── limo_2d.lua                  ← Base Limo config
  │   └── limo_localization_fast.lua ← Limo localization
  └── (upstream: backpack_2d.lua, revo_lds.lua, pr2.lua, etc.)
```

### 2.2 livox_2d.lua (Mapping Configuration)

**Used by:** `full.launch mode:=manager`, `mapping.launch`

| Parameter | Value | Notes |
|-----------|-------|-------|
| `published_frame` | `"odom"` | Cartographer publishes `map→odom` |
| `provide_odom_frame` | `false` | rf2o provides odometry |
| `use_odometry` | `true` | Uses `/odom_rf2o` |
| `use_imu_data` | `false` | IMU not used for SLAM |
| `publish_tracked_pose` | `false` | |
| `TRAJECTORY_BUILDER_2D.min_range` | `0.1` m | |
| `TRAJECTORY_BUILDER_2D.max_range` | `4.0` m | Capped for indoor use |
| `missing_data_ray_length` | `2.0` m | |
| `num_accumulated_range_data` | `1` | No multi-echo |
| `voxel_filter_size` | `0.025` m | |
| `submaps.num_range_data` | `35` | Submap size |
| `submaps.resolution` | `0.05` m | |
| `CERES_SCAN_MATCHER.translation_weight` | `10` | |
| `CERES_SCAN_MATCHER.rotation_weight` | `40` | |
| `CERES_SCAN_MATCHER.occupied_space_weight` | `1` | |
| `CERES_SCAN_MATCHER.max_iterations` | `20` | |
| `REAL_TIME_CORRELATIVE_SCAN_MATCHER.linear_search_window` | `0.1` m | |
| `REAL_TIME_CORRELATIVE_SCAN_MATCHER.angular_search_window` | `0.349` rad (20°) | |
| `POSE_GRAPH.optimize_every_n_nodes` | `50` | Optimization frequency |
| `POSE_GRAPH.constraint_builder.min_score` | `0.70` | Loop closure threshold |
| `POSE_GRAPH.constraint_builder.sampling_ratio` | `0.3` | |
| `POSE_GRAPH.constraint_builder.max_constraint_distance` | `2.0` m | |
| `POSE_GRAPH.global_sampling_ratio` | `0.003` | |
| `POSE_GRAPH.optimization_problem.huber_scale` | `1e2` | |

### 2.3 livox_2d_localization.lua (Localization Configuration)

**Used by:** `localization.launch`, guard scripts

Inherits `livox_2d.lua` with these overrides:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `pure_localization_trimmer.max_submaps_to_keep` | `3` | Keeps only 3 recent submaps |
| `publish_tracked_pose` | `true` | For guard monitoring |
| `POSE_GRAPH.optimize_every_n_nodes` | `2` | Very frequent optimization |
| `POSE_GRAPH.global_sampling_ratio` | `0.1` | Higher for localization |
| `POSE_GRAPH.constraint_builder.min_score` | `0.60` | Lower threshold |
| `POSE_GRAPH.global_localization_min_score` | `0.60` | |
| `TRAJECTORY_BUILDER_2D.submaps.num_range_data` | `30` | Smaller submaps |
| `TRAJECTORY_BUILDER_2D.min_range` | `0.1` m | |
| `TRAJECTORY_BUILDER_2D.max_range` | `10.0` m | Wider range for reloc |
| `missing_data_ray_length` | `3.0` m | |
| `voxel_filter_size` | `0.08` m | Coarser filtering |
| `motion_filter.max_time_seconds` | `1.0` s | |
| `motion_filter.max_distance_meters` | `0.1` m | |
| `motion_filter.max_angle_radians` | `0.017` rad (1°) | |
| `REAL_TIME_CORRELATIVE_SCAN_MATCHER.linear_search_window` | `0.3` m | |
| `REAL_TIME_CORRELATIVE_SCAN_MATCHER.angular_search_window` | `0.524` rad (30°) | |
| `POSE_GRAPH.constraint_builder.sampling_ratio` | `0.5` | |
| `POSE_GRAPH.constraint_builder.max_constraint_distance` | `10.0` m | |
| `CERES_SCAN_MATCHER.occupied_space_weight` | `20` | Higher weight for scans |

### 2.4 livox_2d_localization_fast_reloc.lua (Fast Relocalization)

Inherits `livox_2d.lua` with:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `pure_localization_trimmer.max_submaps_to_keep` | `3` | |
| `POSE_GRAPH.global_sampling_ratio` | `0.01` | Faster global matching |
| `TRAJECTORY_BUILDER_2D.submaps.num_range_data` | `20` | Very small submaps |
| `motion_filter.max_time_seconds` | `0.1` s | Very sensitive motion trigger |
| `POSE_GRAPH.optimize_every_n_nodes` | `20` | |

**Purpose:** This config is used by the guard scripts for fast re-initialization after localization loss. It creates small submaps quickly and uses aggressive global matching to re-localize.

---

## 3. Trajectory Management

### 3.1 Trajectory States

Cartographer trajectories have four states:

| State ID | Name | Meaning |
|----------|------|---------|
| 0 | ACTIVE | Currently building submaps, accepting sensor data |
| 1 | FINISHED | Completed, submaps frozen in pose graph |
| 2 | FROZEN | Finished and frozen (can't create constraints against it) |
| 3 | DELETED | Removed from pose graph |

### 3.2 Trajectory Lifecycle

```
System Start
  │
  └── Trajectory 0: ACTIVE (mapping with livox_2d.lua)
        │
        ├── mode:mapping → (already mapping, no change)
        │
        ├── stop_mapping
        │     ├── /write_state → saves .pbstream
        │     ├── /finish_trajectory(0) → Trajectory 0: FINISHED
        │     └── /start_trajectory(config=livox_2d_localization.lua)
        │           └── Trajectory 1: ACTIVE (pure localization)
        │
        ├── mode:auto_mapping
        │     └── (keeps Trajectory 0 ACTIVE, launches MARL)
        │
        └── mode:sensors
              └── /finish_trajectory(0) → no active trajectory
```

### 3.3 Reference Trajectory

When starting a new trajectory, the system selects a **reference trajectory** (typically ID 0, the first FINISHED one). The new trajectory creates constraints against the reference's submaps:

```
[Trajectory 0: FINISHED] ── reference frame
   submap 0 ─────────────────┐
   submap 1                  │ constraints
   submap 2 ─────────────────┤
                             │
[Trajectory 1: ACTIVE] ──────┘
   submap 3 (trimmed)
```

---

## 4. Guard & Recovery Scripts

The system includes multiple guard scripts that monitor localization health and automatically recover from failures.

### 4.1 Guard Script Comparison

| Script | Detection Method | Recovery Strategy | Lua Config Used |
|--------|-----------------|-------------------|-----------------|
| `cartographer_guard_imu_fast_reloc.py` | IMU roll/pitch fall (>20°/ >30°) | Service-based trajectory swap (no process kill) | fast_reloc |
| `cartographer_guard.py` | Matching score < 0.60 for 10 frames | Trajectory finish/start with cached pose | Default |
| `cartographer_guard_a.py` | Score + scan stability | Trajectory swap | Default |
| `cartographer_guard_imu.py` | IMU roll/pitch fall | Hard reset (kill + relaunch nodes) | N/A |
| `cartographer_guard_imu_score.py` | IMU + score + offset + rotation + idle | Hard reset (kill + relaunch) | N/A |
| `cartographer_guard_score_fast_reloc.py` | Score + offset + rotation + idle | 7-step service-based recovery | fast_reloc |

### 4.2 cartographer_guard_imu_fast_reloc.py (Primary Guard)

**File:** `cartographer_ros/scripts/cartographer_guard_imu_fast_reloc.py`

This is the most sophisticated guard, using **Cartographer service-based recovery** (no process killing).

**State Machine:**
```
STATE_IDLE ──→ (roll>20° or pitch>30°) ──→ STATE_FALLEN
                                                  │
                                           (wait for upright)
                                                  │
                                                  ▼
STATE_FALLEN ──→ (roll<5° and pitch<5° for 5 frames) ──→ STATE_RECOVERING
                                                                │
                                                          6-step recovery
                                                                │
                                                                ▼
STATE_RECOVERING ──→ recovery complete ──→ STATE_IDLE
```

**6-Step Recovery Sequence:**
1. Finish latest active trajectory via `/finish_trajectory`
2. Start fast reloc trajectory with `livox_2d_localization_fast_reloc.lua` (blind, no initial pose)
3. Wait for robot upright (IMU roll/pitch < 5° for 5 consecutive frames)
4. Wait `recovery_settle_duration` (2s), then capture `map→base_footprint` TF pose (up to 30 retries)
5. Finish fast reloc trajectory
6. Start new trajectory with default config (`livox_2d_localization.lua`) + captured pose via `start_trajectory_with_pose`

### 4.3 cartographer_guard_score_fast_reloc.py (Score-Based Guard)

**7-Step Recovery Sequence:**
1. Wait for matching score to recover naturally (optional)
2. Finish latest active trajectory
3. Start fast_reloc trajectory (blind, no pose)
4. Wait 60 seconds for reloc to converge
5. Capture `map→base_footprint` TF pose
6. Finish fast_reloc trajectory
7. Start default trajectory with captured pose

### 4.4 switch_lua.py (Boot-Time Handoff)

**Purpose:** Optimizes startup by starting with a FAST config initially, then swapping to a SLOW (full-featured) config after stabilization.

```
boot:
  cartographer_node starts with livox_2d_localization_fast_reloc.lua
  switch_lua.py starts simultaneously
        │
        └── wait 20 seconds
              │
              ├── Capture map→base_footprint pose
              ├── Finish fast trajectory
              └── Start slow trajectory (livox_2d_localization.lua) with captured pose
                    │
                    └── Import CartographerTrajectoryManagerScore for runtime guards
```

---

## 5. Occupancy Grid Node

The `cartographer_occupancy_grid_node` converts Cartographer's submaps into ROS OccupancyGrid messages.

**Configuration:**
- Resolution: 0.05 m/pixel
- Include unfrozen submaps: `true` (mapping), `false` (localization — to avoid conflict with static map_server)

**Topic Mapping:**
| Mode | Output Topic | Purpose |
|------|-------------|---------|
| Mapping | `/map` | Used by move_base global_costmap |
| Localization | `/carto_submap_viz` | Visualization only (remapped from `/map`) |
| Localization | `/map` (from `map_server`) | Static pre-built map for move_base |

---

## 6. Saved Maps

### 6.1 Map File Formats

Each saved map has three files:
- `.pbstream` — Cartographer binary format (pose graph, submaps, constraints)
- `.pgm` — ROS occupancy grid image (grayscale)
- `.yaml` — ROS map metadata

### 6.2 Map Metadata Format

```yaml
image: test00.pgm
resolution: 0.050000
origin: [-5.544689, -12.431804, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### 6.3 Map Storage Locations

| Location | Contents |
|----------|----------|
| `carto_ws/map/meeting_room/` | Primary maps (test00.pbstream is the default) |
| `carto_ws/map/` | Auto-saved maps (`map_<timestamp>.pbstream`) + named maps |
| `src/map/meeting_room/` | Additional copies of meeting_room map |

### 6.4 Default Map

The default map loaded by `localization.launch` is:
```
map/meeting_room/test00.pbstream
```
Size: 7.0 MB (largest map with most detail)

---

## 7. Cartographer ROS Launch Files

| Launch File | Purpose | Lua Config |
|-------------|---------|------------|
| `livox_2d.launch` | Pure mapping | `livox_2d.lua` |
| `livox_2d_localization.launch` | Localization + map_server | `livox_2d_localization_fast_reloc.lua` |
| `cartographer_guard.launch` | Score-based guard | Default |
| `cartographer_guard_imu.launch` | IMU-based guard (hard reset) | — |
| `cartographer_guard_imu_score.launch` | Combined IMU+score guard | — |
| `cartographer_localization_runtime.launch` | Pure localization node | Default |
| `mode_manager.launch` | Mode switch manager | — |

---

## 8. Cartographer Parameters Summary

### 8.1 Key Tuning Parameters

| Parameter | Mapping | Localization | Fast Reloc | Effect |
|-----------|---------|--------------|------------|--------|
| `submaps.num_range_data` | 35 | 30 | 20 | Larger = more stable, slower to finish |
| `max_range` | 4.0 m | 10.0 m | 10.0 m | Shorter = less noise, longer = more reloc data |
| `optimize_every_n_nodes` | 50 | 2 | 20 | Lower = more CPU, faster convergence |
| `global_sampling_ratio` | 0.003 | 0.1 | 0.01 | Higher = better reloc, more CPU |
| `min_score` | 0.70 | 0.60 | 0.60 | Lower = more loop closures, more false positives |
| `voxel_filter_size` | 0.025 | 0.08 | 0.08 | Larger = coarser, faster |
| `motion_filter.max_time` | 5.0s | 1.0s | 0.1s | Lower = more motion updates |

### 8.2 Performance Considerations

- **Mapping mode** prioritizes accuracy: smaller voxel filter, longer submaps, higher min_score
- **Localization mode** prioritizes responsiveness: very frequent optimization (every 2 nodes), aggressive motion filter
- **Fast reloc** prioritizes speed: tiny submaps (20 scans), very sensitive motion trigger (0.1s)
