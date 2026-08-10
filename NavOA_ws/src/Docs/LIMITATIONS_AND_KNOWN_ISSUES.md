# Limitations & Known Issues

> **Date:** 2026-07-24  
> **Scope:** NavOA_ros stack, Cartographer integration, and overall SlamNav system

---

## 1. Critical Known Issues

### 1.1 TF Frame Anchor Mismatch (HIGH PRIORITY)

**Source:** [Navigation Drift Executive Summary](../../Navigation_Drift_Executive_Summary.md)

**Problem:** Cartographer is publishing `map → base_link` directly instead of `map → odom`. This bypasses the EKF odometry frame, causing global map alignment drift and inconsistent object positioning.

**Current (broken) flow:**
```
[LiDAR] → RF2O → EKF → /odom
                 \ → base_link
                        |
                    Cartographer → map → base_link ← WRONG
                        |
                    move_base / map_server
```

**Desired flow:**
```
[LiDAR] → RF2O → EKF → /odom → Cartographer
                 \ → base_link        |
                                              ▼
                                          map → odom ← CORRECT
                                              |
                                      move_base / map_server
```

**Impact:**
- Robot pose may look correct locally, but the global map reference drifts
- Planner obstacles and live Cartographer localization don't share the same origin
- Recovery from localization loss is slow because the system searches the wrong transform chain

**Recommended Fix:**
1. Change Cartographer launch configs so `published_frame = "odom"` and `provide_odom_frame = false`
2. Ensure `move_base` uses a planner map generated from the same saved map used by Cartographer
3. Publish a known initial pose at startup to speed relocalization

---

## 2. NavOA_ros Stack Limitations

### 2.1 Single-threaded Python Control Loop

The NavOA core control loop (`NavOA_core.py`) runs at **50 Hz in Python**. This creates several constraints:

| Limitation | Impact |
|-----------|--------|
| Python GIL prevents parallel computation | Cannot fully utilize multi-core CPUs for neural network inference |
| 50 Hz ceiling is tight for real-time control | Missing iterations under load can cause uneven velocity commands |
| Blocking operations (TF lookup, serial I/O) can stall the loop | Delayed reactions to obstacles |

**Mitigation:** `vel_filter.py` provides a hardware-level safety layer that catches dangerous commands.

### 2.2 PyTorch Dependency

NavOA uses **PyTorch** (`torch.nn.Module`) for its neural network inference:

- Adds ~500 MB+ to the workspace footprint
- Requires CUDA or CPU-compatible PyTorch build
- No ONNX or TensorRT optimization currently implemented
- Model inference time on CPU may limit control loop frequency on low-power SBCs

### 2.3 Limited Sensor Integration

| Sensor | In NavOA? | In vel_filter? | In Cartographer? |
|--------|-----------|----------------|-----------------|
| LiDAR 2D | ✅ Primary input | ✅ Forward stop zones | ✅ Primary input |
| Ultrasonic | ❌ Not used | ✅ Virtual bumper | ❌ Not used |
| Depth Camera | ❌ Not used | ⚠️ Camscan bias (optional) | ❌ Not used |
| IMU | ❌ Not used | ❌ Not used | ❌ `use_imu_data=false` |

The rich sensor suite (depth camera, 3 ultrasonic, IMU) is underutilized by the core navigation and SLAM algorithms.

### 2.4 vel_filter.py Complexity

The velocity filter (`vel_filter.py`) has evolved into a **single file with multiple overlapping safety behaviors**:

- Forward lidar stop/slow zones
- Ultrasonic virtual bumper (front, left, right)
- Camscan bias for glass detection
- Free-space reactive steering
- Auto-reverse on collision
- Reverse side detection with lateral evasion latching
- Narrow passage mode
- Speed clamps and deadbands

**Risks:**
- Unpredictable interactions between safety behaviors (e.g., ultrasonic evasion conflicting with free-space steering)
- State management complexity (latched states across multiple behaviors)
- Debugging is difficult due to the single-file monolithic structure

**Recommendation:** Consider refactoring into a pipeline architecture with independently testable filter stages.

### 2.5 No Wheel Odometry

The robot has **no wheel encoder odometry**. All odometry comes from:

- `rf2o_laser_odometry` (primary, range-flow from LiDAR scans)
- IMU Z-axis angular velocity (fused via EKF)

**Impact:**
- Odometry degrades in featureless environments (long corridors, open spaces)
- Cannot detect wheel slippage independently
- No backup if LiDAR fails or is occluded

### 2.6 ROS 1 Dependency

The entire stack runs on **ROS 1** (Melodic/Noetic):

- No ROS 2 migration path currently
- `rosbridge_server` provides Web API but adds latency and serialization overhead
- No native DDS security, QoS, or lifecycle management
- `roslibpy` WebSocket communication is fragile under poor network conditions

---

## 3. Cartographer Integration Limitations

### 3.1 Configuration Fragmentation

There are **5+ Lua configuration files** for different modes, with subtle variations:

| Config File | Variations |
|------------|------------|
| `livox_2d.lua` | Base mapping config |
| `livox_mapping.lua` | Mapping with different range/matcher settings |
| `livox_2d_localization.lua` | Base localization |
| `livox_2d_localization_fast_reloc.lua` | Fast reloc (smaller submaps, faster motion filter) |
| `livox_2d_localization_old.lua` | Conservative settings |
| `livox_2d_localization_warmup.lua` | Separate TF frames for warmup |
| `limo_2d.lua` | Different `provide_odom_frame` setting |
| `limo_localization_fast.lua` | Even smaller trimmer (2 submaps) |

**Impact:** Hard to know which config is optimal. Changes must be propagated across multiple files. Some files are unused copies/experiments (`livox_2d copy.lua`, `livox_2d_localization copy.lua`, etc.)

### 3.2 Guard Script Proliferation

There are **6 guard/relaunch scripts** with different detection and recovery strategies:

| Script | Detection | Recovery | Status |
|--------|-----------|----------|--------|
| `cartographer_guard.py` | Score < 0.60 | Service-based | Active |
| `cartographer_guard_a.py` | Score + scan stability | Service-based | Active |
| `cartographer_guard_imu.py` | IMU roll/pitch | Hard reset | Active |
| `cartographer_guard_imu_score.py` | IMU + score + offset + rotation + idle | Hard reset | Active |
| `cartographer_guard_imu_fast_reloc.py` | IMU roll/pitch | Service-based (fast reloc) | **Recommended** |
| `cartographer_guard_score_fast_reloc.py` | Score + offset + rotation + idle | 7-step service-based | Active |

**Impact:**
- Unclear which guard is the "right" one to run
- Hard reset scripts kill and relaunch cartographer_node, which is more disruptive
- Recovery behavior differs between scripts
- No unified health monitoring — multiple guards may fire simultaneously

**Recommendation:** Consolidate to `cartographer_guard_imu_fast_reloc.py` as the primary guard (service-based recovery, no process killing) and retire the others.

### 3.3 Pure Localization Trimmer Behavior

The `pure_localization_trimmer` keeps only the last N submaps (3 in most configs):

- **Limitation:** Very sensitive to initial pose. If the initial pose is wrong, the first 3 submaps will be in the wrong place, and the trimmer discards older submaps that might help convergence
- **Workaround:** Guard scripts re-initialize with fast reloc (no initial pose) → capture corrected pose → restart with pose

### 3.4 No 3D SLAM

- Cartographer is configured for 2D only (`use_trajectory_builder_2d=true`)
- The depth camera (HP60C) is available but not used for SLAM
- No elevation mapping or 3D obstacle detection
- No support for multi-floor navigation

### 3.5 Map Size Scaling

As maps grow (the default `test001.pbstream` is 7.0 MB):
- Pose graph optimization takes longer
- Global localization search space increases
- The `global_sampling_ratio` parameter must be tuned down for large maps (currently 0.1 in localization mode), which increases CPU usage
- No explicit map culling or submap pruning strategy

### 3.6 IMU Not Used in SLAM

Despite having a dedicated IMU sensor, `use_imu_data=false` in all Cartographer Lua configs:

- **Rationale:** 2D LiDAR SLAM is robust without IMU for planar motion
- **Missed opportunity:** IMU could help with fast rotation handling, ceiling/floor detection, and 3D orientation estimation
- IMU is only used by guard scripts for fall detection and by EKF for angular velocity fusion

### 3.7 `switch_lua.py` Complexity

The boot-time handoff (`switch_lua.py`) adds complexity:
- Starts with fast config → waits 20s → captures pose → finishes → starts slow config
- If the fast config doesn't converge within 20s, the captured pose is wrong
- The handoff adds a ~20s delay before full localization quality is available
- Two mechanisms exist (`boot_handoff_enabled` vs `boot_swap_enabled`) with unclear differences

---

## 4. System-Level Limitations

### 4.1 Single Point of Failure: runner_node.py

The entire system lifecycle depends on `runner_node.py`:

- If `runner_node.py` crashes (and respawn=true recovers it), all mode state is lost
- No persistent state storage between crashes
- No mode recovery — the system defaults to SENSORS_ONLY state
- The MARL subprocess is orphaned if runner_node crashes without cleanup

### 4.2 No Graceful Degradation

| Failure | Current Behavior | Desired Behavior |
|---------|-----------------|-----------------|
| LiDAR failure | Navigation stops (no /scan) | Use ultrasonic + camera as fallback |
| Cartographer crash | System enters SENSORS_ONLY | Fall back to rf2o + AMCL |
| Network loss (web) | No remote control | Continue autonomous mission |
| Motor controller hang | vel_filter publishes commands that go nowhere | Detect and execute emergency stop |

### 4.3 No Built-in Emergency Stop

- No dedicated hardware emergency stop integration
- `vel_filter.py` provides software-level safety but relies on ROS topic communication
- No heartbeat/watchdog between ROS computer and motor controller

### 4.4 MARL Subprocess Management

The MARL exploration subprocess is the **only** subprocess and has rough edges:

- Output is discarded (`stdout=DEVNULL, stderr=DEVNULL`) — debugging failures is hard
- No health monitoring — if the subprocess dies, runner_node doesn't know
- Kill sequence (SIGTERM → 3s → SIGKILL) may leave ROS nodes orphaned if the process doesn't respond
- No graceful shutdown signal — MARL nodes stop mid-exploration

### 4.5 No Simulation Integration in Production Stack

- Gazebo simulation exists in `NavOA_ros/example/gazebo_limo/` but is **not integrated** with the main launch system (`runner/full.launch`)
- No way to run the complete system in simulation for testing
- The MARL exploration module has RL environment wrappers (`RealWorldExplorationEnv`) but they're not wired up

### 4.6 Serial Port Conflicts

| Port | Connected To | Potential Conflict |
|------|-------------|-------------------|
| `/dev/ttyS1` | Motor controller | Also used by `imu_publisher` (ttyS6) in `camera_imu_lidar.launch` |
| `/dev/ttyS4` | LiDAR | Dedicated — no conflict |
| `/dev/ttyS6` | WIT IMU | Also used by ultrasonic in some configs |

`camera_imu_lidar.launch` uses `imu_publisher.py` on `/dev/ttyS6`, while `sensors.launch` uses `serial_cmd_vel_imu_bridge.py` on `/dev/ttyS1` for IMU data. These are separate ports but the configuration is easy to confuse.

---

## 5. Code Quality Issues

### 5.1 Dead / Unused Code

| File | Status | Notes |
|------|--------|-------|
| `real_robot_cmd/` (deleted from git) | Deleted | Still referenced in some launch files |
| Multiple Lua config copies (`livox_2d copy.lua`, etc.) | Unused copies | Experimentation artifacts |
| `NavOA_core copy.py` | Unused copy | Backup file |
| Some guard scripts | Likely redundant | See section 3.2 |

### 5.2 Documentation Gaps

- `NavOA_ros/example/gazebo_limo/` has in-line comments but no standalone README
- Sensor calibration procedures not documented
- No network setup guide for FastAPI + rosbridge
- Lua parameter tuning guide does not exist
- Map saving/conversion workflow not documented

### 5.3 Workspace Organization

- Three nested catkin workspaces (`NavOA_ws`, `carto_ws`, `ascam_ws`) add build complexity
- `abseil-cpp` at both `NavOA_ws/src/` and `carto_ws/src/` (duplicated dependency)
- Map files stored in two locations (`src/map/` and `carto_ws/map/`)
- Mixed `catkin_make_isolated` and `catkin build` build patterns

---

## 6. Performance Bottlenecks

### 6.1 CPU Usage

| Component | CPU Estimate | Notes |
|-----------|-------------|-------|
| Cartographer pose graph optimization | 30-60% of one core | Increases with map size and `optimize_every_n_nodes` |
| NavOA PyTorch inference | 20-40% of one core | Per-frame at 50 Hz |
| rf2o laser odometry | ~5% of one core | ~0.9ms per frame |
| vel_filter.py | ~5% of one core | Simple arithmetic |
| Serial bridge | ~5% of one core | CSV parsing + serial I/O |

**Total:** ~70-120% of one core (not parallelized), likely CPU-bound on the robot's SBC.

### 6.2 Memory

| Component | Memory | Notes |
|-----------|--------|-------|
| PyTorch runtime | ~400-800 MB | Model weights + CUDA context (if available) |
| Cartographer pose graph | ~50-500 MB | Depends on map size and number of submaps |
| ROS message buffers | ~50-200 MB | Scan data, TF tree, costmaps |
| **Total** | **~500 MB - 1.5 GB** | |

### 6.3 Network

| Interface | Bandwidth | Latency |
|-----------|-----------|---------|
| rosbridge WebSocket (9090) | ~100 KB/s status updates | < 100ms LAN |
| Serial cmd_vel (ttyS1) | ~100 bytes/s | < 10ms |
| Serial LiDAR (ttyS4) | ~50 KB/s | < 5ms |
| Serial IMU (ttyS6) | ~1 KB/s | < 5ms |
