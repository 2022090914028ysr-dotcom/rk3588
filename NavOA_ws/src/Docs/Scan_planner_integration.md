# SCAN-Planner + NavOA Integration — Plan, Changes, Troubleshooting

Robot: HighTorque Mini Pi biped (RK3588 onboard, `192.168.110.205`; ROS master on leg PC `192.168.110.132`). ROS 1 Noetic.

Status: **Step 1 (config/build) done on branch `feature/scan-navoa-step1`.** Hardware gates (Step 2–4) pending.

---

## 1. The plan

Integrate the teammate's working SCAN-Planner + NavOA package into the robot with this **fixed** topic chain:

```text
/move_base_simple/goal          (RViz 2D Nav Goal)
        |
        v
SCAN-Planner  (controller_mode:=none, pure planning)
        |
 /planning/bspline
        |
        v
bspline_to_path
        |
 /planning/path
        |
        v
path_segment_publisher
        |
 /split_goal
        |
        v
NavOA  <----- /scan  (2D LiDAR, 360°)
        |
 /NavOA_cmd_vel
        |
        v
vel_filter  (sole base velocity publisher)
        |
 /cmd_vel
        |
        v
base
```

Division of responsibilities:

- **SCAN-Planner** — route source. Forward 3D **depth camera** (`/ascamera_hp60c/depth0/image_raw`), sliding grid map, B-spline local trajectory. Runs in pure-planning mode (no closed/open-loop controller, so it never publishes velocity).
- **`bspline_to_path`** — converts SCAN's `scan_planner/Bspline` into a standard `nav_msgs/Path`.
- **`path_segment_publisher`** — splits the local path by distance into NavOA sub-goals (`/split_goal`).
- **NavOA** — local avoider. 360° **2D LiDAR** (`/scan`), MPC/neural (dune/pan) controller.
- **`vel_filter`** — velocity limit + last-level safety; the **only** node publishing to base `/cmd_vel`.

### Protected constraints (do NOT change)

- `ekf_se` (EKF) is the sole `odom -> base_link` owner. Do not re-enable `publish_tf` on the rf2o/leg node. Do not change EKF config.
- Do **not** start `move_base`, `navoa_nav_only.launch`, GlobalPlanner, or global/local costmaps.
- Do **not** start SCAN `closed_loop_controller` or `open_loop_controller`.
- After bring-up, `rostopic info /cmd_vel` must show `vel_filter` as the **only** publisher.

---

## 2. Robot values (teammate default → our robot)

| Item | Teammate value | Our value |
|---|---|---|
| world / map frame | `world` / `map` | `odom` |
| odometry topic | `/LIO/odom_vehicle`, `/LIO/odom_imu` | `/odom` (leg odometry; rf2o disabled) |
| depth topic | `/camera/aligned_depth_to_color/image_raw` | `/ascamera_hp60c/depth0/image_raw` |
| depth intrinsics | — | fx 421.330017, fy 420.851471, cx 317.461792, cy 240.692566, D=0, R=I, 640×480 |
| depth encoding | — | 16UC1 mm, `k_depth_scaling_factor=1000.0` |
| 2D laser | `/scan` | `/scan` (frame `laser_frame`) |
| camera mount (base_link → camera_link_0) | — | `0.10 0.00 0.50 0 -1.5707963 0` |
| robot height / base_link center | — | 0.6 m / 0.45 m |
| footprint (length × width) | 0.60 × 0.30 | **0.25 × 0.30** |
| max forward / angular speed | — | 0.30 m/s / 1.5 rad/s (vel_filter) |
| SCAN max_vel / max_acc | 0.75 / 0.5 | 0.30 / 0.30 |

---

## 3. Changes applied

All under `src/`. Commits: `9838efd` (SCAN) and `45dd15d` (NavOA).

### 3.1 SCAN-Planner config (commit `9838efd`)

[src/SCAN-Planner/src/planner/plan_manage/launch/run.launch](src/SCAN-Planner/src/planner/plan_manage/launch/run.launch)

- `is_real_world` `false → true`
- `sensor_type` `lidar → depth`
- `world_frame` `world → odom`
- `launch_go2_support` / `launch_odom_visualization` `true → false`
- `body_pose_topic` `/LIO/odom_vehicle → /odom` *(placeholder, confirm §6)*
- `sensor_pose_topic` `/LIO/odom_imu → /odom` *(placeholder, confirm §6)*
- `depth_topic` `→ /ascamera_hp60c/depth0/image_raw`
- intrinsics `cx/cy/fx/fy` → `317.4617919921875 / 240.69256591796875 / 421.33001708984375 / 420.8514709472656`
- `controller_mode` `closed_loop → none`

[src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml](src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml)

- `max_vel` `0.75 → 0.30`, `max_acc` `0.5 → 0.30`
- `double_cylinder_radius` `0.25 → 0.15`, `double_cylinder_offset` `0.18 → 0.0`
- `body_height` `0.4 → 0.45`

[src/SCAN-Planner/src/planner/plan_env/src/grid_map.cpp](src/SCAN-Planner/src/planner/plan_env/src/grid_map.cpp)

- `depth_extrinsic_` = body → camera (0.10 m forward, 0.50 m up, R_y(−π/2)):

```cpp
mp_.depth_extrinsic_ <<
    0.0,  0.0, -1.0,  0.10,
    0.0,  1.0,  0.0,  0.00,
    1.0,  0.0,  0.0,  0.50,
    0.0,  0.0,  0.0,  1.00;
```

  Roll-corrected alternate (commented in source) if the depth cloud reads 90° rolled at Gate 1:

```cpp
//  0.0, -1.0,  0.0,  0.10,
//  0.0,  0.0, -1.0,  0.00,
//  1.0,  0.0,  0.0,  0.50,
//  0.0,  0.0,  0.0,  1.00;
```

[src/SCAN-Planner/src/planner/plan_manage/src/bspline_to_path.cpp](src/SCAN-Planner/src/planner/plan_manage/src/bspline_to_path.cpp)

- `frame_id` default `map → odom`
- `flatten_z` `false → true`

### 3.2 NavOA config (commit `45dd15d`)

[src/NavOA_ros/example/gazebo_limo/config/NavOA_planner_limo.yaml](src/NavOA_ros/example/gazebo_limo/config/NavOA_planner_limo.yaml)

- robot `length` `0.60 → 0.25` (width `0.30` unchanged)

[src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch](src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch)

- `map_frame` `map → odom`
- `path_progress_timeout` `25.0 → 0.0` (disables the move_base no-progress replan)
- `return_to_previous_goal_enabled` `true → false`
- added `scan_topic` arg (default `/scan`) and `path_topic` arg (default `/move_base/GlobalPlanner/plan`)
- `/scan` remap → `$(arg scan_topic)`; `scan_topic` forwarded to `vel_filter`; `path_topic` forwarded to `path_segment_publisher`

[src/NavOA_ros/scripts/path_segment_publisher.py](src/NavOA_ros/scripts/path_segment_publisher.py)

- parameterized `path_topic` (default `/move_base/GlobalPlanner/plan`), `split_goal_topic` (`/split_goal`), `goal_input_topic` (`/move_base_simple/goal`), `cancel_topic` (`/move_base/cancel`), `final_tolerance` (`0.15`).
- Defaults preserve standalone move_base compatibility; the combined launch overrides `path_topic:=/planning/path`.

### 3.3 New files

[src/NavOA_ros/launch/scan_navoa.launch](src/NavOA_ros/launch/scan_navoa.launch) — combined launcher (SCAN pure-planning + `bspline_to_path` + `path_segment_publisher` + NavOA + `vel_filter`), with args `world_frame / base_frame / lidar_frame / body_pose_topic / sensor_pose_topic / cloud_topic / cloud_is_world / depth_topic / scan_topic / cmd_vel_topic / use_rknn`.

`src/SCAN-Planner/src/simulator/CATKIN_IGNORE` — skips the 7 simulator/Utils packages so only the 5 core packages (`scan_planner`, `plan_env`, `path_searching`, `bspline_opt`, `traj_utils`) build in the parent workspace.

---

## 4. Build & run

```bash
cd /home/ubuntu/Robit_SlamNav/NavOA_ws
source /opt/ros/noetic/setup.bash
catkin_make_isolated --use-ninja
source devel_isolated/setup.bash
```

Run the combined system (conda env for NavOA neural net — `control`/`nav` per readme, or `neupan` per the integration notes):

```bash
roslaunch NavOA_ros scan_navoa.launch
# no RKNN runtime on the board:
#   roslaunch NavOA_ros scan_navoa.launch use_rknn:=false
```

---

## 5. Safety check before first real-robot run (Step 2)

Jack up the wheels or disconnect the base enable. Start the system, then check the chain in order:

```bash
rostopic hz /planning/bspline
rostopic hz /planning/path
rostopic echo /split_goal
rostopic hz /NavOA_cmd_vel
rostopic hz /cmd_vel
rostopic info /cmd_vel     # must list vel_filter as the ONLY publisher
```

---

## 6. Troubleshooting

Diagnose in this order (each row is "symptom → most likely cause"):

| Symptom | Cause / fix |
|---|---|
| No `/planning/bspline` | SCAN never planned: check pose topic, depth topic, goal, and `world_frame` TF. |
| B-spline but no `/planning/path` | `bspline_to_path` not running (launched by `scan_navoa.launch`). |
| Path but no `/split_goal` | `path_segment_publisher` `path_topic` wrong — must be `/planning/path`. |
| Sub-goal but no velocity | NavOA TF, `/scan`, model files (`.pth`/`.rknn`), or Python/conda env. |
| Velocity but base doesn't move | `cmd_vel_topic` and base enable state. |
| Obstacles appear in the wrong direction | `laser_frame`, LaserScan zero-angle direction, TF (`world -> base`, `world -> laser`). |
| **Depth cloud rolled 90°** | swap in the roll-corrected `depth_extrinsic_` (alternate matrix in `grid_map.cpp`). |
| SCAN/NavOA disagree on frame | `world_frame`, NavOA `map_frame`, and odom `header.frame_id` must all be the same (`odom`); do not fake the string — fix TF. |
| Robot sways / goals change often | increase `segment_length` toward `1.0`; narrow turning → decrease toward `0.6`. |
| `vel_filter` not the only `/cmd_vel` publisher | a controller is running: check SCAN `controller_mode=none`, no `move_base`, no teleop. |

### Open items to confirm at bring-up

1. **`body_pose_topic` / `sensor_pose_topic` = `/odom`** — placeholder; confirm the leg-odom topic name and its `header.frame_id` is `odom`.
2. **`depth_extrinsic_` roll** — verify at Gate 1; swap the alternate matrix if rolled.
3. **`use_rknn`** — `true` (RK3588 NPU); set `false` if RKNN runtime is absent.
4. **conda env** for NavOA neural net (build is unaffected).
