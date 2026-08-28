# Cartographer 2D Tuning Reference — Biped, 360° 2D LiDAR, Leg Odometry

**Platform:** walking biped, 0.3 m/s, head-mounted 360° 2D scanner (80 Hz `LaserScan`),
leg odometry on `/SMX/Odom_flat` (200 Hz), yesense IMU, RK3588 (8 cores).
**Cartographer:** ROS1, `tracking_frame = base_footprint`, `published_frame = odom`,
`provide_odom_frame = false`.

**Status:** mapping closed. Localization validated over 22 m through feature-poor
area with no flip, no teleport, no drift.

---

## 1. Symptom → cause → parameter

Read this table first. Find the symptom, then read the section it points to.

| Symptom | Actual cause | Parameter |
|---|---|---|
| Geometry erased behind the robot as it walks | No-return beams inserted as long free rays | `missing_data_ray_length` → §3.1 |
| Walls smear / double over a few metres of walking | Gyro bias corrupting the yaw prior | `use_imu_data` → §2.1 |
| Walls smear, IMU already off | Matcher lacks authority to correct the prior | `ceres_scan_matcher` weights → §2.2 |
| Map folds; distant areas stitched onto each other | Constraints proposed beyond sensor range | `max_constraint_distance` → §4.1 |
| Sudden 180° flip in a corridor | Whole-map global search accepting an aliased match | `global_*` → §4.2 |
| Pose jitters one grid cell per scan | Brute-force local search picking coin-flip shifts | `use_online_correlative_scan_matching` → §2.3 |
| "Ignored subdivision … not before current subdivision time" | Sub-timestamps colliding between scans | `num_subdivisions_per_laser_scan` → §3.3 |
| Almost no green matched scan; radial white fans | Matching a 45° wedge instead of a full revolution | subdivisions/accumulation pairing → §3.3 |
| `Queue waiting for data: (0, imu)` | IMU frame has no TF to tracking frame | §2.1 |
| `CHECK failed: sensor_to_tracking->translation().norm() < 1e-5` | IMU frame not colocated with tracking frame | §2.1 |
| Walls grey and soft, structure otherwise correct | Log-odds accumulating too slowly | `hit_probability` → §3.2 |
| Interior speckle; transient objects burned in | Log-odds accumulating too fast | `hit_probability` → §3.2 |
| Pose drifts in featureless stretch, recovers later | Per-scan correction ceiling too low | `occupied_space_weight` → §2.2 |

---

## 2. The scan matcher

### 2.1 `use_imu_data` — the single most consequential parameter here

**Set to `false` on this platform.**

Two facts from `pose_extrapolator.cc`:

1. `AdvanceImuTracker()` uses `angular_velocity_from_odometry_` **only when the IMU
   data queue is empty**. The moment IMU data is present, odometry yaw is ignored
   entirely; odometry then contributes translation only, via `ExtrapolateTranslation`.
2. `ImuTracker` corrects orientation from the gravity vector, which constrains roll
   and pitch. **Yaw is open-loop gyro integration with no bias observability.**

Measured on this robot: gyro-z bias = **−0.135 °/s (−8.1 °/min)** over 11,754
stationary samples. Leg odometry yaw = **0.0004 °/s**. Enabling the IMU replaced the
yaw source with one ~340× worse.

The `robot_localization` EKF is unaffected because it estimates and subtracts gyro
bias as part of its state. Cartographer does not.

**Bias test — run before ever enabling the IMU:**

```bash
timeout 60 rostopic echo -p /imu/data/angular_velocity/z > /tmp/gz.csv
python3 - <<'EOF'
import csv, math
w=[float(r['field']) for r in csv.DictReader(open('/tmp/gz.csv'))]
b=sum(w)/len(w)
print(f"n={len(w)}  bias={math.degrees(b):+.5f} deg/s  ->  {math.degrees(b)*60:+.2f} deg/min")
EOF
```

Pass: under 0.5 °/min. Fail: over 2 °/min — leave the IMU off.
Re-measure warm; MEMS gyro bias is strongly temperature-dependent.

**If you do enable it, two hard requirements:**

- The IMU `frame_id` must have a TF to `tracking_frame`, or every sample is dropped
  (`Queue waiting for data: (0, imu)`).
- That TF's **translation must be zero** (`norm() < 1e-5`) or Cartographer aborts at
  startup. Cartographer rotates IMU data into the tracking frame but does not
  translate linear acceleration. Publish `0 0 0 <real rotation>`. Zeroing the
  translation is safe in 2D: angular velocity is translation-invariant, gravity
  direction is translation-invariant, and the centripetal contamination it ignores
  only affects the gravity estimate, which is averaged over
  `imu_gravity_time_constant = 10 s`.

### 2.2 `ceres_scan_matcher` weights — the authority ceiling

From `ceres_scan_matcher_2d.cc`, the occupied-space residual is scaled by
`occupied_space_weight / sqrt(point_cloud.size())` per point. With N points:

```
C_occ   = Σ (w_occ/√N)² (1−Mᵢ)²  =  w_occ² · mean[(1−M)²]
C_rot   = w_rot²   · Δθ²
C_trans = w_trans² · Δd²
```

**Key consequence:** `mean[(1−M)²] ≤ 1`, so total scan evidence is **bounded by
w_occ²**. Setting `C_rot = w_occ²` gives the maximum correction the matcher can ever
buy in one scan:

```
Δθ_max = w_occ / w_rot        Δd_max = w_occ / w_trans
```

| `w_occ` | `w_rot` | max yaw correction | `w_trans` | max translation correction |
|---|---|---|---|---|
| 1.0 | 200 | 0.29° | 20 | 5 cm |
| 1.0 | 40 (stock) | 1.43° | 10 (stock) | 10 cm |
| **2.0** | **40** | **2.86°** | **10** | **20 cm** |
| 1.0 | 8 | 7.2° | 4 | 25 cm |

**Only the ratio matters.** Raising `w_occ` scales both ceilings together, which is
usually what you want; lowering `w_rot` alone changes the rotation/translation balance.

Relation to visible smear: lateral error = `R · θ`. At 8 m, 1° = 14 cm, 2° = 28 cm.
So a matcher capped at 0.29°/scan cannot remove a 2° accumulated error faster than a
drifting prior re-injects it — a ratchet.

**Mapping used 1.0 / 10 / 40. Localization uses 2.0 / 10 / 40.**

Rule: mapping and localization should use the **same** weights by default — a matcher
tuned differently from the one that drew the map disagrees with it systematically.
Deviating upward in localization is acceptable because the map is frozen: a bad match
can only move the pose, not corrupt the map.

### 2.3 `use_online_correlative_scan_matching`

**Mapping: `true`. Localization: `false`.**

The RTCSM is a brute-force local search. With a poor prior it's a necessary crutch.
With a good prior it *adds* noise — it picks whichever nearby integer-pixel shift
scores marginally higher this scan versus last, changing the pose by a grid cell on a
coin flip. Symptom is walls thick and blurry rather than cleanly doubled.

---

## 3. Range data and the grid

### 3.1 `missing_data_ray_length` — a licence to erase

A beam that returns nothing (out of range, dark surface, glossy surface) is **not
discarded**. It is inserted as a free ray of this length. Every dropout bulldozes
that far into the map.

**Rule: `missing_data_ray_length` must be shorter than the distance at which you
still trust your beam plane.** It is not a licence to see.

Set to **1.5** here. At 5.0 it produced the radial white streaks and erased geometry
behind the robot.

### 3.2 `hit_probability` / `miss_probability` — grid stickiness

Cartographer updates in log-odds. `hit_probability = p` gives per-hit odds ratio
`p/(1−p)`. Net hits to drive a cell from 0.5 to the 0.9 clamp:

| `hit_probability` | odds | net hits to p = 0.9 |
|---|---|---|
| 0.55 | 1.222 | 11 |
| **0.58** | 1.381 | **6.8** |
| 0.60 | 1.500 | 5.4 |
| 0.62 | 1.857 | 4.5 |
| 0.65 | 1.857 | 3.5 |

`miss_probability = 0.49` is held constant: ~55 consecutive misses to decay 0.9 → 0.5.
That asymmetry is the point — sharp *and* sticky.

**Chose 0.58.** 0.62 and 0.65 darkened walls but burned transients into open floor as
speckle, and on a navigation map each speckle becomes an inflated phantom obstacle.
Go to 0.62 only if genuine thin obstacles (chair legs, poles) fail to register.

Note the geometric floor this cannot fix: beam spacing along a wall is `R · Δφ`. At
1° angular resolution that's 3.5 cm at 2 m but 14 cm at 8 m, so distant walls are hit
in only ~1 of 3 cells per scan and always fill in more slowly.

### 3.3 `num_subdivisions_per_laser_scan`

**Set to 1 on this platform.**

**The pairing rule: if you subdivide by N, you must set
`num_accumulated_range_data = N`.** Subdividing splits one revolution into N wedges
delivered as *separate range data*. The matcher runs on whatever accumulates — so
with accumulation at 1, it tries to match a 45° wedge (two short wall fragments,
no positional constraint) against the map. Result: no green matched scan, and a
radial fan of free-space cones inserted at N different wrong poses.

Even paired correctly it isn't worth it here. Deskew corrects *yaw* smear during
turning; at 80 Hz there is negligible ego-motion within one scan. It also caused
"Ignored subdivision … not before current subdivision time" — interpolated
sub-timestamps colliding when two scans arrive tens of microseconds apart.

Revisit only if the map degrades specifically during turns and stays clean on straights.

### 3.4 `num_accumulated_range_data`

**Set to 1.** Accumulation densifies a *non-repetitive* (Livox-style) pattern. With a
repetitive 2D scanner it only adds latency and motion smear — no new geometry.

### 3.5 `max_range` / `min_range`

`max_range = 8.0`, `min_range = 0.3`.

- `min_range` must exceed the robot's own worst-case self-return. At 0.1 the head
  lidar risks inserting the robot's body as an obstacle that rotates with it.
- `max_range` must not be raised in localization beyond what the map was built with:
  far returns then land on unknown cells and add a constant biased penalty. Change it
  in mapping and localization **together**.
- Slice geometry bounds it too. A 2D plane on a pitching head sweeps `R·sin(θ)` of
  wall height. At 5° pitch: ±0.35 m at 4 m, ±0.70 m at 8 m, ±1.05 m at 12 m. 12.0 was
  past this platform's slice stability.

---

## 4. The pose graph

### 4.1 `max_constraint_distance` — the range rule

**`max_constraint_distance` must be BELOW `max_range`.** Set to **6.0** against
`max_range = 8.0`.

Two submaps further apart than the sensor can see share **no observable geometry**.
Every constraint proposed at that distance is one piece of corridor matched against a
different piece that happens to look the same. At 15.0 (with `max_range = 8.0`) the
RViz Constraints display showed a dense fan of long constraints bundled along one
band — the fingerprint of a self-similar corridor being stitched to itself.

The failure at the other end is real too: at 2.0 you must re-approach within two
metres of the original submap centre before a constraint is even *considered*, so
loops never close and drift accumulates unbounded.

### 4.2 The teleport controls

A 180° flip **cannot** come from the local matcher — its angular search window is
±20°. Only whole-map global search, which matches over ±180°, can produce one.

| Parameter | Value | Why |
|---|---|---|
| `global_sampling_ratio` | 0.003 | How often a node is thrown at the whole map. 0.01 was 3× stock. |
| `global_constraint_search_after_n_seconds` | 60 | **The panic trigger.** Default 10 s is about how long it takes to walk a blank corridor — so the corridor itself fires the teleport. |
| `global_localization_min_score` | 0.70 | At 0.45 a symmetric-corridor hypothesis clears threshold easily. |
| `min_score` | 0.65 | Local constraint floor. |

**Trade-off:** this configuration will not rescue itself from a kidnap. That is
deliberate — the rescue mechanism *was* the bug. Start trajectories with a known
initial pose (`/initialpose` or launch args) rather than relying on global search.

### 4.3 Optimizer robustness

| Parameter | Value | Why |
|---|---|---|
| `huber_scale` | 5e0 | At 1e2 the Huber loss is effectively quadratic everywhere — **no outlier rejection at all**, so one bad constraint dominates the solve. |
| `optimize_every_n_nodes` | 35 mapping / 20 localization | At 90 with ~1 node/s, ~90 s of drift accumulates before anything is pulled straight. |
| `odometry_translation_weight` | 1e4 | 1e2/1e1 discards a good source; visibly worse maps. |
| `odometry_rotation_weight` | 1e4 | |

**Non-obvious:** odometry weights are *intra-trajectory* — they link consecutive
nodes. A rigid 180° flip of the whole track against the frozen map satisfies every one
of them perfectly. **Odometry weight cannot prevent a global flip.** Only constraint
admission (§4.2) can. What it does buy is a rigid track, which is harder to distort
and snaps back cleanly once a good constraint arrives.

### 4.4 `motion_filter`

Mapping default (0.2 m / 1° / 5 s) is too coarse: at 0.3 m/s that's one node per
0.66 s, too few anchors for the graph. Localization uses **0.10 m / 1° / 0.5 s**.

Do not go to 0.02 m / 0.5° / 0.1 s — that floods the graph for no information gain,
and CPU starvation degrades matching on its own.

---

## 5. CPU

`num_background_threads = 4`, and **explicitly pin
`POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 3`** — the stock
`pose_graph.lua` sets it to 7, which fights the background thread limit.

Watch **system** load, not Cartographer's own %CPU:

```bash
top -H -p $(pgrep -f cartographer_node)
ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -20
```

Load average 10 on 8 cores was observed. Cartographer showed only 12% because it was
*scheduling-limited*, not compute-limited: the scan callback was randomly delayed
5–50 ms, desynchronising extrapolated pose from scan timestamp. Run RViz on a laptop
over `ROS_MASTER_URI`, not on the robot.

---

## 6. Final values

### Mapping (`livox_2d.lua`)

```lua
num_subdivisions_per_laser_scan = 1
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.5
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight    = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight       = 40.0
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter
  .probability_grid_range_data_inserter.hit_probability  = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter
  .probability_grid_range_data_inserter.miss_probability = 0.49
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.optimization_problem.huber_scale = 5e0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight    = 1e4
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.15
POSE_GRAPH.constraint_builder.max_constraint_distance = 6.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 4.0
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
```

### Localization (deltas from the above)

```lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds    = 0.5
TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 60.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 3
```

---

## 7. Method notes

Things that cost time in this investigation and are worth not repeating.

1. **Change one parameter per run.** Several runs moved three or four variables at
   once and produced results that could not be attributed to anything.
2. **A duplicate key in Lua resolves silently — last wins.** Both
   `num_subdivisions_per_laser_scan` and `huber_scale` ended up defined twice. Grep
   for duplicates before blaming a value.
3. **Measure before hypothesising.** Every wrong turn here came from reasoning about
   symptoms; the root cause was found in 60 seconds by integrating a stationary gyro.
   The per-jump attribution table (displacement, yaw, source per event) was more
   diagnostic than any screenshot.
4. **`tf_echo` printing `At time 0.000`** means the transform chain does not exist.
   It is not a stale value — it is a fallback.
5. **Use the RViz Constraints display**, namespace *"Inter residuals, same
   trajectory"*. Short residual stubs = healthy loop closures. Long ones = the
   optimizer being pulled between constraints that disagree. Far more informative than
   watching the map.
6. **Watch accepted match scores** (`log_matches = true`, on by default). If they
   cluster just above `min_score`, the threshold is admitting borderline junk — raise it.
7. **When config stops helping, the answer is physical.** A 2D slice on a pitching
   head, or a corridor with no distinguishable geometry, contains information no
   parameter can recover. Landmarks, reflective tape, or moving the sensor to the
   torso are legitimate answers.