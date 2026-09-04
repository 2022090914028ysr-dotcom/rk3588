# SCAN-Planner Parameter Tuning Guide

Robot: HighTorque Mini Pi biped. Depth camera (ascamera_hp60c) at 0.36 m above floor, landscape, level.
Footprint 0.25 × 0.30 m. `base_link` at 0.45 m, `base_footprint` at ground (odom z = 0).

All parameters live in `src/SCAN-Planner/src/planner/plan_manage/launch/advanced_param.xml`
unless noted. Values shown are the **current** settings.

---

## How to use this guide

Change **one parameter at a time**, restart `scan_navoa.launch`, and observe.
If you change two and the behaviour improves, you don't know which one did it — and
this project has already lost days to exactly that mistake.

For every test, know what you expect to see *before* you run it. If the result
surprises you, that's information; if you didn't have an expectation, it's noise.

---

## 1. Occupancy mapping — what becomes an obstacle

These decide what the robot *sees*. Get these wrong and everything downstream is
garbage, no matter how well the planner is tuned.

### `grid_map/min_obstacle_height` = 0.15

**What it does.** Any depth return whose world z is less than this above
`ground_height` is treated as floor and marked FREE. Everything taller is a
candidate obstacle. This is the ground filter that was added to `raycastProcess()`
to stop the floor being painted as a wall.

**Increase (e.g. 0.15 → 0.25):** more floor noise is ignored. Safer against
false floor obstacles and the "ring" failure. **But** the robot goes blind to
anything shorter than the threshold — a 20 cm step, a cable tray, a low box.

**Decrease (e.g. 0.15 → 0.08):** low obstacles start being detected. **But** you
approach the sensor's floor-noise band (measured at ~0.07–0.11 m on this robot),
and false floor obstacles reappear — the robot encircles itself and A* fails with
`a star error, force return!`.

**Symptom → fix.**
- Ring of obstacles around a stationary robot in an empty room → **increase**.
- Robot walks into low objects it should see → **decrease**, but re-test the ring.

**Safe range here:** 0.12 – 0.25. Below 0.12 the floor noise wins.

---

### `grid_map/ground_height` = 0.05

**What it does.** Defines where "the floor" is in the map's z frame, and sets the
map origin z. `min_obstacle_height` is measured *relative to this*.

**Important on this robot:** `base_footprint` is at ground level and odom z = 0,
so the physical floor is at z = 0. The 0.05 here is a small tolerance, not a
position. Effective obstacle cutoff = `ground_height + min_obstacle_height` =
0.05 + 0.15 = **0.20 m**.

**Increase:** raises the whole floor reference; fewer near-floor returns count as
obstacles. Equivalent in effect to raising `min_obstacle_height`.

**Decrease toward 0:** tighter to the true floor. More sensitive to noise.

**Don't** set this to a large negative value. That was tried during debugging (−0.45,
based on `base_link` height) and it put the map origin below the real floor,
placing legitimate floor returns inside the obstacle band.

---

### `grid_map/resolution` = 0.05

**What it does.** Voxel size in metres. Everything else — inflation steps, raycast
cost, memory — scales off this.

**Increase (0.05 → 0.10):** ~8× less memory and much faster raycasting, but the
map gets coarse. Narrow gaps blur shut; a 0.30 m doorway gap represented in 0.10 m
voxels has very little margin.

**Decrease (0.05 → 0.025):** finer detail, better gap traversal. Costs ~8× memory
and CPU. On the RK3588 this is a real constraint — watch the update rate on
`/grid_map/occupancy_inflate` (should stay near 20 Hz).

**Leave at 0.05** unless you have a specific gap-traversal problem and CPU headroom.

---

### `grid_map/p_hit` = 0.85, `p_miss` = 0.30, `p_occ` = 0.80

**What they do.** Log-odds fusion. Each map update cycle, a voxel gets one vote:
"hit" (something is here) or "miss" (ray passed through). `p_hit`/`p_miss` set how
much one vote moves the voxel's belief; `p_occ` is the threshold at which a voxel
counts as occupied for planning.

Current log values (printed at startup): hit `+1.735`, miss `−0.847`, occupied
threshold `+1.386`, clamps `−1.992` to `+3.892`.

**Raise `p_hit`** (0.85 → 0.95): obstacles appear faster, in fewer frames. Also
means a single noisy return can create an obstacle. More false positives.

**Lower `p_hit`** (0.85 → 0.70): needs more consistent evidence before believing
an obstacle. Slower to react to something that suddenly appears — dangerous for a
moving robot.

**Raise `p_miss`** toward 0.5: obstacles clear faster once they're gone. Stale
obstacles disappear quickly, but real obstacles also flicker out when partially
occluded.

**Lower `p_miss`** (0.30 → 0.15): obstacles persist longer. Good for stability,
bad if the map fills with ghosts of things that have moved.

**Raise `p_occ`** (0.80 → 0.90): more evidence needed before the planner treats a
voxel as blocking. Fewer false obstacles, slower to react.

**Rule of thumb:** at 20 Hz with the current values, a voxel goes from fully free
to occupied in about 2 cycles (~100 ms), and from fully occupied to free in about
3 cycles (~150 ms). That's a good balance for walking speed. Change these only if
you see specific flicker or lag.

---

### `grid_map/max_ray_length` = 5.0

**What it does.** Maximum distance a ray is traced for free-space carving. Points
beyond this are clamped and marked FREE, not occupied.

Relevant detail on this robot: depth pixels that return raw = 0 (no measurement —
ceiling, out of range, absorbing surface) get projected to a synthetic
`max_ray_length + 0.1` = 5.1 m sheet. Those correctly map to FREE, so the sheet
does *not* create obstacles — but it does mean you'll see a flat point layer at
5.1 m in `/grid_map/depth_cloud`. That is expected, not a bug.

**Increase:** longer free-space carving; the map clears more aggressively at range.
Costs raycast time.

**Decrease:** cheaper, but less clearing far out — stale obstacles linger past
the shorter horizon.

---

### `grid_map/depth_filter_maxdist` = 2.5, `depth_filter_mindist` = 0.25

**What they do.** Depth returns outside this range are discarded before
deprojection. Nothing closer than 0.25 m or further than 2.5 m enters the map.

**`maxdist` increase** (2.5 → 3.5): see obstacles further out, plan earlier around
them. **But** depth accuracy on this camera degrades past ~2.5 m — you'd be adding
noisy points. The camera's usable range was measured at 2–3 m.

**`maxdist` decrease** (2.5 → 1.5): only trust close returns. Cleaner map, but the
planner learns about obstacles late, which at walking speed means abrupt avoidance.

**`mindist` increase** (0.25 → 0.4): rejects very close returns. Useful if the
robot's own body or a mount edge appears in frame.

**`mindist` decrease:** risk of self-observation. 0.25 is reasonable for this mount.

**Interaction to remember:** `maxdist` must be ≤ the camera's honest range, and
`planning_horizon` should not greatly exceed `maxdist` — the planner shouldn't plan
into space it has no data for.

---

### `grid_map/skip_pixel` = 2

**What it does.** Process every Nth pixel of the depth image. 2 means half the
rows and half the columns → ~1/4 the points.

**Increase (2 → 4):** ~4× faster deprojection, less CPU. Thin obstacles (a chair
leg, a pole) may be missed entirely because the sampling skips over them.

**Decrease (2 → 1):** every pixel processed. Best detection of thin objects, ~4×
the CPU cost. On RK3588 watch that map update rate stays near 20 Hz.

**If the robot misses narrow vertical obstacles, try 1 before touching anything else.**

---

### `grid_map/obstacles_inflation_z_up` = 0.1, `obstacles_inflation_z_down` = 0.10

**What they do.** How far each occupied voxel is inflated vertically — *upward* and
*downward*. This is **not** a ground filter (a mistake made earlier in this project);
it makes obstacles taller and deeper, not fewer.

**Increase:** obstacles become vertically larger. More conservative — the robot
avoids things with more margin above and below. Can cause an obstacle at 0.25 m to
inflate down into the floor band.

**Decrease:** tighter vertical bounds around real obstacles.

For a ground robot that plans in 2D-ish, these matter less than the horizontal
inflation. Leave them small.

---

### `grid_map/sliding_map_size_x / _y / _z` = 10.0 / 10.0 / 5.0

**What they do.** Dimensions of the robot-centred sliding map. The map follows the
robot; anything outside is forgotten.

**Increase:** more memory of the surroundings, useful for backtracking out of dead
ends. Costs memory: doubling x and y quadruples the voxel count.

**Decrease:** cheaper, but the robot forgets obstacles sooner — it can turn around
and re-discover a wall it just passed.

10 × 10 m at 0.05 m resolution is already 200 × 200 × 100 voxels. Be careful going
larger on this hardware.

---

### `grid_map/map_sliding_thresh` = 0.2

**What it does.** How far the robot must move before the sliding map re-centres.

**Increase:** fewer re-centre operations (cheaper), but the robot can drift toward
the edge of its own map.

**Decrease:** map stays tightly centred, more frequent shifting cost.

---

## 2. Robot shape — collision checking

### `grid_map/double_cylinder_radius` = 0.15

**What it does.** Radius of the collision cylinders SCAN uses to model the robot.
This is the core of SCAN-Planner's yaw-aware twin-cylinder footprint.

Your robot is 0.25 × 0.30 m, so half-width ≈ 0.15 m. The current value matches the
body exactly — **with zero safety margin**.

**Increase (0.15 → 0.20):** the planner keeps more clearance from obstacles. Safer,
fewer scrapes. **But** narrow gaps become unpassable: a 0.5 m gap with 0.20 m
radius leaves only 0.05 m either side after the cylinder, and the planner may
refuse to enter.

**Decrease (0.15 → 0.12):** squeezes through tighter gaps, but the real body is
wider than the model — you *will* clip obstacles.

**For first testing, use 0.18–0.20.** Get reliable avoidance first, tighten later
when you specifically need gap traversal.

---

### `grid_map/double_cylinder_offset` = 0.0

**What it does.** Distance between the two cylinder centres along the body axis.
0.0 collapses it to a single cylinder — no yaw-awareness.

**Increase (0.0 → 0.10):** models an elongated body; the collision check becomes
orientation-dependent, which is the whole point of SCAN's twin-cylinder design.
The robot understands it can fit through a gap sideways but not head-on.

For a nearly-square footprint (0.25 × 0.30) the offset genuinely should be small.
0.0 is defensible here. Raise it only if you find the robot's length mattering.

---

## 3. Planner behaviour

### `fsm/planning_horizon` / `manager/planning_horizon` = 3.5

**What it does.** How far ahead SCAN plans in one trajectory.

**Increase (3.5 → 5.0):** smoother, more globally sensible paths. **But** you're
planning into space beyond `depth_filter_maxdist` (2.5 m) — the far part of the
path is planned through *unknown* space, and gets re-planned constantly as data
arrives.

**Decrease (3.5 → 2.0):** every planned metre is backed by real sensor data.
Paths are more reactive and less smooth; the robot may not commit to going around
a large obstacle because it can't see past it.

**Recommendation for this robot:** keep it at or below **2.5–3.0**, matching the
camera's honest range. Planning further is planning on faith.

---

### `fsm/thresh_replan` = 1.0, `fsm/thresh_no_replan` = 0.1

**What they do.** `thresh_replan` — replan when the goal moves more than this.
`thresh_no_replan` — don't bother replanning if the change is smaller than this.

**Lower `thresh_replan`:** more frequent replanning, more responsive to a changing
goal, more CPU.

**Raise `thresh_no_replan`:** fewer pointless replans when the goal jitters.

Rarely needs tuning for a fixed goal.

---

### `fsm/emergency_time_` = 1.0

**What it does.** Time horizon for the emergency-stop check. If a collision is
predicted within this many seconds along the current trajectory, the FSM triggers
`EMERGENCY_STOP`.

**Increase (1.0 → 2.0):** stops earlier, more conservative. Can cause frequent
unnecessary stops in cluttered space — you saw the `Suddenly discovered obstacles.
emergency stop!` loop when the map had false obstacles.

**Decrease (1.0 → 0.5):** fewer nuisance stops, less reaction time.

At 0.20 m/s, 1.0 s is 0.20 m of stopping distance. That is thin. If the robot
starts hitting things at speed, raise this before anything else.

---

### `manager/control_points_distance` = 0.2

**What it does.** Spacing of B-spline control points. Effectively the trajectory's
spatial resolution.

**Increase (0.2 → 0.4):** fewer control points, faster optimisation, smoother but
less agile paths. The trajectory can't make tight turns.

**Decrease (0.2 → 0.1):** finer control, sharper avoidance manoeuvres possible,
slower to optimise and more prone to wiggly paths.

**If the robot can't turn tightly enough around obstacles, decrease this.**

---

### `manager/feasibility_tolerance` = 0.5

**What it does.** How much velocity/acceleration violation is tolerated before the
trajectory is rejected as dynamically infeasible.

You saw this in the logs: `This refined trajectory is unsafe or dynamically
infeasible. Skip publishing it.`

**Increase (0.5 → 1.0):** more trajectories accepted. Fewer planning failures, but
the robot may be commanded beyond its real limits.

**Decrease (0.5 → 0.2):** stricter. More `final_plan_success=0` and more replan
cycles, but every published trajectory is comfortably within limits.

**If you see frequent "dynamically infeasible" messages, increase this OR lower
`max_vel`/`max_acc`.** The second is usually the right answer.

---

## 4. Trajectory optimisation weights

These four lambdas are the relative importance of competing objectives. **Only the
ratios matter** — multiplying all four by 10 changes nothing.

### `optimization/lambda_smooth` = 1.0

Penalises jerky, wiggly trajectories.

**Increase:** smoother, more graceful paths. Pushed too far, the path becomes so
committed to smoothness that it will not deviate around obstacles.

**Decrease:** the path can turn sharply, at the cost of comfort and tracking error.

---

### `optimization/lambda_collision` = 1.0

**The most important one for your current problem.** Penalises proximity to
obstacles.

**Increase (1.0 → 10.0 → 100.0):** the path is pushed hard away from obstacles.
This is the first thing to raise if the planner draws paths straight through
visible obstacles.

**Decrease:** the path tolerates passing close to (or through) obstacles.

**Diagnostic use:** if setting this to 100 makes the robot avoid obstacles, the
problem was a weight imbalance. If setting it to 100 changes *nothing*, the
collision cost is not reading the occupancy map at all — that is a code bug, not
a tuning problem, and no parameter will fix it.

---

### `optimization/lambda_feasibility` = 0.1

Penalises velocity/acceleration limit violations during optimisation.

**Increase:** trajectories respect dynamic limits more strictly; fewer rejections
downstream at the feasibility check.

**Decrease:** the optimiser ignores dynamics and leaves it to the feasibility
check to reject — more `infeasible` messages.

---

### `optimization/dist0` = 0.2

**What it does.** The clearance distance the collision cost aims for. Obstacles
closer than `dist0` to the trajectory incur cost; beyond it, none.

**Increase (0.2 → 0.4):** the planner keeps a wider berth. Combined with
`double_cylinder_radius = 0.15`, a `dist0` of 0.4 means the path aims to stay
0.4 m clear of obstacle *surfaces*.

**Decrease:** paths hug obstacles more closely. Necessary for tight gaps, risky
otherwise.

**This works together with `lambda_collision`.** `dist0` sets *where* the penalty
starts; `lambda_collision` sets *how hard* it pushes. If paths clip obstacles,
raising `dist0` is often more effective than raising the lambda, because it widens
the region the optimiser is even aware of.

---

## 5. Speed limits

### `max_vel` = 0.20, `max_acc` = 0.20

Set once at the top of `advanced_param.xml` and propagated to
`manager/`, `optimization/`, and `closed_loop_controller/`.

**Increase:** faster traversal. Everything gets harder — less reaction time, more
feasibility rejections, larger stopping distance relative to `emergency_time_`,
more demand on the leg controller.

**Decrease:** safer, more forgiving of planning latency and map lag.

**For a biped doing first autonomous runs, 0.15–0.20 m/s is right.** Don't raise
this until obstacle avoidance is reliably working at low speed.

Note `vel_filter` independently caps at 0.30 m/s / 1.5 rad/s as a final safety
layer, and it is the sole publisher to `/cmd_vel`.

---

## 6. Tuning recipes for common symptoms

### Robot walks straight through visible obstacles
1. `lambda_collision` 1.0 → 100.0. Restart. Test.
   - Path now curves → it was a weight problem; tune down to find the balance.
   - No change → collision cost isn't reading the map; this is a code bug.
2. If (1) works, also raise `dist0` 0.2 → 0.3 for margin.
3. Raise `double_cylinder_radius` 0.15 → 0.20 so the model matches the real body
   plus margin.

### Ring of false obstacles around a stationary robot
1. Raise `min_obstacle_height` 0.15 → 0.20.
2. Verify the camera extrinsic still puts the floor at z ≈ 0.
3. Check `ground_height` matches where the floor actually is in odom (0.0–0.05 here).

### `a star error, force return!` on every cycle
The robot is enclosed by obstacles in its own map. Almost always false obstacles —
go to the ring recipe above. Check `/grid_map/occupancy_inflate` with the robot
stationary in an open area: the robot's own cell must be free.

### "This refined trajectory is unsafe or dynamically infeasible"
1. Lower `max_vel` / `max_acc` (0.20 → 0.15).
2. Or raise `manager/feasibility_tolerance` (0.5 → 0.8).
3. Or raise `lambda_feasibility` (0.1 → 0.5) so the optimiser respects limits earlier.

### Robot misses thin obstacles (chair legs, poles)
1. `skip_pixel` 2 → 1.
2. Consider `resolution` 0.05 → 0.04 if CPU allows.

### Robot won't fit through a gap it should fit through
1. Lower `double_cylinder_radius` toward the true half-width (0.15).
2. Lower `dist0` (0.2 → 0.1).
3. Lower `control_points_distance` (0.2 → 0.1) so the path can turn sharply enough.

### Emergency stops constantly in normal space
1. Check for false obstacles first — this is usually a map problem, not a
   parameter problem.
2. If the map is clean, lower `emergency_time_` (1.0 → 0.7).

---

## 7. Known-good baseline for this robot

If tuning goes wrong, return to this:

```xml
<!-- speeds -->
max_vel = 0.20
max_acc = 0.20

<!-- map -->
grid_map/resolution               = 0.05
grid_map/min_obstacle_height      = 0.15
grid_map/ground_height            = 0.05
grid_map/depth_filter_maxdist     = 2.5
grid_map/depth_filter_mindist     = 0.25
grid_map/skip_pixel               = 2
grid_map/max_ray_length           = 5.0
grid_map/p_hit / p_miss / p_occ   = 0.85 / 0.30 / 0.80

<!-- robot shape -->
grid_map/double_cylinder_radius   = 0.15
grid_map/double_cylinder_offset   = 0.0

<!-- planner -->
fsm/planning_horizon              = 3.5
fsm/emergency_time_               = 1.0
manager/control_points_distance   = 0.2
manager/feasibility_tolerance     = 0.5

<!-- optimisation -->
optimization/lambda_smooth        = 1.0
optimization/lambda_collision     = 1.0
optimization/lambda_feasibility   = 0.1
optimization/lambda_fitness       = 1.0
optimization/dist0                = 0.2
```

---

## 8. Things that are NOT tuning parameters

Learned the hard way during this integration — if these are wrong, no amount of
parameter tuning will help:

- **Camera extrinsic** (`sensors.launch` static transform). Must be
  `0.10 0.00 -0.09` with the standard optical rotation
  `(-0.5, 0.5, -0.5, 0.5)`. Adding "pitch" to the Euler middle angle with
  yaw = −90 / roll = −90 does **not** pitch the camera — it rolls the image about
  the forward axis. Days were lost to this.
- **TF chain.** `odom → base_footprint (ground, z=0) → base_link (+0.45) →
  camera (−0.09)`. The EKF must be the sole publisher of the odom TF.
- **Odometry topic.** SCAN's `body_pose_topic` / `sensor_pose_topic` must point at
  a topic that is actually publishing. `[FSM]: state: INIT / no odom.` means it
  isn't — nothing else works until that's fixed.
- **Camera mount orientation.** Landscape, level. A portrait mount requires
  rotating the image *and* swapping the intrinsics together; doing only one is
  worse than doing neither.

---

## 9. Test protocol

Always in this order. Do not skip ahead — a failure at step 2 makes steps 3–5
meaningless.

1. **Stationary, open area, no goal, 30 s.** Watch `/grid_map/occupancy_inflate`.
   Pass: no obstacles around the robot, its own cell free.
2. **Stationary with an obstacle 1 m ahead.** Pass: obstacle appears in the map at
   roughly the right position and height.
3. **Straight-line goal, 1.5 m, open space.** Pass: `final_plan_success=1`, robot
   moves and arrives.
4. **Goal past an obstacle.** Pass: the planned path visibly routes around the
   obstacle, and the robot follows it.
5. **Goal requiring a 90° turn.** Turning is what accumulates false obstacles, so
   this is where map problems resurface. Pass: no ring, no `a star error`.

Only after all five pass is it worth running longer traversals or raising speed.