# Spec: `localization_jump_guard` — map→odom failure detection and gating

**Target:** ROS1 (Noetic), Python, package `Robit_SlamNav/NavOA_ws`.
**Purpose:** allow Cartographer pure-localization to run with aggressive global
relocalization enabled, while rejecting the false ~180° relocalizations it
occasionally accepts.

---

## 1. Problem

Cartographer's pose graph accepts any global constraint scoring above
`global_localization_min_score` and applies it with no cross-check. On a map with a
180°-symmetric corridor, a wrong hypothesis clears the threshold. The result is a
step change in the `map → odom` transform — measured on this robot at **4.29 m /
174.2°** and **4.23 m / 177.9°** in a single update.

**The discriminating fact:** `map → odom` is a *drift correction*. It does not change
when the robot moves — that motion lives in `odom → base_footprint`. Legitimate
corrections are centimetres and fractions of a degree, applied gradually. A 4 m
step in `map → odom` therefore means the estimate teleported while the robot did
not, and leg odometry (validated at 0.0004 °/s yaw) contradicts it directly.

That contradiction is machine-checkable. Cartographer does not check it. This node does.

---

## 2. Architecture

Cartographer broadcasts `map → odom` directly to `/tf`. A guard node **cannot**
intercept or suppress another node's TF broadcast. So Cartographer must be stopped
from publishing it, and the guard becomes the sole publisher.

```
                    ┌────────────────────┐
  /scan, /odom ───► │  cartographer_node │
                    │  publish_to_tf =   │
                    │       FALSE        │
                    └─────────┬──────────┘
                              │ /tracked_pose   (PoseStamped, frame_id = "map",
                              │                  pose of base_footprint in map)
                              ▼
                    ┌────────────────────┐
   /tf: odom ──────►│ localization_      │
   → base_footprint │   jump_guard       │──► /tf:  map → odom   (gated)
   (leg odom / EKF) │                    │──► ~/status, ~/rejections
                    └────────────────────┘
```

### Prerequisite config change

In `livox_2d_localization.lua`:

```lua
options.publish_to_tf        = false   -- guard becomes the sole map->odom publisher
options.publish_tracked_pose = true    -- already set; this is now load-bearing
```

**Verify `publish_to_tf` exists in this Cartographer version first:**

```bash
grep -rn "publish_to_tf" $(rospack find cartographer_ros)/../  2>/dev/null
grep -rn "publish_to_tf" ~/Robit_SlamNav/NavOA_ws/src/carto_ws/src/cartographer_ros/
```

If the option is absent, do **not** attempt to remap `/tf` for `cartographer_node` —
tf2 remapping affects both the broadcaster and the listener, which breaks
Cartographer's own `odom → base_footprint` lookups. Instead apply a two-line patch in
`cartographer_ros/node.cc`, guarding the `tf_broadcaster_.sendTransform(...)` call in
`PublishLocalTrajectoryData()` behind a ROS param.

---

## 3. Core algorithm

### 3.1 Computing the candidate transform

On each `/tracked_pose` message:

```
T_map_base   = pose from /tracked_pose                (map → base_footprint)
T_odom_base  = tf lookup(odom, base_footprint, at pose.header.stamp)
T_map_odom   = T_map_base · inverse(T_odom_base)      ← candidate
```

Use `tf.transformations`:

```python
M_map_base  = concatenate_matrices(translation_matrix(t_mb), quaternion_matrix(q_mb))
M_odom_base = concatenate_matrices(translation_matrix(t_ob), quaternion_matrix(q_ob))
M_map_odom  = concatenate_matrices(M_map_base, inverse_matrix(M_odom_base))
```

Look up `odom → base_footprint` **at the pose's own timestamp**, not `Time(0)`.
Mismatched timestamps inject spurious deltas that look like jumps.

### 3.2 Gate

Compare candidate against `held` (the currently published transform):

```
Δd = ‖ translation(candidate) − translation(held) ‖        (2D: x, y only)
Δθ = wrap_to_pi( yaw(candidate) − yaw(held) )             (absolute value)
```

```
if not locked:                      # startup, or after reset_lock
    accept  → held = candidate; locked = True
elif Δd ≤ max_jump_translation and Δθ ≤ max_jump_rotation:
    accept  → held = candidate                            # normal small correction
else:
    reject  → held unchanged; run persistence check (3.3)
```

### 3.3 Persistence override — do not get permanently stuck

A pure reject rule fails if Cartographer is *right* and the guard is wrong: the robot
stays wrongly localized forever. Override:

- Maintain a buffer of consecutive **rejected** candidates.
- If every candidate in the buffer agrees with the others to within
  `persistence_radius_m` / `persistence_radius_rad`, **and** the buffer spans more
  than `persistence_seconds`, then Cartographer is consistently asserting a different
  pose — accept it and log at WARN.
- Any accepted candidate clears the buffer.

Rationale: a spurious flip is a single bad constraint that later constraints disagree
with. A genuine relocalization is asserted consistently by every subsequent node.

**Known limitation to document in the code:** a *persistent* wrong relocalization
still passes this rule. This node reduces flip frequency; it does not make the map
unambiguous. The durable fix is breaking the map's 180° symmetry physically.

### 3.4 Publishing

Publish `held` on a timer at `publish_rate` (default 100 Hz, matching the existing
`pose_publish_period_sec = 0.01`), **not** only on update — TF must not go stale.

Stamp each broadcast `rospy.Time.now() + transform_tolerance` (default 0.1 s), the
same forward-dating AMCL uses, so consumers looking up slightly ahead don't fail.

---

## 4. Parameters

| Param | Type | Default | Notes |
|---|---|---|---|
| `~map_frame` | str | `map` | |
| `~odom_frame` | str | `odom` | |
| `~base_frame` | str | `base_footprint` | must equal Cartographer's `tracking_frame` |
| `~tracked_pose_topic` | str | `/tracked_pose` | |
| `~max_jump_translation` | float | `0.5` | metres, single update |
| `~max_jump_rotation` | float | `30.0` | degrees, single update |
| `~publish_rate` | float | `100.0` | Hz |
| `~transform_tolerance` | float | `0.1` | s, forward-dating |
| `~persistence_seconds` | float | `5.0` | override window |
| `~persistence_radius_m` | float | `0.3` | agreement radius |
| `~persistence_radius_deg` | float | `10.0` | agreement radius |
| `~publish_identity_before_lock` | bool | `true` | see §6 |

Threshold rationale: real corrections on this platform are centimetre-scale; observed
flips were 4.2 m / 174°. Anything between 0.5 m and 3 m separates them cleanly. Start
at 0.5 m / 30° and widen only if normal operation triggers rejections.

---

## 5. Interfaces

**Subscribes**
- `/tracked_pose` — `geometry_msgs/PoseStamped`
- `/tf` — via `tf2_ros.Buffer` / `TransformListener`

**Publishes**
- `/tf` — `map → odom`, via `tf2_ros.TransformBroadcaster`
- `~status` — `std_msgs/String`, 1 Hz: `LOCKED` / `UNLOCKED` / `REJECTING`
- `~rejection_count` — `std_msgs/Int32`, cumulative

**Services**
- `~reset_lock` — `std_srvs/Empty`. Clears `held`, sets `locked = False`, accepts the
  next candidate unconditionally. Use after a deliberate kidnap or a known-bad lock.

---

## 6. Edge cases — all must be handled

1. **No `/tracked_pose` received yet.** If `publish_identity_before_lock` is true,
   publish identity so the nav stack has a complete TF tree; log WARN once. Otherwise
   publish nothing.
2. **`odom → base_footprint` lookup fails** (`LookupException`, `ExtrapolationException`).
   Skip this candidate, keep publishing `held`. Do not treat as a rejection.
3. **Time jumps backwards** (bag replay, `use_sim_time`). If
   `pose.header.stamp < last_stamp`, reset all state and unlock.
4. **Cartographer restarts.** Detected as case 3 or a long `/tracked_pose` gap
   (> 5 s). Reset and unlock.
5. **First lock during cold-start relocalization.** `locked` is false, so the first
   candidate is accepted whatever its magnitude. Correct — that's the intended
   relocalization landing.
6. **Robot genuinely stationary.** Candidates will be near-identical; gate passes
   trivially. No special handling.

---

## 7. Skeleton

```python
#!/usr/bin/env python
import math, rospy, tf2_ros
from collections import deque
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Int32
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import (quaternion_matrix, translation_matrix,
                                concatenate_matrices, inverse_matrix,
                                translation_from_matrix, quaternion_from_matrix,
                                euler_from_quaternion)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class JumpGuard(object):
    def __init__(self):
        self.map_frame  = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.max_d      = rospy.get_param('~max_jump_translation', 0.5)
        self.max_th     = math.radians(rospy.get_param('~max_jump_rotation', 30.0))
        self.tol        = rospy.Duration(rospy.get_param('~transform_tolerance', 0.1))
        self.persist_s  = rospy.get_param('~persistence_seconds', 5.0)
        self.persist_d  = rospy.get_param('~persistence_radius_m', 0.3)
        self.persist_th = math.radians(rospy.get_param('~persistence_radius_deg', 10.0))
        self.pub_ident  = rospy.get_param('~publish_identity_before_lock', True)

        self.buffer   = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.bcaster  = tf2_ros.TransformBroadcaster()

        self.held        = None      # 4x4 matrix, currently published map->odom
        self.locked      = False
        self.rejects     = 0
        self.last_stamp  = None
        self.pending     = deque()   # (stamp, matrix) of consecutive rejections

        self.status_pub = rospy.Publisher('~status', String, queue_size=1)
        self.count_pub  = rospy.Publisher('~rejection_count', Int32, queue_size=1)
        rospy.Service('~reset_lock', Empty, self.on_reset)
        rospy.Subscriber(rospy.get_param('~tracked_pose_topic', '/tracked_pose'),
                         PoseStamped, self.on_pose, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param('~publish_rate', 100.0)),
                    self.on_publish)

    # --- TODO(agent): implement ------------------------------------------
    # on_pose(msg):
    #   guard edge cases 3 & 4 (stamp regression / long gap) -> reset
    #   M_map_base  <- msg.pose
    #   M_odom_base <- self.buffer.lookup_transform(self.odom_frame,
    #                       self.base_frame, msg.header.stamp,
    #                       rospy.Duration(0.05))     # edge case 2
    #   cand = concatenate_matrices(M_map_base, inverse_matrix(M_odom_base))
    #   if not self.locked: accept(cand); return
    #   dd, dth = delta(cand, self.held)
    #   if dd <= self.max_d and dth <= self.max_th: accept(cand)
    #   else: reject(cand, msg.header.stamp)          # then persistence check
    #
    # delta(a, b) -> (2D translation norm, |wrapped yaw diff|)
    # accept(m)   -> self.held = m; self.locked = True; self.pending.clear()
    # reject(m,t) -> self.rejects += 1; append to self.pending;
    #                if all entries mutually within persist_d/persist_th
    #                and span > persist_s: WARN + accept(m)
    #                else: WARN once per rejection with dd/dth values
    #
    # on_publish(evt):
    #   m = self.held if self.held is not None else (identity if pub_ident else None)
    #   if m is None: return
    #   build TransformStamped(map_frame -> odom_frame),
    #   stamp = rospy.Time.now() + self.tol, broadcast
    # ---------------------------------------------------------------------

    def on_reset(self, _):
        rospy.logwarn('jump_guard: lock reset by service call')
        self.held, self.locked = None, False
        self.pending.clear()
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('localization_jump_guard')
    JumpGuard()
    rospy.spin()
```

---

## 8. Acceptance tests

Run all four. Each must pass before enabling aggressive global relocalization.

| # | Test | Expected |
|---|---|---|
| 1 | Replay the bag containing the 174.2° (t=178 s) and 177.9° (t=190 s) flips | Both rejected; `~rejection_count` ≥ 2; `map → odom` continuous throughout |
| 2 | Normal 50 m walk, strict localization config | **Zero** rejections. Any rejection means thresholds are too tight — widen and re-run |
| 3 | Cold start away from map origin, reloc config (`global_sampling_ratio = 0.05`) | First lock accepted within grace; subsequent flips rejected; robot converges and stays |
| 4 | Carry the robot 5 m with the stack running, then call `~reset_lock` | Rejects while carried (correct — it can't know); re-locks cleanly after the service call |

Instrument test 2 with the existing v4 jump-detection script for a numeric jump count
rather than a visual judgement. Baseline to beat: **76** `map → odom` jumps over
234 s.

---

## 9. Launch

```xml
<node pkg="navoa_localization" type="localization_jump_guard.py"
      name="localization_jump_guard" output="screen">
  <param name="base_frame"            value="base_footprint"/>
  <param name="max_jump_translation"  value="0.5"/>
  <param name="max_jump_rotation"     value="30.0"/>
  <param name="publish_rate"          value="100.0"/>
</node>
```

Start it **before** `cartographer_node` so the TF tree is never incomplete.

---

## 10. What this does not fix

The guard suppresses transient false relocalizations. It cannot distinguish a
*persistently* wrong pose from a correct one, because on a 180°-symmetric map that
distinction does not exist in the sensor data. Breaking the symmetry physically — one
asymmetric object in the offending corridor, then re-map — removes the wrong answer
rather than filtering it. Treat this node as defence in depth, not a substitute.