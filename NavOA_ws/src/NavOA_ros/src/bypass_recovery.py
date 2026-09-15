"""Side-corridor bypass recovery for NeuPAN deadlocks.

NeuPAN is a local continuous optimizer: next to a wall, decelerating to a
stop is a local minimum and the network never discovers that the other
side of the corridor is open.  This module adds that missing topological
decision as a small, deterministic state machine:

  1. Detect the deadlock (near obstacle + no forward motion over time).
  2. Evaluate the left/right corridors in the robot frame.
  3. If the robot is already inside the hard-stop zone, back up first
     (only when the rear is clear) to recover maneuvering space.
  4. Commit to a lateral subgoal so NeuPAN tracks around the obstacle.
  5. Retry a bounded number of times, then hand the problem upstream by
     requesting an ephemeral mark of the obstacle in the global costmap.

The class is pure logic (numpy only) so it can be unit tested without a
running ROS graph; NavOA_core wraps it and publishes the ROS side effects.
"""
import numpy as np

STATE_IDLE = 'idle'
STATE_REVERSE = 'reverse'
STATE_BYPASS = 'bypass'

SIDE_LEFT = +1   # robot frame: +y
SIDE_RIGHT = -1  # robot frame: -y


def forward_obstacle_distance(robot_state, obstacles, half_angle=0.8,
                              forward_min=0.05):
    """Distance to the nearest obstacle inside the robot's forward cone.

    Unlike the planner's all-around min_distance, this ignores flank and
    rear contacts — a robot driving along a wall must not read as
    "obstacle ahead".  Returns None when the forward cone is clear.
    """
    if obstacles is None or np.size(obstacles) == 0 or obstacles.shape[1] == 0:
        return None
    x = float(robot_state[0, 0])
    y = float(robot_state[1, 0])
    th = float(robot_state[2, 0])
    c, s = np.cos(th), np.sin(th)
    dx = np.asarray(obstacles[0], dtype=np.float64) - x
    dy = np.asarray(obstacles[1], dtype=np.float64) - y
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    mask = (bx > forward_min) & (np.abs(np.arctan2(by, bx)) < half_angle)
    if not np.any(mask):
        return None
    return float(np.min(np.hypot(bx[mask], by[mask])))


class BypassDecision:
    """One update() result consumed by the caller (NavOA_core or a test).

    state: 'idle' | 'reverse' | 'bypass' | 'aborted'
    new_goal: (3,1) map-frame subgoal when a bypass detour is (re)started
    twist: (vx, wz) direct command while reversing; None otherwise
    trigger_global_mark: ask the global costmap to mark obstacles now
    side: +1 left / -1 right of the corridor chosen for this detour
    """

    __slots__ = ('state', 'new_goal', 'twist', 'trigger_global_mark',
                 'side', 'reason')

    def __init__(self, state=STATE_IDLE, new_goal=None, twist=None,
                 trigger_global_mark=False, side=0, reason=''):
        self.state = state
        self.new_goal = new_goal
        self.twist = twist
        self.trigger_global_mark = trigger_global_mark
        self.side = side
        self.reason = reason

    def __repr__(self):
        goal = None if self.new_goal is None else np.round(
            self.new_goal[:, 0], 3).tolist()
        return ('BypassDecision(state=%s, goal=%s, twist=%s, mark=%s, '
                'side=%d, reason=%s)' % (
                    self.state, goal, self.twist, self.trigger_global_mark,
                    self.side, self.reason))


class BypassRecovery:
    """Deadlock detector + lateral detour state machine.

    All distances are meters, all times seconds.  ``robot_state`` is a
    (3,1) [x, y, theta] in the map frame; ``obstacles`` is (2, N) in the
    same frame (the fused laser+depth points); ``min_distance`` is the
    planner's current body-to-obstacle clearance.
    """

    def __init__(self,
                 trigger_distance=0.18,
                 stuck_time=0.8,
                 forward_offset=0.45,
                 lateral_offset=0.55,
                 corridor_x=(0.20, 1.00),
                 corridor_y=(0.15, 0.75),
                 robot_half_width=0.11,
                 min_clearance=0.06,
                 hard_stop_threshold=0.05,
                 hard_stop_margin=0.03,
                 reverse_distance=0.15,
                 reverse_speed=0.12,
                 reverse_timeout=1.6,
                 rear_clear_distance=0.40,
                 bypass_timeout=10.0,
                 max_retries=3,
                 rest_time=4.0,
                 stalled_vx=0.03):
        if trigger_distance <= 0 or stuck_time <= 0:
            raise ValueError('trigger_distance and stuck_time must be positive')
        self.trigger_distance = float(trigger_distance)
        self.stuck_time = float(stuck_time)
        self.forward_offset = float(forward_offset)
        self.lateral_offset = float(lateral_offset)
        self.corridor_x = (float(corridor_x[0]), float(corridor_x[1]))
        self.corridor_y = (float(corridor_y[0]), float(corridor_y[1]))
        self.robot_half_width = float(robot_half_width)
        self.min_clearance = float(min_clearance)
        self.hard_stop_threshold = float(hard_stop_threshold)
        self.hard_stop_margin = float(hard_stop_margin)
        self.reverse_distance = float(reverse_distance)
        self.reverse_speed = float(reverse_speed)
        self.reverse_timeout = float(reverse_timeout)
        self.rear_clear_distance = float(rear_clear_distance)
        self.bypass_timeout = float(bypass_timeout)
        self.max_retries = int(max_retries)
        self.rest_time = float(rest_time)
        self.stalled_vx = float(stalled_vx)

        self.state = STATE_IDLE
        self._stuck_accum = 0.0
        self._bypass_stuck_accum = 0.0
        self._last_now = None
        self._last_finish = None
        self._reverse_origin = None
        self._reverse_start = None
        self._bypass_goal = None
        self._bypass_start = None
        self._retries = 0
        self.last_decision_reason = ''

    # ------------------------------------------------------------------ api
    @property
    def active(self):
        return self.state in (STATE_REVERSE, STATE_BYPASS)

    @property
    def bypass_goal(self):
        return self._bypass_goal

    def notify_arrived(self):
        """The lateral subgoal was reached; release the detour lock."""
        self._finish('bypass-reached')

    def notify_goal_override(self):
        """An external authoritative goal cancelled the detour."""
        self._finish('goal-override')

    def reset(self):
        """Full reset (new navigation task)."""
        self.state = STATE_IDLE
        self._stuck_accum = 0.0
        self._bypass_stuck_accum = 0.0
        self._reverse_origin = None
        self._reverse_start = None
        self._bypass_goal = None
        self._bypass_start = None
        self._retries = 0

    def update(self, robot_state, obstacles, min_distance, cmd_vx, now):
        """Advance the state machine; call once per control cycle.

        Returns a BypassDecision.  While ``twist`` is set the caller must
        forward that command directly (the planner is still hard-stopped);
        while ``new_goal`` is set the caller must install it as the
        planner goal exactly once.
        """
        if self._last_now is None:
            dt = 0.0
        else:
            dt = max(0.0, min(0.5, float(now) - self._last_now))
        self._last_now = float(now)

        if self.state == STATE_IDLE:
            decision = self._update_idle(
                robot_state, obstacles, min_distance, cmd_vx, dt, now)
        elif self.state == STATE_REVERSE:
            decision = self._update_reverse(
                robot_state, obstacles, min_distance, dt, now)
        else:
            decision = self._update_bypass(
                robot_state, obstacles, min_distance, cmd_vx, dt, now)
        self.last_decision_reason = decision.reason
        return decision

    # ----------------------------------------------------------- internals
    def _finish(self, reason):
        self._last_finish = self._last_now
        self.state = STATE_IDLE
        self._bypass_goal = None
        self._stuck_accum = 0.0
        self._bypass_stuck_accum = 0.0
        self._reverse_origin = None
        self._reverse_start = None

    def _update_idle(self, robot_state, obstacles, min_distance, cmd_vx,
                     dt, now):
        near = (min_distance is not None
                and np.isfinite(min_distance)
                and min_distance < self.trigger_distance)
        # Reverse creep is a stall signal too: NeuPAN answers a
        # goal line through a near obstacle with "back up slowly and
        # wiggle", which never clears the blockage while looking
        # alive to a pure |vx| threshold.  Only clearly FORWARD motion
        # counts as progress.
        stalled = float(cmd_vx) < self.stalled_vx

        if not (near and stalled):
            self._stuck_accum = 0.0
            return BypassDecision(reason='monitoring')

        self._stuck_accum += dt
        if self._stuck_accum < self.stuck_time:
            return BypassDecision(reason='stuck-building')

        # Cooldown after a finished/aborted detour: give the (possibly
        # replanned) global path a chance to work before re-triggering.
        if (self._last_finish is not None
                and now - self._last_finish < self.rest_time):
            return BypassDecision(reason='rest-cooldown')

        return self._start_detour(robot_state, obstacles, min_distance,
                                 now, 'stuck-detected')

    def _start_detour(self, robot_state, obstacles, min_distance, now,
                      reason):
        """Evaluate corridors and begin reverse/bypass/abort."""
        self._stuck_accum = 0.0
        self._bypass_stuck_accum = 0.0

        if self._retries >= self.max_retries:
            self._retries = 0
            self._finish('retry-exhausted')
            return BypassDecision(
                state='aborted', trigger_global_mark=True,
                reason='retry-exhausted:' + reason)

        side, clearance = self.evaluate_corridors(robot_state, obstacles)
        in_hard_stop = (min_distance is not None
                        and np.isfinite(min_distance)
                        and min_distance < self.hard_stop_threshold
                        + self.hard_stop_margin)

        if side is None:
            # Both corridors blocked: only the global planner can help.
            self._retries = 0
            self._finish('both-corridors-blocked')
            return BypassDecision(
                state='aborted', trigger_global_mark=True,
                reason='both-corridors-blocked:' + reason)

        if in_hard_stop and self._rear_clear(robot_state, obstacles):
            # Back out of the hard-stop zone first; the detour goal is
            # chosen after the retreat when geometry is meaningful again.
            self.state = STATE_REVERSE
            self._reverse_origin = robot_state[:2, 0].copy()
            self._reverse_start = now
            self._retries += 1
            return BypassDecision(
                state=STATE_REVERSE, twist=(-self.reverse_speed, 0.0),
                trigger_global_mark=True,
                reason='hard-stop-backing-up:%s' % reason)

        if in_hard_stop and not self._rear_clear(robot_state, obstacles):
            # Hard-stopped with a blocked rear: never back up blindly.
            if clearance >= self.min_clearance:
                goal = self._make_bypass_goal(robot_state, side)
                return self._enter_bypass(goal, side, now,
                                          'bypass-from-hard-stop')
            self._retries = 0
            self._finish('hard-stop-no-retreat')
            return BypassDecision(
                state='aborted', trigger_global_mark=True,
                reason='hard-stop-no-retreat:' + reason)

        goal = self._make_bypass_goal(robot_state, side)
        return self._enter_bypass(goal, side, now, reason)

    def _update_reverse(self, robot_state, obstacles, min_distance, dt,
                        now):
        # Continuous rear safety during the retreat.
        if not self._rear_clear(robot_state, obstacles):
            side, _clearance = self.evaluate_corridors(
                robot_state, obstacles)
            if side is not None:
                goal = self._make_bypass_goal(robot_state, side)
                return self._enter_bypass(goal, side, now,
                                          'reverse-interrupted')
            self._finish('rear-blocked-while-reversing')
            return BypassDecision(
                state='aborted', trigger_global_mark=True,
                reason='rear-blocked-while-reversing')

        traveled = float(np.linalg.norm(
            robot_state[:2, 0] - self._reverse_origin))
        elapsed = now - self._reverse_start
        if traveled < self.reverse_distance and elapsed < self.reverse_timeout:
            return BypassDecision(
                state=STATE_REVERSE, twist=(-self.reverse_speed, 0.0),
                reason='backing-up(%.2fm)' % traveled)

        side, clearance = self.evaluate_corridors(robot_state, obstacles)
        if side is None:
            self._finish('after-reverse-blocked')
            return BypassDecision(
                state='aborted', trigger_global_mark=True,
                reason='after-reverse-blocked')
        goal = self._make_bypass_goal(robot_state, side)
        return self._enter_bypass(goal, side, now, 'after-reverse')

    def _update_bypass(self, robot_state, obstacles, min_distance, cmd_vx,
                       dt, now):
        elapsed = now - self._bypass_start
        if elapsed > self.bypass_timeout:
            self._finish('bypass-timeout')
            return BypassDecision(
                state='aborted', trigger_global_mark=True,
                reason='bypass-timeout')

        # A new obstacle may deadlock the detour itself.  Reverse creep
        # counts as stalled here as well (see _update_idle).
        near = (min_distance is not None and np.isfinite(min_distance)
                and min_distance < self.trigger_distance)
        stalled = float(cmd_vx) < self.stalled_vx
        if near and stalled:
            self._bypass_stuck_accum += dt
        else:
            self._bypass_stuck_accum = 0.0

        if (self._bypass_stuck_accum >= self.stuck_time
                and self._retries < self.max_retries):
            side, _clearance = self.evaluate_corridors(
                robot_state, obstacles)
            if side is None:
                self._finish('bypass-blocked')
                return BypassDecision(
                    state='aborted', trigger_global_mark=True,
                    reason='bypass-blocked')
            goal = self._make_bypass_goal(robot_state, side)
            self._bypass_stuck_accum = 0.0
            return self._enter_bypass(goal, side, now, 'bypass-reroute',
                                      count_retry=True)

        return BypassDecision(state=STATE_BYPASS,
                              reason='detour-in-progress')

    def _enter_bypass(self, goal, side, now, reason, count_retry=False):
        if count_retry or self.state != STATE_BYPASS:
            self._retries += 1
        self.state = STATE_BYPASS
        self._bypass_goal = goal
        self._bypass_start = now
        self._bypass_stuck_accum = 0.0
        self._reverse_origin = None
        self._reverse_start = None
        return BypassDecision(
            state=STATE_BYPASS, new_goal=goal, side=side,
            trigger_global_mark=True, reason='bypass-start:' + reason)

    # ------------------------------------------------------------ geometry
    def _to_base(self, obstacles, robot_state):
        """Map-frame (2,N) points -> robot-frame (2,N)."""
        xy = np.asarray(obstacles, dtype=np.float64)
        x, y, th = (float(robot_state[0, 0]), float(robot_state[1, 0]),
                    float(robot_state[2, 0]))
        c, s = np.cos(th), np.sin(th)
        dx = xy[0] - x
        dy = xy[1] - y
        return np.vstack((c * dx + s * dy, -s * dx + c * dy))

    def evaluate_corridors(self, robot_state, obstacles):
        """Score the left/right detour corridors in the robot frame.

        Returns (side, clearance): the wider side (+1 left / -1 right)
        and its lateral clearance, or (None, best_clearance) when both
        are below ``min_clearance``.
        """
        if obstacles is None or np.size(obstacles) == 0 or \
                obstacles.shape[1] == 0:
            return None, 0.0

        base = self._to_base(obstacles, robot_state)
        bx, by = base[0], base[1]
        x_min, x_max = self.corridor_x
        y_min, y_max = self.corridor_y

        best_side, best_clear = None, -np.inf
        for side in (SIDE_LEFT, SIDE_RIGHT):
            sy = side * by  # normalize: this corridor lies at sy > 0
            mask = ((bx > x_min) & (bx < x_max)
                    & (sy > y_min) & (sy < y_max))
            if not np.any(mask):
                clear = y_max - self.robot_half_width
            else:
                clear = float(np.min(sy[mask])) - self.robot_half_width
            if clear > best_clear:
                best_side, best_clear = side, clear

        if best_clear < self.min_clearance:
            return None, best_clear
        return best_side, best_clear

    def _make_bypass_goal(self, robot_state, side):
        """Lateral subgoal in the map frame: forward+sideways offset."""
        x, y, th = (float(robot_state[0, 0]), float(robot_state[1, 0]),
                    float(robot_state[2, 0]))
        fx = self.forward_offset
        fy = side * self.lateral_offset
        gx = x + np.cos(th) * fx - np.sin(th) * fy
        gy = y + np.sin(th) * fx + np.cos(th) * fy
        return np.array([[gx], [gy], [th]])

    def _rear_clear(self, robot_state, obstacles):
        """True when the corridor behind the robot is free far enough
        for a full reverse_distance retreat (plus body and margin).

        Only the strip the body actually sweeps when reversing straight
        back counts; a wall alongside the flank (common in corridors)
        must not veto the retreat.
        """
        if obstacles is None or np.size(obstacles) == 0 or \
                obstacles.shape[1] == 0:
            return True
        need = self.reverse_distance + 0.05
        if self.rear_clear_distance < need:
            # Configured margin is too small to certify a retreat.
            return False
        base = self._to_base(obstacles, robot_state)
        bx, by = base[0], base[1]
        behind = ((bx < 0.05)
                  & (np.abs(by) < self.robot_half_width + 0.05)
                  & (np.hypot(bx, by) < self.rear_clear_distance))
        return not bool(np.any(behind))
