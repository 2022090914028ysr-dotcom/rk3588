#!/usr/bin/env python3
"""Unit tests for bypass_recovery and the directional point quota.

Run:  python3 test_bypass_recovery.py
(No ROS graph needed; pure numpy logic.)
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bypass_recovery import (  # noqa: E402
    BypassRecovery, SIDE_LEFT, SIDE_RIGHT, forward_obstacle_distance)

PASS, FAIL = 0, 0


def check(name, cond, detail=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print('  [PASS] %s' % name)
    else:
        FAIL += 1
        print('  [FAIL] %s  %s' % (name, detail))


def state(x=0.0, y=0.0, th=0.0):
    return np.array([[x], [y], [th]])


def wall(y, x0, x1, n=25):
    return np.vstack((np.linspace(x0, x1, n), np.full(n, y)))


def obstacle_cluster(cx, cy, r=0.05, n=12):
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.vstack((cx + r * np.cos(ang), cy + r * np.sin(ang)))


# ----------------------------------------------------------------- tests
def test_corridor_evaluation():
    print('== corridor evaluation ==')
    br = BypassRecovery()

    # Left wall at y=0.274 (the reported deadlock geometry), right open.
    obs = np.hstack((wall(0.274, 0.2, 0.9), obstacle_cluster(0.30, 0.0)))
    side, clear = br.evaluate_corridors(state(), obs)
    check('left-blocked chooses right', side == SIDE_RIGHT, str(side))
    check('right clearance large', clear > 0.5, '%.2f' % clear)

    # Mirror: right wall, left open.
    obs = np.hstack((wall(-0.274, 0.2, 0.9), obstacle_cluster(0.30, 0.0)))
    side, clear = br.evaluate_corridors(state(), obs)
    check('right-blocked chooses left', side == SIDE_LEFT, str(side))

    # Both blocked inside the corridor band.
    obs = np.hstack((wall(0.16, 0.25, 0.9), wall(-0.16, 0.25, 0.9)))
    side, clear = br.evaluate_corridors(state(), obs)
    check('both blocked -> None', side is None, str(side))

    # Obstacle glued to the nose (hard-stop geometry) must not affect the
    # side scoring: x < corridor_x_min.
    obs = obstacle_cluster(0.05, 0.0)
    side, clear = br.evaluate_corridors(state(), obs)
    check('nose obstacle ignored for sides', side is not None, str(side))

    # Obstacle sitting inside the right corridor narrows it.
    obs = np.hstack((wall(0.6, 0.2, 0.9), obstacle_cluster(0.5, -0.25)))
    side, clear = br.evaluate_corridors(state(), obs)
    check('right corridor narrowed by obstacle -> left',
          side == SIDE_LEFT, str(side))

    # No obstacles at all.
    side, clear = br.evaluate_corridors(state(), None)
    check('no obstacles -> None', side is None, str(side))


def test_bypass_goal_transform():
    print('== lateral subgoal transform ==')
    br = BypassRecovery(forward_offset=0.45, lateral_offset=0.55)

    goal = br._make_bypass_goal(state(0, 0, 0), SIDE_RIGHT)
    check('right goal (th=0)',
          np.allclose(goal[:, 0], [0.45, -0.55, 0.0]), str(goal[:, 0]))

    # Robot at (1,1) facing +y (90 deg): forward is +y, right is +x.
    goal = br._make_bypass_goal(state(1.0, 1.0, np.pi / 2), SIDE_RIGHT)
    check('right goal (th=90)',
          np.allclose(goal[:, 0], [1.55, 1.45, np.pi / 2]),
          str(np.round(goal[:, 0], 3)))

    goal = br._make_bypass_goal(state(0, 0, np.pi / 2), SIDE_LEFT)
    check('left goal (th=90)',
          np.allclose(goal[:, 0], [-0.55, 0.45, np.pi / 2]),
          str(np.round(goal[:, 0], 3)))


def test_rear_clear():
    print('== rear safety ==')
    br = BypassRecovery(reverse_distance=0.15, rear_clear_distance=0.40)

    check('empty world rear clear', br._rear_clear(state(), None))
    check('front obstacle only -> rear clear',
          br._rear_clear(state(), obstacle_cluster(0.3, 0.0)))

    rear_obs = obstacle_cluster(-0.2, 0.0)
    check('rear obstacle at 0.2m -> blocked',
          not br._rear_clear(state(), rear_obs))

    far_rear = obstacle_cluster(-0.6, 0.0)
    check('rear obstacle beyond margin -> clear',
          br._rear_clear(state(), far_rear))

    flank = obstacle_cluster(0.0, 0.45)
    check('flank-only obstacle -> rear clear',
          br._rear_clear(state(), flank))


def _drive_stuck(br, obs, min_d, t0, seconds, dt=0.05):
    """Call update() while hard-stopped until the detour decision fires."""
    t, decision = t0, None
    for _ in range(int(seconds / dt)):
        decision = br.update(state(), obs, min_d, 0.0, t)
        if decision.state != 'idle':
            return decision
        t += dt
    return decision


def test_state_machine_hard_stop():
    print('== state machine: hard stop -> reverse -> bypass ==')
    br = BypassRecovery(stuck_time=0.4)
    obs = np.hstack((wall(0.274, 0.2, 0.9),      # left wall
                     obstacle_cluster(0.20, 0.0)))  # nose obstacle (0.018-ish)

    d = _drive_stuck(br, obs, min_d=0.018, t0=1.0, seconds=2.0)
    check('hard-stop triggers reverse', d.state == 'reverse' and d.twist,
          d.reason)
    check('global mark requested on trigger',
          d.trigger_global_mark is True)

    # Reverse 0.15 m: robot now at (-0.15, 0).
    robot = state(-0.15, 0.0)
    d = br.update(robot, obs, 0.16, 0.0, 1.8)
    check('reverse completed -> bypass goal',
          d.state == 'bypass' and d.new_goal is not None, d.reason)
    if d.new_goal is not None:
        check('bypass goal on the right', d.side == SIDE_RIGHT, str(d.side))
        check('bypass goal position',
              np.allclose(d.new_goal[:2, 0], [0.45 - 0.15, -0.55]),
              str(np.round(d.new_goal[:2, 0], 3)))

    # Follow the detour; a fresh min_distance must not retrigger.
    d = br.update(robot, obs, 0.30, 0.2, 2.0)
    check('detour cruising (no new goal)', d.state == 'bypass'
          and d.new_goal is None, d.reason)

    # Arrival releases the lock.
    br.notify_arrived()
    check('arrival releases lock', not br.active)
    # Rest cooldown blocks an immediate re-trigger.
    d = _drive_stuck(br, obs, 0.018, 2.5, seconds=1.0)
    check('cooldown blocks immediate retrigger', d.state == 'idle', d.reason)


def test_state_machine_soft_stall():
    print('== state machine: soft stall (outside hard stop) ==')
    br = BypassRecovery(stuck_time=0.4)
    obs = np.hstack((wall(0.274, 0.2, 0.9), obstacle_cluster(0.42, 0.0)))

    d = _drive_stuck(br, obs, min_d=0.12, t0=1.0, seconds=2.0)
    check('soft stall goes straight to bypass',
          d.state == 'bypass' and d.new_goal is not None,
          '%s/%s' % (d.state, d.reason))
    check('no reverse twist issued', d.twist is None)


def test_state_machine_blocked_rear():
    print('== state machine: hard stop with blocked rear ==')
    br = BypassRecovery(stuck_time=0.4)
    obs = np.hstack((wall(0.16, 0.2, 0.9),    # left wall (clearance < 0.06)
                     wall(-0.16, 0.2, 0.9),   # right wall (clearance < 0.06)
                     obstacle_cluster(-0.20, 0.0),   # rear obstacle
                     obstacle_cluster(0.20, 0.0)))   # nose obstacle
    d = _drive_stuck(br, obs, min_d=0.018, t0=1.0, seconds=2.0)
    # Both corridors blocked -> abort + global mark (never blind reverse).
    check('both blocked + rear blocked -> abort',
          d.state == 'aborted' and d.trigger_global_mark, d.reason)

    # Now open the right wall: abort becomes a direct bypass (rear is
    # still blocked, so backing up is not allowed, but the corridor is
    # wide enough to side-step from standstill).
    br2 = BypassRecovery(stuck_time=0.4)
    obs2 = np.hstack((wall(0.274, 0.2, 0.9),
                      obstacle_cluster(-0.20, 0.0),
                      obstacle_cluster(0.20, 0.0)))
    d = _drive_stuck(br2, obs2, min_d=0.018, t0=1.0, seconds=2.0)
    check('blocked rear + open corridor -> bypass without reverse',
          d.state == 'bypass' and d.twist is None, '%s/%s' % (d.state, d.reason))


def test_directional_quota():
    print('== directional quota (obstacle_fusion) ==')
    # Import from the fusion module without ROS (numpy-only function).
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'obstacle_fusion_quota',
        Path(__file__).resolve().parent / 'obstacle_fusion.py')
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:  # rospy import guard
        print('  [SKIP] obstacle_fusion needs rospy: %s' % exc)
        return
    limit = mod._limit_directional

    # 300 rear points at 0.3 m, 40 forward points at 0.5 m, 30 left, 30
    # right (side points placed abeam, ~90 deg, so they land in the side
    # buckets).  Budget 100: nearest-only would pick nearly all rear.
    ang = np.linspace(np.pi - 0.3, np.pi + 0.3, 300)  # around rear
    rear = np.vstack((0.3 * np.cos(ang), 0.3 * np.sin(ang)))
    fwd = np.vstack((np.linspace(0.4, 0.6, 40), np.zeros(40)))
    left = np.vstack((np.linspace(-0.1, 0.1, 30), np.full(30, 0.5)))
    right = np.vstack((np.linspace(-0.1, 0.1, 30), np.full(30, -0.5)))
    pts = np.hstack((rear, fwd, left, right))  # (2, 400)

    out = limit(pts, np.zeros(2), 0.0, 100)
    check('output capped at budget', out.shape[1] == 100, str(out.shape))
    fx, fy = out[0], out[1]
    n_front = int(np.sum((np.abs(np.arctan2(fy, fx)) < np.deg2rad(60))))
    n_rear = int(np.sum((np.abs(np.arctan2(fy, fx)) > np.deg2rad(135))))
    # All 40 forward points survive the quota; the rear is capped by its
    # bucket (10) plus at most the leftover quota handed to the nearest
    # remaining points (rear at 0.3 m wins that overflow).
    check('all forward points survive the quota', n_front == 40, str(n_front))
    check('rear compressed from 300 to <=20', n_rear <= 20, str(n_rear))

    # Nearest-only comparison: rear would dominate.
    out_near = mod._limit_nearest(pts, np.zeros(2), 100)
    rfx, rfy = out_near[0], out_near[1]
    n_front_near = int(np.sum(np.abs(np.arctan2(rfy, rfx)) < np.deg2rad(60)))
    check('nearest-only starves the front (regression guard)',
          n_front_near < 45, str(n_front_near))

    # Small inputs pass through untouched.
    small = np.hstack((fwd, left))
    out_small = limit(small, np.zeros(2), 0.0, 100)
    check('small cloud passthrough', out_small.shape[1] == small.shape[1])


def test_state_machine_reverse_creep():
    print('== state machine: reverse creep counts as stalled ==')
    # Field signature: NeuPAN answers a goal line through the obstacle
    # with slow reverse + wiggle (vx ~ -0.06, wz ~ 0.1) for minutes.
    # The |vx| threshold would call that "moving" and never trigger.
    br = BypassRecovery(stuck_time=0.4)
    obs = np.hstack((wall(0.30, 0.2, 0.9), obstacle_cluster(0.42, 0.0)))

    t, decision = 1.0, None
    for _ in range(int(2.0 / 0.05)):
        decision = br.update(state(), obs, 0.121, -0.066, t)  # reverse creep
        if decision.state != 'idle':
            break
        t += 0.05
    check('reverse creep near obstacle triggers bypass',
          decision.state == 'bypass' and decision.new_goal is not None,
          decision.reason)

    # Clearly forward motion never triggers, even near the obstacle.
    br2 = BypassRecovery(stuck_time=0.4)
    t, decision = 1.0, None
    for _ in range(int(2.0 / 0.05)):
        decision = br2.update(state(), obs, 0.121, 0.25, t)  # cruising
        if decision.state != 'idle':
            break
        t += 0.05
    check('forward motion does not trigger', decision.state == 'idle',
          decision.reason)


def test_forward_obstacle_distance():
    print('== forward-cone distance (early mark input) ==')
    front_pt = np.array([[0.40], [0.05]])
    flank_pt = np.array([[0.05], [0.30]])       # beside the robot
    rear_pt = np.array([[-0.30], [0.0]])
    wide_front = np.array([[1.20], [0.50]])

    d = forward_obstacle_distance(state(), front_pt)
    check('front point detected', d is not None and abs(d - 0.403) < 0.01,
          str(d))

    d = forward_obstacle_distance(
        state(), np.hstack((front_pt, flank_pt, rear_pt)))
    check('mixed cloud picks the front cone only',
          d is not None and abs(d - 0.403) < 0.01, str(d))

    d = forward_obstacle_distance(
        state(), np.hstack((flank_pt, rear_pt)))
    check('flank+rear only -> cone clear', d is None, str(d))

    d = forward_obstacle_distance(state(), wide_front)
    check('beyond early-mark range still reported',
          d is not None and abs(d - 1.30) < 0.01, str(d))

    # Rotated robot: cone follows the heading.
    robot = state(0.0, 0.0, np.pi / 2)   # facing +y
    d = forward_obstacle_distance(robot, np.array([[0.05], [0.45]]))
    check('cone rotates with heading',
          d is not None and abs(d - 0.453) < 0.01, str(d))

    check('empty world -> None',
          forward_obstacle_distance(state(), None) is None)


if __name__ == '__main__':
    test_corridor_evaluation()
    test_bypass_goal_transform()
    test_rear_clear()
    test_state_machine_hard_stop()
    test_state_machine_soft_stall()
    test_state_machine_blocked_rear()
    test_state_machine_reverse_creep()
    test_forward_obstacle_distance()
    test_directional_quota()
    print('\n%d passed, %d failed' % (PASS, FAIL))
    sys.exit(1 if FAIL else 0)
