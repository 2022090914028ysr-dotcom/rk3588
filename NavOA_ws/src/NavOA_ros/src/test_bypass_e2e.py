#!/usr/bin/env python3
"""End-to-end bypass recovery test against the real NeuPAN planner.

Reproduces the field deadlock: a left-hand corridor whose inner bend
holds a low obstacle only the depth camera sees.  NeuPAN alone grinds
to a hard stop (local minimum); with BypassRecovery driving it the
robot backs out, detours right, and reaches the original goal.

Run with the control conda environment (torch + cvxpy):
  ~/miniconda3/envs/control/bin/python3 test_bypass_e2e.py
"""
import sys
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parent
PKG_PARENT = SRC.parent                     # NavOA_ros
sys.path.insert(0, str(PKG_PARENT / "NavOA"))   # NavOA package
sys.path.insert(0, str(SRC))                    # bypass_recovery

from NavOA import NavOA                      # noqa: E402
from bypass_recovery import BypassRecovery    # noqa: E402

CONFIG = PKG_PARENT / 'example/gazebo_limo/config/NavOA_planner_limo.yaml'
CHECKPOINT = (PKG_PARENT /
              'example/gazebo_limo/pretrain_limo/model_5000.pth')

DT = 0.4        # planner step_time
STEPS = 150     # 60 simulated seconds


def make_scene():
    """Left wall + low obstacle glued to the nose; right side open.

    The obstacle's nearest point sits 0.18 m ahead of the robot center;
    with a 0.161 m half-length that is the field-observed hard-stop
    geometry (min_distance ~ 0.019 m).
    """
    wall = np.vstack((np.linspace(-0.3, 0.9, 28),
                      np.full(28, 0.30)))
    ang = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    obstacle = np.vstack((0.24 + 0.06 * np.cos(ang),
                          0.00 + 0.06 * np.sin(ang)))
    return np.hstack((wall, obstacle))


def quat_yaw_of(_):
    return 0.0


def run(bypass_enabled, verbose=True):
    planner = NavOA.init_from_yaml(str(CONFIG), pan={
        'dune_checkpoint': str(CHECKPOINT),
        'use_rknn': False,
        'dune_rknn': None,
        'rknn_target': '',
        'rknn_core': 'auto',
    })

    state = np.array([[0.0], [0.0], [0.0]])
    obstacles = make_scene()
    external_goal = np.array([[1.8], [0.5], [0.0]])

    planner.update_initial_path_from_goal(state, external_goal)
    planner.reset()

    br = (BypassRecovery(stuck_time=0.8, trigger_distance=0.18)
          if bypass_enabled else None)
    goal = external_goal
    deferred = None
    last_min_d = float('inf')
    vx_cmd = 0.0
    t = 0.0
    trace = [state[:2, 0].copy()]
    events = []
    saw_reverse = False
    saw_bypass = False
    penetration = 0.0

    for k in range(STEPS):
        decision = None
        if br is not None:
            decision = br.update(state, obstacles, last_min_d, vx_cmd, t)
            if decision.state == 'reverse':
                saw_reverse = True
                if verbose and not events or events[-1] != 'reverse':
                    events.append('reverse')
                vx_cmd, wz = decision.twist[0], 0.0
            else:
                if decision.new_goal is not None:
                    saw_bypass = True
                    events.append('bypass-goal:%s' % (
                        'right' if decision.side < 0 else 'left'))
                    goal = decision.new_goal
                    planner.update_initial_path_from_goal(state, goal)
                    planner.reset()
                    # NavOA_core defers external goals during a detour.
                    deferred = external_goal.copy()
                if decision.state == 'aborted':
                    events.append('abort:%s' % decision.reason)
                    break

        if decision is None or decision.state != 'reverse':
            action, info = planner(state, obstacles)
            try:
                last_min_d = float(planner.min_distance)
            except Exception:
                last_min_d = float('inf')
            penetration = min(penetration, last_min_d)
            if info['arrive']:
                if br is not None and br.active:
                    br.notify_arrived()
                    events.append('bypass-reached')
                    if deferred is not None:
                        goal = deferred
                        deferred = None
                        planner.update_initial_path_from_goal(state, goal)
                        planner.reset()
                    vx_cmd, wz = 0.0, 0.0
                else:
                    events.append('goal-reached')
                    vx_cmd, wz = 0.0, 0.0
                    break
            elif info['stop']:
                vx_cmd, wz = 0.0, 0.0
            elif action is not None:
                vx_cmd, wz = float(action[0, 0]), float(action[1, 0])
            else:
                vx_cmd, wz = 0.0, 0.0

        # Differential-drive integration.
        x, y, th = state[:, 0]
        state = np.array([
            [x + vx_cmd * np.cos(th) * DT],
            [y + vx_cmd * np.sin(th) * DT],
            [th + wz * DT]])
        trace.append(state[:2, 0].copy())
        t += DT

        if not br and k == STEPS - 1:
            events.append('budget-exhausted')

    trace = np.array(trace)
    dist_goal = float(np.linalg.norm(state[:2, 0] - external_goal[:2, 0]))
    return {
        'final_state': state,
        'trace': trace,
        'dist_goal': dist_goal,
        'traveled': float(np.sum(np.linalg.norm(np.diff(trace, axis=0),
                                                axis=1))),
        'min_y': float(trace[:, 1].min()),
        'events': events,
        'saw_reverse': saw_reverse,
        'saw_bypass': saw_bypass,
        'penetration': float(penetration),
    }


def main():
    print('== control: bypass DISABLED (expect deadlock) ==')
    off = run(bypass_enabled=False)
    print('  events:', off['events'])
    print('  traveled=%.3fm dist_goal=%.3fm min_y=%.3f'
          % (off['traveled'], off['dist_goal'], off['min_y']))
    deadlocked = off['traveled'] < 0.10 and off['dist_goal'] > 1.0
    print('  deadlocked:', deadlocked)

    print('\n== experiment: bypass ENABLED (expect right detour) ==')
    on = run(bypass_enabled=True)
    print('  events:', on['events'])
    print('  traveled=%.3fm dist_goal=%.3fm min_y=%.3f penetration=%.3f'
          % (on['traveled'], on['dist_goal'], on['min_y'],
             on['penetration']))
    print('  final state:', np.round(on['final_state'][:, 0], 3).tolist())

    checks = [
        ('control run deadlocks without bypass', deadlocked),
        ('experiment saw the reverse phase', on['saw_reverse']),
        ('experiment committed to the right corridor',
         on['saw_bypass'] and on['min_y'] < -0.20),
        ('experiment reached the original goal',
         on['dist_goal'] < 0.40),
        ('no obstacle penetration', on['penetration'] > -0.02),
    ]
    failed = 0
    for name, ok in checks:
        print('  [%s] %s' % ('PASS' if ok else 'FAIL', name))
        failed += 0 if ok else 1
    print('\n%d/%d checks passed' % (len(checks) - failed, len(checks)))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
