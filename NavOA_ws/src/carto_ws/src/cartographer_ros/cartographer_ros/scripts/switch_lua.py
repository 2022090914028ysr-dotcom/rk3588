#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
switch_lua.py — boot-time orchestrator for the fast→slow Lua handoff.

Two mutually-orthogonal mechanisms are available, selected via ROS params:

  ~boot_handoff_enabled  (default: False)
      Legacy path: cartographer is launched WITHOUT an auto-started
      trajectory, and this script drives the full 5-step recovery
      sequence (finish-noop → start fast → settle → capture → start slow
      with pose). Used historically; kept for fallback.

  ~boot_swap_enabled     (default: True)
      New path: cartographer is launched with the FAST Lua and
      auto-starts a trajectory at T=0. After `boot_swap_delay_s` seconds
      (default 20.0), this script captures the current pose, finishes
      the fast trajectory, and starts the slow Lua with that pose.

If both are enabled, the boot-handoff timer fires first and the swap
timer becomes a no-op (the manager is busy). Logged as a warning.

After the swap (or handoff), the imported CartographerTrajectoryManagerScore
continues to run its runtime score / offset / rotation guards.
"""

import math
import rospy
from cartographer_guard_score_fast_reloc import CartographerTrajectoryManagerScore


def _do_fast_to_slow_swap(manager):
    """Finish the currently-active trajectory and start a new one with
    the slow Lua, using the latest map→base_footprint pose. Reset
    runtime-guard baselines so they don't false-trigger after the swap.
    """
    # Refuse to swap if a recovery is already in progress.
    with manager._lock:
        if manager.state != manager.STATE_IDLE:
            rospy.logwarn("[SWAP] manager busy (state=%d); skipping swap.",
                          manager.state)
            return
        manager.state = manager.STATE_RECOVERING

    try:
        rospy.loginfo("[SWAP] fast → slow handoff starting.")

        # 1. Capture current pose from the fast Lua trajectory.
        pose = manager.get_map_to_base_footprint()
        if pose is None:
            rospy.logerr("[SWAP] could not get map→base_footprint pose; "
                         "aborting swap (fast Lua will keep running).")
            return
        x, y, yaw = pose
        yaw_deg = math.degrees(yaw)
        rospy.loginfo("[SWAP] captured pose: x=%.3f y=%.3f yaw=%.1f deg",
                      x, y, yaw_deg)

        # 2. Finish the fast Lua's trajectory.
        finished = manager.finish_latest_trajectory()
        if finished is None:
            rospy.logerr("[SWAP] could not finish fast trajectory; "
                         "aborting.")
            return
        rospy.sleep(0.5)  # small gap so cartographer fully closes it

        # 3. Start the slow Lua's trajectory with the captured pose.
        new_id = manager.start_trajectory_with_pose(
            x=x, y=y, yaw_deg=yaw_deg,
            config_file=manager.config_file)
        if new_id is None:
            rospy.logerr("[SWAP] could not start slow trajectory!")
            return

        # 4. Reset runtime-guard baselines.
        manager.node_start_time = rospy.Time.now().to_sec()
        manager.last_position = None
        manager.last_yaw = None
        manager.offset_violation_count = 0
        manager.rotation_violation_count = 0
        manager.score_low_count = 0

        rospy.loginfo("[SWAP] slow Lua active (trajectory_id=%d). "
                      "Runtime guards re-armed.", new_id)
    finally:
        with manager._lock:
            manager.state = manager.STATE_IDLE
            manager.last_reset_time = rospy.Time.now().to_sec()


if __name__ == "__main__":
    rospy.init_node("switch_lua")
    manager = CartographerTrajectoryManagerScore()

    # ── Legacy boot handoff path (off by default) ──────────────────────
    boot_handoff_enabled = rospy.get_param("~boot_handoff_enabled", False)
    if boot_handoff_enabled:
        delay = rospy.get_param("~boot_handoff_delay_s", 1.0)
        def _boot_handoff_cb(_evt):
            manager._trigger_recovery("boot handoff", source="BOOT")
        rospy.Timer(rospy.Duration(delay),
                    _boot_handoff_cb, oneshot=True)
        rospy.loginfo("Boot handoff scheduled in %.1fs.", delay)

    # ── New delayed swap path (on by default) ──────────────────────────
    boot_swap_enabled = rospy.get_param("~boot_swap_enabled", True)
    if boot_swap_enabled:
        swap_delay = rospy.get_param("~boot_swap_delay_s", 20.0)
        def _boot_swap_cb(_evt):
            _do_fast_to_slow_swap(manager)
        rospy.Timer(rospy.Duration(swap_delay),
                    _boot_swap_cb, oneshot=True)
        rospy.loginfo("Fast→slow swap scheduled at T=%.1fs.", swap_delay)

    if not boot_handoff_enabled and not boot_swap_enabled:
        rospy.logwarn("Both boot_handoff_enabled and boot_swap_enabled "
                      "are false — runtime guards only, no boot-time "
                      "Lua transition will occur.")

    rospy.spin()
