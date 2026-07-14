#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mode_manager.py — System mode orchestrator for SlamNav.

Alias for runner_node.py — starts the full system lifecycle manager.

Commands via /runner/command (std_msgs/String):
  "mode:mapping"       — Switch to SLAM mapping mode
  "mode:localization"  — Start trajectory services for seamless switching)
  "mode:auto_mapping"   — Switch to autonomous mapping mode
  "mode:sensors"       — Sensor stack only (no SLAM/navigation)
  "stop_mapping"       — Save map + finish mapping + start localization
  "start_mapping"      — Alias for "mode:mapping"
  "save_map [name]"    — Save current map with optional name
  "list_maps"          — List saved .pbstream files in JSON status
  "status"             — Publish current system status
  "shutdown"           — Gracefully shut down the system

Status on /runner/status (JSON std_msgs/String):
  See runner_node.py for full schema.
"""

import os
import sys

# Allow import when installed via catkin (which wraps scripts in a shim).
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import rospy
from runner_node import RunnerNode

if __name__ == "__main__":
    rospy.init_node("runner_manager")
    node = RunnerNode()
    rospy.spin()
