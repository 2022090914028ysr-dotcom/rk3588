#!/usr/bin/env python3
"""Catch and print Cartographer node logs (rosout) to the terminal.

Cartographer's glog output (including the ``[GlobalLoc]`` odometry-gate
diagnostics) is bridged into rosconsole, so it goes out over the ``/rosout``
topic to the ROS master and the launching terminal. It is *not* written to
``~/.ros/log`` on the robot. This script taps that same ``/rosout`` stream so
you can watch it from any machine on the ROS graph.

Usage:
    rosrun cartographer_ros carto_loger.py              # print all cartographer_node logs
    rosrun cartographer_ros carto_loger.py --only GlobalLoc   # print only [GlobalLoc] lines
    rosrun cartographer_ros carto_loger.py --only PoseJump,GlobalLoc,MatchResult
    rosrun cartographer_ros carto_loger.py --node cartographer_node --match GlobalLoc

``--only`` accepts a comma-separated list and keeps a line if it contains *any*
of the substrings. Diagnostic tags: ``[Pose]`` (per-scan pose trace, ~scan rate),
``[PoseJump]`` (threshold warning when a pose component jumps), ``[GlobalLoc]``
(odometry-gate decision: ACCEPT/REJECT) and ``[MatchResult]`` (every constraint).
"""


import argparse
import sys

import rospy
from rosgraph_msgs.msg import Log

# ANSI styling for the lines we care about.
_BOLD = "\033[1m"
_GREEN = "\033[32m"
_RED = "\033[31m"
_YELLOW = "\033[33m"
_RESET = "\033[0m"

_LEVELS = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO ",
    Log.WARN: "WARN ",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}


def make_callback(node_filter, only_matches, match_highlight):
    """Return a /rosout callback closure with the given filters."""

    def callback(msg):
        # node_filter is a substring matched against msg.name (e.g. "/cartographer_node").
        if node_filter and node_filter not in msg.name:
            return

        text = msg.msg
        if only_matches and not any(m in text for m in only_matches):
            return

        level = _LEVELS.get(msg.level, "?    ")
        stamp = msg.header.stamp.to_sec()

        if match_highlight and match_highlight in text:
            # Highlight the whole line; colour the decision word too.
            color = _GREEN if "ACCEPT" in text else (_RED if "REJECT" in text else _YELLOW)
            print("%s%.3f [%s] %s%s%s" % (_BOLD, stamp, level, color, text, _RESET))
        else:
            print("%.3f [%s] %s" % (stamp, level, text))
        sys.stdout.flush()

    return callback


def main():
    parser = argparse.ArgumentParser(
        description="Print Cartographer node /rosout messages to the terminal.")
    parser.add_argument("--node", default="cartographer_node",
                        help="substring matched against the rosout node name (default: cartographer_node)")
    parser.add_argument("--only", dest="only_match", default=None,
                        help="print only lines containing any of these comma-separated "
                             "substrings (e.g. PoseJump,GlobalLoc,MatchResult)")
    parser.add_argument("--match", dest="match_highlight", default="GlobalLoc",
                        help="substring whose lines get highlighted (default: GlobalLoc)")
    args = parser.parse_args()

    only_matches = ([m.strip() for m in args.only_match.split(",") if m.strip()]
                    if args.only_match else [])

    rospy.init_node("carto_loger", anonymous=True)
    sub = rospy.Subscriber(
        "/rosout", Log,
        make_callback(args.node, only_matches, args.match_highlight),
        queue_size=1000)

    rospy.loginfo("carto_loger: watching /rosout for node matching '%s'%s",
                  args.node,
                  (" and only lines containing any of %s" % only_matches)
                  if only_matches else "")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
