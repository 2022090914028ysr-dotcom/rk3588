#!/usr/bin/env python3
import os
import sys
from pathlib import Path

# Disable ROS logging config (keeps logs readable in some environments)
os.environ.setdefault('ROS_PYTHON_LOG_CONFIG_FILE', '')

# Ensure we can import NavOA_ros/src modules (NavOA_core.py lives there)
_pkg_dir = Path(__file__).resolve().parents[1]  # NavOA_ros/
_src_dir = _pkg_dir / "src"
sys.path.insert(0, str(_src_dir))

import rospy  # noqa: E402
from NavOA_core import NavOA_core  # noqa: E402


def main() -> None:
    node = NavOA_core()
    node.run()


if __name__ == '__main__':
    main()
