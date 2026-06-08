#!/usr/bin/env python3
import os
import sys
from pathlib import Path

# Disable ROS logging config (keeps logs readable in some environments)
os.environ.setdefault('ROS_PYTHON_LOG_CONFIG_FILE', '')

# Ensure we can import NavOA_ros/src modules (NavOA_core.py lives there)
_script_dir = Path(__file__).resolve().parent
_pkg_dir = _script_dir
if not (_pkg_dir / "src").exists():
    # Installed scripts are flattened into lib/NavOA_ros, so use parent directory.
    _pkg_dir = _script_dir.parent
_src_dir = _pkg_dir / "src"
if _src_dir.exists():
    sys.path.insert(0, str(_src_dir))

import rospy  # noqa: E402
from NavOA_core import NavOA_core  # noqa: E402


def main() -> None:
    node = NavOA_core()
    node.run()


if __name__ == '__main__':
    main()
