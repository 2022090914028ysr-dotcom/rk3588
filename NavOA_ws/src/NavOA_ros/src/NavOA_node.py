#!/usr/bin/env python
import os
import sys
from pathlib import Path

# Disable ROS logging config
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''

# Ensure we can import NavOA_core from the local src folder.
_script_dir = Path(__file__).resolve().parent
_pkg_dir = _script_dir
if not (_pkg_dir / "NavOA_core.py").exists():
    _pkg_dir = _script_dir.parent
_src_dir = _pkg_dir / "src"
if _src_dir.exists():
    sys.path.insert(0, str(_src_dir))

# Set up basic logging
import logging
logging.basicConfig(level=logging.INFO, format='[%(name)s][%(levelname)s]: %(message)s')

import rospy
from NavOA_core import NavOA_core

if __name__ == '__main__':
    NavOA_node = NavOA_core()
    NavOA_node.run()
