#!/usr/bin/env python
import rospy

import os
# Disable ROS logging config
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''

# Set up basic logging
import logging
logging.basicConfig(level=logging.INFO, format='[%(name)s][%(levelname)s]: %(message)s')

import rospy
from NavOA_core import NavOA_core

if __name__ == '__main__':
    NavOA_node = NavOA_core()
    NavOA_node.run()
