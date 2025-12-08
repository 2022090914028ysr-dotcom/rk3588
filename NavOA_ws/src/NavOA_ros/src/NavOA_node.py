#!/usr/bin/env python
import rospy
from NavOA_core import NavOA_core

if __name__ == '__main__':
    NavOA_node = NavOA_core()
    NavOA_node.run()
