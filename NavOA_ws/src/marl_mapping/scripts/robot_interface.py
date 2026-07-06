#!/usr/bin/python3
"""
Robot Interface Script - Standalone test for robot connection.
"""

import rospy
from marl_mapping.env.robot_interface import SingleRobot


def main():
    rospy.init_node('robot_interface_test', anonymous=True)

    robot_name = rospy.get_param('~robot_name', 'car1')
    robot = SingleRobot(rb_name=robot_name, check_topic=True)

    rospy.loginfo("Robot interface ready for: %s", robot_name)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        data = robot.get_all_data()
        ekf_pos = robot.get_ekf_pos()
        rospy.loginfo("EKF Pos: (%.3f, %.3f, %.3f)",
                      ekf_pos[0], ekf_pos[1], ekf_pos[2])

        rgb = robot.get_rgb()
        if rgb is not None:
            rospy.loginfo("RGB shape: %s", rgb.shape)
        else:
            rospy.loginfo("RGB: None")

        map_data = robot.get_map()
        if map_data is not None:
            rospy.loginfo("Map shape: %s", map_data.shape)
        else:
            rospy.loginfo("Map: None")

        rate.sleep()


if __name__ == '__main__':
    main()
