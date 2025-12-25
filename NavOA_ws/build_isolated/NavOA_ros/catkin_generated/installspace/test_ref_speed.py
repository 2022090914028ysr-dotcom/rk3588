#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Float32

received = None

def cb(msg):
    global received
    received = msg.data
    rospy.loginfo(f"Received ref_speed: {received}")

if __name__ == '__main__':
    rospy.init_node('test_ref_speed_node', anonymous=True)

    # wait for node and topic
    rospy.loginfo('Waiting for /NavOA/ref_speed...')
    rospy.wait_for_message('/NavOA/ref_speed', Float32, timeout=5)
    sub = rospy.Subscriber('/NavOA/ref_speed', Float32, cb)

    # connect to dynamic_reconfigure server (node name in launch is NavOA_control)
    rospy.loginfo('Connecting to dynamic_reconfigure server at /NavOA_control')
    client = Client('/NavOA_control', timeout=5)

    new_speed = 0.25
    rospy.loginfo(f'Updating ref_speed -> {new_speed}')
    client.update_configuration({'ref_speed': new_speed})

    # wait for callback
    rospy.sleep(1.0)

    if received is None:
        rospy.logerr('Did not receive /NavOA/ref_speed update')
        exit(1)

    if abs(received - new_speed) < 1e-6:
        rospy.loginfo('ref_speed update successful')
        exit(0)
    else:
        rospy.logerr(f'ref_speed mismatch: {received} != {new_speed}')
        exit(2)
