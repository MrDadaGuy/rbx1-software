#! /usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from robot import initRobot, robotMove, waitForRobot

ros_ext = [[-2.09, 2.09], [-math.pi/2, math.pi/2], [-math.pi/2, math.pi/2], [-math.pi, math.pi], [-math.pi/2, math.pi/2], [-math.pi, math.pi], [-math.pi/2, 0]]     # extents
rbx_ext = [[-13139, 13139], [-4196, 4196], [16960, -16960], [-1542, 1542], [-3282, 3282], [-0, 0], [0, 7]]  # TODO:  last two are wrong i"m sure, 4, 2, 1 are good

last_joint_states = np.zeros((7,), dtype=int)     # refers to RBX joint states, not ROS

def remap(x, in_min, in_max, out_min, out_max):
    return int(round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))


def processJointState(joint_states):
    global last_joint_states
    new_joint_states = np.zeros((7,), dtype=int)

    # TODO:  probably capture the raw joint states from ROS for last_joint_states, it will be more precise pre-rounding
    for j in range(len(ros_ext)):
        new_joint_states[j] = remap(joint_states.position[j], ros_ext[j][0], ros_ext[j][1], rbx_ext[j][0], rbx_ext[j][1])

    if not np.array_equal(new_joint_states, last_joint_states):
        print("MOVE to {}".format(new_joint_states))
        robotMove(80, new_joint_states)
        last_joint_states = new_joint_states


def listener():
    rospy.init_node('rbx_arm', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, processJointState)
    rospy.spin()

if __name__ == '__main__':
    initRobot()
    listener()
