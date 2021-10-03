#! /usr/bin/env python
# -*- encoding: utf-8 -*-

import roslib
import numpy as np
import time
from math import pi, sqrt, atan2, sin, cos
import threading
from enum import Enum

import rospy
import rosparam

import rospy
from std_msgs.msg import String, Float64

def jump_command_publisher():
    log_level = rospy.DEBUG
    pub = rospy.Publisher('/jump', Float64, queue_size=5)
    rospy.init_node('jump_command_publisher', anonymous=True)
    loop_count = 0
    step_count = 0
    cmd = Float64()
    CMD_HIGH = 0.7
    CMD_LOW = -0.7
    cmd.data = 0
    rospy.sleep(1.0)
    pub.publish(cmd)
    rospy.sleep(1.0)
    r = rospy.Rate(3) # 100hz
    while not rospy.is_shutdown():
        if loop_count % 3 == 0:
            cmd.data = CMD_HIGH
        else:
            cmd.data = CMD_LOW
        loop_count += 1
        pub.publish(cmd)
        r.sleep()

if __name__ == '__main__':
    try:
        jump_command_publisher()
    except rospy.ROSInterruptException: pass
