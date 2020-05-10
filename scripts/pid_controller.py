#!/usr/bin/env python3
'''simple pid controller for the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand
RATE=50

rospy.init_node("pid")
angleSub = rospy.Subscriber('angle',AngleReading,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,queue_size=1)
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)

