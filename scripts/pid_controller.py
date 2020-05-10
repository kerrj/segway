#!/usr/bin/env python3
'''simple pid controller for the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand
rospy.init_node("pid")
angleSub = rospy.Subscriber('angle',AngleReading,queue_size=1)

