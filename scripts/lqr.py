#!/usr/bin/env python3
'''lqr controller for balancing the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand
from util import clip
from math import copysign
RATE=100
ANGLE_OFFSET=.003
WHEEL_RAD = .04
th = None
thdot = None
x = None
xdot = None#vel in m/s

def angleCB(msg):
    global th,thdot
    th=msg.th+ANGLE_OFFSET
    thdot=msg.thdot
def encoderCB(msg):
    global x,xdot,lvel,rvel
    x = (msg.leftAngle+msg.rightAngle)*(WHEEL_RAD/2)
    xdot=(msg.leftVel+msg.rightVel)*(WHEEL_RAD/2)
    lvel=msg.leftVel
    rvel=msg.rightVel

rospy.init_node("lqr")
angleSub = rospy.Subscriber('angle',AngleReading,angleCB,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,encoderCB,queue_size=1)
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)

K=np.array([-3.8730   ,-5.2373  ,-55.9266   ,-6.6755])#x=15,m-.15,l=.19
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        continue
    command=MotorCommand()
    command.header.stamp=rospy.get_rostime()
    u=-K.dot(np.array([[x],[xdot],[th],[thdot]]))
    dxdot_tracking = (u/RATE) 
    command.left=  dxdot_tracking/WHEEL_RAD + xdottrack/WHEEL_RAD
    command.right= dxdot_tracking/WHEEL_RAD + xdottrack/WHEEL_RAD 
    controlPub.publish(command)

