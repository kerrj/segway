#!/usr/bin/env python3
'''lqr controller for balancing the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand
from util import RunningAverage
RATE=60
ANGLE_OFFSET=-.005
WHEEL_RAD = .04
SPEED_WINDOW=4
th = None
thdot = None
x = None
xdot = None
xdotfiltered = None
speedAvg=RunningAverage(SPEED_WINDOW)


def angleCB(msg):
    global th,thdot
    th=msg.th+ANGLE_OFFSET
    thdot=msg.thdot
def encoderCB(msg):
    global x,xdot,xdotfiltered
    #just use the left value for now
    x = (msg.leftAngle+msg.rightAngle)*(WHEEL_RAD/2)
    xdot=(msg.leftVel+msg.rightVel)*(WHEEL_RAD/2)
    speedAvg.add(xdot)
    xdotfiltered = speedAvg.value()

rospy.init_node("lqr")
angleSub = rospy.Subscriber('angle',AngleReading,angleCB,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,encoderCB,queue_size=1)
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)

start=rospy.get_rostime()
#K=np.array([-1,-2.39,-41.6,-4.55])#all 1's in R matrix

#K=np.array([-7.07,-6.81,-49.83,-5.70])#x=50
K=np.array([-7.42,-7.01,-50.2,-5.70])#x=55
#K=np.array([-7.75,-7.2,-50.53,-5.8])#x=60
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        start=rospy.get_rostime()
        continue
    command=MotorCommand()
    command.stamp=rospy.get_rostime()
    u=-K.dot(np.array([[x],[xdotfiltered],[th],[thdot]]))
    v=xdotfiltered + (u/RATE)
    command.left=v/WHEEL_RAD
    command.right=v/WHEEL_RAD
    controlPub.publish(command)

