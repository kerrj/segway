#!/usr/bin/env python3
'''lqr controller for balancing the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand
from util import RunningAverage
RATE=100
ANGLE_OFFSET=-.005
WHEEL_RAD = .04
th = None
thdot = None
x = None
xdot = None
xdotfiltered = None


def angleCB(msg):
    global th,thdot
    th=msg.th+ANGLE_OFFSET
    thdot=msg.thdot
def encoderCB(msg):
    global x,xdot,xdotfiltered
    #just use the left value for now
    x = (msg.leftAngle+msg.rightAngle)*(WHEEL_RAD/2)
    xdot=(msg.leftVel+msg.rightVel)*(WHEEL_RAD/2)

rospy.init_node("lqr")
angleSub = rospy.Subscriber('angle',AngleReading,angleCB,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,encoderCB,queue_size=1)
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)

start=rospy.get_rostime()

#K=np.array([-1,-2.39,-41.6,-4.55])#all 1's in R matrix
#K=np.array([-7.75,-11.25,-60.48,-7.20])#x=60,xdot=60
#K=np.array([-3.1623,-4.2773,-45.1746,-5.0523])#x=10
K=np.array([-3.1623,-5.3807,-47.7417,-5.4126])#x=10,xdot=10
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        start=rospy.get_rostime()
        continue
    command=MotorCommand()
    command.stamp=rospy.get_rostime()
    u=-K.dot(np.array([[x],[xdot],[th],[thdot]]))
    v=xdot + (u/RATE)
    command.left=v/WHEEL_RAD
    command.right=v/WHEEL_RAD
    controlPub.publish(command)

