#!/usr/bin/env python3
'''lqr controller for balancing the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand,BaseCommand
from util import clip,increment
from math import copysign
RATE=100
ANGLE_OFFSET=.003
WHEEL_RAD = .04
WHEEL_SEP = 0.1588
TURN_IN_PLACE_THRESH=.01#threshold below which vel is considered 0 for turning in place
MAX_LINEAR_VEL=9*WHEEL_RAD#in m/s
MAX_ANG_VEL=3
th = None
thdot = None
x = None
xdot = None#vel in m/s
targetVel=BaseCommand(rospy.Time(0),0,0)

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
def targetCB(msg):
    global targetVel
    if abs(msg.velocity)>MAX_LINEAR_VEL or abs(msg.omega)>MAX_ANG_VEL:
        rospy.logwarn("Target vel over max speed, scaling down")
        scale=max(abs(msg.velocity)/MAX_LINEAR_VEL,abs(msg.omega)/MAX_ANG_VEL)
        msg.velocity/=scale
        msg.omega/=scale
    targetVel=msg

rospy.init_node("lqr")
angleSub = rospy.Subscriber('angle',AngleReading,angleCB,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,encoderCB,queue_size=1)
targetSub = rospy.Subscriber('target_vel',BaseCommand,targetCB,queue_size=1) 
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)

K=np.array([-7.4494   ,-60.2510    ,-7.3056    ,7.0711])#tracking term 50
start=rospy.get_rostime()
xi=0#integrator value
xiscale=2
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        start=rospy.get_rostime()
        continue
    command=MotorCommand()
    command.header.stamp=rospy.get_rostime()
    xi+=(1/RATE)*(targetVel.velocity-xdot)*xiscale
    u=-K.dot(np.array([[xdot],[th],[thdot],[xi]]))
    dxdot_tracking = (u/RATE)#this is acceleration component from stabilization, not including forward vel
    newxdot=(u/RATE) + xdot
    if abs(targetVel.velocity)<.01:
        #w=(vr-vl)/WIDTH
        rffvel=((targetVel.omega*WHEEL_SEP)/2)/WHEEL_RAD
        lffvel=-rffvel
    else:
        w=targetVel.omega*(xdot/targetVel.velocity)
        rffvel=((w*WHEEL_SEP)/2)/WHEEL_RAD
        lffvel=-rffvel
    command.left=  newxdot/WHEEL_RAD + lffvel
    command.right= newxdot/WHEEL_RAD + rffvel
    controlPub.publish(command)

