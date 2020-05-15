#!/usr/bin/env python3
'''lqi controller for balancing the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand
from util import clip
from math import copysign
RATE=100
ANGLE_OFFSET=.003#-.005
WHEEL_RAD = .04
MAX_VEL=1
SPEED_RAMP=3/RATE#ramp value in rad/s^2 to achieve targetVel
th = None
thdot = None
x = None
xdot = None#vel in m/s
xanchor=0
targetVel=MotorCommand(rospy.Time(0),0,0)
lvel=0#vel in m/s
rvel=0

def angleCB(msg):
    global th,thdot
    th=msg.th+ANGLE_OFFSET
    thdot=msg.thdot
def encoderCB(msg):
    global x,xdot
    x = (msg.leftAngle+msg.rightAngle)*(WHEEL_RAD/2)
    xdot=(msg.leftVel+msg.rightVel)*(WHEEL_RAD/2)
def targetCB(msg):
    global targetVel
    if abs(msg.left)>MAX_VEL or abs(msg.right)>MAX_VEL:
        rospy.logwarn("Target vel over max speed, scaling down")
        scale=MAX_VEL/max(abs(msg.left),abs(msg.right))
        msg.left*=scale
        msg.right*=scale
    targetVel=msg
def increment(v,t,a):
    dv=copysign(a,t-v)
    newv=v+dv
    if abs(v-t)<=1.1*a:newv=t
    return newv,newv-v

rospy.init_node("lqr")
angleSub = rospy.Subscriber('angle',AngleReading,angleCB,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,encoderCB,queue_size=1)
targetSub = rospy.Subscriber('target_vel',MotorCommand,targetCB,queue_size=1) 
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)

K=np.array([-0.7071   ,-3.0172   ,-50.5654    ,-5.9256    ,0.7071])#1s everywhere
#K=np.array([-2.2678   ,-4.1772   ,-52.9527    ,-6.2726    ,0.3780])#x=6
start=rospy.get_rostime()
xi=0#integrator value
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        start=rospy.get_rostime()
        continue
    if rospy.get_rostime()-start>rospy.Duration(5):
        print("go")
        targetVel.left=-.2
        targetVel.right=-.2
    if rospy.get_rostime()-start>rospy.Duration(8):
        print("Stop")
        targetVel.left=.2
        targetVel.right=.2
    command=MotorCommand()
    command.stamp=rospy.get_rostime()
    lvel,lacc=increment(lvel,targetVel.left,SPEED_RAMP)
    rvel,racc=increment(rvel,targetVel.right,SPEED_RAMP)
    if abs(lvel)>0 or abs(rvel)>0:
        xtrack=0
        xdottrack=xdot-(lvel+rvel)*(WHEEL_RAD/2)
        xanchor=x
    else:
        xtrack=x-xanchor
        xdottrack=xdot
    r=(lvel+rvel)*(WHEEL_RAD/2)
    xi+=(1/RATE)*(r-xdot)
    u=-K.dot(np.array([[xtrack],[xdottrack],[th],[thdot],[xi]]))
    v=xdottrack + (u/RATE)
    command.left=v/WHEEL_RAD+lacc
    command.right=v/WHEEL_RAD+racc
    controlPub.publish(command)

