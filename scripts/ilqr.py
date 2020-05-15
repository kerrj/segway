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
MAX_VEL=3
SPEED_RAMP=3/RATE#ramp value in rad/s^2 to achieve targetVel
th = None
thdot = None
x = None
xdot = None#vel in m/s
lvel = None
rvel = None
xanchor=0
targetVel=MotorCommand(rospy.Time(0),0,0)
lffvel=0
rffvel=0

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

K=np.array([ -6.3974   ,-58.2077    ,-7.0080    ,5.4772])#tracking term 25
start=rospy.get_rostime()
xi=0#integrator value
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        start=rospy.get_rostime()
        continue
    command=MotorCommand()
    command.stamp=rospy.get_rostime()
    lffvel,lffacc=increment(lffvel,targetVel.left,SPEED_RAMP)
    rffvel,rffacc=increment(rffvel,targetVel.right,SPEED_RAMP)
    r=(lffvel+rffvel)*(WHEEL_RAD/2)
    u=-K.dot(np.array([[xdot],[th],[thdot],[xi]]))
    xi+=(1/RATE)*(r-xdot)
    dxdot_tracking = (u/RATE)#this is acceleration component from stabilization, not including forward vel
    command.left=  dxdot_tracking/WHEEL_RAD + xdot/WHEEL_RAD + (lffvel-(lffvel+rffvel)/2) 
    command.right= dxdot_tracking/WHEEL_RAD + xdot/WHEEL_RAD + (rffvel-(lffvel+rffvel)/2)
    controlPub.publish(command)

