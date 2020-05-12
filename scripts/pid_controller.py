#!/usr/bin/env python3
'''simple pid controller for the segway'''

import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand
from util import RunningAverage,clip
RATE=60
ANGLE_OFFSET=-.005
WHEEL_RAD = .04
SPEED_WINDOW=round(.4*RATE)
th = None
thdot = None
x = None
xdot = None
speedAvg=RunningAverage(SPEED_WINDOW)


class PID(object):
    def __init__(self,p,i,d,ioutputcap=None,outputcap=None):
        #icap is in units of output, ie capped output after multiplying by i
        self.p=p
        self.i=i
        self.d=d
        if ioutputcap is None:
            self.icap=float('inf')
        else:
            self.icap=ioutputcap/self.i if self.i>0 else 1
        self.sum=0
        self.lasterr=0
        self.outputcap=outputcap
    def step(self,err,derr,dt):
        self.sum+=err
        self.sum=clip(self.sum,-self.icap/dt,self.icap/dt)
        if self.outputcap is None:
            return self.p*err+self.i*self.sum*dt+self.d*derr
        else:
            return clip(self.p*err+self.i*self.sum*dt+self.d*derr,-self.outputcap,self.outputcap)

def angleCB(msg):
    global th,thdot
    th=msg.th+ANGLE_OFFSET
    thdot=msg.thdot
def encoderCB(msg):
    global x,xdot
    #just use the left value for now
    x = msg.leftAngle*WHEEL_RAD
    speedAvg.add(msg.leftVel*WHEEL_RAD)
    xdot = speedAvg.value()

rospy.init_node("pid")
angleSub = rospy.Subscriber('angle',AngleReading,angleCB,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,encoderCB,queue_size=1)
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)
pid1=PID(.25,0.05,.08,ioutputcap=.01,outputcap=.045)
pid2=PID(75,250,1.6,20)

targetpos=0
targetspeed=0.03
start=rospy.get_rostime()
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        start=rospy.get_rostime()
        continue
    targetpos+=targetspeed/RATE
    command=MotorCommand()
    command.stamp=rospy.get_rostime()
    intermediate=pid1.step(x-targetpos,xdot-targetspeed,1/RATE)
    u=pid2.step(th+intermediate,thdot,1/RATE)
    command.left=u
    command.right=u
    controlPub.publish(command)

