#!/usr/bin/env python3
'''node for controlling the motors given either cmd_vel or cmd_torque commands
commands are interpreted as positive being in the forward direction WRT robot's movement
command units are in radians and rad/s, conversion to encoders takes place in this file
'''
from roboclaw_driver.roboclaw_3 import Roboclaw
import rospy
from segway.msg import MotorCommand,EncoderReading
from threading import Lock
from math import pi
from queue import deque

def clip(v,minv,maxv):
    return min(max(v,minv),maxv)

class RunningAverage(object):
    def __init__(self,size):
        self.q=deque()
        self.sum=0
        self.size=size
    def add(self,elem):
        if len(self.q) == self.size:
            self.q.append(elem)
            self.sum=self.sum+elem-self.q.popleft()
        else:
            self.q.append(elem)
            self.sum+=elem
    def avg(self):
        return self.sum/len(self.q)

COUNTS_PER_REV=1632.67
COUNTS_PER_RAD=COUNTS_PER_REV/(2*pi)
ADR=0x80
MAX_SPEED=4*pi#2 rev/sec
TIMEOUT=1
RATE=200
SPEED_WINDOW=10#number of samples to average speed over

rc=Roboclaw('/dev/ttyACM0',115200)
rc.Open()
rcLock=Lock()

#callback functions
def torque_cb(msg):
    rospy.error("Torque control not implemented yet")
def vel_cb(msg):
    #change m1/m2 correspondence here on both lines below
    m1vel = round(COUNTS_PER_RAD*clip(msg.left,-MAX_SPEED,MAX_SPEED))
    m2vel = round(COUNTS_PER_RAD*clip(msg.right,-MAX_SPEED,MAX_SPEED))
    lastCommandTime=msg.stamp
    #if rospy.get_rostime() - lastCommandTime > rospy.duration(TIMEOUT):
    #    continue
    rcLock.acquire()
    rc.SpeedM1M2(ADR,m1vel,m2vel)
    rcLock.release()
def shutdown():
    try:
        rc.ForwardM1(ADR,0)
        rc.ForwardM2(ADR,0)
    except:
        rospy.logerr("Error while sending 0 vel on shutdown")
rospy.init_node("motor_node")
rospy.on_shutdown(shutdown)
lastCommandTime=rospy.get_rostime()
velsub=rospy.Subscriber('cmd_vel',MotorCommand,vel_cb,queue_size=1)
torsub=rospy.Subscriber('cmd_torque',MotorCommand,torque_cb,queue_size=1)
statepub=rospy.Publisher('encoders',EncoderReading,queue_size=10)
rate=rospy.Rate(RATE)

m1last=rc.ReadEncM1(ADR)[1]
m2last=rc.ReadEncM2(ADR)[1]
m1AvgDelta=RunningAverage(SPEED_WINDOW)
m2AvgDelta=RunningAverage(SPEED_WINDOW)
while not rospy.is_shutdown():
    rate.sleep()
    rcLock.acquire()
    #if rospy.get_rostime() - lastCommandTime > rospy.Duration(TIMEOUT):
    #    rc.ForwardM1(ADR,0)
    #    rc.ForwardM2(ADR,0)
    m1enc=rc.ReadEncM1(ADR)[1]
    m2enc=rc.ReadEncM2(ADR)[1]
    #m1clicks=rc.ReadISpeedM1(ADR)[1]
    #m2clicks=rc.ReadSpeedM2(ADR)[1]
    rcLock.release()
    m1clicks=m1enc-m1last
    m2clicks=m2enc-m2last
    m1last=m1enc
    m2last=m2enc
    stamp=rospy.get_rostime()
    m1pos=m1enc/COUNTS_PER_RAD
    m2pos=m2enc/COUNTS_PER_RAD
    m1delta=m1clicks/COUNTS_PER_RAD
    m2delta=m2clicks/COUNTS_PER_RAD
    m1AvgDelta.add(m1delta)
    m2AvgDelta.add(m2delta)
    e=EncoderReading()
    e.stamp=stamp
    #CHANGE M1/M2 CORRESPONDENCE BELOW ON ALL 4 LINES
    e.leftAngle=m1pos
    e.leftVel=RATE*m1AvgDelta.avg()
    e.rightAngle=m2pos
    e.rightVel=RATE*m2AvgDelta.avg()
    statepub.publish(e)
