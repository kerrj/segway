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
from util import * 
from queue import deque
from math import copysign
COUNTS_PER_REV=1632.67
COUNTS_PER_RAD=COUNTS_PER_REV/(2*pi)
ADR=0x80
MAX_SPEED=20#units in radians
TIMEOUT=.25
RATE=100
rospy.init_node("motor_node")
rc=Roboclaw('/dev/ttyACM0',115200)
rc.Open()
rc.ResetEncoders(ADR)
rcLock=Lock()
lastCommandTime=rospy.get_rostime()
#callback functions
def torque_cb(msg):
    rospy.error("Torque control not implemented yet")
def vel_cb(msg):
    global lastCommandTime
    #change m1/m2 correspondence here on both lines below
    m1vel = round(COUNTS_PER_RAD*clip(msg.left,-MAX_SPEED,MAX_SPEED))
    m2vel = round(COUNTS_PER_RAD*clip(msg.right,-MAX_SPEED,MAX_SPEED))
    lastCommandTime=msg.header.stamp
    if rospy.get_rostime() - lastCommandTime > rospy.Duration(TIMEOUT):
        rospy.logwarn("Ignoring stale cmd_vel message")
        return
    rcLock.acquire()
    rc.SpeedM1M2(ADR,m1vel,m2vel)
    rcLock.release()
def shutdown():
    try:
        rc.ForwardM1(ADR,0)
        rc.ForwardM2(ADR,0)
    except:
        rospy.logerr("Error while sending 0 vel on shutdown")
rospy.on_shutdown(shutdown)
velsub=rospy.Subscriber('cmd_vel',MotorCommand,vel_cb,queue_size=1)
torsub=rospy.Subscriber('cmd_torque',MotorCommand,torque_cb,queue_size=1)
statepub=rospy.Publisher('encoders',EncoderReading,queue_size=10)
rate=rospy.Rate(RATE)

m1last=rc.ReadEncM1(ADR)[1]
m2last=rc.ReadEncM2(ADR)[1]
rospy.loginfo("Beginning roboclaw node")
m1AvgSpeed=AdaptiveAverage([4],[0])
m2AvgSpeed=AdaptiveAverage([4],[0])
#last=rospy.get_rostime().to_time()
while not rospy.is_shutdown():
    rate.sleep()
    #cur=rospy.get_rostime().to_time()
    #print(cur-last)
    #last=cur
    rcLock.acquire()
    if rospy.get_rostime() - lastCommandTime > rospy.Duration(TIMEOUT):
        rc.ForwardM1(ADR,0)
        rc.ForwardM2(ADR,0)
    m1enc=rc.ReadEncM1(ADR)[1]
    m2enc=rc.ReadEncM2(ADR)[1]
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
    m1AvgSpeed.add(RATE*m1delta)
    m2AvgSpeed.add(RATE*m2delta)
    e=EncoderReading()
    e.header.stamp=stamp
    #CHANGE M1/M2 CORRESPONDENCE BELOW ON ALL 4 LINES
    e.leftAngle=m1pos
    e.leftVel=m1AvgSpeed.value()
    e.rawLeftVel=RATE*m1delta
    e.rightAngle=m2pos
    e.rightVel=m2AvgSpeed.value()
    e.rawRightVel=RATE*m2delta
    statepub.publish(e)
