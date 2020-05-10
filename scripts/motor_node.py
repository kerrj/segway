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

def clip(v,minv,maxv):
    return min(max(v,minv),maxv)

COUNTS_PER_REV=1632.67
COUNTS_PER_RAD=COUNTS_PER_REV/(2*pi)
ADR=0x80
MAX_SPEED=4*pi#2 rev/sec

#rc=Roboclaw('/dev/ttyACM0',115200)
#rc.Open()
rcLock=Lock()

#callback functions
def torque_cb(msg):
    rospy.error("Torque control not implemented yet")
def vel_cb(msg):
    #change m1/m2 correspondence here on both lines below
    m1vel = round(clip(msg.left*COUNTS_PER_RAD,-MAX_SPEED,MAX_SPEED))
    m2vel = round(clip(msg.right*COUNTS_PER_RAD,-MAX_SPEED,MAX_SPEED))
    lastCommandTime=msg.stamp
    rcLock.acquire()
    rc.SpeedM1M2(ADR,m1vel,m2vel)
    rcLock.release()
def shutdown():
    try:
        rc.SpeedM1M2(ADR,0,0)
    except:
        rospy.logerr("Error while sending 0 vel on shutdown")
rospy.init_node("motor_node")
rospy.on_shutdown(shutdown)
lastCommandTime=rospy.get_rostime()
velsub=rospy.Subscriber('cmd_vel',MotorCommand,vel_cb,queue_size=1)
torsub=rospy.Subscriber('cmd_torque',MotorCommand,torque_cb,queue_size=1)
statepub=rospy.Publisher('encoders',EncoderReading)
rate=rospy.Rate(100)
while not rospy.is_shutdown():
    rate.sleep()
    rcLock.acquire()
    if rospy.get_rostime() - lastCommandTime > rospy.Duration(1):
        rc.SpeedM1M2(ADR,0,0)
    m1enc=rc.ReadEncM1(ADR)
    m2enc=rc.ReadEncM2(ADR)
    m1clicks=rc.ReadSpeedM1(ADR)
    m2clicks=rc.ReadSpeedM2(ADR)
    stamp=rospy.get_rostime()
    rcLock.release()
    m1pos=m1enc/COUNTS_PER_RAD
    m2pos=m2enc/COUNTS_PER_RAD
    m1speed=m1clicks/COUNTS_PER_RAD
    m2speed=m2clicks/COUNTS_PER_RAD
    e=EncoderReading()
    e.stamp=stamp
    #CHANGE M1/M2 CORRESPONDENCE BELOW ON ALL 4 LINES
    e.leftAngle=m1pos
    e.leftSpeed=m1speed
    e.rightAngle=m2pos
    e.rightSpeed=m2speed
    statepub.publish(e)
