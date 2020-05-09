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

COUNTS_PER_REV=1632.67
COUNTS_PER_RAD=COUNTS_PER_REV/(2*pi)
ADR=0x80
#rc=Roboclaw('/dev/ttyACM0',115200)
#rc.Open()
#rc.ResetEncoders(ADR)
rcLock=Lock()

#callback functions
def torque_cb(msg):
    rospy.error("Torque control not implemented yet")
def vel_cb(msg):
    #change m1/m2 correspondence here on both lines below
    m1vel = round(msg.left*COUNTS_PER_RAD)
    m2vel = round(msg.right*COUNTS_PER_RAD)
    rcLock.acquire()
    rc.SpeedM1M2(ADR,m1vel,m2vel)
    rcLock.release()

rospy.init_node("motor_node")
velsub=rospy.Subscriber('cmd_vel',MotorCommand,vel_cb,queue_size=1)
torsub=rospy.Subscriber('cmd_torque',MotorCommand,torque_cb,queue_size=1)
statepub=rospy.Publisher('encoders',EncoderReading,queue_size=10)
rate=rospy.Rate(100)
while not rospy.is_shutdown():
    rcLock.acquire()
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
    rate.sleep()
