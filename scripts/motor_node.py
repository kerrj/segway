#!/usr/bin/env python3

#node for controlling the motors given either cmd_vel or cmd_torque commands
from roboclaw_driver.roboclaw_3 import Roboclaw
import rospy
from segway.msg import MotorCommand,EncoderReading
from threading import Lock
#rc=Roboclaw('/dev/ttyACM0',115200)
#rc.Open()
rcLock=Lock()
rospy.init_node("motor_node")
velsub=rospy.Subscriber('cmd_vel',MotorCommand,queue_size=1)
torsub=rospy.Subscriber('cmd_torque',MotorCommand,queue_size=1)
statepub=rospy.Publisher('encoders',EncoderReading,queue_size=10)
