#!/usr/bin/env python3
import rospy
import numpy as np
from segway.msg import BaseCommand,Odometry,Path
from pure_pursuit import PurePursuit
from math import copysign
from util import RunningAverage,clip
pp=PurePursuit(.18,.18,.03)#.1588 is wheel separation
def pathCB(msg):
    global pp
    pp.updatePath(msg)
rospy.init_node("waypoint_following")
rospy.Subscriber("waypoints",Path,pathCB,queue_size=1)
def odomCB(msg):
    global pp
    pp.updateOdom(msg)
rospy.Subscriber('odometry',Odometry,odomCB,queue_size=1)

cmdpub=rospy.Publisher('target_vel',BaseCommand,queue_size=1)
rate=rospy.Rate(30)
stopped=False

while not rospy.is_shutdown():
    rate.sleep()
    v,w=pp.getControl(.3)
    c=BaseCommand()
    c.header.stamp=rospy.get_rostime()
    c.velocity=v
    c.omega=clip(w,-3,3)
    if not stopped:
        cmdpub.publish(c)
    stopped=v==0
