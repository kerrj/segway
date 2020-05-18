#!/usr/bin/env python3
import rospy
import numpy as np
from segway.msg import BaseCommand,Odometry,Path
from pure_pursuit import PurePursuit
pp=PurePursuit(.2,.1)
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
rate=rospy.Rate(5)
while not rospy.is_shutdown():
    rate.sleep()
    v,w=pp.getControl(.1)
    c=BaseCommand()
    c.header.stamp=rospy.get_rostime()
    c.velocity=v
    c.omega=w
    cmdpub.publish(c)
