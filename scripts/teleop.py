#!/usr/bin python3
import rospy
from segway.msg import BaseCommand 
rospy.init_node("teleop")
cmd_pub=rospy.Publisher("target_vel",BaseCommand,queue_size=1)
svel=.25
turnvel=1
print("Type command (wasd for direction). Multiple chars on one line will mix commands. Blank line to stop")
while not rospy.is_shutdown():
    cmd=input("Command: ")
    print(cmd)
    msg=BaseCommand()
    msg.header.stamp=rospy.get_rostime()
    msg.velocity=0
    msg.omega=0
    if 'w' in cmd:
        msg.velocity+=svel
    if 's' in cmd:
        msg.velocity-=svel
    if 'a' in cmd:
        msg.omega+=turnvel
    if 'd' in cmd:
        msg.omega-=turnvel
    cmd_pub.publish(msg)

