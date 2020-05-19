#!/usr/bin python3
import rospy
from segway.msg import BaseCommand,Path
from geometry_msgs.msg import Point
rospy.init_node("teleop")
cmd_pub=rospy.Publisher("target_vel",BaseCommand,queue_size=1)
pathpub=rospy.Publisher('waypoints',Path,queue_size=1)
svel=.25
turnvel=1
print("Type command (wasd for direction). Multiple chars on one line will mix commands. Blank line to stop")
while not rospy.is_shutdown():
    cmd=input("Command: ")
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
    if 'l:' in cmd:
        s=cmd[cmd.find('l:')+2:]
        pairs=s.split(';')
        path=Path()
        l=[]
        for pair in pairs:
            xy=pair.split(',')
            x=float(xy[0][1:])
            y=float(xy[1][:-1])
            p=Point(x,y,0.)
            l.append(p)
        path.waypoints=l
        pathpub.publish(path)
    cmd_pub.publish(msg)

