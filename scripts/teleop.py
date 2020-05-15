#!/usr/bin python3
import rospy
from segway.msg import MotorCommand
rospy.init_node("teleop")
cmd_pub=rospy.Publisher("target_vel",MotorCommand,queue_size=1)
svel=2.5
turnvel=.75
while not rospy.is_shutdown():
    cmd=input("Type dir (wasd or space to stop)")
    print(cmd)
    msg=MotorCommand()
    msg.stamp=rospy.get_rostime()
    if cmd=='w':
        msg.left=svel
        msg.right=svel
    elif cmd=='s':
        msg.left=-svel
        msg.right=-svel
    elif cmd=='a':
        msg.left=-turnvel
        msg.right=turnvel
    elif cmd=='d':
        msg.left=turnvel
        msg.right=-turnvel
    else:
        msg.left=0
        msg.right=0
    cmd_pub.publish(msg)

