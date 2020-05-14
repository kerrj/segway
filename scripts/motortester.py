import rospy
from segway.msg import MotorCommand
rospy.init_node("motor_tester")
cmd_pub=rospy.Publisher("cmd_vel",MotorCommand,queue_size=1)

rate=rospy.Rate(60)
v=0
while not rospy.is_shutdown():
    #v+=.01
    rate.sleep()
    if int(rospy.get_rostime().to_time())%3==0:
        v=1
    if int(rospy.get_rostime().to_time())%3==1:
        v=0
    if int(rospy.get_rostime().to_time())%3==2:
        v=-3
    cmd=MotorCommand()
    cmd.stamp=rospy.get_rostime()
    cmd.left=v
    cmd.right=0
    cmd_pub.publish(cmd)
