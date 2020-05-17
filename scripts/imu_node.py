#!/usr/bin/env python3
import BNO055
import rospy
import numpy as np
from segway.msg import AngleReading
bno = BNO055.BNO055(serial_port='/dev/ttyS0',rst=18)
rospy.init_node('imu_node')
# Initialize the BNO055 and stop if something went wrong.
if not bno.begin(mode=BNO055.OPERATION_MODE_IMUPLUS):
    rospy.signal_shutdown('Failed to initialize BNO055!')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
while status<5:
    status,self_test,error=bno.get_system_status()
    rospy.loginfo('System status: {0}'.format(status))
    rospy.loginfo('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
bno.load_calib()
# Print out an error if system status is in error mode.
if status == 0x01:
    rospy.logerr('System error: {0}'.format(error))
    rospy.logerr('See datasheet section 4.3.59 for the meaning.')
    rospy.signal_shutdown("BNO055 error")

rate=rospy.Rate(100)
statepub=rospy.Publisher('angle',AngleReading,queue_size=10)
rospy.sleep(rospy.Duration(1))
rospy.loginfo("Beginning imu stream")
while not rospy.is_shutdown():
    rate.sleep()
    msg=AngleReading()
    xGrav,yGrav,zGrav=bno.read_gravity()
    xOmega,yOmega,zOmega=bno.read_gyroscope()
    msg.header.stamp=rospy.get_rostime()
    msg.thdot=-xOmega
    msg.zdot=-zOmega
    #get the angle between grav vector and upright position
    projectedGrav=np.array([yGrav,zGrav])
    desiredDir=np.array([0,-1])
    sgn=-np.sign(yGrav)
    #sign is negative on the below line to make thdot and th match
    msg.th=-sgn*np.arccos(projectedGrav.dot(desiredDir)/(np.linalg.norm(projectedGrav)*np.linalg.norm(desiredDir)))
    statepub.publish(msg)
