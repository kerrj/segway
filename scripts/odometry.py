#!/usr/bin/env python3
import rospy
import message_filters
from segway.msg import AngleReading,EncoderReading,Odometry
import numpy as np
from kalman import EKF

WHEEL_RAD = .04
WHEEL_SEP = 0.1588
COUNTS_PER_REV=1632.67

def f(x,dt):
    #x=(x,y,v,th,thdot)
    return np.array([
        [x[0,0]+dt*x[2,0]*np.cos(x[3,0])],
        [x[1,0]+dt*x[2,0]*np.sin(x[3,0])],
        [x[2,0]],
        [x[3,0]+dt*x[4,0]],
        [x[4,0]]])
def h(x):
    return np.array([
        [x[2,0]],
        [x[4,0]],
        [x[4,0]]])
#there is no process noise (suppose very little wheel slip)
Q=np.zeros((5,5))
#set the variance of velocity high to factor in observations
Q[2,2]=10
Q[4,4]=10
#calculate the covariance for encoder readings below
A=np.array([[.5,.5],[-1/WHEEL_SEP,1/WHEEL_SEP]])
pos_res=(2*np.pi*WHEEL_RAD)/COUNTS_PER_REV
speed_res=pos_res/.01
sigma=speed_res**2/12
Renc=sigma*A.dot(A.T)#encoder covariance
R=np.zeros((3,3))#observation covariance
R[2,2]=.00001#estimate of the variance of the imu ang velocity
R[0:2,0:2]=Renc
kf=EKF(f,h,Q,R,np.zeros((5,1)),np.zeros((5,5)))

rospy.init_node("odometry")
odompub=rospy.Publisher('odometry',Odometry,queue_size=10)
lastUpdate=rospy.Time(0)
def updateOdom(angMsg,encMsg):
    global lastUpdate,kf
    t=rospy.get_rostime()
    vl=encMsg.rawLeftVel*WHEEL_RAD
    vr=encMsg.rawRightVel*WHEEL_RAD
    v=(vl+vr)/2
    wenc=(vr-vl)/WHEEL_SEP
    wimu=angMsg.zdot
    z=np.array([[v],[wenc],[wimu]])
    x=kf.update(z,fargs=((t-lastUpdate).to_sec(),))
    o=Odometry()
    o.header.stamp=t
    o.x=x[0,0]
    o.y=x[1,0]
    o.th=x[3,0]
    odompub.publish(o)
    lastUpdate=t

encsub=message_filters.Subscriber('encoders',EncoderReading)
angsub=message_filters.Subscriber('angle',AngleReading)
#first magic number is queue size, second is "slop" meaning max time distance across which messages can be matched
ts=message_filters.ApproximateTimeSynchronizer([angsub,encsub],10,.01)
ts.registerCallback(updateOdom)
rospy.spin()
