#!/usr/bin/env python3
import rospy
import message_filters
from segway.msg import AngleReading,EncoderReading,Odometry
import numpy as np
from kalman import EKF
from util import wrapTo2Pi
WHEEL_RAD = .04
WHEEL_SEP = 0.1588
COUNTS_PER_REV=1632.67
IMU_FUDGE_FACTOR=1/1.04950#this is some voodoo since the IMU for some reason overestimates velocity by a small factor, which i measured here

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
Q[2,2]=100
Q[4,4]=100
#calculate the covariance for encoder readings below
A=np.array([[.5,.5],[-1/WHEEL_SEP,1/WHEEL_SEP]])
pos_res=(2*np.pi*WHEEL_RAD)/COUNTS_PER_REV
speed_res=pos_res/.01
sigma=speed_res**2/12
Renc=sigma*A.dot(A.T)#encoder covariance
R=np.zeros((3,3))#observation covariance
R[2,2]=.0005#estimate of the variance of the imu ang velocity
R[0:2,0:2]=Renc
kf=EKF(f,h,Q,R,np.zeros((5,1)),np.zeros((5,5)))

rospy.init_node("odometry")
odompub=rospy.Publisher('odometry',Odometry,queue_size=10)
#rk=rospy.Publisher('rk4',Odometry,queue_size=10)
#odom=Odometry()
lastUpdate=rospy.Time(0)
def updateOdom(angMsg,encMsg):
    global lastUpdate,kf
    t=rospy.get_rostime()
    if lastUpdate<rospy.Time(1):
        lastUpdate=t
        return
    #RK4 stuff here for comparison
    #l_v=encMsg.rawLeftVel*WHEEL_RAD/100
    #r_v=encMsg.rawRightVel*WHEEL_RAD/100
    #v=(l_v+r_v)/2.0
    #o=(r_v-l_v)/WHEEL_SEP
    #RT=odom.th
    #opt1=RT+o
    #opt2=v/6.0
    #opt3=o/2.0
    #opt4=RT+opt3
    #odom.header.stamp=t
    #odom.x=odom.x+(opt2)*(np.cos(RT)+4.0*np.cos(opt4)+np.cos(opt1))
    #odom.y=odom.y+(opt2)*(np.sin(RT)+4.0*np.sin(opt4)+np.sin(opt1))
    #odom.th=wrapTo2Pi(opt1)
    #rk.publish(odom)
    #EKF stuff here
    vl=encMsg.rawLeftVel*WHEEL_RAD
    vr=encMsg.rawRightVel*WHEEL_RAD
    v=(vl+vr)/2
    wenc=(vr-vl)/WHEEL_SEP
    wimu=angMsg.zdot * IMU_FUDGE_FACTOR
    z=np.array([[v],[wenc],[wimu]])
    x=kf.update(z,fargs=((t-lastUpdate).to_sec(),))
    o=Odometry()
    o.header.stamp=t
    o.x=x[0,0]
    o.y=x[1,0]
    o.th=wrapTo2Pi(x[3,0])
    odompub.publish(o)
    lastUpdate=t

encsub=message_filters.Subscriber('encoders',EncoderReading)
angsub=message_filters.Subscriber('angle',AngleReading)
#first magic number is queue size, second is "slop" meaning max time distance across which messages can be matched
ts=message_filters.ApproximateTimeSynchronizer([angsub,encsub],10,.01)
ts.registerCallback(updateOdom)
rospy.spin()
