#!/usr/bin/env python3
import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand,BaseCommand
from util import clip,increment
from math import copysign
from mpc import LinearMPC
RATE=100
ANGLE_OFFSET=-.003
WHEEL_RAD = .04
WHEEL_SEP = 0.1588
TURN_IN_PLACE_THRESH=.01#threshold below which vel is considered 0 for turning in place
MAX_LINEAR_VEL=.25#in m/s
COMMAND_TIMEOUT=1
MAX_ANG_VEL=3
th = None
thdot = None
x = None
xdot = None#vel in m/s
targetVel=BaseCommand()

def angleCB(msg):
    global th,thdot
    th=msg.th+ANGLE_OFFSET
    thdot=msg.thdot
def encoderCB(msg):
    global x,xdot,lvel,rvel
    x = (msg.leftAngle+msg.rightAngle)*(WHEEL_RAD/2)
    xdot=(msg.leftVel+msg.rightVel)*(WHEEL_RAD/2)
    lvel=msg.leftVel
    rvel=msg.rightVel
def targetCB(msg):
    global targetVel
    if abs(msg.velocity)>MAX_LINEAR_VEL or abs(msg.omega)>MAX_ANG_VEL:
        scale=max(abs(msg.velocity)/MAX_LINEAR_VEL,abs(msg.omega)/MAX_ANG_VEL)
        msg.velocity/=scale
        msg.omega/=scale
    targetVel=msg
def get_ref(xfollow,N,dt):
    global targetVel
    xr=np.repeat(np.array([[x],[targetVel.velocity],[0],[0]]),N+1,1)
    xr[0,:]=xfollow+(dt*targetVel.velocity)*np.cumsum(np.ones((N+1,1)))
    return xr
rospy.init_node("mpc")
angleSub = rospy.Subscriber('angle',AngleReading,angleCB,queue_size=1)
encoderSub = rospy.Subscriber('encoders',EncoderReading,encoderCB,queue_size=1)
targetSub = rospy.Subscriber('target_vel',BaseCommand,targetCB,queue_size=1) 
controlPub = rospy.Publisher('cmd_vel',MotorCommand,queue_size=1)

rate=rospy.Rate(RATE)
#MPC initialization
A=np.zeros((4,4))
A[0,1]=1
A[1,2]=6.9126
A[2,3]=1
A[3,2]=81.6765
B=np.array([[0],[.7432],[0],[3.6319]])
Q=np.diag([20,30,1,1.])#dot is important
R=1
S=450*Q
N=50
dt=1/RATE
xlo=np.array([[-np.inf],[-np.inf],[-np.inf],[-np.inf]])
xhi=-xlo
xcons=np.hstack([xlo,xhi])
ucons=np.array([[-20,20]])
mpc=LinearMPC(A,B,Q,R,S,N,dt,u_constraints=ucons,x_constraints=xcons)
mpc.solve(np.zeros((4,1)),get_ref(0,N,dt))#prime the solver for warm starts
lastloop=rospy.get_rostime()
xfollow=0
while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        continue
    command=MotorCommand()
    t=rospy.get_rostime()
    if t-lastloop>rospy.Duration(1.05/RATE):
        rospy.logwarn("MPC loop timing over")
    command.header.stamp=t
    if t-targetVel.header.stamp>rospy.Duration(COMMAND_TIMEOUT):
        targetVel.velocity=0
        targetVel.omega=0
        targetVel.header.stamp=t
    xinit=np.array([[x],[xdot],[th],[thdot]])
    xref=get_ref(xfollow,N,dt)
    xfollow=xref[0,0]#update the reference position
    ut,xt=mpc.solve(xinit,xref)#solve mpc here
    u=ut[0,0]#0th element is the control we apply here
    dxdot_tracking = (u/RATE)#this is acceleration component from stabilization, not including forward vel
    newxdot=(u/RATE) + xdot
    if abs(targetVel.velocity)<.01:
        #w=(vr-vl)/WIDTH
        rffvel=((targetVel.omega*WHEEL_SEP)/2)/WHEEL_RAD
        lffvel=-rffvel
    else:
        w=targetVel.omega*(xdot/targetVel.velocity)
        rffvel=((w*WHEEL_SEP)/2)/WHEEL_RAD
        lffvel=-rffvel
    command.left=  newxdot/WHEEL_RAD + lffvel
    command.right= newxdot/WHEEL_RAD + rffvel
    controlPub.publish(command)
    lastloop=t

