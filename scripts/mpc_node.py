#!/usr/bin/env python3
import rospy
import numpy as np
from segway.msg import AngleReading,EncoderReading,MotorCommand,BaseCommand
from util import clip,increment
from math import copysign
from mpc import LinearMPC
RATE=75
ANGLE_OFFSET=-.003
WHEEL_RAD = .04
WHEEL_SEP = 0.1588
TURN_IN_PLACE_THRESH=.01#threshold below which vel is considered 0 for turning in place
MAX_LINEAR_VEL=.25#in m/s
COMMAND_TIMEOUT=1
MAX_ANG_VEL=3
LINEAR_ACCEL=.2
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
def get_ref(xinit,N):
    global targetVel,x,xdot,LINEAR_ACCEL
    return np.repeat(np.zeros((4,1)),N+1,1)
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
Q=np.eye(4)
Q[0,0]=50
Q[1,1]=1
R=1
S=Q
N=75
dt=1/RATE
ucons=np.array([[-15,15]])
mpc=LinearMPC(A,B,Q,R,S,N,dt,u_constraints=ucons)

while not rospy.is_shutdown():
    rate.sleep()
    if th is None or x is None:
        continue
    command=MotorCommand()
    t=rospy.get_rostime()
    command.header.stamp=t
    if t-targetVel.header.stamp>rospy.Duration(COMMAND_TIMEOUT):
        targetVel.velocity=0
        targetVel.omega=0
        targetVel.header.stamp=t
    xinit=np.array([[x],[xdot],[th],[thdot]])
    xref=get_ref(xinit,N)
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

